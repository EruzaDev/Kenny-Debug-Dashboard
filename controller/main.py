"""Resident policy, continuous SLAM-map refinement, and cloud mission control.

Run with ROS sourced and the simulator's Python environment:
python -m controller.main --environment /path/environment.json --model /path/best.zip
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.error import URLError

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'Kenny---Automotive-self-driving-trash-can'))
from controller.mapping import AdaptiveMap, compose, inverse
from controller.missions import CloudAPI, Missions
from controller.planning import Planner
from controller.policy import ResidentPolicy, guard
from controller.sensors import lidar_features, depth_features, depth_image_points
from controller.tracking import Tracker
from controller.safety import StopGate
from controller.localization import MarkerAnchor
from controller.pwm_navigation import PwmNavigator
from controller.roaming import roaming_direction, MeasuredScanHistory, DirectionStability
from pi.receiver import save


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def confirmed_marker_proximity(sightings, now, limit=.70):
    """Require three recent independent depth planes of the requested ArUco."""
    return (len(sightings) >= 3 and now-sightings[-1][0] <= .5
            and sightings[-1][0]-sightings[-3][0] <= 1.3
            and all(.1 <= distance <= limit for _, distance in list(sightings)[-3:]))


def make_node(args, policy, environment):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped, TransformStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as Route
    from sensor_msgs.msg import LaserScan, Image, CameraInfo
    from std_msgs.msg import Bool, String, Float32MultiArray
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException

    class MainController(Node):
        def __init__(self):
            super().__init__('kenny_main_controller')
            self.mutex = threading.RLock()
            self.mapping = AdaptiveMap(environment)
            self.policy = policy
            self.drive_mode = getattr(args, 'drive_mode', 'velocity')
            self.bounded_pwm = self.drive_mode == 'pwm' and getattr(args, 'commissioning', False)
            self.supervised_continuous = self.drive_mode == 'velocity' and getattr(args, 'commissioning', False)
            self.commissioning_clearance = self.bounded_pwm or self.supervised_continuous
            self.pwm_navigation = PwmNavigator()
            self.motor_status = {}
            self.motor_time = -math.inf
            self.last_pwm = [0.,0.]
            self.pwm_stage = 'Idle'
            self.roaming = getattr(args,'roam',False)
            self.local_pose = None
            self.roam_goal = None
            self.roam_route = []
            self.roam_reason = 'Waiting for measured local clearance'
            self.roam_direction = None
            self.roam_deadline = None
            self.scan_history=MeasuredScanHistory()
            self.direction_stability=DirectionStability()
            self.roam_clearance={}
            self.alignment = None
            self.initial_pose = None if getattr(args, 'auto_marker_start', False) else args.initial_pose or [*environment['start'], 0.]
            self.marker_start_sequence = None
            self.marker_start_samples = []
            self.marker_start_time = -math.inf
            self.marker_ranges = {}
            self.arrival_source = None
            self.aruco_heading_pose = None
            self.aruco_heading_time = -math.inf
            self.heading_source = 'slam_policy'
            self.heading_error_deg = None
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.broadcaster = StaticTransformBroadcaster(self)
            self.pose = None
            self.pose_time = self.odom_time = self.map_time = -math.inf
            self.absolute_pose_time = -math.inf
            self.last_slam_stamp = -math.inf
            self.last_odom_stamp = -math.inf
            self.pose_observed_at = None
            self.velocity = [0., 0.]
            self.uncertainty = math.inf
            self.slam_uncertainty = math.inf
            self.anchor = MarkerAnchor(getattr(args, 'marker_sigma', .15))
            self.odom_distance = 0.
            self.odom_position = None
            self.marker_time = -math.inf
            self.sensors = {}
            self.goal_job = None
            self.goal = None
            self.route = []
            self.last_plan = -math.inf
            self.planned_revision = -1
            self.planned_goal = None
            self.status = 'Model loaded; waiting for live SLAM, pose, and sensors'
            self.guard_reason = ''
            self.stop_gate = StopGate()
            self.supervised_heartbeat = -math.inf
            self.stop_callbacks = MutuallyExclusiveCallbackGroup()
            self.last_command = [0., 0.]
            self.motion_allowed = False
            self.last_tf_error = ''
            self.sensor_errors = {}
            self.depth_observation = {}
            self.inference_ms = None
            self.decision_sequence = 0
            self.odom_sequence = 0
            self.last_map = None
            self.map_event = threading.Event()
            self.depth_info = None
            self.plan_event = threading.Event()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub = self.create_publisher(OccupancyGrid, '/kenny/map', qos)
            self.command_pub = self.create_publisher(TwistStamped, '/kenny/cmd_vel', 10)
            self.allow_pub = self.create_publisher(Bool, '/kenny/motion_allowed', 10)
            self.pwm_pub = self.create_publisher(Float32MultiArray, '/kenny/manual_pwm', 1)
            self.status_pub = self.create_publisher(String, '/kenny/controller/status', 10)
            self.mission_pub = self.create_publisher(String, '/kenny/mission/events', 20)
            self.queue_pub = self.create_publisher(String, '/kenny/requests/status', 10)
            self.last_mission_event = None
            self.last_plan_event = None
            self.last_motion_event = None
            self.route_pub = self.create_publisher(Route, '/kenny/route', 1)
            self.create_subscription(OccupancyGrid, args.slam_topic, self.on_map, qos)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_depth_info, qos_profile_sensor_data)
            self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, args.slam_pose_topic, self.on_slam_pose, 10)
            self.create_subscription(Float32MultiArray, '/kenny/downward', self.on_downward, qos_profile_sensor_data)
            self.create_subscription(MarkerArray, '/kenny/marker_observations', self.on_markers, 10)
            self.create_subscription(String, '/kenny/aruco/status', self.on_marker_start, 10)
            self.create_subscription(String, '/kenny/motors/status', self.on_motor_status, 10)
            self.create_subscription(Bool, '/kenny/emergency_stop', self.on_emergency_stop, qos,
                                      callback_group=self.stop_callbacks)
            self.create_subscription(Bool, '/kenny/supervised_clearance', self.on_supervised_clearance, 1)
            self.create_timer(.05 if self.drive_mode=='pwm' else policy.config.dt, self.control)
            self.create_timer(.1, self.update_pose)
            self.create_timer(1., self.publish_status)
            self.create_timer(10., self.persist)
            self.create_timer(1., self.plan_event.set)

        def fresh_stamp(self, stamp, limit=.5):
            age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(stamp)
            return -.1 <= age <= limit

        def sensor_transform(self, frame, stamp):
            # Missing calibration must not block sensor callbacks and starve the stop heartbeat.
            tf = self.buffer.lookup_transform(args.base_frame, frame, rclpy.time.Time())
            if stamp_seconds(tf.header.stamp) and not self.fresh_stamp(tf.header.stamp):
                raise TransformException('Sensor-to-base transform is stale')
            p, q = tf.transform.translation, tf.transform.rotation
            return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]

        def mission_event(self, event):
            with self.mutex:
                if event['stage'] == 'waiting':
                    errors = [self.last_tf_error, *self.sensor_errors.values()]
                    event = {**event, 'detail': event['detail'] + (' · ' + ' · '.join(dict.fromkeys(e for e in errors if e)) if any(errors) else '')}
                self.last_mission_event = event
            self.mission_pub.publish(String(data=json.dumps(event, allow_nan=False)))
            self.get_logger().info('Request process: ' + json.dumps(event, allow_nan=False))

        def mission_queue(self, jobs):
            self.queue_pub.publish(String(data=json.dumps({'jobs': jobs, 'polled_at': time.time()}, allow_nan=False)))

        def update_pose(self):
            try:
                tf = self.buffer.lookup_transform(args.slam_frame, args.base_frame, rclpy.time.Time())
                if not self.fresh_stamp(tf.header.stamp, .5):
                    return
                p = tf.transform.translation
                self.last_tf_error = ''
                slam_pose = [p.x, p.y, yaw(tf.transform.rotation)]
                with self.mutex:
                    if self.alignment is None:
                        if self.initial_pose is None:
                            self.last_tf_error = 'Waiting for a consistent surveyed ArUco robot pose and calibrated camera-to-base TF'
                            return
                        # Align the first real SLAM pose to the supplied approximate
                        # starting pose, without seeding SLAM with invented scans.
                        self.alignment = compose(self.initial_pose, inverse(slam_pose))
                        msg = TransformStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id, msg.child_frame_id = 'map', args.slam_frame
                        msg.transform.translation.x, msg.transform.translation.y = self.alignment[:2]
                        msg.transform.rotation.z = math.sin(self.alignment[2] / 2)
                        msg.transform.rotation.w = math.cos(self.alignment[2] / 2)
                        self.broadcaster.sendTransform(msg)
                    stamp = stamp_seconds(tf.header.stamp)
                    if stamp <= self.last_slam_stamp:
                        return
                    self.last_slam_stamp = stamp
                    self.pose = compose(self.alignment, slam_pose)
                    self.pose_time = time.monotonic()
                    self.absolute_pose_time = self.pose_time
                    if not math.isfinite(self.uncertainty) and getattr(args, 'commissioning', False):
                        self.uncertainty = .35
                    self.pose_observed_at = stamp_seconds(tf.header.stamp)
                    # Marker corrections change map->SLAM alignment as well as
                    # the internal pose; publish the same frame used by planning.
                    msg = TransformStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id, msg.child_frame_id = 'map', args.slam_frame
                    msg.transform.translation.x, msg.transform.translation.y = self.alignment[:2]
                    msg.transform.rotation.z = math.sin(self.alignment[2] / 2)
                    msg.transform.rotation.w = math.cos(self.alignment[2] / 2)
                    self.broadcaster.sendTransform(msg)
                    pending = self.last_map
                if pending is not None and self.map_time == -math.inf:
                    self.on_map(pending)
            except TransformException as error:
                self.last_tf_error = str(error)
                return

        def on_marker_start(self, msg):
            try:
                packet = json.loads(msg.data)
                age = self.get_clock().now().nanoseconds / 1e9 - packet['observed_at']
                estimate = packet.get('robot_pose')
                if not -.1 <= age <= .5 or packet['dictionary'] != environment['dictionary']:
                    return
                with self.mutex:
                    if packet['sequence'] == self.marker_start_sequence:
                        return
                    self.marker_start_sequence = packet['sequence']
                    now = time.monotonic()
                    for detection in packet.get('detections', []):
                        distance = detection.get('depth_range_m')
                        mid = detection.get('id')
                        if (type(mid) is not int or mid not in self.mapping.markers or
                                type(distance) not in (int,float) or not math.isfinite(distance) or
                                not .1 <= distance <= 2.5 or detection.get('depth_samples',0) <= 0):
                            continue
                        sightings = self.marker_ranges.setdefault(mid,deque(maxlen=3))
                        if sightings and now-sightings[-1][0] > .75:
                            sightings.clear()
                        sightings.append((now,float(distance)))
                    if not estimate:
                        return
                    pose = np.asarray(estimate['pose'], dtype=float)
                    if pose.shape != (3,) or not np.isfinite(pose).all():
                        return
                    if time.monotonic() - self.marker_start_time > .75:
                        self.marker_start_samples = []
                    self.marker_start_time = time.monotonic()
                    if estimate.get('depth_supported_ids'):
                        reference = self.pose if self.pose is not None and time.monotonic()-self.pose_time <= .5 else None
                        accepted = self.anchor.observe(pose, time.monotonic(), self.odom_distance, reference)
                        if accepted:
                            self.aruco_heading_pose = list(pose)
                            self.aruco_heading_time = now
                        if accepted and self.pose is not None:
                            old = list(self.pose)
                            error = math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2]))
                            self.pose = [old[0]+.35*(pose[0]-old[0]), old[1]+.35*(pose[1]-old[1]), old[2]+.35*error]
                            if self.alignment is not None:
                                self.alignment = compose(compose(self.pose, inverse(old)), self.alignment)
                            self.pose_time = time.monotonic()
                            self.absolute_pose_time = self.pose_time
                            self.pose_observed_at = packet['observed_at']
                    samples = self.marker_start_samples
                    if samples and (np.linalg.norm(pose[:2] - samples[-1][:2]) > .15 or abs(math.atan2(math.sin(pose[2]-samples[-1][2]), math.cos(pose[2]-samples[-1][2]))) > .15):
                        samples.clear()
                    samples.append(pose)
                    self.marker_start_samples = samples[-3:]
                    if len(self.marker_start_samples) == 3 and self.alignment is None:
                        values = np.asarray(self.marker_start_samples)
                        self.initial_pose = [*np.median(values[:, :2], axis=0), math.atan2(np.sin(values[:, 2]).mean(), np.cos(values[:, 2]).mean())]
            except (ValueError, KeyError, TypeError):
                return

        def on_emergency_stop(self, msg):
            self.stop_gate.update(msg.data)
            if msg.data:
                with self.mutex:
                    self.publish_command([0., 0.], False)

        def on_supervised_clearance(self, msg):
            with self.mutex:
                self.supervised_heartbeat = time.monotonic() if msg.data else -math.inf

        def on_motor_status(self, msg):
            try:
                status=json.loads(msg.data)
                if not isinstance(status,dict):
                    return
                with self.mutex:
                    self.motor_status=status
                    self.motor_time=time.monotonic()
            except ValueError:
                return

        def on_map(self, msg):
            if msg.header.frame_id != args.slam_frame or not self.fresh_stamp(msg.header.stamp, 3.):
                return
            with self.mutex:
                self.last_map = msg
                if self.alignment is not None:
                    self.map_event.set()

        def update_map(self):
            with self.mutex:
                msg, alignment = self.last_map, self.alignment
            if msg is None or alignment is None or not self.fresh_stamp(msg.header.stamp, 3.):
                return
            p = msg.info.origin.position
            grid = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                    'origin': [p.x, p.y, yaw(msg.info.origin.orientation)], 'data': list(msg.data)}
            # Rebuild a separate map: the control and planning threads retain a
            # consistent published grid while resampling runs off their lock.
            replacement = AdaptiveMap(environment)
            try:
                replacement.update_slam(grid, alignment)
            except ValueError as error:
                self.get_logger().error(str(error))
                return
            with self.mutex:
                if alignment != self.alignment:
                    self.map_event.set()
                    return
                # A newer map remains queued, but publishing this fresh result
                # avoids starving map_time when rebuild takes over one frame.
                replacement.markers = self.mapping.markers
                replacement.landmarks = self.mapping.landmarks
                replacement.revision = self.mapping.revision + 1
                self.mapping = replacement
                self.map_time = time.monotonic()
                self.plan_event.set()
            grid = replacement.grid
            result = OccupancyGrid()
            result.header.frame_id = 'map'
            result.header.stamp = self.get_clock().now().to_msg()
            result.info.resolution = grid['resolution']
            result.info.width, result.info.height = grid['width'], grid['height']
            result.info.origin.position.x, result.info.origin.position.y = grid['origin'][:2]
            result.info.origin.orientation.w = 1.
            result.data = grid['data']
            self.map_pub.publish(result)

        def on_depth_info(self, msg):
            if msg.k[0] > 0 and msg.k[4] > 0:
                self.depth_info = msg

        def on_scan(self, msg):
            if not self.fresh_stamp(msg.header.stamp):
                self.sensor_errors['lidar'] = 'Rejected stale/future scan timestamp'
                return
            try:
                translation, quaternion = self.sensor_transform(msg.header.frame_id, msg.header.stamp)
                packet = lidar_features(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                        msg.range_max, translation, quaternion, self.policy.robot.lidar_range)
            except (TransformException, ValueError) as error:
                self.sensor_errors['lidar'] = str(error)
                return
            with self.mutex:
                self.sensor_errors.pop('lidar', None)
                self.sensors['lidar'] = (*packet, time.monotonic())
                age=self.get_clock().now().nanoseconds/1e9-stamp_seconds(msg.header.stamp)
                self.scan_history.add(*packet,time.monotonic()-max(0.,age),self.local_pose)

        def on_depth(self, msg):
            if not self.fresh_stamp(msg.header.stamp):
                self.sensor_errors['depth'] = 'Rejected stale/future depth timestamp'
                return
            try:
                if self.depth_info is None:
                    raise ValueError('Waiting for depth camera calibration')
                translation, quaternion = self.sensor_transform(msg.header.frame_id, msg.header.stamp)
                points = depth_image_points(msg, self.depth_info)
                observation = {}
                depth, floor = depth_features(points, translation, quaternion, self.policy.robot, observation)
            except (TransformException, ValueError, AssertionError) as error:
                self.sensor_errors['depth'] = str(error)
                return
            with self.mutex:
                self.sensor_errors.pop('depth', None)
                now = time.monotonic()
                self.sensors['depth'], self.sensors['floor'] = (*depth, now), (*floor, now)
                self.depth_observation = observation

        def on_downward(self, msg):
            # [hazard_left, hazard_center, hazard_right, valid_left, valid_center, valid_right].
            data = np.asarray(msg.data)
            if data.shape != (6,) or not np.isfinite(data).all() or not np.isin(data, [0, 1]).all():
                return
            with self.mutex:
                self.sensors['downward'] = (data[:3].astype(bool), data[3:].astype(bool), time.monotonic())

        def on_odom(self, msg):
            if msg.header.frame_id != 'odom' or msg.child_frame_id != args.base_frame or not self.fresh_stamp(msg.header.stamp):
                return
            v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            if not np.isfinite([v, w]).all():
                return
            with self.mutex:
                stamp = stamp_seconds(msg.header.stamp)
                if stamp <= self.last_odom_stamp:
                    return
                position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
                if not np.isfinite(position).all():
                    return
                if self.odom_position is not None:
                    self.odom_distance += math.dist(position, self.odom_position)
                self.odom_position = position
                new_local = [*position,yaw(msg.pose.pose.orientation)]
                if not np.isfinite(new_local).all():
                    return
                now = time.monotonic()
                if self.alignment is None and self.initial_pose is not None and self.local_pose is not None:
                    # Keep the latched marker start in step with measured motion
                    # while waiting for the first SLAM transform.
                    self.initial_pose = compose(self.initial_pose, compose(inverse(self.local_pose), new_local))
                if self.pose is not None and self.local_pose is not None and now-self.absolute_pose_time <= 2.5:
                    self.pose = compose(self.pose, compose(inverse(self.local_pose), new_local))
                    self.pose_time = now
                    self.pose_observed_at = stamp_seconds(msg.header.stamp)
                self.local_pose = new_local
                self.last_odom_stamp = stamp
                self.velocity, self.odom_time = [v, w], time.monotonic()
                self.odom_sequence += 1

        def on_slam_pose(self, msg):
            if msg.header.frame_id != args.slam_frame or not self.fresh_stamp(msg.header.stamp):
                return
            covariance = np.asarray(msg.pose.covariance)
            if not np.isfinite(covariance).all() or min(covariance[0], covariance[7]) < 0:
                return
            with self.mutex:
                self.slam_uncertainty = max(.02, math.sqrt(max(covariance[0], covariance[7])))
                if self.anchor.pose is None:
                    self.uncertainty = self.slam_uncertainty

        def on_markers(self, msg):
            with self.mutex:
                if self.alignment is None:
                    return
                for marker in msg.markers:
                    if marker.header.frame_id != args.slam_frame or not self.fresh_stamp(marker.header.stamp):
                        continue
                    p = marker.pose.position
                    if self.mapping.observe_marker(marker.id, [p.x, p.y, yaw(marker.pose.orientation)], stamp_seconds(marker.header.stamp)):
                        self.marker_time = time.monotonic()

        def navigation_sensors(self, now):
            sensors = dict(self.sensors)
            recent, _ = self.scan_history.packet(now, self.local_pose)
            if recent is not None:
                sensors['lidar'] = recent
            return sensors

        def snapshot(self, ignore_stop=False):
            with self.mutex:
                now = time.monotonic()
                supervised = now-self.supervised_heartbeat <= .25
                if self.anchor.pose is not None:
                    self.uncertainty = self.anchor.uncertainty(now, self.odom_distance)
                reason = '' if ignore_stop else self.stop_gate.reason()
                if reason:
                    pass
                elif self.pose is None or now - self.pose_time > .5:
                    reason = 'SLAM pose stale/unavailable'
                elif self.mapping.grid is None:
                    reason = 'SLAM map stale/unavailable'
                elif now - self.odom_time > .5 and not self.commissioning_clearance:
                    reason = 'measured odometry stale/unavailable'
                elif self.uncertainty > 2.5 and not supervised:
                    reason = 'SLAM pose covariance unavailable/high'
                else:
                    _, reason = guard([0., 0.], self.navigation_sensors(now), now, uncertainty=self.uncertainty,
                                      require_downward=not getattr(args, 'flat_floor', False),
                                      commissioning=getattr(args, 'commissioning', False),
                                      bounded_pwm=self.commissioning_clearance or supervised)
                    # Translation clearance is checked against the actual command;
                    # do not prevent a footprint-clear escape pivot at readiness.
                    if reason == 'observed_obstacle' and self.drive_mode == 'velocity':
                        reason = ''
                return {'ready': not reason, 'reason': reason, 'pose': self.pose,
                        'velocity': list(self.velocity), 'odom_sequence': self.odom_sequence}

        def destination(self, job):
            with self.mutex:
                return self.mapping.destination(job)

        def tracking_packet(self):
            with self.mutex:
                if self.pose is None or time.monotonic() - self.pose_time > .5:
                    return None
                route = self.route
                if len(route) > 200:
                    route = [route[i] for i in np.linspace(0, len(route) - 1, 200).astype(int)]
                return {'pose': {'frame_id': 'map', 'x': self.pose[0], 'y': self.pose[1], 'yaw': self.pose[2]},
                        'observed_at': self.pose_observed_at, 'source': 'physical',
                        'uncertainty_m': self.uncertainty if math.isfinite(self.uncertainty) else None,
                        'status': 'navigating' if self.goal_job and not self.guard_reason else 'waiting' if self.goal_job else 'idle',
                        'detail': (self.guard_reason or self.status)[:256],
                        'goal': {'x': self.goal[0], 'y': self.goal[1], 'marker_id': self.goal_job['marker_id']} if self.goal_job and self.goal else None,
                        'job_id': self.goal_job.get('id') if self.goal_job else None, 'route': route}

        def set_goal(self, job):
            with self.mutex:
                self.goal_job = job
                self.goal = self.mapping.destination(job)
                self.route = []
                self.last_plan = -math.inf
                self.policy.destination_changed()
                self.pwm_navigation.reset()
                self.plan_event.set()

        def plan_once(self):
            # Route search can be slower than one control tick on a large map.
            # Keep it off the ROS sensor/inference callback and outside the lock.
            with self.mutex:
                if self.goal_job is None or self.pose is None:
                    return
                job_id = self.goal_job['id'] if 'id' in self.goal_job else self.goal_job['marker_id']
                goal = self.mapping.destination(self.goal_job)
                pose, grid, revision = list(self.pose), self.mapping.grid, self.mapping.revision
            route = Planner(grid, self.policy.robot.radius).path(pose, goal)
            if not route and getattr(self.mapping, 'prior', None) is not None:
                # Fallback to the surveyed simulation/prior map if live SLAM has local gaps
                route = Planner(self.mapping.prior, self.policy.robot.radius).path(pose, goal)
            direct_commissioning = (not route and getattr(args, 'commissioning', False) and
                                    math.dist(pose[:2], goal[:2]) <= .75)
            if direct_commissioning:
                # A short target directly in front can be reached under the live
                # depth/LiDAR guard even when SLAM has not marked the robot cell.
                route = [pose, goal]
            with self.mutex:
                if self.goal_job is None:
                    return
                current_id = self.goal_job.get('id', self.goal_job['marker_id'])
                if current_id != job_id or self.mapping.destination(self.goal_job) != goal:
                    self.plan_event.set()
                    return
                self.route = route
                self.last_plan, self.planned_revision, self.planned_goal = time.monotonic(), revision, goal
                self.publish_route()
                key = (job_id, bool(route))
                if key != self.last_plan_event:
                    self.last_plan_event = key
                    self.mission_event({'time': time.time(), 'stage': 'route_planned' if route else 'route_blocked',
                        'job_id': self.goal_job.get('id'), 'marker_id': self.goal_job['marker_id'],
                        'detail': ('Direct commissioning route: live sensor guard required' if direct_commissioning else
                                   f'Route ready: {len(route)} points' if route else
                                   'No traversable route in the current map')})

        def publish_command(self, command, allowed):
            if self.goal_job:
                key = (self.goal_job.get('id'), bool(allowed), self.guard_reason if not allowed else '')
                if key != self.last_motion_event:
                    self.last_motion_event = key
                    self.mission_event({'time': time.time(), 'stage': 'motion_permitted' if allowed else 'motion_blocked',
                        'job_id': self.goal_job.get('id'), 'marker_id': self.goal_job['marker_id'],
                        'detail': 'Following planned route with measured sensor checks' if allowed else self.guard_reason or 'Motion permission withheld'})
            self.last_command = list(command)
            self.motion_allowed = allowed
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = args.base_frame
            msg.twist.linear.x, msg.twist.angular.z = float(command[0]), float(command[1])
            self.command_pub.publish(msg)
            if self.drive_mode=='pwm':
                self.pwm_pub.publish(Float32MultiArray(data=self.last_pwm if allowed else [0.,0.]))
            self.allow_pub.publish(Bool(data=allowed))
            self.policy.record_executed(command)

        def control(self):
            if self.roaming:
                self.control_roaming()
                return
            if self.drive_mode=='pwm':
                self.control_pwm()
                return
            with self.mutex:
                now = time.monotonic()
                snapshot = self.snapshot()
                command, allowed = [0., 0.], False
                if self.goal_job is not None and snapshot['ready']:
                    self.goal = self.mapping.destination(self.goal_job)
                    if now - self.last_plan > 1. or self.planned_revision != self.mapping.revision or self.planned_goal != self.goal:
                        self.plan_event.set()
                    if self.planned_goal != self.goal:
                        self.route = []
                    if self.route or self.supervised_continuous:
                        grid = self.mapping.grid
                        if self.route:
                            inference_start = time.perf_counter()
                            command = self.policy.predict(self.navigation_sensors(now), self.pose, self.goal, self.route,
                                [grid['width'] * grid['resolution'], grid['height'] * grid['resolution']],
                                self.velocity, self.uncertainty, now - self.marker_time, now, bool(self.guard_reason))
                            self.inference_ms = (time.perf_counter() - inference_start) * 1000
                        else:
                            command = [args.max_speed, 0.]
                            self.inference_ms = None
                        distance = math.dist(self.pose[:2], self.goal[:2])
                        if distance <= .25:
                            angle = math.atan2(math.sin(self.goal[2] - self.pose[2]), math.cos(self.goal[2] - self.pose[2]))
                            command = [0., max(-.3, min(.3, angle)) if abs(angle) > .25 else 0.]
                        self.heading_source, self.heading_error_deg = 'slam_policy', None
                        marker_fresh = self.aruco_heading_pose is not None and now-self.aruco_heading_time <= .75
                        if self.pose is not None:
                            from controller.pwm_navigation import aruco_guided_velocity
                            heading_pose = self.aruco_heading_pose if marker_fresh else self.pose
                            route = self.route or [list(self.pose[:2]), list(self.goal[:2])]
                            command, error = aruco_guided_velocity(command, self.pose, route,
                                self.goal, heading_pose, args.max_speed)
                            if error is not None:
                                self.heading_source = 'surveyed_aruco' if marker_fresh else 'slam_fallback'
                                self.heading_error_deg = math.degrees(error)
                        command, self.guard_reason = guard(command, self.navigation_sensors(now), now, self.policy.robot.radius,
                                                           args.max_speed, uncertainty=self.uncertainty,
                                                           require_downward=not getattr(args, 'flat_floor', False),
                                                           commissioning=getattr(args, 'commissioning', False),
                                                            measured_speed=self.velocity[0])
                        allowed = not self.guard_reason
                    else:
                        self.guard_reason = 'No route on the current refined map'
                else:
                    self.guard_reason = snapshot['reason'] if self.goal_job else ''
                self.publish_command(command, allowed)

        def control_pwm(self):
            with self.mutex:
                now=time.monotonic()
                snapshot=self.snapshot()
                reason=snapshot['reason']
                if not reason and (now-self.motor_time>.5 or not self.motor_status.get('bridge',{}).get('connected')):
                    reason='Motor telemetry stale/unavailable'
                if not reason and (self.motor_status.get('faults') or self.motor_status.get('estop') or self.motor_status.get('bridge',{}).get('motion_inhibited')):
                    reason='Motor fault or command transport inhibited'
                if not reason and not self.motor_status.get('manualPwmSupported'):
                    reason='Direct PWM firmware not supported'
                clear=not reason and self.goal_job is not None and (bool(self.route) or self.bounded_pwm)
                if self.goal_job is not None and (not self.route or now-self.last_plan>1 or self.planned_revision!=self.mapping.revision):
                    self.plan_event.set()
                if clear:
                    self.goal=self.mapping.destination(self.goal_job)
                    step_route = self.route or ([list(self.pose[:2]), list(self.goal[:2])] if self.bounded_pwm else [])
                    if now-self.last_plan>1 or self.planned_revision!=self.mapping.revision:
                        self.plan_event.set()
                    # PWM is not a velocity command. Guard a conservative initial
                    # step speed, increasing clearance with measured wheel speed.
                    anticipated = max(.3, abs(self.velocity[0]))
                    intended, _ = self.pwm_navigation.target(self.pose, step_route, self.goal)
                    turning = intended[0] * intended[1] < 0 or abs(self.velocity[1]) > .1
                    trans_speed = 0. if turning else anticipated
                    rot_speed = .3 if turning else 0.
                    guarded, reason = guard([trans_speed, rot_speed], self.navigation_sensors(now), now,
                        self.policy.robot.radius, speed_limit=anticipated, uncertainty=self.uncertainty,
                        require_downward=not getattr(args, 'flat_floor', False),
                        commissioning=getattr(args, 'commissioning', False), measured_speed=trans_speed,
                        bounded_pwm=self.bounded_pwm)
                    # Direct PWM cannot execute a velocity-scaled command.
                    if not reason and not self.bounded_pwm and guarded[0] < min(anticipated, .05 if getattr(args, 'commissioning', False) else anticipated):
                        reason = 'PWM translation clearance insufficient'
                    clear=not reason
                if self.goal_job is not None and self.pose is not None:
                    step_route = self.route or ([list(self.pose[:2]), list(self.goal[:2])] if self.bounded_pwm else [])
                    self.last_pwm,self.pwm_stage=self.pwm_navigation.update(now,self.pose,step_route,
                        self.goal,self.motor_status,self.velocity,self.odom_sequence,clear,
                        bounded_pwm=self.bounded_pwm)
                    if clear and not self.route:
                        self.pwm_stage += ' · direct measured-clearance commissioning (map route unavailable)'
                else:
                    self.last_pwm=[0.,0.]
                    self.pwm_navigation.reset()
                    self.pwm_stage='Waiting for request and verified localization'
                self.guard_reason=reason or ('' if clear else 'No traversable route on the current map' if self.goal_job else 'Waiting for a request')
                allowed=clear and any(self.last_pwm)
                self.publish_command([0.,0.],allowed)

        def control_roaming(self):
            with self.mutex:
                now=time.monotonic()
                reason=self.stop_gate.reason()
                if not reason and (self.local_pose is None or now-self.odom_time>.5):
                    reason='Local encoder odometry unavailable/stale'
                if not reason and (now-self.motor_time>.5 or not self.motor_status.get('bridge',{}).get('connected')):
                    reason='Motor telemetry unavailable/stale'
                if not reason and (self.motor_status.get('faults') or self.motor_status.get('estop') or self.motor_status.get('bridge',{}).get('motion_inhibited')):
                    reason='Firmware fault or motor transport inhibited'
                if not reason and not self.motor_status.get('manualPwmSupported'):
                    reason='Direct PWM firmware unavailable'
                if not reason and self.roam_deadline is None:
                    self.roam_deadline=now+60.
                if not reason and now>=self.roam_deadline:
                    reason='60-second local roaming session finished; motors stopped'
                direction=None
                if not reason:
                    sensors=dict(self.sensors)
                    recent,history=self.scan_history.packet(now,self.local_pose)
                    if recent is not None:
                        sensors['lidar']=recent
                    diagnostics={'scan_history':history}
                    direction,reason=roaming_direction(sensors,now,self.velocity,self.policy.robot.radius,
                        preferred=self.roam_direction,diagnostics=diagnostics)
                    self.roam_clearance=diagnostics
                    stable=self.direction_stability.update(direction,now)
                    if direction is not None and stable is None:
                        reason='Waiting for consistent measured clearance before startup'
                    direction=stable
                else:
                    self.direction_stability.update(None,now)
                clear=direction is not None and not self.stop_gate.reason()
                if clear and self.pwm_navigation.phase=='driving' and direction!=self.roam_direction:
                    clear=False
                    reason='Clearance changed; stopping before choosing a new direction'
                if clear and self.pwm_navigation.phase in ('idle','coasting'):
                    x,y,heading=self.local_pose
                    self.roam_direction=direction
                    if direction==0:
                        self.roam_goal=[x+.7*math.cos(heading),y+.7*math.sin(heading),heading]
                        self.roam_route=[[x,y],self.roam_goal[:2]]
                    else:
                        self.roam_goal=[x,y,heading+direction]
                        self.roam_route=[[x,y]]
                if self.local_pose is not None and self.roam_goal is not None:
                    self.last_pwm,self.pwm_stage=self.pwm_navigation.update(now,self.local_pose,self.roam_route,
                        self.roam_goal,self.motor_status,self.velocity,self.odom_sequence,clear)
                    if self.pwm_navigation.phase=='idle':
                        self.roam_goal=None
                else:
                    self.last_pwm=[0.,0.]
                    self.pwm_stage=reason or 'Waiting for local direction'
                self.roam_reason=reason
                self.guard_reason='' if clear else reason
                self.status='Local roaming: '+reason
                self.publish_command([0.,0.],clear and any(self.last_pwm))

        def publish_route(self):
            from geometry_msgs.msg import PoseStamped
            message = Route()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            for x, y in self.route:
                point = PoseStamped()
                point.header = message.header
                point.pose.position.x, point.pose.position.y = x, y
                point.pose.orientation.w = 1.
                message.poses.append(point)
            self.route_pub.publish(message)

        def arrived(self):
            with self.mutex:
                if self.goal_job is None:
                    return False
                now = time.monotonic()
                if confirmed_marker_proximity(self.marker_ranges.get(self.goal_job['marker_id'],()),now):
                    self.arrival_source = 'three fresh depth-supported marker sightings'
                    return True
                if (self.goal is None or self.pose is None or now-self.pose_time > .5 or
                        not math.isfinite(self.uncertainty) or self.uncertainty > .75):
                    return False
                marker = self.mapping.markers.get(self.goal_job['marker_id'])
                close = math.dist(self.pose[:2],self.goal[:2]) <= .45 or (marker is not None and
                        math.dist(self.pose[:2],[marker['x'],marker['y']]) <= .55)
                if close:
                    self.arrival_source = 'fresh localized pose near requested marker'
                return close

        def stop(self):
            with self.mutex:
                self.goal = self.goal_job = None
                self.route = []
                self.publish_command([0., 0.], False)
            deadline = time.monotonic() + 5.
            confirmed, last_sequence = 0, -1
            while time.monotonic() < deadline:
                sample = self.snapshot(ignore_stop=True)
                motor_stopped = not self.motor_status.get('armed', True) or all(abs(r) < 1.0 for r in self.motor_status.get('rpm', [1]))
                vel_stopped = abs(sample['velocity'][0]) < .04 and abs(sample['velocity'][1]) < .06
                if sample['odom_sequence'] != last_sequence:
                    last_sequence = sample['odom_sequence']
                    confirmed = confirmed + 1 if (vel_stopped or motor_stopped) else 0
                    if confirmed >= 2:
                        return
                time.sleep(.05)
            if not self.motor_status.get('armed', False):
                return
            raise RuntimeError('Physical stopping not confirmed by fresh odometry; API trip remains blocked')

        def persist(self):
            with self.mutex:
                if self.map_time == -math.inf:
                    return
                result = self.mapping.export(self.pose)
            try:
                args.refined.parent.mkdir(parents=True, exist_ok=True)
                save(args.refined, result)
            except OSError as error:
                self.get_logger().error(f'Cannot save refined map: {error}')

        def publish_status(self):
            with self.mutex:
                snapshot = self.snapshot()
                now = time.monotonic()
                self.decision_sequence += 1
                def age(value):
                    return round(now - value, 3) if math.isfinite(value) else None
                sensors = {}
                for name in ('lidar', 'depth', 'floor', 'downward'):
                    packet = self.sensors.get(name)
                    sensors[name] = None if packet is None else {
                        'age_s': age(packet[2]), 'valid_fraction': float(np.mean(packet[1])),
                        'minimum': float(np.min(packet[0][packet[1]])) if np.any(packet[1]) else None,
                        'values': packet[0].tolist(), 'valid': packet[1].tolist()}
                policy_explanation = self.policy.last_explanation
                decision = {
                    'id': self.decision_sequence,
                    'recorded_at': time.time(),
                    'belief_pose': list(self.pose) if self.pose is not None else None,
                    'belief_pose_source': 'slam_covariance',
                    'actual_pose': list(self.local_pose) if self.local_pose is not None else None,
                    'goal': list(self.goal) if self.goal is not None else None,
                    'route_points': len(self.route),
                    'route_revision': self.mapping.revision,
                    'requested_command': list(policy_explanation['policy_command']) if policy_explanation else list(self.last_command),
                    'guarded_command': list(self.last_command),
                    'motion_allowed': bool(self.motion_allowed),
                    'guard': self.guard_reason or None,
                    'heading_source': self.heading_source,
                    'heading_error_deg': self.heading_error_deg,
                    'uncertainty_m': self.uncertainty if math.isfinite(self.uncertainty) else None,
                    'policy': policy_explanation,
                }
                message = {**snapshot, 'status': self.status, 'guard': self.guard_reason,
                            'ready_without_stop': self.snapshot(ignore_stop=True)['ready'],
                           'goal': self.goal, 'marker_id': self.goal_job['marker_id'] if self.goal_job else None,
                           'map_revision': self.mapping.revision, 'observed_cells': int(self.mapping.observed.sum()),
                            'model_resident': True, 'emergency_stop': self.stop_gate.reason(),
                            'terrain_profile': 'flat_floor' if getattr(args, 'flat_floor', False) else 'full_sensing',
                           'command': self.last_command, 'motion_allowed': self.motion_allowed,
                           'ages_s': {'pose': age(self.pose_time), 'map': age(self.map_time),
                                      'odom': age(self.odom_time), 'marker': age(self.marker_time)},
                           'sensors': sensors, 'route_points': len(self.route),
                            'uncertainty_m': self.uncertainty if math.isfinite(self.uncertainty) else None,
                            'localization_source': 'surveyed_rgbd_aruco + lidar_slam + encoders' if self.anchor.pose is not None else 'slam_covariance',
                            'commissioning': getattr(args, 'commissioning', False),
                            'drive_mode': self.drive_mode, 'manual_pwm': list(self.last_pwm),
                            'heading_source': self.heading_source, 'heading_error_deg': self.heading_error_deg,
                            'supervised_clearance': now-self.supervised_heartbeat <= .25,
                            'pwm_stage': self.pwm_stage,
                            'roaming': self.roaming, 'roam_reason': self.roam_reason,
                            'local_odom_pose': self.local_pose,
                            'roam_clearance': dict(self.roam_clearance),
                            'last_tf_error': self.last_tf_error, 'sensor_errors': dict(self.sensor_errors),
                            'depth_observation': dict(self.depth_observation),
                            'mission': self.last_mission_event,
                            'decision': decision,
                            'inference_ms': self.inference_ms}
                if self.roaming:
                    message.update(mode='local_roaming',ready=not bool(self.guard_reason),
                                   reason=self.guard_reason,goal=None,marker_id=None)
                self.status_pub.publish(String(data=json.dumps(message, allow_nan=False)))

    return MainController()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--api', default=os.environ.get('KENNY_API_URL', 'https://kenny-api.onrender.com'))
    parser.add_argument('--worker', default='kenny-main')
    parser.add_argument('--state', type=Path, default=ROOT / 'data/main-controller.json')
    parser.add_argument('--refined', type=Path, default=ROOT / 'data/refined-environment.json')
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--auto-marker-start', type=lambda value: value.lower() == 'true', default=False)
    parser.add_argument('--flat-floor', type=lambda value: value.lower() == 'true', default=False)
    parser.add_argument('--commissioning', type=lambda value: value.lower() == 'true', default=False)
    parser.add_argument('--drive-mode', choices=('velocity','pwm'), default='velocity')
    parser.add_argument('--roam', type=lambda value:value.lower()=='true', default=False)
    parser.add_argument('--marker-sigma', type=float, default=.15)
    parser.add_argument('--target-marker', type=int, default=-1)
    parser.add_argument('--slam-topic', default='/map')
    parser.add_argument('--slam-frame', default='slam_map')
    parser.add_argument('--slam-pose-topic', default='/pose')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--depth-topic', default='/camera/depth/image_raw')
    parser.add_argument('--switch-margin', type=float, default=.25)
    parser.add_argument('--trip-timeout', type=float, default=1800.)
    parser.add_argument('--max-speed', type=float, default=.1)
    args, ros_args = parser.parse_known_args()
    if not 0 < args.max_speed <= .1 or args.switch_margin < 0 or args.trip_timeout <= 0 or not np.isfinite([args.max_speed, args.switch_margin, args.trip_timeout]).all():
        parser.error('Use finite positive speed ≤0.1 m/s and timeout, and a nonnegative switch margin')
    if args.slam_frame == 'map':
        parser.error('SLAM frame must differ from the estimate map frame (use slam_map)')
    if not math.isfinite(args.marker_sigma) or args.marker_sigma <= 0:
        parser.error('Marker survey/mount uncertainty must be finite and positive')
    if args.auto_marker_start:
        args.initial_pose = None
    if args.initial_pose and not np.isfinite(args.initial_pose).all():
        parser.error('Initial pose must be finite')
    environment = json.loads(args.environment.read_text())
    policy = ResidentPolicy(args.model)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    rclpy.init(args=ros_args)
    node = make_node(args, policy, environment)
    api = CloudAPI(args.api, os.environ.get('KENNY_PI_TOKEN') or os.environ.get('KENNY_DISPATCH_TOKEN', ''))
    missions = None if args.roam else Missions(api, node, args.state, args.worker, args.switch_margin, args.trip_timeout,
                        target_marker=args.target_marker if args.target_marker >= 0 else None, resume_stopped=True)
    stopped = threading.Event()

    def receive():
        while rclpy.ok() and not stopped.is_set():
            try:
                node.status = missions.step()
            except (URLError, TimeoutError, ConnectionError) as error:
                node.status = f'API unavailable; retrying: {error}'
            except Exception as error:
                node.status = f'Mission controller retry after error: {error}'
                node.get_logger().error(f'Mission step error: {error}')
                stopped.wait(2.)
            stopped.wait(1.)

    thread = threading.Thread(target=receive, daemon=True)
    if not args.roam:
        thread.start()

    def plan():
        while rclpy.ok() and not stopped.is_set():
            node.plan_event.wait(1.)
            node.plan_event.clear()
            try:
                node.plan_once()
            except Exception as error:
                node.get_logger().error(f'Route planning failed: {error}')
                with node.mutex:
                    node.route = []

    planner_thread = threading.Thread(target=plan, daemon=True)
    planner_thread.start()
    def refresh_map():
        while rclpy.ok() and not stopped.is_set():
            if node.map_event.wait(.5):
                if stopped.is_set():
                    break
                node.map_event.clear()
                try:
                    node.update_map()
                except Exception as error:
                    node.get_logger().error(f'Map update failed: {error}')

    map_thread = threading.Thread(target=refresh_map, daemon=True)
    map_thread.start()
    tracker = None if args.roam else Tracker(api, missions.worker)

    def track():
        while rclpy.ok() and not stopped.is_set():
            try:
                tracker.publish(node.tracking_packet())
            except Exception as error:
                node.get_logger().warning(f'Live position upload unavailable: {error}')
            stopped.wait(1.)

    tracking_thread = threading.Thread(target=track, daemon=True)
    if not args.roam:
        tracking_thread.start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopped.set()
        node.plan_event.set()
        node.map_event.set()
        with node.mutex:
            node.goal = node.goal_job = None
            if rclpy.ok():
                node.publish_command([0., 0.], False)
        node.persist()
        executor.shutdown()
        if thread.is_alive():
            thread.join(timeout=16.)
        planner_thread.join(timeout=5.)
        map_thread.join(timeout=5.)
        if tracking_thread.is_alive():
            tracking_thread.join(timeout=16.)
        if missions is not None:
            missions.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
