"""Calibrated RGB ArUco poses for live landmark refinement (no RGB/depth registration)."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from .sensors import rotation
from .depth_cloud import project_depth


def marker_pose(corners, size, camera_matrix, distortion, maximum_error=2.):
    import cv2
    half = size / 2
    square = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    ok, vectors, translations, errors = cv2.solvePnPGeneric(square, np.asarray(corners, dtype=np.float64).reshape(4, 2),
        camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    ranked = sorted(zip(errors.ravel(), vectors, translations), key=lambda item: item[0])
    error, vector, translation = ranked[0]
    if not math.isfinite(float(error)) or error > maximum_error or translation[2, 0] <= 0:
        return None
    orientation, _ = cv2.Rodrigues(vector)
    return translation.ravel(), orientation


def make_detector(dictionary):
    import cv2
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMax = 63
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.minMarkerPerimeterRate = .015
    parameters.polygonalApproxAccuracyRate = .05
    parameters.minCornerDistanceRate = .02
    parameters.detectInvertedMarker = True
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def valid_quad(corners):
    points = np.asarray(corners, dtype=float).reshape(-1, 2)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    area = abs(np.sum(points[:, 0]*np.roll(points[:, 1], -1)-points[:, 1]*np.roll(points[:, 0], -1))) / 2
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0]*following[:, 1]-edges[:, 1]*following[:, 0]
    return bool(lengths.min() >= 8 and lengths.sum() >= 40 and area >= 100 and
                ((cross > 0).all() or (cross < 0).all()))


def decode_markers(image, detector, markers, camera=None, diagnostics=None):
    """Report decoded IDs even when calibration/TF cannot produce world poses."""
    import cv2
    attempts, rejected = [], []
    def agreed_ids():
        votes = {}
        for pass_index, (corners, found_ids, _) in enumerate(attempts):
            if found_ids is not None:
                for corner, mid in zip(corners, found_ids.ravel()):
                    if valid_quad(corner):
                        votes.setdefault(int(mid), []).append((pass_index, corner.reshape(4, 2).mean(axis=0)))
        agreed = {mid for mid, sightings in votes.items() if any(
            a != b and np.linalg.norm(x-y) <= 25 for a, x in sightings for b, y in sightings)}
        return agreed, votes
    def attempt(working, method, scale=1., selected_detector=None):
        found_corners, found_ids, discarded = (selected_detector or detector).detectMarkers(working)
        rejected.extend(c / scale for c in discarded)
        attempts.append(([c / scale for c in found_corners], found_ids, method))
    attempt(image, 'adaptive multi-window')
    resized = cv2.resize(image, None, fx=.75, fy=.75, interpolation=cv2.INTER_AREA)
    attempt(resized, 'scaled adaptive detection', .75)
    consistent_ids, votes = agreed_ids()
    if not consistent_ids:
        contrast = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(image)
        attempt(contrast, 'contrast-enhanced adaptive detection')
        consistent_ids, votes = agreed_ids()
    if not consistent_ids:
        # Shiny tile floors create illumination gradients larger than the usual
        # threshold windows. Normalize locally without loosening dictionary decoding.
        binary_parameters = detector.getDetectorParameters()
        binary_parameters.adaptiveThreshWinSizeMin = 3
        binary_parameters.adaptiveThreshWinSizeMax = 3
        binary_parameters.adaptiveThreshWinSizeStep = 1
        binary_detector = cv2.aruco.ArucoDetector(detector.getDictionary(), binary_parameters)
        for working, scale in ((resized, .75),):
            window = min(201, min(working.shape[:2]) // 2 * 2 - 1)
            if window < 3:
                continue
            for constant in (3, 7, 15):
                binary = cv2.adaptiveThreshold(working, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              cv2.THRESH_BINARY, window, constant)
                attempt(binary, 'illumination-normalized floor marker', scale, binary_detector)
        consistent_ids, votes = agreed_ids()
    # A decoded ID must agree across preprocessing passes. A single glare-induced
    # decode is not enough to establish a marker identity or the robot's map pose.
    chosen = {}
    for found_corners, found_ids, found_method in attempts:
        if found_ids is not None:
            for corner, mid in zip(found_corners, found_ids.ravel()):
                mid = int(mid)
                center = corner.reshape(4, 2).mean(axis=0)
                spatial_votes = sum(any(int(other_id) == mid and valid_quad(other) and
                    np.linalg.norm(other.reshape(4, 2).mean(axis=0)-center) <= 25
                    for other, other_id in zip(cs, ids.ravel())) for cs, ids, _ in attempts if ids is not None)
                if mid in consistent_ids and mid not in chosen and valid_quad(corner) and spatial_votes >= 2:
                    chosen[mid] = (corner, found_method)
    corners = [item[0] for item in chosen.values()]
    ids = np.array(list(chosen), dtype=np.int32).reshape(-1, 1) if chosen else None
    method = ' + '.join(dict.fromkeys(item[1] for item in chosen.values())) if chosen else 'No consistent decode'
    if diagnostics is not None:
        diagnostics.update(rejected_candidates=len(rejected), detection_method=method,
                           decoded_count=0 if ids is None else len(ids),
                           unconfirmed_ids=sorted(set(votes) - consistent_ids))
    detections, poses = [], {}
    if ids is None:
        return detections, poses
    for image_corners, marker_id in zip(corners, ids.ravel()):
        marker_id = int(marker_id)
        reference = markers.get(marker_id)
        detection = {'id': marker_id, 'corners': image_corners.reshape(4, 2).tolist(),
                     'registered': reference is not None, 'range_m': None,
                     'pose_published': False, 'pose_status': 'ID decoded'}
        if reference is None:
            detection['pose_status'] = 'ID not in selected map'
        elif camera is None:
            detection['pose_status'] = 'Camera calibration unavailable'
        else:
            result = marker_pose(image_corners, reference['size'], camera['matrix'], camera['distortion'])
            if result is None:
                detection['pose_status'] = 'Pose quality rejected'
            else:
                translation, orientation = result
                detection['range_m'] = float(np.linalg.norm(translation))
                if detection['range_m'] <= 3.5:
                    poses[marker_id] = result
                    detection['pose_status'] = 'Camera-relative pose; waiting for map TF'
                else:
                    detection['pose_status'] = 'Outside 3.5 m pose limit'
        detections.append(detection)
    return detections, poses


def surveyed_robot_pose(reference, translation, marker_rotation, base_translation, base_rotation):
    """Invert measured camera-from-marker using the surveyed marker and camera mount."""
    c, s = math.cos(reference['yaw']), math.sin(reference['yaw'])
    # Floor marker +Y follows map yaw + pi/2; wall +Z is its outward normal.
    world_rotation = (np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
                      if reference['mount'] == 'floor' else
                      np.array([[s, 0, c], [-c, 0, s], [0, -1., 0]]))
    camera_rotation = world_rotation @ marker_rotation.T
    camera_position = np.array([reference['x'], reference['y'], reference.get('z', 0.)]) - camera_rotation @ translation
    robot_rotation = camera_rotation @ base_rotation.T
    robot_position = camera_position - robot_rotation @ np.asarray(base_translation)
    # Robot is assumed on level floor. Allow reasonable camera tilt tolerances (up to 25 deg).
    if robot_rotation[2, 2] < math.cos(math.radians(25)) or abs(robot_position[2]) > .3:
        return None
    return [float(robot_position[0]), float(robot_position[1]), math.atan2(robot_rotation[1, 0], robot_rotation[0, 0])]


def depth_supported_translation(corners, translation, marker_rotation, points, matrix, distortion, diagnostics=None):
    """Intersect the RGB marker-center ray with its measured depth plane."""
    import cv2
    diagnostics = diagnostics if diagnostics is not None else {}
    def reject(reason):
        diagnostics['reason'] = reason
        return None
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > .1) & (points[:, 2] < 3.)]
    if len(points) < 12:
        return reject('Not enough valid depth measurements')
    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, distortion)
    pixels = pixels.reshape(-1, 2)
    polygon = np.asarray(corners).reshape(4, 2)
    polygon = polygon.mean(axis=0) + .65 * (polygon - polygon.mean(axis=0))
    # Convex-polygon mask without assuming registered RGB/depth pixels.
    edges = np.roll(polygon, -1, axis=0) - polygon
    delta = pixels[:, None, :] - polygon[None, :, :]
    cross = edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]
    points = points[np.all(cross >= 0, axis=1) | np.all(cross <= 0, axis=1)]
    diagnostics['marker_depth_points'] = len(points)
    if len(points) < 12:
        return reject('Not enough depth points on the RGB marker')
    center = np.median(points, axis=0)
    points = points[np.abs(points[:, 2] - center[2]) < .12]
    if len(points) < 12:
        return reject('Depth discontinuity on the marker')
    center = np.median(points, axis=0)
    _, _, vectors = np.linalg.svd(points - center, full_matrices=False)
    normal = vectors[-1]
    residual = np.abs((points - center) @ normal)
    diagnostics['median_z_m'] = float(center[2])
    diagnostics['plane_error_m'] = float(np.quantile(residual, .8))
    diagnostics['normal_difference_deg'] = math.degrees(math.acos(min(1., abs(float(np.dot(normal, marker_rotation[:, 2]))))))
    diagnostics['plane_normal_in_camera'] = list(map(float, normal if normal @ marker_rotation[:, 2] >= 0 else -normal))
    if diagnostics['plane_error_m'] > .025:
        return reject('Depth points do not form a consistent marker plane')
    if diagnostics['normal_difference_deg'] > 20:
        return reject('RGB and depth marker orientations disagree')
    denominator = float(normal @ translation)
    if abs(denominator) < 1e-6:
        return reject('Marker center ray is parallel to the depth plane')
    scale = float(normal @ center) / denominator
    diagnostics['depth_to_rgb_scale'] = scale
    if not .05 <= scale <= 10:
        return reject('Depth/RGB geometry has an implausible scale')
    if not .5 <= scale <= 1.5:
        diagnostics['size_warning'] = 'Measured depth disagrees with configured printed marker size'
    return np.asarray(translation) * scale, len(points), float(np.median(residual))


def make_node(args, environment):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException
    cv2.setNumThreads(1)
    markers = {m['id']: m for m in environment['markers']}
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, environment['dictionary']))
    detector = make_detector(dictionary)

    class Detector(Node):
        def __init__(self):
            super().__init__('kenny_marker_detector')
            self.camera = None
            self.depth = None
            self.depth_camera = None
            self.last_processed = -math.inf
            self.last_stamp = None
            self.sequence = 0
            self.latest = {'state': 'Waiting for RGB image', 'detections': []}
            self.cv = CvBridge()
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.pub = self.create_publisher(MarkerArray, '/kenny/marker_observations', 10)
            self.scan_pub = self.create_publisher(String, '/kenny/aruco/status', 10)
            self.create_subscription(CameraInfo, args.camera_info, self.intrinsics, qos_profile_sensor_data)
            latest_image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.image, self.image, latest_image_qos)
            self.create_subscription(Image, '/camera/depth/image_raw', lambda msg: setattr(self, 'depth', msg), latest_image_qos)
            self.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda msg: setattr(self, 'depth_camera', msg), qos_profile_sensor_data)
            self.create_timer(1., self.publish_scan)

        def publish_scan(self):
            age = None if self.last_stamp is None else self.get_clock().now().nanoseconds / 1e9 - self.last_stamp
            packet = {**self.latest, 'sequence': self.sequence, 'observed_at': self.last_stamp,
                      'dictionary': environment['dictionary'], 'image_age_s': age}
            if age is not None and not -.1 <= age <= .5:
                packet.update(state='RGB image stale', detections=[])
            self.scan_pub.publish(String(data=json.dumps(packet, allow_nan=False)))

        def intrinsics(self, msg):
            if msg.k[0] > 0 and msg.k[4] > 0 and np.isfinite(msg.k).all():
                self.camera = msg

        def image(self, msg):
            if time.monotonic() - self.last_processed < 1. / args.max_rate:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not -.1 <= self.get_clock().now().nanoseconds / 1e9 - stamp <= .5:
                return
            self.last_processed = time.monotonic()
            self.last_stamp = stamp
            self.sequence += 1
            camera = None
            if (self.camera is not None and self.camera.header.frame_id == msg.header.frame_id
                    and self.camera.width == msg.width and self.camera.height == msg.height):
                camera = {'matrix': np.asarray(self.camera.k).reshape(3, 3), 'distortion': np.asarray(self.camera.d)}
            try:
                image = self.cv.imgmsg_to_cv2(msg, desired_encoding='mono8')
                detection_diagnostics = {}
                detection_start = time.perf_counter()
                detections, poses = decode_markers(image, detector, markers, camera, detection_diagnostics)
                detection_diagnostics['processing_ms'] = round((time.perf_counter() - detection_start) * 1000, 1)
            except Exception as error:
                self.latest = {'state': 'Image detection failed', 'detail': str(error), 'detections': []}
                self.publish_scan()
                return
            self.latest = {'state': 'Scanning', 'frame': msg.header.frame_id,
                            'width': msg.width, 'height': msg.height, 'detections': detections,
                            'detection_diagnostics': detection_diagnostics}
            depth_points = None
            if poses and self.depth is not None and self.depth_camera is not None:
                depth_stamp = self.depth.header.stamp.sec + self.depth.header.stamp.nanosec / 1e9
                self.latest['depth_time_offset_s'] = stamp - depth_stamp
                if abs(stamp - depth_stamp) <= .12:
                    try:
                        # The Astra publishes rigid internal extrinsics on /tf with a
                        # slight timestamp lag. Use their latest common transform.
                        depth_tf = self.buffer.lookup_transform(msg.header.frame_id, self.depth.header.frame_id, rclpy.time.Time())
                        tf_stamp = depth_tf.header.stamp.sec + depth_tf.header.stamp.nanosec / 1e9
                        if tf_stamp and self.get_clock().now().nanoseconds / 1e9 - tf_stamp > .5:
                            raise ValueError('RGB/depth camera transform is stale')
                        q, p = depth_tf.transform.rotation, depth_tf.transform.translation
                        depth_points = project_depth(self.depth, self.depth_camera)
                        depth_points = depth_points[::max(1, math.ceil(len(depth_points) / 40000))]
                        depth_points = depth_points @ rotation([q.x, q.y, q.z, q.w]).T + [p.x, p.y, p.z]
                    except (TransformException, ValueError, AssertionError) as error:
                        self.latest['depth_error'] = str(error)
            supported_ids = []
            for detection in detections:
                marker_id = detection['id']
                detection['depth_status'] = 'Waiting for synchronized RGB/depth geometry'
                if marker_id not in poses or depth_points is None:
                    continue
                translation, orientation = poses[marker_id]
                detection['camera_position_m'] = list(map(float, translation))
                depth_diagnostics = {}
                support = depth_supported_translation(detection['corners'], translation, orientation,
                                                       depth_points, camera['matrix'], camera['distortion'], depth_diagnostics)
                detection['depth_diagnostics'] = depth_diagnostics
                if support is None:
                    detection['depth_status'] = depth_diagnostics.get('reason', 'Depth plane quality/coverage rejected')
                    continue
                translation, count, error = support
                normal = np.asarray(depth_diagnostics['plane_normal_in_camera'])
                right = orientation[:, 0] - normal * (normal @ orientation[:, 0])
                right /= np.linalg.norm(right)
                orientation = np.column_stack((right, np.cross(normal, right), normal))
                poses[marker_id] = (translation, orientation)
                detection.update(range_m=float(np.linalg.norm(translation)), depth_range_m=float(np.linalg.norm(translation)),
                    depth_samples=count, depth_plane_error_m=error, depth_status='RGB marker + measured depth plane',
                    camera_position_m=list(map(float, translation)))
                if 'size_warning' in depth_diagnostics:
                    detection['marker_size_warning'] = depth_diagnostics['size_warning']
                    detection['depth_inferred_size_m'] = markers[marker_id]['size'] * depth_diagnostics['depth_to_rgb_scale']
                supported_ids.append(marker_id)
            if poses:
                try:
                    base_tf = self.buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
                    base_stamp = base_tf.header.stamp.sec + base_tf.header.stamp.nanosec / 1e9
                    if base_stamp and self.get_clock().now().nanoseconds / 1e9 - base_stamp > .5:
                        raise TransformException('Camera-to-base transform is stale')
                    p, q = base_tf.transform.translation, base_tf.transform.rotation
                    candidates = []
                    for marker_id, (translation, orientation) in poses.items():
                        pose = surveyed_robot_pose(markers[marker_id], translation, orientation,
                            [p.x, p.y, p.z], rotation([q.x, q.y, q.z, q.w]))
                        if pose is not None:
                            candidates.append((marker_id, pose))
                    if candidates:
                        values = np.asarray([pose for _, pose in candidates])
                        heading = math.atan2(np.sin(values[:, 2]).mean(), np.cos(values[:, 2]).mean())
                        if (np.max(np.linalg.norm(values[:, :2] - np.median(values[:, :2], axis=0), axis=1)) <= .2
                                and np.max(np.abs(np.arctan2(np.sin(values[:, 2]-heading), np.cos(values[:, 2]-heading)))) <= .2):
                            self.latest['robot_pose'] = {'pose': [*map(float, np.median(values[:, :2], axis=0)), heading],
                                'marker_ids': [mid for mid, _ in candidates], 'source': 'surveyed_aruco',
                                'depth_supported_ids': supported_ids}
                except TransformException as error:
                    self.latest['localization_error'] = 'Camera-to-base calibration unavailable: ' + str(error)
            if not poses:
                self.publish_scan()
                self.pub.publish(MarkerArray())
                return
            try:
                tf = self.buffer.lookup_transform(args.frame, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            except TransformException as error:
                try:
                    tf = self.buffer.lookup_transform(args.frame, msg.header.frame_id, rclpy.time.Time())
                    tf_stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
                    if abs(stamp - tf_stamp) > .1:
                        raise TransformException('World transform is not within 100 ms of the observed marker')
                    self.latest['world_pose_time_offset_s'] = stamp - tf_stamp
                except TransformException:
                    self.latest['detail'] = str(error)
                    self.publish_scan()
                    self.pub.publish(MarkerArray())
                    return
            q, p = tf.transform.rotation, tf.transform.translation
            camera_rotation = rotation([q.x, q.y, q.z, q.w])
            output = MarkerArray()
            for detection in detections:
                marker_id = detection['id']
                if marker_id not in poses:
                    continue
                reference = markers[marker_id]
                translation, marker_rotation = poses[marker_id]
                center = camera_rotation @ translation + [p.x, p.y, p.z]
                orientation = camera_rotation @ marker_rotation
                direction = orientation[:, 2] if reference['mount'] == 'wall' else orientation[:, 1]
                if np.linalg.norm(direction[:2]) < .5:
                    detection['pose_status'] = 'Marker orientation rejected'
                    continue
                heading = math.atan2(direction[1], direction[0]) - (math.pi / 2 if reference['mount'] == 'floor' else 0.)
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = args.frame, msg.header.stamp
                marker.id, marker.ns, marker.type, marker.action = int(marker_id), 'observed_aruco', Marker.CUBE, Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
                marker.pose.orientation.z, marker.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
                marker.scale.x = marker.scale.y = reference['size']
                marker.scale.z = .01
                output.markers.append(marker)
                detection.update(pose_published=True, pose_status='Map-frame pose published')
            self.pub.publish(output)
            self.publish_scan()

    return Detector()


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, required=True)
    parser.add_argument('--frame', default='slam_map')
    parser.add_argument('--image', default='/camera/color/image_raw')
    parser.add_argument('--camera-info', default='/camera/color/camera_info')
    parser.add_argument('--max-rate', type=float, default=3.)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.max_rate) or not 0 < args.max_rate <= 10:
        parser.error('Use a detection rate in (0,10] Hz')
    environment = json.loads(args.environment.read_text())
    rclpy.init(args=ros_args)
    node = make_node(args, environment)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
