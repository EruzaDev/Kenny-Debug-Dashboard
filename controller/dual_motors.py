"""ROS motor bridge for the two KENNY_SIDE_V2 boards (LF, RF, LR, RR)."""
import json
import argparse
import math
from pathlib import Path
import secrets
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'joystick/scripts'))
from dual_side_bridge import Board, PORTS

from controller.motors import WheelOdometry, signed_delta
from controller.safety import StopGate
from controller.gyro_odometry import GyroYaw

CPR = 660


def wheel_counts(left, right):
    # Left Motor 1 is 18/21, right Motor 1 is 34/25.
    return [left[0], right[1], left[1], right[0]]


def forward_counts(left, right):
    """Left Hall A/B wiring counts forward rotation negative on both left wheels."""
    lf, rf, lr, rr = wheel_counts(left, right)
    return [-lf, rf, -lr, rr]


def duties(command, manual=None, bench=0.):
    if bench:
        return [191, 191]
    if manual is not None:
        if len(manual) != 2 or any(not math.isfinite(v) or abs(v) > 1 for v in manual):
            raise ValueError('Invalid direct PWM command')
        return [max(-191, min(191, round(v * 255))) for v in manual]
    v, w = command
    if not all(math.isfinite(x) for x in (v, w)) or abs(v) > .1 or abs(w) > .6:
        raise ValueError('Invalid velocity command')
    left, right = v / .1 - w / .6, v / .1 + w / .6
    scale = max(1., abs(left), abs(right))
    return [round(left * 191 / scale), round(right * 191 / scale)]


def wheel_targets(command, radius=.04, track=.235, cpr=CPR):
    """Robot forward/turn velocity -> signed wheel encoder transitions/s."""
    v, w = command
    if not all(math.isfinite(x) for x in (v, w)) or abs(v) > .1 or abs(w) > .6:
        raise ValueError('Invalid velocity command')
    scale = cpr / (2 * math.pi * radius)
    return [round((v - w * track / 2) * scale), round((v + w * track / 2) * scale)]


class DualLink:
    def __init__(self):
        self.boards = []
        try:
            for side in ('left', 'right'):
                self.boards.append(Board(PORTS[side]))
                self.boards[-1].serial.timeout = .04
                self.boards[-1].no_status_retry = True
                if self.boards[-1].protocol.startswith('KENNY_SIDE_V3'):
                    self.boards[-1].send(f'SIDE {side[0].upper()}', f'OK SIDE {side[0].upper()}')
            self.velocity_supported = all(board.protocol.startswith('KENNY_SIDE_V3') for board in self.boards)
            self.boot = secrets.randbits(31)
            self.last_ms = None
            self.last_counts = None
            self.lease = False
            self.sequence = 0
            self.pwm = [0, 0]
            self.command_mode = None
            self.cached_status = None
            self.sample_fresh = False
            self.next_status = -math.inf
            self.last_sample = -math.inf
        except Exception:
            self.close()
            raise

    def status(self):
        now = time.monotonic()
        if self.cached_status is not None and now < self.next_status:
            self.sample_fresh = False
            return dict(self.cached_status)
        def poll(item):
            side, board = item
            for attempt in range(2 if not self.lease else 1):
                try:
                    return board.status()
                except (OSError, ValueError, RuntimeError) as error:
                    if attempt or self.lease:
                        if not self.lease and self.cached_status is not None and now - self.last_sample < 2.:
                            raise RuntimeError('Encoder telemetry unavailable') from error
                        raise RuntimeError(f'{side} ESP32 status unavailable: {error}') from error
                    board.serial.reset_input_buffer()
        left, right = self.parallel(poll, list(zip(('left', 'right'), self.boards)))
        stamps = (left['ms'], right['ms'])
        if self.last_ms is not None and any(not 0 < (new - old) % 2**32 < 5000
                                            for new, old in zip(stamps, self.last_ms)):
            raise RuntimeError('Side ESP32 rebooted or encoder telemetry stale')
        self.last_ms = stamps
        raw_counts = wheel_counts(left['counts'], right['counts'])
        counts = forward_counts(left['counts'], right['counts'])
        invalid = wheel_counts(left['invalid'], right['invalid'])
        rates = forward_counts(left['rates'], right['rates'])
        self.last_counts = counts
        rpm = [v * 60 / CPR for v in rates]
        result = {'ready': True, 'armed': left['armed'] or right['armed'], 'estop': False,
                'faults': 0, 'encodersEnabled': True, 'manualPwmSupported': True,
                'velocitySupported': self.velocity_supported,
                'cpr': CPR, 'watchdog': 200, 'boot': self.boot, 'uptime': int(time.monotonic() * 1000) % 2**32,
                'counts': counts, 'rawCounts': raw_counts, 'rpm': rpm, 'pwm': [abs(self.pwm[0]), abs(self.pwm[1]),
                abs(self.pwm[0]), abs(self.pwm[1])],
                'encoderInvalidTransitions': invalid,
                'bridge': {'connected': True, 'mode': 'dual_side_robot',
                           'drive_mode': self.command_mode,
                           'left_port': PORTS['left'], 'right_port': PORTS['right']}}
        self.cached_status = result
        self.sample_fresh = True
        self.last_sample = time.monotonic()
        # Keep measured odom/TF alive during motion as well as while stopped.
        self.next_status = self.last_sample + .08
        return dict(result)

    def parallel(self, function, items):
        with ThreadPoolExecutor(max_workers=2) as executor:
            return list(executor.map(function, items))

    def send_pair(self, commands):
        return self.parallel(lambda item: item[0].send(*item[1]), list(zip(self.boards, commands)))

    def drive(self, pair):
        if not any(pair):
            self.stop()
            return
        if self.lease and self.command_mode != 'pwm':
            self.stop()
        if not self.lease:
            try:
                self.send_pair([('ARM', 'OK ARM')] * 2)
            except Exception:
                self.stop()
                raise
            self.lease = True
            self.sequence = 0
        self.sequence = (self.sequence + 1) % 2**32
        try:
            self.send_pair([(f'P {self.sequence} {duty} {duty}', f'OK P {self.sequence}') for duty in pair])
        except Exception:
            self.stop()
            raise
        self.pwm = list(pair)
        self.command_mode = 'pwm'

    def drive_wheel(self, wheel, duty=191):
        """Bounded single-wheel PWM; board channels are LF/LR and RR/RF."""
        if type(wheel) is not int or wheel not in range(4) or type(duty) is not int or not 0 < duty <= 191:
            raise ValueError('Invalid wheel or PWM duty')
        if self.lease and self.command_mode != 'wheel':
            self.stop()
        if not self.lease:
            try:
                self.send_pair([('ARM', 'OK ARM')] * 2)
            except Exception:
                self.stop()
                raise
            self.lease = True
            self.sequence = 0
        self.sequence = (self.sequence + 1) % 2**32
        outputs = ([duty if wheel == 0 else 0, duty if wheel == 2 else 0],
                   [duty if wheel == 3 else 0, duty if wheel == 1 else 0])
        try:
            self.send_pair([(f'P {self.sequence} {pair[0]} {pair[1]}', f'OK P {self.sequence}') for pair in outputs])
        except Exception:
            self.stop()
            raise
        self.pwm = [duty if wheel in (0, 2) else 0, duty if wheel in (1, 3) else 0]
        self.command_mode = 'wheel'

    def velocity(self, targets):
        if not self.velocity_supported:
            raise RuntimeError('Both ESP32s need KENNY_SIDE_V3 firmware for velocity control')
        if not any(targets):
            self.stop()
            return
        if any(type(value) is not int or abs(value) > 1600 for value in targets):
            raise ValueError('Wheel speed target out of bounds')
        if self.lease and self.command_mode != 'velocity':
            self.stop()
        if not self.lease:
            try:
                self.send_pair([('ARM', 'OK ARM')] * 2)
            except Exception:
                self.stop()
                raise
            self.lease = True
            self.sequence = 0
        self.sequence = (self.sequence + 1) % 2**32
        try:
            self.send_pair([(f'V {self.sequence} {target}', f'OK V {self.sequence}') for target in targets])
        except Exception:
            self.stop()
            raise
        self.command_mode = 'velocity'
        # PWM is generated on the MCU; actual duty is available from STATUS
        # after disarm, not measured while the serial link is carrying V.
        self.pwm = [0, 0]

    def stop(self):
        self.lease = False
        self.pwm = [0, 0]
        self.command_mode = None
        self.parallel(lambda board: board.stop(), self.boards)

    def close(self):
        for board in self.boards:
            board.close()
        self.boards = []


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool, Float32, Float32MultiArray, Int8, String
    from geometry_msgs.msg import TwistStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_ros import TransformBroadcaster

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--continuous', action='store_true', help='Use V targets on both V3 ESP32s')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('kenny_dual_side_motor_bridge')
    gate = StopGate()
    lock = threading.Lock()
    command = [0., 0.]
    manual = [None, -math.inf]
    bench = [0., -math.inf]
    bench_wheel = [None, -math.inf]
    received = [-math.inf]
    allowed = [False, -math.inf]
    stopped = threading.Event()
    failed = threading.Event()
    status_pub = node.create_publisher(String, '/kenny/motors/status', 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    broadcaster = TransformBroadcaster(node)
    # LR Hall channel has frequent invalid transitions while stopped. Use the
    # two front wheels for pose; retain all four raw counters in diagnostics.
    odom = WheelOdometry(front_only=True)
    gyro = GyroYaw()
    gyro_lock = threading.Lock()
    stationary = [False, -math.inf]
    odom_sample_time = [None]

    def imu_sample(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        now = time.monotonic()
        if not 0 <= age <= .1:
            return
        g, a = msg.angular_velocity, msg.linear_acceleration
        with gyro_lock:
            if msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
                gyro.reset('IMU gyro/acceleration unavailable')
                return
            gyro.observe(now-age, [g.x, g.y, g.z], [a.x, a.y, a.z],
                         stationary[0] and now-stationary[1] <= .2, msg.header.frame_id)

    node.create_subscription(Imu, '/imu/data_raw', imu_sample, qos_profile_sensor_data)

    def stop(msg):
        with lock:
            gate.update(msg.data)
            if msg.data:
                received[0] = -math.inf
                allowed[:] = [False, -math.inf]
                manual[:] = [None, -math.inf]
                bench[:] = [0., -math.inf]

    def velocity(msg):
        age = node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        values = [msg.twist.linear.x, msg.twist.angular.z]
        if not -.1 <= age <= .35 or any(not math.isfinite(v) for v in values) or abs(values[0]) > .1 or abs(values[1]) > .6:
            return
        with lock:
            command[:] = values
            received[0] = time.monotonic()

    def permission(msg):
        with lock:
            allowed[:] = [msg.data, time.monotonic()]

    def direct_pwm(msg):
        if len(msg.data) != 2 or any(not math.isfinite(v) or abs(v) > 1 for v in msg.data):
            return
        with lock:
            manual[:] = [list(msg.data), time.monotonic()]

    def bench_pwm(msg):
        if msg.data == .75:
            with lock:
                bench[:] = [.75, time.monotonic()]

    def wheel_selection(msg):
        if -1 <= msg.data < 4:
            with lock:
                bench_wheel[:] = [msg.data, time.monotonic()]

    node.create_subscription(Bool, '/kenny/emergency_stop', stop,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TwistStamped, '/kenny/cmd_vel', velocity, 1)
    node.create_subscription(Bool, '/kenny/motion_allowed', permission, 1)
    node.create_subscription(Float32MultiArray, '/kenny/manual_pwm', direct_pwm, 1)
    node.create_subscription(Float32, '/kenny/bench_pwm', bench_pwm, 1)
    node.create_subscription(Int8, '/kenny/calibration/wheel', wheel_selection, 1)

    def publish_odom(telemetry):
        now = time.monotonic()
        previous_time, odom_sample_time[0] = odom_sample_time[0], now
        with gyro_lock:
            yaw_delta = gyro.delta(previous_time, now) if previous_time is not None else None
            telemetry['imu_fusion'] = {'calibrated': gyro.bias is not None,
                'used': yaw_delta is not None, 'reason': gyro.reason or ('' if yaw_delta is not None else 'IMU interval unavailable'),
                'yaw_delta_rad': yaw_delta}
        try:
            measured = odom.update(telemetry, yaw_delta=yaw_delta)
        except ValueError as error:
            if str(error) != 'Stale encoder telemetry':
                raise
            # A gap is not motion evidence. Start a new measured baseline.
            odom.previous = None
            odom_sample_time[0] = None
            return
        if measured is None:
            return
        pose, twist = measured
        with gyro_lock:
            # Use raw front-wheel rates, never the fused result, for calibration.
            stationary[:] = [not telemetry['armed'] and all(abs(telemetry['rpm'][i]) < .5 for i in (0, 1)), now]
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = twist
        msg.pose.covariance[0] = msg.pose.covariance[7] = .0004 + .01 * odom.distance
        msg.pose.covariance[35] = .001 + .01 * odom.distance
        msg.twist.covariance[0], msg.twist.covariance[35] = .0025, .01
        odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = msg.header, 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = pose[:2]
        tf.transform.rotation = msg.pose.pose.orientation
        broadcaster.sendTransform(tf)

    def run():
        link = None
        try:
            link = DualLink()
            if args.continuous and not link.velocity_supported:
                raise RuntimeError('Continuous drive needs KENNY_SIDE_V3 firmware on both ESP32s')
            last_drive = -math.inf
            while not stopped.is_set() and rclpy.ok():
                start = time.monotonic()
                if link.lease and start - last_drive >= .18:
                    raise RuntimeError('Motor command heartbeat expired; STOP required')
                with lock:
                    now = time.monotonic()
                    permitted = (not gate.reason() and allowed[0] and now-allowed[1] < .25
                                 and now-received[0] < .25)
                    direct = manual[0] if now-manual[1] < .25 else None
                    selected_wheel = bench_wheel[0] if now-bench_wheel[1] < .25 else None
                    pulse = (.75 if selected_wheel is not None and selected_wheel >= 0 else
                             bench[0] if selected_wheel == -1 and now-bench[1] < .25 else 0.)
                    use_velocity = args.continuous and direct is None and not pulse
                    pair = (wheel_targets(command) if use_velocity else
                            [0, 0] if pulse and selected_wheel is None else
                            duties(command, direct, pulse)) if permitted else [0, 0]
                if permitted and any(pair):
                    last_drive = time.monotonic()
                    if pulse and selected_wheel is not None and selected_wheel >= 0:
                        link.drive_wheel(selected_wheel)
                    elif use_velocity:
                        link.velocity(pair)
                    else:
                        link.drive(pair)
                else:
                    if link.lease or (link.cached_status or {}).get('armed'):
                        link.stop()
                telemetry = link.status()
                with lock:
                    telemetry['bridge']['stop_reason'] = gate.reason()
                    telemetry['bridge']['command_age_s'] = now-received[0] if math.isfinite(received[0]) else None
                    telemetry['bridge']['permission_age_s'] = now-allowed[1] if math.isfinite(allowed[1]) else None
                # Reflect the most recent output, not the previous STATUS cycle.
                telemetry['armed'] = link.lease
                telemetry['pwm'] = [abs(link.pwm[0]), abs(link.pwm[1]), abs(link.pwm[0]), abs(link.pwm[1])]
                if link.sample_fresh:
                    publish_odom(telemetry)
                status_pub.publish(String(data=json.dumps(telemetry)))
                stopped.wait(max(0., .1 - (time.monotonic() - start)))
        except Exception as error:
            node.get_logger().error('Two-ESP32 motor bridge stopped: ' + str(error))
            failed.set()
            if rclpy.ok():
                status_pub.publish(String(data=json.dumps({'bridge': {'connected': False, 'error': str(error)}})))
            stopped.set()
        finally:
            if link is not None:
                link.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        thread.join(timeout=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if failed.is_set():
            raise SystemExit(1)


if __name__ == '__main__':
    main()
