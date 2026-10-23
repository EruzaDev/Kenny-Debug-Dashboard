"""ROS velocity/encoder adapter for KENNY_USB_V1 (LF, RF, LR, RR)."""
import argparse
import json
import math
import threading
import time
from controller.safety import StopGate

def signed_delta(value, previous):
    return (int(value) - int(previous) + 2**31) % 2**32 - 2**31


class WheelOdometry:
    def __init__(self, radius=.04, track=.235, front_only=False):
        if not math.isfinite(radius) or not math.isfinite(track) or radius <= 0 or track <= 0:
            raise ValueError('Wheel radius and track must be positive and finite')
        self.radius, self.track = radius, track
        self.front_only = front_only
        self.previous = None
        self.pose = [0., 0., 0.]
        self.distance = 0.

    def update(self, status, yaw_delta=None, gyro_weight=.8):
        if yaw_delta is not None and (not math.isfinite(yaw_delta) or not 0 <= gyro_weight <= 1):
            raise ValueError('Invalid gyro odometry correction')
        if not status.get('encodersEnabled') or status.get('cpr', 0) <= 0:
            raise ValueError('Autonomous odometry requires real, enabled encoders')
        if len(status['counts']) != 4:
            raise ValueError('Expected four encoder counters')
        previous, self.previous = self.previous, status
        if previous is None:
            return None
        if status['boot'] != previous['boot']:
            raise RuntimeError('ESP32 rebooted; stop and restart localization before continuing')
        elapsed = (status['uptime'] - previous['uptime']) % 2**32 / 1000
        if not 0 < elapsed < 1:
            raise ValueError('Stale encoder telemetry')
        meters = [signed_delta(a, b) * 2 * math.pi * self.radius / status['cpr']
                  for a, b in zip(status['counts'], previous['counts'])]
        measured = meters[:2] if self.front_only else meters
        if any(abs(d) > 2 * elapsed for d in measured):
            raise RuntimeError('Encoder discontinuity; stopping')
        left, right = (meters[0], meters[1]) if self.front_only else ((meters[0] + meters[2]) / 2, (meters[1] + meters[3]) / 2)
        distance, angle = (left + right) / 2, (right - left) / self.track
        if yaw_delta is not None:
            angle = (1-gyro_weight)*angle + gyro_weight*yaw_delta
        arc = distance if abs(angle) < 1e-9 else distance * math.sin(angle / 2) / (angle / 2)
        self.pose[0] += arc * math.cos(self.pose[2] + angle / 2)
        self.pose[1] += arc * math.sin(self.pose[2] + angle / 2)
        self.pose[2] = math.atan2(math.sin(self.pose[2] + angle), math.cos(self.pose[2] + angle))
        self.distance += abs(distance)
        return list(self.pose), [distance / elapsed, angle / elapsed]


class SerialLink:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 115200
        self.serial.timeout = .03
        self.serial.write_timeout = .15
        self.serial.exclusive = True
        self.serial.dtr = self.serial.rts = False
        self.serial.open()
        time.sleep(2)
        self.serial.reset_input_buffer()
        if self.request('HELLO', timeout=3.).get('protocol') != 'KENNY_USB_V1':
            self.serial.close()
            raise RuntimeError('Not a KENNY_USB_V1 motor controller')
        self.request('STOP')

    def request(self, command, timeout=.2):
        self.serial.write((command + '\n').encode('ascii'))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                packet = json.loads(self.serial.readline())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(packet, dict):
                if packet.get('ok') is False:
                    raise CommandRejected(command, packet)
                return packet
        self.serial.write(b'STOP\n')
        self.serial.close()
        raise RuntimeError('Motor reply timeout; connection closed to discard late replies')

    def close(self):
        if self.serial.is_open:
            try:
                self.request('STOP')
            finally:
                self.serial.close()


class CommandRejected(RuntimeError):
    def __init__(self, command, packet):
        super().__init__(packet.get('error', 'Motor command rejected'))
        self.command_type = command.split()[0]
        self.packet = packet


FAULT_NAMES = {1: 'watchdog_expired', 2: 'hardware_estop', 4: 'bad_command',
               8: 'encoder_no_motion_or_wrong_direction', 16: 'motor_task_timing',
               32: 'configuration', 64: 'pwm_initialization', 128: 'motor_driver_i2c',
               256: 'wheel_overspeed'}


def rejection_details(link, error):
    # A rejected packet is fully consumed, so STATUS is safe to read. Preserve
    # fault/armed state before stopping; never replay motion or auto-arm here.
    details = {'command_type': error.command_type, 'error': str(error),
               'response': error.packet}
    try:
        telemetry = link.request('STATUS')
        details['firmware'] = telemetry
        details['fault_names'] = [name for bit, name in FAULT_NAMES.items()
                                  if telemetry.get('faults', 0) & bit]
        if not details['fault_names'] and not telemetry.get('armed'):
            details['reason'] = 'Firmware session was disarmed; lease is no longer valid'
    except Exception as status_error:
        details['status_error'] = str(status_error)
    finally:
        try:
            link.request('STOP')
        except Exception as stop_error:
            details['stop_error'] = str(stop_error)
    return details


def clear_stopped_fault(link, telemetry, stop_reason):
    if not stop_reason or telemetry.get('armed') or not telemetry.get('ready') or telemetry.get('estop'):
        raise ValueError('Fault clear requires STOP latched and a disarmed, ready motor controller')
    rpm = telemetry.get('rpm', [])
    if len(rpm) != 4 or not all(math.isfinite(v) and abs(v) < 1 for v in rpm):
        raise ValueError('All four wheels must be stationary before clearing a fault')
    if telemetry.get('faults', 0) & ~(1 | 4 | 8 | 16 | 256):
        raise ValueError('Hardware/configuration fault requires repair before reset')
    # Firmware ARM clears recoverable faults and initializes zero velocities.
    # No P/V is emitted; immediately disarm even if ARM raises/rejects.
    try:
        link.request('ARM')
    finally:
        link.request('STOP')
    return link.request('STATUS')


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, Float32, Float32MultiArray
    from tf2_ros import TransformBroadcaster
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--wheel-radius', type=float, default=.04)
    parser.add_argument('--track', type=float, default=.235)
    args, ros_args = parser.parse_known_args()
    odometry = WheelOdometry(args.wheel_radius, args.track)
    rclpy.init(args=ros_args)
    node = Node('kenny_motor_bridge')
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    status_pub = node.create_publisher(String, '/kenny/motors/status', 10)
    broadcaster = TransformBroadcaster(node)
    mutex = threading.Lock()
    command, allowed = [0., 0.], [False, -math.inf]
    received = [-math.inf]
    stopped = threading.Event()
    stop_gate = StopGate()
    bench_duty = [0., -math.inf]
    reset_generation = [0]
    clear_fault = threading.Event()
    manual_pwm = [0.,0.,-math.inf]

    def emergency_stop(msg):
        with mutex:
            stop_gate.update(msg.data)
            if msg.data:
                reset_generation[0] += 1
                allowed[:] = [False, -math.inf]
                received[0] = -math.inf
                bench_duty[:] = [0., -math.inf]
                manual_pwm[:]=[0.,0.,-math.inf]

    def bench_pwm(msg):
        with mutex:
            if msg.data == .75:
                bench_duty[:] = [.75, time.monotonic()]

    def velocity(msg):
        age = node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        values = [msg.twist.linear.x, msg.twist.angular.z]
        if not -.1 <= age <= .2 or not all(math.isfinite(v) for v in values) or abs(values[0]) > .1 or abs(values[1]) > .6:
            return
        with mutex:
            command[:] = values
            received[0] = time.monotonic()

    def permission(msg):
        with mutex:
            allowed[:] = [msg.data, time.monotonic()]

    def manual_drive(msg):
        if len(msg.data)!=2 or any(not math.isfinite(v) or abs(v)>1 or (v and abs(v)<.699) for v in msg.data):
            return
        with mutex:
            manual_pwm[:]=[float(msg.data[0]),float(msg.data[1]),time.monotonic()]

    node.create_subscription(TwistStamped, '/kenny/cmd_vel', velocity, 1)
    node.create_subscription(Bool, '/kenny/motion_allowed', permission, 1)
    node.create_subscription(Float32, '/kenny/bench_pwm', bench_pwm, 1)
    node.create_subscription(Float32MultiArray, '/kenny/manual_pwm', manual_drive, 1)
    node.create_subscription(Bool, '/kenny/motors/clear_fault',
                             lambda msg: clear_fault.set() if msg.data else None, 1)
    from rclpy.qos import QoSProfile, DurabilityPolicy
    node.create_subscription(Bool, '/kenny/emergency_stop', emergency_stop,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def run():
        link = None
        try:
            link = SerialLink(args.port)
            lease, sequence, last_status = None, 0, -math.inf
            permission_lost_at = None
            telemetry = {}
            rejection, rejected_generation = None, None
            last_drive = -math.inf
            while not stopped.is_set() and rclpy.ok():
                begin = time.monotonic()
                if clear_fault.is_set():
                    clear_fault.clear()
                    try:
                        telemetry = clear_stopped_fault(link, link.request('STATUS'), stop_gate.reason())
                        lease = None
                        rejected_generation = None
                        node.get_logger().info('Recoverable motor fault cleared while stationary; outputs remain disarmed')
                    except (ValueError, CommandRejected) as error:
                        node.get_logger().error('Motor fault clear refused: '+str(error))
                    last_status = -math.inf
                if begin - last_status >= .1:
                    telemetry = link.request('STATUS')
                    last_status = begin
                    published_at = time.monotonic()
                    status_pub.publish(String(data=json.dumps({**telemetry, 'bridge': {
                        'connected': True, 'port': args.port, 'stop_reason': stop_gate.reason(),
                        'command_age_s': max(0., published_at - received[0]) if math.isfinite(received[0]) else None,
                        'permission_age_s': max(0., published_at - allowed[1]) if math.isfinite(allowed[1]) else None,
                        'rejection': rejection,
                        'motion_inhibited': rejected_generation is not None,
                        'drive_age_s': published_at-last_drive if math.isfinite(last_drive) else None}})))
                    measured = odometry.update(telemetry)
                    if measured is not None:
                        pose, twist = measured
                        msg = Odometry()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
                        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
                        msg.twist.twist.linear.x, msg.twist.twist.angular.z = twist
                        # Configurable/calibratable wheel-slip uncertainty, not perfect pose.
                        msg.pose.covariance[0] = msg.pose.covariance[7] = .0004 + .01 * odometry.distance
                        msg.pose.covariance[35] = .001 + .01 * odometry.distance
                        msg.twist.covariance[0], msg.twist.covariance[35] = .0025, .01
                        odom_pub.publish(msg)
                        tf = TransformStamped()
                        tf.header, tf.child_frame_id = msg.header, 'base_link'
                        tf.transform.translation.x, tf.transform.translation.y = pose[:2]
                        tf.transform.rotation = msg.pose.pose.orientation
                        broadcaster.sendTransform(tf)
                with mutex:
                    current = time.monotonic()
                    if rejected_generation is not None and reset_generation[0] > rejected_generation:
                        rejected_generation = None
                    hard_stop = rejected_generation is not None or bool(stop_gate.reason())
                    permitted = not hard_stop and allowed[0] and current - allowed[1] < .25 and current - received[0] < .25
                    v, w = command if permitted else [0., 0.]
                    pulse = bench_duty[0] if current - bench_duty[1] < .25 else 0.
                hard_stop = hard_stop or not telemetry.get('ready', False) or bool(telemetry.get('faults') or telemetry.get('estop'))
                if hard_stop:
                    permission_lost_at = None
                    if lease is not None or telemetry.get('armed'):
                        link.request('STOP')
                        lease = None
                else:
                    try:
                        if not permitted and permission_lost_at is None:
                            permission_lost_at = current
                        if permitted:
                            permission_lost_at = None
                        # A brief guard pause sends zero velocity under the
                        # existing lease; sustained loss of host permission
                        # still disarms rather than holding a dormant lease.
                        if not permitted and (lease is None or current - permission_lost_at >= 1.):
                            if lease is not None or telemetry.get('armed'):
                                link.request('STOP')
                                lease = None
                            stopped.wait(max(0., .05 - (time.monotonic() - begin)))
                            continue
                        if lease is not None and not telemetry.get('armed'):
                            raise RuntimeError('Firmware disarmed while host lease was active')
                        # Stop an expired host lease explicitly instead of sending
                        # it to a firmware watchdog that may already have disarmed.
                        if lease is not None and current-last_drive >= .28:
                            link.request('STOP')
                            lease = None
                            with mutex:
                                rejected_generation = reset_generation[0]
                            rejection = {'reason': 'Host command delivery gap exceeded 280 ms; explicit stop/reset required',
                                         'drive_gap_s': current-last_drive}
                            node.get_logger().error(rejection['reason'])
                            continue
                        if lease is None:
                            lease = link.request('ARM')['lease']
                            sequence = 0
                        # ARM/serial work may take time: recheck input freshness
                        # before emitting motion, and take the newest command.
                        with mutex:
                            send_at = time.monotonic()
                            safe = not stop_gate.reason() and rejected_generation is None
                            fresh = safe and allowed[0] and send_at-allowed[1] < .25 and send_at-received[0] < .25
                            v, w = command if fresh else (0., 0.)
                            pulse = bench_duty[0] if fresh and send_at-bench_duty[1] < .25 else 0.
                            direct_pwm=manual_pwm[:2] if fresh and send_at-manual_pwm[2]<.25 else None
                        if not safe:
                            link.request('STOP')
                            lease = None
                            continue
                        sequence = (sequence + 1) % 2**32
                        last_drive = time.monotonic()
                        if pulse:
                            link.request(f'P {lease} {sequence} {pulse:.2f}')
                        elif direct_pwm is not None:
                            if not telemetry.get('manualPwmSupported'):
                                raise RuntimeError('Firmware does not support direct manual PWM')
                            link.request(f'D {lease} {sequence} {direct_pwm[0]:.4f} {direct_pwm[1]:.4f}')
                        else:
                            link.request(f'V {lease} {sequence} {v:.5f} {w:.5f}')
                    except CommandRejected as error:
                        rejection = rejection_details(link, error)
                        lease = None
                        with mutex:
                            rejected_generation = reset_generation[0]
                        node.get_logger().error('Motor command rejected; motion latched off: ' + json.dumps(rejection))
                        if 'status_error' in rejection or 'stop_error' in rejection:
                            raise RuntimeError('Cannot verify firmware after rejection: ' + json.dumps(rejection))
                stopped.wait(max(0., .05 - (time.monotonic() - begin)))
        except Exception as error:
            node.get_logger().error(f'Motor bridge stopped: {error}')
            if rclpy.ok():
                status_pub.publish(String(data=json.dumps({'bridge': {'connected': False, 'error': str(error)}})))
            stopped.set()
        finally:
            if link is not None:
                link.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    from rclpy.executors import ExternalShutdownException
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopped.set()
        thread.join(timeout=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
