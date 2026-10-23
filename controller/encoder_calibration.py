"""Manually measure all four wheel encoders on the two-board robot.

Observe only: does not arm motors, drive wheels or change calibration settings.
Run with kenny-motors.service active and the robot disarmed:
    python -m controller.encoder_calibration --output /tmp/encoder-calibration.json
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time


def delta(new, old):
    return (int(new) - int(old) + 2**31) % 2**32 - 2**31


def measure(start, end):
    changes = [delta(a, b) for a, b in zip(end['counts'], start['counts'])]
    invalid = ([delta(a, b) for a, b in zip(end['invalid'], start['invalid'])]
               if start.get('invalid') is not None and end.get('invalid') is not None else None)
    result = {'counts_delta_lf_rf_lr_rr': changes, 'invalid_transition_delta': invalid}
    if start.get('odom') and end.get('odom'):
        a, b = start['odom'], end['odom']
        result['odom_displacement_m'] = round(math.dist(a[:2], b[:2]), 4)
        result['odom_heading_change_deg'] = round(math.degrees(math.atan2(
            math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))), 2)
    return result


def calculate(stages, revolutions, distance, turn_degrees, configured_cpr=660,
              configured_radius=.04, configured_track=.235):
    wheels = ('lf', 'rf', 'lr', 'rr')
    straight = stages['straight']['counts_delta_lf_rf_lr_rr']
    turn = stages['left_turn']['counts_delta_lf_rf_lr_rr']
    single = [stages[f'{wheel}_wheel']['counts_delta_lf_rf_lr_rr'] for wheel in wheels]
    cpr = [sample[i] / revolutions for i, sample in enumerate(single)]
    result = {'cpr_by_wheel_lf_rf_lr_rr': [round(value, 2) for value in cpr],
              'configured_cpr': configured_cpr, 'odometry_wheels': ['lf', 'rf'],
              'wheel_checks': {wheel: {'forward_positive': sample[i] > 0,
                                       'other_wheel_deltas': [value for j, value in enumerate(sample) if j != i]}
                               for i, (wheel, sample) in enumerate(zip(wheels, single))}}
    warnings = []
    for i, (wheel, sample) in enumerate(zip(wheels, single)):
        if any(abs(value) > max(5, abs(sample[i]) * .02) for j, value in enumerate(sample) if j != i):
            warnings.append(f'{wheel}: another wheel encoder changed during isolated rotation')
    for name, stage in stages.items():
        invalid = stage.get('invalid_transition_delta')
        if invalid and any(value > 0 for value in invalid):
            warnings.append(f'{name}: invalid encoder transitions increased; inspect Hall signals')
    if any(value <= 0 for value in straight):
        warnings.append('Straight forward roll did not increment all four encoders')
    if turn[0] >= 0 or turn[1] <= 0 or turn[2] >= 0 or turn[3] <= 0:
        warnings.append('Left turn should decrement left and increment right encoder counts')
    result['warnings'] = warnings
    if any(value <= 0 for value in cpr):
        result['calibration_error'] = 'Forward counts must be positive on all four wheels; check encoder wiring/sign'
        return result
    radii = [distance * cpr[i] / (2 * math.pi * value) if value > 0 else None
             for i, value in enumerate(straight)]
    result['radius_from_straight_m'] = {wheel: round(radius, 5) if radius else None
                                        for wheel, radius in zip(wheels, radii)}
    result['configured_radius_m'] = configured_radius
    if radii[0] and radii[1]:
        # Current /odom uses only LF/RF. Rear-wheel turn estimates are diagnostic
        # since skid/slip and a noisy LR channel can distort their reading.
        turn_distance = [2 * math.pi * radius * counts / counts_per_rev
                         if radius else None for radius, counts, counts_per_rev in zip(radii, turn, cpr)]
        result['track_from_turn_m'] = round((turn_distance[1] - turn_distance[0]) /
                                             math.radians(turn_degrees), 5)
        result['rear_track_from_turn_m'] = (round((turn_distance[3] - turn_distance[2]) /
                                                 math.radians(turn_degrees), 5)
                                            if turn_distance[2] is not None and turn_distance[3] is not None else None)
        result['configured_track_m'] = configured_track
    else:
        result['calibration_error'] = 'Straight roll must advance both front encoders for odometry calibration'
    result['odom_straight_m'] = stages['straight'].get('odom_displacement_m')
    result['odom_turn_deg'] = stages['left_turn'].get('odom_heading_change_deg')
    return result


class Listener:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.odom = None
        self.last_message = None
        self.rejected_reason = None

    def on_status(self, text):
        try:
            packet = json.loads(text)
            with self.lock:
                self.last_message = time.monotonic()
                if (packet.get('bridge', {}).get('mode') != 'dual_side_robot' or
                        not packet.get('encodersEnabled') or len(packet.get('counts', [])) != 4):
                    self.rejected_reason = (f"Expected dual_side_robot with four encoders; got bridge="
                                            f"{packet.get('bridge')} encodersEnabled={packet.get('encodersEnabled')}")
                else:
                    self.rejected_reason = None
                    self.status = (self.last_message, packet)
        except (ValueError, TypeError, AttributeError):
            return

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        with self.lock:
            self.odom = (time.monotonic(), [msg.pose.pose.position.x,
                                          msg.pose.pose.position.y, yaw])

    def capture(self, previous_uptime=None):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            with self.lock:
                status, odom = self.status, self.odom
            now = time.monotonic()
            if status and now - status[0] < .4 and status[1].get('uptime') != previous_uptime:
                packet = status[1]
                if packet.get('armed') or any(packet.get('pwm', [])):
                    raise RuntimeError('Motor controller is armed or PWM is nonzero; stop it before manual calibration')
                return {'counts': list(packet['counts']),
                        'invalid': packet.get('encoderInvalidTransitions'),
                        'uptime': packet.get('uptime'),
                        'odom': list(odom[1]) if odom and now - odom[0] < .4 else None}
            time.sleep(.03)
        with self.lock:
            reason = self.rejected_reason if self.last_message else None
        raise RuntimeError(reason or 'No fresh two-board encoder status on /kenny/motors/status; '
                           'check systemctl --user status kenny-motors.service and ros2 topic info /kenny/motors/status')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revolutions', type=float, default=2., help='Forward turns per raised front wheel')
    parser.add_argument('--distance', type=float, default=1., help='Measured straight floor roll in meters')
    parser.add_argument('--turn-degrees', type=float, default=90., help='Measured left floor turn, below 180 degrees')
    parser.add_argument('--output', type=Path, default=Path('/tmp/encoder-calibration.json'))
    args, ros_args = parser.parse_known_args()
    if not all(math.isfinite(x) and x > 0 for x in (args.revolutions, args.distance, args.turn_degrees)) or args.turn_degrees >= 180:
        parser.error('Use positive finite revolutions/distance and turn-degrees below 180')

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    rclpy.init(args=ros_args)
    node = Node('kenny_encoder_calibration')
    listener = Listener()
    node.create_subscription(String, '/kenny/motors/status', lambda msg: listener.on_status(msg.data), 10)
    node.create_subscription(Odometry, '/odom', listener.on_odom, 10)
    running = threading.Event()
    running.set()
    thread = threading.Thread(target=lambda: spin(node, running), daemon=True)
    thread.start()
    stages = {}
    instructions = [
        ('lf_wheel', f'Raise wheels; rotate ONLY left-front forward {args.revolutions:g} turns.'),
        ('rf_wheel', f'Keep wheels raised; rotate ONLY right-front forward {args.revolutions:g} turns.'),
        ('lr_wheel', f'Keep wheels raised; rotate ONLY left-rear forward {args.revolutions:g} turns.'),
        ('rr_wheel', f'Keep wheels raised; rotate ONLY right-rear forward {args.revolutions:g} turns.'),
        ('straight', f'Place on floor; manually roll straight forward {args.distance:g} m (measure chassis travel).'),
        ('left_turn', f'Manually turn chassis LEFT {args.turn_degrees:g} degrees in place on the floor.'),
    ]
    try:
        print('Keep the two-board motor bridge running with STOP latched / motors disarmed. No motor commands are sent.')
        for name, instruction in instructions:
            input(f'\n{name}: position robot at start, then press Enter to capture baseline...')
            start = listener.capture()
            print(f'Baseline LF/RF/LR/RR: {start["counts"]}. {instruction}')
            input('When motion is complete and wheels have stopped, press Enter...')
            end = listener.capture(start['uptime'])
            stages[name] = measure(start, end)
            print(json.dumps(stages[name]))
        results = calculate(stages, args.revolutions, args.distance, args.turn_degrees)
        report = {'inputs': {'revolutions': args.revolutions, 'distance_m': args.distance,
                             'left_turn_degrees': args.turn_degrees}, 'stages': stages,
                  'calibration': results}
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(f'\nSaved {args.output}\n{json.dumps(results, indent=2)}')
    finally:
        running.clear()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


def spin(node, running):
    import rclpy
    while running.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)


if __name__ == '__main__':
    main()
