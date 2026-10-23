"""Passive ROS check for encoder odometry gaps during robot motion.

Run: python -m controller.odom_diagnostic --seconds 30 --output /tmp/kenny-odom.json
Start the normal robot drive while it records; this tool never publishes commands.
"""
import argparse
import json
import math
from pathlib import Path
import time


class OdomDiagnostic:
    def __init__(self, stale_after=.5):
        self.stale_after = stale_after
        self.last_odom = None
        self.odom_samples = 0
        self.gaps = []
        self.events = []
        self.armed = False
        self.armed_since = None
        self.armed_intervals = []
        self.armed_odom_samples = 0
        self.armed_stale = False
        self.armed_stale_intervals = []
        self.bridge = {}
        self.motor_samples = 0
        self.armed_count_changes = 0
        self.previous_counts = None
        self.last_counts = None
        self.last_invalid_transitions = None

    def _refresh(self, now):
        if self.armed and self.last_odom is not None and now - self.last_odom >= self.stale_after:
            if not self.armed_stale:
                self.armed_stale = True
                self.armed_stale_intervals.append({'start_s': round(now, 3), 'end_s': None})
        elif self.armed_stale:
            self.armed_stale_intervals[-1]['end_s'] = round(now, 3)
            self.armed_stale = False

    def odom(self, now):
        self._refresh(now)
        if self.last_odom is not None:
            gap = now - self.last_odom
            if gap >= self.stale_after:
                self.gaps.append({'at_s': round(now, 3), 'gap_s': round(gap, 3),
                                  'armed': self.armed})
        self.last_odom = now
        self.odom_samples += 1
        if self.armed:
            self.armed_odom_samples += 1
        self._refresh(now)

    def motor(self, packet, now):
        self._refresh(now)
        bridge = packet.get('bridge', {})
        self.bridge = {key: bridge.get(key) for key in ('connected', 'mode', 'drive_mode', 'error')}
        new_armed = packet.get('armed') is True
        self.motor_samples += 1
        counts = packet.get('counts')
        if isinstance(counts, list) and len(counts) == 4:
            if new_armed and self.previous_counts is not None and counts != self.previous_counts:
                self.armed_count_changes += 1
            self.previous_counts = list(counts)
            self.last_counts = list(counts)
        self.last_invalid_transitions = packet.get('encoderInvalidTransitions', self.last_invalid_transitions)
        if new_armed != self.armed:
            if new_armed:
                self.armed_since = now
            else:
                self.armed_intervals.append({'start_s': round(self.armed_since, 3),
                                             'end_s': round(now, 3)})
                self.armed_since = None
            self.armed = new_armed
            self._refresh(now)
            self.events.append({'at_s': round(now, 3), 'type': 'armed' if new_armed else 'disarmed',
                                'odom_age_s': self.age(now)})

    def age(self, now):
        return round(now - self.last_odom, 3) if self.last_odom is not None else None

    def controller(self, packet, now):
        self._refresh(now)
        self.events.append({'at_s': round(now, 3), 'type': 'controller',
                            'armed': self.armed, 'odom_age_s': self.age(now),
                            'reported_odom_age_s': packet.get('ages_s', {}).get('odom'),
                            'guard': packet.get('guard'), 'reason': packet.get('reason'),
                            'motion_allowed': packet.get('motion_allowed'),
                            'drive_mode': packet.get('drive_mode'),
                            'pwm_stage': packet.get('pwm_stage')})

    def result(self, now):
        self._refresh(now)
        intervals = list(self.armed_intervals)
        if self.armed:
            intervals.append({'start_s': round(self.armed_since, 3), 'end_s': round(now, 3)})
        stale = [dict(item) for item in self.armed_stale_intervals]
        if stale and stale[-1]['end_s'] is None:
            stale[-1]['end_s'] = round(now, 3)
        return {'odom_samples': self.odom_samples, 'armed_odom_samples': self.armed_odom_samples,
                'motor_samples': self.motor_samples, 'armed_count_changes': self.armed_count_changes,
                'last_counts': self.last_counts, 'last_encoder_invalid_transitions': self.last_invalid_transitions,
                'armed_intervals': intervals, 'armed_stale_intervals': stale,
                'odom_gaps_at_least_0_5s': self.gaps, 'bridge': self.bridge,
                'events': self.events,
                'interpretation': ('No armed interval observed; repeat while the robot is driving' if not intervals else
                                   'Odom became stale while armed; inspect controller guard/reason at matching times' if stale else
                                   'No stale odometry observed while armed')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', type=Path, default=Path('/tmp/kenny-odom.json'))
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rclpy.init(args=ros_args)
    node = Node('kenny_odom_diagnostic')
    diagnostic = OdomDiagnostic()
    start = time.monotonic()

    def motor(msg):
        try:
            diagnostic.motor(json.loads(msg.data), time.monotonic() - start)
        except (ValueError, TypeError, AttributeError):
            pass

    def controller(msg):
        try:
            diagnostic.controller(json.loads(msg.data), time.monotonic() - start)
        except (ValueError, TypeError, AttributeError):
            pass

    node.create_subscription(Odometry, '/odom', lambda msg: diagnostic.odom(time.monotonic() - start), 10)
    node.create_subscription(String, '/kenny/motors/status', motor, 10)
    node.create_subscription(String, '/kenny/controller/status', controller, 10)
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = diagnostic.result(time.monotonic() - start)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(f"Saved {args.output}: {report['interpretation']}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
