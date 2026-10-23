"""Feedback-checked direct-PWM path navigator with confident Forward, Sway, and Turn."""
import math
import numpy as np


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def path_lookahead(pose, route, lookahead=0.40, min_idx=0):
    """Interpolate a target point along the route polyline by physical arc length.

    Avoids cutting corners through walls on sharp bends, ensuring the robot
    faithfully follows the planned path corridor.
    """
    if not route:
        return None, 0
    if len(route) == 1:
        return list(route[0][:2]), 0

    pts = np.asarray(route, dtype=float)[:, :2]
    pos = np.asarray(pose[:2], dtype=float)

    start_idx = max(0, min(min_idx, len(pts) - 2))
    sub_pts = pts[start_idx:]
    if len(sub_pts) < 2:
        return pts[-1].tolist(), len(pts) - 1

    segs = np.diff(sub_pts, axis=0)
    lengths = np.linalg.norm(segs, axis=1)
    valid = lengths > 1e-6

    deltas = pos - sub_pts[:-1]
    t = np.zeros(len(segs))
    t[valid] = np.clip(np.sum(deltas[valid] * segs[valid], axis=1) / (lengths[valid] ** 2), 0.0, 1.0)
    projs = sub_pts[:-1] + t[:, None] * segs
    dists = np.linalg.norm(pos - projs, axis=1)

    local_idx = int(np.argmin(dists))
    best_idx = start_idx + local_idx

    rem_dist = (1.0 - t[local_idx]) * lengths[local_idx]
    if lookahead <= rem_dist:
        pt = projs[local_idx] + (lookahead / max(lengths[local_idx], 1e-6)) * segs[local_idx]
        return pt.tolist(), best_idx

    accum = rem_dist
    curr = local_idx + 1
    while curr < len(segs):
        if accum + lengths[curr] >= lookahead:
            frac = (lookahead - accum) / max(lengths[curr], 1e-6)
            pt = sub_pts[curr] + frac * segs[curr]
            return pt.tolist(), best_idx
        accum += lengths[curr]
        curr += 1
    return pts[-1].tolist(), len(pts) - 1


class PwmNavigator:
    def __init__(self, duty=.75):
        if not .7 <= duty <= 1:
            raise ValueError('Direct PWM duty must be between 70% and 100%')
        self.duty = duty
        self.progress_index = 0
        self.current_route = None
        self.reset()

    def reset(self):
        self.phase = 'idle'
        self.started = self.powered = None
        self.initial_pose = None
        self.directions = [0., 0.]
        self.burst_s = .1
        self.target_turned = 0.
        self.mode = 'idle'
        self.last_sequence = None
        self.transition_until = None
        self.progress_index = 0
        self.current_route = None

    @staticmethod
    def turn_burst(error, wheel_speed=.4, track=.235):
        # 75% PWM produces roughly 0.4 m/s wheel travel. Skid-steer friction
        # yields effective yaw rate ~1.8 rad/s (efficiency ~0.55).
        # Proportional burst sizing ensures confident rotation without stalling.
        eff_yaw_rate = (2 * wheel_speed / track) * 0.55
        return max(.20, min(1.8, abs(error) / max(0.1, eff_yaw_rate)))

    @staticmethod
    def calculate_heading_error(pose, route, goal, min_idx=0):
        distance = math.dist(pose[:2], goal[:2])
        if distance <= .25:
            return wrapped(goal[2] - pose[2]), len(route) - 1 if route else 0
        if not route:
            return None, 0
        lookahead = min(0.40, max(0.20, distance))
        target_pt, idx = path_lookahead(pose, route, lookahead=lookahead, min_idx=min_idx)
        if target_pt is None:
            return None, 0
        error = wrapped(math.atan2(target_pt[1] - pose[1], target_pt[0] - pose[0]) - pose[2])
        return error, idx

    def heading_error(self, pose, route, goal):
        if self.current_route != route:
            self.current_route = list(route) if route else []
            self.progress_index = 0
        error, idx = self.calculate_heading_error(pose, route, goal, min_idx=self.progress_index)
        self.progress_index = max(self.progress_index, idx)
        return error

    def target(self, pose, route, goal):
        """Three-tier motion model: Straight Forward, Continuous Sway, or In-Place Turn."""
        dist_to_goal = math.dist(pose[:2], goal[:2])
        if dist_to_goal <= .25:
            error = self.heading_error(pose, route, goal)
            if error is None or abs(error) <= .25:
                return [0., 0.], 'Within arrival tolerance'
            sign = 1 if error > 0 else -1
            return [-sign * self.duty, sign * self.duty], 'Aligning destination heading'

        error = self.heading_error(pose, route, goal)
        if error is None:
            return [0., 0.], 'No measured-map route'

        abs_err = abs(error)

        # Tier 1: Confident Straight Forward when path alignment is within ~4.5°
        if abs_err <= 0.08:
            return [self.duty, self.duty], 'Advancing straight along route'

        # Tier 2: Continuous Forward Sway (curving) between ~5° and ~35°
        # Both sides drive forward with differential speed, curving onto the route
        # without killing momentum or doing jerky in-place stops.
        if abs_err < 0.60:
            steer = (abs_err - 0.08) / (0.60 - 0.08)
            inner = round(max(0.38, self.duty * (1.0 - 0.50 * steer)), 3)
            if error > 0:
                # Target is to the right: left side full power, right side inner duty
                return [self.duty, inner], f'Swaying right along route (err={math.degrees(error):.1f}°)'
            else:
                # Target is to the left: left side inner duty, right side full power
                return [inner, self.duty], f'Swaying left along route (err={math.degrees(error):.1f}°)'

        # Tier 3: Dedicated In-Place Turn for sharp corners, corridor bends, and 180° turns
        sign = 1 if error > 0 else -1
        return [-sign * self.duty, sign * self.duty], f'Turning toward route (err={math.degrees(error):.1f}°)'

    def update(self, now, pose, route, goal, motor, velocity, sequence, clear, bounded_pwm=False):
        if not clear:
            self.phase = 'idle'
            self.directions = [0., 0.]
            return [0., 0.], 'PWM withheld by measured navigation guard'

        desired, reason = self.target(pose, route, goal)
        if not any(desired):
            self.phase = 'idle'
            self.directions = [0., 0.]
            return [0., 0.], reason

        is_turn = desired[0] * desired[1] < 0
        curr_error = self.heading_error(pose, route, goal) or 0.

        # Brief 0.08s zero-duty transition when switching between in-place turn and forward drive
        # to protect H-bridge polarity reversals without causing noticeable stutter.
        was_turn = self.directions[0] * self.directions[1] < 0
        if self.phase == 'driving' and is_turn != was_turn:
            self.transition_until = now + 0.08
            self.phase = 'transitioning'

        if self.phase == 'transitioning':
            if now < (self.transition_until or 0):
                return [0., 0.], 'Switching motion mode'
            self.phase = 'idle'
            self.transition_until = None

        if is_turn:
            # Execute confident in-place pivot until heading aligns with the corridor
            if self.phase != 'turning':
                self.phase = 'turning'
                self.started = now
                self.powered = None
                self.initial_pose = list(pose)
                self.target_turned = abs(curr_error)
                self.burst_s = self.turn_burst(curr_error)

            pwm = motor.get('pwm', [])
            if self.powered is None and any(value > 0 for value in pwm):
                self.powered = now

            turned = abs(wrapped(pose[2] - self.initial_pose[2]))
            aligned = abs(curr_error) <= 0.20
            deadline = self.powered is not None and now - self.powered >= self.burst_s
            timeout = now - self.started >= max(1.5, self.burst_s + 0.4)

            if turned >= max(0.2, self.target_turned - 0.1) or aligned or deadline or timeout:
                self.phase = 'idle'
                self.directions = [0., 0.]
                return [0., 0.], 'Turn completed; continuing along route'

            self.directions = list(desired)
            return list(desired), reason

        # Tier 1 & 2: Forward and Sway drive continuously without discrete pulse stops!
        # When clear, Kenny glides forward along the corridor with adaptive steering.
        self.phase = 'driving'
        self.directions = list(desired)
        return list(desired), reason


def aruco_guided_velocity(command, pose, route, goal, marker_pose, maximum=.05):
    """Use a fresh surveyed marker yaw to steer a forward arc, not a timed pivot."""
    if math.dist(pose[:2], marker_pose[:2]) > .35 or abs(wrapped(pose[2]-marker_pose[2])) > .6:
        return list(command), None
    measured = [*pose[:2], marker_pose[2]]
    error, _ = PwmNavigator.calculate_heading_error(measured, route, goal)
    if error is None:
        return list(command), None
    turn = max(-.45, min(.45, 1.2 * error))
    if math.dist(pose[:2], goal[:2]) <= .25:
        return [0., turn if abs(error) > .25 else 0.], error
    if abs(error) >= .85:
        return [0., turn], error
    forward = min(maximum, max(.025, .75 * maximum + .25 * command[0])) * max(.35, math.cos(error))
    turn = max(-.45, min(.45, .85 * turn + .15 * command[1]))
    return [forward, turn], error
