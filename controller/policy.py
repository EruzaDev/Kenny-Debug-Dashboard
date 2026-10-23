"""Resident PPO and the checkpoint's exact live observation layout."""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np

FEATURES = [('lidar_range', 72), ('lidar_valid', 72), ('depth_range', 36), ('depth_valid', 36),
            ('floor_drop_range', 12), ('floor_valid', 12), ('downward_hazard', 3), ('downward_valid', 3),
            ('route_points_xy', 6), ('route_valid', 3), ('goal_relative_xy', 2), ('measured_velocity', 2),
            ('previous_requested_action', 2), ('previous_executed_action', 2),
            ('localization_health', 3), ('sensor_age_and_guard', 3)]


def local(points, pose):
    delta = np.asarray(points) - pose[:2]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return delta @ np.array([[c, -s], [s, c]])


def lookahead(route, pose):
    if not len(route):
        return np.zeros((3, 2)), np.zeros(3)
    route = np.asarray(route)
    if len(route) == 1:
        return local(np.repeat(route, 3, axis=0), pose), np.ones(3)
    segments = np.diff(route, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    fractions = np.divide(np.sum((pose[:2] - route[:-1]) * segments, axis=1), lengths ** 2,
                          out=np.zeros_like(lengths), where=lengths > 1e-9)
    projections = route[:-1] + np.clip(fractions, 0, 1)[:, None] * segments
    index = int(np.argmin(np.linalg.norm(projections - pose[:2], axis=1)))
    forward = np.vstack((projections[index], route[index + 1:]))
    cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(forward, axis=0), axis=1))]
    points = np.column_stack([np.interp([.5, 1., 2.], cumulative, forward[:, axis]) for axis in (0, 1)])
    return local(points, pose), np.ones(3)


class ResidentPolicy:
    def __init__(self, checkpoint):
        from kenny_rl.config import load_config
        from kenny_rl.env import KennyEnv
        from stable_baselines3 import PPO
        import torch
        torch.set_num_threads(1)
        checkpoint = Path(checkpoint).resolve()
        source = checkpoint.parent.parent if checkpoint.parent.name == 'checkpoints' else checkpoint.parent
        self.robot, self.config, training = load_config(source / 'config.json')
        self.contract = json.loads((source / 'contract.json').read_text())
        env = KennyEnv(self.robot, self.config)
        expected = json.loads(json.dumps(env.contract()))
        env.close()
        if self.contract != expected or self.contract['features'] != [list(v) for v in FEATURES]:
            raise ValueError('Checkpoint contract is incompatible with the live feature adapter')
        self.model = PPO.load(checkpoint, device='cpu', custom_objects={
            'lr_schedule': lambda _: 0., 'learning_rate': 0.,
            'clip_range': lambda _: float(training.get('clip_range', .2)), 'clip_range_vf': None})
        self.history = deque(maxlen=self.config.history)
        self.requested = self.executed = np.array([-1., 0.], dtype=np.float32)
        self.last_explanation = None
        # Allocate and warm inference once. This is not a sensor observation or command.
        self.model.predict(np.zeros(self.contract['frame_size'] * self.config.history, dtype=np.float32), deterministic=True)

    def destination_changed(self):
        # Clear old goal-relative features, while keeping sensors/map/model alive.
        self.history.clear()
        self.requested = self.executed = np.array([-1., 0.], dtype=np.float32)

    def predict(self, sensors, pose, goal, route, dimensions, velocity, uncertainty, marker_age, now, intervened):
        r = self.robot
        sensors = dict(sensors)
        # An absent sensor is encoded as absent (validity=0), never as measured safe floor.
        if 'downward' not in sensors:
            sensors['downward'] = (np.zeros(3), np.zeros(3), now)
        points, route_valid = lookahead(route, np.asarray(pose))
        values = [sensors['lidar'][0] / r.lidar_range, sensors['lidar'][1],
                  sensors['depth'][0] / r.camera_range, sensors['depth'][1],
                  sensors['floor'][0] / r.camera_range, sensors['floor'][1],
                  sensors['downward'][0], sensors['downward'][1], points.ravel() / 3., route_valid,
                  local([goal[:2]], np.asarray(pose))[0] / np.asarray(dimensions),
                  np.asarray(velocity) / [r.max_speed, r.max_turn_rate], self.requested, self.executed,
                  [uncertainty / .5, min(marker_age / 30., 1.), float(uncertainty < .35)],
                  [max(0., now - sensors['lidar'][2]), 0., float(intervened)]]
        names = [name for name, _ in FEATURES]
        raw_values = [np.asarray(v).ravel().tolist() for v in values]
        frame = np.clip(np.concatenate([np.asarray(v).ravel() for v in values]), -1, 1).astype(np.float32)
        if frame.size != self.contract['frame_size'] or not np.isfinite(frame).all():
            raise ValueError('Live observation does not match checkpoint contract')
        if not self.history:
            self.history.extend(frame.copy() for _ in range(self.config.history))
        else:
            self.history.append(frame)
        action, _ = self.model.predict(np.concatenate(self.history), deterministic=True)
        if np.asarray(action).shape != (2,) or not np.isfinite(action).all():
            raise ValueError('Policy returned an invalid action')
        self.requested = np.clip(action, -1, 1)
        command = [(self.requested[0] + 1) * r.max_speed / 2, self.requested[1] * r.max_turn_rate]
        self.last_explanation = {
            'time': float(now),
            'feature_names': names,
            'features': raw_values,
            'normalized_action': self.requested.tolist(),
            'policy_command': list(command),
            'goal_relative_m': values[10],
            'measured_velocity': values[11],
            'localization_health': values[14],
            'sensor_age_and_guard': values[15],
            'intervened_by_previous_guard': bool(intervened),
            'history_length': len(self.history),
        }
        return command

    def record_executed(self, command):
        self.executed = np.array([2 * command[0] / self.robot.max_speed - 1,
                                  command[1] / self.robot.max_turn_rate], dtype=np.float32)


def isolated_coverage(valid, distances, clearance, wrap=False):
    """Bridge up to two missing angular rays between measured clear rays."""
    valid = np.asarray(valid, dtype=bool)
    distances = np.asarray(distances)
    left, right = np.roll(valid, 1), np.roll(valid, -1)
    left2, right2 = np.roll(valid, 2), np.roll(valid, -2)
    if not wrap:
        left[0] = right[-1] = False
        left2[:2] = right2[-2:] = False
    bridge1 = (~valid & left & right &
               (np.roll(distances, 1) > clearance) &
               (np.roll(distances, -1) > clearance))
    bridge2 = (~valid & ((left & right2 & (np.roll(distances, 1) > clearance) & (np.roll(distances, -2) > clearance)) |
                         (left2 & right & (np.roll(distances, 2) > clearance) & (np.roll(distances, -1) > clearance))))
    return valid | bridge1 | bridge2


def guard(command, sensors, now, radius=.163, speed_limit=.1, braking=.35, uncertainty=.05, require_downward=True, commissioning=False, measured_speed=None, bounded_pwm=False):
    v, w = float(command[0]), float(command[1])
    if not np.isfinite([v, w]).all():
        return [0., 0.], 'invalid_policy_command'
    for name in ('lidar', 'depth', 'floor') + (('downward',) if require_downward else ()):
        if name not in sensors or now - sensors[name][2] > .8:
            return [0., 0.], f'{name}_stale'
    lidar, depth, floor = (sensors[name] for name in ('lidar', 'depth', 'floor'))
    downward = sensors.get('downward')
    if downward is not None and (not np.asarray(downward[1]).all() or np.asarray(downward[0]).any()):
        return [0., 0.], 'downward_invalid_or_hazard'
    depth_valid, floor_valid = np.asarray(depth[1], dtype=bool), np.asarray(floor[1], dtype=bool)
    v, w = min(max(v, 0.), min(speed_limit, .05) if commissioning else speed_limit), max(-.3 if commissioning else -.6, min(.3 if commissioning else .6, w))
    stopping_speed=v if measured_speed is None else max(v,abs(measured_speed))
    # Commissioning PWM consists of <=0.1 s steps, with an independent 200 ms
    # firmware watchdog. Use a measured 0.30 m cutoff for this explicit profile
    # instead of a continuous-drive braking distance/uncertainty envelope.
    stopping = .30 if bounded_pwm else radius + uncertainty + .1 + stopping_speed * .3 + stopping_speed * stopping_speed / (2 * braking)
    camera_covered = (depth_valid.sum() >= 32 and floor_valid.sum() >= 10 and
                      isolated_coverage(depth_valid, depth[0], stopping + .1)[2:-2].all() and
                      isolated_coverage(floor_valid, floor[0], stopping + .15)[1:-1].all())
    turning_clearance = radius + .04
    measured_lidar = np.asarray(lidar[1], dtype=bool)
    rotation_clear = (isolated_coverage(measured_lidar, lidar[0], turning_clearance, wrap=True).all()
                      and not np.any(np.asarray(lidar[0])[measured_lidar] < turning_clearance)
                      and not np.any(np.asarray(depth[0])[depth_valid] < turning_clearance))
    if abs(w) > .05 and not rotation_clear:
        return [0., 0.], 'turning_footprint_unavailable_or_blocked'

    if v > 0:
        if np.count_nonzero(lidar[1]) < (65 if bounded_pwm else 60) or not camera_covered:
            return [0., 0.], 'sensor_coverage_incomplete'
        angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
        relevant = np.abs(angles) < math.pi / 3
        lidar_valid = isolated_coverage(lidar[1], lidar[0], stopping + .1, wrap=True)
        if not lidar_valid[relevant].all():
            return [0., 0.], 'lidar_forward_coverage_incomplete'
        floor_hazard = np.any(np.asarray(floor[0]) < (stopping if bounded_pwm else stopping + .15))
        clearance = min(np.min(np.asarray(lidar[0])[relevant & measured_lidar]),
                        np.min(np.asarray(depth[0])[depth_valid]))
        if clearance < stopping:
            if abs(w) > .05 and rotation_clear and not floor_hazard:
                return [0., w], ''
            return [0., 0.], 'observed_obstacle'
        if floor_hazard:
            return [0., 0.], 'observed_floor_drop'
        if not bounded_pwm and clearance < stopping + .25:
            v *= max(0., (clearance - stopping) / .25)
    return [v, w], ''
