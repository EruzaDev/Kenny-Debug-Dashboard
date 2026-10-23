"""Gravity-referenced MPU yaw increments for a level, planar mobile base.

No compass heading or acceleration-derived translation is invented. Calibration
requires disarmed, measured-stationary wheels and two seconds of stable IMU data.
"""
from collections import deque
import math

import numpy as np


class GyroYaw:
    def __init__(self):
        self.samples = deque(maxlen=250)
        self.calibration = deque(maxlen=150)
        self.vertical = None
        self.bias = None
        self.gravity_magnitude = None
        self.frame = None
        self.last_stamp = None
        self.reason = 'Waiting for stationary IMU calibration'

    def reset(self, reason):
        self.samples.clear()
        self.calibration.clear()
        self.vertical = self.bias = None
        self.gravity_magnitude = None
        self.reason = reason

    def observe(self, stamp, gyro, acceleration, stationary, frame):
        gyro, acceleration = np.asarray(gyro, dtype=float), np.asarray(acceleration, dtype=float)
        if (not frame or not math.isfinite(stamp) or gyro.shape != (3,) or acceleration.shape != (3,)
                or not np.isfinite(gyro).all() or not np.isfinite(acceleration).all()):
            self.reset('Invalid IMU measurement')
            return
        if self.frame != frame:
            self.reset('IMU frame changed; recalibrating')
            self.frame = frame
            self.last_stamp = None
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                return
            if stamp-self.last_stamp > .10:
                self.reset('IMU gap; stationary recalibration required')
        self.last_stamp = stamp
        magnitude = float(np.linalg.norm(acceleration))
        # This installed compatible chip reads ~9.25 m/s² at rest. Estimate
        # its baseline rather than requiring an ideal factory accelerometer.
        gravity_valid = (8.8 <= magnitude <= 10.8 if self.gravity_magnitude is None
                         else abs(magnitude-self.gravity_magnitude) <= .5)
        if not gravity_valid or np.linalg.norm(gyro) > 2.:
            self.reset('IMU acceleration/rotation outside planar operating range')
            return
        if self.bias is None:
            if not stationary or np.linalg.norm(gyro) > .2:
                self.calibration.clear()
                self.reason = 'Waiting for disarmed, stationary calibration'
                return
            self.calibration.append((stamp, gyro, acceleration))
            if len(self.calibration) < 75 or stamp-self.calibration[0][0] < 2.:
                return
            rates = np.array([sample[1] for sample in self.calibration])
            accelerations = np.array([sample[2] for sample in self.calibration])
            if np.max(np.std(rates, axis=0)) > .005 or np.max(np.std(accelerations, axis=0)) > .10:
                self.calibration.clear()
                self.reason = 'IMU vibration during calibration'
                return
            self.bias = rates.mean(axis=0)
            gravity = accelerations.mean(axis=0)
            self.gravity_magnitude = float(np.linalg.norm(gravity))
            self.vertical = gravity / self.gravity_magnitude
        # A changed gravity direction means the base tilted or the board moved.
        if np.dot(acceleration/magnitude, self.vertical) < math.cos(math.radians(10)):
            self.reset('IMU tilted/remounted; stationary recalibration required')
            return
        self.samples.append((stamp, float(np.dot(gyro-self.bias, self.vertical))))
        self.reason = ''

    def delta(self, start, end):
        """Integrate exactly the encoder interval, allowing <=60 ms edge hold."""
        if self.bias is None or not self.samples or not 0 < end-start < 1.:
            return None
        points = list(self.samples)
        if points[0][0] > start or end-points[-1][0] > .06 or points[-1][0] > end+.1:
            return None
        times = [start] + [t for t, _ in points if start < t < end] + [end]
        coverage = [t for t, _ in points if start-.06 <= t <= end+.06]
        if not coverage or max(np.diff([start, *[t for t in coverage if start < t < end], end])) > .06:
            return None
        rates = np.interp(times, [t for t, _ in points], [r for _, r in points])
        return float(sum((rates[i]+rates[i+1])*.5*(times[i+1]-times[i]) for i in range(len(times)-1)))
