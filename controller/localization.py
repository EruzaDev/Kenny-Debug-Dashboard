"""Uncertainty from consistent surveyed RGB-D landmarks, propagated with wheel travel."""
from collections import deque
import math
import numpy as np


class MarkerAnchor:
    def __init__(self, sigma=.15):
        if not math.isfinite(sigma) or sigma <= 0:
            raise ValueError('Marker survey/mount uncertainty must be positive')
        self.sigma = sigma
        self.samples = deque(maxlen=3)
        self.last_sample = -math.inf
        self.observed = -math.inf
        self.distance = 0.
        self.pose = None
        self.base_uncertainty = math.inf
        self.disagreement = False

    def observe(self, pose, now, distance, reference=None):
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            return False
        if now - self.last_sample > .75:
            self.samples.clear()
        if self.samples:
            previous = self.samples[-1]
            turn = abs(math.atan2(math.sin(pose[2]-previous[2]), math.cos(pose[2]-previous[2])))
            if np.linalg.norm(pose[:2]-previous[:2]) > .15 or turn > .15:
                self.samples.clear()
        self.last_sample = now
        self.samples.append(pose)
        if len(self.samples) < 3:
            return False
        values = np.asarray(self.samples)
        center = np.median(values[:, :2], axis=0)
        residual = math.dist(center, reference[:2]) if reference is not None else 0.
        self.disagreement = residual > 1.2
        if self.disagreement:
            return False
        spread = float(np.max(np.linalg.norm(values[:, :2]-center, axis=1)))
        self.base_uncertainty = math.sqrt(self.sigma**2 + spread**2 + residual**2)
        self.pose = [*map(float, center), math.atan2(np.sin(values[:, 2]).mean(), np.cos(values[:, 2]).mean())]
        self.observed, self.distance = now, distance
        return True

    def uncertainty(self, now, distance):
        if self.pose is None or self.disagreement:
            return 1.2
        travel = max(0., distance-self.distance)
        age = max(0., now-self.observed)
        return min(2.4, math.sqrt(self.base_uncertainty**2 + (.04*travel)**2 + (.002*age)**2))
