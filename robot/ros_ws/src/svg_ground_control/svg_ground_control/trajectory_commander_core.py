"""Pure helpers for trajectory_commander.

The ROS node wraps these helpers; tests import this module without needing a
running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def vector3(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f'{name} must contain exactly 3 values')
    return arr


def clamp_position(position, bounds_min, bounds_max) -> np.ndarray:
    return np.minimum(np.maximum(
        vector3(position, 'position'),
        vector3(bounds_min, 'bounds_min')),
        vector3(bounds_max, 'bounds_max'))


@dataclass
class ScriptedTrajectory:
    script_type: str
    center: np.ndarray
    radius_m: float
    amplitude: np.ndarray
    period_s: float
    waypoints: np.ndarray
    segment_durations: np.ndarray
    loop: bool

    def __post_init__(self) -> None:
        self.script_type = str(self.script_type)
        self.center = vector3(self.center, 'center')
        self.amplitude = np.asarray(self.amplitude, dtype=float)
        if self.amplitude.shape != (2,):
            raise ValueError('amplitude must contain exactly 2 values')
        self.radius_m = float(self.radius_m)
        self.period_s = float(self.period_s)
        self.loop = bool(self.loop)
        if self.period_s <= 0.0:
            raise ValueError('period_s must be positive')
        if self.radius_m < 0.0:
            raise ValueError('radius_m must be non-negative')
        if self.script_type not in ('circle', 'figure8', 'waypoints'):
            raise ValueError('script_type must be circle, figure8, or waypoints')

        self.waypoints = np.asarray(self.waypoints, dtype=float)
        if self.waypoints.size == 0:
            self.waypoints = np.zeros((0, 3), dtype=float)
        else:
            self.waypoints = self.waypoints.reshape(-1, 3)
        self.segment_durations = np.asarray(self.segment_durations, dtype=float)

        if self.script_type == 'waypoints':
            if len(self.waypoints) == 0:
                raise ValueError('waypoints script requires at least one waypoint')
            expected = max(0, len(self.waypoints) - 1)
            if len(self.segment_durations) != expected:
                raise ValueError(
                    f'segment_durations needs {expected} values for '
                    f'{len(self.waypoints)} waypoint(s)')
            if np.any(self.segment_durations <= 0.0):
                raise ValueError('all segment_durations must be positive')

    def sample(self, elapsed_s: float) -> np.ndarray:
        t = max(0.0, float(elapsed_s))
        if self.script_type == 'circle':
            theta = 2.0 * math.pi * t / self.period_s
            return self.center + np.array([
                self.radius_m * math.cos(theta),
                self.radius_m * math.sin(theta),
                0.0,
            ])
        if self.script_type == 'figure8':
            theta = 2.0 * math.pi * t / self.period_s
            return self.center + np.array([
                self.amplitude[0] * math.sin(theta),
                self.amplitude[1] * math.sin(theta) * math.cos(theta),
                0.0,
            ])
        return self._sample_waypoints(t)

    def _sample_waypoints(self, t: float) -> np.ndarray:
        if len(self.waypoints) == 1:
            return self.waypoints[0].copy()

        cumulative = np.concatenate(([0.0], np.cumsum(self.segment_durations)))
        total = float(cumulative[-1])
        if self.loop and total > 0.0:
            t = math.fmod(t, total)
        elif t >= total:
            return self.waypoints[-1].copy()

        segment = int(np.searchsorted(cumulative, t, side='right') - 1)
        segment = min(max(segment, 0), len(self.segment_durations) - 1)
        local_t = t - cumulative[segment]
        alpha = float(local_t / self.segment_durations[segment])
        return ((1.0 - alpha) * self.waypoints[segment]
                + alpha * self.waypoints[segment + 1])
