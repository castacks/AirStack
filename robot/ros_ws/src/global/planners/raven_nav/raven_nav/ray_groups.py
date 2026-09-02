"""The `RayGroup` value type shared by the ray behaviour and `ray_targets`.

A RayGroup is one bundle of same-bearing rays (origin + direction, no range)
that the ray behaviour's 45-degree angle binning produced — the OG grouping
lives in `raven_nav/behaviors/ray_behavior.py:angle_bin_groups`. This module
holds only the dataclass so `ray_targets.py` (kept as-is) has no dependency on
the behaviours.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class RayGroup:
    label: str
    ray_origins: np.ndarray
    ray_dirs:    np.ndarray
    ray_scores:  np.ndarray

    avg_origin:        np.ndarray = field(init=False)
    avg_dir:           np.ndarray = field(init=False)
    num_rays:          int        = field(init=False)
    avg_score:         float      = field(init=False)
    max_score:         float      = field(init=False)
    min_dist_to_robot: float      = field(init=False)
    avg_dist_to_robot: float      = field(init=False)

    def finalize(self, robot_pos: np.ndarray) -> None:
        self.num_rays = len(self.ray_origins)
        self.avg_origin = self.ray_origins.mean(axis=0)
        d = self.ray_dirs.mean(axis=0)
        self.avg_dir = d / (np.linalg.norm(d) + 1e-6)
        self.avg_score = float(np.asarray(self.ray_scores).mean())
        self.max_score = float(np.asarray(self.ray_scores).max())
        dists = np.linalg.norm(
            self.ray_origins - np.asarray(robot_pos, dtype=float)[None, :],
            axis=1)
        self.min_dist_to_robot = float(dists.min())
        self.avg_dist_to_robot = float(dists.mean())

    # Backwards-compatible alias: the kept tests and `ray_targets` were written
    # against the private name.
    _finalize = finalize
