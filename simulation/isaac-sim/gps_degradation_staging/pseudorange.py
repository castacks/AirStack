"""
pseudorange.py

Weighted Least Squares (WLS) pseudorange solver.

Returns the instantaneous position bias (east, north, up) in metres
caused by multipath and thermal noise. This is the geometry-driven
component; the slow atmospheric/clock drift is handled by the OU process
in model.py.

Weights: sin²(elevation) — upweights high-elevation sats with cleaner signals.
"""
from __future__ import annotations

import math
import random
from typing import List, Tuple

import numpy as np


class PseudorangeSolver:
    def __init__(self, cfg, rng=None):
        self._cfg = cfg
        self._rng = rng or random.Random()

    def solve(self, sv_signals: list) -> Tuple[float, float, float]:
        """Return WLS position error (east, north, up) in metres."""
        tracked = list(sv_signals)
        if len(tracked) < 4:
            return 0.0, 0.0, 0.0

        cfg = self._cfg
        rows, pr_errors, weights = [], [], []

        for s in tracked:
            az = math.radians(s.azimuth_deg)
            el = math.radians(s.elevation_deg)
            c  = math.cos(el)
            rows.append([c * math.sin(az), c * math.cos(az), math.sin(el), 1.0])

            # Pseudorange error = multipath bias + elevation-scaled thermal noise
            thermal = cfg.uere_base_m / max(math.sin(el), 0.05)
            if s.elevation_deg < cfg.low_elevation_threshold_deg:
                thermal *= cfg.low_elevation_noise_factor
            pr_errors.append(s.multipath_extra_m + self._rng.gauss(0.0, thermal))

            weights.append(math.sin(el) ** 2)

        A = np.array(rows, dtype=float)
        b = np.array(pr_errors, dtype=float)
        W = np.diag(weights)

        try:
            AtW = A.T @ W
            x   = np.linalg.solve(AtW @ A, AtW @ b)
        except np.linalg.LinAlgError:
            return 0.0, 0.0, 0.0

        cap = cfg.multipath_max_extra_m
        return (
            float(max(-cap, min(cap, x[0]))),
            float(max(-cap, min(cap, x[1]))),
            float(max(-cap, min(cap, x[2]))),
        )
