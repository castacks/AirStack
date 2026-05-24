"""
Multipath position error via weighted least-squares pseudorange solve.

For each usable satellite (LOS or NLOS-reflected), a pseudorange perturbation
is assembled from:
  - LOS satellites: elevation-dependent Gaussian noise only
  - NLOS-reflected: multipath_extra_m bias + noise

A WLS solve over the design matrix projects these pseudorange errors into
ENU position space. The result is the position error the receiver would report
relative to truth.

References:
  Axelrad et al. 1996 — SNR-based multipath, elevation-dependent noise
  Pant et al. 2022 — GNSS multipath Gazebo plugin (pseudorange geometry)
"""

import math
import numpy as np
from typing import List, Tuple

from .visibility import SatelliteVisibility
from .config import GpsDegradationConfig


def compute_position_error(
    visibility: List[SatelliteVisibility],
    cfg: GpsDegradationConfig,
    rng: np.random.Generator,
) -> Tuple[float, float, float]:
    """
    Returns (delta_east_m, delta_north_m, delta_up_m) position error.
    Returns (0.0, 0.0, 0.0) if fewer than 4 usable satellites.

    Fully NLOS-blocked satellites (is_los=False, multipath_extra_m=0) are
    excluded — they carry no usable pseudorange.
    """
    usable = [sv for sv in visibility
              if sv.is_los or sv.multipath_extra_m > 0.0]
    if len(usable) < 4:
        return 0.0, 0.0, 0.0

    n_sats = len(usable)
    A = np.zeros((n_sats, 4))       # [east, north, up, clock]
    delta_pr = np.zeros(n_sats)
    weights = np.zeros(n_sats)

    for i, sv in enumerate(usable):
        e, n, u = sv.sat.enu_unit
        A[i] = [e, n, u, 1.0]

        el_deg = sv.sat.elevation_deg
        el_rad = math.radians(max(el_deg, 1.0))

        # Elevation-dependent noise sigma (higher elevation = lower noise)
        sigma = cfg.uere_base_m / math.sin(el_rad)
        if el_deg < cfg.low_elevation_threshold_deg:
            sigma *= cfg.low_elevation_noise_factor

        bias = sv.multipath_extra_m if not sv.is_los else 0.0
        delta_pr[i] = bias + rng.normal(0.0, sigma)

        # WLS weight: sin²(elevation) — standard geodetic weighting
        weights[i] = math.sin(el_rad) ** 2

    W = np.diag(weights)
    AtWA = A.T @ W @ A
    AtWb = A.T @ W @ delta_pr

    try:
        dx = np.linalg.solve(AtWA, AtWb)
    except np.linalg.LinAlgError:
        return 0.0, 0.0, 0.0

    # dx = [delta_east, delta_north, delta_up, delta_clock_bias]
    return float(dx[0]), float(dx[1]), float(dx[2])
