"""
DOP (Dilution of Precision) from visible LOS satellite geometry.

Standard geometry matrix approach: Q = (A^T A)^{-1} where each row of A is
[east, north, up, 1] for a unit vector toward a satellite. HDOP = sqrt(Q[0,0]
+ Q[1,1]), VDOP = sqrt(Q[2,2]).

Only LOS satellites contribute to DOP. NLOS-reflected satellites are excluded
because their pseudoranges carry a multipath bias that pollutes the geometry
estimate. Four satellites are the minimum for a 3D position + clock solve.
"""

import numpy as np
from typing import List, Tuple

from .visibility import SatelliteVisibility


def compute_dop(
    visibility: List[SatelliteVisibility],
) -> Tuple[float, float, float]:
    """
    Compute HDOP, VDOP, PDOP from LOS satellite ENU unit vectors.

    Returns:
        (hdop, vdop, pdop) — positive floats.
        Returns (99.0, 99.0, 99.0) if fewer than 4 LOS satellites (no 3D fix).
    """
    los = [sv for sv in visibility if sv.is_los]
    if len(los) < 4:
        return 99.0, 99.0, 99.0

    # Design matrix: each row [east, north, up, 1]
    A = np.zeros((len(los), 4))
    for i, sv in enumerate(los):
        e, n, u = sv.sat.enu_unit
        A[i] = [e, n, u, 1.0]

    AtA = A.T @ A
    try:
        Q = np.linalg.inv(AtA)
    except np.linalg.LinAlgError:
        return 99.0, 99.0, 99.0

    hdop = float(np.sqrt(max(Q[0, 0] + Q[1, 1], 0.0)))
    vdop = float(np.sqrt(max(Q[2, 2], 0.0)))
    pdop = float(np.sqrt(max(Q[0, 0] + Q[1, 1] + Q[2, 2], 0.0)))
    return hdop, vdop, pdop
