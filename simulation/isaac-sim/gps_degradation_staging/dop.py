"""
dop.py

GPS DOP (Dilution of Precision) from the satellite geometry matrix.

Q = (AᵀA)⁻¹  where A rows are [East, North, Up, 1] unit vectors of LOS sats.
HDOP = sqrt(Q[0,0] + Q[1,1])
VDOP = sqrt(Q[2,2])
PDOP = sqrt(Q[0,0] + Q[1,1] + Q[2,2])

Returns (99, 99, 99) when fewer than 4 LOS satellites are available.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


class DopCalculator:
    def compute(self, sv_signals: list) -> Tuple[float, float, float]:
        """Return (hdop, vdop, pdop). Uses only LOS satellites."""
        los = [s for s in sv_signals if s.is_los]
        if len(los) < 4:
            return 99.0, 99.0, 99.0

        rows = []
        for s in los:
            az = math.radians(s.azimuth_deg)
            el = math.radians(s.elevation_deg)
            c  = math.cos(el)
            rows.append([c * math.sin(az), c * math.cos(az), math.sin(el), 1.0])

        A = np.array(rows, dtype=float)
        try:
            Q = np.linalg.inv(A.T @ A)
        except np.linalg.LinAlgError:
            return 99.0, 99.0, 99.0

        hdop = math.sqrt(max(Q[0, 0] + Q[1, 1], 0.0))
        vdop = math.sqrt(max(Q[2, 2], 0.0))
        pdop = math.sqrt(max(Q[0, 0] + Q[1, 1] + Q[2, 2], 0.0))
        return float(hdop), float(vdop), float(pdop)
