# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Pure-numeric LiDAR filter validation helpers (no ROS imports).

Shared by:
  scripts/validate_lidar_filter_clouds.py  — runtime echo-based cloud check (robot container)
  test/test_validation_core.py             — pytest unit tests (``pytest -m unit``)
"""

from __future__ import annotations

import numpy as np


def near_range_tolerance(near_range_m: float) -> float:
    """Slack around ``near_range_m`` (same rule as the validate script)."""
    return max(0.05, float(near_range_m) * 0.05)


def ranges_xyz_from_points_xyz(points: np.ndarray) -> np.ndarray | None:
    """Euclidean range per row for ``(N, 3)`` xyz.

    Returns ``None`` if shape is wrong or any coordinate is non-finite.
    """
    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        return None
    if arr.size == 0:
        return np.array([], dtype=np.float64)
    if not np.isfinite(arr).all():
        return None
    return np.linalg.norm(arr, axis=1)


def validate_filtered_ranges(
    ranges: np.ndarray,
    near_range_m: float,
    *,
    long_range_min_m: float = 2.0,
) -> tuple[bool, str]:
    """Check filtered cloud range statistics against ``near_range_m``.

    Returns ``(True, "")`` on success, else ``(False, reason)``.
    """
    fr = np.asarray(ranges, dtype=np.float64)
    if fr.size == 0:
        return False, 'filtered cloud is empty'
    mn_f = float(fr.min())
    tol = near_range_tolerance(near_range_m)
    if mn_f < float(near_range_m) - tol:
        return (
            False,
            f'filtered min range {mn_f:.4f}m < near_range_m ({near_range_m}) - tol {tol:.4f}',
        )
    if float(fr.max()) < long_range_min_m:
        return (
            False,
            f'expected long-range returns; filtered max range {float(fr.max()):.4f}m',
        )
    return True, ''


def raw_filtered_near_range_ok(
    mn_raw: float,
    mn_filtered: float,
    near_range_m: float,
    tol: float | None = None,
) -> bool:
    """If raw has near-field clutter, filtered minimum must still clear ``near_range_m``."""
    t = near_range_tolerance(near_range_m) if tol is None else tol
    if mn_raw < float(near_range_m) - t and mn_filtered < float(near_range_m) - t:
        return False
    return True
