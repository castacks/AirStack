# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for ``validation_core`` (numpy-only)."""

import sys
from pathlib import Path

import numpy as np
import pytest

# validation_core lives in the package module directory (importable as a
# package member both here and in the validate_lidar_filter_clouds.py script).
_pkg_root = Path(__file__).resolve().parent.parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

from lidar_point_cloud_filter.validation_core import (  # noqa: E402
    near_range_tolerance,
    ranges_xyz_from_points_xyz,
    raw_filtered_near_range_ok,
    validate_filtered_ranges,
)


@pytest.mark.unit
def test_near_range_tolerance():
    assert near_range_tolerance(0.5) == pytest.approx(0.05)
    assert near_range_tolerance(2.0) == pytest.approx(0.1)


@pytest.mark.unit
def test_ranges_xyz_from_points_xyz():
    pts = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    r = ranges_xyz_from_points_xyz(pts)
    assert r is not None
    assert r[0] == pytest.approx(5.0)
    assert r[1] == pytest.approx(1.0)


@pytest.mark.unit
def test_ranges_xyz_rejects_nonfinite():
    pts = np.array([[1.0, np.nan, 0.0]], dtype=np.float64)
    assert ranges_xyz_from_points_xyz(pts) is None


@pytest.mark.unit
def test_validate_filtered_ranges_ok():
    # All points between 1 m and 5 m from origin — clears default long_range_min_m=2
    fr = np.array([1.0, 2.0, 5.0], dtype=np.float64)
    ok, msg = validate_filtered_ranges(fr, near_range_m=0.75)
    assert ok
    assert msg == ''


@pytest.mark.unit
def test_validate_filtered_ranges_too_close():
    fr = np.array([0.2, 5.0], dtype=np.float64)
    ok, msg = validate_filtered_ranges(fr, near_range_m=0.75)
    assert not ok
    assert 'min range' in msg


@pytest.mark.unit
def test_validate_filtered_ranges_no_long_range():
    fr = np.array([1.0, 1.5], dtype=np.float64)
    ok, msg = validate_filtered_ranges(fr, near_range_m=0.75)
    assert not ok
    assert 'long-range' in msg


@pytest.mark.unit
def test_raw_filtered_near_range_ok():
    tol = near_range_tolerance(0.75)
    assert raw_filtered_near_range_ok(0.1, 0.8, 0.75, tol)
    assert not raw_filtered_near_range_ok(0.1, 0.2, 0.75, tol)
