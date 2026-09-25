# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Unit tests for mtl_trajectory_follower.gimbal_math (pure Python)."""

import math
import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parent.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from mtl_trajectory_follower import gimbal_math as G  # noqa: E402

D = math.radians

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_task_formula_in_enu():
    # drone at 30 m, target 30 m east on the ground -> 45 deg down, yaw 0 (East)
    pitch, yaw = G.look_at_angles((0, 0, 30), (30, 0, 0))
    assert pitch == pytest.approx(D(45))
    assert yaw == pytest.approx(0.0)
    # target north-west, below
    pitch, yaw = G.look_at_angles((10, 10, 20), (0, 20, 0))
    assert yaw == pytest.approx(D(135))
    assert pitch == pytest.approx(math.atan2(20, math.hypot(10, 10)))
    # straight down
    assert G.look_at_angles((5, 5, 30), (5, 5, 0))[0] == pytest.approx(D(90))


def test_euler_matrix_round_trip_and_boresight():
    for r, p, y in [(0.1, 0.2, 0.3), (-0.5, 1.2, -2.0), (0.0, -0.3, 3.0), (0.4, D(89.9), 1.0)]:
        m = G.euler_zyx_to_matrix(r, p, y)
        r2, p2, y2 = G.matrix_to_euler_zyx(m)
        assert _close(G.euler_zyx_to_matrix(r2, p2, y2)[0], m[0], 1e-7)
        assert _close(G.euler_zyx_to_matrix(r2, p2, y2)[2], m[2], 1e-7)
        b = (m[0][0], m[1][0], m[2][0])
        assert _close(b, G.boresight_from_euler(p, y))


def test_gimbal_lock_uses_yaw_hint():
    m = G.euler_zyx_to_matrix(0.3, D(90), 0.7)
    r, p, y = G.matrix_to_euler_zyx(m, yaw_hint=0.7)
    assert p == pytest.approx(D(90))
    assert y == pytest.approx(0.7)
    assert r == pytest.approx(0.3, abs=1e-6)


def test_quaternion_round_trip_and_optical_frame():
    m = G.euler_zyx_to_matrix(0.2, -0.4, 1.1)
    q = G.matrix_to_quat(m)
    assert _close(G.quat_to_matrix(*q)[1], m[1], 1e-9)
    # optical: z = gimbal x, x = -gimbal y, y = -gimbal z
    r = G.quat_to_matrix(*G.OPTICAL_FROM_GIMBAL_QUAT)
    cols = [tuple(r[i][j] for i in range(3)) for j in range(3)]
    assert _close(cols[0], (0, -1, 0)) and _close(cols[1], (0, 0, -1)) and _close(cols[2], (1, 0, 0))


def test_two_axis_points_exactly():
    pos, tgt = (3.0, -4.0, 30.0), (20.0, 5.0, 0.0)
    roll, pitch, yaw = G.two_axis_command(pos, tgt)
    assert roll == 0.0
    b = G.boresight_from_euler(pitch, yaw)
    s = (tgt[2] - pos[2]) / b[2]
    assert pos[0] + s * b[0] == pytest.approx(tgt[0])
    assert pos[1] + s * b[1] == pytest.approx(tgt[1])


def test_single_axis_nadir_and_abeam_cases():
    # tau = 0: a point directly abeam to the right is reachable exactly
    pos = (0.0, 0.0, 30.0)
    (roll, pitch, yaw), d = G.single_axis_command(pos, (0.0, -30.0, 0.0), 0.0, 0.0, D(80), D(5))
    assert d["phi"] == pytest.approx(D(45))       # right of track
    assert d["dp"] == pytest.approx(0.0)
    assert d["miss_m"] == pytest.approx(0.0, abs=1e-9)
    b = G.boresight_from_euler(pitch, yaw)
    assert b[0] == pytest.approx(0.0, abs=1e-12)   # no along-track component
    # forward-tilted mount (30 deg): the swept line stands h tan(tau) ahead
    ahead = 30.0 * math.tan(D(30))
    _, d = G.single_axis_command(pos, (ahead, 0.0, 0.0), 0.0, D(30), D(80), D(5))
    assert d["dp"] == pytest.approx(0.0, abs=1e-12)
    assert d["miss_m"] == pytest.approx(0.0, abs=1e-9)


def test_single_axis_respects_pitch_nudge_and_travel():
    pos = (0.0, 0.0, 30.0)
    # a point far ahead of the swept line: the nudge saturates at 5 deg, miss > 0
    _, d = G.single_axis_command(pos, (60.0, 0.0, 0.0), 0.0, D(30), D(80), D(5))
    assert abs(d["dp"]) == pytest.approx(D(5))
    assert d["dp_clipped"] == 1.0 and d["miss_m"] > 5.0
    # a point far to the left: phi clamps at the travel limit
    _, d = G.single_axis_command(pos, (0.0, 1000.0, 29.0), 0.0, 0.0, D(60), D(5))
    assert d["phi"] == pytest.approx(-D(60))
    assert d["phi_clipped"] == 1.0
    # yaw of the command follows the airframe: rotate the whole problem by 90 deg
    (_, p1, y1), _ = G.single_axis_command(pos, (5.0, -20.0, 0.0), 0.0, D(30), D(80), D(5))
    (_, p2, y2), _ = G.single_axis_command(pos, (20.0, 5.0, 0.0), D(90), D(30), D(80), D(5))
    assert p1 == pytest.approx(p2)
    assert G.wrap_pi(y2 - y1) == pytest.approx(D(90))


def test_slew_limit_wraps_yaw():
    out = G.slew_limit((0.0, 0.0, D(179)), (0.5, 1.0, D(-179)), D(1))
    assert out[0] == pytest.approx(D(1)) and out[1] == pytest.approx(D(1))
    assert out[2] == pytest.approx(D(-180)) or out[2] == pytest.approx(D(180))
