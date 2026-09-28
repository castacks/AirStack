# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Stdlib tests of tigris_search_planner.rewards (the Python mirror of belief.cpp)."""

import math
import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from tigris_search_planner.rewards import (Detection, Grid, RewardParams, look_from_gimbal,  # noqa: E402
                                           look_from_pose, original_update, score_looks)

pytestmark = pytest.mark.unit

SCENARIO = {
    "mission": {"area": {"size_m": 200.0, "center_ned": [0.0, 0.0], "belief_res_m": 2.0}},
    "sensor": {"fov_deg": 60.0, "tilt_deg": 30.0,
               "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "p_out_of_range": 1e-6, "dt_ref_s": 0.1}},
    "airstack": {"belief": {"bumps": [{"n": 0.0, "e": 40.0, "sigma_n": 20.0, "sigma_e": 20.0, "amplitude": 0.4}],
                            "belief_cap": 0.85, "base_uncertainty": 0.0}},
}


def test_grid_mass_is_a_pmf_and_presence_is_floored():
    g = Grid.from_scenario(SCENARIO, res=4.0)
    assert g.nx == 50
    assert math.fsum(g.mass) == pytest.approx(1.0)
    assert min(g.presence0) >= 0.01 and max(g.presence0) <= 0.85 + 1e-12


def test_pose_and_gimbal_looks_agree():
    det = Detection.from_scenario(SCENARIO)
    a = look_from_pose(0, 0, 30, math.pi / 2, math.radians(60), math.radians(30), det, 1.0)
    b = look_from_gimbal(0, 0, 30, math.radians(60), math.pi / 2, math.radians(60), det, 1.0)
    assert a.gy == pytest.approx(30 * math.tan(math.radians(30)))
    assert (a.gx, a.gy, a.radius) == pytest.approx((b.gx, b.gy, b.radius))
    assert look_from_pose(0, 0, 80, 0, math.radians(60), math.radians(30), det, 1.0) is None


def test_original_update_branches():
    det, rp = Detection(), RewardParams()
    r, q = original_update(0.3, 20.0, det, rp)
    assert q < 0.3 and r > 0
    _, q = original_update(0.7, 20.0, det, rp)
    assert q > 0.7
    r, q = original_update(0.3, 100.0, det, rp)
    assert r == pytest.approx(0.0) and q == pytest.approx(0.3)


def test_curves_are_monotone_and_rate_independent():
    g = Grid.from_scenario(SCENARIO, res=4.0)
    det, rp = Detection.from_scenario(SCENARIO), RewardParams()
    fov, tilt = math.radians(60), math.radians(30)

    def run(dt):
        n = int(round(10.0 / dt))
        out = []
        for k in range(n + 1):
            t = k * dt
            x = -40.0 + 6.0 * t
            out.append((t, 6.0 * t, look_from_pose(x, 0.0, 30.0, 0.0, fov, tilt, det, (dt if k else 0.1) / det.dt_ref)))
        return score_looks(g, det, rp, out, edge_m=20.0)

    a, b = run(0.1), run(0.05)
    assert all(y >= x - 1e-12 for x, y in zip(a.matched, a.matched[1:]))
    assert all(y >= x - 1e-12 for x, y in zip(a.original, a.original[1:]))
    assert a.matched[-1] > 0 and a.original[-1] > 0
    assert a.matched[-1] == pytest.approx(b.matched[-1], rel=0.02)  # dt / dt_ref weighting
