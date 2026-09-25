# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Unit tests for the Isaac-free helpers of search_mission_scene.py."""

import math
import sys
from pathlib import Path

import pytest

_UTILS = Path(__file__).resolve().parents[2]  # simulation/isaac-sim/utils
if str(_UTILS) not in sys.path:
    sys.path.insert(0, str(_UTILS))

from mtl_scene import config as C  # noqa: E402
from mtl_scene import gimbal as G  # noqa: E402

REPO = Path(__file__).resolve().parents[5]

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_usd_camera_looks_along_gimbal_x_with_up_along_z():
    q = G.USD_CAMERA_IN_GIMBAL_QUAT
    assert _close(G.quat_rotate(q, (0, 0, -1)), (1, 0, 0))   # USD view direction -> boresight
    assert _close(G.quat_rotate(q, (0, 1, 0)), (0, 0, 1))    # USD up -> gimbal up
    assert _close(G.quat_rotate(q, (1, 0, 0)), (0, -1, 0))   # image right


def test_euler_convention_matches_follower():
    # pitch +90 deg (nadir): boresight straight down; pitch 45 & yaw 90: north-down
    q = G.quat_from_euler_zyx(0.0, math.pi / 2, 0.3)
    assert _close(G.quat_rotate(q, (1, 0, 0)), (0, 0, -1))
    q = G.quat_from_euler_zyx(0.0, math.radians(45), math.radians(90))
    s = math.sqrt(0.5)
    assert _close(G.quat_rotate(q, (1, 0, 0)), (0, s, -s))
    for rpy in [(0.1, 0.2, 0.3), (-0.4, -0.9, 2.5)]:
        assert _close(G.euler_zyx_from_quat(G.quat_from_euler_zyx(*rpy)), rpy, 1e-9)


def test_gimbal_pose_is_stabilised_and_relative_pose_round_trips():
    body_q = G.quat_from_euler_zyx(0.2, -0.1, 1.0)
    pos, q = G.gimbal_world_pose((10, 20, 30), body_q, (0.1, 0.0, -0.08), (0.0, 0.7, -0.4))
    # orientation ignores the airframe attitude (earth-stabilised)
    assert _close(q, G.quat_from_euler_zyx(0.0, 0.7, -0.4))
    # position rides the airframe
    off = G.quat_rotate(body_q, (0.1, 0.0, -0.08))
    assert _close(pos, (10 + off[0], 20 + off[1], 30 + off[2]))
    root_p, root_q = (5.0, -3.0, 0.07), G.quat_from_euler_zyx(0, 0, 0.5)
    lp, lq = G.relative_pose(root_p, root_q, pos, q)
    back = G.quat_rotate(root_q, lp)
    assert _close((back[0] + 5.0, back[1] - 3.0, back[2] + 0.07), pos)
    assert _close(G.quat_mul(root_q, lq), q) or _close(G.quat_mul(root_q, lq), tuple(-c for c in q))


def test_focal_length_for_60_deg():
    f = G.focal_length_mm(60.0)
    assert 2 * math.degrees(math.atan(20.955 / (2 * f))) == pytest.approx(60.0)


def test_actuator_clamps_and_slews():
    ax = G.GimbalAxis(G.GimbalLimits(slew_rate=math.radians(90)), (0.0, math.radians(60), 0.0))
    s = ax.step((0.0, math.radians(200), 0.0), 0.1)            # beyond the pitch stop
    assert s[1] == pytest.approx(math.radians(69))
    for _ in range(100):
        s = ax.step((0.0, math.radians(200), 0.0), 0.1)
    assert s[1] == pytest.approx(math.radians(110))
    for _ in range(3):
        s = ax.step((0.0, s[1], math.radians(179)), 1.0)
    assert s[2] == pytest.approx(math.radians(179))
    s = ax.step((0.0, s[1], math.radians(-179)), 0.01)        # short way round: +0.9 deg, not -358
    assert s[2] == pytest.approx(math.radians(179.9))


def test_config_from_scenario_and_fleet(tmp_path):
    sc = {"sensor": {"fov_deg": 60.0}, "aircraft": {"altitude_m": 30.0},
          "team": {"agents": [{"name": "robot_1", "home_ned": [-170.0, -170.0]},
                              {"name": "robot_2", "home_ned": [-170.0, -158.0]}]},
          "airstack": {"sim_gimbal": {"width": 320, "slew_rate_deg_s": 90}}}
    cfgs = C.drone_configs_from_scenario(sc)
    assert [c["domain_id"] for c in cfgs] == [1, 2]
    assert (cfgs[1]["x_m"], cfgs[1]["y_m"]) == (-158.0, -170.0)   # ENU x = e, y = n
    assert C.spawn_mismatches(cfgs, sc) == []
    cfgs[1]["x_m"] = 0.0
    assert C.spawn_mismatches(cfgs, sc)
    g = C.gimbal_params(sc)
    assert g["width"] == 320 and g["height"] == 480 and g["slew_rate_rad_s"] == pytest.approx(math.pi / 2)
    assert C.remap_path("/root/AirStack/config/fleets/f.yaml", Path("/isaac-sim/AirStack")) == \
        "/isaac-sim/AirStack/config/fleets/f.yaml"


def test_repo_fleet_and_bundle_agree():
    fleet = REPO / "config" / "fleets" / "mtl_search_fleet.yaml"
    bundle = REPO / "stacks" / "mtl_search" / "config"
    if not fleet.is_file() or not (bundle / "scenario.json").is_file():
        pytest.skip("repo layout not available")
    pytest.importorskip("yaml")
    cfgs = C.drone_configs_from_fleet(fleet, REPO)
    sc, gt, png = C.load_bundle(bundle)
    assert [c["robot_name"] for c in cfgs] == ["robot_1", "robot_2", "robot_3"]
    assert all(c["gimbal"] and not c["camera"] and not c["lidar"] for c in cfgs)
    assert C.spawn_mismatches(cfgs, sc) == []
    assert png is not None and len(gt["targets"]) > 0
