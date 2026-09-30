# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Unit tests for mtl_search_planner.scenario (stdlib-only scenario module)."""

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parent.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from mtl_search_planner import scenario as S  # noqa: E402

REPO = _PKG.parents[5]  # <repo>/robot/ros_ws/src/global/planners/<pkg>

MISSION = {
    "name": "t", "seed": 5,
    "area": {"size_m": 200.0, "center_ned": [10.0, -30.0], "belief_res_m": 2.0},
    "belief": {"num_centroids": 3, "max_prior_peak": 0.4, "sigma_min_m": 15.0, "sigma_max_m": 25.0,
               "belief_cap": 0.85, "edge_margin_frac": 0.15},
    "targets": {"count": 6, "min_separation_m": 8.0},
    "team": {"max_flight_time_s": 60.0, "max_flight_distance_m": float("inf")},
    "aircraft": {"altitude_m": 30.0, "speed_mps": 6.0, "min_turn_radius_m": 12.0, "dt": 0.1},
    "mapping": {"target_cell_size_m": 20.0, "minimum_belief_mass": 2.0e-3, "max_cluster_radius_m": 45.0},
    "sensor": {"fov_deg": 60.0, "single_axis_gimbal": True, "tilt_deg": 30.0, "max_slant_range_m": 90.0,
               "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "threshold": 0.9}},
    "render": {"texture_px": 64, "show_valid_cells": True},
}
AGENTS = [{"name": "robot_1", "home_ned": [-80.0, -120.0]}, {"name": "robot_2", "home_ned": [-80.0, -108.0]}]

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def test_frames_round_trip():
    for n, e, d in [(0, 0, 0), (12.5, -3.0, -30.0), (-170, -158, 2.0)]:
        x, y, z = S.ned_to_enu(n, e, d)
        assert (x, y, z) == (e, n, -d)
        assert S.enu_to_ned(x, y, z) == (float(n), float(e), float(d))
    home = (-158.0, -170.0, 0.0)
    p = (10.0, 20.0, 30.0)
    assert S.map_to_world(S.world_to_map(p, home), home) == pytest.approx(p)


def test_build_is_deterministic_and_well_formed():
    sc1, gt1, _ = S.build_scenario(MISSION, AGENTS)
    sc2, gt2, _ = S.build_scenario(MISSION, AGENTS)
    assert json.dumps(sc1, sort_keys=True) == json.dumps(sc2, sort_keys=True)
    assert json.dumps(gt1, sort_keys=True) == json.dumps(gt2, sort_keys=True)
    assert sc1["schema"] == S.SCENARIO_SCHEMA
    assert sc1["team"]["max_flight_distance_m"] is None  # JSON has no inf
    n_min, n_max, e_min, e_max = S.area_bounds(sc1["mission"]["area"])
    assert sc1["cells"]["centers"], "expected valid cells"
    for n, e in sc1["cells"]["centers"]:
        assert n_min <= n <= n_max and e_min <= e <= e_max
    assert all(m > 2.0e-3 for m in sc1["cells"]["mass"])  # kept by per-cell belief mass
    assert sc1["cells"]["total_map_mass"] == pytest.approx(1.0, abs=1e-9)
    assert 0.0 < sum(sc1["cells"]["mass"]) <= 1.0
    assert sc1["mapping"]["minimum_belief_mass"] == 2.0e-3
    assert "mean_information_thresh" not in sc1["mapping"]
    assert "targets" not in json.dumps(sc1)  # ground truth never reaches the planner file
    tg = gt1["targets"]
    assert len(tg) == 6
    for i, a in enumerate(tg):
        for b in tg[i + 1:]:
            assert math.hypot(a["n"] - b["n"], a["e"] - b["e"]) >= 8.0
    assert [a["name"] for a in sc1["team"]["agents"]] == ["robot_1", "robot_2"]
    assert sc1["team"]["agents"][1]["start_ned"] == [-80.0, -108.0]


def test_altitude_separation_layers_agents():
    sc0, _, _ = S.build_scenario(MISSION, AGENTS)
    assert all("altitude_offset_m" not in a for a in sc0["team"]["agents"])  # off by default
    team = dict(MISSION["team"], altitude_separation_m=1.0)
    sc1, _, _ = S.build_scenario(dict(MISSION, team=team), AGENTS + [{"name": "robot_3", "home_ned": [-80.0, -96.0]}])
    assert [a["altitude_offset_m"] for a in sc1["team"]["agents"]] == [0.0, 1.0, 2.0]
    assert sc1["team"]["altitude_separation_m"] == 1.0
    assert sc1["aircraft"]["altitude_m"] == 30.0  # the team is still planned at one altitude


def test_solver_overrides_pass_through():
    sc, _, _ = S.build_scenario(dict(MISSION, solver={"extend_dist_m": 24.0}), AGENTS)
    assert sc["solver"] == {"extend_dist_m": 24.0}


def test_info_aware_block_is_normalised_and_optional():
    sc0, _, _ = S.build_scenario(MISSION, AGENTS)
    assert "info_aware" not in sc0  # no block in the mission -> bundle unchanged (plain planner)
    sc, _, _ = S.build_scenario(dict(MISSION, info_aware={"enabled": True}), AGENTS)
    assert sc["info_aware"] == {"enabled": True, "report_both": True}
    sc2, _, _ = S.build_scenario(dict(MISSION, info_aware={
        "enabled": False, "report_both": False, "level_sets": [[1], [0.5, 1]], "max_moves": "8",
        "persistence": 0.5, "reach_scales": [1, 0.8], "split_merge": 0}), AGENTS)
    assert sc2["info_aware"] == {"enabled": False, "report_both": False, "level_sets": [[1.0], [0.5, 1.0]],
                                 "reach_scales": [1.0, 0.8], "persistence": 0.5, "max_moves": 8,
                                 "split_merge": False}
    with pytest.raises(ValueError):
        S.build_scenario(dict(MISSION, info_aware={"level_sets": [[0.5, 0.9]]}), AGENTS)
    with pytest.raises(ValueError):
        S.build_scenario(dict(MISSION, info_aware={"level_sets": [[0.7, 0.5, 1.0]]}), AGENTS)


def test_seed_changes_the_world():
    other = dict(MISSION, seed=6)
    a, _, _ = S.build_scenario(MISSION, AGENTS)
    b, _, _ = S.build_scenario(other, AGENTS)
    assert a["cells"]["centers"] != b["cells"]["centers"]


def test_belief_mass_is_consistent():
    grid, bumps = S.generate_belief(MISSION["area"], MISSION["belief"], 3)
    assert len(bumps) == 3
    assert grid.shape == (101, 101)
    # the prior is a probability mass function over the raster
    assert grid.total_mass == pytest.approx(1.0, abs=1e-12)
    assert 0.0 < grid.peak < 1.0
    assert min(min(r) for r in grid.values) >= 0.0
    centers, masses, cell = S.extract_valid_cells(grid, 20.0, 0.0)
    assert cell == 20.0
    # thresh 0 keeps every block: the blocks tile all but the last raster row/col
    assert sum(masses) <= 1.0 + 1e-9
    assert sum(masses) >= 0.9


def test_cells_are_kept_by_mass_and_scale_free():
    grid, _ = S.generate_belief(MISSION["area"], MISSION["belief"], 3)
    thr = 2.0e-3
    centers, masses, _ = S.extract_valid_cells(grid, 20.0, thr)
    assert centers and all(m > thr for m in masses)
    # a host grid in any units gives the same cells and masses (normalised inside)
    scaled = S.BeliefGrid([[7.5 * v for v in row] for row in grid.values], grid.n_axis, grid.e_axis, grid.res_m)
    c2, m2, _ = S.extract_valid_cells(scaled, 20.0, thr)
    assert c2 == centers and m2 == pytest.approx(masses, rel=1e-9)
    # a stricter threshold keeps a strict subset
    c3, _, _ = S.extract_valid_cells(grid, 20.0, 20 * thr)
    assert 0 < len(c3) < len(centers) and all(c in centers for c in c3)
    with pytest.raises(ValueError):
        S.extract_valid_cells(grid, 20.0, 1.0)  # a probability must be < 1
    with pytest.raises(ValueError, match="minimum_belief_mass"):
        S.extract_valid_cells(grid, 20.0, 0.999)  # no block holds that much


def test_retired_mean_threshold_is_refused():
    old = dict(MISSION, mapping={"target_cell_size_m": 20.0, "mean_information_thresh": 0.05})
    with pytest.raises(ValueError, match="minimum_belief_mass"):
        S.build_scenario(old, AGENTS)


def test_agent_lookup_and_fleet_consistency():
    sc, _, _ = S.build_scenario(MISSION, AGENTS)
    idx, a = S.agent_entry(sc, "robot_2")
    assert idx == 1 and a["home_ned"] == [-80.0, -108.0]
    assert S.agent_home_enu(sc, "robot_1") == (-120.0, -80.0, 0.0)
    with pytest.raises(KeyError):
        S.agent_entry(sc, "robot_3")
    ok = S.check_fleet_consistency(sc, {"robot_1": [-120, -80, 0.07], "robot_2": [-108, -80, 0.07]})
    assert ok == []
    bad = S.check_fleet_consistency(sc, {"robot_1": [-100, -80, 0.07], "robot_3": [0, 0, 0]})
    assert any("robot_1" in p for p in bad) and any("robot_3" in p for p in bad)
    assert any("robot_2" in p for p in bad)


def test_detection_probability_matches_moon_sigmoid():
    det = MISSION["sensor"]["detection"] | {"p_out_of_range": 1e-6}
    assert S.detection_probability(30.0, det) == pytest.approx(1.0 / (1.1 + math.exp(0.1 * (30 - 61))))
    assert S.detection_probability(61.0, det) == pytest.approx(1.0 / 2.1)
    assert S.detection_probability(61.01, det) == 1e-6


def test_png_writer(tmp_path):
    sc, gt, grid = S.build_scenario(MISSION, AGENTS)
    paths = S.write_scenario_bundle(tmp_path, sc, gt, grid, texture_px=32)
    data = paths["belief_png"].read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    assert int.from_bytes(data[16:20], "big") == 32 and int.from_bytes(data[20:24], "big") == 32
    assert S.load_scenario(paths["scenario"])["mission"]["name"] == "t"
    assert len(S.load_ground_truth(paths["ground_truth"])["targets"]) == 6


def test_committed_bundle_is_up_to_date():
    """stacks/mtl_search/config must match mission.yaml + the fleet (regenerate if not)."""
    script = REPO / "scripts" / "mtl_generate_scenario.py"
    if not script.is_file() or not (REPO / "stacks" / "mtl_search" / "config" / "mission.yaml").is_file():
        pytest.skip("repo layout not available")
    pytest.importorskip("yaml")
    res = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr or res.stdout


# --------------------------------------------------------------------------- #
# planner selection, the curve block and the follower's gimbal law
# --------------------------------------------------------------------------- #
def test_planner_block_defaults_to_orienteering_and_is_optional():
    sc0, _, _ = S.build_scenario(MISSION, AGENTS)
    assert "planner" not in sc0 and "curve" not in sc0 and "follower" not in sc0["airstack"]
    sc, _, _ = S.build_scenario(dict(MISSION, planner={}), AGENTS)
    assert sc["planner"] == {"type": "orienteering", "compare_orienteering": True}
    sc2, _, _ = S.build_scenario(dict(MISSION, planner={"type": "curve", "compare_orienteering": False}), AGENTS)
    assert sc2["planner"] == {"type": "curve", "compare_orienteering": False}


@pytest.mark.parametrize("block, match", [
    ({"type": "curvy"}, "planner.type"),
    ({"typ": "curve"}, "unknown key"),
    ({"compare_orienteering": "yes"}, "compare_orienteering"),
])
def test_planner_block_refuses_bad_keys(block, match):
    with pytest.raises(ValueError, match=match):
        S.build_scenario(dict(MISSION, planner=block), AGENTS)


def test_curve_block_writes_only_what_the_mission_sets():
    curve = {"representation": "bspline", "endpoint_mode": ["return_home", "open"], "sweep_freq_hz": 0.1,
             "altitude_stagger_m": 1, "kernel": {"table_step_m": 1.5}, "init_strategies": ["greedy"],
             "max_realloc_trials": 2, "reallocate": False, "destinations_ned": [[0, 0], [10, 20]]}
    sc, _, _ = S.build_scenario(dict(MISSION, planner={"type": "curve"}, curve=curve), AGENTS)
    assert sc["curve"] == {"representation": "bspline", "endpoint_mode": ["return_home", "open"],
                           "destinations_ned": [[0.0, 0.0], [10.0, 20.0]], "kernel": {"table_step_m": 1.5},
                           "init_strategies": ["greedy"], "altitude_stagger_m": 1.0, "sweep_freq_hz": 0.1,
                           "max_realloc_trials": 2, "reallocate": False}
    # absent keys are left to the adapter's auto-scaling: nothing invented here
    sc2, _, _ = S.build_scenario(dict(MISSION, curve={}), AGENTS)
    assert sc2["curve"] == {}
    # every key the generator accepts is one the C++ adapter (mtlc_plan) reads
    assert set(S._CURVE_FLOAT + S._CURVE_INT + S._CURVE_BOOL + S._CURVE_OTHER) == {
        "representation", "endpoint_mode", "destinations_ned", "altitude_stagger_m", "kernel", "sweep_freq_hz",
        "sweep_range_margin", "knot_spacing_m", "num_control_points", "sample_spacing_m", "initial_heading_deg",
        "fast_grid_step_m", "explore_grid_step_m", "max_iter", "explore_iter", "explore_iter_warm",
        "coordination_sweeps", "init_strategies", "reallocate", "realloc_rounds", "max_realloc_trials"}


@pytest.mark.parametrize("curve, match", [
    ({"sweep_frq_hz": 0.2}, "unknown key"),
    ({"kernel": {"tablestep_m": 2}}, "unknown key"),
    ({"representation": "spline"}, "representation"),
    ({"endpoint_mode": "home"}, "endpoint_mode"),
    ({"endpoint_mode": ["open", "back"]}, "endpoint_mode"),
    ({"init_strategies": ["random"]}, "init_strategies"),
    ({"init_strategies": []}, "init_strategies"),
    ({"max_iter": 1.5}, "integer"),
    ({"sweep_freq_hz": "fast"}, "number"),
    ({"sweep_freq_hz": float("inf")}, "finite"),
    ({"reallocate": 1}, "true or false"),
    ({"destinations_ned": [[1, 2, 3]]}, "destinations_ned"),
])
def test_curve_block_refuses_bad_keys_and_values(curve, match):
    with pytest.raises(ValueError, match=match):
        S.build_scenario(dict(MISSION, curve=curve), AGENTS)


def test_misspelt_mission_block_is_refused():
    with pytest.raises(ValueError, match="did you mean 'curve'"):
        S.build_scenario(dict(MISSION, curv={"sweep_freq_hz": 0.1}), AGENTS)
    with pytest.raises(ValueError, match="did you mean 'planner'"):
        S.build_scenario(dict(MISSION, planer={"type": "curve"}), AGENTS)


def test_curve_mission_must_be_flyable():
    curve = dict(MISSION, planner={"type": "curve"})
    S.build_scenario(curve, AGENTS)  # the test mission is fine (60 s budget, 30 m, beta 61 m)
    with pytest.raises(ValueError, match="finite budget"):
        S.build_scenario(dict(curve, team={"max_flight_time_s": float("inf"),
                                           "max_flight_distance_m": float("inf")}), AGENTS)
    with pytest.raises(ValueError, match="single_axis_gimbal"):
        S.build_scenario(dict(curve, sensor=dict(MISSION["sensor"], single_axis_gimbal=False)), AGENTS)
    # 0.98 * 61 * cos(30 deg) = 51.8 m: robot_2 at 30 + 25 m cannot see the ground
    with pytest.raises(ValueError, match="agent 2"):
        S.build_scenario(dict(curve, curve={"altitude_stagger_m": 25.0}), AGENTS)
    # the same scenario flown by the orienteering planner is not checked
    S.build_scenario(dict(MISSION, planner={"type": "orienteering"}, curve={"altitude_stagger_m": 25.0},
                          team={"max_flight_time_s": float("inf")}), AGENTS)


def test_follower_gimbal_law():
    sc, _, _ = S.build_scenario(dict(MISSION, follower={}), AGENTS)
    assert sc["airstack"]["follower"] == {"gimbal_law": "open_loop"}
    sc2, _, _ = S.build_scenario(dict(MISSION, follower={"gimbal_law": "aim_point"}), AGENTS)
    assert sc2["airstack"]["follower"] == {"gimbal_law": "aim_point"}
    with pytest.raises(ValueError, match="gimbal_law"):
        S.build_scenario(dict(MISSION, follower={"gimbal_law": "wobble"}), AGENTS)
    with pytest.raises(ValueError, match="unknown key"):
        S.build_scenario(dict(MISSION, follower={"law": "open_loop"}), AGENTS)


def test_committed_mission_selects_orienteering_and_carries_the_curve_block():
    """The stack's mission.yaml: orienteering flown by default, the curve block ready to switch."""
    yaml = pytest.importorskip("yaml")
    path = REPO / "stacks" / "mtl_search" / "config" / "mission.yaml"
    if not path.is_file():
        pytest.skip("repo layout not available")
    m = yaml.safe_load(path.read_text(encoding="utf-8"))["mission"]
    assert m["planner"]["type"] == "orienteering"
    assert S.planner_of(m["planner"]) == {"type": "orienteering", "compare_orienteering": True}
    cv = S.curve_of(m["curve"])
    # the planned altitudes are the flown deconfliction layers
    assert cv["altitude_stagger_m"] == m["team"]["altitude_separation_m"]
    # the sweep stays within the gimbal rate the follower and the sim use (120 deg/s)
    h, tilt, beta = m["aircraft"]["altitude_m"], math.radians(m["sensor"]["tilt_deg"]), m["sensor"]["detection"]["beta"]
    alpha = math.acos(h / (0.98 * beta * math.cos(tilt)))            # the slant-range-bound amplitude
    assert math.degrees(alpha * 2 * math.pi * cv["sweep_freq_hz"]) < 0.95 * m["sim_gimbal"]["slew_rate_deg_s"]
    assert S.follower_of(m["follower"])["gimbal_law"] in S.GIMBAL_LAWS
    # and it would be flyable as a curve mission
    sc = {"team": {"max_flight_time_s": m["team"]["max_flight_time_s"],
                   "max_flight_distance_m": None, "agents": [{}, {}, {}]},
          "aircraft": m["aircraft"], "sensor": m["sensor"], "curve": cv}
    S.check_curve_feasible(sc)
