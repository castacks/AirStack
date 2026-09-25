# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Node-level test of mtl_metrics_logger against hermetic ROS stubs."""

import importlib
import json
import math
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
for p in (_HERE, _HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import _ros_stubs as S  # noqa: E402

SCEN = {
    "schema": "mtl.scenario/1",
    "mission": {"name": "t", "area": {"size_m": 200.0, "center_ned": [0.0, 0.0], "belief_res_m": 2.0}},
    "mapping": {"target_cell_size_m": 20.0},
    "sensor": {"fov_deg": 60.0, "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0,
                                              "p_out_of_range": 1e-6, "threshold": 0.9, "dt_ref_s": 0.1}},
    "team": {"agents": [{"name": "robot_1", "start_ned": [-50.0, -60.0], "home_ned": [-50.0, -60.0]}]},
    "cells": {"centers": [[-50.0, -30.0]], "mass": [4.0]},
}
# target at world (e=-30, n=-50): 30 m east of the robot's home
GT = {"schema": "mtl.ground_truth/1", "targets": [{"index": 0, "n": -50.0, "e": -30.0},
                                                   {"index": 1, "n": 80.0, "e": 80.0}]}

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


@pytest.fixture
def node(monkeypatch, tmp_path):
    S.install(monkeypatch)
    monkeypatch.delitem(sys.modules, "mtl_metrics_logger.logger_node", raising=False)
    mod = importlib.import_module("mtl_metrics_logger.logger_node")
    (tmp_path / "scenario.json").write_text(json.dumps(SCEN))
    (tmp_path / "ground_truth.json").write_text(json.dumps(GT))
    S.Node.overrides = {"scenario_file": str(tmp_path / "scenario.json"),
                        "runs_root": str(tmp_path / "runs"), "agent_name": "robot_1"}
    return mod.MtlMetricsLogger(), tmp_path


def status(state, name, progress=0.0):
    st = S.Msg(plan_id="t/robot_1/run42", state=state, state_name=name, progress_m=progress,
               remaining_m=60.0 - progress, cross_track_error_m=0.3, total_m=60.0)
    st.carrot = S.Vector3(x=progress + 14.4, y=0.0, z=30.0)
    st.aim_point = S.Vector3(x=progress, y=0.0, z=0.0)
    return st


def test_records_scores_and_writes_outputs(node):
    n, tmp = node
    plan = S.Msg(plan_id="t/robot_1/run42", run_id="run42", start_mission=True, planned_length_m=60.0)
    plan.map_origin_in_world = S.Vector3(x=-60.0, y=-50.0, z=0.0)   # home ENU
    plan.trajectory.waypoints = [S.Msg(position=S.Vector3(x=float(k), y=0.0, z=30.0)) for k in range(61)]
    plan.serviced_cells = [0]
    n.subs["search/plan"](plan)
    tick = n.timers[0][1]
    for k in range(121):        # 60 m at 10 m/s-equivalent, 20 Hz
        x = 0.5 * k
        n.subs["odometry"](S.odom(x, 0.0, 30.0))
        n.subs["gimbal/cmd_pitch_yaw"](S.Vector3(x=0.0, y=math.pi / 2, z=0.0))
        n.subs["gimbal/state"](S.Vector3(x=0.0, y=math.pi / 2, z=0.0))
        n.subs["search/follower_status"](status(2, "SEARCH", x))
        S.SimClock.advance(0.05)
        tick()
    n.subs["search/follower_status"](status(3, "COMPLETE", 60.0))
    S.SimClock.advance(0.05)
    tick()
    out = tmp / "runs" / "run42" / "robot_1"
    det = json.loads((out / "detection.json").read_text())
    assert det["summary"]["targets_detected"] == 1       # the one it flew over (map x=30)
    assert det["targets"][0]["responsible_agent"] == "robot_1"
    assert det["summary"]["gimbal_measured_fraction"]["robot_1"] == 1.0
    assert det["summary"]["belief_mass_covered"] == 4.0
    assert (out / "report.html").stat().st_size > 5000
    csv = (out / "telemetry.csv").read_text().splitlines()
    assert csv[0].startswith("t,agent,state") and len(csv) == 1 + 122
    assert (tmp / "runs" / "run42" / "ground_truth.json").is_file()
    # markers: target 0 turned green
    m = n.pubs["search/detection_markers"].msgs[-1].markers[0]
    assert m.color.g > 0.6 and m.color.r < 0.1
    # footprint published in the map frame around the boresight point
    fp = n.pubs["search/footprint"].msgs[-1]
    assert len(fp.polygon.points) == 24


def test_ignores_preview_plans(node):
    n, tmp = node
    plan = S.Msg(plan_id="t/robot_1/preview", run_id="", start_mission=False)
    plan.map_origin_in_world = S.Vector3()
    n.subs["search/plan"](plan)
    n.subs["odometry"](S.odom(0, 0, 30))
    n.subs["search/follower_status"](S.Msg(plan_id="t/robot_1/preview", state=2, state_name="SEARCH"))
    n.timers[0][1]()
    assert n.rows == []
