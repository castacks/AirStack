# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Unit tests for mtl_metrics_logger detection scoring and run outputs (pure Python)."""

import json
import math
import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parent.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from mtl_metrics_logger import analysis as A  # noqa: E402
from mtl_metrics_logger import report as R  # noqa: E402
from mtl_metrics_logger.detection import (DetectionModel, TeamScorer, boresight_ground_point,  # noqa: E402
                                          footprint_radius, resample_hold)

M = DetectionModel(a=1.1, b=0.1, c=61.0, beta=61.0, p_out_of_range=1e-6, threshold=0.9, dt_ref_s=0.1)
FOV = math.radians(60)
NADIR = math.pi / 2

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def test_moon_sigmoid_values():
    assert M.probability(0.0) == pytest.approx(1 / (1.1 + math.exp(-6.1)))
    assert M.probability(30.0) == pytest.approx(1 / (1.1 + math.exp(0.1 * (30 - 61))))
    assert M.probability(61.0) == pytest.approx(1 / 2.1)
    assert M.probability(61.5) == 1e-6


def test_boresight_ground_point_and_footprint():
    gx, gy, slant = boresight_ground_point((10.0, 5.0, 30.0), math.radians(45), 0.0)
    assert (gx, gy) == pytest.approx((40.0, 5.0))
    assert slant == pytest.approx(30.0 * math.sqrt(2))
    assert footprint_radius(slant, FOV) == pytest.approx(slant * math.tan(math.radians(30)))
    assert boresight_ground_point((0, 0, 30), 0.0, 0.0) is None            # horizon
    assert boresight_ground_point((0, 0, 30), -0.2, 0.0) is None           # looking up
    assert boresight_ground_point((0, 0, -1), NADIR, 0.0) is None          # below the ground


def _hover(rate_hz, seconds=1.0, targets=((0.0, 0.0, 0.0),)):
    sc = TeamScorer(targets, [], [], M, FOV)
    dt = 1.0 / rate_hz
    for k in range(int(round(seconds * rate_hz)) + 1):
        sc.step(k * dt, {"a": {"pos": (0.0, 0.0, 30.0), "pitch": NADIR, "yaw": 0.0}}, dt if k else 0.0)
    return sc


def test_rate_independence():
    p10 = _hover(10).targets[0].p_det
    p20 = _hover(20).targets[0].p_det
    p50 = _hover(50).targets[0].p_det
    assert p20 == pytest.approx(p10, abs=1e-9) and p50 == pytest.approx(p10, abs=1e-9)
    # and the reference law: 10 looks of P at dt = dt_ref
    p = M.probability(30.0)
    assert p10 == pytest.approx(1 - (1 - p) ** 10, abs=1e-12)


def test_footprint_and_range_gates():
    r = 30.0 * math.tan(FOV / 2)  # ~17.3 m at nadir from 30 m
    sc = _hover(10, targets=((r - 0.5, 0.0, 0.0), (r + 0.5, 0.0, 0.0)))
    assert sc.targets[0].observations > 0 and sc.targets[1].observations == 0
    # from 70 m the look point is beyond beta = 61 m: nothing is seen
    sc = TeamScorer([(0, 0, 0)], [], [], M, FOV)
    sc.step(0.0, {"a": {"pos": (0, 0, 70.0), "pitch": NADIR, "yaw": 0.0}}, 0.1)
    assert sc.targets[0].observations == 0


def test_team_fusion_multiplies_miss_and_names_responsible_agent():
    one = _hover(10, seconds=0.2).targets[0].p_miss
    sc = TeamScorer([(0, 0, 0)], [], [], M, FOV)
    for k in range(3):
        look = {"pos": (0.0, 0.0, 30.0), "pitch": NADIR, "yaw": 0.0}
        sc.step(k * 0.1, {"robot_1": look, "robot_2": dict(look)}, 0.1 if k else 0.0)
    assert sc.targets[0].p_miss == pytest.approx(one * one, rel=1e-12)
    assert sc.targets[0].observers == {"robot_1", "robot_2"}
    assert sc.targets[0].detected
    assert sc.targets[0].detected_by in ("robot_1", "robot_2")
    assert sc.targets[0].detection_time_s == pytest.approx(0.1)


def test_cells_coverage_and_summary():
    sc = TeamScorer([], [(0.0, 0.0), (100.0, 0.0)], [3.0, 7.0], M, FOV)
    sc.step(0.0, {"a": {"pos": (0.0, 0.0, 30.0), "pitch": NADIR, "yaw": 0.0}}, 0.0)
    sc.step(0.1, {"a": {"pos": (1.0, 0.0, 30.0), "pitch": NADIR, "yaw": 0.0}}, 0.1)
    s = sc.summary()
    assert s["cells_covered"] == 1 and s["belief_mass_covered"] == 3.0
    assert s["total_path_length_m"] == pytest.approx(1.0)
    assert s["belief_mass_per_km"] == pytest.approx(3000.0)


def test_resample_hold():
    assert resample_hold([0.0, 1.0, 2.0], ["a", "b", "c"], [-1.0, 0.5, 1.0, 5.0]) == ["a", "a", "b", "c"]


SCEN = {
    "mission": {"name": "t", "area": {"size_m": 100.0, "center_ned": [0.0, 0.0], "belief_res_m": 2.0}},
    "mapping": {"target_cell_size_m": 20.0},
    "sensor": {"fov_deg": 60.0, "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0,
                                              "p_out_of_range": 1e-6, "threshold": 0.9, "dt_ref_s": 0.1}},
    "cells": {"centers": [[0.0, 0.0], [0.0, 40.0]], "mass": [5.0, 5.0]},
}
GT = {"targets": [{"index": 0, "n": 0.0, "e": 2.0}, {"index": 1, "n": 45.0, "e": 45.0}]}


def _rows(agent, x0, n=40, measured=True):
    rows = []
    for k in range(n):
        x = x0 + 0.5 * k
        rows.append({"t": 0.05 * k, "x_world": x, "y_world": 0.0, "z_world": 30.0,
                     "meas_pitch": NADIR if measured else None, "meas_yaw": 0.0 if measured else None,
                     "gimbal_measured": int(measured), "cmd_pitch": NADIR, "cmd_yaw": 0.0,
                     "xte_m": 0.1, "pointing_error_m": 0.2, "bore_x_world": x, "bore_y_world": 0.0})
    return rows


def test_write_run_outputs_team(tmp_path):
    res = A.write_run_outputs(
        tmp_path, scenario=SCEN, ground_truth=GT,
        rows_by_agent={"robot_1": _rows("robot_1", -10.0), "robot_2": _rows("robot_2", 0.0, measured=False)},
        planned_by_agent={"robot_1": {"planned": [[-10, 0], [10, 0]], "home": [-10, 0], "serviced_cells": [0, 1]},
                          "robot_2": {"planned": [[0, 0], [20, 0]], "home": [0, 0], "serviced_cells": [0]}},
        title="t", subtitle="s", belief_png=b"\x89PNG fake")
    for name in ("telemetry.csv", "detection.json", "report.html"):
        assert (tmp_path / name).is_file()
    det = json.loads((tmp_path / "detection.json").read_text())
    s = det["summary"]
    assert s["targets_detected"] == 1 and s["targets_total"] == 2
    assert det["targets"][0]["detected"] and det["targets"][0]["responsible_agent"] in ("robot_1", "robot_2")
    assert s["gimbal_measured_fraction"] == {"robot_1": 1.0, "robot_2": 0.0}
    assert s["planned_belief_mass"] == 10.0 and s["belief_mass_covered"] == 5.0
    assert s["realized_over_planned_mass"] == 0.5
    rows = R.read_telemetry_csv(tmp_path / "telemetry.csv")
    assert len(rows) == 80 and {r["agent"] for r in rows} == {"robot_1", "robot_2"}
    html = (tmp_path / "report.html").read_text()
    assert '<script id="payload" type="application/json">' in html and "</main>" in html
    payload = html.split('type="application/json">', 1)[1].split("</script>", 1)[0]
    data = json.loads(payload)
    assert data["summary"]["targets_detected"] == 1 and len(data["agents"]) == 2
