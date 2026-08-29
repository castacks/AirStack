# Copyright (c) 2024 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contracts that keep infrastructure failures out of simulation metrics."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from harness.run_meta import (
    build_run_meta,
    campaign_fingerprint,
    classify_run,
    simulation_metrics_comparable,
)
from parse_metrics import generate_report, merge_metrics


pytestmark = pytest.mark.unit


def _write_junit(run_dir: Path, testcase: str) -> None:
    run_dir.mkdir()
    (run_dir / "results.xml").write_text(
        '<?xml version="1.0" encoding="utf-8"?>'
        f'<testsuites><testsuite tests="1">{testcase}</testsuite></testsuites>'
    )


def _item(nodeid: str, *, failed=False, skipped=False):
    report = SimpleNamespace(
        failed=failed,
        skipped=skipped,
        passed=not failed and not skipped,
    )
    return SimpleNamespace(
        nodeid=nodeid,
        _rep_setup=SimpleNamespace(failed=False, skipped=False, passed=True),
        _rep_call=report,
        _rep_teardown=SimpleNamespace(failed=False, skipped=False, passed=True),
    )


def _setup_failed_item(nodeid: str):
    return SimpleNamespace(
        nodeid=nodeid,
        _rep_setup=SimpleNamespace(failed=True, skipped=False, passed=False),
        _rep_call=None,
        _rep_teardown=None,
    )


def _phase_report(nodeid: str, when: str, outcome: str):
    return SimpleNamespace(
        nodeid=nodeid,
        when=when,
        failed=outcome == "failed",
        skipped=outcome == "skipped",
        passed=outcome == "passed",
    )


def test_completed_simulation_failure_is_a_valid_campaign():
    meta = build_run_meta(
        [_item(
            "system/test_fixed_trajectory.py::TestFixedTrajectory::test_circle",
            failed=True,
        )],
        exitstatus=1,
        mark_expression="autonomy",
    )
    assert meta["outcome"] == "simulation"
    assert meta["simulation_completed"] == 1
    assert meta["failed"] == 1


def test_collection_exit_is_not_simulation_performance():
    meta = build_run_meta([], exitstatus=2, mark_expression="optitrack")
    assert meta["outcome"] == "collection_error"
    assert meta["simulation_completed"] == 0


def test_interrupted_partial_simulation_is_not_comparable():
    meta = build_run_meta(
        [_item(
            "system/test_fixed_trajectory.py::TestFixedTrajectory::test_takeoff",
        )],
        exitstatus=2,
        mark_expression="autonomy",
    )
    assert meta["outcome"] == "incomplete"
    assert meta["complete"] is False


def test_setup_only_failure_is_not_policy_performance():
    nodeid = (
        "system/test_fixed_trajectory.py::TestFixedTrajectory::test_takeoff"
    )
    meta = build_run_meta(
        [_setup_failed_item(nodeid)],
        exitstatus=1,
        mark_expression="autonomy",
        reports=[_phase_report(nodeid, "setup", "failed")],
    )
    assert meta["outcome"] == "simulation_not_executed"
    assert meta["simulation_completed"] == 0


def test_fail_fast_partial_campaign_is_not_comparable():
    first = (
        "system/test_fixed_trajectory.py::TestFixedTrajectory::test_takeoff"
    )
    meta = build_run_meta(
        [
            _item(first, failed=True),
            SimpleNamespace(
                nodeid=(
                    "system/test_fixed_trajectory.py::"
                    "TestFixedTrajectory::test_circle"
                ),
                _rep_setup=None,
                _rep_call=None,
                _rep_teardown=None,
            ),
        ],
        exitstatus=1,
        mark_expression="autonomy",
        reports=[_phase_report(first, "call", "failed")],
    )
    assert meta["outcome"] == "incomplete"
    assert meta["simulation_completed"] == 1
    assert meta["simulation_selected"] == 2


def test_teardown_error_makes_campaign_incomplete():
    nodeid = (
        "system/test_fixed_trajectory.py::TestFixedTrajectory::test_circle"
    )
    meta = build_run_meta(
        [_item(nodeid)],
        exitstatus=1,
        mark_expression="autonomy",
        reports=[
            _phase_report(nodeid, "call", "passed"),
            _phase_report(nodeid, "teardown", "failed"),
        ],
    )
    assert meta["outcome"] == "incomplete"
    assert meta["complete"] is False


def test_only_identical_campaigns_are_comparable():
    current = build_run_meta(
        [_item(
            "system/test_fixed_trajectory.py::TestFixedTrajectory::test_circle[a]",
        )],
        exitstatus=0,
        mark_expression="autonomy",
    )
    same = dict(current)
    different = dict(current, campaign_fingerprint="different")
    assert simulation_metrics_comparable(current, same)
    assert not simulation_metrics_comparable(current, different)


def test_campaign_fingerprint_matches_pytest_and_junit_ids():
    pytest_id = (
        "system/test_fixed_trajectory.py::TestFixedTrajectory::test_circle[a]"
    )
    junit_id = (
        "system.test_fixed_trajectory.TestFixedTrajectory.test_circle[a]"
    )
    assert campaign_fingerprint([pytest_id]) == campaign_fingerprint([junit_id])


def test_collection_error_report_suppresses_pass_rates(tmp_path):
    run_dir = tmp_path / "collection-error"
    _write_junit(
        run_dir,
        '<testcase classname="robot.ros_ws.src.foo.test.test_node" '
        'name="collection_error" time="0"><error message="No module named rclpy"/>'
        "</testcase>",
    )

    assert classify_run(run_dir)["outcome"] == "collection_error"
    markdown, regressed = generate_report(run_dir)
    assert "Simulation metrics are not comparable" in markdown
    assert "Pass rates" not in markdown
    assert regressed is False


def test_sim_setup_error_report_is_not_policy_performance(tmp_path):
    run_dir = tmp_path / "setup-error"
    _write_junit(
        run_dir,
        '<testcase classname="system.test_fixed_trajectory.TestFixedTrajectory" '
        'name="test_takeoff[isaacsim]" time="1">'
        '<error message="airstack up failed"/></testcase>',
    )

    markdown, regressed = generate_report(run_dir)
    assert "Simulation metrics are not comparable" in markdown
    assert "Pass rates" not in markdown
    assert regressed is False


def test_incomplete_artifact_is_not_simulation_performance(tmp_path):
    run_dir = tmp_path / "incomplete"
    run_dir.mkdir()
    (run_dir / "metrics.json").write_text("{}")

    markdown, regressed = generate_report(run_dir)
    assert "timeout or cancellation" in markdown
    assert "Pass rates" not in markdown
    assert regressed is False


def test_truncated_junit_is_not_simulation_performance(tmp_path):
    run_dir = tmp_path / "truncated"
    run_dir.mkdir()
    (run_dir / "results.xml").write_text("<testsuites><testsuite>")

    markdown, regressed = generate_report(run_dir)
    assert "Simulation metrics are not comparable" in markdown
    assert "Pass rates" not in markdown
    assert regressed is False


def test_real_sim_failure_keeps_metrics_and_pass_rate(tmp_path):
    run_dir = tmp_path / "sim-failure"
    _write_junit(
        run_dir,
        '<testcase classname="system.test_fixed_trajectory.TestFixedTrajectory" '
        'name="test_circle[isaacsim-iter1]" time="3.5">'
        '<failure message="cross-track tolerance exceeded"/></testcase>',
    )
    (run_dir / "metrics.json").write_text(json.dumps({
        "system/test_fixed_trajectory.TestFixedTrajectory."
        "test_circle[isaacsim-iter1]": {
            "cross_track_error_mean_m": {
                "value": 4.2,
                "unit": "m",
                "direction": "lower_is_better",
            },
        },
    }))

    merged = merge_metrics(run_dir)
    assert list(merged) == [
        "system.test_fixed_trajectory.TestFixedTrajectory."
        "test_circle[isaacsim]"
    ]
    only_metrics = next(iter(merged.values()))
    assert only_metrics["status"] == "failed"
    assert only_metrics["cross_track_error_mean_m"]["value"] == 4.2

    markdown, regressed = generate_report(run_dir)
    assert "### Pass rates" in markdown
    assert "cross_track_error_mean_m" in markdown
    assert "0%" in markdown
    assert "not comparable" not in markdown
    assert regressed is False
