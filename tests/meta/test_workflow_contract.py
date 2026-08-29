"""Contracts for trustworthy GitHub Actions result identity and policy."""

import pytest

from harness.discovery import repo_path

pytestmark = pytest.mark.unit


def _workflow() -> str:
    return repo_path(".github", "workflows", "system-tests.yml").read_text()


def test_comment_head_resolution_never_falls_back_to_default_branch():
    workflow = _workflow()
    assert "${COMMENT_HEAD_SHA:-$EVENT_SHA}" not in workflow
    assert "PR head SHA was not resolved" in workflow
    assert 'echo "tested_sha=$COMMENT_HEAD_SHA"' in workflow


def test_comment_runs_cancel_older_run_for_same_pr():
    workflow = _workflow()
    assert "github.event.issue.number || github.run_id" in workflow
    assert "cancel-in-progress: true" in workflow


def test_report_job_installs_declared_dependencies():
    # parse_metrics.py imports the tests/harness package, so the report job
    # must install the full test requirements — a bare `pip install tabulate`
    # crashed every report with ModuleNotFoundError: yaml (issue behind #407).
    report = _workflow().split("\n  report:", 1)[1]
    assert "pip install -r tests/requirements.txt" in report
    assert "pip install tabulate" not in report
    requirements = repo_path("tests", "requirements.txt").read_text().lower()
    assert "pyyaml" in requirements
    assert "tabulate" in requirements


def test_metric_deltas_are_advisory_but_parser_errors_block():
    workflow = _workflow()
    assert "- name: Fail on report integrity error" in workflow
    assert "Metric regression detected" not in workflow
    assert "parser_exit=2" in workflow


def test_tested_identity_is_written_into_campaign_metadata():
    system = _workflow()
    unit = repo_path(".github", "workflows", "unit-tests.yml").read_text()
    assert "AIRSTACK_TESTED_SHA: ${{ steps.identity.outputs.tested_sha }}" in system
    assert "AIRSTACK_PR_NUMBER: ${{ steps.identity.outputs.pr_number }}" in system
    assert "AIRSTACK_TESTED_SHA: ${{ github.sha }}" in unit


def test_baseline_search_downloads_candidates_then_selects_by_fingerprint():
    workflow = _workflow()
    assert "-f per_page=20" in workflow
    assert 'gh run download "$run_id"' in workflow
    assert "select_baseline_path" in workflow
    assert "dawidd6/action-download-artifact" not in workflow


def test_manual_campaign_can_select_minimal_algorithm_sweeps():
    workflow = _workflow()
    assert "trajectory_types:" in workflow
    assert "takeoff_velocities:" in workflow
    assert "args.extend(['--trajectory-types', trajectories])" in workflow
    assert "args.extend(['--takeoff-velocities', velocities])" in workflow
