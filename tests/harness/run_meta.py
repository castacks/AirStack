"""Run-level outcome metadata for honest CI and metrics reporting."""

from __future__ import annotations

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from harness.test_ids import canonical_test_id, normalize_csv


RUN_META_FILENAME = "run_meta.json"

SIMULATION_MODULES = (
    "system.test_liveliness.",
    "system.test_sensors.",
    "system.test_takeoff_hover_land.",
    "system.test_fixed_trajectory.",
    "system.test_waypoint_flight.",
    "system.test_optitrack_e2e.",
)


def is_simulation_test_id(test_id: str) -> bool:
    """Whether a test belongs to a GPU/simulation campaign."""
    canonical = canonical_test_id(test_id)
    return canonical.startswith(SIMULATION_MODULES)


CAMPAIGN_OPTION_KEYS = (
    "sim",
    "num_robots",
    "stress_iterations",
    "stable_duration",
    "stable_interval",
    "gui",
    "takeoff_velocities",
    "trajectory_types",
    "waypoints",
    "waypoint_tolerance",
    "goal_tolerance",
    "waypoint_timeout",
)


def normalize_campaign_config(raw: dict | None) -> dict:
    """Return stable, JSON-safe behavior-changing campaign configuration."""
    raw = raw or {}
    result = {}
    for key in CAMPAIGN_OPTION_KEYS:
        value = raw.get(key)
        if key in ("sim", "num_robots", "takeoff_velocities", "trajectory_types"):
            cast = int if key == "num_robots" else str
            result[key] = normalize_csv(value, cast=cast)
        elif isinstance(value, Path):
            result[key] = str(value)
        elif value is not None:
            result[key] = value
    return result


def campaign_fingerprint(test_ids, campaign_config: dict | None = None) -> str:
    """Stable identity for exact tests plus behavior-changing configuration."""
    canonical_ids = sorted(
        canonical_test_id(str(test_id).replace("::", ".")).replace(".py.", ".")
        for test_id in test_ids
    )
    if not canonical_ids:
        return ""
    payload = json.dumps(
        {
            "test_ids": canonical_ids,
            "config": normalize_campaign_config(campaign_config),
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def _item_outcome(item) -> str | None:
    """Return the final outcome recorded on a pytest item."""
    reports = [
        getattr(item, "_rep_setup", None),
        getattr(item, "_rep_call", None),
        getattr(item, "_rep_teardown", None),
    ]
    if any(rep is not None and rep.failed for rep in reports):
        return "failed"
    if any(rep is not None and rep.skipped for rep in reports):
        return "skipped"
    call = getattr(item, "_rep_call", None)
    if call is not None and call.passed:
        return "passed"
    return None


def _report_details(reports) -> tuple[dict[str, str], set[str], set[str]]:
    """Collapse phase reports and identify items that reached call phase."""
    priority = {"passed": 0, "skipped": 1, "failed": 2}
    outcomes = {}
    call_nodeids = set()
    infrastructure_error_nodeids = set()
    for report in reports or []:
        nodeid = getattr(report, "nodeid", None)
        when = getattr(report, "when", None)
        if not nodeid or when not in ("setup", "call", "teardown"):
            continue
        if report.failed:
            outcome = "failed"
            if when != "call":
                infrastructure_error_nodeids.add(nodeid)
        elif report.skipped:
            outcome = "skipped"
        elif when == "call" and report.passed:
            outcome = "passed"
        else:
            continue
        if when == "call":
            call_nodeids.add(nodeid)
        previous = outcomes.get(nodeid, "passed")
        outcomes[nodeid] = max((previous, outcome), key=priority.get)
    return outcomes, call_nodeids, infrastructure_error_nodeids


def build_run_meta(items, exitstatus: int, mark_expression: str = "",
                   reports=None, campaign_config: dict | None = None,
                   tested_identity: dict | None = None) -> dict:
    """Build serializable run metadata from a completed pytest session."""
    report_outcomes, call_nodeids, infrastructure_error_nodeids = _report_details(
        reports
    )
    if report_outcomes:
        completed_by_id = report_outcomes
    else:
        completed_by_id = {
            str(item.nodeid): outcome
            for item in items
            if (outcome := _item_outcome(item)) is not None
        }
        call_nodeids = {
            str(item.nodeid)
            for item in items
            if getattr(item, "_rep_call", None) is not None
        }
        infrastructure_error_nodeids = {
            str(item.nodeid)
            for item in items
            if any(
                report is not None and report.failed
                for report in (
                    getattr(item, "_rep_setup", None),
                    getattr(item, "_rep_teardown", None),
                )
            )
        }
    completed = list(completed_by_id.values())
    selected_test_ids = sorted(canonical_test_id(item.nodeid) for item in items)
    simulation_items = [
        item for item in items if is_simulation_test_id(str(item.nodeid))
    ]
    simulation_completed = [
        item for item in simulation_items if str(item.nodeid) in call_nodeids
    ]
    simulation_finalized = [
        item for item in simulation_items
        if str(item.nodeid) in completed_by_id
    ]
    simulation_infrastructure_errors = [
        item
        for item in simulation_items
        if str(item.nodeid) in infrastructure_error_nodeids
    ]

    call_failures = [
        nodeid for nodeid, status in completed_by_id.items()
        if status == "failed" and nodeid in call_nodeids
    ]
    if exitstatus == 2:
        # Pytest uses exit 2 for both collection aborts and user/runner
        # interruption. Reports prove that execution had already begun.
        outcome = "incomplete" if completed else "collection_error"
        failure_class = "interrupted" if completed else "collection"
    elif exitstatus in (3, 4):
        outcome = "internal_error"
        failure_class = "ci_integrity"
    elif exitstatus == 5 or not items:
        outcome = "no_tests"
        failure_class = "no_tests"
    elif not call_nodeids:
        outcome = (
            "simulation_not_executed" if simulation_items
            else "tests_not_executed"
        )
        failure_class = "infrastructure"
    elif simulation_items and not simulation_completed:
        outcome = "simulation_not_executed"
        failure_class = "infrastructure"
    elif simulation_infrastructure_errors:
        outcome = "incomplete"
        failure_class = "infrastructure"
    elif len(simulation_finalized) != len(simulation_items):
        outcome = "incomplete"
        failure_class = "infrastructure"
    elif simulation_items:
        outcome = "simulation"
        failure_class = "assertion" if call_failures else "none"
    else:
        outcome = "non_simulation"
        failure_class = "assertion" if call_failures else "none"

    normalized_config = normalize_campaign_config(campaign_config)
    simulation_ids = [
        canonical_test_id(item.nodeid)
        for item in simulation_items
    ]
    fingerprint = campaign_fingerprint(simulation_ids, normalized_config)
    complete = outcome in ("simulation", "non_simulation")
    return {
        "schema_version": 2,
        "complete": complete,
        "completion_state": "completed" if complete else outcome,
        "failure_class": failure_class,
        "outcome": outcome,
        "pytest_exitstatus": int(exitstatus),
        "mark_expression": mark_expression,
        "selected_tests": len(items),
        "completed_tests": len(completed),
        "passed": completed.count("passed"),
        "failed": completed.count("failed"),
        "skipped": completed.count("skipped"),
        "simulation_selected": len(simulation_items),
        "simulation_completed": len(simulation_completed),
        "simulation_finalized": len(simulation_finalized),
        "selected_test_ids": selected_test_ids,
        "campaign_config": normalized_config,
        "campaign_fingerprint": fingerprint,
        "campaign": {
            "schema_version": 1,
            "selected_test_ids": sorted(simulation_ids),
            "config": normalized_config,
            "fingerprint": fingerprint,
        },
        "tested_identity": tested_identity or {
            "sha": os.environ.get("AIRSTACK_TESTED_SHA", ""),
            "pr_number": os.environ.get("AIRSTACK_PR_NUMBER", ""),
        },
    }


def write_run_meta(run_dir: Path, items, exitstatus: int,
                   mark_expression: str = "", reports=None,
                   campaign_config: dict | None = None,
                   tested_identity: dict | None = None) -> Path:
    """Write ``run_meta.json`` for a normally completed pytest session."""
    path = Path(run_dir) / RUN_META_FILENAME
    path.write_text(json.dumps(
        build_run_meta(
            items,
            exitstatus,
            mark_expression,
            reports,
            campaign_config,
            tested_identity,
        ),
        indent=2,
        sort_keys=True,
    ) + "\n")
    return path


def _classify_junit(results_xml: Path) -> dict:
    """Infer legacy run state when ``run_meta.json`` is unavailable."""
    cases = list(ET.parse(results_xml).iter("testcase"))
    errors = sum(tc.find("error") is not None for tc in cases)
    failures = sum(tc.find("failure") is not None for tc in cases)
    skipped = sum(tc.find("skipped") is not None for tc in cases)
    simulation = sum(
        is_simulation_test_id(f"{tc.get('classname')}.{tc.get('name')}")
        for tc in cases
    )
    simulation_ids = [
        f"{tc.get('classname')}.{tc.get('name')}"
        for tc in cases
        if is_simulation_test_id(f"{tc.get('classname')}.{tc.get('name')}")
    ]
    simulation_errors = sum(
        tc.find("error") is not None
        for tc in cases
        if is_simulation_test_id(f"{tc.get('classname')}.{tc.get('name')}")
    )

    if errors:
        outcome = "incomplete" if simulation else "collection_error"
    elif not cases:
        outcome = "no_tests"
    elif simulation:
        outcome = "simulation"
    else:
        outcome = "non_simulation"

    return {
        "schema_version": 1,
        "complete": outcome in ("simulation", "non_simulation"),
        "completion_state": (
            "completed" if outcome in ("simulation", "non_simulation") else outcome
        ),
        "failure_class": (
            "infrastructure" if outcome == "incomplete"
            else "collection" if outcome == "collection_error"
            else "no_tests" if outcome == "no_tests"
            else "assertion" if failures else "none"
        ),
        "outcome": outcome,
        "pytest_exitstatus": None,
        "mark_expression": "",
        "selected_tests": len(cases),
        "completed_tests": len(cases),
        "passed": len(cases) - errors - failures - skipped,
        "failed": errors + failures,
        "skipped": skipped,
        "simulation_selected": simulation,
        "simulation_completed": simulation - simulation_errors,
        "campaign_fingerprint": campaign_fingerprint(simulation_ids),
        "inferred": True,
    }


def classify_run(run_dir: Path) -> dict:
    """Read run metadata or infer whether an artifact is comparable."""
    run_dir = Path(run_dir)
    meta_path = run_dir / RUN_META_FILENAME
    if meta_path.exists():
        try:
            meta = json.loads(meta_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            return {
                "schema_version": 1,
                "complete": False,
                "outcome": "incomplete",
                "reason": f"Run metadata could not be read: {exc}",
            }
        if meta.get("outcome") in ("simulation", "non_simulation"):
            results_xml = run_dir / "results.xml"
            if not results_xml.exists():
                return {
                    "schema_version": 1,
                    "complete": False,
                    "outcome": "incomplete",
                    "reason": "Run metadata exists but JUnit results are missing.",
                }
            try:
                ET.parse(results_xml)
            except (OSError, ET.ParseError) as exc:
                return {
                    "schema_version": 1,
                    "complete": False,
                    "outcome": "incomplete",
                    "reason": f"JUnit results were not finalized: {exc}",
                }
        return meta

    results_xml = run_dir / "results.xml"
    if results_xml.exists():
        try:
            return _classify_junit(results_xml)
        except (OSError, ET.ParseError) as exc:
            return {
                "schema_version": 1,
                "complete": False,
                "outcome": "incomplete",
                "reason": f"JUnit results were not finalized: {exc}",
            }

    if (run_dir / "metrics.json").exists():
        return {
            "schema_version": 1,
            "complete": False,
            "outcome": "incomplete",
            "reason": "Metrics exist but pytest did not finalize JUnit/run metadata.",
        }

    return {
        "schema_version": 1,
        "complete": False,
        "outcome": "missing_results",
        "reason": "No pytest result artifact was produced.",
    }


def simulation_metrics_comparable(meta: dict, baseline: dict | None = None) -> bool:
    """Whether a complete simulation may be compared with a like campaign."""
    valid = bool(
        meta
        and meta.get("complete")
        and meta.get("outcome") == "simulation"
        and meta.get("campaign_fingerprint")
    )
    if not valid or baseline is None:
        return valid
    return bool(
        baseline.get("complete")
        and baseline.get("outcome") == "simulation"
        and baseline.get("campaign_fingerprint") == meta["campaign_fingerprint"]
    )


def comparability_reason(meta: dict, baseline: dict | None = None) -> str:
    """Human explanation shared by summaries, reports, and baseline selection."""
    if not meta:
        return "run metadata is missing"
    if not meta.get("complete"):
        return f"campaign is not complete ({meta.get('completion_state', meta.get('outcome'))})"
    if meta.get("outcome") != "simulation":
        return f"run outcome is {meta.get('outcome', 'unknown')}, not a simulation campaign"
    if not meta.get("campaign_fingerprint"):
        return "campaign fingerprint is missing"
    if baseline is None:
        return "no baseline campaign was supplied"
    if not baseline.get("complete") or baseline.get("outcome") != "simulation":
        return "baseline is not a completed simulation campaign"
    if baseline.get("campaign_fingerprint") != meta.get("campaign_fingerprint"):
        return "baseline campaign configuration does not match"
    return ""
