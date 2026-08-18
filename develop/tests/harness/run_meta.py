"""Run-level outcome metadata for honest CI and metrics reporting."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from harness.test_ids import canonical_test_id


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


def campaign_fingerprint(test_ids) -> str:
    """Stable identity for the exact selected simulation campaign."""
    canonical_ids = sorted(
        canonical_test_id(str(test_id).replace("::", ".")).replace(".py.", ".")
        for test_id in test_ids
    )
    if not canonical_ids:
        return ""
    payload = "\n".join(canonical_ids).encode()
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
                   reports=None) -> dict:
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
    simulation_items = [
        item for item in items if is_simulation_test_id(str(item.nodeid))
    ]
    simulation_completed = [
        item for item in simulation_items if str(item.nodeid) in call_nodeids
    ]
    simulation_infrastructure_errors = [
        item
        for item in simulation_items
        if str(item.nodeid) in infrastructure_error_nodeids
    ]

    if exitstatus == 2:
        # Pytest uses exit 2 for both collection aborts and user/runner
        # interruption. Reports prove that execution had already begun.
        outcome = "incomplete" if completed else "collection_error"
    elif exitstatus in (3, 4):
        outcome = "internal_error"
    elif exitstatus == 5 or not items:
        outcome = "no_tests"
    elif not call_nodeids:
        outcome = (
            "simulation_not_executed" if simulation_items
            else "tests_not_executed"
        )
    elif simulation_items and not simulation_completed:
        outcome = "simulation_not_executed"
    elif simulation_infrastructure_errors:
        outcome = "incomplete"
    elif len(simulation_completed) != len(simulation_items):
        outcome = "incomplete"
    elif simulation_items:
        outcome = "simulation"
    else:
        outcome = "non_simulation"

    return {
        "schema_version": 1,
        "complete": outcome != "incomplete",
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
        "campaign_fingerprint": campaign_fingerprint(
            item.nodeid for item in simulation_items
        ),
    }


def write_run_meta(run_dir: Path, items, exitstatus: int,
                   mark_expression: str = "", reports=None) -> Path:
    """Write ``run_meta.json`` for a normally completed pytest session."""
    path = Path(run_dir) / RUN_META_FILENAME
    path.write_text(json.dumps(
        build_run_meta(items, exitstatus, mark_expression, reports),
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
        "complete": outcome != "incomplete",
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
