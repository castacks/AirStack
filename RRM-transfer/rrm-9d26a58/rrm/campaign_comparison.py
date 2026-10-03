"""Read-only paired measurement over verified campaigns of the same implementation."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from statistics import median

from .acceptance_campaign import ROOT, digest, verify_campaign
from .benchmark_evidence import _sha256

SCHEMA = "rrm-core-comparison/v1"
GOALS = ("VERIFIED_GOAL", "GOAL_NOT_VERIFIED", "UNKNOWN")


class ComparisonError(ValueError):
    """The inputs cannot support the requested paired report."""


def _read_bound(directory, name, manifest):
    path = directory / name
    data = path.read_bytes()
    expected = manifest["artifacts"][name]
    if digest(expected) != digest({"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}):
        raise ComparisonError("input_changed:" + name)
    return data


def _load_campaign(directory):
    """Verify, then consume only bytes bound to that exact verified manifest."""
    before = (directory / "manifest.json").read_bytes()
    verification = verify_campaign(directory)
    if not verification["valid"]:
        raise ComparisonError("campaign_invalid:" + ",".join(verification["findings"]))
    manifest = json.loads(before)
    if (directory / "manifest.json").read_bytes() != before:
        raise ComparisonError("input_manifest_changed")
    spec = json.loads(_read_bound(directory, "campaign.json", manifest))["spec"]
    summary = json.loads(_read_bound(directory, "summary.json", manifest))
    if summary["source_unchanged"] is not True or digest(manifest["source"]) != digest(manifest["final_source"]):
        raise ComparisonError("campaign_source_drift")
    run_ids = []
    for attempt in spec["attempts"]:
        name = "attempts/" + attempt["attempt_id"] + "/events.jsonl"
        starts = []
        if name in manifest["artifacts"]:
            for line in _read_bound(directory, name, manifest).splitlines():
                try:
                    event = json.loads(line)
                except (ValueError, UnicodeError):
                    continue  # Invalid trace remains an unqualified attempted outcome.
                if isinstance(event, dict) and event.get("kind") == "run_start":
                    starts.append(event.get("run_id"))
        if len(starts) > 1 or any(not isinstance(run_id, str) or not run_id.strip() for run_id in starts):
            raise ComparisonError("invalid_run_identity:" + attempt["attempt_id"])
        run_ids.append(starts[0] if starts else None)
    observed = [run_id for run_id in run_ids if run_id is not None]
    if len(observed) != len(set(observed)):
        raise ComparisonError("reused_run_identity")
    if (directory / "manifest.json").read_bytes() != before:
        raise ComparisonError("input_manifest_changed")
    return {"manifest": manifest, "manifest_sha256": hashlib.sha256(before).hexdigest(),
            "spec": spec, "summary": summary, "run_ids": run_ids}


def distribution(values):
    """Median and nearest-rank p95; an empty sample has null statistics."""
    if any(type(value) not in {int, float} or not math.isfinite(value) for value in values):
        raise ComparisonError("invalid_latency_sample")
    ordered = sorted(values)
    return {"count": len(ordered), "median": median(ordered) if ordered else None,
            "p95": ordered[math.ceil(0.95 * len(ordered)) - 1] if ordered else None,
            "max": ordered[-1] if ordered else None}


def _goal(value):
    if value is True:
        return GOALS[0]
    if value is False:
        return GOALS[1]
    if value is None:
        return GOALS[2]
    raise ComparisonError("invalid_goal_value")


def paired_measurements(reference, candidate):
    """Compute every scheduled pair; callers must supply independently verified data."""
    if not reference or len(reference) != len(candidate):
        raise ComparisonError("attempt_schedule_mismatch")
    matrix = {left: {right: 0 for right in GOALS} for left in GOALS}
    pairs = []
    for left, right in zip(reference, candidate):
        identity = {key: left[key] for key in ("attempt_id", "scenario_id", "repetition")}
        if digest(identity) != digest({key: right[key] for key in identity}):
            raise ComparisonError("attempt_schedule_mismatch")
        left_goal, right_goal = _goal(left["goal_met"]), _goal(right["goal_met"])
        matrix[left_goal][right_goal] += 1
        durations = [a["harness"]["elapsed_s"] for a in (left, right)]
        if any(type(value) not in {int, float} or not math.isfinite(value) or value < 0 for value in durations):
            raise ComparisonError("invalid_harness_duration")
        complete = [a["evidence_complete"] for a in (left, right)]
        if any(type(value) is not bool for value in complete) \
                or any(a["goal_met"] is not None and not a["evidence_complete"] for a in (left, right)):
            raise ComparisonError("invalid_goal_qualification")
        pairs.append({**identity, "reference_goal": left_goal, "candidate_goal": right_goal,
                      "reference_evidence_complete": complete[0], "candidate_evidence_complete": complete[1],
                      "reference_harness_status": left["harness"]["status"],
                      "candidate_harness_status": right["harness"]["status"],
                      "reference_expectation_match": left["expectation_match"],
                      "candidate_expectation_match": right["expectation_match"],
                      "jointly_qualified": all(complete),
                      "reference_harness_s": durations[0], "candidate_harness_s": durations[1],
                      "candidate_minus_reference_harness_s": durations[1] - durations[0]})
    known = sum(pair["reference_goal"] != "UNKNOWN" and pair["candidate_goal"] != "UNKNOWN" for pair in pairs)
    joint = [pair for pair in pairs if pair["jointly_qualified"]]
    latency = {}
    for name, subset in (("all_attempts", pairs), ("jointly_qualified_pairs", joint)):
        latency[name] = {"reference": distribution([p["reference_harness_s"] for p in subset]),
                         "candidate": distribution([p["candidate_harness_s"] for p in subset]),
                         "candidate_minus_reference": distribution(
                             [p["candidate_minus_reference_harness_s"] for p in subset])}
    return {"attempted_pairs": len(pairs), "goal_outcome_matrix": matrix,
            "both_goal_reports_available": known, "pairs_with_unknown_goal": len(pairs) - known,
            "candidate_goal_gains": matrix["GOAL_NOT_VERIFIED"]["VERIFIED_GOAL"],
            "candidate_goal_losses": matrix["VERIFIED_GOAL"]["GOAL_NOT_VERIFIED"],
            "jointly_qualified_pairs": len(joint), "pairs": pairs,
            "harness_latency_s": {"p95_method": "nearest_rank", **latency}}


def compare_campaigns(reference_directory, candidate_directory):
    """Compare exact frozen schedules from distinct, same-source campaign bundles."""
    try:
        reference_directory, candidate_directory = Path(reference_directory), Path(candidate_directory)
        if reference_directory.resolve() == candidate_directory.resolve():
            raise ComparisonError("same_campaign_directory")
        left, right = (_load_campaign(directory) for directory in (reference_directory, candidate_directory))
        if left["manifest_sha256"] == right["manifest_sha256"] \
                or digest(left["manifest"]["artifacts"]) == digest(right["manifest"]["artifacts"]):
            raise ComparisonError("cloned_campaign_bundle")
        if digest(left["spec"]) != digest(right["spec"]):
            raise ComparisonError("frozen_configuration_mismatch")
        pins = ("runtime_sha256", "runtime_file_count", "runner_sha256")
        if any(digest(left["manifest"]["source"][key]) != digest(right["manifest"]["source"][key]) for key in pins):
            raise ComparisonError("executable_source_mismatch")
        if set(run_id for run_id in left["run_ids"] if run_id is not None).intersection(
                run_id for run_id in right["run_ids"] if run_id is not None):
            raise ComparisonError("reused_run_identity_across_campaigns")
        report = paired_measurements(left["summary"]["attempts"], right["summary"]["attempts"])
        for index, pair in enumerate(report["pairs"]):
            pair["reference_run_id"] = left["run_ids"][index]
            pair["candidate_run_id"] = right["run_ids"][index]
        metric_names = ("verified_goal_rate", "legacy_task_success_rate", "expectation_match_rate",
                        "replay_qualified_acceptance_rate", "evidence_complete_rate")
        arms, deltas = {}, {}
        for name, bundle in (("reference", left), ("candidate", right)):
            arms[name] = {"manifest_sha256": bundle["manifest_sha256"],
                          "configuration_sha256": bundle["manifest"]["configuration_sha256"],
                          "source": bundle["manifest"]["source"],
                          "attempts_with_run_identity": sum(run_id is not None for run_id in bundle["run_ids"]),
                          "attempts_without_run_identity": sum(run_id is None for run_id in bundle["run_ids"]),
                          "metrics": {key: bundle["summary"][key] for key in metric_names},
                          "unknown_goal_outcomes": bundle["summary"]["unknown_goal_outcomes"],
                          "safety_adjudication": bundle["summary"]["safety_adjudication"]}
        for name in metric_names:
            difference = right["summary"][name]["numerator"] - left["summary"][name]["numerator"]
            deltas[name] = {"numerator_difference": difference, "denominator": report["attempted_pairs"],
                            "candidate_minus_reference": difference / report["attempted_pairs"]}
        # Recheck both complete inventories after consuming inputs; detect concurrent edits.
        for directory, bundle in ((reference_directory, left), (candidate_directory, right)):
            if _sha256(directory / "manifest.json") != bundle["manifest_sha256"] \
                    or not verify_campaign(directory)["valid"]:
                raise ComparisonError("input_changed_during_comparison")
        return {"schema_version": SCHEMA, "scope": "same_implementation_mock_repeatability",
                "performance_claim_authorized": False, "configuration_sha256": digest(left["spec"]),
                "comparator_sha256": _sha256(ROOT / "scripts" / "core_comparison.py"),
                "arms": arms, "all_attempt_rate_differences": deltas, **report}
    except ComparisonError:
        raise
    except (OSError, ValueError, TypeError, KeyError, AttributeError, IndexError) as error:
        raise ComparisonError("comparison_unreadable:" + type(error).__name__) from error


def verify_comparison(reference, candidate, report_path):
    """Reconstruct the entire report including type-sensitive counts and input pins."""
    try:
        expected = compare_campaigns(reference, candidate)
        if digest(json.loads(Path(report_path).read_text())) != digest(expected):
            raise ComparisonError("comparison_reconstruction_mismatch")
    except (OSError, ValueError, TypeError) as error:
        return {"schema_version": SCHEMA, "valid": False, "findings": [str(error)]}
    return {"schema_version": SCHEMA, "valid": True, "findings": []}
