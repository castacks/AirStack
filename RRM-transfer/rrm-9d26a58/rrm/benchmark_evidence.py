"""Fail-closed replay and aggregation for the core Oracle benchmark.

This module measures only the deterministic MockWorld/ScriptedOracle regression
suite.  It deliberately does not turn that fixture into a SIL performance claim.
"""

from __future__ import annotations

import hashlib
import json
import math
import platform
import statistics
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from .loop import THETA_DIV
from .trace import TRACE_EVENT_SCHEMA


MANIFEST_SCHEMA = "rrm-core-benchmark-manifest/v1"
METRICS_SCHEMA = "rrm-core-benchmark-metrics/v1"
REPLAY_SCHEMA = "rrm-core-benchmark-replay/v1"


class EvidenceError(ValueError):
    """Raised when benchmark evidence is incomplete or internally inconsistent."""


@dataclass(frozen=True)
class EpisodeReplay:
    task_id: str
    valid: bool
    findings: tuple[str, ...]
    metrics: dict[str, Any] | None
    planning_calls_ms: tuple[float, ...]
    trace_sha256: str

    def report(self, trace_name: str) -> dict[str, Any]:
        return {
            "trace": trace_name,
            "task_id": self.task_id,
            "valid": self.valid,
            "findings": list(self.findings),
            "trace_sha256": self.trace_sha256,
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_digest(rrm_root: Path) -> tuple[str, int]:
    """Hash the executable core source, including uncommitted/untracked files."""
    paths = sorted((rrm_root / "rrm").glob("*.py"))
    paths.extend([rrm_root / "scripts" / "oracle_loop.py", rrm_root / "requirements.txt"])
    digest = hashlib.sha256()
    for path in sorted(paths):
        relative = path.relative_to(rrm_root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        content = path.read_bytes()
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest(), len(paths)


def _read_events(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    events: list[dict[str, Any]] = []
    findings: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        return events, [f"trace_unreadable:{type(exc).__name__}"]

    if not lines:
        return events, ["trace_empty"]
    for line_number, line in enumerate(lines, start=1):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            findings.append(f"invalid_json:line={line_number}")
            continue
        if not isinstance(value, dict):
            findings.append(f"event_not_object:line={line_number}")
            continue
        events.append(value)
    return events, findings


def _integer(value: Any) -> bool:
    return type(value) is int and value >= 0


def _number(value: Any) -> bool:
    return type(value) in {int, float} and math.isfinite(value)


def _rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "value": numerator / denominator if denominator else None,
    }


def replay_trace(path: Path, *, expected_task_id: str | None = None) -> EpisodeReplay:
    """Validate and independently reconstruct one episode's reported counters."""
    events, findings = _read_events(path)
    task_id = expected_task_id or path.stem
    trace_hash = _sha256(path) if path.is_file() else ""

    for index, event in enumerate(events):
        if event.get("schema_version") != TRACE_EVENT_SCHEMA:
            findings.append(f"unsupported_schema:sequence={index}")
        if event.get("sequence") != index:
            findings.append(f"non_contiguous_sequence:expected={index}")
        if not _number(event.get("wall_time_s")):
            findings.append(f"invalid_wall_time:sequence={index}")
        if not _number(event.get("monotonic_time_s")):
            findings.append(f"invalid_monotonic_time:sequence={index}")
        if index and _number(event.get("monotonic_time_s")) and _number(
                events[index - 1].get("monotonic_time_s")):
            if event["monotonic_time_s"] < events[index - 1]["monotonic_time_s"]:
                findings.append(f"monotonic_time_regressed:sequence={index}")
        if not isinstance(event.get("kind"), str) or not event["kind"]:
            findings.append(f"invalid_kind:sequence={index}")

    if not events or events[0].get("kind") != "run_start":
        findings.append("missing_run_start")
    if not events or events[-1].get("kind") != "episode_end":
        findings.append("missing_episode_end")

    starts = [event for event in events if event.get("kind") == "run_start"]
    ends = [event for event in events if event.get("kind") == "episode_end"]
    if len(starts) != 1:
        findings.append(f"run_start_count:{len(starts)}")
    if len(ends) != 1:
        findings.append(f"episode_end_count:{len(ends)}")
    if not starts or not ends:
        return EpisodeReplay(task_id, False, tuple(dict.fromkeys(findings)), None, (), trace_hash)

    start, end = starts[0], ends[0]
    actual_task_id = start.get("task_id")
    if not isinstance(actual_task_id, str) or not actual_task_id:
        findings.append("invalid_start_task_id")
    else:
        task_id = actual_task_id
    if expected_task_id is not None and actual_task_id != expected_task_id:
        findings.append("task_id_does_not_match_expected")
    if end.get("task_id") != actual_task_id:
        findings.append("terminal_task_id_mismatch")

    try:
        labels = EvaluationLabels.from_record(start.get("evaluation_labels"))
    except ValueError as exc:
        findings.append(f"invalid_evaluation_labels:{exc}")
        labels = None

    plans = [event for event in events if event.get("kind") == "plan"]
    replans = [event for event in events if event.get("kind") == "replan"]
    dispatches = [event for event in events if event.get("kind") == "dispatch"]
    divergences = [
        event for event in events
        if event.get("kind") == "divergence"
        and _number(event.get("magnitude"))
        and event["magnitude"] > THETA_DIV
    ]
    safety1_rejections = [
        event for event in events
        if event.get("kind") == "safety1" and event.get("verdict") == "FAIL"
    ]
    safety2_rejections = [
        event for event in events
        if event.get("kind") == "safety2" and event.get("verdict") == "FAIL"
    ]
    safety_events = {
        "symbolic": [event for event in events if event.get("kind") == "safety1"],
        "numeric": [event for event in events if event.get("kind") == "safety2"],
    }
    convergence_events = [event for event in events
                          if event.get("kind") == "replan_convergence"]
    uncertainty_gates = [event for event in events
                         if event.get("kind") == "uncertainty_gate"]
    failed_uncertainty_gates = [event for event in uncertainty_gates
                                if event.get("verdict") == "FAIL"]
    initial_uncertainty_abort = bool(
        failed_uncertainty_gates
        and failed_uncertainty_gates[0].get("phase") == "planning"
    )

    if len(plans) != (0 if initial_uncertainty_abort else 1):
        findings.append(f"initial_plan_count:{len(plans)}")

    for gate in uncertainty_gates:
        uncertainty = gate.get("uncertainty")
        threshold = gate.get("threshold")
        phase = gate.get("phase")
        verdict = gate.get("verdict")
        if not _number(uncertainty) or not 0.0 <= uncertainty <= 1.0 \
                or not _number(threshold) or not 0.0 <= threshold <= 1.0:
            findings.append(f"invalid_uncertainty_gate_value:sequence={gate.get('sequence')}")
            continue
        expected_verdict = "PASS" if uncertainty <= threshold else "FAIL"
        if verdict != expected_verdict:
            findings.append(f"invalid_uncertainty_gate_verdict:sequence={gate.get('sequence')}")
        if phase not in {"planning", "pre_action", "dispatch"}:
            findings.append(f"invalid_uncertainty_gate_phase:sequence={gate.get('sequence')}")
    if len(failed_uncertainty_gates) > 1:
        findings.append(f"uncertainty_gate_failure_count:{len(failed_uncertainty_gates)}")

    action_fields = {"action_id", "verb", "targets", "params", "action_digest"}
    action_catalog: dict[tuple[int, str], dict[str, Any]] = {}
    catalog_sequences: dict[tuple[int, str], int] = {}
    plan_events = plans + replans
    for expected_version, event in enumerate(plan_events):
        version = event.get("version")
        if version != expected_version:
            findings.append(
                f"invalid_plan_version:expected={expected_version}:actual={version}"
            )
        actions = event.get("actions")
        if not isinstance(actions, list):
            findings.append(f"invalid_plan_action_catalog:version={version}")
            continue
        local_ids: set[str] = set()
        for action in actions:
            if not isinstance(action, dict) or set(action) != action_fields:
                findings.append(f"invalid_plan_action_record:version={version}")
                continue
            action_id = action.get("action_id")
            if not isinstance(action_id, str) or not action_id:
                findings.append(f"invalid_plan_action_id:version={version}")
                continue
            if action_id in local_ids:
                findings.append(f"duplicate_plan_action_id:version={version}")
                continue
            local_ids.add(action_id)
            if not isinstance(action.get("verb"), str) or not action["verb"] \
                    or not isinstance(action.get("targets"), list) \
                    or any(not isinstance(target, str) for target in action["targets"]) \
                    or not isinstance(action.get("params"), dict):
                findings.append(f"invalid_plan_action_payload:version={version}")
                continue
            digest_payload = {key: action[key] for key in (
                "action_id", "verb", "targets", "params",
            )}
            encoded = json.dumps(
                digest_payload, sort_keys=True, separators=(",", ":"), default=str,
            )
            expected_digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
            if action.get("action_digest") != expected_digest:
                findings.append(f"invalid_plan_action_digest:version={version}")
                continue
            if _integer(version):
                ref = (version, action_id)
                action_catalog[ref] = action
                catalog_sequences[ref] = event.get("sequence", -1)

    action_event_kinds = {"safety1", "safety2", "apply", "dispatch", "divergence"}
    for event in events:
        if event.get("kind") not in action_event_kinds:
            continue
        version = event.get("plan_version")
        action_id = event.get("action_id")
        if not _integer(version) or not isinstance(action_id, str) or not action_id:
            findings.append(f"invalid_action_reference:sequence={event.get('sequence')}")
            continue
        ref = (version, action_id)
        if ref not in action_catalog:
            findings.append(f"unknown_action_reference:sequence={event.get('sequence')}")
        elif catalog_sequences[ref] >= event.get("sequence", -1):
            findings.append(f"action_reference_before_plan:sequence={event.get('sequence')}")
        elif event.get("action_digest") != action_catalog[ref]["action_digest"]:
            findings.append(f"action_digest_mismatch:sequence={event.get('sequence')}")

    for gate in uncertainty_gates:
        if gate.get("phase") == "planning":
            if any(key in gate for key in ("plan_version", "action_id", "action_digest")):
                findings.append(
                    f"planning_uncertainty_gate_has_action:sequence={gate.get('sequence')}"
                )
            continue
        version = gate.get("plan_version")
        action_id = gate.get("action_id")
        ref = (version, action_id)
        if not _integer(version) or not isinstance(action_id, str) \
                or ref not in action_catalog:
            findings.append(
                f"unknown_uncertainty_action_reference:sequence={gate.get('sequence')}"
            )
        elif gate.get("action_digest") != action_catalog[ref]["action_digest"]:
            findings.append(
                f"uncertainty_action_digest_mismatch:sequence={gate.get('sequence')}"
            )

    for gate in failed_uncertainty_gates:
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = events[sequence + 1:]
        if gate.get("phase") == "dispatch":
            if not following or following[0].get("kind") != "dispatch" \
                    or following[0].get("termination") != "UNCERTAIN":
                findings.append("uncertainty_dispatch_without_terminal_record")
            following = following[1:]
        if len(following) != 1 or following[0].get("kind") != "episode_end":
            findings.append("execution_after_uncertainty_failure")

    for event in replans:
        trigger = event.get("trigger")
        if not isinstance(trigger, dict):
            findings.append(f"invalid_replan_trigger:version={event.get('version')}")
            continue
        trigger_version = trigger.get("plan_version")
        trigger_id = trigger.get("action_id")
        if not _integer(trigger_version) or not isinstance(trigger_id, str) \
                or (trigger_version, trigger_id) not in action_catalog:
            findings.append(f"unknown_replan_trigger:version={event.get('version')}")
        if _integer(event.get("version")) \
                and trigger_version != event["version"] - 1:
            findings.append(f"invalid_replan_trigger_version:version={event.get('version')}")

    if any(not _integer(event.get("cycles")) for event in dispatches):
        findings.append("invalid_dispatch_cycles")
    planning_events = plans + replans
    if any(not _number(event.get("latency_ms")) or event["latency_ms"] < 0
           for event in planning_events):
        findings.append("invalid_planning_latency")
        planning_calls: tuple[float, ...] = ()
    else:
        planning_calls = tuple(float(event["latency_ms"]) for event in planning_events)

    reconstructed = {
        "task_id": task_id,
        "task_success": end.get("task_success"),
        "goal_met": end.get("goal_met"),
        "aborted": end.get("aborted"),
        "replans": len(replans),
        "action_count": len(dispatches),
        "inner_cycles": sum(event.get("cycles", 0) for event in dispatches
                            if _integer(event.get("cycles"))),
        "divergences": len(divergences),
        # The legacy RunMetrics field is a count of safety rejections, not proof
        # that an unsafe action was dispatched. Preserve that distinction here.
        "safety_rejections": len(safety1_rejections) + len(safety2_rejections),
        "planning_latency_ms": round(sum(planning_calls), 3),
        "reasoner_calls": len(planning_calls),
    }

    for boolean_field in ("task_success", "goal_met", "aborted"):
        if type(reconstructed[boolean_field]) is not bool:
            findings.append(f"invalid_terminal_{boolean_field}")

    if len(convergence_events) > 1:
        findings.append(f"replan_convergence_count:{len(convergence_events)}")
    for convergence in convergence_events:
        reason = convergence.get("reason")
        rejected = convergence.get("rejected_action")
        replacement = convergence.get("replacement_action")
        rejected_id = convergence.get("rejected_action_id")
        replacement_id = convergence.get("replacement_action_id")
        rejected_version = convergence.get("rejected_plan_version")
        replacement_version = convergence.get("replacement_plan_version")
        if not isinstance(rejected, dict) or set(rejected) != {"verb", "targets", "params"}:
            findings.append("invalid_replan_convergence_rejected_action")
        if not isinstance(rejected_id, str) or not rejected_id:
            findings.append("invalid_replan_convergence_rejected_action_id")
        rejected_ref = (rejected_version, rejected_id)
        if not _integer(rejected_version) or rejected_ref not in action_catalog:
            findings.append("invalid_replan_convergence_rejected_reference")
        elif {key: value for key, value in action_catalog[rejected_ref].items()
              if key not in {"action_id", "action_digest"}} != rejected:
            findings.append("invalid_replan_convergence_rejected_payload")
        if reason == "UNCHANGED_REJECTED_ACTION":
            if not isinstance(replacement, dict) or replacement != rejected:
                findings.append("invalid_replan_convergence_unchanged_payload")
            if not isinstance(replacement_id, str) or not replacement_id:
                findings.append("invalid_replan_convergence_replacement_action_id")
            replacement_ref = (replacement_version, replacement_id)
            if not _integer(replacement_version) or replacement_ref not in action_catalog:
                findings.append("invalid_replan_convergence_replacement_reference")
            elif {key: value for key, value in action_catalog[replacement_ref].items()
                  if key not in {"action_id", "action_digest"}} != replacement:
                findings.append("invalid_replan_convergence_replacement_payload")
        elif reason == "NO_ALTERNATIVE_AFTER_REJECTION":
            if replacement is not None or replacement_id is not None:
                findings.append("invalid_replan_convergence_missing_alternative")
        else:
            findings.append("invalid_replan_convergence_reason")
        if reconstructed["aborted"] is not True:
            findings.append("replan_convergence_without_abort")
        sequence = convergence.get("sequence")
        if _integer(sequence):
            preceding = events[max(0, sequence - 2):sequence]
            if len(preceding) != 2 \
                    or preceding[0].get("kind") != "safety1" \
                    or preceding[0].get("verdict") != "FAIL" \
                    or preceding[1].get("kind") != "replan":
                findings.append("invalid_replan_convergence_causal_order")
            else:
                trigger = preceding[1].get("trigger")
                if preceding[0].get("action_id") != rejected_id \
                        or preceding[0].get("plan_version") != rejected_version \
                        or not isinstance(trigger, dict) \
                        or trigger.get("action_id") != rejected_id \
                        or trigger.get("plan_version") != rejected_version:
                    findings.append("invalid_replan_convergence_action_binding")
                if preceding[1].get("version") != replacement_version:
                    findings.append("invalid_replan_convergence_plan_binding")
                if any(event.get("sim_t") != convergence.get("sim_t")
                       for event in preceding):
                    findings.append("invalid_replan_convergence_state_binding")
            if sequence + 1 >= len(events) \
                    or events[sequence + 1].get("kind") != "episode_end":
                findings.append("replan_convergence_not_terminal")
            forbidden = {"safety1", "safety2", "apply", "dispatch", "replan"}
            if any(event.get("sequence", -1) > sequence and event.get("kind") in forbidden
                   for event in events):
                findings.append("execution_after_replan_convergence")

    expect_abort = start.get("expect_abort")
    if type(expect_abort) is not bool:
        findings.append("invalid_expect_abort")
    elif type(reconstructed["task_success"]) is bool and type(reconstructed["goal_met"]) is bool \
            and type(reconstructed["aborted"]) is bool:
        expected_success = reconstructed["aborted"] if expect_abort else reconstructed["goal_met"]
        if reconstructed["task_success"] is not expected_success:
            findings.append("terminal_success_semantics_mismatch")

    confusion = {"true_positive": 0, "false_positive": 0,
                 "true_negative": 0, "false_negative": 0}
    if labels is not None:
        expected_by_stage = {
            "symbolic": labels.symbolic_safety,
            "numeric": labels.numeric_safety,
        }
        for stage, stage_events in safety_events.items():
            expected_unsafe = expected_by_stage[stage] is SafetyLabel.UNSAFE
            for event in stage_events:
                if event.get("verdict") not in {"PASS", "FAIL"}:
                    findings.append(f"invalid_safety_verdict:{stage}")
                    continue
                rejected = event["verdict"] == "FAIL"
                key = {
                    (True, True): "true_positive",
                    (False, True): "false_positive",
                    (False, False): "true_negative",
                    (True, False): "false_negative",
                }[(expected_unsafe, rejected)]
                confusion[key] += 1

        if labels.expected_terminal is TerminalLabel.GOAL_VERIFIED:
            if reconstructed["goal_met"] is not True or reconstructed["aborted"] is not False:
                findings.append("expected_terminal_mismatch:GOAL_VERIFIED")
        elif reconstructed["goal_met"] is not False or reconstructed["aborted"] is not True:
            findings.append("expected_terminal_mismatch:SAFE_ABORT")

        failure_injected = labels.failure_kind is not FailureKind.NONE
        failure_detected = bool(divergences) if failure_injected else None
        recovery_eligible = labels.failure_recoverable is True
        recovery_succeeded = (
            failure_detected is True
            and reconstructed["goal_met"] is True
            and reconstructed["aborted"] is False
        ) if recovery_eligible else None
        reconstructed["adjudication"] = {
            "labels": labels.as_record(),
            "safety_confusion": confusion,
            "failure_injected": failure_injected,
            "failure_detected": failure_detected,
            "recovery_eligible": recovery_eligible,
            "recovery_succeeded": recovery_succeeded,
        }

    terminal_count_fields = {
        "replans": reconstructed["replans"],
        "action_count": reconstructed["action_count"],
        "inner_cycles": reconstructed["inner_cycles"],
        "divergences": reconstructed["divergences"],
        "safety_rejections": reconstructed["safety_rejections"],
    }
    for field, expected in terminal_count_fields.items():
        if end.get(field) != expected:
            findings.append(f"terminal_counter_mismatch:{field}")

    terminal_latency = end.get("planning_latency_ms")
    if not _number(terminal_latency):
        findings.append("invalid_terminal_planning_latency")
    elif abs(float(terminal_latency) - reconstructed["planning_latency_ms"]) > 0.005:
        findings.append("terminal_counter_mismatch:planning_latency_ms")

    divergence_replans = sum(
        1 for event in replans
        if isinstance(event.get("trigger"), dict)
        and _number(event["trigger"].get("magnitude"))
        and event["trigger"]["magnitude"] > THETA_DIV
    )
    expected_recoveries = 0 if reconstructed["aborted"] is True else divergence_replans
    if end.get("recoveries") != expected_recoveries:
        findings.append("terminal_counter_mismatch:recoveries")
    reconstructed["recoveries"] = expected_recoveries

    unique_findings = tuple(dict.fromkeys(findings))
    return EpisodeReplay(task_id, not unique_findings, unique_findings,
                         reconstructed if not unique_findings else None,
                         planning_calls, trace_hash)


def _latency_summary(values: Iterable[float]) -> dict[str, Any]:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "total": 0.0, "median": None, "p95": None, "max": None}
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "count": len(ordered),
        "total": round(sum(ordered), 3),
        "median": round(statistics.median(ordered), 3),
        "p95": round(ordered[p95_index], 3),
        "max": round(ordered[-1], 3),
    }


def _git_value(repo_root: Path, *args: str) -> str | None:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), *args], capture_output=True, text=True, check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def _write_json(path: Path, value: dict[str, Any]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def write_benchmark_evidence(trace_dir: Path, *, task_ids: list[str],
                             run_meta: dict[str, Any], repo_root: Path) -> None:
    """Replay a completed suite and write its immutable evidence bundle."""
    replays = [replay_trace(trace_dir / f"{task_id}.jsonl", expected_task_id=task_id)
               for task_id in task_ids]
    replay_report = {
        "schema_version": REPLAY_SCHEMA,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "valid": all(replay.valid for replay in replays),
        "expected_tasks": task_ids,
        "valid_trace_count": sum(replay.valid for replay in replays),
        "trace_count": len(replays),
        "traces": [replay.report(f"{task_id}.jsonl")
                   for task_id, replay in zip(task_ids, replays)],
    }
    replay_path = trace_dir / "replay-report.json"
    _write_json(replay_path, replay_report)
    if not replay_report["valid"]:
        raise EvidenceError("benchmark trace replay failed; see replay-report.json")

    episode_metrics = [replay.metrics for replay in replays if replay.metrics is not None]
    planning_calls = [latency for replay in replays for latency in replay.planning_calls_ms]
    passed = sum(metric["task_success"] for metric in episode_metrics)
    total = len(episode_metrics)
    confusion = {
        key: sum(metric["adjudication"]["safety_confusion"][key]
                 for metric in episode_metrics)
        for key in ("true_positive", "false_positive", "true_negative", "false_negative")
    }
    unsafe_labels = confusion["true_positive"] + confusion["false_negative"]
    rejections = confusion["true_positive"] + confusion["false_positive"]
    safe_labels = confusion["true_negative"] + confusion["false_positive"]
    injected = [metric for metric in episode_metrics
                if metric["adjudication"]["failure_injected"]]
    detected = sum(metric["adjudication"]["failure_detected"] is True for metric in injected)
    recoverable = [metric for metric in episode_metrics
                   if metric["adjudication"]["recovery_eligible"]]
    recovered = sum(metric["adjudication"]["recovery_succeeded"] is True
                    for metric in recoverable)
    metrics = {
        "schema_version": METRICS_SCHEMA,
        "benchmark_scope": "deterministic_mock_oracle_regression",
        "performance_claim_authorized": False,
        "performance_claim_reason": (
            "One deterministic MockWorld seed is a component regression, not the "
            "30-seed integrated SIL campaign required by SCRUM-8."
        ),
        "evidence_complete": True,
        "task_success_rate": {
            "numerator": passed,
            "denominator": total,
            "value": passed / total if total else None,
        },
        "safety_verifier": {
            "decision_scope": "labelled_symbolic_and_numeric_safety_decisions",
            "confusion_matrix": confusion,
            "recall": _rate(confusion["true_positive"], unsafe_labels),
            "precision": _rate(confusion["true_positive"], rejections),
            "false_negative_rate": _rate(confusion["false_negative"], unsafe_labels),
            "false_refusal_rate": _rate(confusion["false_positive"], safe_labels),
        },
        "failure_detection_rate": _rate(detected, len(injected)),
        "recovery_success_rate": _rate(recovered, len(recoverable)),
        "planning_latency_ms": _latency_summary(planning_calls),
        "totals": {
            "actions_attempted": sum(metric["action_count"] for metric in episode_metrics),
            "replans": sum(metric["replans"] for metric in episode_metrics),
            "divergences": sum(metric["divergences"] for metric in episode_metrics),
            "recoveries": sum(metric["recoveries"] for metric in episode_metrics),
            "safety_rejections": sum(metric["safety_rejections"] for metric in episode_metrics),
        },
        "episodes": episode_metrics,
    }
    metrics_path = trace_dir / "metrics.json"
    _write_json(metrics_path, metrics)

    commit = _git_value(repo_root, "rev-parse", "HEAD")
    status = _git_value(repo_root, "status", "--porcelain")
    rrm_root = Path(__file__).resolve().parents[1]
    source_digest, source_file_count = _source_digest(rrm_root)
    artifacts = {
        path.name: {"sha256": _sha256(path), "bytes": path.stat().st_size}
        for path in sorted(trace_dir.iterdir())
        if path.is_file() and path.name != "manifest.json"
    }
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "benchmark_scope": "deterministic_mock_oracle_regression",
        "source": {
            "git_commit": commit,
            "git_branch": _git_value(repo_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(status) if status is not None else None,
            "rrm_runtime_source_sha256": source_digest,
            "rrm_runtime_source_file_count": source_file_count,
        },
        "runtime": {
            "python": sys.version.split()[0],
            "implementation": platform.python_implementation(),
            "platform": platform.platform(),
        },
        "configuration": run_meta,
        "task_ids": task_ids,
        "repetitions_per_seed": 1,
        "simulator": {"applicable": False, "reason": "MockWorld regression"},
        "gpu": {"applicable": False, "reason": "No model or simulator is used"},
        "model": {"applicable": False, "reason": "ScriptedOracle has no model weights"},
        "artifacts": artifacts,
    }
    _write_json(trace_dir / "manifest.json", manifest)
