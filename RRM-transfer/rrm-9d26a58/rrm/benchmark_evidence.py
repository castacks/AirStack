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
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from .contracts import (
    ApprovalDecision, ApprovalScope, CapabilityDeclaration, DispatchContext,
    PermissionDeclaration, SafetyDecision, admission_rejection_reason,
)
from .loop import THETA_DIV, _constraints_record, _trace_action_record
from .core_stop import StopStateEvidence
from .core_deadlines import CoreCallLimits
from .uncertainty import validate_uncertainty
from .schema import AbstractAction, Divergence, NumericLimitProfile, Task, TaskGraph, Trajectory, Verb, WorldState
from .safety import NumericSafetyVerifier, SafetyVerifier
from .verbs import VERB_TABLE, expected_effects_of, holds
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


def _record_digest(record: dict[str, Any]) -> str:
    return hashlib.sha256(json.dumps(
        record, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")).hexdigest()


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

    write_ids = [event.get("evidence_write_id") for event in events[1:]]
    if any(not isinstance(value, str) or not value.strip() for value in write_ids) \
            or len(write_ids) != len(set(str(value) for value in write_ids)):
        findings.append("invalid_evidence_write_identity")

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
    run_id = start.get("run_id")
    if not isinstance(run_id, str) or not run_id.strip():
        findings.append("invalid_run_id")
    for event in events:
        if event.get("run_id") != run_id:
            findings.append(f"run_id_mismatch:sequence={event.get('sequence')}")

    task_events = [event for event in events if event.get("kind") == "task_declaration"]
    task_record = None
    task_digest = None
    task_revision = None
    if len(task_events) != 1:
        findings.append(f"task_declaration_count:{len(task_events)}")
    else:
        declaration = task_events[0]
        task_record = declaration.get("task")
        task_digest = declaration.get("task_digest")
        task_revision = declaration.get("task_revision")
        try:
            task = Task.model_validate(task_record)
            if task_record != task.model_dump(mode="json") \
                    or task_digest != _record_digest(task_record) \
                    or task_revision != task.revision \
                    or task.id != start.get("task_id") \
                    or task.expect_abort != start.get("expect_abort"):
                findings.append("task_declaration_mismatch")
        except (TypeError, ValueError):
            findings.append("invalid_task_declaration")
        if declaration.get("sequence", -1) <= start.get("sequence", -1):
            findings.append("task_declaration_order")

    limit_events = [event for event in events if event.get("kind") == "call_limits_declaration"]
    limits = None
    if len(limit_events) != 1:
        findings.append("call_limits_declaration_count_mismatch")
    else:
        record = limit_events[0].get("limits")
        try:
            limits = CoreCallLimits(**{key: value for key, value in record.items()
                                      if key != "version"})
            if limits.record() != record or limit_events[0].get("sequence") != 1:
                raise ValueError("noncanonical limits")
        except (AttributeError, TypeError, ValueError):
            findings.append("invalid_call_limits_declaration")
            limits = None

    def deadline_valid(record, label=None) -> bool:
        if not isinstance(record, dict) or limits is None:
            return False
        base_fields = {"call", "timeout_s", "elapsed_s", "operation_pending"}
        expected_fields = (base_fields | {"event_kind", "event_write_id", "event_committed",
                                          "event_reconstructible"}
                           if record.get("call") == "trace_write" else base_fields)
        if set(record) != expected_fields:
            return False
        configured = {
            "reasoner_plan": limits.reasoner_s, "reasoner_replan": limits.reasoner_s,
            "policy_step": limits.policy_s, "observe": limits.observation_s,
            "begin_dispatch": limits.adapter_s, "apply": limits.adapter_s,
            "end_dispatch": limits.adapter_s, "cancel_dispatch": limits.stop_s,
            "observe_safe_state": limits.stop_s, "trace_write": limits.evidence_s,
        }
        return bool(record.get("call") in configured
                    and (label is None or record.get("call") == label)
                    and record.get("timeout_s") == configured.get(record.get("call"))
                    and _number(record.get("elapsed_s"))
                    and record["elapsed_s"] >= record["timeout_s"]
                    and type(record.get("operation_pending")) is bool
                    and (record.get("call") != "trace_write" or (
                        isinstance(record.get("event_kind"), str)
                        and bool(record["event_kind"].strip())
                        and isinstance(record.get("event_write_id"), str)
                        and bool(record["event_write_id"].strip())
                        and type(record.get("event_committed")) is bool
                        and type(record.get("event_reconstructible")) is bool)))

    state_sequences: dict[str, int] = {}
    state_payloads: dict[str, WorldState] = {}
    for event in (item for item in events if item.get("kind") == "world_state"):
        payload = event.get("state")
        digest = event.get("state_digest")
        if not isinstance(payload, dict) or not isinstance(digest, str):
            findings.append(f"invalid_world_state_event:sequence={event.get('sequence')}")
            continue
        try:
            validated = WorldState.model_validate(payload).model_dump(mode="json")
        except Exception:
            findings.append(f"invalid_world_state_payload:sequence={event.get('sequence')}")
            continue
        encoded = json.dumps(validated, sort_keys=True, separators=(",", ":"))
        expected = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        if digest != expected:
            findings.append(f"world_state_digest_mismatch:sequence={event.get('sequence')}")
            continue
        try:
            validate_uncertainty(WorldState.model_validate(validated))
        except ValueError:
            findings.append(f"uncertainty_evidence_mismatch:sequence={event.get('sequence')}")
            continue
        state_sequences.setdefault(digest, event.get("sequence", -1))
        state_payloads[digest] = WorldState.model_validate(validated)

    state_bound_kinds = {
        "plan", "replan", "uncertainty_gate", "safety1", "safety2", "apply",
        "dispatch", "divergence", "episode_end", "capability_gate", "permission_gate",
        "numeric_profile_gate", "context_gate", "dispatch_intent", "approval_gate",
        "authorization_gate", "interruption_gate", "dynamic_safety_gate",
        "observation_failure", "execution_fault", "reasoner_request", "reasoner_failure",
    }
    for event in events:
        if event.get("kind") not in state_bound_kinds:
            continue
        digest = event.get("state_digest")
        sequence = event.get("sequence", -1)
        if digest is None and (
                event.get("kind") == "observation_failure"
                and event.get("phase") == "planning"
                or event.get("kind") == "episode_end"
                and event.get("terminal_observation") == "UNAVAILABLE"
                and any(item.get("kind") == "observation_failure"
                        and item.get("phase") == "planning" for item in events)):
            continue
        if not isinstance(digest, str) or digest not in state_sequences:
            findings.append(f"unknown_state_reference:sequence={sequence}")
        elif state_sequences[digest] >= sequence:
            findings.append(f"state_reference_before_observation:sequence={sequence}")
        if event.get("kind") == "divergence":
            before_digest = event.get("before_state_digest")
            if not isinstance(before_digest, str) or before_digest not in state_sequences:
                findings.append(f"unknown_before_state_reference:sequence={sequence}")
            elif state_sequences[before_digest] >= sequence:
                findings.append(f"before_state_reference_before_observation:sequence={sequence}")
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
    capability_events = [event for event in events
                         if event.get("kind") == "capability_declaration"]
    capability_gates = [event for event in events if event.get("kind") == "capability_gate"]
    permission_events = [event for event in events
                         if event.get("kind") == "permission_declaration"]
    permission_gates = [event for event in events if event.get("kind") == "permission_gate"]
    approval_gates = [event for event in events if event.get("kind") == "approval_gate"]
    profile_events = [event for event in events
                      if event.get("kind") == "numeric_profile_declaration"]
    profile_gates = [event for event in events if event.get("kind") == "numeric_profile_gate"]
    constraints_events = [event for event in events
                          if event.get("kind") == "constraints_declaration"]
    authority_events = [event for event in events if event.get("kind") == "admission_authority"]
    authorization_gates = [event for event in events if event.get("kind") == "authorization_gate"]
    capabilities = None
    capability_digest = None
    capability_sequence = None
    if len(capability_events) != 1:
        findings.append(f"capability_declaration_count:{len(capability_events)}")
    else:
        record = capability_events[0].get("capability")
        capability_digest = capability_events[0].get("capability_digest")
        capability_sequence = capability_events[0].get("sequence")
        try:
            if not isinstance(record, dict) or set(record) != {
                "embodiment_id", "revision", "operations", "resources",
                "available_resources", "limits_ref",
            }:
                raise ValueError("invalid fields")
            capabilities = CapabilityDeclaration(
                embodiment_id=record["embodiment_id"], revision=record["revision"],
                operations=frozenset(record["operations"]),
                resources=frozenset(record["resources"]),
                available_resources=frozenset(record["available_resources"]),
                limits_ref=record["limits_ref"],
            )
            encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
            if capability_digest != hashlib.sha256(encoded.encode()).hexdigest():
                findings.append("capability_declaration_digest_mismatch")
        except (KeyError, TypeError, ValueError):
            findings.append("invalid_capability_declaration")
            capabilities = None
    permission = None
    permission_digest = None
    permission_sequence = None
    if len(permission_events) != 1:
        findings.append(f"permission_declaration_count:{len(permission_events)}")
    else:
        record = permission_events[0].get("permission")
        permission_digest = permission_events[0].get("permission_digest")
        permission_sequence = permission_events[0].get("sequence")
        try:
            if not isinstance(record, dict) or set(record) != {
                "authority_id", "revision", "task_id", "embodiment_id",
                "operations", "resources",
            }:
                raise ValueError("invalid fields")
            permission = PermissionDeclaration(
                authority_id=record["authority_id"], revision=record["revision"],
                task_id=record["task_id"], embodiment_id=record["embodiment_id"],
                operations=frozenset(record["operations"]),
                resources=frozenset(record["resources"]),
            )
            canonical = {
                "authority_id": permission.authority_id,
                "revision": permission.revision,
                "task_id": permission.task_id,
                "embodiment_id": permission.embodiment_id,
                "operations": sorted(permission.operations),
                "resources": sorted(permission.resources),
            }
            if record != canonical:
                findings.append("noncanonical_permission_declaration")
            encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
            if permission_digest != hashlib.sha256(encoded.encode()).hexdigest():
                findings.append("permission_declaration_digest_mismatch")
        except (AttributeError, KeyError, TypeError, ValueError):
            findings.append("invalid_permission_declaration")
            permission = None
    profile = None
    profile_digest = None
    profile_sequence = None
    if len(profile_events) != 1:
        findings.append(f"numeric_profile_declaration_count:{len(profile_events)}")
    else:
        record = profile_events[0].get("profile")
        profile_digest = profile_events[0].get("profile_digest")
        profile_sequence = profile_events[0].get("sequence")
        try:
            profile = NumericLimitProfile.model_validate(record)
            canonical = profile.model_dump(mode="json")
            if record != canonical:
                findings.append("noncanonical_numeric_profile")
            encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":"))
            if profile_digest != hashlib.sha256(encoded.encode()).hexdigest():
                findings.append("numeric_profile_digest_mismatch")
        except (TypeError, ValueError):
            findings.append("invalid_numeric_profile")
            profile = None
    constraints_digest = None
    if len(constraints_events) != 1:
        findings.append(f"constraints_declaration_count:{len(constraints_events)}")
    else:
        record = constraints_events[0].get("constraints")
        constraints_digest = constraints_events[0].get("constraints_digest")
        if not isinstance(record, dict) or not isinstance(profile_digest, str) \
                or record != _constraints_record(profile_digest) \
                or constraints_digest != _record_digest(record):
            findings.append("invalid_constraints_declaration")
    authority_epoch = None
    initial_stop_generation = None
    if len(authority_events) != 1:
        findings.append(f"admission_authority_count:{len(authority_events)}")
    else:
        authority = authority_events[0]
        authority_epoch = authority.get("authority_epoch")
        initial_stop_generation = authority.get("stop_generation")
        if not isinstance(authority_epoch, str) or not authority_epoch.strip() \
                or not _integer(initial_stop_generation) \
                or authority.get("evidence_kind") not in {
                    "trusted_in_process", "synthetic_fixture",
                }:
            findings.append("invalid_admission_authority")
    uncertainty_gates = [event for event in events
                         if event.get("kind") == "uncertainty_gate"]
    failed_uncertainty_gates = [event for event in uncertainty_gates
                                if event.get("verdict") == "FAIL"]
    initial_uncertainty_abort = bool(
        failed_uncertainty_gates
        and failed_uncertainty_gates[0].get("phase") == "planning"
    )

    initial_observation_abort = any(event.get("kind") == "observation_failure"
                                    and event.get("phase") == "planning" for event in events)
    reasoner_failures = [event for event in events if event.get("kind") == "reasoner_failure"]
    initial_reasoner_abort = any(event.get("phase") == "plan" for event in reasoner_failures)
    if len(plans) != (0 if initial_uncertainty_abort or initial_observation_abort or initial_reasoner_abort else 1):
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
    plan_digests: dict[int, str] = {}
    bound_plan_id: str | None = None
    bound_mission: str | None = None
    for expected_version, event in enumerate(plan_events):
        version = event.get("version")
        if version != expected_version:
            findings.append(
                f"invalid_plan_version:expected={expected_version}:actual={version}"
            )
        actions = event.get("actions")
        plan = event.get("plan")
        if not isinstance(plan, dict) or not isinstance(event.get("plan_digest"), str):
            findings.append(f"invalid_plan_record:version={version}")
        else:
            try:
                graph = TaskGraph.model_validate(plan.get("graph"))
                if plan["graph"] != graph.model_dump(mode="json") \
                        or plan.get("plan_id") != graph.mission_id \
                        or plan.get("version") != graph.version \
                        or plan.get("mission_text") != graph.mission_text \
                        or actions != [_trace_action_record(action) for action in graph.nodes]:
                    findings.append(f"plan_graph_mismatch:version={version}")
            except (TypeError, ValueError):
                findings.append(f"invalid_plan_graph:version={version}")
            if event["plan_digest"] != _record_digest(plan) \
                    or plan.get("actions") != actions \
                    or plan.get("version") != version \
                    or plan.get("plan_id") != event.get("plan_id") \
                    or plan.get("task_revision") != task_revision \
                    or plan.get("task_digest") != task_digest \
                    or event.get("task_revision") != task_revision \
                    or event.get("task_digest") != task_digest:
                findings.append(f"plan_binding_mismatch:version={version}")
            if not isinstance(plan.get("plan_id"), str) or not plan["plan_id"].strip() \
                    or not isinstance(plan.get("mission_text"), str):
                findings.append(f"invalid_plan_identity:version={version}")
            elif isinstance(task_record, dict) \
                    and plan["mission_text"] != task_record.get("mission"):
                findings.append(f"plan_mission_task_mismatch:version={version}")
            elif bound_plan_id is None:
                bound_plan_id = plan["plan_id"]
                bound_mission = plan["mission_text"]
            elif plan["plan_id"] != bound_plan_id \
                    or plan["mission_text"] != bound_mission:
                findings.append(f"plan_identity_changed:version={version}")
            if _integer(version):
                plan_digests[version] = event["plan_digest"]
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

    context_gates = [event for event in events if event.get("kind") == "context_gate"]
    for gate in context_gates:
        sequence = gate.get("sequence")
        version = gate.get("plan_version")
        ref = (version, gate.get("action_id"))
        reasons = []
        current_task = gate.get("current_task")
        current_plan = gate.get("current_plan")
        if not isinstance(current_task, dict) or not isinstance(current_plan, dict):
            findings.append(f"invalid_context_payload:sequence={sequence}")
            continue
        if gate.get("task_revision") != task_revision or current_task.get("revision") != task_revision:
            reasons.append("task_revision_changed")
        if _record_digest(current_task) != task_digest:
            reasons.append("task_payload_changed")
        if current_plan.get("plan_id") != bound_plan_id:
            reasons.append("plan_id_changed")
        if _record_digest(current_plan) != plan_digests.get(version):
            reasons.append("plan_payload_changed")
        if ref not in action_catalog or gate.get("action_digest") != action_catalog[ref]["action_digest"]:
            reasons.append("action_not_in_bound_plan")
        if gate.get("task_digest") != task_digest \
                or gate.get("task_id") != current_task.get("id") \
                or gate.get("observed_task_digest") != _record_digest(current_task) \
                or gate.get("plan_id") != bound_plan_id \
                or gate.get("plan_digest") != plan_digests.get(version) \
                or gate.get("observed_plan_digest") != _record_digest(current_plan) \
                or gate.get("reasons") != reasons \
                or gate.get("verdict") != ("DENY" if reasons else "ALLOW"):
            findings.append(f"context_gate_mismatch:sequence={sequence}")
        if gate.get("phase") not in {"pre_action", "pre_chunk", "pre_apply"}:
            findings.append(f"invalid_context_phase:sequence={sequence}")

    intents = [event for event in events if event.get("kind") == "dispatch_intent"]
    intent_by_id: dict[str, dict[str, Any]] = {}
    last_action_position: dict[int, int] = {}
    for intent in intents:
        dispatch_id = intent.get("dispatch_id")
        sequence = intent.get("sequence")
        ref = (intent.get("plan_version"), intent.get("action_id"))
        if not isinstance(dispatch_id, str) or not dispatch_id.strip() \
                or dispatch_id in intent_by_id:
            findings.append(f"invalid_dispatch_id:sequence={sequence}")
        else:
            intent_by_id[dispatch_id] = intent
        if ref not in action_catalog or intent.get("action_digest") != action_catalog[ref]["action_digest"] \
                or intent.get("task_id") != task_id \
                or intent.get("task_revision") != task_revision \
                or intent.get("task_digest") != task_digest \
                or intent.get("plan_id") != bound_plan_id \
                or intent.get("plan_digest") != plan_digests.get(intent.get("plan_version")):
            findings.append(f"dispatch_intent_binding_mismatch:sequence={sequence}")
        version = intent.get("plan_version")
        plan_event = next((item for item in plan_events
                           if item.get("version") == version), None)
        plan_actions = plan_event.get("actions") if plan_event is not None else None
        ordered_ids = [item.get("action_id") for item in plan_actions
                       if isinstance(item, dict)] if isinstance(plan_actions, list) else []
        if intent.get("action_id") not in ordered_ids:
            findings.append(f"dispatch_not_in_ordered_plan:sequence={sequence}")
        elif _integer(version):
            position = ordered_ids.index(intent["action_id"])
            previous = last_action_position.get(version, 0)
            if (version not in last_action_position and position != 0) \
                    or position < previous or position > previous + 1:
                findings.append(f"dispatch_order_mismatch:sequence={sequence}")
            last_action_position[version] = position
        preceding = events[:sequence] if _integer(sequence) else []
        if not preceding or preceding[-1].get("kind") != "authorization_gate" \
                or preceding[-1].get("verdict") != "ALLOW":
            findings.append(f"dispatch_intent_without_authorization:sequence={sequence}")
        else:
            authorization_gate = preceding[-1]
            decision_record = authorization_gate.get("decision")
            if authorization_gate.get("dispatch_id") != dispatch_id \
                    or intent.get("authorization_digest") != authorization_gate.get("decision_digest") \
                    or intent.get("authorization_context_digest") != authorization_gate.get("context_digest") \
                    or not isinstance(decision_record, dict) \
                    or intent.get("authorization_decision_id") != decision_record.get("decision_id"):
                findings.append(f"dispatch_intent_authorization_mismatch:sequence={sequence}")
        prior_approval = next((item for item in reversed(preceding)
                               if item.get("kind") == "approval_gate"), None)
        approval_record = (prior_approval.get("approval")
                           if prior_approval is not None else None)
        if prior_approval is None or prior_approval.get("verdict") != "ALLOW" \
                or (prior_approval.get("plan_version"), prior_approval.get("action_id")) != ref \
                or intent.get("approval_digest") != prior_approval.get("approval_digest") \
                or not isinstance(approval_record, dict) \
                or intent.get("approval_revision") != approval_record.get("revision") \
                or intent.get("approval_decision_id") != approval_record.get("decision_id"):
            findings.append(f"dispatch_intent_approval_mismatch:sequence={sequence}")
    for event in events:
        if event.get("kind") not in {"safety2", "apply", "dispatch"}:
            continue
        sequence = event.get("sequence")
        intent = intent_by_id.get(event.get("dispatch_id"))
        if intent is None or intent.get("sequence", -1) >= sequence \
                or any(event.get(field) != intent.get(field) for field in (
                    "plan_id", "plan_version", "plan_digest", "action_id", "action_digest",
                    "approval_digest", "approval_revision", "approval_decision_id",
                    "authorization_digest", "authorization_decision_id",
                    "authorization_context_digest",
                )):
            findings.append(f"dispatch_binding_mismatch:sequence={sequence}")
    for intent in intents:
        terminals = [event for event in dispatches
                     if event.get("dispatch_id") == intent.get("dispatch_id")]
        if len(terminals) != 1:
            findings.append(f"dispatch_terminal_count:sequence={intent.get('sequence')}")
    dynamic_gates = [event for event in events if event.get("kind") == "dynamic_safety_gate"]
    for gate in dynamic_gates:
        sequence = gate.get("sequence")
        dispatch_id = gate.get("dispatch_id")
        ref = (gate.get("plan_version"), gate.get("action_id"))
        action_record = action_catalog.get(ref)
        state = state_payloads.get(gate.get("state_digest"))
        if not _integer(sequence) or sequence == 0 or action_record is None \
                or state is None or dispatch_id not in intent_by_id \
                or (intent_by_id[dispatch_id].get("plan_version"),
                    intent_by_id[dispatch_id].get("action_id")) != ref \
                or intent_by_id[dispatch_id].get("sequence", -1) >= sequence \
                or gate.get("action_digest") != action_record.get("action_digest") \
                or events[sequence - 1].get("kind") != "context_gate" \
                or events[sequence - 1].get("phase") != "pre_chunk" \
                or events[sequence - 1].get("verdict") != "ALLOW" \
                or events[sequence - 1].get("dispatch_id") != dispatch_id:
            findings.append(f"invalid_dynamic_safety_binding:sequence={sequence}")
            continue
        try:
            action = AbstractAction(id=action_record["action_id"],
                                    verb=action_record["verb"],
                                    targets=action_record["targets"],
                                    params=action_record["params"])
            expected = SafetyVerifier().verify(action, state)
        except (KeyError, TypeError, ValueError):
            findings.append(f"invalid_dynamic_safety_action:sequence={sequence}")
            continue
        if gate.get("verdict") != expected.verdict \
                or gate.get("checked") != expected.checked \
                or gate.get("violations") != [v.model_dump() for v in expected.violations] \
                or not _integer(gate.get("cycle")) or gate["cycle"] < 1:
            findings.append(f"dynamic_safety_verdict_mismatch:sequence={sequence}")
        if gate.get("verdict") == "FAIL":
            following = events[sequence + 1:]
            if not following or following[0].get("kind") != "stop_request" \
                    or following[0].get("dispatch_id") != dispatch_id \
                    or following[0].get("reason") != "dynamic_symbolic_safety":
                findings.append(f"dynamic_safety_without_stop:sequence={sequence}")
    for gate in (event for event in events if event.get("kind") == "context_gate"
                 and event.get("phase") == "pre_apply"):
        sequence = gate.get("sequence")
        prior = events[sequence - 1] if _integer(sequence) and sequence > 0 else {}
        if prior.get("kind") != "dynamic_safety_gate" \
                or prior.get("verdict") != "PASS" \
                or prior.get("dispatch_id") != gate.get("dispatch_id"):
            findings.append(f"pre_apply_without_dynamic_safety:sequence={sequence}")
    for gate in (event for event in context_gates
                 if event.get("phase") == "pre_chunk" and event.get("verdict") == "ALLOW"):
        sequence = gate.get("sequence")
        record = action_catalog.get((gate.get("plan_version"), gate.get("action_id")))
        state = state_payloads.get(gate.get("state_digest"))
        if not _integer(sequence) or record is None or state is None:
            continue
        try:
            action = AbstractAction(id=record["action_id"], verb=record["verb"],
                                    targets=record["targets"], params=record["params"])
            effects = expected_effects_of(action)
            already_complete = bool(effects and all(holds(effect, state) for effect in effects))
        except (KeyError, TypeError, ValueError):
            continue
        if not already_complete and (sequence + 1 >= len(events)
                                     or events[sequence + 1].get("kind") not in {
                                         "dynamic_safety_gate", "execution_fault"}
                                     or events[sequence + 1].get("dispatch_id") != gate.get("dispatch_id")):
            findings.append(f"missing_dynamic_safety_gate:sequence={sequence}")
    stop_requests = [event for event in events if event.get("kind") == "stop_request"]
    stop_cancels = [event for event in events if event.get("kind") == "stop_cancel"]
    stop_states = [event for event in events if event.get("kind") == "stop_safe_state"]
    interruption_gates = [event for event in events if event.get("kind") == "interruption_gate"]
    observation_failures = [event for event in events
                            if event.get("kind") == "observation_failure"]
    execution_faults = [event for event in events if event.get("kind") == "execution_fault"]
    if len(execution_faults) > 1:
        findings.append("execution_fault_count_mismatch")
    for fault in execution_faults:
        sequence = fault.get("sequence")
        dispatch_id = fault.get("dispatch_id")
        intent = intent_by_id.get(dispatch_id)
        state = state_payloads.get(fault.get("state_digest"))
        phase = fault.get("phase")
        deadline = fault.get("deadline")
        late_evidence = None
        if fault.get("error_type") == "CoreCallTimeout":
            phase_call = {"policy_step": "policy_step", "observation": "observe",
                          "dispatch_terminal_observation": "observe",
                          "post_dispatch_observation": "observe", "begin_dispatch": "begin_dispatch",
                          "apply": "apply", "end_dispatch": "end_dispatch"}.get(phase)
            if not deadline_valid(deadline) or deadline.get("call") not in {phase_call, "trace_write"}:
                findings.append(f"invalid_execution_deadline:sequence={sequence}")
            elif deadline.get("call") == "trace_write":
                matches = [event for event in events
                           if event.get("evidence_write_id") == deadline.get("event_write_id")]
                if deadline.get("event_committed") is not True \
                        or deadline.get("event_reconstructible") is not True \
                        or len(matches) != 1 \
                        or matches[0].get("kind") != deadline.get("event_kind") \
                        or matches[0].get("sequence") != sequence - 1:
                    findings.append(f"invalid_late_evidence_binding:sequence={sequence}")
                else:
                    late_evidence = matches[0]
        elif deadline is not None:
            findings.append(f"unexpected_execution_deadline:sequence={sequence}")
        phases = {
            "begin_dispatch", "observation", "symbolic_verification", "policy_step",
            "context_validation", "trajectory_validation", "numeric_validation",
            "apply", "apply_evidence", "dispatch_terminal_observation",
            "post_dispatch_observation", "end_dispatch",
        }
        if not _integer(sequence) or intent is None or state is None \
                or intent.get("sequence", -1) >= sequence \
                or (fault.get("plan_version"), fault.get("action_id")) != (
                    intent.get("plan_version"), intent.get("action_id")) \
                or fault.get("action_digest") != intent.get("action_digest") \
                or fault.get("sim_t") != state.t or phase not in phases \
                or not isinstance(fault.get("error_type"), str) \
                or not fault.get("error_type", "").strip() \
                or fault.get("execution_outcome") != "UNKNOWN" \
                or not _integer(fault.get("cycles")) or not 0 <= fault["cycles"] <= 6:
            findings.append(f"invalid_execution_fault:sequence={sequence}")
            continue
        preceding = events[:sequence]
        previous = preceding[-1] if preceding else {}
        prior_applies = [event for event in preceding if event.get("kind") == "apply"
                         and event.get("dispatch_id") == dispatch_id]
        prior_numeric = [event for event in preceding if event.get("kind") == "safety2"
                         and event.get("dispatch_id") == dispatch_id]
        last_cycle = prior_applies[-1].get("cycle") if prior_applies else 0
        if not _integer(last_cycle):
            findings.append(f"execution_fault_prior_cycle_invalid:sequence={sequence}")
            last_cycle = -1
        if fault["cycles"] not in {last_cycle, last_cycle + 1} \
                or phase == "begin_dispatch" and (fault["cycles"] != 0 or prior_numeric) \
                or phase in {"apply", "apply_evidence"} and (
                    not prior_numeric or prior_numeric[-1].get("verdict") != "PASS"
                    or prior_numeric[-1].get("cycle") != fault["cycles"]):
            findings.append(f"execution_fault_cycle_mismatch:sequence={sequence}")
        boundary_valid = {
            "begin_dispatch": previous.get("kind") == "dispatch_intent",
            "symbolic_verification": previous.get("kind") == "context_gate"
                and previous.get("phase") == "pre_chunk" and previous.get("verdict") == "ALLOW",
            "policy_step": previous.get("kind") == "dynamic_safety_gate"
                and previous.get("verdict") == "PASS",
            "context_validation": previous.get("kind") in {"dynamic_safety_gate", "uncertainty_gate"}
                and previous.get("verdict") == "PASS",
            "trajectory_validation": previous.get("kind") == "context_gate"
                and previous.get("phase") == "pre_apply" and previous.get("verdict") == "ALLOW",
            "numeric_validation": previous.get("kind") == "context_gate"
                and previous.get("phase") == "pre_apply" and previous.get("verdict") == "ALLOW",
            "apply": previous.get("kind") == "safety2" and previous.get("verdict") == "PASS",
            "apply_evidence": previous.get("kind") == "safety2" and previous.get("verdict") == "PASS",
            "observation": previous.get("kind") in {"dispatch_intent", "apply"},
            "dispatch_terminal_observation": previous.get("kind") == "apply" and last_cycle == 6,
            "post_dispatch_observation": previous.get("kind") in {
                "context_gate", "uncertainty_gate", "interruption_gate"},
            "end_dispatch": previous.get("kind") == "world_state"
                and previous.get("phase") == "post_dispatch",
        }.get(phase, False)
        allowed_late_evidence = {
            "observation": {"world_state", "uncertainty_gate"},
            "context_validation": {"context_gate"},
            "symbolic_verification": {"dynamic_safety_gate"},
            "numeric_validation": {"safety2"},
            "apply_evidence": {"apply"},
            "dispatch_terminal_observation": {"world_state", "uncertainty_gate"},
            "post_dispatch_observation": {"world_state"},
        }
        if late_evidence is not None:
            expected_world_phase = {
                "observation": "dispatch",
                "dispatch_terminal_observation": "dispatch_terminal",
                "post_dispatch_observation": "post_dispatch",
            }.get(phase)
            verdict = late_evidence.get("verdict")
            semantic_boundary_valid = (
                late_evidence.get("kind") not in {
                    "uncertainty_gate", "dynamic_safety_gate", "safety2", "context_gate",
                }
                or verdict == ("ALLOW" if late_evidence.get("kind") == "context_gate"
                               else "PASS")
            )
            semantic_boundary_valid = (semantic_boundary_valid
                                       and deadline.get("event_reconstructible") is True)
            boundary_valid = (late_evidence is previous
                              and late_evidence.get("kind") in allowed_late_evidence.get(phase, set())
                              and late_evidence.get("dispatch_id", dispatch_id) == dispatch_id
                              and semantic_boundary_valid
                              and (late_evidence.get("kind") != "world_state"
                                   or late_evidence.get("phase") == expected_world_phase))
        if previous.get("kind") == "stop_safe_state" \
                and previous.get("intervention_id") == fault.get("prior_intervention_id"):
            boundary_valid = True
        if not boundary_valid or previous.get("dispatch_id", dispatch_id) != dispatch_id:
            findings.append(f"execution_fault_boundary_mismatch:sequence={sequence}")
        matching_stops = [event for event in stop_requests
                          if event.get("dispatch_id") == dispatch_id]
        following = events[sequence + 1:]
        if len(matching_stops) == 1 and fault.get("prior_intervention_id") != (
                matching_stops[0].get("intervention_id")
                if matching_stops[0].get("sequence", -1) < sequence else None):
            findings.append(f"execution_fault_prior_stop_mismatch:sequence={sequence}")
        if len(matching_stops) != 1 or (matching_stops[0].get("sequence", -1) > sequence
                and (not following or following[0].get("kind") != "stop_request"
                     or following[0].get("reason") != "execution_fault"
                     or following[0].get("dispatch_id") != dispatch_id)):
            findings.append(f"execution_fault_without_stop:sequence={sequence}")
        if any(event.get("kind") in {"world_state", "apply", "replan", "plan"}
               for event in following):
            findings.append(f"evidence_or_execution_after_fault:sequence={sequence}")
        terminals = [event for event in dispatches if event.get("dispatch_id") == dispatch_id]
        if len(terminals) != 1 or terminals[0].get("termination") != "INTERRUPTED" \
                or terminals[0].get("state_scope") != "LAST_KNOWN" \
                or terminals[0].get("state_digest") != fault.get("state_digest") \
                or terminals[0].get("cycles") != fault.get("cycles"):
            findings.append(f"execution_fault_terminal_mismatch:sequence={sequence}")
        if end.get("terminal_observation") != "UNAVAILABLE" \
                or end.get("state_digest") != fault.get("state_digest") \
                or end.get("goal_met") is not False or end.get("task_success") is not False \
                or end.get("aborted") is not True:
            findings.append(f"execution_fault_success_claim:sequence={sequence}")
    if len(observation_failures) > 1:
        findings.append("observation_failure_count_mismatch")
    for failure in observation_failures:
        sequence = failure.get("sequence")
        dispatch_id = failure.get("dispatch_id")
        intent = intent_by_id.get(dispatch_id)
        state = state_payloads.get(failure.get("state_digest"))
        following = events[sequence + 1:] if _integer(sequence) else []
        phase = failure.get("phase")
        if phase in {"planning", "pre_action"}:
            common_valid = (
                _integer(sequence)
                and failure.get("failure_kind") in {"observation_error", "invalid_payload"}
                and failure.get("dispatch_id") is None
                and failure.get("cycle") is None
                and failure.get("observed_state_digest") is None
                and failure.get("observed_sim_t") is None
            )
            if phase == "planning":
                scoped = (failure.get("sim_t") is None
                          and failure.get("state_digest") is None
                          and failure.get("plan_version") is None
                          and failure.get("action_id") is None
                          and failure.get("action_digest") is None
                          and not state_payloads and not plans and not replans
                          and not uncertainty_gates)
            else:
                ref = (failure.get("plan_version"), failure.get("action_id"))
                scoped = (state is not None and failure.get("sim_t") == state.t
                          and ref in action_catalog
                          and failure.get("action_digest") == action_catalog[ref]["action_digest"]
                          and catalog_sequences[ref] < sequence)
            if not common_valid or not scoped:
                findings.append(f"invalid_pre_dispatch_observation_failure:sequence={sequence}")
            if len(following) != 1 or following[0].get("kind") != "episode_end" \
                    or stop_requests or end.get("terminal_observation") != "UNAVAILABLE" \
                    or end.get("state_digest") != failure.get("state_digest") \
                    or end.get("sim_t") != failure.get("sim_t") \
                    or end.get("goal_met") is not False \
                    or end.get("task_success") is not False \
                    or end.get("aborted") is not True \
                    or end.get("stop_status") != "NOT_REQUESTED":
                findings.append(f"pre_dispatch_observation_terminal_mismatch:sequence={sequence}")
            continue
        if not _integer(sequence) or intent is None or state is None \
                or intent.get("sequence", -1) >= sequence \
                or (failure.get("plan_version"), failure.get("action_id")) != (
                    intent.get("plan_version"), intent.get("action_id")) \
                or failure.get("action_digest") != intent.get("action_digest") \
                or failure.get("sim_t") != state.t \
                or failure.get("phase") != "dispatch" \
                or failure.get("failure_kind") not in {
                    "observation_error", "invalid_payload", "stale_after_apply",
                } or not _integer(failure.get("cycle")) \
                or not 1 <= failure["cycle"] <= 7:
            findings.append(f"invalid_observation_failure:sequence={sequence}")
        if failure.get("failure_kind") == "stale_after_apply":
            observed = events[sequence - 1] if _integer(sequence) and sequence > 0 else {}
            prior_applies = [event for event in (events[:sequence] if _integer(sequence) else [])
                             if event.get("kind") == "apply"
                             and event.get("dispatch_id") == dispatch_id]
            if observed.get("kind") != "world_state" \
                    or observed.get("state_digest") != failure.get("observed_state_digest") \
                    or observed.get("sim_t") != failure.get("observed_sim_t") \
                    or not prior_applies \
                    or not _integer(failure.get("observed_sim_t")) \
                    or not _integer(prior_applies[-1].get("sim_t")) \
                    or failure["observed_sim_t"] > prior_applies[-1].get("sim_t", -1):
                findings.append(f"stale_observation_evidence_mismatch:sequence={sequence}")
        elif failure.get("observed_state_digest") is not None \
                or failure.get("observed_sim_t") is not None:
            findings.append(f"unexpected_observed_state_on_failure:sequence={sequence}")
        if not following or following[0].get("kind") != "stop_request" \
                or following[0].get("reason") != "observation_unavailable" \
                or following[0].get("dispatch_id") != dispatch_id:
            findings.append(f"observation_failure_without_stop:sequence={sequence}")
        if any(event.get("kind") in {"world_state", "apply", "replan", "plan"}
               for event in following):
            findings.append(f"evidence_or_execution_after_observation_loss:sequence={sequence}")
        terminals = [event for event in dispatches if event.get("dispatch_id") == dispatch_id]
        if len(terminals) != 1 or terminals[0].get("termination") != "INTERRUPTED" \
                or terminals[0].get("state_scope") != "LAST_KNOWN" \
                or terminals[0].get("state_digest") != failure.get("state_digest"):
            findings.append(f"observation_loss_terminal_mismatch:sequence={sequence}")
        if end.get("terminal_observation") != "UNAVAILABLE" \
                or end.get("state_digest") != failure.get("state_digest") \
                or end.get("goal_met") is not False \
                or end.get("task_success") is not False \
                or end.get("aborted") is not True:
            findings.append(f"observation_loss_success_claim:sequence={sequence}")
    if not observation_failures and not execution_faults and not reasoner_failures \
            and end.get("terminal_observation") != "OBSERVED":
        findings.append("terminal_observation_mismatch")
    if any(event.get("state_scope") != (
            "LAST_KNOWN" if any(
                event.get("dispatch_id") == failure.get("dispatch_id")
                for failure in observation_failures + execution_faults)
            else "OBSERVED") for event in dispatches):
        findings.append("dispatch_state_scope_mismatch")
    if len(stop_requests) > 1 or len(stop_cancels) != len(stop_requests) \
            or len(stop_states) != len(stop_requests):
        findings.append("stop_evidence_count_mismatch")
    stop_status = "NOT_REQUESTED"
    for request in stop_requests:
        sequence = request.get("sequence")
        dispatch_id = request.get("dispatch_id")
        generation = request.get("generation")
        intervention_id = request.get("intervention_id")
        if type(request.get("pending_execution_at_request")) is not bool:
            findings.append(f"invalid_stop_pending_evidence:sequence={sequence}")
        if request.get("reason") == "dynamic_symbolic_safety" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "dynamic_safety_gate"
                or events[sequence - 1].get("verdict") != "FAIL"
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"dynamic_stop_without_failed_gate:sequence={sequence}")
        if request.get("reason") == "observation_unavailable" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "observation_failure"
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"observation_stop_without_failure:sequence={sequence}")
        if request.get("reason") == "active_uncertainty" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "uncertainty_gate"
                or events[sequence - 1].get("phase") != "dispatch"
                or events[sequence - 1].get("verdict") != "FAIL"
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"uncertainty_stop_without_failed_gate:sequence={sequence}")
        if request.get("reason") == "active_numeric_safety" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "safety2"
                or events[sequence - 1].get("verdict") != "FAIL"
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"numeric_stop_without_failed_gate:sequence={sequence}")
        if request.get("reason") == "execution_fault" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "execution_fault"
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"fault_stop_without_fault:sequence={sequence}")
        if request.get("reason") == "active_context_changed" and (
                not _integer(sequence) or sequence == 0
                or events[sequence - 1].get("kind") != "context_gate"
                or events[sequence - 1].get("verdict") != "DENY"
                or events[sequence - 1].get("phase") not in {"pre_chunk", "pre_apply"}
                or events[sequence - 1].get("dispatch_id") != dispatch_id):
            findings.append(f"context_stop_without_denial:sequence={sequence}")
        if not _integer(sequence) or sequence + 2 >= len(events) \
                or not isinstance(intervention_id, str) or not intervention_id.strip() \
                or not isinstance(request.get("reason"), str) or not request["reason"].strip() \
                or not _integer(generation) \
                or not _integer(initial_stop_generation) \
                or request.get("previous_generation") != initial_stop_generation \
                or generation != initial_stop_generation + 1 \
                or dispatch_id not in intent_by_id \
                or intent_by_id[dispatch_id].get("sequence", -1) >= sequence:
            findings.append(f"invalid_stop_request:sequence={sequence}")
            continue
        cancel, safe = events[sequence + 1:sequence + 3]
        if cancel.get("kind") != "stop_cancel" or safe.get("kind") != "stop_safe_state" \
                or any(event.get("dispatch_id") != dispatch_id \
                       or event.get("intervention_id") != intervention_id \
                       or event.get("generation") != generation for event in (cancel, safe)) \
                or type(cancel.get("acknowledged")) is not bool:
            findings.append(f"invalid_stop_chain:sequence={sequence}")
            continue
        evidence = safe.get("evidence")
        for event, label in ((cancel, "cancel_dispatch"), (safe, "observe_safe_state")):
            if event.get("deadline") is not None and not deadline_valid(event["deadline"], label):
                findings.append(f"invalid_stop_deadline:sequence={sequence}")
        if cancel.get("deadline") is not None and cancel.get("acknowledged") is not False \
                or safe.get("deadline") is not None and evidence is not None:
            findings.append(f"stop_deadline_outcome_mismatch:sequence={sequence}")
        # The registry can gain an already-admitted racing callback between the
        # request and proof. Its presence must suppress confirmation, not make
        # otherwise conservative SAFE_UNCONFIRMED evidence unreplayable.
        if type(safe.get("pending_execution")) is not bool:
            findings.append(f"invalid_safe_pending_evidence:sequence={sequence}")
        try:
            observed = StopStateEvidence(**evidence) if isinstance(evidence, dict) else None
            if evidence is not None and (observed is None or asdict(observed) != evidence):
                raise ValueError("noncanonical evidence")
        except (TypeError, ValueError):
            findings.append(f"invalid_stop_state:sequence={sequence}")
            observed = None
        if observed is not None and (observed.dispatch_id != dispatch_id
                                     or observed.generation != generation):
            findings.append(f"stop_state_scope_mismatch:sequence={sequence}")
        confirmed = bool(safe.get("pending_execution") is False
                         and cancel["acknowledged"] and observed is not None
                         and observed.dispatch_id == dispatch_id
                         and observed.generation == generation
                         and observed.motion_stopped and not observed.active_motion_command
                         and observed.safe_condition_met
                         and observed.evidence_kind == "synthetic_mock")
        stop_status = "SAFE_CONFIRMED_MOCK" if confirmed else "SAFE_UNCONFIRMED"
        if safe.get("status") != stop_status:
            findings.append(f"stop_status_mismatch:sequence={sequence}")
        terminal = [event for event in dispatches if event.get("dispatch_id") == dispatch_id]
        if len(terminal) != 1 or terminal[0].get("termination") != "INTERRUPTED" \
                or terminal[0].get("intervention_id") != intervention_id \
                or terminal[0].get("stop_status") != stop_status \
                or terminal[0].get("sequence", -1) <= safe.get("sequence", -1):
            findings.append(f"stop_without_interrupted_terminal:sequence={sequence}")
        if any(event.get("kind") in {"apply", "replan"} and
               (event.get("dispatch_id") == dispatch_id or event.get("kind") == "replan")
               for event in events[sequence + 1:]):
            findings.append(f"execution_after_stop:sequence={sequence}")
        matching_gates = [event for event in interruption_gates
                          if event.get("dispatch_id") == dispatch_id]
        if len(matching_gates) != 1 or len(terminal) != 1 \
                or matching_gates[0].get("sequence", -1) <= safe.get("sequence", -1) \
                or matching_gates[0].get("sequence", -1) >= terminal[0].get("sequence", -1) \
                or matching_gates[0].get("intervention_id") != intervention_id \
                or matching_gates[0].get("observed_generation") != generation \
                or matching_gates[0].get("expected_generation") != initial_stop_generation \
                or matching_gates[0].get("safe_status") != stop_status \
                or matching_gates[0].get("stopped") is not True \
                or matching_gates[0].get("verdict") != "INTERRUPT":
            findings.append(f"invalid_interruption_gate:sequence={sequence}")
    if len(interruption_gates) != len(stop_requests) \
            or any(event.get("termination") == "INTERRUPTED" for event in dispatches) \
            != bool(stop_requests):
        findings.append("interruption_evidence_mismatch")
    if end.get("stop_status") != stop_status \
            or any(event.get("stop_status") != (stop_status if event.get("termination") == "INTERRUPTED"
                                                else "NOT_REQUESTED") for event in dispatches):
        findings.append("terminal_stop_status_mismatch")
    for gate in context_gates:
        sequence = gate.get("sequence")
        if not _integer(sequence) or gate.get("phase") == "pre_action":
            continue
        intent = intent_by_id.get(gate.get("dispatch_id"))
        if intent is None or intent.get("sequence", -1) >= sequence:
            findings.append(f"context_without_dispatch_intent:sequence={sequence}")
        following = events[sequence + 1:]
        if gate.get("phase") == "pre_apply" and gate.get("verdict") == "ALLOW":
            if not following or following[0].get("kind") not in {"safety2", "execution_fault"} \
                    or following[0].get("dispatch_id") != gate.get("dispatch_id"):
                findings.append(f"pre_apply_without_numeric_check:sequence={sequence}")
        if gate.get("phase") == "pre_chunk":
            preceding = events[:sequence]
            if not preceding or preceding[-1].get("kind") != "uncertainty_gate" \
                    or preceding[-1].get("phase") != "dispatch" \
                    or preceding[-1].get("verdict") != "PASS" \
                    or preceding[-1].get("dispatch_id") != gate.get("dispatch_id"):
                findings.append(f"pre_chunk_without_uncertainty_allow:sequence={sequence}")
        if gate.get("verdict") == "DENY":
            if not following or following[0].get("kind") != "stop_request" \
                    or following[0].get("reason") != "active_context_changed" \
                    or following[0].get("dispatch_id") != gate.get("dispatch_id"):
                findings.append(f"context_denial_without_stop:sequence={sequence}")
    for event in (item for item in events if item.get("kind") == "safety2"):
        sequence = event.get("sequence")
        if event.get("verdict") == "FAIL":
            following = events[sequence + 1:] if _integer(sequence) else []
            terminals = [terminal for terminal in dispatches
                         if terminal.get("dispatch_id") == event.get("dispatch_id")]
            if not following or following[0].get("kind") != "stop_request" \
                    or following[0].get("reason") != "active_numeric_safety" \
                    or following[0].get("dispatch_id") != event.get("dispatch_id") \
                    or len(terminals) != 1 \
                    or terminals[0].get("termination") != "INTERRUPTED":
                findings.append(f"numeric_rejection_without_stop_terminal:sequence={sequence}")
        if not _integer(sequence) or sequence == 0 \
                or events[sequence - 1].get("kind") != "context_gate" \
                or events[sequence - 1].get("phase") != "pre_apply" \
                or events[sequence - 1].get("verdict") != "ALLOW" \
                or events[sequence - 1].get("dispatch_id") != event.get("dispatch_id"):
            findings.append(f"numeric_without_context_allow:sequence={sequence}")
    for gate in (item for item in uncertainty_gates
                 if item.get("phase") == "dispatch" and item.get("verdict") == "PASS"):
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = events[sequence + 1:]
        prior = events[sequence - 1] if sequence else {}
        if prior.get("kind") == "world_state" and prior.get("phase") == "dispatch":
            next_event = following[0] if following else {}
            continued = (next_event.get("kind") == "context_gate"
                         and next_event.get("phase") == "pre_chunk"
                         and next_event.get("dispatch_id") == gate.get("dispatch_id"))
            receipt_fault = (next_event.get("kind") == "execution_fault"
                             and next_event.get("phase") == "observation"
                             and next_event.get("dispatch_id") == gate.get("dispatch_id")
                             and isinstance(next_event.get("deadline"), dict)
                             and next_event["deadline"].get("call") == "trace_write"
                             and next_event["deadline"].get("event_committed") is True
                             and next_event["deadline"].get("event_write_id")
                             == gate.get("evidence_write_id"))
            if not continued and not receipt_fault:
                findings.append(f"dispatch_uncertainty_without_context:sequence={sequence}")

    action_event_kinds = {"safety1", "safety2", "apply", "dispatch", "divergence",
                          "dynamic_safety_gate", "execution_fault"}
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

    for gate in capability_gates:
        ref = (gate.get("plan_version"), gate.get("action_id"))
        sequence = gate.get("sequence")
        if not _integer(sequence):
            findings.append("invalid_capability_gate_sequence")
            continue
        if not _integer(capability_sequence) or capability_sequence >= sequence:
            findings.append(f"capability_reference_before_declaration:sequence={sequence}")
        if ref not in action_catalog:
            findings.append(f"unknown_capability_action_reference:sequence={sequence}")
            continue
        action = action_catalog[ref]
        if gate.get("action_digest") != action["action_digest"] \
                or gate.get("operation") != action["verb"]:
            findings.append(f"capability_action_mismatch:sequence={sequence}")
        if gate.get("capability_digest") != capability_digest:
            findings.append(f"capability_reference_mismatch:sequence={sequence}")
        if capabilities is not None:
            try:
                required = VERB_TABLE[Verb(action["verb"])].required_resources
            except (KeyError, ValueError):
                findings.append(f"invalid_capability_operation:sequence={sequence}")
                continue
            if gate.get("required_resources") != sorted(required):
                findings.append(f"capability_resource_mismatch:sequence={sequence}")
            reasons = list(capabilities.rejection_reasons(action["verb"], required))
            expected = "DENY" if reasons else "ALLOW"
            if gate.get("reasons") != reasons or gate.get("verdict") != expected:
                findings.append(f"invalid_capability_gate_verdict:sequence={sequence}")
        if gate.get("verdict") == "DENY":
            following = [event for event in events[sequence + 1:]
                         if event.get("kind") != "world_state"]
            if len(following) != 1 or following[0].get("kind") != "episode_end":
                findings.append("execution_after_capability_denial")

    for gate in (event for event in uncertainty_gates
                 if event.get("phase") == "pre_action"
                 and event.get("verdict") == "PASS"):
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = events[sequence + 1:]
        if not following or following[0].get("kind") != "context_gate" \
                or following[0].get("phase") != "pre_action":
            findings.append(f"missing_context_gate:sequence={sequence}")
            continue
        context_gate = following[0]
        if (context_gate.get("plan_version"), context_gate.get("action_id")) != (
                gate.get("plan_version"), gate.get("action_id")):
            findings.append(f"context_gate_action_order_mismatch:sequence={sequence}")
        if context_gate.get("verdict") == "DENY":
            tail = [event for event in following[1:] if event.get("kind") != "world_state"]
            if len(tail) != 1 or tail[0].get("kind") != "episode_end":
                findings.append("execution_after_context_denial")
            continue
        if len(following) < 2 or following[1].get("kind") != "capability_gate":
            findings.append(
                f"missing_capability_gate:sequence={context_gate.get('sequence')}"
            )
            continue
        capability_gate = following[1]
        if (capability_gate.get("plan_version"), capability_gate.get("action_id")) != (
                gate.get("plan_version"), gate.get("action_id")):
            findings.append(f"capability_gate_action_order_mismatch:sequence={sequence}")

    for gate in (event for event in capability_gates if event.get("verdict") == "ALLOW"):
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = events[sequence + 1:]
        if not following or following[0].get("kind") != "permission_gate":
            findings.append(f"capability_allow_without_permission:sequence={sequence}")
            continue
        permission_gate = following[0]
        if (permission_gate.get("plan_version"), permission_gate.get("action_id")) != (
                gate.get("plan_version"), gate.get("action_id")):
            findings.append(f"capability_permission_action_mismatch:sequence={sequence}")

    for gate in permission_gates:
        ref = (gate.get("plan_version"), gate.get("action_id"))
        sequence = gate.get("sequence")
        if not _integer(sequence):
            findings.append("invalid_permission_gate_sequence")
            continue
        if not _integer(permission_sequence) or permission_sequence >= sequence:
            findings.append(f"permission_reference_before_declaration:sequence={sequence}")
        if ref not in action_catalog:
            findings.append(f"unknown_permission_action_reference:sequence={sequence}")
            continue
        action = action_catalog[ref]
        if gate.get("action_digest") != action["action_digest"] \
                or gate.get("operation") != action["verb"]:
            findings.append(f"permission_action_mismatch:sequence={sequence}")
        if gate.get("permission_digest") != permission_digest:
            findings.append(f"permission_reference_mismatch:sequence={sequence}")
        try:
            required = VERB_TABLE[Verb(action["verb"])].required_resources
        except (KeyError, ValueError):
            findings.append(f"invalid_permission_operation:sequence={sequence}")
            continue
        if gate.get("required_resources") != sorted(required):
            findings.append(f"permission_resource_mismatch:sequence={sequence}")
        embodiment_id = capabilities.embodiment_id if capabilities is not None else None
        if gate.get("task_id") != actual_task_id \
                or gate.get("embodiment_id") != embodiment_id:
            findings.append(f"permission_scope_mismatch:sequence={sequence}")
        if permission is not None and embodiment_id is not None:
            reasons = list(permission.rejection_reasons(
                task_id=actual_task_id,
                embodiment_id=embodiment_id,
                operation=action["verb"],
                resources=required,
            ))
            expected = "DENY" if reasons else "ALLOW"
            if gate.get("reasons") != reasons or gate.get("verdict") != expected:
                findings.append(f"invalid_permission_gate_verdict:sequence={sequence}")
        preceding = events[:sequence]
        if not preceding or preceding[-1].get("kind") != "capability_gate" \
                or preceding[-1].get("verdict") != "ALLOW":
            findings.append(f"permission_without_capability_allow:sequence={sequence}")
        if gate.get("verdict") == "DENY":
            following = [event for event in events[sequence + 1:]
                         if event.get("kind") != "world_state"]
            if len(following) != 1 or following[0].get("kind") != "episode_end":
                findings.append("execution_after_permission_denial")

    for gate in (event for event in permission_gates if event.get("verdict") == "ALLOW"):
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = events[sequence + 1:]
        if not following or following[0].get("kind") != "approval_gate":
            findings.append(f"permission_allow_without_approval:sequence={sequence}")
            continue
        approval_gate = following[0]
        if (approval_gate.get("plan_version"), approval_gate.get("action_id")) != (
                gate.get("plan_version"), gate.get("action_id")):
            findings.append(f"permission_approval_action_mismatch:sequence={sequence}")

    for gate in approval_gates:
        sequence = gate.get("sequence")
        version = gate.get("plan_version")
        ref = (version, gate.get("action_id"))
        if not _integer(sequence):
            findings.append("invalid_approval_gate_sequence")
            continue
        preceding = events[:sequence]
        if not preceding or preceding[-1].get("kind") != "permission_gate" \
                or preceding[-1].get("verdict") != "ALLOW":
            findings.append(f"approval_without_permission_allow:sequence={sequence}")
        if ref not in action_catalog:
            findings.append(f"unknown_approval_action_reference:sequence={sequence}")
            continue
        action = action_catalog[ref]
        try:
            expected_scope = ApprovalScope(
                run_id=run_id, task_id=task_id,
                task_revision=task_revision, task_digest=task_digest,
                plan_id=bound_plan_id, plan_version=version,
                plan_digest=plan_digests[version],
                action_id=action["action_id"],
                action_digest=action["action_digest"],
            )
        except (KeyError, TypeError, ValueError):
            findings.append(f"invalid_approval_scope_reference:sequence={sequence}")
            continue
        if gate.get("scope") != expected_scope.as_record() \
                or gate.get("action_digest") != action["action_digest"]:
            findings.append(f"approval_scope_mismatch:sequence={sequence}")
        record = gate.get("approval")
        if record is None:
            status = gate.get("approval_status")
            if status not in {"missing", "invalid"}:
                findings.append(f"invalid_approval_status:sequence={sequence}")
            reasons = (["invalid_approval_decision"] if status == "invalid"
                       else ["approval_missing"])
            if gate.get("approval_digest") is not None:
                findings.append(f"approval_digest_without_decision:sequence={sequence}")
        else:
            if gate.get("approval_status") != "present":
                findings.append(f"invalid_approval_status:sequence={sequence}")
            try:
                if not isinstance(record, dict) or set(record) != {
                    "decision_id", "approver_id", "revision", "evidence_kind",
                    "verdict", "scope",
                } or not isinstance(record["scope"], dict):
                    raise ValueError("invalid approval fields")
                decision = ApprovalDecision(
                    decision_id=record["decision_id"],
                    approver_id=record["approver_id"],
                    revision=record["revision"],
                    evidence_kind=record["evidence_kind"],
                    verdict=record["verdict"],
                    scope=ApprovalScope(**record["scope"]),
                )
                if record != decision.as_record():
                    findings.append(f"noncanonical_approval_decision:sequence={sequence}")
                if gate.get("approval_digest") != _record_digest(record):
                    findings.append(f"approval_digest_mismatch:sequence={sequence}")
                reasons = list(decision.rejection_reasons(expected_scope))
            except (KeyError, TypeError, ValueError):
                findings.append(f"invalid_approval_decision:sequence={sequence}")
                reasons = ["invalid_approval_decision"]
        if gate.get("reasons") != reasons \
                or gate.get("verdict") != ("DENY" if reasons else "ALLOW"):
            findings.append(f"approval_gate_verdict_mismatch:sequence={sequence}")
        following = events[sequence + 1:]
        if gate.get("verdict") == "DENY":
            terminal = [event for event in following if event.get("kind") != "world_state"]
            if len(terminal) != 1 or terminal[0].get("kind") != "episode_end":
                findings.append("execution_after_approval_denial")
        elif not following or following[0].get("kind") != "numeric_profile_gate" \
                or (following[0].get("plan_version"), following[0].get("action_id")) != ref:
            findings.append(f"approval_allow_without_numeric_profile:sequence={sequence}")

    used_authorization_decisions: set[str] = set()
    used_authorization_dispatches: set[tuple[str, str]] = set()
    for gate in authorization_gates:
        sequence = gate.get("sequence")
        ref = (gate.get("plan_version"), gate.get("action_id"))
        if not _integer(sequence):
            findings.append("invalid_authorization_gate_sequence")
            continue
        preceding = events[:sequence]
        if not preceding or preceding[-1].get("kind") != "safety1" \
                or preceding[-1].get("verdict") != "PASS" \
                or (preceding[-1].get("plan_version"), preceding[-1].get("action_id")) != ref:
            findings.append(f"authorization_without_safety_allow:sequence={sequence}")
        if ref not in action_catalog:
            findings.append(f"unknown_authorization_action:sequence={sequence}")
            continue
        prior_approval = next((item for item in reversed(preceding)
                               if item.get("kind") == "approval_gate"), None)
        if prior_approval is None or prior_approval.get("verdict") != "ALLOW" \
                or (prior_approval.get("plan_version"), prior_approval.get("action_id")) != ref:
            findings.append(f"authorization_without_approval:sequence={sequence}")
        context_record = gate.get("context")
        guard_state = gate.get("guard_state")
        if not isinstance(context_record, dict) or not isinstance(guard_state, dict):
            findings.append(f"invalid_authorization_context:sequence={sequence}")
            continue
        try:
            context = DispatchContext(**context_record)
            if asdict(context) != context_record \
                    or gate.get("context_digest") != _record_digest(context_record):
                findings.append(f"authorization_context_digest_mismatch:sequence={sequence}")
        except (TypeError, ValueError):
            findings.append(f"invalid_authorization_context:sequence={sequence}")
            continue
        action = action_catalog[ref]
        expected_refs = {
            "run_id": run_id,
            "task_revision": task_digest,
            "plan_revision": plan_digests.get(ref[0]),
            "action_id": action["action_id"],
            "dispatch_id": gate.get("dispatch_id"),
            "action_digest": action["action_digest"],
            "state_revision": gate.get("state_digest"),
            "capability_revision": capability_digest,
            "permission_revision": permission_digest,
            "approval_revision": prior_approval.get("approval_digest") if prior_approval else None,
            "constraints_revision": constraints_digest,
            "authority_epoch": authority_epoch,
        }
        if any(context_record.get(key) != value for key, value in expected_refs.items()) \
                or gate.get("action_digest") != action["action_digest"]:
            findings.append(f"authorization_context_reference_mismatch:sequence={sequence}")
        if not isinstance(guard_state.get("stopped"), bool) \
                or not isinstance(guard_state.get("decision_used"), bool) \
                or not isinstance(guard_state.get("dispatch_used"), bool) \
                or not isinstance(guard_state.get("authority_epoch"), str) \
                or not _integer(guard_state.get("stop_generation")) \
                or set(guard_state) != {
                    "stopped", "decision_used", "dispatch_used",
                    "authority_epoch", "stop_generation",
                }:
            findings.append(f"invalid_authorization_guard_state:sequence={sequence}")
            continue
        record = gate.get("decision")
        decision = None
        if record is None:
            status = gate.get("decision_status")
            if status not in {"missing", "invalid"} or gate.get("decision_digest") is not None:
                findings.append(f"invalid_authorization_decision_status:sequence={sequence}")
            expected_reason = ("clock_unavailable" if gate.get("now") is None
                               else "invalid_decision" if status == "invalid"
                               else "decision_missing")
        else:
            if gate.get("decision_status") != "present":
                findings.append(f"invalid_authorization_decision_status:sequence={sequence}")
            try:
                if not isinstance(record, dict) or set(record) != {
                    "decision_id", "context", "verdict", "issued_at", "expires_at",
                } or not isinstance(record["context"], dict):
                    raise ValueError("invalid decision fields")
                decision = SafetyDecision(
                    decision_id=record["decision_id"],
                    context=DispatchContext(**record["context"]),
                    verdict=record["verdict"], issued_at=record["issued_at"],
                    expires_at=record["expires_at"],
                )
                if asdict(decision) != record:
                    findings.append(f"noncanonical_authorization_decision:sequence={sequence}")
                if gate.get("decision_digest") != _record_digest(record):
                    findings.append(f"authorization_decision_digest_mismatch:sequence={sequence}")
                now = gate.get("now")
                if not _number(now):
                    findings.append(f"invalid_authorization_clock:sequence={sequence}")
                    expected_reason = "outside_validity_window"
                else:
                    expected_reason = admission_rejection_reason(
                        decision, context, now=now,
                        stopped=guard_state["stopped"],
                        authority_epoch=guard_state["authority_epoch"],
                        stop_generation=guard_state["stop_generation"],
                        decision_used=guard_state["decision_used"],
                        dispatch_used=guard_state["dispatch_used"],
                    )
                if guard_state["decision_used"] != (
                        decision.decision_id in used_authorization_decisions) \
                        or guard_state["dispatch_used"] != (
                            (context.run_id, context.dispatch_id)
                            in used_authorization_dispatches):
                    findings.append(f"authorization_guard_history_mismatch:sequence={sequence}")
            except (KeyError, TypeError, ValueError):
                findings.append(f"invalid_authorization_decision:sequence={sequence}")
                expected_reason = "invalid_decision"
        if gate.get("reason") != expected_reason \
                or gate.get("verdict") != ("ALLOW" if expected_reason is None else "DENY"):
            findings.append(f"authorization_gate_verdict_mismatch:sequence={sequence}")
        following = events[sequence + 1:]
        if gate.get("verdict") == "ALLOW":
            if guard_state["stopped"] or context.stop_generation != guard_state["stop_generation"] \
                    or guard_state["authority_epoch"] != authority_epoch \
                    or context.stop_generation != initial_stop_generation:
                findings.append(f"authorization_authority_mismatch:sequence={sequence}")
            if decision is not None:
                used_authorization_decisions.add(decision.decision_id)
                used_authorization_dispatches.add((context.run_id, context.dispatch_id))
            if not following or following[0].get("kind") != "dispatch_intent" \
                    or following[0].get("dispatch_id") != context.dispatch_id:
                findings.append(f"authorization_allow_without_intent:sequence={sequence}")
        else:
            terminal = [event for event in following if event.get("kind") != "world_state"]
            if len(terminal) != 1 or terminal[0].get("kind") != "episode_end":
                findings.append("execution_after_authorization_denial")

    for gate in profile_gates:
        sequence = gate.get("sequence")
        ref = (gate.get("plan_version"), gate.get("action_id"))
        if not _integer(sequence):
            findings.append("invalid_numeric_profile_gate_sequence")
            continue
        if not _integer(profile_sequence) or profile_sequence >= sequence:
            findings.append(f"numeric_profile_before_declaration:sequence={sequence}")
        if ref not in action_catalog or gate.get("action_digest") != action_catalog[ref]["action_digest"]:
            findings.append(f"numeric_profile_action_mismatch:sequence={sequence}")
        if gate.get("profile_digest") != profile_digest \
                or gate.get("capability_digest") != capability_digest:
            findings.append(f"numeric_profile_reference_mismatch:sequence={sequence}")
        reasons = []
        if capabilities is not None and profile is not None:
            if capabilities.limits_ref != profile.ref:
                reasons.append("limits_ref_mismatch")
            if capabilities.embodiment_id != profile.embodiment_id:
                reasons.append("profile_embodiment_mismatch")
            if gate.get("reasons") != reasons or gate.get("verdict") != (
                    "DENY" if reasons else "ALLOW"):
                findings.append(f"invalid_numeric_profile_gate_verdict:sequence={sequence}")
        preceding = events[:sequence]
        if not preceding or preceding[-1].get("kind") != "approval_gate" \
                or preceding[-1].get("verdict") != "ALLOW":
            findings.append(f"numeric_profile_without_approval:sequence={sequence}")
        following = events[sequence + 1:]
        if gate.get("verdict") == "DENY":
            terminal = [event for event in following if event.get("kind") != "world_state"]
            if len(terminal) != 1 or terminal[0].get("kind") != "episode_end":
                findings.append("execution_after_numeric_profile_denial")
        elif not following or following[0].get("kind") != "safety1" \
                or (following[0].get("plan_version"), following[0].get("action_id")) != ref:
            findings.append(f"numeric_profile_allow_without_safety:sequence={sequence}")

    numeric_by_cycle: dict[tuple[int, str, int], dict[str, Any]] = {}
    for event in safety_events["numeric"]:
        sequence = event.get("sequence")
        key = (event.get("plan_version"), event.get("action_id"), event.get("cycle"))
        if key in numeric_by_cycle:
            findings.append(f"duplicate_numeric_decision:sequence={sequence}")
        numeric_by_cycle[key] = event
        if event.get("profile_digest") != profile_digest:
            findings.append(f"numeric_profile_reference_mismatch:sequence={sequence}")
        record = event.get("trajectory")
        if not isinstance(record, dict):
            findings.append(f"missing_numeric_trajectory:sequence={sequence}")
            continue
        try:
            traj = Trajectory.model_validate(record)
            canonical = traj.model_dump(mode="json")
            if record != canonical:
                findings.append(f"noncanonical_numeric_trajectory:sequence={sequence}")
            encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":"))
            if event.get("trajectory_digest") != hashlib.sha256(encoded.encode()).hexdigest():
                findings.append(f"numeric_trajectory_digest_mismatch:sequence={sequence}")
            if traj.action_id != event.get("action_id"):
                findings.append(f"numeric_trajectory_action_mismatch:sequence={sequence}")
            state = state_payloads.get(event.get("state_digest"))
            if profile is not None and state is not None:
                verdict = NumericSafetyVerifier(profile).verify(
                    traj, state, expected_action_id=event.get("action_id"),
                )
                if event.get("verdict") != verdict.verdict \
                        or event.get("checked") != verdict.checked \
                        or event.get("violations") != [v.model_dump() for v in verdict.violations]:
                    findings.append(f"numeric_verdict_mismatch:sequence={sequence}")
        except (TypeError, ValueError):
            findings.append(f"invalid_numeric_trajectory:sequence={sequence}")

    for event in (item for item in events if item.get("kind") == "apply"):
        sequence = event.get("sequence")
        key = (event.get("plan_version"), event.get("action_id"), event.get("cycle"))
        numeric = numeric_by_cycle.get(key)
        if numeric is None or numeric.get("verdict") != "PASS" \
                or numeric.get("sequence") != sequence - 1:
            findings.append(f"apply_without_numeric_allow:sequence={sequence}")
            continue
        trajectory = numeric.get("trajectory")
        if event.get("profile_digest") != profile_digest \
                or event.get("trajectory_digest") != numeric.get("trajectory_digest") \
                or not isinstance(trajectory, dict) \
                or event.get("terminal") != trajectory.get("terminal") \
                or event.get("max_velocity") != trajectory.get("max_velocity"):
            findings.append(f"apply_numeric_binding_mismatch:sequence={sequence}")

    for gate in failed_uncertainty_gates:
        sequence = gate.get("sequence")
        if not _integer(sequence):
            continue
        following = [event for event in events[sequence + 1:]
                     if event.get("kind") != "world_state"]
        if gate.get("phase") == "dispatch":
            expected_kinds = ["stop_request", "stop_cancel", "stop_safe_state",
                             "interruption_gate", "dispatch", "episode_end"]
            terminal_index = 4
            if any(fault.get("dispatch_id") == gate.get("dispatch_id")
                   for fault in execution_faults):
                expected_kinds.insert(4, "execution_fault")
                terminal_index = 5
            if [event.get("kind") for event in following] != expected_kinds \
                    or following[0].get("reason") != "active_uncertainty" \
                    or following[0].get("dispatch_id") != gate.get("dispatch_id") \
                    or following[terminal_index].get("termination") != "INTERRUPTED" \
                    or following[terminal_index].get("dispatch_id") != gate.get("dispatch_id"):
                findings.append("uncertainty_dispatch_without_stop_terminal")
        elif len(following) != 1 or following[0].get("kind") != "episode_end":
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
    requests = [event for event in events if event.get("kind") == "reasoner_request"]
    outcomes = plans + replans + reasoner_failures
    request_ids = [event.get("request_id") for event in requests]
    if any(not isinstance(value, str) or not value.strip() for value in request_ids) \
            or len(set(str(value) for value in request_ids)) != len(request_ids):
        findings.append("invalid_reasoner_request_identity")
    for outcome in outcomes:
        matches = [request for request in requests
                   if request.get("request_id") == outcome.get("request_id")]
        if len(matches) != 1:
            findings.append("reasoner_outcome_without_request")
    for request in requests:
        sequence = request.get("sequence", -1)
        phase = request.get("phase")
        matches = [outcome for outcome in outcomes
                   if outcome.get("request_id") == request.get("request_id")]
        if len(matches) != 1:
            findings.append("reasoner_request_outcome_count")
            continue
        outcome = matches[0]
        if phase not in {"plan", "replan"} \
                or outcome.get("sequence") != sequence + 1 \
                or (outcome.get("kind") != "reasoner_failure" and outcome.get("kind") != phase) \
                or (outcome.get("kind") == "reasoner_failure" and outcome.get("phase") != phase) \
                or outcome.get("state_digest") != request.get("state_digest") \
                or outcome.get("sim_t") != request.get("sim_t") \
                or request.get("task_digest") != task_digest \
                or request.get("task_revision") != task_revision:
            findings.append("reasoner_request_binding_mismatch")
        if len(authority_events) != 1 \
                or request.get("authority_epoch") != authority_events[0].get("authority_epoch") \
                or request.get("stop_generation") != authority_events[0].get("stop_generation") \
                or not _integer(request.get("stop_generation")):
            findings.append("reasoner_request_authority_mismatch")
        previous = request.get("previous_plan")
        accepted_before = [event for event in plans + replans if event.get("sequence", -1) < sequence]
        if phase == "plan":
            if accepted_before or previous is not None \
                    or request.get("previous_plan_digest") is not None or request.get("trigger") is not None:
                findings.append("invalid_initial_reasoner_request")
        elif phase == "replan":
            if not accepted_before or previous != accepted_before[-1].get("plan") \
                    or request.get("previous_plan_digest") != accepted_before[-1].get("plan_digest"):
                findings.append("reasoner_previous_plan_mismatch")
            trigger = request.get("trigger")
            if not isinstance(trigger, dict) or not accepted_before \
                    or trigger.get("plan_version") != accepted_before[-1].get("version") \
                    or (trigger.get("plan_version"), trigger.get("action_id")) not in action_catalog:
                findings.append("reasoner_trigger_mismatch")
            if outcome.get("kind") == "replan" and outcome.get("trigger") != trigger:
                findings.append("reasoner_trigger_mismatch")
            preceding = events[sequence - 1] if sequence > 0 else {}
            try:
                parsed_trigger = Divergence.model_validate(trigger)
                if preceding.get("kind") == "divergence":
                    valid_trigger = (
                        parsed_trigger.action_id == preceding.get("action_id")
                        and parsed_trigger.plan_version == preceding.get("plan_version")
                        and parsed_trigger.magnitude == preceding.get("magnitude")
                        and [str(item) for item in parsed_trigger.unmet] == preceding.get("unmet")
                        and [str(item) for item in parsed_trigger.surprise] == preceding.get("surprise"))
                elif preceding.get("kind") == "safety1" and preceding.get("verdict") == "FAIL":
                    valid_trigger = parsed_trigger == Divergence(
                        action_id=preceding.get("action_id"), plan_version=preceding.get("plan_version"))
                else:
                    valid_trigger = False
                if not valid_trigger or preceding.get("state_digest") != request.get("state_digest"):
                    findings.append("reasoner_trigger_causal_mismatch")
            except (TypeError, ValueError):
                findings.append("invalid_reasoner_trigger")
        if outcome.get("kind") == "reasoner_failure":
            deadline = outcome.get("deadline")
            if not isinstance(outcome.get("exception_type"), str) or not outcome.get("exception_type") \
                    or (outcome.get("exception_type") == "CoreCallTimeout") != (deadline is not None) \
                    or deadline is not None and (not deadline_valid(deadline, "reasoner_" + phase)
                        or not _number(outcome.get("latency_ms"))
                        or outcome["latency_ms"] + 0.01 < deadline["elapsed_s"] * 1000):
                findings.append("invalid_reasoner_failure")
            if events[outcome.get("sequence", -1) + 1:] != [end] \
                    or end.get("terminal_observation") != "UNAVAILABLE" \
                    or end.get("state_digest") != outcome.get("state_digest") \
                    or end.get("aborted") is not True or end.get("goal_met") is not False \
                    or end.get("task_success") is not False:
                findings.append("reasoner_failure_terminal_mismatch")
    if len(reasoner_failures) > 1:
        findings.append("reasoner_failure_count")
    planning_events = outcomes
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
        "replans": len(replans) + sum(event.get("phase") == "replan" for event in reasoner_failures),
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
            preceding = [event for event in events[:sequence]
                         if event.get("kind") != "reasoner_request"][-2:]
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
            after_convergence = [event for event in events[sequence + 1:]
                                 if event.get("kind") != "world_state"]
            if len(after_convergence) != 1 \
                    or after_convergence[0].get("kind") != "episode_end":
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
        expected_success = (False if observation_failures or execution_faults or reasoner_failures else
                            reconstructed["aborted"] if expect_abort
                            else reconstructed["goal_met"])
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
