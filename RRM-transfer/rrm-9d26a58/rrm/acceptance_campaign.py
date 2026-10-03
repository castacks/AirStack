"""All-attempt, mock-only acceptance export and independent offline verification."""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Event

from .acceptance_fixtures import (LIMITS, SCENARIOS, FixturePolicy, FixtureReasoner,
                                  FixtureTracer, FixtureWorld, case_by_id, safety_rule, task_and_labels)
from .benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
                        mock_admission, mock_permission)
from .benchmark_evidence import EpisodeReplay, _git_value, _sha256, _source_digest, replay_trace
from .core_admission import CoreAdmission
from .loop import _capability_record, run
from .safety import NumericSafetyVerifier, SafetyVerifier
from .schema import RunMetrics
from .safety_event_evidence import aggregate_scores, score_events

SCHEMA = "rrm-core-acceptance/v2"
ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "scripts" / "core_acceptance.py"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def specification(case_ids=None, repetitions=1, attempt_timeout_s=5.0):
    if type(repetitions) is not int or not 1 <= repetitions <= 100:
        raise ValueError("repetitions must be an integer in [1,100]")
    if type(attempt_timeout_s) not in {int, float} or not math.isfinite(attempt_timeout_s) \
            or attempt_timeout_s <= 0:
        raise ValueError("harness timeout must be finite positive seconds")
    ids = list(case_ids) if case_ids is not None else [case.id for case in SCENARIOS]
    if not ids or len(ids) != len(set(ids)) or any(case_id not in {c.id for c in SCENARIOS} for case_id in ids):
        raise ValueError("scenario IDs must be nonempty, known and unique")
    spec = {
        "schema_version": SCHEMA, "scope": "mock_component_acceptance",
        "case_ids": ids, "cases": [case_by_id(case_id).record() for case_id in ids],
        "safety_ground_truth": {case_id: safety_rule(case_by_id(case_id)) for case_id in ids},
        "repetitions": repetitions, "seed": 0, "distinct_stochastic_seeds": False,
        "attempt_timeout_s": attempt_timeout_s, "call_limits": LIMITS.record(),
        "profile": MOCK_NUMERIC_PROFILE.model_dump(mode="json"),
        "capabilities": _capability_record(MOCK_CAPABILITIES),
        "attempts": [{"attempt_id": f"{case_id}-r{repeat:03d}", "scenario_id": case_id,
                      "repetition": repeat} for repeat in range(1, repetitions + 1) for case_id in ids],
    }
    return json.loads(json.dumps(spec))


def worker(directory, attempt_id):
    """Execute exactly one frozen fixture. Parent owns acceptance and replay."""
    config = json.loads((directory.parent.parent / "campaign.json").read_text())
    spec, config_hash = config["spec"], config["sha256"]
    if digest(spec) != config_hash:
        raise ValueError("configuration digest mismatch")
    attempt = next(item for item in spec["attempts"] if item["attempt_id"] == attempt_id)
    case = case_by_id(attempt["scenario_id"])
    if digest(spec) != digest(specification(spec["case_ids"], spec["repetitions"], spec["attempt_timeout_s"])):
        raise ValueError("configuration differs from authored matrix")
    task, labels = task_and_labels(case, attempt_id)
    release = Event()
    tracer = FixtureTracer(directory / "events.jsonl", {
        "task_id": task.id, "expect_abort": task.expect_abort,
        "evaluation_labels": labels.as_record(), "scenario_id": case.id,
        "configuration_sha256": config_hash, "attempt_id": attempt_id,
    }, case.mode)
    base = mock_admission()
    admission = CoreAdmission(base.guard, base.provider, base.evidence_kind, call_limits=LIMITS)
    result = {**attempt, "configuration_sha256": config_hash, "metrics": None, "error": None}
    started = time.monotonic()
    try:
        metrics = run(task, FixtureWorld(case.mode), FixtureReasoner(task.goal, case.mode, release),
                      SafetyVerifier(), FixturePolicy(case.mode, release),
                      NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                      capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                      approval=SyntheticApprovalProvider(), admission=admission)
        result["metrics"] = metrics.model_dump()
    except Exception as error:
        result["error"] = type(error).__name__
    finally:
        release.set()
        tracer.close()
    result["elapsed_s"] = time.monotonic() - started
    write_json(directory / "result.json", result)
    return 0


def evaluate(directory, attempt, config_hash, harness, timeout_s=5.0):
    """Reconstruct outcomes, never turn a worker's success claim into evidence."""
    case = case_by_id(attempt["scenario_id"])
    path = directory / "events.jsonl"
    try:
        replay = replay_trace(path, expected_task_id=attempt["attempt_id"])
    except (ValueError, TypeError, KeyError, AttributeError, IndexError) as error:
        replay = EpisodeReplay(attempt["attempt_id"], False,
                               ("replay_unreadable:" + type(error).__name__,), None, (),
                               _sha256(path) if path.is_file() else "")
    findings = list(replay.findings)
    result = None
    try:
        result = json.loads((directory / "result.json").read_text())
        if not isinstance(result, dict) or digest({key: result.get(key) for key in attempt}) != digest(attempt) \
                or result.get("configuration_sha256") != config_hash:
            raise ValueError("attempt binding mismatch")
        if type(result.get("elapsed_s")) not in {int, float} \
                or not math.isfinite(result["elapsed_s"]) or result["elapsed_s"] < 0:
            raise ValueError("invalid worker duration")
        if result.get("metrics") is not None:
            parsed = RunMetrics.model_validate(result["metrics"])
            if digest(parsed.model_dump()) != digest(result["metrics"]):
                raise ValueError("noncanonical worker metrics")
    except (OSError, ValueError, TypeError) as error:
        findings.append("worker_result_invalid:" + type(error).__name__)
        result = None
    events = []
    try:
        events = [json.loads(line) for line in path.read_text().splitlines()]
        if any(not isinstance(event, dict) for event in events):
            raise ValueError("trace events must be objects")
        if not events or events[0].get("attempt_id") != attempt["attempt_id"] \
                or events[0].get("scenario_id") != case.id \
                or events[0].get("configuration_sha256") != config_hash:
            findings.append("trace_campaign_binding_mismatch")
        task, labels = task_and_labels(case, attempt["attempt_id"])
        declarations = {
            "call_limits_declaration": ("limits", LIMITS.record()),
            "numeric_profile_declaration": ("profile", MOCK_NUMERIC_PROFILE.model_dump(mode="json")),
            "capability_declaration": ("capability", _capability_record(MOCK_CAPABILITIES)),
            "task_declaration": ("task", task.model_dump(mode="json")),
        }
        if events and digest(events[0].get("evaluation_labels")) != digest(labels.as_record()):
            findings.append("trace_campaign_binding_mismatch:labels")
        for kind, (field, expected) in declarations.items():
            matching = [event for event in events if event.get("kind") == kind]
            if len(matching) != 1 or digest(matching[0].get(field)) != digest(expected):
                findings.append("trace_campaign_binding_mismatch:" + kind)
    except (OSError, ValueError, AttributeError):
        findings.append("trace_unreadable")
        events = []
    status = harness.get("status")
    duration = harness.get("elapsed_s")
    valid_harness = (status in {"EXITED", "TIMEOUT", "LAUNCH_ERROR"}
        and type(duration) in {int, float} and math.isfinite(duration) and duration >= 0
        and set(harness) == ({"status", "returncode", "elapsed_s", "error"}
                            if status == "LAUNCH_ERROR" else {"status", "returncode", "elapsed_s"})
        and (type(harness.get("returncode")) is int if status == "EXITED"
             else harness.get("returncode") is None)
        and (status != "TIMEOUT" or duration >= timeout_s)
        and (status != "LAUNCH_ERROR" or isinstance(harness.get("error"), str)))
    if not valid_harness:
        findings.append("harness_invalid_record")
    harness_ok = valid_harness and status == "EXITED" and harness.get("returncode") == 0
    if not harness_ok:
        findings.append("harness_" + str(harness.get("status")))
    if replay.valid and result is not None and events:
        expected_metrics = {key: events[-1].get(key) for key in RunMetrics.model_fields}
        if digest(result.get("metrics")) != digest(expected_metrics) or result.get("error") is not None:
            findings.append("worker_result_invalid:counter_join")
    complete = replay.valid and result is not None and harness_ok and not findings
    label_readable = True
    try:
        event_labels = [json.loads(line) for line in (directory / "safety-labels.jsonl").read_text().splitlines()]
    except (OSError, ValueError):
        event_labels = []
        label_readable = False
    safety_score = score_events(events, event_labels, safety_rule(case), config_hash,
                                trace_complete=complete and label_readable)
    if not label_readable:
        safety_score["findings"].append("safety_labels_unavailable")
        safety_score["label_join_valid"] = False
    findings.extend(safety_score["findings"])
    complete = complete and safety_score["qualified"]
    metric = replay.metrics if complete else None
    checks = {}
    if case.error is not None:
        expected_prefix = (
            "run_start call_limits_declaration task_declaration capability_declaration "
            "permission_declaration numeric_profile_declaration constraints_declaration "
            "admission_authority world_state uncertainty_gate reasoner_request plan world_state "
            "uncertainty_gate context_gate capability_gate permission_gate approval_gate "
            "numeric_profile_gate safety1 authorization_gate dispatch_intent world_state "
            "uncertainty_gate context_gate dynamic_safety_gate context_gate").split()
        prefix_valid = ([e.get("kind") for e in events] == expected_prefix
                        and set(replay.findings) == {"missing_episode_end", "episode_end_count:0"})
        if prefix_valid:
            intent, authorization = events[21], events[20]
            prefix_valid = bool(intent.get("dispatch_id")
                and intent.get("dispatch_id") == authorization.get("dispatch_id")
                and events[-1].get("dispatch_id") == intent.get("dispatch_id")
                and events[-1].get("phase") == "pre_apply"
                and events[-1].get("verdict") == "ALLOW"
                and authorization.get("verdict") == "ALLOW"
                and events[19].get("verdict") == "PASS")
        checks["authored_partial_prefix_observed"] = prefix_valid
        checks["partial_label_join"] = safety_score["label_join_valid"]
        checks["expected_incomplete_error"] = bool(
            harness_ok and result is not None and result.get("error") == case.error
            and result.get("metrics") is None and not replay.valid
            and not any(e.get("kind") == "episode_end" for e in events))
    elif metric is not None:
        end = events[-1]
        worker_metrics = result.get("metrics")
        checks["worker_replay_counters"] = bool(isinstance(worker_metrics, dict)
            and worker_metrics and result.get("error") is None
            and all(end.get(key) == value for key, value in worker_metrics.items())
            and worker_metrics.get("task_id") == attempt["attempt_id"])
        for key in ("goal_met", "task_success", "aborted", "replans"):
            checks[key] = metric[key] == getattr(case, key)
        checks["applies"] = sum(e.get("kind") == "apply" for e in events) == case.applies
        checks["stop_status"] = end.get("stop_status") == case.stop_status
        checks["stop_reason"] = [e.get("reason") for e in events if e.get("kind") == "stop_request"] \
            == ([case.stop_reason] if case.stop_reason else [])
        if case.failure_event is not None:
            kind, phase = case.failure_event
            checks["failure_phase"] = [e.get("phase") for e in events if e.get("kind") == kind] == [phase]
        if case.dynamic_verdicts is not None:
            checks["dynamic_verdicts"] = [e.get("verdict") for e in events
                if e.get("kind") == "dynamic_safety_gate"] == list(case.dynamic_verdicts)
    else:
        checks["complete_evidence_required"] = False
    if any(name.startswith(("worker_result_invalid", "trace_campaign_binding", "trace_unreadable", "harness_", "safety_label"))
           for name in findings):
        checks["attempt_integrity"] = False
    matched = bool(checks) and all(checks.values())
    return {**attempt, "harness": harness, "evidence_complete": complete,
            "replay_valid": replay.valid, "expectation_match": matched,
            "acceptance_pass": matched and complete,
            "expected_incomplete": case.error is not None,
            "goal_met": metric.get("goal_met") if metric else None,
            "legacy_task_success": metric.get("task_success") if metric else None,
            "stop_status": events[-1].get("stop_status") if complete else "UNKNOWN",
            "safety_adjudication": safety_score,
            "checks": checks, "findings": findings}


def _rate(count, denominator):
    return {"numerator": count, "denominator": denominator,
            "value": count / denominator if denominator else None}


def summarize(attempts):
    total = len(attempts)
    return {"schema_version": SCHEMA, "scope": "mock_component_acceptance",
            "performance_claim_authorized": False, "attempted_runs": total,
            "verified_goal_rate": _rate(sum(a["goal_met"] is True for a in attempts), total),
            "legacy_task_success_rate": _rate(sum(a["legacy_task_success"] is True for a in attempts), total),
            "expectation_match_rate": _rate(sum(a["expectation_match"] for a in attempts), total),
            "replay_qualified_acceptance_rate": _rate(sum(a["acceptance_pass"] for a in attempts), total),
            "evidence_complete_rate": _rate(sum(a["evidence_complete"] for a in attempts), total),
            "unknown_goal_outcomes": sum(a["goal_met"] is None for a in attempts),
            "safety_adjudication": aggregate_scores(attempts),
            "campaign_expectations_met": bool(attempts) and all(a["expectation_match"] for a in attempts),
            "attempts": attempts}


def _source():
    source_hash, count = _source_digest(ROOT)
    repo = ROOT.parents[1]
    return {"runtime_sha256": source_hash, "runtime_file_count": count,
            "runner_sha256": _sha256(CLI), "git_commit": _git_value(repo, "rev-parse", "HEAD"),
            "git_branch": _git_value(repo, "rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(_git_value(repo, "status", "--porcelain"))}


def run_campaign(output, *, case_ids=None, repetitions=1, attempt_timeout_s=5.0,
                 command_factory=None):
    spec = specification(case_ids, repetitions, attempt_timeout_s)
    config_hash = digest(spec)
    output.mkdir(parents=True, exist_ok=False)
    source = _source()
    write_json(output / "campaign.json", {"spec": spec, "sha256": config_hash})
    reports = []
    with (output / "attempts.jsonl").open("x", encoding="utf-8") as ledger:
        def record(value):
            ledger.write(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")
            ledger.flush()
            os.fsync(ledger.fileno())
        for attempt in spec["attempts"]:
            directory = output / "attempts" / attempt["attempt_id"]
            directory.mkdir(parents=True)
            record({"kind": "STARTED", **attempt, "configuration_sha256": config_hash})
            started = time.monotonic()
            harness = {"status": "EXITED", "returncode": None}
            command = ([sys.executable, str(CLI), "--worker", attempt["attempt_id"],
                        "--output", str(directory.resolve())] if command_factory is None
                       else command_factory(attempt, directory))
            with (directory / "worker.log").open("xb") as log:
                try:
                    process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                             timeout=attempt_timeout_s, check=False)
                    harness["returncode"] = process.returncode
                except subprocess.TimeoutExpired:
                    harness["status"] = "TIMEOUT"
                except OSError as error:
                    harness["status"] = "LAUNCH_ERROR"
                    harness["error"] = type(error).__name__
            harness["elapsed_s"] = time.monotonic() - started
            write_json(directory / "harness.json", harness)
            report = evaluate(directory, attempt, config_hash, harness, attempt_timeout_s)
            write_json(directory / "assessment.json", report)
            reports.append(report)
            record({"kind": "FINISHED", **attempt, "configuration_sha256": config_hash,
                    "assessment_sha256": _sha256(directory / "assessment.json")})
    summary = summarize(reports)
    final_source = _source()
    summary["source_unchanged"] = digest(source) == digest(final_source)
    if not summary["source_unchanged"]:
        summary["campaign_expectations_met"] = False
    write_json(output / "summary.json", summary)
    artifacts = {path.relative_to(output).as_posix(): {"sha256": _sha256(path), "bytes": path.stat().st_size}
                 for path in sorted(output.rglob("*")) if path.is_file()}
    write_json(output / "manifest.json", {"schema_version": SCHEMA, "source": source,
               "final_source": final_source,
               "created_at": datetime.now(timezone.utc).isoformat(),
               "configuration_sha256": config_hash, "artifacts": artifacts})
    return summary


def verify_campaign(output):
    """Verify a sealed campaign against the authored matrix and independent replay."""
    findings = []
    try:
        manifest = json.loads((output / "manifest.json").read_text())
        current = _source()
        if any(manifest["source"].get(key) != current[key]
               for key in ("runtime_sha256", "runtime_file_count", "runner_sha256")):
            findings.append("reader_source_mismatch")
        config = json.loads((output / "campaign.json").read_text())
        spec = config["spec"]
        if manifest["schema_version"] != SCHEMA or digest(spec) != digest(specification(
                spec["case_ids"], spec["repetitions"], spec["attempt_timeout_s"])):
            findings.append("matrix_mismatch")
        config_hash = digest(spec)
        if config["sha256"] != config_hash or manifest["configuration_sha256"] != config_hash:
            findings.append("configuration_digest_mismatch")
        inventory = {p.relative_to(output).as_posix() for p in output.rglob("*") if p.is_file()
                     and p != output / "manifest.json"}
        if set(manifest["artifacts"]) != inventory:
            findings.append("artifact_inventory_mismatch")
        for name in inventory:
            path = output / name
            if path.is_symlink() or path.resolve().is_relative_to(output.resolve()) is False:
                findings.append("unsafe_artifact_path")
                continue
            if digest(manifest["artifacts"].get(name)) != digest(
                    {"sha256": _sha256(path), "bytes": path.stat().st_size}):
                findings.append("artifact_digest_mismatch:" + name)
        records = [json.loads(line) for line in (output / "attempts.jsonl").read_text().splitlines()]
        if len(records) != len(spec["attempts"]) * 2:
            findings.append("attempt_ledger_count_mismatch")
        reports = []
        for index, attempt in enumerate(spec["attempts"]):
            directory = output / "attempts" / attempt["attempt_id"]
            expected_start = {"kind": "STARTED", **attempt, "configuration_sha256": config_hash}
            expected_end = {"kind": "FINISHED", **attempt, "configuration_sha256": config_hash,
                            "assessment_sha256": _sha256(directory / "assessment.json")}
            if digest(records[index * 2:index * 2 + 2]) != digest([expected_start, expected_end]):
                findings.append("attempt_ledger_binding_mismatch")
            harness = json.loads((directory / "harness.json").read_text())
            report = evaluate(directory, attempt, config_hash, harness, spec["attempt_timeout_s"])
            if digest(report) != digest(json.loads((directory / "assessment.json").read_text())):
                findings.append("assessment_reconstruction_mismatch:" + attempt["attempt_id"])
            reports.append(report)
        summary = json.loads((output / "summary.json").read_text())
        expected = summarize(reports)
        expected["source_unchanged"] = digest(manifest["source"]) == digest(manifest["final_source"])
        if expected["source_unchanged"] is not True:
            expected["campaign_expectations_met"] = False
        if digest(summary) != digest(expected):
            findings.append("summary_reconstruction_mismatch")
    except (OSError, ValueError, TypeError, KeyError, AttributeError, StopIteration) as error:
        findings.append("bundle_unreadable:" + type(error).__name__)
    return {"schema_version": SCHEMA, "valid": not findings, "findings": sorted(set(findings))}
