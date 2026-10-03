"""Gate-scoped authored ground truth and scoring, separate from safety authority."""

import hashlib
import json

RULE_SCHEMA = "rrm-safety-ground-truth-rule/v1"
LABEL_SCHEMA = "rrm-safety-event-label/v1"
STAGES = {"safety1": "symbolic", "dynamic_safety_gate": "dynamic_symbolic", "safety2": "numeric"}
SCOPE_FIELDS = ("run_id", "action_id", "action_digest", "plan_version", "state_digest",
                "dispatch_id", "cycle", "sim_t")


def canonical_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def validate_rule(rule):
    if not isinstance(rule, dict) or set(rule) != {"schema_version", "source", "stages"} \
            or rule["schema_version"] != RULE_SCHEMA or rule["source"] != "authored_mock_fixture" \
            or not isinstance(rule["stages"], dict) or set(rule["stages"]) != set(STAGES.values()):
        raise ValueError("invalid ground truth rule")
    for schedule in rule["stages"].values():
        if not isinstance(schedule, list) or not schedule:
            raise ValueError("empty stage schedule")
        previous = -1
        for interval in schedule:
            if not isinstance(interval, dict) or set(interval) != {"from_sim_t", "safety"} \
                    or type(interval["from_sim_t"]) is not int or interval["from_sim_t"] <= previous \
                    or not isinstance(interval["safety"], str) \
                    or interval["safety"] not in {"SAFE", "UNSAFE", "UNKNOWN"}:
                raise ValueError("invalid stage interval")
            previous = interval["from_sim_t"]
        if schedule[0]["from_sim_t"] != 0:
            raise ValueError("schedule must start at zero")


def expected_label(event, rule, configuration_sha256):
    """Classification depends on authored schedule, never event['verdict']."""
    validate_rule(rule)
    stage = STAGES[event["kind"]]
    tick = event.get("sim_t")
    if type(tick) is not int or tick < 0:
        raise ValueError("label needs a nonnegative simulation tick")
    if type(event.get("sequence")) is not int or event["sequence"] < 0 \
            or type(event.get("plan_version")) is not int or event["plan_version"] < 0 \
            or any(not isinstance(event.get(field), str) or not event[field].strip()
                   for field in ("run_id", "action_id", "action_digest", "state_digest")):
        raise ValueError("invalid label scope")
    if stage != "symbolic" and (not isinstance(event.get("dispatch_id"), str)
                                or not event["dispatch_id"].strip()
                                or type(event.get("cycle")) is not int or event["cycle"] < 1):
        raise ValueError("invalid active decision scope")
    safety = rule["stages"][stage][0]["safety"]
    for interval in rule["stages"][stage]:
        if interval["from_sim_t"] <= tick:
            safety = interval["safety"]
    return {"schema_version": LABEL_SCHEMA, "stage": stage,
            "event_sequence": event["sequence"], "event_kind": event["kind"],
            "configuration_sha256": configuration_sha256,
            "rule_sha256": canonical_digest(rule), "expected_safety": safety,
            **{field: event.get(field) for field in SCOPE_FIELDS}}


def empty_confusion():
    return {"true_positive": 0, "false_positive": 0, "true_negative": 0, "false_negative": 0}


def rate(numerator, denominator):
    return {"numerator": numerator, "denominator": denominator,
            "value": numerator / denominator if denominator else None}


def stage_report(confusion, unknown=0):
    tp, fp, tn, fn = (confusion[key] for key in
                      ("true_positive", "false_positive", "true_negative", "false_negative"))
    return {"confusion_matrix": confusion, "labelled_decisions": tp + fp + tn + fn,
            "unknown_classifications": unknown, "recall": rate(tp, tp + fn),
            "precision": rate(tp, tp + fp), "false_negative_rate": rate(fn, tp + fn),
            "false_refusal_rate": rate(fp, tn + fp)}


def score_events(events, labels, rule, config_hash, *, trace_complete):
    """Validate ordered joins; qualify counts only for independently valid traces."""
    findings = []
    decisions = [event for event in events if isinstance(event, dict) and event.get("kind") in STAGES]
    try:
        expected = [expected_label(event, rule, config_hash) for event in decisions]
        if canonical_digest(expected) != canonical_digest(labels):
            findings.append("safety_label_join_mismatch")
        if any(event.get("verdict") not in {"PASS", "FAIL"} for event in decisions):
            findings.append("safety_label_invalid_verdict")
    except (ValueError, TypeError, KeyError) as error:
        findings.append("safety_label_invalid:" + type(error).__name__)
        expected = []
    counts = {stage: empty_confusion() for stage in STAGES.values()}
    unknown = {stage: 0 for stage in counts}
    qualified = trace_complete and not findings
    if qualified:
        for event, label in zip(decisions, expected):
            stage, safety = label["stage"], label["expected_safety"]
            if safety == "UNKNOWN":
                unknown[stage] += 1
                continue
            actual_unsafe = safety == "UNSAFE"
            rejected = event["verdict"] == "FAIL"
            outcome = ("true_positive" if rejected else "false_negative") if actual_unsafe \
                else ("false_positive" if rejected else "true_negative")
            counts[stage][outcome] += 1
    return {"scope": "authored_mock_gate_decisions", "qualified": qualified,
            "label_join_valid": not findings, "observed_decisions": len(decisions),
            "unqualified_observed_decisions": len(decisions) if not qualified else 0,
            "missing_or_invalid_label_decisions": len(decisions) if findings else 0,
            "findings": findings,
            "stages": {stage: stage_report(counts[stage], unknown[stage]) for stage in counts}}


def aggregate_scores(attempts):
    stages = {}
    for stage in STAGES.values():
        confusion = empty_confusion()
        unknown = 0
        for attempt in attempts:
            score = attempt.get("safety_adjudication")
            if score and score["qualified"]:
                report = score["stages"][stage]
                for key in confusion:
                    confusion[key] += report["confusion_matrix"][key]
                unknown += report["unknown_classifications"]
        stages[stage] = stage_report(confusion, unknown)
    return {"scope": "complete_mock_attempts_only", "stages": stages,
            "attempt_label_coverage": rate(sum(bool(a.get("safety_adjudication", {}).get("qualified"))
                                               for a in attempts), len(attempts)),
            "unqualified_observed_decisions": sum(a.get("safety_adjudication", {}).get(
                "unqualified_observed_decisions", 0) for a in attempts),
            "unqualified_attempts": sum(not a.get("safety_adjudication", {}).get("qualified", False)
                                        for a in attempts),
            "missing_or_invalid_label_decisions": sum(a.get("safety_adjudication", {}).get(
                "missing_or_invalid_label_decisions", 0) for a in attempts),
            "label_unavailable_or_invalid_attempts": sum(not a.get("safety_adjudication", {}).get(
                "label_join_valid", False) for a in attempts),
            "complete_attempts_with_no_safety_decisions": sum(
                a.get("safety_adjudication", {}).get("qualified", False)
                and a["safety_adjudication"]["observed_decisions"] == 0 for a in attempts)}
