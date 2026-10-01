"""Reference-loop one-use admission is exact-scoped and replayable."""

import copy
from dataclasses import replace
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import (
    MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
    SyntheticSafetyDecisionProvider, mock_admission, mock_permission, run_suite,
)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.contracts import AdmissionGuard, SafetyDecision
from rrm.core_admission import CoreAdmission
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class CoreSingleUseAdmissionTests(unittest.TestCase):
    def _episode(self, admission: CoreAdmission, *, two_actions=False, expect_abort=True):
        task = Task(id="c06-mock", mission="place cup" if two_actions else "pick cup",
                    goal=(_p("on", "obj_cup", "obj_table") if two_actions else
                          _p("holding", "$self", "obj_cup")),
                    expect_abort=expect_abort)
        labels = EvaluationLabels(
            SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
            TerminalLabel.SAFE_ABORT if expect_abort else TerminalLabel.GOAL_VERIFIED,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": expect_abort,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                              MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(), admission=admission)
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_exact_allow_is_consumed_before_intent(self):
        metrics, events, replay = self._episode(mock_admission(), expect_abort=False)
        self.assertTrue(metrics.task_success)
        gate = next(event for event in events if event["kind"] == "authorization_gate")
        intent = next(event for event in events if event["kind"] == "dispatch_intent")
        self.assertEqual(gate["verdict"], "ALLOW")
        self.assertEqual(gate["sequence"] + 1, intent["sequence"])
        self.assertEqual(gate["decision_digest"], intent["authorization_digest"])
        self.assertEqual(gate["context_digest"], intent["authorization_context_digest"])
        self.assertTrue(replay.valid, replay.findings)

    def test_missing_stopped_expired_and_stale_decisions_do_not_dispatch(self):
        class Missing:
            def decide(self, context, *, now):
                return None

        class Expired:
            def decide(self, context, *, now):
                return SafetyDecision("expired", context, "ALLOW", now - 2, now - 1)

        class Stale:
            def decide(self, context, *, now):
                changed = replace(context, state_revision="wrong-state")
                return SafetyDecision("stale", changed, "ALLOW", now, now + 5)

        class Denied:
            def decide(self, context, *, now):
                return SafetyDecision("denied", context, "DENY", now, now + 5)

        cases = (
            (CoreAdmission(AdmissionGuard(), SyntheticSafetyDecisionProvider(),
                           "synthetic_fixture"), "stopped"),
            (replace(mock_admission(), provider=Missing()), "decision_missing"),
            (replace(mock_admission(), provider=Expired()), "outside_validity_window"),
            (replace(mock_admission(), provider=Stale()), "stale_context"),
            (replace(mock_admission(), provider=Denied()), "not_allowed"),
            (replace(mock_admission(), clock=lambda: float("nan")), "clock_unavailable"),
        )
        for admission, reason in cases:
            with self.subTest(reason=reason):
                metrics, events, replay = self._episode(admission)
                gate = next(event for event in events if event["kind"] == "authorization_gate")
                self.assertTrue(metrics.aborted)
                self.assertEqual(gate["reason"], reason)
                self.assertFalse(any(event["kind"] in {"dispatch_intent", "apply", "dispatch"}
                                     for event in events))
                self.assertTrue(replay.valid, replay.findings)

    def test_reused_decision_id_denies_second_action(self):
        class Reused:
            def decide(self, context, *, now):
                return SafetyDecision("reused-id", context, "ALLOW", now, now + 5)

        admission = replace(mock_admission(), provider=Reused())
        metrics, events, replay = self._episode(admission, two_actions=True)
        gates = [event for event in events if event["kind"] == "authorization_gate"]
        self.assertTrue(metrics.aborted)
        self.assertEqual([gate["reason"] for gate in gates], [None, "decision_consumed"])
        self.assertEqual(len([event for event in events if event["kind"] == "dispatch_intent"]), 1)
        self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_context_decision_gate_and_intent_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            original = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            cases = (
                ("context", "authorization_gate", "context_digest", "0" * 64,
                 "authorization_context_digest_mismatch"),
                ("decision", "authorization_gate", "decision_digest", "0" * 64,
                 "authorization_decision_digest_mismatch"),
                ("gate", "authorization_gate", "verdict", "DENY",
                 "authorization_gate_verdict_mismatch"),
                ("intent", "dispatch_intent", "authorization_decision_id", "other",
                 "dispatch_intent_authorization_mismatch"),
                ("constraints", "constraints_declaration", "constraints_digest", "0" * 64,
                 "invalid_constraints_declaration"),
            )
            for name, kind, field, value, finding in cases:
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    next(event for event in events if event["kind"] == kind)[field] = value
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(event) + "\n" for event in events))
                    replay = replay_trace(path, expected_task_id="T1")
                    self.assertFalse(replay.valid)
                    self.assertTrue(any(item.startswith(finding) for item in replay.findings),
                                    replay.findings)


if __name__ == "__main__":
    unittest.main()
