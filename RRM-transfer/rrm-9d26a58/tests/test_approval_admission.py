"""Explicit approval is a separate, exact-scoped gate after permission."""

import copy
from dataclasses import replace
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import (
    MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
    mock_admission, mock_permission, run_suite,
)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.contracts import ApprovalDecision
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class ApprovalAdmissionTests(unittest.TestCase):
    def _episode(self, provider, *, expect_abort=True):
        task = Task(id="approval-test", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=expect_abort)
        labels = EvaluationLabels(
            SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
            TerminalLabel.SAFE_ABORT if expect_abort else TerminalLabel.GOAL_VERIFIED,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": expect_abort,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(
                    task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                    MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                    capabilities=MOCK_CAPABILITIES,
                    permission=mock_permission(task.id), approval=provider,
                    admission=mock_admission(),
                )
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_only_exact_explicit_approval_reaches_safety(self):
        metrics, events, replay = self._episode(
            SyntheticApprovalProvider(), expect_abort=False,
        )
        self.assertTrue(metrics.task_success)
        gate = next(event for event in events if event["kind"] == "approval_gate")
        self.assertEqual(gate["verdict"], "ALLOW")
        self.assertEqual(gate["approval"]["evidence_kind"], "synthetic_fixture")
        self.assertEqual(gate["scope"], gate["approval"]["scope"])
        self.assertTrue(any(event["kind"] == "apply" for event in events))
        self.assertTrue(replay.valid, replay.findings)

    def test_missing_denied_and_wrong_scope_abort_before_safety(self):
        class Missing:
            def decide(self, scope):
                return None

        class Denied:
            def decide(self, scope):
                base = SyntheticApprovalProvider().decide(scope)
                return replace(base, verdict="DENY")

        class WrongTask:
            def decide(self, scope):
                base = SyntheticApprovalProvider().decide(scope)
                return replace(base, scope=replace(scope, task_id="other-task"))

        class WrongPlan:
            def decide(self, scope):
                base = SyntheticApprovalProvider().decide(scope)
                return replace(base, scope=replace(scope, plan_version=scope.plan_version + 1))

        class WrongAction:
            def decide(self, scope):
                base = SyntheticApprovalProvider().decide(scope)
                return replace(base, scope=replace(scope, action_digest="0" * 64))

        for provider, reason in (
            (Missing(), "approval_missing"),
            (Denied(), "approval_denied"),
            (WrongTask(), "approval_scope_mismatch"),
            (WrongPlan(), "approval_scope_mismatch"),
            (WrongAction(), "approval_scope_mismatch"),
        ):
            with self.subTest(reason=reason):
                metrics, events, replay = self._episode(provider)
                gate = next(event for event in events if event["kind"] == "approval_gate")
                self.assertTrue(metrics.aborted)
                self.assertEqual(gate["verdict"], "DENY")
                self.assertIn(reason, gate["reasons"])
                self.assertFalse(any(event["kind"] in {"safety1", "dispatch", "apply"}
                                     for event in events))
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_approval_tampering_and_removed_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            original = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            cases = (
                ("scope", lambda gate: gate["scope"].update(task_id="other-task"),
                 "approval_scope_mismatch"),
                ("decision", lambda gate: gate["approval"].update(verdict="DENY"),
                 "approval_digest_mismatch"),
                ("digest", lambda gate: gate.update(approval_digest="0" * 64),
                 "approval_digest_mismatch"),
            )
            for name, mutate, finding in cases:
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    mutate(next(event for event in events if event["kind"] == "approval_gate"))
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(event) + "\n" for event in events))
                    replay = replay_trace(path, expected_task_id="T1")
                    self.assertFalse(replay.valid)
                    self.assertTrue(any(item.startswith(finding) for item in replay.findings),
                                    replay.findings)

            events = [event for event in original if event["kind"] != "approval_gate"]
            for sequence, event in enumerate(events):
                event["sequence"] = sequence
            path = Path(directory) / "missing-gate.jsonl"
            path.write_text("".join(json.dumps(event) + "\n" for event in events))
            replay = replay_trace(path, expected_task_id="T1")
            self.assertFalse(replay.valid)
            self.assertTrue(any(item.startswith("permission_allow_without_approval")
                                for item in replay.findings), replay.findings)

            events = copy.deepcopy(original)
            next(event for event in events if event["kind"] == "dispatch_intent")[
                "approval_digest"
            ] = "0" * 64
            path = Path(directory) / "wrong-intent-approval.jsonl"
            path.write_text("".join(json.dumps(event) + "\n" for event in events))
            replay = replay_trace(path, expected_task_id="T1")
            self.assertFalse(replay.valid)
            self.assertTrue(any(item.startswith("dispatch_intent_approval_mismatch")
                                for item in replay.findings), replay.findings)


if __name__ == "__main__":
    unittest.main()
