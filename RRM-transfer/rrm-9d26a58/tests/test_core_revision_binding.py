"""Ordered core plans, task revisions, and dispatch attempts are trace-bound."""

import copy
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
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class RevisionBindingTests(unittest.TestCase):
    def test_ordered_plan_change_during_policy_step_aborts_before_apply(self) -> None:
        task = Task(id="plan-change", mission="pick cup",
                    goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)

        class RetainingOracle(ScriptedOracle):
            def plan(self, mission, ws):
                self.graph = super().plan(mission, ws)
                return self.graph

        oracle = RetainingOracle(task.goal)

        class ReorderingPolicy(MockPolicy):
            def step(self, action, ws):
                trajectory = super().step(action, ws)
                oracle.graph.nodes.reverse()
                return trajectory

        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                result = run(task, MockWorld(), oracle, SafetyVerifier(),
                             ReorderingPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                             capabilities=MOCK_CAPABILITIES,
                             permission=mock_permission(task.id),
                             approval=SyntheticApprovalProvider(), admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        self.assertGreater(len(oracle.graph.nodes), 1)
        self.assertTrue(result.aborted)
        self.assertFalse(any(event["kind"] == "apply" for event in events))
        self.assertIn("plan_payload_changed", next(
            event for event in events if event["kind"] == "context_gate"
            and event["phase"] == "pre_apply"
        )["reasons"])
        self.assertTrue(replay.valid, replay.findings)

    def test_task_change_during_policy_step_aborts_before_apply(self) -> None:
        task = Task(id="revision-change", revision="task-v1", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)

        class MutatingPolicy(MockPolicy):
            def step(self, action, ws):
                trajectory = super().step(action, ws)
                task.revision = "task-v2"
                return trajectory

        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                result = run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                             MutatingPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                             capabilities=MOCK_CAPABILITIES,
                             permission=mock_permission(task.id),
                             approval=SyntheticApprovalProvider(), admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        self.assertTrue(result.aborted)
        self.assertFalse(any(event["kind"] == "apply" for event in events))
        self.assertEqual(next(event for event in events if event["kind"] == "dispatch")
                         ["termination"], "INTERRUPTED")
        self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_run_plan_and_dispatch_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            original = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            self.assertTrue(replay_trace(output / "T1.jsonl", expected_task_id="T1").valid)
            cases = [
                ("run", "safety1", "run_id", "wrong", "run_id_mismatch"),
                ("task", "task_declaration", "task_digest", "0" * 64,
                 "task_declaration_mismatch"),
                ("plan", "plan", "plan_digest", "0" * 64, "plan_binding_mismatch"),
                ("attempt", "apply", "dispatch_id", "wrong", "dispatch_binding_mismatch"),
            ]
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
