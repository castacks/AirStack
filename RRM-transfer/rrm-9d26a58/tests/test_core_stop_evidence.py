"""Mock C08 interruption is fail-closed and replay-bound, not physical proof."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
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


class StoppingPolicy(MockPolicy):
    def __init__(self, admission, world):
        super().__init__()
        self.admission = admission
        self.world = world
        self.steps = 0
        self.outcome = None

    def step(self, action, ws):
        self.steps += 1
        if self.steps == 2:
            self.outcome = self.admission.stop.request_stop(
                dispatch_id=self.world.active_dispatch_id, reason="operator_stop")
        return super().step(action, ws)


class NoCancelWorld(MockWorld):
    def cancel_dispatch(self, dispatch_id, generation):
        return False

    def observe_safe_state(self, dispatch_id, generation):
        return None


class CoreStopEvidenceTests(unittest.TestCase):
    def _episode(self, world_type=MockWorld):
        task = Task(id="c08-mock", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        admission = mock_admission()
        world = world_type()
        policy = StoppingPolicy(admission, world)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world, ScriptedOracle(task.goal), SafetyVerifier(),
                              policy, NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(), admission=admission)
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay, policy

    def test_stop_fences_later_chunks_and_replays_synthetic_hold(self):
        metrics, events, replay, policy = self._episode()
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.replans, 0)
        self.assertEqual(policy.steps, 2)
        self.assertEqual(policy.outcome.status, "SAFE_CONFIRMED_MOCK")
        self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
        self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                         "INTERRUPTED")
        self.assertTrue(replay.valid, replay.findings)

    def test_missing_cancel_ack_does_not_assert_safe(self):
        metrics, events, replay, policy = self._episode(NoCancelWorld)
        self.assertTrue(metrics.aborted)
        self.assertEqual(policy.outcome.status, "SAFE_UNCONFIRMED")
        self.assertEqual(next(e for e in events if e["kind"] == "episode_end")["stop_status"],
                         "SAFE_UNCONFIRMED")
        self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
        self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_stop_tampering(self):
        _, original, replay, _ = self._episode()
        self.assertTrue(replay.valid, replay.findings)
        cases = (
            ("cancel", "stop_cancel", "acknowledged", False),
            ("status", "stop_safe_state", "status", "SAFE_UNCONFIRMED"),
            ("generation", "stop_request", "generation", 999),
            ("gate", "interruption_gate", "observed_generation", 999),
        )
        with tempfile.TemporaryDirectory() as directory:
            for name, kind, field, value in cases:
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    next(e for e in events if e["kind"] == kind)[field] = value
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    self.assertFalse(replay_trace(path, expected_task_id="c08-mock").valid)

            for name in ("missing_cancel", "post_stop_apply"):
                events = copy.deepcopy(original)
                if name == "missing_cancel":
                    events = [e for e in events if e["kind"] != "stop_cancel"]
                else:
                    forged = copy.deepcopy(next(e for e in events if e["kind"] == "apply"))
                    terminal_index = next(i for i, e in enumerate(events)
                                          if e["kind"] == "episode_end")
                    events.insert(terminal_index, forged)
                for sequence, event in enumerate(events):
                    event["sequence"] = sequence
                path = Path(directory) / f"{name}.jsonl"
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                result = replay_trace(path, expected_task_id="c08-mock")
                self.assertFalse(result.valid)
                if name == "post_stop_apply":
                    self.assertTrue(any(f.startswith("execution_after_stop")
                                        for f in result.findings), result.findings)


if __name__ == "__main__":
    unittest.main()
