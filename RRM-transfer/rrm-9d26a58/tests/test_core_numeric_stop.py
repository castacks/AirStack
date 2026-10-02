"""Rejected numeric chunks cancel active mock dispatch instead of replanning."""

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


class RejectedChunkPolicy(MockPolicy):
    def __init__(self, reject_at=2):
        super().__init__()
        self.reject_at = reject_at
        self.calls = 0

    def step(self, action, ws):
        self.calls += 1
        trajectory = super().step(action, ws)
        if self.calls >= self.reject_at:
            return trajectory.model_copy(update={"max_velocity": 20.0})
        return trajectory


class NoCancelWorld(MockWorld):
    def cancel_dispatch(self, dispatch_id, generation):
        return False


class NoSafeObservationWorld(MockWorld):
    def observe_safe_state(self, dispatch_id, generation):
        return None


class CoreNumericStopTests(unittest.TestCase):
    def _episode(self, world_type=MockWorld, reject_at=2):
        task = Task(id="numeric-stop", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.UNSAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        admission = mock_admission()
        policy = RejectedChunkPolicy(reject_at)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world_type(), ScriptedOracle(task.goal),
                              SafetyVerifier(), policy,
                              NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(), admission=admission)
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay, admission, policy

    def test_first_and_later_rejections_stop_without_replan(self):
        for reject_at in (1, 2):
            with self.subTest(reject_at=reject_at):
                metrics, events, replay, admission, policy = self._episode(reject_at=reject_at)
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), reject_at - 1)
                self.assertEqual(policy.calls, reject_at)
                self.assertEqual(metrics.replans, 0)
                self.assertEqual(metrics.safety_rejections, 1)
                self.assertTrue(metrics.aborted)
                self.assertFalse(events[-1]["goal_met"])
                self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                                 "INTERRUPTED")
                failure = next(e for e in events if e["kind"] == "safety2"
                               and e["verdict"] == "FAIL")
                request = events[failure["sequence"] + 1]
                self.assertEqual(request["kind"], "stop_request")
                self.assertEqual(request["reason"], "active_numeric_safety")
                self.assertEqual(request["dispatch_id"], failure["dispatch_id"])
                self.assertEqual(events[-1]["stop_status"], "SAFE_CONFIRMED_MOCK")
                self.assertTrue(admission.guard.snapshot(
                    decision_id="probe", run_id="probe", dispatch_id="probe")["stopped"])
                self.assertTrue(replay.valid, replay.findings)

    def test_missing_cancel_or_safe_observation_remains_unconfirmed(self):
        for world_type in (NoCancelWorld, NoSafeObservationWorld):
            with self.subTest(world=world_type.__name__):
                metrics, events, replay, _, _ = self._episode(world_type)
                self.assertTrue(metrics.aborted)
                self.assertEqual(metrics.replans, 0)
                self.assertEqual(events[-1]["stop_status"], "SAFE_UNCONFIRMED")
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_missing_detached_or_contradictory_stop(self):
        _, original, replay, _, _ = self._episode()
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("missing_stop", "reason", "dispatch", "terminal", "cancel", "safe"):
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    if name == "missing_stop":
                        events = [e for e in events if e["kind"] not in {
                            "stop_request", "stop_cancel", "stop_safe_state", "interruption_gate"}]
                        for sequence, event in enumerate(events):
                            event["sequence"] = sequence
                    elif name in {"reason", "dispatch"}:
                        request = next(e for e in events if e["kind"] == "stop_request")
                        request["reason" if name == "reason" else "dispatch_id"] = "wrong"
                    elif name == "terminal":
                        next(e for e in events if e["kind"] == "dispatch")["termination"] = "UNSAFE"
                    elif name == "cancel":
                        next(e for e in events if e["kind"] == "stop_cancel")["acknowledged"] = False
                    else:
                        next(e for e in events if e["kind"] == "stop_safe_state")["evidence"] = None
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    result = replay_trace(path, expected_task_id="numeric-stop")
                    self.assertFalse(result.valid, result.findings)


if __name__ == "__main__":
    unittest.main()
