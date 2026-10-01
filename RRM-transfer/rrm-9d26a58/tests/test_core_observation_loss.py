"""Active mock observation loss interrupts without a terminal success claim."""

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


class ObservationLossWorld(MockWorld):
    failure_mode = "exception"

    def __init__(self):
        super().__init__()
        self.applications = 0
        self.initial_snapshot = super().observe()

    def observe(self):
        if self.state.t >= 1:
            if self.failure_mode == "exception":
                raise ConnectionError("mock observation channel unavailable")
            if self.failure_mode == "invalid":
                return {"not": "WorldState"}
            return self.initial_snapshot.model_copy(deep=True)
        return super().observe()

    def _apply_active(self, action, traj):
        self.applications += 1
        super()._apply_active(action, traj)

    def observe_safe_state(self, dispatch_id, generation):
        return None  # no independent safe-state observation on this fixture


class InvalidObservationWorld(ObservationLossWorld):
    failure_mode = "invalid"


class StaleObservationWorld(ObservationLossWorld):
    failure_mode = "stale"


class CoreObservationLossTests(unittest.TestCase):
    def _episode(self, world_type):
        task = Task(id="observation-loss", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        world = world_type()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world, ScriptedOracle(task.goal), SafetyVerifier(),
                              MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE),
                              tracer, capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(),
                              admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, world, events, replay

    def test_missing_invalid_and_stale_observation_fail_closed(self):
        for world_type, kind in ((ObservationLossWorld, "observation_error"),
                                 (InvalidObservationWorld, "invalid_payload"),
                                 (StaleObservationWorld, "stale_after_apply")):
            with self.subTest(mode=world_type.failure_mode):
                metrics, world, events, replay = self._episode(world_type)
                failure = next(e for e in events if e["kind"] == "observation_failure")
                terminal = events[-1]
                self.assertEqual(failure["failure_kind"], kind)
                self.assertEqual(world.applications, 1)
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
                self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                                 "INTERRUPTED")
                self.assertEqual(metrics.replans, 0)
                self.assertTrue(metrics.aborted)
                self.assertFalse(metrics.task_success)
                self.assertEqual(terminal["terminal_observation"], "UNAVAILABLE")
                self.assertEqual(terminal["stop_status"], "SAFE_UNCONFIRMED")
                self.assertFalse(terminal["goal_met"])
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_loss_and_success_tampering(self):
        _, _, original, replay = self._episode(ObservationLossWorld)
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("failure_kind", "missing_failure", "goal_claim", "status"):
                events = copy.deepcopy(original)
                failure = next(e for e in events if e["kind"] == "observation_failure")
                if name == "failure_kind":
                    failure["failure_kind"] = "all_good"
                elif name == "missing_failure":
                    events.remove(failure)
                    for sequence, event in enumerate(events):
                        event["sequence"] = sequence
                elif name == "goal_claim":
                    events[-1]["task_success"] = True
                else:
                    events[-1]["terminal_observation"] = "OBSERVED"
                path = Path(directory) / f"{name}.jsonl"
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                self.assertFalse(replay_trace(path, expected_task_id="observation-loss").valid)

            _, _, stale_events, _ = self._episode(StaleObservationWorld)
            stale = next(e for e in stale_events if e["kind"] == "observation_failure")
            stale["observed_sim_t"] = 999
            path = Path(directory) / "forged_stale.jsonl"
            path.write_text("".join(json.dumps(e) + "\n" for e in stale_events))
            result = replay_trace(path, expected_task_id="observation-loss")
            self.assertFalse(result.valid)
            self.assertTrue(any(f.startswith("stale_observation_evidence_mismatch")
                                for f in result.findings), result.findings)


if __name__ == "__main__":
    unittest.main()
