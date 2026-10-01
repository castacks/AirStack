"""Missing pre-dispatch snapshots abort without invented stop or success evidence."""

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


class PreDispatchLossWorld(MockWorld):
    fail_on = 1
    invalid = False

    def __init__(self):
        super().__init__()
        self.observations = 0
        self.applications = 0
        self.cancellations = 0

    def observe(self):
        self.observations += 1
        if self.observations == self.fail_on:
            if self.invalid:
                return {"invalid": "snapshot"}
            raise ConnectionError("mock observation unavailable")
        return super().observe()

    def _apply_active(self, action, traj):
        self.applications += 1
        super()._apply_active(action, traj)

    def cancel_dispatch(self, dispatch_id, generation):
        self.cancellations += 1
        return super().cancel_dispatch(dispatch_id, generation)


class InitialInvalidWorld(PreDispatchLossWorld):
    invalid = True


class PreActionLossWorld(PreDispatchLossWorld):
    fail_on = 2


class PreActionInvalidWorld(PreActionLossWorld):
    invalid = True


class CorePreDispatchObservationTests(unittest.TestCase):
    def _episode(self, world_type):
        task = Task(id="pre-dispatch-loss", mission="pick cup",
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

    def test_initial_and_pre_action_loss_abort_without_dispatch(self):
        cases = ((PreDispatchLossWorld, "planning", "observation_error"),
                 (InitialInvalidWorld, "planning", "invalid_payload"),
                 (PreActionLossWorld, "pre_action", "observation_error"),
                 (PreActionInvalidWorld, "pre_action", "invalid_payload"))
        for world_type, phase, kind in cases:
            with self.subTest(world=world_type.__name__):
                metrics, world, events, replay = self._episode(world_type)
                failure = next(e for e in events if e["kind"] == "observation_failure")
                self.assertEqual((failure["phase"], failure["failure_kind"]), (phase, kind))
                self.assertEqual(world.applications, 0)
                self.assertEqual(world.cancellations, 0)
                self.assertFalse(any(e["kind"] in {"dispatch_intent", "dispatch", "apply",
                                                        "stop_request"} for e in events))
                self.assertEqual(len([e for e in events if e["kind"] == "plan"]),
                                 0 if phase == "planning" else 1)
                self.assertTrue(metrics.aborted)
                self.assertFalse(metrics.task_success)
                self.assertEqual(events[-1]["terminal_observation"], "UNAVAILABLE")
                self.assertEqual(events[-1]["stop_status"], "NOT_REQUESTED")
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_forged_pre_dispatch_evidence(self):
        _, _, original, replay = self._episode(PreActionLossWorld)
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("phase", "state", "success", "stop"):
                events = copy.deepcopy(original)
                failure = next(e for e in events if e["kind"] == "observation_failure")
                if name == "phase":
                    failure["phase"] = "planning"
                elif name == "state":
                    failure["state_digest"] = "0" * 64
                elif name == "success":
                    events[-1]["task_success"] = True
                else:
                    events[-1]["stop_status"] = "SAFE_CONFIRMED_MOCK"
                path = Path(directory) / f"{name}.jsonl"
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                self.assertFalse(replay_trace(path, expected_task_id="pre-dispatch-loss").valid)


if __name__ == "__main__":
    unittest.main()
