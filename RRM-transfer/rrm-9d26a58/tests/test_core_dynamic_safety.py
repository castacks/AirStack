"""Observed hazards during a mock dispatch stop before the next chunk."""

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
from rrm.schema import Task, WorldObject
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class DynamicHazardWorld(MockWorld):
    hazard = "human"

    def _apply_active(self, action, traj):
        super()._apply_active(action, traj)
        if self.state.t == 1 and not traj.terminal:
            if self.hazard == "human":
                self.state.objects.append(WorldObject(
                    id="entrant", cls="person", pose=(0.42, -0.2, 0.0)))
            else:
                self.state.get("obj_cup").pose = (9.0, 9.0, 0.0)


class MovedTargetWorld(DynamicHazardWorld):
    hazard = "moved_target"


class CoreDynamicSafetyTests(unittest.TestCase):
    def _episode(self, world_type):
        task = Task(id="dynamic-hazard", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world_type(), ScriptedOracle(task.goal), SafetyVerifier(),
                              MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE),
                              tracer, capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(),
                              admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_human_entry_and_moved_target_stop_active_dispatch(self):
        for world_type, expected_check in ((DynamicHazardWorld, "human_proximity"),
                                           (MovedTargetWorld, "workspace")):
            with self.subTest(hazard=world_type.hazard):
                metrics, events, replay = self._episode(world_type)
                gates = [e for e in events if e["kind"] == "dynamic_safety_gate"]
                self.assertEqual([e["verdict"] for e in gates], ["PASS", "FAIL"])
                self.assertIn(expected_check,
                              [v["check"] for v in gates[-1]["violations"]])
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
                self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                                 "INTERRUPTED")
                self.assertEqual(next(e for e in events if e["kind"] == "stop_request")["reason"],
                                 "dynamic_symbolic_safety")
                self.assertTrue(metrics.aborted)
                self.assertEqual(metrics.replans, 0)
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_dynamic_gate_tampering(self):
        _, original, replay = self._episode(DynamicHazardWorld)
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("verdict", "state", "missing"):
                events = copy.deepcopy(original)
                failed = next(e for e in events if e["kind"] == "dynamic_safety_gate"
                              and e["verdict"] == "FAIL")
                if name == "verdict":
                    failed["verdict"] = "PASS"
                elif name == "state":
                    failed["state_digest"] = "0" * 64
                else:
                    events.remove(failed)
                    for sequence, event in enumerate(events):
                        event["sequence"] = sequence
                path = Path(directory) / f"{name}.jsonl"
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                self.assertFalse(replay_trace(path, expected_task_id="dynamic-hazard").valid)


if __name__ == "__main__":
    unittest.main()
