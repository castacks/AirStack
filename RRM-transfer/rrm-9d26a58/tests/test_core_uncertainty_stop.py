"""Active uncertainty requests mock cancellation; missing evidence stays unconfirmed."""

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
from rrm.uncertainty import aggregate_uncertainty
from rrm.verbs import _p
from rrm.world import MockWorld


class ActiveWeakEvidenceWorld(MockWorld):
    def observe(self):
        state = super().observe()
        if state.t >= 1:
            state.get("obj_cup").confidence = 0.5
            state.uncertainty = aggregate_uncertainty(state)
        return state


class NoCancelAcknowledgementWorld(ActiveWeakEvidenceWorld):
    def cancel_dispatch(self, dispatch_id, generation):
        return False

    def observe_safe_state(self, dispatch_id, generation):
        return None


class TerminalWeakEvidenceWorld(MockWorld):
    def __init__(self):
        super().__init__(fail_grasp_always=True)

    def observe(self):
        state = super().observe()
        if state.t >= 6:
            state.get("obj_cup").confidence = 0.5
            state.uncertainty = aggregate_uncertainty(state)
        return state


class CoreUncertaintyStopTests(unittest.TestCase):
    def _episode(self, world_type):
        task = Task(id="active-uncertainty", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world_type(), ScriptedOracle(task.goal),
                              SafetyVerifier(), MockPolicy(),
                              NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(),
                              admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_active_uncertainty_stops_with_and_without_acknowledgement(self):
        for world_type, expected_status in (
                (ActiveWeakEvidenceWorld, "SAFE_CONFIRMED_MOCK"),
                (NoCancelAcknowledgementWorld, "SAFE_UNCONFIRMED")):
            with self.subTest(world=world_type.__name__):
                metrics, events, replay = self._episode(world_type)
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
                self.assertEqual(metrics.replans, 0)
                self.assertTrue(metrics.aborted)
                failed_gate = next(e for e in events if e["kind"] == "uncertainty_gate"
                                   and e["verdict"] == "FAIL")
                self.assertEqual(failed_gate["phase"], "dispatch")
                self.assertEqual(events[failed_gate["sequence"] + 1]["kind"], "stop_request")
                self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                                 "INTERRUPTED")
                self.assertEqual(events[-1]["stop_status"], expected_status)
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_bare_abort_and_stop_tampering(self):
        _, original, replay = self._episode(ActiveWeakEvidenceWorld)
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("reason", "terminal", "cancel_ack", "missing_stop"):
                events = copy.deepcopy(original)
                if name == "reason":
                    next(e for e in events if e["kind"] == "stop_request")["reason"] = "other"
                elif name == "terminal":
                    next(e for e in events if e["kind"] == "dispatch")["termination"] = "UNCERTAIN"
                elif name == "cancel_ack":
                    next(e for e in events if e["kind"] == "stop_cancel")["acknowledged"] = False
                else:
                    events = [e for e in events if e["kind"] not in {
                        "stop_request", "stop_cancel", "stop_safe_state", "interruption_gate"}]
                    for sequence, event in enumerate(events):
                        event["sequence"] = sequence
                path = Path(directory) / f"{name}.jsonl"
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                result = replay_trace(path, expected_task_id="active-uncertainty")
                self.assertFalse(result.valid)
                if name in {"reason", "terminal", "missing_stop"}:
                    self.assertTrue(any(f.startswith("uncertainty_dispatch_without_stop_terminal")
                                        for f in result.findings), result.findings)

    def test_terminal_observation_uncertainty_also_requests_stop(self):
        metrics, events, replay = self._episode(TerminalWeakEvidenceWorld)
        self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 6)
        self.assertEqual(metrics.replans, 0)
        self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                         "INTERRUPTED")
        self.assertEqual(next(e for e in events if e["kind"] == "stop_request")["reason"],
                         "active_uncertainty")
        self.assertTrue(replay.valid, replay.findings)


if __name__ == "__main__":
    unittest.main()
