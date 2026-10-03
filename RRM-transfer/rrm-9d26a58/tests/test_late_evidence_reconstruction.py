"""Late committed trace writes remain exactly bound to their execution fault."""

import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.core_admission import CoreAdmission
from rrm.core_deadlines import CoreCallLimits
from rrm.loop import CoreEvidenceUnavailable, run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task, WorldObject
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


LIMITS = CoreCallLimits(policy_s=0.08, observation_s=0.08, adapter_s=0.08,
                        stop_s=0.08, evidence_s=0.04)


class LateCommittedTracer(Tracer):
    """Commit one selected record, then return after its caller deadline."""

    def __init__(self, path, metadata, target):
        self.target, self.delayed = target, False
        super().__init__(path, metadata)

    def event(self, kind, **fields):
        sequence = super().event(kind, **fields)
        selected = (self.target(kind, fields) if callable(self.target)
                    else kind == self.target if isinstance(self.target, str)
                    else (kind, fields.get("phase")) == self.target)
        if selected and not self.delayed:
            self.delayed = True
            time.sleep(LIMITS.evidence_s * 1.5)
        return sequence


class PreCommitStallTracer(Tracer):
    """Hold one record before the append so its timeout cannot claim a receipt."""

    def __init__(self, path, metadata, target):
        self.target, self.delayed = target, False
        self.started, self.release, self.finished = (threading.Event() for _ in range(3))
        super().__init__(path, metadata)

    def event(self, kind, **fields):
        if kind == self.target and not self.delayed:
            self.delayed = True
            self.started.set()
            self.release.wait(1.0)
            try:
                return super().event(kind, **fields)
            finally:
                self.finished.set()
        return super().event(kind, **fields)


class DynamicHazardWorld(MockWorld):
    def _apply_active(self, action, traj):
        super()._apply_active(action, traj)
        if self.state.t == 1 and not traj.terminal:
            self.state.objects.append(WorldObject(
                id="entrant", cls="person", pose=(0.42, -0.2, 0.0)))


class LateEvidenceReconstructionTests(unittest.TestCase):
    def episode(self, target):
        task = Task(id="late-evidence", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        base = mock_admission()
        admission = CoreAdmission(base.guard, base.provider, base.evidence_kind,
                                  call_limits=LIMITS)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = LateCommittedTracer(path, {"task_id": task.id, "expect_abort": True,
                                         "evaluation_labels": labels.as_record()}, target)
            metrics = run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                          MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                          capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                          approval=SyntheticApprovalProvider(), admission=admission)
            tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_late_dynamic_numeric_and_apply_records_are_reconstructible(self):
        expectations = (
            ("dynamic_safety_gate", "symbolic_verification"),
            ("safety2", "numeric_validation"),
            ("apply", "apply_evidence"),
            (("world_state", "dispatch"), "observation"),
            (("uncertainty_gate", "dispatch"), "observation"),
            (("world_state", "post_dispatch"), "post_dispatch_observation"),
        )
        for target, phase in expectations:
            with self.subTest(target=target):
                metrics, events, replay = self.episode(target)
                self.assertTrue(metrics.aborted)
                self.assertFalse(metrics.task_success)
                fault = next(event for event in events if event["kind"] == "execution_fault")
                deadline = fault["deadline"]
                late = events[fault["sequence"] - 1]
                self.assertEqual(fault["phase"], phase)
                self.assertEqual(deadline["call"], "trace_write")
                self.assertTrue(deadline["operation_pending"])
                target_kind = target if isinstance(target, str) else target[0]
                self.assertEqual(deadline["event_kind"], target_kind)
                self.assertEqual(deadline["event_write_id"], late["evidence_write_id"])
                self.assertIs(deadline["event_committed"], True)
                self.assertIs(deadline["event_reconstructible"], True)
                self.assertEqual(late["kind"], target_kind)
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_wrong_late_observation_phase(self):
        _, events, replay = self.episode(("world_state", "post_dispatch"))
        self.assertTrue(replay.valid, replay.findings)
        late = next(event for event in events if event["kind"] == "execution_fault")
        events[late["sequence"] - 1]["phase"] = "dispatch"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "altered.jsonl"
            path.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(path, expected_task_id="late-evidence")
        self.assertFalse(result.valid)
        self.assertTrue(any("execution_fault_boundary_mismatch" in item
                            for item in result.findings), result.findings)

    def test_replay_rejects_forged_late_write_identity_kind_order_and_types(self):
        _, original, replay = self.episode("dynamic_safety_gate")
        self.assertTrue(replay.valid, replay.findings)
        for mode in ("missing_id", "wrong_id", "duplicate_id", "wrong_kind", "bool_id",
                     "target_bool_id", "wrong_phase", "pending_type", "not_committed",
                     "not_reconstructible", "reconstructible_type",
                     "not_adjacent"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                events = copy.deepcopy(original)
                fault = next(event for event in events if event["kind"] == "execution_fault")
                deadline = fault["deadline"]
                late = events[fault["sequence"] - 1]
                if mode == "missing_id":
                    del deadline["event_write_id"]
                elif mode == "wrong_id":
                    deadline["event_write_id"] = "forged"
                elif mode == "duplicate_id":
                    events[1]["evidence_write_id"] = deadline["event_write_id"]
                elif mode == "wrong_kind":
                    deadline["event_kind"] = "safety2"
                elif mode == "bool_id":
                    deadline["event_write_id"] = True
                elif mode == "target_bool_id":
                    late["evidence_write_id"] = True
                elif mode == "wrong_phase":
                    fault["phase"] = "numeric_validation"
                elif mode == "pending_type":
                    deadline["operation_pending"] = 1
                elif mode == "not_committed":
                    deadline["event_committed"] = False
                elif mode == "not_reconstructible":
                    deadline["event_reconstructible"] = False
                elif mode == "reconstructible_type":
                    deadline["event_reconstructible"] = 1
                else:
                    late["evidence_write_id"] = "other"
                    events[fault["sequence"] - 2]["evidence_write_id"] = deadline["event_write_id"]
                path = Path(directory) / "altered.jsonl"
                path.write_text("".join(json.dumps(event) + "\n" for event in events))
                self.assertFalse(replay_trace(path, expected_task_id="late-evidence").valid)

    def test_every_bounded_core_record_has_unique_write_identity(self):
        _, events, replay = self.episode("safety2")
        identities = [event.get("evidence_write_id") for event in events[1:]]
        self.assertTrue(all(isinstance(value, str) and value for value in identities))
        self.assertEqual(len(identities), len(set(identities)))
        self.assertTrue(replay.valid, replay.findings)

    def test_precommit_timeout_remains_incomplete_after_late_append(self):
        task = Task(id="precommit-timeout", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        base = mock_admission()
        admission = CoreAdmission(base.guard, base.provider, base.evidence_kind,
                                  call_limits=LIMITS)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = PreCommitStallTracer(
                path, {"task_id": task.id, "expect_abort": True,
                       "evaluation_labels": labels.as_record()}, "dynamic_safety_gate")
            try:
                with self.assertRaises(CoreEvidenceUnavailable):
                    run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                        MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                        capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                        approval=SyntheticApprovalProvider(), admission=admission)
            finally:
                tracer.release.set()
                self.assertTrue(tracer.finished.wait(1.0))
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            fault = next(event for event in events if event["kind"] == "execution_fault")
            self.assertIs(fault["deadline"]["event_committed"], False)
            self.assertGreater(
                next(event["sequence"] for event in events
                     if event["kind"] == "dynamic_safety_gate"), fault["sequence"])
            self.assertFalse(replay_trace(path, expected_task_id=task.id).valid)

    def test_committed_late_rejecting_gate_returns_no_complete_metrics(self):
        task = Task(id="late-reject", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        base = mock_admission()
        admission = CoreAdmission(base.guard, base.provider, base.evidence_kind,
                                  call_limits=LIMITS)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = LateCommittedTracer(
                path, {"task_id": task.id, "expect_abort": True,
                       "evaluation_labels": labels.as_record()},
                lambda kind, fields: kind == "dynamic_safety_gate"
                and fields.get("verdict") == "FAIL")
            try:
                with self.assertRaises(CoreEvidenceUnavailable):
                    run(task, DynamicHazardWorld(), ScriptedOracle(task.goal),
                        SafetyVerifier(), MockPolicy(),
                        NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                        capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                        approval=SyntheticApprovalProvider(), admission=admission)
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            fault = next(event for event in events if event["kind"] == "execution_fault")
            self.assertIs(fault["deadline"]["event_committed"], True)
            self.assertIs(fault["deadline"]["event_reconstructible"], False)
            self.assertFalse(replay_trace(path, expected_task_id=task.id).valid)


if __name__ == "__main__":
    unittest.main()
