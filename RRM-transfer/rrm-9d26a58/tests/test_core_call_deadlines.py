"""Event-controlled stalls test bounded waits, late results and safe-state proof."""

import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from threading import Event

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.core_admission import CoreAdmission
from rrm.core_deadlines import CoreCallLimits, CoreCallTimeout, bounded_call
from rrm.core_stop import StopStateEvidence
from rrm.loop import CoreEvidenceUnavailable, run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import DispatchCancelled, MockWorld


LIMITS = CoreCallLimits(policy_s=0.08, observation_s=0.08, adapter_s=0.08,
                        stop_s=0.08, evidence_s=0.08)


class Stall:
    def __init__(self):
        self.entered, self.release, self.finished = Event(), Event(), Event()

    def wait(self):
        self.entered.set()
        if not self.release.wait(5):
            raise RuntimeError("test did not release its worker")


class StallingPolicy(MockPolicy):
    def __init__(self, stall, self_stop=False):
        super().__init__()
        self.stall, self.calls = stall, 0
        self.self_stop = self_stop

    def step(self, action, state):
        self.calls += 1
        if self.calls == 2:
            try:
                if self.self_stop:
                    self.admission.stop.request_stop(
                        dispatch_id=self.world.active_dispatch_id, reason="operator_stop")
                self.stall.wait()
                return super().step(action, state)
            finally:
                self.stall.finished.set()
        return super().step(action, state)


class StallingWorld(MockWorld):
    def __init__(self, mode, stall):
        super().__init__()
        self.mode, self.stall = mode, stall
        self.apply_calls, self.cancel_calls = 0, 0
        self.late_start_fenced = False

    def begin_dispatch(self, action, dispatch_id):
        if self.mode == "begin":
            try:
                self.stall.wait()
                try:
                    return super().begin_dispatch(action, dispatch_id)
                except DispatchCancelled:
                    self.late_start_fenced = True
                    raise
            finally:
                self.stall.finished.set()
        return super().begin_dispatch(action, dispatch_id)

    def observe(self):
        if self.mode == "observe" and self.state.t == 1:
            try:
                self.stall.wait()
                return super().observe()
            finally:
                self.stall.finished.set()
        return super().observe()

    def apply(self, action, trajectory):
        self.apply_calls += 1
        if self.mode in {"apply", "permissive", "self_stop"} and self.apply_calls == 2:
            try:
                if self.mode == "self_stop":
                    self.admission.stop.request_stop(
                        dispatch_id=self.active_dispatch_id, reason="operator_stop")
                self.stall.wait()
                return super().apply(action, trajectory)
            finally:
                self.stall.finished.set()
        return super().apply(action, trajectory)

    def _apply_active(self, action, trajectory):
        if self.mode == "locked" and self.state.t == 1:
            try:
                # Hold the real adapter lock: cancellation, observation and
                # cleanup must all time out rather than blocking the parent.
                self.stall.wait()
                return super()._apply_active(action, trajectory)
            finally:
                self.stall.finished.set()
        return super()._apply_active(action, trajectory)

    def end_dispatch(self, dispatch_id):
        if self.mode == "end":
            try:
                self.stall.wait()
                return super().end_dispatch(dispatch_id)
            finally:
                self.stall.finished.set()
        return super().end_dispatch(dispatch_id)

    def cancel_dispatch(self, dispatch_id, generation):
        self.cancel_calls += 1
        if self.mode == "cancel":
            try:
                self.stall.wait()
                return super().cancel_dispatch(dispatch_id, generation)
            finally:
                self.stall.finished.set()
        if self.mode == "permissive":
            return True
        return super().cancel_dispatch(dispatch_id, generation)

    def observe_safe_state(self, dispatch_id, generation):
        if self.mode == "safe":
            try:
                self.stall.wait()
                return super().observe_safe_state(dispatch_id, generation)
            finally:
                self.stall.finished.set()
        if self.mode == "permissive":
            return StopStateEvidence(dispatch_id, generation, self.state.t, time.monotonic(),
                                     True, False, True, "MOCK_HOLD", "synthetic_mock")
        return super().observe_safe_state(dispatch_id, generation)


class StallingTracer(Tracer):
    def __init__(self, path, meta, stall):
        self.stall, self.triggered = stall, False
        super().__init__(path, meta)

    def event(self, kind, **fields):
        if kind == "apply" and not self.triggered:
            self.triggered = True
        elif not self.triggered:
            return super().event(kind, **fields)
        try:
            self.stall.wait()
            return super().event(kind, **fields)
        finally:
            self.stall.finished.set()


class CoreCallDeadlineTests(unittest.TestCase):
    def episode(self, mode="policy", evidence_dir=None):
        stall = Stall()
        world = StallingWorld(mode, stall)
        policy_stall = stall if mode not in {"cancel", "safe"} else Stall()
        policy = (StallingPolicy(policy_stall, self_stop=mode == "self_policy")
                  if mode in {"policy", "self_policy", "cancel", "safe"} else MockPolicy())
        # For stop-channel stalls the policy raises promptly, triggering cancellation.
        if mode in {"cancel", "safe"}:
            class FailingPolicy(MockPolicy):
                def step(self, action, state):
                    raise RuntimeError("trigger stop channel")
            policy = FailingPolicy()
        task = Task(id="call-deadline", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        base = mock_admission()
        admission = CoreAdmission(base.guard, base.provider, base.evidence_kind, call_limits=LIMITS)
        world.admission = admission
        if isinstance(policy, StallingPolicy):
            policy.admission, policy.world = admission, world
        metrics, error = None, None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            meta = {"task_id": task.id, "expect_abort": True, "evaluation_labels": labels.as_record()}
            tracer = StallingTracer(path, meta, stall) if mode == "trace" else Tracer(path, meta)
            started = time.monotonic()
            try:
                try:
                    metrics = run(task, world, ScriptedOracle(task.goal), SafetyVerifier(), policy,
                                  NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                                  capabilities=MOCK_CAPABILITIES,
                                  permission=mock_permission(task.id),
                                  approval=SyntheticApprovalProvider(), admission=admission)
                except CoreEvidenceUnavailable as caught:
                    error = caught
                elapsed = time.monotonic() - started
                # Timeout returned while the selected callback is still parked.
                self.assertTrue(stall.entered.is_set())
                self.assertFalse(stall.release.is_set())
                before_release = admission.stop.outcome
            finally:
                stall.release.set()
                policy_stall.release.set()
                self.assertTrue(stall.finished.wait(1), mode)
                # All tests use one shared release event for stop/evidence workers.
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
            if evidence_dir is not None:
                target = Path(evidence_dir) / f"{mode}.jsonl"
                target.write_text(path.read_text())
        return metrics, events, replay, world, admission, error, elapsed, before_release

    def test_validated_limits_and_completed_calls(self):
        for value in (0, -1, float("nan"), float("inf"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                CoreCallLimits(policy_s=value)
        self.assertEqual(bounded_call("fixture", 1.0, lambda: 42), 42)
        with self.assertRaisesRegex(ValueError, "fixture error"):
            bounded_call("fixture", 1.0, lambda: (_ for _ in ()).throw(ValueError("fixture error")))

    def test_policy_observation_adapter_and_cleanup_deadlines(self):
        for mode, phase, applies in (("policy", "policy_step", 1),
                                     ("observe", "observation", 1),
                                     ("begin", "begin_dispatch", 0),
                                     ("apply", "apply", 1),
                                     ("locked", "apply", 1),
                                     ("end", "end_dispatch", 3)):
            with self.subTest(mode=mode):
                metrics, events, replay, world, admission, error, elapsed, outcome = self.episode(mode)
                self.assertIsNone(error)
                self.assertTrue(metrics.aborted)
                self.assertFalse(metrics.task_success)
                self.assertEqual(metrics.replans, 0)
                self.assertLess(elapsed, 1.5)
                self.assertEqual(world.cancel_calls, 1)
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), applies)
                fault = next(e for e in events if e["kind"] == "execution_fault")
                self.assertEqual(fault["phase"], phase)
                self.assertEqual(fault["error_type"], "CoreCallTimeout")
                self.assertGreaterEqual(fault["deadline"]["elapsed_s"], fault["deadline"]["timeout_s"])
                self.assertIs(admission.stop.outcome, outcome)
                self.assertTrue(replay.valid, replay.findings)
                if mode == "begin":
                    self.assertTrue(world.late_start_fenced)
                    self.assertIsNone(world.active_dispatch_id)
                if mode == "policy":
                    self.assertEqual(world.apply_calls, 1)
                if mode in {"apply", "locked", "begin", "end"}:
                    self.assertEqual(outcome.status, "SAFE_UNCONFIRMED")

    def test_pending_actuation_overrules_permissive_safe_report(self):
        _, events, replay, _, admission, _, _, before_release = self.episode("permissive")
        self.assertTrue(before_release.pending_execution)
        self.assertEqual(before_release.status, "SAFE_UNCONFIRMED")
        self.assertTrue(next(e for e in events if e["kind"] == "stop_cancel")["acknowledged"])
        self.assertTrue(replay.valid, replay.findings)
        self.assertIs(admission.stop.outcome, before_release)

    def test_callback_stop_and_deadline_share_one_generation_and_interruption(self):
        for mode in ("self_stop", "self_policy"):
            with self.subTest(mode=mode):
                metrics, events, replay, world, admission, error, elapsed, outcome = self.episode(mode)
                self.assertIsNone(error)
                self.assertFalse(metrics.task_success)
                self.assertLess(elapsed, 1.5)
                self.assertEqual(world.cancel_calls, 1)
                self.assertEqual(len([e for e in events if e["kind"] == "stop_request"]), 1)
                self.assertEqual(len([e for e in events if e["kind"] == "interruption_gate"]), 1)
                self.assertEqual(outcome.status, "SAFE_UNCONFIRMED" if mode == "self_stop"
                                 else "SAFE_CONFIRMED_MOCK")
                self.assertTrue(replay.valid, replay.findings)

    def test_cancel_and_safe_observation_deadlines_never_upgrade_late_results(self):
        for mode, kind in (("cancel", "stop_cancel"), ("safe", "stop_safe_state")):
            with self.subTest(mode=mode):
                metrics, events, replay, _, admission, error, elapsed, outcome = self.episode(mode)
                self.assertIsNone(error)
                self.assertFalse(metrics.task_success)
                self.assertLess(elapsed, 1.5)
                self.assertEqual(outcome.status, "SAFE_UNCONFIRMED")
                self.assertIsNotNone(next(e for e in events if e["kind"] == kind)["deadline"])
                self.assertIs(admission.stop.outcome, outcome)
                self.assertTrue(replay.valid, replay.findings)

    def test_stalled_trace_does_not_prevent_cancellation(self):
        metrics, events, replay, world, admission, error, elapsed, _ = self.episode("trace")
        self.assertIsInstance(error, CoreEvidenceUnavailable)
        self.assertIsNone(metrics)
        self.assertLess(elapsed, 1.5)
        self.assertEqual(world.cancel_calls, 1)
        self.assertFalse(admission.stop.outcome.trace_complete)
        self.assertFalse(any(e["kind"] == "episode_end" for e in events))
        self.assertFalse(replay.valid)

    def test_deadline_replay_rejects_limits_duration_label_and_pending_tampering(self):
        _, original, replay, _, _, _, _, _ = self.episode()
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("limits", "duration", "call", "missing", "pending"):
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    fault = next(e for e in events if e["kind"] == "execution_fault")
                    if name == "limits":
                        next(e for e in events if e["kind"] == "call_limits_declaration")["limits"]["policy_s"] = 1
                    elif name == "duration":
                        fault["deadline"]["elapsed_s"] = 0
                    elif name == "call":
                        fault["deadline"]["call"] = "apply"
                    elif name == "missing":
                        fault["deadline"] = None
                    else:
                        next(e for e in events if e["kind"] == "stop_safe_state")["pending_execution"] = True
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    self.assertFalse(replay_trace(path, expected_task_id="call-deadline").valid)

    def test_replay_accepts_conservative_pending_work_that_appears_after_stop_request(self):
        _, events, replay, _, _, _, _, _ = self.episode("permissive")
        self.assertTrue(replay.valid, replay.findings)
        # A racing callback can enter the registry after the request snapshot;
        # proof still correctly refuses confirmation while it remains pending.
        next(e for e in events if e["kind"] == "stop_request")["pending_execution_at_request"] = False
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pending-race.jsonl"
            path.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(path, expected_task_id="call-deadline")
        self.assertTrue(result.valid, result.findings)


if __name__ == "__main__":
    unittest.main()
