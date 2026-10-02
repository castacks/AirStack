"""Fault injection across active dispatch, evidence writes and adapter cleanup."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.loop import CoreEvidenceUnavailable, run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.uncertainty import aggregate_uncertainty
from rrm.verbs import _p
from rrm.world import DispatchCancelled, MockWorld


class FaultPolicy(MockPolicy):
    def __init__(self, mode="exception", at=2):
        super().__init__()
        self.mode, self.at, self.calls = mode, at, 0

    def step(self, action, ws):
        self.calls += 1
        trajectory = super().step(action, ws)
        if self.calls != self.at:
            return trajectory
        if self.mode == "exception":
            raise RuntimeError("injected policy failure")
        if self.mode == "missing":
            return None
        if self.mode == "nan":
            return trajectory.model_copy(update={"max_velocity": float("nan")})
        if self.mode == "shape":
            return trajectory.model_copy(update={"waypoints": [(0.0,)]})
        if self.mode == "unsafe":
            return trajectory.model_copy(update={"max_velocity": 20.0})
        raise AssertionError("unsupported fixture")


class FaultWorld(MockWorld):
    def __init__(self, mode="apply", at=2, acknowledge=True, safe_observation=True):
        super().__init__()
        self.mode, self.at = mode, at
        self.acknowledge, self.safe_observation = acknowledge, safe_observation
        self.apply_calls = 0
        self.cancel_calls = 0
        self.ended = False
        self.goal_physical = False

    def begin_dispatch(self, action, dispatch_id):
        if self.mode == "begin_before":
            raise RuntimeError("begin failed before acceptance")
        super().begin_dispatch(action, dispatch_id)
        if self.mode == "begin_after":
            raise RuntimeError("acceptance reply lost")

    def apply(self, action, trajectory):
        self.apply_calls += 1
        if self.apply_calls == self.at and self.mode in {"apply", "partial", "cancelled"}:
            if self.mode == "partial":
                super().apply(action, trajectory)
                self.goal_physical = self.state.robot.holding == "obj_cup"
            if self.mode == "cancelled":
                raise DispatchCancelled("unsolicited adapter rejection")
            raise RuntimeError("application outcome unknown")
        super().apply(action, trajectory)

    def observe(self):
        if self.mode == "observation" and self.state.t >= 1:
            raise RuntimeError("active observation lost")
        if self.mode == "post" and self.state.robot.holding == "obj_cup":
            # First effect-check snapshot succeeds; the later post-dispatch one fails.
            if getattr(self, "effect_seen", False):
                raise RuntimeError("post-dispatch observation lost")
            self.effect_seen = True
        if self.mode == "after_stop" and self.cancel_calls:
            raise RuntimeError("observation lost after interruption")
        return super().observe()

    def end_dispatch(self, dispatch_id):
        self.ended = True
        if self.mode == "cleanup":
            raise RuntimeError("adapter cleanup failed")
        super().end_dispatch(dispatch_id)

    def cancel_dispatch(self, dispatch_id, generation):
        self.cancel_calls += 1
        if not self.acknowledge:
            return False
        return super().cancel_dispatch(dispatch_id, generation)

    def observe_safe_state(self, dispatch_id, generation):
        if not self.safe_observation:
            return None
        return super().observe_safe_state(dispatch_id, generation)


class FaultNumeric(NumericSafetyVerifier):
    def __init__(self):
        super().__init__(MOCK_NUMERIC_PROFILE)
        self.calls = 0

    def verify(self, trajectory, state, **kwargs):
        self.calls += 1
        if self.calls == 2:
            raise RuntimeError("numeric verifier unavailable")
        return super().verify(trajectory, state, **kwargs)


class FaultSymbolic(SafetyVerifier):
    def verify(self, action, state):
        if state.t >= 1:
            raise RuntimeError("symbolic verifier unavailable")
        return super().verify(action, state)


class WriteFaultTracer(Tracer):
    def __init__(self, path, metadata, kind="safety2", permanent=False):
        self.fail_kind, self.permanent, self.triggered = kind, permanent, False
        super().__init__(path, metadata)

    def event(self, kind, **fields):
        if kind == self.fail_kind and not self.triggered:
            self.triggered = True
            raise OSError("injected evidence write failure")
        if self.triggered and self.permanent:
            raise OSError("evidence storage unavailable")
        return super().event(kind, **fields)


class ExecutionFaultTests(unittest.TestCase):
    def _episode(self, *, world=None, policy=None, numeric=None, verifier=None,
                 tracer_kind=None, permanent=False, task=None, reasoner=None):
        task = task or Task(id="execution-fault", mission="pick cup",
                            goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        metadata = {"task_id": task.id, "expect_abort": True,
                    "evaluation_labels": labels.as_record()}
        world = world or FaultWorld(mode="normal")
        admission = mock_admission()
        error = None
        metrics = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = (WriteFaultTracer(path, metadata, tracer_kind, permanent)
                      if tracer_kind else Tracer(path, metadata))
            try:
                metrics = run(task, world, reasoner or ScriptedOracle(task.goal),
                              verifier or SafetyVerifier(), policy or MockPolicy(),
                              numeric or NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(), admission=admission)
            except CoreEvidenceUnavailable as caught:
                error = caught
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay, world, admission, error

    def _assert_fault(self, result, phase, applies):
        metrics, events, replay, world, admission, error = result
        self.assertIsNone(error)
        self.assertTrue(metrics.aborted)
        self.assertFalse(metrics.task_success)
        self.assertEqual(metrics.replans, 0)
        fault = next(e for e in events if e["kind"] == "execution_fault")
        self.assertEqual(fault["phase"], phase)
        self.assertEqual(fault["execution_outcome"], "UNKNOWN")
        self.assertEqual(len([e for e in events if e["kind"] == "apply"]), applies)
        self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                         "INTERRUPTED")
        self.assertEqual(events[-1]["terminal_observation"], "UNAVAILABLE")
        self.assertFalse(events[-1]["goal_met"])
        self.assertEqual(world.cancel_calls, 1)
        self.assertTrue(world.ended)
        self.assertTrue(admission.guard.snapshot(
            decision_id="probe", run_id="probe", dispatch_id="probe")["stopped"])
        self.assertTrue(replay.valid, replay.findings)

    def test_policy_exception_and_invalid_outputs_before_and_after_a_chunk(self):
        for mode in ("exception", "missing", "nan", "shape"):
            for at in (1, 2):
                with self.subTest(mode=mode, at=at):
                    result = self._episode(policy=FaultPolicy(mode, at))
                    self._assert_fault(result, "policy_step" if mode == "exception"
                                       else "trajectory_validation", at - 1)

    def test_adapter_faults_include_partial_application_and_cleanup(self):
        for mode, at, phase, applies in (
                ("begin_before", 1, "begin_dispatch", 0),
                ("begin_after", 1, "begin_dispatch", 0),
                ("apply", 1, "apply", 0), ("apply", 2, "apply", 1),
                ("partial", 3, "apply", 2), ("cancelled", 2, "apply", 1),
                ("cleanup", 2, "end_dispatch", 3),
                ("post", 2, "post_dispatch_observation", 3)):
            with self.subTest(mode=mode, at=at):
                result = self._episode(world=FaultWorld(mode, at))
                self._assert_fault(result, phase, applies)
                if mode == "partial":
                    self.assertTrue(result[3].goal_physical)
                if mode == "begin_before":
                    self.assertEqual(result[1][-1]["stop_status"], "SAFE_UNCONFIRMED")

    def test_safety_verifier_exceptions_interrupt(self):
        self._assert_fault(self._episode(numeric=FaultNumeric()), "numeric_validation", 1)
        self._assert_fault(self._episode(verifier=FaultSymbolic()), "symbolic_verification", 1)

    def test_fault_without_ack_or_observation_is_unconfirmed(self):
        for acknowledgement, observation in ((False, True), (True, False)):
            with self.subTest(ack=acknowledgement, observation=observation):
                result = self._episode(world=FaultWorld(acknowledge=acknowledgement,
                                                       safe_observation=observation))
                self._assert_fault(result, "apply", 1)
                self.assertEqual(result[1][-1]["stop_status"], "SAFE_UNCONFIRMED")

    def test_observation_loss_after_an_existing_stop_does_not_stop_twice(self):
        class UncertainWorld(FaultWorld):
            def observe(self):
                state = super().observe()
                if state.t >= 1:
                    state.get("obj_cup").confidence = 0.5
                    state.uncertainty = aggregate_uncertainty(state)
                return state

        for world, policy in ((FaultWorld("after_stop"), FaultPolicy("unsafe")),
                              (UncertainWorld("after_stop"), MockPolicy())):
            with self.subTest(world=type(world).__name__):
                result = self._episode(world=world, policy=policy)
                self._assert_fault(result, "post_dispatch_observation", 1)
                self.assertEqual(len([e for e in result[1] if e["kind"] == "stop_request"]), 1)

    def test_transient_evidence_failures_still_produce_valid_fault_traces(self):
        for kind, phase, applies in (("safety2", "numeric_validation", 0),
                                     ("apply", "apply_evidence", 0)):
            with self.subTest(kind=kind):
                self._assert_fault(self._episode(tracer_kind=kind), phase, applies)

    def test_permanent_evidence_failure_cancels_before_reporting_incomplete_evidence(self):
        result = self._episode(tracer_kind="apply", permanent=True)
        metrics, events, replay, world, admission, error = result
        self.assertIsInstance(error, CoreEvidenceUnavailable)
        self.assertIsNone(metrics)
        self.assertEqual(world.apply_calls, 1)
        self.assertEqual(world.cancel_calls, 1)
        self.assertTrue(world.ended)
        self.assertEqual(admission.stop.outcome.status, "SAFE_CONFIRMED_MOCK")
        self.assertFalse(admission.stop.outcome.trace_complete)
        self.assertFalse(any(e["kind"] == "episode_end" for e in events))
        self.assertFalse(replay.valid)

    def test_stop_chain_write_failure_never_reports_complete_metrics(self):
        for kind in ("stop_request", "stop_cancel", "stop_safe_state", "interruption_gate"):
            with self.subTest(kind=kind):
                metrics, events, replay, world, admission, error = self._episode(
                    policy=FaultPolicy("unsafe"), tracer_kind=kind)
                self.assertIsInstance(error, CoreEvidenceUnavailable)
                self.assertIsNone(metrics)
                self.assertEqual(world.cancel_calls, 1)
                self.assertTrue(world.ended)
                self.assertEqual(admission.stop.outcome.status, "SAFE_CONFIRMED_MOCK")
                self.assertFalse(any(e["kind"] == "episode_end" for e in events))
                self.assertFalse(replay.valid)

    def test_observation_failure_logging_error_still_cancels_and_reports_incomplete_evidence(self):
        metrics, events, replay, world, admission, error = self._episode(
            world=FaultWorld("observation"), tracer_kind="observation_failure")
        self.assertIsInstance(error, CoreEvidenceUnavailable)
        self.assertIsNone(metrics)
        self.assertEqual(world.apply_calls, 1)
        self.assertEqual(world.cancel_calls, 1)
        self.assertEqual(admission.stop.outcome.status, "SAFE_CONFIRMED_MOCK")
        self.assertFalse(replay.valid)

    def test_policy_mutation_cannot_rewrite_safety_or_fault_evidence(self):
        class MutatingPolicy(MockPolicy):
            def step(self, action, state):
                state.t = 999
                state.robot.base_pose = (100.0, 100.0, 100.0)
                raise RuntimeError("policy mutated its private view")

        result = self._episode(policy=MutatingPolicy())
        self._assert_fault(result, "policy_step", 0)
        fault = next(e for e in result[1] if e["kind"] == "execution_fault")
        self.assertEqual(fault["sim_t"], 0)

    def test_task_plan_and_action_changes_after_a_chunk_cancel_dispatch(self):
        for mode in ("task_pre_apply", "task_pre_chunk", "plan", "action", "version"):
            with self.subTest(mode=mode):
                task = Task(id="execution-fault", mission="place cup",
                            goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)

                class RetainingOracle(ScriptedOracle):
                    def plan(self, mission, state):
                        self.graph = super().plan(mission, state)
                        return self.graph

                oracle = RetainingOracle(task.goal)

                class MutatingPolicy(MockPolicy):
                    def step(self, action, state):
                        trajectory = super().step(action, state)
                        if state.t == 1:
                            if mode == "task_pre_apply":
                                task.revision = "changed"
                            elif mode == "plan":
                                oracle.graph.nodes.reverse()
                            elif mode == "action":
                                oracle.graph.nodes[0].targets = ["obj_table"]
                            elif mode == "version":
                                oracle.graph.version += 1
                        return trajectory

                class MutatingWorld(FaultWorld):
                    def _apply_active(self, action, trajectory):
                        super()._apply_active(action, trajectory)
                        if mode == "task_pre_chunk" and self.state.t == 1:
                            task.revision = "changed"

                metrics, events, replay, world, _, error = self._episode(
                    task=task, reasoner=oracle, policy=MutatingPolicy(),
                    world=MutatingWorld("normal"))
                self.assertIsNone(error)
                self.assertTrue(metrics.aborted)
                self.assertEqual(metrics.replans, 0)
                self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
                self.assertEqual(world.cancel_calls, 1)
                self.assertEqual(next(e for e in events if e["kind"] == "stop_request")["reason"],
                                 "active_context_changed")
                self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                                 "INTERRUPTED")
                self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_fault_tampering(self):
        _, original, replay, _, _, _ = self._episode(policy=FaultPolicy())
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for name in ("scope", "stage", "boundary", "cycles", "outcome", "missing", "reason",
                         "success", "terminal", "late_apply"):
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    fault = next(e for e in events if e["kind"] == "execution_fault")
                    if name == "scope":
                        fault["dispatch_id"] = "wrong"
                    elif name == "stage":
                        fault["phase"] = "imaginary_stage"
                    elif name == "boundary":
                        fault["phase"] = "apply"
                    elif name == "cycles":
                        fault["cycles"] = 6
                    elif name == "outcome":
                        fault["execution_outcome"] = "SUCCEEDED"
                    elif name == "missing":
                        events.remove(fault)
                    elif name == "reason":
                        next(e for e in events if e["kind"] == "stop_request")["reason"] = "wrong"
                    elif name == "success":
                        events[-1]["task_success"] = True
                    elif name == "terminal":
                        events[-1]["terminal_observation"] = "OBSERVED"
                    else:
                        applied = copy.deepcopy(next(e for e in events if e["kind"] == "apply"))
                        events.insert(len(events) - 1, applied)
                    for sequence, event in enumerate(events):
                        event["sequence"] = sequence
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    self.assertFalse(replay_trace(path, expected_task_id="execution-fault").valid)


if __name__ == "__main__":
    unittest.main()
