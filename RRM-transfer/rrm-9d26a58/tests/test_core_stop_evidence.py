"""Mock C08 interruption is fail-closed and replay-bound, not physical proof."""

import copy
import json
import tempfile
import time
import unittest
from dataclasses import replace
from threading import Event
from pathlib import Path

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
from rrm.benchmark_evidence import replay_trace
from rrm.core_stop import StopStateEvidence
from rrm.core_stop import CoreStopSupervisor
from rrm.core_deadlines import CoreCallLimits
from rrm.contracts import AdmissionGuard
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
    def __init__(self, admission, world, pending_hook=None):
        super().__init__()
        self.admission = admission
        self.world = world
        self.steps = 0
        self.outcome = None
        self.pending_hook = pending_hook

    def step(self, action, ws):
        self.steps += 1
        if self.steps == 2:
            self.outcome = self.admission.stop.request_stop(
                dispatch_id=self.world.active_dispatch_id, reason="operator_stop",
                pending_execution=self.pending_hook)
        return super().step(action, ws)


class NoCancelWorld(MockWorld):
    def cancel_dispatch(self, dispatch_id, generation):
        return False

    def observe_safe_state(self, dispatch_id, generation):
        return None


class StaleSafeWorld(MockWorld):
    def observe_safe_state(self, dispatch_id, generation):
        evidence = super().observe_safe_state(dispatch_id, generation)
        return replace(evidence, observed_at_monotonic=time.monotonic()-10.)


class FutureSafeWorld(MockWorld):
    def observe_safe_state(self, dispatch_id, generation):
        evidence = super().observe_safe_state(dispatch_id, generation)
        return replace(evidence, observed_at_monotonic=time.monotonic()+10.)


class CoreStopEvidenceTests(unittest.TestCase):
    def test_proof_age_includes_later_pending_execution_check(self):
        class StoppedWorld:
            def cancel_dispatch(self, dispatch_id, generation):
                return True

            def observe_safe_state(self, dispatch_id, generation):
                evidence = StopStateEvidence(dispatch_id, generation, 1, time.monotonic(),
                                             True, False, True, 'MOCK_HOLD', 'synthetic_mock')
                time.sleep(.12)
                return evidence

        def slow_pending():
            time.sleep(.12)
            return False

        supervisor = CoreStopSupervisor(AdmissionGuard(), CoreCallLimits(stop_s=.2))
        supervisor.bind(StoppedWorld(), Tracer(None, {}))
        outcome = supervisor.request_stop(dispatch_id='d', reason='operator_stop',
                                          pending_execution=slow_pending)
        self.assertEqual(outcome.status, 'SAFE_UNCONFIRMED')
        self.assertIsNotNone(outcome.safe_state)
        self.assertFalse(outcome.pending_execution)

    def _episode(self, world_type=MockWorld, *, pending_hook=None, call_limits=None):
        task = Task(id="c08-mock", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        admission = mock_admission()
        if call_limits is not None:
            admission = replace(admission, call_limits=call_limits)
        world = world_type()
        policy = StoppingPolicy(admission, world, pending_hook)
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

    def test_pending_unknown_and_later_resolution_replay_as_observed(self):
        for mode in ('timeout', 'invalid', 'exception', 'resolved'):
            with self.subTest(mode=mode):
                gate = Event()
                calls = []

                def status():
                    calls.append(True)
                    if mode == 'invalid':
                        return None
                    if mode == 'exception':
                        raise RuntimeError('status unavailable')
                    if mode == 'timeout' or len(calls) == 1:
                        gate.wait(.5)
                    return False

                try:
                    _, events, replay, policy = self._episode(
                        pending_hook=status, call_limits=CoreCallLimits(stop_s=.1))
                    expected = 'SAFE_CONFIRMED_MOCK' if mode == 'resolved' else 'SAFE_UNCONFIRMED'
                    self.assertEqual(policy.outcome.status, expected)
                    self.assertTrue(replay.valid, replay.findings)
                    safe = next(event for event in events if event['kind'] == 'stop_safe_state')
                    self.assertIs(safe['pending_status_known'], mode == 'resolved')
                finally:
                    gate.set()

    def test_old_or_future_safe_evidence_cannot_confirm_stop(self):
        for world_type in (StaleSafeWorld, FutureSafeWorld):
            with self.subTest(world=world_type.__name__):
                _, events, replay, policy = self._episode(world_type)
                self.assertEqual(policy.outcome.status, 'SAFE_UNCONFIRMED')
                self.assertIsNotNone(policy.outcome.safe_state)
                safe = next(event for event in events if event['kind'] == 'stop_safe_state')
                self.assertIs(safe['evidence_fresh'], False)
                self.assertTrue(replay.valid, replay.findings)

    def test_stop_evidence_rejects_invalid_identity_and_clock_types(self):
        good = dict(dispatch_id='dispatch', generation=1, observed_t=2,
                    observed_at_monotonic=10., motion_stopped=True,
                    active_motion_command=False, safe_condition_met=True,
                    controller_mode='MOCK_HOLD', evidence_kind='synthetic_mock')
        cases = [('dispatch_id', None), ('controller_mode', 1), ('evidence_kind', []),
                 ('generation', True), ('observed_t', False)]
        cases += [('observed_at_monotonic', value) for value in
                  (None, True, '10', -1., float('nan'), float('inf'), 10**1000)]
        for field, value in cases:
            with self.subTest(field=field, type=type(value).__name__):
                with self.assertRaises(ValueError):
                    StopStateEvidence(**{**good, field: value})

    def test_replay_independently_rejects_freshness_tampering(self):
        _, original, replay, _ = self._episode()
        self.assertTrue(replay.valid, replay.findings)
        with tempfile.TemporaryDirectory() as directory:
            for case in ('old_sample', 'future_sample', 'false_verdict', 'missing_window',
                         'wrong_clock', 'wrong_budget', 'reversed_window', 'bool_window',
                         'overflow_window', 'invalid_identity', 'proof_before_observation',
                         'future_proof'):
                with self.subTest(case=case):
                    events = copy.deepcopy(original)
                    safe = next(event for event in events if event['kind'] == 'stop_safe_state')
                    window = safe['observation_window']
                    if case == 'old_sample':
                        safe['evidence']['observed_at_monotonic'] -= 10.
                    elif case == 'future_sample':
                        safe['evidence']['observed_at_monotonic'] += 10.
                    elif case == 'false_verdict':
                        safe['evidence_fresh'] = False
                    elif case == 'missing_window':
                        safe.pop('observation_window')
                    elif case == 'wrong_clock':
                        window['clock'] = 'unrelated_clock'
                    elif case == 'wrong_budget':
                        window['max_age_s'] *= 2
                    elif case == 'reversed_window':
                        window['started_s'] = window['finished_s']+1.
                    elif case == 'bool_window':
                        window['started_s'] = True
                    elif case == 'overflow_window':
                        window['finished_s'] = 10**1000
                    elif case == 'proof_before_observation':
                        window['proof_checked_s'] = window['started_s']-1.
                    elif case == 'future_proof':
                        window['proof_checked_s'] = safe['monotonic_time_s']+1.
                    else:
                        safe['evidence']['controller_mode'] = None
                    path = Path(directory)/(case+'.jsonl')
                    path.write_text(''.join(json.dumps(event)+'\n' for event in events))
                    result = replay_trace(path, expected_task_id='c08-mock')
                    self.assertFalse(result.valid, result.findings)

    def test_replay_rejects_unknown_pending_status_promoted_to_clear(self):
        _, original, _, _ = self._episode()
        with tempfile.TemporaryDirectory() as directory:
            for case in ('request_unknown', 'proof_unknown', 'missing_known', 'deadline_known'):
                with self.subTest(case=case):
                    events = copy.deepcopy(original)
                    request = next(event for event in events if event['kind'] == 'stop_request')
                    safe = next(event for event in events if event['kind'] == 'stop_safe_state')
                    if case == 'request_unknown':
                        request['pending_status_known'] = False
                    elif case == 'proof_unknown':
                        safe['pending_status_known'] = False
                    elif case == 'missing_known':
                        safe.pop('pending_status_known')
                    else:
                        budget = safe['observation_window']['max_age_s']
                        safe['pending_status_deadline'] = dict(
                            call='pending_status', timeout_s=budget, elapsed_s=2*budget,
                            operation_pending=True)
                    path = Path(directory)/(case+'.jsonl')
                    path.write_text(''.join(json.dumps(event)+'\n' for event in events))
                    result = replay_trace(path, expected_task_id='c08-mock')
                    self.assertFalse(result.valid)
                    self.assertTrue(any(finding.startswith('invalid_pending_status')
                                        for finding in result.findings), result.findings)

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
