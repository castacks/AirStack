"""Read-only pending-status hooks cannot block cancellation or invent knowledge."""
import time
from threading import Event
import unittest

from rrm.contracts import AdmissionGuard
from rrm.core_deadlines import CoreCallLimits
from rrm.core_stop import CoreStopSupervisor, StopStateEvidence


class MemoryTracer:
    def __init__(self):
        self.events = []

    def event(self, kind, **fields):
        self.events.append(dict(kind=kind, **fields))


class StoppedWorld:
    def __init__(self):
        self.cancel_at = None

    def cancel_dispatch(self, dispatch_id, generation):
        self.cancel_at = time.monotonic()
        return True

    def observe_safe_state(self, dispatch_id, generation):
        return StopStateEvidence(dispatch_id, generation, 1, time.monotonic(),
                                 True, False, True, 'MOCK_HOLD', 'synthetic_mock')


class CorePendingStatusTests(unittest.TestCase):
    def supervisor(self, registry=None):
        world, trace = StoppedWorld(), MemoryTracer()
        supervisor = CoreStopSupervisor(AdmissionGuard(), CoreCallLimits(stop_s=.05), registry)
        supervisor.bind(world, trace)
        return supervisor, world, trace

    def test_stalled_registry_or_request_hook_does_not_block_cancel(self):
        for location in ('registry', 'request'):
            with self.subTest(location=location):
                gate = Event()

                def stalled():
                    gate.wait(.5)
                    return False

                supervisor, world, trace = self.supervisor(stalled if location == 'registry' else None)
                started = time.monotonic()
                try:
                    outcome = supervisor.request_stop(
                        dispatch_id='d', reason='operator_stop',
                        pending_execution=stalled if location == 'request' else None)
                    elapsed = time.monotonic()-started
                    self.assertLess(elapsed, .35)
                    self.assertLess(world.cancel_at-started, .25)
                    self.assertEqual(outcome.status, 'SAFE_UNCONFIRMED')
                    self.assertTrue(outcome.pending_execution)
                    for event in trace.events:
                        if event['kind'] in ('stop_request', 'stop_safe_state'):
                            self.assertIs(event['pending_status_known'], False)
                            self.assertEqual(event['pending_status_deadline']['call'], 'pending_status')
                    gate.set()
                    time.sleep(.02)
                    self.assertIs(supervisor.outcome, outcome)
                    self.assertEqual(outcome.status, 'SAFE_UNCONFIRMED')
                finally:
                    gate.set()

    def test_nonboolean_status_is_unknown(self):
        for value in (None, 0, 1, '', 'false', [], {}):
            with self.subTest(type=type(value).__name__):
                supervisor, _, trace = self.supervisor()
                outcome = supervisor.request_stop(dispatch_id='d', reason='operator_stop',
                                                  pending_execution=lambda: value)
                self.assertEqual(outcome.status, 'SAFE_UNCONFIRMED')
                self.assertTrue(outcome.pending_execution)
                safe = next(event for event in trace.events if event['kind'] == 'stop_safe_state')
                self.assertIs(safe['pending_status_known'], False)

    def test_initial_unknown_can_be_resolved_by_fresh_final_query(self):
        gate = Event()
        calls = []

        def status():
            calls.append(True)
            if len(calls) == 1:
                gate.wait(.5)
            return False

        supervisor, _, trace = self.supervisor()
        try:
            outcome = supervisor.request_stop(dispatch_id='d', reason='operator_stop',
                                              pending_execution=status)
            request = next(event for event in trace.events if event['kind'] == 'stop_request')
            safe = next(event for event in trace.events if event['kind'] == 'stop_safe_state')
            self.assertIs(request['pending_status_known'], False)
            self.assertIs(safe['pending_status_known'], True)
            self.assertEqual(outcome.status, 'SAFE_CONFIRMED_MOCK')
            self.assertFalse(outcome.pending_execution)
        finally:
            gate.set()

    def test_known_boolean_status_remains_distinct(self):
        for value in (False, True):
            with self.subTest(value=value):
                supervisor, _, trace = self.supervisor()
                outcome = supervisor.request_stop(dispatch_id='d', reason='operator_stop',
                                                  pending_execution=lambda: value)
                expected = 'SAFE_UNCONFIRMED' if value else 'SAFE_CONFIRMED_MOCK'
                self.assertEqual(outcome.status, expected)
                safe = next(event for event in trace.events if event['kind'] == 'stop_safe_state')
                self.assertIs(safe['pending_status_known'], True)
                self.assertIsNone(safe['pending_status_deadline'])

    def test_hook_exception_is_unknown_and_cancellation_still_occurs(self):
        def unavailable():
            raise RuntimeError('status unavailable')

        supervisor, world, trace = self.supervisor()
        outcome = supervisor.request_stop(dispatch_id='d', reason='operator_stop',
                                          pending_execution=unavailable)
        self.assertIsNotNone(world.cancel_at)
        self.assertEqual(outcome.status, 'SAFE_UNCONFIRMED')
        safe = next(event for event in trace.events if event['kind'] == 'stop_safe_state')
        self.assertIs(safe['pending_status_known'], False)


if __name__ == '__main__':
    unittest.main()
