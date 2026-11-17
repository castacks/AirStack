"""Stop results are scoped to one dispatch and one still-latched generation."""
import unittest
import time

from rrm.contracts import AdmissionGuard
from rrm.core_stop import CoreStopSupervisor, StopStateEvidence
from rrm.trace import Tracer


class StoppedWorld:
    def cancel_dispatch(self, dispatch_id, generation):
        return True

    def observe_safe_state(self, dispatch_id, generation):
        return StopStateEvidence(dispatch_id, generation, 0, time.monotonic(),
                                 True, False, True, 'MOCK_HOLD', 'synthetic_mock')


class StopCacheTests(unittest.TestCase):
    def supervisor(self):
        guard = AdmissionGuard()
        supervisor = CoreStopSupervisor(guard)
        supervisor.bind(StoppedWorld(), Tracer(None, {}))
        outcome = supervisor.request_stop(dispatch_id='first', reason='operator_stop')
        self.assertEqual(outcome.status, 'SAFE_CONFIRMED_MOCK')
        return guard, supervisor, outcome

    def test_duplicate_request_retains_identity_and_generation(self):
        guard, supervisor, outcome = self.supervisor()
        again = supervisor.request_stop(dispatch_id='first', reason='operator_stop')
        self.assertIs(again, outcome)
        self.assertEqual(guard.generation, outcome.generation)

    def test_different_dispatch_cannot_borrow_confirmation(self):
        guard, supervisor, outcome = self.supervisor()
        with self.assertRaisesRegex(ValueError, 'dispatch'):
            supervisor.request_stop(dispatch_id='different', reason='operator_stop')
        self.assertEqual(guard.generation, outcome.generation)

    def test_reset_guard_is_reinhibited_before_cached_result_is_refused(self):
        for dispatch in ('first', 'different'):
            with self.subTest(dispatch=dispatch):
                guard, supervisor, outcome = self.supervisor()
                self.assertTrue(guard.reset(generation=outcome.generation, authorized=True,
                                            safe_confirmed=True, evidence_ref='new-evidence'))
                reset_generation = guard.generation
                with self.assertRaisesRegex(RuntimeError, 'generation'):
                    supervisor.request_stop(dispatch_id=dispatch, reason='operator_stop')
                self.assertEqual(guard.generation, reset_generation+1)
                self.assertTrue(guard.retain_stop(generation=guard.generation))
                # The previous result remains history, never a new confirmation.
                self.assertIs(supervisor.outcome, outcome)

    def test_newer_stop_generation_cannot_reuse_old_result(self):
        guard, supervisor, _ = self.supervisor()
        generation = guard.stop()
        with self.assertRaisesRegex(RuntimeError, 'generation'):
            supervisor.request_stop(dispatch_id='first', reason='operator_stop')
        self.assertEqual(guard.generation, generation)

    def test_malformed_reset_cannot_open_cached_generation(self):
        for change in ({'generation': True}, {'generation': 1.0}, {'evidence_ref': None},
                       {'evidence_ref': 1}, {'evidence_ref': []}, {'evidence_ref': '  '}):
            with self.subTest(change=change):
                guard, _, outcome = self.supervisor()
                request = dict(generation=outcome.generation, authorized=True,
                               safe_confirmed=True, evidence_ref='fresh')
                self.assertFalse(guard.reset(**{**request, **change}))
                self.assertEqual(guard.generation, outcome.generation)
                self.assertTrue(guard.retain_stop(generation=outcome.generation))


if __name__ == '__main__':
    unittest.main()
