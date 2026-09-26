"""CPU-only contract tests for the action-applying live probe validators."""

import unittest

from simulation.hand_live_action_probe import (
    _has_ordered_events,
    sample_until_safe,
    validate_action_evidence,
)
from simulation.hand_live_boundary_probe import validate_boundary_evidence


class LiveActionProbeValidatorTests(unittest.TestCase):
    def setUp(self):
        self.valid = {
            "profile_mismatches": [],
            "qualified_scene_equivalent": True,
            "target_status": "TARGET_APPLIED",
            "target_motion_observed": True,
            "stop_accepted": True,
            "stop_status": "HOLD_APPLIED",
            "stop_liveness": {"stop_pending": False, "stop_to_hold_s": 0.01,
                              "max_stop_to_hold_s": 0.1},
            "required_safe_count": 5,
            "stop_initial_safe_count": 5,
            "stop_safe_count": 5,
            "stop_events": ["C09_ADAPTER_ENQUEUE", "C09_ADAPTER_APPLY_INTENT",
                            "C08_ADAPTER_STOP_REQUEST", "C08_ADAPTER_HOLD_APPLIED"],
            "watchdog_reason": "TICK_STALE",
            "watchdog_status": "HOLD_APPLIED",
            "watchdog_target_motion_observed": True,
            "watchdog_initial_safe_count": 5,
            "watchdog_safe_count": 5,
            "watchdog_events": ["C09_ADAPTER_ENQUEUE", "C09_ADAPTER_APPLY_INTENT",
                                "C08_ADAPTER_LIVENESS_FAULT"],
        }

    def test_accepts_complete_measured_evidence(self):
        self.assertEqual(validate_action_evidence(**self.valid), [])
        self.assertTrue(_has_ordered_events(["a", "noise", "b"], ("a", "b")))

    def test_safe_sampler_is_bounded_and_stops_on_first_evidence(self):
        class World:
            def __init__(self):
                self.steps = 0

            def step(self, *, render):
                self.steps += 1

        class Adapter:
            def __init__(self, safe_at):
                self.calls = 0
                self.safe_at = safe_at

            def sample(self, generation):
                self.calls += 1
                return {"generation": generation} if self.calls == self.safe_at else None

        world, adapter = World(), Adapter(3)
        evidence, steps = sample_until_safe(world=world, adapter=adapter,
            generation=7, max_steps=5)
        self.assertEqual((evidence, steps), ({"generation": 7}, 3))
        self.assertEqual(world.steps, 3)
        world, adapter = World(), Adapter(6)
        self.assertEqual(sample_until_safe(world=world, adapter=adapter,
            generation=7, max_steps=5), (None, 5))
        self.assertEqual(world.steps, 5)

    def test_fails_closed_for_each_evidence_axis(self):
        cases = [
            ({"profile_mismatches": ["joint_0_identity"]}, "profile_mismatch"),
            ({"qualified_scene_equivalent": False},
             "qualified_scene_not_recreated"),
            ({"target_status": "IDLE"}, "target_not_applied"),
            ({"target_motion_observed": False}, "target_motion_not_observed"),
            ({"stop_initial_safe_count": 4},
             "stop_initial_safe_state_unconfirmed"),
            ({"stop_accepted": False}, "stop_not_accepted"),
            ({"stop_status": "INHIBITED"}, "stop_hold_not_applied"),
            ({"stop_liveness": {"stop_pending": True}}, "stop_deadline_not_met"),
            ({"stop_safe_count": 4}, "stop_safe_state_unconfirmed"),
            ({"stop_events": []}, "stop_ledger_sequence_missing"),
            ({"watchdog_reason": "HEALTHY"}, "watchdog_did_not_fence"),
            ({"watchdog_target_motion_observed": False},
             "watchdog_target_motion_not_observed"),
            ({"watchdog_initial_safe_count": 4},
             "watchdog_initial_safe_state_unconfirmed"),
            ({"watchdog_status": "IDLE"}, "watchdog_hold_not_applied"),
            ({"watchdog_safe_count": 0}, "watchdog_safe_state_unconfirmed"),
            ({"watchdog_events": []}, "watchdog_ledger_sequence_missing"),
        ]
        for change, expected in cases:
            with self.subTest(expected=expected):
                self.assertIn(expected,
                    validate_action_evidence(**{**self.valid, **change}))


class LiveBoundaryProbeValidatorTests(unittest.TestCase):
    def setUp(self):
        self.valid = {
            "profile_mismatches": [],
            "qualified_scene_equivalent": True,
            "reset_succeeded": True,
            "target_status": "TARGET_APPLIED",
            "target_motion_observed": True,
            "stop_accepted": True,
            "hold_status": "HOLD_APPLIED",
            "stopped_confirmed": True,
            "required_safe_count": 5,
            "safe_count": 5,
            "boundary_events": ["C06_AUTHORIZATION_CONSUMED", "C08_RESET",
                "C06_AUTHORIZATION_CONSUMED", "C09_DISPATCH_INTENT",
                "C08_STOP_RECEIVED", "C08_CANCEL_ACCEPTED", "C08_MOTION_STOPPED",
                "C08_SAFE_CONFIRMED", "C09_DISPATCH_RECONCILED"],
            "adapter_events": ["C09_ADAPTER_ENQUEUE", "C09_ADAPTER_APPLY_INTENT",
                "C08_ADAPTER_STOP_REQUEST", "C08_ADAPTER_HOLD_APPLIED"],
        }

    def test_accepts_complete_boundary_lifecycle(self):
        self.assertEqual(validate_boundary_evidence(**self.valid), [])

    def test_fails_closed_for_each_boundary_axis(self):
        cases = [
            ({"profile_mismatches": ["joint_count_not_23"]}, "profile_mismatch"),
            ({"qualified_scene_equivalent": False},
             "qualified_scene_not_recreated"),
            ({"reset_succeeded": False}, "authorized_reset_failed"),
            ({"target_status": "IDLE"}, "target_not_applied"),
            ({"target_motion_observed": False}, "target_motion_not_observed"),
            ({"stop_accepted": False}, "boundary_stop_not_accepted"),
            ({"hold_status": "INHIBITED"}, "hold_not_applied"),
            ({"safe_count": 4}, "stopped_state_not_confirmed"),
            ({"stopped_confirmed": False}, "stopped_state_not_confirmed"),
            ({"boundary_events": []}, "boundary_ledger_sequence_missing"),
            ({"adapter_events": []}, "adapter_ledger_sequence_missing"),
        ]
        for change, expected in cases:
            with self.subTest(expected=expected):
                self.assertIn(expected,
                    validate_boundary_evidence(**{**self.valid, **change}))


if __name__ == "__main__":
    unittest.main()
