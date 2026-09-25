"""CPU-only contract tests for the live Isaac physics-callback probe."""

import unittest

from simulation.hand_live_callback_probe import (
    PhysicsCallbackRecorder,
    validate_callback_run,
)


class _Clock:
    def __init__(self, values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


class _Adapter:
    def __init__(self):
        self.times = []

    def tick(self, *, now):
        self.times.append(now)
        return "IDLE"


class LiveCallbackProbeTests(unittest.TestCase):
    def test_recorder_preserves_callback_timing_and_tick_context(self):
        adapter = _Adapter()
        recorder = PhysicsCallbackRecorder(adapter,
            asset_is_active=lambda: False,
            clock=_Clock([1.0, 1.001, 1.02, 1.022]))
        recorder(1 / 120)
        recorder(1 / 120)
        self.assertEqual(adapter.times, [1.0, 1.02])
        self.assertEqual(recorder.statuses, ["IDLE", "IDLE"])
        self.assertEqual(recorder.physics_dts_s, [1 / 120, 1 / 120])
        self.assertEqual(recorder.asset_active, [False, False])
        self.assertAlmostEqual(recorder.callback_gaps_s[0], 0.02)
        self.assertAlmostEqual(recorder.callback_durations_s[0], 0.001)
        self.assertAlmostEqual(recorder.callback_durations_s[1], 0.002)

    def test_validator_fails_closed_for_each_callback_invariant(self):
        evidence = {"tick_count": 3, "healthy": True, "reason": "HEALTHY",
                    "motion_enabled": False}
        valid = dict(statuses=["IDLE"] * 3, physics_dts_s=[1 / 120] * 3,
            callback_gaps_s=[0.01, 0.02], asset_active=[False] * 3,
            evidence=evidence,
            apply_action_calls=0, expected_steps=3, expected_physics_dt_s=1 / 120,
            max_callback_gap_s=0.1, initial_max_velocity_rad_s=0.0,
            allowed_velocity_rad_s=1e-6)
        self.assertEqual(validate_callback_run(**valid), [])
        cases = [
            ({"statuses": ["IDLE"] * 2}, "callback_count_mismatch"),
            ({"statuses": ["IDLE", "INHIBITED", "IDLE"]}, "non_idle_callback"),
            ({"physics_dts_s": [1 / 120, 0.01, 1 / 120]}, "physics_dt_mismatch"),
            ({"callback_gaps_s": [0.01, 0.2]}, "callback_gap_exceeded"),
            ({"asset_active": [False, True, False]},
             "asset_active_during_callback"),
            ({"evidence": {**evidence, "tick_count": 2}},
             "heartbeat_tick_count_mismatch"),
            ({"evidence": {**evidence, "healthy": False, "reason": "TICK_STALE"}},
             "heartbeat_unhealthy"),
            ({"evidence": {**evidence, "motion_enabled": True}},
             "motion_not_inhibited"),
            ({"apply_action_calls": 1}, "action_call_observed"),
            ({"initial_max_velocity_rad_s": 2e-6}, "joint_velocity_observed"),
            ({"initial_max_velocity_rad_s": float("nan")},
             "invalid_joint_state_measurement"),
        ]
        for change, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                self.assertIn(expected_error,
                    validate_callback_run(**{**valid, **change}))


if __name__ == "__main__":
    unittest.main()
