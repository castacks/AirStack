"""CPU-only artifact/profile tests for the disabled live-binding smoke script."""

from copy import deepcopy
import hashlib
import json
import unittest

from rrm.hand_qualification import evaluate_hand_probe
from simulation.hand_live_binding_probe import (
    _NoActionArticulation, compare_profile, validate_idle_heartbeat, verify_artifacts,
)
from test_hand_qualification import qualified_probe


class LiveBindingProbeTests(unittest.TestCase):
    def setUp(self):
        self.probe = qualified_probe()
        self.probe_bytes = json.dumps(self.probe, sort_keys=True).encode()
        self.qualification = evaluate_hand_probe(self.probe,
            probe_sha256=hashlib.sha256(self.probe_bytes).hexdigest())
        self.qualification_bytes = json.dumps(self.qualification, sort_keys=True).encode()
        self.live = [{"index": index, "name": item["name"],
                      "lower_rad": item["lower_rad"],
                      "upper_rad": item["upper_rad"],
                      "max_velocity_rad_s": item["max_velocity_rad_s"]}
                     for index, item in enumerate(self.probe["joint_limits"])]

    def test_exact_qualification_and_profile_pass(self):
        probe, qualification = verify_artifacts(self.probe_bytes,
                                                 self.qualification_bytes)
        self.assertEqual(qualification, self.qualification)
        self.assertEqual(compare_profile(probe, self.live), [])

    def test_mutation_and_nonpassing_gate_fail_closed(self):
        altered = deepcopy(self.probe)
        altered["joint_limits"][0]["upper_rad"] = 0.5
        with self.assertRaises(ValueError):
            verify_artifacts(json.dumps(altered, sort_keys=True).encode(),
                             self.qualification_bytes)
        altered_qualification = deepcopy(self.qualification)
        altered_qualification["gates"]["independent_stop"]["status"] = "FAIL"
        with self.assertRaises(ValueError):
            verify_artifacts(self.probe_bytes,
                             json.dumps(altered_qualification, sort_keys=True).encode())

    def test_name_index_and_each_numeric_limit_mismatch(self):
        for key, value in (("name", "changed"), ("index", 99),
                           ("lower_rad", -0.5), ("upper_rad", 0.5),
                           ("max_velocity_rad_s", 3.0)):
            with self.subTest(key=key):
                live = deepcopy(self.live)
                live[0][key] = value
                self.assertTrue(compare_profile(self.probe, live))
        self.assertEqual(compare_profile(self.probe, self.live[:-1]),
                         ["joint_count_not_23"])

    def test_no_action_wrapper_forbids_apply(self):
        wrapped = _NoActionArticulation(object())
        with self.assertRaisesRegex(RuntimeError, "forbids_apply_action"):
            wrapped.apply_action(object())
        self.assertEqual(wrapped.apply_calls, 1)

    def test_idle_heartbeat_report_requires_exact_healthy_zero_action_run(self):
        evidence = {"tick_count": 1000, "healthy": True, "reason": "HEALTHY",
                    "motion_enabled": False, "max_tick_duration_s": 0.001}
        self.assertEqual(validate_idle_heartbeat(statuses=["IDLE"] * 1000,
            evidence=evidence, apply_action_calls=0, expected_ticks=1000,
            max_tick_gap_s=0.1), [])
        cases = [
            (["IDLE"] * 999, evidence, 0, "idle_tick_count_mismatch"),
            (["IDLE"] * 999 + ["INHIBITED"], evidence, 0, "non_idle_tick"),
            (["IDLE"] * 1000, evidence, 1, "action_call_observed"),
            (["IDLE"] * 1000, {**evidence, "tick_count": 999}, 0,
             "heartbeat_tick_count_mismatch"),
            (["IDLE"] * 1000, {**evidence, "healthy": False,
                                "reason": "TICK_STALE"}, 0, "heartbeat_unhealthy"),
            (["IDLE"] * 1000, {**evidence, "motion_enabled": True}, 0,
             "motion_not_inhibited"),
            (["IDLE"] * 1000, {**evidence, "max_tick_duration_s": 0.2}, 0,
             "tick_duration_exceeded"),
        ]
        for statuses, changed, calls, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                self.assertIn(expected_error, validate_idle_heartbeat(statuses=statuses,
                    evidence=changed, apply_action_calls=calls, expected_ticks=1000,
                    max_tick_gap_s=0.1))


if __name__ == "__main__":
    unittest.main()
