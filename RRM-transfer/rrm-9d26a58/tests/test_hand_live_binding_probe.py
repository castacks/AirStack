"""CPU-only artifact/profile tests for the disabled live-binding smoke script."""

from copy import deepcopy
import hashlib
import json
import unittest

from rrm.hand_qualification import evaluate_hand_probe
from simulation.hand_live_binding_probe import (
    _NoActionArticulation, compare_profile, verify_artifacts,
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


if __name__ == "__main__":
    unittest.main()
