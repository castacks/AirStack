"""CPU-only tests for the qualified live tabletop fixture contract."""

from copy import deepcopy
import json
from pathlib import Path
import unittest

from simulation.hand_live_qualified_fixture import (
    EXPECTED_JOINT_NAMES,
    validate_qualified_fixture_spec,
)


class QualifiedLiveFixtureContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        artifact = Path(__file__).parents[1] / \
            ".rrm-artifacts/hand-controller-probe-20260924-s/probe.json"
        cls.probe = json.loads(artifact.read_text())
        cls.physics_dt_s = cls.probe["physics_dt_s"]

    def test_checked_in_qualification_probe_defines_complete_fixture(self):
        self.assertEqual(validate_qualified_fixture_spec(
            self.probe, requested_physics_dt_s=self.physics_dt_s), [])
        self.assertEqual(tuple(item["name"] for item in self.probe["joint_limits"]),
                         EXPECTED_JOINT_NAMES)

    def test_contract_fails_closed_for_each_fixture_binding(self):
        cases = [
            (("schema_version", "wrong"), "probe_schema_mismatch"),
            (("asset_url", "omniverse://wrong"), "asset_url_mismatch"),
            (("asset_sha256", "not-a-digest"), "asset_sha256_missing"),
            (("scene_seed", 1), "scene_seed_mismatch"),
            (("scene_entities", {}), "scene_entities_mismatch"),
            (("articulation_root", "/World/wrong"), "articulation_root_mismatch"),
            (("joint_limits", []), "joint_profile_incomplete"),
            (("controller", {}), "configured_stiffness_invalid"),
            (("safe_state", {}), "safe_state_contract_invalid"),
        ]
        for (key, value), expected in cases:
            with self.subTest(expected=expected):
                probe = deepcopy(self.probe)
                probe[key] = value
                self.assertIn(expected, validate_qualified_fixture_spec(
                    probe, requested_physics_dt_s=self.physics_dt_s))
        self.assertIn("physics_dt_mismatch", validate_qualified_fixture_spec(
            self.probe, requested_physics_dt_s=self.physics_dt_s * 2))

    def test_joint_and_gain_mutations_are_rejected(self):
        probe = deepcopy(self.probe)
        probe["joint_limits"][10]["clamped_default_position_rad"] = -10.0
        self.assertIn("joint_10_fixture_invalid", validate_qualified_fixture_spec(
            probe, requested_physics_dt_s=self.physics_dt_s))

        for key, bad_value, expected in (
                ("configured_stiffness", [1.0], "configured_stiffness_invalid"),
                ("configured_damping", [-1.0] * 23, "configured_damping_invalid"),
                ("applied_as_configured", False, "qualified_gain_evidence_invalid")):
            with self.subTest(key=key):
                probe = deepcopy(self.probe)
                probe["controller"]["runtime_gain_override"][key] = bad_value
                self.assertIn(expected, validate_qualified_fixture_spec(
                    probe, requested_physics_dt_s=self.physics_dt_s))


if __name__ == "__main__":
    unittest.main()
