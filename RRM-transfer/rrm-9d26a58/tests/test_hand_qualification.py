"""Fail-closed tests for Kuka-Allegro qualification evidence."""

from __future__ import annotations

from copy import deepcopy
import unittest

from rrm.hand_qualification import evaluate_hand_probe


def qualified_probe() -> dict:
    limits = []
    for index in range(23):
        limits.append({
            "index": index, "name": f"joint-{index}",
            "lower_rad": -1.0, "upper_rad": 1.0,
            "clamped_default_position_rad": 0.0,
            "max_velocity_rad_s": 2.0, "max_effort": 5.0,
            "stiffness": 10.0, "damping": 1.0,
        })
    return {
        "schema_version": "rrm-hand-controller-probe/v1",
        "scene_recipe_sha256": "a" * 64,
        "ros_connected": False, "execution_dispatch": False,
        "joint_limits": limits,
        "controller": {
            "arm_targets_clipped_to_limits": True,
            "velocity_limits_respected": True,
            "arm_converged": True,
            "final_arm_position_error_rad": 0.001,
            "declared_hold_mode": "POSITION_HOLD",
            "max_observed_joint_velocities_rad_s": [1.0] * 23,
            "runtime_gain_override": {
                "applied_as_configured": True,
                "limits_changed": False,
            },
        },
        "post_command_reset_hashes_match": True,
        "post_command_reset_samples": [
            {"episode_id": f"episode-{index}", "state_sha256_rounded_1e-4": "b" * 64}
            for index in range(3)
        ],
        "contact_observation": {
            "fingertip_contact_sensor_active": True,
            "known_contact_detected": True,
            "red_block_named_in_peak_contacts": True,
            "baseline_peak_force_n": 0.0,
            "challenge_peak_force_n": 1.0,
            "detection_threshold_n": 0.1,
            "grasp_claimed": False,
        },
        "safe_state": {
            "safe_state_achieved": True, "controller_mode": "POSITION_HOLD",
            "active_motion_command": False,
            "joint_velocity_threshold_rad_s": 0.1,
            "object_velocity_threshold_m_s": 0.01,
            "consecutive_window_required": 5,
            "samples": [{
                "verdict": "SAFE_CONFIRMED",
                "max_joint_velocity_rad_s": 0.05,
                "max_object_velocity_m_s": 0.001,
                "consecutive_safe_count": 5,
            }],
        },
        "independent_stop": {
            "motion_started": True, "stop_command_sent": True,
            "safe_state_achieved": True, "controller_mode": "POSITION_HOLD",
            "active_motion_command": False, "stop_latency_s": 0.5,
            "commanded_joint_names": [f"iiwa7_joint_{index}" for index in range(1, 8)],
        },
    }


class HandQualificationTests(unittest.TestCase):
    def test_complete_probe_allows_only_one_bounded_contact_trial(self):
        result = evaluate_hand_probe(qualified_probe(), probe_sha256="c" * 64)
        self.assertTrue(result["ready_for_single_bounded_contact_trial"])
        self.assertFalse(result["contact_stability_qualified"])
        self.assertFalse(result["grasp_execution_qualified"])
        self.assertFalse(result["c06_c08_c09_complete"])
        self.assertFalse(result["execution_dispatch"])
        self.assertTrue(all(gate["status"] == "PASS" for gate in result["gates"].values()))

    def test_each_missing_safety_dependency_fails_closed(self):
        mutations = (
            ("controller", "velocity_limits_respected"),
            ("contact_observation", "known_contact_detected"),
            ("safe_state", "safe_state_achieved"),
            ("independent_stop", "safe_state_achieved"),
        )
        for section, field in mutations:
            with self.subTest(section=section, field=field):
                probe = deepcopy(qualified_probe())
                probe[section][field] = False
                self.assertFalse(evaluate_hand_probe(
                    probe, probe_sha256="c" * 64,
                )["ready_for_single_bounded_contact_trial"])

    def test_dispatch_or_ros_connected_probe_is_never_qualified(self):
        for field in ("execution_dispatch", "ros_connected"):
            with self.subTest(field=field):
                probe = qualified_probe()
                probe[field] = True
                result = evaluate_hand_probe(probe, probe_sha256="c" * 64)
                self.assertFalse(result["ready_for_single_bounded_contact_trial"])


if __name__ == "__main__":
    unittest.main()
