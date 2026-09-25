"""Fail-closed qualification of isolated Kuka-Allegro probe evidence.

This module evaluates a persisted simulator report. It has no Isaac, ROS, controller,
or dispatch dependency and grants no execution authority.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_SCHEMA = "rrm-hand-controller-probe/v1"
QUALIFICATION_SCHEMA = "rrm-hand-qualification/v1"


def _gate(passed: bool, *reasons: str) -> dict[str, Any]:
    return {
        "status": "PASS" if passed else "FAIL",
        "reasons": [] if passed else [reason for reason in reasons if reason],
    }


def evaluate_hand_probe(probe: dict[str, Any], *, probe_sha256: str) -> dict[str, Any]:
    """Evaluate controller, reset, contact-observer, stop, and safe-state gates."""
    observation_only = (
        probe.get("schema_version") == EXPECTED_SCHEMA
        and probe.get("ros_connected") is False
        and probe.get("execution_dispatch") is False
    )

    limits = probe.get("joint_limits") if isinstance(probe.get("joint_limits"), list) else []
    controller = probe.get("controller") if isinstance(probe.get("controller"), dict) else {}
    gain_record = controller.get("runtime_gain_override") if isinstance(
        controller.get("runtime_gain_override"), dict
    ) else {}
    observed_velocities = controller.get("max_observed_joint_velocities_rad_s")
    limits_complete = len(limits) == 23 and all(
        isinstance(item, dict)
        and item.get("lower_rad") is not None
        and item.get("upper_rad") is not None
        and item.get("max_velocity_rad_s") is not None
        and item.get("max_effort") is not None
        and item.get("stiffness") is not None
        and item.get("damping") is not None
        and item.get("lower_rad") <= item.get("clamped_default_position_rad") <= item.get("upper_rad")
        for item in limits
    )
    controller_passed = (
        observation_only
        and limits_complete
        and controller.get("arm_targets_clipped_to_limits") is True
        and controller.get("velocity_limits_respected") is True
        and controller.get("arm_converged") is True
        and float(controller.get("final_arm_position_error_rad", float("inf"))) <= 0.01
        and controller.get("declared_hold_mode") == "POSITION_HOLD"
        and gain_record.get("applied_as_configured") is True
        and gain_record.get("limits_changed") is False
        and isinstance(observed_velocities, list)
        and len(observed_velocities) == len(limits)
        and all(
            isinstance(observed, (int, float))
            and math.isfinite(observed)
            and 0.0 <= observed <= float(limit["max_velocity_rad_s"])
            for observed, limit in zip(observed_velocities, limits)
        )
    )

    resets = probe.get("post_command_reset_samples")
    reset_passed = (
        observation_only
        and probe.get("post_command_reset_hashes_match") is True
        and isinstance(resets, list)
        and len(resets) >= 3
        and len({item.get("episode_id") for item in resets}) == len(resets)
        and all(item.get("state_sha256_rounded_1e-4") for item in resets)
    )

    contact = probe.get("contact_observation") if isinstance(
        probe.get("contact_observation"), dict
    ) else {}
    contact_observer_passed = (
        observation_only
        and contact.get("fingertip_contact_sensor_active") is True
        and contact.get("known_contact_detected") is True
        and contact.get("red_block_named_in_peak_contacts") is True
        and float(contact.get("baseline_peak_force_n", float("inf")))
        < float(contact.get("detection_threshold_n", 0.0))
        and float(contact.get("challenge_peak_force_n", 0.0))
        >= float(contact.get("detection_threshold_n", float("inf")))
        and contact.get("grasp_claimed") is False
    )

    safe_state = probe.get("safe_state") if isinstance(probe.get("safe_state"), dict) else {}
    safe_state_passed = (
        observation_only
        and safe_state.get("safe_state_achieved") is True
        and safe_state.get("controller_mode") == "POSITION_HOLD"
        and safe_state.get("active_motion_command") is False
        and bool(safe_state.get("samples"))
        and safe_state["samples"][-1].get("verdict") == "SAFE_CONFIRMED"
        and float(safe_state["samples"][-1].get("max_joint_velocity_rad_s", float("inf")))
        <= float(safe_state.get("joint_velocity_threshold_rad_s", 0.0))
        and float(safe_state["samples"][-1].get("max_object_velocity_m_s", float("inf")))
        <= float(safe_state.get("object_velocity_threshold_m_s", 0.0))
        and int(safe_state["samples"][-1].get("consecutive_safe_count", 0))
        >= int(safe_state.get("consecutive_window_required", 1))
    )

    stop = probe.get("independent_stop") if isinstance(
        probe.get("independent_stop"), dict
    ) else {}
    stop_passed = (
        observation_only
        and stop.get("motion_started") is True
        and stop.get("stop_command_sent") is True
        and stop.get("safe_state_achieved") is True
        and stop.get("controller_mode") == "POSITION_HOLD"
        and stop.get("active_motion_command") is False
        and stop.get("commanded_joint_names") == [f"iiwa7_joint_{index}" for index in range(1, 8)]
        and 0.0 < float(stop.get("stop_latency_s", float("inf"))) <= 1.0
    )

    gates = {
        "controller_limits": _gate(controller_passed, "controller_or_limit_evidence_incomplete"),
        "post_command_reset": _gate(reset_passed, "repeatable_distinct_reset_evidence_missing"),
        "contact_observer": _gate(contact_observer_passed, "known_filtered_contact_not_observed"),
        "safe_state": _gate(safe_state_passed, "safe_state_not_confirmed"),
        "independent_stop": _gate(stop_passed, "independent_stop_not_confirmed_within_1s"),
    }
    ready_for_contact_trial = all(item["status"] == "PASS" for item in gates.values())
    return {
        "schema_version": QUALIFICATION_SCHEMA,
        "probe_sha256": probe_sha256,
        "scene_recipe_sha256": probe.get("scene_recipe_sha256"),
        "gates": gates,
        "ready_for_single_bounded_contact_trial": ready_for_contact_trial,
        "contact_stability_qualified": False,
        "grasp_execution_qualified": False,
        "c06_c08_c09_complete": False,
        "execution_dispatch": False,
    }


def evaluate_hand_probe_file(path: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    return evaluate_hand_probe(
        json.loads(payload), probe_sha256=hashlib.sha256(payload).hexdigest(),
    )
