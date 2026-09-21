"""Fail-closed command mission sequencing and recovery tests; no ROS execution."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

import airstack_command_mission as mission
from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal


def proposal(kind: DroneTaskKind, action_id: str) -> DroneTaskProposal:
    common = dict(task_id="task-1", action_id=action_id, robot_name="robot_1", kind=kind)
    if kind is DroneTaskKind.TAKEOFF:
        return DroneTaskProposal(**common, target_altitude_m=1.5, velocity_m_s=0.5)
    if kind is DroneTaskKind.LAND:
        return DroneTaskProposal(**common, velocity_m_s=0.5)
    return DroneTaskProposal(
        **common, min_altitude_agl_m=1.0, max_altitude_agl_m=3.0,
        min_flight_speed_m_s=0.5, max_flight_speed_m_s=2.0, time_limit_s=15.0,
    )


def write_plan(path: Path) -> None:
    takeoff = proposal(DroneTaskKind.TAKEOFF, "takeoff-0")
    explore = proposal(DroneTaskKind.EXPLORE, "explore-1")
    recovery = proposal(DroneTaskKind.LAND, "takeoff-0-recovery-land")
    path.write_text(json.dumps({
        "schema_version": "rrm-airstack-command-plan/v1",
        "actions": [takeoff.model_dump(mode="json"), explore.model_dump(mode="json")],
        "recovery": {
            "trigger": "verified_takeoff_mismatch_while_airborne",
            "action": recovery.model_dump(mode="json"),
        },
    }), encoding="utf-8")


class CommandMissionRecoveryTests(unittest.TestCase):
    def test_takeoff_mismatch_lands_and_halts_before_requested_next_action(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                outcome = kwargs["outcome_json"]
                if item.kind is DroneTaskKind.TAKEOFF:
                    outcome.write_text(json.dumps({
                        "action_success": True,
                        "verdict": "MISMATCH",
                        "reasons": ["takeoff_horizontal_displacement_mismatch"],
                        "post_vehicle_state": {"connected": True, "armed": True},
                        "post_odometry": {"z": 1.2},
                    }), encoding="utf-8")
                    return 4
                outcome.write_text(json.dumps({
                    "action_success": True,
                    "verdict": "VERIFIED",
                    "post_vehicle_state": {"connected": True, "armed": False},
                    "post_odometry": {"z": 0.02},
                }), encoding="utf-8")
                return 0

            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute):
                self.assertEqual(mission.main(), 4)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF, DroneTaskKind.LAND])
            result = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(result["status"], "RECOVERED_HALT")
            self.assertEqual(result["recovery"]["outcome"]["verdict"], "VERIFIED")
            self.assertFalse(any(item["action_id"] == "explore-1" for item in result["results"]))

    def test_unknown_takeoff_outcome_does_not_issue_blind_recovery(self):
        takeoff = proposal(DroneTaskKind.TAKEOFF, "takeoff-0")
        self.assertFalse(mission._needs_takeoff_recovery(takeoff, {
            "verdict": "UNCONFIRMED", "reason": "action_timeout",
            "post_vehicle_state": {"connected": True, "armed": True},
            "post_odometry": {"z": 1.2},
        }))

    def test_terminal_takeoff_abort_with_airborne_evidence_uses_recovery(self):
        takeoff = proposal(DroneTaskKind.TAKEOFF, "takeoff-0")
        self.assertTrue(mission._needs_takeoff_recovery(takeoff, {
            "action_success": False,
            "verdict": "UNCONFIRMED",
            "post_vehicle_state": {"connected": True, "armed": True},
            "post_odometry": {"z": 0.6},
        }))

    def test_takeoff_plan_without_predeclared_recovery_is_rejected(self):
        takeoff = proposal(DroneTaskKind.TAKEOFF, "takeoff-0")
        with self.assertRaisesRegex(SystemExit, "predeclared recovery"):
            mission._recovery_action(
                {"actions": [takeoff.model_dump(mode="json")]}, (takeoff,)
            )


if __name__ == "__main__":
    unittest.main()
