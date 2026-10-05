"""Fail-closed command mission sequencing and recovery tests; no ROS execution."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

import airstack_command_mission as mission
from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal
from rrm.airstack_drone import MapWaypoint
from rrm.airstack_command import RelativeNavigationBinding


class RelativeBindingTests(unittest.TestCase):
    def setUp(self):
        self.proposal = DroneTaskProposal(
            task_id="t", action_id="navigate-0", robot_name="robot_1",
            kind=DroneTaskKind.NAVIGATE, frame_id="map", goal_tolerance_m=.1,
            waypoints=(MapWaypoint(x=1., y=0., z=1.1),),
        )
        self.intent = RelativeNavigationBinding(
            action_id="navigate-0", direction="forward", distance_m=1.,
        )
        self.observation = {
            "connected": True, "armed": True, "airborne": True,
            "missing_state": [], "stale_state": [], "flight_state_consistent": True,
            "frame_id": "map", "child_frame_id": "base_link",
            "position": {"x": 1., "y": 0., "z": 1.1}, "yaw_rad": math.pi / 2,
            "odometry_source_stamp_ns": 1_000_000_000,
            "state_receipt_age_s": {"airborne": .1, "vehicle": .1, "odometry": .1},
        }

    def test_fresh_heading_binding_precedes_satisfaction(self):
        self.assertEqual(mission._replan_decision(self.proposal, self.observation)
                         ["decision"], "SKIP_SATISFIED")
        bound = mission._bind_relative(self.proposal, self.intent, self.observation)
        self.assertAlmostEqual(bound.waypoints[0].x, 1.)
        self.assertAlmostEqual(bound.waypoints[0].y, 1.)
        self.assertEqual(bound.waypoints[0].z, 1.1)
        self.assertEqual(bound.goal_tolerance_m, .1)
        self.assertEqual(mission._replan_decision(bound, self.observation)
                         ["decision"], "CONTINUE")
        self.assertEqual(self.proposal.waypoints[0].y, 0.)

    def test_missing_stale_invalid_or_unframed_state_has_no_fallback(self):
        for change in ({"yaw_rad": None}, {"yaw_rad": float("nan")},
                       {"yaw_rad": True}, {"position": {"x": float("inf"), "y": 0, "z": 1}},
                       {"missing_state": ["vehicle"]}, {"stale_state": ["odometry"]},
                       {"frame_id": "odom"}, {"child_frame_id": None},
                       {"connected": False}, {"armed": False}, {"airborne": False},
                       {"flight_state_consistent": False}, {"flight_state_consistent": None},
                       {"odometry_source_stamp_ns": None}, {"missing_state": None},
                       {"state_receipt_age_s": {"odometry": 3.0}}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                mission._bind_relative(self.proposal, self.intent,
                                       {**self.observation, **change})

    def test_reviewed_binding_rejects_unknown_duplicate_and_loose_action(self):
        raw = self.intent.model_dump(mode="json")
        for bindings, proposals in (
                ([{**raw, "action_id": "unknown"}], (self.proposal,)),
                ([raw, raw], (self.proposal,)),
                ([raw], (self.proposal.model_copy(update={"goal_tolerance_m": .5}),))):
            with self.subTest(bindings=bindings), self.assertRaises(ValueError):
                mission._relative_bindings(
                    {"grounding": {"relative_navigation_bindings": bindings}}, proposals)

    def test_first_airborne_relative_action_observes_and_dispatches_bound_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = root / "plan.json"
            evidence = root / "evidence"
            plan.write_text(json.dumps({
                "schema_version": "rrm-airstack-command-plan/v1",
                "actions": [self.proposal.model_dump(mode="json")],
                "grounding": {"relative_navigation_bindings": [
                    self.intent.model_dump(mode="json")]},
            }))
            calls = []
            def dispatch(action, outcome):
                calls.append(action)
                outcome.write_text(json.dumps({"verdict": "VERIFIED"}))
                return 0
            with patch.object(sys, "argv", ["mission", "--plan-json", str(plan),
                                            "--evidence-dir", str(evidence), "--execute"]), \
                    patch.object(mission, "_observe_state", return_value=self.observation), \
                    patch.object(mission, "_dispatch", side_effect=dispatch):
                self.assertEqual(mission.main(), 0)
            self.assertEqual(len(calls), 1)
            self.assertAlmostEqual(calls[0].waypoints[0].y, 1.)
            record = json.loads((evidence / "replan-0000-navigate-0.json").read_text())
            self.assertEqual(record["relative_binding"]["intent"]["distance_m"], 1.)
            self.assertEqual(record["decision"], "CONTINUE")


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


def write_plan(path: Path, *, replan: bool = False) -> None:
    takeoff = proposal(DroneTaskKind.TAKEOFF, "takeoff-0")
    explore = proposal(DroneTaskKind.EXPLORE, "explore-1")
    recovery = proposal(DroneTaskKind.LAND, "takeoff-0-recovery-land")
    value = {
        "schema_version": "rrm-airstack-command-plan/v1",
        "actions": [takeoff.model_dump(mode="json"), explore.model_dump(mode="json")],
        "recovery": {
            "trigger": "verified_takeoff_mismatch_while_airborne",
            "action": recovery.model_dump(mode="json"),
        },
    }
    if replan:
        value["replan_policy"] = {
            "mode": "observe_between_actions", "blind_retry": False,
        }
    path.write_text(json.dumps(value), encoding="utf-8")


def state(*, airborne: bool, armed: bool, position=None) -> dict:
    return {
        "schema_version": "airstack-task-discovery/v1",
        "missing_state": [], "stale_state": [], "connected": True,
        "airborne": airborne, "armed": armed,
        "position": position or {"x": 0.0, "y": 0.0, "z": 1.5},
    }


class CommandMissionRecoveryTests(unittest.TestCase):
    def test_landing_can_reconcile_fresh_armed_low_altitude_contradiction(self):
        land = proposal(DroneTaskKind.LAND, "land-reconcile")
        fresh = state(airborne=True, armed=True, position={"x": 0, "y": 0, "z": 0.02})
        fresh["flight_state_consistent"] = False
        fresh["stale_state"] = ["vdb_map"]
        self.assertEqual(mission._replan_decision(land, fresh)["decision"], "CONTINUE")
        explore = proposal(DroneTaskKind.EXPLORE, "ordinary")
        self.assertEqual(mission._replan_decision(explore, fresh)["decision"], "HALT")
        for key, value in (("connected", False), ("armed", None), ("airborne", None),
                           ("stale_state", ["odometry"]), ("missing_state", ["vehicle"])):
            bad = dict(fresh, **{key: value})
            self.assertEqual(mission._replan_decision(land, bad)["decision"], "HALT", key)

    def test_exploration_without_progress_lands_from_fresh_airborne_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                mismatch = item.kind is DroneTaskKind.EXPLORE
                kwargs["outcome_json"].write_text(json.dumps({
                    "action_success": True,
                    "verdict": "MISMATCH" if mismatch else "VERIFIED",
                    "reasons": ["exploration_progress_mismatch"] if mismatch else [],
                }), encoding="utf-8")
                return 4 if mismatch else 0

            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute), patch.object(
                    mission, "_observe_state", return_value=state(airborne=True, armed=True)):
                self.assertEqual(mission.main(), 4)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF, DroneTaskKind.EXPLORE,
                                     DroneTaskKind.LAND])
            outcome = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(outcome["status"], "RECOVERED_HALT")
            self.assertEqual(outcome["replans"][-1]["reason"], "action_effect_not_verified")

    def test_delayed_airborne_evidence_after_failed_takeoff_triggers_recovery_land(self):
        """Regression: low terminal Z can precede delayed uncontrolled ascent."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                if item.kind is DroneTaskKind.TAKEOFF:
                    kwargs["outcome_json"].write_text(json.dumps({
                        "action_success": False,
                        "verdict": "UNCONFIRMED",
                        "reasons": ["takeoff_horizontal_displacement_mismatch"],
                        "post_vehicle_state": {"connected": True, "armed": True},
                        "post_odometry": {"z": -0.36},
                    }), encoding="utf-8")
                    return 4
                kwargs["outcome_json"].write_text(json.dumps({
                    "action_success": True,
                    "verdict": "VERIFIED",
                    "post_vehicle_state": {"connected": True, "armed": False},
                    "post_odometry": {"z": 0.02},
                }), encoding="utf-8")
                return 0

            delayed_airborne = state(
                airborne=True, armed=True,
                position={"x": 0.7, "y": -0.8, "z": 0.65},
            )
            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute), patch.object(
                    mission, "_observe_state",
                    side_effect=[delayed_airborne, delayed_airborne]), patch.object(
                    mission.time, "sleep", return_value=None):
                self.assertEqual(mission.main(), 4)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF, DroneTaskKind.LAND])
            monitor = json.loads(
                (evidence / "takeoff-recovery-monitor.json").read_text(encoding="utf-8")
            )
            self.assertEqual(monitor["decision"], "RECOVER")
            self.assertEqual(monitor["sample_count"], 2)
            outcome = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(outcome["status"], "RECOVERED_HALT")
            self.assertEqual(outcome["recovery_monitor"]["decision"], "RECOVER")
            self.assertFalse(any(item["action_id"] == "explore-1"
                                 for item in outcome["results"]))

    def test_failed_takeoff_that_settles_grounded_does_not_dispatch_land(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                kwargs["outcome_json"].write_text(json.dumps({
                    "action_success": False, "verdict": "UNCONFIRMED",
                    "post_vehicle_state": {"connected": True, "armed": True},
                    "post_odometry": {"z": -0.1},
                }), encoding="utf-8")
                return 4

            grounded = state(
                airborne=False, armed=False,
                position={"x": 0.0, "y": 0.0, "z": 0.02},
            )
            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute), patch.object(
                    mission, "_observe_state", side_effect=[grounded, grounded]), patch.object(
                    mission.time, "sleep", return_value=None):
                self.assertEqual(mission.main(), 4)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF])
            outcome = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(outcome["status"], "HALTED")
            self.assertEqual(outcome["recovery_monitor"]["decision"], "SAFE_GROUNDED")
            self.assertIsNone(outcome["recovery"])

    def test_unresolved_takeoff_monitor_timeout_is_explicit_recovery_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory)
            unknown = state(
                airborne=False, armed=True,
                position={"x": 0.0, "y": 0.0, "z": -0.2},
            )
            with patch.object(mission, "_observe_state", return_value=unknown), patch.object(
                    mission.time, "monotonic", side_effect=[0.0, 0.0, 2.0, 2.0]), \
                    patch.object(mission.time, "sleep", return_value=None):
                result = mission._monitor_failed_takeoff(
                    "robot_1", evidence, timeout_s=1.0, poll_interval_s=0.0
                )
            self.assertEqual(result["decision"], "TIMEOUT")
            self.assertEqual(result["sample_count"], 1)
            self.assertFalse(result["execution_dispatch"])

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

    def test_verified_action_observes_and_reconciles_before_next_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path, replan=True)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                kwargs["outcome_json"].write_text(json.dumps({
                    "action_success": True, "verdict": "VERIFIED", "reasons": [],
                }), encoding="utf-8")
                return 0

            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute), patch.object(
                    mission, "_observe_state", return_value=state(airborne=True, armed=True)):
                self.assertEqual(mission.main(), 0)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF, DroneTaskKind.EXPLORE])
            decision = json.loads(
                (evidence / "replan-0001-explore-1.json").read_text(encoding="utf-8")
            )
            self.assertEqual(decision["decision"], "CONTINUE")
            outcome = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(outcome["status"], "VERIFIED")
            self.assertEqual(len(outcome["replans"]), 1)

    def test_incompatible_fresh_state_halts_without_dispatch_or_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            evidence = root / "evidence"
            write_plan(plan_path, replan=True)
            calls = []

            def execute(item, _timeout, **kwargs):
                calls.append(item.kind)
                kwargs["outcome_json"].write_text(json.dumps({
                    "action_success": True, "verdict": "VERIFIED", "reasons": [],
                }), encoding="utf-8")
                return 0

            argv = ["airstack_command_mission.py", "--plan-json", str(plan_path),
                    "--evidence-dir", str(evidence), "--execute"]
            with patch.object(sys, "argv", argv), patch.object(
                    mission.dispatcher, "_execute", side_effect=execute), patch.object(
                    mission, "_observe_state", return_value=state(airborne=False, armed=False)):
                self.assertEqual(mission.main(), 4)

            self.assertEqual(calls, [DroneTaskKind.TAKEOFF])
            outcome = json.loads((evidence / "mission-outcome.json").read_text())
            self.assertEqual(outcome["status"], "HALTED")
            self.assertEqual(outcome["replans"][0]["reason"],
                             "airborne_action_without_flight_state")

    def test_replan_can_skip_only_an_independently_satisfied_action(self):
        land = proposal(DroneTaskKind.LAND, "land-1")
        decision = mission._replan_decision(
            land, state(airborne=False, armed=False, position={"x": 0, "y": 0, "z": 0.02})
        )
        self.assertEqual(decision["decision"], "SKIP_SATISFIED")
        unavailable = state(airborne=True, armed=True)
        unavailable["stale_state"] = ["odometry"]
        self.assertEqual(mission._replan_decision(land, unavailable)["decision"], "HALT")
        explore = proposal(DroneTaskKind.EXPLORE, "explore-2")
        contradictory = state(
            airborne=True, armed=True, position={"x": 0, "y": 0, "z": 0.02}
        )
        decision = mission._replan_decision(explore, contradictory)
        self.assertEqual((decision["decision"], decision["reason"]),
                         ("HALT", "contradictory_airborne_altitude"))
        inconsistent = state(
            airborne=False, armed=False, position={"x": 0, "y": 0, "z": 1.2}
        )
        inconsistent["flight_state_consistent"] = False
        decision = mission._replan_decision(land, inconsistent)
        self.assertEqual((decision["decision"], decision["reason"]),
                         ("HALT", "contradictory_flight_state"))

    def test_replan_recovery_requires_fresh_meaningful_airborne_evidence(self):
        airborne = {
            "decision": "HALT",
            "observation": state(
                airborne=True, armed=True,
                position={"x": 0.0, "y": 0.0, "z": 1.2},
            ),
        }
        self.assertTrue(mission._needs_replan_recovery(airborne))
        low = json.loads(json.dumps(airborne))
        low["observation"]["position"]["z"] = 0.1
        self.assertFalse(mission._needs_replan_recovery(low))
        stale = json.loads(json.dumps(airborne))
        stale["observation"]["stale_state"] = ["odometry"]
        self.assertFalse(mission._needs_replan_recovery(stale))
        contradictory = json.loads(json.dumps(airborne))
        contradictory["observation"]["flight_state_consistent"] = False
        self.assertFalse(mission._needs_replan_recovery(contradictory))
        unknown = {"decision": "HALT", "reason": "observation_failed"}
        self.assertFalse(mission._needs_replan_recovery(unknown))


if __name__ == "__main__":
    unittest.main()
