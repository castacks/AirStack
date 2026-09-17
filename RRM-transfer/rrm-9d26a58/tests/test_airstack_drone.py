"""RRM drone proposals map only to the public AirStack task action boundary."""

import unittest

from pydantic import ValidationError

from rrm.airstack_drone import (
    DroneOutcomeVerdict,
    DroneTaskKind,
    DroneTaskProposal,
    MapWaypoint,
    OdometryEvidence,
    VehicleStateEvidence,
    verify_drone_outcome,
)


def proposal(kind, **changes):
    values = dict(task_id="task-1", action_id="action-1", robot_name="robot_1", kind=kind)
    values.update(changes)
    return DroneTaskProposal(**values)


class DroneProposalTests(unittest.TestCase):
    def test_takeoff_maps_to_existing_task_action(self):
        result = proposal(DroneTaskKind.TAKEOFF, target_altitude_m=2.0, velocity_m_s=1.0).preview()
        self.assertEqual(result["action_name"], "/robot_1/tasks/takeoff")
        self.assertEqual(result["goal"], {"target_altitude_m": 2.0, "velocity_m_s": 1.0})
        self.assertFalse(result["execution_requested"])

    def test_land_and_navigate_preview_exact_goal_shapes(self):
        land = proposal(DroneTaskKind.LAND, velocity_m_s=0.3).preview()
        self.assertEqual(land["action_name"], "/robot_1/tasks/land")
        navigate = proposal(DroneTaskKind.NAVIGATE, frame_id="map",
                            waypoints=(MapWaypoint(x=1.0, y=2.0, z=3.0),),
                            goal_tolerance_m=0.5).preview()
        self.assertEqual(navigate["action_name"], "/robot_1/tasks/navigate")
        self.assertEqual(navigate["goal"]["global_plan"]["header"]["frame_id"], "map")

    def test_invalid_parameters_and_frames_are_rejected(self):
        with self.assertRaises(ValidationError):
            proposal(DroneTaskKind.TAKEOFF, target_altitude_m=0.0, velocity_m_s=1.0)
        with self.assertRaises(ValidationError):
            proposal(DroneTaskKind.LAND, velocity_m_s=-0.1)
        with self.assertRaises(ValidationError):
            proposal(DroneTaskKind.NAVIGATE, frame_id="earth",
                     waypoints=(MapWaypoint(x=0, y=0, z=1),), goal_tolerance_m=1.0)

    def test_runner_is_explicitly_gated_and_has_no_direct_control_surface(self):
        from pathlib import Path
        runner = (Path(__file__).parents[1] / "scripts" / "airstack_drone_dispatch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"--execute"', runner)
        self.assertIn('"--verify-observation"', runner)
        self.assertIn("ActionClient", runner)
        self.assertIn("create_subscription(", runner)
        for prohibited in ("create_publisher(", "create_client(",
                           "trajectory_override", "RobotCommand"):
            self.assertNotIn(prohibited, runner)

    def test_takeoff_outcome_requires_fresh_independent_target_observation(self):
        takeoff = proposal(DroneTaskKind.TAKEOFF, target_altitude_m=2.0, velocity_m_s=1.0)
        pre = OdometryEvidence(received_monotonic_s=10.0, source_stamp_ns=1,
                               frame_id="map", child_frame_id="base_link",
                               x=0.0, y=0.0, z=0.02)
        post = OdometryEvidence(received_monotonic_s=11.0, source_stamp_ns=2,
                                frame_id="map", child_frame_id="base_link",
                                x=0.0, y=0.0, z=1.8)
        result = verify_drone_outcome(
            takeoff, action_success=True, action_message="takeoff complete",
            pre_odometry=pre, post_odometry=post,
            post_vehicle_state=VehicleStateEvidence(
                received_monotonic_s=11.1, connected=True, armed=True),
            dispatch_monotonic_s=10.1, now_monotonic_s=11.2,
        )
        self.assertEqual(result.verdict, DroneOutcomeVerdict.VERIFIED)

        mismatch = verify_drone_outcome(
            takeoff, action_success=True, action_message="takeoff complete",
            pre_odometry=pre,
            post_odometry=post.model_copy(update={"z": 1.69}),
            post_vehicle_state=VehicleStateEvidence(
                received_monotonic_s=11.1, connected=True, armed=True),
            dispatch_monotonic_s=10.1, now_monotonic_s=11.2,
        )
        self.assertEqual(mismatch.verdict, DroneOutcomeVerdict.MISMATCH)
        self.assertIn("takeoff_altitude_mismatch", mismatch.reasons)

    def test_landing_outcome_fails_closed_without_fresh_disarmed_evidence(self):
        land = proposal(DroneTaskKind.LAND, velocity_m_s=1.0)
        pre = OdometryEvidence(received_monotonic_s=10.0, source_stamp_ns=1,
                               frame_id="map", child_frame_id="base_link",
                               x=0.0, y=0.0, z=2.0)
        post = OdometryEvidence(received_monotonic_s=11.0, source_stamp_ns=2,
                                frame_id="map", child_frame_id="base_link",
                                x=0.0, y=0.0, z=0.02)
        verified = verify_drone_outcome(
            land, action_success=True, action_message="landing complete",
            pre_odometry=pre, post_odometry=post,
            post_vehicle_state=VehicleStateEvidence(
                received_monotonic_s=11.1, connected=True, armed=False),
            dispatch_monotonic_s=10.1, now_monotonic_s=11.2,
        )
        self.assertEqual(verified.verdict, DroneOutcomeVerdict.VERIFIED)

        unconfirmed = verify_drone_outcome(
            land, action_success=True, action_message="landing complete",
            pre_odometry=pre, post_odometry=post,
            post_vehicle_state=VehicleStateEvidence(
                received_monotonic_s=11.1, connected=True, armed=True),
            dispatch_monotonic_s=10.1, now_monotonic_s=11.2,
        )
        self.assertEqual(unconfirmed.verdict, DroneOutcomeVerdict.UNCONFIRMED)
        self.assertIn("post_vehicle_still_armed", unconfirmed.reasons)

    def test_stale_or_unsuccessful_result_never_verifies_an_effect(self):
        takeoff = proposal(DroneTaskKind.TAKEOFF, target_altitude_m=2.0, velocity_m_s=1.0)
        stale = OdometryEvidence(received_monotonic_s=1.0, source_stamp_ns=1,
                                 frame_id="map", child_frame_id="base_link",
                                 x=0.0, y=0.0, z=2.0)
        result = verify_drone_outcome(
            takeoff, action_success=False, action_message="canceled",
            pre_odometry=stale, post_odometry=stale, post_vehicle_state=None,
            dispatch_monotonic_s=2.0, now_monotonic_s=10.0,
        )
        self.assertEqual(result.verdict, DroneOutcomeVerdict.UNCONFIRMED)
        self.assertIn("task_result_unsuccessful", result.reasons)
        self.assertIn("post_odometry_stale", result.reasons)


if __name__ == "__main__":
    unittest.main()
