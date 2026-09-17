#!/usr/bin/env python3
"""Render or explicitly dispatch one validated RRM proposal via AirStack task actions.

Without ``--execute`` this prints the exact task action/goal and does not import ROS.
With it, the process creates one ActionClient for the selected AirStack task endpoint.
Optional verification adds read-only odometry and MAVROS-state subscriptions. It never
issues a PX4/MAVROS command, creates a service client/publisher, or uses a trajectory
topic directly.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from rrm.airstack_drone import (
    DroneTaskKind,
    DroneTaskProposal,
    OdometryEvidence,
    VehicleStateEvidence,
    verify_drone_outcome,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proposal-json", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="send exactly one task action goal")
    parser.add_argument("--server-timeout-s", type=float, default=5.0)
    parser.add_argument("--verify-observation", action="store_true",
                        help="require fresh read-only odometry/state evidence for the result")
    parser.add_argument("--outcome-json", type=Path,
                        help="write the verified/unconfirmed outcome record (requires --verify-observation)")
    parser.add_argument("--observation-timeout-s", type=float, default=5.0)
    parser.add_argument("--max-observation-age-s", type=float, default=1.0)
    parser.add_argument("--takeoff-acceptance-distance-m", type=float, default=0.3)
    parser.add_argument("--landing-max-altitude-m", type=float, default=0.3)
    return parser


def _load(path: Path) -> DroneTaskProposal:
    return DroneTaskProposal.model_validate_json(path.read_text(encoding="utf-8"))


def _execute(proposal: DroneTaskProposal, timeout_s: float, *, verify_observation: bool,
             outcome_json: Path | None, observation_timeout_s: float,
             max_observation_age_s: float, takeoff_acceptance_distance_m: float,
             landing_max_altitude_m: float) -> int:
    """Use only the existing task-action server selected by the proposal."""
    if timeout_s <= 0:
        raise ValueError("server timeout must be positive")
    verification_bounds = (observation_timeout_s, max_observation_age_s,
                           takeoff_acceptance_distance_m, landing_max_altitude_m)
    if (not all(math.isfinite(value) for value in verification_bounds)
            or observation_timeout_s <= 0 or max_observation_age_s <= 0
            or takeoff_acceptance_distance_m <= 0 or landing_max_altitude_m < 0):
        raise ValueError("outcome verification bounds are invalid")
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path
    from task_msgs.action import LandTask, NavigateTask, TakeoffTask

    action_type = {
        DroneTaskKind.TAKEOFF: TakeoffTask,
        DroneTaskKind.NAVIGATE: NavigateTask,
        DroneTaskKind.LAND: LandTask,
    }[proposal.kind]
    rclpy.init()
    node = rclpy.create_node("rrm_drone_task_adapter")
    client = ActionClient(node, action_type, proposal.action_name)
    latest_odometry: OdometryEvidence | None = None
    latest_vehicle_state: VehicleStateEvidence | None = None

    if verify_observation:
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        def on_odometry(message: Odometry) -> None:
            nonlocal latest_odometry
            stamp = message.header.stamp
            try:
                latest_odometry = OdometryEvidence(
                    received_monotonic_s=time.monotonic(),
                    source_stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
                    frame_id=message.header.frame_id,
                    child_frame_id=message.child_frame_id,
                    x=message.pose.pose.position.x,
                    y=message.pose.pose.position.y,
                    z=message.pose.pose.position.z,
                )
            except ValueError as exc:
                node.get_logger().warning(f"ignored incompatible outcome odometry: {exc}")

        def on_vehicle_state(message: State) -> None:
            nonlocal latest_vehicle_state
            latest_vehicle_state = VehicleStateEvidence(
                received_monotonic_s=time.monotonic(),
                connected=message.connected,
                armed=message.armed,
            )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        prefix = f"/{proposal.robot_name}"
        node.create_subscription(Odometry, f"{prefix}/odometry_conversion/odometry", on_odometry, qos)
        node.create_subscription(State, f"{prefix}/interface/mavros/state", on_vehicle_state, qos)
    try:
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"task server unavailable: {proposal.action_name}")
        if verify_observation:
            deadline = time.monotonic() + observation_timeout_s
            while rclpy.ok() and latest_odometry is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.25, deadline - time.monotonic()))
            if (latest_odometry is None
                    or time.monotonic() - latest_odometry.received_monotonic_s
                    > max_observation_age_s):
                raise RuntimeError("fresh map -> base_link odometry unavailable before dispatch")
        pre_odometry = latest_odometry
        dispatch_monotonic_s = time.monotonic()
        if proposal.kind is DroneTaskKind.TAKEOFF:
            goal = TakeoffTask.Goal()
            goal.target_altitude_m = proposal.target_altitude_m
            goal.velocity_m_s = proposal.velocity_m_s
        elif proposal.kind is DroneTaskKind.LAND:
            goal = LandTask.Goal()
            goal.velocity_m_s = proposal.velocity_m_s
        else:
            goal = NavigateTask.Goal()
            goal.goal_tolerance_m = proposal.goal_tolerance_m
            goal.global_plan = Path()
            goal.global_plan.header.frame_id = proposal.frame_id
            for waypoint in proposal.waypoints:
                pose = PoseStamped()
                pose.header.frame_id = proposal.frame_id
                pose.pose.position = Point(x=waypoint.x, y=waypoint.y, z=waypoint.z)
                pose.pose.orientation.w = 1.0
                goal.global_plan.poses.append(pose)

        def feedback_callback(message) -> None:
            print(json.dumps({"event": "feedback", "status": message.feedback.status}), flush=True)

        response = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(node, response)
        handle = response.result()
        if handle is None or not handle.accepted:
            print(json.dumps({"event": "goal_rejected", "action_id": proposal.action_id}))
            if verify_observation:
                verification = verify_drone_outcome(
                    proposal,
                    action_success=False,
                    action_message="goal rejected",
                    pre_odometry=pre_odometry,
                    post_odometry=latest_odometry,
                    post_vehicle_state=latest_vehicle_state,
                    dispatch_monotonic_s=dispatch_monotonic_s,
                    now_monotonic_s=time.monotonic(),
                    max_observation_age_s=max_observation_age_s,
                    takeoff_acceptance_distance_m=takeoff_acceptance_distance_m,
                    landing_max_altitude_m=landing_max_altitude_m,
                )
                record = verification.model_dump(mode="json")
                print(json.dumps({"event": "outcome_verification", **record}, sort_keys=True))
                if outcome_json is not None:
                    outcome_json.parent.mkdir(parents=True, exist_ok=True)
                    outcome_json.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                                            encoding="utf-8")
                return 4
            return 2
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        wrapped = result.result()
        outcome = wrapped.result
        print(json.dumps({"event": "result", "action_id": proposal.action_id,
                          "success": outcome.success, "message": outcome.message}))
        if not verify_observation:
            return 0 if outcome.success else 3

        completion_monotonic_s = time.monotonic()
        deadline = completion_monotonic_s + observation_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            post_odom_ready = (
                latest_odometry is not None
                and latest_odometry.received_monotonic_s >= completion_monotonic_s
            )
            post_state_ready = (
                proposal.kind is DroneTaskKind.NAVIGATE
                or (latest_vehicle_state is not None
                    and latest_vehicle_state.received_monotonic_s >= completion_monotonic_s)
            )
            if post_odom_ready and post_state_ready:
                break
            rclpy.spin_once(node, timeout_sec=min(0.25, deadline - time.monotonic()))
        verification = verify_drone_outcome(
            proposal,
            action_success=outcome.success,
            action_message=outcome.message,
            pre_odometry=pre_odometry,
            post_odometry=latest_odometry,
            post_vehicle_state=latest_vehicle_state,
            dispatch_monotonic_s=dispatch_monotonic_s,
            now_monotonic_s=time.monotonic(),
            max_observation_age_s=max_observation_age_s,
            takeoff_acceptance_distance_m=takeoff_acceptance_distance_m,
            landing_max_altitude_m=landing_max_altitude_m,
        )
        record = verification.model_dump(mode="json")
        print(json.dumps({"event": "outcome_verification", **record}, sort_keys=True))
        if outcome_json is not None:
            outcome_json.parent.mkdir(parents=True, exist_ok=True)
            outcome_json.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                                    encoding="utf-8")
        return 0 if verification.verdict.value == "VERIFIED" else 4
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = _parser().parse_args()
    if args.outcome_json is not None and not args.verify_observation:
        raise SystemExit("--outcome-json requires --verify-observation")
    proposal = _load(args.proposal_json)
    preview = proposal.preview()
    if not args.execute:
        print(json.dumps(preview, indent=2, sort_keys=True))
        return 0
    preview["execution_requested"] = True
    print(json.dumps(preview, sort_keys=True), flush=True)
    return _execute(
        proposal,
        args.server_timeout_s,
        verify_observation=args.verify_observation,
        outcome_json=args.outcome_json,
        observation_timeout_s=args.observation_timeout_s,
        max_observation_age_s=args.max_observation_age_s,
        takeoff_acceptance_distance_m=args.takeoff_acceptance_distance_m,
        landing_max_altitude_m=args.landing_max_altitude_m,
    )


if __name__ == "__main__":
    sys.exit(main())
