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
from datetime import datetime, timezone
import hashlib
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
    parser.add_argument("--action-timeout-s", type=float, default=90.0,
                        help="deadline for completion; requests cancel on expiry")
    parser.add_argument("--verify-observation", action="store_true",
                        help="require fresh read-only odometry/state evidence for the result")
    parser.add_argument("--outcome-json", type=Path,
                        help="write the verified/unconfirmed outcome record (requires --verify-observation)")
    parser.add_argument("--observation-timeout-s", type=float, default=5.0)
    parser.add_argument("--max-observation-age-s", type=float, default=1.0)
    parser.add_argument("--takeoff-acceptance-distance-m", type=float, default=0.3)
    parser.add_argument("--takeoff-max-horizontal-displacement-m", type=float, default=0.3,
                        help="maximum XY motion allowed while a takeoff completes")
    parser.add_argument("--landing-max-altitude-m", type=float, default=0.3)
    return parser


def _load(path: Path) -> DroneTaskProposal:
    return DroneTaskProposal.model_validate_json(path.read_text(encoding="utf-8"))


def _execute(proposal: DroneTaskProposal, timeout_s: float, *, verify_observation: bool,
             outcome_json: Path | None, observation_timeout_s: float,
             max_observation_age_s: float, takeoff_acceptance_distance_m: float,
             takeoff_max_horizontal_displacement_m: float,
             landing_max_altitude_m: float, action_timeout_s: float = 90.0) -> int:
    """Use only the existing task-action server selected by the proposal."""
    if not all(math.isfinite(t) and t > 0 for t in (timeout_s, action_timeout_s)):
        raise ValueError("server and action timeouts must be finite and positive")
    verification_bounds = (observation_timeout_s, max_observation_age_s,
                           takeoff_acceptance_distance_m,
                           takeoff_max_horizontal_displacement_m,
                           landing_max_altitude_m)
    if (not all(math.isfinite(value) for value in verification_bounds)
            or observation_timeout_s <= 0 or max_observation_age_s <= 0
            or takeoff_acceptance_distance_m <= 0
            or takeoff_max_horizontal_displacement_m <= 0
            or landing_max_altitude_m < 0):
        raise ValueError("outcome verification bounds are invalid")
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Point, Point32, PoseStamped
    from nav_msgs.msg import Path
    from task_msgs.action import ExplorationTask, LandTask, NavigateTask, TakeoffTask

    action_type = {
        DroneTaskKind.TAKEOFF: TakeoffTask,
        DroneTaskKind.NAVIGATE: NavigateTask,
        DroneTaskKind.LAND: LandTask,
        DroneTaskKind.EXPLORE: ExplorationTask,
    }[proposal.kind]
    # Keep ROS from consuming SIGINT and shutting its context down before this
    # adapter can ask the active AirStack action server to cancel the goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("rrm_drone_task_adapter")
    client = ActionClient(node, action_type, proposal.action_name)
    latest_odometry: OdometryEvidence | None = None
    latest_vehicle_state: VehicleStateEvidence | None = None
    latest_linear_speed_m_s: float | None = None
    latest_global_plan_sha256: str | None = None
    dispatch_monotonic_s: float | None = None

    if verify_observation:
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        def on_odometry(message: Odometry) -> None:
            nonlocal latest_odometry, latest_linear_speed_m_s
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
                velocity = message.twist.twist.linear
                latest_linear_speed_m_s = math.sqrt(
                    velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
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

        def on_global_plan(message: Path) -> None:
            nonlocal latest_global_plan_sha256
            if dispatch_monotonic_s is None:
                return
            points = [
                [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                for pose in message.poses
            ]
            digest = hashlib.sha256(json.dumps(points, separators=(",", ":")).encode()).hexdigest()
            if digest == latest_global_plan_sha256:
                return
            latest_global_plan_sha256 = digest
            path_length_m = sum(
                math.dist(points[index - 1], points[index])
                for index in range(1, len(points))
            )
            stamp = message.header.stamp
            print(json.dumps({
                "event": "global_plan_update",
                "action_id": proposal.action_id,
                "source_stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
                "frame_id": message.header.frame_id,
                "waypoint_count": len(points),
                "path_length_m": round(path_length_m, 3),
                "start": points[0] if points else None,
                "goal": points[-1] if points else None,
                "plan_sha256": digest,
            }, sort_keys=True), flush=True)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        prefix = f"/{proposal.robot_name}"
        node.create_subscription(Odometry, f"{prefix}/odometry_conversion/odometry", on_odometry, qos)
        node.create_subscription(State, f"{prefix}/interface/mavros/state", on_vehicle_state, qos)
        node.create_subscription(Path, f"{prefix}/global_plan", on_global_plan, qos)
    try:
        if not client.wait_for_server(timeout_sec=timeout_s):
            record = {
                "event": "task_server_unavailable",
                "task_id": proposal.task_id,
                "action_id": proposal.action_id,
                "kind": proposal.kind.value,
                "action_name": proposal.action_name,
                "goal_sent": False,
                "physical_outcome": "NOT_DISPATCHED",
                "verdict": "UNCONFIRMED",
                "reasons": ["task_server_unavailable"],
            }
            print(json.dumps(record, sort_keys=True), flush=True)
            if outcome_json is not None:
                outcome_json.parent.mkdir(parents=True, exist_ok=True)
                outcome_json.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                                        encoding="utf-8")
            return 6
        if verify_observation:
            deadline = time.monotonic() + observation_timeout_s
            while (rclpy.ok() and (latest_odometry is None or latest_vehicle_state is None)
                   and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=min(0.25, deadline - time.monotonic()))
            if (latest_odometry is None
                    or time.monotonic() - latest_odometry.received_monotonic_s
                    > max_observation_age_s):
                raise RuntimeError("fresh map -> base_link odometry unavailable before dispatch")
            if (latest_vehicle_state is None or not latest_vehicle_state.connected
                    or time.monotonic() - latest_vehicle_state.received_monotonic_s
                    > max_observation_age_s):
                raise RuntimeError("fresh connected vehicle state unavailable before dispatch")
            if (proposal.kind in {DroneTaskKind.NAVIGATE, DroneTaskKind.EXPLORE}
                    and not latest_vehicle_state.armed):
                raise RuntimeError("airborne task requires an armed vehicle; perform takeoff first")
            if (proposal.kind in {DroneTaskKind.NAVIGATE, DroneTaskKind.EXPLORE}
                    and latest_odometry.z <= 0.3):
                raise RuntimeError(
                    "airborne state contradicts map altitude; land/reconcile before motion"
                )
        pre_odometry = latest_odometry
        dispatch_monotonic_s = time.monotonic()
        print(json.dumps({
            "event": "preflight_observation",
            "action_id": proposal.action_id,
            "kind": proposal.kind.value,
            "armed": latest_vehicle_state.armed if latest_vehicle_state else None,
            "connected": latest_vehicle_state.connected if latest_vehicle_state else None,
            "position": ({
                "x": pre_odometry.x, "y": pre_odometry.y, "z": pre_odometry.z,
            } if pre_odometry else None),
            "linear_speed_m_s": latest_linear_speed_m_s,
            "source_stamp_ns": pre_odometry.source_stamp_ns if pre_odometry else None,
        }, sort_keys=True), flush=True)
        if proposal.kind is DroneTaskKind.TAKEOFF:
            goal = TakeoffTask.Goal()
            goal.target_altitude_m = proposal.target_altitude_m
            goal.velocity_m_s = proposal.velocity_m_s
        elif proposal.kind is DroneTaskKind.LAND:
            goal = LandTask.Goal()
            goal.velocity_m_s = proposal.velocity_m_s
        elif proposal.kind is DroneTaskKind.EXPLORE:
            goal = ExplorationTask.Goal()
            for point in proposal.search_bounds:
                goal.search_bounds.points.append(Point32(x=point.x, y=point.y, z=0.0))
            goal.min_altitude_agl = proposal.min_altitude_agl_m
            goal.max_altitude_agl = proposal.max_altitude_agl_m
            goal.min_flight_speed = proposal.min_flight_speed_m_s
            goal.max_flight_speed = proposal.max_flight_speed_m_s
            goal.time_limit_sec = proposal.time_limit_s
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
            feedback = message.feedback
            record = {"event": "feedback", "action_id": proposal.action_id,
                      "status": getattr(feedback, "status", None)}
            for field in ("progress", "distance_to_goal", "best_confidence",
                          "current_altitude_m", "target_altitude_m"):
                if hasattr(feedback, field):
                    record[field] = getattr(feedback, field)
            position = getattr(feedback, "current_position", None)
            if position is not None:
                record["current_position"] = {
                    "x": position.x, "y": position.y, "z": position.z,
                }
            record["elapsed_s"] = round(time.monotonic() - dispatch_monotonic_s, 3)
            record["linear_speed_m_s"] = latest_linear_speed_m_s
            if latest_odometry is not None:
                record["odometry"] = {
                    "x": latest_odometry.x,
                    "y": latest_odometry.y,
                    "z": latest_odometry.z,
                    "source_stamp_ns": latest_odometry.source_stamp_ns,
                }
                if pre_odometry is not None:
                    record["horizontal_displacement_m"] = round(math.hypot(
                        latest_odometry.x - pre_odometry.x,
                        latest_odometry.y - pre_odometry.y,
                    ), 4)
                    record["vertical_displacement_m"] = round(
                        latest_odometry.z - pre_odometry.z, 4
                    )
            print(json.dumps(record, sort_keys=True), flush=True)

        response = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(node, response, timeout_sec=timeout_s)
        if not response.done():
            # Acceptance may have happened remotely. Never retry blindly.
            raise RuntimeError("goal acknowledgement timed out; remote goal state UNKNOWN; inspect before retry")
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
                    takeoff_max_horizontal_displacement_m=(
                        takeoff_max_horizontal_displacement_m),
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
        try:
            rclpy.spin_until_future_complete(node, result, timeout_sec=action_timeout_s)
        except KeyboardInterrupt:
            stop_received_at = datetime.now(timezone.utc).isoformat()
            stop_received_monotonic_s = time.monotonic()
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=timeout_s)
            cancel_reply = cancel.result() if cancel.done() else None
            cancel_ack_monotonic_s = time.monotonic()
            cancel_acknowledged = bool(cancel_reply and cancel_reply.goals_canceling)
            motion_stopped = False
            motion_stopped_monotonic_s = None
            stop_speed_threshold_m_s = 0.10
            consecutive_low_speed_samples = 0
            last_sample_received_s = None
            if verify_observation and cancel_acknowledged:
                deadline = time.monotonic() + observation_timeout_s
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
                    if (latest_odometry is None or latest_linear_speed_m_s is None
                            or latest_odometry.received_monotonic_s == last_sample_received_s
                            or latest_odometry.received_monotonic_s < stop_received_monotonic_s):
                        continue
                    last_sample_received_s = latest_odometry.received_monotonic_s
                    if latest_linear_speed_m_s <= stop_speed_threshold_m_s:
                        consecutive_low_speed_samples += 1
                        if consecutive_low_speed_samples >= 3:
                            motion_stopped = True
                            motion_stopped_monotonic_s = time.monotonic()
                            break
                    else:
                        consecutive_low_speed_samples = 0
            interrupt_record = {
                "event": "operator_stop",
                "action_id": proposal.action_id,
                "task_id": proposal.task_id,
                "verdict": "MOTION_STOPPED" if motion_stopped else "UNCONFIRMED",
                "operator_stop_received_at": stop_received_at,
                "cancel_acknowledged": cancel_acknowledged,
                "cancel_ack_latency_ms": round(
                    (cancel_ack_monotonic_s - stop_received_monotonic_s) * 1000, 3
                ),
                "motion_stop_threshold_m_s": stop_speed_threshold_m_s,
                "observed_linear_speed_m_s": latest_linear_speed_m_s,
                "motion_stopped_latency_ms": (
                    round((motion_stopped_monotonic_s - stop_received_monotonic_s) * 1000, 3)
                    if motion_stopped_monotonic_s is not None else None
                ),
                "physical_stop_verified": motion_stopped,
            }
            print(json.dumps(interrupt_record), flush=True)
            if outcome_json is not None:
                outcome_json.parent.mkdir(parents=True, exist_ok=True)
                outcome_json.write_text(json.dumps(interrupt_record, indent=2) + "\n",
                                        encoding="utf-8")
            return 7
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=timeout_s)
            cancel_reply = cancel.result() if cancel.done() else None
            timeout_record = {"event": "action_timeout", "action_id": proposal.action_id,
                              "task_id": proposal.task_id, "verdict": "UNCONFIRMED",
                              "cancel_acknowledged": bool(cancel_reply and cancel_reply.goals_canceling),
                              "physical_stop_verified": False}
            print(json.dumps(timeout_record), flush=True)
            if outcome_json is not None:
                outcome_json.parent.mkdir(parents=True, exist_ok=True)
                outcome_json.write_text(json.dumps(timeout_record, indent=2) + "\n", encoding="utf-8")
            # Cancellation acknowledgement is not proof that the vehicle stopped.
            # Do not continue a mission or retry when completion is unknown.
            return 5
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
                proposal.kind in {DroneTaskKind.NAVIGATE, DroneTaskKind.EXPLORE}
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
            takeoff_max_horizontal_displacement_m=takeoff_max_horizontal_displacement_m,
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
        if rclpy.ok():
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
        takeoff_max_horizontal_displacement_m=args.takeoff_max_horizontal_displacement_m,
        landing_max_altitude_m=args.landing_max_altitude_m,
        action_timeout_s=args.action_timeout_s,
    )


if __name__ == "__main__":
    sys.exit(main())
