#!/usr/bin/env python3
"""Take off, then stream trajectory overrides that follow the scripted person path."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from airstack_msgs.srv import TrajectoryMode
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from task_msgs.action import TakeoffTask


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def normalize_robot_name(robot_name: str) -> str:
    return robot_name.strip("/")


def namespaced(robot_name: str, suffix: str) -> str:
    return f"/{normalize_robot_name(robot_name)}/{suffix.strip('/')}"


def parse_vec3(value: str, label: str) -> list[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"{label} must have exactly three comma-separated values")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} values must be numbers") from exc


def spin_until(node: Node, future, timeout_s: float, description: str) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            node.get_logger().error(f"Timed out while waiting for {description}")
            return False
    return future.done()


def send_takeoff(node: Node, client: ActionClient, altitude: float, velocity: float, timeout_s: float) -> bool:
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("takeoff action server is not available")
        return False

    goal = TakeoffTask.Goal()
    goal.target_altitude_m = float(altitude)
    goal.velocity_m_s = float(velocity)

    node.get_logger().info("Sending takeoff goal")
    goal_future = client.send_goal_async(goal)
    if not spin_until(node, goal_future, timeout_s, "takeoff goal acceptance"):
        return False

    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("takeoff goal was rejected")
        return False

    result_future = goal_handle.get_result_async()
    if not spin_until(node, result_future, timeout_s, "takeoff result"):
        return False

    wrapped = result_future.result()
    result = wrapped.result
    status_name = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
    success = bool(getattr(result, "success", False)) and wrapped.status == GoalStatus.STATUS_SUCCEEDED
    node.get_logger().info(
        f"takeoff result: status={status_name} success={success} "
        f"message={getattr(result, 'message', '')!r}"
    )
    return success


class PersonFollower(Node):
    def __init__(self, args):
        super().__init__("takeoff_then_follow_person")
        self.args = args
        self.robot_odom = None
        self.follow_started_at = None
        self.follow_enabled = False
        self.published_once = False

        self.create_subscription(
            Odometry,
            args.odometry_topic,
            self._odom_cb,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectoryXYZVYaw,
            namespaced(args.robot, "trajectory_controller/trajectory_override"),
            1,
        )
        self.mode_client = self.create_client(
            TrajectoryMode,
            namespaced(args.robot, "trajectory_controller/set_trajectory_mode"),
        )
        self.timer = self.create_timer(1.0 / args.rate, self._publish_follow_trajectory)

    def _odom_cb(self, msg: Odometry):
        self.robot_odom = msg

    def wait_for_inputs(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot_odom is not None:
                return True
        self.get_logger().error("Timed out waiting for robot odometry")
        return False

    def set_track_mode(self) -> bool:
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("trajectory mode service is not available")
            return False
        req = TrajectoryMode.Request()
        req.mode = TrajectoryMode.Request.TRACK
        future = self.mode_client.call_async(req)
        if not spin_until(self, future, 5.0, "trajectory mode response"):
            return False
        return bool(future.result().success)

    def _target_position(self):
        if self.follow_started_at is None:
            self.follow_started_at = time.monotonic()
        elapsed = max(0.0, time.monotonic() - self.follow_started_at)
        sx, sy, sz = self.args.line_start
        ex, ey, ez = self.args.line_target
        dx = ex - sx
        dy = ey - sy
        dz = ez - sz
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        ratio = 1.0 if length < 1e-6 else min(1.0, elapsed * self.args.line_speed / length)
        px = sx + ratio * dx
        py = sy + ratio * dy
        pz = sz + ratio * dz
        person_yaw = math.atan2(dy, dx) if length >= 1e-6 else 0.0
        return self._apply_offset(px, py, pz, person_yaw)

    def _line_follow_points(self):
        sx, sy, sz = self.args.line_start
        ex, ey, ez = self.args.line_target
        person_yaw = math.atan2(ey - sy, ex - sx)
        start = self._apply_offset(sx, sy, sz, person_yaw)
        target = self._apply_offset(ex, ey, ez, person_yaw)
        return start, target

    def _apply_offset(self, px, py, pz, person_yaw):
        ox = self.args.offset_x
        oy = self.args.offset_y
        oz = self.args.offset_z
        if self.args.offset_frame == "person":
            cos_yaw = math.cos(person_yaw)
            sin_yaw = math.sin(person_yaw)
            ox, oy = cos_yaw * ox - sin_yaw * oy, sin_yaw * ox + cos_yaw * oy

        return px + ox, py + oy, pz + oz

    def _make_waypoint(self, x: float, y: float, z: float, velocity: float, yaw: float) -> WaypointXYZVYaw:
        wp = WaypointXYZVYaw()
        wp.position.x = float(x)
        wp.position.y = float(y)
        wp.position.z = float(z)
        wp.velocity = float(velocity)
        wp.yaw = float(yaw)
        return wp

    def _publish_follow_trajectory(self):
        if not self.follow_enabled:
            return
        if self.robot_odom is None:
            return
        if self.args.publish_once and self.published_once:
            return

        robot = self.robot_odom.pose.pose.position
        if self.args.publish_once:
            tx = robot.x + self.args.relative_end_x
            ty = robot.y + self.args.relative_end_y
            tz = robot.z + self.args.relative_end_z
            yaw = math.atan2(ty - robot.y, tx - robot.x)
            waypoints = [
                self._make_waypoint(robot.x, robot.y, robot.z, 0.1, yaw),
                self._make_waypoint(tx, ty, tz, self.args.velocity, yaw),
            ]
        else:
            tx, ty, tz = self._target_position()
            yaw = math.atan2(ty - robot.y, tx - robot.x)
            waypoints = [
                self._make_waypoint(robot.x, robot.y, robot.z, 0.1, yaw),
                self._make_waypoint(tx, ty, tz, self.args.velocity, yaw),
            ]

        traj = TrajectoryXYZVYaw()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = self.args.frame_id
        traj.waypoints = waypoints
        self.trajectory_pub.publish(traj)
        self.published_once = True
        if self.args.publish_once:
            self.get_logger().info(
                f"Published follow trajectory to ({tx:.2f}, {ty:.2f}, {tz:.2f})"
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Take off, then follow the scripted person path by streaming trajectory overrides."
    )
    parser.add_argument("--robot", default="/robot_1", help="Robot namespace, e.g. /robot_1")
    parser.add_argument(
        "--odometry-topic",
        default="/robot_1/odometry_conversion/odometry",
        help="Odometry topic used to seed follow trajectories.",
    )
    parser.add_argument("--skip-takeoff", action="store_true")
    parser.add_argument("--takeoff-altitude", type=float, default=2.0)
    parser.add_argument("--takeoff-velocity", type=float, default=1.0)
    parser.add_argument("--takeoff-timeout", type=float, default=120.0)
    parser.add_argument("--wait-after-takeoff", type=float, default=0.5)
    parser.add_argument("--input-timeout", type=float, default=20.0)

    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--offset-frame", choices=("map", "person"), default="map")
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=2.0)
    parser.add_argument("--offset-z", type=float, default=2.0)
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument(
        "--stream-overrides",
        dest="publish_once",
        action="store_false",
        help="Continuously republish trajectory overrides. This resets the trajectory controller each publish.",
    )
    parser.set_defaults(publish_once=True)
    parser.add_argument("--relative-end-x", type=float, default=0.0)
    parser.add_argument("--relative-end-y", type=float, default=2.0)
    parser.add_argument("--relative-end-z", type=float, default=0.0)
    parser.add_argument("--line-start", default="-9.0,-10.5,0.0")
    parser.add_argument("--line-target", default="-4.4,-10.5,0.0")
    parser.add_argument("--line-speed", type=float, default=1.0)
    args = parser.parse_args()
    args.line_start = parse_vec3(args.line_start, "--line-start")
    args.line_target = parse_vec3(args.line_target, "--line-target")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = PersonFollower(args)

    try:
        if not args.skip_takeoff:
            takeoff_client = ActionClient(node, TakeoffTask, namespaced(args.robot, "tasks/takeoff"))
            if not send_takeoff(
                node,
                takeoff_client,
                args.takeoff_altitude,
                args.takeoff_velocity,
                args.takeoff_timeout,
            ):
                return 1
            if args.wait_after_takeoff > 0.0:
                node.get_logger().info(f"Waiting {args.wait_after_takeoff:.1f}s after takeoff")
                time.sleep(args.wait_after_takeoff)

        if not node.wait_for_inputs(args.input_timeout):
            return 1
        if not node.set_track_mode():
            return 1
        node.follow_started_at = None
        node.follow_enabled = True

        target_description = f"line {args.line_start} -> {args.line_target} at {args.line_speed:.2f} m/s"
        node.get_logger().info(
            f"Following {target_description} with offset "
            f"({args.offset_x:.2f}, {args.offset_y:.2f}, {args.offset_z:.2f}) in {args.offset_frame} frame"
        )
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
