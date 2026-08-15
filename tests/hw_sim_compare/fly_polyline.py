#!/usr/bin/env python3
"""Fly a map-frame polyline via trajectory_controller, then land.

Runs inside the robot container. Used to replay the hardware waypoint mission
without DROAN (which needs disparity and stalled on the first goal).
"""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from airstack_msgs.srv import TrajectoryMode
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from task_msgs.action import LandTask, TakeoffTask


def _interp_segment(a, b, spacing: float):
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-6:
        return [a]
    n = max(1, int(math.ceil(dist / spacing)))
    return [
        (a[0] + dx * i / n, a[1] + dy * i / n, a[2] + dz * i / n)
        for i in range(n)
    ]


class PolylineFlyer(Node):
    def __init__(self, mission: dict):
        super().__init__("hw_replay_polyline_flyer")
        self.mission = mission
        self.robot = "robot_1"
        self.odom = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry,
            f"/{self.robot}/odometry_conversion/odometry",
            self._odom_cb,
            qos,
        )
        self.traj_pub = self.create_publisher(
            TrajectoryXYZVYaw,
            f"/{self.robot}/trajectory_controller/trajectory_override",
            1,
        )
        self.mode_cli = self.create_client(
            TrajectoryMode,
            f"/{self.robot}/trajectory_controller/set_trajectory_mode",
        )
        self.takeoff_cli = ActionClient(
            self, TakeoffTask, f"/{self.robot}/tasks/takeoff"
        )
        self.land_cli = ActionClient(self, LandTask, f"/{self.robot}/tasks/land")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def wait_odom(self, timeout=15.0):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError("no odometry")

    def xyz(self):
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)

    def set_mode(self, mode: int) -> None:
        if not self.mode_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("set_trajectory_mode unavailable")
        req = TrajectoryMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.result() or not fut.result().success:
            raise RuntimeError(f"set_trajectory_mode {mode} failed")

    def send_action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("action server unavailable")
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("action goal rejected")
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        result = res_fut.result()
        if result is None:
            raise RuntimeError("action timed out")
        if not result.result.success:
            raise RuntimeError(result.result.message)
        return result.result

    def takeoff(self):
        alt = float(self.mission["takeoff_altitude_m"])
        vel = float(self.mission["takeoff_velocity_m_s"])
        self.get_logger().info(f"takeoff {alt}m @ {vel}m/s")
        g = TakeoffTask.Goal()
        g.target_altitude_m = alt
        g.velocity_m_s = vel
        self.send_action(self.takeoff_cli, g, max(40.0, alt / max(vel, 0.1) + 20.0))

    def land(self):
        vel = float(self.mission["land_velocity_m_s"])
        self.get_logger().info(f"land @ {vel}m/s")
        g = LandTask.Goal()
        g.velocity_m_s = vel
        self.send_action(self.land_cli, g, 40.0)

    def publish_path(self, pts, velocity: float) -> None:
        traj = TrajectoryXYZVYaw()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "map"
        for x, y, z in pts:
            wp = WaypointXYZVYaw()
            wp.position = Point(x=float(x), y=float(y), z=float(z))
            wp.velocity = float(velocity)
            wp.yaw = 0.0
            traj.waypoints.append(wp)
        self.set_mode(TrajectoryMode.Request.TRACK)
        # give subscribers a moment after mode switch
        time.sleep(0.2)
        self.traj_pub.publish(traj)
        self.get_logger().info(f"published {len(traj.waypoints)} waypoints")

    def wait_near(self, goal, tol: float, timeout: float) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            d = math.dist(self.xyz(), goal)
            if int((time.time() - t0) * 2) % 4 == 0:
                self.get_logger().info(f"dist to {goal} = {d:.3f}m")
            if d <= tol:
                self.get_logger().info(f"reached {goal} err={d:.3f}m")
                return
        raise RuntimeError(f"timeout approaching {goal}, last {self.xyz()}")

    def run(self) -> None:
        self.wait_odom()
        z = self.xyz()[2]
        alt = float(self.mission["takeoff_altitude_m"])
        if z < alt - 0.25:
            self.takeoff()
            self.wait_odom()

        wps = [(w["x"], w["y"], w["z"]) for w in self.mission["waypoints"]]
        pts = []
        cur = self.xyz()
        chain = [cur] + wps
        for a, b in zip(chain, chain[1:]):
            pts.extend(_interp_segment(a, b, 0.25))
        pts.append(wps[-1])

        # Hardware cruise was ~0.3–0.6 m/s; 0.5 matches the recorded transit.
        self.publish_path(pts, velocity=0.5)
        self.wait_near(wps[-1], float(self.mission.get("goal_tolerance_m", 0.3)), 90.0)
        time.sleep(1.0)
        self.land()


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: fly_polyline.py <mission.json>")
    with open(sys.argv[1], encoding="utf-8") as f:
        mission = json.loads(f.read())
    rclpy.init()
    node = PolylineFlyer(mission)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
