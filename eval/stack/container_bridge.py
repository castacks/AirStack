#!/usr/bin/env python3
"""In-container ROS 2 bridge for the S.A.F.E. eval harness.

Runs INSIDE the robot container (it needs rclpy + airstack_msgs, which the
host does not have) as a single long-lived `docker exec -i` process:

    stdin  ← JSON lines from the host harness
    stdout → JSON lines to the host harness

It only touches the stack's REAL interfaces:
    publishes  /{robot}/global_plan                                (nav_msgs/Path)
    publishes  /{robot}/trajectory_controller/trajectory_override  (airstack_msgs/TrajectoryXYZVYaw)
    subscribes /{robot}/odometry_conversion/odometry               (frame_id + EKF pose passthrough)

(Scenario control and ground truth ride a direct TCP link to the sim — see
eval/stack/scenario_client.py — not this bridge.)

stdin message kinds:
    {"kind": "global_plan",   "waypoints": [[x,y,z], ...]}
    {"kind": "traj_override", "waypoints": [[x,y,z], ...], "velocity": 1.0}

stdout message kinds (one JSON object per line):
    {"kind": "odom", "frame": ..., "pos": [...]} — throttled EKF odometry
    {"kind": "ready"}           — bridge is up and spinning
"""

import argparse
import json
import queue
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

try:
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
except ImportError:  # workspace not built — global_plan / command paths still work
    TrajectoryXYZVYaw = WaypointXYZVYaw = None


def emit(obj: dict) -> None:
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


class SafeEvalBridge(Node):

    def __init__(self, robot: str):
        super().__init__(
            "safe_eval_bridge",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.robot = robot
        self._frame = "map"
        self._last_odom_emit = 0.0

        self.plan_pub = self.create_publisher(Path, f"/{robot}/global_plan", 1)
        self.traj_pub = (
            self.create_publisher(TrajectoryXYZVYaw,
                                  f"/{robot}/trajectory_controller/trajectory_override", 1)
            if TrajectoryXYZVYaw is not None else None
        )

        self.create_subscription(Odometry, f"/{robot}/odometry_conversion/odometry",
                                 self._on_odom, 10)

        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self.create_timer(0.02, self._drain_inbox)
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        emit({"kind": "ready"})

    # ── ROS → host ────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self._frame = msg.header.frame_id or self._frame
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self._last_odom_emit >= 1.0:
            self._last_odom_emit = t
            p = msg.pose.pose.position
            emit({"kind": "odom", "frame": self._frame,
                  "pos": [round(p.x, 3), round(p.y, 3), round(p.z, 3)]})

    # ── host → ROS ────────────────────────────────────────────────────────

    def _stdin_loop(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                self._inbox.put(json.loads(line))
            except (ValueError, TypeError):
                emit({"kind": "error", "error": f"bad stdin line: {line[:120]}"})

    def _drain_inbox(self):
        while True:
            try:
                msg = self._inbox.get_nowait()
            except queue.Empty:
                return
            try:
                self._handle(msg)
            except Exception as e:
                emit({"kind": "error", "error": str(e)})

    def _handle(self, msg: dict):
        kind = msg.get("kind")
        if kind == "global_plan":
            self.plan_pub.publish(self._make_path(msg["waypoints"]))
        elif kind == "traj_override":
            if self.traj_pub is None:
                raise RuntimeError("airstack_msgs not available — build the workspace first")
            self.traj_pub.publish(
                self._make_traj(msg["waypoints"], float(msg.get("velocity", 1.0))))
        else:
            raise ValueError(f"unknown kind '{kind}'")

    def _make_path(self, waypoints) -> Path:
        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = float(wp[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _make_traj(self, waypoints, velocity: float):
        traj = TrajectoryXYZVYaw()
        traj.header.frame_id = self._frame
        traj.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            w = WaypointXYZVYaw()
            w.position.x = float(wp[0])
            w.position.y = float(wp[1])
            w.position.z = float(wp[2])
            w.velocity = float(wp[3]) if len(wp) > 3 else velocity
            w.yaw = 0.0
            traj.waypoints.append(w)
        return traj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robot_1")
    args = parser.parse_args()

    rclpy.init()
    node = SafeEvalBridge(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
