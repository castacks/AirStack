#!/usr/bin/env python3
"""ROS 2 bridge sidecar for AirStack × S.A.F.E. benchmark.

Run this inside the AirStack robot Docker container after sourcing the ROS 2
Jazzy workspace.  The benchmark process (running in the Isaac Sim Python process)
connects over newline-delimited JSON on AIRSTACK_DROAN_PORT or AIRSTACK_SUPER_PORT.

Protocol:
    reset  → {"type": "reset"}            ← {"ok": true}
    step   → {"type": "step",
               "pos": [x,y,z],
               "target": [x,y,z],
               "obstacles": [[x,y,z], ...],
               "env_bounds": [xmin,ymin,zmin,xmax,ymax,zmax]}
           ← {"cmd": {"velocity": [vx,vy,vz]}}   (or {"cmd": null} if not ready)

The sidecar:
  1. Publishes odometry (nav_msgs/Odometry) from obs["pos"]
  2. Publishes goal    (geometry_msgs/PoseStamped) from obs["target"]
  3. Publishes costmap / occupancy from obs["obstacles"] inflated to AirStack's
     costmap resolution, forwarded to the planner's expected topic
  4. Reads the planner's velocity output from /ROBOT_NAME/planning/pos_cmd
     (TrajectoryPoint) and returns the velocity field

Usage:
    python3 airstack_ros_sidecar.py --planner droan
    python3 airstack_ros_sidecar.py --planner super
"""

from __future__ import annotations

import argparse
import json
import os
import socketserver
import struct
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


ROBOT_NAME = os.environ.get("ROBOT_NAME", "drone")

# Topic names match AirStack's standard topic conventions (AGENTS.md)
ODOM_TOPIC   = f"/{ROBOT_NAME}/odometry"
GOAL_TOPIC   = f"/{ROBOT_NAME}/goal"
CLOUD_TOPIC  = f"/{ROBOT_NAME}/safe_benchmark/obstacles"
CMD_TOPIC    = f"/{ROBOT_NAME}/planning/pos_cmd"

WORLD_FRAME  = "map"


def _get_port(planner: str) -> int:
    if planner == "droan":
        return int(os.environ.get("AIRSTACK_DROAN_PORT", "8780"))
    if planner == "super":
        return int(os.environ.get("AIRSTACK_SUPER_PORT", "8781"))
    return int(os.environ.get("AIRSTACK_SIDECAR_PORT", "8780"))


class AirstackBridge(Node):
    def __init__(self, planner: str) -> None:
        super().__init__(f"safe_airstack_{planner}_sidecar")
        self._lock = threading.Lock()
        self._latest_cmd: Any = None
        self._goal_sent = False

        self.odom_pub  = self.create_publisher(Odometry,      ODOM_TOPIC,  10)
        self.goal_pub  = self.create_publisher(PoseStamped,   GOAL_TOPIC,  10)
        self.cloud_pub = self.create_publisher(PointCloud2,   CLOUD_TOPIC, 10)

        # Subscribe to the planner's velocity output.
        # AirStack trajectory controller listens on trajectory_controller/trajectory_segment_to_add
        # but the planner should output a TwistStamped on pos_cmd or similar.
        # Adapt the topic/message type to whichever the chosen planner publishes.
        self.cmd_sub = self.create_subscription(
            TwistStamped,
            CMD_TOPIC,
            self._on_cmd,
            10,
        )

    def reset(self) -> dict:
        with self._lock:
            self._latest_cmd = None
            self._goal_sent  = False
        return {"ok": True}

    def step(self, req: dict) -> dict:
        stamp = self.get_clock().now().to_msg()
        pos    = _vec3(req.get("pos"))
        target = _vec3(req.get("target"))
        obstacles = req.get("obstacles", [])

        self._publish_odom(pos, stamp)

        with self._lock:
            has_cmd = self._latest_cmd is not None

        if not has_cmd:
            self._publish_goal(target, stamp)

        if obstacles:
            self._publish_cloud(obstacles, stamp)

        with self._lock:
            cmd = self._latest_cmd

        if cmd is None:
            return {"cmd": None}

        vx = cmd.twist.linear.x
        vy = cmd.twist.linear.y
        vz = cmd.twist.linear.z
        return {"cmd": {"velocity": [vx, vy, vz]}}

    def _on_cmd(self, msg: TwistStamped) -> None:
        with self._lock:
            self._latest_cmd = msg

    def _publish_odom(self, pos: list[float], stamp: Any) -> None:
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.child_frame_id  = f"{ROBOT_NAME}/base_link"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def _publish_goal(self, target: list[float], stamp: Any) -> None:
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _publish_cloud(self, pts: list[list[float]], stamp: Any) -> None:
        """Publish obstacle centres as a PointCloud2 for the planner's costmap."""
        n = len(pts)
        if n == 0:
            return
        msg = PointCloud2()
        msg.header.stamp    = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.height    = 1
        msg.width     = n
        msg.is_dense  = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 × float32
        msg.row_step   = 12 * n
        msg.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        flat = [coord for pt in pts for coord in (float(pt[0]), float(pt[1]), float(pt[2]))]
        msg.data = struct.pack("<" + "f" * (3 * n), *flat)
        self.cloud_pub.publish(msg)


def _vec3(value: Any) -> list[float]:
    value = value or []
    out = [0.0, 0.0, 0.0]
    for i, item in enumerate(value[:3]):
        out[i] = float(item)
    return out


class Handler(socketserver.StreamRequestHandler):
    bridge: AirstackBridge

    def handle(self) -> None:
        while True:
            line = self.rfile.readline()
            if not line:
                break
            try:
                req = json.loads(line.decode("utf-8"))
                if req.get("type") == "reset":
                    resp = self.bridge.reset()
                elif req.get("type") == "step":
                    resp = self.bridge.step(req)
                else:
                    resp = {"error": f"unknown type: {req.get('type')}"}
            except Exception as exc:
                resp = {"error": str(exc)}
            try:
                self.wfile.write((json.dumps(resp, separators=(",", ":")) + "\n").encode())
                self.wfile.flush()
            except OSError:
                break


class _ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", choices=("droan", "super"), default="droan")
    args = parser.parse_args()

    port = _get_port(args.planner)

    rclpy.init()
    bridge = AirstackBridge(args.planner)
    Handler.bridge = bridge

    server = _ThreadingTCPServer(("0.0.0.0", port), Handler)

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    bridge.get_logger().warn(f"AirStack {args.planner} sidecar listening on 0.0.0.0:{port}")
    try:
        server.serve_forever()
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
