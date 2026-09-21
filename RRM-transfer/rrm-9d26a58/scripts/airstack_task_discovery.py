#!/usr/bin/env python3
"""Read-only discovery of real AirStack task servers and current flight state."""
from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.action.graph import get_action_names_and_types
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class Discovery(Node):
    def __init__(self, robot: str):
        super().__init__("rrm_task_discovery")
        root = f"/{robot}"
        self.airborne = self.vehicle = self.odometry = self.vdb_map = None
        self.received = {}
        self.create_subscription(Bool, root + "/takeoff_landing_planner/is_airborne",
                                 self._store("airborne"), qos_profile_sensor_data)
        self.create_subscription(State, root + "/interface/mavros/state",
                                 self._store("vehicle"), qos_profile_sensor_data)
        self.create_subscription(Odometry, root + "/odometry_conversion/odometry",
                                 self._store("odometry"), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, root + "/vdb_mapping/vdb_map_pointcloud",
                                 self._store("vdb_map"), qos_profile_sensor_data)

    def _store(self, name):
        def callback(message):
            setattr(self, name, message)
            self.received[name] = time.monotonic()
        return callback


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robot_1")
    parser.add_argument("--timeout-s", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = Discovery(args.robot)
    try:
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline and any(
                getattr(node, name) is None
                for name in ("airborne", "vehicle", "odometry", "vdb_map")):
            rclpy.spin_once(node, timeout_sec=0.1)
        known_actions = {name: sorted(types) for name, types in get_action_names_and_types(node)}
        server_names = set()
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                for topic_name, _ in node.get_publisher_names_and_types_by_node(name, namespace):
                    suffix = "/_action/status"
                    if topic_name.endswith(suffix):
                        server_names.add(topic_name[:-len(suffix)])
            except RuntimeError:
                continue
        servers = {
            name: known_actions[name] for name in sorted(server_names)
            if name.startswith(f"/{args.robot}/tasks/") and name in known_actions
        }
        now = time.monotonic()
        stale = [name for name, received in node.received.items() if now - received > 2.0]
        pose = node.odometry.pose.pose.position if node.odometry is not None else None
        orientation = node.odometry.pose.pose.orientation if node.odometry is not None else None
        yaw = (math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        ) if orientation is not None else None)
        vdb_received = node.received.get("vdb_map")
        report = {
            "schema_version": "airstack-task-discovery/v1",
            "robot_name": args.robot,
            "task_servers": servers,
            "missing_state": [name for name in ("airborne", "vehicle", "odometry")
                              if getattr(node, name) is None],
            "stale_state": stale,
            "connected": bool(node.vehicle.connected) if node.vehicle is not None else None,
            "armed": bool(node.vehicle.armed) if node.vehicle is not None else None,
            "airborne": bool(node.airborne.data) if node.airborne is not None else None,
            "frame_id": node.odometry.header.frame_id if node.odometry is not None else None,
            "child_frame_id": node.odometry.child_frame_id if node.odometry is not None else None,
            "position": ({"x": pose.x, "y": pose.y, "z": pose.z} if pose is not None else None),
            "yaw_rad": yaw,
            "vdb_map_available": node.vdb_map is not None,
            "vdb_map_fresh": bool(vdb_received is not None and now - vdb_received <= 2.0),
            "vdb_map_frame_id": (node.vdb_map.header.frame_id
                                 if node.vdb_map is not None else None),
            "vdb_map_point_count": (node.vdb_map.width * node.vdb_map.height
                                    if node.vdb_map is not None else None),
            "execution_dispatch": False,
        }
        print(json.dumps(report, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
