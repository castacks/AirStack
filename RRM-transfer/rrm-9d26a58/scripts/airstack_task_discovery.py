#!/usr/bin/env python3
"""Read-only discovery of real AirStack task servers and current flight state."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))

import rclpy
from rclpy.action.graph import get_action_server_names_and_types_by_node
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from rrm.airstack_environment import finite_xyz_bounds


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


def vdb_bounds(message: PointCloud2 | None) -> dict[str, float] | None:
    """Extract finite VDB point extents without treating them as free-space truth."""
    if message is None:
        return None
    values = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True,
    )

    def xyz_points():
        for value in values:
            names = getattr(getattr(value, "dtype", None), "names", None)
            if names and all(name in names for name in ("x", "y", "z")):
                yield value["x"], value["y"], value["z"]
            else:
                yield value[0], value[1], value[2]

    return finite_xyz_bounds(xyz_points())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robot_1")
    parser.add_argument("--timeout-s", type=float, default=4.0)
    parser.add_argument("--state-only", action="store_true",
                        help="return after flight state arrives; map evidence remains best-effort")
    args = parser.parse_args()
    rclpy.init()
    node = Discovery(args.robot)
    try:
        deadline = time.monotonic() + args.timeout_s
        required = ("airborne", "vehicle", "odometry") if args.state_only else (
            "airborne", "vehicle", "odometry", "vdb_map"
        )
        while time.monotonic() < deadline and any(
                getattr(node, name) is None for name in required):
            rclpy.spin_once(node, timeout_sec=0.1)
        server_types = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                for action_name, types in get_action_server_names_and_types_by_node(
                        node, name, namespace):
                    server_types.setdefault(action_name, set()).update(types)
            except RuntimeError:
                continue
        servers = {
            name: sorted(types) for name, types in sorted(server_types.items())
            if name.startswith(f"/{args.robot}/tasks/")
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
        flight_state_reasons = []
        if pose is not None and node.airborne is not None:
            if node.airborne.data and pose.z <= 0.3:
                flight_state_reasons.append("airborne_flag_below_0.3m_map_altitude")
            elif not node.airborne.data and pose.z > 0.5:
                flight_state_reasons.append("grounded_flag_above_0.5m_map_altitude")
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
            "flight_state_consistent": not flight_state_reasons,
            "flight_state_reasons": flight_state_reasons,
            "vdb_map_available": node.vdb_map is not None,
            "vdb_map_fresh": bool(vdb_received is not None and now - vdb_received <= 2.0),
            "vdb_map_frame_id": (node.vdb_map.header.frame_id
                                 if node.vdb_map is not None else None),
            "vdb_map_point_count": (node.vdb_map.width * node.vdb_map.height
                                    if node.vdb_map is not None else None),
            "vdb_map_bounds": vdb_bounds(node.vdb_map),
            "execution_dispatch": False,
        }
        print(json.dumps(report, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
