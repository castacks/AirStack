#!/usr/bin/env python3
"""Read-only AirStack preflight observer for one navigation corridor.

This process subscribes and inspects the ROS graph. It creates no publisher, service
client, or action client and cannot send a task or vehicle command.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
import tf2_ros


class Observer(Node):
    def __init__(self, robot: str):
        super().__init__("rrm_feasibility_observer")
        root = f"/{robot}"
        self.odometry = self.vehicle = self.cloud = None
        self.airborne = self.has_control = self.stuck = None
        self.received = {}
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, root + "/odometry_conversion/odometry",
                                 self._store("odometry"), qos_profile_sensor_data)
        self.create_subscription(State, root + "/interface/mavros/state",
                                 self._store("vehicle"), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, root + "/sensors/ouster/point_cloud",
                                 self._store("cloud"), qos_profile_sensor_data)
        self.create_subscription(Bool, root + "/takeoff_landing_planner/is_airborne",
                                 self._store("airborne"), qos_profile_sensor_data)
        self.create_subscription(Bool, root + "/interface/has_control",
                                 self._store("has_control"), qos_profile_sensor_data)
        self.create_subscription(Bool, root + "/droan/stuck",
                                 self._store("stuck"), qos_profile_sensor_data)

    def _store(self, name):
        def callback(message):
            setattr(self, name, message)
            self.received[name] = time.monotonic()
        return callback


def _rotation_translation(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])
    t = transform.translation
    return rotation, np.array([t.x, t.y, t.z])


def _corridor_report(node: Observer, target: np.ndarray, clearance: float) -> dict:
    cloud = node.cloud
    transform = node.tf_buffer.lookup_transform(
        "map", cloud.header.frame_id, Time(), timeout=Duration(seconds=1)
    ).transform
    rotation, translation = _rotation_translation(transform)
    raw = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    points = np.asarray(raw, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    mapped = points @ rotation.T + translation
    pose = node.odometry.pose.pose.position
    start = np.array([pose.x, pose.y, pose.z], dtype=float)
    vector = target - start
    length = float(np.linalg.norm(vector))
    if length <= 1e-6:
        minimum_clearance, obstacle_count = math.inf, 0
    else:
        relative = mapped - start
        fraction = np.clip((relative @ vector) / (length * length), 0.0, 1.0)
        closest = start + fraction[:, None] * vector
        distances = np.linalg.norm(mapped - closest, axis=1)
        # Ignore returns immediately around the vehicle body while retaining the
        # endpoint. The Ouster does not normally see the vehicle itself, but this
        # keeps fixture/mount points from becoming false obstacles.
        start_distance = np.linalg.norm(mapped - start, axis=1)
        relevant = distances[start_distance > clearance]
        minimum_clearance = float(np.min(relevant)) if relevant.size else math.inf
        obstacle_count = int(np.count_nonzero(relevant < clearance))
    sensor_ranges = np.linalg.norm(points, axis=1)
    observed_range = float(np.percentile(sensor_ranges, 90)) if sensor_ranges.size else 0.0
    coverage = bool(points.shape[0] >= 500 and observed_range >= length + clearance)
    return {
        "cloud_frame": cloud.header.frame_id,
        "point_count": int(points.shape[0]),
        "observed_range_m": observed_range,
        "corridor_length_m": length,
        "required_clearance_m": clearance,
        "minimum_observed_clearance_m": minimum_clearance,
        "obstacle_point_count": obstacle_count,
        "coverage_sufficient": coverage,
        "collision_free": coverage and obstacle_count == 0,
        "start": {"x": float(start[0]), "y": float(start[1]), "z": float(start[2])},
        "target": {"x": float(target[0]), "y": float(target[1]), "z": float(target[2])},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robot_1")
    parser.add_argument("--target-x", required=True, type=float)
    parser.add_argument("--target-y", required=True, type=float)
    parser.add_argument("--target-z", required=True, type=float)
    parser.add_argument("--clearance-m", default=0.4, type=float)
    parser.add_argument("--timeout-s", default=8.0, type=float)
    args = parser.parse_args()
    rclpy.init()
    node = Observer(args.robot)
    deadline = time.monotonic() + args.timeout_s
    required = ("odometry", "vehicle", "cloud", "airborne", "has_control", "stuck")
    try:
        while time.monotonic() < deadline and any(getattr(node, name) is None for name in required):
            rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        missing = [name for name in required if getattr(node, name) is None]
        stale = [name for name in required if name in node.received and now - node.received[name] > 2.0]
        actions = {name: types for name, types in get_action_names_and_types(node)}
        report = {
            "schema_version": "airstack-feasibility-observation/v1",
            "observed_monotonic_s": now,
            "missing_channels": missing,
            "stale_channels": stale,
            "actions": actions,
            "execution_dispatch": False,
        }
        if not missing:
            twist = node.odometry.twist.twist.linear
            report.update({
                "connected": bool(node.vehicle.connected),
                "armed": bool(node.vehicle.armed),
                "airborne": bool(node.airborne.data),
                "has_control": bool(node.has_control.data),
                "planner_stuck": bool(node.stuck.data),
                "linear_speed_m_s": math.sqrt(twist.x*twist.x + twist.y*twist.y + twist.z*twist.z),
                "odometry_frame_id": node.odometry.header.frame_id,
                "odometry_child_frame_id": node.odometry.child_frame_id,
            })
            try:
                report["corridor"] = _corridor_report(
                    node, np.array([args.target_x, args.target_y, args.target_z]), args.clearance_m,
                )
            except Exception as error:
                report["corridor_error"] = type(error).__name__
        print(json.dumps(report, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
