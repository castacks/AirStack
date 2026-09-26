#!/usr/bin/env python3
"""
Pre-armed takeoff diagnostic: captures the full control chain from trajectory
through PID through attitude to PX4, starting BEFORE the takeoff command and
continuing through the entire attempt.

Records to a JSONL file:
  - tracking_point (what the trajectory controller commands the PID to follow)
  - odom (where the drone actually is)
  - PID z/vz info (position and velocity Z PID outputs)
  - attitude_target (roll/pitch/yaw/thrust sent to PX4)
  - mavros state (armed, mode, connected)
  - extended_state (landed_state)

Usage:
  docker exec airstack-robot-desktop-1 bash -lc '
    source /root/AirStack/robot/ros_ws/install/setup.bash
    python3 /root/AirStack/RRM-transfer/rrm-9d26a58/scripts/takeoff_diagnostic.py \
      --output /root/AirStack/.rrm-artifacts/takeoff-diagnostics/full-chain-trace.jsonl \
      --duration 30
  '
"""
import argparse
import json
import os
import sys
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Message types
from airstack_msgs.msg import Odometry as AirstackOdometry
from nav_msgs.msg import Odometry as NavOdometry
from mavros_msgs.msg import AttitudeTarget, State, ExtendedState
from pid_controller_msgs.msg import PIDInfo
from std_msgs.msg import Float32


class TakeoffDiagnosticNode(Node):
    def __init__(self, output_path, duration):
        super().__init__("takeoff_diagnostic")
        self.output_path = output_path
        self.duration = duration
        self.start_time = None
        self.samples = []
        self.lock = threading.Lock()

        # Latest state for each channel
        self._latest = {}

        # Best-effort QoS for MAVROS topics
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Tracking point from trajectory controller
        self.create_subscription(
            AirstackOdometry,
            "/robot_1/trajectory_controller/tracking_point",
            lambda msg: self._record("tracking_point", {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "vx": msg.twist.linear.x,
                "vy": msg.twist.linear.y,
                "vz": msg.twist.linear.z,
                "frame": msg.header.frame_id,
            }),
            10,
        )

        # Physical odometry
        self.create_subscription(
            NavOdometry,
            "/robot_1/odometry_conversion/odometry",
            lambda msg: self._record("odom", {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "vz": msg.twist.twist.linear.z,
                "frame": msg.header.frame_id,
            }),
            sensor_qos,
        )

        # Z PID info
        self.create_subscription(
            PIDInfo,
            "/robot_1/pid_controller/z_pid_info",
            lambda msg: self._record("z_pid", {
                "target": msg.target,
                "measured": msg.measured,
                "error": msg.error,
                "p_comp": msg.p_component,
                "i_comp": msg.i_component,
                "d_comp": msg.d_component,
                "control": msg.control,
                "dt": msg.dt,
            }),
            10,
        )

        # VZ PID info
        self.create_subscription(
            PIDInfo,
            "/robot_1/pid_controller/vz_pid_info",
            lambda msg: self._record("vz_pid", {
                "target": msg.target,
                "measured": msg.measured,
                "error": msg.error,
                "p_comp": msg.p_component,
                "i_comp": msg.i_component,
                "d_comp": msg.d_component,
                "ff_comp": msg.ff_component,
                "constant": msg.constant,
                "control": msg.control,
                "dt": msg.dt,
            }),
            10,
        )

        # X PID info
        self.create_subscription(
            PIDInfo,
            "/robot_1/pid_controller/x_pid_info",
            lambda msg: self._record("x_pid", {
                "target": msg.target,
                "measured": msg.measured,
                "error": msg.error,
                "control": msg.control,
            }),
            10,
        )

        # Y PID info
        self.create_subscription(
            PIDInfo,
            "/robot_1/pid_controller/y_pid_info",
            lambda msg: self._record("y_pid", {
                "target": msg.target,
                "measured": msg.measured,
                "error": msg.error,
                "control": msg.control,
            }),
            10,
        )

        # Attitude target sent to PX4
        self.create_subscription(
            AttitudeTarget,
            "/robot_1/interface/mavros/setpoint_raw/attitude",
            lambda msg: self._record("attitude_cmd", {
                "qx": msg.orientation.x,
                "qy": msg.orientation.y,
                "qz": msg.orientation.z,
                "qw": msg.orientation.w,
                "thrust": msg.thrust,
                "type_mask": msg.type_mask,
            }),
            10,
        )

        # Target attitude (what PX4 acknowledges)
        self.create_subscription(
            AttitudeTarget,
            "/robot_1/interface/mavros/setpoint_raw/target_attitude",
            lambda msg: self._record("px4_target_att", {
                "qx": msg.orientation.x,
                "qy": msg.orientation.y,
                "qz": msg.orientation.z,
                "qw": msg.orientation.w,
                "thrust": msg.thrust,
            }),
            sensor_qos,
        )

        # MAVROS state
        self.create_subscription(
            State,
            "/robot_1/interface/mavros/state",
            lambda msg: self._record("state", {
                "armed": msg.armed,
                "mode": msg.mode,
                "connected": msg.connected,
            }),
            sensor_qos,
        )

        # Extended state
        self.create_subscription(
            ExtendedState,
            "/robot_1/interface/mavros/extended_state",
            lambda msg: self._record("ext_state", {
                "landed_state": msg.landed_state,
            }),
            sensor_qos,
        )

        # Trajectory completion
        self.create_subscription(
            Float32,
            "/robot_1/trajectory_controller/trajectory_completion_percentage",
            lambda msg: self._record("traj_completion", {
                "pct": msg.data,
            }),
            10,
        )

        # Tracking error
        self.create_subscription(
            Float32,
            "/robot_1/trajectory_controller/tracking_error",
            lambda msg: self._record("tracking_error", {
                "error": msg.data,
            }),
            10,
        )

        # 50 Hz consolidated snapshot
        self.create_timer(0.02, self._snapshot_timer)

        self.get_logger().info(
            f"Takeoff diagnostic started, recording for {duration}s to {output_path}"
        )

    def _record(self, channel, data):
        """Update latest data for a channel."""
        with self.lock:
            self._latest[channel] = data

    def _snapshot_timer(self):
        """Write consolidated snapshot at 50 Hz."""
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self._flush_and_exit()
            return

        with self.lock:
            snapshot = {"t": elapsed}
            snapshot.update({k: v for k, v in self._latest.items()})

        self.samples.append(snapshot)

    def _flush_and_exit(self):
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, "w") as f:
            for s in self.samples:
                f.write(json.dumps(s, default=str) + "\n")
        self.get_logger().info(
            f"Wrote {len(self.samples)} snapshots to {self.output_path}"
        )
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = TakeoffDiagnosticNode(args.output, args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
