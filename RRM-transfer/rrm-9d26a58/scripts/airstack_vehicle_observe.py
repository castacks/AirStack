#!/usr/bin/env python3
"""Read fresh AirStack vehicle state for reconciliation; never issue a command."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="robot_1")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--minimum-odometry-samples", type=int, default=3)
    args = parser.parse_args()
    if args.timeout_s <= 0 or args.minimum_odometry_samples < 3:
        raise SystemExit("observation bounds are invalid")

    import rclpy
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("rrm_grounded_state_observer")
    latest_state = None
    latest_odometry = None
    latest_speed_m_s = None
    source_stamps: set[int] = set()

    def on_state(message: State) -> None:
        nonlocal latest_state
        latest_state = message

    def on_odometry(message: Odometry) -> None:
        nonlocal latest_odometry, latest_speed_m_s
        latest_odometry = message
        stamp = message.header.stamp
        source_stamps.add(stamp.sec * 1_000_000_000 + stamp.nanosec)
        velocity = message.twist.twist.linear
        latest_speed_m_s = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    prefix = f"/{args.robot_name}"
    node.create_subscription(State, f"{prefix}/interface/mavros/state", on_state, qos)
    node.create_subscription(
        Odometry, f"{prefix}/odometry_conversion/odometry", on_odometry, qos
    )
    deadline = time.monotonic() + args.timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        if latest_state is not None and len(source_stamps) >= args.minimum_odometry_samples:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))

    try:
        if latest_state is None or latest_odometry is None or latest_speed_m_s is None:
            raise RuntimeError("fresh vehicle state and odometry were not both observed")
        if len(source_stamps) < args.minimum_odometry_samples:
            raise RuntimeError("insufficient fresh odometry samples")
        position = latest_odometry.pose.pose.position
        stamp = latest_odometry.header.stamp
        record = {
            "schema_version": "rrm-grounded-observation/v1",
            "observed_at": datetime.now(timezone.utc).isoformat(),
            "robot_name": args.robot_name,
            "connected": bool(latest_state.connected),
            "armed": bool(latest_state.armed),
            "mode": latest_state.mode,
            "frame_id": latest_odometry.header.frame_id,
            "child_frame_id": latest_odometry.child_frame_id,
            "source_stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            "odometry_samples": len(source_stamps),
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "linear_speed_m_s": latest_speed_m_s,
        }
        print(json.dumps(record, sort_keys=True), flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
