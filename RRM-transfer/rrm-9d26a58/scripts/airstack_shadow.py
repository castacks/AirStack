#!/usr/bin/env python3
"""Observe an AirStack robot into an RRM shadow-mode evidence bundle.

This process creates ROS subscriptions and a timer only.  It contains no action,
service, publisher, trajectory, or PX4-command client.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from uuid import uuid4

from rrm.airstack_shadow import AirStackShadowAdapter, JsonlEvidenceWriter


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="robot_1")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--snapshot-period-s", type=float, default=1.0)
    parser.add_argument("--max-observation-age-s", type=float, default=1.0)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.duration_s <= 0 or args.snapshot_period_s <= 0:
        raise SystemExit("duration and snapshot period must be positive")

    # Keep ROS imports here so contract/unit tests require only the standard library.
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    run_id = f"shadow-{uuid4().hex}"
    prefix = f"/{args.robot_name}"
    source_topics = {
        "odometry": f"{prefix}/interface/mavros/local_position/odom",
        "mavros_state": f"{prefix}/interface/mavros/state",
        "tf": "/tf",
        "task_status": [
            f"{prefix}/tasks/{action}/_action/status"
            for action in ("takeoff", "navigate", "land")
        ],
    }
    writer = JsonlEvidenceWriter(args.output_dir, {
        "run_id": run_id,
        "robot_name": args.robot_name,
        "source_topics": source_topics,
        "max_observation_age_s": args.max_observation_age_s,
    })
    adapter = AirStackShadowAdapter(
        run_id=run_id,
        robot_name=args.robot_name,
        max_observation_age_s=args.max_observation_age_s,
        writer=writer,
    )
    rclpy.init()
    node = Node("rrm_shadow_observer")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_odom(message: Odometry) -> None:
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        pose = message.pose.pose.position
        adapter.ingest_odometry(
            x=pose.x, y=pose.y, z=pose.z,
            frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            source_stamp_ns=stamp_ns,
        )

    def on_state(message: State) -> None:
        adapter.ingest_mavros_state(
            connected=message.connected,
            armed=message.armed,
            mode=message.mode,
        )

    def on_tf(message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
                adapter.ingest_map_to_base_link_transform()
                break

    def task_callback(action: str):
        def on_status(message: GoalStatusArray) -> None:
            adapter.ingest_task_status(
                action=action,
                status_codes=[item.status for item in message.status_list],
            )
        return on_status

    node.create_subscription(Odometry, source_topics["odometry"], on_odom, qos)
    node.create_subscription(State, source_topics["mavros_state"], on_state, qos)
    node.create_subscription(TFMessage, source_topics["tf"], on_tf, qos)
    for action in ("takeoff", "navigate", "land"):
        node.create_subscription(
            GoalStatusArray,
            f"{prefix}/tasks/{action}/_action/status",
            task_callback(action),
            qos,
        )

    last_snapshot = None

    def snapshot_timer() -> None:
        nonlocal last_snapshot
        last_snapshot = adapter.snapshot()
        node.get_logger().info(
            f"shadow snapshot={last_snapshot.sequence} readiness={last_snapshot.readiness} "
            f"execution_inhibited={last_snapshot.execution_inhibited}"
        )

    node.create_timer(args.snapshot_period_s, snapshot_timer)
    deadline = time.monotonic() + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.25, deadline - time.monotonic()))
        if last_snapshot is None:
            last_snapshot = adapter.snapshot()
        report = adapter.completeness_report(last_snapshot)
        (args.output_dir / "replay-report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 0
    finally:
        writer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
