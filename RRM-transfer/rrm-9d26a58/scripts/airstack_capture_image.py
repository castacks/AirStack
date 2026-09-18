#!/usr/bin/env python3
"""Capture one ROS image plus immutable metadata; never sends a task or control command."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True, help="ROS sensor_msgs/Image topic")
    parser.add_argument("--odometry-topic", required=True,
                        help="canonical nav_msgs/Odometry topic matched to the image")
    parser.add_argument("--state-topic", default=None,
                        help="optional MAVROS state topic; capture proceeds without it")
    parser.add_argument("--output", type=Path, required=True, help="PNG destination")
    parser.add_argument("--timeout-s", type=float, default=15.0)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        raise SystemExit("--timeout-s must be positive")

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image

    has_mavros_state = args.state_topic is not None
    if has_mavros_state:
        from mavros_msgs.msg import State

    class Capture(Node):
        def __init__(self) -> None:
            super().__init__("rrm_readonly_image_capture")
            self.bridge = CvBridge()
            self.record: dict[str, object] | None = None
            self.image: Image | None = None
            self.odometry: Odometry | None = None
            self.state = None  # Optional: may remain None if MAVROS is absent
            self.subscription = self.create_subscription(
                Image, args.topic, self.callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.odometry_subscription = self.create_subscription(
                Odometry, args.odometry_topic, self.odometry_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            if has_mavros_state:
                self.state_subscription = self.create_subscription(
                    State, args.state_topic, self.state_callback,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                )

        def callback(self, message: Image) -> None:
            if self.record is not None:
                return
            self.image = message
            self.write_if_complete()

        def odometry_callback(self, message: Odometry) -> None:
            if self.record is None:
                self.odometry = message
                self.write_if_complete()

        def state_callback(self, message) -> None:
            if self.record is None:
                self.state = message
                self.write_if_complete()

        def write_if_complete(self) -> None:
            if self.record is not None or self.image is None or self.odometry is None:
                return
            # MAVROS state is optional; proceed without it if not configured
            if has_mavros_state and self.state is None:
                return
            image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.output), image):
                raise RuntimeError(f"failed to write {args.output}")
            odometry = self.odometry
            stamp_ns = odometry.header.stamp.sec * 1_000_000_000 + odometry.header.stamp.nanosec
            velocity = odometry.twist.twist.linear
            self.record = {
                "topic": args.topic,
                "encoding": self.image.encoding,
                "source_stamp_ns": self.image.header.stamp.sec * 1_000_000_000
                + self.image.header.stamp.nanosec,
                "frame_id": self.image.header.frame_id,
                "shape": list(image.shape),
                "sha256": _sha256(args.output),
                "capture_mode": "read_only",
                "captured_at": datetime.now(timezone.utc).isoformat(),
                "vehicle": {
                    "connected": self.state.connected if self.state is not None else None,
                    "armed": self.state.armed if self.state is not None else None,
                    "odometry_frame_id": odometry.header.frame_id,
                    "odometry_child_frame_id": odometry.child_frame_id,
                    "odometry_stamp_ns": stamp_ns,
                    "x": odometry.pose.pose.position.x,
                    "y": odometry.pose.pose.position.y,
                    "z": odometry.pose.pose.position.z,
                    "linear_speed_m_s": (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5,
                },
            }

    rclpy.init()
    node = Capture()
    deadline = time.monotonic() + args.timeout_s
    try:
        while node.record is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.record is None:
            raise SystemExit(f"no image arrived on {args.topic} within {args.timeout_s}s")
        sidecar = args.output.with_suffix(args.output.suffix + ".json")
        sidecar.write_text(json.dumps(node.record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"capture={args.output} metadata={sidecar} sha256={node.record['sha256']}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
