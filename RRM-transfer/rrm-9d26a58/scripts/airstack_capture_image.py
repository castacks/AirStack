#!/usr/bin/env python3
"""Capture one ROS image plus immutable metadata; never sends a task or control command."""

from __future__ import annotations

import argparse
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
    from sensor_msgs.msg import Image

    class Capture(Node):
        def __init__(self) -> None:
            super().__init__("rrm_readonly_image_capture")
            self.bridge = CvBridge()
            self.record: dict[str, object] | None = None
            self.subscription = self.create_subscription(
                Image, args.topic, self.callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        def callback(self, message: Image) -> None:
            if self.record is not None:
                return
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.output), image):
                raise RuntimeError(f"failed to write {args.output}")
            self.record = {
                "topic": args.topic,
                "encoding": message.encoding,
                "source_stamp_ns": message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
                "frame_id": message.header.frame_id,
                "shape": list(image.shape),
                "sha256": _sha256(args.output),
                "capture_mode": "read_only",
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
