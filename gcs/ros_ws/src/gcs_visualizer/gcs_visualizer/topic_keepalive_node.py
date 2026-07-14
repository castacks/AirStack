#!/usr/bin/env python3
"""Keep GCS-side routed visualization topics discoverable for Foxglove."""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TopicKeepalive(Node):
    def __init__(self):
        super().__init__("gcs_topic_keepalive")
        self.declare_parameter("robot_name_prefix", "robot")
        self.declare_parameter("num_robots", int(os.getenv("NUM_ROBOTS", "1")))

        prefix = self.get_parameter("robot_name_prefix").value
        num_robots = self.get_parameter("num_robots").value

        self._subs = []
        for index in range(1, num_robots + 1):
            robot_name = f"{prefix}_{index}"
            self._subscribe_image(f"/{robot_name}/reid_overlay")

        self.get_logger().info(
            f"subscribed to {len(self._subs)} GCS keepalive topic(s)"
        )

    def _subscribe_image(self, topic):
        self._subs.append(
            self.create_subscription(Image, topic, lambda _msg: None, SENSOR_QOS)
        )


def main(args=None):
    rclpy.init(args=args)
    node = TopicKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
