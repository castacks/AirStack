#!/usr/bin/env python3
"""Keep rayfronts' per-query voxel heatmaps alive by subscribing silently.

rayfronts only publishes a voxel-similarity topic while it has a subscriber
(publish_query_results guards each cloud with `if has_subscriber(pub)`). Only
.../voxels_sim/all is normally subscribed (by raven / semantic_search_task), so
the per-query heatmaps .../voxels_sim/q{q}_{label} never stream — and the GCS
(which relies on the DDS router bridge) sees nothing, because the router's bare
DDS reader does NOT count as a subscriber (same reason topic_keepalive_node.py
exists for Isaac Sim's on-demand publishers).

The per-query topic names are dynamic (one per mission query), so this node
discovers any topic matching `topic_pattern` and holds a silent subscription
open. rayfronts advertises those publishers each cycle once /all is subscribed,
so discovery finds them; subscribing then makes rayfronts stream them and the
DDS router bridges them to the GCS.
"""

import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2


# BEST_EFFORT is compatible with rayfronts' RELIABLE writer (a best-effort
# reader still matches and counts as a subscriber) and avoids buffering the
# large clouds we immediately drop.
KEEPALIVE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class VoxelKeepalive(Node):
    def __init__(self):
        super().__init__('voxel_keepalive')
        domain = os.getenv('ROS_DOMAIN_ID', '1')
        # Per-query voxel heatmaps only (q0_, q1_, ...); NOT voxels_sim/all,
        # which already has a real subscriber.
        # Use [0-9] not \d — a backslash in the launch-file param value gets
        # stripped during XML/param parsing, turning \d+ into a literal "d+".
        default_pattern = (
            rf'^/robot_{domain}/rayfronts/msg_serv/voxels_sim/q[0-9]+')
        self.declare_parameter('topic_pattern', default_pattern)
        self.declare_parameter('discovery_period_s', 5.0)
        self._pattern = re.compile(self.get_parameter('topic_pattern').value)
        self._subs: dict = {}
        self.create_timer(
            float(self.get_parameter('discovery_period_s').value),
            self._discover)
        self._discover()
        self.get_logger().info(
            f'voxel_keepalive watching {self._pattern.pattern}')

    def _discover(self):
        for topic, types in self.get_topic_names_and_types():
            if topic in self._subs:
                continue
            if ('sensor_msgs/msg/PointCloud2' in types
                    and self._pattern.match(topic)):
                self._subs[topic] = self.create_subscription(
                    PointCloud2, topic, lambda _msg: None, KEEPALIVE_QOS)
                self.get_logger().info(f'keepalive subscribed: {topic}')


def main(args=None):
    rclpy.init(args=args)
    node = VoxelKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
