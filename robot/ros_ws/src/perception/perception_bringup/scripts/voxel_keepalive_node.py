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
        # Per-query voxel heatmaps (q0_, q1_, ...) PLUS the two whole-map
        # clouds the GCS draws.
        #
        # `voxels_sim/all` is included even though raven/semantic_search_task
        # normally subscribes to it: raven only holds that subscription while a
        # SemanticSearchTask goal is active, so between goals the cloud stops
        # and the operator's Foxglove panel goes blank for reasons that have
        # nothing to do with the map. Holding it here makes the layer
        # continuous. The extra cost is nil when raven is already subscribed —
        # the publish happens once regardless of subscriber count.
        #
        # `voxel_rgb` (the coloured map) and `voxel_occ` have NO subscriber at
        # all in this stack, so without this they never stream and the GCS
        # bridge carries nothing. These are the layers you actually look at to
        # answer "did the map even cover where the casualty is".
        #
        # Use [0-9] not \d — a backslash in the launch-file param value gets
        # stripped during XML/param parsing, turning \d+ into a literal "d+".
        # The AGGREGATE clouds only — deliberately NOT the per-query
        # q{k}_{label} topics.
        #
        # Subscribing to a topic here is what MAKES rayfronts serialise and
        # send it, so this list is a cost, not just a filter: holding all 34
        # per-query clouds open made the shared server publish 34 point clouds
        # per query cycle for one robot, which is the same load that took the
        # DDS router from 130 MB to 870 MB RSS in the incident documented in
        # dds_router.yaml. voxels_sim/all already carries every query as a
        # separate `sim_K` field on ONE cloud, so the per-query topics are pure
        # duplication — colour by sim_K instead. sim_0 is the target.
        #
        # The end-anchor keeps voxel_occ from dragging in voxel_occ_tiles,
        # which is a separate and much larger layer.
        default_pattern = (
            rf'^/robot_{domain}/rayfronts/'
            rf'(msg_serv/(voxels_sim|rays_sim)/all|voxel_rgb|voxel_occ)$')
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
