#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */
#
# Formation visualization node.
#
# Subscribes to each agent's /<ns>/state topic, keeps the latest position in a
# map keyed by agent ID, and publishes a thin LINE_LIST visualization_msgs/Marker
# on /formation_viz showing every configured edge in the formation graph.
#
# Parameters:
#   - num_agents      [int]    Number of agents in the formation (default: 5)
#   - agent_prefix    [string] Namespace prefix (default: "NX"); topics are
#                              /<prefix>01/state, /<prefix>02/state, ...
#   - map_frame_id    [string] Marker frame_id (default: "map")
#   - line_width      [double] LINE_LIST scale.x in meters (default: 0.03)
#   - rate_hz         [double] Publish rate (default: 10.0)
#   - edge_pairs      [int[]]  Flattened list of edges [i,j,i,j,...] as 1-based
#                              agent IDs. Empty = full graph (every pair).
#   - color_rgba      [double[4]] RGBA marker color (default: [1, 0.8, 0, 0.9])

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from dynus_interfaces.msg import State
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class FormationVizNode(Node):
    def __init__(self):
        super().__init__('formation_viz_node')

        # --- parameters ---
        self.declare_parameter('num_agents', 5)
        self.declare_parameter('agent_prefix', 'NX')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('line_width', 0.03)
        # 30 Hz is smooth enough for human perception and light on rclpy's
        # single-threaded executor. Raising higher (e.g. 60) adds little
        # visual benefit and stresses the RViz render loop.
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('edge_pairs', [])
        self.declare_parameter('color_rgba', [1.0, 0.85, 0.0, 0.9])

        self.num_agents = int(self.get_parameter('num_agents').value)
        self.prefix = str(self.get_parameter('agent_prefix').value)
        self.map_frame = str(self.get_parameter('map_frame_id').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        rgba = list(self.get_parameter('color_rgba').value)
        if len(rgba) != 4:
            rgba = [1.0, 0.85, 0.0, 0.9]
        self.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]),
                               b=float(rgba[2]), a=float(rgba[3]))

        # Edge list: 1-based agent ID pairs. Empty → full graph.
        raw_edges = list(self.get_parameter('edge_pairs').value)
        if raw_edges:
            if len(raw_edges) % 2 != 0:
                self.get_logger().error(
                    f'edge_pairs must have even length, got {len(raw_edges)}; '
                    'falling back to full graph')
                raw_edges = []
        if raw_edges:
            self.edges = [(int(raw_edges[2 * k]), int(raw_edges[2 * k + 1]))
                          for k in range(len(raw_edges) // 2)]
        else:
            self.edges = [(i, j)
                          for i in range(1, self.num_agents + 1)
                          for j in range(i + 1, self.num_agents + 1)]

        self.get_logger().info(
            f'FormationVizNode: num_agents={self.num_agents} prefix={self.prefix} '
            f'edges={self.edges}')

        # --- state storage ---
        # latest (x, y, z) per agent ID (1-based). None until first msg received.
        self.positions = {aid: None for aid in range(1, self.num_agents + 1)}

        # --- QoS: pure-viz consumer wants the freshest sample, not history ---
        # depth=10 + RELIABLE made the lines laggy: when rclpy's single-threaded
        # executor fell behind, state messages piled up in the queue and the
        # callbacks drained them in FIFO order, so self.positions reflected
        # stale poses by the time the timer fired. depth=1 + BEST_EFFORT tells
        # rmw "always give me the latest state, drop anything older", which is
        # exactly what a visualization needs.
        state_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subs = []
        for aid in range(1, self.num_agents + 1):
            topic = f'/{self.prefix}{aid:02d}/state'
            self.subs.append(self.create_subscription(
                State, topic, self._make_state_cb(aid), state_qos))

        # --- publisher + timer ---
        self.marker_pub = self.create_publisher(Marker, '/formation_viz', 10)
        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self._publish_marker)

    def _make_state_cb(self, aid: int):
        def _cb(msg: State):
            self.positions[aid] = (msg.pos.x, msg.pos.y, msg.pos.z)
        return _cb

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'formation_edges'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color = self.color
        marker.lifetime.sec = 0
        # 80 ms — at 30 Hz publish, the previous frame is overwritten well
        # before this expires, so RViz never shows a stale edge for long if
        # this node (or an agent) drops out.
        marker.lifetime.nanosec = int(8e7)

        # LINE_LIST: each pair of consecutive points is one line segment.
        for i, j in self.edges:
            pi = self.positions.get(i)
            pj = self.positions.get(j)
            if pi is None or pj is None:
                continue
            marker.points.append(Point(x=pi[0], y=pi[1], z=pi[2]))
            marker.points.append(Point(x=pj[0], y=pj[1], z=pj[2]))

        # Publishing an empty LINE_LIST is a no-op in RViz, so the first few
        # frames before any /NX##/state message arrives are harmless.
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = FormationVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
