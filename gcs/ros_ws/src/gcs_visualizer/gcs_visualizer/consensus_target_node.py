"""Fleet-consensus confirmed-target AABBs.

Collects every robot's `confirmed_targets` gossip payload (global ENU), dedup-
merges them, votes on the label of overlapping boxes, and publishes one set:
    /gcs/consensus_targets   std_msgs/String  (JSON list)
    /gcs/consensus_AABBs     visualization_msgs/MarkerArray

Params: stale_timeout_s (0.0, 0=never expire), dedup_margin_m (2.0),
cross_label_merge (True), publish_period_s (0.5).
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile

# Match gossip_node GOSSIP_QOS (BEST_EFFORT/VOLATILE) — same as payload_visualizer.
GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

PAYLOAD_NAME = 'confirmed_targets'


class _Cluster:
    """One consensus AABB accumulating contributions from one or more robots."""
    __slots__ = ('lo', 'hi', 'votes', 'contributors', 'visited',
                 'best_conf', 'latest_ts', 'best_center', 'best_size', 'best_yaw')

    def __init__(self, center, size, yaw=0.0):
        h = size / 2.0
        self.lo = center - h
        self.hi = center + h
        self.votes = {}          # label -> summed weight
        self.contributors = set()
        self.visited = False
        self.best_conf = -1.0
        self.latest_ts = 0.0
        self.best_center = np.asarray(center, dtype=float)
        self.best_size = np.asarray(size, dtype=float)
        self.best_yaw = float(yaw)

    @property
    def center(self):
        return (self.lo + self.hi) / 2.0

    @property
    def size(self):
        return self.hi - self.lo

    def surface_gap(self, center, size):
        h = size / 2.0
        gap = np.maximum(np.abs(self.center - center) - (self.size / 2.0 + h), 0.0)
        return float(np.linalg.norm(gap))

    def absorb(self, center, size, label, status, conf, ts, robot, yaw=0.0):
        h = size / 2.0
        self.lo = np.minimum(self.lo, center - h)
        self.hi = np.maximum(self.hi, center + h)
        self.votes[label] = self.votes.get(label, 0.0) + max(float(conf), 0.1)
        self.contributors.add(robot)
        if str(status).lower() == 'visited':
            self.visited = True
        if float(conf) >= self.best_conf:
            self.best_conf = float(conf)
            self.best_center = np.asarray(center, dtype=float)
            self.best_size = np.asarray(size, dtype=float)
            self.best_yaw = float(yaw)
        self.latest_ts = max(self.latest_ts, float(ts))

    def label(self):
        return max(sorted(self.votes), key=lambda k: self.votes[k])


class ConsensusTargetNode(Node):
    def __init__(self):
        super().__init__('consensus_target_node')

        # 0 = never expire (static scenes; robots split the map).
        self.stale_timeout_s = float(
            self.declare_parameter('stale_timeout_s', 0.0).value)
        self.dedup_margin_m = float(
            self.declare_parameter('dedup_margin_m', 2.0).value)
        self.cross_label_merge = bool(
            self.declare_parameter('cross_label_merge', True).value)
        publish_period_s = float(
            self.declare_parameter('publish_period_s', 0.5).value)

        # robot_name -> (targets_list, recv_monotonic_s)
        self._robot_targets = {}

        self.create_subscription(
            PeerProfileMsg, '/gossip/peers', self._on_peer, GOSSIP_QOS)
        self._json_pub = self.create_publisher(
            String, '/gcs/consensus_targets', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/gcs/consensus_AABBs', 10)

        self.create_timer(publish_period_s, self._tick)
        self.get_logger().info(
            'ConsensusTargetNode started '
            f'(stale={self.stale_timeout_s}s margin={self.dedup_margin_m}m '
            f'cross_label={self.cross_label_merge})')

    # ----------------------------------------------------------------- intake
    def _on_peer(self, msg: PeerProfileMsg) -> None:
        try:
            profile = PeerProfile.from_ros_msg(msg)
            payload = profile.get_payload_by_name(PAYLOAD_NAME)
        except Exception:
            return
        if payload is None or not getattr(payload, 'data', ''):
            return
        try:
            items = json.loads(payload.data)
        except (ValueError, TypeError):
            return
        if not isinstance(items, list):
            return
        targets = []
        for it in items:
            try:
                targets.append({
                    'center': np.array([float(it['cx']), float(it['cy']),
                                        float(it['cz'])]),
                    'size': np.array([float(it.get('sx', 1.0)),
                                      float(it.get('sy', 1.0)),
                                      float(it.get('sz', 1.0))]),
                    'label': str(it.get('label', '?')),
                    'status': str(it.get('status', 'observing')),
                    'confidence': float(it.get('confidence', 0.0)),
                    'ts': float(it.get('ts', 0.0)),
                    'yaw': float(it.get('yaw', 0.0)),
                })
            except (KeyError, TypeError, ValueError):
                continue
        now = self.get_clock().now().nanoseconds * 1e-9
        self._robot_targets[msg.robot_name] = (targets, now)

    # --------------------------------------------------------------- consensus
    def _merge(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        clusters = []
        for robot, (targets, recv) in self._robot_targets.items():
            if self.stale_timeout_s > 0.0 and now - recv > self.stale_timeout_s:
                continue  # opt-in only: expire a silent robot's targets
            for t in targets:
                hit = None
                for c in clusters:
                    if (not self.cross_label_merge
                            and t['label'] not in c.votes):
                        continue
                    if c.surface_gap(t['center'], t['size']) <= self.dedup_margin_m:
                        hit = c
                        break
                if hit is None:
                    hit = _Cluster(t['center'], t['size'], t['yaw'])
                    clusters.append(hit)
                hit.absorb(t['center'], t['size'], t['label'], t['status'],
                           t['confidence'], t['ts'], robot, t['yaw'])
        return clusters

    def _tick(self) -> None:
        clusters = self._merge()
        self._publish_json(clusters)
        self._publish_markers(clusters)

    def _publish_json(self, clusters) -> None:
        out = []
        for c in clusters:
            ctr, sz = c.best_center, c.best_size
            out.append({
                'label': c.label(),
                'cx': float(ctr[0]), 'cy': float(ctr[1]), 'cz': float(ctr[2]),
                'sx': float(sz[0]), 'sy': float(sz[1]), 'sz': float(sz[2]),
                'yaw': float(c.best_yaw),
                'status': 'visited' if c.visited else 'observing',
                'confidence': c.best_conf,
                'num_contributors': len(c.contributors),
                'contributors': sorted(c.contributors),
                'votes': c.votes,
            })
        self._json_pub.publish(String(data=json.dumps(out)))

    def _publish_markers(self, clusters) -> None:
        out = MarkerArray()
        now = self.get_clock().now().to_msg()
        # DELETEALL each publish retracts dropped targets (lifetime=0 persists).
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.ns = 'consensus_targets'
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        for j, c in enumerate(clusters):
            ctr, sz, yaw = c.best_center, c.best_size, c.best_yaw
            n = len(c.contributors)
            visited = c.visited
            label = c.label()
            # green=visited, cyan→blue by agreement otherwise.
            if visited:
                rgb = (0.1, 0.85, 0.2)
            else:
                rgb = (0.2, 0.7, 0.95) if n >= 2 else (0.95, 0.55, 0.15)
            box = Marker()
            box.header.frame_id = 'map'
            box.header.stamp = now
            box.ns = 'consensus_targets'
            box.id = j * 2
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(ctr[0])
            box.pose.position.y = float(ctr[1])
            box.pose.position.z = float(ctr[2])
            box.pose.orientation.z = math.sin(yaw / 2.0)
            box.pose.orientation.w = math.cos(yaw / 2.0)
            box.scale.x = float(max(sz[0], 0.1))
            box.scale.y = float(max(sz[1], 0.1))
            box.scale.z = float(max(sz[2], 0.1))
            box.color.r, box.color.g, box.color.b = rgb
            box.color.a = 0.3 + 0.12 * min(n, 3)
            box.lifetime = Duration(sec=0, nanosec=0)
            out.markers.append(box)

            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = now
            txt.ns = 'consensus_targets'
            txt.id = j * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(ctr[0])
            txt.pose.position.y = float(ctr[1])
            txt.pose.position.z = float(ctr[2]) + float(max(sz[2], 0.1)) / 2.0 + 1.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 1.4
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            status = 'visited' if visited else 'observing'
            txt.text = f'{label} [{status}] x{n}'
            txt.lifetime = Duration(sec=0, nanosec=0)
            out.markers.append(txt)
        self._marker_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
