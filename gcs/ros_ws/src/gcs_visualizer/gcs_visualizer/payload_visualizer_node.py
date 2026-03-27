"""
payload_visualizer_node.py — GCS visualization for gossip payloads.

Reads PeerProfile payloads from /gossip/peers and republishes each to its own
topic (/gcs/payload/{robot_name}/{payload_name}) so Foxglove exposes full
visualization controls per payload independently.

Payloads arrive already transformed into global ENU by gossip_node — handlers
only need to apply the display z-offset and republish.

To add a new payload: add a handler method and register it in PAYLOAD_HANDLERS
keyed by the payload name (last segment of the topic in gossip_payloads.yaml).

Handler signature:
    def _handle_<name>(self, robot_name, msg, i, now):
        # i — stable robot index (for marker IDs: i * 100000 + offset)
"""

from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile
from gcs_visualizer.gcs_utils import transform_marker_array, transform_point_cloud2, point_cloud2_to_cube_marker

# Must match frame_utils.DEFAULT_ORIGIN_ALT — used to compute the z-offset between
# gossip_node's fixed ENU origin and the GCS display datum (first GPS altitude seen).
_GOSSIP_ORIGIN_ALT = 90.0

_GOSSIP_SEEN_SIZE = 50

GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class PayloadVisualizerNode(Node):
    def __init__(self):
        super().__init__('payload_visualizer_node')

        self._gps_boot      = {}
        self._last_stamp    = {}
        self._alt_ground    = None
        self._payload_cache = {}
        self._pubs          = {}
        self._seen: OrderedDict = OrderedDict()

        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)

        self.get_logger().info('PayloadVisualizerNode started')

    def _pub_for(self, topic, msg_type):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(msg_type, topic, 10)
        return self._pubs[topic]

    def _update_boot(self, robot_name: str, gps_fix) -> None:
        if gps_fix.status.status < 0:
            return
        if self._alt_ground is None:
            self._alt_ground = gps_fix.altitude
        if robot_name not in self._gps_boot:
            self._gps_boot[robot_name] = True

    def _display_z_offset(self) -> float:
        """Correct for the altitude datum difference between gossip_node and foxglove_visualizer."""
        if self._alt_ground is None:
            return 0.0
        return _GOSSIP_ORIGIN_ALT - self._alt_ground

    def _on_peer_profile(self, msg: PeerProfileMsg) -> None:
        robot_name = msg.robot_name

        msg_id = (robot_name,
                  msg.gps_fix.header.stamp.sec,
                  msg.gps_fix.header.stamp.nanosec)
        if msg_id in self._seen:
            return
        self._seen[msg_id] = None
        if len(self._seen) > _GOSSIP_SEEN_SIZE:
            self._seen.popitem(last=False)

        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        if new_t < self._last_stamp.get(robot_name, 0.0):
            return
        self._last_stamp[robot_name] = new_t

        self._update_boot(robot_name, msg.gps_fix)

        if robot_name not in self._gps_boot:
            return

        profile = PeerProfile.from_ros_msg(msg)
        now = self.get_clock().now().to_msg()
        robot_index = self._robot_index(robot_name)

        for name, (type_str, handler) in self.PAYLOAD_HANDLERS.items():
            payload = profile.get_payload_by_name(name)
            if payload is not None:
                self._payload_cache[(robot_name, name)] = payload
            cached = self._payload_cache.get((robot_name, name))
            if cached is not None:
                handler(self, robot_name, cached, robot_index, now)

    def _robot_index(self, robot_name: str) -> int:
        """Stable integer index for a robot name (alphabetical order)."""
        known = sorted(self._gps_boot.keys())
        return known.index(robot_name) if robot_name in known else 0

    def _handle_filtered_rays(self, robot_name, msg, i, now):
        bz = self._display_z_offset()
        out_ma = transform_marker_array(msg, 0.0, 0.0, bz)
        for k, m in enumerate(out_ma.markers):
            m.header.stamp = now
            m.ns = f'{robot_name}_filtered_rays'
            m.id = i * 100000 + k
            m.lifetime = Duration(sec=2, nanosec=0)
        self._pub_for(f'/gcs/payload/{robot_name}/filtered_rays', MarkerArray).publish(out_ma)

    def _handle_frontier_viewpoints(self, robot_name, msg, i, now):
        out = transform_point_cloud2(msg, 0.0, 0.0, self._display_z_offset())
        out.header.stamp = now
        self._pub_for(f'/gcs/payload/{robot_name}/frontier_viewpoints', PointCloud2).publish(out)

    def _handle_rgb_voxels(self, robot_name, msg, i, now):
        marker = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_voxel_rgb',
            marker_id=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            scale=0.5,
        )
        if marker is not None:
            out = MarkerArray()
            out.markers.append(marker)
            self._pub_for(f'/gcs/payload/{robot_name}/voxel_rgb', MarkerArray).publish(out)

    PAYLOAD_HANDLERS = {
        'filtered_rays':       ('visualization_msgs/msg/MarkerArray', _handle_filtered_rays),
        'frontier_viewpoints': ('sensor_msgs/msg/PointCloud2',        _handle_frontier_viewpoints),
        'voxel_rgb':           ('sensor_msgs/msg/PointCloud2',        _handle_rgb_voxels),
    }


def main(args=None):
    rclpy.init(args=args)
    node = PayloadVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
