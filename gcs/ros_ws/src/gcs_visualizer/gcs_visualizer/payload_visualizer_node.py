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
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile
from visualization_msgs.msg import Marker

from gcs_visualizer.gcs_utils import (
    transform_marker_array, transform_point_cloud2, point_cloud2_to_cube_marker,
    ROBOT_COLORS,
)

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

# Match the publisher in foxglove_visualizer_node._ground_msl_pub. Latched
# delivery: a late-joining subscriber receives the most recent value on
# subscribe, so startup order between the two visualizer nodes doesn't matter.
GROUND_MSL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
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

        # Source of truth for the ground reference altitude. Computed by
        # foxglove_visualizer_node from msl - odom_z so it isn't biased by a
        # robot that's already in the air at GCS startup.
        self.create_subscription(
            Float64, '/gcs/map_origin/ground_msl',
            self._on_ground_msl, GROUND_MSL_QOS)

        self.get_logger().info('PayloadVisualizerNode started')

    def _on_ground_msl(self, msg: Float64) -> None:
        new_val = float(msg.data)
        if self._alt_ground != new_val:
            self.get_logger().info(
                f'ground_msl received: {new_val:.2f} m '
                f'(display z-offset = {_GOSSIP_ORIGIN_ALT - new_val:.2f} m)')
        self._alt_ground = new_val

    def _pub_for(self, topic, msg_type):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(msg_type, topic, 10)
        return self._pubs[topic]

    def _update_boot(self, robot_name: str, gps_fix) -> None:
        # Per-robot first-seen flag — independent of the ground reference,
        # which now comes from the latched /gcs/map_origin/ground_msl topic.
        if gps_fix.status.status < 0:
            return
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

    def _handle_raw_frontiers(self, robot_name, msg, i, now):
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        marker = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_raw_frontiers',
            marker_id=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            fallback_color=(color[0], color[1], color[2], 1.0),
            scale=0.4,
        )
        if marker is None:
            return
        marker.type = Marker.SPHERE_LIST
        marker.colors = []
        out = MarkerArray()
        out.markers.append(marker)
        self._pub_for(f'/gcs/payload/{robot_name}/raw_frontiers', MarkerArray).publish(out)

    def _handle_navigation_mode(self, robot_name, msg, i, now):
        # Pure passthrough — no spatial transform needed for a String.
        self._pub_for(
            f'/gcs/payload/{robot_name}/navigation_mode', String).publish(msg)

    def _handle_rgb_voxels(self, robot_name, msg, i, now):
        size = 0.5
        cubes = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_voxel_rgb',
            marker_id=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            scale=size,
        )
        out = MarkerArray()
        if cubes is not None:
            out.markers.append(cubes)
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            edges = self._build_cube_edges(
                cubes.points, size,
                ns=f'{robot_name}_voxel_rgb_edges',
                marker_id=i * 100000 + 1,
                stamp=now,
                color=color,
                lifetime=Duration(sec=2, nanosec=0),
            )
            out.markers.append(edges)
        self._pub_for(f'/gcs/payload/{robot_name}/voxel_rgb', MarkerArray).publish(out)

    @staticmethod
    def _build_cube_edges(centers, size, ns, marker_id, stamp, color, lifetime):
        from geometry_msgs.msg import Point as GPoint
        from std_msgs.msg import ColorRGBA
        h = size / 2.0 * 1.03
        corners = [
            (-h, -h, -h), ( h, -h, -h), ( h,  h, -h), (-h,  h, -h),
            (-h, -h,  h), ( h, -h,  h), ( h,  h,  h), (-h,  h,  h),
        ]
        edge_idx = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        m.lifetime = lifetime
        pts = []
        for c in centers:
            cx, cy, cz = c.x, c.y, c.z
            for a, b in edge_idx:
                ax, ay, az = corners[a]
                bx, by, bz = corners[b]
                pts.append(GPoint(x=cx + ax, y=cy + ay, z=cz + az))
                pts.append(GPoint(x=cx + bx, y=cy + by, z=cz + bz))
        m.points = pts
        return m

    PAYLOAD_HANDLERS = {
        'filtered_rays':       ('visualization_msgs/msg/MarkerArray', _handle_filtered_rays),
        'raw_frontiers':       ('sensor_msgs/msg/PointCloud2',        _handle_raw_frontiers),
        'voxel_rgb':           ('sensor_msgs/msg/PointCloud2',        _handle_rgb_voxels),
        'navigation_mode':     ('std_msgs/msg/String',                _handle_navigation_mode),
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
