"""
payload_visualizer_node.py
==========================
Project-specific GCS visualization node. Reads PeerProfile payloads from
/gossip/peers, applies coordinate frame translation, and publishes for Foxglove.

Payloads are defined in gossip_payloads.yaml on the robot side — no DDS router
changes are needed when adding new payload types.

How to adapt for your project
------------------------------
1. On the robot: add entries to gossip_payloads.yaml (topic + ROS type).
2. Here: add a handler method and register it in PAYLOAD_HANDLERS.

Handler signature:
    def _handle_<name>(self, robot_name, msg, boot, i, now, array):
        # msg   — deserialized ROS message
        # boot  — (bx, by, bz) ENU translation from robot odom frame to global map
        # i     — stable robot index (use for marker IDs to avoid collisions)
        # now   — current ROS timestamp
        # array — MarkerArray to append Marker messages to
        # For non-Marker outputs (e.g. PointCloud2), publish a separate topic directly.

Coordinate frame note
---------------------
Payload data is in each robot's local odometry frame (origin = robot's first
GPS fix in ENU). Apply the boot offset to translate into the global 'map' frame.

Use transform_marker_array() for MarkerArray payloads.
Use transform_point_cloud2() for PointCloud2 payloads.
Both helpers are in gcs_utils.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import struct

from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile

from gcs_visualizer.gcs_utils import gps_to_enu, transform_marker_array

GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

GPS_SUFFIX = '/interface/mavros/global_position/global'


class PayloadVisualizerNode(Node):
    def __init__(self):
        super().__init__('payload_visualizer_node')

        self._gps_boot      = {}   # robot_name -> (bx, by, bz) ENU at first GPS fix
        self._alt_ground    = None
        self._payload_cache = {}   # (robot_name, type_str) -> last deserialized payload msg

        self._markers_pub = self.create_publisher(
            MarkerArray, '/gcs/payload_markers', 10)

        # Subscribe to /gossip/peers — PeerProfiles arrive here from all robots
        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)

        self.get_logger().info('PayloadVisualizerNode started')

    # ── Colour palette for per-robot point clouds ─────────────────────────────

    _ROBOT_COLORS = [
        (0.0, 1.0, 1.0, 1.0),   # cyan
        (1.0, 0.5, 0.0, 1.0),   # orange
        (0.5, 1.0, 0.0, 1.0),   # lime
        (1.0, 0.0, 0.5, 1.0),   # pink
        (0.5, 0.0, 1.0, 1.0),   # purple
    ]

    # ── Boot offset tracking ───────────────────────────────────────────────

    def _update_boot_from_gps(self, robot_name: str, gps_fix) -> None:
        """Track the first GPS fix seen per robot as the boot (odom origin) offset."""
        if gps_fix.status.status < 0:
            return
        if self._alt_ground is None:
            self._alt_ground = gps_fix.altitude
        if robot_name not in self._gps_boot:
            pos = gps_to_enu(
                gps_fix.latitude, gps_fix.longitude,
                gps_fix.altitude, self._alt_ground)
            self._gps_boot[robot_name] = pos

    # ── Peer profile callback ─────────────────────────────────────────────

    def _on_peer_profile(self, msg: PeerProfileMsg) -> None:
        robot_name = msg.robot_name
        self._update_boot_from_gps(robot_name, msg.gps_fix)

        boot = self._gps_boot.get(robot_name)
        if boot is None:
            return  # no GPS fix yet, skip

        profile = PeerProfile.from_ros_msg(msg)
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        robot_index = self._robot_index(robot_name)

        for type_str, handler in self.PAYLOAD_HANDLERS.items():
            payload = profile.get_payload(type_str)
            if payload is not None:
                self._payload_cache[(robot_name, type_str)] = payload
            cached = self._payload_cache.get((robot_name, type_str))
            if cached is not None:
                handler(self, robot_name, cached, boot, robot_index, now, array)

        if array.markers:
            self._markers_pub.publish(array)

    def _robot_index(self, robot_name: str) -> int:
        """Stable integer index for a robot name (alphabetical order)."""
        known = sorted(self._gps_boot.keys())
        return known.index(robot_name) if robot_name in known else 0

    # ── Payload handlers ──────────────────────────────────────────────────
    # Each handler receives:
    #   robot_name  – e.g. "robot_1"
    #   msg         – deserialized ROS message
    #   boot        – (bx, by, bz) ENU translation for this robot's odom frame
    #   i           – stable robot index (for marker IDs)
    #   now         – current ROS timestamp
    #   array       – MarkerArray to append Marker messages to
    #
    # For non-marker outputs (e.g. PointCloud2), publish directly from here.

    def _handle_filtered_rays(self, robot_name, msg, boot, i, now, array):
        """Translate filtered_rays MarkerArray from odom frame to ENU map frame."""
        bx, by, bz = boot
        lifetime = Duration(sec=2, nanosec=0)
        markers = transform_marker_array(
            msg, bx, by, bz,
            ns=f'{robot_name}_filtered_rays',
            id_base=i * 100000,
            stamp=now, lifetime=lifetime)
        array.markers.extend(markers)

    def _handle_frontier_viewpoints(self, robot_name, msg, boot, i, now, array):
        """Convert frontier_viewpoints PointCloud2 to a POINTS Marker in the map frame."""
        bx, by, bz = boot
        r, g, b, a = self._ROBOT_COLORS[i % len(self._ROBOT_COLORS)]

        # Extract xyz from the PointCloud2 and apply boot offset
        field_offsets = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z')}
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            return
        ox, oy, oz = field_offsets['x'], field_offsets['y'], field_offsets['z']
        ps = msg.point_step
        n_points = msg.width * msg.height
        data = bytes(msg.data)

        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = f'{robot_name}_frontier_viewpoints'
        m.id = i * 100000 + 90000
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.lifetime = Duration(sec=2, nanosec=0)
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)

        for idx in range(n_points):
            base = idx * ps
            x, = struct.unpack_from('<f', data, base + ox)
            y, = struct.unpack_from('<f', data, base + oy)
            z, = struct.unpack_from('<f', data, base + oz)
            m.points.append(Point(x=x + bx, y=y + by, z=z + bz))

        array.markers.append(m)

    # ── Handler registry ──────────────────────────────────────────────────
    # Maps ROS type string -> handler method.
    # Add or remove entries here to change what gets visualized.
    PAYLOAD_HANDLERS = {
        'visualization_msgs/msg/MarkerArray': _handle_filtered_rays,
        'sensor_msgs/msg/PointCloud2':        _handle_frontier_viewpoints,
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
