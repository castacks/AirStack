"""
payload_visualizer_node.py
==========================
Project-specific GCS visualization node. Reads PeerProfile payloads from
/gossip/peers, applies coordinate frame translation, and publishes each payload
to its own topic: /gcs/payload/{robot_name}/{payload_name}

Publishing per-topic (rather than a single MarkerArray) lets Foxglove expose
its full visualization controls (point size, color mapping, etc.) for each payload.

Payloads are defined in gossip_payloads.yaml on the robot side — no DDS router
changes are needed when adding new payload types.

How to add a new payload for visualization
------------------------------------------
1. Read gossip_payloads.yaml and find the `type:` field of the new entry.
2. Add a handler method here (see signature below).
3. If the type is unique (not already in PAYLOAD_HANDLERS), add it to the dict.
   If the type already exists (e.g. a second PointCloud2), dispatch by index in
   _on_peer_profile after the PAYLOAD_HANDLERS loop (order matches yaml order).

Handler signature:
    def _handle_<name>(self, robot_name, msg, boot, i, now):
        # msg  — deserialized ROS message
        # boot — (bx, by, bz) ENU offset: add to all positions to go odom→map frame
        # i    — stable robot index (for marker IDs: i * 100000 + unique_offset)
        # now  — current ROS timestamp
        # Publish to self._pub_for('/gcs/payload/{robot_name}/{name}', MsgType)

Coordinate frame note
---------------------
Payload data is in each robot's local odometry frame. Apply the boot offset to
translate into the global 'map' frame before publishing.

Use transform_marker_array() for MarkerArray payloads.
Use transform_point_cloud2() for PointCloud2 payloads (preserves all fields + rgb).
Both are in gcs_utils.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile

from gcs_visualizer.gcs_utils import gps_to_enu, transform_marker_array, transform_point_cloud2

GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class PayloadVisualizerNode(Node):
    def __init__(self):
        super().__init__('payload_visualizer_node')

        self._gps_boot      = {}   # robot_name -> (bx, by, bz) ENU at first GPS fix
        self._alt_ground    = None
        self._payload_cache = {}   # (robot_name, cache_key) -> last deserialized payload msg
        self._pubs          = {}   # topic -> Publisher (created lazily)

        # Subscribe to /gossip/peers — PeerProfiles arrive here from all robots
        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)

        self.get_logger().info('PayloadVisualizerNode started')

    # ── Lazy publisher helper ──────────────────────────────────────────────

    def _pub_for(self, topic, msg_type):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(msg_type, topic, 10)
        return self._pubs[topic]

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
        robot_index = self._robot_index(robot_name)

        for type_str, handler in self.PAYLOAD_HANDLERS.items():
            payload = profile.get_payload(type_str)
            if payload is not None:
                self._payload_cache[(robot_name, type_str)] = payload
            cached = self._payload_cache.get((robot_name, type_str))
            if cached is not None:
                handler(self, robot_name, cached, boot, robot_index, now)

        # voxel_rgb is the 2nd PointCloud2 payload (frontier_viewpoints is 1st)
        pc2_list = [p for p in profile._payloads if p["type"] == "sensor_msgs/msg/PointCloud2"]
        if len(pc2_list) >= 2:
            voxel_rgb = deserialize_message(bytes(pc2_list[1]["data"]), PointCloud2)
            self._payload_cache[(robot_name, 'rgb_voxels')] = voxel_rgb
        cached_voxels = self._payload_cache.get((robot_name, 'rgb_voxels'))
        if cached_voxels is not None:
            self._handle_rgb_voxels(robot_name, cached_voxels, boot, robot_index, now)

    def _robot_index(self, robot_name: str) -> int:
        """Stable integer index for a robot name (alphabetical order)."""
        known = sorted(self._gps_boot.keys())
        return known.index(robot_name) if robot_name in known else 0

    # ── Payload handlers ──────────────────────────────────────────────────
    # Each handler receives:
    #   robot_name – e.g. "robot_1"
    #   msg        – deserialized ROS message
    #   boot       – (bx, by, bz) ENU translation for this robot's odom frame
    #   i          – stable robot index (for marker IDs)
    #   now        – current ROS timestamp
    # Publish to self._pub_for('/gcs/payload/{robot_name}/{name}', MsgType)

    def _handle_filtered_rays(self, robot_name, msg, boot, i, now):
        """Publish filtered_rays MarkerArray translated into the map frame."""
        bx, by, bz = boot
        out = MarkerArray()
        out.markers.extend(transform_marker_array(
            msg, bx, by, bz,
            ns=f'{robot_name}_filtered_rays',
            id_base=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0)))
        self._pub_for(f'/gcs/payload/{robot_name}/filtered_rays', MarkerArray).publish(out)

    def _handle_frontier_viewpoints(self, robot_name, msg, boot, i, now):
        """Publish frontier_viewpoints PointCloud2 translated into the map frame."""
        bx, by, bz = boot
        transformed = transform_point_cloud2(msg, bx, by, bz)
        transformed.header.stamp = now
        self._pub_for(f'/gcs/payload/{robot_name}/frontier_viewpoints', PointCloud2).publish(transformed)

    def _handle_rgb_voxels(self, robot_name, msg, boot, i, now):
        """Publish voxel_rgb PointCloud2 translated into the map frame."""
        bx, by, bz = boot
        transformed = transform_point_cloud2(msg, bx, by, bz)
        transformed.header.stamp = now
        self._pub_for(f'/gcs/payload/{robot_name}/rgb_voxels', PointCloud2).publish(transformed)

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
