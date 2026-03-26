"""
payload_visualizer_node.py
==========================
Project-specific GCS visualization node. Reads PeerProfile payloads from
/gossip/peers and publishes each payload to its own topic:
/gcs/payload/{robot_name}/{payload_name}

Publishing per-topic (rather than a single MarkerArray) lets Foxglove expose
its full visualization controls (point size, color mapping, etc.) for each payload.

Payloads are defined in gossip_payloads.yaml on the robot side — no DDS router
changes are needed when adding new payload types.

Coordinate frame note
---------------------
Payloads arrive already transformed into the global ENU 'map' frame by
gossip_node on each robot.  No coordinate transform is needed here — handlers
just set frame_id='map' and republish.

How to add a new payload for visualization
------------------------------------------
1. Read gossip_payloads.yaml and find the `type:` field of the new entry.
2. Add a handler method here (see signature below).
3. If the type is unique (not already in PAYLOAD_HANDLERS), add it to the dict.
   If the type already exists (e.g. a second PointCloud2), dispatch by index in
   _on_peer_profile after the PAYLOAD_HANDLERS loop (order matches yaml order).

Handler signature:
    def _handle_<name>(self, robot_name, msg, i, now):
        # msg  — deserialized ROS message (already in global ENU / 'map' frame)
        # i    — stable robot index (for marker IDs: i * 100000 + unique_offset)
        # now  — current ROS timestamp
        # Publish to self._pub_for('/gcs/payload/{robot_name}/{name}', MsgType)
"""

from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile
from gcs_visualizer.gcs_utils import transform_marker_array, transform_point_cloud2

# ENU origin altitude used by gossip_node when transforming payloads at source.
# Must match frame_utils.DEFAULT_ORIGIN_ALT.
_GOSSIP_ORIGIN_ALT = 90.0


# Seen-set size: same reasoning as gossip_node — covers well beyond the window
# in which DDS Router duplicates arrive.
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

        self._gps_boot      = {}   # robot_name -> True once a valid GPS fix has been seen
        self._last_stamp    = {}   # robot_name -> float: ROS stamp of last accepted message
        self._alt_ground    = None # altitude (m) of first GPS fix — display z datum
        self._payload_cache = {}   # (robot_name, cache_key) -> last deserialized payload msg
        self._pubs          = {}   # topic -> Publisher (created lazily)
        self._seen: OrderedDict = OrderedDict()  # seen-set for duplicate suppression

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

    # ── GPS boot tracking (for robot index, not transforms) ───────────────

    def _update_boot(self, robot_name: str, gps_fix) -> None:
        """Record that a valid GPS fix has been seen for this robot.

        Also tracks alt_ground (first GPS altitude seen across all robots) so
        payload z values can be shifted to match the foxglove_visualizer datum.
        """
        if gps_fix.status.status < 0:
            return
        if self._alt_ground is None:
            self._alt_ground = gps_fix.altitude
        if robot_name not in self._gps_boot:
            self._gps_boot[robot_name] = True

    def _display_z_offset(self) -> float:
        """Z shift to apply to payload positions before display.

        gossip_node transforms payloads with a fixed ENU origin_alt of 90 m.
        foxglove_visualizer uses alt_ground (first GPS fix altitude) as its datum.
        If those two differ, payloads appear above or below the drone.
        This correction aligns them without touching the peer-profile data.
        """
        if self._alt_ground is None:
            return 0.0
        return _GOSSIP_ORIGIN_ALT - self._alt_ground

    # ── Peer profile callback ─────────────────────────────────────────────

    def _on_peer_profile(self, msg: PeerProfileMsg) -> None:
        robot_name = msg.robot_name

        # Seen-set deduplication: drop exact duplicates before any other work.
        # Key is (robot_name, sec, nanosec) — robot_name prevents cross-robot collisions.
        msg_id = (robot_name,
                  msg.gps_fix.header.stamp.sec,
                  msg.gps_fix.header.stamp.nanosec)
        if msg_id in self._seen:
            return
        self._seen[msg_id] = None
        if len(self._seen) > _GOSSIP_SEEN_SIZE:
            self._seen.popitem(last=False)

        # Per-robot ordering: accept only if this message is newer than the last
        # accepted one for THIS robot.  robot_1 and robot_2 are tracked independently
        # so neither can block the other.
        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        if new_t < self._last_stamp.get(robot_name, 0.0):
            return
        self._last_stamp[robot_name] = new_t

        self._update_boot(robot_name, msg.gps_fix)

        if robot_name not in self._gps_boot:
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
                handler(self, robot_name, cached, robot_index, now)

        # voxel_rgb is the 2nd PointCloud2 payload (frontier_viewpoints is 1st)
        pc2_list = [p for p in profile._payloads if p["type"] == "sensor_msgs/msg/PointCloud2"]
        if len(pc2_list) >= 2:
            voxel_rgb = deserialize_message(bytes(pc2_list[1]["data"]), PointCloud2)
            self._payload_cache[(robot_name, 'rgb_voxels')] = voxel_rgb
        cached_voxels = self._payload_cache.get((robot_name, 'rgb_voxels'))
        if cached_voxels is not None:
            self._handle_rgb_voxels(robot_name, cached_voxels, robot_index, now)

    def _robot_index(self, robot_name: str) -> int:
        """Stable integer index for a robot name (alphabetical order)."""
        known = sorted(self._gps_boot.keys())
        return known.index(robot_name) if robot_name in known else 0

    # ── Payload handlers ──────────────────────────────────────────────────
    # Each handler receives:
    #   robot_name – e.g. "robot_1"
    #   msg        – deserialized ROS message (already in global ENU / 'map' frame)
    #   i          – stable robot index (for marker IDs)
    #   now        – current ROS timestamp
    # Publish to self._pub_for('/gcs/payload/{robot_name}/{name}', MsgType)

    def _handle_filtered_rays(self, robot_name, msg, i, now):
        """Republish filtered_rays MarkerArray shifted to the GCS display datum."""
        bz = self._display_z_offset()
        out_ma = transform_marker_array(msg, 0.0, 0.0, bz)
        for k, m in enumerate(out_ma.markers):
            m.header.stamp = now
            m.ns = f'{robot_name}_filtered_rays'
            m.id = i * 100000 + k
            m.lifetime = Duration(sec=2, nanosec=0)
        self._pub_for(f'/gcs/payload/{robot_name}/filtered_rays', MarkerArray).publish(out_ma)

    def _handle_frontier_viewpoints(self, robot_name, msg, i, now):
        """Republish frontier_viewpoints PointCloud2 shifted to the GCS display datum."""
        out = transform_point_cloud2(msg, 0.0, 0.0, self._display_z_offset())
        out.header.stamp = now
        self._pub_for(f'/gcs/payload/{robot_name}/frontier_viewpoints', PointCloud2).publish(out)

    def _handle_rgb_voxels(self, robot_name, msg, i, now):
        """Republish voxel_rgb PointCloud2 shifted to the GCS display datum."""
        out = transform_point_cloud2(msg, 0.0, 0.0, self._display_z_offset())
        out.header.stamp = now
        self._pub_for(f'/gcs/payload/{robot_name}/rgb_voxels', PointCloud2).publish(out)

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
