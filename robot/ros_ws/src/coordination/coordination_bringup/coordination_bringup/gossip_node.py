"""
gossip_node.py
==============
ROS 2 node that implements the gossip-protocol coordination layer.

Each robot:
  - Subscribes to its own GPS fix (NavSatFix) and compass heading, and global plan
  - Publishes its own PeerProfile to /gossip/peers (shared gossip domain via DDS Router)
  - Subscribes to /gossip/peers to receive all peer profiles
  - Maintains a registry of known peers (latest-wins, no expiry)
  - Republishes the full registry as a latched topic for planners to consume

Publish rate:
  - Fixed 1 Hz wall-clock timer (publish_rate parameter) — always fires regardless of
    sim time, so heartbeating works even when the robot is stationary.
  - A new waypoint also triggers an immediate extra publish.
"""

import os
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import rosidl_runtime_py.utilities as rosidl_utils

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from coordination_msgs.msg import PeerProfile as PeerProfileMsg

from coordination_bringup.peer_profile import PeerProfile
from coordination_bringup.frame_utils import (
    gps_to_enu, heading_to_quat, transform_marker_array, transform_point_cloud2,
)

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2


# QoS for gossip topic: best-effort, keep last 1 (low-overhead, high-freq)
GOSSIP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# MAVROS and other sensor topics publish as BEST_EFFORT
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# QoS for the registry topic: reliable + transient-local so late subscribers
# immediately receive the latest snapshot
REGISTRY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class GossipNode(Node):

    def __init__(self):
        super().__init__("gossip_node")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("robot_name", os.environ.get("ROBOT_NAME", "unknown_robot"))
        self.declare_parameter("publish_rate", 1.0)   # Hz — wall-clock timer
        self.declare_parameter("payload_topics_config", "")  # path to gossip_payloads.yaml

        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        payload_config_path = self.get_parameter("payload_topics_config").get_parameter_value().string_value

        # ── State ────────────────────────────────────────────────────────
        self._profile = PeerProfile(robot_name=self._robot_name)

        # Boot pose: set on first valid GPS fix + first heading reading.
        # Used to transform payload data from local odom frame → global ENU
        # before attaching to PeerProfile, so every receiver gets world-frame data.
        self._boot_pos: tuple | None = None   # (bx, by, bz) ENU metres
        self._boot_quat: tuple | None = None  # (qx, qy, qz, qw) rotation

        # Registry: robot_name → latest accepted PeerProfile (monotonic by stamp)
        self._registry: dict[str, PeerProfileMsg] = {}
        self._registry_lock = threading.Lock()

        # Per-robot inbox: buffers incoming peer messages so every robot gets
        # processed fairly on each drain tick, regardless of DDS arrival order.
        # Stores only the latest message per robot (by gossip stamp).
        self._peer_inbox: dict[str, PeerProfileMsg] = {}
        self._peer_inbox_lock = threading.Lock()

        # Payload cache: topic → (msg, stamp) or None until first message arrives
        self._payload_cache: dict[str, object] = {}
        self._payload_subs: list = []

        # ── Payload topic subscriptions (config-driven) ──────────────────
        if payload_config_path:
            self._setup_payload_subscriptions(payload_config_path)

        # ── Subscriptions ────────────────────────────────────────────────
        self._navsat_sub = self.create_subscription(
            NavSatFix,
            f"/{self._robot_name}/interface/mavros/global_position/raw/fix",
            self._on_navsat,
            SENSOR_QOS,
        )
        self._compass_sub = self.create_subscription(
            Float64,
            f"/{self._robot_name}/interface/mavros/global_position/compass_hdg",
            self._on_compass,
            SENSOR_QOS,
        )
        self._path_sub = self.create_subscription(
            Path,
            f"/{self._robot_name}/global_plan",
            self._on_global_plan,
            SENSOR_QOS,
        )
        self._peer_sub = self.create_subscription(
            PeerProfileMsg,
            "/gossip/peers",
            self._on_peer_msg,
            GOSSIP_QOS,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._gossip_pub = self.create_publisher(
            PeerProfileMsg,
            "/gossip/peers",
            GOSSIP_QOS,
        )
        # Registry snapshot – latched so planners that start late still get it
        self._registry_pub = self.create_publisher(
            PeerProfileMsg,
            f"/{self._robot_name}/coordination/peer_registry",
            REGISTRY_QOS,
        )

        # ── Publish timer (wall clock — unaffected by sim time) ──────────
        # Use a wall-clock timer so heartbeating works even when sim is paused
        # or use_sim_time is set.
        period = 1.0 / max(publish_rate, 0.01)
        self._publish_timer = self.create_timer(
            period,
            self._publish_tick,
            clock=ROSClock(),
        )

        # ── Peer inbox drain timer (5 Hz wall clock) ──────────────────────
        # Drains the per-robot inbox at a fixed rate so all robots are
        # processed fairly — robot N never blocks robot M from being updated.
        self._drain_timer = self.create_timer(
            0.2,
            self._drain_peer_inbox,
            clock=ROSClock(),
        )

        self.get_logger().info(
            f"GossipNode started for '{self._robot_name}' "
            f"(publish_rate={publish_rate:.1f} Hz wall-clock, "
            f"{len(self._payload_cache)} payload topic(s))"
        )

    # ------------------------------------------------------------------ #
    # Config-driven payload subscriptions                                  #
    # ------------------------------------------------------------------ #

    def _setup_payload_subscriptions(self, config_path: str) -> None:
        """
        Parse gossip_payloads.yaml and create a subscription for each entry.

        Each subscription caches the latest message in self._payload_cache.
        Messages are only added to the profile at publish time, so a topic
        that hasn't published yet simply contributes no payload.
        """
        try:
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f"Could not load payload config '{config_path}': {e}")
            return

        for entry in cfg.get("payload_topics", []):
            topic_template = entry.get("topic", "")
            type_str = entry.get("type", "")
            if not topic_template or not type_str:
                continue

            topic = topic_template.replace("{robot_name}", self._robot_name)

            try:
                msg_class = rosidl_utils.get_message(type_str)
            except Exception as e:
                self.get_logger().warn(f"Unknown payload type '{type_str}': {e}")
                continue

            # Cache stores (msg, stamp) tuples. stamp is extracted from the
            # message header if present, otherwise the current ROS clock time.
            self._payload_cache[topic] = None  # None = not received yet

            def _make_callback(t):
                def cb(msg):
                    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
                    if stamp is None:
                        stamp = self.get_clock().now().to_msg()
                    self._payload_cache[t] = (msg, stamp)
                return cb

            sub = self.create_subscription(msg_class, topic, _make_callback(topic), SENSOR_QOS)
            self._payload_subs.append(sub)
            self.get_logger().info(f"Payload subscription: {topic} ({type_str})")

    # ------------------------------------------------------------------ #
    # Subscription callbacks                                               #
    # ------------------------------------------------------------------ #

    def _on_navsat(self, msg: NavSatFix) -> None:
        # Only store valid fixes — ignore NO_FIX so GPS never zeros out.
        if msg.status.status < 0:
            return
        self._profile.set_gps_from_navsat(msg)
        # Record boot position on first valid GPS fix
        if self._boot_pos is None:
            self._boot_pos = gps_to_enu(msg.latitude, msg.longitude, msg.altitude)
            # Only set boot_quat now if heading has already been received
            # (profile.heading != 0.0 means a compass reading arrived first).
            # If heading hasn't arrived yet, _on_compass will set boot_quat
            # once the first reading comes in.
            if self._boot_quat is None and self._profile.heading != 0.0:
                self._boot_quat = heading_to_quat(self._profile.heading)

    def _on_compass(self, msg: Float64) -> None:
        self._profile.set_heading(msg.data)
        # Set boot quat on first heading reading once we have GPS
        if self._boot_pos is not None and self._boot_quat is None:
            self._boot_quat = heading_to_quat(msg.data)

    def _on_global_plan(self, msg: Path) -> None:
        self._profile.set_waypoint_from_path(msg)

    def _on_peer_msg(self, msg: PeerProfileMsg) -> None:
        if msg.robot_name == self._robot_name:
            return  # discard own messages echoed back from the gossip domain
        # Stage in per-robot inbox: keep only the newest message per robot so
        # all robots are drained together on the next tick (fair processing).
        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        with self._peer_inbox_lock:
            existing = self._peer_inbox.get(msg.robot_name)
            if existing is not None:
                old_t = (existing.gps_fix.header.stamp.sec
                         + existing.gps_fix.header.stamp.nanosec * 1e-9)
                if new_t < old_t:
                    return  # already buffered a newer message this tick
            self._peer_inbox[msg.robot_name] = msg

    def _drain_peer_inbox(self) -> None:
        """Process all buffered peer messages at 5 Hz, one per robot per tick."""
        with self._peer_inbox_lock:
            inbox = dict(self._peer_inbox)
            self._peer_inbox.clear()
        for msg in inbox.values():
            self._update_registry(msg)

    # ------------------------------------------------------------------ #
    # Publish logic                                                        #
    # ------------------------------------------------------------------ #

    def _publish_tick(self) -> None:
        """Wall-clock timer callback — publish own profile at fixed rate."""
        self._publish_own()

    def _publish_own(self) -> None:
        # Rebuild payloads from cache on every tick.
        # If the boot pose is known, transform each payload into global ENU so
        # every receiver (peers and GCS) gets world-frame data directly.
        # If boot pose is not yet known, skip payloads this tick — we cannot
        # produce a meaningful global-frame transform without GPS.
        self._profile.clear_payloads()
        if self._boot_pos is not None and self._boot_quat is not None:
            bx, by, bz = self._boot_pos
            q = self._boot_quat
            for topic, entry in self._payload_cache.items():
                if entry is not None:
                    msg, stamp = entry
                    transformed = self._transform_to_global(msg, bx, by, bz, q)
                    self._profile.add_payload(transformed, stamp=stamp)

        # Stamp the outgoing profile with the current ROS clock time.
        # Receivers use this stamp — not the MAVROS GPS stamp — to enforce
        # monotonic ordering and discard out-of-order messages.
        self._profile.gps_fix.header.stamp = self.get_clock().now().to_msg()

        self._gossip_pub.publish(self._profile.to_ros_msg())

    def _transform_to_global(self, msg, bx, by, bz, q):
        """Transform a payload message from local odom frame to global ENU."""
        if isinstance(msg, MarkerArray):
            return transform_marker_array(msg, bx, by, bz, q)
        if isinstance(msg, PointCloud2):
            return transform_point_cloud2(msg, bx, by, bz, q)
        return msg  # unknown type — pass through untransformed

    # ------------------------------------------------------------------ #
    # Registry management                                                  #
    # ------------------------------------------------------------------ #

    def _update_registry(self, msg: PeerProfileMsg) -> None:
        """Update registry, then republish snapshot.

        Accept only if the incoming message is newer (or equal) to what we
        already have.  gossip_node stamps gps_fix.header with the ROS clock
        at publish time, so this gives strict monotonic ordering — stale or
        out-of-order messages are silently dropped.
        """
        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)

        with self._registry_lock:
            existing = self._registry.get(msg.robot_name)
            if existing is not None:
                old_t = (existing.gps_fix.header.stamp.sec
                         + existing.gps_fix.header.stamp.nanosec * 1e-9)
                if new_t < old_t:
                    return  # out-of-order — discard
            self._registry[msg.robot_name] = msg

        self._registry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GossipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
