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

        # Registry: robot_name → latest PeerProfile ROS msg
        self._registry: dict[str, PeerProfileMsg] = {}

        # Payload cache: topic → latest message (None until first message arrives)
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
            f"/{self._robot_name}/random_walk/global_plan",
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
        self._profile.set_gps_from_navsat(msg)

    def _on_compass(self, msg: Float64) -> None:
        self._profile.set_heading(msg.data)

    def _on_global_plan(self, msg: Path) -> None:
        self._profile.set_waypoint_from_path(msg)

    def _on_peer_msg(self, msg: PeerProfileMsg) -> None:
        if msg.robot_name == self._robot_name:
            return  # discard own messages echoed back from the gossip domain
        self._update_registry(msg)

    # ------------------------------------------------------------------ #
    # Publish logic                                                        #
    # ------------------------------------------------------------------ #

    def _publish_tick(self) -> None:
        """Wall-clock timer callback — publish own profile at fixed rate."""
        self._publish_own()

    def _publish_own(self) -> None:
        # Rebuild payloads from cache on every tick. The cache retains the last
        # received message per topic indefinitely, so peers always get the most
        # recent known data even if the source publishes slower than gossip rate.
        self._profile.clear_payloads()
        for topic, entry in self._payload_cache.items():
            if entry is not None:
                msg, stamp = entry
                self._profile.add_payload(msg, stamp=stamp)
        self._gossip_pub.publish(self._profile.to_ros_msg())

    # ------------------------------------------------------------------ #
    # Registry management                                                  #
    # ------------------------------------------------------------------ #

    def _update_registry(self, msg: PeerProfileMsg) -> None:
        """Update registry with latest-wins semantics, then republish snapshot."""
        existing = self._registry.get(msg.robot_name)
        if existing is not None:
            # Only update if the incoming message is newer
            new_stamp = msg.gps_fix.header.stamp.sec + msg.gps_fix.header.stamp.nanosec * 1e-9
            old_stamp = existing.gps_fix.header.stamp.sec + existing.gps_fix.header.stamp.nanosec * 1e-9
            if new_stamp <= old_stamp:
                return

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
