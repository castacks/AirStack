"""
gossip_node.py — ROS 2 gossip-protocol coordination layer.

Each robot publishes its PeerProfile to /gossip/peers at 1 Hz (wall-clock,
unaffected by sim time). A new waypoint triggers an immediate extra publish.
"""

import os
import threading
import yaml
from collections import OrderedDict

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


# Deduplication seen-set size. At 1 Hz, 50 entries covers ~5 s for 10 robots —
# well beyond the window in which DDS Router duplicates arrive.
_GOSSIP_SEEN_SIZE = 50

# BEST_EFFORT: at 1 Hz a dropped message recovers on the next tick; reliability
# overhead across domain bridges was the main source of DDS-layer amplification.
GOSSIP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# RELIABLE + TRANSIENT_LOCAL so late-joining planners get the full snapshot.
REGISTRY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class GossipNode(Node):

    def __init__(self):
        super().__init__("gossip_node")

        self.declare_parameter("robot_name", os.environ.get("ROBOT_NAME", "unknown_robot"))
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("payload_topics_config", "")

        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        payload_config_path = self.get_parameter("payload_topics_config").get_parameter_value().string_value

        self._profile = PeerProfile(robot_name=self._robot_name)

        # Boot pose: set on first valid GPS fix + first heading.
        # Used to transform payload data from local odom → global ENU before broadcast.
        self._boot_pos: tuple | None = None   # (bx, by, bz) ENU metres
        self._boot_quat: tuple | None = None  # (qx, qy, qz, qw)

        self._registry: dict[str, PeerProfileMsg] = {}
        self._registry_lock = threading.Lock()

        # Per-robot inbox: holds only the latest message per robot so all robots
        # are drained together on each tick regardless of DDS arrival order.
        self._peer_inbox: dict[str, PeerProfileMsg] = {}
        self._peer_inbox_lock = threading.Lock()

        self._seen: OrderedDict = OrderedDict()
        self._payload_cache: dict[str, object] = {}
        self._payload_subs: list = []

        if payload_config_path:
            self._setup_payload_subscriptions(payload_config_path)

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

        self._gossip_pub = self.create_publisher(PeerProfileMsg, "/gossip/peers", GOSSIP_QOS)
        self._registry_pub = self.create_publisher(
            PeerProfileMsg,
            f"/{self._robot_name}/coordination/peer_registry",
            REGISTRY_QOS,
        )

        period = 1.0 / max(publish_rate, 0.01)
        self._publish_timer = self.create_timer(period, self._publish_tick, clock=ROSClock())
        self._drain_timer = self.create_timer(0.2, self._drain_peer_inbox, clock=ROSClock())

        self.get_logger().info(
            f"GossipNode started for '{self._robot_name}' "
            f"(publish_rate={publish_rate:.1f} Hz wall-clock, "
            f"{len(self._payload_cache)} payload topic(s))"
        )

    def _setup_payload_subscriptions(self, config_path: str) -> None:
        """Parse gossip_payloads.yaml and subscribe to each listed topic.

        Topics that haven't published yet simply contribute no payload on that tick.
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

            self._payload_cache[topic] = None

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

    def _on_navsat(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:  # ignore NO_FIX so GPS never zeros out
            return
        self._profile.set_gps_from_navsat(msg)
        if self._boot_pos is None:
            self._boot_pos = gps_to_enu(msg.latitude, msg.longitude, msg.altitude)
            if self._boot_quat is None and self._profile.heading != 0.0:
                self._boot_quat = heading_to_quat(self._profile.heading)

    def _on_compass(self, msg: Float64) -> None:
        self._profile.set_heading(msg.data)
        if self._boot_pos is not None and self._boot_quat is None:
            self._boot_quat = heading_to_quat(msg.data)

    def _on_global_plan(self, msg: Path) -> None:
        self._profile.set_waypoint_from_path(msg)

    def _on_peer_msg(self, msg: PeerProfileMsg) -> None:
        if msg.robot_name == self._robot_name:
            return  # discard own messages echoed back from the gossip domain

        msg_id = (msg.robot_name,
                  msg.gps_fix.header.stamp.sec,
                  msg.gps_fix.header.stamp.nanosec)
        if msg_id in self._seen:
            return
        self._seen[msg_id] = None
        if len(self._seen) > _GOSSIP_SEEN_SIZE:
            self._seen.popitem(last=False)

        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        with self._peer_inbox_lock:
            existing = self._peer_inbox.get(msg.robot_name)
            if existing is not None:
                old_t = (existing.gps_fix.header.stamp.sec
                         + existing.gps_fix.header.stamp.nanosec * 1e-9)
                if new_t < old_t:
                    return
            self._peer_inbox[msg.robot_name] = msg

    def _drain_peer_inbox(self) -> None:
        with self._peer_inbox_lock:
            inbox = dict(self._peer_inbox)
            self._peer_inbox.clear()
        for msg in inbox.values():
            self._update_registry(msg)

    def _publish_tick(self) -> None:
        self._publish_own()

    def _publish_own(self) -> None:
        self._profile.clear_payloads()
        if self._boot_pos is not None:
            bx, by, bz = self._boot_pos
            # PX4/MAVROS odom frame is ENU-aligned regardless of drone heading —
            # only translation is needed; rotation would incorrectly rotate payloads.
            q = (0.0, 0.0, 0.0, 1.0)
            for topic, entry in self._payload_cache.items():
                if entry is not None:
                    msg, stamp = entry
                    transformed = self._transform_to_global(msg, bx, by, bz, q)
                    self._profile.add_payload(transformed, stamp=stamp)

        # Stamp with current ROS clock (not MAVROS GPS stamp) so receivers can
        # enforce monotonic ordering across ticks.
        self._profile.gps_fix.header.stamp = self.get_clock().now().to_msg()
        self._gossip_pub.publish(self._profile.to_ros_msg())

    def _transform_to_global(self, msg, bx, by, bz, q):
        if isinstance(msg, MarkerArray):
            return transform_marker_array(msg, bx, by, bz, q)
        if isinstance(msg, PointCloud2):
            return transform_point_cloud2(msg, bx, by, bz, q)
        return msg  # unknown type — pass through untransformed

    def _update_registry(self, msg: PeerProfileMsg) -> None:
        """Accept msg only if newer than what we have; republish updated snapshot."""
        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)

        with self._registry_lock:
            existing = self._registry.get(msg.robot_name)
            if existing is not None:
                old_t = (existing.gps_fix.header.stamp.sec
                         + existing.gps_fix.header.stamp.nanosec * 1e-9)
                if new_t < old_t:
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
