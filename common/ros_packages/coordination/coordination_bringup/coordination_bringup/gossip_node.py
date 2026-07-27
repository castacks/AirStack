"""
gossip_node.py — ROS 2 gossip-protocol coordination layer.

Each robot publishes its PeerProfile to /gossip/peers at 1 Hz (wall-clock,
unaffected by sim time). A new waypoint triggers an immediate extra publish.
"""

import copy
import os
import threading
import time
import traceback
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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_msgs.msg import CoverageGrid

from coordination_bringup.peer_profile import PeerProfile
from coordination_bringup.frame_utils import (
    gps_to_enu, heading_to_quat, transform_marker_array, transform_point_cloud2,
)
from coordination_bringup.comms_model import should_accept, MAX_RELAY_HOPS, COMMS_RANGE_M

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2


PEER_STALE_S = 10.0        # no PeerProfile from a peer for this long → log STALE
PEER_LOG_PERIOD_S = 5.0    # period of the periodic connectivity heartbeat line

# rayfronts publishes voxels_sim/all in RDF (right-down-forward, camera-optical);
# raven converts it to FLU as [rdf_z, -rdf_x, -rdf_y] (raven_nav_node _vox_all_cb).
# That swap is a pure rotation; this is its quaternion (x, y, z, w). Payloads
# flagged `rotate_rdf_to_flu` get R(q)·p applied before the boot translation so
# the gossiped cloud lands in global ENU like every other payload.
_RDF_TO_FLU_QUAT = (-0.5, 0.5, -0.5, 0.5)

# BEST_EFFORT/VOLATILE: gossip is lossy-by-design. RELIABLE over the
# LARGE_DATA (TCP) transport back-pressured the ddsrouter under load and
# collapsed the whole mesh (SAMPLE_LOST storm); best-effort + lifetime=0 on the
# GCS markers keeps the last state instead.
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

# RELIABLE + TRANSIENT_LOCAL so late-joining planners get the full snapshot;
# depth >= fleet size (KEEP_LAST replays across all peers on the topic).
REGISTRY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
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

        self._peer_inbox: dict[str, PeerProfileMsg] = {}
        self._peer_inbox_lock = threading.Lock()

        # per-origin last accepted stamp (ns); stamp-based so it survives restarts
        self._seen_stamp_ns: dict[str, int] = {}
        self._payload_cache: dict[str, object] = {}
        self._payload_names: dict[str, str] = {}  # topic → short name (last path segment)
        self._payload_rotations: dict[str, tuple] = {}  # topic → pre-translation quat, or None
        self._payload_min_interval: dict[str, float] = {}  # topic → min send interval s (0 = every tick)
        self._payload_last_sent: dict[str, tuple] = {}  # topic → (content_key, last_monotonic_s)
        self._payload_subs: list = []

        self._coverage_topic = None

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

        # Peer-freshness watchdog: log when this robot stops hearing each peer
        # (tee'd to /tmp/gossip_<robot>.log so mission log collection grabs it).
        self._peer_last_rx: dict[str, float] = {}
        self._peer_stale: set[str] = set()
        self._last_peer_summary = 0.0
        try:
            self._diag_file = open(f"/tmp/gossip_{self._robot_name}.log", "a", buffering=1)
        except Exception:
            self._diag_file = None
        self._watchdog_timer = self.create_timer(2.0, self._peer_watchdog, clock=ROSClock())

        # Record the comms-model configuration up front so a run's radius / relay
        # setting is recoverable from the log alone — it otherwise lives only in
        # the hardcoded comms_model.py source and is lost once the run is over.
        self._diag(f"CONFIG comms_range_m={COMMS_RANGE_M:.1f} max_relay_hops={MAX_RELAY_HOPS} "
                   f"num_robots={os.environ.get('NUM_ROBOTS', '?')} robot={self._robot_name} "
                   f"publish_rate={publish_rate:.2f}")

        self.get_logger().info(
            f"GossipNode started for '{self._robot_name}' "
            f"(comms range={COMMS_RANGE_M:.0f} m, relay_hops={MAX_RELAY_HOPS}, "
            f"publish_rate={publish_rate:.1f} Hz, {len(self._payload_cache)} payload topic(s))"
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

        for entry in cfg.get("payload_topics") or []:
            topic_template = entry.get("topic", "")
            type_str = entry.get("type", "")
            if not topic_template or not type_str:
                continue

            topic = topic_template.replace("{robot_name}", self._robot_name)
            # Short human-readable name for the payload (the receiver dispatches on
            # it). Defaults to the last path segment, but an explicit `name:` is
            # required when the segment is ambiguous (e.g. '.../voxels_sim/all').
            self._payload_names[topic] = (
                entry.get("name") or topic_template.rstrip("/").split("/")[-1])
            if self._payload_names[topic] == "explored_area_coverage":
                self._coverage_topic = topic
            # Payloads already in the local map ENU frame need translation only
            # (rotation None). RDF clouds (rayfronts voxels_sim) need the RDF→FLU
            # rotation applied before that translation.
            self._payload_rotations[topic] = (
                _RDF_TO_FLU_QUAT if entry.get("rotate_rdf_to_flu") else None)
            # Opt-in throttle: a payload with min_interval_s > 0 is attached only
            # when its content changed AND that interval has elapsed (heavy
            # viz-only clouds). Unset = attached every tick as before.
            self._payload_min_interval[topic] = float(entry.get("min_interval_s", 0.0))

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

        if self._boot_pos is not None:
            own = gps_to_enu(self._profile.gps_fix.latitude,
                             self._profile.gps_fix.longitude,
                             self._profile.gps_fix.altitude)
            if not should_accept(msg, own[:2]):
                return

        stamp = msg.gps_fix.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self._seen_stamp_ns.get(msg.robot_name, -1):
            return
        self._seen_stamp_ns[msg.robot_name] = stamp_ns
        self._peer_last_rx[msg.robot_name] = time.monotonic()

        with self._peer_inbox_lock:
            self._peer_inbox[msg.robot_name] = msg

        if msg.relay_hops < MAX_RELAY_HOPS and self._boot_pos is not None:
            relay = copy.deepcopy(msg)
            relay.source = PeerProfileMsg.SOURCE_RELAYED
            relay.relay_hops = msg.relay_hops + 1
            relay.relay_lat = self._profile.gps_fix.latitude
            relay.relay_lon = self._profile.gps_fix.longitude
            self._gossip_pub.publish(relay)

    def _drain_peer_inbox(self) -> None:
        with self._peer_inbox_lock:
            inbox = dict(self._peer_inbox)
            self._peer_inbox.clear()
        for msg in inbox.values():
            self._update_registry(msg)

    def _diag(self, line: str) -> None:
        if self._diag_file is None:
            return
        try:
            # epoch (t=) after the human timestamp so offline analysis can align
            # the gossip log to bag/odom epoch time without timezone guessing.
            self._diag_file.write(
                f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] t={time.time():.2f} {line}\n")
        except Exception:
            pass

    def _own_enu(self):
        """Own ENU (x, y) from the latest GPS fix, or None before boot/GPS."""
        if self._boot_pos is None:
            return None
        try:
            x, y, _ = gps_to_enu(self._profile.gps_fix.latitude,
                                 self._profile.gps_fix.longitude,
                                 self._profile.gps_fix.altitude)
            return x, y
        except Exception:
            return None

    def _peer_watchdog(self) -> None:
        """Per-peer STALE/RECOVER transitions + a periodic connectivity heartbeat
        carrying this robot's own ENU position and each peer's last-heard age +
        UP/STALE state. With own_enu in both robots' heartbeats, the inter-robot
        distance behind every connect/drop is reconstructable from the logs alone
        (pair the two robots' HB lines by the logged epoch) — no bag needed."""
        if not self._peer_last_rx:
            return
        now = time.monotonic()
        matched = self._peer_sub.get_publisher_count()
        own = self._own_enu()
        own_s = f"own_enu={own[0]:.1f},{own[1]:.1f}" if own else "own_enu=?"
        for name in sorted(self._peer_last_rx):
            age = now - self._peer_last_rx[name]
            if age > PEER_STALE_S and name not in self._peer_stale:
                self._peer_stale.add(name)
                self.get_logger().warn(f"[gossip] peer {name} STALE — no PeerProfile for {age:.1f}s")
                self._diag(f"STALE   peer={name} silent_for={age:.1f}s {own_s} matched_writers={matched}")
            elif age <= PEER_STALE_S and name in self._peer_stale:
                self._peer_stale.discard(name)
                self.get_logger().info(f"[gossip] peer {name} RECOVERED")
                self._diag(f"RECOVER peer={name} {own_s} matched_writers={matched}")
        if now - self._last_peer_summary >= PEER_LOG_PERIOD_S:
            self._last_peer_summary = now
            peers = " ".join(
                f"{n}=age{now - self._peer_last_rx[n]:.0f}s,{'STALE' if n in self._peer_stale else 'UP'}"
                for n in sorted(self._peer_last_rx))
            self.get_logger().info(f"[gossip] heartbeat {own_s} {peers}")
            self._diag(f"HB {own_s} {peers} matched_writers={matched}")

    def _publish_tick(self) -> None:
        self._publish_own()

    def _payload_due(self, topic, msg, interval) -> bool:
        """Throttle + on-change gate: attach only if the payload content changed
        since the last send AND >= interval seconds have elapsed. Content-hashed
        (rayfronts re-stamps every publish even when the map is unchanged)."""
        try:
            key = hash(bytes(msg.data))
        except Exception:
            key = None
        last_key, last_t = self._payload_last_sent.get(topic, (None, 0.0))
        now_t = time.monotonic()
        if key is not None and key == last_key:
            return False
        if now_t - last_t < interval:
            return False
        self._payload_last_sent[topic] = (key, now_t)
        return True

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
                    interval = self._payload_min_interval.get(topic, 0.0)
                    if interval > 0.0 and not self._payload_due(topic, msg, interval):
                        continue
                    if topic == self._coverage_topic:
                        self._attach_coverage(msg, stamp, bx, by)
                    else:
                        rot = self._payload_rotations.get(topic) or q
                        transformed = self._transform_to_global(msg, bx, by, bz, rot)
                        self._profile.add_payload(transformed, stamp=stamp, name=self._payload_names.get(topic, ""))

            # Translate the waypoint into the same global ENU frame as payloads.
            # PeerProfile.waypoint is filled from /<robot>/global_plan in the
            # sender's local 'map' frame; receivers run global_enu_to_local on it
            # assuming it's global, so we MUST do the local->global step here.
            if self._profile.has_waypoint():
                self._profile.waypoint = self._translate_waypoint(
                    self._profile.waypoint, bx, by, bz)

        # Current ROS clock, not MAVROS GPS stamp: receivers dedup/order on it.
        self._profile.gps_fix.header.stamp = self.get_clock().now().to_msg()
        self._gossip_pub.publish(self._profile.to_ros_msg())

    def _attach_coverage(self, msg, stamp, bx, by):
        out = CoverageGrid()
        out.resolution = msg.resolution
        out.origin_x = msg.origin_x + bx
        out.origin_y = msg.origin_y + by
        out.width = msg.width
        out.height = msg.height
        out.data = msg.data
        self._profile.add_payload(out, stamp=stamp, name="explored_area_coverage")

    @staticmethod
    def _translate_waypoint(wp, bx, by, bz):
        """Return a copy of PoseStamped wp with position translated by (bx, by, bz)
        and frame_id set to 'map'. Orientation is left untouched."""
        out = PoseStamped()
        out.header.stamp = wp.header.stamp
        out.header.frame_id = 'map'
        out.pose.position.x = wp.pose.position.x + bx
        out.pose.position.y = wp.pose.position.y + by
        out.pose.position.z = wp.pose.position.z + bz
        out.pose.orientation = wp.pose.orientation
        return out

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
    except BaseException:
        node._diag("CRASH\n" + traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
