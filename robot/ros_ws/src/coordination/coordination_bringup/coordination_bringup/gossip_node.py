"""
gossip_node.py
==============
ROS 2 node that implements the gossip-protocol coordination layer.

Each robot:
  - Subscribes to its own odometry and global plan
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

import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Odometry, Path
from coordination_msgs.msg import PeerProfile as PeerProfileMsg

from coordination_bringup.peer_profile import PeerProfile


# QoS for gossip topic: best-effort, keep last 1 (low-overhead, high-freq)
GOSSIP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
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

        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        # ── State ────────────────────────────────────────────────────────
        self._profile = PeerProfile(robot_name=self._robot_name)

        # Registry: robot_name → latest PeerProfile ROS msg
        self._registry: dict[str, PeerProfileMsg] = {}

        # ── Subscriptions ────────────────────────────────────────────────
        self._odom_sub = self.create_subscription(
            Odometry,
            f"/{self._robot_name}/odometry_conversion/odometry",
            self._on_odom,
            GOSSIP_QOS,
        )
        self._path_sub = self.create_subscription(
            Path,
            f"/{self._robot_name}/random_walk/global_plan",
            self._on_global_plan,
            GOSSIP_QOS,
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
            f"(publish_rate={publish_rate:.1f} Hz wall-clock)"
        )

    # ------------------------------------------------------------------ #
    # Subscription callbacks                                               #
    # ------------------------------------------------------------------ #

    def _on_odom(self, msg: Odometry) -> None:
        self._profile.set_pose_from_odom(msg)

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
        self._gossip_pub.publish(self._profile.to_ros_msg())

    # ------------------------------------------------------------------ #
    # Registry management                                                  #
    # ------------------------------------------------------------------ #

    def _update_registry(self, msg: PeerProfileMsg) -> None:
        """Update registry with latest-wins semantics, then republish snapshot."""
        existing = self._registry.get(msg.robot_name)
        if existing is not None:
            # Only update if the incoming message is newer
            new_stamp = msg.pose.header.stamp.sec + msg.pose.header.stamp.nanosec * 1e-9
            old_stamp = existing.pose.header.stamp.sec + existing.pose.header.stamp.nanosec * 1e-9
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
