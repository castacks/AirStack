from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional

from rclpy.serialization import deserialize_message, serialize_message
import rosidl_runtime_py.utilities as rosidl_utils

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_msgs.msg import PeerProfilePayload as PeerProfilePayloadMsg


class Source(IntEnum):
    DIRECT = 0
    RELAYED = 1


@dataclass
class PeerProfile:
    """Base peer state broadcast over the gossip bus."""

    robot_name: str
    gps_fix: NavSatFix = field(default_factory=NavSatFix)
    heading: float = 0.0  # degrees clockwise from North (0-360)
    waypoint: PoseStamped = field(default_factory=PoseStamped)
    source: Source = Source.DIRECT
    relay_hops: int = 0

    _payloads: List[Dict[str, Any]] = field(default_factory=list, repr=False)

    def set_gps_from_navsat(self, msg: NavSatFix) -> None:
        self.gps_fix = msg

    def set_heading(self, degrees: float) -> None:
        self.heading = float(degrees)

    def set_waypoint_from_path(self, path: Optional[Path]) -> None:
        """Extract goal (last pose) from a Path. None or empty path sets waypoint to all-zeros (no plan)."""
        if path is not None and len(path.poses) > 0:
            self.waypoint = path.poses[-1]
        else:
            self.waypoint = PoseStamped()

    def has_waypoint(self) -> bool:
        s = self.waypoint.header.stamp
        return s.sec != 0 or s.nanosec != 0

    def add_payload(self, msg: Any, stamp=None) -> None:
        """Serialize and attach a ROS message as a payload.

        Pass the source message's header stamp so receivers can judge data freshness
        independently of the gossip message timestamp.
        """
        from builtin_interfaces.msg import Time
        type_str = _ros_type_string(msg)
        self._payloads.append({
            "type": type_str,
            "data": serialize_message(msg),
            "stamp": stamp if stamp is not None else Time(),
        })

    def clear_payloads(self) -> None:
        self._payloads.clear()

    def get_payload(self, payload_type: str) -> Optional[Any]:
        """Return the first payload matching payload_type (e.g. 'nav_msgs/msg/OccupancyGrid'), or None."""
        for p in self._payloads:
            if p["type"] == payload_type:
                msg_class = rosidl_utils.get_message(payload_type)
                return deserialize_message(p["data"], msg_class)
        return None

    def get_payload_with_stamp(self, payload_type: str):
        """Like get_payload() but returns (msg, stamp). Returns (None, None) if not found."""
        for p in self._payloads:
            if p["type"] == payload_type:
                msg_class = rosidl_utils.get_message(payload_type)
                return deserialize_message(p["data"], msg_class), p.get("stamp")
        return None, None

    def get_all_payloads(self) -> List[Any]:
        result = []
        for p in self._payloads:
            msg_class = rosidl_utils.get_message(p["type"])
            result.append(deserialize_message(p["data"], msg_class))
        return result

    def payload_types(self) -> List[str]:
        return [p["type"] for p in self._payloads]

    def to_ros_msg(self) -> PeerProfileMsg:
        msg = PeerProfileMsg()
        msg.robot_name = self.robot_name
        msg.gps_fix = self.gps_fix
        msg.heading = self.heading
        msg.waypoint = self.waypoint
        msg.source = int(self.source)
        msg.relay_hops = self.relay_hops
        msg.payloads = [
            PeerProfilePayloadMsg(
                stamp=p.get("stamp") or PeerProfilePayloadMsg().stamp,
                payload_type=p["type"],
                payload_data=list(p["data"]),
            )
            for p in self._payloads
        ]
        return msg

    @classmethod
    def from_ros_msg(cls, msg: PeerProfileMsg) -> "PeerProfile":
        profile = cls(robot_name=msg.robot_name)
        profile.gps_fix = msg.gps_fix
        profile.heading = msg.heading
        profile.waypoint = msg.waypoint
        profile.source = Source(msg.source)
        profile.relay_hops = msg.relay_hops
        profile._payloads = [
            {"type": p.payload_type, "data": bytes(p.payload_data), "stamp": p.stamp}
            for p in msg.payloads
        ]
        return profile


def _ros_type_string(msg: Any) -> str:
    """Return the fully-qualified ROS type string for a message instance.
    E.g. nav_msgs.msg.OccupancyGrid → 'nav_msgs/msg/OccupancyGrid'
    """
    module = type(msg).__module__
    name = type(msg).__name__
    # Convert "nav_msgs.msg._occupancy_grid" → "nav_msgs/msg"
    parts = module.split(".")
    if len(parts) >= 2:
        return f"{parts[0]}/{parts[1]}/{name}"
    return f"{module}/{name}"
