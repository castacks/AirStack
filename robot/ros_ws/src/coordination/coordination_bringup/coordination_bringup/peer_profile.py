"""
peer_profile.py
===============
Base class for gossip-protocol peer state.  Subclass this to attach
task-specific data as typed ROS payloads.

Usage
-----
# Basic – just use the base class
profile = PeerProfile("robot_1")
profile.set_gps_from_navsat(navsat_msg)
profile.set_heading(compass_hdg_msg.data)
profile.set_waypoint_from_path(path_msg)
ros_msg = profile.to_ros_msg()

# Extended – subclass and add payloads
class SearchProfile(PeerProfile):
    def attach_frontier(self, frontier_msg):
        self.add_payload(frontier_msg)

profile = SearchProfile("robot_1")
profile.attach_frontier(my_frontier)
ros_msg = profile.to_ros_msg()

# Receiving side
profile = PeerProfile.from_ros_msg(ros_msg)
grid = profile.get_payload("nav_msgs/msg/OccupancyGrid")
"""

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
    """
    Base peer state broadcast over the gossip bus.

    Subclass and override ``extra_payloads_to_attach()`` to automatically
    include task-specific ROS messages every time ``to_ros_msg()`` is called,
    or call ``add_payload()`` / ``clear_payloads()`` manually.
    """

    robot_name: str
    gps_fix: NavSatFix = field(default_factory=NavSatFix)
    heading: float = 0.0  # degrees clockwise from North (0-360)
    waypoint: PoseStamped = field(default_factory=PoseStamped)
    source: Source = Source.DIRECT
    relay_hops: int = 0

    # Internal payload store: list of dicts with keys "type" and "data"
    _payloads: List[Dict[str, Any]] = field(default_factory=list, repr=False)

    # ------------------------------------------------------------------ #
    # GPS / waypoint helpers                                               #
    # ------------------------------------------------------------------ #

    def set_gps_from_navsat(self, msg: NavSatFix) -> None:
        """Store GPS fix from a sensor_msgs/NavSatFix message."""
        self.gps_fix = msg

    def set_heading(self, degrees: float) -> None:
        """Set heading in degrees clockwise from North (0-360)."""
        self.heading = float(degrees)

    def set_waypoint_from_path(self, path: Optional[Path]) -> None:
        """
        Extract the goal (last pose) from a nav_msgs/Path.
        Passing None or an empty path leaves waypoint as all-zeros,
        which signals 'no plan available' to receivers.
        """
        if path is not None and len(path.poses) > 0:
            self.waypoint = path.poses[-1]
        else:
            self.waypoint = PoseStamped()

    def has_waypoint(self) -> bool:
        """Return True if a valid waypoint has been set (non-zero stamp)."""
        s = self.waypoint.header.stamp
        return s.sec != 0 or s.nanosec != 0

    # ------------------------------------------------------------------ #
    # Payload management                                                   #
    # ------------------------------------------------------------------ #

    def add_payload(self, msg: Any) -> None:
        """Serialize and attach any ROS message as an additional payload."""
        type_str = _ros_type_string(msg)
        self._payloads.append({
            "type": type_str,
            "data": serialize_message(msg),
        })

    def clear_payloads(self) -> None:
        self._payloads.clear()

    def get_payload(self, payload_type: str) -> Optional[Any]:
        """
        Deserialize and return the first payload matching *payload_type*.
        *payload_type* is the fully-qualified ROS type string,
        e.g. ``"nav_msgs/msg/OccupancyGrid"``.
        Returns None if not found.
        """
        for p in self._payloads:
            if p["type"] == payload_type:
                msg_class = rosidl_utils.get_message(payload_type)
                return deserialize_message(p["data"], msg_class)
        return None

    def get_all_payloads(self) -> List[Any]:
        """Deserialize and return all payloads in order."""
        result = []
        for p in self._payloads:
            msg_class = rosidl_utils.get_message(p["type"])
            result.append(deserialize_message(p["data"], msg_class))
        return result

    def payload_types(self) -> List[str]:
        """Return the type strings of all attached payloads."""
        return [p["type"] for p in self._payloads]

    # ------------------------------------------------------------------ #
    # Serialisation                                                        #
    # ------------------------------------------------------------------ #

    def to_ros_msg(self) -> PeerProfileMsg:
        msg = PeerProfileMsg()
        msg.robot_name = self.robot_name
        msg.gps_fix = self.gps_fix
        msg.heading = self.heading
        msg.waypoint = self.waypoint
        msg.source = int(self.source)
        msg.relay_hops = self.relay_hops
        msg.payloads = [
            PeerProfilePayloadMsg(payload_type=p["type"], payload_data=list(p["data"]))
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
            {"type": p.payload_type, "data": bytes(p.payload_data)}
            for p in msg.payloads
        ]
        return profile


# ------------------------------------------------------------------ #
# Internal helpers                                                     #
# ------------------------------------------------------------------ #

def _ros_type_string(msg: Any) -> str:
    """Return the fully-qualified ROS type string for a message instance.

    E.g. nav_msgs.msg.OccupancyGrid → "nav_msgs/msg/OccupancyGrid"
    """
    module = type(msg).__module__    # e.g. "nav_msgs.msg._occupancy_grid"
    name = type(msg).__name__        # e.g. "OccupancyGrid"
    # Convert "nav_msgs.msg._occupancy_grid" → "nav_msgs/msg"
    parts = module.split(".")
    if len(parts) >= 2:
        pkg = parts[0]
        sub = parts[1]
        return f"{pkg}/{sub}/{name}"
    return f"{module}/{name}"
