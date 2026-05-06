"""Per-peer state cache derived from /gossip/peers.

All cached arrays are in this robot's local 'map' frame (ENU relative to
this robot's boot GPS), converted from the gossip frame via
frame_utils.global_enu_to_local[_batch].
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
from sensor_msgs_py import point_cloud2

from coordination_bringup.frame_utils import (
    DEFAULT_ORIGIN_ALT,
    global_enu_to_local,
    global_enu_to_local_batch,
    gps_to_enu,
)
from coordination_bringup.peer_profile import PeerProfile
from coordination_msgs.msg import PeerProfile as PeerProfileMsg


# Trailing integer in robot_name (e.g. "robot_2" -> 2).
_ROBOT_ID_RE = re.compile(r"(\d+)$")


def _extract_robot_id(robot_name: str) -> Optional[int]:
    m = _ROBOT_ID_RE.search(robot_name)
    return int(m.group(1)) if m else None


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class PeerState:
    peer_positions:  Dict[str, np.ndarray] = field(default_factory=dict)  # (3,)
    peer_waypoints:  Dict[str, np.ndarray] = field(default_factory=dict)  # (3,)
    peer_nav_modes:  Dict[str, str]        = field(default_factory=dict)
    peer_frontiers:  Dict[str, np.ndarray] = field(default_factory=dict)  # (N,3)
    peer_last_seen:  Dict[str, float]      = field(default_factory=dict)
    peer_ids:        Dict[str, int]        = field(default_factory=dict)

    # payload_ttl: ignore individual stale payloads (peer is alive but, e.g.,
    # rayfronts went silent).
    payload_ttl_sec: float = 30.0

    def update(self, msg: PeerProfileMsg, my_boot_enu: np.ndarray,
               my_alt_ground: Optional[float], now_sec: float) -> None:
        """Caller must have verified msg.robot_name != self and my_boot_enu is set."""
        name = msg.robot_name
        profile = PeerProfile.from_ros_msg(msg)

        gps = msg.gps_fix
        if gps.status.status >= 0:
            peer_global_enu = np.array(
                gps_to_enu(gps.latitude, gps.longitude, gps.altitude),
                dtype=np.float64,
            )
            self.peer_positions[name] = global_enu_to_local(
                peer_global_enu, my_boot_enu,
                local_alt_ground=my_alt_ground,
            )

        if profile.has_waypoint():
            wp = msg.waypoint.pose.position
            self.peer_waypoints[name] = global_enu_to_local(
                (wp.x, wp.y, wp.z), my_boot_enu,
                local_alt_ground=my_alt_ground,
            )

        nav_msg, nav_stamp = profile.get_payload_with_stamp("std_msgs/msg/String")
        if nav_msg is not None and self._payload_fresh(nav_stamp, now_sec):
            self.peer_nav_modes[name] = str(nav_msg.data)

        front_msg, front_stamp = profile.get_payload_with_stamp(
            "sensor_msgs/msg/PointCloud2")
        if front_msg is not None and self._payload_fresh(front_stamp, now_sec):
            pts = list(point_cloud2.read_points(
                front_msg, field_names=("x", "y", "z"), skip_nans=True))
            if pts:
                arr_global = np.array(
                    [[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
                self.peer_frontiers[name] = global_enu_to_local_batch(
                    arr_global, my_boot_enu,
                    local_alt_ground=my_alt_ground,
                )
            else:
                self.peer_frontiers[name] = np.zeros((0, 3), dtype=np.float64)

        peer_id = _extract_robot_id(name)
        if peer_id is not None:
            self.peer_ids[name] = peer_id

        self.peer_last_seen[name] = now_sec

    def _payload_fresh(self, stamp, now_sec: float) -> bool:
        # Zero stamp = no stamp set; treat as fresh.
        ts = _stamp_to_sec(stamp)
        if ts == 0.0:
            return True
        return (now_sec - ts) <= self.payload_ttl_sec
