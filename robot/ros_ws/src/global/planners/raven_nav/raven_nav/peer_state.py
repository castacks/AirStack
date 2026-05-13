"""Per-peer state cache derived from /gossip/peers.

All cached arrays are in this robot's local 'map' frame (ENU relative to
this robot's boot GPS), converted from the gossip frame via
frame_utils.global_enu_to_local[_batch].
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

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
class PeerRays:
    origins: np.ndarray  # (N, 3) in local map frame
    dirs:    np.ndarray  # (N, 3) FLU unit vectors
    scores:  np.ndarray  # (N, K) — K = len(query_labels)


@dataclass
class PeerConfirmedTarget:
    """One AABB a peer is observing or has visited. Coords already in local frame."""
    label: str
    center: np.ndarray  # (3,) [cx, cy, cz]
    size:   np.ndarray  # (3,) [sx, sy, sz]
    status: str         # 'observing' | 'visited'
    confidence: float
    ts: float


@dataclass
class PeerState:
    peer_positions:  Dict[str, np.ndarray] = field(default_factory=dict)  # (3,)
    peer_waypoints:  Dict[str, np.ndarray] = field(default_factory=dict)  # (3,)
    peer_nav_modes:  Dict[str, str]        = field(default_factory=dict)
    peer_frontiers:  Dict[str, np.ndarray] = field(default_factory=dict)  # (N,3)
    peer_rays:       Dict[str, 'PeerRays']  = field(default_factory=dict)
    # Per-group bids (one BidEntry per ray group). Imported lazily in update()
    # to avoid a hard dependency from this module on bid_manager.
    peer_bids:       Dict[str, list] = field(default_factory=dict)
    peer_completed:  Dict[str, Set[str]]   = field(default_factory=dict)
    peer_committed_target: Dict[str, str]  = field(default_factory=dict)  # "" if uncommitted
    # XY centers (N,2) of frontier zones this peer has cleared (in our local frame).
    peer_completed_zones: Dict[str, np.ndarray] = field(default_factory=dict)
    peer_confirmed_targets: Dict[str, list]  = field(default_factory=dict)  # list[PeerConfirmedTarget]
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

        # Per-payload freshness gating is disabled: the gossip publisher may
        # use sim time while we use wall time, which makes absolute-stamp
        # comparisons meaningless. Each gossip tick overwrites the previous
        # value, so consumers can rely on peer_last_seen (wall-clock, set
        # below) for liveness, and on the most-recent message for content.
        # Two separate String payloads exist (navigation_mode + completed_targets);
        # use by-name lookups so we don't grab the wrong one.
        nav_msg, _ = profile.get_payload_by_name_with_stamp("navigation_mode")
        if nav_msg is not None:
            self.peer_nav_modes[name] = str(nav_msg.data)

        comp_msg, _ = profile.get_payload_by_name_with_stamp("completed_targets")
        if comp_msg is not None:
            try:
                lst = json.loads(comp_msg.data)
                if isinstance(lst, list):
                    self.peer_completed[name] = {str(x) for x in lst}
            except (json.JSONDecodeError, TypeError):
                pass

        ct_msg, _ = profile.get_payload_by_name_with_stamp("committed_target")
        if ct_msg is not None:
            self.peer_committed_target[name] = str(ct_msg.data)

        front_msg, _ = profile.get_payload_by_name_with_stamp("raw_frontiers")
        if front_msg is not None:
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

        rays_msg, _ = profile.get_payload_by_name_with_stamp("shared_rays")
        if rays_msg is not None:
            self.peer_rays[name] = self._parse_shared_rays(
                rays_msg, my_boot_enu, my_alt_ground)

        zones_msg, _ = profile.get_payload_by_name_with_stamp(
            "completed_frontier_zones")
        if zones_msg is not None:
            pts = list(point_cloud2.read_points(
                zones_msg, field_names=("x", "y"), skip_nans=True))
            if pts:
                arr_global = np.array(
                    [[p[0], p[1], 0.0] for p in pts], dtype=np.float64)
                local = global_enu_to_local_batch(
                    arr_global, my_boot_enu, local_alt_ground=my_alt_ground)
                self.peer_completed_zones[name] = local[:, :2]
            else:
                self.peer_completed_zones[name] = np.zeros((0, 2),
                                                           dtype=np.float64)

        ct_targets_msg, _ = profile.get_payload_by_name_with_stamp(
            "confirmed_targets")
        if ct_targets_msg is not None:
            self.peer_confirmed_targets[name] = self._parse_confirmed_targets(
                ct_targets_msg, my_boot_enu, my_alt_ground)

        # Look up the bid payload by name (gossip auto-derives "bids" from
        # the topic /<robot>/bids). By-name avoids type-import issues if
        # airstack_msgs isn't loadable for some reason — get_payload_with_stamp
        # would raise during deserialize.
        try:
            bid_msg, bid_stamp = profile.get_payload_by_name_with_stamp("bids")
        except Exception as e:
            bid_msg, bid_stamp = None, None
            print(f'[peer_state] BidVector deserialize failed for {name}: {e}')
        if bid_msg is None:
            payload_names = [p.get("name", "") for p in profile._payloads]
            print(f'[peer_state] no "bids" payload for {name} '
                  f'(available payload names: {payload_names})')
        else:
            # Don't gate on stamp-vs-now: the publisher (gossip_node) may use
            # sim time while we use wall time, which makes absolute-time
            # comparisons meaningless. Gossip refresh is implicit — the most
            # recent PeerProfile overwrites the previous bid each tick, so a
            # truly stale peer just stops updating peer_bids[name].
            # Per-group bids: parallel arrays produce one BidEntry per row.
            # Origins arrive in global ENU (gossip translates origin_x/y/z);
            # convert back into our local frame for the auction.
            from raven_nav.bid_manager import BidEntry
            n_rows = len(bid_msg.labels)
            entries: list = []
            if n_rows > 0:
                origins_global = np.array(
                    list(zip(bid_msg.origin_x, bid_msg.origin_y,
                             bid_msg.origin_z)),
                    dtype=np.float64,
                )
                origins_local = global_enu_to_local_batch(
                    origins_global, my_boot_enu,
                    local_alt_ground=my_alt_ground,
                )
                for i in range(n_rows):
                    entries.append(BidEntry(
                        label=str(bid_msg.labels[i]),
                        value=float(bid_msg.values[i]),
                        avg_origin=origins_local[i],
                        avg_dir=np.array([
                            float(bid_msg.dir_x[i]),
                            float(bid_msg.dir_y[i]),
                            float(bid_msg.dir_z[i]),
                        ], dtype=np.float64),
                        num_rays=int(bid_msg.num_rays[i]),
                        avg_score=float(bid_msg.avg_score[i]),
                    ))
            self.peer_bids[name] = entries
            summary = [(e.label, round(e.value, 2)) for e in entries]
            print(f'[peer_state] applied bids for {name}: {summary}')

        peer_id = _extract_robot_id(name)
        if peer_id is not None:
            self.peer_ids[name] = peer_id

        self.peer_last_seen[name] = now_sec

    def compute_peer_ray_groups(
        self, name: str, query_labels: list, target_objects: list,
        score_threshold: float,
        min_altitude: float = 1.5, max_altitude: float = 100.0,
        angle_thresh_deg: float = 45.0,
    ):
        """Rebuild a peer's ray groups locally from its gossiped raw rays.

        Avoids a separate gossip payload — `compute_ray_groups` is
        deterministic, so applying it to the peer's rays (already transformed
        into this robot's local map) yields the same groups the peer would
        compute itself. The peer's own position is used as `cur_pose` so the
        derived avg_dist_to_robot stays anchored to the observer.
        """
        from raven_nav.ray_groups import compute_ray_groups
        rays = self.peer_rays.get(name)
        if rays is None or len(rays.origins) == 0:
            return []
        peer_pos = self.peer_positions.get(name)
        if peer_pos is None:
            peer_pos = rays.origins.mean(axis=0)
        return compute_ray_groups(
            ray_origins=rays.origins,
            ray_dirs=rays.dirs,
            ray_scores=rays.scores,
            query_labels=query_labels,
            target_objects=target_objects,
            score_threshold=score_threshold,
            cur_pose=np.asarray(peer_pos),
            min_altitude=min_altitude,
            max_altitude=max_altitude,
            angle_thresh_deg=angle_thresh_deg,
        )

    def _parse_shared_rays(self, msg, my_boot_enu, my_alt_ground) -> 'PeerRays':
        """Decode x,y,z,dx,dy,dz,sim_* PointCloud2 into PeerRays in local frame."""
        names = [f.name for f in msg.fields]
        sim_fields = sorted([f for f in names if f.startswith('sim_')],
                            key=lambda s: int(s.split('_', 1)[1]))
        fields = ('x', 'y', 'z', 'dx', 'dy', 'dz') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(
            msg, field_names=fields, skip_nans=True))
        if not pts:
            return PeerRays(
                origins=np.zeros((0, 3), dtype=np.float64),
                dirs=np.zeros((0, 3), dtype=np.float32),
                scores=np.zeros((0, len(sim_fields)), dtype=np.float32))
        arr = np.array([list(p) for p in pts], dtype=np.float64)
        origins_global = arr[:, :3]
        dirs = arr[:, 3:6].astype(np.float32)
        scores = arr[:, 6:].astype(np.float32)
        origins_local = global_enu_to_local_batch(
            origins_global, my_boot_enu, local_alt_ground=my_alt_ground)
        return PeerRays(origins=origins_local, dirs=dirs, scores=scores)

    def _parse_confirmed_targets(
        self, msg, my_boot_enu, my_alt_ground,
    ) -> list:
        """Decode JSON list of {label, cx, cy, cz, sx, sy, sz, status, confidence, ts}.

        Centers are gossiped in global ENU and converted to local frame.
        Sizes pass through unchanged (frame-invariant).
        """
        try:
            lst = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return []
        if not isinstance(lst, list) or not lst:
            return []
        out: list = []
        centers_global = np.array(
            [[float(d['cx']), float(d['cy']), float(d['cz'])] for d in lst],
            dtype=np.float64,
        )
        centers_local = global_enu_to_local_batch(
            centers_global, my_boot_enu, local_alt_ground=my_alt_ground)
        for i, d in enumerate(lst):
            out.append(PeerConfirmedTarget(
                label=str(d.get('label', '')),
                center=centers_local[i],
                size=np.array([float(d.get('sx', 0.0)),
                               float(d.get('sy', 0.0)),
                               float(d.get('sz', 0.0))]),
                status=str(d.get('status', 'observing')),
                confidence=float(d.get('confidence', 0.0)),
                ts=float(d.get('ts', 0.0)),
            ))
        return out

    def _payload_fresh(self, stamp, now_sec: float) -> bool:
        # Zero stamp = no stamp set; treat as fresh.
        ts = _stamp_to_sec(stamp)
        if ts == 0.0:
            return True
        return (now_sec - ts) <= self.payload_ttl_sec
