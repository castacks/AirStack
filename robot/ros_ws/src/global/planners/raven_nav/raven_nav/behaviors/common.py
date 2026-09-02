"""Shared, ROS-free plumbing for the four RAVEN behaviours.

Nothing in `raven_nav.behaviors` imports rclpy or a ROS message type: every
behaviour is pure numpy so it can be unit-tested on a host with no ROS
installed.  Message construction lives in `raven_nav/ros_io.py`, which the node
owns.

Frames
------
The original RAVEN behaviours read the mapper's tensors directly and did the
RDF -> FLU flip themselves (`[p[2], -p[0], -p[1]]`).  Here rays and voxels
arrive over ROS already flipped by `raven_nav_node._ray_all_cb` /
`_vox_all_cb` (same formula), so the behaviours see FLU throughout.  Frontiers
are the one exception: they arrive raw and `FrontierBehavior` does the flip
itself, exactly as the OG file did.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np


def rdf_to_flu(arr: np.ndarray) -> np.ndarray:
    """(N,3) RDF -> (N,3) FLU: x=z, y=-x, z=-y.

    OG frontier_behavior.py:26-30 / ray_behavior.py:67-68 /
    voxel_behavior.py:87-92 all spell out this same permutation.
    """
    a = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
    return np.stack([a[:, 2], -a[:, 0], -a[:, 1]], axis=1)


def points_in_polygon(pts_xy: np.ndarray, poly_xy: np.ndarray) -> np.ndarray:
    """Vectorised even-odd ray cast. pts_xy: (N,2); poly_xy: (M,2)."""
    pts_xy = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    if pts_xy.size == 0 or poly_xy is None or np.asarray(poly_xy).shape[0] < 3:
        return np.ones(pts_xy.shape[0], dtype=bool)
    poly_xy = np.asarray(poly_xy, dtype=np.float64)
    px, py = pts_xy[:, 0], pts_xy[:, 1]
    inside = np.zeros(pts_xy.shape[0], dtype=bool)
    m = poly_xy.shape[0]
    for i in range(m):
        x1, y1 = poly_xy[i]
        x2, y2 = poly_xy[(i + 1) % m]
        cond_y = (y1 > py) != (y2 > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = (x2 - x1) * (py - y1) / (y2 - y1 + 1e-12) + x1
        inside ^= cond_y & (px < x_cross)
    return inside


def point_in_polygon(xy, poly_xy) -> bool:
    """Scalar convenience wrapper. Unconstrained (True) when poly is None/<3."""
    return bool(points_in_polygon(np.asarray(xy, dtype=float)[:2].reshape(1, 2),
                                  poly_xy)[0])


def clamp_z(p, min_altitude: float, max_altitude: float) -> np.ndarray:
    """DEVIATION 1 from OG — clamp a waypoint's z into the mission altitude band.

    The OG behaviours emitted whatever z the map handed them: a frontier
    centroid's, a ray origin's, a standoff point 1 m off a cluster surface.  In
    AirStack the altitude band is part of the SemanticSearchTask goal
    (`min_altitude_agl` / `max_altitude_agl`), and a casualty lying on the
    ground gives a standoff z of ~0.5 m AGL, which droan will not fly.  Every
    waypoint any behaviour emits goes through here.
    """
    q = np.asarray(p, dtype=np.float64).copy().reshape(3)
    lo = float(min_altitude)
    hi = float(max_altitude)
    if hi < lo:
        lo, hi = hi, lo
    q[2] = float(np.clip(q[2], lo, hi))
    return q


@dataclass
class TickContext:
    """Everything a behaviour may read on one tick. All arrays are FLU except
    `frontiers`, which is the raw rayfronts cloud (cols 0:3 are RDF)."""

    cur_pose: np.ndarray                      # (3,) FLU, local `map` frame
    now: float = 0.0                          # seconds (sim clock)
    query_labels: List[str] = field(default_factory=list)   # column order
    target_objects: List[str] = field(default_factory=list)
    ray_origins: Optional[np.ndarray] = None  # (N,3) FLU
    ray_dirs: Optional[np.ndarray] = None     # (N,3) FLU unit
    ray_scores: Optional[np.ndarray] = None   # (N,Q) softmax
    vox_xyz: Optional[np.ndarray] = None      # (M,3) FLU
    vox_scores: Optional[np.ndarray] = None   # (M,Q) softmax
    frontiers: Optional[np.ndarray] = None    # (K,>=3) RDF cols 0:3
    search_area_xy: Optional[np.ndarray] = None   # (P,2) local frame or None
    min_altitude: float = 1.5
    max_altitude: float = 100.0
    waypoint_locked: bool = False
    target_waypoint: Optional[np.ndarray] = None
    target_waypoint2: Optional[np.ndarray] = None

    def clamp(self, p) -> np.ndarray:
        return clamp_z(p, self.min_altitude, self.max_altitude)

    def inside_area(self, p) -> bool:
        """DEVIATION 2 from OG — respect the mission's search polygon."""
        return point_in_polygon(np.asarray(p, dtype=float), self.search_area_xy)

    def target_columns(self) -> List[int]:
        """Indices of `target_objects` inside `query_labels` (OG
        `[queries_labels['text'].index(t) for t in target_objects]`, but
        tolerant of a target that rayfronts has not registered yet)."""
        return columns_for(self.query_labels, self.target_objects)


def columns_for(query_labels: Sequence[str], wanted: Sequence[str]) -> List[int]:
    """Column indices of `wanted` within `query_labels`, in `wanted` order.

    Matching is case-insensitive and tolerant of the rayfronts topic-name
    sanitisation (spaces vs underscores), because the authoritative column
    order is parsed out of the `q{k}_{label}` topic names.
    """
    if not query_labels or not wanted:
        return []
    norm = {_norm_label(l): i for i, l in enumerate(query_labels)}
    out = []
    for w in wanted:
        i = norm.get(_norm_label(w))
        if i is not None and i not in out:
            out.append(i)
    return out


def _norm_label(s: str) -> str:
    return str(s).strip().lower().replace('_', ' ')


@dataclass
class BehaviorOutput:
    """What a behaviour's `execute` produced this tick.

    `path` is the list of waypoints the node should publish as a
    `nav_msgs/Path` (empty = publish nothing, exactly like the OG behaviours'
    early returns).
    """

    waypoint_locked: bool = False
    target_waypoint: Optional[np.ndarray] = None
    target_waypoint2: Optional[np.ndarray] = None
    path: List[np.ndarray] = field(default_factory=list)
    note: str = ''
