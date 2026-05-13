"""Triangulation-based target instance separation from RayGroups.

A RayGroup is a bundle of rays (origin + direction, no range) that the local
network believes are looking at the same semantic class. With only origin and
direction, a single bundle gives a bearing but not a position. To localize a
target and to decide whether two bundles look at the same physical thing, we
triangulate pairs of bundles: the closest-approach point between the two skew
lines is our estimate of where they converge.

Five gates determine "same target":
  1. Labels match.
  2. Angular separation between the two avg directions is at least
     ANGLE_MIN_DEG (otherwise triangulation is ill-conditioned).
  3. The closest-approach perpendicular distance is at most EPS_MAX_M.
  4. The triangulation point sits at least T_MIN_M ahead of each bundle's
     origin (kills the "ray originating over house 1 pointing at house 2"
     false positive where the rays pass *near* house 1 but aren't aimed there).
  5. The triangulation point sits inside the search polygon.

A bounding-box pre-filter runs first: if a ray group pierces a known
confirmed-target AABB with matching label, attach the group to that target
instead of letting it spawn a new triangulated hypothesis. Voxel BBs are
higher-confidence than ray triangulations, so they anchor the system.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from raven_nav.ray_groups import RayGroup


# Thresholds — tune in sim. Documented in the design doc.
EPS_MAX_M = 5.0
T_MIN_M = 5.0
ANGLE_MIN_DEG = 15.0
ANGLE_PARALLEL_DEG = 8.0
NOMINAL_RANGE_M = 25.0
MIN_SCORE = 0.6


@dataclass
class RayTarget:
    label: str
    position: np.ndarray                       # (3,)
    contributing_group_ids: List[int]
    status: str = 'unconfirmed'                # unconfirmed | confirmed | visited
    confidence: float = 0.0
    last_update_ts: float = 0.0
    bb_id: Optional[int] = None                # set when attached to a known AABB
    position_cov: np.ndarray = field(default_factory=lambda: np.eye(3) * 100.0)


def triangulate(
    o_a: np.ndarray, d_a: np.ndarray,
    o_b: np.ndarray, d_b: np.ndarray,
) -> Tuple[np.ndarray, float, float, float, float]:
    """Closest-approach between two 3D lines.

    Returns (P, eps, t_a, t_b, angle_deg):
      P     — midpoint of the shortest segment connecting the lines.
      eps   — perpendicular distance between the lines at closest approach.
      t_a   — signed parameter along d_a giving the foot of the perpendicular.
      t_b   — same along d_b.
      angle_deg — angle between d_a and d_b (acute).

    Parallel lines (cross product near zero): returns inf eps and zero
    parameters so the gates in is_same_target reject the pair cleanly.
    """
    n = np.cross(d_a, d_b)
    n_dot = float(np.dot(n, n))
    cos_a = float(np.clip(np.dot(d_a, d_b), -1.0, 1.0))
    angle_deg = float(np.degrees(np.arccos(abs(cos_a))))
    if n_dot < 1e-9:
        return np.zeros(3), float('inf'), 0.0, 0.0, angle_deg
    diff = o_b - o_a
    t_a = float(np.dot(np.cross(diff, d_b), n) / n_dot)
    t_b = float(np.dot(np.cross(diff, d_a), n) / n_dot)
    foot_a = o_a + t_a * d_a
    foot_b = o_b + t_b * d_b
    p = (foot_a + foot_b) / 2.0
    eps = float(np.linalg.norm(foot_a - foot_b))
    return p, eps, t_a, t_b, angle_deg


def is_same_target(
    group_a: RayGroup, group_b: RayGroup,
    polygon_xy: Optional[np.ndarray],
    eps_max: float = EPS_MAX_M,
    t_min: float = T_MIN_M,
    angle_min_deg: float = ANGLE_MIN_DEG,
) -> Tuple[bool, Optional[np.ndarray], float]:
    """Five-gate same-target test. Returns (decision, P, eps).

    P and eps are returned even when the decision is False so callers can log
    why a pair was rejected. P is None only when triangulation itself failed
    (parallel lines).
    """
    if group_a.label != group_b.label:
        return False, None, float('inf')
    p, eps, t_a, t_b, angle_deg = triangulate(
        group_a.avg_origin, group_a.avg_dir,
        group_b.avg_origin, group_b.avg_dir,
    )
    if angle_deg < angle_min_deg:
        return False, p if eps < float('inf') else None, eps
    if eps > eps_max:
        return False, p, eps
    if t_a < t_min or t_b < t_min:
        return False, p, eps
    if polygon_xy is not None and len(polygon_xy) >= 3:
        if not _point_in_polygon_xy(p[0], p[1], polygon_xy):
            return False, p, eps
    return True, p, eps


def ray_aabb_hits(
    origin: np.ndarray, direction: np.ndarray,
    bb: np.ndarray, t_min_ahead: float = T_MIN_M,
) -> Tuple[bool, float]:
    """Slab method ray-AABB intersection. Returns (hit, t_enter).

    bb is [cx, cy, cz, sx, sy, sz] (center + full sizes), matching the format
    voxel_behavior produces in target_voxel_clusters.

    `hit` is True only if the ray actually enters the box from in front of the
    origin AND the entry distance is at least t_min_ahead (so a ray that
    starts inside or right at the box edge isn't counted — that pathological
    case is the user's "ray origin sitting on top of house 1" example).
    """
    cx, cy, cz, sx, sy, sz = bb
    half = np.array([sx, sy, sz]) / 2.0
    center = np.array([cx, cy, cz])
    o_local = origin - center
    t_lo = -np.inf
    t_hi = np.inf
    for axis in range(3):
        d = direction[axis]
        if abs(d) < 1e-9:
            if abs(o_local[axis]) > half[axis]:
                return False, float('inf')
            continue
        t1 = (-half[axis] - o_local[axis]) / d
        t2 = (half[axis] - o_local[axis]) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_lo = max(t_lo, t1)
        t_hi = min(t_hi, t2)
    if t_hi < max(t_lo, 0.0):
        return False, float('inf')
    t_enter = t_lo if t_lo > 0 else 0.0
    if t_enter < t_min_ahead:
        return False, t_enter
    return True, t_enter


def least_squares_intersect(
    origins: np.ndarray, dirs: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Closed-form least-squares point of multiple skew lines.

    Minimizes sum_i |(I - d_i d_i^T) (P - O_i)|^2. Returns (P, residuals_cov).
    Falls back to the centroid of the foot-of-perpendicular set when the
    normal equation is singular (e.g. all directions nearly parallel).
    """
    n = origins.shape[0]
    a = np.zeros((3, 3))
    b = np.zeros(3)
    for i in range(n):
        d = dirs[i]
        proj = np.eye(3) - np.outer(d, d)
        a += proj
        b += proj @ origins[i]
    try:
        p = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        p = origins.mean(axis=0)
    residuals = np.zeros(n)
    for i in range(n):
        d = dirs[i]
        proj = np.eye(3) - np.outer(d, d)
        r = proj @ (p - origins[i])
        residuals[i] = float(np.linalg.norm(r))
    # Crude isotropic covariance scaled by residual RMS; fine for display.
    rms = float(np.sqrt(np.mean(residuals ** 2))) if n else 100.0
    cov = np.eye(3) * max(rms ** 2, 0.25)
    return p, cov


def build_targets(
    own_groups: List[RayGroup],
    peer_groups: List[RayGroup],
    known_bbs: List[Tuple[int, str, np.ndarray]],
    polygon_xy: Optional[np.ndarray],
    now_ts: float,
    eps_max: float = EPS_MAX_M,
    t_min: float = T_MIN_M,
    angle_min_deg: float = ANGLE_MIN_DEG,
) -> List[RayTarget]:
    """Build RayTarget list from own + peer ray groups, anchored by known BBs.

    known_bbs is a list of (bb_id, label, bb_array) where bb_array is
    [cx,cy,cz,sx,sy,sz]. A ray group that pierces a matching-label BB is
    attached to that bb_id and skipped from triangulation.

    The result is one RayTarget per cluster of "same target" ray groups
    plus one RayTarget per bb_id that has at least one attached group. BBs
    with no attached rays do NOT appear here — they'll be carried by the
    confirmed-target list in discoveries.py.
    """
    groups = list(own_groups) + list(peer_groups)
    if not groups:
        return []

    # Pre-filter by score (already filtered at compute_ray_groups time but
    # honor MIN_SCORE in case thresholds drift).
    groups = [g for g in groups if g.avg_score >= MIN_SCORE]
    if not groups:
        return []

    n = len(groups)

    # Step 1: BB pre-filter.
    bb_attachments: dict = {}  # bb_id -> [group_idx, ...]
    consumed = [False] * n
    for i, g in enumerate(groups):
        for bb_id, bb_label, bb in known_bbs:
            if bb_label != g.label:
                continue
            hit, _ = ray_aabb_hits(g.avg_origin, g.avg_dir, bb, t_min_ahead=t_min)
            if hit:
                bb_attachments.setdefault(bb_id, []).append(i)
                consumed[i] = True
                break

    # Step 2: triangulation graph over remaining groups.
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        if consumed[i]:
            continue
        for j in range(i + 1, n):
            if consumed[j]:
                continue
            same, _, _ = is_same_target(
                groups[i], groups[j], polygon_xy,
                eps_max=eps_max, t_min=t_min, angle_min_deg=angle_min_deg,
            )
            if same:
                union(i, j)

    # Step 3: emit one RayTarget per BB-attached set, one per connected
    # component of remaining groups.
    bb_lookup = {bb_id: (label, bb) for bb_id, label, bb in known_bbs}
    targets: List[RayTarget] = []

    for bb_id, idxs in bb_attachments.items():
        label, bb = bb_lookup[bb_id]
        pos = np.array(bb[:3])
        scores = [groups[i].avg_score for i in idxs]
        targets.append(RayTarget(
            label=label,
            position=pos,
            contributing_group_ids=idxs,
            status='confirmed',
            confidence=max(scores),
            last_update_ts=now_ts,
            bb_id=bb_id,
            position_cov=np.eye(3) * 0.5,
        ))

    components: dict = {}
    for i in range(n):
        if consumed[i]:
            continue
        components.setdefault(find(i), []).append(i)

    for _, idxs in components.items():
        members = [groups[i] for i in idxs]
        label = members[0].label
        scores = [m.avg_score for m in members]
        if len(idxs) == 1:
            m = members[0]
            pos = m.avg_origin + m.avg_dir * NOMINAL_RANGE_M
            status = 'unconfirmed'
            cov = np.eye(3) * (NOMINAL_RANGE_M ** 2)
        else:
            origins = np.stack([m.avg_origin for m in members], axis=0)
            dirs = np.stack([m.avg_dir for m in members], axis=0)
            pos, cov = least_squares_intersect(origins, dirs)
            status = 'confirmed'
        targets.append(RayTarget(
            label=label,
            position=pos,
            contributing_group_ids=idxs,
            status=status,
            confidence=max(scores),
            last_update_ts=now_ts,
            position_cov=cov,
        ))

    return targets


def _point_in_polygon_xy(x: float, y: float, polygon_xy: np.ndarray) -> bool:
    """Even-odd ray-cast inside test (handles non-convex polygons)."""
    n = len(polygon_xy)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon_xy[i, 0], polygon_xy[i, 1]
        xj, yj = polygon_xy[j, 0], polygon_xy[j, 1]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi):
            inside = not inside
        j = i
    return inside
