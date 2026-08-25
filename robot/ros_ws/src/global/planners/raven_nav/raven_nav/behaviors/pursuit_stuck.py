"""Stuck detection + approach-variant cycling for target pursuit (ray/voxel).

Frontier exploration already blacklists viewpoints it can't reach
(frontier_behavior's [stuck] strikes). The pursuit modes had nothing: a
ray/BB approach waypoint the local planner can't reach left the drone parked
against an obstacle for the rest of the mission. The target is usually REAL —
only the chosen approach point is bad — so on a stuck strike the behaviors
try a DIFFERENT waypoint for the same target (lateral / higher / shorter
along the bearing for rays, rotated azimuth for voxel standoffs) instead of
abandoning it. Only after every variant fails does the target go on a TTL
cooldown (ray) or count as reached-as-close-as-possible (voxel, matching the
existing enveloped-approach convention).
"""
from __future__ import annotations

import numpy as np

# Same stuck heuristic as frontier_behavior so one tuning story covers both.
STUCK_DISTANCE_M = 0.3
STUCK_TIMEOUT_S = 5.0
# A waypoint that moved farther than this is a new approach — restart tracking.
REGROUP_RADIUS_M = 6.0


class PursuitStuckMonitor:
    """Fires one strike when the drone barely moves for STUCK_TIMEOUT_S while
    holding (roughly) the same pursuit waypoint. One instance in the node
    covers both pursuit modes; reset() on mode change / no waypoint."""

    def __init__(self, get_clock):
        self.get_clock = get_clock
        self._tracked_wp_xy = None
        self._last_motion_xy = None
        self._last_motion_time_s = None

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def reset(self) -> None:
        self._tracked_wp_xy = None
        self._last_motion_xy = None
        self._last_motion_time_s = None

    def update(self, cur_pose, waypoint) -> bool:
        """Feed one tick; True exactly when a stuck strike fires."""
        if cur_pose is None or waypoint is None:
            self.reset()
            return False
        now = self._now_s()
        cur_xy = np.asarray(cur_pose, dtype=float)[:2]
        wp_xy = np.asarray(waypoint, dtype=float)[:2]

        if (self._tracked_wp_xy is None
                or float(np.linalg.norm(wp_xy - self._tracked_wp_xy))
                > REGROUP_RADIUS_M):
            self._tracked_wp_xy = wp_xy.copy()
            self._last_motion_xy = cur_xy.copy()
            self._last_motion_time_s = now
            return False

        if float(np.linalg.norm(cur_xy - self._last_motion_xy)) \
                > STUCK_DISTANCE_M:
            self._last_motion_xy = cur_xy.copy()
            self._last_motion_time_s = now
            return False

        if now - self._last_motion_time_s > STUCK_TIMEOUT_S:
            # Re-arm so one stuck episode counts once, not every tick.
            self._last_motion_xy = cur_xy.copy()
            self._last_motion_time_s = now
            self._tracked_wp_xy = None
            return True
        return False


# ── ray approach variants ────────────────────────────────────────────────────
#
# Variant 0 is the canonical investigation pair (origin + dir*6 / *12).
# Later variants keep the same lead but move the approach: sidestep left/
# right of the bearing, climb, or stop short. Cycled one per stuck strike.
RAY_APPROACH_VARIANTS = 5
_LATERAL_M = 5.0
_CLIMB_M = 4.0


def ray_variant_waypoints(origin, dir_norm, variant: int):
    """(investigate_wp, continue_wp) for approach-variant `variant` of a ray
    lead. Pure geometry — altitude clamping is the caller's job."""
    o = np.asarray(origin, dtype=float)
    d = np.asarray(dir_norm, dtype=float)
    base1 = o + d * 6.0
    base2 = o + d * 12.0
    if variant <= 0:
        return base1, base2
    perp = np.array([-d[1], d[0], 0.0])
    n = float(np.linalg.norm(perp[:2]))
    perp = perp / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
    if variant == 1:
        off = perp * _LATERAL_M
    elif variant == 2:
        off = -perp * _LATERAL_M
    elif variant == 3:
        off = np.array([0.0, 0.0, _CLIMB_M])
    else:   # stop short of the usual investigation point
        return o + d * 3.0, o + d * 6.0
    return base1 + off, base2 + off


def rotate_dir_xy(d, angle_deg: float):
    """Rotate a 3D direction about +Z by angle_deg (z kept), renormalized."""
    d = np.asarray(d, dtype=float)
    a = np.deg2rad(angle_deg)
    c, s = float(np.cos(a)), float(np.sin(a))
    out = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
    n = float(np.linalg.norm(out))
    return out / n if n > 1e-6 else d
