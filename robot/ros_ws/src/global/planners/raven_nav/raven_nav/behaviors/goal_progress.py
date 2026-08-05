"""No-progress detection for goal selection, and a cooldown on the region.

``PursuitStuckMonitor`` catches a drone *parked*: barely moving (0.3 m) for a few
seconds while holding one waypoint. It does not catch the other way a search
stalls — the drone keeps flying, keeps picking new waypoints, and gets nowhere,
because every goal it picks is a few metres away.

Observed in both stacks. VLFM's explore cost is
``dist + peer + novelty - value_weight * sim`` with ``value_weight`` 300 against a
distance term in raw metres, so a high-similarity ray five metres away outscores
every distant frontier and is re-picked indefinitely; one robot logged the same
``-> (5,-2,8)`` for ~250 consecutive ticks. raven's ray bid is ``-distance``,
nearest-first by design, so a lead that materialises on top of the drone
automatically outbids every real target and stays committed — one robot ended its
window holding a target 3.2 m away with 30 targets unassigned. In both cases the
drone drifted 6-70 m over a full 15-minute window while its peers covered
hundreds.

The shared symptom is what this measures: **net displacement, not per-tick
motion**. A robot that has not left a ``PROGRESS_RADIUS_M`` circle in
``PROGRESS_TIMEOUT_S`` is not searching, whatever its instantaneous speed. The
goal region it kept choosing goes on a TTL cooldown so selection has to look
elsewhere; the cooldown expires, so a genuinely good region is only deferred, not
abandoned.

Thresholds are deliberately far outside normal operation: healthy robots in this
sweep flew a median ~43 m per goal and 200-300 m per window, so 15 m of net
displacement in 60 s cannot describe one. That is the same 15 m criterion the
offline ``trajectory_progress.py`` uses to flag a run, and across 264 robot-runs
it never once flagged a robot that was actually searching.
"""
from __future__ import annotations

import numpy as np

# Net displacement over PROGRESS_TIMEOUT_S below which the robot is not
# searching, whatever its instantaneous speed.
PROGRESS_RADIUS_M = 15.0
PROGRESS_TIMEOUT_S = 60.0
# Cooldown footprint and duration. The radius covers the neighbourhood the drone
# has been circling, not just the exact point, because the selected goal jitters
# by a few metres each tick while describing the same place.
COOLDOWN_RADIUS_M = 20.0
COOLDOWN_S = 120.0


class GoalProgressMonitor:
    """Watches (position, chosen goal) and cools down goal regions that are not
    producing travel. One instance per behaviour that selects goals."""

    def __init__(self, get_clock,
                 progress_radius_m: float = PROGRESS_RADIUS_M,
                 progress_timeout_s: float = PROGRESS_TIMEOUT_S,
                 cooldown_radius_m: float = COOLDOWN_RADIUS_M,
                 cooldown_s: float = COOLDOWN_S):
        self.get_clock = get_clock
        self.progress_radius_m = float(progress_radius_m)
        self.progress_timeout_s = float(progress_timeout_s)
        self.cooldown_radius_m = float(cooldown_radius_m)
        self.cooldown_s = float(cooldown_s)
        self._anchor_xy = None          # where the no-progress window started
        self._anchor_time_s = None
        self._blocked = []              # [(xy, expiry_s)]

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def reset(self) -> None:
        self._anchor_xy = None
        self._anchor_time_s = None

    def clear(self) -> None:
        """Reset progress tracking *and* drop every cooldown."""
        self.reset()
        self._blocked = []

    def _expire(self, now: float) -> None:
        self._blocked = [b for b in self._blocked if b[1] > now]

    def update(self, cur_pose, goal_xyz):
        """Feed one tick. Returns the cooled-down centre when a region is put on
        cooldown this tick, else None."""
        if cur_pose is None or goal_xyz is None:
            self.reset()
            return None
        now = self._now_s()
        self._expire(now)
        cur_xy = np.asarray(cur_pose, dtype=float)[:2]

        if self._anchor_xy is None:
            self._anchor_xy = cur_xy.copy()
            self._anchor_time_s = now
            return None

        # Left the circle -> real travel happened, restart the window.
        if float(np.linalg.norm(cur_xy - self._anchor_xy)) > self.progress_radius_m:
            self._anchor_xy = cur_xy.copy()
            self._anchor_time_s = now
            return None

        if now - self._anchor_time_s <= self.progress_timeout_s:
            return None

        centre = np.asarray(goal_xyz, dtype=float)[:2].copy()
        self._blocked.append((centre, now + self.cooldown_s))
        # Re-arm so one stall produces one cooldown, not one per tick.
        self._anchor_xy = cur_xy.copy()
        self._anchor_time_s = now
        return centre

    def cool_down(self, xy) -> bool:
        """Put a region on cooldown now, without waiting out the stall timer.

        For callers that already know a place is spent — a lead the drone has
        reached, say — where waiting PROGRESS_TIMEOUT_S to discover it would
        waste most of a minute. Returns True if this opened a new cooldown,
        False if it extended one that already covered the point."""
        now = self._now_s()
        self._expire(now)
        c = np.asarray(xy, dtype=float)[:2].copy()
        for i, (cc, expiry) in enumerate(self._blocked):
            if float(np.linalg.norm(c - cc)) <= self.cooldown_radius_m:
                self._blocked[i] = (cc, max(expiry, now + self.cooldown_s))
                return False
        self._blocked.append((c, now + self.cooldown_s))
        return True

    def is_blocked(self, xy) -> bool:
        """Is this single point inside a live cooldown?"""
        now = self._now_s()
        self._expire(now)
        p = np.asarray(xy, dtype=float)[:2]
        for c, _ in self._blocked:
            if float(np.linalg.norm(p - c)) <= self.cooldown_radius_m:
                return True
        return False

    def blocked_mask(self, xys) -> np.ndarray:
        """Boolean mask over an (N, >=2) array: True where a point is cooled down."""
        pts = np.asarray(xys, dtype=float)
        out = np.zeros(pts.shape[0], dtype=bool)
        if not self._blocked or pts.shape[0] == 0:
            return out
        now = self._now_s()
        self._expire(now)
        for c, _ in self._blocked:
            out |= np.linalg.norm(pts[:, :2] - c, axis=1) <= self.cooldown_radius_m
        return out

    @property
    def n_blocked(self) -> int:
        self._expire(self._now_s())
        return len(self._blocked)
