"""Persistent detection reporting: what raven tells the rest of AirStack.

DEVIATION 4 from the OG paper logic: RAVEN's voxel behaviour kept clusters only
so it could fly to them, and its `visited_clusters` list was throwaway state.
The AirStack benchmark scores detections, so the clusters are also folded into
a persistent memory here, merged with `raven_nav.discoveries` (the same merge
`compile_results.py` re-runs offline) and published as
`/<robot>/raven_nav/confirmed_targets` + `/<robot>/raven_nav/discoveries`.

This is reporting only — nothing here steers the drone.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from raven_nav.discoveries import (
    ConfirmedTarget, merge_confirmed_targets,
)

# A target counts as visited once the drone passes within this of the box
# SURFACE (OG voxel_behavior.py:194 used the same 3 m against the standoff
# waypoint).
VISIT_REACH_M = 3.0
# rayfronts occasionally maps a cluster below ground; those are mapping errors,
# not targets.
MIN_BB_CENTER_Z = -0.5


def aabb_surface_dist(ca, sa, cb, sb) -> float:
    """Surface-to-surface gap between two AABBs (0 when they overlap). Pass a
    zero size to treat that argument as a point."""
    ca, sa, cb, sb = (np.asarray(v, dtype=float) for v in (ca, sa, cb, sb))
    gap = np.maximum(np.abs(ca[:3] - cb[:3]) - (sa[:3] + sb[:3]) / 2.0, 0.0)
    return float(np.linalg.norm(gap))


class DetectionMemory:
    """Accumulates voxel clusters into merged, status-carrying AABBs."""

    def __init__(self, min_confidence: float = 0.0,
                 visit_reach_m: float = VISIT_REACH_M) -> None:
        self.min_confidence = float(min_confidence)
        self.visit_reach_m = float(visit_reach_m)
        # Merged boxes, keyed by a coarse (label, centre) cell so a box that
        # drifts by a voxel is the same instance tick to tick.
        self._targets: Dict[Tuple, ConfirmedTarget] = {}
        # Visited geometry survives even after the live cluster disappears.
        self._visited: List[Tuple[str, np.ndarray, np.ndarray]] = []

    # ── keys / lookup ───────────────────────────────────────────────────────
    @staticmethod
    def _key(label: str, center) -> Tuple:
        c = np.round(np.asarray(center, dtype=float)[:3] / 2.0).astype(int)
        return (str(label), int(c[0]), int(c[1]), int(c[2]))

    def is_visited(self, label: str, center, size=None) -> bool:
        size = np.zeros(3) if size is None else size
        for vl, vc, vs in self._visited:
            if vl != label:
                continue
            if aabb_surface_dist(center, size, vc, vs) <= self.visit_reach_m:
                return True
        return False

    # ── updates ─────────────────────────────────────────────────────────────
    def update(self, clusters: Iterable, now_ts: float) -> List[ConfirmedTarget]:
        """Fold this tick's voxel clusters in and return the merged list."""
        fresh: List[ConfirmedTarget] = []
        for c in clusters:
            center = np.asarray(c.center, dtype=float)
            if float(center[2]) < MIN_BB_CENTER_Z:
                continue
            if float(c.confidence) < self.min_confidence:
                continue
            fresh.append(ConfirmedTarget(
                label=c.label, center=center,
                size=np.asarray(c.size, dtype=float),
                status='visited' if self.is_visited(c.label, center, c.size)
                       else 'observing',
                confidence=float(c.confidence), ts=float(now_ts)))
        for ct in merge_confirmed_targets(fresh):
            self._targets[self._key(ct.label, ct.center)] = ct
        # Re-stamp status on everything we remember, so a box first seen from
        # afar flips to visited once the drone gets there.
        for key, ct in list(self._targets.items()):
            if self.is_visited(ct.label, ct.center, ct.size):
                if ct.status != 'visited':
                    self._targets[key] = ConfirmedTarget(
                        label=ct.label, center=ct.center, size=ct.size,
                        status='visited', confidence=ct.confidence, ts=ct.ts)
        return self.confirmed_targets()

    def mark_visited(self, label: str, center, size) -> bool:
        """Called when the voxel behaviour finishes a cluster."""
        center = np.asarray(center, dtype=float)
        size = np.asarray(size, dtype=float)
        if self.is_visited(label, center, size):
            return False
        self._visited.append((str(label), center, size))
        return True

    def mark_reached(self, cur_pose) -> List[ConfirmedTarget]:
        """Passive arrival: any remembered box whose SURFACE the drone comes
        within `visit_reach_m` of flips to visited, even if the drone was doing
        something else (this is what keeps the frontier-only baseline
        producing visited detections)."""
        if cur_pose is None:
            return []
        p = np.asarray(cur_pose, dtype=float)[:3]
        flipped = []
        for ct in list(self._targets.values()):
            if str(ct.status).lower() == 'visited':
                continue
            if aabb_surface_dist(p, np.zeros(3), ct.center, ct.size) \
                    <= self.visit_reach_m:
                self.mark_visited(ct.label, ct.center, ct.size)
                flipped.append(ct)
        if flipped:
            for key, ct in list(self._targets.items()):
                if self.is_visited(ct.label, ct.center, ct.size):
                    self._targets[key] = ConfirmedTarget(
                        label=ct.label, center=ct.center, size=ct.size,
                        status='visited', confidence=ct.confidence, ts=ct.ts)
        return flipped

    # ── reads ───────────────────────────────────────────────────────────────
    def confirmed_targets(self) -> List[ConfirmedTarget]:
        live = list(self._targets.values())
        live_centers = [np.asarray(c.center, dtype=float) for c in live]
        out = list(live)
        for label, vc, vs in self._visited:
            if any(float(np.linalg.norm(vc - lc)) <= 3.0 for lc in live_centers):
                continue
            out.append(ConfirmedTarget(label=label, center=vc, size=vs,
                                       status='visited', confidence=1.0,
                                       ts=0.0))
        return merge_confirmed_targets(out)

    def completed_labels(self) -> List[str]:
        """Labels with at least one visited box — the `completed_targets`
        topic payload."""
        return sorted({ct.label for ct in self.confirmed_targets()
                       if str(ct.status).lower() == 'visited'})


class TargetEventLog:
    """First-discovery / first-confirmation / first-visit per instance.

    Timestamps are absolute sim seconds; `compile_results.py` reads
    `target_events[].{label,pos_enu,first_*_ts}`.
    """

    MATCH_DIST_M = 8.0

    def __init__(self) -> None:
        self.events: List[dict] = []

    def _match(self, label: str, pos_enu: np.ndarray) -> Optional[dict]:
        best, best_d = None, self.MATCH_DIST_M
        for ev in self.events:
            if ev['label'] != label:
                continue
            d = float(np.linalg.norm(np.asarray(ev['pos_enu']) - pos_enu))
            if d <= best_d:
                best, best_d = ev, d
        return best

    def update(self, discoveries, to_world, now: float,
               target_labels=()) -> List[Tuple[str, dict]]:
        """Returns the (milestone, event) pairs newly reached this tick, so the
        node can log DISCOVERED / CONFIRMED / VISITED lines."""
        targets = {str(t) for t in target_labels}
        milestones: List[Tuple[str, dict]] = []
        for d in discoveries:
            if targets and d.label not in targets:
                continue
            pos_enu = np.asarray(to_world(d.position), dtype=float)
            has_aabb = d.size is not None
            is_visited = has_aabb and str(d.status).lower() == 'visited'
            ev = self._match(d.label, pos_enu)
            if ev is None:
                ev = {'label': d.label, 'instance_id': d.instance_id,
                      'pos_enu': pos_enu.tolist(),
                      'first_discovered_ts': now,
                      'first_confirmed_ts': None,
                      'first_visited_ts': None}
                self.events.append(ev)
                milestones.append(('DISCOVERED', ev))
            elif has_aabb:
                ev['pos_enu'] = pos_enu.tolist()
                ev['instance_id'] = d.instance_id
            if has_aabb and ev['first_confirmed_ts'] is None:
                ev['first_confirmed_ts'] = now
                milestones.append(('CONFIRMED', ev))
            if is_visited and ev['first_visited_ts'] is None:
                ev['first_visited_ts'] = now
                milestones.append(('VISITED', ev))
        return milestones
