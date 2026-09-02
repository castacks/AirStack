"""Frontier-based exploration — numpy port of the original RAVEN behaviour.

Source: RayFronts_raven/rayfronts/behaviors/frontier_behavior.py

The OG pipeline, unchanged:
  frontiers (RDF) -> FLU -> drop z <= 1.5 -> DBSCAN(eps 2.7, min_samples 3)
  -> cluster centroids with z > 2.0 are the viewpoints
  -> score = distance, plus 5*(1 - cos) against the current motion vector when
     a waypoint already exists (momentum)
  -> uniformly random pick among the 5 best
  -> if unlocked: wp1 = viewpoint, wp2 = wp1 + 2*unit(wp1 - pose), lock
  -> unlock once the drone is within 5 m of wp1

`condition_check` is unconditionally True (OG line 16): frontier is the
fallback at the bottom of the priority chain.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np
from sklearn.cluster import DBSCAN

from raven_nav.behaviors.common import BehaviorOutput, TickContext, rdf_to_flu

# ── OG constants (frontier_behavior.py line numbers) ────────────────────────
FRONTIER_MIN_Z_M = 1.5          # OG:33 — raw frontier altitude floor
DBSCAN_EPS_M = 2.7              # OG:37
DBSCAN_MIN_SAMPLES = 3          # OG:37
VIEWPOINT_MIN_Z_M = 2.0         # OG:47 — centroid altitude floor
MOMENTUM_WEIGHT = 5.0           # OG:64
TOP_N = 5                       # OG:69
LEAD_DIST_M = 2.0               # OG:84 — wp2 sits this far past wp1
UNLOCK_RADIUS_M = 5.0           # OG:106


class FrontierBehavior:
    name = 'Frontier-based'

    def __init__(self, rng: Optional[np.random.Generator] = None) -> None:
        # OG used torch.randint over the top-5; a seedable Generator makes the
        # same choice reproducible in tests.
        self.rng = rng if rng is not None else np.random.default_rng()
        # Last tick's intermediates, for the node's viz/debug publishers.
        self.viewpoints: np.ndarray = np.zeros((0, 3))
        self.kept_frontiers: np.ndarray = np.zeros((0, 3))
        self.raw_frontiers_flu: np.ndarray = np.zeros((0, 3))
        self.last_scores: np.ndarray = np.zeros((0,))
        self.chosen_index: int = -1

    # OG:15-16 — frontier always fires.
    def condition_check(self, ctx: TickContext) -> bool:   # noqa: ARG002
        return True

    def compute_viewpoints(self, ctx: TickContext) -> np.ndarray:
        """OG:25-49 — frontier cloud to viewpoint centroids (FLU)."""
        self.raw_frontiers_flu = np.zeros((0, 3))
        self.kept_frontiers = np.zeros((0, 3))
        self.viewpoints = np.zeros((0, 3))
        fr = ctx.frontiers
        if fr is None or len(fr) == 0:
            return self.viewpoints
        flu = rdf_to_flu(np.asarray(fr, dtype=np.float64)[:, :3])
        self.raw_frontiers_flu = flu
        # OG:33 — "filter out frontier points that are under the height of
        # 1.5m". The floor is the mission's min_altitude_agl here (default
        # 1.5 = the OG literal); everything else is verbatim.
        kept = flu[flu[:, 2] > float(ctx.min_altitude)]
        self.kept_frontiers = kept
        if kept.shape[0] == 0:
            return self.viewpoints
        labels = DBSCAN(eps=DBSCAN_EPS_M,
                        min_samples=DBSCAN_MIN_SAMPLES).fit(kept).labels_
        vps: List[np.ndarray] = []
        for l in sorted({int(x) for x in labels} - {-1}):
            centroid = kept[labels == l].mean(axis=0)
            if centroid[2] > VIEWPOINT_MIN_Z_M:      # OG:47
                # DEVIATION 2 — outside the mission polygon is not ours to fly.
                if ctx.inside_area(centroid):
                    vps.append(centroid)
        self.viewpoints = (np.stack(vps) if vps else np.zeros((0, 3)))
        return self.viewpoints

    def execute(self, ctx: TickContext) -> BehaviorOutput:
        out = BehaviorOutput(waypoint_locked=ctx.waypoint_locked,
                             target_waypoint=ctx.target_waypoint,
                             target_waypoint2=ctx.target_waypoint2)
        vps = self.compute_viewpoints(ctx)
        self.chosen_index = -1
        if vps.shape[0] == 0:
            # OG would have raised on torch.stack([]) here; publishing nothing
            # and holding the previous waypoint is the sane equivalent.
            out.note = 'no viewpoints'
            return out

        pose = np.asarray(ctx.cur_pose, dtype=np.float64).reshape(3)
        distances = np.linalg.norm(vps - pose[None, :], axis=1)
        if ctx.target_waypoint is not None:
            # OG:57-65 — momentum: prefer viewpoints in the direction we are
            # already flying.
            motion = np.asarray(ctx.target_waypoint, dtype=np.float64) - pose
            motion = motion / (np.linalg.norm(motion) + 1e-6)
            cand = vps - pose[None, :]
            cand = cand / (np.linalg.norm(cand, axis=1, keepdims=True) + 1e-6)
            scores = distances + MOMENTUM_WEIGHT * (1.0 - cand @ motion)
        else:
            scores = distances
        self.last_scores = scores

        n = int(min(TOP_N, vps.shape[0]))                        # OG:69-72
        top = np.argsort(scores, kind='stable')[:n]
        best_idx = int(top[int(self.rng.integers(0, n))])
        self.chosen_index = best_idx
        best = vps[best_idx]

        wp1 = out.target_waypoint
        wp2 = out.target_waypoint2
        if not out.waypoint_locked:                              # OG:79-85
            wp1 = ctx.clamp(best)
            d = wp1 - pose
            nrm = float(np.linalg.norm(d))
            d = d / nrm if nrm > 1e-9 else np.array([1.0, 0.0, 0.0])
            wp2 = ctx.clamp(wp1 + LEAD_DIST_M * d)
            out.waypoint_locked = True
        if wp1 is None or wp2 is None:
            out.note = 'no waypoint'
            return out

        out.target_waypoint = np.asarray(wp1, dtype=np.float64)
        out.target_waypoint2 = np.asarray(wp2, dtype=np.float64)
        out.path = [out.target_waypoint, out.target_waypoint2]   # OG:87-103

        if float(np.linalg.norm(pose - out.target_waypoint)) < UNLOCK_RADIUS_M:
            out.waypoint_locked = False                          # OG:106-107
        return out

    def frontier_table(self, ctx: TickContext) -> str:
        """Human-readable dump for /<robot>/debug/frontier_table."""
        lines = [
            f'frontiers raw={self.raw_frontiers_flu.shape[0]} '
            f'kept(z>{ctx.min_altitude:g})={self.kept_frontiers.shape[0]} '
            f'viewpoints={self.viewpoints.shape[0]}'
        ]
        if self.viewpoints.shape[0]:
            lines.append(f'{"#":>3} {"x":>8} {"y":>8} {"z":>7} {"score":>9}')
            order = np.argsort(self.last_scores, kind='stable') \
                if self.last_scores.shape[0] == self.viewpoints.shape[0] \
                else np.arange(self.viewpoints.shape[0])
            for rank, i in enumerate(order[:20]):
                v = self.viewpoints[int(i)]
                s = (float(self.last_scores[int(i)])
                     if self.last_scores.shape[0] > int(i) else float('nan'))
                mark = '*' if int(i) == self.chosen_index else ' '
                lines.append(f'{rank:>3} {v[0]:8.1f} {v[1]:8.1f} {v[2]:7.1f} '
                             f'{s:9.2f}{mark}')
        return '\n'.join(lines)
