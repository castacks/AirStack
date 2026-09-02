"""Frontier-based exploration — numpy port of the original RAVEN behaviour.

Source: RayFronts_raven/rayfronts/behaviors/frontier_behavior.py

The OG pipeline, unchanged:
  frontiers (RDF) -> FLU -> drop z <= 1.5 -> DBSCAN(eps 2.7, min_samples 3)
  -> cluster centroids with z > 2.0 are the viewpoints
  -> score = distance, plus 5*(1 - cos) against the current motion vector when
     a waypoint already exists (momentum), plus the anti-revisit novelty
     penalty below
  -> uniformly random pick among the 5 best
  -> if unlocked: wp1 = viewpoint, wp2 = wp1 + 2*unit(wp1 - pose), lock
  -> unlock once the drone is within 5 m of wp1

`condition_check` is unconditionally True (OG line 16): frontier is the
fallback at the bottom of the priority chain.

Anti-revisit
------------
`distance + momentum` alone makes the drone re-fly ground it has already
cleared: a frontier centroid sitting in the middle of covered terrain is
*near*, so it keeps winning. The pre-rewrite multi-robot node solved this with
a novelty penalty and it is re-enabled here in its SINGLE-AGENT form (no peer
repulsion, no peer completed-zones, no stuck blacklist) — see
`git show 35ce9e93:.../behaviors/frontier_behavior.py`, constants at OG2:82-83,
`_neighborhood_density` at OG2:126-146, application at OG2:556-564.
"""
from __future__ import annotations

from typing import List, Optional, Set, Tuple

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

# ── anti-revisit constants, lifted verbatim from the pre-rewrite node ───────
# (35ce9e93 frontier_behavior.py:82-83 — cited below as OG2).
# A viewpoint whose whole neighbourhood is already observed is scored as if it
# were 100 m further away than it is; untouched ground pays nothing. 100 m is
# deliberately larger than any single frontier hop, so a fresh viewpoint beats
# a stale one at any distance the DBSCAN stage can produce, while ties among
# equally-stale candidates are still broken by distance + momentum.
# DEFAULT 0.0 = OFF: the user explicitly wants frontier REVISITS allowed
# (2026-09-02, "it can revisit frontiers, that's fine") — only TARGET
# revisits are excluded (ray_behavior's visited-AABB filter). Set env
# RAVEN_FRONTIER_NOVELTY_W (e.g. 100.0 = OG2:82's weight) to re-enable the
# coverage-novelty bias.
import os as _os
try:
    NOVELTY_WEIGHT = float(_os.getenv('RAVEN_FRONTIER_NOVELTY_W', '') or 0.0)
except ValueError:
    NOVELTY_WEIGHT = 0.0
# Per-hop ALTITUDE CAP for frontier waypoints. The centroid's own z is
# still the destination — reachable over several hops — but one hop may
# move z by at most this much, so exploration stops lurching across the
# whole altitude band (user, live 2026-09-02: urban targets sit at all
# altitudes, so no altitude HOLD — just no abrupt changes). 0 disables
# the cap (OG parity: OG flew straight to the centroid's z).
try:
    FRONTIER_MAX_DZ_M = float(
        _os.getenv('RAVEN_FRONTIER_MAX_DZ_M', '') or 2.0)
except ValueError:
    FRONTIER_MAX_DZ_M = 2.0
NOVELTY_NEIGHBORHOOD_CELLS = 5  # OG2:83 — k, an 11x11-cell (5.5 m) window at 0.5 m


def neighborhood_observed_fraction(pts_xy: np.ndarray,
                                   cells: Optional[Set[Tuple[int, int]]],
                                   cell_size_m: float, k: int) -> np.ndarray:
    """Fraction of the (2k+1)x(2k+1) cell window around each point that is in
    `cells`. 0 in fully novel areas, 1 fully observed.

    Verbatim port of OG2:126-146 `_neighborhood_density` (renamed: "density"
    read as a point density, which it is not). `cells` is the coverage
    tracker's live set of (ix, iy) tuples — see `TickContext.observed_cells`
    for the aliasing contract.
    """
    pts_xy = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    n = pts_xy.shape[0]
    if n == 0 or not cells:
        return np.zeros(n, dtype=np.float64)
    cx = np.floor(pts_xy[:, 0] / cell_size_m).astype(np.int64)
    cy = np.floor(pts_xy[:, 1] / cell_size_m).astype(np.int64)
    side = 2 * k + 1
    total = float(side * side)
    out = np.zeros(n, dtype=np.float64)
    for i in range(n):
        c = 0
        x0, y0 = int(cx[i]), int(cy[i])
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                if (x0 + dx, y0 + dy) in cells:
                    c += 1
        out[i] = c / total
    return out


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
        self.last_novelty: np.ndarray = np.zeros((0,))
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
        # 1.5 = the OG literal).
        #
        # DEVIATION 3 — the OG had a floor only. A ceiling is added so the
        # mission's altitude band is a real band on BOTH sides: a frontier
        # 40 m up is not a place this drone may fly, and keeping it only to
        # have clamp_z() squash the waypoint back to max_altitude produced
        # viewpoints whose z bore no relation to the cluster they came from.
        # ctx.max_altitude defaults to 100.0, so this is a no-op unless the
        # mission sets max_altitude_agl.
        kept = flu[(flu[:, 2] > float(ctx.min_altitude))
                   & (flu[:, 2] < float(ctx.max_altitude))]
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
        self.last_novelty = np.zeros((0,))
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

        # OG2:556-564 — anti-revisit. Penalty is on the XY footprint only:
        # coverage is a 2D plate, and two viewpoints over the same block at
        # different altitudes are the same ground.
        novelty = NOVELTY_WEIGHT * neighborhood_observed_fraction(
            vps[:, :2], ctx.observed_cells, ctx.coverage_cell_size_m,
            NOVELTY_NEIGHBORHOOD_CELLS)
        self.last_novelty = novelty
        scores = scores + novelty
        self.last_scores = scores

        n = int(min(TOP_N, vps.shape[0]))                        # OG:69-72
        top = np.argsort(scores, kind='stable')[:n]
        best_idx = int(top[int(self.rng.integers(0, n))])
        self.chosen_index = best_idx
        best = vps[best_idx]

        wp1 = out.target_waypoint
        wp2 = out.target_waypoint2
        if not out.waypoint_locked:                              # OG:79-85
            if FRONTIER_MAX_DZ_M > 0.0:
                # deviation 5: rate-limit the vertical component per hop
                # (see FRONTIER_MAX_DZ_M above).
                zlo = float(pose[2]) - FRONTIER_MAX_DZ_M
                zhi = float(pose[2]) + FRONTIER_MAX_DZ_M
                best = np.array(
                    [best[0], best[1], float(np.clip(best[2], zlo, zhi))])
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
        n_obs = len(ctx.observed_cells) if ctx.observed_cells else 0
        lines = [
            f'frontiers raw={self.raw_frontiers_flu.shape[0]} '
            f'kept({ctx.min_altitude:g}<z<{ctx.max_altitude:g})'
            f'={self.kept_frontiers.shape[0]} '
            f'viewpoints={self.viewpoints.shape[0]}',
            f'novelty: observed_cells={n_obs} '
            f'cell={ctx.coverage_cell_size_m:g}m '
            f'k={NOVELTY_NEIGHBORHOOD_CELLS} weight={NOVELTY_WEIGHT:g}',
        ]
        if self.viewpoints.shape[0]:
            lines.append(f'{"#":>3} {"x":>8} {"y":>8} {"z":>7} '
                         f'{"novelty":>9} {"score":>9}')
            order = np.argsort(self.last_scores, kind='stable') \
                if self.last_scores.shape[0] == self.viewpoints.shape[0] \
                else np.arange(self.viewpoints.shape[0])
            for rank, i in enumerate(order[:20]):
                v = self.viewpoints[int(i)]
                s = (float(self.last_scores[int(i)])
                     if self.last_scores.shape[0] > int(i) else float('nan'))
                # +0 = untouched ground, +100 = fully re-covered.
                nov = (float(self.last_novelty[int(i)])
                       if self.last_novelty.shape[0] > int(i) else float('nan'))
                mark = '*' if int(i) == self.chosen_index else ' '
                lines.append(f'{rank:>3} {v[0]:8.1f} {v[1]:8.1f} {v[2]:7.1f} '
                             f'{nov:9.2f} {s:9.2f}{mark}')
        return '\n'.join(lines)
