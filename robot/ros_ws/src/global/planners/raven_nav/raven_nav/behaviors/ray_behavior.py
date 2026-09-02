"""Ray-based search — numpy port of the original RAVEN behaviour.

Source: RayFronts_raven/rayfronts/behaviors/ray_behavior.py

The OG pipeline, unchanged:
  rays whose softmax score on ANY target column exceeds 0.95
  -> keep only rays whose (origin + unit xy dir) target lies AHEAD of the drone
  -> greedy 45-degree XY angle binning (incremental centroid, first fit wins)
  -> per group: mean origin, normalised mean direction, density = ray count
  -> best group = argmin( |mean_origin - pose| - 5 * density )
  -> wp1 = origin + 6*dir, wp2 = origin + 12*dir
  -> unlock once the drone is within 4 m of wp2

Two documented departures from the OG source:

* the RDF -> FLU flip happens in `raven_nav_node._ray_all_cb` (same formula)
  rather than here, because rays reach us over a PointCloud2 instead of out of
  the mapper's tensors;
* OG:85 applied the forward-filter mask to `xy_dirs_np_normed` only, then
  indexed the UNFILTERED `orig_world`/`dir_world` with the filtered group
  indices (`OG:116-117`). That is an off-by-selection bug: with any ray
  filtered out the group averages are computed from the wrong rays. Here the
  mask is applied to all three arrays, which is what the code plainly meant.
  `test_og_parity.py` pins the two implementations together on inputs where no
  ray is filtered, so the bug is the only difference.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from raven_nav.behaviors.common import BehaviorOutput, TickContext
from raven_nav.ray_groups import RayGroup

# ── OG constants (ray_behavior.py line numbers) ─────────────────────────────
RAY_SCORE_THRESHOLD = 0.95      # OG:42 — exposed as the `score_threshold` param
ANGLE_BIN_DEG = 45.0            # OG:90
MIN_RAYS_PER_GROUP = 1          # OG:110
DENSITY_WEIGHT = 5.0            # OG:128 (`k`)
MAGNITUDE_M = 6.0               # OG:139 — wp1 = origin + 6*dir, wp2 = +12*dir
UNLOCK_RADIUS_M = 4.0           # OG:187
# Padding (m) added to each face of a visited AABB for the ray exclusion; a
# BB and a ray bearing are both estimates, so a graze must still count.
VISITED_RAY_PAD_M = 3.0


@dataclass
class RayAnalysis:
    """Everything the OG `condition_check` + the front half of `execute`
    derive from one ray cloud. Recomputed every tick by the node so passive
    reporting (discoveries) works even when ray mode never fires."""

    orig: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    dirs: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    scores: np.ndarray = field(default_factory=lambda: np.zeros((0,)))
    labels: List[str] = field(default_factory=list)
    groups: List[Dict] = field(default_factory=list)
    averages: List[tuple] = field(default_factory=list)   # (origin, dir, density)
    order: List[int] = field(default_factory=list)        # groups, best first

    @property
    def has_rays(self) -> bool:
        return self.orig.shape[0] > 0


def angle_bin_groups(xy_dirs: np.ndarray,
                     angle_deg: float = ANGLE_BIN_DEG) -> List[Dict]:
    """OG:87-111, verbatim: greedy first-fit binning of unit XY directions.

    Each group keeps a running mean of its member directions as the centroid;
    a direction joins the first group whose centroid is within `angle_deg`.
    Order-sensitive by construction — that is the OG behaviour.
    """
    thresh = float(np.cos(np.deg2rad(angle_deg)))
    groups: List[Dict] = []
    for i, xy_dir in enumerate(np.asarray(xy_dirs, dtype=np.float64)):
        assigned = False
        for g in groups:
            if float(np.dot(xy_dir, g['centroid'])) >= thresh:
                g['indices'].append(i)
                g['rays'].append(xy_dir)
                c = np.mean(g['rays'], axis=0)
                g['centroid'] = c / np.linalg.norm(c)
                assigned = True
                break
        if not assigned:
            groups.append({'centroid': xy_dir.copy(),
                           'rays': [xy_dir], 'indices': [i]})
    return [g for g in groups if len(g['rays']) >= MIN_RAYS_PER_GROUP]


class RayBehavior:
    name = 'Ray-based'

    def __init__(self, score_threshold: float = RAY_SCORE_THRESHOLD) -> None:
        self.score_threshold = float(score_threshold)
        self.analysis = RayAnalysis()
        self.current_target: str = ''

    # ── analysis ────────────────────────────────────────────────────────────
    def analyse(self, ctx: TickContext,
                columns: Optional[List[int]] = None,
                threshold: Optional[float] = None) -> RayAnalysis:
        """OG:16-54 (`condition_check`) + OG:61-125 (grouping).

        `columns` defaults to the target columns; the LVLM behaviour reuses
        this with its guiding columns and its own (lower) threshold.
        """
        thr = self.score_threshold if threshold is None else float(threshold)
        a = RayAnalysis()
        o, d, s = ctx.ray_origins, ctx.ray_dirs, ctx.ray_scores
        if o is None or d is None or s is None or len(o) == 0:
            return a
        cols = ctx.target_columns() if columns is None else list(columns)
        if not cols:
            return a
        s = np.asarray(s, dtype=np.float64)
        if s.ndim != 2 or max(cols) >= s.shape[1]:
            return a
        rel = s[:, cols]
        # SUMMED positive mass across the target columns (extends OG:44-46
        # to multi-positive queries — near-synonym positives split the
        # softmax at true targets; see voxel_behavior.detect for the live
        # measurement). Single target: sum == the column, OG unchanged.
        keep = np.nonzero(rel.sum(axis=1) > thr)[0]
        if keep.size == 0:
            return a

        orig = np.asarray(o, dtype=np.float64)[keep]
        dirs = np.asarray(d, dtype=np.float64)[keep]
        best_rel = np.argmax(rel[keep], axis=1)
        labels = [str(ctx.query_labels[cols[int(b)]]) for b in best_rel]
        scores = rel[keep][np.arange(keep.size), best_rel]

        xy = dirs[:, :2]
        nrm = np.linalg.norm(xy, axis=1, keepdims=True)
        xy_unit = xy / np.where(nrm > 1e-12, nrm, 1.0)

        # OG:74-85 — drop rays whose target lies behind the drone in XY.
        cur_xy = np.asarray(ctx.cur_pose, dtype=np.float64)[:2]
        to_target = (orig[:, :2] + xy_unit) - cur_xy[None, :]
        forward = np.einsum('ij,ij->i', xy_unit, to_target) > 0
        if not np.any(forward):
            return a
        orig, dirs, xy_unit = orig[forward], dirs[forward], xy_unit[forward]
        labels = [l for l, f in zip(labels, forward) if f]
        scores = scores[forward]

        # TARGET ANTI-REVISIT (single-agent port of the old ray exclusion):
        # drop rays that enter a VISITED target's AABB (padded) — those rays
        # are the already-serviced person still lighting up the encoder.
        if ctx.visited_bbs:
            from raven_nav.ray_targets import ray_aabb_hits
            keep2 = np.ones(orig.shape[0], dtype=bool)
            for bb in ctx.visited_bbs:
                bb = np.asarray(bb, dtype=float)
                pad = bb.copy()
                pad[3:6] = pad[3:6] + 2.0 * VISITED_RAY_PAD_M
                for i in range(orig.shape[0]):
                    if keep2[i] and ray_aabb_hits(orig[i], dirs[i], pad)[0]:
                        keep2[i] = False
            if not np.all(keep2):
                orig, dirs, xy_unit = orig[keep2], dirs[keep2], xy_unit[keep2]
                labels = [l for l, k in zip(labels, keep2) if k]
                scores = scores[keep2]
            if orig.shape[0] == 0:
                return a

        groups = angle_bin_groups(xy_unit)
        averages = []
        for g in groups:                                   # OG:113-125
            idx = g['indices']
            avg_o = orig[idx].mean(axis=0)
            avg_d = dirs[idx].mean(axis=0)
            avg_d = avg_d / (np.linalg.norm(avg_d) + 1e-12)
            averages.append((avg_o, avg_d, len(g['rays'])))
        pose = np.asarray(ctx.cur_pose, dtype=np.float64)
        cost = [float(np.linalg.norm(av[0] - pose)) - DENSITY_WEIGHT * av[2]
                for av in averages]                        # OG:128-129
        order = list(np.argsort(np.asarray(cost), kind='stable'))

        a.orig, a.dirs, a.scores = orig, dirs, scores
        a.labels, a.groups, a.averages = labels, groups, averages
        a.order = [int(i) for i in order]
        return a

    def update(self, ctx: TickContext) -> RayAnalysis:
        """Recompute and cache. Called once per tick by the node."""
        self.analysis = self.analyse(ctx)
        return self.analysis

    def condition_check(self, ctx: TickContext) -> bool:   # noqa: ARG002
        """OG:16-54 — fire when at least one ray beats the threshold."""
        return self.analysis.has_rays and bool(self.analysis.groups)

    # ── execution ───────────────────────────────────────────────────────────
    def execute(self, ctx: TickContext) -> BehaviorOutput:
        out = BehaviorOutput(waypoint_locked=ctx.waypoint_locked,
                             target_waypoint=ctx.target_waypoint,
                             target_waypoint2=ctx.target_waypoint2)
        a = self.analysis
        if not a.averages:
            out.note = 'no ray groups'                     # OG:132-135
            return out
        pose = np.asarray(ctx.cur_pose, dtype=np.float64)
        for gi in a.order:
            origin, direction, _density = a.averages[gi]
            direction = direction / (np.linalg.norm(direction) + 1e-12)
            wp1 = ctx.clamp(origin + direction * MAGNITUDE_M)
            wp2 = ctx.clamp(origin + direction * MAGNITUDE_M * 2.0)
            # DEVIATION 2 — never route outside the mission polygon.
            if not (ctx.inside_area(wp1) and ctx.inside_area(wp2)):
                continue
            out.target_waypoint = wp1
            out.target_waypoint2 = wp2
            out.path = [wp1, wp2]                          # OG:162-181
            self.current_target = self._group_label(gi)
            if float(np.linalg.norm(pose - wp2)) < UNLOCK_RADIUS_M:
                out.waypoint_locked = False                # OG:187-188
            return out
        out.note = 'all ray groups outside search_area'
        return out

    def _group_label(self, gi: int) -> str:
        idx = self.analysis.groups[gi]['indices']
        counts: Dict[str, int] = {}
        for i in idx:
            lbl = self.analysis.labels[i]
            counts[lbl] = counts.get(lbl, 0) + 1
        return max(counts, key=lambda k: (counts[k], k)) if counts else ''

    # ── reporting ───────────────────────────────────────────────────────────
    def ray_groups(self) -> List[RayGroup]:
        """The OG angle groups as `RayGroup`s, for ray_targets/discoveries."""
        a = self.analysis
        out: List[RayGroup] = []
        for gi, g in enumerate(a.groups):
            idx = g['indices']
            rg = RayGroup(label=self._group_label(gi),
                          ray_origins=a.orig[idx],
                          ray_dirs=a.dirs[idx],
                          ray_scores=a.scores[idx])
            rg.finalize(np.zeros(3))
            out.append(rg)
        return out

    def arrows(self):
        """(origins, dirs, group_id) triple for the filtered_rays MarkerArray."""
        a = self.analysis
        gid = np.full(a.orig.shape[0], -1, dtype=int)
        for i, g in enumerate(a.groups):
            for j in g['indices']:
                gid[j] = i
        return a.orig, a.dirs, gid

    def group_table(self) -> str:
        a = self.analysis
        lines = [f'ray groups={len(a.groups)} rays_kept={a.orig.shape[0]} '
                 f'threshold={self.score_threshold:g}']
        if a.averages:
            lines.append(f'{"rank":>4} {"grp":>3} {"label":<18} {"n":>4} '
                         f'{"ox":>8} {"oy":>8} {"oz":>7} {"dx":>6} {"dy":>6}')
            for rank, gi in enumerate(a.order):
                o, d, n = a.averages[gi]
                lines.append(f'{rank:>4} {gi:>3} {self._group_label(gi):<18} '
                             f'{n:>4} {o[0]:8.1f} {o[1]:8.1f} {o[2]:7.1f} '
                             f'{d[0]:6.2f} {d[1]:6.2f}')
        return '\n'.join(lines)
