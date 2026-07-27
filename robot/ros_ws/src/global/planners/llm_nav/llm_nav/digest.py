"""Map -> text: turn RayFronts output into an LLM-readable digest.

This module is the "tokenizer" of the semantic map. It is pure numpy/scipy
(no ROS imports) so it can be unit-tested outside the container:

- voxel instances: per-voxel argmax over the label bank, connected-component
  clustering per object label (26-conn on the 0.5 m grid, same as raven's
  voxel_behavior), tracked across ticks so IDs are stable (commitments
  reference them);
- ray groups: rays bucketed by (azimuth sector, top-1 label), each group
  geometrically associated to the instance its line passes near — or marked
  as pointing beyond the mapped area;
- rendering: compact text lines with top-k labels + scores, sizes in meters,
  and direction/distance from the robot. Scores are raw cos-sims displayed
  x100 (rayfronts runs with compute_prob=False here).

All numeric knobs are presentation/geometry parameters, not detection
thresholds — the LLM judges instances from size + labels + scores together.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .mission_log import log_call

# rayfronts low_memory.yaml mapping.vox_size — must match the deployed config.
VOX_SIZE = 0.5
# Largest dense grid we will run connected-components on (cells).
_MAX_GRID_CELLS = 60_000_000
# Cross-tick instance association: base margin (m); the effective margin grows
# with the instance's footprint because a partially-mapped object's centroid
# shifts as new faces are observed (a 12 m house can move its centroid >3 m
# between ticks while "growing").
_TRACK_MATCH_M = 3.0
_TRACK_MATCH_EXTENT_FRAC = 0.6
_TRACK_MATCH_CAP_M = 10.0
_TRACK_MAX_MISSES = 6
# Segmentation is VOCABULARY-FREE: adjacent voxels join a cluster when their
# per-query score VECTORS (a projection of the open-set embedding onto the
# query directions) are cosine-similar — two voxels of one object look alike
# in embedding space no matter which single label wins either of them.
_AFFINITY_TAU = 0.90
# Description is BASELINE-RELATIVE: a cluster is "known" as label L only if
# its mean score for L beats L's own map-wide average by this ratio —
# otherwise it is an honest 'unknown'. (Raw argmax force-classifies
# out-of-vocabulary structures into whichever label has the highest ambient
# baseline — the house-sized "bus stop".)
_KNOWN_RATIO = 1.2

_COMPASS = ['E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE']


def compass_of(dx: float, dy: float) -> str:
    """8-way compass name for a map-frame XY direction (x=East, y=North)."""
    ang = math.degrees(math.atan2(dy, dx)) % 360.0
    return _COMPASS[int(((ang + 22.5) % 360.0) // 45.0)]


def point_in_polygon_xy(x: float, y: float, poly: list) -> bool:
    """Ray-cast point-in-polygon in XY. <3 vertices => unbounded (always inside)."""
    n = len(poly)
    if n < 3:
        return True
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi):
            inside = not inside
        j = i
    return inside


def nearest_point_in_polygon_xy(x: float, y: float, poly: list,
                                inset_m: float = 1.0) -> tuple:
    """Closest boundary point to (x, y), nudged inset_m toward the centroid."""
    n = len(poly)
    if n < 3:
        return x, y
    best = (x, y)
    best_d2 = float('inf')
    for i in range(n):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        ex, ey = bx - ax, by - ay
        seg2 = ex * ex + ey * ey
        t = 0.0 if seg2 < 1e-9 else ((x - ax) * ex + (y - ay) * ey) / seg2
        t = max(0.0, min(1.0, t))
        px, py = ax + t * ex, ay + t * ey
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (px, py)
    cx = sum(p[0] for p in poly) / n
    cy = sum(p[1] for p in poly) / n
    dx, dy = cx - best[0], cy - best[1]
    d = (dx * dx + dy * dy) ** 0.5
    if d > 1e-6:
        f = min(inset_m, d) / d
        best = (best[0] + dx * f, best[1] + dy * f)
    return best


def aabb_surface_distance(p: np.ndarray, bbox_min: np.ndarray,
                          bbox_max: np.ndarray) -> float:
    """Distance from point p to the box surface (0 if inside)."""
    clamped = np.minimum(np.maximum(p, bbox_min), bbox_max)
    return float(np.linalg.norm(p - clamped))


def standoff_point(robot: np.ndarray, centroid: np.ndarray,
                   bbox_min: np.ndarray, bbox_max: np.ndarray,
                   standoff_m: float) -> np.ndarray:
    """Point standoff_m outside the instance AABB, on the robot's side.

    Uses the AABB support distance along the centroid->robot direction, so the
    waypoint clears the box face regardless of approach angle.
    """
    d = robot - centroid
    n = np.linalg.norm(d)
    d = d / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
    half = (bbox_max - bbox_min) / 2.0
    support = float(np.abs(d) @ half)
    return centroid + d * (support + standoff_m)


@dataclass
class Instance:
    id: str
    label: str
    centroid: np.ndarray          # (3,) map/FLU
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    n_voxels: int
    top_labels: list              # [(label, cos_score), ...] best-first
    hits: int = 1
    misses: int = 0
    visited: bool = False
    # Target-label instances whose target score barely beats another label:
    # (runner-up label, its score) — rendered as AMBIGUOUS for the LLM and
    # excluded from passive auto-visits and the deterministic fallback.
    runner_up: 'tuple | None' = None
    ambiguous: bool = False
    oversized: bool = False       # far larger than the LLM's expected size
    undersized: bool = False      # far smaller — likely a partial view


@dataclass
class RayGroup:
    id: str
    label: str                    # top-1 label of the group's rays
    n_rays: int
    mean_origin: np.ndarray       # (3,)
    mean_dir: np.ndarray          # (3,) unit
    top_labels: list
    assoc_instance_id: 'str | None' = None
    blocked: bool = False
    visited_target: bool = False  # points at an already-visited target


@dataclass
class Digest:
    text: str
    instances: list = field(default_factory=list)     # [Instance]
    ray_groups: list = field(default_factory=list)    # [RayGroup]
    selectable_instances: set = field(default_factory=set)
    selectable_rays: set = field(default_factory=set)
    # compass sector -> centroid (np(3)) of that sector's frontier cells;
    # keys double as the goto_frontier ids offered to the LLM.
    frontier_targets: dict = field(default_factory=dict)
    # '<surface> <compass>' -> waypoint at the mapped surface's far end in
    # that direction (+ a step into the unmapped continuation) — the
    # follow_surface ids ("follow the road NE to find the bus stop").
    surface_follow: dict = field(default_factory=dict)
    # instances offerable to scan_instance (hint objects: a building may hide
    # a human target behind its windows).
    scan_ids: set = field(default_factory=set)

    def structured(self) -> dict:
        return {
            'instances': [
                dict(id=i.id, label=i.label, top_labels=i.top_labels,
                     centroid=[round(float(v), 1) for v in i.centroid],
                     size=[round(float(v), 1)
                           for v in (i.bbox_max - i.bbox_min)],
                     n_voxels=i.n_voxels, hits=i.hits, visited=i.visited)
                for i in self.instances],
            'ray_groups': [
                dict(id=r.id, label=r.label, n_rays=r.n_rays,
                     origin=[round(float(v), 1) for v in r.mean_origin],
                     dir=[round(float(v), 2) for v in r.mean_dir],
                     top_labels=r.top_labels, assoc=r.assoc_instance_id,
                     blocked=r.blocked)
                for r in self.ray_groups],
            'selectable_instances': sorted(self.selectable_instances),
            'selectable_rays': sorted(self.selectable_rays),
            'frontier_sectors': sorted(self.frontier_targets),
            'surface_follow': sorted(self.surface_follow),
        }


class DigestBuilder:
    """Holds the label bank + tracked instances; builds Digest snapshots."""

    def __init__(self, target: str, object_labels: list, surface_labels: list,
                 floor_cos: float = 0.12, min_instance_voxels: int = 5,
                 max_instances_shown: int = 15, max_ray_groups: int = 8,
                 ray_sector_deg: float = 30.0, assoc_radius_m: float = 8.0,
                 assoc_max_range_m: float = 40.0, label_aliases: dict = None):
        self.target = target
        self.object_labels = list(object_labels)
        self.surface_labels = list(surface_labels)
        # alias -> canonical (e.g. {'roof': 'house'}): part labels cluster into
        # their parent object so one physical house is one instance.
        self.label_aliases = {k.lower(): v.lower()
                              for k, v in (label_aliases or {}).items()}
        self.floor_cos = floor_cos
        self.min_instance_voxels = min_instance_voxels
        # Same-label clusters closer than this bridge into one instance
        # (occlusion gaps: a house split by a tree). 0 = touching only.
        self.merge_gap_m = 0.0
        self.max_instances_shown = max_instances_shown
        self.max_ray_groups = max_ray_groups
        self.ray_sector_deg = ray_sector_deg
        self.assoc_radius_m = assoc_radius_m
        self.assoc_max_range_m = assoc_max_range_m

        self.labels: list = []            # column order, set once detected
        self._object_cols: list = []
        # Per-column map-wide mean score (this scene's baseline per label) —
        # CLIP-family labels have different similarity baselines, so a raw
        # score only means something relative to its own label's background.
        self.label_bg = None
        self._surface_pts: dict = {}      # surface label -> subsampled xyz
        # LLM-provided typical [l, w, h] of the target (perception_params).
        # min_voxels is only a FLOOR — this is the ceiling side: clusters far
        # larger than expected get flagged (a house-sized "bus stop" is a
        # mislabeled house, not a big bus stop).
        self.expected_size = None
        self._instances: dict = {}        # id -> Instance
        self._merged_redirect: dict = {}  # absorbed id -> surviving id
        self.last_merges: list = []       # [(absorbed, survivor)] from last track
        self._next_instance_num = 1
        self.grid_overflow = False

    # ── labels ────────────────────────────────────────────────────────────────

    def set_labels(self, labels: list):
        self.labels = list(labels)
        obj = {o.lower() for o in self.object_labels}
        obj.add(self.target.lower())
        self._object_cols = [i for i, l in enumerate(self.labels)
                             if l.lower() in obj]
        # Column -> canonical label (aliases collapse part labels into their
        # parent object; canonical groups drive clustering and ray grouping).
        self._canonical_of_col = {
            i: self.label_aliases.get(l.lower(), l.lower())
            for i, l in enumerate(self.labels)}
        self._canonical_groups = {}
        for col in self._object_cols:
            self._canonical_groups.setdefault(
                self._canonical_of_col[col], []).append(col)

    def set_perception_params(self, min_voxels=None, score_floor=None,
                              merge_gap_m=None, expected_size_m=None):
        """Runtime perception tuning — set by the LLM per target (startup
        one-shot + `retune` action), not hand-configured. Config values are
        only the pre-LLM fallback."""
        if min_voxels is not None:
            self.min_instance_voxels = int(min_voxels)
        if score_floor is not None:
            self.floor_cos = float(score_floor)
        if merge_gap_m is not None:
            self.merge_gap_m = float(merge_gap_m)
        if expected_size_m is not None:
            self.expected_size = [float(v) for v in expected_size_m]

    def add_object_labels(self, labels: list):
        """Extend the object vocabulary (LLM-suggested context labels). The
        new columns appear once rayfronts registers the queries and the node
        re-detects the column order."""
        known = {o.lower() for o in self.object_labels}
        for l in labels:
            if l.lower() not in known:
                self.object_labels.append(l)
        if self.labels:
            self.set_labels(self.labels)

    def set_label_aliases(self, aliases: dict):
        """Replace the alias map (e.g. from the LLM's part-of answer) and
        rebuild the canonical column groups."""
        self.label_aliases = {k.lower(): v.lower() for k, v in aliases.items()}
        if self.labels:
            self.set_labels(self.labels)

    @property
    def ready(self) -> bool:
        return bool(self.labels)

    def instance(self, iid: str) -> 'Instance | None':
        """Lookup by id, following merge redirects so a commitment made to a
        fragment stays valid after the fragment is absorbed into a larger
        instance."""
        seen = set()
        while iid in self._merged_redirect and iid not in seen:
            seen.add(iid)
            iid = self._merged_redirect[iid]
        return self._instances.get(iid)

    def mark_visited(self, iid: str):
        inst = self.instance(iid)   # follows merge redirects
        if inst is not None:
            inst.visited = True

    # ── voxel instances ───────────────────────────────────────────────────────

    @log_call
    def update_instances(self, vox_xyz: np.ndarray, vox_scores: np.ndarray):
        """Extract clusters from the latest voxel cloud and refresh the tracker."""
        if vox_scores is not None and len(vox_scores):
            q = min(vox_scores.shape[1], len(self.labels)) or vox_scores.shape[1]
            # Per-label AMBIENT baseline: raw cos only means something
            # relative to its own label's background level in THIS scene.
            # 25th percentile, not mean — the mean degenerates when one
            # object dominates the map (early mission: the first house IS
            # the map, its own scores become "background" and salience
            # collapses to 1).
            self.label_bg = np.quantile(vox_scores[:, :q], 0.25, axis=0)
            self._update_surfaces(vox_xyz, vox_scores[:, :q])
        clusters = self._extract_clusters(vox_xyz, vox_scores)
        self._track(clusters)
        return len(clusters)

    def _update_surfaces(self, vox_xyz, scores):
        """Subsampled positions per SURFACE label (roads etc.) — context the
        LLM needs for relational reasoning ('bus stops sit along roads')."""
        if not self.labels:
            return
        surf = {s.lower() for s in self.surface_labels}
        best = np.argmax(scores, axis=1)
        self._surface_pts = {}
        for i, l in enumerate(self.labels):
            if l.lower() not in surf or i >= scores.shape[1]:
                continue
            m = best == i
            if int(m.sum()) < 10:
                continue
            pts = vox_xyz[m]
            if len(pts) > 400:
                pts = pts[np.random.default_rng(0).choice(
                    len(pts), 400, replace=False)]
            self._surface_pts[l.lower()] = pts

    def _extract_clusters(self, vox_xyz, vox_scores) -> list:
        """Vocabulary-free segmentation + baseline-relative description.

        1. SEGMENT: voxels above the participation floor group by geometric
           adjacency AND score-vector cosine affinity — the per-query score
           vector is the open-set embedding's projection onto the query
           directions, so two voxels of one object look alike regardless of
           which single label wins either of them. No label decides
           membership.
        2. DESCRIBE: each cluster's mean scores are divided by each label's
           map-wide baseline; the most SALIENT label names the cluster, and
           if nothing clears _KNOWN_RATIO it is an honest 'unknown' —
           out-of-vocabulary structures are never force-classified (per-voxel
           raw argmax produced the house-sized "bus stop": 'bus stop' has a
           high ambient baseline and wins wherever real labels dip).
        Surface-described clusters are dropped here (the SURFACES line covers
        them); object and unknown clusters become instances."""
        if vox_xyz is None or len(vox_xyz) == 0 or not self.ready:
            return []
        q = min(vox_scores.shape[1], len(self.labels))
        scores = vox_scores[:, :q]
        keep = scores.max(axis=1) >= self.floor_cos
        if int(keep.sum()) < self.min_instance_voxels:
            return []
        xyz = vox_xyz[keep]
        sc = scores[keep]
        comp = self._affinity_components(xyz, sc)
        if comp is None:
            return []
        clusters = []
        surf = {s.lower() for s in self.surface_labels}
        for c in np.unique(comp):
            m = comp == c
            if int(m.sum()) < self.min_instance_voxels:
                continue
            mean_scores = sc[m].mean(axis=0)
            label = self._describe(mean_scores)
            if label in surf:
                continue
            clusters.append(self._make_cluster(label, xyz[m], mean_scores))
        return self._merge_gap_clusters(clusters)

    def _affinity_components(self, xyz, sc) -> 'np.ndarray | None':
        """Connected components where an edge needs 26-adjacency AND
        score-vector cosine >= _AFFINITY_TAU."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        cells = np.floor(xyz / VOX_SIZE).astype(np.int64)
        mins = cells.min(axis=0)
        shape = cells.max(axis=0) - mins + 1
        if int(np.prod(shape)) > _MAX_GRID_CELLS:
            self.grid_overflow = True
            return None
        cells = cells - mins
        n = len(cells)
        idx_grid = -np.ones(shape, dtype=np.int64)
        idx_grid[cells[:, 0], cells[:, 1], cells[:, 2]] = np.arange(n)
        norm = np.maximum(np.linalg.norm(sc, axis=1), 1e-9)
        offsets = [(dx, dy, dz)
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                   for dz in (-1, 0, 1) if (dx, dy, dz) > (0, 0, 0)]
        rows, cols = [], []
        for o in offsets:
            nb = cells + np.array(o)
            ok = np.all((nb >= 0) & (nb < shape), axis=1)
            j = np.full(n, -1, dtype=np.int64)
            j[ok] = idx_grid[nb[ok, 0], nb[ok, 1], nb[ok, 2]]
            ok = j >= 0
            a = np.nonzero(ok)[0]
            b = j[a]
            cos = (sc[a] * sc[b]).sum(axis=1) / (norm[a] * norm[b])
            good = cos >= _AFFINITY_TAU
            rows.append(a[good])
            cols.append(b[good])
        r = np.concatenate(rows) if rows else np.zeros(0, dtype=np.int64)
        c2 = np.concatenate(cols) if cols else np.zeros(0, dtype=np.int64)
        g = coo_matrix((np.ones(len(r)), (r, c2)), shape=(n, n))
        _, comp = connected_components(g, directed=False)
        return comp

    def _salience(self, mean_scores) -> np.ndarray:
        """Score / that label's own map-wide baseline — the cross-label-
        comparable quantity (raw cos baselines differ per label)."""
        bg = self.label_bg
        if bg is None or len(bg) < len(mean_scores):
            bg = np.full(len(mean_scores), 1e-3)
        return mean_scores / np.maximum(bg[:len(mean_scores)], 1e-3)

    def _describe(self, mean_scores) -> str:
        """Canonical label of the most salient column, or 'unknown' when no
        label meaningfully beats its own baseline. Absolute fallback: a label
        scoring far above the participation floor is known even when it
        dominates the map (ambient ~= itself, salience ~= 1)."""
        ratio = self._salience(mean_scores)
        best = int(np.argmax(ratio))
        if (float(ratio[best]) < _KNOWN_RATIO
                and float(mean_scores[best]) < 2.0 * self.floor_cos):
            return 'unknown'
        # Between the two criteria pick the stronger claim: if the salience
        # winner is weak but some label is absolutely strong, name that one.
        if float(ratio[best]) < _KNOWN_RATIO:
            best = int(np.argmax(mean_scores))
        return self._canonical_of_col.get(best, self.labels[best].lower())

    def _make_cluster(self, label, pts, mean_scores) -> dict:
        """Cluster dict + the target-skepticism flags (salience-based)."""
        runner_up, ambiguous, oversized, undersized = None, False, False, False
        if label == self.target.lower():
            ratio = self._salience(mean_scores)
            tcols = [c for c in self._canonical_groups.get(label, [])
                     if c < len(mean_scores)]
            t_ratio = max((float(ratio[c]) for c in tcols), default=0.0)
            others = [(float(ratio[i]), i)
                      for i in range(len(mean_scores)) if i not in tcols]
            if others:
                ru_r, ru_i = max(others)
                runner_up = (self.labels[ru_i],
                             round(float(mean_scores[ru_i]), 3))
                ambiguous = t_ratio < 1.15 * ru_r
            if self.expected_size is not None:
                ext = (pts.max(axis=0) - pts.min(axis=0)) + VOX_SIZE
                vol = float(np.prod(np.maximum(ext, VOX_SIZE)))
                exp_vol = float(np.prod(self.expected_size))
                # Asymmetric on purpose: partial views only ever make an
                # object look SMALLER, so oversized is a hard mislabel
                # signal while undersized just means "may still grow".
                oversized = vol > 4.0 * exp_vol
                undersized = vol < exp_vol / 4.0
        return dict(
            label=label,
            centroid=pts.mean(axis=0),
            bbox_min=pts.min(axis=0) - VOX_SIZE / 2,
            bbox_max=pts.max(axis=0) + VOX_SIZE / 2,
            n_voxels=len(pts),
            top_labels=self._top_labels(mean_scores),
            runner_up=runner_up,
            ambiguous=ambiguous,
            oversized=oversized,
            undersized=undersized,
        )

    def _merge_gap_clusters(self, clusters: list) -> list:
        """Merge same-described-label clusters whose boxes touch (affinity
        splits within one object, e.g. wall vs roof facets) or sit within
        merge_gap_m (occlusion splits). Keeps the larger cluster's
        description/flags."""
        slack = max(self.merge_gap_m, 0.5)
        merged = True
        while merged and len(clusters) > 1:
            merged = False
            for i in range(len(clusters)):
                for j in range(i + 1, len(clusters)):
                    a, b = clusters[i], clusters[j]
                    if a['label'] != b['label']:
                        continue
                    if not self._bbox_overlap(a['bbox_min'], a['bbox_max'],
                                              b['bbox_min'], b['bbox_max'],
                                              slack=slack):
                        continue
                    big, small = (a, b) if a['n_voxels'] >= b['n_voxels'] \
                        else (b, a)
                    tot = a['n_voxels'] + b['n_voxels']
                    big['centroid'] = (
                        a['centroid'] * a['n_voxels']
                        + b['centroid'] * b['n_voxels']) / tot
                    big['bbox_min'] = np.minimum(a['bbox_min'], b['bbox_min'])
                    big['bbox_max'] = np.maximum(a['bbox_max'], b['bbox_max'])
                    big['n_voxels'] = tot
                    clusters[i] = big
                    del clusters[j]
                    merged = True
                    break
                if merged:
                    break
        return clusters

    @staticmethod
    def _bbox_overlap(min1, max1, min2, max2, slack: float = 0.5) -> bool:
        """True if the two AABBs intersect (with slack so touching counts)."""
        return bool(np.all(min1 <= max2 + slack) and np.all(min2 <= max1 + slack))

    def _match_margin(self, inst) -> float:
        """Adaptive centroid-match margin: grows with the instance footprint
        (a partially-mapped object's centroid shifts while growing), capped so
        one runaway blob can't vacuum up the whole neighborhood."""
        ext = inst.bbox_max - inst.bbox_min
        return min(max(_TRACK_MATCH_M,
                       _TRACK_MATCH_EXTENT_FRAC * float(max(ext[0], ext[1]))),
                   _TRACK_MATCH_CAP_M)

    def _track(self, clusters: list):
        """Match new clusters to existing same-label instances, growth-aware.

        MANY-TO-ONE: several same-tick fragments of one object all match the
        same instance and are folded into its union (one-to-one matching made
        every extra fragment spawn a throwaway ID that merged a tick later —
        the ID-churn seen in the first PoC runs). When an updated instance now
        spans previously separate instances, those are absorbed; visited
        status carries over only if the absorbed centroid lies INSIDE the
        survivor (edge-slivers must not mark a whole blob as seen).
        """
        self.last_merges = []
        # Pass 1: best existing instance per cluster (no removal — fragments
        # of one object may all pick the same instance).
        assignments = {}
        new_clusters = []
        for cl in clusters:
            best_id, best_d = None, None
            for iid, inst in self._instances.items():
                if inst.label != cl['label']:
                    continue
                d = float(np.linalg.norm(inst.centroid - cl['centroid']))
                if d < self._match_margin(inst) or self._bbox_overlap(
                        inst.bbox_min, inst.bbox_max,
                        cl['bbox_min'], cl['bbox_max']):
                    if best_d is None or d < best_d:
                        best_id, best_d = iid, d
            if best_id is not None:
                assignments.setdefault(best_id, []).append(cl)
            else:
                new_clusters.append(cl)
        # Pass 2: update each matched instance with its clusters' union.
        for iid, cls in assignments.items():
            inst = self._instances[iid]
            n_tot = sum(c['n_voxels'] for c in cls)
            inst.centroid = sum(
                (np.asarray(c['centroid'], dtype=float) * c['n_voxels']
                 for c in cls), np.zeros(3)) / max(n_tot, 1)
            inst.bbox_min = np.min([c['bbox_min'] for c in cls], axis=0)
            inst.bbox_max = np.max([c['bbox_max'] for c in cls], axis=0)
            biggest = max(cls, key=lambda c: c['n_voxels'])
            inst.n_voxels = n_tot
            inst.top_labels = biggest['top_labels']
            inst.runner_up = biggest.get('runner_up')
            inst.ambiguous = bool(biggest.get('ambiguous'))
            inst.oversized = bool(biggest.get('oversized'))
            inst.undersized = bool(biggest.get('undersized'))
            inst.hits += 1
            inst.misses = 0
        # Pass 3: absorb unmatched OLD instances now spanned by an updated one.
        matched_ids = set(assignments)
        for iid in matched_ids:
            inst = self._instances.get(iid)
            if inst is None:
                continue
            for oid, other in list(self._instances.items()):
                if oid == iid or oid in matched_ids:
                    continue
                if other.label == inst.label and self._bbox_overlap(
                        other.bbox_min, other.bbox_max,
                        inst.bbox_min, inst.bbox_max):
                    centroid_inside = bool(
                        np.all(other.centroid >= inst.bbox_min)
                        and np.all(other.centroid <= inst.bbox_max))
                    if centroid_inside:
                        inst.visited = inst.visited or other.visited
                    elif other.visited:
                        continue    # keep the visited sliver as its own record
                    inst.hits += other.hits
                    del self._instances[oid]
                    self._merged_redirect[oid] = iid
                    self.last_merges.append((oid, iid))
        # Pass 4: unmatched clusters become new instances.
        created = set()
        for cl in new_clusters:
            iid = f'V{self._next_instance_num}'
            self._next_instance_num += 1
            created.add(iid)
            self._instances[iid] = Instance(
                id=iid, label=cl['label'], centroid=cl['centroid'].copy(),
                bbox_min=cl['bbox_min'], bbox_max=cl['bbox_max'],
                n_voxels=cl['n_voxels'], top_labels=cl['top_labels'],
                runner_up=cl.get('runner_up'),
                ambiguous=bool(cl.get('ambiguous')),
                oversized=bool(cl.get('oversized')),
                undersized=bool(cl.get('undersized')))
        # Pass 4b: same-label containment cleanup. A box fully inside a bigger
        # same-label box is a duplicate partial view of the same object — one
        # house mapped as disconnected fragments yields two clusters whose
        # envelopes nest, and since both get matched every tick, pass 3
        # (which skips matched-vs-matched on purpose) never merges them.
        # Ordered big->small so chains collapse in one sweep; visited carries
        # over (the fragment IS part of the surviving object).
        by_volume = sorted(
            self._instances.values(),
            key=lambda i: -float(np.prod(np.maximum(i.bbox_max - i.bbox_min,
                                                    0.1))))
        for big in by_volume:
            if big.id not in self._instances:
                continue
            for small in by_volume:
                if (small.id == big.id or small.id not in self._instances
                        or small.label != big.label):
                    continue
                if (np.all(small.bbox_min >= big.bbox_min - 0.5)
                        and np.all(small.bbox_max <= big.bbox_max + 0.5)):
                    big.visited = big.visited or small.visited
                    big.hits += small.hits
                    del self._instances[small.id]
                    self._merged_redirect[small.id] = big.id
                    self.last_merges.append((small.id, big.id))
        # Pass 5: miss-count everything untouched this tick.
        for iid in list(self._instances.keys()):
            if iid in matched_ids or iid in created:
                continue
            inst = self._instances[iid]
            inst.misses += 1
            # Visited instances are kept forever so the LLM sees its history.
            if inst.misses > _TRACK_MAX_MISSES and not inst.visited:
                del self._instances[iid]

    # ── ray groups ────────────────────────────────────────────────────────────

    def _points_at_visited(self, g: 'RayGroup', visited_points: list) -> bool:
        """True if the group's line passes near a visited-target location.

        Checked against the node's visited POSITIONS (not just live
        instances): a visited house's voxels/rays keep scoring high forever,
        and rays hitting its unmapped back side often fail instance
        association — without this, they present as fresh leads and pull the
        drone back to targets it has already seen."""
        inst = self.instance(g.assoc_instance_id) if g.assoc_instance_id else None
        if inst is not None and inst.visited:
            return True
        for p in visited_points:
            v = np.asarray(p, dtype=float) - g.mean_origin
            t = float(v @ g.mean_dir)
            if t <= 0 or t > self.assoc_max_range_m:
                continue
            if float(np.linalg.norm(v - t * g.mean_dir)) <= self.assoc_radius_m:
                return True
        return False

    @log_call
    def build_ray_groups(self, origins, dirs, scores,
                         blacklisted_dirs: list,
                         visited_points: list = ()) -> list:
        """Bucket rays by (azimuth sector, top-1 object label) and associate
        each group to the instance its mean line passes near, if any."""
        if origins is None or len(origins) == 0 or not self.ready:
            return []
        q = min(scores.shape[1], len(self.labels))
        sc = scores[:, :q]
        best = np.argmax(sc, axis=1)
        best_s = sc[np.arange(len(sc)), best]
        keep = np.isin(best, self._object_cols) & (best_s >= self.floor_cos)
        if not keep.any():
            return []
        origins, dirs, sc, best = origins[keep], dirs[keep], sc[keep], best[keep]

        az = np.degrees(np.arctan2(dirs[:, 1], dirs[:, 0])) % 360.0
        sector = (az // self.ray_sector_deg).astype(int)
        groups = {}
        for i in range(len(origins)):
            canonical = self._canonical_of_col[int(best[i])]
            groups.setdefault((sector[i], canonical), []).append(i)

        target_col = self._target_col()
        raw = []
        for (sec, canonical), idxs in groups.items():
            idxs = np.asarray(idxs)
            mean_dir = dirs[idxs].mean(axis=0)
            n = np.linalg.norm(mean_dir)
            if n < 1e-6:
                continue
            mean_dir = mean_dir / n
            mean_scores = sc[idxs].mean(axis=0)
            raw.append(RayGroup(
                id='',
                label=canonical,
                n_rays=len(idxs),
                mean_origin=origins[idxs].mean(axis=0),
                mean_dir=mean_dir,
                top_labels=self._top_labels(mean_scores),
            ))
        # Rank: target evidence first, then ray count.
        def rank(g):
            t = dict((l, s) for l, s in g.top_labels).get(
                self.labels[target_col] if target_col is not None else '', 0.0)
            return (-t, -g.n_rays)
        raw.sort(key=rank)
        raw = raw[:self.max_ray_groups]
        for i, g in enumerate(raw):
            g.id = f'R{i + 1}'
            g.assoc_instance_id = self._associate(g)
            g.blocked = any(
                self._dir_agreement_xy(g.mean_dir, bd) > math.cos(math.radians(30.0))
                for bd in blacklisted_dirs)
            g.visited_target = self._points_at_visited(g, list(visited_points))
        return raw

    def _fmt_labels(self, top_labels) -> str:
        """Render 'label X/Y': X = score, Y = that label's map-wide average.
        X >> Y is genuinely distinctive; X ~= Y is baseline noise."""
        parts = []
        for l, s in top_labels:
            bg = None
            if self.label_bg is not None:
                for i, lbl in enumerate(self.labels):
                    if lbl == l and i < len(self.label_bg):
                        bg = float(self.label_bg[i])
                        break
            if bg is not None:
                parts.append(f'{l} {s * 100:.0f}/{bg * 100:.0f}')
            else:
                parts.append(f'{l} {s * 100:.0f}')
        return ', '.join(parts)

    def _top_labels(self, mean_scores) -> list:
        """Top-3 (label, score), dropping near-zero entries but keeping top-1."""
        order = np.argsort(-mean_scores)[:3]
        out = [(self.labels[i], round(float(mean_scores[i]), 3))
               for i in order if mean_scores[i] > 0.02]
        if not out and len(order):
            i = order[0]
            out = [(self.labels[i], round(float(mean_scores[i]), 3))]
        return out

    @staticmethod
    def _dir_agreement_xy(a, b) -> float:
        """Cosine between the XY projections of two directions (0 if degenerate)."""
        a2 = np.asarray(a, dtype=float)[:2]
        b2 = np.asarray(b, dtype=float)[:2]
        na, nb = np.linalg.norm(a2), np.linalg.norm(b2)
        if na < 1e-6 or nb < 1e-6:
            return 0.0
        return float(a2 @ b2 / (na * nb))

    def _target_col(self) -> 'int | None':
        for i, l in enumerate(self.labels):
            if l.lower() == self.target.lower():
                return i
        return None

    def _associate(self, g: RayGroup) -> 'str | None':
        best_id, best_d = None, self.assoc_radius_m
        for iid, inst in self._instances.items():
            v = inst.centroid - g.mean_origin
            t = float(v @ g.mean_dir)
            if t <= 0 or t > self.assoc_max_range_m:
                continue
            d = float(np.linalg.norm(v - t * g.mean_dir))
            if d < best_d:
                best_id, best_d = iid, d
        return best_id

    # ── rendering ─────────────────────────────────────────────────────────────

    @log_call
    def build(self, robot_pos: np.ndarray, heading_deg: float,
              status_line: str, ray_origins, ray_dirs, ray_scores,
              frontiers, blacklist: dict, blacklisted_dirs: list,
              visited_points: list = (), scanned: set = frozenset()) -> Digest:
        """Assemble the full text digest + structured snapshot."""
        now_groups = self.build_ray_groups(
            ray_origins, ray_dirs, ray_scores, blacklisted_dirs,
            visited_points)
        import time as _time
        now = _time.time()

        insts = sorted(
            self._instances.values(),
            key=lambda i: (-dict(i.top_labels).get(self.target, 0.0),
                           -i.n_voxels))
        shown = insts[:self.max_instances_shown]

        lines = [f'TARGET: {self.target}']
        if robot_pos is not None:
            lines.append(
                f'YOU: at ({robot_pos[0]:.0f}, {robot_pos[1]:.0f}), '
                f'altitude {robot_pos[2]:.0f}m, heading {compass_of(math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg)))}.')
        lines.append(f'STATUS: {status_line}')

        selectable_i, selectable_r, scan_ids = set(), set(), set()
        if shown:
            lines.append('MAPPED OBJECTS (scores are similarity x100, '
                         'typically 5-30, higher = stronger match):')
            for inst in shown:
                size = inst.bbox_max - inst.bbox_min
                rel = inst.centroid - (robot_pos if robot_pos is not None
                                       else np.zeros(3))
                dist = float(np.linalg.norm(rel[:2]))
                labels_str = self._fmt_labels(inst.top_labels)
                flags = []
                blacklisted = blacklist.get(inst.id, 0.0) > now
                if inst.visited:
                    flags.append('VISITED')
                if blacklisted:
                    flags.append('BLOCKED')
                if inst.ambiguous and inst.runner_up:
                    flags.append(f'AMBIGUOUS — could be {inst.runner_up[0]}')
                if inst.oversized and self.expected_size is not None:
                    e = self.expected_size
                    flags.append(f'TOO BIG for a {self.target} '
                                 f'(typical ~{e[0]:.0f}x{e[1]:.0f}x{e[2]:.0f}m)')
                if inst.undersized:
                    flags.append(f'PARTIAL VIEW? smaller than a typical '
                                 f'{self.target}, may grow as mapped')
                if inst.label == 'unknown':
                    flags.append('UNRECOGNIZED structure — no vocabulary '
                                 'label fits; closest scores shown')
                if inst.id in scanned:
                    flags.append('SCANNED')
                flag_str = f' [{", ".join(flags)}]' if flags else ''
                lines.append(
                    f'  {inst.id}: {labels_str} | '
                    f'~{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}m '
                    f'({inst.n_voxels} voxels) | {dist:.0f}m {compass_of(rel[0], rel[1])} | '
                    f'seen {inst.hits}x{flag_str}')
                if not inst.visited and not blacklisted:
                    selectable_i.add(inst.id)
                if not blacklisted:
                    scan_ids.add(inst.id)
        else:
            lines.append('MAPPED OBJECTS: none yet.')

        if now_groups:
            lines.append('RAY LEADS (rays at the map edge pointing at things '
                         'seen from afar; following one extends the map that way):')
            for g in now_groups:
                labels_str = self._fmt_labels(g.top_labels)
                rel = g.mean_origin - (robot_pos if robot_pos is not None
                                       else np.zeros(3))
                dist = float(np.linalg.norm(rel[:2]))
                if g.assoc_instance_id:
                    where = f'points at {g.assoc_instance_id}'
                else:
                    where = 'points beyond the mapped area'
                flags = ''
                if g.visited_target:
                    flags += ' [points at an ALREADY-VISITED target]'
                if g.blocked:
                    flags += ' [BLOCKED]'
                lines.append(
                    f'  {g.id}: {g.n_rays} rays {compass_of(g.mean_dir[0], g.mean_dir[1])} '
                    f'from {dist:.0f}m away | {labels_str} | {where}{flags}')
                if not g.blocked and not g.visited_target:
                    selectable_r.add(g.id)
        else:
            lines.append('RAY LEADS: none.')

        surfaces_line, surface_follow = self._surfaces_line(robot_pos)
        if surfaces_line:
            lines.append(surfaces_line)

        frontier_line, frontier_targets = self._frontier_sectors(
            frontiers, robot_pos)
        lines.append(frontier_line)

        visited = [i for i in insts if i.visited]
        if visited:
            lines.append('ALREADY VISITED: ' + ', '.join(
                f'{i.id} ({i.label})' for i in visited))

        return Digest(text='\n'.join(lines),
                      instances=shown, ray_groups=now_groups,
                      selectable_instances=selectable_i,
                      selectable_rays=selectable_r,
                      frontier_targets=frontier_targets,
                      surface_follow=surface_follow,
                      scan_ids=scan_ids)

    def _surfaces_line(self, robot_pos) -> tuple:
        """(text line, follow-targets) for surface labels (roads etc.) — the
        context AND the actions for relational target reasoning ('bus stops
        sit along roads -> follow the road'). For each clearly elongated
        surface, its two axis-end directions become follow_surface ids whose
        waypoint is the mapped far end plus a step into the unmapped
        continuation."""
        if not self._surface_pts or robot_pos is None:
            return '', {}
        parts = []
        follow = {}
        for label, pts in self._surface_pts.items():
            rel = pts[:, :2] - robot_pos[:2]
            d = np.linalg.norm(rel, axis=1)
            near_i = int(np.argmin(d))
            desc = (f'{label}: nearest {d[near_i]:.0f}m '
                    f'{compass_of(rel[near_i, 0], rel[near_i, 1])}')
            if len(pts) >= 30:
                xy = pts[:, :2] - pts[:, :2].mean(axis=0)
                cov = xy.T @ xy / len(xy)
                w, v = np.linalg.eigh(cov)
                if w[1] > 3.0 * w[0]:   # clearly elongated (a road, not a lawn)
                    ax = v[:, 1]
                    names = []
                    for sign in (1.0, -1.0):
                        u = ax * sign
                        name = compass_of(u[0], u[1])
                        names.append(name)
                        proj = xy @ u
                        end = pts[int(np.argmax(proj))].astype(float).copy()
                        # step past the mapped end, into where it continues
                        end[0] += u[0] * 10.0
                        end[1] += u[1] * 10.0
                        follow[f'{label} {name}'] = end
                    desc += f', runs {names[0]}–{names[1]} (followable)'
            parts.append(desc)
        line = 'SURFACES: ' + '; '.join(parts) + '.' if parts else ''
        return line, follow

    def _frontier_sectors(self, frontiers, robot_pos) -> tuple:
        """(text line, {sector name -> frontier centroid np(3)}) for the top
        unexplored compass sectors around the robot."""
        if frontiers is None or len(frontiers) == 0 or robot_pos is None:
            return 'UNEXPLORED: unknown (no frontier data yet).', {}
        rel = frontiers[:, :2] - robot_pos[:2]
        dist = np.linalg.norm(rel, axis=1)
        ang = np.degrees(np.arctan2(rel[:, 1], rel[:, 0])) % 360.0
        sec = (((ang + 22.5) % 360.0) // 45.0).astype(int)
        parts = []
        targets = {}
        for s in range(8):
            m = sec == s
            if int(m.sum()) < 5:
                continue
            parts.append((int(m.sum()), _COMPASS[s], float(dist[m].min())))
            targets[_COMPASS[s]] = frontiers[m].mean(axis=0)
        if not parts:
            return 'UNEXPLORED: mostly explored around here.', {}
        parts.sort(reverse=True)
        top = ', '.join(f'{name}: {c} frontier cells (nearest {d:.0f}m)'
                        for c, name, d in parts[:4])
        # Count-descending insertion order — the fallback picks the first key.
        ordered = {name: targets[name] for _, name, _ in parts[:4]}
        return f'UNEXPLORED (by compass direction): {top}.', ordered
