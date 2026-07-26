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
                              merge_gap_m=None):
        """Runtime perception tuning — set by the LLM per target (startup
        one-shot + `retune` action), not hand-configured. Config values are
        only the pre-LLM fallback."""
        if min_voxels is not None:
            self.min_instance_voxels = int(min_voxels)
        if score_floor is not None:
            self.floor_cos = float(score_floor)
        if merge_gap_m is not None:
            self.merge_gap_m = float(merge_gap_m)

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
        clusters = self._extract_clusters(vox_xyz, vox_scores)
        self._track(clusters)
        return len(clusters)

    def _extract_clusters(self, vox_xyz, vox_scores) -> list:
        if vox_xyz is None or len(vox_xyz) == 0 or not self.ready:
            return []
        q = min(vox_scores.shape[1], len(self.labels))
        scores = vox_scores[:, :q]
        best = np.argmax(scores, axis=1)
        best_s = scores[np.arange(len(scores)), best]
        keep = np.isin(best, self._object_cols) & (best_s >= self.floor_cos)
        if not keep.any():
            return []
        xyz = vox_xyz[keep]
        sc = scores[keep]
        bestk = best[keep]

        clusters = []
        for canonical, cols in self._canonical_groups.items():
            cols_q = [c for c in cols if c < q]
            if not cols_q:
                continue
            sel = np.isin(bestk, cols_q)
            if int(sel.sum()) < self.min_instance_voxels:
                continue
            clusters.extend(self._cluster_one_label(
                canonical, xyz[sel], sc[sel]))
        return clusters

    def _cluster_one_label(self, label, xyz, sc) -> list:
        from scipy import ndimage
        cells = np.floor(xyz / VOX_SIZE).astype(np.int64)
        # merge_gap: dilate the occupancy grid k cells before labeling, so
        # same-label patches within the gap join into one component (points
        # keep their original cells; pad the grid so dilation isn't clipped).
        k = max(0, int(round(self.merge_gap_m / VOX_SIZE)))
        mins = cells.min(axis=0) - k
        shape = cells.max(axis=0) - mins + 1 + k
        if int(np.prod(shape)) > _MAX_GRID_CELLS:
            self.grid_overflow = True
            return []
        grid = np.zeros(shape, dtype=bool)
        idx = cells - mins
        grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        if k > 0:
            grid = ndimage.binary_dilation(
                grid, structure=np.ones((3, 3, 3)), iterations=k)
        labeled, n_comp = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        comp_of_pt = labeled[idx[:, 0], idx[:, 1], idx[:, 2]]
        out = []
        for c in range(1, n_comp + 1):
            m = comp_of_pt == c
            if int(m.sum()) < self.min_instance_voxels:
                continue
            pts = xyz[m]
            mean_scores = sc[m].mean(axis=0)
            out.append(dict(
                label=label,
                centroid=pts.mean(axis=0),
                bbox_min=pts.min(axis=0) - VOX_SIZE / 2,
                bbox_max=pts.max(axis=0) + VOX_SIZE / 2,
                n_voxels=int(m.sum()),
                top_labels=self._top_labels(mean_scores),
            ))
        return out

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
            inst.n_voxels = n_tot
            inst.top_labels = max(cls, key=lambda c: c['n_voxels'])['top_labels']
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
                n_voxels=cl['n_voxels'], top_labels=cl['top_labels'])
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
              visited_points: list = ()) -> Digest:
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

        selectable_i, selectable_r = set(), set()
        if shown:
            lines.append('MAPPED OBJECTS (scores are similarity x100, '
                         'typically 5-30, higher = stronger match):')
            for inst in shown:
                size = inst.bbox_max - inst.bbox_min
                rel = inst.centroid - (robot_pos if robot_pos is not None
                                       else np.zeros(3))
                dist = float(np.linalg.norm(rel[:2]))
                labels_str = ', '.join(
                    f'{l} {s * 100:.0f}' for l, s in inst.top_labels)
                flags = []
                blacklisted = blacklist.get(inst.id, 0.0) > now
                if inst.visited:
                    flags.append('VISITED')
                if blacklisted:
                    flags.append('BLOCKED')
                flag_str = f' [{", ".join(flags)}]' if flags else ''
                lines.append(
                    f'  {inst.id}: {labels_str} | '
                    f'~{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}m '
                    f'({inst.n_voxels} voxels) | {dist:.0f}m {compass_of(rel[0], rel[1])} | '
                    f'seen {inst.hits}x{flag_str}')
                if not inst.visited and not blacklisted:
                    selectable_i.add(inst.id)
        else:
            lines.append('MAPPED OBJECTS: none yet.')

        if now_groups:
            lines.append('RAY LEADS (rays at the map edge pointing at things '
                         'seen from afar; following one extends the map that way):')
            for g in now_groups:
                labels_str = ', '.join(
                    f'{l} {s * 100:.0f}' for l, s in g.top_labels)
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
                      frontier_targets=frontier_targets)

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
