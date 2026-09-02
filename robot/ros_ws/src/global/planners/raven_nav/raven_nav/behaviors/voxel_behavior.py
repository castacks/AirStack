"""Voxel-based go-to-object — numpy port of the original RAVEN behaviour.

Source: RayFronts_raven/rayfronts/behaviors/voxel_behavior.py

The OG pipeline, unchanged:
  voxels whose softmax score on ANY target column exceeds 0.98
  -> snap onto the 0.5 m grid, 26-connected connected-component labelling
  -> components with >= 30 voxels become axis-aligned boxes (centre, size)
  -> a box within 10 m (cuboid surface distance) of a visited box is "visited"
  -> fly to the NEAREST unvisited box: cast a ray from the drone to the box
     centre, take the entry point on its surface, stand off 1.0 m short;
     wp2 = that standoff point (locked), wp1 = 0.8 of the way there
  -> within 3 m of wp2 the box is marked visited and the waypoint unlocks

Departures from the OG source, all documented:

* the RDF -> FLU flip happens in `raven_nav_node._vox_all_cb` (same axis
  permutation the OG applied to the cluster centres at OG:87-92), so the CCL
  runs directly in FLU and no post-hoc swap is needed;
* the CCL is a sparse union-find rather than `scipy.ndimage.label` over a dense
  occupancy array. Identical components (pinned by `test_og_parity.py`), but a
  handful of stray voxels 500 m apart no longer allocates a dense grid with
  10^9 cells;
* OG kept `target_voxel_clusters` as a dict it overwrote by index each tick, so
  clusters from a denser earlier tick lingered forever. Rebuilt per tick here;
* clusters carry a `label` (argmax over the target columns, summed over the
  component's voxels) and a `confidence`, which the OG had no use for but the
  AirStack detection reporting does;
* the cluster box is half a voxel off in the OG (it treats the published voxel
  position as a min corner; rayfronts publishes centres) — corrected here, see
  the comment at the box computation.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from raven_nav.behaviors.common import BehaviorOutput, TickContext

# ── OG constants (voxel_behavior.py line numbers) ───────────────────────────
VOXEL_SCORE_THRESHOLD = 0.98    # OG:51 — exposed as `voxel_score_threshold`
VOX_SIZE_M = 0.5                # OG:60
MIN_CLUSTER_VOXELS = 30         # OG:78 — exposed as `voxel_min_cluster_size`
VISITED_NEAR_M = 10.0           # OG:236 (`is_near_visited` threshold)
STANDOFF_M = 1.0                # OG:158 (`offset`)
MID_ALPHA = 0.8                 # OG:167
VISIT_REACH_M = 3.0             # OG:194


@dataclass
class VoxelCluster:
    label: str
    center: np.ndarray          # (3,) FLU
    size: np.ndarray            # (3,)
    num_voxels: int
    confidence: float

    def as_bb(self) -> np.ndarray:
        return np.concatenate([np.asarray(self.center, dtype=float),
                               np.asarray(self.size, dtype=float)])


def cuboid_distance(center_a, size_a, center_b, size_b) -> float:
    """OG:239-245 — surface-to-surface gap between two AABBs (0 if they touch)."""
    ca = np.asarray(center_a, dtype=float)
    sa = np.asarray(size_a, dtype=float)
    cb = np.asarray(center_b, dtype=float)
    sb = np.asarray(size_b, dtype=float)
    gap = np.maximum(np.abs(ca[:3] - cb[:3]) - (sa[:3] + sb[:3]) / 2.0, 0.0)
    return float(np.linalg.norm(gap))


_NEIGHBOUR_OFFSETS = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
    if (dx, dy, dz) > (0, 0, 0)
)   # forward half of the 26-neighbourhood — OG:69 `np.ones((3,3,3))`


def _neighbour_edges(coords: np.ndarray):
    """Index pairs of 26-connected voxels, vectorised.

    Coordinates are packed into one int64 key per voxel and each of the 13
    forward neighbour offsets is resolved with a single `searchsorted` over the
    sorted keys — the alternative, a per-voxel dict probe, costs a second per
    tick once a loose `voxel_score_threshold` leaves 100k voxels standing.
    """
    n = coords.shape[0]
    stride = np.int64(1) << np.int64(21)
    if int(coords.max()) >= int(stride) // 2:
        return None                      # absurd extent; caller falls back
    basis = np.array([stride * stride, stride, 1], dtype=np.int64)
    keys = coords.astype(np.int64) @ basis
    order = np.argsort(keys, kind='stable')
    skeys = keys[order]
    src_all, dst_all = [], []
    for off in _NEIGHBOUR_OFFSETS:
        target = keys + int(np.array(off, dtype=np.int64) @ basis)
        idx = np.searchsorted(skeys, target)
        ok = idx < n
        idx_c = np.where(ok, idx, 0)
        ok &= skeys[idx_c] == target
        if not np.any(ok):
            continue
        src_all.append(np.nonzero(ok)[0])
        dst_all.append(order[idx_c[ok]])
    if not src_all:
        return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
    return np.concatenate(src_all), np.concatenate(dst_all)


def _components_from_edges(n: int, src, dst) -> np.ndarray:
    """Union-find over an edge list; labels in first-appearance order."""
    parent = list(range(n))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in zip(src.tolist(), dst.tolist()):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    labels = np.empty(n, dtype=np.int64)
    seen = {}
    for i in range(n):
        r = find(i)
        if r not in seen:
            seen[r] = len(seen)
        labels[i] = seen[r]
    return labels


def connected_components(coords: np.ndarray) -> np.ndarray:
    """26-connected CCL over integer voxel coordinates (N,3) -> (N,) label ids.

    Same components as OG `scipy.ndimage.label` with a 3x3x3 structuring
    element (voxel_behavior.py:69-70), pinned by test_og_parity.py — but sparse,
    so a handful of stray voxels 500 m apart no longer allocates a dense grid
    with 10^9 cells. Labels are 0-based, assigned in first-appearance order, so
    the output is deterministic.
    """
    n = coords.shape[0]
    if n == 0:
        return np.zeros((0,), dtype=np.int64)
    edges = _neighbour_edges(coords)
    if edges is None:
        return _dict_probe_components(coords)
    src, dst = edges
    try:
        from scipy.sparse import coo_matrix          # noqa: PLC0415
        from scipy.sparse.csgraph import connected_components as _cc
        adj = coo_matrix((np.ones(src.size, dtype=np.uint8), (src, dst)),
                         shape=(n, n))
        _k, raw = _cc(adj, directed=False)
    except ImportError:
        return _components_from_edges(n, src, dst)
    # renumber in first-appearance order so the labelling is stable
    _uniq, first = np.unique(raw, return_index=True)
    remap = np.empty(int(raw.max()) + 1, dtype=np.int64)
    remap[raw[np.sort(first)]] = np.arange(first.size, dtype=np.int64)
    return remap[raw]


def _dict_probe_components(coords: np.ndarray) -> np.ndarray:
    """Fallback for coordinates too large to pack into one int64 key."""
    n = coords.shape[0]
    index = {(int(c[0]), int(c[1]), int(c[2])): i for i, c in enumerate(coords)}
    src, dst = [], []
    for i in range(n):
        x, y, z = int(coords[i, 0]), int(coords[i, 1]), int(coords[i, 2])
        for dx, dy, dz in _NEIGHBOUR_OFFSETS:
            j = index.get((x + dx, y + dy, z + dz))
            if j is not None:
                src.append(i)
                dst.append(j)
    return _components_from_edges(n, np.array(src, dtype=np.int64),
                                  np.array(dst, dtype=np.int64))


class VoxelBehavior:
    name = 'Voxel-based'

    def __init__(self, score_threshold: float = VOXEL_SCORE_THRESHOLD,
                 min_cluster_size: int = MIN_CLUSTER_VOXELS,
                 vox_size: float = VOX_SIZE_M) -> None:
        self.score_threshold = float(score_threshold)
        self.min_cluster_size = int(min_cluster_size)
        self.vox_size = float(vox_size)
        self.clusters: List[VoxelCluster] = []
        self.unvisited: List[VoxelCluster] = []
        # OG:18 — [cx,cy,cz,sx,sy,sz] rows, accumulated for the whole mission.
        self.visited_clusters: List[np.ndarray] = []
        self.visited_labels: List[Tuple[str, np.ndarray, np.ndarray]] = []
        self.newly_visited: List[VoxelCluster] = []

    # ── detection (OG condition_check body, OG:32-109) ──────────────────────
    def detect(self, ctx: TickContext) -> List[VoxelCluster]:
        self.clusters = []
        xyz, scores = ctx.vox_xyz, ctx.vox_scores
        cols = ctx.target_columns()
        if xyz is None or scores is None or len(xyz) == 0 or not cols:
            return self.clusters
        scores = np.asarray(scores, dtype=np.float64)
        if scores.ndim != 2 or max(cols) >= scores.shape[1]:
            return self.clusters
        rel = scores[:, cols]
        mask = (rel > self.score_threshold).any(axis=1)     # OG:53-55
        if not np.any(mask):
            return self.clusters
        pts = np.round(np.asarray(xyz, dtype=np.float64)[mask], 3)   # OG:59
        rel = rel[mask]

        min_coords = pts.min(axis=0)
        # OG:64 `((filtered_vox - min_coords)/vox_size).long()` truncates;
        # rayfronts voxel positions are exact multiples of vox_size
        # (`geometry3d.pointcloud_to_sparse_voxels`: round(p/v)*v), so rounding
        # is the same lattice index and survives float32 noise.
        coords = np.round((pts - min_coords) / self.vox_size).astype(np.int64)
        comp = connected_components(coords)
        for cid in range(int(comp.max()) + 1 if comp.size else 0):
            sel = np.nonzero(comp == cid)[0]
            if sel.size < self.min_cluster_size:            # OG:78-79
                continue
            cc = coords[sel]
            min_voxel = cc.min(axis=0)
            max_voxel = cc.max(axis=0)
            # OG:83-84 took the box as [min_p, max_p + vox], i.e. it treated
            # the published voxel position as the voxel's MIN CORNER. It is the
            # voxel CENTRE (`geometry3d.pointcloud_to_sparse_voxels` rounds),
            # so the occupied volume is [min_p - vox/2, max_p + vox/2]: the OG
            # box has the right size but sits half a voxel off along every
            # axis. Corrected here; `test_og_parity.py` pins the two together
            # through exactly that offset.
            half = self.vox_size / 2.0
            min_world = min_voxel * self.vox_size + min_coords - half
            max_world = (max_voxel + 1) * self.vox_size + min_coords - half
            center = (min_world + max_world) / 2.0
            size = max_world - min_world
            col_sums = rel[sel].sum(axis=0)
            best = int(np.argmax(col_sums))
            label = str(ctx.query_labels[cols[best]])
            confidence = float(np.mean(rel[sel][:, best]))
            self.clusters.append(VoxelCluster(
                label=label, center=center, size=size,
                num_voxels=int(sel.size), confidence=confidence))
        return self.clusters

    def update(self, ctx: TickContext) -> List[VoxelCluster]:
        """Recompute clusters and the unvisited subset. Called every tick by
        the node — including in the frontier-only baseline, which is how that
        arm still reports passive detections."""
        self.detect(ctx)
        self.unvisited = [c for c in self.clusters
                          if not self.is_near_visited(c.center, c.size)
                          and ctx.inside_area(c.center)]   # DEVIATION 2
        return self.clusters

    def condition_check(self, ctx: TickContext) -> bool:   # noqa: ARG002
        """OG:105-106 — fire only when an unvisited cluster exists."""
        return len(self.unvisited) > 0

    def is_near_visited(self, center, size,
                        threshold: float = VISITED_NEAR_M) -> bool:
        """OG:236-237."""
        return any(cuboid_distance(center, size, v[:3], v[3:6]) < threshold
                   for v in self.visited_clusters)

    def mark_visited(self, cluster: VoxelCluster) -> None:
        if self.is_near_visited(cluster.center, cluster.size, threshold=1e-6):
            return
        self.visited_clusters.append(cluster.as_bb())
        self.visited_labels.append(
            (cluster.label, np.asarray(cluster.center, dtype=float),
             np.asarray(cluster.size, dtype=float)))
        self.newly_visited.append(cluster)

    # ── execution (OG:111-200) ──────────────────────────────────────────────
    def execute(self, ctx: TickContext) -> BehaviorOutput:
        out = BehaviorOutput(waypoint_locked=ctx.waypoint_locked,
                             target_waypoint=ctx.target_waypoint,
                             target_waypoint2=ctx.target_waypoint2)
        pose = np.asarray(ctx.cur_pose, dtype=np.float64).reshape(3)
        ordered = sorted(self.unvisited,
                         key=lambda c: float(np.linalg.norm(pose - c.center)))
        if not ordered:
            out.note = 'no unvisited clusters'
            return out

        for i, cluster in enumerate(ordered):
            center = np.asarray(cluster.center, dtype=np.float64)
            half = np.asarray(cluster.size, dtype=np.float64) / 2.0
            d = center - pose
            nrm = float(np.linalg.norm(d))
            if nrm < 1e-9:
                continue
            dir_norm = d / nrm
            ray_origin_local = pose - center
            tmin, tmax = -np.inf, np.inf
            for axis in range(3):                          # OG:144-153
                if dir_norm[axis] != 0:
                    t1 = (-half[axis] - ray_origin_local[axis]) / dir_norm[axis]
                    t2 = (half[axis] - ray_origin_local[axis]) / dir_norm[axis]
                    tmin = max(tmin, min(t1, t2))
                    tmax = min(tmax, max(t1, t2))
                elif abs(ray_origin_local[axis]) > half[axis]:
                    continue
            if tmax < max(tmin, 0):                        # OG:154-155
                continue
            t_hit = tmin if tmin > 0 else tmax             # OG:156
            surface = pose + dir_norm * t_hit
            adjacent = surface - dir_norm * STANDOFF_M     # OG:158-159
            if i != 0:
                # OG:161 — only the nearest cluster ever produces a waypoint.
                break
            if not out.waypoint_locked or out.target_waypoint2 is None:  # OG:162-164
                out.target_waypoint2 = ctx.clamp(adjacent)
                out.waypoint_locked = True
            wp2 = np.asarray(out.target_waypoint2, dtype=np.float64)
            wp1 = ctx.clamp(pose * (1.0 - MID_ALPHA) + wp2 * MID_ALPHA)  # OG:168
            out.target_waypoint = wp1
            out.path = [wp1, wp2]                          # OG:172-188
            break

        if out.target_waypoint2 is not None and float(np.linalg.norm(
                pose - np.asarray(out.target_waypoint2))) < VISIT_REACH_M:
            self.mark_visited(ordered[0])                  # OG:194-198
            out.waypoint_locked = False
        return out

    # ── reporting ───────────────────────────────────────────────────────────
    def voxel_table(self) -> str:
        lines = [f'voxel clusters={len(self.clusters)} '
                 f'unvisited={len(self.unvisited)} '
                 f'visited={len(self.visited_clusters)} '
                 f'thr={self.score_threshold:g} min_vox={self.min_cluster_size}']
        if self.clusters:
            lines.append(f'{"label":<18} {"n":>6} {"cx":>8} {"cy":>8} {"cz":>7} '
                         f'{"sx":>6} {"sy":>6} {"sz":>6} {"conf":>6} {"state":>9}')
            for c in self.clusters:
                state = ('visited' if self.is_near_visited(c.center, c.size)
                         else 'unvisited')
                lines.append(
                    f'{c.label:<18} {c.num_voxels:>6} {c.center[0]:8.1f} '
                    f'{c.center[1]:8.1f} {c.center[2]:7.1f} {c.size[0]:6.1f} '
                    f'{c.size[1]:6.1f} {c.size[2]:6.1f} {c.confidence:6.2f} '
                    f'{state:>9}')
        return '\n'.join(lines)
