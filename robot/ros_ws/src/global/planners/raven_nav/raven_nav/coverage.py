"""Observed-area bookkeeping and the polygon-completion gate.

DEVIATION 4 from the OG paper logic: RAVEN had no notion of "done". AirStack's
`semantic_search_task` ends a search when raven publishes
`navigation_mode == 'complete'`, so the single-agent rewrite keeps this
(single-robot) coverage tracker — own cells only, no peer merging.

A cell is observed when the drone occupies it, when a mapped voxel falls in it,
or when it lies on the 2D line from the drone toward a frontier (stopping short
of the frontier itself so the frontier is not painted as explored).
"""
from __future__ import annotations

from typing import Optional, Set, Tuple

import numpy as np

from raven_nav.behaviors.common import points_in_polygon

# Stop this far short of a frontier point when raycasting, so the frontier cell
# itself stays unobserved (otherwise the polygon reports as cleared while the
# unexplored edge is still there).
RAYCAST_PULLBACK_M = 5.0


def polygon_area_xy(poly_xy: Optional[np.ndarray]) -> float:
    """Shoelace area. 0 for degenerate input."""
    if poly_xy is None or np.asarray(poly_xy).shape[0] < 3:
        return 0.0
    p = np.asarray(poly_xy, dtype=np.float64)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.abs(np.dot(x, np.roll(y, -1))
                              - np.dot(y, np.roll(x, -1))))


class CoverageTracker:
    def __init__(self, cell_size_m: float = 0.5,
                 raycast_range_m: float = 30.0,
                 raycast_min_step_m: float = 5.0) -> None:
        self.cell_size_m = float(cell_size_m)
        self.raycast_range_m = float(raycast_range_m)
        self.raycast_min_step_m = float(raycast_min_step_m)
        self.cells: Set[Tuple[int, int]] = set()
        self._last_raycast_xy: Optional[np.ndarray] = None
        self.fraction: float = 0.0
        self.milestone: int = -1

    # ── stamping ────────────────────────────────────────────────────────────
    def to_cells(self, xys: np.ndarray) -> np.ndarray:
        xys = np.asarray(xys, dtype=np.float64).reshape(-1, 2)
        if xys.size == 0:
            return np.zeros((0, 2), dtype=np.int64)
        return np.floor(xys / self.cell_size_m).astype(np.int64)

    def stamp_points(self, xys: np.ndarray) -> None:
        cells = self.to_cells(xys)
        if cells.shape[0]:
            self.cells.update(map(tuple, cells.tolist()))

    def stamp_raycast(self, origin_xy, targets_xy) -> bool:
        """Paint the lines from `origin_xy` toward each target. Returns False
        (and does nothing) when the drone has not moved far enough since the
        last stamp — hover otherwise repaints the same starburst every tick."""
        if targets_xy is None or np.asarray(targets_xy).shape[0] == 0:
            return False
        origin = np.asarray(origin_xy, dtype=np.float64).reshape(2)
        if (self._last_raycast_xy is not None
                and float(np.linalg.norm(origin - self._last_raycast_xy))
                < self.raycast_min_step_m):
            return False
        tgt = np.asarray(targets_xy, dtype=np.float64).reshape(-1, 2)
        delta = tgt - origin[None, :]
        dist = np.linalg.norm(delta, axis=1)
        nz = dist > 1e-6
        if not np.any(nz):
            return False
        delta, dist = delta[nz], dist[nz]
        clamp = np.minimum(dist, self.raycast_range_m) - RAYCAST_PULLBACK_M
        keep = clamp > 0.0
        if not np.any(keep):
            self._last_raycast_xy = origin.copy()
            return False
        delta, dist, clamp = delta[keep], dist[keep], clamp[keep]
        unit = delta / dist[:, None]
        end = origin[None, :] + unit * clamp[:, None]
        step = max(self.cell_size_m * 0.5, 0.05)   # half-cell: no skipped cells
        n_steps = int(np.ceil(float(clamp.max()) / step)) + 1
        ts = np.linspace(0.0, 1.0, n_steps)[None, :, None]
        pts = origin[None, None, :] + (end[:, None, :] - origin[None, None, :]) * ts
        self.stamp_points(pts.reshape(-1, 2))
        self._last_raycast_xy = origin.copy()
        return True

    # ── reporting ───────────────────────────────────────────────────────────
    @property
    def cells_set(self) -> Set[Tuple[int, int]]:
        """The LIVE internal cell set — an alias, not a copy.

        `FrontierBehavior`'s anti-revisit term needs O(1) membership tests over
        the observed cells on every tick. At 0.5 m over a 250 m plate that set
        holds ~250k tuples, so copying it per tick (5 Hz) would cost more than
        the scoring it feeds. Callers must treat the result as READ-ONLY, and
        must not stash it expecting a snapshot: `stamp_points` /
        `stamp_raycast` mutate it in place.
        """
        return self.cells

    def cell_centers_xy(self) -> np.ndarray:
        if not self.cells:
            return np.zeros((0, 2), dtype=np.float64)
        # No sort: the caller only counts these, and a 250 m plate at 0.5 m
        # is a quarter of a million cells every tick.
        arr = np.array(list(self.cells), dtype=np.float64)
        return (arr + 0.5) * self.cell_size_m

    def coverage_fraction(self, polygon_xy: Optional[np.ndarray]) -> float:
        area = polygon_area_xy(polygon_xy)
        centers = self.cell_centers_xy()
        if area <= 0.0 or centers.shape[0] == 0:
            self.fraction = 0.0
            return 0.0
        inside = points_in_polygon(centers, polygon_xy)
        n = int(np.count_nonzero(inside))
        self.fraction = float(min(n * (self.cell_size_m ** 2) / area, 1.0))
        return self.fraction

    def new_milestones(self, fraction: float):
        """Yield each newly crossed 10% bucket, once per run."""
        bucket = min(int(fraction * 10), 10)
        while self.milestone < bucket:
            self.milestone += 1
            if self.milestone > 0:
                yield self.milestone * 10

    def packed_grid(self):
        """(resolution, origin_x, origin_y, width, height, data) for
        coordination_msgs/CoverageGrid — a row-major packed bitmask."""
        if not self.cells:
            return (self.cell_size_m, 0.0, 0.0, 0, 0, b'')
        arr = np.array(list(self.cells), dtype=np.int64)
        min_c, max_c = arr.min(axis=0), arr.max(axis=0)
        width = int(max_c[0] - min_c[0] + 1)
        height = int(max_c[1] - min_c[1] + 1)
        occ = np.zeros((height, width), dtype=np.uint8)
        occ[arr[:, 1] - min_c[1], arr[:, 0] - min_c[0]] = 1
        return (self.cell_size_m,
                float(min_c[0]) * self.cell_size_m,
                float(min_c[1]) * self.cell_size_m,
                width, height,
                np.packbits(occ.reshape(-1)).tobytes())
