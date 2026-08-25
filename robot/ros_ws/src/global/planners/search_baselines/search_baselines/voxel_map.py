"""3D occupancy with ray carving, and rayfronts-style frontier extraction.

WHY THIS EXISTS. Co-NavGPT2 collapses everything into ONE altitude slab:
`Map_Extraction` rasterises the cloud into a 2D grid using a single height band,
so every frontier it produces sits at the same height by construction. That is
correct for a quadruped in a house and wrong for a drone, which can go over,
under and between things.

WHAT RAYFRONTS ACTUALLY DOES (`mapping/frontier_vdb_map.py`):

    parallel_filter_cells_in_bbox(
        occ_map_vdb,
        cell_type_to_iterate = CellType.Empty,   # iterate observed-FREE voxels
        neighborhood_r       = r,
        min_unobserved       = n,                # ... with >= n UNOBSERVED neighbours
        min_empty            = ...,
        min_occupied         = ...)

so a frontier is an EMPTY voxel standing next to UNOBSERVED space. Reproduced
here, in numpy, over a bounded volume.

The part that cannot be skipped is the THREE-STATE map. "Not occupied" is not
the same as "empty": a voxel is only empty once something has actually looked
through it. That distinction is the whole basis of a frontier, and getting it
needs RAY CARVING — walk from the camera to each depth return marking free, and
mark the return itself occupied. Everything untouched stays unobserved.

Carving is sampled rather than exact-DDA: `carve_samples` points along each ray.
A sample count near `max_range / voxel` visits every voxel a DDA would, and
below that it can skip cells in the far field where the ray is long and the
voxels are coarse. That trade is deliberate — an exact DDA in numpy costs more
than the frontier extraction it feeds.
"""

import numpy as np

try:
    from scipy import ndimage
    _HAVE_SCIPY = True
except ImportError:                                   # pragma: no cover
    _HAVE_SCIPY = False

UNOBSERVED = 0
EMPTY = 1
OCCUPIED = 2


class VoxelMap:
    """Three-state occupancy over a bounded, axis-aligned volume.

    Dense rather than sparse on purpose: the volume is bounded by the search
    area and the altitude band, so at the sizes this runs at the whole grid is a
    few MB of uint8 and every operation is a vectorised numpy call. A VDB buys
    nothing until the volume is unbounded.
    """

    def __init__(self, bounds_min, bounds_max, voxel_m):
        self.vox = float(voxel_m)
        self.lo = np.asarray(bounds_min, dtype=np.float64)
        self.hi = np.asarray(bounds_max, dtype=np.float64)
        self.dims = np.maximum(
            1, np.ceil((self.hi - self.lo) / self.vox).astype(int))
        self.grid = np.zeros(tuple(self.dims), dtype=np.uint8)  # UNOBSERVED

    # ── indexing ──────────────────────────────────────────────────────────
    def to_idx(self, pts):
        """(N,3) world -> (N,3) int index, unclipped."""
        return np.floor((np.asarray(pts, dtype=np.float64) - self.lo)
                        / self.vox).astype(np.int64)

    def to_world(self, idx):
        """(N,3) index -> (N,3) world centre."""
        return self.lo + (np.asarray(idx, dtype=np.float64) + 0.5) * self.vox

    def _inside(self, idx):
        return np.all((idx >= 0) & (idx < self.dims), axis=1)

    def _flat(self, idx):
        return (idx[:, 0] * self.dims[1] * self.dims[2]
                + idx[:, 1] * self.dims[2] + idx[:, 2])

    # ── integration ───────────────────────────────────────────────────────
    def integrate(self, cam_xyz, points, carve_samples=24, max_points=20000,
                  rng=None):
        """Carve free space to `points` and mark the returns occupied.

        Occupied is written AFTER empty so a surface is not erased by the rays
        of its neighbours: many rays pass close to a wall and their samples land
        in the wall's own voxel, and if free won the surface would dissolve.
        """
        pts = np.asarray(points, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] == 0:
            return 0, 0
        if pts.shape[0] > max_points:
            rng = rng or np.random.default_rng(0)
            pts = pts[rng.choice(pts.shape[0], max_points, replace=False)]

        cam = np.asarray(cam_xyz, dtype=np.float64).reshape(1, 3)

        # EMPTY: samples along each ray, stopping just short of the return so
        # the surface voxel is not carved by its own ray.
        t = np.linspace(0.0, 0.97, int(max(2, carve_samples)))[None, :, None]
        free = cam[:, None, :] + (pts[:, None, :] - cam[:, None, :]) * t
        fidx = self.to_idx(free.reshape(-1, 3))
        fidx = fidx[self._inside(fidx)]
        n_free = 0
        if fidx.shape[0]:
            flat = np.unique(self._flat(fidx))
            self.grid.reshape(-1)[flat] = EMPTY
            n_free = flat.size

        # OCCUPIED last, so surfaces survive.
        oidx = self.to_idx(pts)
        oidx = oidx[self._inside(oidx)]
        n_occ = 0
        if oidx.shape[0]:
            flat = np.unique(self._flat(oidx))
            self.grid.reshape(-1)[flat] = OCCUPIED
            n_occ = flat.size
        return n_free, n_occ

    # ── frontiers ─────────────────────────────────────────────────────────
    def frontiers(self, neighborhood_r=1, min_unobserved=1, min_empty=1,
                  min_occupied=0, z_range=None, subsample=1):
        """Frontier voxel CENTRES as (N,3) world points.

        Same predicate rayfronts uses: iterate EMPTY voxels, keep those with at
        least `min_unobserved` unobserved, `min_empty` empty and `min_occupied`
        occupied neighbours inside a (2r+1)^3 box.

        `min_occupied` defaults to 0 — rayfronts can demand a nearby surface to
        suppress frontiers floating in open sky, which matters indoors where
        everything is near a wall. Outdoors from the air it would delete exactly
        the frontiers that matter, so it is off unless asked for.
        """
        if not _HAVE_SCIPY:
            raise RuntimeError('scipy is required for frontier extraction')
        k = 2 * int(neighborhood_r) + 1
        counts = {}
        for state, name in ((UNOBSERVED, 'unobs'), (EMPTY, 'empty'),
                            (OCCUPIED, 'occ')):
            m = (self.grid == state).astype(np.float32)
            # uniform_filter is a separable box mean; x k^3 recovers the count.
            counts[name] = ndimage.uniform_filter(
                m, size=k, mode='constant', cval=0.0) * (k ** 3)

        cand = (self.grid == EMPTY)
        cand &= counts['unobs'] >= float(min_unobserved)
        cand &= counts['empty'] >= float(min_empty)
        if min_occupied > 0:
            cand &= counts['occ'] >= float(min_occupied)

        if z_range is not None:
            zc = self.lo[2] + (np.arange(self.dims[2]) + 0.5) * self.vox
            zmask = (zc >= z_range[0]) & (zc <= z_range[1])
            cand &= zmask[None, None, :]

        idx = np.argwhere(cand)
        if idx.shape[0] == 0:
            return np.zeros((0, 3)), np.zeros((0,))
        unobs = counts['unobs'][cand]
        if subsample > 1:
            # rayfronts subsamples on a coarser voxel grid and keeps the count;
            # same idea, done by integer division of the index.
            coarse = idx // int(subsample)
            _, first, inv = np.unique(coarse, axis=0, return_index=True,
                                      return_inverse=True)
            gain = np.zeros(first.size)
            np.add.at(gain, inv, unobs)
            idx, unobs = idx[first], gain
        return self.to_world(idx), unobs

    def as_points(self, states=(EMPTY, OCCUPIED), max_points=200000, rng=None):
        """Voxel centres and an RGB per state, for a PointCloud2.

        UNOBSERVED is omitted by default — it is the overwhelming majority of any
        bounded volume early in a run, and drawing it would bury the two states
        that carry information under a solid block.
        """
        out_xyz, out_rgb = [], []
        colours = {EMPTY: (0.15, 0.85, 0.25), OCCUPIED: (0.9, 0.2, 0.15),
                   UNOBSERVED: (0.35, 0.35, 0.4)}
        for st in states:
            idx = np.argwhere(self.grid == st)
            if idx.shape[0] == 0:
                continue
            out_xyz.append(self.to_world(idx))
            out_rgb.append(np.tile(np.asarray(colours[st], dtype=np.float32),
                                   (idx.shape[0], 1)))
        if not out_xyz:
            return np.zeros((0, 3)), np.zeros((0, 3))
        xyz = np.concatenate(out_xyz, axis=0)
        rgb = np.concatenate(out_rgb, axis=0)
        if xyz.shape[0] > max_points:
            rng = rng or np.random.default_rng(0)
            sel = rng.choice(xyz.shape[0], max_points, replace=False)
            xyz, rgb = xyz[sel], rgb[sel]
        return xyz, rgb

    def stats(self):
        tot = self.grid.size
        return {
            'dims': [int(d) for d in self.dims],
            'voxel_m': self.vox,
            'unobserved': int((self.grid == UNOBSERVED).sum()),
            'empty': int((self.grid == EMPTY).sum()),
            'occupied': int((self.grid == OCCUPIED).sum()),
            'observed_frac': float((self.grid != UNOBSERVED).sum()) / tot,
        }
