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
        # WHEN each voxel was last written (integrate count). The map is a
        # NAVIGATION aid — free space to fly through and frontiers to pick —
        # not a survey of the scene, so `forget_older_than` lets the far past
        # go back to UNOBSERVED and the memory stays local to the drone.
        self._stamp = np.zeros(tuple(self.dims), dtype=np.uint32)
        self.tick = 0
        # Persistent frontier set, rayfronts-style: {flat_index: gain}.
        # Frontiers ACCUMULATE and are invalidated only where a new
        # observation actually touched the map — see `frontiers_persistent`.
        self._fronti = {}
        self._last_bbox = None   # (i0,j0,k0,i1,j1,k1) of the last integrate

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
                  rng=None, max_range_m=None):
        """Carve free space to `points` and mark the returns occupied.

        Occupied is written AFTER empty so a surface is not erased by the rays
        of its neighbours: many rays pass close to a wall and their samples land
        in the wall's own voxel, and if free won the surface would dissolve.

        `max_range_m` is the range past which stereo depth is not trusted. A
        return beyond it means "nothing found within range", NOT a surface: the
        ray is carved out to the limit and no voxel is marked occupied. Without
        this every frame stamps a shell of false obstacles at the depth horizon,
        which is what puts solid voxels in open air at heights nothing reaches.
        """
        pts = np.asarray(points, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] == 0:
            return 0, 0
        if pts.shape[0] > max_points:
            rng = rng or np.random.default_rng(0)
            pts = pts[rng.choice(pts.shape[0], max_points, replace=False)]

        cam = np.asarray(cam_xyz, dtype=np.float64).reshape(1, 3)

        # Split at the range limit: everything carves, only in-range returns
        # occupy.
        occ_pts = pts
        if max_range_m is not None and max_range_m > 0.0:
            d = np.linalg.norm(pts - cam, axis=1)
            far = d > float(max_range_m)
            if far.any():
                dirs = (pts - cam) / np.maximum(d[:, None], 1e-9)
                pts = np.where(far[:, None], cam + dirs * float(max_range_m), pts)
                occ_pts = pts[~far]

        # EMPTY: samples along each ray, stopping just short of the return so
        # the surface voxel is not carved by its own ray.
        t = np.linspace(0.0, 0.97, int(max(2, carve_samples)))[None, :, None]
        free = cam[:, None, :] + (pts[:, None, :] - cam[:, None, :]) * t
        fidx = self.to_idx(free.reshape(-1, 3))
        fidx = fidx[self._inside(fidx)]
        self.tick += 1
        n_free = 0
        if fidx.shape[0]:
            flat = np.unique(self._flat(fidx))
            self.grid.reshape(-1)[flat] = EMPTY
            self._stamp.reshape(-1)[flat] = self.tick
            n_free = flat.size

        # The ACTIVE WINDOW: every voxel this observation could have changed.
        # Frontier refresh is confined to it, so a frontier the drone is no
        # longer looking at survives instead of being recomputed away.
        touched = fidx if fidx.shape[0] else None
        if occ_pts.shape[0]:
            _o = self.to_idx(occ_pts)
            _o = _o[self._inside(_o)]
            if _o.shape[0]:
                touched = _o if touched is None else np.vstack([touched, _o])
        if touched is not None and touched.shape[0]:
            lo = touched.min(axis=0)
            hi = touched.max(axis=0)
            self._last_bbox = (int(lo[0]), int(lo[1]), int(lo[2]),
                               int(hi[0]), int(hi[1]), int(hi[2]))

        # OCCUPIED last, so surfaces survive.
        if occ_pts.shape[0] == 0:
            return n_free, 0
        oidx = self.to_idx(occ_pts)
        oidx = oidx[self._inside(oidx)]
        n_occ = 0
        if oidx.shape[0]:
            flat = np.unique(self._flat(oidx))
            self.grid.reshape(-1)[flat] = OCCUPIED
            self._stamp.reshape(-1)[flat] = self.tick
            n_occ = flat.size
        return n_free, n_occ

    def forget_older_than(self, max_age_ticks):
        """Voxels not written for more than `max_age_ticks` integrates go
        back to UNOBSERVED, and the persistent frontiers standing in them are
        dropped. Returns (voxels forgotten, frontiers dropped). 0 disables."""
        max_age = int(max_age_ticks)
        if max_age <= 0:
            return 0, 0
        old = (self.grid != UNOBSERVED) & ((self.tick - self._stamp) > max_age)
        n = int(old.sum())
        if n:
            self.grid[old] = UNOBSERVED
        dropped = 0
        if self._fronti:
            keep = {}
            for kk, v in self._fronti.items():
                idx = np.clip(np.round(v[0]).astype(int), 0, self.dims - 1)
                if self.grid[idx[0], idx[1], idx[2]] == UNOBSERVED:
                    dropped += 1
                else:
                    keep[kk] = v
            self._fronti = keep
        return n, dropped

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

    def frontiers_persistent(self, neighborhood_r=1, min_unobserved=4,
                             min_empty=2, min_occupied=0, subsampling=4,
                             subsampling_min_fronti=10, z_range=None):
        """Frontiers the way rayfronts maintains them: ACCUMULATE globally,
        invalidate locally.

        `frontiers()` recomputes the whole set from the whole map on every
        call. That makes the candidate set unstable: the frontier a robot
        committed to is usually absent from the next call (it either got
        observed on arrival, or fell out of a top-N), so a goal cannot be held
        and the robot parks on a target nothing re-selects.

        rayfronts instead keeps a persistent set and only re-evaluates the
        ACTIVE WINDOW — the bbox of the voxels the latest observation actually
        wrote (`frontier_vdb_map.update_frontiers`):

            outside = frontiers outside the bbox      -> KEPT untouched
            inside  = recomputed from the current map -> REPLACES the old ones

        so a frontier behind the robot survives until the robot looks there
        again. Defaults are rayfronts' own (r=1, min_unobserved=4,
        min_empty=2, min_occupied=0, subsampling=4, min_fronti=10).

        Returns (frontiers (N,3) world, gain (N,)) over the WHOLE accumulated
        set, not just this tick's window.
        """
        if not _HAVE_SCIPY:
            raise RuntimeError('scipy is required for frontier extraction')
        bbox = self._last_bbox
        if bbox is None:
            return np.zeros((0, 3)), np.zeros((0,))

        r = int(neighborhood_r)
        i0, j0, k0, i1, j1, k1 = bbox
        # Pad by r so a cell on the window edge still sees its full
        # neighbourhood, then clip back to the window when selecting.
        pi0, pj0, pk0 = max(0, i0 - r), max(0, j0 - r), max(0, k0 - r)
        pi1 = min(self.dims[0] - 1, i1 + r)
        pj1 = min(self.dims[1] - 1, j1 + r)
        pk1 = min(self.dims[2] - 1, k1 + r)
        sub = self.grid[pi0:pi1 + 1, pj0:pj1 + 1, pk0:pk1 + 1]
        if sub.size == 0:
            return self._fronti_arrays()

        k = 2 * r + 1
        box = np.ones((k, k, k), dtype=np.float32)
        cnt = {}
        for state, name in ((UNOBSERVED, 'unobs'), (EMPTY, 'empty'),
                            (OCCUPIED, 'occ')):
            m = (sub == state).astype(np.float32)
            cnt[name] = ndimage.convolve(m, box, mode='constant', cval=0.0)
        # The cell itself is counted by the box; discount it.
        sel = ((sub == EMPTY)
               & (cnt['unobs'] >= min_unobserved)
               & (cnt['empty'] - 1 >= min_empty)
               & (cnt['occ'] >= min_occupied))
        # Only cells genuinely inside the active window are authoritative;
        # the pad ring exists to give them correct neighbour counts.
        sel[:i0 - pi0, :, :] = False
        sel[:, :j0 - pj0, :] = False
        sel[:, :, :k0 - pk0] = False
        if pi1 > i1: sel[i1 - pi0 + 1:, :, :] = False
        if pj1 > j1: sel[:, j1 - pj0 + 1:, :] = False
        if pk1 > k1: sel[:, :, k1 - pk0 + 1:] = False

        idx = np.argwhere(sel)
        if idx.shape[0]:
            idx = idx + np.array([pi0, pj0, pk0])
            gains = cnt['unobs'][sel]
        else:
            gains = np.zeros((0,))

        if z_range is not None and idx.shape[0]:
            zw = self.to_world(idx)[:, 2]
            keep = (zw >= z_range[0]) & (zw <= z_range[1])
            idx, gains = idx[keep], gains[keep]

        # Voxel-grid subsample, as rayfronts does: group on a lattice
        # `subsampling` times coarser and keep only groups with enough
        # frontier cells in them, so a single stray voxel is not a frontier.
        ss = max(1, int(subsampling))
        fresh = {}
        if idx.shape[0]:
            keys = idx // ss
            for n in range(idx.shape[0]):
                kk = (int(keys[n, 0]), int(keys[n, 1]), int(keys[n, 2]))
                e = fresh.get(kk)
                if e is None:
                    fresh[kk] = [idx[n].astype(np.float64), float(gains[n]), 1]
                else:
                    e[0] = e[0] + idx[n]
                    e[1] += float(gains[n])
                    e[2] += 1
            fresh = {kk: v for kk, v in fresh.items()
                     if v[2] >= subsampling_min_fronti}

        # Invalidate the window in the accumulated set, then insert the fresh
        # ones. A key survives only if its cell lies OUTSIDE the active bbox.
        lo = (i0 // ss, j0 // ss, k0 // ss)
        hi = (i1 // ss, j1 // ss, k1 // ss)
        self._fronti = {
            kk: v for kk, v in self._fronti.items()
            if not (lo[0] <= kk[0] <= hi[0] and lo[1] <= kk[1] <= hi[1]
                    and lo[2] <= kk[2] <= hi[2])}
        for kk, v in fresh.items():
            self._fronti[kk] = (v[0] / v[2], v[1] / v[2])
        return self._fronti_arrays()

    def _fronti_arrays(self):
        """The accumulated set as (world (N,3), gain (N,))."""
        if not self._fronti:
            return np.zeros((0, 3)), np.zeros((0,))
        idx = np.array([v[0] for v in self._fronti.values()], dtype=np.float64)
        gain = np.array([v[1] for v in self._fronti.values()], dtype=np.float64)
        return self.to_world(idx), gain

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
