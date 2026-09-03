"""Experimental Numba implementation of soot_bake.uv_position_map.

The production implementation is intentionally untouched.  ``install``
patches one process and retains the exact triangle order, texel-centre test,
barycentric interpolation, seam rule and last-triangle-wins behaviour.
"""

import math

import numpy as np

from . import soot_bake as base

try:
    from numba import njit
except ImportError:  # pragma: no cover - Isaac 5.1 currently ships numba
    njit = None


if njit is not None:
    @njit(cache=True, nogil=True)
    def _raster(points, tri, raw_uv, px):
        pos = np.zeros((px, px, 3), dtype=np.float32)
        mask = np.zeros((px, px), dtype=np.bool_)
        eps = 1e-6
        for t in range(tri.shape[0]):
            use = np.empty((3, 2), dtype=np.float64)
            umin, umax = 1e300, -1e300
            vmin, vmax = 1e300, -1e300
            for k in range(3):
                for a in range(2):
                    x = raw_uv[t, k, a]
                    w = x % 1.0
                    if w == 0.0 and x != 0.0:
                        w = 1.0
                    use[k, a] = w
                umin = min(umin, use[k, 0]); umax = max(umax, use[k, 0])
                vmin = min(vmin, use[k, 1]); vmax = max(vmax, use[k, 1])
            if umax - umin > 0.5 or vmax - vmin > 0.5:
                for k in range(3):
                    use[k, 0] = raw_uv[t, k, 0]
                    use[k, 1] = raw_uv[t, k, 1]

            xA, yA = use[0, 0] * px, (1.0 - use[0, 1]) * px
            xB, yB = use[1, 0] * px, (1.0 - use[1, 1]) * px
            xC, yC = use[2, 0] * px, (1.0 - use[2, 1]) * px
            denom = (yB - yC) * (xA - xC) + (xC - xB) * (yA - yC)
            if abs(denom) < 1e-9:
                continue
            cmin = int(math.floor(min(xA, xB, xC)))
            cmax = min(int(math.ceil(max(xA, xB, xC))), cmin + px)
            rmin = int(math.floor(min(yA, yB, yC)))
            rmax = min(int(math.ceil(max(yA, yB, yC))), rmin + px)
            if cmax <= cmin or rmax <= rmin:
                continue
            ia, ib, ic = tri[t, 0], tri[t, 1], tri[t, 2]
            for rr in range(rmin, rmax):
                pr = rr + 0.5
                for cc in range(cmin, cmax):
                    pc = cc + 0.5
                    wA = ((yB - yC) * (pc - xC)
                          + (xC - xB) * (pr - yC)) / denom
                    wB = ((yC - yA) * (pc - xC)
                          + (xA - xC) * (pr - yC)) / denom
                    wC = 1.0 - wA - wB
                    if wA < -eps or wB < -eps or wC < -eps:
                        continue
                    ri, ci = rr % px, cc % px
                    for a in range(3):
                        pos[ri, ci, a] = np.float32(
                            wA * points[ia, a] + wB * points[ib, a]
                            + wC * points[ic, a])
                    mask[ri, ci] = True
        return pos, mask


def uv_position_map(points, face_vertex_counts, face_vertex_indices, uv,
                    uv_interp, uv_indices=None, face_ids=None,
                    px=base.BAKE_PX):
    if njit is None:
        return base.uv_position_map(
            points, face_vertex_counts, face_vertex_indices, uv, uv_interp,
            uv_indices=uv_indices, face_ids=face_ids, px=px)
    if uv_interp not in ("vertex", "varying", "faceVarying"):
        raise ValueError("unknown uv_interp %r" % (uv_interp,))
    points = np.ascontiguousarray(points, dtype=np.float64)
    tri, _tri_face, tri_slot = base.triangles(
        face_vertex_counts, face_vertex_indices, face_ids)
    if not len(tri):
        return (np.zeros((px, px, 3), dtype=np.float32),
                np.zeros((px, px), dtype=bool))
    tri_uv = base._corner_uv(
        tri, tri_slot, uv, uv_interp, uv_indices)
    return _raster(points, np.ascontiguousarray(tri, dtype=np.int64),
                   np.ascontiguousarray(tri_uv, dtype=np.float64), int(px))


def install():
    if njit is None:
        print("[fast_soot] numba unavailable; using production rasterizer")
        return False
    base.uv_position_map = uv_position_map
    print("[fast_soot] Numba UV position-map rasterizer installed")
    return True
