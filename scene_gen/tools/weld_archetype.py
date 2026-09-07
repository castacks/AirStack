#!/usr/bin/env python3
"""weld_archetype — merge duplicate vertices an archetype ships redundantly.

    AirStack/.venv/bin/python scene_gen/tools/weld_archetype.py <in.usd> [out.usd]

`mesh_damage.Soup` is UNWELDED by design -- three vertices per triangle -- so
that `_clip_by_plane` can interpolate a per-vertex UV at every cut. That is
right during the fracture and wrong afterwards: nothing merges them back, so
archetypes reach disk carrying up to 61% redundant vertices (measured across
five of them, 2026-08-31).

WELDS ON (position, st) TOGETHER, never position alone. Two vertices at the
same point but on opposite sides of a UV seam are genuinely different vertices,
and merging them drags the texture across the seam. That is why the naive
position-only figure is an upper bound and this saves less than it.

Meshes whose `st` is faceVarying are left alone: a faceVarying primvar has one
value per face-corner, not per point, so the point array is not what carries
the seam and welding it would need the corners re-split.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def weld_stage(stage, pos_tol=1e-5, uv_tol=1e-6):
    from pxr import UsdGeom, Sdf, Vt
    import numpy as np
    before = after = 0
    skipped = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not cnt or not idx:
            continue
        P = np.asarray(pts, dtype=np.float64)
        before += len(P)

        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        ST = None
        if pv:
            if pv.GetInterpolation() == UsdGeom.Tokens.faceVarying:
                skipped += 1
                after += len(P)
                continue
            v = pv.Get()
            if v is not None and len(v) == len(P):
                ST = np.asarray(v, dtype=np.float64)

        key = np.round(P / pos_tol).astype(np.int64)
        if ST is not None:
            key = np.c_[key, np.round(ST / uv_tol).astype(np.int64)]
        view = np.ascontiguousarray(key).view(
            [('', key.dtype)] * key.shape[1]).ravel()
        _u, first, inv = np.unique(view, return_index=True,
                                   return_inverse=True)
        if len(first) == len(P):
            after += len(P)
            continue
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
            P[first].astype(np.float32)))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(
            inv[np.asarray(idx, dtype=np.int64)].astype(np.int32)))
        if ST is not None:
            pv.Set(Vt.Vec2fArray.FromNumpy(ST[first].astype(np.float32)))

        # EVERY OTHER PER-POINT ARRAY HAS TO MOVE TOO. Rewriting `points` and
        # leaving a vertex-interpolated `normals` (or displayColor, or a
        # tangent) at its old length leaves the mesh internally inconsistent,
        # and Hydra says so at load:
        #   has corrupted data in primvar 'normal': buffer size 15319
        #   doesn't match expected size 14191 in vertex primvars
        # faceVarying primvars are indexed per face-corner, and neither the
        # face count nor the corner order changes here, so they are correct
        # as they stand and must NOT be touched.
        per_point = (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying)
        for other in UsdGeom.PrimvarsAPI(m).GetPrimvars():
            if pv and other.GetName() == pv.GetName():
                continue
            if other.GetInterpolation() not in per_point:
                continue
            val = other.Get()
            if val is None or len(val) != len(P):
                continue
            other.Set(type(val).FromNumpy(np.asarray(val)[first]))
        na = m.GetNormalsAttr()
        if na.HasAuthoredValue() and m.GetNormalsInterpolation() in per_point:
            nv = na.Get()
            if nv is not None and len(nv) == len(P):
                na.Set(type(nv).FromNumpy(np.asarray(nv)[first]))
        after += len(first)
    return before, after, skipped


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src")
    ap.add_argument("out", nargs="?", default="")
    ap.add_argument("--pos-tol", type=float, default=1e-5,
                    help="metres; vertices closer than this may merge. Past a "
                         "few millimetres this stops being a de-duplication "
                         "and starts being a decimation -- distinct corners "
                         "collapse onto each other and faces go degenerate")
    ap.add_argument("--uv-tol", type=float, default=1e-6)
    a = ap.parse_args(argv)
    out = a.out or (os.path.splitext(a.src)[0] + ".welded.usd")

    from pxr import Usd
    t0 = time.time()
    shutil.copy2(a.src, out)
    t_copy = time.time() - t0
    stage = Usd.Stage.Open(out)
    t1 = time.time()
    before, after, skipped = weld_stage(stage, a.pos_tol, a.uv_tol)
    t_weld = time.time() - t1
    t2 = time.time()
    stage.GetRootLayer().Save()
    t_save = time.time() - t2

    sb, sa = os.path.getsize(a.src), os.path.getsize(out)
    print(f"  points   {before:>12,} -> {after:>12,}   "
          f"({100*(before-after)/before:.1f}% removed)")
    # Welding usually makes the file BIGGER: crate compresses the largely
    # sequential index buffer well, and remapping scrambles that order. Saying
    # "-23.6% smaller" for a file that grew reads as a win.
    print(f"  file     {sb/1e6:>9.1f} MB -> {sa/1e6:>9.1f} MB   "
          f"({abs(100*(sb-sa)/sb):.1f}% "
          f"{'smaller' if sa <= sb else 'LARGER'})")
    print(f"  time     copy {t_copy:.1f}s  weld {t_weld:.1f}s  save {t_save:.1f}s"
          f"   total {t_copy+t_weld+t_save:.1f}s")
    if skipped:
        print(f"  skipped  {skipped} mesh(es) with faceVarying st")
    return 0


if __name__ == "__main__":
    sys.exit(main())
