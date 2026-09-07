#!/usr/bin/env python3
"""smooth_rubble — take the sawteeth off fractured rubble.

    AirStack/.venv/bin/python scene_gen/tools/smooth_rubble.py <in.usd> <out.usd> \
        --method taubin --iters 4 [--target rubble]

Voronoi fracture leaves two kinds of ugliness: SLIVERS (near-degenerate
triangles along a clip boundary, which read as sawteeth) and hard corners where
three cut planes meet. The methods here attack them differently:

  slivers   drop triangles whose area or aspect ratio is degenerate. Removes
            the sawtooth without touching anything else, but leaves holes where
            the sliver was load-bearing.
  laplace   move each vertex toward the average of its neighbours. Cheap and
            effective, and it SHRINKS -- a rubble pile visibly deflates.
  taubin    laplace with an alternating negative pass (mu = -1.02*lambda) that
            re-inflates. Costs two passes per iteration and roughly preserves
            volume, which is what you want on debris that has to keep sitting
            where the settle put it.

WELDING FIRST IS NOT OPTIONAL. `mesh_damage.Soup` is unwelded -- three vertices
per triangle, none shared -- so a vertex has no neighbours and every smoothing
method here is a no-op. This welds on (position, st) before smoothing, which is
the same operation `weld_archetype.py` does and is why the two belong together.

BOUNDARY VERTICES ARE PINNED by default: a fragment is an open shell, and
smoothing its rim drags the silhouette inward and opens gaps against the
neighbours it was cut from.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _weld(P, idx, ST, pos_tol=1e-5, uv_tol=1e-6):
    key = np.round(P / pos_tol).astype(np.int64)
    if ST is not None:
        key = np.c_[key, np.round(ST / uv_tol).astype(np.int64)]
    view = np.ascontiguousarray(key).view([('', key.dtype)] * key.shape[1]).ravel()
    _u, first, inv = np.unique(view, return_index=True, return_inverse=True)
    return P[first], inv[idx], (ST[first] if ST is not None else None), inv


def _tris(counts, idx):
    counts = np.asarray(counts, dtype=np.int64)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = []
    for c, s in zip(counts, starts):
        f = idx[s:s + c]
        for k in range(1, c - 1):
            out.append((f[0], f[k], f[k + 1]))
    return np.asarray(out, dtype=np.int64) if out else np.zeros((0, 3), np.int64)


def _adjacency(P, F):
    """Neighbour sum / count per vertex, and which vertices are on a boundary."""
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    both = np.concatenate([e, e[:, ::-1]])
    nsum = np.zeros_like(P)
    np.add.at(nsum, both[:, 0], P[both[:, 1]])
    ncnt = np.bincount(both[:, 0], minlength=len(P)).astype(np.float64)
    # An edge used by ONE triangle is a boundary edge; its vertices get pinned.
    se = np.sort(e, axis=1)
    _u, inv, cts = np.unique(se, axis=0, return_inverse=True, return_counts=True)
    bnd = np.zeros(len(P), dtype=bool)
    bedge = se[np.isin(inv, np.nonzero(cts == 1)[0])] if len(cts) else se[:0]
    if len(bedge):
        bnd[bedge.reshape(-1)] = True
    return nsum, np.maximum(ncnt, 1.0), bnd


def _smooth(P, F, method, iters, lam=0.5, pin=True):
    if not len(F):
        return P
    nsum, ncnt, bnd = _adjacency(P, F)
    mu = -1.02 * lam
    for _ in range(iters):
        for step in ((lam,) if method == "laplace" else (lam, mu)):
            nsum = np.zeros_like(P)
            e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
            both = np.concatenate([e, e[:, ::-1]])
            np.add.at(nsum, both[:, 0], P[both[:, 1]])
            avg = nsum / ncnt[:, None]
            delta = step * (avg - P)
            if pin:
                delta[bnd] = 0.0
            P = P + delta
    return P


def _drop_slivers(P, F, min_area=1e-4, max_aspect=30.0):
    a = P[F[:, 1]] - P[F[:, 0]]
    b = P[F[:, 2]] - P[F[:, 0]]
    area = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    e0 = np.linalg.norm(a, axis=1)
    e1 = np.linalg.norm(b, axis=1)
    e2 = np.linalg.norm(P[F[:, 2]] - P[F[:, 1]], axis=1)
    longest = np.maximum(np.maximum(e0, e1), e2)
    shortest = np.minimum(np.minimum(e0, e1), e2)
    aspect = longest / np.maximum(shortest, 1e-12)
    keep = (area > min_area) & (aspect < max_aspect)
    return F[keep], int((~keep).sum())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--method", default="taubin",
                    choices=["laplace", "taubin", "slivers"])
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--lam", type=float, default=0.5)
    ap.add_argument("--target", default="rubble",
                    help="prim-name prefix to touch; 'all' for every mesh")
    ap.add_argument("--no-pin", action="store_true")
    a = ap.parse_args(argv)

    from pxr import Usd, UsdGeom, Sdf, Vt
    t0 = time.time()
    shutil.copy2(a.src, a.out)
    stage = Usd.Stage.Open(a.out)
    t1 = time.time()
    touched = dropped = before = after = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if a.target != "all" and not prim.GetName().startswith(a.target):
            continue
        m = UsdGeom.Mesh(prim)
        pts, cnt, idx = (m.GetPointsAttr().Get(),
                         m.GetFaceVertexCountsAttr().Get(),
                         m.GetFaceVertexIndicesAttr().Get())
        if not pts or not cnt or not idx:
            continue
        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        ST = None
        if pv and pv.GetInterpolation() != UsdGeom.Tokens.faceVarying:
            v = pv.Get()
            if v is not None and len(v) == len(pts):
                ST = np.asarray(v, dtype=np.float64)
        P = np.asarray(pts, dtype=np.float64)
        before += len(P)
        Pw, idxw, STw, _ = _weld(P, np.asarray(idx, dtype=np.int64), ST)
        F = _tris(cnt, idxw)
        if a.method == "slivers":
            F, nd = _drop_slivers(Pw, F)
            dropped += nd
        else:
            Pw = _smooth(Pw, F, a.method, a.iters, a.lam, not a.no_pin)
        after += len(Pw)
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(Pw.astype(np.float32)))
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(
            np.full(len(F), 3, dtype=np.int32)))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(
            F.reshape(-1).astype(np.int32)))
        if STw is not None:
            pv.Set(Vt.Vec2fArray.FromNumpy(STw.astype(np.float32)))
        touched += 1
    t2 = time.time()
    stage.GetRootLayer().Save()
    print(f"  method {a.method:<8} iters {a.iters}  meshes {touched}"
          + (f"  slivers dropped {dropped:,}" if a.method == "slivers" else ""))
    print(f"  points {before:,} -> {after:,}   file "
          f"{os.path.getsize(a.src)/1e6:.1f} -> {os.path.getsize(a.out)/1e6:.1f} MB")
    print(f"  time   {t2-t1:.1f}s work, {time.time()-t0:.1f}s total")
    return 0


if __name__ == "__main__":
    sys.exit(main())
