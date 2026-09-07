#!/usr/bin/env python3
"""bevel_rubble — round fracture edges by CUTTING CORNERS, not by moving points.

    AirStack/.venv/bin/python scene_gen/tools/bevel_rubble.py <in.usd> <out.usd> \
        [--width 0.10] [--angle 40] [--passes 1] [--frag-max 0]

WHY A BEVEL AND NOT A SMOOTHER. Laplacian, Taubin and every other umbrella
operator can only MOVE the vertices a mesh already has. Hand one a triangular
prism and it has six points to work with, all of them on corners: the prism
comes back a slightly different prism, or inside out. Rounding a corner means
ADDING geometry there. Measured on this library, taubin at 200 iterations moved
points a mean of 0.76 m and made the rubble thinner, not rounder.

THE OPERATOR. Per face, pull each corner in toward the face centroid by
`width`; that opens a gap along every edge and a hole at every vertex, which
are then filled:

    shrunk face   one per original triangle, same material, same UVs
    bridge        the strip along an edge between two shrunk faces
    cap           a fan around a vertex, one triangle per incident edge

A corner only moves where the surface is actually creased -- `--angle` is the
dihedral threshold, so the flat interior of a fracture face (which triangulation
fills with near-coplanar edges) is left alone and the cost lands on the
silhouette rims that read as jagged. `--angle 0` beveles everything, which is
the classic truncation and is here mostly to show what it costs.

The bevel width at a vertex is clamped to a fraction of the shortest edge
meeting it, so thin debris plates cannot be cut through -- the failure mode
that made taubin dissolve `house_02`'s timber.

TOPOLOGY. Bridges need an edge with exactly two faces. The fracture soup is
welded on position first (see `refine_rubble`) which makes most of it manifold;
whatever is left is reported and skipped, leaving the original edge sharp.
"""
from __future__ import annotations
import argparse, os, shutil, sys, time
import numpy as np

_SG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SG not in sys.path:
    sys.path.insert(0, _SG)


def _tris(counts, idx):
    counts = np.asarray(counts, np.int64); idx = np.asarray(idx, np.int64)
    st = np.concatenate([[0], np.cumsum(counts)[:-1]])
    keep, out = counts >= 3, []
    for c, s in zip(counts[keep], st[keep]):
        f = idx[s:s + c]
        for k in range(1, c - 1):
            out.append((f[0], f[k], f[k + 1]))
    return np.asarray(out, np.int64) if out else np.zeros((0, 3), np.int64)


def _key(arr, tol):
    k = np.round(arr / tol).astype(np.int64)
    return np.ascontiguousarray(k).view([('', k.dtype)] * k.shape[1]).ravel()


def _components(n, F):
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    g = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)


def _pairs(Fn):
    """Manifold edges as (instance_a, instance_b). Instance i is corner i%3 of
    face i//3, running to the next corner -- so an instance names both the edge
    and which face/corner it came from, which is what the bridge needs."""
    T = len(Fn)
    u = Fn[:, [0, 1, 2]].T.ravel(order="F")
    v = Fn[:, [1, 2, 0]].T.ravel(order="F")
    lo, hi = np.minimum(u, v), np.maximum(u, v)
    key = np.c_[lo, hi]
    view = np.ascontiguousarray(key).view([('', key.dtype)] * 2).ravel()
    _uq, inv = np.unique(view, return_inverse=True)
    nE = inv.max() + 1 if len(inv) else 0
    cnt = np.bincount(inv, minlength=nE)
    order = np.argsort(inv, kind="stable")
    start = np.searchsorted(inv[order], np.arange(nE))
    two = cnt == 2
    return order[start[two]], order[start[two] + 1], u, v, int((cnt != 2).sum())


def bevel(Pn, Fn, d):
    """One pass. Returns (tri_pos, tri_corner) where tri_corner indexes the
    ORIGINAL corner each new vertex inherits its UV and material from; -1 marks
    a cap centre, which takes them from the triangle's first corner."""
    T = len(Fn)
    cen = Pn[Fn].mean(axis=1)
    C = np.empty((T, 3, 3))
    for k in range(3):
        idx = Fn[:, k]
        dv = cen - Pn[idx]
        L = np.linalg.norm(dv, axis=1, keepdims=True)
        step = np.minimum(d[idx][:, None], 0.45 * L)
        C[:, k, :] = Pn[idx] + dv / np.maximum(L, 1e-12) * step

    i0, i1, eu, _ev, n_bad = _pairs(Fn)
    a, b = eu[i0], eu[i1]                       # the two endpoint nodes
    live = (d[a] > 0) | (d[b] > 0)
    i0, i1, a, b = i0[live], i1[live], a[live], b[live]
    f0, k0 = i0 // 3, i0 % 3
    f1, k1 = i1 // 3, i1 % 3
    cu0, cv0 = f0 * 3 + k0, f0 * 3 + (k0 + 1) % 3
    cu1, cv1 = f1 * 3 + k1, f1 * 3 + (k1 + 1) % 3

    Cf = C.reshape(-1, 3)
    quad = [np.c_[cu0, cv0, cu1], np.c_[cu0, cu1, cv1]]      # bridge strip
    caps = []
    for node, c_f0, c_f1 in ((a, cu0, cv1), (b, cv0, cu1)):
        m = d[node] > 0
        if not m.any():
            continue
        caps.append((Pn[node[m]], np.c_[c_f0[m], c_f1[m]]))

    pos = [C]
    corner = [np.arange(T * 3).reshape(T, 3)]
    for q in quad:
        pos.append(Cf[q]); corner.append(q)
    for centre, cc in caps:
        p = np.empty((len(cc), 3, 3))
        p[:, 0, :] = centre; p[:, 1, :] = Cf[cc[:, 0]]; p[:, 2, :] = Cf[cc[:, 1]]
        pos.append(p)
        corner.append(np.c_[cc[:, 0], cc[:, 0], cc[:, 1]])   # centre borrows f0
    return np.concatenate(pos), np.concatenate(corner), n_bad


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src"); ap.add_argument("out")
    ap.add_argument("--width", type=float, default=0.10,
                    help="bevel cut-back in metres")
    ap.add_argument("--angle", type=float, default=40.0,
                    help="dihedral degrees above which an edge is creased; "
                         "0 beveles every corner (truncation)")
    ap.add_argument("--passes", type=int, default=1,
                    help="repeat with half the width each time -- 2 passes "
                         "reads as a rounded edge rather than a flat chamfer")
    ap.add_argument("--frag-max", type=float, default=0.0,
                    help="only bevel fragments smaller than this (0 = all)")
    ap.add_argument("--target", default="rubble,interior",
                    help="comma-separated mesh-name prefixes to process; "
                         "`interior_` comes out of the same merge as "
                         "`rubble_` and is duplicated the same way")
    ap.add_argument("--keep-duplicates", action="store_true",
                    help="skip the duplicate-triangle pass (see DUPLICATES)")
    ap.add_argument("--simplify", type=float, default=0.0,
                    help="snap points to a grid of this size first, collapsing "
                         "the facets a bevel cannot exceed (see TESSELLATION)")
    a = ap.parse_args(argv)

    _targets = tuple(t for t in (a.target or "").split(",") if t)

    from pxr import Usd, UsdGeom, Vt, Sdf
    t0 = time.time()
    shutil.copy2(a.src, a.out)
    stage = Usd.Stage.Open(a.out)

    prims, PP, UU, FF = [], [], [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.GetName().startswith(_targets):
            continue
        m = UsdGeom.Mesh(prim)
        pts, cnt, idx = (m.GetPointsAttr().Get(), m.GetFaceVertexCountsAttr().Get(),
                         m.GetFaceVertexIndicesAttr().Get())
        if not pts or not cnt or not idx:
            continue
        v = np.asarray(pts, np.float64)
        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        uvv = pv.Get() if pv else None
        uv = (np.asarray(uvv, np.float64) if uvv is not None and len(uvv) == len(v)
              else np.zeros((len(v), 2)))
        prims.append(prim); PP.append(v); UU.append(uv)
        FF.append(_tris(cnt, np.asarray(idx, np.int64)))
    if not prims:
        print("  nothing matched"); return 1

    off = np.cumsum([0] + [len(p) for p in PP])
    P_all = np.vstack(PP); U_all = np.vstack(UU)
    F_all = np.vstack([f + off[i] for i, f in enumerate(FF)])
    fprim = np.concatenate([np.full(len(f), i, np.int64) for i, f in enumerate(FF)])

    # adjacency weld -- position only, across prims (see refine_rubble)
    _u, firstp, node = np.unique(_key(P_all, 1e-4), return_index=True,
                                 return_inverse=True)
    Pn, Fn = P_all[firstp], node[F_all]

    # DUPLICATES. Every rubble triangle is authored TWICE, same winding, on
    # prims that are already doubleSided -- measured on house_02: 175,812 of
    # 352,368 triangles are an exact repeat of another. That makes every edge
    # carry four faces, so no edge is manifold and the bevel has nothing to
    # bridge. Dropping the repeat is also the single largest geometry saving
    # in this file, and it is invisible.
    n_dup = 0
    if not a.keep_duplicates:
        srt = np.sort(Fn, axis=1)
        sv = np.ascontiguousarray(srt).view([('', srt.dtype)] * 3).ravel()
        _uq, keep = np.unique(sv, return_index=True)
        keep = np.sort(keep)
        n_dup = len(Fn) - len(keep)
        F_all, Fn, fprim = F_all[keep], Fn[keep], fprim[keep]

    # TESSELLATION. A corner cut cannot be wider than the facets meeting at
    # that corner, and rubble arrives finely triangulated -- median edge 0.15 m
    # on shards 1-3 m across, so the clamp holds every chamfer under ~0.05 m and
    # nothing reads at building scale. Snapping to a grid first collapses the
    # small facets, which is what makes a visible bevel possible: the coarse
    # mesh is the POINT, not a compromise.
    n_simp = 0
    if a.simplify > 0:
        g = a.simplify
        Q = np.round(Pn / g) * g
        _uq, firstq, invq = np.unique(_key(Q, 1e-6), return_index=True,
                                      return_inverse=True)
        Pn2, Fn2 = Q[firstq], invq[Fn]
        live = ((Fn2[:, 0] != Fn2[:, 1]) & (Fn2[:, 1] != Fn2[:, 2])
                & (Fn2[:, 0] != Fn2[:, 2]))
        n_simp = len(Fn) - int(live.sum())
        Pn, Fn = Pn2, Fn2[live]
        F_all, fprim = F_all[live], fprim[live]
        srt = np.sort(Fn, axis=1)
        sv = np.ascontiguousarray(srt).view([('', srt.dtype)] * 3).ravel()
        _u2, keep2 = np.unique(sv, return_index=True)
        keep2 = np.sort(keep2)
        n_simp += len(Fn) - len(keep2)
        F_all, Fn, fprim = F_all[keep2], Fn[keep2], fprim[keep2]

    tri_corner = np.arange(len(F_all) * 3).reshape(-1, 3)   # into F_all.ravel()
    tri_face = np.arange(len(F_all))

    n_bad = 0
    for p in range(max(1, a.passes)):
        width = a.width / (2 ** p)
        # crease test
        i0, i1, eu, ev, bad = _pairs(Fn)
        n_bad = max(n_bad, bad)
        fn = np.cross(Pn[Fn[:, 1]] - Pn[Fn[:, 0]], Pn[Fn[:, 2]] - Pn[Fn[:, 0]])
        fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
        cosang = (fn[i0 // 3] * fn[i1 // 3]).sum(axis=1)
        ang = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
        sharp = ang >= a.angle
        d = np.zeros(len(Pn))
        for nn in (eu[i0[sharp]], eu[i1[sharp]]):
            d[nn] = width
        # Never cut through a thin plate: clamp to the shortest edge at a
        # node. DEGENERATE edges are excluded -- 428 zero-length edges in
        # house_02 alone, and one of them at a node would otherwise clamp the
        # bevel there to nothing.
        elen = np.linalg.norm(Pn[eu] - Pn[ev], axis=1)
        ok = elen > 1e-4
        shortest = np.full(len(Pn), np.inf)
        np.minimum.at(shortest, eu[ok], elen[ok])
        np.minimum.at(shortest, ev[ok], elen[ok])
        d = np.where(np.isfinite(shortest), np.minimum(d, 0.35 * shortest), 0.0)
        if a.frag_max > 0:
            nfrag, lab = _components(len(Pn), Fn)
            diag = np.zeros(nfrag)
            for c in range(nfrag):
                s = Pn[lab == c]
                if len(s):
                    diag[c] = np.linalg.norm(s.max(0) - s.min(0))
            d[diag[lab] >= a.frag_max] = 0.0
        if not (d > 0).any():
            print("  no creased edges at this angle"); break

        pos, corner, _b = bevel(Pn, Fn, d)
        # carry provenance: each new corner inherits an ORIGINAL corner
        src = tri_corner.ravel()[np.minimum(corner, tri_corner.size - 1)]
        tri_corner = src
        tri_face = tri_face[np.minimum(corner[:, 0] // 3, len(tri_face) - 1)]
        # re-weld for the next pass / for output
        flat = pos.reshape(-1, 3)
        _u2, first2, inv2 = np.unique(_key(flat, 1e-6), return_index=True,
                                      return_inverse=True)
        Pn, Fn = flat[first2], inv2.reshape(-1, 3)

    tri_prim = fprim[tri_face]
    Fsrc = F_all.ravel()

    # --- write back, welding each prim on (position, st) -------------------
    n_out = 0
    for i, prim in enumerate(prims):
        m = UsdGeom.Mesh(prim)
        sel = tri_prim == i
        f_local, c_local = Fn[sel], tri_corner[sel]
        if not len(f_local):
            continue
        vpos = Pn[f_local].reshape(-1, 3)
        vuv = U_all[Fsrc[c_local]].reshape(-1, 2)
        kk = np.c_[np.round(vpos / 1e-5).astype(np.int64),
                   np.round(vuv / 1e-6).astype(np.int64)]
        view = np.ascontiguousarray(kk).view([('', kk.dtype)] * kk.shape[1]).ravel()
        _u3, f3, inv3 = np.unique(view, return_index=True, return_inverse=True)
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(vpos[f3].astype(np.float32)))
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(
            np.full(len(f_local), 3, np.int32)))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(
            inv3.astype(np.int32)))
        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        if pv is None:
            pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.SetInterpolation(UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray.FromNumpy(vuv[f3].astype(np.float32)))
        # Authored normals belong to the old topology.
        if m.GetNormalsAttr().HasAuthoredValue():
            m.GetNormalsAttr().Clear()
        n_out += len(f3)

    stage.GetRootLayer().Save()
    print(f"  bevel w={a.width:g} ang={a.angle:g} x{a.passes}  "
          f"simp={a.simplify:g} dup {n_dup:,} collapsed {n_simp:,}  "
          f"tris {len(F_all):,} -> {len(Fn):,}  "
          f"points {len(P_all):,} -> {n_out:,}  non-manifold {n_bad:,}  "
          f"{time.time()-t0:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
