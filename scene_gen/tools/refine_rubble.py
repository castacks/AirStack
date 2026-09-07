#!/usr/bin/env python3
"""refine_rubble — per-FRAGMENT appearance passes that keep UVs intact.

    AirStack/.venv/bin/python scene_gen/tools/refine_rubble.py <in.usd> <out.usd> \
        --method taubin|shrink|cluster|normals|cull|hull [--frag-max 3.0]

TWO WELDS, FOR TWO DIFFERENT JOBS. Conflating them is what broke the first
version of this tool.

  ADJACENCY weld -- POSITION ONLY, across every rubble prim. The bake merges
  fragments BY MATERIAL, so one shard has its cut faces in
  `rubble_FractureCore_*` and its brick in `rubble_UnrealMaterial_*`; only a
  position weld rejoins them, and only then can a pass reach the silhouette
  seam between the two. Used to find neighbours and connected components --
  never to decide what gets written.

  OUTPUT weld -- (POSITION, ST) TOGETHER, within one prim. Merges exactly the
  duplicates the unwelded fracture soup creates, and never collapses a real UV
  seam. Measured: 56% of points removed with UVs intact.

Vertices split across a UV seam therefore stay separate in the file but share
one adjacency node, so a smoothing pass moves them together and the seam does
not tear. Welding on position alone for output instead removed 5x more points
for NO reliable memory gain (+1.17..1.39 GB vs +1.23 GB) while destroying every
UV -- measured 2026-08-31.
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
    out = []
    for c, s in zip(counts, st):
        f = idx[s:s+c]
        for k in range(1, c-1):
            out.append((f[0], f[k], f[k+1]))
    return np.asarray(out, np.int64) if out else np.zeros((0,3), np.int64)


def _key(arr, tol):
    k = np.round(arr / tol).astype(np.int64)
    return np.ascontiguousarray(k).view([('', k.dtype)] * k.shape[1]).ravel()


def _components(n, F):
    try:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        e = np.concatenate([F[:,[0,1]], F[:,[1,2]], F[:,[2,0]]])
        g = coo_matrix((np.ones(len(e)), (e[:,0], e[:,1])), shape=(n, n))
        return connected_components(g, directed=False)
    except Exception:
        return 1, np.zeros(n, np.int64)


def _taubin(P, F, iters, lam=0.5):
    e = np.concatenate([F[:,[0,1]], F[:,[1,2]], F[:,[2,0]]])
    both = np.concatenate([e, e[:,::-1]])
    cnt = np.maximum(np.bincount(both[:,0], minlength=len(P)), 1).astype(np.float64)
    for _ in range(iters):
        for step in (lam, -1.02*lam):
            s = np.zeros_like(P)
            np.add.at(s, both[:,0], P[both[:,1]])
            P = P + step * (s/cnt[:,None] - P)
    return P


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src"); ap.add_argument("out")
    ap.add_argument("--method", required=True,
                    choices=["taubin","shrink","cluster","normals","cull","hull"])
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--lam", type=float, default=0.5,
                    help="taubin shrink step; mu is fixed at -1.02*lam")
    ap.add_argument("--frag-max", type=float, default=3.0)
    ap.add_argument("--amount", type=float, default=0.12)
    ap.add_argument("--target", default="rubble")
    a = ap.parse_args(argv)

    from pxr import Usd, UsdGeom, Vt, Sdf
    t0 = time.time()
    shutil.copy2(a.src, a.out)
    stage = Usd.Stage.Open(a.out)

    prims, PP, UU, FF = [], [], [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.GetName().startswith(a.target):
            continue
        m = UsdGeom.Mesh(prim)
        pts, cnt, idx = (m.GetPointsAttr().Get(), m.GetFaceVertexCountsAttr().Get(),
                         m.GetFaceVertexIndicesAttr().Get())
        if not pts or not cnt or not idx: continue
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
    P_all = np.vstack(PP)
    F_all = np.vstack([f + off[i] for i, f in enumerate(FF)])
    fprim = np.concatenate([np.full(len(f), i, np.int64) for i, f in enumerate(FF)])

    # --- adjacency weld: position only, across prims -----------------------
    _u, firstp, node = np.unique(_key(P_all, 1e-4), return_index=True,
                                 return_inverse=True)
    Pn = P_all[firstp]; Fn = node[F_all]
    nfrag, lab = _components(len(Pn), Fn)
    diag = np.zeros(nfrag)
    for c in range(nfrag):
        s = Pn[lab == c]
        if len(s): diag[c] = np.linalg.norm(s.max(0) - s.min(0))
    small = diag < a.frag_max

    # --- run the method in NODE space, keep only the DISPLACEMENT ----------
    D = np.zeros_like(Pn); drop = np.zeros(nfrag, bool); hull_faces = None
    if a.method == "taubin":
        D = _taubin(Pn.copy(), Fn, a.iters, a.lam) - Pn
    elif a.method == "shrink":
        for c in np.nonzero(small)[0]:
            m_ = lab == c
            if m_.any():
                cen = Pn[m_].mean(0); D[m_] = (cen - Pn[m_]) * a.amount
    elif a.method == "cluster":
        g = max(a.amount, 1e-3)
        m_ = small[lab]
        D[m_] = np.round(Pn[m_]/g)*g - Pn[m_]
    elif a.method == "cull":
        drop = small
    elif a.method == "hull":
        from scipy.spatial import ConvexHull
        hull_faces = {}
        flab = lab[Fn[:,0]]
        for c in np.nonzero(small)[0]:
            vid = np.unique(Fn[flab == c])
            if len(vid) < 4: continue
            try: hull_faces[c] = vid[ConvexHull(Pn[vid]).simplices]
            except Exception: pass

    # --- write back, welding each prim on (position, st) -------------------
    changed = 0
    for i, prim in enumerate(prims):
        m = UsdGeom.Mesh(prim)
        v_local = PP[i] + D[node[off[i]:off[i+1]]]
        f_local = FF[i]
        if a.method == "cull":
            keep = ~drop[lab[node[f_local[:,0] + off[i]]]]
            f_local = f_local[keep]; changed += int((~keep).sum())
        elif a.method == "hull" and hull_faces:
            flab_i = lab[node[f_local[:,0] + off[i]]]
            keep = np.ones(len(f_local), bool)
            add = []
            for c, hf in hull_faces.items():
                sel = flab_i == c
                if not sel.any(): continue
                keep &= ~sel
                # hull is in NODE space; map to this prim's nearest local verts
                nl = node[off[i]:off[i+1]]
                lut = {n: j for j, n in enumerate(nl)}
                mapped = np.array([[lut.get(x, -1) for x in tri] for tri in hf])
                add.append(mapped[(mapped >= 0).all(axis=1)])
                changed += 1
            f_local = np.vstack([f_local[keep]] + [x for x in add if len(x)]) \
                if add else f_local[keep]
        if not len(f_local):
            for at in (m.GetPointsAttr(), m.GetFaceVertexCountsAttr(),
                       m.GetFaceVertexIndicesAttr()):
                at.Set(type(at.Get())() if at.Get() is not None else None)
            continue
        kk = np.c_[np.round(v_local/1e-5).astype(np.int64),
                   np.round(UU[i]/1e-6).astype(np.int64)]
        view = np.ascontiguousarray(kk).view([('', kk.dtype)]*kk.shape[1]).ravel()
        _u2, f2, inv2 = np.unique(view, return_index=True, return_inverse=True)
        f_new = inv2[f_local]
        used = np.unique(f_new); remap = np.zeros(len(f2), np.int64)
        remap[used] = np.arange(len(used))
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
            v_local[f2][used].astype(np.float32)))
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(
            np.full(len(f_new), 3, np.int32)))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(
            remap[f_new].reshape(-1).astype(np.int32)))
        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        if pv is None:
            pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.SetInterpolation(UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray.FromNumpy(UU[i][f2][used].astype(np.float32)))
        if a.method == "normals":
            tri = f_new.reshape(-1,3)
            Pw = v_local[f2][used]
            fn = np.cross(Pw[remap[tri[:,1]]]-Pw[remap[tri[:,0]]],
                          Pw[remap[tri[:,2]]]-Pw[remap[tri[:,0]]])
            acc = np.zeros((len(used),3))
            for k in range(3): np.add.at(acc, remap[tri[:,k]], fn)
            ln = np.linalg.norm(acc, axis=1, keepdims=True)
            m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(
                (acc/np.maximum(ln,1e-12)).astype(np.float32)))
            m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    stage.GetRootLayer().Save()
    tot = sum(len(p) for p in PP)
    # `changed` only counts topology edits, so a displacement method reports 0
    # and looks like a no-op. Say how far the points actually MOVED instead.
    dn = np.linalg.norm(D, axis=1)
    print(f"  {a.method:<8} shards {nfrag:,} (small {int(small.sum()):,}) "
          f"moved mean {dn.mean():.3f}m max {dn.max():.3f}m  "
          f"changed {changed:,}   points {tot:,} -> "
          f"{sum(len(UsdGeom.Mesh(p).GetPointsAttr().Get() or []) for p in prims):,}"
          f"   {time.time()-t0:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
