#!/usr/bin/env python
"""slice_bench.py — CURRENT slicer vs the keep-it-in-VTK architecture.

Same mesh, same partition, same VTK. Measures the four things the redesign is
supposed to remove:

  * NumPy->VTK conversion, once per clip
  * vtkTriangleFilter, once per clip
  * np.unique(axis=0) dedup, once per clip
  * clipping the FULL mesh twice per storey band instead of sweeping

CURRENT  = the shipped pattern: every clip is `_to_vtk -> vtkClipPolyData ->
           vtkTriangleFilter -> np.unique -> dict`, and each storey band is
           two independent clips of the WHOLE building.

SWEEP    = convert once; `GenerateClippedOutputOn()` so one Update() yields
           both halves; recurse into the shrinking remainder; stay in
           vtkPolyData throughout; dedup once per final piece.

PLANE    = the same sweep with `vtkPolyDataPlaneClipper`. It is threaded but
           has no complementary output, so each plane is run twice with
           opposite normals. Sequential and STDThread are measured together.

Equivalence is checked, not assumed: both paths must agree on total output
triangle count and on total surface area to 0.5 %, or the timing is
meaningless.

    uv run --with vtk --with numpy --with usd-core python slice_bench.py <asset.usd> [--storeys N] [--bays N]
"""
import argparse
import os
import sys
import time
from collections import defaultdict

import numpy as np
import vtk
from vtk.util import numpy_support as ns

DEDUP = True
T = defaultdict(float)
N = defaultdict(int)


class phase:
    def __init__(self, k): self.k = k
    def __enter__(self): self.t = time.perf_counter(); return self
    def __exit__(self, *a):
        T[self.k] += time.perf_counter() - self.t
        N[self.k] += 1


# ---------------------------------------------------------------- shared ---
def to_vtk(m):
    pd = vtk.vtkPolyData()
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(m["P"]), deep=True))
    pd.SetPoints(pts)
    f = m["tris"]
    cells = np.hstack([np.full((len(f), 1), 3, dtype=np.int64), f]).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(f), ns.numpy_to_vtkIdTypeArray(np.ascontiguousarray(cells),
                                                   deep=True))
    pd.SetPolys(ca)
    uv = ns.numpy_to_vtk(np.ascontiguousarray(m["UV"]), deep=True)
    uv.SetName("st")
    pd.GetPointData().SetTCoords(uv)
    mid = ns.numpy_to_vtk(np.ascontiguousarray(m["MID"]), deep=True,
                          array_type=vtk.VTK_INT)
    mid.SetName("mid")
    pd.GetCellData().AddArray(mid)
    return pd


def from_vtk(pd, dedup=True):
    """The shipped `_from_vtk`: triangulate, then merge position+uv."""
    with phase("triangulate"):
        tf = vtk.vtkTriangleFilter()
        tf.SetInputData(pd)
        tf.Update()
        out = tf.GetOutput()
    if out.GetNumberOfCells() == 0:
        return None
    with phase("vtk->numpy"):
        P = ns.vtk_to_numpy(out.GetPoints().GetData()).astype(np.float64)
        UVa = out.GetPointData().GetTCoords()
        UV = (ns.vtk_to_numpy(UVa).astype(np.float64) if UVa is not None
              else np.zeros((len(P), 2)))
        polys = ns.vtk_to_numpy(out.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        MIDa = out.GetCellData().GetArray("mid")
        MID = (ns.vtk_to_numpy(MIDa).astype(np.int32) if MIDa is not None
               else np.zeros(len(polys), np.int32))
    if dedup == "compact":
        # Keep every corner distinct (faceVarying UVs make that natural) but
        # DROP POINTS NO TRIANGLE REFERENCES. vtkClipPolyData's output point
        # list carries the originals of fully-clipped-away triangles, so a
        # raw no-dedup piece is not merely de-indexed, it is padded.
        with phase("compact (1-D)"):
            used, inv = np.unique(polys.ravel(), return_inverse=True)
            P2, UV2 = P[used], UV[used]
            tris = inv.reshape(-1, 3)
        return {"P": P2, "UV": UV2, "tris": tris.astype(np.int64), "MID": MID}
    if not dedup:
        return {"P": P, "UV": UV, "tris": polys.astype(np.int64), "MID": MID}
    with phase("np.unique dedup"):
        key = np.hstack([P, UV])
        _u, first, inv = np.unique(key, axis=0, return_index=True,
                                   return_inverse=True)
        P2, UV2 = P[first], UV[first]
        tris = inv[polys.ravel()].reshape(-1, 3)
    return {"P": P2, "UV": UV2, "tris": tris.astype(np.int64), "MID": MID}


def _clipper(pd, normal, origin, both=False):
    pl = vtk.vtkPlane()
    pl.SetOrigin(*[float(q) for q in origin])
    pl.SetNormal(*[float(q) for q in normal])
    cl = vtk.vtkClipPolyData()
    cl.SetInputData(pd)
    cl.SetClipFunction(pl)
    cl.InsideOutOn()
    cl.SetLocator(vtk.vtkNonMergingPointLocator())   # decal UVs — load-bearing
    if both:
        cl.GenerateClippedOutputOn()
    cl.Update()
    return cl


# --------------------------------------------------------------- CURRENT ---
def clip_current(m, normal, origin):
    """Exactly the shipped `clip()`: dict in, dict out, conversion each way."""
    with phase("numpy->vtk"):
        pd = to_vtk(m)
    with phase("clip Update()"):
        cl = _clipper(pd, normal, origin)
        out = cl.GetOutput()
    return from_vtk(out, dedup=True)


def run_current(m, z_cuts, x_cuts, y_cuts):
    pieces = []
    for lo, hi in zip(z_cuts[:-1], z_cuts[1:]):
        band = clip_current(m, (0.0, 0.0, 1.0), (0.0, 0.0, hi))     # below hi
        if band is None or len(band["tris"]) == 0:
            continue
        band = clip_current(band, (0.0, 0.0, -1.0), (0.0, 0.0, lo))  # above lo
        if band is None or len(band["tris"]) == 0:
            continue
        for xl, xh in zip(x_cuts[:-1], x_cuts[1:]):
            col = clip_current(band, (1.0, 0.0, 0.0), (xh, 0.0, 0.0))
            if col is None or len(col["tris"]) == 0:
                continue
            col = clip_current(col, (-1.0, 0.0, 0.0), (xl, 0.0, 0.0))
            if col is None or len(col["tris"]) == 0:
                continue
            for yl, yh in zip(y_cuts[:-1], y_cuts[1:]):
                p = clip_current(col, (0.0, 1.0, 0.0), (0.0, yh, 0.0))
                if p is None or len(p["tris"]) == 0:
                    continue
                p = clip_current(p, (0.0, -1.0, 0.0), (0.0, yl, 0.0))
                if p is not None and len(p["tris"]):
                    pieces.append(p)
    return pieces


# ----------------------------------------------------------------- SWEEP ---
def split_both(pd, normal, origin):
    """One Update(), both halves. `GetOutput()` is the kept (inside-out) side,
    `GetClippedOutput()` the other — so a sweep pays one clip per plane
    instead of two, and each subsequent cut sees only the remainder."""
    with phase("clip Update()"):
        cl = _clipper(pd, normal, origin, both=True)
        keep = vtk.vtkPolyData(); keep.ShallowCopy(cl.GetOutput())
        rest = vtk.vtkPolyData(); rest.ShallowCopy(cl.GetClippedOutput())
    return keep, rest


def _plane_clip(pd, normal, origin):
    """One side from vtkPolyDataPlaneClipper (which exposes no locator)."""
    pl = vtk.vtkPlane()
    pl.SetOrigin(*[float(q) for q in origin])
    pl.SetNormal(*[float(q) for q in normal])
    cl = vtk.vtkPolyDataPlaneClipper()
    cl.SetInputData(pd)
    cl.SetPlane(pl)
    cl.CappingOff()
    cl.ClippingLoopsOff()
    cl.Update()
    out = vtk.vtkPolyData()
    out.ShallowCopy(cl.GetOutput())
    return out


def split_both_plane(pd, normal, origin):
    """Both halves via two clips; PlaneClipper has no second-half output."""
    with phase("plane clip Update()"):
        # PlaneClipper retains the normal-facing side; the existing sweep's
        # `keep` convention is the opposite side (`InsideOutOn()`).
        keep = _plane_clip(pd, tuple(-q for q in normal), origin)
        rest = _plane_clip(pd, normal, origin)
    return keep, rest


def sweep(pd, axis, cuts, splitter=split_both):
    """Partition `pd` along `axis` at `cuts` (ascending), staying in VTK.
    Returns the slabs between consecutive cuts."""
    nrm = [0.0, 0.0, 0.0]; nrm[axis] = 1.0
    out, rest = [], pd
    for c in cuts[1:-1]:
        org = [0.0, 0.0, 0.0]; org[axis] = float(c)
        below, above = splitter(rest, tuple(nrm), tuple(org))
        out.append(below)
        rest = above
        if rest.GetNumberOfCells() == 0:
            break
    out.append(rest)
    return out


def run_sweep(m, z_cuts, x_cuts, y_cuts):
    with phase("numpy->vtk"):
        pd0 = to_vtk(m)
    # trim to the outer band once, then sweep — no re-conversion anywhere
    pieces = []
    for band in sweep(pd0, 2, z_cuts):
        if band.GetNumberOfCells() == 0:
            continue
        for col in sweep(band, 0, x_cuts):
            if col.GetNumberOfCells() == 0:
                continue
            for cell in sweep(col, 1, y_cuts):
                if cell.GetNumberOfCells() == 0:
                    continue
                d = from_vtk(cell, dedup=DEDUP)      # dedup ONCE, at the end
                if d is not None and len(d["tris"]):
                    pieces.append(d)
    return pieces


def run_plane_sweep(m, z_cuts, x_cuts, y_cuts):
    with phase("numpy->vtk"):
        pd0 = to_vtk(m)
    pieces = []
    for band in sweep(pd0, 2, z_cuts, splitter=split_both_plane):
        if band.GetNumberOfCells() == 0:
            continue
        for col in sweep(band, 0, x_cuts, splitter=split_both_plane):
            if col.GetNumberOfCells() == 0:
                continue
            for cell in sweep(col, 1, y_cuts, splitter=split_both_plane):
                if cell.GetNumberOfCells() == 0:
                    continue
                d = from_vtk(cell, dedup=DEDUP)
                if d is not None and len(d["tris"]):
                    pieces.append(d)
    return pieces


# ------------------------------------------------------------------- util ---
def area_of(pieces):
    tot = 0.0
    for d in pieces:
        P, t = d["P"], d["tris"]
        if len(t) == 0:
            continue
        a, b, c = P[t[:, 0]], P[t[:, 1]], P[t[:, 2]]
        tot += float(np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() * 0.5)
    return tot


def load(path):
    """Mesh -> {P, UV, tris, MID, MATS}, de-indexed, with REAL material ids.

    `MID` is an index into `MATS` (a list of material prim paths), taken from
    the source's `GeomSubset`s — not the mesh index. That matters twice:

      * the slicer carries `mid` as VTK CELL data, so material assignment is
        what survives (or does not survive) a clip. Keying it on mesh index
        would make every per-material check vacuous — it would be asking
        "did the mesh survive", which it always does.
      * the comparison stage binds these back, so a uv shift shows up as a
        differently-textured face instead of a flat colour.

    A face in no subset gets the mesh's own bound material, or -1.
    """
    from pxr import Usd, UsdGeom, UsdShade
    st = Usd.Stage.Open(path)
    Ps, UVs, Ts, MIDs = [], [], [], []
    mats, mat_ix = [], {}

    def mat_id(prim):
        if prim is None or not prim:
            return -1
        q = str(prim.GetPath())
        if q not in mat_ix:
            mat_ix[q] = len(mats)
            mats.append(q)
        return mat_ix[q]

    base = 0
    xc = UsdGeom.XformCache()
    for pr in st.Traverse():
        if not pr.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(pr)
        pts = m.GetPointsAttr().Get()
        fvc = m.GetFaceVertexCountsAttr().Get()
        fvi = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not fvc or not fvi:
            continue
        M = np.array(xc.GetLocalToWorldTransform(pr), dtype=np.float64)
        P = np.asarray(pts, dtype=np.float64)
        P = P @ M[:3, :3] + M[3, :3]
        pv = UsdGeom.PrimvarsAPI(pr).GetPrimvar("st")
        uv = (np.asarray(pv.Get(), dtype=np.float64)
              if pv and pv.Get() is not None else None)
        idx = np.asarray(fvi, dtype=np.int64)

        # face -> material, from the subsets
        nfaces = len(fvc)
        face_mat = np.full(nfaces, mat_id(
            UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0].GetPrim()
            if UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
            else None), dtype=np.int32)
        for child in pr.GetChildren():
            if child.GetTypeName() != "GeomSubset":
                continue
            sub = UsdGeom.Subset(child)
            ind = sub.GetIndicesAttr().Get()
            if not ind:
                continue
            b = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()[0]
            face_mat[np.asarray(ind, dtype=np.int64)] = mat_id(
                b.GetPrim() if b else None)

        # de-index, fanning polygons to triangles, carrying the face material
        cur, corners, uvc, tri_mat = 0, [], [], []
        for f, c in enumerate(fvc):
            c = int(c)
            for k in range(1, c - 1):
                corners.extend([idx[cur], idx[cur + k], idx[cur + k + 1]])
                uvc.extend([cur, cur + k, cur + k + 1])
                tri_mat.append(face_mat[f])
            cur += c
        if not corners:
            continue
        corners = np.asarray(corners, dtype=np.int64)
        Pc = P[corners]
        if uv is not None and len(uv) == len(idx):          # faceVarying
            UVc = uv[np.asarray(uvc, dtype=np.int64)]
        elif uv is not None and len(uv) == len(P):          # vertex
            UVc = uv[corners]
        else:
            UVc = np.zeros((len(Pc), 2))
        n = len(Pc)
        Ps.append(Pc); UVs.append(UVc)
        Ts.append(np.arange(base, base + n, dtype=np.int64).reshape(-1, 3))
        MIDs.append(np.asarray(tri_mat, dtype=np.int32))
        base += n

    return {"P": np.vstack(Ps), "UV": np.vstack(UVs),
            "tris": np.vstack(Ts), "MID": np.concatenate(MIDs),
            "MATS": mats}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asset")
    ap.add_argument("--storeys", type=int, default=8)
    ap.add_argument("--bays", type=int, default=3)
    ap.add_argument("--backend", default="Sequential")
    a = ap.parse_args()

    vtk.vtkSMPTools().SetBackend(a.backend)
    print("VTK %s | SMP %s | threads %d"
          % (vtk.vtkVersion.GetVTKVersion(),
             vtk.vtkSMPTools().GetBackend(),
             vtk.vtkSMPTools().GetEstimatedNumberOfThreads()))

    t0 = time.perf_counter()
    m = load(a.asset)
    print("asset  : %s" % os.path.basename(a.asset))
    print("loaded : %d tris, %d corner points  (%.1fs)"
          % (len(m["tris"]), len(m["P"]), time.perf_counter() - t0))

    lo = m["P"].min(axis=0); hi = m["P"].max(axis=0)
    z = np.linspace(lo[2], hi[2], a.storeys + 1)
    x = np.linspace(lo[0], hi[0], a.bays + 1)
    y = np.linspace(lo[1], hi[1], a.bays + 1)
    print("partition: %d storeys x %d x %d bays = %d cells\n"
          % (a.storeys, a.bays, a.bays, a.storeys * a.bays * a.bays))

    results = {}
    global DEDUP
    variants = [("CURRENT", run_current, True, a.backend),
                ("SWEEP", run_sweep, True, a.backend),
                ("SWEEP_NODEDUP", run_sweep, False, a.backend),
                ("SWEEP_COMPACT", run_sweep, "compact", a.backend),
                ("PLANE_SEQ_COMPACT", run_plane_sweep, "compact", "Sequential"),
                ("PLANE_STD_COMPACT", run_plane_sweep, "compact", "STDThread")]
    for name, fn, dd, backend in variants:
        if not vtk.vtkSMPTools.SetBackend(backend):
            print("%s: SKIP — SMP backend %s unavailable\n" % (name, backend))
            continue
        vtk.vtkSMPTools.Initialize(0)
        DEDUP = dd
        T.clear(); N.clear()
        t = time.perf_counter()
        pieces = fn(m, z, x, y)
        wall = time.perf_counter() - t
        tris = sum(len(p["tris"]) for p in pieces)
        results[name] = (wall, len(pieces), tris, area_of(pieces))
        results[name] = results[name] + (sum(len(p["P"]) for p in pieces),)
        print("%s [%s/%d]: %.2fs — %d pieces, %d tris"
              % (name, vtk.vtkSMPTools.GetBackend(),
                 vtk.vtkSMPTools.GetEstimatedNumberOfThreads(), wall,
                 len(pieces), tris))
        for k in sorted(T, key=lambda k: -T[k]):
            print("      %-18s %7.2fs  (%d calls)" % (k, T[k], N[k]))
        print()

    (wc, pc, tc, ac, vc) = results["CURRENT"]
    (ws, ps, ts, as_, vs) = results["SWEEP"]
    (wn, pn, tn, an, vn) = results["SWEEP_NODEDUP"]
    (wk, pk, tk, ak, vk) = results["SWEEP_COMPACT"]
    print("=" * 62)
    print("equivalence: pieces %d vs %d | tris %d vs %d | area %.1f vs %.1f (%+.2f%%)"
          % (pc, ps, tc, ts, ac, as_, 100.0 * (as_ - ac) / ac if ac else 0.0))
    ok = abs(as_ - ac) <= 0.005 * max(ac, 1e-9)
    print("area agrees within 0.5%%: %s" % ("YES" if ok else "NO — timing is not comparable"))
    print("SPEEDUP  sweep+dedup : %.2fx  (%.2fs -> %.2fs)  %d verts" % (wc/ws if ws else 0, wc, ws, vs))
    print("SPEEDUP  sweep no-dedup: %.2fx  (%.2fs -> %.2fs)  %d verts (%.2fx more)"
          % (wc/wn if wn else 0, wc, wn, vn, vn/max(vs,1)))
    print("no-dedup area %.1f (%+.2f%%)" % (an, 100.0*(an-ac)/ac if ac else 0))
    print("SPEEDUP  sweep compact : %.2fx  (%.2fs -> %.2fs)  %d verts (%.2fx vs dedup)"
          % (wc/wk if wk else 0, wc, wk, vk, vk/max(vs,1)))
    print("compact area %.1f (%+.2f%%)" % (ak, 100.0*(ak-ac)/ac if ac else 0))
    for name in ("PLANE_SEQ_COMPACT", "PLANE_STD_COMPACT"):
        if name not in results:
            continue
        wp, pp, tp, ap_, vp = results[name]
        plane_ok = (pp == ps and
                    abs(ap_ - ac) <= 0.005 * max(ac, 1e-9))
        print("SPEEDUP  %-18s: %.2fx  (%.2fs -> %.2fs)  %d verts | "
              "pieces %d, tris %d, area %+.2f%% | valid %s"
              % (name.lower(), wc/wp if wp else 0, wc, wp, vp, pp, tp,
                 100.0*(ap_-ac)/ac if ac else 0,
                 "YES" if plane_ok else "NO"))
    print("\nverts: dedup %d | compact %d | nodedup %d  (3x tris = %d)"
          % (vs, vk, vn, 3*ts))


if __name__ == "__main__":
    main()
