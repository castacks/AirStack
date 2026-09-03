#!/usr/bin/env python
"""slice_compare_stage.py — one USD holding every slicer method side by side,
so the "no visual difference" claim can be looked at instead of trusted.

The gate in `slice_visual_gate.py` is arithmetic: area, per-material uv
bounding boxes, and survival of position-duplicate/uv-distinct corners. That
is the right gate — area alone would not have caught the defect this pipeline
actually shipped, where merged wall/decal corners inherited the wall's uv and
a wall came out looking graffitied. But arithmetic is not a look, and the
whole reason the decal bug survived review the first time is that nobody put
the two versions next to each other.

So this writes each method's pieces into its own `/World/<method>` scope,
offset along +X by the building's own width plus a gap, on a common ground
plane. Open it in Isaac and fly down the row: identical geometry means the
methods agree; a shifted uv shows up as a differently-textured face on
otherwise identical geometry.

WHAT IS AND IS NOT CARRIED. Points, triangles, the `st` primvar (faceVarying,
expanded through the indices exactly as `gac_storey_slice` writes it) and a
per-piece `displayColor` keyed to the method. Materials are NOT bound: the
source materials live on Nucleus and binding them would make this stage
un-openable without a server, which defeats the point of a quick look. `st`
is preserved verbatim, so a uv difference is still visible with any textured
material assigned in-viewport.

    python3 scene_gen/tools/slice_compare_stage.py <asset.usd> \\
        --out ~/slice_compare.usd --storeys 8 --bays 3
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np                                   # noqa: E402

#: (scope name, dedup mode, displayColor). Colour is per METHOD so a piece
#: that came out of the wrong branch is obvious at a glance.
METHODS = [
    ("A_current",        "__current__", "Sequential", (0.80, 0.80, 0.82)),
    ("B_sweep_dedup",    True,          "Sequential", (0.35, 0.65, 0.95)),
    ("C_sweep_compact",  "compact",     "Sequential", (0.45, 0.85, 0.45)),
    ("D_sweep_nodedup",  False,         "Sequential", (0.95, 0.65, 0.30)),
    ("E_plane_seq",      "__plane__",   "Sequential", (0.90, 0.45, 0.85)),
    ("F_plane_stdthread", "__plane__",  "STDThread",  (0.95, 0.90, 0.35)),
]


_ALL_METHODS = list(METHODS)


def build_all(asset, storeys, bays):
    import slice_bench as SB
    m = SB.load(asset)
    lo, hi = m["P"].min(axis=0), m["P"].max(axis=0)
    z = np.linspace(lo[2], hi[2], storeys + 1)
    x = np.linspace(lo[0], hi[0], bays + 1)
    y = np.linspace(lo[1], hi[1], bays + 1)

    import vtk
    out = {}
    for name, mode, backend, _c in METHODS:
        vtk.vtkSMPTools().SetBackend(backend)
        vtk.vtkSMPTools().Initialize(0)
        t = time.perf_counter()
        if mode == "__current__":
            SB.DEDUP = True
            pieces = SB.run_current(m, z, x, y)
        elif mode == "__plane__":
            SB.DEDUP = "compact"
            pieces = SB.run_plane_sweep(m, z, x, y)
        else:
            SB.DEDUP = mode
            pieces = SB.run_sweep(m, z, x, y)
        out[name] = (pieces, time.perf_counter() - t)
        print("  %-15s %6.2fs  %d pieces  %d verts"
              % (name, out[name][1], len(pieces),
                 sum(len(p["P"]) for p in pieces)))
    return m, out


def write_stage(m, results, path, asset=None, gap_frac=0.25,
                verbose=True):
    """Author the stage with BULK Vt conversion.

    THE COST IS AUTHORING, NOT `Save()`. Building `Vt.Vec3fArray` from a
    Python list comprehension crosses the Python/C++ boundary once per VALUE:
    for a six-way brownstone comparison that is ~5.1 M face counts, ~15.3 M
    indices, ~15.3 M faceVarying uvs and ~15 M point values — tens of millions
    of tuple/float/int conversions before a byte is serialised.

    MEASURED on this machine, 1 M Vec3f: 0.95 s element-wise vs 0.0072 s
    through `Vt.Vec3fArray.FromNumpy` — **132x**. 3 M ints: 0.34 s vs
    0.0049 s — **68x**. The arrays are already NumPy, so the conversion was
    pure waste.

    `.usdc` is forced: `.usd` lets the format be chosen for you, and a text
    `.usda` of this size is both enormous and slow to format.
    """
    from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, Vt

    t_all = time.perf_counter()
    tt = {"points": 0.0, "counts": 0.0, "indices": 0.0, "uvs": 0.0,
          "define": 0.0, "save": 0.0}

    lo, hi = m["P"].min(axis=0), m["P"].max(axis=0)
    span_x = float(hi[0] - lo[0])
    step = span_x * (1.0 + gap_frac)

    if not path.endswith(".usdc"):
        path = os.path.splitext(path)[0] + ".usdc"
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # THE MATERIALS. Reference the source asset under /World/_MatSrc so its
    # `/Kit/Looks` prims exist in this layer, then bind each piece's faces to
    # them through GeomSubsets. Without this the comparison is flat-shaded
    # colour, which cannot show a uv difference at all -- and a uv difference
    # is the whole thing being compared. The source's textures are RELATIVE
    # paths, so this resolves offline; only the prims are referenced, the
    # geometry under it is deactivated so it does not draw.
    mats = m.get("MATS") or []
    matpath = {}
    if asset and mats:
        src = UsdGeom.Xform.Define(stage, "/World/_MatSrc")
        src.GetPrim().GetReferences().AddReference(os.path.abspath(asset))
        src.GetPrim().SetActive(False)          # materials still compose
        for i, q in enumerate(mats):
            matpath[i] = "/World/_MatSrc" + q

    total = step * max(1, len(results))
    g = UsdGeom.Mesh.Define(stage, "/World/ground")
    pad = span_x
    x0, x1 = float(lo[0]) - pad, float(lo[0]) + total + pad
    y0, y1 = float(lo[1]) - pad, float(hi[1]) + pad
    zg = float(lo[2])
    g.CreatePointsAttr(Vt.Vec3fArray([(x0, y0, zg), (x1, y0, zg),
                                      (x1, y1, zg), (x0, y1, zg)]))
    g.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    g.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    g.CreateDisplayColorAttr(Vt.Vec3fArray([(0.18, 0.18, 0.20)]))

    for i, (name, _mode, _backend, colour) in enumerate(METHODS):
        if name not in results:
            continue
        pieces, secs = results[name]
        scope = UsdGeom.Xform.Define(stage, "/World/%s" % name)
        scope.AddTranslateOp().Set(Gf.Vec3d(step * i, 0.0, 0.0))
        pr = scope.GetPrim()
        pr.SetCustomDataByKey("slicer:method", name)
        pr.SetCustomDataByKey("slicer:seconds", float(secs))
        pr.SetCustomDataByKey("slicer:pieces", len(pieces))
        pr.SetCustomDataByKey("slicer:verts",
                              int(sum(len(q["P"]) for q in pieces)))
        for k, d in enumerate(pieces):
            t = time.perf_counter()
            me = UsdGeom.Mesh.Define(stage, "/World/%s/piece_%04d" % (name, k))
            tt["define"] += time.perf_counter() - t

            t = time.perf_counter()
            me.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(
                np.ascontiguousarray(d["P"], dtype=np.float32)))
            tt["points"] += time.perf_counter() - t

            t = time.perf_counter()
            me.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
                np.full(len(d["tris"]), 3, dtype=np.int32)))
            tt["counts"] += time.perf_counter() - t

            t = time.perf_counter()
            flat = np.ascontiguousarray(d["tris"].ravel(), dtype=np.int32)
            me.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(flat))
            tt["indices"] += time.perf_counter() - t

            t = time.perf_counter()
            # faceVarying, expanded through the indices — the same shape
            # `gac_storey_slice._write_piece` writes, so a uv difference here
            # is a uv difference there. The expansion is a NumPy gather now,
            # not a Python comprehension.
            pv = UsdGeom.PrimvarsAPI(me).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray,
                UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec2fArray.FromNumpy(
                np.ascontiguousarray(d["UV"][flat], dtype=np.float32)))
            tt["uvs"] += time.perf_counter() - t

            me.CreateDisplayColorAttr(Vt.Vec3fArray([colour]))

            # PER-FACE MATERIAL, the same way `gac_storey_slice` writes it:
            # one GeomSubset per material id present on this piece. `mid`
            # rides through VTK as CELL data, so this is exactly the
            # assignment the clip preserved (or lost).
            if matpath:
                MID = d["MID"]
                for mi in np.unique(MID):
                    if int(mi) not in matpath:
                        continue
                    faces = np.nonzero(MID == mi)[0].astype(np.int32)
                    if faces.size == 0:
                        continue
                    gs = UsdGeom.Subset.CreateGeomSubset(
                        me, "mat_%d" % int(mi), UsdGeom.Tokens.face,
                        Vt.IntArray.FromNumpy(faces))
                    mp = stage.GetPrimAtPath(matpath[int(mi)])
                    if mp and mp.IsValid():
                        UsdShade.MaterialBindingAPI.Apply(gs.GetPrim()).Bind(
                            UsdShade.Material(mp))

    t = time.perf_counter()
    stage.GetRootLayer().Save()
    tt["save"] = time.perf_counter() - t

    if verbose:
        print("\nauthoring breakdown (%.2fs total):" % (time.perf_counter() - t_all))
        for k in sorted(tt, key=lambda k: -tt[k]):
            print("   %-9s %6.2fs" % (k, tt[k]))
        print("   %-9s %6.1f MB" % ("file", os.path.getsize(path) / 1e6))
    return path, step


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("asset")
    ap.add_argument("--out", default=os.path.expanduser("~/slice_compare.usd"))
    ap.add_argument("--storeys", type=int, default=8)
    ap.add_argument("--bays", type=int, default=3)
    ap.add_argument("--methods", default="A_current,C_sweep_compact,F_plane_stdthread",
                    help="comma list, or 'all'. Default is the three that carry "
                         "information: the shipped one, the best pure-VTK sweep, "
                         "and the fastest. dedup/nodedup/plane_seq agree with "
                         "compact numerically, so drawing them costs write and "
                         "load time for no visual signal — and nodedup alone is "
                         "5.62M verts.")
    a = ap.parse_args()

    global METHODS
    if a.methods.strip().lower() != "all":
        want = [q.strip() for q in a.methods.split(",") if q.strip()]
        METHODS = [q for q in METHODS if q[0] in want]
        missing = [w for w in want if w not in [q[0] for q in METHODS]]
        if missing:
            sys.exit("unknown method(s): %s\nknown: %s"
                     % (missing, [q[0] for q in _ALL_METHODS]))

    print("slicing %s with %d methods..." % (os.path.basename(a.asset),
                                             len(METHODS)))
    m, results = build_all(a.asset, a.storeys, a.bays)
    path, step = write_stage(m, results, a.out, asset=a.asset)
    print("\nwrote %s" % path)
    print("methods laid out along +X, %.1f m apart, in this order:" % step)
    for i, (name, _mode, _b, c) in enumerate(METHODS):
        if name in results:
            print("   x=%8.1f  %-15s  rgb%s" % (step * i, name, c))
    print("\nFly down +X. Identical silhouettes = the methods agree; a face "
          "that textures differently is a uv difference.")


if __name__ == "__main__":
    main()
