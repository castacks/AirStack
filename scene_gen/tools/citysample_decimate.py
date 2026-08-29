#!/usr/bin/env python
"""Decimate the CitySample façade modules into LOCAL USDs we control.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/citysample_decimate.py

WHY LOCAL COPIES AND NOT JUST A REFERENCE
------------------------------------------
Two problems, one fix.

  * WEIGHT. The pack has no LOD of any kind — probed for variant sets,
    LOD-named prims and purposes across CHC/NYG/SFB: none, one mesh per module
    at one density. GreatAmericanCity ships LOD0-LOD4 and does not need it;
    CitySample is the 83.2M-triangle pack and ships nothing.
  * THE VENDOR FILES DO NOT DRAW. Placed through `apply_placements` the
    modules compose correctly in Kit — 459/459 typed `Mesh` prims, all with
    reachable points, zero empty bounds, 71.04 m of building with the roof
    excluded — and Hydra renders none of it. Re-authoring the geometry into a
    plain Mesh with a plain `UsdPreviewSurface` sidesteps whatever in the
    source composition it is refusing.

WRITTEN IN METRES with `metersPerUnit = 1`, so the centimetre correction that
every other path here needs disappears: the placement scale becomes 1.0 and a
3.25 m bay is 3.25 in the file.

Env:
    CS_FACES    target triangles per module (default 1200)
    CS_ONLY     comma list of family prefixes, e.g. "CHC,NYG"
    CS_OUT      output directory (default scene_gen/assets/citysample)
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

from detail import citysample_building as cs

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get("CS_OUT") or os.path.join(HERE, "..", "assets",
                                               "citysample")
FACES = int(os.environ.get("CS_FACES", "1200"))
ONLY = [q.strip().upper() for q in os.environ.get("CS_ONLY", "").split(",")
        if q.strip()]
CAT = os.path.join(HERE, "..", "_plans", "citysample_kit_local.json")


def _read(url):
    """(verts_m, tris, uv_per_corner, diffuse_texture_url) for a module."""
    st = Usd.Stage.Open(url)
    st.Load()
    S = UsdGeom.GetStageMetersPerUnit(st)
    mesh = None
    for p in st.Traverse():
        if p.IsA(UsdGeom.Mesh):
            mesh = p
            break
    if mesh is None:
        return None
    me = UsdGeom.Mesh(mesh)
    pts = me.GetPointsAttr().Get()
    counts = me.GetFaceVertexCountsAttr().Get()
    idx = me.GetFaceVertexIndicesAttr().Get()
    if pts is None or not counts:
        return None
    V = np.asarray(pts, dtype=np.float64) * S          # METRES from here on
    counts = np.asarray(counts, dtype=np.int64)
    idx = np.asarray(idx, dtype=np.int64)
    # fan-triangulate, carrying the CORNER index so uvs can follow
    tris, corner = [], []
    start = np.zeros(len(counts) + 1, dtype=np.int64)
    np.cumsum(counts, out=start[1:])
    for f, c in enumerate(counts):
        b = start[f]
        for j in range(1, c - 1):
            tris.append((idx[b], idx[b + j], idx[b + j + 1]))
            corner.append((b, b + j, b + j + 1))
    T = np.asarray(tris, dtype=np.int64)
    C = np.asarray(corner, dtype=np.int64)

    uv = None
    api = UsdGeom.PrimvarsAPI(mesh)
    for nm in ("st", "uv0", "st0", "UVMap"):
        pv = api.GetPrimvar(nm)
        if pv and pv.HasValue():
            a = np.asarray(pv.Get(), dtype=np.float64)
            if pv.GetInterpolation() == UsdGeom.Tokens.faceVarying:
                uv = a[C.reshape(-1)] if len(a) > C.max() else None
            else:                                   # vertex interpolation
                uv = a[T.reshape(-1)] if len(a) > T.max() else None
            if uv is not None:
                break

    tex = ""
    try:
        mat = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0]
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
        if (not mat or not mat.GetPrim().IsValid()) and subs:
            mat = UsdShade.MaterialBindingAPI(
                subs[0].GetPrim()).ComputeBoundMaterial()[0]
        if mat and mat.GetPrim().IsValid():
            for c in Usd.PrimRange(mat.GetPrim()):
                sh = UsdShade.Shader(c)
                if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
                    continue
                d = sh.GetInput("diffuseColor")
                if d is not None and d.HasConnectedSource():
                    ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
                    f = ts.GetInput("file")
                    v = f.Get() if f else None
                    if isinstance(v, Sdf.AssetPath) and v.path:
                        tex = v.resolvedPath or v.path
                break
    except Exception:
        tex = ""
    return V, T, uv, tex


def _decimate(V, T, uv, target):
    """Quadric edge collapse to `target` triangles. UVs preserved when given."""
    import pymeshlab
    if len(T) <= target:
        return V, T, uv
    ms = pymeshlab.MeshSet()
    if uv is not None and len(uv) == len(T) * 3:
        m = pymeshlab.Mesh(vertex_matrix=V, face_matrix=T,
                           wedge_tex_coord_matrix=uv.astype(np.float64))
    else:
        m = pymeshlab.Mesh(vertex_matrix=V, face_matrix=T)
        uv = None
    ms.add_mesh(m, "m")
    try:
        if uv is not None:
            ms.meshing_decimation_quadric_edge_collapse_with_texture(
                targetfacenum=int(target), preserveboundary=True,
                preservenormal=True)
        else:
            ms.meshing_decimation_quadric_edge_collapse(
                targetfacenum=int(target), preserveboundary=True,
                preservenormal=True, preservetopology=False)
    except Exception:
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=int(target), preserveboundary=True)
        uv = None
    out = ms.current_mesh()
    V2 = np.asarray(out.vertex_matrix(), dtype=np.float64)
    T2 = np.asarray(out.face_matrix(), dtype=np.int64)
    uv2 = None
    if uv is not None:
        try:
            w = np.asarray(out.wedge_tex_coord_matrix(), dtype=np.float64)
            if len(w) == len(T2) * 3:
                uv2 = w
        except Exception:
            uv2 = None
    return V2, T2, uv2


def _write(path, V, T, uv, tex, name):
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)          # METRES — no cm trap
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    me = UsdGeom.Mesh.Define(st, Sdf.Path("/" + name))
    st.SetDefaultPrim(me.GetPrim())
    me.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in V]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(T)))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in T.reshape(-1)]))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo = V.min(axis=0) if len(V) else np.zeros(3)
    hi = V.max(axis=0) if len(V) else np.zeros(3)
    me.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    if uv is not None and len(uv) == len(T) * 3:
        pv = UsdGeom.PrimvarsAPI(me).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(float(a), float(b)) for a, b in uv]))
    mat = UsdShade.Material.Define(st, Sdf.Path("/" + name + "/M"))
    sh = UsdShade.Shader.Define(st, Sdf.Path("/" + name + "/M/S"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.82)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    if tex and uv is not None:
        rd = UsdShade.Shader.Define(st, Sdf.Path("/" + name + "/M/st"))
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        ts = UsdShade.Shader.Define(st, Sdf.Path("/" + name + "/M/tex"))
        ts.CreateIdAttr("UsdUVTexture")
        ts.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(tex))
        ts.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        ts.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        ts.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            rd.ConnectableAPI(), "result")
        ts.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor",
                       Sdf.ValueTypeNames.Color3f).ConnectToSource(
                           ts.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.34, 0.33, 0.31))
    sh.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
    st.GetRootLayer().Save()
    return lo, hi


def main():
    out_dir = os.path.normpath(OUT)
    os.makedirs(out_dir, exist_ok=True)
    kit = cs.load_kit()
    t0 = time.time()
    local, n, n_tex, in_tri, out_tri = {}, 0, 0, 0, 0
    for fam in sorted(kit):
        if ONLY and fam.upper() not in ONLY:
            continue
        for var in sorted(kit[fam]):
            for lvl in sorted(kit[fam][var], key=int):
                keep = []
                for m in kit[fam][var][lvl]:
                    stem = m["usd"].rsplit("/", 1)[-1][:-4]
                    dst = os.path.join(out_dir, stem + ".usdc")
                    try:
                        if not os.path.exists(dst):
                            got = _read(cs.ASSET_ROOT + m["usd"])
                            if got is None:
                                continue
                            V, T, uv, tex = got
                            in_tri += len(T)
                            V2, T2, uv2 = _decimate(V, T, uv, FACES)
                            out_tri += len(T2)
                            if tex:
                                n_tex += 1
                            _write(dst, V2, T2, uv2, tex,
                                   stem.replace("-", "_"))
                        q = dict(m)
                        q["usd"] = os.path.relpath(dst, os.path.join(HERE, ".."))
                        q["mpu"] = 1.0
                        q["local"] = True
                        keep.append(q)
                        n += 1
                        if n % 25 == 0:
                            print("  %4d modules  %.0f s" % (n, time.time()-t0),
                                  flush=True)
                    except Exception as exc:
                        print("  SKIP %s: %s" % (stem[:44], exc), flush=True)
                if keep:
                    local.setdefault(fam, {}).setdefault(var, {})[lvl] = keep
    json.dump(local, open(os.path.normpath(CAT), "w"), indent=1)
    print("\n%d modules -> %s" % (n, out_dir))
    print("triangles %.2fM -> %.2fM (%.0f%%)   %d carried a texture   %.0f s"
          % (in_tri/1e6, out_tri/1e6, 100.0*out_tri/max(in_tri, 1), n_tex,
             time.time()-t0))
    print("catalogue -> %s" % os.path.normpath(CAT))


if __name__ == "__main__":
    main()
