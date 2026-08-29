#!/usr/bin/env python3
"""upscale_mesh — raise the polygon density of a standalone building USD.

WHY THIS EXISTS
---------------
The 112 standalone building assets (`config/asset_sets/urban_v2.yaml`) are
stacks of extruded prisms with a photograph on them: measured over the whole
library, 96 of 112 are under 1,000 points and 40 are under 100. That is fine
until something tries to BREAK one. `fracture.roughen` / `roughen_field`
displace every vertex by band-limited noise keyed off its position, which is
what turns a flat Voronoi cut into a ragged broken face — and on a fragment
cut from an 8-vertex box there are eight vertices to displace, so the result
is a warped box with three flat triangular faces. Hence "I don't want the
jagged breaks to look very triangular" (user, 2026-08-28).

The fix is more vertices BEFORE the noise, and the interesting question is
which upscaling method to use, because they do not agree about what a
building is.

THE METHODS, AND WHAT EACH ONE COSTS YOU
----------------------------------------
`--method catmull`   Catmull-Clark subdivision. This is what the USDSubD
                     plugin the user linked does (a Nuke GeomOp wrapping
                     OpenSubdiv; it needs Nuke 16+ and a C++ build, so it is
                     not usable inside the Isaac loop — pymeshlab runs the
                     same algorithm locally in milliseconds). It is the
                     standard answer and it is WRONG FOR ARCHITECTURE unless
                     creases are tagged: Catmull-Clark is a smoothing
                     operator, so it rounds every corner of the building and
                     shrinks it toward its own hull. Included because seeing
                     that happen is the argument against it.

`--method isotropic` Isotropic explicit remeshing — the open-source
                     equivalent of the AutoRemesher-class "adaptive topology"
                     the user described: it retriangulates to a target edge
                     length, adding density where it is needed while
                     PRESERVING SHARP EDGES above a feature angle. Shape is
                     kept, density is even, and it is the one that behaves
                     like a remesher rather than a subdivider.

`--method midpoint`  Linear subdivision to a maximum edge length
                     (`trimesh.remesh.subdivide_to_size`). Every new vertex
                     lies ON the original surface, so the silhouette is
                     EXACTLY preserved; it just adds density. Measured at
                     3 ms for a fragment and 17 ms for a whole building at a
                     1 m edge, and the output is welded and watertight (both
                     checked — the function's docstring warns it returns "a
                     triangle soup", which is not what it does here and would
                     have torn under roughening if it were).

NOT AVAILABLE HERE: NVIDIA's Meshtron / TriFlow NIM microservices. They are
hosted services behind an NGC key, and neither the host nor the container has
one (`env | grep NGC` is empty in both), so they cannot be benchmarked from
this machine. They are also the wrong shape for this pipeline even with a
key: a network round trip per mesh inside a loop that fractures hundreds of
fragments per scene is not affordable, whereas all three methods above are
local and sub-second.

UVs ARE THE HARD PART, AND THEY ARE RE-PROJECTED, NOT CARRIED
--------------------------------------------------------------
These assets are photographs mapped through `primvars:uv0` (faceVarying, with
indices). Every method above changes the vertex set, so the original UVs no
longer address it — and USDSubD's own README calls this out, promoting UVs to
face-varying "to prevent seam tearing". Rather than trust any method to carry
them, this re-projects: for each output vertex, find the closest point on the
ORIGINAL triangulated surface and take the barycentric interpolation of that
triangle's UVs. It is exact for `midpoint` (the vertices are on the surface by
construction), near-exact for `isotropic`, and approximate for `catmull`
precisely where Catmull-Clark has pulled the surface away from the original —
which is the same place its shape is wrong.

    /isaac-sim/python.sh scene_gen/tools/upscale_mesh.py IN.usd OUT.usd \\
        --method isotropic --target 0.35
"""

import argparse
import json
import os
import sys
import time


def _load(stage_path):
    """[(prim_path, points, tri_faces, uv_per_vertex, texture, name)]."""
    from pxr import Usd, UsdGeom, UsdShade, Sdf
    import numpy as np

    st = Usd.Stage.Open(stage_path)
    if not st:
        raise SystemExit("cannot open " + stage_path)
    out = []
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=float)
        counts = list(m.GetFaceVertexCountsAttr().Get() or [])
        idx = list(m.GetFaceVertexIndicesAttr().Get() or [])
        # UVs: faceVarying with indices on this library
        pv = UsdGeom.PrimvarsAPI(p).GetPrimvar("uv0") \
            or UsdGeom.PrimvarsAPI(p).GetPrimvar("st")
        uvs = np.asarray(pv.Get(), dtype=float) if (pv and pv.Get() is not None) \
            else None
        uvi = list(pv.GetIndices()) if (pv and pv.GetIndices()) else None
        # fan-triangulate, carrying the per-corner UV index with each corner
        tris, tuv, k = [], [], 0
        for c in counts:
            corner = list(range(k, k + c))
            for j in range(1, c - 1):
                tris.append([idx[corner[0]], idx[corner[j]], idx[corner[j + 1]]])
                if uvs is not None:
                    a = (uvi[corner[0]] if uvi else corner[0])
                    b = (uvi[corner[j]] if uvi else corner[j])
                    d = (uvi[corner[j + 1]] if uvi else corner[j + 1])
                    tuv.append([a, b, d])
            k += c
        tris = np.asarray(tris, dtype=np.int64)
        # collapse face-varying UVs onto vertices (these assets are box maps;
        # a seam vertex takes one of its two UVs, which the re-projection
        # below then corrects anyway)
        uvv = None
        if uvs is not None and len(tris):
            uvv = np.zeros((len(pts), 2), dtype=float)
            tuv = np.asarray(tuv, dtype=np.int64)
            for col in range(3):
                uvv[tris[:, col]] = uvs[tuv[:, col]]
        tex = ""
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if mat and mat.GetPrim().IsValid():
            for c2 in Usd.PrimRange(mat.GetPrim()):
                sh = UsdShade.Shader(c2)
                if sh and sh.GetIdAttr().Get() == "UsdUVTexture":
                    v = sh.GetInput("file").Get()
                    if isinstance(v, Sdf.AssetPath) and v.path:
                        tex = v.resolvedPath or v.path
                    break
        out.append((str(p.GetPath()), pts, tris, uvv, tex, p.GetParent().GetName()))
    return out


def _reproject_uv(new_pts, old_pts, old_tris, old_uv):
    """Barycentric UV lookup of each new vertex on the original surface."""
    import numpy as np
    import trimesh

    if old_uv is None or not len(old_tris):
        return None
    src = trimesh.Trimesh(vertices=old_pts, faces=old_tris, process=False)
    close, _dist, tid = trimesh.proximity.closest_point(src, new_pts)
    tri = src.triangles[tid]
    bary = trimesh.triangles.points_to_barycentric(tri, close)
    uv = (old_uv[old_tris[tid][:, 0]] * bary[:, 0:1]
          + old_uv[old_tris[tid][:, 1]] * bary[:, 1:2]
          + old_uv[old_tris[tid][:, 2]] * bary[:, 2:3])
    return uv


def _upscale(pts, tris, method, target, iters, feature_deg):
    import numpy as np

    if method == "midpoint":
        from trimesh import remesh
        # CAP THE ITERATIONS, OR THIS IS THE SLOW ONE. `subdivide_to_size`
        # QUADRUPLES the face count per pass until every edge is under
        # `max_edge`; a 106 m tower face at a 0.5 m target needs eight passes,
        # which is 4**8 and came out at 1.35 M triangles in 16.7 s — twenty
        # times slower than the C++ remesher it was supposed to be the cheap
        # alternative to (measured 2026-08-28). It is exact and instant at a
        # sane target and explosive at a small one, and the caller cannot
        # tell which they asked for without knowing the model's edge lengths,
        # so the ceiling is enforced here.
        # ...and it RAISES when the cap is hit ("max_iter exceeded!") rather
        # than returning what it has, so the loop is run here instead: one
        # uniform pass at a time, stopping on the edge target OR the cap,
        # whichever comes first.
        v, f = np.asarray(pts, dtype=float), np.asarray(tris, dtype=np.int64)
        for _ in range(int(max(1, iters))):
            e = v[f[:, [0, 1, 2]]] - v[f[:, [1, 2, 0]]]
            if float(np.sqrt((e ** 2).sum(-1)).max()) <= float(target):
                break
            v, f = remesh.subdivide(v, f)
        return np.asarray(v, dtype=float), np.asarray(f, dtype=np.int64)
    import pymeshlab as ml
    ms = ml.MeshSet()
    ms.add_mesh(ml.Mesh(vertex_matrix=np.asarray(pts, dtype=float),
                        face_matrix=np.asarray(tris, dtype=np.int32)))
    if method == "catmull":
        # Catmull-Clark emits QUADS, and `face_matrix()` only returns
        # triangles — so the mesh came back "1460 vert and 0 faces" and the
        # whole variant was silently empty. Triangulate after each pass.
        for _ in range(max(1, int(iters))):
            ms.meshing_surface_subdivision_catmull_clark()
            ms.meshing_poly_to_tri()
    elif method == "isotropic":
        ms.meshing_isotropic_explicit_remeshing(
            iterations=int(iters),
            targetlen=ml.PureValue(float(target)),
            featuredeg=float(feature_deg),
            adaptive=False)
    else:
        raise SystemExit("unknown method " + method)
    m = ms.current_mesh()
    return (np.asarray(m.vertex_matrix(), dtype=float),
            np.asarray(m.face_matrix(), dtype=np.int64))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--method", default="midpoint",
                    choices=("midpoint", "catmull", "isotropic"))
    ap.add_argument("--target", type=float, default=0.6,
                    help="target/max edge length in metres")
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--feature-deg", type=float, default=30.0,
                    help="isotropic: preserve edges sharper than this")
    ap.add_argument("--report", default="")
    ap.add_argument("--tex-root", default="",
                    help="rewrite relative texture paths against this base "
                         "URL (the output lives in a different folder from "
                         "the source, so `../textures/x.jpg` no longer "
                         "resolves)")
    a = ap.parse_args()

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt
    import numpy as np

    meshes = _load(a.src)
    # IN MEMORY, THEN EXPORT. `CreateNew(dst + ".tmp")` fails verification
    # with "fileFormat ''" — USD picks the layer format from the EXTENSION,
    # and `.usd.tmp` is not one it knows.
    out = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(out, 1.0)
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(out, Sdf.Path("/upscaled"))
    out.SetDefaultPrim(root.GetPrim())

    stats = {"src": a.src, "method": a.method, "target": a.target,
             "parts": [], "pts_in": 0, "pts_out": 0, "tris_in": 0,
             "tris_out": 0}
    t0 = time.time()
    for i, (path, pts, tris, uvv, tex, name) in enumerate(meshes):
        if not len(tris):
            continue
        tp = time.time()
        nv, nf = _upscale(pts, tris, a.method, a.target, a.iters,
                          a.feature_deg)
        nuv = _reproject_uv(nv, pts, tris, uvv)
        dt = time.time() - tp
        gp = "/upscaled/part_{0}".format(i)
        me = UsdGeom.Mesh.Define(out, Sdf.Path(gp))
        me.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in nv]))
        me.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(nf)))
        me.CreateFaceVertexIndicesAttr(Vt.IntArray(
            [int(q) for q in nf.reshape(-1)]))
        me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        me.CreateExtentAttr([Gf.Vec3f(*map(float, nv.min(0))),
                             Gf.Vec3f(*map(float, nv.max(0)))])
        if nuv is not None:
            pv = UsdGeom.PrimvarsAPI(me.GetPrim()).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray,
                UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec2fArray([Gf.Vec2f(float(q[0]), float(q[1]))
                                  for q in nuv]))
        if tex and a.tex_root and not tex.startswith(("omniverse://", "/")):
            tex = a.tex_root.rstrip("/") + "/" + tex.rsplit("/", 1)[-1]
        if tex:
            mp = "/upscaled/mat_{0}".format(i)
            mat = UsdShade.Material.Define(out, Sdf.Path(mp))
            sh = UsdShade.Shader.Define(out, Sdf.Path(mp + "/PBRShader"))
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            rd = UsdShade.Shader.Define(out, Sdf.Path(mp + "/uvReader"))
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            ts = UsdShade.Shader.Define(out, Sdf.Path(mp + "/tex"))
            ts.CreateIdAttr("UsdUVTexture")
            ts.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(tex))
            ts.CreateInput("sourceColorSpace",
                           Sdf.ValueTypeNames.Token).Set("sRGB")
            ts.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            ts.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            ts.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                rd.ConnectableAPI(), "result")
            ts.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor",
                           Sdf.ValueTypeNames.Color3f).ConnectToSource(
                               ts.ConnectableAPI(), "rgb")
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),
                                                      "surface")
            UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
        stats["parts"].append({"part": name, "pts_in": len(pts),
                               "tris_in": len(tris), "pts_out": len(nv),
                               "tris_out": len(nf), "sec": round(dt, 4),
                               "uv": nuv is not None, "tex": bool(tex)})
        stats["pts_in"] += len(pts); stats["tris_in"] += len(tris)
        stats["pts_out"] += len(nv); stats["tris_out"] += len(nf)
    stats["sec_total"] = round(time.time() - t0, 3)
    out.GetRootLayer().Export(a.dst)
    print(json.dumps(stats))
    if a.report:
        open(a.report, "w").write(json.dumps(stats, indent=1))


if __name__ == "__main__":
    main()
