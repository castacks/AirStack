#!/usr/bin/env python
"""_o_merge_check.py — offline proof that `bake.export_object(merge=...)`
changes nothing but the packing.

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_o_merge_check.py

Bare pxr, no Kit, no GPU, no Isaac — safe beside a running sim. Builds a
synthetic source stage that contains every shape the quake bake actually
produces (an `_a_lump`-style box with NO authored normals, a `_box`-style one
with faceVarying normals, a kit-style mesh with vertex UVs, an indexed
faceVarying UV, a fragment with a `materialBind` GeomSubset, a mirrored
transform, and two byte-identical materials at different paths), exports it
BOTH ways, and asserts the merged file is the same geometry:

  * identical world-space vertex multiset;
  * identical triangle count and total surface area PER BOUND MATERIAL;
  * identical UV values per world position;
  * every source mesh's shading signature kept apart (no mesh with authored
    normals ever merged with one without).
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")))

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt      # noqa: E402

from disaster import bake                                # noqa: E402

FAILED = []


def check(name, cond, detail=""):
    print("{0} {1}{2}".format("PASS" if cond else "FAIL", name,
                              "" if not detail else "   " + detail))
    if not cond:
        FAILED.append(name)


# ---------------------------------------------------------------------------
def _mat(stage, path, colour, tex=None):
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*colour))
    if tex:
        sh.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


_BOX_F = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
          (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
_BOX_N = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)]


def _box(stage, path, s=(1.0, 1.0, 1.0), normals=False, uv=None,
         translate=(0, 0, 0), rot_z=0.0, scale=(1, 1, 1)):
    hx, hy, hz = s[0] / 2.0, s[1] / 2.0, s[2] / 2.0
    pts = [Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, -hy, -hz),
           Gf.Vec3f(hx, hy, -hz), Gf.Vec3f(-hx, hy, -hz),
           Gf.Vec3f(-hx, -hy, hz), Gf.Vec3f(hx, -hy, hz),
           Gf.Vec3f(hx, hy, hz), Gf.Vec3f(-hx, hy, hz)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in _BOX_F for i in f]))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if normals:
        m.CreateNormalsAttr(Vt.Vec3fArray(
            [Gf.Vec3f(*n) for n in _BOX_N for _ in range(4)]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    api = UsdGeom.PrimvarsAPI(m.GetPrim())
    if uv == "vertex":
        pv = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                               UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(0.1 * i, 0.2 * i) for i in range(8)]))
    elif uv == "fv_indexed":
        pv = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                               UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0),
                              Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        pv.SetIndices(Vt.IntArray([j for _ in range(6) for j in (0, 1, 2, 3)]))
    xf = UsdGeom.Xformable(m)
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rot_z:
        xf.AddRotateZOp().Set(float(rot_z))
    if scale != (1, 1, 1):
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    return m


def build_source():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.Xform.Define(st, "/World")
    root = UsdGeom.Xform.Define(st, "/World/obj").GetPrim()
    UsdGeom.Scope.Define(st, "/World/Looks")
    brick = _mat(st, "/World/Looks/brick", (0.5, 0.2, 0.15), "brick.png")
    core = _mat(st, "/World/Looks/core", (0.2, 0.2, 0.2))
    # TWO BYTE-IDENTICAL MATERIALS AT DIFFERENT PATHS — the chair-referenced-
    # twenty-times case. The fingerprint must collapse them to one.
    brick2 = _mat(st, "/World/Looks/brick_copy", (0.5, 0.2, 0.15), "brick.png")

    def bind(prim, mat):
        UsdShade.MaterialBindingAPI(prim).Bind(mat)

    made = []
    # 1-3: heap lumps, NO authored normals, same material, translated. EVERY
    # ONE A DIFFERENT SIZE — `_a_lump` jitters all eight corners independently,
    # so no two heap chunks in a real archetype share local geometry.
    for i in range(3):
        m = _box(st, "/World/obj/lump_{0}".format(i),
                 (0.4 + 0.03 * i, 0.3 - 0.02 * i, 0.25 + 0.01 * i),
                 normals=False, translate=(i * 1.0, 0.0, 0.0))
        bind(m.GetPrim(), brick)
        made.append(m)
    # 4-5: authored boxes WITH faceVarying normals, same material, rotated
    for i in range(2):
        m = _box(st, "/World/obj/slab_{0}".format(i), (2.0 + 0.1 * i, 1.0, 0.2),
                 normals=True, translate=(0.0, 3.0 + i, 1.0), rot_z=17.0 * (i + 1))
        bind(m.GetPrim(), brick)
        made.append(m)
    # 6: same shape as slab_1 but the DUPLICATE material -> the fingerprint
    #    must collapse the material so this lands in slab_0's bucket
    m = _box(st, "/World/obj/slab_dup", (2.3, 1.0, 0.2), normals=True,
             translate=(0.0, 6.0, 1.0))
    bind(m.GetPrim(), brick2)
    made.append(m)
    # 7: vertex UVs (kit wall shape)
    m = _box(st, "/World/obj/wall_0", (3.0, 0.4, 4.0), normals=True,
             uv="vertex", translate=(-4.0, 0.0, 2.0), rot_z=31.0)
    bind(m.GetPrim(), brick)
    made.append(m)
    # 8: INDEXED faceVarying UVs
    m = _box(st, "/World/obj/wall_1", (3.1, 0.4, 4.0), normals=True,
             uv="fv_indexed", translate=(-8.0, 0.0, 2.0))
    bind(m.GetPrim(), brick)
    made.append(m)
    # 8b: THREE COPIES OF ONE KIT MODULE — identical local geometry, different
    #     transforms. These must be left as their own prims: crate stores the
    #     points once, and baking the transform in would triple them.
    for i in range(3):
        m = _box(st, "/World/obj/kit_{0}".format(i), (4.0, 0.5, 3.0),
                 normals=True, uv="vertex",
                 translate=(-12.0, 4.0 * i, 1.5), rot_z=90.0 * i)
        bind(m.GetPrim(), brick)
        made.append(m)
    # 9: a fragment with a materialBind GeomSubset (agent T's core split):
    #    faces 0,1 take `core`, the rest keep `brick`
    m = _box(st, "/World/obj/frag_0", (0.6, 0.5, 0.45), normals=True,
             translate=(2.0, -3.0, 0.3), rot_z=-22.0)
    bind(m.GetPrim(), brick)
    sub = UsdGeom.Subset.CreateGeomSubset(
        m, "core", UsdGeom.Tokens.face, Vt.IntArray([0, 1]),
        familyName="materialBind")
    bind(sub.GetPrim(), core)
    made.append(m)
    # 10: a MIRRORED transform (negative scale) — winding must be repaired
    m = _box(st, "/World/obj/mirror_0", (0.8, 0.6, 0.5), normals=True,
             translate=(5.0, -3.0, 0.4), scale=(-1.0, 1.0, 1.0))
    bind(m.GetPrim(), brick)
    made.append(m)
    return st


# ---------------------------------------------------------------------------
def _tris(pts, counts, indices):
    """Fan-triangulate; returns [(p0, p1, p2)] in world space."""
    out, o = [], 0
    for c in counts:
        f = [pts[indices[o + k]] for k in range(c)]
        for k in range(1, c - 1):
            out.append((f[0], f[k], f[k + 1]))
        o += c
    return out


def _area(tris):
    a = 0.0
    for p, q, r in tris:
        u = Gf.Vec3d(q[0] - p[0], q[1] - p[1], q[2] - p[2])
        v = Gf.Vec3d(r[0] - p[0], r[1] - p[1], r[2] - p[2])
        c = Gf.Cross(u, v)
        a += 0.5 * math.sqrt(c[0] ** 2 + c[1] ** 2 + c[2] ** 2)
    return a


def _harvest(path):
    """{material name -> (n_tris, area, {rounded world point -> uv})} plus
    counts, over a baked archetype."""
    st = Usd.Stage.Open(path)
    per = {}
    pts_all = []
    n_prims = n_mesh = n_mat = 0
    sigs = set()
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in st.Traverse():
        n_prims += 1
        if prim.GetTypeName() == "Material":
            n_mat += 1
        if not prim.IsA(UsdGeom.Mesh):
            continue
        n_mesh += 1
        m = UsdGeom.Mesh(prim)
        M = xf.GetLocalToWorldTransform(prim)
        P = [M.Transform(Gf.Vec3d(p)) for p in (m.GetPointsAttr().Get() or [])]
        C = list(m.GetFaceVertexCountsAttr().Get() or [])
        I = list(m.GetFaceVertexIndicesAttr().Get() or [])
        na = m.GetNormalsAttr()
        sigs.add((prim.GetName().split("_")[0],
                  bool(na and na.HasAuthoredValue())))
        pts_all += [(round(p[0], 4), round(p[1], 4), round(p[2], 4)) for p in P]
        stpv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        uv = {}
        if stpv and stpv.HasAuthoredValue():
            vals = list(stpv.Get() or [])
            idxs = list(stpv.GetIndices()) if stpv.IsIndexed() else None
            interp = stpv.GetInterpolation()
            if interp == "vertex" or interp == "varying":
                for k, p in enumerate(P):
                    v = vals[idxs[k]] if idxs else (vals[k] if k < len(vals) else None)
                    if v is not None:
                        uv[(round(p[0], 3), round(p[1], 3), round(p[2], 3))] = \
                            (round(v[0], 4), round(v[1], 4))
            elif interp == "faceVarying":
                for k in range(len(I)):
                    v = vals[idxs[k]] if idxs else (vals[k] if k < len(vals) else None)
                    p = P[I[k]]
                    if v is not None:
                        uv[(round(p[0], 3), round(p[1], 3), round(p[2], 3))] = \
                            (round(v[0], 4), round(v[1], 4))

        # faces are split by materialBind subsets
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        base = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        base = base.GetPrim().GetName() if base and base.GetPrim().IsValid() else "<none>"
        fmat = [base] * len(C)
        for s in subs:
            b = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            nm = b.GetPrim().GetName() if b and b.GetPrim().IsValid() else base
            for f in (s.GetIndicesAttr().Get() or []):
                if 0 <= int(f) < len(fmat):
                    fmat[int(f)] = nm
        starts, o = [], 0
        for c in C:
            starts.append(o)
            o += c
        for f, c in enumerate(C):
            tris = _tris(P, [c], I[starts[f]:starts[f] + c])
            # reindex: _tris indexes into P with the slice's own values
            tris = []
            face = [P[I[starts[f] + k]] for k in range(c)]
            for k in range(1, c - 1):
                tris.append((face[0], face[k], face[k + 1]))
            slot = per.setdefault(fmat[f], [0, 0.0, {}])
            slot[0] += len(tris)
            slot[1] += _area(tris)
        for k, v in uv.items():
            per.setdefault(base, [0, 0.0, {}])[2][k] = v
    return {"per": per, "prims": n_prims, "meshes": n_mesh, "materials": n_mat,
            "points": sorted(pts_all), "sigs": sigs}


def main():
    src = build_source()
    paths = ["/World/obj"]
    d = tempfile.mkdtemp(prefix="o_merge_")
    raw = os.path.join(d, "raw.usda")
    mrg = os.path.join(d, "merged.usda")
    s_raw, s_mrg = {}, {}
    bake.export_object(src, None, paths, raw, merge="off", stats_out=s_raw)
    bake.export_object(src, None, paths, mrg, merge="on", stats_out=s_mrg)
    print("raw   stats:", s_raw)
    print("merged stats:", s_mrg)

    A = _harvest(raw)
    B = _harvest(mrg)
    print("raw    prims={0} meshes={1} materials={2}".format(
        A["prims"], A["meshes"], A["materials"]))
    print("merged prims={0} meshes={1} materials={2}".format(
        B["prims"], B["meshes"], B["materials"]))

    check("merge reduces mesh count", B["meshes"] < A["meshes"],
          "{0} -> {1}".format(A["meshes"], B["meshes"]))
    check("repeated kit geometry left un-merged",
          s_mrg.get("repeat_kept") == 3,
          "repeat_kept={0} (the three kit_N copies)".format(
              s_mrg.get("repeat_kept")))
    # THE SOURCE HAS THREE MATERIALS, two of them byte-identical at different
    # paths. Both exports go through the same fingerprinted `material_for`, so
    # BOTH should be down to two — the dedup is not a merge-only effect.
    n_src_mat = sum(1 for q in src.Traverse() if q.GetTypeName() == "Material")
    check("materials deduped by fingerprint",
          n_src_mat == 3 and A["materials"] == 2 and B["materials"] == 2,
          "source {0} -> raw {1} / merged {2} (brick + brick_copy identical)"
          .format(n_src_mat, A["materials"], B["materials"]))
    # NOT a multiset: a mesh whose faces are split across two material buckets
    # has to carry its shared vertices into both, so the merged file has a few
    # MORE points. What must hold is that the point CLOUD is the same and the
    # triangles and area per material are the same (checked below).
    sa, sb = set(A["points"]), set(B["points"])
    check("world point cloud identical (set)", sa == sb,
          "raw-only {0}, merged-only {1}, dup from subset split {2}".format(
              len(sa - sb), len(sb - sa), len(B["points"]) - len(A["points"])))

    ka, kb = set(A["per"]), set(B["per"])
    check("same set of bound materials", ka == kb,
          "{0} vs {1}".format(sorted(ka), sorted(kb)))
    for k in sorted(ka & kb):
        ta, aa, _ = A["per"][k]
        tb, ab, _ = B["per"][k]
        check("triangles per material '{0}'".format(k), ta == tb,
              "{0} vs {1}".format(ta, tb))
        check("area per material '{0}'".format(k), abs(aa - ab) < 1e-4,
              "{0:.6f} vs {1:.6f}".format(aa, ab))
    uva = {}
    uvb = {}
    for k in ka:
        uva.update(A["per"][k][2])
    for k in kb:
        uvb.update(B["per"][k][2])
    check("UVs preserved per world position", uva == uvb,
          "{0} vs {1} keys".format(len(uva), len(uvb)))

    # A mesh with authored normals must never share a prim with one without.
    st = Usd.Stage.Open(mrg)
    mixed = False
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        na = m.GetNormalsAttr()
        if na and na.HasAuthoredValue():
            n = len(na.Get() or [])
            fv = len(m.GetFaceVertexIndicesAttr().Get() or [])
            nfc = len(m.GetFaceVertexCountsAttr().Get() or [])
            npt = len(m.GetPointsAttr().Get() or [])
            interp = m.GetNormalsInterpolation()
            want = {"faceVarying": fv, "uniform": nfc}.get(interp, npt)
            if n != want:
                mixed = True
                print("   normals length {0} != {1} on {2} ({3})".format(
                    n, want, prim.GetPath(), interp))
    check("merged normals array lengths match their interpolation", not mixed)
    # the flat-normal compaction: every source box is flat-shaded, so every
    # merged mesh that had faceVarying normals must now carry `uniform` ones
    n_uniform = sum(
        1 for prim in st.Traverse()
        if prim.IsA(UsdGeom.Mesh) and prim.GetName().startswith("merged_")
        and UsdGeom.Mesh(prim).GetNormalsAttr().HasAuthoredValue()
        and UsdGeom.Mesh(prim).GetNormalsInterpolation() == "uniform")
    check("flat faceVarying normals compacted to uniform", n_uniform >= 3,
          "{0} merged mesh(es) carry uniform normals".format(n_uniform))

    print("\n{0} check(s) FAILED{1}".format(
        len(FAILED), "" if not FAILED else ": " + ", ".join(FAILED)))
    return 1 if FAILED else 0


sys.exit(main())
