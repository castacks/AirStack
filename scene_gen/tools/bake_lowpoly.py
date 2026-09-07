#!/usr/bin/env python3
"""Decimate an asset hard, re-unwrap it, and BAKE its old look onto the new UVs.

WHY THIS EXISTS, AND WHY IT IS NOT IN `mesh_damage`
---------------------------------------------------
The fracture cutter costs cells x faces, and ten assets in `urban_v2` carry 76%
of the pack's geometry — `Amar_Tower` alone is 501,869 faces on a 46,270 m2
envelope. Damaging it took 23 minutes without reaching settle. Decimation is
the only lever on the second term.

But every decimator that must PRESERVE the original parameterisation floors
out, because that is the thing standing in the way:

    vertex clustering, UV-aware       1/16   (shape capped; some smeared texels)
    quadric collapse, seams locked    1/1.5
    quadric collapse, seams free      1/11   (cracks between material subsets)

All three fail for one reason: `Amar_Tower`'s facade is a TILED atlas running
0..51.7 in u across 50 images and 33 material subsets, so almost every edge is
a UV seam and a seam is exactly what a collapse may not cross.

So this does not preserve the parameterisation. It THROWS IT AWAY, decimates
with no UV constraint at all, unwraps the result from scratch, and rebakes the
old surface into the new atlas. Reduction is then bounded by geometry alone,
which is what makes 1/50 reachable. It also collapses 50 textures and 33
material bindings into ONE of each -- `mesh_damage.core_material` measured that
two thirds of a cold scene load is the renderer compiling MDL materials, so
that is a second win rather than a cost.

This is asset preprocessing: it runs once per asset, offline, and writes a new
USD next to a baked PNG. Nothing in the damage pipeline imports it.

    scene_gen/tools/bake_lowpoly.py Amar_Tower --fraction 0.02 --res 2048
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade                  # noqa: E402

from disaster import mesh_damage as M                            # noqa: E402


# ---------------------------------------------------------------------------
# reading the high-poly
# ---------------------------------------------------------------------------

def _diffuse_texture(stage, mat_path):
    """The diffuse image an MDL or UsdPreviewSurface material samples, or None.

    Walks one level through a connection because a UsdPreviewSurface reaches
    its map through a `UsdUVTexture` reader, while MDL usually carries the
    asset on the shader itself.
    """
    mat = UsdShade.Material.Get(stage, mat_path)
    if not mat:
        return None
    src = None
    for out in (mat.GetSurfaceOutput(), mat.GetSurfaceOutput("mdl")):
        if out and out.GetConnectedSource():
            src = out.GetConnectedSource()
            break
    if not src:
        return None
    shader = UsdShade.Shader(src[0].GetPrim())
    wanted = ("diffuse", "basecolor", "base_color", "albedo")
    for inp in shader.GetInputs():
        name = inp.GetBaseName().lower()
        if not any(w in name for w in wanted):
            continue
        val = inp.Get()
        if val is not None and hasattr(val, "resolvedPath"):
            return val.resolvedPath or val.path
        conn = inp.GetConnectedSource()
        if conn:
            for sub in UsdShade.Shader(conn[0].GetPrim()).GetInputs():
                v2 = sub.Get()
                if v2 is not None and hasattr(v2, "resolvedPath"):
                    return v2.resolvedPath or v2.path
    return None


def read_highpoly(usd_path):
    """Triangles, per-corner UVs, and one source image per triangle.

    Returns ``(points, tris, tri_uv, tri_tex, images)`` where *tri_tex* indexes
    *images* and -1 means the triangle had no resolvable texture.
    """
    stage = Usd.Stage.Open(usd_path)
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and p.IsActive()]
    P, T, UV, TX = [], [], [], []
    images, by_path = [], {}
    for prim in meshes:
        counts, idx = M._face_arrays(prim)
        if counts is None or not len(counts):
            continue
        pts = M.get_points(prim)
        slots, face_src = M._triangulate(counts)
        if not len(slots):
            continue
        tri = idx[slots]
        uv_name = M._uv_primvar_name(prim)
        uv = None
        if uv_name:
            pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar(uv_name)
            if pv:
                M._flatten_primvar(pv)
                raw = M._sized(pv.GetAttr().Get())
                if raw is not None and len(raw) == int(counts.sum()):
                    uv = np.asarray([tuple(v) for v in raw],
                                    dtype=np.float64)[slots]
        if uv is None:
            uv = np.zeros((len(tri), 3, 2))

        # which image each ORIGINAL face samples, via its subset binding
        tex_of_face = np.full(len(counts), -1, dtype=np.int64)
        subsets = [c for c in prim.GetChildren() if c.IsA(UsdGeom.Subset)]
        binds = [(UsdGeom.Subset(c), c) for c in subsets]
        if not binds:
            b = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
            path = b.GetMaterialPath() if b else None
            img = _diffuse_texture(stage, path) if path else None
            if img and os.path.exists(img):
                if img not in by_path:
                    by_path[img] = len(images)
                    images.append(img)
                tex_of_face[:] = by_path[img]
        for sub, child in binds:
            if sub.GetElementTypeAttr().Get() != UsdGeom.Tokens.face:
                continue
            b = UsdShade.MaterialBindingAPI(child).GetDirectBinding()
            path = b.GetMaterialPath() if b else None
            img = _diffuse_texture(stage, path) if path else None
            if not img or not os.path.exists(img):
                continue
            if img not in by_path:
                by_path[img] = len(images)
                images.append(img)
            gi = np.asarray(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
            gi = gi[(gi >= 0) & (gi < len(counts))]
            tex_of_face[gi] = by_path[img]

        P.append(pts)
        T.append(tri + sum(len(x) for x in P[:-1]))
        UV.append(uv)
        TX.append(tex_of_face[face_src])
    if not T:
        raise SystemExit("no triangulated mesh found")
    return (np.concatenate(P), np.concatenate(T), np.concatenate(UV),
            np.concatenate(TX), images)


# ---------------------------------------------------------------------------
# decimate, with nothing to protect
# ---------------------------------------------------------------------------

def decimate_free(points, tris, target):
    """Quadric collapse with NO UV term and no seam locking. (verts, faces).

    This is the whole point of the approach: the parameterisation is about to
    be replaced, so the collapser is free to do its job on geometry alone. The
    same filter that floored at 1/1.5 with seams locked reaches the target
    here.
    """
    import pymeshlab as ml

    keep, remap = np.unique(tris.reshape(-1), return_inverse=True)
    ms = ml.MeshSet()
    ms.add_mesh(ml.Mesh(
        vertex_matrix=np.ascontiguousarray(points[keep], dtype=np.float64),
        face_matrix=np.ascontiguousarray(remap.reshape(-1, 3), dtype=np.int32)))
    ms.meshing_remove_duplicate_vertices()
    ms.meshing_remove_unreferenced_vertices()
    if target < len(tris):
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=int(target), preserveboundary=False,
            optimalplacement=True, planarquadric=True, preservenormal=True,
            qualitythr=0.3)
    cm = ms.current_mesh()
    return (np.asarray(cm.vertex_matrix(), dtype=np.float64),
            np.asarray(cm.face_matrix(), dtype=np.int64))


def unwrap(verts, faces):
    """xatlas -> (verts, faces, uv) with uv in [0, 1]. Vertices are duplicated
    at chart seams, which is why the vertex array comes back changed."""
    import xatlas

    vmap, idx, uvs = xatlas.parametrize(verts, faces)
    uv = np.asarray(uvs, dtype=np.float64)
    span = uv.max(axis=0) - uv.min(axis=0)
    if np.any(span > 1.5):                      # some builds return pixels
        uv = (uv - uv.min(axis=0)) / np.maximum(span, 1e-9)
    return verts[np.asarray(vmap)], np.asarray(idx, dtype=np.int64), uv


# ---------------------------------------------------------------------------
# the bake
# ---------------------------------------------------------------------------

def _rasterise(uv, faces, res):
    """Every texel covered by the new atlas. (texel_xy, face_id, barycentric).

    Walks the low-poly triangles rather than the texels: there are ~10k of the
    first and 4M of the second, and only the covered ones are worth a closest-
    point query later.
    """
    XY, FI, BA = [], [], []
    px = uv * (res - 1)
    for f in range(len(faces)):
        a, b, c = px[faces[f]]
        lo = np.floor(np.minimum(np.minimum(a, b), c)).astype(int)
        hi = np.ceil(np.maximum(np.maximum(a, b), c)).astype(int)
        lo = np.maximum(lo - 1, 0)
        hi = np.minimum(hi + 1, res - 1)
        if hi[0] < lo[0] or hi[1] < lo[1]:
            continue
        xs = np.arange(lo[0], hi[0] + 1)
        ys = np.arange(lo[1], hi[1] + 1)
        gx, gy = np.meshgrid(xs, ys)
        p = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float64)
        v0, v1, v2 = b - a, c - a, p - a
        den = v0[0] * v1[1] - v1[0] * v0[1]
        if abs(den) < 1e-12:
            continue
        w1 = (v2[:, 0] * v1[1] - v1[0] * v2[:, 1]) / den
        w2 = (v0[0] * v2[:, 1] - v2[:, 0] * v0[1]) / den
        w0 = 1.0 - w1 - w2
        m = (w0 >= -0.002) & (w1 >= -0.002) & (w2 >= -0.002)
        if not m.any():
            continue
        XY.append(p[m].astype(np.int64))
        FI.append(np.full(int(m.sum()), f, dtype=np.int64))
        BA.append(np.stack([w0[m], w1[m], w2[m]], axis=1))
    if not XY:
        raise SystemExit("the unwrap covered no texels")
    return np.concatenate(XY), np.concatenate(FI), np.concatenate(BA)


def _sample(images, tex_id, uvs):
    """Nearest-texel lookup, per source image. Returns (N, 3) uint8."""
    from PIL import Image

    out = np.zeros((len(uvs), 3), dtype=np.uint8)
    for i, path in enumerate(images):
        sel = np.nonzero(tex_id == i)[0]
        if not len(sel):
            continue
        with Image.open(path) as im:
            arr = np.asarray(im.convert("RGB"))
        h, w = arr.shape[:2]
        u = np.mod(uvs[sel, 0], 1.0) * (w - 1)
        # V IS FLIPPED. USD's `st` runs bottom-up; an image's row 0 is its top.
        v = (1.0 - np.mod(uvs[sel, 1], 1.0)) * (h - 1)
        out[sel] = arr[np.clip(v.astype(int), 0, h - 1),
                       np.clip(u.astype(int), 0, w - 1)]
    return out


def _dilate(rgb, filled, rounds=4):
    """Bleed colour outward past the chart edges.

    Without this the atlas is black between charts, and a renderer sampling
    just outside a triangle — which bilinear filtering and mipmapping both do —
    picks that black up as a dark seam along every chart border.
    """
    for _ in range(int(rounds)):
        if filled.all():
            break
        acc = np.zeros(rgb.shape, dtype=np.float64)
        cnt = np.zeros(filled.shape, dtype=np.float64)
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            s = np.roll(np.roll(rgb, dy, 0), dx, 1)
            f = np.roll(np.roll(filled, dy, 0), dx, 1)
            acc += s * f[..., None]
            cnt += f
        grow = (~filled) & (cnt > 0)
        rgb[grow] = (acc[grow] / cnt[grow][..., None]).astype(np.uint8)
        filled = filled | grow
    return rgb


def bake(hp_pts, hp_tris, hp_uv, hp_tex, images, lp_verts, lp_faces, lp_uv,
         res=2048, chunk=200_000):
    """Rasterise the new atlas and fill it from the old surface."""
    import trimesh

    xy, fi, ba = _rasterise(lp_uv, lp_faces, res)
    print(f"[bake] {len(xy)} texels covered of {res * res} "
          f"({100 * len(xy) / (res * res):.1f}%)", flush=True)

    # Where each texel sits in 3D, from the LOW-poly surface.
    tri3 = lp_verts[lp_faces[fi]]                     # (N, 3, 3)
    pos = (tri3 * ba[:, :, None]).sum(axis=1)

    # THE TRANSFER: nearest point on the HIGH-poly, and what it was wearing.
    mesh = trimesh.Trimesh(vertices=hp_pts, faces=hp_tris, process=False)
    q = trimesh.proximity.ProximityQuery(mesh)
    src_uv = np.zeros((len(pos), 2))
    src_tex = np.full(len(pos), -1, dtype=np.int64)
    for i in range(0, len(pos), chunk):
        sl = slice(i, min(i + chunk, len(pos)))
        close, _dist, fid = q.on_surface(pos[sl])
        tri = hp_pts[hp_tris[fid]]
        # barycentric of the closest point inside the triangle it landed on
        v0 = tri[:, 1] - tri[:, 0]
        v1 = tri[:, 2] - tri[:, 0]
        v2 = close - tri[:, 0]
        d00 = (v0 * v0).sum(1); d01 = (v0 * v1).sum(1); d11 = (v1 * v1).sum(1)
        d20 = (v2 * v0).sum(1); d21 = (v2 * v1).sum(1)
        den = np.maximum(d00 * d11 - d01 * d01, 1e-20)
        w1 = (d11 * d20 - d01 * d21) / den
        w2 = (d00 * d21 - d01 * d20) / den
        w = np.stack([1.0 - w1 - w2, w1, w2], axis=1)
        src_uv[sl] = (hp_uv[fid] * w[:, :, None]).sum(axis=1)
        src_tex[sl] = hp_tex[fid]
        print(f"[bake]   transferred {min(i + chunk, len(pos))}/{len(pos)}",
              flush=True)

    rgb = np.zeros((res, res, 3), dtype=np.uint8)
    filled = np.zeros((res, res), dtype=bool)
    ok = src_tex >= 0
    colours = _sample(images, src_tex[ok], src_uv[ok])
    # V FLIPPED AGAIN on the way out, for the same reason as `_sample`.
    rows = (res - 1) - xy[ok][:, 1]
    cols = xy[ok][:, 0]
    rgb[rows, cols] = colours
    filled[rows, cols] = True
    return _dilate(rgb, filled), float(ok.mean())


# ---------------------------------------------------------------------------
# writing it back out
# ---------------------------------------------------------------------------

def write_usd(out_usd, png_name, verts, faces, uv, up_axis="Z"):
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z if up_axis == "Z"
                           else UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/root/mesh")
    mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in verts])
    mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set([int(i) for i in faces.reshape(-1)])
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    # ONE UV SET, under both the name the source used and `st`, because
    # `_author_soup` in `mesh_damage` binds readers by name and a mismatch
    # renders as flat colour — the bug that made fragments untextured.
    corner = uv[faces].reshape(-1, 2)
    for name in ("st", "uv0"):
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            name, Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying)
        pv.Set([Gf.Vec2f(float(a), float(b)) for a, b in corner])

    mat = UsdShade.Material.Define(stage, "/root/_materials/baked")
    sh = UsdShade.Shader.Define(stage, "/root/_materials/baked/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    reader = UsdShade.Shader.Define(stage, "/root/_materials/baked/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, "/root/_materials/baked/diffuse")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(png_name))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    for wrap in ("wrapS", "wrapT"):
        tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.75)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
    stage.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("asset")
    ap.add_argument("--fraction", type=float, default=0.02)
    ap.add_argument("--res", type=int, default=2048)
    ap.add_argument("--scene", default="urban_v2")
    ap.add_argument("--out", default="scene_gen/assets/lowpoly")
    args = ap.parse_args()

    from PIL import Image

    import local_mirror as ML
    from archetypes import plan as P
    from compile_disaster import load_scene_config

    cfg = load_scene_config(args.scene)
    hits = [i for i in P.build_plan(cfg, "earthquake") if i.type == args.asset]
    if not hits:
        raise SystemExit(f"{args.asset!r} is not in the {args.scene} plan")
    src = ML.local_for(ML.to_url(hits[0].source))

    t0 = time.time()
    hp_pts, hp_tris, hp_uv, hp_tex, images = read_highpoly(src)
    print(f"[bake] high-poly {len(hp_tris)} triangles, {len(images)} textures "
          f"({time.time() - t0:.1f}s)", flush=True)

    t = time.time()
    target = max(64, int(round(len(hp_tris) * args.fraction)))
    lv, lf = decimate_free(hp_pts, hp_tris, target)
    print(f"[bake] decimated -> {len(lf)} faces "
          f"(ratio {len(lf) / len(hp_tris):.4f}) in {time.time() - t:.1f}s",
          flush=True)

    t = time.time()
    lv, lf, luv = unwrap(lv, lf)
    print(f"[bake] unwrapped {len(lf)} faces, {len(lv)} verts "
          f"in {time.time() - t:.1f}s", flush=True)

    t = time.time()
    rgb, hit = bake(hp_pts, hp_tris, hp_uv, hp_tex, images, lv, lf, luv,
                    res=args.res)
    print(f"[bake] baked in {time.time() - t:.1f}s, {100 * hit:.1f}% of texels "
          f"found a textured source", flush=True)

    os.makedirs(args.out, exist_ok=True)
    png = os.path.join(args.out, f"{args.asset}_baked.png")
    Image.fromarray(rgb).save(png)
    usd = os.path.join(args.out, f"{args.asset}_lowpoly.usdc")
    if os.path.exists(usd):
        os.remove(usd)
    write_usd(usd, os.path.basename(png), lv, lf, luv)
    print(f"[bake] wrote {usd}\n[bake] wrote {png}  "
          f"total {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
