#!/usr/bin/env python
"""soot_elevation.py — ONE elevation of a kit building, assembled from the
modules' own 2D materials, with the soot applied — no Isaac Sim.

    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/soot_elevation.py \\
       --style commercial_mid --level F3 --side S --mode skin"

WHY THIS EXISTS. "Can you create a PNG of the material by assembling the
different prim 2D materials of one face of the building and applying the
corresponding soot to it?" (user, 2026-08-30). The offline previews
(`tools/soot_png.py`) draw the soot over a synthetic wall; the sim render
shows it on the real cladding but costs a launch and hides WHICH step went
wrong. This sits between: it opens every module's USD (bare USD — the kit
lives on Nucleus, so this runs in the container with `usd_python.sh`, no
SimulationApp, safe beside a running sim), rasterises each module's
OUTWARD-facing triangles orthographically into elevation space (u along the
side, z up) and samples the module's own base-colour map through its UVs —
so the façade you see is the kit's real art, atlas islands and all — then:

    --mode skin    composites the soot SKIN at every façade pixel: what the
                   fire model says the elevation should look like, on the
                   real materials. The reference.
    --mode baked   bakes the skin into each module's base map THROUGH its
                   UVs with `soot_bake` (the sim path) and renders the
                   elevation from those baked maps: what the prims will
                   actually carry. `skin` and `baked` must agree; where they
                   do not, the bake is wrong, not the fire.
    --mode base    no soot: the assembled elevation alone, for orientation.

Frame conventions come from `quake_flow._piece_frame`: a kit piece is pivoted
at its left end on the wall line, runs along local +X, its art sits at local
-Y (outward), z up; world = pivot + Rz(yaw) . (scale . local). `dw` frame kits
(the dw_terrace family) pivot differently and are not handled here.

Outputs go to /isaac-sim/.nvidia-omniverse/logs/soot_elev/ (host:
~/docker/isaac-sim/logs/soot_elev/) as
    elev_<style>_<level>_<side>_<mode>.png
"""
import argparse
import math
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Usd, UsdGeom, UsdShade                      # noqa: E402
from detail import urban_building as ub                     # noqa: E402
from disaster import quake_flow as qf                       # noqa: E402
from disaster import soot_plume as spl                      # noqa: E402
from disaster import urban_fire as uf                       # noqa: E402

OUT = "/isaac-sim/.nvidia-omniverse/logs/soot_elev"
PPM = 40.0

_STAGES, _ARRAYS, _TEX = {}, {}, {}


def module_data(name):
    """[(subset name, arrays, base rgb or None, bound material prim, shader
    path, input name, tex url)] per mesh/subset of the module USD."""
    if name in _ARRAYS:
        return _ARRAYS[name]
    out = []
    st = _STAGES.get(name)
    if st is None:
        st = Usd.Stage.Open(ub._usd(name))
        _STAGES[name] = st
    if not st:
        _ARRAYS[name] = out
        return out
    for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        arrays = uf._mesh_arrays(prim)
        if arrays is None:
            continue
        # a mesh-level xform inside the module USD (rare; usually identity)
        Mg = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        M = np.array([[float(Mg[r][c]) for c in range(4)] for r in range(4)])
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        targets = [(s.GetPrim(), s) for s in subsets] or [(prim, None)]
        for t, sub in targets:
            bound = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            bprim = bound.GetPrim() if bound else None
            sh_path, inp, tex = spl.find_basecolor(bprim)
            base = None
            if tex:
                base = _TEX.get(tex)
                if base is None:
                    base = spl._read_rgb(tex)
                    _TEX[tex] = base if base is not None else False
                if base is False:
                    base = None
            face_ids = None
            if sub is not None:
                face_ids = [int(k) for k in (sub.GetIndicesAttr().Get() or [])]
            out.append({"sub": sub.GetPrim().GetName() if sub is not None else "mesh",
                        "arrays": arrays, "M": M, "base": base, "face_ids": face_ids,
                        "tex": tex, "mat": bprim, "sh_path": sh_path, "inp": inp})
    _ARRAYS[name] = out
    return out


def triangulate(counts, indices, face_ids=None):
    tri, slot, face = [], [], []
    k = 0
    keep = None if face_ids is None else set(face_ids)
    for fi, c in enumerate(counts):
        c = int(c)
        if keep is None or fi in keep:
            for j in range(1, c - 1):
                tri.append((indices[k], indices[k + j], indices[k + j + 1]))
                slot.append((k, k + j, k + j + 1))
                face.append(fi)
        k += c
    return (np.asarray(tri, dtype=np.int64).reshape(-1, 3),
            np.asarray(slot, dtype=np.int64).reshape(-1, 3),
            np.asarray(face, dtype=np.int64))


def uv_of(arrays, tri, slot):
    uv, interp, ind = arrays["uv"], arrays["interp"], arrays["uv_indices"]
    if interp == "faceVarying":
        idx = ind[slot] if ind is not None else slot
    else:
        idx = ind[tri] if ind is not None else tri
    return uv[idx]                     # (T, 3, 2)


def sample_tex(img, uvs):
    """Bilinear sample of img (H,W,3) at uvs (P,2), v=1 at row 0, wrapping."""
    h, w = img.shape[0], img.shape[1]
    u = np.mod(uvs[:, 0], 1.0) * (w - 1)
    v = (1.0 - np.mod(uvs[:, 1], 1.0)) * (h - 1)
    x0 = np.floor(u).astype(int)
    y0 = np.floor(v).astype(int)
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    fx = (u - x0)[:, None]
    fy = (v - y0)[:, None]
    return ((img[y0, x0] * (1 - fx) + img[y0, x1] * fx) * (1 - fy)
            + (img[y1, x0] * (1 - fx) + img[y1, x1] * fx) * fy)


def render_elevation(ctx, side, mode, sk=None, baked=None, ppm=PPM):
    """Orthographic elevation of `side` of the burning mass: (rgb, depth,
    covered)."""
    f = ctx["fire"]
    m = ctx["info"]["masses"][f["mass"]]
    L = spl.side_length(m, side)
    H = float(m["top"] - m["z0"]) + spl.parapet_height(m)
    z0 = float(m["z0"])
    w, h = int(round(L * ppm)), int(round(H * ppm))
    rgb = np.full((h, w, 3), 0.5, dtype=np.float32)
    depth = np.full((h, w), -1e9, dtype=np.float32)
    ox, oy = qf._outward(m, side)
    n_tri = 0
    for e in qf._els(ctx, mass=f["mass"], role=("wall", "corner", "parapet",
                                                 "parapet_corner"), side=side):
        fr = qf._piece_frame(e)
        if fr is None or fr[6]:
            continue
        yaw = fr[2]
        ca, sa = math.cos(yaw), math.sin(yaw)
        sc = ub._scale(e["name"])
        for part in module_data(e["name"]):
            arrays = part["arrays"]
            tri, slot, _face = triangulate(arrays["counts"], arrays["indices"],
                                           part["face_ids"])
            if len(tri) == 0:
                continue
            P = arrays["points"].astype(np.float64)
            P = P @ part["M"][:3, :3] + part["M"][3, :3]
            P = P * sc
            # local -> world: pivot + Rz(yaw) . p
            wx = fr[0] + ca * P[:, 0] - sa * P[:, 1]
            wy = fr[1] + sa * P[:, 0] + ca * P[:, 1]
            wz = float(e["z"]) + P[:, 2]
            # elevation coordinates
            U = np.array([spl.side_u(m, side, x, y) for x, y in zip(wx, wy)])
            Zc = (H - (wz - z0)) * ppm
            Uc = U * ppm
            D = (wx - m["cx"]) * ox + (wy - m["cy"]) * oy
            base = part["base"]
            if mode == "baked" and baked is not None:
                base = baked.get((e["name"], part["sub"], id(e)), base)
            uvs = uv_of(arrays, tri, slot)
            for k in range(len(tri)):
                a, b, c = tri[k]
                # outward-facing only: world normal against the side's outward
                v1 = np.array([wx[b] - wx[a], wy[b] - wy[a], wz[b] - wz[a]])
                v2 = np.array([wx[c] - wx[a], wy[c] - wy[a], wz[c] - wz[a]])
                nrm = np.cross(v1, v2)
                nn = np.linalg.norm(nrm)
                if nn < 1e-9:
                    continue
                if (nrm[0] * ox + nrm[1] * oy) / nn < 0.2:
                    continue
                xs = np.array([Uc[a], Uc[b], Uc[c]])
                ys = np.array([Zc[a], Zc[b], Zc[c]])
                x0, x1 = int(max(0, math.floor(xs.min()))), int(min(w - 1, math.ceil(xs.max())))
                y0, y1 = int(max(0, math.floor(ys.min()))), int(min(h - 1, math.ceil(ys.max())))
                if x1 < x0 or y1 < y0:
                    continue
                gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5,
                                     np.arange(y0, y1 + 1) + 0.5)
                det = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
                if abs(det) < 1e-9:
                    continue
                l0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / det
                l1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / det
                l2 = 1.0 - l0 - l1
                eps = -1e-3
                inside = (l0 >= eps) & (l1 >= eps) & (l2 >= eps)
                if not inside.any():
                    continue
                d = l0 * D[a] + l1 * D[b] + l2 * D[c]
                rr, cc = gy[inside].astype(int), gx[inside].astype(int)
                dd = d[inside]
                win = dd > depth[rr, cc]
                if not win.any():
                    continue
                rr, cc, dd = rr[win], cc[win], dd[win]
                uvk = uvs[k]
                l0w, l1w, l2w = l0[inside][win], l1[inside][win], l2[inside][win]
                uv = (l0w[:, None] * uvk[0] + l1w[:, None] * uvk[1]
                      + l2w[:, None] * uvk[2])
                if base is not None:
                    col = sample_tex(base, uv)
                else:
                    col = np.full((len(rr), 3), 0.35, dtype=np.float32)
                rgb[rr, cc] = col
                depth[rr, cc] = dd
                n_tri += 1
    covered = depth > -1e8
    if mode == "skin" and sk is not None:
        rows, cols = np.nonzero(covered)
        u = (cols + 0.5) / ppm
        z = z0 + H - (rows + 0.5) / ppm
        # skin sampling in the skin's own coordinates
        off = sk["offsets"][side]
        sc_ = ((off + u) * sk["ppm"]) % sk["rgba"].shape[1]
        sr = np.clip((sk["H"] - (z - sk["z0"])) * sk["ppm"], 0, sk["rgba"].shape[0] - 1)
        s = sk["rgba"][sr.astype(int), sc_.astype(int)]
        a = s[:, 3:4]
        b = rgb[rows, cols]
        grey = b.mean(axis=1, keepdims=True)
        desat = b * (1 - spl.DESAT * a) + grey * (spl.DESAT * a)
        rgb[rows, cols] = desat * (1 - a) + s[:, :3] * a
    return rgb, depth, covered, n_tri


def render_gac_elevation(mesh, images, m, side, ppm=PPM):
    """Orthographic elevation of one side of a MERGED asset (the de-indexed
    mesh `gac_storey_slice.read_mesh` returns, in the cell frame), each
    triangle textured from `images[material index]` (an H x W x 3 array —
    the baked atlas where the fire reached it, the base map elsewhere)."""
    P, UV, MID = mesh["P"], mesh["UV"], mesh["MID"]
    L = spl.side_length(m, side)
    H = float(m["top"] - m["z0"])
    z0 = float(m["z0"])
    w, h = int(round(L * ppm)), int(round(H * ppm))
    rgb = np.full((h, w, 3), 0.5, dtype=np.float32)
    depth = np.full((h, w), -1e9, dtype=np.float32)
    ox, oy = qf._outward(m, side)
    # side-u for every vertex, vectorised (soot_plume.side_u's convention)
    lx = P[:, 0] - m["cx"]
    ly = P[:, 1] - m["cy"]
    W, D = float(m["W"]), float(m["D"])
    U = {"S": lx + W / 2.0, "E": ly + D / 2.0, "N": W / 2.0 - lx,
         "W": D / 2.0 - ly}[side]
    Uc = U * ppm
    Zc = (H - (P[:, 2] - z0)) * ppm
    Dd = lx * ox + ly * oy
    n_tri = 0
    T = len(MID)
    for k in range(T):
        a, b, c = 3 * k, 3 * k + 1, 3 * k + 2
        v1 = P[b] - P[a]
        v2 = P[c] - P[a]
        nx = v1[1] * v2[2] - v1[2] * v2[1]
        ny = v1[2] * v2[0] - v1[0] * v2[2]
        nz = v1[0] * v2[1] - v1[1] * v2[0]
        nn = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nn < 1e-9 or (nx * ox + ny * oy) / nn < 0.2:
            continue
        img = images.get(int(MID[k]))
        if img is None:
            continue
        xs = np.array([Uc[a], Uc[b], Uc[c]])
        ys = np.array([Zc[a], Zc[b], Zc[c]])
        x0, x1 = int(max(0, math.floor(xs.min()))), int(min(w - 1, math.ceil(xs.max())))
        y0, y1 = int(max(0, math.floor(ys.min()))), int(min(h - 1, math.ceil(ys.max())))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        det = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(det) < 1e-9:
            continue
        l0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / det
        l1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / det
        l2 = 1.0 - l0 - l1
        inside = (l0 >= -1e-3) & (l1 >= -1e-3) & (l2 >= -1e-3)
        if not inside.any():
            continue
        d = l0 * Dd[a] + l1 * Dd[b] + l2 * Dd[c]
        rr, cc = gy[inside].astype(int), gx[inside].astype(int)
        dd = d[inside]
        win = dd > depth[rr, cc]
        if not win.any():
            continue
        rr, cc, dd = rr[win], cc[win], dd[win]
        l0w, l1w, l2w = l0[inside][win], l1[inside][win], l2[inside][win]
        uv = (l0w[:, None] * UV[a] + l1w[:, None] * UV[b] + l2w[:, None] * UV[c])
        rgb[rr, cc] = sample_tex(img, uv)
        depth[rr, cc] = dd
        n_tri += 1
    return rgb, depth, depth > -1e8, n_tri


def gac_main(a):
    """`--gac NAME`: the merged asset's own elevation, base or baked."""
    from PIL import Image
    from disaster import gac_fire as gf

    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/b0"
    UsdGeom.Xform.Define(st, cell)
    rng = random.Random(a.seed)
    sides = tuple(v.strip() for v in a.sides.split(",") if v.strip()) or None
    pre = gf.prepare(st, cell, a.gac, a.level, rng, "elev{0}".format(a.seed),
                     sides=sides, out_dir=os.path.join(a.out, "atlases"))
    mesh, m, sooted = pre["mesh"], pre["mass"], pre["sooted"]
    pngs = sooted.get("_png", {}) if sooted else {}
    images = {}
    for k, mat in enumerate(mesh["mats"]):
        if mat is None:
            continue
        _sp, _inp, tex = gf._diffuse_of(mat.GetPrim())
        if not tex:
            continue
        if a.mode == "baked" and tex in pngs:
            images[k] = spl._read_rgb(pngs[tex], max_px=2048)
        else:
            images[k] = spl._read_rgb(tex, max_px=2048)
    side = a.side if a.side in pre["rects"] or a.side else pre["fire"]["sides"][0]
    rgb, depth, covered, n_tri = render_gac_elevation(mesh, images, m, side)
    if a.mode == "skin":
        sk = pre["skin"]
        rows, cols = np.nonzero(covered)
        H = float(m["top"] - m["z0"])
        u = (cols + 0.5) / PPM
        z = m["z0"] + H - (rows + 0.5) / PPM
        off = sk["offsets"][side]
        sc_ = ((off + u) * sk["ppm"]) % sk["rgba"].shape[1]
        sr = np.clip((sk["H"] - (z - sk["z0"])) * sk["ppm"], 0, sk["rgba"].shape[0] - 1)
        s_ = sk["rgba"][sr.astype(int), sc_.astype(int)]
        al = s_[:, 3:4]
        b_ = rgb[rows, cols]
        grey = b_.mean(axis=1, keepdims=True)
        desat = b_ * (1 - spl.DESAT * al) + grey * (spl.DESAT * al)
        rgb[rows, cols] = desat * (1 - al) + s_[:, :3] * al
    path = os.path.join(a.out, "elev_gac_{0}_{1}_{2}_{3}.png".format(
        a.gac, a.level, side, a.mode))
    Image.fromarray((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)).save(path)
    print("[soot_elevation] {0} {1} side {2} mode {3}: {4}x{5}, {6} triangle(s), "
          "{7:.0%} covered, {8}, {9} atlas(es) baked, {10:.1f}s -> {11}".format(
              a.gac, a.level, side, a.mode, rgb.shape[1], rgb.shape[0], n_tri,
              float(covered.mean()), spl.summarise(pre["events"]),
              len(pngs) // 2, time.time() - t0, path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gac", default="", help="a merged GAC asset name instead of a kit style")
    ap.add_argument("--style", default="commercial_mid")
    ap.add_argument("--level", default="F3")
    ap.add_argument("--side", default="S")
    ap.add_argument("--sides", default="", help="burning sides, e.g. S,E")
    ap.add_argument("--mode", default="skin", choices=("base", "skin", "baked"))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--origin", type=float, default=0.25)
    ap.add_argument("--px", type=int, default=768, help="bake resolution for --mode baked")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    if a.gac:
        return gac_main(a)

    from PIL import Image
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    rng = random.Random(a.seed)
    placements = ub.build_building(a.style, 0.0, 0.0, 0.0, rng)
    info = qf.describe(a.style, placements, 0.0, 0.0, 0.0)
    ctx = {"info": info, "rng": rng, "notes": [], "tag": "elev{0}".format(a.seed),
           "cache": {}}
    mtag = max(info["masses"].items(),
               key=lambda kv: (len(kv[1]["levels"]), kv[0] == "main"))[0]
    n_st = len(info["masses"][mtag]["levels"])
    origin = max(0, min(n_st - 1, int(round(a.origin * (n_st - 1)))))
    sides = tuple(v.strip() for v in a.sides.split(",") if v.strip()) or (
        ("S",) if a.level in ("F1", "F2") else ("S", "E"))
    ctx["fire"] = uf.plan_fire(info, a.level, rng, origin=origin, sides=sides)
    ctx["fire"]["events"] = spl.plan_events(ctx, uf._severity)
    heavy = 1.0
    for name, kw in uf.LADDER.get(info["type"], {}).get(a.level, []):
        if name == "smoke_stain":
            heavy = float((kw or {}).get("heavy", 1.0))
    sk = None
    if a.mode in ("skin", "baked"):
        sk = spl.skin(ctx, ctx["fire"]["events"],
                      np.random.default_rng(spl.event_seed(ctx) ^ 0x5EED),
                      finish=ctx["fire"].get("finish") or "char",
                      glass=(info["type"] == "rc_glass"), duration_scale=heavy)
    baked = None
    if a.mode == "baked":
        from disaster import soot_bake as sb
        m = info["masses"][mtag]
        baked = {}
        n_b = 0
        for e in qf._els(ctx, mass=mtag, role=("wall", "corner", "parapet",
                                               "parapet_corner"), side=a.side):
            fr = qf._piece_frame(e)
            if fr is None or fr[6]:
                continue
            yaw = fr[2]
            ca, sa = math.cos(yaw), math.sin(yaw)
            sc = ub._scale(e["name"])
            # the same local -> world as the renderer, as a 4x4 (row-vector)
            R = np.array([[ca * sc, sa * sc, 0.0, 0.0],
                          [-sa * sc, ca * sc, 0.0, 0.0],
                          [0.0, 0.0, sc, 0.0],
                          [fr[0], fr[1], float(e["z"]), 1.0]])
            for part in module_data(e["name"]):
                if part["base"] is None:
                    continue
                arrays = part["arrays"]
                key = (e["name"], part["sub"], a.px)
                pm = ctx["cache"].setdefault("posmap", {}).get(key)
                if pm is None:
                    pm = sb.uv_position_map(arrays["points"], arrays["counts"],
                                            arrays["indices"], arrays["uv"],
                                            arrays["interp"], arrays["uv_indices"],
                                            face_ids=part["face_ids"], px=a.px)
                    ctx["cache"]["posmap"][key] = pm
                pos, mask = pm
                M = part["M"] @ R
                out = sb.bake_module(sk, a.side, m, M, pos, mask, part["base"], px=a.px)
                baked[(e["name"], part["sub"], id(e))] = out
                n_b += 1
        print("[soot_elevation] baked {0} module map(s) at {1} px".format(n_b, a.px))
    rgb, depth, covered, n_tri = render_elevation(ctx, a.side, a.mode, sk=sk, baked=baked)
    path = os.path.join(a.out, "elev_{0}_{1}_{2}_{3}.png".format(a.style, a.level,
                                                                  a.side, a.mode))
    Image.fromarray((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)).save(path)
    print("[soot_elevation] {0} {1} side {2} mode {3}: {4}x{5}, {6} triangle(s) "
          "drawn, {7:.0%} covered, events {8}, {9:.1f}s -> {10}".format(
              a.style, a.level, a.side, a.mode, rgb.shape[1], rgb.shape[0], n_tri,
              float(covered.mean()), spl.summarise(ctx["fire"]["events"]),
              time.time() - t0, path))


if __name__ == "__main__":
    main()
