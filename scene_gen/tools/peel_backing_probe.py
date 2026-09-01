#!/usr/bin/env python
"""peel_backing_probe -- are the `render_peel` stamps ON their wall, and do
they carry the wall's own look?

    python3 scene_gen/tools/peel_backing_probe.py <bake.usd> [<bake.usd> ...]
    python3 scene_gen/tools/peel_backing_probe.py --census <dir-of-bakes>

WHY THIS EXISTS. `fire_bake._judge_candidates` cannot answer either question
for a peel: `peel`/`peelhalo` are not in `fire_bake._CANDIDATE_PREFIXES` at
all, so the airborne judge never even looks at them, and it has nothing to
say about materials. The user's review of the live 500 m fire city
(`kit_brownstone_row_F4_o4_EN_s438`, prim `bake/bake/k14/peel_k14_166`,
2026-08-31) reported BOTH defects on one prim: "this doesn't match the
material of the building it's on. it's also not on the building, it's
floating".

WHAT IT MEASURES, per `peel`/`peelhalo` (and, with `--kinds`, any other flat
stamp family):

  * `backing_m` -- the same test `fire_bake._wall_backing_contact` makes, but
    without vtk (host python has no vtk): fit the stamp's own flat normal
    (SVD on its points), shoot one ray each way from the centroid, and report
    the nearest hit on geometry that is NOT itself a stamp. `None` means
    nothing within `--reach` (default `fire_bake._BACKING_MAX_M`, 0.35 m) --
    i.e. floating.
  * `near_m` -- nearest point on any non-stamp surface to the stamp centroid,
    ray or no ray. Separates "off-plane but beside the wall" from "out in
    space over the street".
  * `shared_mat` -- the stamp's bound material lives OUTSIDE its own
    building's cell, i.e. it is a stage-shared look (one world-triplanar
    megascan for every building on the stage) rather than anything sampled
    from the building it is on. This is the material defect itself, and it
    is measurable with or without the geometry.
  * `map_mismatch` -- the stamp's base-colour map differs from that of the
    prim its backing ray hit. A DIAGNOSTIC, not a verdict: a stamp that is
    deliberately a flat tone (`peelhalo`'s scorch lip, and the peel face
    once it is a tone SAMPLED from the wall's map rather than a map of its
    own) has no map at all and always reads as a mismatch here.

Read-only. Bare `pxr` + `numpy`; safe to run on the host beside a live sim.
"""
import argparse
import glob
import math
import os
import sys

# Prefixes this probe judges. `peel`/`peelhalo` are `urban_fire.r_render_peel`
# via `_scar`; the rest are here so the same numbers can be read for the
# families `fire_bake` DOES judge, as a control.
DEFAULT_KINDS = ("peel", "peelhalo")
# Nothing in these families can back anything: a decal cannot back a decal
# (`fire_bake._wall_backing_contact`'s own rule). `peelrow` is the windrow of
# fallen render at the wall foot -- loose debris, not wall.
STAMP_FAMILIES = ("peel", "peelhalo", "spall", "spallhalo", "crack", "sbar",
                  "rebar", "scar", "plume", "peelrow")
REACH_M = 0.35            # `fire_bake._BACKING_MAX_M`


def _leaf_family(name):
    """`peel_k14_166` -> `peel`; the authored-name convention is
    `<kind>_<tag>_<uid>` (`quake_flow._b_face_mesh`)."""
    parts = name.split("_")
    while parts and (parts[-1].isdigit() or (parts[-1][:1] == "k"
                                             and parts[-1][1:].isdigit())):
        parts.pop()
    return "_".join(parts) if parts else name


def _tri_of(prim, xf):
    """(T,3,3) world triangles of a Mesh, fan-triangulated, or None."""
    import numpy as np
    from pxr import UsdGeom

    m = UsdGeom.Mesh(prim)
    pts = m.GetPointsAttr().Get()
    cnt = m.GetFaceVertexCountsAttr().Get()
    idx = m.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        return None
    P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
    M = np.array(xf.GetLocalToWorldTransform(prim), dtype=float)
    P = P @ M[:3, :3] + M[3, :3]
    idx = list(idx)
    tris, o = [], 0
    for c in cnt:
        c = int(c)
        for k in range(1, c - 1):
            tris.append((idx[o], idx[o + k], idx[o + k + 1]))
        o += c
    if not tris:
        return None
    return P[np.asarray(tris, dtype=np.int64)]


def _base_map(mat):
    """(material name, base-colour map basename or constant) for a material."""
    from pxr import Usd, UsdShade

    if not mat or not mat.GetPrim().IsValid():
        return ("(none)", "-")
    name = mat.GetPrim().GetName()
    best = None
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for key in ("diffuse_texture", "file", "diffuseColor",
                    "diffuse_color_constant", "base_color_constant"):
            i = sh.GetInput(key)
            if not i:
                continue
            if i.HasConnectedSource():
                ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                f = ts.GetInput("file") or ts.GetInput("diffuse_texture")
                v = f.Get() if f else None
                if v is not None:
                    return (name, "tex:" + str(v).rsplit("/", 1)[-1])
            else:
                v = i.Get()
                if v is None:
                    continue
                if key in ("diffuse_texture", "file"):
                    return (name, "tex:" + str(v).rsplit("/", 1)[-1])
                if best is None:
                    best = "rgb:" + ",".join("%.3f" % q for q in tuple(v)[:3])
    return (name, best or "-")


def _flat_normal(P):
    """Unit normal of a near-planar point cloud (`fire_bake._flat_normal`)."""
    import numpy as np

    if P.shape[0] < 3:
        return None
    Q = P - P.mean(axis=0)
    try:
        n = np.linalg.svd(Q, full_matrices=False)[2][2]
    except Exception:
        return None
    ln = float(np.linalg.norm(n))
    return None if ln < 1e-12 else n / ln


def _ray_tris(tris, o, d, reach):
    """Nearest Moller-Trumbore hit distance of ray `o`+t*`d` (t in
    (0, reach]) over `tris` (T,3,3), or None."""
    import numpy as np

    if not len(tris):
        return None, None
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = B - A, C - A
    h = np.cross(d, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    f = np.zeros_like(a)
    f[ok] = 1.0 / a[ok]
    s = o - A
    u = f * np.einsum("ij,ij->i", s, h)
    ok &= (u >= -1e-9) & (u <= 1.0 + 1e-9)
    q = np.cross(s, e1)
    v = f * (q @ d)
    ok &= (v >= -1e-9) & (u + v <= 1.0 + 1e-9)
    t = f * np.einsum("ij,ij->i", e2, q)
    ok &= (t > 1e-6) & (t <= reach)
    if not ok.any():
        return None, None
    w = np.where(ok)[0]
    j = w[int(np.argmin(t[w]))]
    return float(t[j]), int(j)


def _point_tri_dist(tris, p):
    """Min distance from point `p` to a (T,3,3) triangle soup, and the
    winning triangle index (Ericson, vectorised)."""
    import numpy as np

    if not len(tris):
        return None, None
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac, ap = B - A, C - A, p - A
    d1 = np.einsum("ij,ij->i", ab, ap)
    d2 = np.einsum("ij,ij->i", ac, ap)
    bp = p - B
    d3 = np.einsum("ij,ij->i", ab, bp)
    d4 = np.einsum("ij,ij->i", ac, bp)
    cp = p - C
    d5 = np.einsum("ij,ij->i", ab, cp)
    d6 = np.einsum("ij,ij->i", ac, cp)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    den = va + vb + vc
    with np.errstate(divide="ignore", invalid="ignore"):
        v = np.where(np.abs(den) > 1e-20, vb / den, 0.0)
        w = np.where(np.abs(den) > 1e-20, vc / den, 0.0)
    Q = A + v[:, None] * ab + w[:, None] * ac
    # region fixups: clamp to the closest feature when outside the triangle
    def _seg(P0, P1):
        e = P1 - P0
        tt = np.einsum("ij,ij->i", p - P0, e) / np.maximum(
            np.einsum("ij,ij->i", e, e), 1e-20)
        tt = np.clip(tt, 0.0, 1.0)
        return P0 + tt[:, None] * e
    out = (v < 0) | (w < 0) | (v + w > 1) | (np.abs(den) <= 1e-20)
    if out.any():
        cand = np.stack([_seg(A, B), _seg(B, C), _seg(C, A)], axis=1)
        dd = np.linalg.norm(cand - p, axis=2)
        pick = np.argmin(dd, axis=1)
        Q = np.where(out[:, None], cand[np.arange(len(cand)), pick], Q)
    dist = np.linalg.norm(Q - p, axis=1)
    j = int(np.argmin(dist))
    return float(dist[j]), j


def probe(path, kinds=DEFAULT_KINDS, reach=REACH_M, verbose=True):
    """Measure every stamp of `kinds` on one bake FILE."""
    from pxr import Usd

    stage = Usd.Stage.Open(path)
    if stage is None:
        return {"file": path, "error": "could not open"}
    return probe_stage(stage, kinds=kinds, reach=reach, verbose=verbose,
                       label=path)


def probe_stage(stage, kinds=DEFAULT_KINDS, reach=REACH_M, verbose=True,
                label="(stage)", root=None):
    """Measure every stamp of `kinds` on an OPEN stage — so the same
    measurement runs on a bake read off disk and on the stage a repro
    harness (`tools/peel_repro_probe.py`) has just authored in memory."""
    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade

    path = label
    xf = UsdGeom.XformCache()

    cands, wall_tris, wall_owner, owners = [], [], [], []
    if root:
        rp = stage.GetPrimAtPath(root)
        walk = Usd.PrimRange(rp, Usd.TraverseInstanceProxies()) if rp else ()
    else:
        walk = Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
    unloaded = 0
    for prim in walk:
        if prim.IsActive() and prim.HasAuthoredReferences() \
                and not prim.GetChildren():
            # a REFERENCED kit module that did not compose (its asset lives
            # on Nucleus / on the build machine): its geometry is simply not
            # here, so nothing can be measured against it
            unloaded += 1
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        fam = _leaf_family(prim.GetName())
        T = _tri_of(prim, xf)
        if T is None:
            continue
        if fam in kinds:
            cands.append((prim, fam, T))
        if fam in STAMP_FAMILIES:
            continue                       # a decal can never be backing
        wall_tris.append(T)
        wall_owner.append(np.full(len(T), len(owners), dtype=np.int64))
        owners.append(prim)
    W = np.vstack(wall_tris) if wall_tris else np.zeros((0, 3, 3))
    WO = np.concatenate(wall_owner) if wall_owner else np.zeros((0,),
                                                                dtype=np.int64)
    Wc = W.mean(axis=1) if len(W) else np.zeros((0, 3))

    rows, n_float, n_mismatch, n_shared = [], 0, 0, 0
    for prim, fam, T in cands:
        P = T.reshape(-1, 3)
        c = P.mean(axis=0)
        n = _flat_normal(P)
        near = np.linalg.norm(Wc - c, axis=1) <= (reach + 2.0) if len(W) else None
        sub = W[near] if near is not None else W
        subo = WO[near] if near is not None else WO
        backing_m, hit_prim = None, None
        if n is not None and len(sub):
            for sign in (1.0, -1.0):
                t, j = _ray_tris(sub, c, n * sign, reach)
                if t is not None and (backing_m is None or t < backing_m):
                    backing_m, hit_prim = t, owners[int(subo[j])]
        # nearest surface, over a wider net, for the diagnosis
        wide = np.linalg.norm(Wc - c, axis=1) <= 30.0 if len(W) else None
        sub2 = W[wide] if wide is not None else W
        subo2 = WO[wide] if wide is not None else WO
        near_m, near_prim = (None, None)
        if len(sub2):
            d, j = _point_tri_dist(sub2, c)
            near_m, near_prim = d, owners[int(subo2[j])]
        mm = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        my = _base_map(mm)
        ref = hit_prim if hit_prim is not None else near_prim
        wall = ("(none)", "-")
        if ref is not None:
            wm = UsdShade.MaterialBindingAPI(ref).ComputeBoundMaterial()[0]
            wall = _base_map(wm)
            if (wm is None or not wm.GetPrim().IsValid()):
                subs = [q for q in ref.GetChildren()
                        if q.GetTypeName() == "GeomSubset"]
                if subs:
                    wall = _base_map(UsdShade.MaterialBindingAPI(
                        subs[0]).ComputeBoundMaterial()[0])
        floating = backing_m is None
        mismatch = my[1] != wall[1]
        # THE MATERIAL TEST THAT WORKS WITHOUT THE GEOMETRY. A kit bake
        # REFERENCES its modules from Nucleus, so off the build machine the
        # walls do not compose and there is no "the wall behind it" to
        # compare against -- but the defect itself is still visible: a stamp
        # wearing a material from OUTSIDE its own building's cell is wearing
        # a stage-shared look (`/World/bake/FireLooks/brick_bare`, one
        # world-triplanar megascan for every building on the stage) instead
        # of anything sampled from the building it is on.
        cell = str(prim.GetPath().GetParentPath())
        mat_path = ("" if not mm or not mm.GetPrim().IsValid()
                    else str(mm.GetPrim().GetPath()))
        shared = not mat_path.startswith(cell + "/")
        n_float += 1 if floating else 0
        n_mismatch += 1 if mismatch else 0
        n_shared += 1 if shared else 0
        rows.append({"path": str(prim.GetPath()), "kind": fam,
                     "c": tuple(float(q) for q in c),
                     "backing_m": backing_m, "near_m": near_m,
                     "mat": my[0], "map": my[1], "mat_path": mat_path,
                     "shared": shared,
                     "wall": None if ref is None else str(ref.GetPath()),
                     "wall_mat": wall[0], "wall_map": wall[1],
                     "floating": floating, "mismatch": mismatch})
    by_kind = {}
    for r in rows:
        k = by_kind.setdefault(r["kind"], {"n": 0, "floating": 0,
                                           "shared": 0, "mismatch": 0})
        k["n"] += 1
        for q in ("floating", "shared", "mismatch"):
            k[q] += 1 if r[q] else 0
    out = {"file": path, "n": len(rows), "floating": n_float,
           "mismatch": n_mismatch, "shared": n_shared,
           "backing_meshes": len(owners), "unloaded": unloaded,
           "by_kind": by_kind, "rows": rows}
    if verbose:
        print("=" * 100)
        print("PEEL BACKING PROBE  {0}".format(os.path.basename(path)))
        print("  {0} stamp(s) of {1}; {2} floating (no backing within "
              "{3:.2f} m); {4} map mismatch; {5} on a stage-shared "
              "material".format(len(rows), "/".join(kinds), n_float, reach,
                                n_mismatch, n_shared))
        for k in sorted(by_kind):
            q = by_kind[k]
            print("    {0:<10} n={1:<4} floating={2:<4} shared_mat={3:<4} "
                  "map_mismatch={4}".format(k, q["n"], q["floating"],
                                            q["shared"], q["mismatch"]))
        print("  {0} non-stamp mesh(es) available as backing; {1} "
              "referenced prim(s) did not compose{2}".format(
                  len(owners), unloaded, "" if not unloaded else
                  "  <-- their geometry is absent, so `floating` is NOT "
                  "evidence on this file"))
        for r in sorted(rows, key=lambda r: r["path"]):
            print("  {0:<28} {1:<10} backing={2:<8} near={3:<8} "
                  "z={4:6.2f}".format(
                      r["path"].rsplit("/", 1)[-1], r["kind"],
                      "None" if r["backing_m"] is None
                      else "%.3f" % r["backing_m"],
                      "None" if r["near_m"] is None else "%.3f" % r["near_m"],
                      r["c"][2]))
            print("      stamp {0:<22} {1}".format(r["mat"], r["map"]))
            print("      wall  {0:<22} {1}   {2}".format(
                r["wall_mat"], r["wall_map"],
                (r["wall"] or "-").rsplit("/", 1)[-1]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--kinds", default=",".join(DEFAULT_KINDS))
    ap.add_argument("--reach", type=float, default=REACH_M)
    ap.add_argument("--census", action="store_true",
                    help="one line per bake, no per-stamp detail")
    a = ap.parse_args(argv)
    kinds = tuple(k for k in a.kinds.split(",") if k)
    files = []
    for p in a.paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "*.usd")))
        else:
            files += sorted(glob.glob(p))
    tot = {"n": 0, "floating": 0, "mismatch": 0, "shared": 0, "files": 0}
    for f in files:
        r = probe(f, kinds=kinds, reach=a.reach, verbose=not a.census)
        if r.get("error"):
            print("  *** {0}: {1}".format(os.path.basename(f), r["error"]))
            continue
        tot["files"] += 1
        for k in ("n", "floating", "mismatch", "shared"):
            tot[k] += r[k]
        if a.census and r["n"]:
            print("{0:<52} n={1:<4} shared_mat={2:<4} floating={3:<4} "
                  "map_mismatch={4:<4} uncomposed={5}".format(
                      os.path.basename(f), r["n"], r["shared"], r["floating"],
                      r["mismatch"], r["unloaded"]))
    print("\nTOTAL over {0} bake(s): {1} stamp(s), {2} on a stage-shared "
          "material, {3} floating, {4} map mismatch".format(
              tot["files"], tot["n"], tot["shared"], tot["floating"],
              tot["mismatch"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
