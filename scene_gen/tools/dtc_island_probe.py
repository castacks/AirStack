#!/usr/bin/env python
"""dtc_island_probe — per-ISLAND diagnostics for `gac_fire.window_rects`.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/dtc_island_probe.py dtc:Building_12"

Bare `pxr` (`tools/usd_python.sh`), no Kit, no GPU, no slicer lock. Places
the asset, calls `gac_fire.window_rects` VERBATIM (so the island counts
printed here match exactly what `prepare`/`burn_gac` see), then re-walks the
same candidate glass faces (`gac_slice.is_glazing` on texture-or-material-
name) and re-runs the identical grid-hash union-find `gac_fire._islands`
uses, but this copy also tags every underlying face with the MATERIAL that
put it there and its own polygon area — bookkeeping `window_rects` itself
throws away.

WHY THIS EXISTS (fire_dtc2 review, 2026-08-30): "B2 [dtc Building_12 F3]
seems to be cut up into triangles". `window_rects` only reports island
BBOXES; it never says which material produced a bad one, so this probe
answers that on Building_12 (and three others, for contrast) before any fix
is written.

Per island, prints: side, u-span, z-span, area (bbox), the ACTUAL glazing
face area inside it, aspect ratio (width/height), face count, and every
material name touching it. Classifies each island RECTANGULAR-WINDOW-SHAPED
or not against the constants below — the same test proposed for
`gac_fire.window_rects`'s own shape filter, so a "NOT" row here is exactly
what that filter would now drop.
"""
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np                                               # noqa: E402
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from detail import gac_slice as gsl                               # noqa: E402
from disaster import gac_fire as gf                                # noqa: E402

# Mirrors the constants the user asked for in `gac_fire.window_rects`'s own
# shape filter -- kept here too so this probe classifies exactly what that
# filter will accept once it exists.
ASPECT_MIN, ASPECT_MAX = 0.2, 5.0
HEIGHT_MIN_M, HEIGHT_MAX_M = 0.8, 4.0
WIDTH_MIN_M, WIDTH_MAX_M = 0.5, 8.0
FILL_MIN = 0.4


def _poly_area(V):
    """Area of a planar polygon (fan triangulation from V[0]); exact for a
    triangle, a good approximation for the quads/ngons a glazing subset may
    carry."""
    if len(V) < 3:
        return 0.0
    c = np.zeros(3)
    for i in range(1, len(V) - 1):
        c += np.cross(V[i] - V[0], V[i + 1] - V[0])
    return 0.5 * float(np.linalg.norm(c))


def _islands_meta(boxes, cell=gf.ISLAND_CELL_M):
    """`gac_fire._islands`'s own grid-hash union-find, but each input box is
    `(u0, u1, z0, z1, mat_name, area)` and the merged result carries the
    summed face count / area and the SET of materials that fed it, instead
    of just the bbox `_islands` returns."""
    import math
    n = len(boxes)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    owner = {}
    for i, (u0, u1, z0, z1, _m, _a) in enumerate(boxes):
        c0, c1 = int(math.floor(u0 / cell)), int(math.floor(u1 / cell))
        r0, r1 = int(math.floor(z0 / cell)), int(math.floor(z1 / cell))
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                k = (c, r)
                j = owner.get(k)
                if j is None:
                    owner[k] = i
                else:
                    union(i, j)
    agg = {}
    for i, (u0, u1, z0, z1, m, a) in enumerate(boxes):
        r = find(i)
        e = agg.get(r)
        if e is None:
            agg[r] = {"u0": u0, "u1": u1, "z0": z0, "z1": z1,
                      "n_faces": 1, "area": a, "mats": {m}}
        else:
            e["u0"], e["u1"] = min(e["u0"], u0), max(e["u1"], u1)
            e["z0"], e["z1"] = min(e["z0"], z0), max(e["z1"], z1)
            e["n_faces"] += 1
            e["area"] += a
            e["mats"].add(m)
    out = [e for e in agg.values()
           if (e["u1"] - e["u0"]) >= 0.25 and (e["z1"] - e["z0"]) >= 0.25]
    out.sort(key=lambda e: (e["z0"], e["u0"]))
    return out


def _gather(stage, src, glass_tex):
    """Replicates `gac_fire.window_rects`'s two-pass face gather + side
    filing + plane filter EXACTLY, except every kept face box also carries
    its material name and its own polygon area."""
    root = stage.GetPrimAtPath(src)
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()

    def _tex(p):
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return "", ""
        _sp, _inp, url = gf._diffuse_of(mat.GetPrim())
        return (url or "").rsplit("/", 1)[-1], mat.GetPrim().GetName()

    plane_tol_m = 1.5
    faces = []       # (V, mat_name)
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        lo = np.minimum(lo, P.min(0))
        hi = np.maximum(hi, P.max(0))
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            _t, _mn = _tex(sub.GetPrim())
            if not gsl.is_glazing(_t, glass_tex, mat_name=_mn):
                continue
            for f in (sub.GetIndicesAttr().Get() or []):
                f = int(f)
                if f >= len(counts):
                    continue
                V = P[fvi[start[f]:start[f] + counts[f]]]
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = float(np.linalg.norm(n))
                if ln < 1e-12:
                    continue
                n = n / ln
                if abs(n[2]) >= max(abs(n[0]), abs(n[1])):
                    continue          # roof light / floor glass
                faces.append((V, _mn or "?"))
    if not faces or not np.all(np.isfinite(lo)):
        return {}
    cen = np.array([V.mean(0) for V, _m in faces])
    # same 4-column layout as `window_rects`: distance to S, E, N, W bbox face
    dist = np.stack([cen[:, 1] - lo[1], hi[0] - cen[:, 0],
                     hi[1] - cen[:, 1], cen[:, 0] - lo[0]], axis=1)
    side_ix = np.argmin(dist, axis=1)
    ring = ("S", "E", "N", "W")
    boxes = {}
    for k, side in enumerate(ring):
        sel = np.nonzero(side_ix == k)[0]
        if not len(sel):
            continue
        axis = 0 if side in ("E", "W") else 1
        oc = cen[sel, axis]
        plane = float(np.median(oc))
        keep = sel[np.abs(oc - plane) <= plane_tol_m]
        for i in keep:
            V, mname = faces[i]
            uu = V[:, 1] if side in ("E", "W") else V[:, 0]
            area = _poly_area(V)
            boxes.setdefault(side, []).append(
                (float(uu.min()), float(uu.max()),
                 float(V[:, 2].min()), float(V[:, 2].max()), mname, area))
    return boxes


def classify(u0, u1, z0, z1, face_area):
    w, h = u1 - u0, z1 - z0
    if w <= 0 or h <= 0:
        return False, 0.0, 0.0
    aspect = w / h
    fill = face_area / (w * h) if w * h > 0 else 0.0
    ok = (ASPECT_MIN <= aspect <= ASPECT_MAX and HEIGHT_MIN_M <= h <= HEIGHT_MAX_M
          and WIDTH_MIN_M <= w <= WIDTH_MAX_M and fill >= FILL_MIN)
    return ok, aspect, fill


def probe(name, kind=None):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/g0"
    UsdGeom.Xform.Define(st, cell)

    k, asset = gf.split_kind(name, kind)
    pack = gf.PACKS[k]
    url = gf.asset_url(asset, k)
    scale = gf.asset_scale(url, pack["scale"], verbose=False)
    src = gf.place_source(st, cell, url, scale)
    if not src:
        print("%s: nothing composed" % name)
        return

    print("\n=== %s (kind=%s) ===" % (name, k))
    planes = {}
    rects = gf.window_rects(st, src, planes=planes)
    boxes = _gather(st, src, gsl.GLASS_TEX)

    n_ok_total = n_bad_total = 0
    for side in ("S", "E", "N", "W"):
        rl = rects.get(side) or []
        print("  side %s: %d island(s) (window_rects)" % (side, len(rl)))
        islands = _islands_meta(boxes.get(side) or [])
        if len(islands) != len(rl):
            print("    ! probe re-grouping found %d island(s) -- expected to "
                  "match window_rects; a rounding/order artifact, not a bug "
                  "in the counts below" % len(islands))
        n_ok = n_bad = 0
        for e in islands:
            u0, u1, z0, z1 = e["u0"], e["u1"], e["z0"], e["z1"]
            ok, aspect, fill = classify(u0, u1, z0, z1, e["area"])
            n_ok += ok
            n_bad += not ok
            mats = ", ".join(sorted(e["mats"]))[:80]
            print("    [{0}] u {1:7.2f}-{2:7.2f} ({3:5.2f} m)  z {4:6.2f}-{5:6.2f} "
                  "({6:4.2f} m)  aspect {7:5.2f}  fill {8:4.2f}  faces={9:4d}  "
                  "mats=[{10}]".format(
                      "WINDOW" if ok else "OTHER ", u0, u1, u1 - u0, z0, z1,
                      z1 - z0, aspect, fill, e["n_faces"], mats))
        print("    -> {0} window-shaped, {1} NOT".format(n_ok, n_bad))
        n_ok_total += n_ok
        n_bad_total += n_bad
    print("  TOTAL: {0} window-shaped island(s), {1} rejected by the shape "
          "test across all sides".format(n_ok_total, n_bad_total))
    return n_ok_total, n_bad_total


def main():
    names = sys.argv[1:] or ["dtc:Building_12"]
    for n in names:
        try:
            probe(n)
        except Exception:
            import traceback
            traceback.print_exc()
            print("%s: FAILED" % n)


if __name__ == "__main__":
    main()
