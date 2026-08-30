#!/usr/bin/env python
"""dtc_catalogue.py — the Downtown City catalogue the fire pipeline needs,
the analogue of `_plans/gac_buildings.json` for
`omniverse://.../Projects/SEI-COA/scene_gen/assets/downtowncity/`.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/dtc_catalogue.py"

Bare pxr + omni.client, no Kit, no SimulationApp — same recipe as
`tools/nucleus_fetch.py` / `tools/gac_measure.py` / `tools/_dtc_mpu.py`, and
safe to run beside a running Isaac Sim (it only reads Nucleus).

WHAT THIS DOES, IN ORDER
-------------------------
1. ENUMERATE the folder: `omni.client.list(ROOT)` first (proven in this repo
   by `nucleus_fetch.py`, which lists Nucleus folders on the bare interpreter
   with no Kit runtime); if that import or call fails, fall back to probing
   a candidate name list (`Amar_Tower`, `Building_01..20`, `Carved_01..20` —
   `Carved_01` and the "12 DowntownCity Carved_* blocks" are named in
   `config/asset_sets/urban_v2.yaml` and `tools/_dtc_mpu.py`) by attempting
   `Usd.Stage.Open` on each and keeping the ones that resolve to real
   geometry.

2. MEASURE each asset found: `Usd.Stage.Open` + `Usd.PrimRange(...,
   Usd.TraverseInstanceProxies())` (not `st.Traverse()` alone — downtowncity
   assets are not guaranteed to be the single flat mesh GreatAmericanCity is,
   and `PrimRange` + `TraverseInstanceProxies` is the pattern this repo
   already uses everywhere it must not assume that: `gac_fire.window_rects`,
   `gac_slice`, `pack_structure_probe.py`, `freeze.py`, ...).

   Bounding box (`W, D, H, cx, cy, z0`) comes from `UsdGeom.BBoxCache` over
   the pseudo-root, exactly as `tools/gac_measure.py` computes GAC's, so the
   two catalogues are numerically comparable. Everything else (points, tris,
   subsets, materials, glazing) is read per-mesh via the same PrimRange.

   MATERIALS: for every GeomSubset (or, if a mesh has none, the mesh itself),
   the bound material's diffuse map basename comes from
   `disaster.gac_fire._diffuse_of` (imported READ-ONLY — another agent is
   editing that file concurrently; see `_import_gac_fire` below for the
   retry). A material "looks like glazing by NAME" if `glass`, `window`,
   `curtain` or `glazing` appears in the MATERIAL PRIM's own name — this is
   independent of whether it carries a texture at all, which is exactly the
   curtain-wall case `gac_faces.py` had to add `_is_glass` for on
   GreatAmericanCity (a flat PBR material with no image). A material is
   counted as glazing overall if EITHER that name test fires OR its diffuse
   texture basename matches `detail.gac_slice.is_glazing` (the broader,
   already-tuned token list used everywhere else in this pipeline).

   GLAZING PER SIDE: every triangle (fan-triangulated, vectorised — no
   per-triangle Python loop, which is what makes this tractable on a
   557k-point single mesh like Amar_Tower or a merged Carved_* block) is
   binned to S/E/N/W by which bounding-box face its centroid is nearest —
   the same test `gac_fire.window_rects` uses to decide which elevation a
   glass face belongs to — with near-horizontal (roof/floor, |nz| > 0.72)
   triangles excluded first. `glazing_by_side[side]` is the glazing-area
   fraction of that side's total wall area (0.0 if the side has under 8 m2 of
   wall, too small to mean anything).

   STOREY COUNT: only attempted when cheap. The same glazing triangles' Z
   centroids are periodogrammed with `detail.gac_slice.storey_period` (the
   exact function GAC's own storey grid is measured with) — capped at 6000
   samples (a stratified stride, not a truncation, so a huge glazing count
   doesn't change which floors are represented) because `storey_period` is a
   plain double Python loop over (candidate periods x samples). Below
   confidence 0.35, or fewer than 6 glazing triangles anywhere, or a mesh
   whose total triangle count is too large to fan-triangulate in memory
   (`TRI_CAP`, in which case glazing-by-side is ALSO skipped for that one
   mesh and the printed row says so), it is left `null` rather than guessed.

3. WRITE `_plans/dtc_buildings.json` — GAC's schema (`name, mpu, meshes,
   subsets, points, tris, textures, W, D, H, cx, cy, z0, sec`) plus `url`,
   `materials` (list of `{name, tex, glazing_by_name}`), `glazing_materials`
   (`{count, textured, untextured}`), `glazing_by_side` (`{S,E,N,W}` glazing
   area fraction), `sides_with_glazing` (list, threshold `GLAZE_SIDE_MIN`),
   and `storeys` (`{period_m, confidence, n_storeys}` or `null`). Also prints
   the table sorted by H.
"""
import json
import os
import sys
import time

import numpy as np
from pxr import Usd, UsdGeom, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
sys.path.insert(0, SCENE_GEN)

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "scene_gen/assets/downtowncity/")
OUT = os.path.join(SCENE_GEN, "_plans", "dtc_buildings.json")
EXT = ".usdc"

SIDES = ("S", "E", "N", "W")
ROOF_NZ = 0.72              # |nz| above this -> roof/floor, not an elevation
MIN_SIDE_AREA_M2 = 8.0      # below this a side's glazing fraction is not meaningful
GLAZE_SIDE_MIN = 0.02       # side counted as "has glazing" above this fraction
GLAZING_NAME_TOKENS = ("glass", "window", "curtain", "glazing")
TRI_CAP = 4_000_000         # per-mesh fan-triangulation safety valve
STOREY_Z_CAP = 6000         # storey_period is O(periods x samples); stride above this
STOREY_MIN_CONF = 0.35

CANDIDATE_NAMES = (
    ["Amar_Tower"] +
    ["Building_%02d" % i for i in range(1, 21)] +
    ["Carved_%02d" % i for i in range(1, 21)]
)


# ---------------------------------------------------------------------------
# Read-only import of the file another agent is concurrently editing.
# ---------------------------------------------------------------------------
def _import_gac_fire():
    import time as _t
    last = None
    for attempt in range(3):
        try:
            from disaster import gac_fire as gf
            return gf
        except Exception as exc:
            last = exc
            print("import disaster.gac_fire failed (attempt %d): %s -- "
                  "waiting 30s and retrying" % (attempt + 1, exc), flush=True)
            _t.sleep(30)
    raise RuntimeError("could not import disaster.gac_fire after retries: %s" % last)


GF = _import_gac_fire()
from detail import gac_slice as GSL   # noqa: E402  (own module, not being edited)


# ---------------------------------------------------------------------------
# 1. Enumeration
# ---------------------------------------------------------------------------
def enumerate_assets():
    """(method, [names]) — basenames (no extension) found under ROOT."""
    try:
        import omni.client as oc
    except Exception as exc:
        print("import omni.client FAILED (%s) -- this needs the Kit/Nucleus "
              "client library; falling back to name-probing" % exc, flush=True)
        oc = None

    if oc is not None:
        try:
            r, ents = oc.list(ROOT)
        except Exception as exc:
            print("omni.client.list(%s) raised %s -- falling back to "
                  "name-probing" % (ROOT, exc), flush=True)
            r, ents = None, None
        if r == oc.Result.OK:
            names = []
            for e in ents:
                if e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN:
                    continue    # a subdirectory (e.g. a stray textures/ dir)
                low = e.relative_path.lower()
                if low.endswith(".usdc") or low.endswith(".usd"):
                    names.append(os.path.splitext(e.relative_path)[0])
            if names:
                return "omni.client.list", sorted(set(names))
            print("omni.client.list(%s) returned OK with %d entries but none "
                  "were .usd/.usdc -- falling back to name-probing"
                  % (ROOT, len(ents) if ents else 0), flush=True)
        else:
            print("omni.client.list(%s) result=%s -- falling back to "
                  "name-probing" % (ROOT, r), flush=True)

    found = []
    for nm in CANDIDATE_NAMES:
        url = ROOT + nm + EXT
        try:
            st = Usd.Stage.Open(url)
        except Exception:
            continue
        if st is None:
            continue
        try:
            if any(True for _ in Usd.PrimRange(
                    st.GetPseudoRoot(), Usd.TraverseInstanceProxies())):
                found.append(nm)
        except Exception:
            continue
    return "name-probe", found


# ---------------------------------------------------------------------------
# 2. Material / glazing classification
# ---------------------------------------------------------------------------
def _diffuse_texname(mat):
    if mat is None or not mat.GetPrim().IsValid():
        return ""
    _sp, _inp, url = GF._diffuse_of(mat.GetPrim())
    return (url or "").rsplit("/", 1)[-1]


def _looks_glazing_by_name(name):
    low = (name or "").lower()
    return any(t in low for t in GLAZING_NAME_TOKENS)


def _classify_material(mat):
    """(name, tex, glazing_by_name, glazing) for one bound material, or
    (None, "", False, False) if unbound."""
    if mat is None or not mat.GetPrim().IsValid():
        return None, "", False, False
    name = mat.GetPrim().GetName()
    tex = _diffuse_texname(mat)
    by_name = _looks_glazing_by_name(name)
    glazing = by_name or GSL.is_glazing(tex)
    return name, tex, by_name, glazing


# ---------------------------------------------------------------------------
# 3. Per-building measurement
# ---------------------------------------------------------------------------
def measure(name):
    url = ROOT + name + EXT
    t0 = time.time()
    st = Usd.Stage.Open(url)
    if st is None:
        return None
    try:
        st.Load()
    except Exception:
        pass
    S = UsdGeom.GetStageMetersPerUnit(st) or 1.0

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_,
                                                     UsdGeom.Tokens.render])
    rng = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    a, b = rng.GetMin(), rng.GetMax()
    lo = np.array([a[0], a[1], a[2]], dtype=float) * S
    hi = np.array([b[0], b[1], b[2]], dtype=float) * S
    W, D, H = hi - lo
    cx, cy, z0 = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), lo[2]

    xc = UsdGeom.XformCache()
    npts = ntri = nsub = 0
    n_meshes = 0
    mat_by_path = {}          # material prim path (str) -> (name, tex, by_name, glazing)
    side_area = {k: 0.0 for k in SIDES}
    side_glazing_area = {k: 0.0 for k in SIDES}
    glazing_zs = []
    capped_meshes = []

    for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        n_meshes += 1
        npts += len(pts)
        tri_counts = np.maximum(counts - 2, 0)
        ntri += int(tri_counts.sum())

        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        nsub += len(subs)
        sub_of = np.full(len(counts), -1, dtype=np.int64)
        sub_glazing = []      # per-subset-index glazing bool, in subset order
        if subs:
            for si, s in enumerate(subs):
                fi = np.asarray(s.GetIndicesAttr().Get() or [], dtype=np.int64)
                fi = fi[(fi >= 0) & (fi < len(counts))]
                sub_of[fi] = si
                mat = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                mname, tex, by_name, glz = _classify_material(mat)
                sub_glazing.append(glz)
                if mat is not None and mat.GetPrim().IsValid():
                    key = str(mat.GetPrim().GetPath())
                    mat_by_path.setdefault(key, (mname, tex, by_name, glz))
        else:
            mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            mname, tex, by_name, glz = _classify_material(mat)
            sub_of[:] = 0
            sub_glazing.append(glz)
            if mat is not None and mat.GetPrim().IsValid():
                key = str(mat.GetPrim().GetPath())
                mat_by_path.setdefault(key, (mname, tex, by_name, glz))

        total_tris = int(tri_counts.sum())
        if total_tris <= 0:
            continue
        if total_tris > TRI_CAP:
            capped_meshes.append((prim.GetName(), total_tris))
            continue

        M = np.asarray(xc.GetLocalToWorldTransform(prim), dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3] * S

        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        face_id = np.repeat(np.arange(len(counts)), tri_counts)
        ptr = np.concatenate([[0], np.cumsum(tri_counts)])
        j = np.arange(total_tris) - np.repeat(ptr[:-1], tri_counts)
        v0 = fvi[start[face_id]]
        v1 = fvi[start[face_id] + 1 + j]
        v2 = fvi[start[face_id] + 2 + j]
        p0, p1, p2 = P[v0], P[v1], P[v2]

        nvec = np.cross(p1 - p0, p2 - p0)
        lenv = np.linalg.norm(nvec, axis=1)
        valid = lenv > 1e-9
        area = np.zeros(total_tris)
        area[valid] = 0.5 * lenv[valid]
        nz = np.zeros(total_tris)
        nz[valid] = nvec[valid, 2] / lenv[valid]
        keep = valid & (np.abs(nz) <= ROOF_NZ) & (area > 1e-9)
        if not np.any(keep):
            continue

        cen = (p0 + p1 + p2)[keep] / 3.0
        area_k = area[keep]
        face_id_k = face_id[keep]

        # side by nearest bbox face -- gac_fire.window_rects' own test
        dist = np.stack([cen[:, 1] - lo[1], hi[0] - cen[:, 0],
                          hi[1] - cen[:, 1], cen[:, 0] - lo[0]], axis=1)
        side_ix = np.argmin(dist, axis=1)     # 0 S, 1 E, 2 N, 3 W

        si_per_tri = sub_of[face_id_k]
        glz_lookup = np.asarray(list(sub_glazing) + [False], dtype=bool)
        si_safe = np.where(si_per_tri >= 0, si_per_tri, len(sub_glazing))
        glz_k = glz_lookup[si_safe]

        if np.any(glz_k):
            zk = cen[glz_k, 2]
            if len(zk) > 400:                  # keep the z-sample bounded per mesh too
                stride = max(1, len(zk) // 400)
                zk = zk[::stride]
            glazing_zs.extend(zk.tolist())

        for k, side in enumerate(SIDES):
            m = side_ix == k
            if not np.any(m):
                continue
            side_area[side] += float(area_k[m].sum())
            side_glazing_area[side] += float(area_k[m][glz_k[m]].sum())

    if n_meshes == 0:
        return None

    glazing_by_side = {}
    for side in SIDES:
        ar = side_area[side]
        glazing_by_side[side] = (round(side_glazing_area[side] / ar, 3)
                                  if ar >= MIN_SIDE_AREA_M2 else 0.0)
    sides_with_glazing = [s for s in SIDES if glazing_by_side[s] >= GLAZE_SIDE_MIN]

    storeys = None
    if len(glazing_zs) >= 6:
        zs = glazing_zs
        if len(zs) > STOREY_Z_CAP:
            stride = max(1, len(zs) // STOREY_Z_CAP)
            zs = zs[::stride]
        try:
            period, _phase, conf = GSL.storey_period(zs)
        except Exception:
            period, conf = None, 0.0
        if period and conf >= STOREY_MIN_CONF:
            storeys = {"period_m": round(float(period), 2),
                       "confidence": round(float(conf), 3),
                       "n_storeys": int(round(float(H) / period))}

    materials = sorted(
        ({"name": mn, "tex": tex, "glazing_by_name": bn, "glazing": gz}
         for mn, tex, bn, gz in mat_by_path.values()),
        key=lambda d: d["name"] or "")
    glazing_mats = [m for m in materials if m["glazing"]]
    textures = {m["tex"] for m in materials if m["tex"]}

    rec = {
        "name": name, "url": url, "mpu": S,
        "meshes": n_meshes, "subsets": nsub,
        "points": npts, "tris": ntri, "textures": len(textures),
        "W": round(float(W), 1), "D": round(float(D), 1), "H": round(float(H), 1),
        "cx": round(float(cx), 3), "cy": round(float(cy), 3), "z0": round(float(z0), 3),
        "sec": round(time.time() - t0, 1),
        "materials": materials,
        "glazing_materials": {
            "count": len(glazing_mats),
            "textured": sum(1 for m in glazing_mats if m["tex"]),
            "untextured": sum(1 for m in glazing_mats if not m["tex"]),
        },
        "glazing_by_side": glazing_by_side,
        "sides_with_glazing": sides_with_glazing,
        "storeys": storeys,
    }
    if capped_meshes:
        rec["capped_meshes"] = capped_meshes    # glazing/side skipped for these
    return rec


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    method, names = enumerate_assets()
    print("enumeration method: %s -> %d name(s): %s"
          % (method, len(names), ", ".join(names)), flush=True)

    out = []
    for nm in names:
        try:
            rec = measure(nm)
        except Exception as exc:
            print("%-14s FAILED %s" % (nm, exc), flush=True)
            continue
        if rec is None:
            print("%-14s no geometry / open failed" % nm, flush=True)
            continue
        out.append(rec)
        st_txt = ("%d storeys (period %.2fm, conf %.2f)"
                   % (rec["storeys"]["n_storeys"], rec["storeys"]["period_m"],
                      rec["storeys"]["confidence"])) if rec["storeys"] else "?"
        cap_txt = (" CAPPED:%s" % rec["capped_meshes"]) if rec.get("capped_meshes") else ""
        print("%-14s %6.1f x %6.1f x %7.1f m  meshes=%-3d subsets=%-3d "
              "glazing_mats=%d(tex=%d,untex=%d) sides=%s storeys=%s  %.1fs%s"
              % (nm, rec["W"], rec["D"], rec["H"], rec["meshes"], rec["subsets"],
                 rec["glazing_materials"]["count"], rec["glazing_materials"]["textured"],
                 rec["glazing_materials"]["untextured"],
                 ",".join(rec["sides_with_glazing"]) or "-", st_txt, rec["sec"], cap_txt),
              flush=True)

    out.sort(key=lambda r: r["H"])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(os.path.normpath(OUT), "w"), indent=1)
    print("\nwrote %s (%d buildings)" % (os.path.normpath(OUT), len(out)))

    print("\n%-14s %22s %8s %7s %7s %-22s %s"
          % ("name", "W x D x H (m)", "storeys", "meshes", "subs",
             "glazing mats (tex/untex)", "sides w/ glazing"))
    print("-" * 100)
    for r in out:
        wdh = "%.1fx%.1fx%.1f" % (r["W"], r["D"], r["H"])
        st_txt = str(r["storeys"]["n_storeys"]) if r["storeys"] else "?"
        gm = r["glazing_materials"]
        gm_txt = "%d (%d/%d)" % (gm["count"], gm["textured"], gm["untextured"])
        print("%-14s %22s %8s %7d %7d %-22s %s"
              % (r["name"], wdh, st_txt, r["meshes"], r["subsets"], gm_txt,
                 ",".join(r["sides_with_glazing"]) or "-"))


if __name__ == "__main__":
    main()
