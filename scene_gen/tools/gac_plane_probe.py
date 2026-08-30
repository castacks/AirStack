#!/usr/bin/env python
"""gac_plane_probe — verifies the GAC (merged whole-asset) façade-plane fix:
"the fire is floating" (user review, 2026-08-30) traced to `gac_fire.side_frame`
hanging every elevation's wall frame off the MASS BBOX face, which is the
outer extent of whatever sticks out furthest (cornices, canopies, signage) —
often 1-3 m proud of the real wall the windows sit in. `window_rects` now
also measures the real façade plane per side from the glass-island faces
it already visits, and `side_frame` / `openings_provider` / `prepare` thread
it through as an optional `planes` dict.

For each building: per side, the bbox face coordinate, the measured plane,
the inset between them, and how many glass faces fed the measurement. Then
`prepare()` is run and, for the first flame-state event on a burning side
(or any opening if the level plans none), the world point
`urban_fire._flame_sources` would push a flame emitter to is computed BOTH
ways — with `side_frame(m, side)` (no `planes`: the old bbox-face frame) and
`side_frame(m, side, planes)` (the fix) — and the distance from each to the
nearest glass vertex on that side is printed.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/gac_plane_probe.py [NAME ...]"

With no NAME given, probes the six buildings the review named:
SM_Building_02, SM_Building_24, SM_Building_04, SM_Building_06_Small,
SM_Building_09, SM_Building_01.
"""
import random
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                          # noqa: E402
from detail import gac_slice as gsl                              # noqa: E402
from disaster import gac_fire as gf, quake_flow as qf, urban_fire as uf  # noqa: E402

DEFAULT_NAMES = ["SM_Building_02", "SM_Building_24", "SM_Building_04",
                  "SM_Building_06_Small", "SM_Building_09", "SM_Building_01"]
LEVEL = "F3"
SIDES = ("S", "E", "N", "W")
OUT_SIGN = {"E": 1.0, "N": 1.0, "S": -1.0, "W": -1.0}
GLASS_RECESS_M = 0.15   # must match the constant in gac_fire.window_rects


def new_stage():
    """A fresh in-memory stage with one empty cell, `/W/b0` — the same seat
    `place_source` expects, mirroring `gac_fire_probe.py`/`gac_burn_probe.py`.
    A separate stage per pass (raw measurement vs. `prepare()`) avoids
    double-referencing the asset under the same `cell/src` holder path."""
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/b0"
    UsdGeom.Xform.Define(st, cell)
    return st, cell


def face_plane_stats(stage, src, glass_tex):
    """Independent re-walk of the glass faces, mirroring the accumulation
    `gac_fire.window_rects` now does internally: per side, the list of
    per-face outward coordinates (before the +0.15 m recess offset) and
    every glass-face vertex (for the nearest-vertex distance check below).
    A second implementation of the same walk, not a call into
    `window_rects`, so this probe can cross-check that function's `planes`
    output independently."""
    root = stage.GetPrimAtPath(src)
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()

    def _tex(p):
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return ""
        _sp, _inp, url = gf._diffuse_of(mat.GetPrim())
        return (url or "").rsplit("/", 1)[-1]

    raw = {s: [] for s in SIDES}     # per-face outward coordinate
    verts = {s: [] for s in SIDES}   # every glass-face vertex, cell frame
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
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            if not any(g in _tex(sub.GetPrim()).lower() for g in glass_tex):
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
                side = gsl._side_of(*(n / ln))
                if side is None:
                    continue
                oc = V[:, 0] if side in ("E", "W") else V[:, 1]
                raw[side].append(float(oc.mean()))
                verts[side].extend(V.tolist())
    return raw, verts


def bbox_face(bbox, side):
    (x0, y0, _z0), (x1, y1, _z1) = bbox
    return {"S": y0, "E": x1, "N": y1, "W": x0}[side]


def nearest_vertex_dist(pt, vlist):
    if not vlist:
        return None
    A = np.asarray(vlist, dtype=float)
    d = np.linalg.norm(A - np.asarray(pt, dtype=float), axis=1)
    return float(d.min())


def probe_one(name):
    print("=" * 70)
    print("[gac_plane_probe] {0}".format(name))
    t0 = time.time()

    # ---- pass 1: raw glass-face measurement, on its own stage ----
    st1, cell1 = new_stage()
    src1 = gf.place_source(st1, cell1, gf.GAC_DIR + name + ".usd", gf.GAC_SCALE)
    if not src1:
        print("  nothing composed, skipping")
        return
    wins, bbox = gsl.window_centres(st1, src1)
    raw, verts = face_plane_stats(st1, src1, gsl.GLASS_TEX)

    measured = {}
    for side in SIDES:
        vals = raw.get(side) or []
        bf = bbox_face(bbox, side)
        if not vals:
            print("  {0}: bbox_face {1:9.3f}   (no glass on this elevation)"
                  .format(side, bf))
            continue
        plane = float(np.median(vals)) + OUT_SIGN[side] * GLASS_RECESS_M
        inset = abs(bf - plane)
        measured[side] = plane
        print("  {0}: bbox_face {1:9.3f}   measured_plane {2:9.3f}   "
              "inset {3:6.3f} m   ({4} island face(s))"
              .format(side, bf, plane, inset, len(vals)))

    # cross-check against the production window_rects() on the same stage
    planes_chk = {}
    gf.window_rects(st1, src1, planes=planes_chk)
    for side in SIDES:
        if side in measured and side in planes_chk:
            if abs(measured[side] - planes_chk[side]) > 1e-6:
                print("  [WARN] {0}: probe plane {1:.4f} != window_rects "
                      "plane {2:.4f}".format(side, measured[side],
                                              planes_chk[side]))

    # ---- pass 2: prepare() end to end, on a fresh stage ----
    st2, cell2 = new_stage()
    rng = random.Random(7)
    pre = gf.prepare(st2, cell2, name, LEVEL, rng, "probe", verbose=False)
    m, fire, events, planes = pre["mass"], pre["fire"], pre["events"], pre["planes"]
    for side in SIDES:
        if side in measured and side in planes:
            if abs(measured[side] - planes[side]) > 1e-6:
                print("  [WARN] {0}: pass-1 plane {1:.4f} != prepare() "
                      "plane {2:.4f}".format(side, measured[side], planes[side]))

    flame_ev = next((e for e in events if e.get("state") == "flame"
                     and e.get("side") in fire["sides"]), None)
    if flame_ev is None:
        flame_ev = next((e for e in events if e.get("state") == "flame"), None)
    if flame_ev is not None:
        side = flame_ev["side"]
        hua, hub = float(flame_ev["u0"]), float(flame_ev["u1"])
        hva, hvb = float(flame_ev["z_sill"]), float(flame_ev["z_head"])
        out_val = -0.05          # openings_provider's own "out" constant
        desc = "flame event u[{0:.2f},{1:.2f}] z[{2:.2f},{3:.2f}] on {4}" \
            .format(hua, hub, hva, hvb, side)
    else:
        # ANY OPENING, IF NONE: first non-empty provider record on a
        # burning side, any storey.
        side = fire["sides"][0]
        hua = None
        for st_i in range(len(m["levels"])):
            recs = pre["provider"](None, "main", side, st_i)
            if recs:
                op = recs[0]
                hua, hub, hva, hvb = op["hua"], op["hub"], op["hva"], op["hvb"]
                out_val = op["out"]
                break
        if hua is None:
            print("  no flame event and no opening at all on side {0}; "
                  "skipping the flame-distance check".format(side))
            print("  ({0:.1f}s)".format(time.time() - t0))
            return
        desc = "opening (no flame event) u[{0:.2f},{1:.2f}] z[{2:.2f},{3:.2f}] " \
            "on {4}".format(hua, hub, hva, hvb, side)

    u, v = 0.5 * (hua + hub), 0.5 * (hva + hvb)
    out = out_val + uf.FLAME_OUT
    fr_before = gf.side_frame(m, side)             # no planes: bbox-face frame
    fr_after = gf.side_frame(m, side, planes)      # the fix: measured plane
    p_before = qf._b_face_pt(fr_before, u, v, out)
    p_after = qf._b_face_pt(fr_after, u, v, out)
    d_before = nearest_vertex_dist(p_before, verts.get(side) or [])
    d_after = nearest_vertex_dist(p_after, verts.get(side) or [])
    print("  flame check: {0}".format(desc))
    print("    BEFORE (bbox face)      world pt ({0:.3f}, {1:.3f}, {2:.3f})  "
          "-> nearest glass vertex {3}".format(
              p_before[0], p_before[1], p_before[2],
              "n/a" if d_before is None else "%.3f m" % d_before))
    print("    AFTER  (measured plane) world pt ({0:.3f}, {1:.3f}, {2:.3f})  "
          "-> nearest glass vertex {3}".format(
              p_after[0], p_after[1], p_after[2],
              "n/a" if d_after is None else "%.3f m" % d_after))
    print("  ({0:.1f}s)".format(time.time() - t0))


if __name__ == "__main__":
    names = sys.argv[1:] or DEFAULT_NAMES
    for nm in names:
        probe_one(nm)
