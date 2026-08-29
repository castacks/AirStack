#!/usr/bin/env python
"""Which elevations of a GreatAmericanCity building are BLANK?

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_faces.py

A kit building made for a game city is only detailed where the player sees it:
the elevations meant to abut a neighbour, or to face a back alley, are flat
untextured slabs. Placed on an open corner they read as a missing wall, so the
layout has to know which side is which.

Detected from the geometry, not by eye. Every triangle is binned by its
area-weighted outward normal into the four compass elevations, and per side we
record area, triangle DENSITY (tri per m2) and the textures covering it. A
blank elevation is a large area carrying almost no triangles — a detailed
façade with modelled reveals and cornices runs one to two orders of magnitude
denser than a flat slab.

Writes `_plans/gac_faces.json`: per building, per side (N/E/S/W), the metrics
and a `blank` verdict, plus `front` — the densest side, which is the one that
wants to face the street.
"""

import json
import os
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
# Any building library, not just GreatAmericanCity: FACES_ROOT / FACES_NAMES /
# FACES_OUT override the defaults, so a new pack is classified with the same
# measurement rather than by eye.
ROOT = os.environ.get("FACES_ROOT") or (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
OUT = os.path.join(HERE, "..", "_plans",
                   os.environ.get("FACES_OUT") or "gac_faces.json")
NAMES = ([q for q in (os.environ.get("FACES_NAMES") or "").split(",") if q]
         or ["SM_Building_%02d" % i for i in range(1, 6)] +
         ["SM_Building_06_Small"] +
         ["SM_Building_%02d" % i for i in range(7, 32)])
EXT = os.environ.get("FACES_EXT", ".usd")
SIDES = ("E", "N", "W", "S")          # +X, +Y, -X, -Y
NORMALS = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
# a side is blank when it is a real wall (enough area) and carries almost no
# geometry; the split is wide in the data, not a knife-edge
# BLANK IS RELATIVE, NOT ABSOLUTE. A flat slab on a plain building and a
# flat slab on an ornate one have very different triangle densities, so a
# fixed cut-off catches 01 (0.14 against a 42.7 front) and misses 09 (2.80
# against a 5.93 front) — both of which are blank to the eye. Judging each
# side against its OWN building's best elevation catches both.
BLANK_RATIO = 0.55                    # of this building's densest side
BLANK_DENSITY = 4.0                   # ...and never call a dense side blank
MIN_AREA = 40.0                       # m2 — ignore slivers


def _texture_of(prim):
    try:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return ""
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
                    return v.path.rsplit("/", 1)[-1]
            return ""
    except Exception:
        return ""
    return ""


def measure(name):
    st = Usd.Stage.Open(ROOT + name + EXT)
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
    V = np.asarray(me.GetPointsAttr().Get(), dtype=np.float64) * S
    counts = np.asarray(me.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.asarray(me.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    start = np.zeros(len(counts) + 1, dtype=np.int64)
    np.cumsum(counts, out=start[1:])

    # face -> subset, for texture attribution
    sub_of = np.full(len(counts), -1, dtype=np.int64)
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
    tex = []
    for si, s in enumerate(subs):
        fi = np.asarray(s.GetIndicesAttr().Get() or [], dtype=np.int64)
        fi = fi[(fi >= 0) & (fi < len(counts))]
        sub_of[fi] = si
        tex.append(_texture_of(s.GetPrim()))

    lo, hi = V.min(axis=0), V.max(axis=0)
    W, D, H = hi - lo
    acc = {k: {"area": 0.0, "tris": 0, "tex": {}} for k in SIDES}
    for f in range(len(counts)):
        b, c = start[f], counts[f]
        for j in range(1, c - 1):
            a, p2, p3 = V[idx[b]], V[idx[b + j]], V[idx[b + j + 1]]
            n = np.cross(p2 - a, p3 - a)
            ar = 0.5 * float(np.linalg.norm(n))
            if ar <= 0.0:
                continue
            n = n / (2.0 * ar)
            if abs(n[2]) > 0.72:          # roof or floor, not an elevation
                continue
            # the elevation this triangle faces, and only if it is near the
            # OUTSIDE of the building — interior partitions face outward too
            best, bd = None, 0.0
            for k, (nx, ny) in NORMALS.items():
                d = n[0] * nx + n[1] * ny
                if d > bd:
                    best, bd = k, d
            if best is None or bd < 0.55:
                continue
            cen = (a + p2 + p3) / 3.0
            nx, ny = NORMALS[best]
            near = ((hi[0] - cen[0]) if nx > 0 else
                    (cen[0] - lo[0]) if nx < 0 else
                    (hi[1] - cen[1]) if ny > 0 else (cen[1] - lo[1]))
            if near > 2.5:                # not on the skin of that elevation
                continue
            acc[best]["area"] += ar
            acc[best]["tris"] += 1
            t = tex[sub_of[f]] if sub_of[f] >= 0 else ""
            acc[best]["tex"][t] = acc[best]["tex"].get(t, 0.0) + ar

    out = {"name": name, "usd": ROOT + name + EXT,
           "W": round(W, 1), "D": round(D, 1), "H": round(H, 1),
           "cx": round(0.5 * (lo[0] + hi[0]), 3),
           "cy": round(0.5 * (lo[1] + hi[1]), 3), "z0": round(lo[2], 3),
           "sides": {}}
    dens = {}
    for k in SIDES:
        a = acc[k]["area"]
        dens[k] = (acc[k]["tris"] / a) if a > 1e-6 else 0.0
    best = max(dens.values()) or 1.0
    for k in SIDES:
        a = acc[k]["area"]
        top = sorted(acc[k]["tex"].items(), key=lambda q: -q[1])[:2]
        out["sides"][k] = {
            "area_m2": round(a, 1), "tris": acc[k]["tris"],
            "tri_per_m2": round(dens[k], 2),
            "rel": round(dens[k] / best, 3),
            "blank": bool(a >= MIN_AREA and dens[k] < BLANK_DENSITY
                          and dens[k] < BLANK_RATIO * best),
            "tex": [t for t, _ in top]}
    out["front"] = max(SIDES, key=lambda k: dens[k])
    out["blank_sides"] = [k for k in SIDES if out["sides"][k]["blank"]]
    # THE PLACEMENT CLASS FALLS OUT OF THE COUNT, and this is the rule the
    # layout enforces (user, 2026-08-29): a building modelled on three
    # elevations belongs on a corner, two belongs at the end of a run, one
    # belongs in the middle with neighbours covering both flanks.
    n_good = 4 - len(out["blank_sides"])
    out["detailed_sides"] = n_good
    out["place"] = ("any" if n_good >= 4 else "corner" if n_good == 3
                    else "end" if n_good == 2 else "mid")
    return out


def main():
    res = []
    for nm in NAMES:
        try:
            r = measure(nm)
        except Exception as exc:
            print("%-22s FAILED %s" % (nm, exc), flush=True)
            continue
        if r is None:
            continue
        res.append(r)
        print("%-22s %5.1f x %5.1f x %6.1f m  front %s  blank %-12s  %s"
              % (nm, r["W"], r["D"], r["H"], r["front"],
                 ",".join(r["blank_sides"]) or "-",
                 "  ".join("%s %5.2f(%.2f)" % (k, r["sides"][k]["tri_per_m2"],
                                               r["sides"][k]["rel"])
                           for k in SIDES)), flush=True)
    json.dump(res, open(os.path.normpath(OUT), "w"), indent=1)
    print("\nwrote %s" % os.path.normpath(OUT))


if __name__ == "__main__":
    main()
