#!/usr/bin/env python
"""_gac_budget_probe -- THROWAWAY verification script (like the other
tools/_*.py probes). Measures the ACTUAL piece count `slice_to_kit` produces
on real GAC assets, run one at a time in sequence within a single process:
the printed `[storey_slice] budget:` line, the actual placement count and its
breakdown by `_role`, the levers the planner pulled (bay grouping k, whether
BAY_SPLITS/thirds stayed on, the band stride), wall-clock time, and the
building's own W x D x H.

Read-only on the repo except for this one file (per the verification task
that asked for it) -- run with:
    GBP_NAMES=SM_Building_01,... usd_python.sh _gac_budget_probe.py
"""
import contextlib
import gc
import io
import os
import sys
import time

from pxr import Sdf, Usd, UsdGeom

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss
from detail import gac_slice as gsl

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")

NAMES = os.environ.get(
    "GBP_NAMES",
    "SM_Building_01,SM_Building_02,SM_Building_04,SM_Building_09,"
    "SM_Building_24,SM_Building_06_Small").split(",")


def one(name):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/src")
    kid = st.DefinePrim("/W/src/asset")
    kid.GetReferences().AddReference(SEI + name + ".usd")
    st.Load(Sdf.Path("/W/src"))
    UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

    # bbox for reporting, from the same estimator slice_to_kit itself uses
    wins, bbox = gsl.window_centres(st, "/W/src")
    (x0, y0, z0), (x1, y1, z1) = bbox
    W, D, H = x1 - x0, y1 - y0, z1 - z0

    buf = io.StringIO()
    t0 = time.time()
    with contextlib.redirect_stdout(buf):
        placements, g, measured = gss.slice_to_kit(
            st, "/W/src", "/W/kit", name, verbose=True)
    dt = time.time() - t0
    out = buf.getvalue()

    budget_line = next((ln for ln in out.splitlines()
                        if ln.startswith("[storey_slice] budget:")), "<none>")
    summary_line = next((ln for ln in out.splitlines()
                         if "piece(s) actual" in ln), "<none>")
    roofed_line = next((ln for ln in out.splitlines()
                        if "could not be labelled roof" in ln), "")

    by_role = {}
    for p in placements:
        by_role[p["_role"]] = by_role.get(p["_role"], 0) + 1

    print("=" * 78)
    print("ASSET: {0}".format(name))
    print("  bbox: {0:.2f} x {1:.2f} x {2:.2f} m (W x D x H)".format(W, D, H))
    print("  measured grid: {0}, storey_h={1:.3f} m, confidence={2:.2f}, "
          "raw storeys={3}".format(measured, g.get("storey_h", 0.0),
                                   g.get("confidence", 0.0),
                                   len(g.get("storeys") or [])))
    print("  " + budget_line)
    print("  " + summary_line)
    if roofed_line:
        print("  " + roofed_line)
    print("  placements total (actual, counted from the returned list): "
          "{0}".format(len(placements)))
    print("  per-role: " + "  ".join(
        "{0}={1}".format(k, v) for k, v in sorted(by_role.items())))
    print("  wall-clock: {0:.2f} s".format(dt))

    result = {
        "name": name, "W": W, "D": D, "H": H,
        "storey_h": g.get("storey_h", 0.0), "measured": measured,
        "total": len(placements), "by_role": by_role, "dt": dt,
        "budget_line": budget_line, "summary_line": summary_line,
    }
    del st, placements, g
    gc.collect()
    return result


results = []
for nm in [n.strip() for n in NAMES if n.strip()]:
    try:
        results.append(one(nm))
    except Exception as e:
        import traceback
        print("=" * 78)
        print("ASSET: {0} FAILED: {1}".format(nm, e))
        traceback.print_exc()
    gc.collect()

print("\n" + "=" * 78)
print("SUMMARY")
print("{0:22s} {1:>7s} {2:>7s} {3:>7s} {4:>8s} {5:>7s} {6:>8s}".format(
    "asset", "W", "D", "H", "storey_h", "total", "time_s"))
for r in results:
    print("{0:22s} {1:7.1f} {2:7.1f} {3:7.1f} {4:8.2f} {5:7d} {6:8.2f}".format(
        r["name"], r["W"], r["D"], r["H"], r["storey_h"], r["total"], r["dt"]))
