#!/usr/bin/env python
"""One-shot sweep: for all 31 GreatAmericanCity buildings, measure the
storey/bay grid from window centres (`gac_slice.window_centres` +
`measure_grid`), then the window clearance of a 0.5-storey cut
(`gac_storey_slice.cut_lines` + `window_clearance`), and if that cut clips
glass, search offsets 0.30..0.70 for the one with the lowest hit count.

Read-only report script, not part of the build pipeline.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_grid_sweep.py
"""
import statistics
import sys

from pxr import Sdf, Usd, UsdGeom

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_slice as gsl
from detail import gac_storey_slice as gss

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAMES = (["SM_Building_%02d" % i for i in range(1, 6)] +
         ["SM_Building_06_Small"] +
         ["SM_Building_%02d" % i for i in range(7, 32)])
OFFSETS = [0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70]


def measure_one(name):
    url = SEI + name + ".usd"
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/src")
    kid = st.DefinePrim("/W/src/asset")
    kid.GetReferences().AddReference(url)
    st.Load(Sdf.Path("/W/src"))
    UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

    wins, bbox = gsl.window_centres(st, "/W/src")
    g = gsl.measure_grid(wins, bbox, verbose=False, name=name)

    row = {"name": name, "ok": True, "g": g}
    measured = g.get("confidence", 0.0) >= gsl.MIN_CONFIDENCE and "storeys" in g
    row["measured"] = measured
    if not measured:
        return row

    lines5 = gss.cut_lines(g, 0.5, verbose=False)
    clear5, hit5 = gss.window_clearance(lines5, wins)
    row["clear5"], row["hit5"] = clear5, hit5

    best_off, best_hit = 0.5, hit5
    if hit5 and hit5 > 0:
        for off in OFFSETS:
            lines = gss.cut_lines(g, off, verbose=False)
            _c, h = gss.window_clearance(lines, wins)
            if h < best_hit:
                best_hit, best_off = h, off
    row["best_off"], row["best_hit"] = best_off, best_hit
    return row


rows = []
for name in NAMES:
    try:
        rows.append(measure_one(name))
    except Exception as exc:
        rows.append({"name": name, "ok": False,
                      "err": "%s: %s" % (type(exc).__name__, exc)})

# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------
cols = ("%-22s %-15s %-9s %8s %9s %10s %9s %7s %11s %9s" %
        ("name", "WxDxH (m)", "measured?", "storey_h", "n_storeys",
         "confidence", "clear@.5", "hit@.5", "best_off", "hit@best"))
print(cols)
print("-" * len(cols))

n_meas = 0
storey_hs = []
n_hit_pos = 0
n_fixed = 0

for row in rows:
    name = row["name"]
    if not row.get("ok"):
        print("%-22s FAILED: %s" % (name, row.get("err")))
        continue
    g = row["g"]
    if not row["measured"]:
        W, D, H = g.get("W"), g.get("D"), g.get("H")
        dims = "%.1fx%.1fx%.1f" % (W, D, H) if W is not None else "-"
        why = g.get("why", "")
        conf = g.get("confidence", 0.0)
        print("%-22s %-15s %-9s %8s %9s %10.2f %9s %7s %11s %9s  (%s)" % (
            name, dims, "NO", "-", "-", conf, "-", "-", "-", "-", why))
        continue

    n_meas += 1
    storey_hs.append(g["storey_h"])
    dims = "%.1fx%.1fx%.1f" % (g["W"], g["D"], g["H"])
    hit5 = row["hit5"]
    clear5 = "%.2f" % row["clear5"] if row["clear5"] is not None else "-"
    if hit5 and hit5 > 0:
        n_hit_pos += 1
        if row["best_hit"] == 0:
            n_fixed += 1
    print("%-22s %-15s %-9s %8.2f %9d %10.2f %9s %7d %11.2f %9d" % (
        name, dims, "YES", g["storey_h"], len(g["storeys"]), g["confidence"],
        clear5, hit5, row["best_off"], row["best_hit"]))

print()
print("MEASURED: %d / %d" % (n_meas, len(NAMES)))
print("median storey height (measured only): %s" % (
    "%.2f m" % statistics.median(storey_hs) if storey_hs else "n/a"))
print("hit@0.5 > 0: %d ; of those fixed to hit=0 by a per-asset offset: %d" % (
    n_hit_pos, n_fixed))
