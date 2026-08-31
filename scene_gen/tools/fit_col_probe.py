#!/usr/bin/env python
"""fit_col_probe — do the fit-out COLUMNS of a SLICED building stand inside
its measured plan, and do they read as burnt?

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/fit_col_probe.py SM_Building_02 F5c 193 g6"

        <name> <level> <seed> [tag]

Runs the bake's own chain (`gac_fire.burn_gac` on a bare in-memory stage,
`random.Random(seed)` / `np.random.default_rng(seed)`) and then, per fitted
storey, RE-DERIVES the un-clamped `quake_flow.fit_interior` column grid — the
same `pitch` / `nx` / `ny` / `lx` / `ly` arithmetic — so the BEFORE state (how
many grid positions fall outside the storey's measured footprint) and the
AFTER state (how many prims were actually authored, and how far the worst one
sits outside) are both in one run, on one build.

Also tallies the material every authored column ends up bound to, which is
the other half of the review note: `steel` is a flat grey and reads as a
clean post in a burnt room. (fire_dtc3 review, 2026-08-30,
`gac_SM_Building_02_F5c_s193` `fit_g6/col_main_10_2_1`: columns "protrude too
far" and "don't look scorched".)
"""
import math
import random
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf   # noqa: E402
from disaster import quake_flow as qf                             # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_02"
level = sys.argv[2] if len(sys.argv) > 2 else "F5c"
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 193
tag = sys.argv[4] if len(sys.argv) > 4 else "g6"

t0 = time.time()
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(st.GetPrimAtPath("/World"))
UsdGeom.Scope.Define(st, "/World/bake")
cell = "/World/bake/" + tag
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/World/bake")
try:
    ctx = gf.burn_gac(st, cell, name, level, random.Random(seed),
                      np.random.default_rng(seed), mats, tag,
                      flow_root=None, mat_cache={}, ssf=1.0, verbose=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)

print("=" * 78)
for n in ctx["notes"]:
    if n.startswith(("gutted:", "gac:", "partial collapse fit-out")):
        print("   note:", n)

fps = (ctx["fire"].get("footprints") or {})
info = ctx["info"]
fit = ctx["fit"]
print("-" * 78)
print("A. THE COLUMN GRID vs THE MEASURED PLAN  (inset %.2f m, %d storey "
      "footprint(s) measured)" % (qf.FIT_FOOTPRINT_M, len(fps)))
tot_grid = tot_out = 0
for (mtag, i), cols in sorted((fit.get("columns") or {}).items(),
                              key=lambda kv: (kv[0][0], kv[0][1])):
    m = info["masses"][mtag]
    W, D = m["W"] - 2 * qf.WALL_INSET, m["D"] - 2 * qf.WALL_INSET
    pitch = max(4.0, float(m["module"]))
    nx = max(2, int(round(W / pitch)) + 1)
    ny = max(2, int(round(D / pitch)) + 1)
    poly = fps.get(i)
    grid = out = 0
    for a in range(nx):
        for b in range(ny):
            lx = -W / 2.0 + qf.COLUMN_W / 2.0 + a * (W - qf.COLUMN_W) / (nx - 1)
            ly = -D / 2.0 + qf.COLUMN_W / 2.0 + b * (D - qf.COLUMN_W) / (ny - 1)
            wx, wy = qf._to_world(m, lx, ly)
            grid += 1
            if poly and not qf._inside_inset(poly, wx, wy, qf.FIT_FOOTPRINT_M):
                out += 1
    tot_grid += grid
    tot_out += out
    print("   %s storey %-3d grid %3d  outside plan %3d  ->  authored %3d"
          % (mtag, i, grid, out, len(cols)))
print("   TOTAL: %d grid position(s), %d outside the measured plan (BEFORE: "
      "all %d authored, %d of them poking out), %d authored now"
      % (tot_grid, tot_out, tot_grid, tot_out,
         sum(len(c) for c in (fit.get("columns") or {}).values())))

print("-" * 78)
print("B. HOW FAR OUTSIDE — every AUTHORED column, worst first")
worst = []
for (mtag, i), cols in (fit.get("columns") or {}).items():
    poly = fps.get(i)
    if not poly:
        continue
    m = info["masses"][mtag]
    for c in cols:
        pr = st.GetPrimAtPath(c)
        if not pr or not pr.IsValid():
            continue
        xf = UsdGeom.Xformable(pr).GetLocalTransformation()
        wx, wy = float(xf[3][0]), float(xf[3][1])
        # signed clearance = how far inside the hull the centre sits
        cl = 1e9
        n_ = len(poly)
        a2 = sum(poly[k][0] * poly[(k + 1) % n_][1]
                 - poly[(k + 1) % n_][0] * poly[k][1] for k in range(n_))
        s = 1.0 if a2 >= 0 else -1.0
        for k in range(n_):
            j = (k + 1) % n_
            dx, dy = poly[j][0] - poly[k][0], poly[j][1] - poly[k][1]
            el = math.hypot(dx, dy) or 1.0
            cl = min(cl, s * (dx * (wy - poly[k][1])
                              - dy * (wx - poly[k][0])) / el)
        worst.append((cl, c))
worst.sort()
for cl, c in worst[:6]:
    print("   clearance %+.3f m inside the plan   %s" % (cl, c))
print("   %d authored column(s) measured; MIN clearance %+.3f m "
      "(negative = outside the plan)"
      % (len(worst), worst[0][0] if worst else float("nan")))

print("-" * 78)
print("C. WHAT THE COLUMNS ARE BOUND TO")
cnt = Counter()
for (mtag, i), cols in (fit.get("columns") or {}).items():
    for c in cols:
        pr = st.GetPrimAtPath(c)
        if not pr or not pr.IsValid():
            continue
        mm = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
        cnt[mm.GetPrim().GetName() if mm else "(none)"] += 1
for k, v in cnt.most_common():
    print("   %-40s %d" % (k, v))
print("WALL_S %.0f" % (time.time() - t0))
