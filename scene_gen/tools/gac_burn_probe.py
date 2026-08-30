#!/usr/bin/env python
"""gac_burn_probe — `disaster.gac_fire.burn_gac` end to end on a bare USD
stage (no Kit, no Flow, no physics): place, measure, plan, bake, slice,
rebind, run the whole fire ladder on the pieces. Prints the notes and the
counts, so a launch is not the first time the chain runs.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/gac_burn_probe.py SM_Building_02 F3"
"""
import random
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Sdf, Usd, UsdGeom                            # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_02"
level = sys.argv[2] if len(sys.argv) > 2 else "F3"
t0 = time.time()
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/W/bench")
rng = random.Random(7)
nrng = np.random.default_rng(7)
try:
    ctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, "g0",
                      flow_root=None, mat_cache={}, ssf=1.0, verbose=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)
for n in ctx["notes"]:
    print("   note:", n[:220])
g = ctx["gac"]
print("[burn_probe] {0} {1}: {2} piece(s), {3} atlas(es), {4} rebound, {5} glass; "
      "loose {6}, static {7}, authored {8}; {9:.0f}s".format(
          name, level, g["n_pieces"], g["n_atlases"], g["n_rebound"], g["n_glass"],
          len(ctx["loose"]), len(ctx["static_extra"]), len(ctx["authored"]),
          time.time() - t0))
# sanity: how many piece subsets ended up on a sooted material vs the source
n_soot = n_src = 0
from pxr import UsdShade
for p in Usd.PrimRange(st.GetPrimAtPath(cell)):
    if not p.IsA(UsdGeom.Mesh):
        continue
    for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(p)):
        m = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
        path = str(m.GetPrim().GetPath()) if m else ""
        if "/SootLooks/" in path:
            n_soot += 1
        elif "/src/" in path:
            n_src += 1
print("[burn_probe] piece subsets: {0} on sooted atlases, {1} still on the source's own materials".format(n_soot, n_src))
