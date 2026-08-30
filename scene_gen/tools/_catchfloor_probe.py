#!/usr/bin/env python
"""_catchfloor_probe — before/after check for the fire-collapse catch-floor
and fallen-roof-material fix. Runs `gac_fire.burn_gac` on a bare stage
(no Kit, no Flow, no physics) and reports every fit slab's bound material by
storey, plus the bound material of every loose roof/deck/lid/frag piece, so
a run against the pre-fix `.orig` snapshot and a run against the fixed file
can be diffed by material path.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/_catchfloor_probe.py SM_Building_09 F6"

Set CATCHFLOOR_SCENE_GEN to point at an alternate scene_gen root (a
symlink overlay with only disaster/urban_fire.py swapped for the `.orig`
snapshot) to probe the pre-fix behaviour without touching the live file.
"""
import os
import random
import sys
import traceback

import numpy as np

SCENE_GEN = os.environ.get("CATCHFLOOR_SCENE_GEN", "/isaac-sim/AirStack/scene_gen")
sys.dont_write_bytecode = True
sys.path.insert(0, SCENE_GEN)
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf   # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_09"
level = sys.argv[2] if len(sys.argv) > 2 else "F6"
print("[catchfloor] scene_gen root: {0}".format(SCENE_GEN))
print("[catchfloor] urban_fire module: {0}".format(uf.__file__))
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
                      flow_root=None, mat_cache={}, ssf=1.0, verbose=False)
except Exception:
    traceback.print_exc()
    sys.exit(1)


def matname(path):
    pr = st.GetPrimAtPath(path) if path else None
    if not pr or not pr.IsValid():
        return "<invalid>"
    m = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
    return str(m.GetPrim().GetPath()) if m and m.GetPrim().IsValid() else "<none>"


print("\n[catchfloor] fit slabs (mass, storey): material")
slab_counts = {}
for (mtag, storey), slab in sorted((ctx["fit"].get("slabs") or {}).items(),
                                   key=lambda kv: (kv[0][0], kv[0][1])):
    if not slab:
        continue
    mn = matname(slab)
    slab_counts[mn] = slab_counts.get(mn, 0) + 1
    print("   ({0}, {1}): {2}".format(mtag, storey, mn))
print("[catchfloor] SLAB MATERIAL COUNTS:", slab_counts)

prefixes = ("roof", "deck", "roofslab", "lidbrk", "frag")
print("\n[catchfloor] loose roof/deck/lid/frag prims: material")
roof_counts = {}
for p in ctx["loose"]:
    leaf = p.rsplit("/", 1)[-1]
    if not leaf.startswith(prefixes):
        continue
    mn = matname(p)
    roof_counts[mn] = roof_counts.get(mn, 0) + 1
print("[catchfloor] LOOSE ROOF-PIECE MATERIAL COUNTS:", roof_counts)
print("[catchfloor] collapse_storeys:", ctx.get("collapse_storeys"))
print("[catchfloor] fire roof flag:", ctx["fire"].get("roof"),
      "origin:", ctx["fire"].get("origin"), "top:", ctx["fire"].get("top"))
