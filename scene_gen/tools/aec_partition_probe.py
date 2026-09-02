"""aec_partition_probe -- run `gac_fire.burn_gac` end to end (bare USD, no
Kit) for an AEC brownstone and directly measure every `fit_interior`
PARTITION (`part_*`) against the shell's own bbox, storey by storey, using
the REAL `m["levels"]`/`m["top"]` the ladder actually ran with (read straight
off `ctx["info"]["masses"]`) rather than a fresh, separate probe run.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_partition_probe.py \
      Reference_Brownstone5Row F3 1"
"""
import random
import sys

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                  # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "Reference_Brownstone5Row"
level = sys.argv[2] if len(sys.argv) > 2 else "F3"
origin = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 7

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
rng = random.Random(seed)
nrng = np.random.default_rng(seed)

ctx = gf.burn_gac(st, cell, "aec:" + name, level, rng, nrng, mats, "g0",
                  flow_root=None, mat_cache={}, ssf=1.0, origin=origin,
                  verbose=True)
for n in ctx["notes"]:
    print("   note:", n[:220])

fit = ctx.get("fit") or {}
info = ctx["info"]
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

# shell bbox off the sliced pieces themselves
shell_lo = np.full(3, np.inf)
shell_hi = np.full(3, -np.inf)
for p in Usd.PrimRange(st.GetPrimAtPath(cell + "/pieces")):
    if not p.IsA(UsdGeom.Mesh):
        continue
    r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    mn, mx = r.GetMin(), r.GetMax()
    shell_lo = np.minimum(shell_lo, [mn[0], mn[1], mn[2]])
    shell_hi = np.maximum(shell_hi, [mx[0], mx[1], mx[2]])
print("[probe] shell (pieces/*) bbox: lo={0} hi={1}".format(shell_lo, shell_hi))

m = info["masses"]["main"]
print("[probe] mass 'main': z0={0:.3f} top={1:.3f} levels={2}".format(
    m["z0"], m["top"], ["{0:.3f}".format(z) for z in m["levels"]]))

print("[probe] partitions:", len(fit.get("partitions") or []))
for p in (fit.get("partitions") or []):
    prim = st.GetPrimAtPath(p)
    if not prim or not prim.IsValid() or not prim.IsActive():
        print("  {0}  INACTIVE/missing".format(p))
        continue
    r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        print("  {0}  EMPTY bbox".format(p))
        continue
    mn, mx = r.GetMin(), r.GetMax()
    over_hi = float(mx[2]) - float(shell_hi[2])
    flag = "  <<< OVER ROOF" if over_hi > 0.15 else ""
    print("  {0}  z=[{1:.3f},{2:.3f}]  over_roof={3:+.3f}{4}".format(
        p, mn[2], mx[2], over_hi, flag))

print("[probe] slabs:", len(fit.get("slabs") or {}))
for (mtag, storey), slab in (fit.get("slabs") or {}).items():
    prim = st.GetPrimAtPath(slab) if slab else None
    if not prim or not prim.IsValid() or not prim.IsActive():
        continue
    r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    mn, mx = r.GetMin(), r.GetMax()
    over_hi = float(mx[2]) - float(shell_hi[2])
    flag = "  <<< OVER ROOF" if over_hi > 0.15 else ""
    print("  storey={0}  {1}  z=[{2:.3f},{3:.3f}]  over_roof={4:+.3f}{5}".format(
        storey, slab, mn[2], mx[2], over_hi, flag))
