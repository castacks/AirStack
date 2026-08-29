#!/usr/bin/env python
"""Build ONE CitySample family in Kit and report what is really on stage.

Not a scene — a diagnosis. Reports, per placed module prim: whether it
composed a typed gprim at all, its point count, and its world bound. Then the
building's bound EXCLUDING the roof slab.

THAT EXCLUSION IS THE WHOLE POINT. The roof is authored inline at exactly the
stack height, so any check that measures the building's max Z passes whether
or not one module reached the stage — which is how a lineup of bare roof
rectangles reported "measured 78.5 m" twice.
"""
import os, sys
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})
import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom                          # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)
from detail import citysample_building as cs                   # noqa: E402

FAM = os.environ.get("CS_FAM", "CHC")
VAR = os.environ.get("CS_VAR", "A")

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
kit = cs.load_kit()
spec = cs.plan_building(kit, FAM, VAR, 70.0)
W, D = cs.footprint(kit, FAM, VAR)
r = cs.build(stage, "/World/b", kit, spec, 0.0, 0.0, W, D)
for _ in range(60):
    omni.kit.app.get_app().update()

root = stage.GetPrimAtPath("/World/b")
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
kids = [p for p in root.GetChildren() if p.GetName() != "roof"]
typed = withpts = empty_bb = 0
zmax = 0.0
for p in kids:
    t = str(p.GetTypeName() or "")
    if t:
        typed += 1
    n = 0
    for q in Usd.PrimRange(p, Usd.TraverseInstanceProxies()):
        if q.IsA(UsdGeom.Mesh):
            a = UsdGeom.Mesh(q).GetPointsAttr().Get()
            n += len(a) if a is not None else 0
    if n:
        withpts += 1
    bb = bc.ComputeWorldBound(p).ComputeAlignedRange()
    if bb.IsEmpty():
        empty_bb += 1
    else:
        zmax = max(zmax, bb.GetMax()[2])

print("\n" + "=" * 70)
print("CS PROBE  {0}_{1}   authored H {2:.1f} m   {3} module prims"
      .format(FAM, VAR, r["H"], len(kids)))
print("  composed a TYPED prim   : {0} / {1}".format(typed, len(kids)))
print("  reachable points > 0    : {0} / {1}".format(withpts, len(kids)))
print("  EMPTY world bound       : {0} / {1}".format(empty_bb, len(kids)))
print("  max Z of modules only   : {0:.2f} m   (roof excluded)".format(zmax))
for p in kids[:4]:
    st_ = p.GetPrimStack()
    refs = []
    for spec_ in st_:
        refs += [q.assetPath.rsplit("/", 1)[-1]
                 for q in list(spec_.referenceList.prependedItems)
                 + list(spec_.referenceList.explicitItems)]
    print("  e.g. {0:<26} type={1:<6} loaded={2} refs={3}"
          .format(p.GetName()[:26], str(p.GetTypeName() or "-"),
                  p.IsLoaded(), refs[:1]))
print("=" * 70 + "\n")
print("CS PROBE DONE")
simulation_app.close()
