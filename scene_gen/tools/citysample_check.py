"""citysample_check — offline proof that every composed CitySample family is
geometrically sound. Bare `pxr`, no SimulationApp, safe beside a running sim.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/citysample_check.py

Per family: authored against MEASURED height, the ring closing on the
rectangle the packer reserved, the base sitting on z = 0, and every module
reference resolving.

THE LAST TWO ARE THE ONES THAT CATCH REAL BUGS. With the modules failing to
open — which is what a relative reference does, since the catalogue stores
paths relative to the project root — the building's bound is its ROOF SLAB
alone, and footprint, centre and height all still agree, because the slab is
authored from the same numbers the checks compare against. The first cut of
this file passed all three while not one module was on stage.
"""

import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from pxr import Sdf, Usd, UsdGeom
from detail import citysample_building as cs
kit = cs.load_kit()
bad = 0
print("%-8s %-7s %-6s %-16s %-8s %-9s %s" % ("family","H auth","H meas","footprint","prims","unresolved","verdict"))
for fam, var in cs.families(kit):
    W, D = cs.footprint(kit, fam, var)
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, Sdf.Path("/World"))
    sp = cs.plan_building(kit, fam, var, 70.0)
    try:
        r = cs.build(st, "/World/b", kit, sp, 0.0, 0.0, W, D, yaw=180.0)
    except Exception as e:
        print("%-8s BUILD RAISED %s" % (fam+"_"+var, e)); bad += 1; continue
    # NO st.Load HERE. `cs.build` loads its own payloads now, and the first
    # version of this check called Load itself — so it verified a stage the
    # launcher never built and passed all 18 families while the real scene
    # drew nothing but roof slabs.
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bb = bc.ComputeWorldBound(st.GetPrimAtPath("/World/b")).ComputeAlignedRange()
    mn, mx = bb.GetMin(), bb.GetMax()
    nres = sum(1 for q in Usd.PrimRange(st.GetPrimAtPath("/World/b")) if q.IsA(UsdGeom.Mesh))
    unres = r["prims"] - 1 - nres
    ok = (abs((mx[0]-mn[0]) - W) < 1.0 and abs((mx[1]-mn[1]) - D) < 1.0
          and abs(mx[2] - r["H"]) < 1.0 and abs(mn[2]) < 0.8 and unres == 0
          and r["prims"] > 20)
    bad += 0 if ok else 1
    print("%-8s %-7.1f %-6.1f %-16s %-8d %-11d %s"
          % (fam+"_"+var, r["H"], mx[2], "%.1f x %.1f (%.0fx%.0f)" % (mx[0]-mn[0], mx[1]-mn[1], W, D),
             r["prims"], unres, "PASS" if ok else "FAIL"))
print("\n%d family failed" % bad if bad else "\nALL %d FAMILIES PASS" % len(cs.families(kit)))
