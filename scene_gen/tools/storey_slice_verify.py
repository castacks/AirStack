#!/usr/bin/env python
"""Prove the storey slice is CLEAN: flat cuts, UVs interpolated, materials kept.

Three things a render cannot tell apart and this can:
  1. are the cut faces FLAT? every band's z-extent must equal its plane pair,
     and the band must actually carry vertices sitting ON those planes. A
     centroid-binned cut fails this by up to a triangle (the sawtooth).
  2. did the UVs survive the cut, with new vertices interpolated rather than
     snapped to a corner?
  3. did every material present in the source survive into the bands?
"""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss
from detail import gac_slice as gsl

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAME = os.environ.get("GV_NAME", "SM_Building_01")

st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/src")
kid = st.DefinePrim("/W/src/asset")
kid.GetReferences().AddReference(SEI + NAME + ".usd")
st.Load(Sdf.Path("/W/src"))
UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

m = gss.read_mesh(st, "/W/src")
wins, bbox = gsl.window_centres(st, "/W/src")
g = gsl.measure_grid(wins, bbox, name=NAME)
floors = g["storeys"]
src_mats = set(int(q) for q in m["MID"])
src_uv = m["UV"]
print("source: %d tri(s), z %.2f..%.2f, materials %s, uv range u %.3f..%.3f"
      % (len(m["tris"]), m["P"][:, 2].min(), m["P"][:, 2].max(),
         sorted(src_mats), src_uv[:, 0].min(), src_uv[:, 0].max()))

bands = gss.storeys(m, floors)
print("\n%-14s %-8s %-8s %-9s %-7s %s" % ("band", "z_lo", "z_hi", "tris",
                                          "on-plane", "materials"))
worst = 0.0
seen_mats = set()
for lo, hi, b in bands:
    z = b["P"][:, 2]
    # every vertex must lie inside [lo, hi]; the overshoot is the sawtooth
    over = max(0.0, float(z.max() - hi)), max(0.0, float(lo - z.min()))
    worst = max(worst, over[0], over[1])
    on_lo = int(np.sum(np.abs(z - lo) < 1e-4))
    on_hi = int(np.sum(np.abs(z - hi) < 1e-4))
    mids = sorted(set(int(q) for q in b["MID"]))
    seen_mats |= set(mids)
    print("%-14s %-8.2f %-8.2f %-9d %-7s %s"
          % ("%.1f-%.1f" % (lo, hi), z.min(), z.max(), len(b["tris"]),
             "%d/%d" % (on_lo, on_hi), mids))

print("\nWORST overshoot past a cut plane: %.6f m  -> %s"
      % (worst, "FLAT CUTS" if worst < 1e-3 else "RAGGED (triangles crossing)"))
print("materials: %d in source, %d survived into bands, missing %s"
      % (len(src_mats), len(seen_mats), sorted(src_mats - seen_mats) or "none"))
# UV sanity on a mid band
_lo, _hi, mid = bands[len(bands) // 2]
print("mid band uv: u %.3f..%.3f  v %.3f..%.3f  (%d unique uv)"
      % (mid["UV"][:, 0].min(), mid["UV"][:, 0].max(),
         mid["UV"][:, 1].min(), mid["UV"][:, 1].max(),
         len(np.unique(mid["UV"], axis=0))))
