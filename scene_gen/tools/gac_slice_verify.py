#!/usr/bin/env python
"""Verify what `gac_slice.slice_building` actually WROTE — data, not pixels.

A render cannot distinguish "no UVs", "wrong UVs", "wrong material" or "camera
too close"; all four look like a flat brown box. This slices for real with
bare pxr and reports, per piece: primvars written, whether their counts match
the face-vertex count, and which material each subset resolves to.
"""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_slice as gsl

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAME = os.environ.get("GV_NAME", "SM_Building_01")

st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
root = UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(root.GetPrim())
src = "/W/src"
UsdGeom.Xform.Define(st, src)
kid = st.DefinePrim(src + "/asset")
kid.GetReferences().AddReference(SEI + NAME + ".usd")
st.Load(Sdf.Path(src))
UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

# --- what does the SOURCE carry? ---
for prim in Usd.PrimRange(st.GetPrimAtPath(src)):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    me = UsdGeom.Mesh(prim)
    nfv = len(me.GetFaceVertexIndicesAttr().Get() or [])
    print("SOURCE %s: %d face-verts" % (prim.GetName(), nfv))
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        v = pv.Get()
        print("   primvar %-14s %-18s interp=%-12s n=%s" % (
            pv.GetPrimvarName(), pv.GetTypeName(), pv.GetInterpolation(),
            len(v) if v is not None else None))
    break

wins, bbox = gsl.window_centres(st, src)
g = gsl.measure_grid(wins, bbox, name=NAME)
pls = gsl.slice_building(st, src, "/W/pieces", g, "gac_" + NAME)

print("\n--- SLICED PIECES ---")
bad_uv = bad_mat = 0
for k, p in enumerate(pls):
    prim = st.GetPrimAtPath(p["prim_path"])
    me = UsdGeom.Mesh(prim)
    nfv = len(me.GetFaceVertexIndicesAttr().Get() or [])
    pvs = {}
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        v = pv.Get()
        pvs[pv.GetPrimvarName()] = (len(v) if v is not None else 0,
                                    str(pv.GetInterpolation()))
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
    mats = []
    for s in subs:
        m = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
        mats.append(m.GetPrim().GetName() if m and m.GetPrim().IsValid() else "NONE")
    uv_ok = any(n.startswith(("st", "uv")) and c == nfv for n, (c, i) in pvs.items())
    if not uv_ok:
        bad_uv += 1
    if not mats or all(x == "NONE" for x in mats):
        bad_mat += 1
    if k < 4:
        print("  %-22s faceverts=%-6d primvars=%s subsets=%d mats=%s"
              % (prim.GetName(), nfv,
                 {n: c for n, (c, i) in pvs.items()}, len(subs),
                 sorted(set(mats))[:3]))
print("\n%d piece(s): %d with NO usable UV set, %d with NO material"
      % (len(pls), bad_uv, bad_mat))
# does any piece's UV span a real range, or are they all one texel?
p0 = st.GetPrimAtPath(pls[len(pls)//2]["prim_path"])
for pv in UsdGeom.PrimvarsAPI(p0).GetPrimvars():
    v = pv.Get()
    if v is None or not len(v) or "st" not in pv.GetPrimvarName():
        continue
    a = np.asarray([(q[0], q[1]) for q in v], dtype=float)
    print("mid piece %s: u %.4f..%.4f  v %.4f..%.4f  (span %.4f x %.4f)"
          % (pv.GetPrimvarName(), a[:,0].min(), a[:,0].max(),
             a[:,1].min(), a[:,1].max(),
             a[:,0].ptp(), a[:,1].ptp()))
