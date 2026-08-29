#!/usr/bin/env python
"""What is GAC's UV primvar CALLED, and how is it interpolated?

`gac_slice.slice_building` reads `primvars:uv0`. If the pack uses `st` (or
anything else) the slicer writes pieces with NO UVs at all, every face then
samples one texel, and the building renders as a single flat colour — which
is exactly what the assembled view came out as.
"""
from pxr import Usd, UsdGeom

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
for name in ("SM_Building_01", "SM_Building_04", "SM_Building_24"):
    st = Usd.Stage.Open(SEI + name + ".usd")
    for prim in Usd.PrimRange(st.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        api = UsdGeom.PrimvarsAPI(prim)
        rows = []
        for pv in api.GetPrimvars():
            v = pv.Get()
            rows.append("%s[%s] interp=%s indexed=%s n=%d" % (
                pv.GetPrimvarName(), pv.GetTypeName(),
                pv.GetInterpolation(), pv.IsIndexed(),
                len(v) if v is not None else -1))
        print("%-16s %s" % (name, "; ".join(rows) or "(no primvars)"))
        break
