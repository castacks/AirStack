#!/usr/bin/env python
"""Do the sliced pieces carry the SAME set of materials as the source?

A piece bound to one arbitrary material renders as a flat colour with its
geometry and UVs perfectly intact — the brownstone failure. The check is
simply: how many distinct materials does the source use, and how many appear
across the pieces?
"""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss
from detail import gac_slice as gsl

AEC = ("/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
       "Create_Brownstone02/")
SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
TARGETS = [("Reference_Brownstone02", AEC + "Reference_Brownstone02.usd"),
           ("SM_Building_01", SEI + "SM_Building_01.usd")]

for name, usd in TARGETS:
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/src")
    kid = st.DefinePrim("/W/src/asset")
    kid.GetReferences().AddReference(usd)
    st.Load(Sdf.Path("/W/src"))
    UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

    # ground truth: distinct materials actually bound anywhere in the source
    src_mats = set()
    for pr in Usd.PrimRange(st.GetPrimAtPath("/W/src"), Usd.TraverseInstanceProxies()):
        if not pr.IsA(UsdGeom.Mesh):
            continue
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr))
        if subs:
            for sb in subs:
                m = UsdShade.MaterialBindingAPI(sb.GetPrim()).ComputeBoundMaterial()[0]
                if m and m.GetPrim().IsValid():
                    src_mats.add(str(m.GetPrim().GetPath()))
        else:
            m = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
            if m and m.GetPrim().IsValid():
                src_mats.add(str(m.GetPrim().GetPath()))

    m = gss.read_mesh(st, "/W/src", verbose=False)
    used = set(int(q) for q in m["MID"])
    print("%-26s source materials %3d | reader collected %3d | used on faces %3d"
          % (name, len(src_mats), len(m["mats"]), len(used)))
    print("%-26s -> %s" % ("", "OK" if len(used) >= min(len(src_mats), 5)
                           else "COLLAPSED to %d material(s)" % len(used)))
