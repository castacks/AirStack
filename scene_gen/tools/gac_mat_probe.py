#!/usr/bin/env python
"""Where does a GAC material actually LIVE? Decides how to re-home it.

A sliced piece has to carry its own material or it is not a kit — it is a
fragment that only renders while the building it was cut out of is still on
the stage. (Deactivating the source is exactly what turned every sliced
building white.) If each material is its own USD file, re-homing is a
reference; if it is inline in the mesh layer, it has to be copied.
"""
from pxr import Usd, UsdGeom, UsdShade

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
st = Usd.Stage.Open(SEI + "SM_Building_01.usd")
seen = set()
for prim in Usd.PrimRange(st.GetPseudoRoot()):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
        m = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
        if not m or not m.GetPrim().IsValid():
            continue
        p = m.GetPrim()
        if str(p.GetPath()) in seen:
            continue
        seen.add(str(p.GetPath()))
        print("material prim: %s" % p.GetPath())
        for spec in p.GetPrimStack():
            print("    spec in layer: %s" % spec.layer.identifier)
        if len(seen) >= 4:
            raise SystemExit(0)
