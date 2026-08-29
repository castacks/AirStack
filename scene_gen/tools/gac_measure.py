#!/usr/bin/env python
"""Measure the GreatAmericanCity buildings into `_plans/gac_buildings.json`.

CENTRE AND BASE, NOT JUST SIZE. An asset's own origin is not its bounding-box
centre and not necessarily on its floor, so placing one by translating to
(x, y, 0) puts it wherever the exporter happened to leave the pivot. Carrying
`cx, cy, z0` lets a caller land the building's FOOTPRINT on the slot it was
given and its base on the ground.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_measure.py
"""
import json, os, time
from pxr import Sdf, Usd, UsdGeom, UsdShade

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "_plans", "gac_buildings.json")
NAMES = (["SM_Building_%02d" % i for i in range(1, 6)] +
         ["SM_Building_06_Small"] +
         ["SM_Building_%02d" % i for i in range(7, 32)])
out = []
for nm in NAMES:
    t0 = time.time()
    st = Usd.Stage.Open(ROOT + nm + ".usd"); st.Load()
    S = UsdGeom.GetStageMetersPerUnit(st)
    meshes = [p for p in st.Traverse() if p.IsA(UsdGeom.Mesh)]
    npts = ntri = nsub = 0
    texs = set()
    for m in meshes:
        me = UsdGeom.Mesh(m)
        pts = me.GetPointsAttr().Get()
        npts += len(pts) if pts is not None else 0
        cs = me.GetFaceVertexCountsAttr().Get()
        if cs is not None:
            ntri += sum(max(0, int(c) - 2) for c in cs)
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(m))
        nsub += len(subs)
        for tgt in ([s.GetPrim() for s in subs] or [m]):
            mat = UsdShade.MaterialBindingAPI(tgt).ComputeBoundMaterial()[0]
            if not mat or not mat.GetPrim().IsValid():
                continue
            for c in Usd.PrimRange(mat.GetPrim()):
                sh = UsdShade.Shader(c)
                if sh and sh.GetIdAttr().Get() == "UsdUVTexture":
                    f = sh.GetInput("file"); v = f.Get() if f else None
                    if isinstance(v, Sdf.AssetPath) and v.path:
                        texs.add(v.path.rsplit("/", 1)[-1])
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    a, b = r.GetMin(), r.GetMax()
    rec = {"name": nm, "mpu": S, "meshes": len(meshes), "subsets": nsub,
           "points": npts, "tris": ntri, "textures": len(texs),
           "W": round((b[0]-a[0])*S, 1), "D": round((b[1]-a[1])*S, 1),
           "H": round((b[2]-a[2])*S, 1),
           # in METRES, already through the asset's own mpu
           "cx": round(0.5*(a[0]+b[0])*S, 3), "cy": round(0.5*(a[1]+b[1])*S, 3),
           "z0": round(a[2]*S, 3), "sec": round(time.time()-t0, 1)}
    out.append(rec)
    print("%-22s %6.1f x %5.1f x %6.1f m   centre (%8.2f,%8.2f)  base z %7.2f"
          % (nm, rec["W"], rec["D"], rec["H"], rec["cx"], rec["cy"], rec["z0"]),
          flush=True)
json.dump(out, open(os.path.normpath(OUT), "w"), indent=1)
print("\nwrote %s" % os.path.normpath(OUT))
