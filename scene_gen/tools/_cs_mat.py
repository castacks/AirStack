"""What material does a CitySample module carry, and does it resolve?"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from pxr import Sdf, Usd, UsdGeom, UsdShade
import omni.client
from detail import citysample_building as cs
kit = cs.load_kit()
mods = [m for m in kit["CHC"]["A"]["1"] if m["kind"] in ("Wall", "CornerEx")][:2]
mods += [m for m in kit["NYG"]["A"]["1"] if m["kind"] == "Wall"][:1]
for m in mods:
    url = cs.ASSET_ROOT + m["usd"]
    st = Usd.Stage.Open(url); st.Load()
    print("\n=== %s" % m["usd"].rsplit("/", 1)[-1])
    print("   default prim: %s" % (st.GetDefaultPrim().GetName()
                                   if st.GetDefaultPrim() else "NONE"))
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        im = UsdGeom.Imageable(p)
        print("   mesh %-26s vis=%s purpose=%s"
              % (p.GetName()[:26], im.ComputeVisibility(),
                 im.GetPurposeAttr().Get()))
        binding = UsdShade.MaterialBindingAPI(p)
        mat = binding.ComputeBoundMaterial()[0]
        subs = UsdGeom.Subset.GetAllGeomSubsets(im)
        tgts = [s.GetPrim() for s in subs] or []
        if not mat and not tgts:
            print("      NO MATERIAL BOUND")
        for t in ([p] if mat else tgts):
            mm = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            if not mm:
                print("      %-18s no material" % t.GetName()[:18]); continue
            for c in Usd.PrimRange(mm.GetPrim()):
                sh = UsdShade.Shader(c)
                if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
                    continue
                vals = {}
                for k in ("opacity", "opacityThreshold", "diffuseColor",
                          "metallic", "roughness"):
                    i = sh.GetInput(k)
                    if i is None:
                        continue
                    vals[k] = ("<conn>" if i.HasConnectedSource()
                               else i.Get())
                print("      %-18s %s" % (t.GetName()[:18], vals))
                d = sh.GetInput("diffuseColor")
                if d is not None and d.HasConnectedSource():
                    ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
                    f = ts.GetInput("file"); v = f.Get() if f else None
                    if isinstance(v, Sdf.AssetPath):
                        rp = v.resolvedPath or ""
                        print("         tex %s  resolved=%s"
                              % (v.path.rsplit("/", 1)[-1], bool(rp)))
