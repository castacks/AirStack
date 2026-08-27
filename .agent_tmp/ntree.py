import sys
from pxr import Usd, UsdGeom, UsdShade

for url in sys.argv[1:]:
    print("="*70)
    print(url)
    s = Usd.Stage.Open(url)
    if s is None:
        print("  FAILED"); continue
    s.Load()
    print("  metersPerUnit", UsdGeom.GetStageMetersPerUnit(s), "up", UsdGeom.GetStageUpAxis(s))
    dp = s.GetDefaultPrim()
    print("  defaultPrim", dp.GetPath() if dp else None)
    for p in s.Traverse():
        d = p.GetPath().pathString.count('/')
        if d > 4: continue
        extra = ""
        if p.IsA(UsdGeom.Mesh):
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get(); fv = m.GetFaceVertexCountsAttr().Get()
            extra = f" pts={len(pts) if pts else 0} faces={len(fv) if fv else 0}"
            subs = UsdGeom.Subset.GetAllGeomSubsets(m)
            if subs: extra += f" subsets={[x.GetPrim().GetName() for x in subs]}"
            b = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            if b: extra += f" mat={b.GetPrim().GetName()}"
        if p.IsA(UsdShade.Shader):
            sh = UsdShade.Shader(p)
            iid = sh.GetIdAttr().Get()
            texs = []
            for inp in sh.GetInputs():
                v = inp.Get()
                if v is not None and hasattr(v, "path") and v.path:
                    texs.append(f"{inp.GetBaseName()}={v.path}")
            extra = f" id={iid}" + (" " + ";".join(texs) if texs else "")
        if p.GetTypeName() == "Xform":
            xf = UsdGeom.Xformable(p)
            ops = [o.GetOpName() for o in xf.GetOrderedXformOps()]
            if ops: extra = f" xformOps={ops}"
        print("   " + "  "*(d-1) + f"{p.GetName()} <{p.GetTypeName()}>{extra}")
