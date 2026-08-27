import os, sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade
from detail import urban_building as ub
for nm in ("SM_MBuilding05_SkyscraperFacade_A", "SM_MBuilding05_SkyscraperFacade_B",
           "SM_MBuilding05_SkyscraperCorner_B"):
    st = Usd.Stage.Open(ub._usd(nm))
    print("=" * 50, nm)
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get(); idx = m.GetFaceVertexIndicesAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        for pv in UsdGeom.PrimvarsAPI(p).GetPrimvars():
            print("  primvar", pv.GetName(), pv.GetInterpolation(),
                  len(pv.Get() or []), "indexed" if pv.IsIndexed() else "")
        pv = UsdGeom.PrimvarsAPI(p).GetPrimvar("st") or UsdGeom.PrimvarsAPI(p).GetPrimvar("primvars:st")
        if not pv:
            for q in UsdGeom.PrimvarsAPI(p).GetPrimvars():
                if q.GetTypeName().role == "TextureCoordinate":
                    pv = q; break
        if not pv:
            continue
        vals = pv.Get(); ind = pv.GetIndices() if pv.IsIndexed() else None
        interp = pv.GetInterpolation()
        k = 0
        for fi, c in enumerate(cnt):
            c = int(c)
            row = []
            for j in range(c):
                vi = idx[k + j]
                if interp == "faceVarying":
                    t = ind[k + j] if ind is not None else (k + j)
                else:
                    t = ind[vi] if ind is not None else vi
                pt = pts[vi]; st_ = vals[t]
                row.append(("(%.1f,%.1f,%.1f)" % tuple(pt), "(%.3f,%.3f)" % tuple(st_)))
            print("   f%d" % fi, row)
            k += c
    # shader uv transform?
    for p in Usd.PrimRange(st.GetPseudoRoot()):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        i = sh.GetIdAttr().Get()
        if i in ("UsdTransform2d", "UsdPrimvarReader_float2"):
            print("  shader", p.GetName(), i, [(x.GetBaseName(), x.Get()) for x in sh.GetInputs()])
