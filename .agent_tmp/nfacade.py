import sys, json
from pxr import Usd, UsdGeom, UsdShade, Gf

def tex_of_mat(mat_prim):
    out = []
    for p in Usd.PrimRange(mat_prim):
        if p.IsA(UsdShade.Shader):
            sh = UsdShade.Shader(p)
            for inp in sh.GetInputs():
                v = inp.Get()
                if v is not None and hasattr(v, "path") and v.path:
                    out.append(f"{inp.GetBaseName()}={v.path}")
    return out

for url in sys.argv[1:]:
    print("="*78)
    print(url.split('/')[-1], "  <", url)
    s = Usd.Stage.Open(url)
    if s is None:
        print("  FAILED"); continue
    s.Load()
    mpu = UsdGeom.GetStageMetersPerUnit(s)
    dp = s.GetDefaultPrim()
    if not dp or not dp.IsValid():
        ch = list(s.GetPseudoRoot().GetChildren()); dp = ch[0] if ch else None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    print(f"  mpu={mpu} up={UsdGeom.GetStageUpAxis(s)} defaultPrim={dp.GetPath() if dp else None}")
    if dp:
        r = cache.ComputeWorldBound(dp).ComputeAlignedRange()
        if not r.IsEmpty():
            sz = r.GetSize(); mn=r.GetMin(); mx=r.GetMax()
            print(f"  BBOX size(m)= {sz[0]*mpu:.3f} x {sz[1]*mpu:.3f} x {sz[2]*mpu:.3f}"
                  f"   min=({mn[0]*mpu:.2f},{mn[1]*mpu:.2f},{mn[2]*mpu:.2f}) max=({mx[0]*mpu:.2f},{mx[1]*mpu:.2f},{mx[2]*mpu:.2f})")
    tot = 0
    for p in s.Traverse():
        if p.IsA(UsdGeom.Mesh):
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get(); fv = m.GetFaceVertexCountsAttr().Get()
            n = len(pts) if pts else 0; tot += n
            ds = m.GetDoubleSidedAttr().Get()
            r = cache.ComputeWorldBound(p).ComputeAlignedRange()
            sz = r.GetSize() if not r.IsEmpty() else Gf.Vec3d(0,0,0)
            bound = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            mname = bound.GetPrim().GetName() if bound else "-"
            # euler check: closed solid -> V - E + F ~ 2 ; approximate with tri count vs pts
            print(f"   MESH {p.GetPath()}  pts={n} faces={len(fv) if fv else 0} doubleSided={ds} "
                  f"size(m)={sz[0]*mpu:.3f}x{sz[1]*mpu:.3f}x{sz[2]*mpu:.3f} mat={mname}")
    print(f"   TOTAL points={tot}")
    seen=set()
    for p in s.Traverse():
        if p.IsA(UsdShade.Material) and p.GetName() not in seen:
            seen.add(p.GetName())
            print(f"   MAT {p.GetName()}: {tex_of_mat(p)}")
