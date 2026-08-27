import sys
from collections import Counter
from pxr import Usd, UsdGeom, UsdShade, Gf

def analyse(url, verbose=True):
    s = Usd.Stage.Open(url)
    if s is None:
        print(f"{url.split('/')[-1]}: FAILED"); return
    s.Load()
    mpu = UsdGeom.GetStageMetersPerUnit(s)
    dp = s.GetDefaultPrim()
    if not dp or not dp.IsValid():
        ch = list(s.GetPseudoRoot().GetChildren()); dp = ch[0] if ch else None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = cache.ComputeWorldBound(dp).ComputeAlignedRange() if dp else None
    sz = r.GetSize() if (r and not r.IsEmpty()) else Gf.Vec3d(0,0,0)
    mn = r.GetMin() if (r and not r.IsEmpty()) else Gf.Vec3d(0,0,0)
    tot_p = tot_f = 0; nmesh = 0; open_edges = 0; nonman = 0
    for p in s.Traverse():
        if not p.IsA(UsdGeom.Mesh): continue
        nmesh += 1
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get() or []
        fvc = m.GetFaceVertexCountsAttr().Get() or []
        fvi = m.GetFaceVertexIndicesAttr().Get() or []
        tot_p += len(pts); tot_f += len(fvc)
        # weld points by position so duplicated UV-seam verts don't fake open edges
        key = {}; remap = []
        for pt in pts:
            k = (round(pt[0],4), round(pt[1],4), round(pt[2],4))
            if k not in key: key[k] = len(key)
            remap.append(key[k])
        ec = Counter(); i = 0
        for c in fvc:
            loop = fvi[i:i+c]; i += c
            for j in range(c):
                a = remap[loop[j]]; b = remap[loop[(j+1) % c]]
                ec[(min(a,b), max(a,b))] += 1
        open_edges += sum(1 for v in ec.values() if v == 1)
        nonman += sum(1 for v in ec.values() if v > 2)
    closed = "CLOSED-SOLID" if open_edges == 0 else f"OPEN({open_edges} boundary edges)"
    mats = []
    for p in s.Traverse():
        if p.IsA(UsdShade.Material):
            texs = set()
            for q in Usd.PrimRange(p):
                if q.IsA(UsdShade.Shader):
                    sh = UsdShade.Shader(q)
                    for inp in sh.GetInputs():
                        try:
                            v = inp.Get()
                        except Exception:
                            continue
                        if v is not None and hasattr(v, "path") and v.path:
                            t = v.path.split('/')[-1]
                            t = t.replace('Game_ModernCityEnvironment01_Materials_','')
                            for sfx in ('_BaseColor.png','_Normal.png','_Roughness.png','_Metallic.png','_AmbientOcclusion.png','_Specular.png'):
                                t = t.replace(sfx,'')
                            texs.add(t)
            mats.append(f"{p.GetName()}[{','.join(sorted(texs))}]")
    print(f"{url.split('/')[-1]:52s} {sz[0]*mpu:7.3f} x {sz[1]*mpu:6.3f} x {sz[2]*mpu:7.3f} m | "
          f"meshes={nmesh} pts={tot_p} faces={tot_f} | {closed} | mpu={mpu} | origin=({mn[0]*mpu:.2f},{mn[1]*mpu:.2f},{mn[2]*mpu:.2f}) | {';'.join(mats)}")

for u in sys.argv[1:]:
    analyse(u)
