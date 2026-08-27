import os, sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade
from detail import urban_building as ub
NAMES = os.environ.get("G_NAMES", "").split(",")
for nm in NAMES:
    nm = nm.strip()
    if not nm:
        continue
    st = Usd.Stage.Open(ub._usd(nm))
    print("=" * 40, nm)
    if not st:
        print("  cannot open"); continue
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get(); cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(m))
        groups = [(s.GetPrim(), list(s.GetIndicesAttr().Get() or [])) for s in subs] \
            or [(p, list(range(len(cnt or []))))]
        starts, k = [], 0
        for c in cnt:
            starts.append(k); k += int(c)
        for prim, idxs in groups:
            if not idxs:
                continue
            lo = [1e9]*3; hi = [-1e9]*3
            for fi in idxs:
                c = int(cnt[fi]); s = starts[fi]
                for j in range(c):
                    q = pts[idx[s+j]]
                    for a in range(3):
                        lo[a] = min(lo[a], q[a]); hi[a] = max(hi[a], q[a])
            try:
                bm = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                mp = bm.GetPrim()
                lay = ""
                for s_ in (mp.GetPrimStack() or []):
                    if "Materials/" in getattr(s_.layer, "identifier", ""):
                        lay = s_.layer.identifier.rsplit("/", 1)[-1]; break
                op = None
                for q in Usd.PrimRange(mp):
                    sh = UsdShade.Shader(q)
                    if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
                        oi = sh.GetInput("opacity")
                        op = oi.Get() if oi else None
                        break
            except Exception as e:
                lay, op = str(e), None
            print("  {0:<12s} nf={1:5d} bbox=({2:.2f},{3:.2f},{4:.2f})-({5:.2f},{6:.2f},{7:.2f}) mat={8} opacity={9}".format(
                prim.GetName(), len(idxs), lo[0], lo[1], lo[2], hi[0], hi[1], hi[2], lay, op))
