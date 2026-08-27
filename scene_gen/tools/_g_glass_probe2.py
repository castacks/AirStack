#!/usr/bin/env python
"""_g_glass_probe2.py — per-SUBSET bboxes for the curtain-wall modules, plus
a copy of the window BaseColor texture out to the host log dir so the painted
mullion pitch can be measured instead of guessed. Agent G, round 3."""
import os

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "ModernCityEnvironment01/")
OUT = os.environ.get("G_OUT", "/isaac-sim/.nvidia-omniverse/logs/_g_tex")
NAMES = ["SM_MBuilding05_SkyscraperFacade_B", "SM_MBuilding05_SkyscraperCorner_B",
         "SM_MBuilding05_SkyscraperRoofTopFacade",
         "SM_MBuilding05_FirstFloor_A", "SM_MBuilding05_FirstFloor_B",
         "SM_MBuilding05_FirstFloor_C"]


def sub_bbox(pts, counts, indices, idxs):
    lo = [1e9] * 3
    hi = [-1e9] * 3
    starts, k = [], 0
    for c in counts:
        starts.append(k)
        k += int(c)
    for fi in idxs:
        c = int(counts[fi])
        s = starts[fi]
        for j in range(c):
            p = pts[indices[s + j]]
            for a in range(3):
                lo[a] = min(lo[a], p[a]); hi[a] = max(hi[a], p[a])
    return lo, hi


def main():
    from pxr import Usd, UsdGeom, UsdShade
    try:
        os.makedirs(OUT)
    except OSError:
        pass
    for nm in NAMES:
        st = Usd.Stage.Open(ROOT + "Meshes/" + nm + ".usd")
        print("=" * 70)
        if not st:
            print(nm, "CANNOT OPEN"); continue
        print(nm)
        for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get()
            counts = m.GetFaceVertexCountsAttr().Get()
            indices = m.GetFaceVertexIndicesAttr().Get()
            if pts is None:
                continue
            for s in UsdGeom.Subset.GetAllGeomSubsets(m):
                idxs = list(s.GetIndicesAttr().Get() or [])
                lo, hi = sub_bbox(pts, counts, indices, idxs)
                try:
                    bmm = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                    mp = bmm.GetPrim()
                    texs = []
                    for q in Usd.PrimRange(mp):
                        sh = UsdShade.Shader(q)
                        if sh and sh.GetIdAttr().Get() == "UsdUVTexture":
                            f = sh.GetInput("file")
                            v = f.Get() if f else None
                            if v is not None and "BaseColor" in (v.path or ""):
                                texs.append(str(v.path).rsplit("/", 1)[-1])
                except Exception:
                    texs = ["?"]
                print("   {0:<12s} nf={1:4d} bbox=({2:.2f},{3:.2f},{4:.2f})-({5:.2f},{6:.2f},{7:.2f}) {8}".format(
                    s.GetPrim().GetName(), len(idxs), lo[0], lo[1], lo[2],
                    hi[0], hi[1], hi[2], ",".join(texs)))
    # copy the two textures out
    import omni.client
    for t in ("MI_MBuilding05_SkyscraperWindows_BaseColor.png",
              "M_MBuilding05_FirstFloor_A_BaseColor.png",
              "M_MBuilding05_FirstFloor_B_BaseColor.png"):
        url = ROOT + "Materials/Game_ModernCityEnvironment01_Materials_" + t
        res, content = omni.client.read_file(url)
        print("read", t, res)
        if str(res) == "Result.OK" or "OK" in str(res):
            with open(os.path.join(OUT, t), "wb") as fh:
                fh.write(memoryview(content).tobytes())
            print("  -> wrote", os.path.join(OUT, t))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
