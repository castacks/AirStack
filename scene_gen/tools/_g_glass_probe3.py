#!/usr/bin/env python
"""_g_glass_probe3.py — exact faces of the curtain-wall modules + the window
texture pulled to the host so the painted mullion pitch can be measured."""
import os, sys
REPO = "/isaac-sim/AirStack"
sys.path.insert(0, os.path.join(REPO, "scene_gen"))
OUT = os.environ.get("G_OUT", "/isaac-sim/.nvidia-omniverse/logs/_g_tex")
NAMES = ["SM_MBuilding05_SkyscraperFacade_B", "SM_MBuilding05_SkyscraperCorner_B",
         "SM_MBuilding05_SkyscraperRoofTopFacade", "SM_MBuilding04_FirstFloor_B"]


def main():
    from pxr import Usd, UsdGeom, UsdShade
    from detail import urban_building as ub
    try:
        os.makedirs(OUT)
    except OSError:
        pass
    for nm in NAMES:
        st = Usd.Stage.Open(ub._usd(nm))
        print("=" * 60, nm)
        if not st:
            continue
        for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get(); counts = m.GetFaceVertexCountsAttr().Get()
            idx = m.GetFaceVertexIndicesAttr().Get()
            uv = None
            for pv in UsdGeom.PrimvarsAPI(p).GetPrimvars():
                if pv.GetTypeName().role == "TextureCoordinate" or "st" in pv.GetName():
                    uv = pv.Get()
                    break
            if pts is None or len(counts or []) > 40:
                print("  (mesh %s: %d faces, skipping dump)" % (p.GetName(), len(counts or [])))
                continue
            subs = {}
            for s in UsdGeom.Subset.GetAllGeomSubsets(m):
                for i in (s.GetIndicesAttr().Get() or []):
                    subs[int(i)] = s.GetPrim().GetName()
            k = 0
            for fi, c in enumerate(counts):
                c = int(c)
                vs = [tuple(round(float(q), 2) for q in pts[idx[k + j]]) for j in range(c)]
                uvs = []
                if uv is not None and len(uv) > k + c - 1:
                    uvs = [tuple(round(float(q), 3) for q in uv[k + j]) for j in range(c)]
                print("  f{0:<3d} {1:<10s} {2}  uv {3}".format(fi, subs.get(fi, "-"), vs, uvs))
                k += c
    import omni.client
    for t in ("MI_MBuilding05_SkyscraperWindows_BaseColor.png",):
        url = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
               "ModernCityEnvironment01/Materials/"
               "Game_ModernCityEnvironment01_Materials_" + t)
        r = omni.client.read_file(url)
        print("read_file ->", [str(x)[:40] for x in r])
        content = r[-1]
        with open(os.path.join(OUT, t), "wb") as fh:
            fh.write(memoryview(content).tobytes())
        print("wrote", os.path.join(OUT, t), os.path.getsize(os.path.join(OUT, t)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
