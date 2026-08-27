#!/usr/bin/env python
"""_g_glass_probe.py — what IS a curtain-wall kit module, geometrically?

Round 3, agent G. Before authoring a pane grid over a module I need to know
whether the mullions are modelled or painted: if they are geometry I can hide
one pane's glass and keep the cage for free; if they are texture I must draw
the cage myself wherever I blank a pane.

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_g_glass_probe.py
"""
import os

NAMES = os.environ.get("G_NAMES", ",".join([
    "SM_MBuilding05_SkyscraperFacade_B",
    "SM_MBuilding05_SkyscraperFacade_A",
    "SM_MBuilding05_SkyscraperCorner_B",
    "SM_MBuilding05_FirstFloor_A",
    "SM_MBuilding05_FirstFloor_B",
    "SM_MBuilding05_Facade_A",
])).split(",")
ROOT = os.environ.get(
    "G_ROOT",
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "ModernCityEnvironment01/Meshes/")


def main():
    from pxr import Usd, UsdGeom, UsdShade
    for nm in NAMES:
        url = ROOT + nm + ".usd"
        st = Usd.Stage.Open(url)
        print("=" * 70)
        if not st:
            print(nm, "-> CANNOT OPEN")
            continue
        mpu = UsdGeom.GetStageMetersPerUnit(st)
        print(nm, "metersPerUnit", mpu)
        for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get()
            cnts = m.GetFaceVertexCountsAttr().Get()
            if pts is None:
                continue
            xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
            sub = list(UsdGeom.Subset.GetAllGeomSubsets(m)) if hasattr(UsdGeom, "Subset") else []
            try:
                bm = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
                mat = bm.GetPath().pathString if bm else "-"
            except Exception:
                mat = "?"
            print("  mesh {0:<38s} pts={1:5d} faces={2:5d} bbox=({3:.2f},{4:.2f},{5:.2f})-"
                  "({6:.2f},{7:.2f},{8:.2f}) subsets={9} mat={10}".format(
                      p.GetName()[:38], len(pts), len(cnts or []),
                      min(xs), min(ys), min(zs), max(xs), max(ys), max(zs),
                      len(sub), mat.rsplit("/", 1)[-1]))
            for s in sub:
                try:
                    idx = s.GetIndicesAttr().Get()
                    bmm = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                    print("      subset {0:<24s} faces={1:4d} mat={2}".format(
                        s.GetPrim().GetName()[:24], len(idx or []),
                        bmm.GetPath().name if bmm else "-"))
                except Exception as exc:
                    print("      subset ?", exc)
        # textures
        for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            sh = UsdShade.Shader(p)
            if not sh:
                continue
            fid = sh.GetIdAttr().Get()
            if fid != "UsdUVTexture":
                continue
            f = sh.GetInput("file")
            v = f.Get() if f else None
            if v is not None:
                print("  tex", p.GetName(), str(v.path).rsplit("/", 1)[-1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
