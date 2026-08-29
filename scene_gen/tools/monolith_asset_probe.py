#!/usr/bin/env python3
"""Bare-pxr census for candidate standalone monoliths (no SimulationApp)."""

from pxr import Usd, UsdGeom, UsdShade


ASSETS = [
    ("tower_01", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/tower/tower_01_0006.usd"),
    ("tower_03", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/tower/tower_03_0015.usd"),
    ("midrise_02", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_02_0059a.usd"),
    ("midrise_04", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_04_0082.usd"),
    ("midrise_05", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_05_0097a.usd"),
]


def main():
    for name, url in ASSETS:
        stage = Usd.Stage.Open(url)
        if stage is None:
            print(name, "OPEN_FAILED", url)
            continue
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
        bound = cache.ComputeWorldBound(root).ComputeAlignedRange()
        dims = bound.GetSize() if not bound.IsEmpty() else (0, 0, 0)
        meshes = []
        mats = set()
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                pts = mesh.GetPointsAttr().Get() or []
                counts = mesh.GetFaceVertexCountsAttr().Get() or []
                pvars = [(p.GetPrimvarName(), str(p.GetInterpolation()))
                         for p in UsdGeom.PrimvarsAPI(prim).GetPrimvars()]
                meshes.append((str(prim.GetPath()), len(pts), len(counts), pvars))
                mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                if mat:
                    mats.add(str(mat.GetPath()))
        print("ASSET", name, "dims", tuple(round(float(v), 3) for v in dims),
              "meshes", len(meshes), "materials", len(mats))
        for row in meshes:
            print(" MESH", row)
        for mat in sorted(mats):
            print(" MAT", mat)


if __name__ == "__main__":
    main()
