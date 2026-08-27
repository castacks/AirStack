#!/usr/bin/env python
"""_t_usd_check.py — the USD half of agent T's round-3 work, on a bare pxr.

`solidify` is pure numpy and is tested on the host; `fracture.face_subset` is
the only new code that touches USD, and getting `UsdGeom.Subset.CreateGeomSubset`
wrong costs a whole bench run to find out. Runs in ~2 s with no Kit:

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_t_usd_check.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))          # scene_gen on the path


def main():
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Vt, Gf
    from disaster import fracture

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    # a unit box, as triangles, so face normals span all six directions
    import trimesh
    b = trimesh.creation.box(extents=[1, 1, 1])
    m = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/box"))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in b.vertices]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(b.faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(x) for x in np.asarray(b.faces).ravel()]))
    mat = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/core"))
    sub = fracture.face_subset(stage, "/World/box", (0.0, 0.0, 1.0), cos=0.30)
    assert sub is not None and sub.IsValid(), "face_subset returned nothing"
    n_sub = len(UsdGeom.Subset(sub).GetIndicesAttr().Get())
    UsdShade.MaterialBindingAPI(sub).Bind(mat)
    bound = UsdShade.MaterialBindingAPI(sub).ComputeBoundMaterial()[0]
    print("face_subset: {0} of {1} faces in the core subset; bound {2}".format(
        n_sub, len(b.faces), bound.GetPath() if bound else None))
    assert n_sub == 10, n_sub                      # 12 tris, 2 face +Z
    assert bound and bound.GetPath() == mat.GetPath()

    # solidify through prim_to_mesh, on a real open shell
    q = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/panel"))
    q.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 0), Gf.Vec3f(4, 0, 0),
                                      Gf.Vec3f(4, 0, 3), Gf.Vec3f(0, 0, 3)]))
    q.CreateFaceVertexCountsAttr(Vt.IntArray([3, 3]))
    q.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 0, 2, 3]))
    mesh = fracture.prim_to_mesh(stage, "/World/panel")
    sol = fracture.solidify(mesh, 0.38, ref=(2.0, 5.0, 1.5), verbose=True)
    print("panel: {0} -> {1} faces, watertight {2}, volume {3:.3f} "
          "(expect 4*3*0.38 = 4.560)".format(
              len(mesh.faces), len(sol.faces), sol.is_watertight, sol.volume))
    assert abs(sol.volume - 4.56) < 1e-3

    made = fracture.fracture_prim(stage, "/World/panel", "/World/brk", 6,
                                  np.random.default_rng(3), consume=0.0,
                                  solid_m=0.38, solid_ref=(2.0, 5.0, 1.5),
                                  verbose=True)
    print("fracture_prim with solid_m: {0} fragments".format(len(made)))
    assert len(made) >= 3
    n = sum(1 for p in made
            if fracture.face_subset(stage, p, (0.0, -1.0, 0.0)) is not None)
    print("  {0}/{1} fragments got a core subset".format(n, len(made)))

    # quake_flow's tables import and answer
    from disaster import quake_flow as qf
    for bt in ("urm", "rc", "rc_glass"):
        print("  {0}: wall {1} parapet {2} roof {3}".format(
            bt, qf._t_thickness(bt, "wall"), qf._t_thickness(bt, "parapet"),
            qf._t_thickness(bt, "roof")))
    assert qf._t_thickness("urm", "wall", style="brownstone_row") == qf.T_BROWNSTONE_M
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
