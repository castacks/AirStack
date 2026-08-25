"""The Isaac-side boilerplate every damage-lab tool needs, in one place.

Referencing one building onto a fresh stage and exporting the result again is
three lines and four traps, and each trap cost a full grid run to find. They
are documented at the call site below rather than in each tool.
"""

from __future__ import annotations

import os


def fresh_stage(asset_path, scale):
    """A new CONTEXT stage with *asset_path* referenced under /World/Building.

    The context stage, not `Usd.Stage.CreateNew`: `settle` drives
    `SimulationContext`, which attaches PhysX to whatever stage the USD context
    holds, so a standalone stage settles an empty scene in silence.
    """
    import omni.kit.app
    import omni.usd

    from pxr import Gf, Sdf, UsdGeom

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    # TYPELESS, not `Xform.Define`. Some of these assets are a single Mesh at
    # their default prim; referencing one onto a prim already declared an Xform
    # makes the local type win, so the geometry composes as an Xform with
    # GeomSubset children and `mesh_prims` finds nothing — every cell then
    # reports 0 cells in 0.0 s and exports the pristine asset. A typeless prim
    # lets the reference supply the type.
    b = stage.DefinePrim(Sdf.Path("/World/Building"))
    b.GetReferences().AddReference(str(asset_path))
    UsdGeom.Xformable(b).AddScaleOp().Set(Gf.Vec3f(*([float(scale)] * 3)))
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    # PUMP THE APP. A reference is composed by Kit's own update loop, not by
    # `AddReference`; without this the prim has no meshes yet, so `mesh_prims`
    # comes back empty and every cell silently exports the pristine asset.
    for _ in range(4):
        omni.kit.app.get_app().update()
    return stage


def export(stage, out_dir, name):
    """Write *stage* to ``out_dir/name.usd``. Returns the path.

    ROOT LAYER, not `stage.Export`: exporting flattens, which composes the
    referenced building and dies on attributes Kit's crate reader cannot repack
    ("unpack unsupported type enum value 0").
    """
    out = os.path.join(out_dir, str(name) + ".usd")
    stage.GetRootLayer().Export(out)
    return out
