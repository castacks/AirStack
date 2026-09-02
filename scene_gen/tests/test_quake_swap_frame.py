#!/usr/bin/env python3
"""test_quake_swap_frame.py — a swapped-in earthquake bake must compose in
ITS OWN frame (metres, origin-centred, yaw only), not the intact cell's.

    uv run --with usd-core python -m pytest -q scene_gen/tests/test_quake_swap_frame.py

THE DEFECT THIS PINS (eq500_v5_local, 2026-09-02).  `quake.assemble` used
to swap a placed cell's reference and keep every transform op
`apply_placements` had authored for the INTACT asset: the pack scale (0.01
for centimetre-authored GreatAmericanCity), the roll/pitch of a Y-up
original, and the bbox-centroid correction.  The bakes are authored in
metres, centred on the origin, base at z=0 (`gac_quake/*.usd`,
`archetype/bld_*_DG*.usd` — measured with usd-core), so a GAC bake came out
at 1/100 size and a same_art twin stood a half-footprint off its lot.  The
user saw empty lots ringed by full-size rubble and buildings on the street.

Runs under bare usd-core (no Kit): `scene_generator.apply_placements` and
`quake._swap_reference` are pure pxr.  Two tiny assets are written to
`tmp_path`: a centimetre-authored box whose pivot is at a CORNER (the
worst-case GAC shape: both a scale and a centroid offset to inherit) and a
metre-authored, origin-centred "bake".
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

pxr = pytest.importorskip("pxr")
from pxr import Gf, Sdf, Usd, UsdGeom                              # noqa: E402

import scene_generator as sg                                       # noqa: E402
from disaster import quake as q                                    # noqa: E402


def _box_asset(path, sx, sy, sz, origin_at_corner, up="Z"):
    """A single-mesh box asset: `sx x sy x sz` in its own units, pivot at
    the bbox centre (base at 0) or at the min corner."""
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z if up == "Z" else UsdGeom.Tokens.y)
    root = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(st, "/World/box")
    if origin_at_corner:
        x0, y0, z0 = 0.0, 0.0, 0.0
    else:
        x0, y0, z0 = -sx / 2.0, -sy / 2.0, 0.0
    x1, y1, z1 = x0 + sx, y0 + sy, z0 + sz
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4,
                                      1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
    mesh.CreateExtentAttr([Gf.Vec3f(x0, y0, z0), Gf.Vec3f(x1, y1, z1)])
    st.GetRootLayer().Save()
    return path


def _world_bbox(stage, path):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


@pytest.fixture
def city(tmp_path):
    """One GAC-like intact placement (cm box, corner pivot, scale 0.01) at
    (100, 50) yaw 90, plus a metre bake of a different size."""
    intact = _box_asset(str(tmp_path / "SM_Building_99.usd"), 3000.0, 1400.0,
                        3880.0, origin_at_corner=True)          # 30 x 14 x 38.8 m
    bake = _box_asset(str(tmp_path / "gac_SM_Building_99_DG3_s1.usd"), 34.0,
                      18.0, 40.0, origin_at_corner=False)       # spilled a little
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    p = {"usd": intact, "x_m": 100.0, "y_m": 50.0, "z_m": 0.0, "yaw_deg": 90.0,
         "scale": 0.01, "category": "house", "axis_up": "Z"}
    # the resolver is what makes apply_placements centroid-correct a cell
    # (generate_scene_on_stage always passes one); without it the corner
    # pivot would land ON (100, 50) and the trap would not be reproduced
    sg.apply_placements(stage, [p], "/World/stage/generated", 1.0,
                        resolver=sg.SizeResolver(1.0, {}, True))
    return stage, p, bake


def test_intact_cell_lands_at_its_visual_centre_with_the_pack_scale(city):
    stage, p, _bake = city
    mn, mx = _world_bbox(stage, p["prim_path"])
    # yaw 90 swaps the 30 x 14 footprint; centroid-corrected to (100, 50)
    assert abs((mn[0] + mx[0]) / 2 - 100.0) < 1e-3
    assert abs((mn[1] + mx[1]) / 2 - 50.0) < 1e-3
    assert abs((mx[0] - mn[0]) - 14.0) < 1e-3 and abs((mx[1] - mn[1]) - 30.0) < 1e-3
    assert abs(mn[2]) < 1e-6 and abs(mx[2] - 38.8) < 1e-3


def test_naive_swap_reproduces_the_defect(city):
    """The old idiom: keep the ops, swap the reference -> 1/100 size, off
    its centre. Kept as the regression's negative control."""
    stage, p, bake = city
    prim = stage.GetPrimAtPath(p["prim_path"])
    refs = prim.GetReferences()
    refs.ClearReferences()
    refs.AddReference(bake)
    prim.Load()
    mn, mx = _world_bbox(stage, p["prim_path"])
    assert (mx[2] - mn[2]) < 1.0                       # a 40 m bake at 0.4 m
    assert abs((mn[0] + mx[0]) / 2 - 100.0) > 5.0      # and not on its lot


def test_swap_reference_puts_the_bake_on_the_lot_in_metres(city):
    stage, p, bake = city
    prim = stage.GetPrimAtPath(p["prim_path"])
    x, y, yaw = q._swap_reference(stage, prim, p, bake, 1.0)
    assert (x, y, yaw) == (100.0, 50.0, 90.0)
    mn, mx = _world_bbox(stage, p["prim_path"])
    # metre-sized, yawed 90 (34 x 18 -> 18 x 34), centred on the cell, base at 0
    assert abs((mx[0] - mn[0]) - 18.0) < 1e-3
    assert abs((mx[1] - mn[1]) - 34.0) < 1e-3
    assert abs((mx[2] - mn[2]) - 40.0) < 1e-3
    assert abs((mn[0] + mx[0]) / 2 - 100.0) < 1e-3
    assert abs((mn[1] + mx[1]) / 2 - 50.0) < 1e-3
    assert abs(mn[2]) < 1e-6
    # the ops are the bake's frame, not the intact asset's
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op.Get() for op in xf.GetOrderedXformOps()}
    assert tuple(ops["xformOp:scale"]) == (1.0, 1.0, 1.0)
    assert abs(ops["xformOp:rotateXYZ"][2] - 90.0) < 1e-6
    assert abs(ops["xformOp:rotateXYZ"][0]) < 1e-6 and abs(ops["xformOp:rotateXYZ"][1]) < 1e-6


def test_swap_reference_drops_a_y_up_originals_roll(tmp_path):
    """A Y-up same_art original gets roll=+90 from apply_placements; the
    Z-up metre twin must not inherit it."""
    intact = _box_asset(str(tmp_path / "yup_original.usdc"), 20.0, 30.0, 12.0,
                        origin_at_corner=True, up="Y")
    twin = _box_asset(str(tmp_path / "bld_twin_DG2.usd"), 22.0, 12.0, 30.0,
                      origin_at_corner=False)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    p = {"usd": intact, "x_m": -40.0, "y_m": 12.0, "z_m": 0.0, "yaw_deg": 0.0,
         "scale": 1.0, "category": "house", "axis_up": "Y", "roll_deg": 90.0}
    sg.apply_placements(stage, [p], "/World/stage/generated", 1.0,
                        resolver=sg.SizeResolver(1.0, {}, True))
    prim = stage.GetPrimAtPath(p["prim_path"])
    q._swap_reference(stage, prim, p, twin, 1.0)
    mn, mx = _world_bbox(stage, p["prim_path"])
    assert abs((mx[2] - mn[2]) - 30.0) < 1e-3          # 30 m TALL, not 30 m deep
    assert abs((mn[0] + mx[0]) / 2 + 40.0) < 1e-3
    assert abs((mn[1] + mx[1]) / 2 - 12.0) < 1e-3
    assert abs(mn[2]) < 1e-6


def test_assemble_calls_swap_reference_at_every_swap_site():
    """Source-level: no bare `ClearReferences` + `AddReference` pair is left
    in `assemble` (the mono ruin swap in `_mono_pass` is a separate pass
    with its own pool scale and is out of this test's scope)."""
    import ast
    import inspect
    src = inspect.getsource(q.assemble)
    tree = ast.parse(src)
    swaps = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == "_swap_reference"]
    bare = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == "AddReference"]
    assert len(swaps) >= 4, len(swaps)
    assert not bare, "assemble still swaps a reference without re-framing the cell"
