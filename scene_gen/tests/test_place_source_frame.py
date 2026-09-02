#!/usr/bin/env python3
"""test_place_source_frame.py — `gac_fire.place_source` must centre the
merged asset on its cell FOR ANY HOLDER YAW.

    uv run --with usd-core --with pytest python -m pytest -q scene_gen/tests/test_place_source_frame.py

THE DEFECT THIS PINS (2026-09-02, the tornado city's "frame changes when we
reassemble it").  `place_source` used to subtract a WORLD bbox centre from
the cell's world translation and write the delta as the asset's LOCAL
translate.  Right under an unrotated cell (every bake launcher, every probe,
the bench's yaw-0 holders); wrong under a yawed holder, by the rotated
delta.  GreatAmericanCity pivots sit at a CORNER (`_plans/gac_buildings.
json`: cx up to -43 m, cy up to +70 m), so a yaw-90 SM_Building_30-sized
asset landed 109 m from its holder — an empty lot where its intact stood
(already hidden) and a building on a road.

Bare usd-core, no Kit: one corner-pivot centimetre box the size of
SM_Building_30 (6030 x 14220 x 3000 cm, scale 0.01), one holder per yaw.
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

from disaster import gac_fire as gcf                               # noqa: E402


def _corner_box(path, sx, sy, sz):
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(root.GetPrim())
    m = UsdGeom.Mesh.Define(st, "/World/box")
    pts = [(0, 0, 0), (sx, 0, 0), (sx, sy, 0), (0, sy, 0),
           (0, 0, sz), (sx, 0, sz), (sx, sy, sz), (0, sy, sz)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4,
                                   1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
    m.CreateExtentAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(sx, sy, sz)])
    st.GetRootLayer().Save()
    return path


def _holder(stage, x, y, z, yaw, ssf=1.0):
    """The tornado city launcher's `place_holder`: translate, rotateXYZ(0,
    0, yaw), scale ssf, then a `cell` child Xform."""
    h = UsdGeom.Xform.Define(stage, Sdf.Path("/World/tornado/h"))
    h.ClearXformOpOrder()
    h.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, z * ssf))
    h.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw))
    h.AddScaleOp().Set(Gf.Vec3f(ssf, ssf, ssf))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/tornado/h/cell"))
    return "/World/tornado/h/cell"


def _bbox(stage, path):
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = c.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


@pytest.mark.parametrize("yaw", [0.0, 90.0, 180.0, 270.0, 37.0])
def test_source_is_centred_on_the_cell_at_any_yaw(tmp_path, yaw):
    asset = _corner_box(str(tmp_path / "SM_Building_30_like.usd"),
                        6030.0, 14220.0, 3000.0)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    cell = _holder(stage, 100.0, 50.0, 0.0, yaw)
    src = gcf.place_source(stage, cell, asset, 0.01)
    assert src == cell + "/src"
    mn, mx = _bbox(stage, src)
    cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
    assert abs(cx - 100.0) < 1e-3 and abs(cy - 50.0) < 1e-3, (yaw, cx, cy)
    assert abs(mn[2]) < 1e-6                     # base on the cell's ground
    # the asset is the right size (scaled to metres) and yawed with the holder
    w, d = mx[0] - mn[0], mx[1] - mn[1]
    if yaw in (0.0, 180.0):
        assert abs(w - 60.3) < 1e-3 and abs(d - 142.2) < 1e-3
    elif yaw in (90.0, 270.0):
        assert abs(w - 142.2) < 1e-3 and abs(d - 60.3) < 1e-3


def test_source_is_centred_when_the_cell_carries_a_stage_scale(tmp_path):
    """A stage whose metersPerUnit != 1 hands the holder `ssf` on its scale
    op; the centring must still land on the holder in stage units."""
    asset = _corner_box(str(tmp_path / "small.usd"), 20.0, 30.0, 12.0)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    cell = _holder(stage, 10.0, -20.0, 0.0, 90.0, ssf=100.0)
    src = gcf.place_source(stage, cell, asset, 1.0)
    mn, mx = _bbox(stage, src)
    assert abs(0.5 * (mn[0] + mx[0]) - 1000.0) < 1e-3
    assert abs(0.5 * (mn[1] + mx[1]) + 2000.0) < 1e-3
    assert abs(mn[2]) < 1e-6


def test_world_delta_idiom_is_gone():
    """Source-level: the centring must be measured relative to the cell,
    never as a world bound minus the cell's world translation."""
    import inspect
    src = inspect.getsource(gcf.place_source)
    assert "ComputeRelativeBound" in src
    assert "GetLocalToWorldTransform" not in src
