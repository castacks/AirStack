"""The traced site's features as stand-in geometry (`detail/site_features.py`).

Needs `pxr` — it writes prims — but no Isaac and no assets: an in-memory stage
is enough, because the whole pass is "turn a measured rectangle into a box".

The two things worth defending are the ones that would be invisible in a
render: that HEIGHT comes from the spec rather than a default whenever the spec
has one (heights were read off the oblique by counting storeys, and silently
falling back to 5 m would flatten a 16 m drill tower without erroring), and
that the measured BEARING survives onto the prim.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

pytest.importorskip("pxr")
from pxr import Usd, UsdGeom, Gf                                      # noqa: E402

# A REAL `pxr`, OR NOTHING. Half a dozen modules in this tree — two test files
# and every `tools/*_png.py` — install an empty `pxr` stub at import time so
# they can pull in `scene_generator` without usd-core, and they do it with
# `setattr(sys.modules["pxr"], "Usd", <empty module>)`, which overwrites the
# REAL package when an earlier-collected file has already imported it. Nothing
# puts it back, so from that point on this file would fail with
# `module 'Usd' has no attribute 'Stage'` — which looks like a bug in the code
# under test and is not one. Skipping says what actually happened. Run this
# file on its own (or first) and it exercises everything.
if not hasattr(Usd, "Stage"):
    pytest.skip("`pxr` was replaced by another test module's stub — run this "
                "file in its own process", allow_module_level=True)

from detail import site_features as sf                                # noqa: E402
from layout import site_plan as spl                                   # noqa: E402


def _stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    return s


def test_height_prefers_the_spec_over_the_default():
    assert sf.height_of({"kind": "building", "height_m": 16.0}) == 16.0
    assert sf.height_of({"kind": "building"}) == sf.DEFAULT_H["building"]
    # An unknown kind still gets a box rather than raising: the spec's kind
    # list is open, and a feature with no geometry is worse than a guessed one.
    assert sf.height_of({"kind": "gantry"}) > 0.0


def test_build_writes_one_prism_per_feature_with_its_bearing():
    st = _stage()
    feats = [{"kind": "building", "at": [10.0, -20.0], "size_m": [8.0, 4.0],
              "yaw_deg": 30.0, "height_m": 12.0, "note": "x"}]
    out = sf.build(st, feats, "/World/f")
    assert len(out) == 1
    prim = st.GetPrimAtPath(out[0]["prim_path"])
    assert prim.IsValid()
    xf = UsdGeom.Xform(prim)
    ops = {o.GetOpName(): o for o in xf.GetOrderedXformOps()}
    assert ops["xformOp:translate"].Get() == Gf.Vec3d(10.0, -20.0, 0.0)
    assert ops["xformOp:rotateZ"].Get() == pytest.approx(30.0)
    # The measurement, readable off the prim without regenerating the scene.
    assert prim.GetCustomDataByKey("featureKind") == "building"
    assert prim.GetCustomDataByKey("standIn") is True
    assert tuple(prim.GetCustomDataByKey("featureSizeM")) == (8.0, 4.0, 12.0)


def test_box_spans_the_footprint_and_stands_on_the_ground():
    st = _stage()
    sf.build(st, [{"kind": "building", "at": [0.0, 0.0], "size_m": [8.0, 4.0],
                   "yaw_deg": 0.0, "height_m": 12.0}], "/World/f")
    mesh = UsdGeom.Mesh(st.GetPrimAtPath("/World/f/building_00/building_00_box"))
    pts = [tuple(p) for p in mesh.GetPointsAttr().Get()]
    assert min(p[0] for p in pts) == pytest.approx(-4.0)
    assert max(p[0] for p in pts) == pytest.approx(4.0)
    assert min(p[1] for p in pts) == pytest.approx(-2.0)
    assert min(p[2] for p in pts) == pytest.approx(0.0)     # on the ground
    assert max(p[2] for p in pts) == pytest.approx(12.0)


def test_an_outline_only_feature_gets_the_bounds_of_its_outline():
    """A rubble field carries a traced polygon and no `size_m`."""
    st = _stage()
    out = sf.build(st, [{"kind": "rubble", "at": [0.0, 0.0],
                         "poly": [[-5.0, -3.0], [7.0, -3.0], [7.0, 4.0]]}],
                   "/World/f")
    assert out[0]["size_m"][:2] == (12.0, 7.0)


def test_the_committed_plan_builds_every_feature():
    st = _stage()
    spec = spl.load_spec("disaster_city")
    _net, _blocks, info = spl.generate(*spl.region_of(spec), None,
                                       {"_spec": spec})
    out = sf.build(st, info["features"], "/World/f")
    assert len(out) == len(info["features"])
    # Every one carries an authored height: the fallbacks are a guess, and this
    # plan is not supposed to be relying on them.
    assert all(f.get("height_m") for f in info["features"])
    tall = [p for p in out if p["size_m"][2] >= 15.0]
    assert tall and all(p["category"] == "building" for p in tall)


def test_every_scatter_catalogue_the_plan_names_actually_loads():
    """A named catalogue must load, be non-empty, and size every piece.

    THIS EXISTS BECAUSE THE SUITE STAYED GREEN THROUGH A BROKEN CATALOGUE. The
    scatter path is only reached when a feature names one AND an asset root is
    configured, and `build` is called here without a root — so a `NameError` in
    `_load_catalogue` (a deleted module global) sailed through every test and
    was caught by a headless Isaac build instead. Calling the loader directly
    covers it without needing a stage or the network.
    """
    spec = spl.load_spec("disaster_city")
    # A feature may name one catalogue or several (list, or comma-separated).
    named = set()
    for f in (spec.get("features") or []):
        c = f.get("scatter_catalogue")
        if not c:
            continue
        named.update(c if isinstance(c, (list, tuple))
                     else [x.strip() for x in str(c).split(",") if x.strip()])
    assert named, "the plan names no catalogue; this test is watching nothing"
    for rel in named:
        pieces = sf._load_catalogue(rel)
        assert pieces, f"{rel} loaded no pieces"
        for q in pieces:
            assert q.get("url") and len(q.get("size") or ()) == 3, q
            assert max(q["size"][0], q["size"][1]) > 0.0, q


def test_a_mesh_rooted_asset_is_hosted_on_a_typeless_child():
    """`_layer_info` reports whether a layer's defaultPrim is geometry.

    Referencing a Mesh-rooted asset straight onto an Xform composes to nothing
    — no children, empty bound, no error — which is how 210 debris pieces came
    out invisible. The placement code branches on this, so it has to be right.
    """
    from pxr import Usd, UsdGeom
    import tempfile
    import os as _os
    d = tempfile.mkdtemp()
    for name, mk in (("mesh_root.usda", UsdGeom.Mesh),
                     ("xform_root.usda", UsdGeom.Xform)):
        s = Usd.Stage.CreateNew(_os.path.join(d, name))
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        prim = mk.Define(s, "/Root")
        s.SetDefaultPrim(prim.GetPrim())
        s.GetRootLayer().Save()
    st = _stage()
    assert sf._layer_info(st, _os.path.join(d, "mesh_root.usda"))[1] is True
    assert sf._layer_info(st, _os.path.join(d, "xform_root.usda"))[1] is False


def test_a_mesh_behind_a_payload_is_still_seen_as_mesh_rooted():
    """The stub-over-payload shape every FactoryDistrict building uses.

    Opened with `LoadNone` — the cheap open `_layer_info` wants — a defaultPrim
    whose geometry lives in a payload comes back with NO TYPE AT ALL, not
    `Mesh`. Believing that answer composes the mesh onto the Xform and the
    building renders as nothing: it is the invisible-debris bug wearing a
    payload. Anything that reads the type has to compose before it decides.
    """
    from pxr import Usd, UsdGeom
    import tempfile
    import os as _os
    d = tempfile.mkdtemp()
    pay = _os.path.join(d, "warehouse_payload.usda")
    s = Usd.Stage.CreateNew(pay)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    m = UsdGeom.Mesh.Define(s, "/Root")
    s.SetDefaultPrim(m.GetPrim())
    s.GetRootLayer().Save()

    stub = _os.path.join(d, "warehouse.usda")
    s = Usd.Stage.CreateNew(stub)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    # Typeless in the stub, Mesh only once the payload composes -- exactly how
    # the pack ships it.
    p = s.DefinePrim("/Root")
    p.GetPayloads().AddPayload(pay)
    s.SetDefaultPrim(p)
    s.GetRootLayer().Save()

    assert Usd.Stage.Open(stub, load=Usd.Stage.LoadNone) \
        .GetDefaultPrim().GetTypeName() == "", "fixture no longer reproduces it"
    assert sf._layer_info(_stage(), stub)[1] is True


def test_a_mesh_rooted_scatter_piece_carries_drawable_geometry(tmp_path):
    """A scattered piece must end up with real points, not an empty instance.

    TWO BUGS LIVED HERE, both of which left the prims looking healthy: a
    Mesh-rooted asset referenced onto an Xform composed to nothing, and then the
    fix for that marked the Mesh-typed host `instanceable`, which renders the
    PROTOTYPE's descendants — of which a bare Mesh has none. Both produced 210
    correctly-placed, correctly-sized, entirely invisible pieces. Asserting the
    host is not an empty instance and has points catches either.
    """
    import json
    from pxr import Usd, UsdGeom, Vt, Gf
    src = tmp_path / "chunk.usda"
    s = Usd.Stage.CreateNew(str(src))
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    m = UsdGeom.Mesh.Define(s, "/Chunk")          # a BARE MESH as defaultPrim
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 0), Gf.Vec3f(2, 0, 0),
                                      Gf.Vec3f(2, 2, 0), Gf.Vec3f(0, 2, 0)]))
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    s.SetDefaultPrim(m.GetPrim())
    s.GetRootLayer().Save()

    cat = tmp_path / "cat.json"
    cat.write_text(json.dumps({"prefix": "", "pieces": [
        {"name": "chunk", "url": "chunk.usda", "size": [2.0, 2.0, 0.2]}]}))
    sf._CATALOGUE.clear()

    st = _stage()
    out = sf.build(st, [{"kind": "rubble", "at": [0.0, 0.0],
                         "poly": [[-8, -8], [8, -8], [8, 8], [-8, 8]],
                         "asset": "chunk.usda", "scatter_target": 4,
                         "scatter_catalogue": str(cat)}],
                   "/World/f", asset_root=str(tmp_path))
    sf._CATALOGUE.clear()
    assert out and out[0].get("count", 0) > 0, "nothing was scattered"
    hosts = [p for p in st.Traverse() if p.GetName() == "geo"]
    assert hosts, "no host prims were created"
    for h in hosts:
        assert not h.IsInstanceable(), (
            f"{h.GetPath()} is an instance of a Mesh-rooted asset; its "
            f"prototype is empty and it will render nothing")
        pts = UsdGeom.Mesh(h).GetPointsAttr().Get()
        assert pts, f"{h.GetPath()} composed no points"
