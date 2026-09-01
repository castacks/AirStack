"""test_hurricane_asphalt.py — offline, bare-pxr proof for two hurricane
fixes to `suburb_scene.apply_ground` (STREAM ASP):

  JOB 1 — every asphalt-labelled ground surface on the hurricane suburb
  presets (`suburb_hurricane_500_l2`/`l3`) resolves to the damaged-wet pack
  (`Wet_Destroyed_Asphalt.usda`). Before the fix, `roads.road_tile_share`
  defaulted to 0.5 and mixed in `asphalt_road_tile` — the Muyang
  ModularNeighborhood `Road_01_Inst.usd`, a dry, undamaged, Nucleus-only
  asset — onto roughly half of every road network. The fix is a two-line
  preset override (`road_tile_share: 0.0`), plus reading that share BEFORE
  `suburb_scene.apply_ground` loads the material at all, so the Nucleus
  asset is never even referenced on a hurricane stage.

  JOB 2 — each road segment gets ONE image stretched across its own extent
  (`roads.single_stretch: true`), instead of the ~3.6 m tiling that
  `Wet_Destroyed_Asphalt.usda`'s `texture_scale=(0.28, 0.28)` +
  `project_uvw=True` (world-space triplanar) produces regardless of the
  mesh's own UVs. The fix authors `st` normalised 0..1 across each road
  ribbon's own length/width (`_make_ribbon(..., stretch=True)`) and binds
  those ribbons to a DEDICATED material instance (`_stretch_variant`,
  material prim `.../ground/materials/asphalt_stretch`) with `project_uvw`
  forced off and `texture_scale` reset to `(1, 1)` — a second reference to
  the identical `Wet_Destroyed_Asphalt.usda` asset, so only roads change;
  the shared `asphalt` prim (crosswalks, cul-de-sac bulbs, park courts, row-
  home drives) keeps its original world-space tiling untouched.

  Both knobs default to today's behaviour (`road_tile_share: 0.5`,
  `single_stretch: False`) and are set ONLY in the two hurricane presets, so
  `suburb_tornado_1000_l2` (never touching either key) is replayed here too,
  asserting NO change: the Nucleus tile mix is still there, no
  `asphalt_stretch` material is ever created, and the base `asphalt`
  material's own shader inputs are exactly what `Road_Asphalt.usda`
  authors.

No Isaac Sim — `suburb_scene.generate_suburb_on_stage` runs on a real
in-memory USD stage built by bare `pxr`, the same pattern
`tools/hurricane_layout_png.py`'s `Replay` class uses.

    cd scene_gen && python3 -m pytest tests/test_hurricane_asphalt.py -q
"""
import math
import os
import sys

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

import pytest  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, SCENE_GEN_DIR)

from pxr import Usd, UsdGeom, UsdShade  # noqa: E402

import compile_disaster as cd  # noqa: E402
import suburb_scene as ss  # noqa: E402

PARENT = "/World/stage/generated"
WET_ASPHALT_TAIL = "megascans/Wet_Destroyed_Asphalt.usda"


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _build(preset):
    config = cd.load_scene_config(preset)
    stage = Usd.Stage.CreateInMemory()
    binfo = {}
    ss.generate_suburb_on_stage(
        stage, config, parent_path=PARENT, scene_scale_factor=1.0,
        info_out=binfo, assembly=True)
    return stage, binfo, config


def _mat_ref_url(mat_prim):
    """First composition-arc reference asset path authored on a material
    (or shader) prim, or None if it has no reference (e.g. an unresolved
    Nucleus reference still reports its authored path here)."""
    for spec in mat_prim.GetPrimStack():
        for item in spec.referenceList.prependedItems:
            return str(item.assetPath)
    return None


def _bound_mat_path(prim):
    rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel()
    targets = rel.GetTargets() if rel else []
    return str(targets[0]) if targets else None


def _road_meshes(stage):
    gnd = stage.GetPrimAtPath(PARENT + "/ground")
    assert gnd.IsValid(), "no /ground prim on stage"
    return [p for p in gnd.GetChildren() if p.GetName().startswith("road_")]


def _left_edge_cumlen(prim):
    """Cumulative arc length along the ribbon's LEFT column of vertices
    (verts alternate left, right — see `_make_ribbon`), computed purely from
    the authored points. An approximation of the true centreline cumulative
    length at a mitred bend, good enough to distinguish "tiled every few
    metres" from "stretched 0..1 across the whole thing"."""
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    left = [pts[2 * i] for i in range(len(pts) // 2)]
    cum = [0.0]
    for i in range(1, len(left)):
        dx = left[i][0] - left[i - 1][0]
        dy = left[i][1] - left[i - 1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum


# ---------------------------------------------------------------------------
# fixtures — build each preset once per test session
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def hurricane_l3():
    return _build("suburb_hurricane_500_l3")


@pytest.fixture(scope="module")
def tornado_l2():
    return _build("suburb_tornado_1000_l2")


# ---------------------------------------------------------------------------
# JOB 1 (a) — every asphalt path on the hurricane preset is the wet pack
# ---------------------------------------------------------------------------

def test_hurricane_road_tile_material_never_bound(hurricane_l3):
    """`road_tile_share: 0.0` must stop the Nucleus ModularNeighborhood tile
    from ever being BOUND to a road. The prim itself is still defined (same
    shape as every other preset's stage) so that `tools/hurricane_layout_
    png.py`'s `road_pack_color` probe -- which looks it up by a hard-coded
    path -- keeps hitting its existing, already-WARN-only `UNHANDLED_SHADER`
    branch instead of a new "material prim not found" FAIL; see
    `suburb_scene.apply_ground`'s comment above `asphalt_alt = _load_mat(...)`
    for why skipping the load broke that gate."""
    stage, _, _ = hurricane_l3
    prim = stage.GetPrimAtPath(PARENT + "/ground/materials/asphalt_road_tile")
    assert prim.IsValid(), "asphalt_road_tile prim should still be defined"
    bound_to_it = [p for p in Usd.PrimRange(stage.GetPrimAtPath(PARENT + "/ground"))
                   if p.IsA(UsdGeom.Mesh) and _bound_mat_path(p) == str(prim.GetPath())]
    assert bound_to_it == [], f"nothing should bind to it: {bound_to_it}"


def test_hurricane_every_road_segment_is_wet_destroyed(hurricane_l3):
    stage, _, _ = hurricane_l3
    roads = _road_meshes(stage)
    assert len(roads) >= 10, "sanity: expected a real road count for this seed"
    for prim in roads:
        mat_path = _bound_mat_path(prim)
        assert mat_path, f"{prim.GetPath()} has no bound material"
        url = _mat_ref_url(stage.GetPrimAtPath(mat_path))
        assert url and url.endswith(WET_ASPHALT_TAIL), (
            f"{prim.GetPath()} -> {mat_path} -> {url}, expected the wet-destroyed pack")


def test_hurricane_every_asphalt_labelled_surface_is_wet_destroyed(hurricane_l3):
    """Beyond the road ribbons: crosswalks, cul-de-sac bulbs, park courts and
    row-home drives all ride the shared `asphalt` prim, and `driveway_
    asphalt` covers the poured-asphalt driveways. All three material KEYS
    must point at the wet pack. (`path`/`pool_apron` are deliberately
    different, non-asphalt surfaces — a sidewalk and a pool-coping stone —
    and are out of this scope; asserted separately below to record that
    exemption rather than silently skip it.)"""
    stage, _, _ = hurricane_l3
    gnd = stage.GetPrimAtPath(PARENT + "/ground")
    seen_keys = set()
    for prim in Usd.PrimRange(gnd):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mat_path = _bound_mat_path(prim)
        if not mat_path:
            continue
        mat_prim = stage.GetPrimAtPath(mat_path)
        key = mat_prim.GetName()
        if key not in ("asphalt", "asphalt_stretch", "driveway_asphalt"):
            continue
        seen_keys.add(key)
        url = _mat_ref_url(mat_prim)
        assert url and url.endswith(WET_ASPHALT_TAIL), f"{prim.GetPath()} ({key}) -> {url}"
    assert "asphalt_stretch" in seen_keys, "expected at least the road ribbons on the stretch variant"


def test_hurricane_non_asphalt_ground_surfaces_exempt(hurricane_l3):
    """Record, rather than silently ignore, the two ground materials that
    stay non-asphalt on purpose: `path` (sidewalks/footways — Concrete_02)
    and `pool_apron` (Worn_Pavement, the pool-coping surround)."""
    stage, _, _ = hurricane_l3
    mat_scope = stage.GetPrimAtPath(PARENT + "/ground/materials")
    path_url = _mat_ref_url(mat_scope.GetChild("path"))
    assert path_url and "Concrete_02" in path_url
    pool_prim = mat_scope.GetChild("pool_apron")
    if pool_prim.IsValid():
        pool_url = _mat_ref_url(pool_prim)
        assert pool_url and "Worn_Pavement" in pool_url


# ---------------------------------------------------------------------------
# JOB 2 (b)(c) — one stretched instance per road segment
# ---------------------------------------------------------------------------

def test_hurricane_road_st_normalised_0_1(hurricane_l3):
    stage, _, _ = hurricane_l3
    roads = _road_meshes(stage)
    assert roads
    for prim in roads:
        st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st").Get()
        assert st is not None and len(st) >= 4
        us = [v[0] for v in st]
        vs = [v[1] for v in st]
        assert min(us) == pytest.approx(0.0, abs=1e-6)
        assert max(us) == pytest.approx(1.0, abs=1e-6)
        assert min(vs) == pytest.approx(0.0, abs=1e-6)
        assert max(vs) == pytest.approx(1.0, abs=1e-6)
        # monotonically non-decreasing along the ribbon (left column, every
        # other vertex) -- one pass along the road, not folded/repeated.
        left_v = vs[0::2]
        assert all(b >= a - 1e-9 for a, b in zip(left_v, left_v[1:]))


def test_hurricane_road_material_project_uvw_off(hurricane_l3):
    stage, _, _ = hurricane_l3
    sh = UsdShade.Shader(stage.GetPrimAtPath(
        PARENT + "/ground/materials/asphalt_stretch/Shader"))
    assert sh.GetPrim().IsValid(), "asphalt_stretch/Shader must exist"
    assert sh.GetInput("project_uvw").Get() is False
    scale = sh.GetInput("texture_scale").Get()
    assert scale[0] == pytest.approx(1.0) and scale[1] == pytest.approx(1.0)


def test_hurricane_shared_asphalt_material_untouched_by_stretch(hurricane_l3):
    """The `_stretch_variant` override must land on the SEPARATE
    `asphalt_stretch` prim only -- the shared `asphalt` prim (crosswalks,
    bulbs, park courts, row-home drives) keeps its original world-space
    tiling, so those surfaces are unaffected."""
    stage, _, _ = hurricane_l3
    sh = UsdShade.Shader(stage.GetPrimAtPath(PARENT + "/ground/materials/asphalt/Shader"))
    assert sh.GetInput("project_uvw").Get() is True
    scale = sh.GetInput("texture_scale").Get()
    assert scale[0] == pytest.approx(0.28, abs=1e-6)
    assert scale[1] == pytest.approx(0.28, abs=1e-6)


# ---------------------------------------------------------------------------
# (d) Road_Line markings are never touched by any of this
# ---------------------------------------------------------------------------

def test_hurricane_road_line_materials_never_referenced(hurricane_l3):
    """This suburb generator never references the `Road_Line_*` .usda
    materials at all -- dashes/crosswalk bars/stop bars are vertex-coloured
    (`displayColor`), not textured -- so the wet-asphalt swap has nothing to
    touch here. Assert that stays true: no `Road_Line` reference anywhere on
    the stage, and every dash/crosswalk-bar/stop-bar prim still carries no
    bound material and keeps an authored displayColor."""
    stage, _, _ = hurricane_l3
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Material":
            continue
        url = _mat_ref_url(prim)
        assert not (url and "Road_Line" in url), f"{prim.GetPath()} -> {url}"
    gnd = stage.GetPrimAtPath(PARENT + "/ground")
    # `xwalk_mask_*` is the asphalt band the crossing is cut into (correctly
    # material-bound -- see the "3) Crossings" comment in `apply_ground`),
    # not a painted line; only `dash_*`, `stopbar_*` and the painted
    # `xwalk_<node>_<edge>` bars themselves are the vertex-coloured markings.
    def _is_marking(name):
        if name.startswith("xwalk_mask_"):
            return False
        return name.startswith(("dash_", "xwalk_", "stopbar_"))
    marking_prims = [p for p in gnd.GetChildren() if _is_marking(p.GetName())]
    assert marking_prims
    for prim in marking_prims:
        assert _bound_mat_path(prim) is None, f"{prim.GetPath()} unexpectedly bound"
        dc = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        assert dc is not None and len(dc) >= 1


# ---------------------------------------------------------------------------
# non-regression — suburb_tornado_1000_l2 never sets either knob
# ---------------------------------------------------------------------------

def test_tornado_road_tile_mix_unchanged(tornado_l2):
    """`road_tile_share` still defaults to 0.5 and the Nucleus tile is still
    referenced and bound to roughly half the roads -- the hurricane-only
    `road_tile_share: 0.0` override did not leak into a preset that never
    sets it."""
    stage, _, _ = tornado_l2
    prim = stage.GetPrimAtPath(PARENT + "/ground/materials/asphalt_road_tile")
    assert prim.IsValid()
    url = _mat_ref_url(prim)
    assert url and "Road_01_Inst" in url
    roads = _road_meshes(stage)
    alt_bound = sum(1 for p in roads if _bound_mat_path(p) == str(prim.GetPath()))
    assert alt_bound > 0, "expected some roads still on the Nucleus tile"
    frac = alt_bound / len(roads)
    assert 0.3 < frac < 0.7, f"road-tile share drifted far from ~0.5: {frac}"


def test_tornado_no_stretch_material_ever_created(tornado_l2):
    stage, _, _ = tornado_l2
    prim = stage.GetPrimAtPath(PARENT + "/ground/materials/asphalt_stretch")
    assert not prim.IsValid(), "single_stretch defaults False for tornado"


def test_tornado_asphalt_material_shader_unchanged(tornado_l2):
    """The base `asphalt` material (Road_Asphalt.usda for tornado) keeps its
    own authored `project_uvw`/`texture_scale` -- proof the hurricane
    stretch-variant machinery never writes to the shared prim."""
    stage, _, _ = tornado_l2
    sh = UsdShade.Shader(stage.GetPrimAtPath(PARENT + "/ground/materials/asphalt/Shader"))
    assert sh.GetInput("project_uvw").Get() is True
    scale = sh.GetInput("texture_scale").Get()
    assert scale[0] == pytest.approx(0.11, abs=1e-6)
    assert scale[1] == pytest.approx(0.11, abs=1e-6)


def test_tornado_road_uvs_still_tile_not_stretch(tornado_l2):
    """The literal non-regression check: road ribbon `st` on the tornado
    preset must NOT look like the hurricane's normalised-0..1 stretch --
    it must still tile at `asphalt_uv_scale_m` (default 4.0 m) or the
    trim-sheet `ROAD_TRIM_REPEAT_M`/`ROAD_TRIM_V` convention, exactly as
    before this change existed."""
    stage, _, config = tornado_l2
    uv_scale = float((config.get("roads") or {}).get("asphalt_uv_scale_m", 4.0))
    tile_mat_path = PARENT + "/ground/materials/asphalt_road_tile"
    ROAD_TRIM_V = 0.50
    checked_default = checked_alt = 0
    for prim in _road_meshes(stage):
        st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st").Get()
        us = [v[0] for v in st]
        vs = [v[1] for v in st]
        # NEVER the hurricane's normalised signature (both axes exactly 0..1).
        assert not (max(us) == pytest.approx(1.0, abs=1e-6)
                    and max(vs) == pytest.approx(1.0, abs=1e-6)), (
            f"{prim.GetPath()} looks stretch-normalised on the tornado preset")
        cum = _left_edge_cumlen(prim)
        total = cum[-1]
        if _bound_mat_path(prim) == tile_mat_path:
            # trim-sheet: v alternates ROAD_TRIM_V (left) / 0.0 (right)
            assert vs[0] == pytest.approx(ROAD_TRIM_V, abs=1e-6)
            assert vs[1] == pytest.approx(0.0, abs=1e-6)
            u_end_expected = total / 8.0    # ROAD_TRIM_REPEAT_M
            assert us[-2] == pytest.approx(u_end_expected, rel=0.05, abs=0.5)
            checked_alt += 1
        else:
            # tileable swatch: u constant per column, left==0; v grows with
            # arc length / uv_scale (approximated from the mesh's own
            # left-edge geometry -- exact up to the mitre effect at bends).
            assert us[0] == pytest.approx(0.0, abs=1e-6)
            # `total` (the LEFT-EDGE polyline length) approximates but does
            # not exactly equal the true centreline cumulative length the
            # source uses at a mitred bend, so this is a loose corroborating
            # check, not a pixel-exact one -- the hard non-regression proof
            # is the "not 0..1" assertion above plus the shader/material
            # checks elsewhere in this file.
            v_end_expected = total / uv_scale
            assert vs[-2] == pytest.approx(v_end_expected, rel=0.2, abs=2.0)
            checked_default += 1
    assert checked_default >= 5 and checked_alt >= 5


if __name__ == "__main__":
    import sys as _sys
    raise SystemExit(pytest.main([__file__, "-v"] + _sys.argv[1:]))
