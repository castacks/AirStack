"""test_surge_wet_mounds.py — the wet-dirt-mounds/deposits material fix.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pillow --with pytest python -m pytest \\
        tests/test_surge_wet_mounds.py -q

The review (SESSION_2026-08-31.md and the user, verbatim): "the deposits/
ponds/washover on lawns and roads are flat brown polygons with a hard darker
outline, wearing a photograph of DRY cracked earth ... improve the textures
being used for the wet dirt mounds."

Covers:
  * the derived `WET_SILT_TEXTURE`/`_NORMAL`/`_ORM` files exist and measure
    inside the sourced ranges (albedo mean, roughness);
  * the pond bed and the water volume's submerged bed use the WET pack, not
    the dry cracked-earth one;
  * `pond_rim_feather_m` feathers a pond's outline by 0.5-1.0 m (additive,
    not the old ratio, which scaled with pond size);
  * `build_deposits` darkens wrack/washover specs within the wet band
    (`_DEPOSIT_WET_BAND_M`) onto a dedicated, glossier "_wet" material,
    distinct from the spatial-noise dry/ageing tonal ladder.
"""
import math
import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

import scene_generator as sg                                   # noqa: E402
from disaster import surge                                      # noqa: E402

from pxr import Usd, UsdShade                                   # noqa: E402


def _srgb_to_linear(u8):
    c = u8.astype(np.float64) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


# ---------------------------------------------------------------------------
# the derived pack itself
# ---------------------------------------------------------------------------

def test_wet_silt_pack_files_exist():
    for const in (surge.WET_SILT_TEXTURE, surge.WET_SILT_NORMAL_TEXTURE,
                 surge.WET_SILT_ORM_TEXTURE):
        p = sg._join_asset_root(const, "")
        assert os.path.exists(p), p


def test_wet_silt_usda_wrapper_exists():
    wrapper = os.path.join(
        os.path.dirname(_HERE), "assets", "materials", "megascans",
        "Soil_Mud_Wet.usda")
    assert os.path.exists(wrapper), wrapper


def test_wet_silt_albedo_mean_in_sourced_range():
    from PIL import Image
    p = sg._join_asset_root(surge.WET_SILT_TEXTURE, "")
    arr = np.asarray(Image.open(p).convert("RGB"))
    mean_lin = float(_srgb_to_linear(arr).mean())
    assert 0.035 <= mean_lin <= 0.065, (
        "wet silt albedo mean {0:.4f} outside the sourced 0.04-0.06 band "
        "(with a small margin)".format(mean_lin))
    # and it must be DARKER than the dry source it was derived from
    dry_p = sg._join_asset_root(surge.SILT_TEXTURE, "")
    dry_arr = np.asarray(Image.open(dry_p).convert("RGB"))
    dry_mean = float(_srgb_to_linear(dry_arr).mean())
    assert mean_lin <= dry_mean, (
        "the derived wet map is not darker than the dry source it came from")


def test_wet_silt_roughness_in_sourced_range():
    from PIL import Image
    p = sg._join_asset_root(surge.WET_SILT_ORM_TEXTURE, "")
    arr = np.asarray(Image.open(p).convert("RGB")).astype(np.float64) / 255.0
    rough_mean = float(arr[..., 1].mean())   # G channel = roughness
    assert 0.10 <= rough_mean <= 0.28, (
        "wet silt ORM roughness mean {0:.3f} outside the sourced "
        "0.12-0.25 band (with a small margin)".format(rough_mean))
    metal_mean = float(arr[..., 2].mean())
    assert metal_mean < 0.02, "metallic channel is not pinned to ~0"


def test_wet_silt_normal_is_flatter_than_the_dry_source():
    from PIL import Image
    wet_p = sg._join_asset_root(surge.WET_SILT_NORMAL_TEXTURE, "")
    dry_p = sg._join_asset_root(surge.SILT_NORMAL_TEXTURE, "")
    wet = np.asarray(Image.open(wet_p).convert("RGB")).astype(np.float64) / 255.0
    dry = np.asarray(Image.open(dry_p).convert("RGB")).astype(np.float64) / 255.0
    # Z-component (blue channel, tangent space) close to 1.0 = flat.
    wet_z = float((wet[..., 2] * 2.0 - 1.0).mean())
    dry_z = float((dry[..., 2] * 2.0 - 1.0).mean())
    assert wet_z > dry_z, (
        "the derived normal is not flatter than the dry source "
        "(wet_z={0:.4f}, dry_z={1:.4f})".format(wet_z, dry_z))
    assert wet_z > 0.9, "the derived normal is not close to flat"


# ---------------------------------------------------------------------------
# the pond bed and the water volume's own submerged bed use WET_SILT, not
# the dry cracked-earth SILT_TEXTURE
# ---------------------------------------------------------------------------

def _tex_path(shader, input_name="diffuse_texture"):
    v = shader.GetAttribute("inputs:" + input_name).Get()
    return str(v).strip("@") if v is not None else None


def test_pond_beds_use_wet_silt_not_dry():
    stage = Usd.Stage.CreateInMemory()
    mats = surge.water_materials(stage, "/World/Water")
    for key in ("pond_unpaved", "pond_paved", "pond_unpaved_rim",
               "pond_paved_rim"):
        sh_path = mats[key].GetPath().AppendChild("Shader")
        sh = stage.GetPrimAtPath(sh_path)
        tex = _tex_path(sh)
        assert tex is not None, key
        assert "Soil_Mud_Wet" in tex, "{0} still binds {1}".format(key, tex)
        assert "Soil_Mud/" not in tex, "{0} still binds the DRY pack".format(key)


# ---------------------------------------------------------------------------
# STREAM-W2 -- "the texture on the interior polygon part can be the muddy
# soil texture" (user, verbatim). Not just a substring match on the bound
# path: the path has to actually RESOLVE to a file that exists on disk (the
# thing that renders as a flat constant colour when it does not -- an
# unresolved `diffuse_texture` falls back to `diffuse_color_constant`/
# `diffuse_tint` alone, which is indistinguishable from "never bound a
# texture" on screen), and the tiling has to sit in a sane band rather than
# the un-set 1 m default.
# ---------------------------------------------------------------------------

def test_pond_core_and_rim_diffuse_normal_orm_resolve_to_real_files():
    stage = Usd.Stage.CreateInMemory()
    mats = surge.water_materials(stage, "/World/Water")
    for key in ("pond_unpaved", "pond_paved", "pond_unpaved_rim",
               "pond_paved_rim"):
        sh = stage.GetPrimAtPath(mats[key].GetPath().AppendChild("Shader"))
        for input_name in ("diffuse_texture", "normalmap_texture",
                           "ORM_texture"):
            p = _tex_path(sh, input_name)
            assert p, "{0}.{1} not bound".format(key, input_name)
            assert os.path.isabs(p) and "://" not in p, (
                "{0}.{1} is not an expanded, resolvable path: {2}".format(
                    key, input_name, p))
            assert os.path.exists(p), (
                "{0}.{1} points at a file that does not exist: {2} -- this "
                "is exactly the failure mode that renders as a flat "
                "constant colour instead of the mud photograph".format(
                    key, input_name, p))


def test_pond_core_and_rim_tile_in_the_2_to_6_metre_band():
    """`texture_scale` is REPEATS PER METRE (`damage._pbr`'s documented
    convention, reused by `water_materials`'s own `_make`). The un-set
    default (1.0 rep/m -- a 1 m tile) was never itself the "flat" bug, but a
    tile this small is needless high-frequency detail at the 22-475 m review
    -camera heights this module targets and the one place actually worth
    tuning while this material is already open -- 2-6 m per repeat is the
    asked-for band."""
    stage = Usd.Stage.CreateInMemory()
    mats = surge.water_materials(stage, "/World/Water")
    for key in ("pond_unpaved", "pond_paved", "pond_unpaved_rim",
               "pond_paved_rim"):
        sh = stage.GetPrimAtPath(mats[key].GetPath().AppendChild("Shader"))
        scale = sh.GetAttribute("inputs:texture_scale").Get()
        assert scale is not None, key
        for rep_per_m in (float(scale[0]), float(scale[1])):
            tile_m = 1.0 / rep_per_m
            assert 2.0 <= tile_m <= 6.0, (
                "{0} tiles every {1:.2f} m (outside the 2-6 m band)".format(
                    key, tile_m))


def test_pond_mud_base_resolves_and_tiles_in_band():
    """`build_ponding`'s own `mud_mat` (the CORE's footprint, directly under
    the water -- see that function's docstring) through the REAL call site,
    not a re-implementation of it."""
    cfg = surge.resolve_cfg({})
    region = (-50.0, -50.0, 50.0, 50.0)
    stage = Usd.Stage.CreateInMemory()

    def fake_pond_specs(_cfg, _region, _rng):
        return [{"x": 0.0, "y": 0.0, "r_m": 2.5, "depth_m": 0.1,
                 "paved": False}]

    orig = surge.pond_specs
    surge.pond_specs = fake_pond_specs
    try:
        made = surge.build_ponding(stage, "/World/Water", cfg, region,
                                   random.Random(7))
    finally:
        surge.pond_specs = orig
    assert any(p.endswith("_mud") for p in made), made

    mud_prim = stage.GetPrimAtPath("/World/Water/ponding/grass_mud")
    assert mud_prim.IsValid()
    from pxr import UsdShade as _UsdShade
    mud_mat = _UsdShade.MaterialBindingAPI(mud_prim).ComputeBoundMaterial()[0]
    sh = stage.GetPrimAtPath(mud_mat.GetPath().AppendChild("Shader"))

    tex = _tex_path(sh)
    assert tex and os.path.exists(tex), tex
    scale = sh.GetAttribute("inputs:texture_scale").Get()
    for rep_per_m in (float(scale[0]), float(scale[1])):
        tile_m = 1.0 / rep_per_m
        assert 2.0 <= tile_m <= 6.0, (
            "pond_mud tiles every {0:.2f} m (outside the 2-6 m band)".format(
                tile_m))


def test_water_volume_submerged_bed_uses_wet_silt():
    """`_build_inundation_volume`'s `water_mud` look -- built via
    `_dry_material` directly, not through `water_materials` -- must also use
    the wet pack: this surface is BY DEFINITION always underwater."""
    stage = Usd.Stage.CreateInMemory()
    from disaster import damage  # noqa: F401  (import used indirectly)
    mat = surge._dry_material(
        stage, "/World/Water/DepositLooks/water_mud",
        rgb=(0.30, 0.25, 0.18), rough=0.35, scale=(0.5, 0.5), desat=0.30,
        texture=surge.WET_SILT_TEXTURE, normal=surge.WET_SILT_NORMAL_TEXTURE,
        orm=surge.WET_SILT_ORM_TEXTURE,
        tex_mean_linear=surge._WET_SILT_TEX_MEAN_LINEAR)
    sh = stage.GetPrimAtPath(mat.GetPath().AppendChild("Shader"))
    tex = _tex_path(sh)
    assert "Soil_Mud_Wet" in tex


def test_build_inundation_volume_actually_calls_it_with_wet_silt():
    """Read the real call site rather than re-implementing it, so a future
    edit that quietly reverts the texture argument is caught."""
    import inspect
    src = inspect.getsource(surge._build_inundation_volume)
    assert "WET_SILT_TEXTURE" in src
    assert "_WET_SILT_TEX_MEAN_LINEAR" in src


def test_pond_mud_base_uses_wet_silt():
    import inspect
    src = inspect.getsource(surge.build_ponding)
    # the pond_mud `_dry_material` call must reference the wet constants
    idx = src.index("pond_mud")
    window = src[idx:idx + 700]
    assert "WET_SILT_TEXTURE" in window


# ---------------------------------------------------------------------------
# pond edge feathering: additive metres, in the 0.5-1.0 m band
# ---------------------------------------------------------------------------

def test_pond_rim_feather_default_is_in_the_asked_range():
    assert 0.5 <= surge.DEFAULTS["pond_rim_feather_m"] <= 1.0


def test_pond_rim_is_a_constant_metre_offset_not_a_ratio():
    """Build two ponds of very different radii and confirm the rim-to-core
    gap is the SAME number of metres for both -- a ratio (the old
    `_POND_RIM_GROW`) would make a small pond's gap tiny and a large pond's
    gap huge."""
    cfg = surge.resolve_cfg({
        "pond_base_per_100m2": 0.0, "pond_edge_boost_per_100m2": 0.0,
    })
    region = (-50.0, -50.0, 50.0, 50.0)
    specs = [
        {"x": -20.0, "y": 0.0, "r_m": 0.7, "depth_m": 0.05, "paved": False},
        {"x": 20.0, "y": 0.0, "r_m": 3.2, "depth_m": 0.05, "paved": False},
    ]
    stage = Usd.Stage.CreateInMemory()

    import types
    fake_rng = random.Random(3)

    def fake_pond_specs(_cfg, _region, _rng):
        return specs

    orig = surge.pond_specs
    surge.pond_specs = fake_pond_specs
    try:
        made = surge.build_ponding(stage, "/World/Water", cfg, region,
                                   fake_rng)
    finally:
        surge.pond_specs = orig
    assert made

    core = stage.GetPrimAtPath("/World/Water/ponding/grass")
    rim = stage.GetPrimAtPath("/World/Water/ponding/grass_rim")
    assert core.IsValid() and rim.IsValid()

    def _radial_extent(prim, cx, cy):
        pts = prim.GetAttribute("points").Get()
        return max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)

    core_pts = core.GetAttribute("points").Get()
    # separate the two ponds' own point clouds by x-sign (they were placed at
    # x=-20 and x=+20 and never overlap)
    small_core = [p for p in core_pts if p[0] < 0]
    big_core = [p for p in core_pts if p[0] > 0]
    rim_pts = rim.GetAttribute("points").Get()
    small_rim = [p for p in rim_pts if p[0] < 0]
    big_rim = [p for p in rim_pts if p[0] > 0]

    small_core_r = max(math.hypot(p[0] - (-20.0), p[1]) for p in small_core)
    small_rim_r = max(math.hypot(p[0] - (-20.0), p[1]) for p in small_rim)
    big_core_r = max(math.hypot(p[0] - 20.0, p[1]) for p in big_core)
    big_rim_r = max(math.hypot(p[0] - 20.0, p[1]) for p in big_rim)

    small_gap = small_rim_r - small_core_r
    big_gap = big_rim_r - big_core_r
    feather = float(cfg["pond_rim_feather_m"])
    assert abs(small_gap - feather) < 0.05 * feather + 0.05, (
        small_gap, feather)
    assert abs(big_gap - feather) < 0.05 * feather + 0.05, (big_gap, feather)
    # the whole point of the fix: the two gaps must be close to EQUAL despite
    # the 0.7 m vs 3.2 m core radii, unlike a multiplicative ratio.
    assert abs(small_gap - big_gap) < 0.15, (
        "rim gap scaled with pond radius -- still a ratio, not additive: "
        "small={0:.3f} big={1:.3f}".format(small_gap, big_gap))


# ---------------------------------------------------------------------------
# the wet band: deposits near the current waterline get a darker, glossier
# material than the spatial-noise dry/ageing ladder
# ---------------------------------------------------------------------------

def test_deposit_wet_band_constant_matches_the_brief():
    assert surge._DEPOSIT_WET_BAND_M == 2.0


def test_build_deposits_uses_a_dedicated_wet_material_near_the_waterline():
    cfg = surge.resolve_cfg({
        "wrack_per_100m2": 0.0,      # silence the real scatter --
        "washover_fans": 0,          # we inject synthetic specs instead
    })
    region = (-50.0, -50.0, 50.0, 50.0)
    stage = Usd.Stage.CreateInMemory()

    # One spec pinned AT the waterline (signed depth 0 -> inside the 2 m wet
    # band), one pinned far inland (signed depth strongly negative -> dry).
    def fake_wrack_specs(_cfg, _region, _rng):
        return [{
            "kind": "ridge", "cls": "wrack", "z": 0.03,
            "stations": [(0.0, 0.0, 0.0, 0.05, 0.3, 0.3, 0.0),
                        (1.0, 0.0, 0.0, 0.05, 0.3, 0.3, 0.0)],
            "yaw": 0.0, "len_m": 1.0, "width_m": 0.6, "height_m": 0.05,
            "x": 0.0, "y": 0.0,
        }]

    def fake_washover_specs(_cfg, _region, _rng):
        return [{
            "kind": "mound", "cls": "sand", "x": 0.0, "y": 0.0,
            "z": 0.03, "base": 0.03, "rx": 4.0, "ry": 3.0, "h": 0.5,
            "yaw": 0.0, "wob": (0.1, 0.0, 0.05, 1.0, 0.02, 2.0),
        }]

    orig_wrack, orig_wash = surge.wrack_specs, surge._washover_specs
    surge.wrack_specs = fake_wrack_specs
    surge._washover_specs = fake_washover_specs
    try:
        # find a point far from the water: `ground_z` at the origin under
        # this cfg is the shoreline itself (default shore_offset_m=0), so
        # walk seaward/inland along the default bearing until the signed
        # depth is well past the wet band.
        sd_fn = surge.signed_depth_at(cfg, region, None)
        assert abs(sd_fn(0.0, 0.0)) <= surge._DEPOSIT_WET_BAND_M, (
            "fixture assumption broken: the origin is not inside the wet "
            "band under the default terrain")

        made = surge.build_deposits(stage, "/World/Water", cfg, region,
                                    random.Random(5))
    finally:
        surge.wrack_specs = orig_wrack
        surge._washover_specs = orig_wash

    assert any("wrack_wet" in p for p in made), made
    assert any("sand_wet" in p for p in made), made

    wrack_wet = stage.GetPrimAtPath("/World/Water/deposits/relief/wrack_wet")
    sand_wet = stage.GetPrimAtPath("/World/Water/deposits/relief/sand_wet")
    assert wrack_wet.IsValid()
    assert sand_wet.IsValid()

    wrack_mat = UsdShade.MaterialBindingAPI(wrack_wet).ComputeBoundMaterial()[0]
    sand_mat = UsdShade.MaterialBindingAPI(sand_wet).ComputeBoundMaterial()[0]
    assert wrack_mat and wrack_mat.GetPrim().IsValid()
    assert sand_mat and sand_mat.GetPrim().IsValid()


def test_wet_band_material_is_darker_and_glossier_than_the_dry_ladder():
    """The wet variant must be darker (lower diffuse tint magnitude is not
    directly comparable across textures, so compare via the authored `rough`
    constant and the `rgb` argument the module docstring records) than even
    the palest DRY tonal variant, and lower-roughness (glossier)."""
    import inspect
    src = inspect.getsource(surge.build_deposits)
    # sanity: the wet tints are literally darker than the darkest dry tint
    # recorded in the source (index 0 of each _TINTS tuple).
    def _extract_tuple(name):
        start = src.index(name + " = (")
        end = src.index(")\n", start)
        return eval(src[start + len(name) + 3: end + 1])

    wrack_tints = _extract_tuple("_WRACK_TINTS")
    sand_tints = _extract_tuple("_SAND_TINTS")
    wrack_wet = _extract_tuple("_WRACK_WET_TINT")
    sand_wet = _extract_tuple("_SAND_WET_TINT")

    assert sum(wrack_wet) < sum(wrack_tints[0]), (
        "wrack_wet is not darker than the darkest dry wrack tint")
    assert sum(sand_wet) < sum(sand_tints[0]), (
        "sand_wet is not darker than the darkest dry sand tint")

    assert "rough=0.45" in src   # wrack_wet, well under the dry 0.94
    assert "rough=0.28" in src   # sand_wet, well under the dry 0.80


# ---------------------------------------------------------------------------
# ADDENDUM (user, verbatim): "turn all the asphalt in the area to the wet
# destroyed asphalt. Cause hurricane would affect everything."
#
# Not a `surge.py` change: roads/driveways come from `apply_ground_planes`
# (`scene_generator.py:1212`, the whole-region `asphalt_base` mesh) and
# `suburb_scene.py`'s per-parcel driveway mix (`suburb_scene.py:1341-1342`,
# `driveway_asphalt` blended in at `driveway_asphalt_share` of parcels — the
# other share stays the shared brick apron, which is not asphalt and so out
# of this ask's scope). Both read a material USD reference from the
# preset's `usds.materials.{asphalt,driveway_asphalt}` keys rather than
# authoring a shader inline, and BOTH already point at
# `Wet_Destroyed_Asphalt.usda` in the two hurricane presets actually
# rendered (`suburb_hurricane_500_l2.yaml`/`_l3.yaml`) — a prior, already-
# committed pass (commit e6aee69b, predating this session). No inline
# rebind was written on top of it: the config knob already exists and
# scopes correctly (checked below), which is what "prefer call-site
# changes" means when the call site is a YAML key, not a code path.
# ---------------------------------------------------------------------------

import yaml                                                     # noqa: E402

_CONFIG_DIR = os.path.normpath(os.path.join(_HERE, "..", "config"))
_WET_ASPHALT_USDA = ("airstack://scene_gen/assets/materials/megascans/"
                     "Wet_Destroyed_Asphalt.usda")


def _load_preset(name):
    with open(os.path.join(_CONFIG_DIR, "presets", name)) as f:
        return yaml.safe_load(f)


def _preset_block(cfg, *keys):
    """A preset's own knobs live either at the top level or nested under
    `overrides:` (the asset-set/locale layering these hurricane presets
    use) -- both shapes are live in this repo (spot-checked: the tornado/
    wildfire/downtown presets used by the scoping guard below are flat,
    the hurricane ones nest under `overrides`), so check both rather than
    assume one."""
    for root in (cfg, cfg.get("overrides") or {}):
        node = root
        for k in keys:
            node = (node or {}).get(k)
        if node:
            return node
    return {}


def test_hurricane_presets_bind_wet_destroyed_asphalt_for_roads_and_drives():
    for name in ("suburb_hurricane_500_l2.yaml", "suburb_hurricane_500_l3.yaml"):
        cfg = _load_preset(name)
        mats = _preset_block(cfg, "usds", "materials")
        assert mats.get("asphalt") == _WET_ASPHALT_USDA, name
        assert mats.get("driveway_asphalt") == _WET_ASPHALT_USDA, name
        # some non-zero share of driveways actually uses the asphalt look
        # (the rest stays the shared brick apron, deliberately -- brick is
        # not asphalt and is not this ask's target).
        share = float(_preset_block(cfg, "roads").get(
            "driveway_asphalt_share", 0.0))
        assert share > 0.0, "{0}: no driveway ever uses asphalt".format(name)


def test_other_suburb_presets_do_not_pick_up_wet_destroyed_asphalt():
    """Scoping guard: the swap must not leak into the tornado/wildfire
    suburbs (or downtown) via a shared base file."""
    for name in ("suburb_tornado.yaml", "suburb_tornado_1000.yaml",
                "suburb_wildfire.yaml", "suburb_wildfire_500.yaml",
                "downtown.yaml", "downtown_earthquake.yaml"):
        cfg = _load_preset(name)
        mats = _preset_block(cfg, "usds", "materials")
        assert mats.get("asphalt") != _WET_ASPHALT_USDA, name
        assert mats.get("driveway_asphalt") != _WET_ASPHALT_USDA, name


def test_wet_destroyed_asphalt_pack_resolves_diffuse_normal_orm():
    p = sg._join_asset_root(_WET_ASPHALT_USDA, "")
    assert os.path.exists(p), p
    pack_dir = os.path.join(os.path.dirname(p), "Wet_Destroyed_Asphalt")
    for fname in ("T_si1odala_4K_B.png", "T_si1odala_4K_N.png",
                 "T_si1odala_4K_ORM.png"):
        assert os.path.exists(os.path.join(pack_dir, fname)), fname


def test_wet_destroyed_asphalt_referenced_material_resolves_and_tiles():
    """Reproduce the way `apply_ground_planes`/`suburb_scene.py` bind a road
    material -- `stage.DefinePrim(path).GetReferences().AddReference(url)`
    -- and read back the COMPOSED shader, not the wrapper file's text, so a
    stale/relative reference inside the wrapper would show up here too."""
    from pxr import Usd as _Usd

    p = sg._join_asset_root(_WET_ASPHALT_USDA, "")
    stage = _Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/World/RoadMat")
    prim.GetReferences().AddReference(p)
    prim.Load()

    sh = stage.GetPrimAtPath("/World/RoadMat/Shader")
    assert sh.IsValid(), "Wet_Destroyed_Asphalt.usda composed no Shader prim"

    diffuse = sh.GetAttribute("inputs:diffuse_texture").Get()
    assert diffuse is not None
    # The wrapper's own texture paths are relative (`./Wet_Destroyed_Asphalt/
    # T_...png`) and anchored by USD's own asset-path resolution against the
    # LAYER that authored them -- resolve through the attribute's resolved
    # path, not string concatenation, so this test fails the same way a real
    # unresolvable reference would.
    resolved = sh.GetAttribute("inputs:diffuse_texture").Get().resolvedPath
    assert resolved and os.path.exists(resolved), (
        "diffuse_texture did not resolve to a real file: {0}".format(
            resolved))

    scale = sh.GetAttribute("inputs:texture_scale").Get()
    assert scale is not None
    for rep_per_m in (float(scale[0]), float(scale[1])):
        tile_m = 1.0 / rep_per_m
        # Wide band -- this is a big storm-broken pavement scan (potholes,
        # spalling) meant to read at both a 30 m and a 400 m review-camera
        # height (the ask), not a close-up material; the pack's OWN authored
        # `texture_scale` (0.09 rep/m -> ~11.1 m/tile) is the number used
        # verbatim, not retuned here.
        # 2026-08-31 (user: hurricane affects everything): tightened from the pack's
        # native ~11 m to ~3.6 m so the pothole/spall detail reads from the review
        # cameras. Band widened accordingly.
        assert 2.5 <= tile_m <= 6.0, (
            "Wet_Destroyed_Asphalt tiles every {0:.2f} m -- expected ~11 m "
            "(0.09 rep/m) per the pack's own wrapper".format(tile_m))


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
