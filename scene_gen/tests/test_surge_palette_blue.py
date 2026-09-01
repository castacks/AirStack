"""test_surge_palette_blue.py — the BLUE water A/B variant.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pillow --with pytest python -m pytest \\
        tests/test_surge_palette_blue.py -q

The ask: a matched A/B at the end -- brown (today's default) vs BLUE water,
selected ONLY by `SURGE_PALETTE=blue`, geometry untouched, and with ponds/
deposits staying mud-coloured regardless ("a puddle on a lawn is not blue").

`disaster.surge.water_materials` has no `cfg` parameter (its own documented
API contract) -- it reads `SURGE_PALETTE` from the environment directly, so
these tests set/clear the env var around each stage build rather than
passing a config dict.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import surge                                     # noqa: E402

from pxr import Usd, Gf                                         # noqa: E402


def _clear_surge_env():
    for k in list(os.environ):
        if k.startswith("SURGE_"):
            del os.environ[k]


def setup_function(_fn):
    _clear_surge_env()


def teardown_function(_fn):
    _clear_surge_env()


def _rgb(shader, input_name="diffuse_color_constant"):
    v = shader.GetAttribute("inputs:" + input_name).Get()
    return tuple(round(float(c), 6) for c in v)


def _body_and_pond_shaders(stage, path_root="/World/Water"):
    surge.water_materials(stage, path_root)
    body = stage.GetPrimAtPath(path_root + "/WaterLooks/Body/Shader")
    band0 = stage.GetPrimAtPath(path_root + "/WaterLooks/Body_band00/Shader")
    pond_g = stage.GetPrimAtPath(path_root + "/WaterLooks/PondGrass/Shader")
    pond_p = stage.GetPrimAtPath(path_root + "/WaterLooks/PondPaved/Shader")
    return body, band0, pond_g, pond_p


# ---------------------------------------------------------------------------
# the palette exists and is a real blue
# ---------------------------------------------------------------------------

def test_blue_palette_is_registered_and_is_actually_blue():
    assert "blue" in surge._PALETTE
    r, g, b = surge._PALETTE["blue"]
    assert b > r and b > 0.10, surge._PALETTE["blue"]
    # low turbidity / clear-coastal, not a saturated swimming-pool blue
    assert max(r, g, b) < 0.30, surge._PALETTE["blue"]


# ---------------------------------------------------------------------------
# SURGE_PALETTE=blue changes the water BODY
# ---------------------------------------------------------------------------

def test_blue_palette_changes_the_water_body_colour():
    st_default = Usd.Stage.CreateInMemory()
    body_def, band0_def, _, _ = _body_and_pond_shaders(st_default)
    rgb_def = _rgb(body_def)
    rgb_band_def = _rgb(band0_def)

    os.environ["SURGE_PALETTE"] = "blue"
    st_blue = Usd.Stage.CreateInMemory()
    body_blue, band0_blue, _, _ = _body_and_pond_shaders(st_blue)
    rgb_blue = _rgb(body_blue)
    rgb_band_blue = _rgb(band0_blue)

    assert rgb_def != rgb_blue, "SURGE_PALETTE=blue did not change the body colour"
    assert rgb_band_def != rgb_band_blue, (
        "SURGE_PALETTE=blue did not change the opacity-band colour")
    # and it should actually read as MORE blue: the blue channel should end
    # up relatively larger than red once selected
    assert (rgb_blue[2] / max(1e-6, rgb_blue[0])) > (
        rgb_def[2] / max(1e-6, rgb_def[0]))


def test_default_palette_is_unaffected_by_an_explicit_sediment_request():
    """`SURGE_PALETTE=sediment` (the documented default value) must produce
    the SAME body colour as leaving the variable unset entirely -- i.e.
    setting it to its own default is a no-op."""
    st_unset = Usd.Stage.CreateInMemory()
    body_unset, _, _, _ = _body_and_pond_shaders(st_unset)
    rgb_unset = _rgb(body_unset)

    os.environ["SURGE_PALETTE"] = "sediment"
    st_explicit = Usd.Stage.CreateInMemory()
    body_explicit, _, _, _ = _body_and_pond_shaders(st_explicit)
    rgb_explicit = _rgb(body_explicit)

    assert rgb_unset == rgb_explicit


# ---------------------------------------------------------------------------
# ponds (and, by the same mechanism, every OTHER "_dry_material"-built
# deposit look) do NOT follow the water body's palette
# ---------------------------------------------------------------------------

def test_blue_palette_does_not_tint_the_ponds():
    st_default = Usd.Stage.CreateInMemory()
    _, _, pond_g_def, pond_p_def = _body_and_pond_shaders(st_default)
    rgb_pond_g_def = _rgb(pond_g_def)
    rgb_pond_p_def = _rgb(pond_p_def)

    os.environ["SURGE_PALETTE"] = "blue"
    st_blue = Usd.Stage.CreateInMemory()
    _, _, pond_g_blue, pond_p_blue = _body_and_pond_shaders(st_blue)
    rgb_pond_g_blue = _rgb(pond_g_blue)
    rgb_pond_p_blue = _rgb(pond_p_blue)

    assert rgb_pond_g_def == rgb_pond_g_blue, (
        "a blue water palette leaked into the unpaved pond colour: "
        "{0} -> {1}".format(rgb_pond_g_def, rgb_pond_g_blue))
    assert rgb_pond_p_def == rgb_pond_p_blue, (
        "a blue water palette leaked into the paved pond colour: "
        "{0} -> {1}".format(rgb_pond_p_def, rgb_pond_p_blue))
    # and the pond stays MUD, not blue, under either palette
    for rgb in (rgb_pond_g_def, rgb_pond_g_blue, rgb_pond_p_def, rgb_pond_p_blue):
        assert rgb[0] >= rgb[2], "pond colour is not mud-brown: {0}".format(rgb)


def test_carbonate_palette_also_does_not_tint_the_ponds():
    """The SAME bug (ponds inheriting whatever `rgb` the water body picked)
    would have been live for the pre-existing `"carbonate"`/`"blackwater"`
    palettes too, not just the new `"blue"` one -- confirm the fix is
    general, not special-cased to `"blue"`."""
    st_default = Usd.Stage.CreateInMemory()
    _, _, pond_g_def, _ = _body_and_pond_shaders(st_default)
    rgb_pond_def = _rgb(pond_g_def)

    os.environ["SURGE_PALETTE"] = "carbonate"
    st_carb = Usd.Stage.CreateInMemory()
    _, _, pond_g_carb, _ = _body_and_pond_shaders(st_carb)
    rgb_pond_carb = _rgb(pond_g_carb)

    assert rgb_pond_def == rgb_pond_carb


# ---------------------------------------------------------------------------
# geometry-side: build_ponding/build_deposits do not import a palette at all
# ---------------------------------------------------------------------------

def test_pond_geometry_functions_take_no_palette_input():
    """`pond_specs`/`build_ponding` never read `SURGE_PALETTE` themselves --
    the only place a palette choice can reach a pond is through the shared
    `water_materials` dict, which the two tests above already pin down."""
    import inspect
    src = inspect.getsource(surge.build_ponding)
    assert "SURGE_PALETTE" not in src
    src2 = inspect.getsource(surge.build_deposits)
    assert "SURGE_PALETTE" not in src2


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
