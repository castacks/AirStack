#!/usr/bin/env python3
"""test_hurricane_layout_png.py -- fast, offline unit tests of the
colour-prediction CORE in `scene_gen/tools/hurricane_layout_png.py`: the
OmniPBR multiply, the custom-MDL raw-texel-mean-plus-DEAD_TINT path, and the
green/brown hue-flip check. Tiny synthetic PNG fixtures (a handful of pixels)
and, for the `pxr`-aware wrapper, tiny synthetic USD stages -- no Isaac Sim,
no real archetype assets, runs in well under a second.

    cd scene_gen && python3 -m pytest tests/test_hurricane_layout_png.py -q

WHY THIS FILE MATTERS: the tool it tests exists because a diffuse_tint that
LOOKS authored can still be a dead knob (custom MDL, no declared parameter)
and a diffuse_color_constant that LOOKS like a real colour can still never
reach the pixel (OmniPBR, texture bound, tint left neutral). Both are pinned
here directly against synthetic fixtures built to reproduce EXACTLY those
two shapes, plus the reverse (a case where the tint IS live) so the tests
cannot pass by always flagging.
"""
import os
import sys
import tempfile

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from PIL import Image  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(SCENE_GEN_DIR, "tools"))
sys.path.insert(0, SCENE_GEN_DIR)

import hurricane_layout_png as hlp  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade  # noqa: E402


# ---------------------------------------------------------------------------
# fixtures: tiny synthetic PNGs
# ---------------------------------------------------------------------------

def _make_png(path, rgba):
    """A 4x4 solid-colour RGBA PNG, `rgba` in 0..255 ints."""
    arr = np.zeros((4, 4, 4), dtype=np.uint8)
    arr[..., 0] = rgba[0]
    arr[..., 1] = rgba[1]
    arr[..., 2] = rgba[2]
    arr[..., 3] = rgba[3]
    Image.fromarray(arr, mode="RGBA").save(path)


GREEN_SRGB_255 = (61, 84, 60, 255)     # ~ the measured beech-leaf mean, sRGB
BROWN_SRGB_255 = (143, 97, 41, 255)    # a dead-leaf brown, sRGB


# ---------------------------------------------------------------------------
# 1) srgb/linear round-trip and texture sampling
# ---------------------------------------------------------------------------

def test_srgb_linear_round_trip():
    vals = np.array([0.0, 0.02, 0.18, 0.5, 0.73, 1.0])
    back = hlp.linear_to_srgb(hlp.srgb_to_linear(vals))
    assert np.allclose(back, vals, atol=1e-6)


def test_sample_texture_mean_linear_matches_hand_conversion(tmp_path):
    path = str(tmp_path / "solid.png")
    _make_png(path, (128, 64, 32, 255))
    mean = hlp.sample_texture_mean_linear(path)
    assert mean is not None
    expect = hlp.srgb_to_linear(np.array([128, 64, 32]) / 255.0)
    assert np.allclose(mean, expect, atol=1e-3)


def test_sample_texture_mean_linear_is_alpha_weighted(tmp_path):
    """A texture that is half fully-transparent black and half opaque white
    must average to WHITE (alpha=0 pixels must not pull the mean down),
    never to grey -- the exact bug an unweighted mean would introduce on a
    leaf-card alpha-cutout texture with a huge transparent margin."""
    path = str(tmp_path / "half.png")
    arr = np.zeros((4, 4, 4), dtype=np.uint8)
    arr[:2, :, :3] = 0
    arr[:2, :, 3] = 0        # transparent black, top half
    arr[2:, :, :3] = 255
    arr[2:, :, 3] = 255      # opaque white, bottom half
    Image.fromarray(arr, mode="RGBA").save(path)
    mean = hlp.sample_texture_mean_linear(path)
    assert mean is not None
    assert all(c > 0.95 for c in mean)


def test_sample_texture_mean_linear_missing_file_returns_none():
    assert hlp.sample_texture_mean_linear("/no/such/file/anywhere.png") is None


# ---------------------------------------------------------------------------
# 2) hue-flip check
# ---------------------------------------------------------------------------

def test_hue_flip_flags_green():
    green_lin = hlp.srgb_to_linear(np.array([c / 255.0 for c in GREEN_SRGB_255[:3]]))
    assert hlp.is_green_not_brown(green_lin) is True


def test_hue_flip_passes_brown():
    brown_lin = hlp.srgb_to_linear(np.array([c / 255.0 for c in BROWN_SRGB_255[:3]]))
    assert hlp.is_green_not_brown(brown_lin) is False


def test_hue_flip_passes_neutral_grey():
    grey = (0.4, 0.4, 0.4)
    assert hlp.is_green_not_brown(grey) is False


# ---------------------------------------------------------------------------
# 3) OmniPBR multiply semantics (colour-math core, no pxr)
# ---------------------------------------------------------------------------

def test_omnipbr_multiplies_texture_by_tint():
    tex_mean = (0.5, 0.5, 0.5)
    tint = (0.56, 0.38, 0.16)
    rgb, flags = hlp.omnipbr_predict(tex_mean, diffuse_tint=tint, tint_authored=True)
    assert rgb == pytest.approx(tuple(t * 0.5 for t in tint))
    assert not any(f.startswith("DEAD_TINT") for f in flags)


def test_omnipbr_no_texture_falls_back_to_constant():
    rgb, flags = hlp.omnipbr_predict(None, diffuse_color_constant=(0.2, 0.3, 0.4))
    assert rgb == pytest.approx((0.2, 0.3, 0.4))
    assert not flags


def test_omnipbr_flags_inert_constant_under_bound_texture():
    """The `detail.modular_house.skin_material` shape of bug: a texture is
    bound, `diffuse_color_constant` carries a real, deliberate colour, and
    `diffuse_tint` was never touched -- the constant can never reach the
    pixel. Must flag DEAD_TINT and predict the RAW texture mean, not the
    (inert) constant."""
    tex_mean = (0.3, 0.3, 0.3)
    rgb, flags = hlp.omnipbr_predict(
        tex_mean, diffuse_tint=None, diffuse_color_constant=(0.9, 0.1, 0.1),
        tint_authored=False)
    assert rgb == pytest.approx(tex_mean)
    assert any(f.startswith("DEAD_TINT") for f in flags)


def test_omnipbr_neutral_constant_under_texture_is_not_flagged():
    """A near-white/default `diffuse_color_constant` next to a bound texture
    is the ordinary, unremarkable case (nobody meant it as a colour) and
    must NOT be flagged -- the check has to distinguish "looks like a real
    colour choice" from "left at the default"."""
    tex_mean = (0.3, 0.3, 0.3)
    rgb, flags = hlp.omnipbr_predict(
        tex_mean, diffuse_tint=None, diffuse_color_constant=(1.0, 1.0, 1.0),
        tint_authored=False)
    assert not any(f.startswith("DEAD_TINT") for f in flags)


# ---------------------------------------------------------------------------
# 4) custom (non-OmniPBR) MDL: raw texel mean + DEAD_TINT
# ---------------------------------------------------------------------------

_ZERO_PARAM_MDL = """mdl 1.4;
using ::OmniPBR import OmniPBR;
export material Fake_leaf(*)
 = OmniPBR(
    diffuse_texture: texture_2d("./textures/fake_basecolor.png", ::tex::gamma_srgb),
    diffuse_tint: color(1.f, 1.f, 1.f),
    reflection_roughness_constant: 0.7);
"""

_PARAMETERISED_MDL = """mdl 1.4;
using ::OmniPBR import OmniPBR;
export material Real_leaf(color diffuse_tint = color(1.f, 1.f, 1.f))
 = OmniPBR(
    diffuse_texture: texture_2d("./textures/fake_basecolor.png", ::tex::gamma_srgb),
    diffuse_tint: diffuse_tint);
"""


def _write_mdl_with_texture(tmp_path, mdl_text, rgba):
    mdl_dir = tmp_path / "materials"
    tex_dir = mdl_dir / "textures"
    tex_dir.mkdir(parents=True)
    _make_png(str(tex_dir / "fake_basecolor.png"), rgba)
    mdl_path = mdl_dir / "Fake_leaf.mdl"
    mdl_path.write_text(mdl_text)
    return str(mdl_path)


def test_custom_mdl_zero_param_wrapper_flags_dead_tint_on_authored_override(tmp_path):
    mdl_path = _write_mdl_with_texture(tmp_path, _ZERO_PARAM_MDL, GREEN_SRGB_255)
    rgb, flags, debug = hlp.custom_mdl_predict(
        mdl_path, authored_inputs={"diffuse_tint": (0.56, 0.38, 0.16)})
    assert debug["declared_params"] == []
    assert any(f.startswith("DEAD_TINT") for f in flags)
    expect = hlp.srgb_to_linear(np.array([c / 255.0 for c in GREEN_SRGB_255[:3]]))
    assert rgb == pytest.approx(tuple(expect), abs=1e-3)


def test_custom_mdl_raw_mean_is_never_multiplied_by_the_dead_tint(tmp_path):
    """The core of the bug this tool exists to catch: even though the
    authored override SAYS brown (0.56, 0.38, 0.16), the predicted colour
    must be the texture's own RAW green mean, unmultiplied -- because that
    tint cannot reach the pixel. If this test ever required the tint to be
    applied, it would be testing the wrong renderer."""
    mdl_path = _write_mdl_with_texture(tmp_path, _ZERO_PARAM_MDL, GREEN_SRGB_255)
    rgb, flags, debug = hlp.custom_mdl_predict(
        mdl_path, authored_inputs={"diffuse_tint": (0.56, 0.38, 0.16)})
    tinted_wrong = tuple(0.56 * c for c in
                         hlp.srgb_to_linear(np.array([c / 255.0 for c in GREEN_SRGB_255[:3]])))
    assert rgb != pytest.approx(tinted_wrong, abs=1e-3)
    rgb_lin = np.array(rgb)
    assert hlp.is_green_not_brown(rgb_lin) is True


def test_custom_mdl_declared_parameter_is_not_flagged(tmp_path):
    """The reverse case: a custom MDL that DOES declare `diffuse_tint` as a
    real formal parameter must NOT be flagged for that same override --
    otherwise the tool would cry wolf on every correctly-authored custom
    material and nobody would trust it."""
    mdl_path = _write_mdl_with_texture(tmp_path, _PARAMETERISED_MDL, GREEN_SRGB_255)
    mdl_path = mdl_path.replace("Fake_leaf.mdl", "Fake_leaf.mdl")
    # rewrite with the parameterised text under the same texture directory
    with open(mdl_path, "w") as f:
        f.write(_PARAMETERISED_MDL)
    rgb, flags, debug = hlp.custom_mdl_predict(
        mdl_path, authored_inputs={"diffuse_tint": (0.56, 0.38, 0.16)})
    assert debug["declared_params"] == ["diffuse_tint"]
    assert not any(f.startswith("DEAD_TINT") for f in flags)


def test_custom_mdl_missing_texture_flags_missing_tex(tmp_path):
    mdl_dir = tmp_path / "materials"
    mdl_dir.mkdir(parents=True)
    mdl_path = mdl_dir / "Fake_leaf.mdl"
    mdl_path.write_text(_ZERO_PARAM_MDL)  # texture file never written
    rgb, flags, debug = hlp.custom_mdl_predict(str(mdl_path), authored_inputs={})
    assert rgb is None
    assert any(f.startswith("MISSING_TEX") for f in flags)


# ---------------------------------------------------------------------------
# 5) MDL parameter-list parsing
# ---------------------------------------------------------------------------

def test_parse_mdl_material_star_means_zero_params():
    info = hlp.parse_mdl_material(_ZERO_PARAM_MDL)
    assert info["has_decl"] is True
    assert info["params"] == set()


def test_parse_mdl_material_finds_declared_param():
    info = hlp.parse_mdl_material(_PARAMETERISED_MDL)
    assert "diffuse_tint" in info["params"]


def test_parse_mdl_material_finds_base_color_texture():
    info = hlp.parse_mdl_material(_ZERO_PARAM_MDL)
    names = [t[1] for t in info["textures"]]
    assert any("fake_basecolor.png" in n for n in names)


# ---------------------------------------------------------------------------
# 6) the pxr-aware wrapper against tiny synthetic USD stages -- proves the
#    OmniPBR-vs-custom-MDL CLASSIFICATION itself (info:id / sourceAsset
#    basename), not just the maths above.
# ---------------------------------------------------------------------------

def _define_omnipbr_material(stage, path, tex_path=None, tint=None, const=None):
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    if tex_path:
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_path))
    if tint is not None:
        sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    if const is not None:
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*const))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _define_custom_mdl_material(stage, path, mdl_path, tint=None):
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.SetSourceAsset(Sdf.AssetPath(mdl_path), "mdl")
    sh.SetSourceAssetSubIdentifier("Fake_leaf", "mdl")
    if tint is not None:
        sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def test_predict_shader_color_omnipbr_end_to_end(tmp_path):
    tex_path = str(tmp_path / "tex.png")
    _make_png(tex_path, (255, 255, 255, 255))  # white texture, mean ~1.0 linear
    usda = str(tmp_path / "stage.usda")
    stage = Usd.Stage.CreateNew(usda)
    mat = _define_omnipbr_material(stage, "/Looks/M", tex_path=tex_path,
                                   tint=(0.5, 0.25, 0.1))
    stage.GetRootLayer().Save()
    stage2 = Usd.Stage.Open(usda)
    prim = stage2.GetPrimAtPath("/Looks/M")
    rgb, flags, debug = hlp.predict_shader_color(prim)
    assert debug["family"] == "OmniPBR"
    # white texture * tint, converted back to sRGB, should read close to the
    # tint itself (linear (0.5,0.25,0.1) -> srgb is brighter, but still
    # R > G > B, same ordering as the tint).
    assert rgb[0] > rgb[1] > rgb[2]


def test_predict_shader_color_custom_mdl_end_to_end_flags_dead_tint(tmp_path):
    mdl_path = _write_mdl_with_texture(tmp_path, _ZERO_PARAM_MDL, GREEN_SRGB_255)
    usda = str(tmp_path / "stage.usda")
    stage = Usd.Stage.CreateNew(usda)
    _define_custom_mdl_material(stage, "/Looks/M", mdl_path, tint=(0.56, 0.38, 0.16))
    stage.GetRootLayer().Save()
    stage2 = Usd.Stage.Open(usda)
    prim = stage2.GetPrimAtPath("/Looks/M")
    rgb, flags, debug = hlp.predict_shader_color(prim)
    assert debug["family"] == "custom_mdl"
    assert any(f.startswith("DEAD_TINT") for f in flags)
    assert hlp.is_green_not_brown(hlp.srgb_to_linear(np.array(rgb))) is True


def test_predict_shader_color_missing_texture_returns_none_and_flags(tmp_path):
    usda = str(tmp_path / "stage.usda")
    stage = Usd.Stage.CreateNew(usda)
    _define_omnipbr_material(stage, "/Looks/M", tex_path="./does_not_exist.png")
    stage.GetRootLayer().Save()
    stage2 = Usd.Stage.Open(usda)
    prim = stage2.GetPrimAtPath("/Looks/M")
    rgb, flags, debug = hlp.predict_shader_color(prim)
    assert rgb is None
    assert any(f.startswith("MISSING_TEX") for f in flags)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
