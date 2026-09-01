"""test_surge_asset_paths.py — the unlocated "airstack:// reaches Hydra" bug.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pillow --with pytest python -m pytest \\
        tests/test_surge_asset_paths.py -q

Every render logged (SESSION_2026-08-31.md §4 item 3):

    parameter 'diffuse_texture': References an asset that can not be found:
    'airstack://scene_gen/assets/materials/megascans/Soil_Mud/T_pjuph20_2K_B.png'

i.e. the RAW `airstack://` scheme reaching Hydra unexpanded, even though
`scene_generator._join_asset_root` resolves it correctly in isolation. The
site: `disaster.planks.skin_material` bound `Sdf.AssetPath(str(texture))`
directly, with no `_join_asset_root` call, and `disaster.washaway.
apply_washaway`'s "wet"/"scoured" levels call it with `surge.SILT_TEXTURE` —
an unexpanded string — for exactly this material.

This file checks BOTH halves: every texture constant `disaster.surge`
defines resolves, through the SAME join the builders use, to a real file on
disk; and the actual authored `diffuse_texture` a stage ends up with (via
`planks.skin_material` and via `disaster.ground.overlay_material`, the two
non-`surge.py` call sites that bind a `surge` texture constant) is an
expanded, existing path — never the raw scheme.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

import scene_generator as sg                                  # noqa: E402
from disaster import surge, ground, planks                    # noqa: E402

from pxr import Usd                                            # noqa: E402


def _texture_constants():
    """Every `..._TEXTURE` string constant `disaster.surge` defines."""
    return {n: getattr(surge, n) for n in dir(surge)
            if n.endswith("_TEXTURE") and isinstance(getattr(surge, n), str)}


# ---------------------------------------------------------------------------
# every surge.py texture constant, joined the same way the builders do
# ---------------------------------------------------------------------------

def test_every_surge_texture_constant_carries_the_airstack_scheme():
    """Sanity on the fixture itself: if a constant stops using `airstack://`
    (e.g. someone hardcodes an absolute path), the join below would trivially
    "pass" without ever exercising `_join_asset_root` at all."""
    consts = _texture_constants()
    assert len(consts) >= 12, "expected at least the 4 texture families x 3 maps"
    for name, value in consts.items():
        assert value.startswith("airstack://"), (name, value)


def test_every_surge_texture_constant_resolves_to_an_existing_file():
    consts = _texture_constants()
    missing = []
    unexpanded = []
    for name, value in consts.items():
        joined = sg._join_asset_root(value, "")
        if joined.startswith("airstack://") or "://" in joined and not (
                joined.startswith("omniverse://") or joined.startswith("file://")):
            # `_join_asset_root` must have expanded the LOCAL `airstack://`
            # scheme away entirely for a repo-relative path -- if it did
            # not, the raw scheme would reach Hydra exactly as reported.
            unexpanded.append((name, value, joined))
            continue
        if not os.path.isabs(joined):
            unexpanded.append((name, value, joined))
            continue
        if not os.path.exists(joined):
            missing.append((name, value, joined))
    assert not unexpanded, (
        "constant(s) did not expand off the airstack:// scheme: {0}"
        .format(unexpanded))
    assert not missing, (
        "constant(s) joined to a path that does not exist on disk: {0}"
        .format(missing))


# ---------------------------------------------------------------------------
# the actual bug site: planks.skin_material with a raw surge constant
# ---------------------------------------------------------------------------

def test_skin_material_expands_a_raw_surge_texture_constant():
    """This is `washaway.apply_washaway`'s exact call shape for the "wet"
    level: `planks.skin_material(stage, path, surge.SILT_TEXTURE, tint=...)`.
    The authored `diffuse_texture` must be an absolute, existing path with no
    `airstack://` scheme left in it — the pre-fix code authored the raw
    string verbatim."""
    stage = Usd.Stage.CreateInMemory()
    mat = planks.skin_material(stage, "/World/Test/MudLook", surge.SILT_TEXTURE,
                               tint=(0.55, 0.48, 0.38))
    shader = stage.GetPrimAtPath(mat.GetPath().AppendChild("Shader"))
    tex = shader.GetAttribute("inputs:diffuse_texture").Get()
    assert tex is not None, "no diffuse_texture authored at all"
    tex_str = str(tex)
    assert "airstack://" not in tex_str, (
        "the raw airstack:// scheme reached the authored asset path: {0}"
        .format(tex_str))
    resolved = tex_str.strip("@")
    assert os.path.isabs(resolved), resolved
    assert os.path.exists(resolved), resolved


def test_skin_material_with_no_texture_authors_none():
    """`washaway`'s `_slab_material`-style call passes `texture=None` (a
    concrete slab, no cladding photo) — the fix must not force a join on a
    falsy texture."""
    stage = Usd.Stage.CreateInMemory()
    mat = planks.skin_material(stage, "/World/Test/Slab", None)
    shader = stage.GetPrimAtPath(mat.GetPath().AppendChild("Shader"))
    attr = shader.GetAttribute("inputs:diffuse_texture")
    assert not (attr.IsValid() and attr.HasAuthoredValue())


def test_skin_material_passes_through_an_already_absolute_path():
    """`_join_asset_root` must be a no-op for a path that is already
    absolute or a real URL — the fix must not double-prefix or corrupt a
    caller that (unlike `washaway`'s two SILT_TEXTURE call sites) already
    hands over a resolved path."""
    stage = Usd.Stage.CreateInMemory()
    abs_path = sg._join_asset_root(surge.SILT_TEXTURE, "")
    mat = planks.skin_material(stage, "/World/Test/Abs", abs_path)
    shader = stage.GetPrimAtPath(mat.GetPath().AppendChild("Shader"))
    tex = str(shader.GetAttribute("inputs:diffuse_texture").Get()).strip("@")
    assert tex == abs_path


# ---------------------------------------------------------------------------
# ground.overlay_material's texture/normal/orm all resolve too
# ---------------------------------------------------------------------------

def test_overlay_material_resolves_normal_and_orm_paths():
    stage = Usd.Stage.CreateInMemory()
    mat = ground.overlay_material(
        stage, "/World/Test/Overlay", 0.5, 20.0,
        texture=surge.WET_SILT_TEXTURE, normal_tex=surge.WET_SILT_NORMAL_TEXTURE,
        orm_tex=surge.WET_SILT_ORM_TEXTURE)
    shader = stage.GetPrimAtPath(mat.GetPath().AppendChild("Shader"))
    for attr_name in ("diffuse_texture", "normalmap_texture", "ORM_texture"):
        v = str(shader.GetAttribute("inputs:" + attr_name).Get())
        assert "airstack://" not in v, (attr_name, v)
        resolved = v.strip("@")
        assert os.path.isabs(resolved), (attr_name, resolved)
        assert os.path.exists(resolved), (attr_name, resolved)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
