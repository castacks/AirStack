#!/usr/bin/env python3
"""test_quake_ground_materials.py — does `ctx["ground_at"]` (round 5, WP E)
actually steer the buckled-slab/kerb/spill looks in `quake_flow`, and is it
a no-op when absent?

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pyyaml --with pytest python -m pytest \\
        tests/test_quake_ground_materials.py -q

WHY THIS EXISTS
---------------
The user, round 5: "For like bent/broken sidewalk/asphalt use the material
of what's near where the broken ground is placed, not the grassy sidewalk
one." `disaster.ground_class.GroundClass` answers "what's near where" from
`city_layout`; `quake_flow._c_ground_look` is what makes the pavement/kerb/
spill pieces actually USE that answer via `ctx["ground_at"]`, wired through
`_c_ground_response`.

Two things need checking, both host-side (`usd-core`, no Kit/Isaac) — every
material this file touches is authored directly (`damage._pbr`, OmniPBR
`Shader`/`Material` prims defined in place), never a Nucleus REFERENCE, so a
real in-memory `Usd.Stage` with no Nucleus resolves them exactly as the bake
and the city would:

  * `_c_ground_look` itself, in isolation: with `ctx["ground_at"]` present it
    calls `ground_class.look_for` and NEVER touches the caller's own rng
    coin; without it, it calls the coin (or `default_key`) exactly as every
    call site did before this existed.
  * `_c_ground_response` end to end, on a stub mass `m`: a tilt authored with
    `ground_at` returning "road" everywhere binds every buckled pavement
    slab (`_c_pave_break`) and every kerb segment (`_c_kerb`) to the
    "asph" look; "grass" everywhere binds them to "soil"; either way the
    HEAVE WEDGE itself (the soil ribbon `_c_heave` authors) stays "soil" —
    the piece that represents the earth is not what changed. Without
    `ground_at` at all, kerb segments stay unconditionally "pave" (their
    historical, un-randomised look) and slabs keep drawing the historical
    pave/asph coin — i.e. `ctx` with no `"ground_at"` key authors exactly
    what it always did.
"""
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import ground_class, quake_flow as qf         # noqa: E402

from pxr import Usd, UsdGeom, UsdShade                       # noqa: E402


PARENT = "/World/Bldg"


def _new_stage():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, PARENT)
    return st


def _mass(**over):
    m = {"cx": 0.0, "cy": 0.0, "yaw": 0.0, "W": 24.0, "D": 16.0,
         "z0": 0.0, "top": 12.0, "levels": [0.0], "module": 4.0}
    m.update(over)
    return m


def _look_key(stage, path):
    """The `_C_TEX` key name (e.g. "asph") a prim at *path* is bound to, via
    the `<parent>/QuakeLooks/c_<key>` naming `_c_look` always uses, or None
    if unbound / the prim doesn't exist."""
    pr = stage.GetPrimAtPath(path)
    if not pr or not pr.IsValid():
        return None
    mat = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
    if not mat or not mat.GetPrim().IsValid():
        return None
    name = mat.GetPath().name
    return name[2:] if name.startswith("c_") else name


def _paths_under(stage, root, name_contains):
    """Every prim under *root* whose own name contains *name_contains* —
    e.g. "slab" matches `_c_pave_break`'s `<tag>_slab_<recipe-tag>_<uid>`
    naming whatever the caller's own `tag` prefix was."""
    root_pr = stage.GetPrimAtPath(root)
    out = []
    if not root_pr or not root_pr.IsValid():
        return out
    for pr in Usd.PrimRange(root_pr):
        if name_contains in pr.GetName():
            out.append(str(pr.GetPath()))
    return out


# ---------------------------------------------------------------------------
# `_c_ground_look` in isolation
# ---------------------------------------------------------------------------

def test_ground_look_absent_calls_the_coin_and_uses_default_key():
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    calls = []

    def coin():
        calls.append(1)
        return "asph"

    ctx = {"stage": st, "parent": PARENT, "mats": mats, "tag": "t",
           "n_uid": 0}
    got = qf._c_ground_look(ctx, 1.0, 2.0, None, coin)
    assert len(calls) == 1                       # the coin WAS drawn
    assert got is mats.get("c_asph") or got is not None
    assert _look_key_of_material(got) == "asph"

    # and with no coin at all, the plain default is used, no draw needed
    got2 = qf._c_ground_look(ctx, 1.0, 2.0, "pave")
    assert _look_key_of_material(got2) == "pave"


def test_ground_look_present_never_calls_the_coin():
    st = _new_stage()
    mats = qf.materials(st, PARENT)

    def coin():
        raise AssertionError("the coin must not be drawn when ground_at is set")

    ctx = {"stage": st, "parent": PARENT, "mats": mats, "tag": "t",
           "n_uid": 0, "ground_at": lambda x, y: "road"}
    got = qf._c_ground_look(ctx, 5.0, 5.0, None, coin)
    assert _look_key_of_material(got) == "asph"

    ctx["ground_at"] = lambda x, y: "grass"
    got2 = qf._c_ground_look(ctx, 5.0, 5.0, "pave", coin)
    assert _look_key_of_material(got2) == "soil"

    ctx["ground_at"] = lambda x, y: "sidewalk"
    assert _look_key_of_material(
        qf._c_ground_look(ctx, 0.0, 0.0, "asph", coin)) == "pave"

    ctx["ground_at"] = lambda x, y: "paved"
    assert _look_key_of_material(
        qf._c_ground_look(ctx, 0.0, 0.0, "soil", coin)) == "pave"


def _look_key_of_material(mat):
    if mat is None:
        return None
    name = mat.GetPath().name
    return name[2:] if name.startswith("c_") else name


# ---------------------------------------------------------------------------
# `_c_ground_response`, end to end
# ---------------------------------------------------------------------------

def _run_tilt(ground_at, seed=7, scope_name="g"):
    st = _new_stage()
    scope = "{0}/{1}".format(PARENT, scope_name)
    mats = qf.materials(st, PARENT)
    m = _mass()
    rng = random.Random(seed)
    out = qf._c_ground_response(
        st, m, low_side="S", drop_m=1.4, rise_m=0.7, parent=scope,
        mats=mats, rng=rng, tag=scope_name, ground_at=ground_at)
    assert out["low"] == "S"
    slabs = _paths_under(st, scope, "slab")
    kerbs = _paths_under(st, scope, "kerb")
    heaves = _paths_under(st, scope, "heave")
    assert slabs, "no buckled pavement slabs were authored — nothing to test"
    assert kerbs, "no kerb segments were authored — nothing to test"
    assert heaves, "no heave wedge was authored — nothing to test"
    return {
        "slab_keys": {_look_key(st, p) for p in slabs},
        "kerb_keys": {_look_key(st, p) for p in kerbs},
        "heave_keys": {_look_key(st, p) for p in heaves},
    }


def test_ground_response_road_everywhere_makes_asphalt_slabs_and_kerbs():
    res = _run_tilt(lambda x, y: "road")
    assert res["slab_keys"] == {"asph"}
    assert res["kerb_keys"] == {"asph"}
    # the earth wedge itself is unaffected — it is soil, not a road surface
    assert res["heave_keys"] == {"soil"}


def test_ground_response_grass_everywhere_keeps_slabs_and_kerbs_soil():
    res = _run_tilt(lambda x, y: "grass")
    assert res["slab_keys"] == {"soil"}
    assert res["kerb_keys"] == {"soil"}
    assert res["heave_keys"] == {"soil"}


def test_ground_response_without_ground_at_matches_historical_coin():
    # No `ground_at` at all: the kerb is UNCONDITIONALLY "pave" (it never
    # drew a coin, even before this module existed), and the slabs draw the
    # historical pave/asph coin — never "soil", never anything ground-class
    # driven.
    res = _run_tilt(None)
    assert res["kerb_keys"] == {"pave"}
    assert res["slab_keys"] <= {"pave", "asph"}
    assert res["heave_keys"] == {"soil"}


def test_ground_response_ground_at_is_deterministic():
    a = _run_tilt(lambda x, y: "road", seed=42, scope_name="a")
    b = _run_tilt(lambda x, y: "road", seed=42, scope_name="b")
    assert a == b


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
