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

ROUND 5, SECOND PASS ("don't use Worn Pavement material anywhere ... use a
mixture of Crushed Asphalt Ground, Damaged Asphalt (there's 2 types)" / "make
the cracks in ground larger" / "use the same texture as dirt mounds in
tornado suburb", user, 2026-08-31) added the tests below it:

  * `_C_TEX["pave"/"asph"/"raft"]` are now 3-way MIXTURES, so `_c_look`'s
    material name gained a `_<index>` suffix (`_look_family` strips it back
    for the assertions above, which only ever cared about the family);
  * nothing under a quake ground response may still reference
    `Worn_Pavement.usda`, checked over the AUTHORED bindings, not just the
    mats table;
  * the mixture actually mixes across buildings (crc32 of `ctx["tag"]`);
  * `C_FISSURE_M` / `_c_fissures`'s default `width` scale by the module-level
    `FISSURE_SCALE` (env `EQ_FISSURE_SCALE`, default 1.75);
  * `_C_TEX["soil"]`/`["silt"]` are IMPORTED from `scour_relief._TEX`, not
    retyped, so the two disasters' mud cannot drift apart again.
"""
import os
import random
import subprocess
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


def _look_family(key):
    """Strip a mixture-variant suffix back to the family name.

    `_c_look` names a tuple-valued `_C_TEX` entry's material "<key>_<index>"
    (e.g. "asph_2") so neighbouring buildings can draw different source
    textures under the same family (round 5: "use a mixture of Crushed
    Asphalt Ground, Damaged Asphalt (there's 2 types)"). A single-rel key
    (e.g. "soil") has no such suffix and passes through unchanged."""
    if key is None:
        return None
    if "_" in key:
        base, _, tail = key.rpartition("_")
        if base and tail.isdigit():
            return base
    return key


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
    assert got is not None
    # "asph" is now a MIXTURE (round 5: "use a mixture of Crushed Asphalt
    # Ground, Damaged Asphalt (there's 2 types)"), so the material is named
    # "c_asph_<index>", not "c_asph" — the FAMILY still has to be "asph".
    assert _look_family(_look_key_of_material(got)) == "asph"

    # and with no coin at all, the plain default is used, no draw needed
    got2 = qf._c_ground_look(ctx, 1.0, 2.0, "pave")
    assert _look_family(_look_key_of_material(got2)) == "pave"


def test_ground_look_present_never_calls_the_coin():
    st = _new_stage()
    mats = qf.materials(st, PARENT)

    def coin():
        raise AssertionError("the coin must not be drawn when ground_at is set")

    ctx = {"stage": st, "parent": PARENT, "mats": mats, "tag": "t",
           "n_uid": 0, "ground_at": lambda x, y: "road"}
    got = qf._c_ground_look(ctx, 5.0, 5.0, None, coin)
    assert _look_family(_look_key_of_material(got)) == "asph"

    ctx["ground_at"] = lambda x, y: "grass"
    got2 = qf._c_ground_look(ctx, 5.0, 5.0, "pave", coin)
    assert _look_family(_look_key_of_material(got2)) == "soil"

    ctx["ground_at"] = lambda x, y: "sidewalk"
    assert _look_family(_look_key_of_material(
        qf._c_ground_look(ctx, 0.0, 0.0, "asph", coin))) == "pave"

    ctx["ground_at"] = lambda x, y: "paved"
    assert _look_family(_look_key_of_material(
        qf._c_ground_look(ctx, 0.0, 0.0, "soil", coin))) == "pave"


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


def _families(keys):
    return {_look_family(k) for k in keys}


def test_ground_response_road_everywhere_makes_asphalt_slabs_and_kerbs():
    res = _run_tilt(lambda x, y: "road")
    assert _families(res["slab_keys"]) == {"asph"}
    assert _families(res["kerb_keys"]) == {"asph"}
    # the earth wedge itself is unaffected — it is soil, not a road surface
    assert _families(res["heave_keys"]) == {"soil"}


def test_ground_response_grass_everywhere_keeps_slabs_and_kerbs_soil():
    res = _run_tilt(lambda x, y: "grass")
    assert _families(res["slab_keys"]) == {"soil"}
    assert _families(res["kerb_keys"]) == {"soil"}
    assert _families(res["heave_keys"]) == {"soil"}


def test_ground_response_without_ground_at_matches_historical_coin():
    # No `ground_at` at all: the kerb is UNCONDITIONALLY "pave" (it never
    # drew a coin, even before this module existed), and the slabs draw the
    # historical pave/asph coin — never "soil", never anything ground-class
    # driven.
    res = _run_tilt(None)
    assert _families(res["kerb_keys"]) == {"pave"}
    assert _families(res["slab_keys"]) <= {"pave", "asph"}
    assert _families(res["heave_keys"]) == {"soil"}


def test_ground_response_ground_at_is_deterministic():
    # SAME tag both times: the mixture variant is picked from a crc32 of
    # `ctx["tag"]` (round 5), so two DIFFERENT tags legitimately draw
    # different source textures under the same seed — that is the "different
    # buildings differ" feature, not nondeterminism. The two runs still land
    # on separate stages (`_run_tilt` builds its own `_new_stage()` each
    # call), so reusing "a" for both authors into independent namespaces.
    a = _run_tilt(lambda x, y: "road", seed=42, scope_name="a")
    b = _run_tilt(lambda x, y: "road", seed=42, scope_name="a")
    assert a == b


# ---------------------------------------------------------------------------
# ROUND 5, SECOND PASS: no Worn_Pavement, the asphalt mixture, larger
# fissures, the shared soil/silt table
# ---------------------------------------------------------------------------

def _bound_reference_asset_paths(stage, prim_path):
    """Every `assetPath` string a REFERENCE arc on the material bound to the
    prim at `prim_path` carries (e.g. ".../Damaged_Asphalt.usda"), or []."""
    pr = stage.GetPrimAtPath(prim_path)
    if not pr or not pr.IsValid():
        return []
    mat = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
    if not mat or not mat.GetPrim().IsValid():
        return []
    op = mat.GetPrim().GetMetadata("references")
    if not op:
        return []
    return [str(r.assetPath) for r in op.GetAddedOrExplicitItems()]


def test_no_ground_piece_binds_worn_pavement():
    """Over a tilt, a settlement AND `_buckled_pavement` (the one ground
    call site that bound the shared `mats["concrete"]` directly instead of
    going through `_c_look`/`_c_ground_look`) — nowhere does an authored
    ground piece's bound material reference `Worn_Pavement.usda`, with or
    without a `ground_at` sampler wired in."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    rng = random.Random(11)
    root = "{0}/wp".format(PARENT)
    qf._c_ground_response(st, m, low_side="S", drop_m=1.6, rise_m=0.8,
                          parent=root + "/tilt", mats=mats, rng=rng,
                          tag="wptilt", ground_at=lambda x, y: "road")
    qf._c_ground_response(st, m, sink_m=0.9, parent=root + "/settle",
                          mats=mats, rng=rng, tag="wpsettle")
    ctx = qf._c_ctx(st, root + "/kerb", mats, rng, tag="wpkerb")
    ctx["ground_at"] = lambda x, y: "sidewalk"
    qf._buckled_pavement(ctx, m, 8)

    root_pr = st.GetPrimAtPath(root)
    checked, bad = 0, []
    for pr in Usd.PrimRange(root_pr):
        mat = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        checked += 1
        for ref in _bound_reference_asset_paths(st, pr.GetPath()):
            if "Worn_Pavement" in ref:
                bad.append((str(pr.GetPath()), ref))
    assert checked > 20, "too few bound pieces authored to be a real check"
    assert not bad, "ground pieces still bind Worn_Pavement: {0}".format(bad)


def test_buckled_pavement_follows_ground_at():
    """The one ground call site this round moved off the shared
    `mats["concrete"]` directly onto `_c_ground_look` — same idiom `_c_kerb`
    and `_c_lip_slabs` already used."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT + "/bpg", mats, random.Random(5), tag="bpg")
    ctx["ground_at"] = lambda x, y: "grass"
    made = qf._buckled_pavement(ctx, m, 8)
    assert made
    assert _families({_look_key(st, p) for p in made}) == {"soil"}

    st2 = _new_stage()
    mats2 = qf.materials(st2, PARENT)
    ctx2 = qf._c_ctx(st2, PARENT + "/bpr", mats2, random.Random(5), tag="bpr")
    ctx2["ground_at"] = lambda x, y: "road"
    made2 = qf._buckled_pavement(ctx2, m, 8)
    assert made2
    assert _families({_look_key(st2, p) for p in made2}) == {"asph"}


def test_asphalt_mixture_draws_at_least_two_variants():
    """"Use a mixture ... " has to mean more than one source texture actually
    gets used, not a `_C_TEX` tuple nothing ever indexes past 0. Different
    building TAGS pick their mixture variant from a crc32 of `ctx["tag"]`
    (round 5), so a spread of tags should turn up at least 2 of the 3 asphalt
    sources somewhere in a city-sized sample."""
    variants = set()
    for i in range(15):
        st = _new_stage()
        mats = qf.materials(st, PARENT)
        m = _mass()
        tag = "mix{0}".format(i)
        qf._c_ground_response(st, m, low_side="S", drop_m=1.3, rise_m=0.6,
                              parent="{0}/{1}".format(PARENT, tag),
                              mats=mats, rng=random.Random(i), tag=tag)
        for p in _paths_under(st, "{0}/{1}".format(PARENT, tag), "slab"):
            k = _look_key(st, p)
            if _look_family(k) in ("pave", "asph"):
                variants.add(k)
    assert len(variants) >= 2, (
        "the asphalt mixture only ever drew one variant across 15 buildings: "
        "{0}".format(variants))


def test_fissure_knobs_scaled_up_and_env_overridable():
    """Round 5: "make the cracks in ground larger" — `FISSURE_SCALE` sits in
    the requested 1.5-2x band by default, and is reachable from the
    environment without a code edit (a fresh interpreter, since the module
    reads `EQ_FISSURE_SCALE` once at import time)."""
    assert 1.5 <= qf.FISSURE_SCALE <= 2.0
    assert qf.C_FISSURE_M == (2.0 * qf.FISSURE_SCALE, 6.0 * qf.FISSURE_SCALE)

    code = (
        "import sys; sys.path.insert(0, {0!r});"
        "from disaster import quake_flow as qf;"
        "print(qf.FISSURE_SCALE, qf.C_FISSURE_M)"
    ).format(os.path.normpath(os.path.join(_HERE, "..")))
    env = dict(os.environ, EQ_FISSURE_SCALE="2.0")
    out = subprocess.run([sys.executable, "-c", code], env=env,
                         capture_output=True, text=True, check=True).stdout
    assert "2.0 (4.0, 12.0)" in out, out


def test_fissures_use_soil_look_not_a_flat_crack_color():
    """"Are you using the mud texture? It looks a little odd for the cracks
    in the ground ... use the same texture we're using for dirt mounds in
    tornado suburb" — a fissure box now binds the soil/silt `_c_look`
    texture family, not the flat, textureless `crack` colour (which stays
    reserved for the dark VOID of an opened gap, `_c_gap`)."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    made = qf._c_fissures({"stage": st, "parent": PARENT, "mats": mats,
                           "tag": "fis", "rng": random.Random(3), "n_uid": 0,
                           "authored": []}, m)
    assert made
    families = _families({_look_key(st, p) for p in made})
    assert families <= {"soil", "silt"}
    assert families, "no fissure box resolved to a _c_look material at all"


def test_quake_soil_and_silt_match_scour_relief_field_for_field():
    """The regression diagnosis: by the time this test was written the two
    tables already agreed field-for-field (same file, tint, roughness,
    repeats, desaturation) — this pins that down as an executable fact
    instead of a comment, and the import in `quake_flow` (`_C_TEX["soil"]`/
    `["silt"]` ARE `scour_relief._TEX["soil"]`/`["silt"]`, not a copy of the
    same numbers) means they cannot drift apart again regardless."""
    from disaster import scour_relief as sr
    assert qf._C_TEX["soil"] == sr._TEX["soil"]
    assert qf._C_TEX["silt"] == sr._TEX["silt"]
    assert qf._C_TEX["soil"] is sr._TEX["soil"]     # imported, not retyped


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
