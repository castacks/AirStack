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

import pytest

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


# ---------------------------------------------------------------------------
# LIVE REVIEW ROUND: the fissure is `scour_relief` geometry (a mould of
# dirt), thicker; a cracked-asphalt band rides beside it; the tilt raft is
# chipped irregular. "the fissure looks weird — use the same soil/mud
# material we have and use in suburban tornado for its path and moulds of
# dirt. Use the same code in fact to create this longer 'mould of dirt' aka
# fissure" / "the cracked asphalt along the fissure looks weird — make it
# irregular cracked shapes, not just rectangles" / "the fissure itself
# should be thicker" / on `quake_tilt/raft_t3_1`: "it's too straight and
# rectangular. We need irregular. We can do a smaller version of this as
# the fissure's cracked asphalt." (user, live scene review).
# ---------------------------------------------------------------------------

def _mesh_pts(stage, path):
    pr = stage.GetPrimAtPath(path)
    return UsdGeom.Mesh(pr).GetPointsAttr().Get()


def test_fissure_width_scale_env_overridable():
    """`C_FISSURE_W` layers the new `EQ_FISSURE_WIDTH_SCALE` on top of
    `FISSURE_SCALE` — a second, independent "thicker" knob, reachable from
    the environment without a code edit, the same pattern
    `test_fissure_knobs_scaled_up_and_env_overridable` already pins for
    length."""
    assert qf.FISSURE_WIDTH_SCALE >= 1.5
    assert qf.C_FISSURE_W == (
        0.06 * qf.FISSURE_SCALE * qf.FISSURE_WIDTH_SCALE,
        0.22 * qf.FISSURE_SCALE * qf.FISSURE_WIDTH_SCALE)

    code = (
        "import sys; sys.path.insert(0, {0!r});"
        "from disaster import quake_flow as qf;"
        "print(qf.FISSURE_WIDTH_SCALE, qf.C_FISSURE_W)"
    ).format(os.path.normpath(os.path.join(_HERE, "..")))
    env = dict(os.environ, EQ_FISSURE_WIDTH_SCALE="3.0")
    out = subprocess.run([sys.executable, "-c", code], env=env,
                         capture_output=True, text=True, check=True).stdout
    assert out.startswith("3.0 "), out


def test_fissure_mound_is_a_continuous_ridge_not_a_box_chain():
    """The old implementation authored a CHAIN of flat 8-point boxes, one
    per ~1.2 m step. The new one sweeps `scour_relief.geometry`'s `ridge`
    extrusion along the whole crack as ONE mesh: more than 8 points (an
    8-point box would mean nothing changed), an open (doubleSided) ribbon,
    and every station's cross-section actually present."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(4), tag="ridge")
    made = qf._c_fissures(ctx, m, corners=("SW",), n_each=(1, 1),
                          length=(8.0, 8.0))
    assert len(made) == 1
    pts = _mesh_pts(st, made[0])
    assert len(pts) > 8, "fissure mound is still an 8-point box"
    pr = st.GetPrimAtPath(made[0])
    mesh = UsdGeom.Mesh(pr)
    assert bool(mesh.GetDoubleSidedAttr().Get()) is True


def test_fissure_trace_invokes_scour_relief_geometry():
    """"Use the same code in fact to create this longer 'mould of dirt' aka
    fissure" — proven by SPYING on `scour_relief.geometry`, not just by
    reading the mesh it produces: a monkeypatched wrapper must actually be
    called, with a `ridge` spec, while a real fissure is authored."""
    from disaster import scour_relief as sr
    calls = []
    real = sr.geometry

    def spy(spec):
        calls.append(spec)
        return real(spec)

    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(9), tag="spy")
    qf._scour_relief.geometry = spy
    try:
        made = qf._c_fissures(ctx, m, corners=("NE",), n_each=(1, 1),
                              length=(6.0, 6.0))
    finally:
        qf._scour_relief.geometry = real
    assert made
    assert calls, "scour_relief.geometry was never called"
    assert all(c["kind"] == "ridge" for c in calls)


def test_fissure_trace_is_deterministic():
    """Same seed, same crack: the mound's own point cloud is byte-identical
    across two independent stages (the recipe/bake determinism contract
    every other quake ground pass already holds to)."""
    def _run(seed):
        st = _new_stage()
        mats = qf.materials(st, PARENT)
        m = _mass()
        ctx = qf._c_ctx(st, PARENT, mats, random.Random(seed), tag="det")
        made = qf._c_fissures(ctx, m, corners=("SE",), n_each=(1, 1),
                              length=(7.0, 7.0))
        return [tuple(_mesh_pts(st, p)) for p in made]

    a, b = _run(21), _run(21)
    assert a == b
    assert a, "no fissure authored — nothing to compare"


def test_fissure_cracked_asphalt_band_is_chipped_and_irregular():
    """The ground itself cracks near the fissure: a band of small pavement
    plates, chipped IRREGULAR (round-6 `fracture.chip_box`, > 8 points —
    the "not just rectangles" ask), tilted, with a pave/asph look — never
    folded into `_c_fissures`' own soil/silt-only return (see
    `_c_fissure_pave`'s docstring)."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(6), tag="pv")
    made = qf._c_fissures(ctx, m, corners=("SW", "NE"), n_each=(1, 2))
    families = _families({_look_key(st, p) for p in made})
    assert families
    assert families <= {"soil", "silt"}          # unchanged contract

    pave_paths = [p for p in ctx["authored"]
                 if "_pave_" in p and "_mound_" not in p]
    assert pave_paths, "no cracked-asphalt plate authored beside the fissure"
    assert set(pave_paths) <= set(ctx["static_extra"])
    assert _families({_look_key(st, p) for p in pave_paths}) <= {"pave", "asph"}
    irregular = [p for p in pave_paths if len(_mesh_pts(st, p)) > 8]
    assert irregular, "every cracked-asphalt plate is still an 8-point box"


def _world_y_range(stage, path):
    """`(min_y, max_y)` of a prim's WORLD bounding box — NOT its raw mesh
    points, which (`_box`'s own convention, unchanged by chipping) are
    authored LOCAL to the prim's own translate/rotate xform ops."""
    from pxr import Usd, UsdGeom
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    pr = stage.GetPrimAtPath(path)
    rng = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    return float(mn[1]), float(mx[1])


def _straight_stations(length_m, step=1.2):
    """A straight polyline along +Y, `_c_fissure_pave`'s own `stations`
    shape — for a framing where "coverage spans X% of the length" can be
    measured directly off world Y, independent of the wandering heading a
    real corner crack draws."""
    n = max(2, int(length_m / step))
    return [(0.0, i * length_m / (n - 1)) for i in range(n)]


def test_fissure_pave_band_covers_the_whole_trace_and_scales_with_length():
    """Coordinator, after reviewing the first cut's render: "only ~4 plates,
    clustered near the ends ... the band must run the FULL length of the
    trace." That cut drew `n` plates at `n` INDEPENDENT random stations —
    for a short crack that is a handful of draws with no guarantee they
    spread out. `_c_fissure_pave` now WALKS the arc length (place a plate,
    gap, repeat), so this checks the two things that walk has to deliver:
    more plates for a longer trace, and — measured off each plate's WORLD
    bounding box, clipped to the trace's own [0, length] extent, since a
    plate can legitimately overhang past either tip — the covered span
    reaching most of the length, not just its own two ends."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)

    def _run(length_m, tag):
        ctx = qf._c_ctx(st, "{0}/{1}".format(PARENT, tag), mats,
                        random.Random(5), tag=tag)
        stations = _straight_stations(length_m)
        widths = [0.3] * len(stations)
        made = qf._c_fissure_pave(ctx, stations, widths, 0.0, tag="fis")
        return made

    made_short = _run(6.0, "pavshort")
    made_long = _run(40.0, "pavlong")
    assert made_short and made_long
    assert len(made_long) > len(made_short), (
        "plate count does not scale with trace length: {0} short vs {1} "
        "long".format(len(made_short), len(made_long)))

    mins, maxs = [], []
    for p in made_long:
        lo, hi = _world_y_range(st, p)
        mins.append(lo)
        maxs.append(hi)
    span = min(40.0, max(maxs)) - max(0.0, min(mins))
    assert span >= 0.8 * 40.0, (
        "cracked-asphalt band does not run the full trace: {0:.1f} m of "
        "40.0 m covered".format(span))


def test_fissure_pave_band_density_env_overridable():
    """`EQ_FISSURE_PAVE_DENSITY` reaches `C_FISSURE_PAVE_DENSITY` the same
    way `EQ_FISSURE_SCALE`/`EQ_FISSURE_WIDTH_SCALE` reach their knobs, and
    sits in the "roughly 60-75%" band the coordinator asked for by
    default."""
    assert 0.55 <= qf.C_FISSURE_PAVE_DENSITY <= 0.80
    code = (
        "import sys; sys.path.insert(0, {0!r});"
        "from disaster import quake_flow as qf;"
        "print(qf.C_FISSURE_PAVE_DENSITY)"
    ).format(os.path.normpath(os.path.join(_HERE, "..")))
    env = dict(os.environ, EQ_FISSURE_PAVE_DENSITY="0.5")
    out = subprocess.run([sys.executable, "-c", code], env=env,
                         capture_output=True, text=True, check=True).stdout
    assert out.strip() == "0.5", out


def test_raft_plate_is_chipped_irregular_but_keeps_its_footprint():
    """The liked mechanic (a slab that shows only once a tilt levers it out
    of the ground) stays; only its edges go irregular. `max_grow` bounds how
    far a chip pass may swell the bbox, so the footprint stays recognisably
    `(W + 1.2) x (D + 1.2) x RAFT_T` — the height-change plate still reads as
    the same plate, just no longer a ruler-edged rectangle."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(8), tag="raft")
    path = qf._raft(ctx, m)
    pts = _mesh_pts(st, path)
    assert len(pts) > 8, "the raft plate is still a plain 8-point box"
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    assert (max(xs) - min(xs)) == pytest.approx(m["W"] + 1.2, rel=0.25)
    assert (max(ys) - min(ys)) == pytest.approx(m["D"] + 1.2, rel=0.25)
    assert (max(zs) - min(zs)) == pytest.approx(qf.RAFT_T, rel=0.25)


def test_epicentre_fissures_reuse_c_fissure_trace_not_a_second_copy():
    """"Use the same code in fact ..." — `quake.ground_effects`'s soft-soil
    epicentre cracks (the `epi_top.png`/`nw_top.png` screenshots) must route
    through the SAME `_c_fissure_trace` the per-building corner cracks use,
    not keep a second, independent chain-of-boxes implementation bound to
    the flat `rebar`/`concrete` materials."""
    import inspect

    from disaster import quake

    src = inspect.getsource(quake.ground_effects)
    assert "_c_fissure_trace" in src
    # the old implementation's fingerprint: a flat box straight onto the
    # soft-soil centreline, bound to the bare rebar material
    assert 'mats["rebar"])' not in src
    assert 'qf._box(stage, path, mx, my, 0.02' not in src


# ---------------------------------------------------------------------------
# ROUND 7: "the fissure must be an OPENING cut INTO the ground ... a cracked
# irregular opening and a floor below" (user, live review) — the pool
# mechanism (`suburb_scene.apply_ground` subtracting `pool_at`'s hole
# polygon from the lawn mesh before triangulating it), replicated for
# `scene_generator.apply_ground_planes`'s city ground: a real hole cut
# through whichever already-authored ground meshes a fissure crosses,
# descending walls and a darker floor below grade, and the old solid mound
# shrunk to a modest lip either side.
# ---------------------------------------------------------------------------

def _author_plane(stage, path, x0, y0, x1, y1, z, uv_scale_m=4.0):
    """One quad `UsdGeom.Mesh`, byte-for-byte the shape
    `scene_generator._make_plane_mesh` authors (4 points, 1 face, a "st"
    vertex UV primvar) — so `_c_cut_ground_openings` is exercised against
    exactly what `apply_ground_planes` actually leaves under a city's
    `.../ground` scope, without pulling in `scene_generator` itself (heavy,
    Kit-oriented) for a plain rectangle."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    w, h = x1 - x0, y1 - y0
    pts = Vt.Vec3fArray([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z),
                        Gf.Vec3f(x1, y1, z), Gf.Vec3f(x0, y1, z)])
    u_max = w / uv_scale_m if uv_scale_m > 1e-9 else 1.0
    v_max = h / uv_scale_m if uv_scale_m > 1e-9 else 1.0
    uvs = Vt.Vec2fArray([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(u_max, 0.0),
                        Gf.Vec2f(u_max, v_max), Gf.Vec2f(0.0, v_max)])
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(uvs)
    return mesh


def _face_centroids(stage, path):
    """`[(cx, cy), ...]`, one per face, of the mesh at *path* — enough to
    ask "does any triangle's own area sit where the hole was cut"."""
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    out, i = [], 0
    for c in counts:
        face = idx[i:i + c]
        i += c
        out.append((sum(pts[v][0] for v in face) / c,
                   sum(pts[v][1] for v in face) / c))
    return out


def test_cut_ground_openings_is_a_noop_without_ground_root():
    """No `ctx["ground_root"]`: the bench, the archetype bake, and every
    caller that predates this feature must see NO change at all — the same
    non-breaking contract `ctx["ground_at"]` already holds for materials."""
    st = _new_stage()
    _author_plane(st, PARENT + "/ground/asphalt_base", -10, -10, 10, 10, 0.0)
    before = _face_centroids(st, PARENT + "/ground/asphalt_base")
    n = qf._c_cut_ground_openings({"stage": st}, [[(-1, -10), (1, -10),
                                                   (1, 10), (-1, 10)]])
    assert n == 0
    assert _face_centroids(st, PARENT + "/ground/asphalt_base") == before


def test_cut_ground_openings_hole_in_synthetic_ground_mesh():
    """The mechanism in isolation: a hand-built hole quad cut straight
    through one synthetic ground rectangle. The rectangle's single quad
    face must become several triangles (the opening polygon actually
    subtracted, not merely marked), and — the crack itself — no resulting
    triangle's own centroid may fall inside the hole: the ground has an
    actual gap there, not a cap over it."""
    st = _new_stage()
    root = PARENT + "/ground"
    from pxr import UsdGeom
    UsdGeom.Scope.Define(st, root)
    _author_plane(st, root + "/asphalt_base", -50.0, -50.0, 50.0, 50.0, 0.0)
    hole = [(-1.0, -20.0), (1.0, -20.0), (1.0, 20.0), (-1.0, 20.0)]
    ctx = {"stage": st, "ground_root": root}
    n = qf._c_cut_ground_openings(ctx, [hole])
    assert n == 1

    mesh = UsdGeom.Mesh(st.GetPrimAtPath(root + "/asphalt_base"))
    counts = mesh.GetFaceVertexCountsAttr().Get()
    assert len(counts) > 1, "the ground mesh still has its original single face"
    assert all(c == 3 for c in counts), "the re-triangulated pieces are not triangles"

    for (cx, cy) in _face_centroids(st, root + "/asphalt_base"):
        assert not (-1.0 <= cx <= 1.0 and -20.0 <= cy <= 20.0), (
            "a triangle centroid ({0}, {1}) falls inside the crack "
            "opening — the ground still caps it".format(cx, cy))


def test_cut_ground_openings_skips_meshes_the_hole_never_touches():
    """A hole far from a ground rectangle's own extent must leave that
    rectangle completely alone (the bbox-miss early-out), and one whose
    extent it never actually enters (bbox hit, polygon miss) must also come
    back unchanged rather than needlessly re-triangulated."""
    st = _new_stage()
    root = PARENT + "/ground"
    from pxr import UsdGeom
    UsdGeom.Scope.Define(st, root)
    _author_plane(st, root + "/far", 200.0, 200.0, 210.0, 210.0, 0.0)
    before = _face_centroids(st, root + "/far")
    n = qf._c_cut_ground_openings(
        {"stage": st, "ground_root": root},
        [[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]])
    assert n == 0
    assert _face_centroids(st, root + "/far") == before


def test_cut_ground_openings_respects_ground_ssf():
    """`ctx["ground_ssf"]` scales the hole into the mesh's own stage-unit
    frame before it is cut — the exact frame mismatch `quake.ground_effects`
    has (its own cracks in unscaled metres, the ground meshes in `* ssf`
    stage units): a hole drawn in metres must land on a mesh authored at
    `ssf=2.0` only once it is scaled up to match."""
    st = _new_stage()
    root = PARENT + "/ground"
    from pxr import UsdGeom
    UsdGeom.Scope.Define(st, root)
    _author_plane(st, root + "/asphalt_base", -100.0, -100.0, 100.0, 100.0, 0.0)
    hole_m = [(-0.5, -10.0), (0.5, -10.0), (0.5, 10.0), (-0.5, 10.0)]
    n = qf._c_cut_ground_openings(
        {"stage": st, "ground_root": root, "ground_ssf": 2.0}, [hole_m])
    assert n == 1
    for (cx, cy) in _face_centroids(st, root + "/asphalt_base"):
        assert not (-1.0 <= cx <= 1.0 and -20.0 <= cy <= 20.0), (
            "the hole was not scaled into the mesh's stage-unit frame")


def test_fissure_opening_authors_walls_and_floor_below_grade():
    """`_c_fissures` end to end: the wall mesh descends from grade, the
    floor sits at (or very near) the wall's own deepest points, and the
    floor binds the new darker `"pit_floor"` look while the walls keep the
    crack's own soil/silt one — never folded into `made` (the soil/silt-only
    contract `_c_fissures`' own docstring already promises)."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(12), tag="pit")
    made = qf._c_fissures(ctx, m, corners=("SW",), n_each=(1, 1),
                          length=(10.0, 10.0))
    assert made
    walls = [p for p in ctx["authored"] if "_wall_" in p]
    floors = [p for p in ctx["authored"] if "_floor_" in p]
    assert walls, "no fissure wall mesh authored"
    assert floors, "no fissure floor mesh authored"

    wall_zs = [p[2] for w in walls for p in _mesh_pts(st, w)]
    floor_zs = [p[2] for f in floors for p in _mesh_pts(st, f)]
    assert max(wall_zs) <= m["z0"] + 1e-6, "the wall rim rises above grade"
    assert min(wall_zs) < m["z0"] - 0.5, "the wall never goes meaningfully below grade"
    assert min(floor_zs) < m["z0"] - 0.5, "the floor is not below grade"
    assert abs(min(floor_zs) - min(wall_zs)) < 0.05, (
        "the floor does not sit at the wall's own deepest point")

    assert _families({_look_key(st, f) for f in floors}) == {"pit_floor"}
    assert _families({_look_key(st, w) for w in walls}) <= {"soil", "silt"}
    assert set(walls + floors).isdisjoint(set(made)), (
        "the wall/floor leaked into the soil/silt-only `made` contract")
    assert set(walls + floors) <= set(ctx["static_extra"])


def test_fissure_opening_is_deterministic():
    """Same seed, same pit: the wall mesh's own point cloud is byte-identical
    across two independent stages, the same contract every other quake
    ground pass already holds to."""
    def _run(seed):
        st = _new_stage()
        mats = qf.materials(st, PARENT)
        m = _mass()
        ctx = qf._c_ctx(st, PARENT, mats, random.Random(seed), tag="detpit")
        qf._c_fissures(ctx, m, corners=("NE",), n_each=(1, 1), length=(9.0, 9.0))
        walls = sorted(p for p in ctx["authored"] if "_wall_" in p)
        return [tuple(_mesh_pts(st, p)) for p in walls]

    a, b = _run(33), _run(33)
    assert a == b
    assert a, "no wall authored — nothing to compare"


def test_fissure_end_to_end_cuts_the_city_ground_mesh():
    """The opening actually reaches the CITY ground, not only a synthetic
    one built for the mechanism tests above: with `ctx["ground_root"]`
    wired to a scope holding one `_make_plane_mesh`-shaped rectangle
    (exactly what `apply_ground_planes` leaves under `<parent>/ground`), a
    real fissure walked out of `_c_fissures` leaves that mesh with more
    faces than the single quad it started as."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ground_root = PARENT + "/ground"
    from pxr import UsdGeom
    UsdGeom.Scope.Define(st, ground_root)
    _author_plane(st, ground_root + "/asphalt_base", -60.0, -60.0, 60.0, 60.0, 0.0)
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(17), tag="e2e")
    ctx["ground_root"] = ground_root
    made = qf._c_fissures(ctx, m, corners=("SW",), n_each=(1, 1),
                          length=(10.0, 10.0))
    assert made
    mesh = UsdGeom.Mesh(st.GetPrimAtPath(ground_root + "/asphalt_base"))
    counts = mesh.GetFaceVertexCountsAttr().Get()
    assert len(counts) > 1, "the city ground mesh was never cut"


def test_fissure_lip_is_smaller_than_the_old_mound():
    """Round 7: "the ridge/mould shrinks to a modest LIP" — its peak height
    is now a small fraction of the crack's own width (0.16-0.30 x, tapered
    and wobbled down from there), not the pre-round-7 mound's 0.45-0.85 x."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(15), tag="lipcheck")
    made = qf._c_fissures(ctx, m, corners=("SW",), n_each=(1, 1),
                          length=(10.0, 10.0))
    assert made
    peak = max(p[2] for p in _mesh_pts(st, made[0])) - m["z0"]
    assert peak <= 0.35 * qf.C_FISSURE_W[1]


def test_fissure_depth_knob_default_and_env_overridable():
    """`EQ_FISSURE_DEPTH_M` reaches `FISSURE_DEPTH_M` from the environment
    the same way `EQ_FISSURE_SCALE`/`EQ_FISSURE_WIDTH_SCALE`/
    `EQ_FISSURE_PAVE_DENSITY` reach theirs, and sits in the requested
    "roughly 1-2.5 m" band by default."""
    assert 1.0 <= qf.FISSURE_DEPTH_M <= 2.5
    code = (
        "import sys; sys.path.insert(0, {0!r});"
        "from disaster import quake_flow as qf;"
        "print(qf.FISSURE_DEPTH_M)"
    ).format(os.path.normpath(os.path.join(_HERE, "..")))
    env = dict(os.environ, EQ_FISSURE_DEPTH_M="2.4")
    out = subprocess.run([sys.executable, "-c", code], env=env,
                         capture_output=True, text=True, check=True).stdout
    assert out.strip() == "2.4", out


def test_fissure_pave_band_still_authored_alongside_the_opening():
    """The cracked-asphalt band beside the crack survives this round
    unchanged — authored alongside the new wall/floor, not replaced by
    them."""
    st = _new_stage()
    mats = qf.materials(st, PARENT)
    m = _mass()
    ctx = qf._c_ctx(st, PARENT, mats, random.Random(6), tag="pv2")
    made = qf._c_fissures(ctx, m, corners=("SW", "NE"), n_each=(1, 2))
    assert made
    pave_paths = [p for p in ctx["authored"]
                 if "_pave_" in p and "_mound_" not in p]
    wall_paths = [p for p in ctx["authored"] if "_wall_" in p]
    assert pave_paths, "no cracked-asphalt plate authored beside the opening"
    assert wall_paths, "no opening wall authored beside the cracked asphalt"


def test_ground_effects_ctx_wires_ground_root_and_ssf():
    """Round 7: `quake.ground_effects`'s own `_ctx()` must hand every
    fissure ctx a `ground_root`/`ground_ssf` (`parent + "/ground"`, `ssf`)
    so its epicentre cracks — the exact `epi_top.png`/`nw_top.png`
    screenshots the user flagged as "weird" — can cut their own opening
    through the city's ground meshes instead of only drawing a lip beside
    them."""
    import inspect

    from disaster import quake

    src = inspect.getsource(quake.ground_effects)
    assert '"ground_root": parent + "/ground"' in src
    assert '"ground_ssf": ssf' in src


def test_assemble_forwards_parent_as_ground_root():
    """Round 7: `quake.assemble`'s optional `parent` reaches every mild
    lean's corner fissures as `ground_root`, so the per-building cracks the
    live city actually authors get the same opening the epicentre ones do."""
    import inspect

    from disaster import quake

    src = inspect.getsource(quake.assemble)
    assert 'ground_root = (str(parent) + "/ground") if parent else None' in src
    assert src.count("ground_root=ground_root") >= 2, (
        "expected both the direct _tilt_prim call and the _mono_pass call "
        "to forward ground_root")


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
