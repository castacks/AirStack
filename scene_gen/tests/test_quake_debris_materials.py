#!/usr/bin/env python3
"""test_quake_debris_materials.py — does `_DEBRIS_LOOK` (round 7) actually
route each kit family's DEBRIS core/chunk material to match its OWN facade,
instead of the shared `urm`/`rc`/`rc_glass` bucket `_t_core_mat`/
`_chunk_material` used before it?

    cd scene_gen && uv run --python 3.13 --with vtk --with usd-core \\
        --with numpy --with pytest --with shapely --with scipy \\
        python -m pytest tests/test_quake_debris_materials.py -q

WHY THIS EXISTS
---------------
Live-scene review, on a `commercial_mid` DG3 pile's `merged_c_brick` chunks
(`bake.py`'s merge pass names a bucket after the material prim its meshes are
bound to, so every chunk `_t_core_mat`/`_chunk_material` sent to `_c_look_at
(..., "brick")` lands in exactly this bucket): "look good but seem to be the
wrong material and it doesn't match the rest of the building." Before this
round, `_t_core_mat`/`_chunk_material` drew their core/chunk mix from
`btype` ALONE (`urm`/`rc`/`rc_glass` — three buckets for eight kit families
in `urban_building.STYLES`), which is exactly `_FACADE_SCAR_BRICK_P`'s own
diagnosis for the STANDING-wall scar case ("`type` is what conflates 03 with
the genuine brick/stone families"), never promoted to the debris case until
now.

CHECKED, NOT ASSUMED. This agent pulled the kit's own baked base-colour
texture atlases (`~/scorch_previews/bake_probe/SM_MBuilding0{1,4}_*_base.png`)
before writing `_DEBRIS_LOOK` in `disaster/quake_flow.py`:

  * `SM_MBuilding04_Facade_A/B` — the asset `commercial`/`commercial_mid`/
    `department_store`/`highrise_04` (family "04") all reference — bakes as
    genuine reddish-brown coursed brick next to a plain grey stone accent
    panel. Brick-DOMINANT is correct for this family: the family the live-
    review complaint's building is actually in (`commercial_mid` -> family
    "04") was already drawing the right material CLASS. This file's
    `test_commercial_mid_core_stays_brick_dominant_its_kit_is_genuinely_brick`
    asserts that finding directly rather than force a category change the
    kit's own art does not support.
  * `SM_MBuilding01_Facade_A`/`FirstFloor_A` (family "01" — `apartment`/
    `apartment_tall`/`apartment_long`/`highrise_01`) bakes as a grey-tan
    ASHLAR stone, no brick anywhere. `church` (`CivilianArea`'s stone church
    set, `FAMILY_TYPE` already calls it "stone church") is the same case.
    Both were drawing the SAME brick-heavy mix as 04 before this table,
    purely because all four share the `urm` bucket — THAT is the actual
    "wrong material" bug fixed here, exercised below by
    `test_stone_families_are_no_longer_brick_dominant`.
  * family "03" (`brownstone`/`brownstone_row`/`walkup`) keeps the
    brick-fix round's own finding (`_FACADE_SCAR_BRICK_P["03"] = 0.12`):
    pale modern cladding over a frame, not brick or stone.
  * family "05" (`rc_glass`) gets a 0 % masonry-infill share in
    `_t_core_mat`'s non-urm branch — a modern glass curtain-wall tower has
    no rendered pier to shed masonry from at all.

This is host-side (`usd-core`, no Kit/Isaac): `_t_core_mat`/`_chunk_material`
only ever author `damage._pbr` OmniPBR prims and reference an unresolved
(harmless, host-side) `airstack://` asset path — nothing here needs Nucleus
or a render. WHAT THIS CANNOT SEE: whether the two "brick" tones (the kit's
own baked atlas vs. `Brick_Wall_Worn`) still match under RTX — that needs a
render (see `tools/debris_match_bench.py`).
"""
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_flow as qf                      # noqa: E402
from detail import urban_building as ub                    # noqa: E402

from pxr import Usd, UsdGeom                                # noqa: E402


PARENT = "/World/Bldg"

# The four styles the task singled out for verification, plus their kit
# family (`urban_building.STYLES[...]["family"]`) — asserted below so this
# file fails loudly if a future edit ever changes which family a style maps
# to (the whole point of keying `_DEBRIS_LOOK` on family, not style name).
_CHECK_STYLES = {
    "commercial_mid": "04",
    "apartment_long": "01",
    "brownstone_row": "03",
    "office_plain": "02",
}


def _new_stage():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, PARENT)
    return st


def _mat_name(mat):
    """The short name of a material `_t_core_mat`/`_chunk_material` returned
    — the flat `mats["stone"]`/`mats["mortar"]`/... prim's own name, or
    `_c_look_at`'s `c_brick` for the world-projected brick look. `None` if
    the function returned nothing (a bug this file wants to catch, not
    paper over)."""
    assert mat is not None, "material function returned None"
    return mat.GetPath().name


def _sample_core(stage, mats, btype, family, seed, n=3000):
    rng = random.Random(seed)
    counts = {}
    for _ in range(n):
        k = _mat_name(qf._t_core_mat(stage, PARENT, mats, btype, rng,
                                     family=family))
        counts[k] = counts.get(k, 0) + 1
    return counts


def _sample_chunk(stage, mats, btype, family, seed, n=3000, inner_p=1.0):
    rng = random.Random(seed)
    counts = {}
    for _ in range(n):
        k = _mat_name(qf._chunk_material(stage, PARENT, {}, None, mats,
                                         btype, rng, inner_p, family=family))
        counts[k] = counts.get(k, 0) + 1
    return counts


def _dominant(counts):
    return max(counts, key=counts.get)


def _share(counts, key):
    total = sum(counts.values())
    return counts.get(key, 0) / float(total)


# ---------------------------------------------------------------------------
# Table completeness: every kit family in `urban_building.STYLES` resolves
# cleanly (no exception, no None) through both functions, whether or not it
# has its own `_DEBRIS_LOOK` row.
# ---------------------------------------------------------------------------
def test_debris_look_covers_every_kit_family_with_no_errors():
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    families = sorted({spec["family"] for spec in ub.STYLES.values()})
    # sanity: this is the same family set `FAMILY_TYPE`/`check()` promise
    assert families == sorted(qf.FAMILY_TYPE.keys())
    for family in families:
        btype = qf.FAMILY_TYPE[family]
        core_counts = _sample_core(stage, mats, btype, family, seed=0, n=300)
        chunk_counts = _sample_chunk(stage, mats, btype, family, seed=0, n=300)
        assert core_counts, "no core material drawn for family {0}".format(family)
        assert chunk_counts, "no chunk material drawn for family {0}".format(family)


def test_debris_look_rows_only_name_real_flat_or_c_look_materials():
    """Every `masonry` value in the table is a key `_t_core_mat`/
    `_chunk_material` actually know how to resolve (`"brick"` -> `_c_look_at`,
    `"stone"` -> the new flat `mats["stone"]`) — a typo here would silently
    fall through to `mats.get(key) or mats.get("plaster")` and read as a
    grey card, the very bug class this table exists to fix."""
    for family, row in qf._DEBRIS_LOOK.items():
        if "masonry" in row:
            assert row["masonry"] in ("brick", "stone"), (family, row)
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    assert mats.get("stone") is not None


def test_check_still_passes_with_the_new_table_in_place():
    """`quake_flow.check()` is the existing host-side completeness gate
    (every ladder recipe known, every family has a `FAMILY_TYPE`) — this
    round touched neither, so it must still pass unchanged."""
    assert qf.check(verbose=False) == []


# ---------------------------------------------------------------------------
# Per-style verification (the task's four named styles): resolved debris
# material family vs. the building's own facade family.
# ---------------------------------------------------------------------------
def test_named_styles_map_to_the_expected_kit_family():
    for style, family in _CHECK_STYLES.items():
        assert ub.STYLES[style]["family"] == family, style


def test_commercial_mid_core_stays_brick_dominant_its_kit_is_genuinely_brick():
    """`commercial_mid` -> family "04". `bake_probe`'s own baked atlas for
    `SM_MBuilding04_Facade_A/B` is genuine reddish-brown brick — the debris
    channel was already right here, and this table keeps it that way rather
    than force it toward concrete/plaster on an unverified assumption."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    btype = qf.FAMILY_TYPE["04"]
    assert btype == "urm"
    core = _sample_core(stage, mats, btype, "04", seed=1)
    chunk = _sample_chunk(stage, mats, btype, "04", seed=2)
    print("[debris-match] commercial_mid (family 04) core:", core)
    print("[debris-match] commercial_mid (family 04) chunk:", chunk)
    assert _dominant(core) == "c_brick"
    assert _dominant(chunk) == "c_brick"
    assert _share(core, "c_brick") > 0.60
    assert _share(chunk, "c_brick") > 0.35


def test_stone_families_are_no_longer_brick_dominant():
    """`apartment_long` -> family "01"; `church` -> family "church". Both
    bake as dressed stone (`SM_MBuilding01_Facade_A`/`FirstFloor_A`) or are
    documented as one (`FAMILY_TYPE["church"]` comment: "stone church") —
    THIS is the bug the live-review complaint's fix actually targets: before
    `_DEBRIS_LOOK`, both shared `urm`'s brick-heavy default with the
    genuinely-brick families and shed red brick chunks instead of stone."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    for family in ("01", "church"):
        assert ub.STYLES["apartment_long"]["family"] == "01"  # sanity, once
        btype = qf.FAMILY_TYPE[family]
        assert btype == "urm"
        core = _sample_core(stage, mats, btype, family, seed=3)
        chunk = _sample_chunk(stage, mats, btype, family, seed=4)
        print("[debris-match] family {0} core:".format(family), core)
        print("[debris-match] family {0} chunk:".format(family), chunk)
        assert _dominant(core) == "stone", family
        assert _dominant(chunk) == "stone", family
        assert core.get("c_brick", 0) == 0, "family {0} core drew brick".format(family)
        assert chunk.get("c_brick", 0) == 0, "family {0} chunk drew brick".format(family)


def test_brownstone_row_stays_plaster_mortar_dominant_not_brick():
    """`brownstone_row` -> family "03": pale modern cladding, not brick or
    stone (brick-fix round's own finding, `_FACADE_SCAR_BRICK_P["03"] =
    0.12`, matched here). `_FACADE_SCAR_BRICK_P` only ever touched
    STANDING-wall scars, never `_chunk_material`/`_t_core_mat`'s DEBRIS mix —
    so before this round, family 03's fractured debris was STILL drawing the
    shared `urm` bucket's brick-heavy 0.45/0.70 share despite its facade
    being pale cladding. This asserts that is now fixed too."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    btype = qf.FAMILY_TYPE["03"]
    assert btype == "urm"
    core = _sample_core(stage, mats, btype, "03", seed=5)
    chunk = _sample_chunk(stage, mats, btype, "03", seed=6)
    print("[debris-match] brownstone_row (family 03) core:", core)
    print("[debris-match] brownstone_row (family 03) chunk:", chunk)
    assert _dominant(core) != "c_brick"
    assert _dominant(chunk) != "c_brick"
    assert _share(core, "c_brick") < 0.20
    assert _share(chunk, "c_brick") < 0.20


def test_office_plain_rc_family_is_unaffected_by_the_table():
    """`office_plain` -> family "02", a plain `rc` family with no
    `_DEBRIS_LOOK` row (no counter-evidence was found for it) — its mix must
    render byte-identical to the pre-round-7 default: mortar/dark_concrete-
    dominant with a minority masonry-infill brick share."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    btype = qf.FAMILY_TYPE["02"]
    assert btype == "rc"
    assert "02" not in qf._DEBRIS_LOOK
    core = _sample_core(stage, mats, btype, "02", seed=7)
    chunk = _sample_chunk(stage, mats, btype, "02", seed=8)
    print("[debris-match] office_plain (family 02) core:", core)
    print("[debris-match] office_plain (family 02) chunk:", chunk)
    assert _dominant(core) in ("mortar", "dark_concrete")
    assert _dominant(chunk) in ("mortar", "dark_concrete")
    # the "a fifth of the core faces take the brick map" period-masonry
    # share is unchanged (~0.22)
    assert 0.15 < _share(core, "c_brick") < 0.30
    # `_chunk_material`'s non-urm branch never had a brick option at all
    assert chunk.get("c_brick", 0) == 0


# ---------------------------------------------------------------------------
# family "05" (rc_glass): no masonry-infill share at all
# ---------------------------------------------------------------------------
def test_rc_glass_tower_core_never_draws_brick():
    """family "05" (`tower`/`skyscraper_b`/`highrise_step`/`fam05` towers): a
    modern glass curtain-wall tower over a concrete podium has no rendered
    masonry pier at all (the same reasoning `r_facade_scars` already uses to
    skip every RC family outright) — `_DEBRIS_LOOK["05"]["p_infill"] = 0.0`
    zeroes the period-masonry share `_t_core_mat`'s non-urm branch otherwise
    draws for a plain `rc` family."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    btype = qf.FAMILY_TYPE["05"]
    assert btype == "rc_glass"
    core = _sample_core(stage, mats, btype, "05", seed=9)
    print("[debris-match] family 05 (rc_glass) core:", core)
    assert core.get("c_brick", 0) == 0


def test_p_infill_default_matches_the_legacy_rc_share_exactly():
    """A non-urm family with NO table row (`"civ"`, or `"02"` again) must
    draw brick at the ORIGINAL 0.63..0.85 (22 %) cut — this is the
    "additive, nothing that was already right moves" guarantee the table's
    own comment makes, checked as a number rather than just eyeballed."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    for family in ("02", "civ"):
        assert family not in qf._DEBRIS_LOOK
        counts = _sample_core(stage, mats, "rc", family, seed=11, n=6000)
        assert abs(_share(counts, "c_brick") - 0.22) < 0.03, family


# ---------------------------------------------------------------------------
# Determinism: same rng seed -> the identical draw sequence, every time.
# ---------------------------------------------------------------------------
def test_core_and_chunk_material_are_deterministic_given_the_same_seed():
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    for family, btype in (("04", "urm"), ("01", "urm"), ("03", "urm"),
                          ("02", "rc"), ("05", "rc_glass")):
        seq_a = [_mat_name(qf._t_core_mat(stage, PARENT, mats, btype,
                                          random.Random(42), family=family))
                for _ in range(200)]
        seq_b = [_mat_name(qf._t_core_mat(stage, PARENT, mats, btype,
                                          random.Random(42), family=family))
                for _ in range(200)]
        assert seq_a == seq_b, family


def test_family_absent_from_table_behaves_exactly_like_urm_default():
    """A hypothetical urm family with no `_DEBRIS_LOOK` row falls through to
    `_DEBRIS_LOOK_URM_DEFAULT`, which is numerically the ORIGINAL flat
    0.70/0.45 brick shares — passing `family=None` (the pre-round-7 call
    shape, still reachable from any caller this agent missed) must render
    identically to passing an unknown family string."""
    stage = _new_stage()
    mats = qf.materials(stage, PARENT)
    a = _sample_core(stage, mats, "urm", None, seed=20, n=4000)
    b = _sample_core(stage, mats, "urm", "not_a_real_family", seed=20, n=4000)
    assert a == b
    assert abs(_share(a, "c_brick") - 0.70) < 0.03
