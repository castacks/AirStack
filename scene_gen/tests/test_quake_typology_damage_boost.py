#!/usr/bin/env python3
"""test_quake_typology_damage_boost.py — `disaster.typology_damage_boost`
(item 2, user 2026-09-01, verbatim: "more diversity in buildings destroyed —
especially brownstone districts with the small buildings and the brick
mid-rise districts").

    pytest -q scene_gen/tests/test_quake_typology_damage_boost.py

WHY STYLE, NOT DISTRICT/TYPOLOGY
---------------------------------
The damage grade is drawn purely off the RADIAL field (`sg.make_damage_
field`), so which district reads destroyed is an accident of where it
landed relative to the epicentre. The natural fix is a per-typology
intensity multiplier, keyed off the REAL typology a building's block was
zoned — but that information never reaches `disaster/quake.py`: a
placement dict (`scene_generator.build_city`'s own return value) carries
only `usd`/pose/`category` (`x_m`, `y_m`, `z_m`, `yaw_deg`, `roll_deg`,
`pitch_deg`, `scale`, `prim_path`, `axis_up`) — district/typology membership
is resolved and forgotten inside `detail/districts.py`'s own rezone, code
this session does not own. `TYPOLOGY_STYLE_FAMILIES` is the documented
fallback: a preset-level label (`"rowhouse"`, `"brick_midrise"`) maps onto
the `detail.urban_building.STYLES` KIT styles that read as that district in
practice — `brownstone`/`brownstone_row` (family "03") for rowhouse, the
family-"04" brick mid/high-rise stock for brick_midrise.

DEFAULT EMPTY: `_typology_damage_boost({})` is `{}`, and `_boosted_
intensity` is then a no-op for every style — every preset but
`downtown_earthquake` (and any run of it that never sets the key) draws the
IDENTICAL rng sequence it always has.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "tools")))

from disaster import quake as q               # noqa: E402


# ---------------------------------------------------------------------------
# `_typology_damage_boost` — reads `disaster.typology_damage_boost`, empty
# by default
# ---------------------------------------------------------------------------
def test_boost_table_empty_when_key_absent():
    assert q._typology_damage_boost({}) == {}
    assert q._typology_damage_boost({"disaster": {}}) == {}
    assert q._typology_damage_boost(
        {"disaster": {"damaged_fraction": 0.0}}) == {}


def test_boost_table_reads_configured_multipliers_as_float():
    cfg = {"disaster": {"typology_damage_boost": {"rowhouse": 2, "brick_midrise": 1.4}}}
    out = q._typology_damage_boost(cfg)
    assert out == {"rowhouse": 2.0, "brick_midrise": 1.4}
    assert isinstance(out["rowhouse"], float)


# ---------------------------------------------------------------------------
# `TYPOLOGY_STYLE_FAMILIES` — the style-family membership table
# ---------------------------------------------------------------------------
def test_rowhouse_family_is_the_brownstone_styles():
    assert set(q.TYPOLOGY_STYLE_FAMILIES["rowhouse"]) == \
        {"brownstone", "brownstone_row"}


def test_brick_midrise_family_is_the_family_04_kit_stock():
    assert set(q.TYPOLOGY_STYLE_FAMILIES["brick_midrise"]) == \
        {"commercial", "commercial_mid", "highrise_04", "department_store"}


def test_style_families_match_the_real_manifest_family_tag():
    """Every style named in `TYPOLOGY_STYLE_FAMILIES` must actually carry
    the family the label implies, read off the REAL baked manifest — not
    just asserted by comment. `rowhouse` -> family "03" (brownstone,
    `quake_flow.FAMILY_TYPE["03"] == "urm"`); `brick_midrise` -> family
    "04" (brick commercial, also `urm`)."""
    _SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
    arch_dir = os.path.join(_SCENE_GEN, "assets", "archetype")
    manifest = q.load_manifest(arch_dir)
    for style in q.TYPOLOGY_STYLE_FAMILIES["rowhouse"]:
        rec = manifest.get((style, "DG0"))
        assert rec is not None, f"{style} DG0 missing from the real manifest"
        assert rec["family"] == "03"
    for style in q.TYPOLOGY_STYLE_FAMILIES["brick_midrise"]:
        rec = manifest.get((style, "DG0"))
        assert rec is not None, f"{style} DG0 missing from the real manifest"
        assert rec["family"] == "04"


# ---------------------------------------------------------------------------
# `_boosted_intensity` — the pure per-building multiplier
# ---------------------------------------------------------------------------
def test_boosted_intensity_is_a_noop_with_an_empty_table():
    assert q._boosted_intensity(0.7, "brownstone_row", {}) == 0.7
    assert q._boosted_intensity(0.7, "brownstone_row", None) == 0.7


def test_boosted_intensity_is_a_noop_for_a_style_matching_no_family():
    boost = {"rowhouse": 1.5, "brick_midrise": 1.5}
    assert q._boosted_intensity(0.7, "office_wide", boost) == 0.7
    assert q._boosted_intensity(0.7, "tower", boost) == 0.7


def test_boosted_intensity_is_a_noop_for_a_falsy_style():
    boost = {"rowhouse": 1.5}
    assert q._boosted_intensity(0.7, None, boost) == 0.7
    assert q._boosted_intensity(0.7, "", boost) == 0.7


@pytest.mark.parametrize("style", ["brownstone", "brownstone_row"])
def test_boosted_intensity_multiplies_rowhouse_family_styles(style):
    boost = {"rowhouse": 1.5}
    assert q._boosted_intensity(0.6, style, boost) == pytest.approx(0.9)


@pytest.mark.parametrize(
    "style", ["commercial", "commercial_mid", "highrise_04", "department_store"])
def test_boosted_intensity_multiplies_brick_midrise_family_styles(style, ):
    boost = {"brick_midrise": 1.3}
    assert q._boosted_intensity(0.6, style, boost) == pytest.approx(0.78)


def test_boosted_intensity_only_applies_the_label_present_in_the_table():
    """A table that boosts ONLY `brick_midrise` must leave a rowhouse-family
    style untouched, and vice versa — the two labels are independent
    knobs."""
    assert q._boosted_intensity(0.6, "brownstone", {"brick_midrise": 2.0}) == 0.6
    assert q._boosted_intensity(0.6, "commercial_mid", {"rowhouse": 2.0}) == 0.6


def test_boosted_intensity_not_clamped_above_one():
    """`quake_flow.level_for_intensity` clamps its own `v` draw — an
    `inten` pushed above 1.0 by a large boost is a legitimate way to raise
    the odds of the top grades, not a value this function should cap."""
    assert q._boosted_intensity(0.9, "brownstone", {"rowhouse": 2.0}) == \
        pytest.approx(1.8)


# ---------------------------------------------------------------------------
# End to end, offline, on the REAL preset (`downtown_earthquake`) — same
# same-process harness idiom as `test_quake_style_blacklist.py`'s
# `_assemble_styles`: a real `generate_scene_on_stage` layout, a real
# in-memory pxr stage, a real `disaster.quake.assemble` call, entirely local
# files (no Nucleus/Isaac Sim needed — see that module's own header).
#
# NOT WIRED THROUGH `spec_overrides`: `compile_disaster.load_scene_config`'s
# `overrides` block is DEEP-MERGED (its own documented contract), and
# `deep_merge` recurses into a DICT-valued key rather than replacing it — an
# override of `{"typology_damage_boost": {}}` therefore merges NOTHING into
# an already non-empty table (the shipped preset's own `{rowhouse: ...,
# brick_midrise: ...}`) and leaves it untouched, not cleared. So the two
# variants below are built by mutating the COMPILED config dict directly
# (key absent vs. key present-but-`{}`), sidestepping that merge semantics
# entirely rather than fighting it.
# ---------------------------------------------------------------------------
def _assemble_grades_from_config(config):
    from pxr import Usd
    import generate_scene
    import layout_dry_run as ldr

    ldr._localize_building_urls(config["usds"])
    stage = Usd.Stage.CreateInMemory()
    placements = generate_scene.generate_scene_on_stage(
        stage, config, parent_path="/World/stage/generated",
        scene_scale_factor=1.0, snap_to_ground=False)

    _SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
    arch_dir = os.path.join(_SCENE_GEN, "assets", "archetype")
    gac_dir = os.path.join(_SCENE_GEN, "assets", "gac_quake")
    q.assemble(stage, config, placements, arch_dir, seed=11,
              gac_dir=gac_dir, verbose=False)

    houses = [p for p in placements if p.get("category") == "house"]
    return [q.style_of(p.get("usd")) for p in houses]


@pytest.mark.parametrize("seed,region_m", [(9, 250.0)])
def test_key_absent_and_key_present_empty_are_indistinguishable_end_to_end(
        seed, region_m):
    """Whether `disaster.typology_damage_boost` is MISSING entirely or
    present as an explicit `{}` must produce the exact same (style, level)
    sequence, in the same order — `_typology_damage_boost`'s `dis.get(...)
    or {}` already treats the two the same at the pure-function level
    (`test_boost_table_empty_when_key_absent`); this proves the WIRING in
    `assemble` (`dmg_boost` computed ONCE, then looked up per building)
    carries that all the way through a real run without perturbing the rng
    sequence — the guarantee every OTHER preset (which never sets the key
    at all) relies on."""
    import copy

    import compile_disaster

    overrides = {"region_m": [float(region_m), float(region_m)], "seed": seed}
    base_config = compile_disaster.load_scene_config(
        "downtown_earthquake", spec_overrides=overrides)

    config_absent = copy.deepcopy(base_config)
    config_absent.setdefault("disaster", {}).pop("typology_damage_boost", None)

    config_empty = copy.deepcopy(base_config)
    config_empty.setdefault("disaster", {})["typology_damage_boost"] = {}

    assert _assemble_grades_from_config(config_absent) == \
        _assemble_grades_from_config(config_empty)
