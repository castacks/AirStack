#!/usr/bin/env python3
"""test_tornado_city_dry_run.py — `disaster/tornado_city.py` (the city-scale
damageable-set predicate, height class, T-level draw and manifest-record
helpers) and `tools/tornado_city_dry_run.py` (the dry run tool that reads a
REAL `fire_city_placements_dump.v1` and applies them), stream C of
`scene_gen/_plans/urban_tornado_plan.md` §4.

    python3 -m pytest -q scene_gen/tests/test_tornado_city_dry_run.py
    python3 scene_gen/tests/test_tornado_city_dry_run.py

Pure python, host-side, no Kit, no Nucleus, no stage — every gate/level/
height-class test here runs against SYNTHETIC placements the same way
`test_urban_fire_city.py` does. The one exception is
`test_real_lvl2_dump_manifest_and_checks`, which reads the REAL
`_plans/city_placements_downtown_fire_1500_lvl2.json` dump and preset — it is
skipped, not failed, when that file is not present on this machine.
"""
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
for _p in (_SCENE_GEN_DIR, _TOOLS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from disaster import gac_fire as gf                        # noqa: E402
from disaster import tornado_city as tc                    # noqa: E402
from disaster import urban_fire_city as ufc                # noqa: E402

import tornado_city_dry_run as tcdr                         # noqa: E402

# ---------------------------------------------------------------------------
# Fixtures shared by every test
# ---------------------------------------------------------------------------
KIT_ARCHETYPE_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                     "SEI-COA/scene_gen/assets/archetype/")
AEC_BROWNSTONE = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/Demos/"
                  "AEC/Buildings/Brownstone/Brownstone_01.usd")
MUYANG = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Muyang/DownTown/Assets/BG_Building_A.usd"


def _placement(usd, x=50.0, y=50.0, yaw=0.0, category="house", i=0):
    return {"category": category, "usd": usd, "x_m": x, "y_m": y,
           "z_m": 0.0, "yaw_deg": yaw, "prim_path": "/World/stage/house_{0}".format(i),
           "i": i}


def _size_of(placement, W, D, H):
    """`size_of` for `damageable()` — reads geometry from `size_of`, never
    from the placement dict (`urban_fire_city.burnable`'s own contract)."""
    return {placement["usd"]: (W, D, H)}


def _damage(usd, W, D, H, **kw):
    p = _placement(usd, **kw)
    return (p,) + tc.damageable(p, _size_of(p, W, D, H))


# ---------------------------------------------------------------------------
# damageable(): §2.3 gates 1-4, exact refusal reason per gate
# ---------------------------------------------------------------------------
def test_gate1_rejects_non_house_category():
    p, ok, reason, route, kind, name, bakeable = _damage(
        gf.GAC_DIR + "SM_Building_04.usd", 31.5, 28.0, 42.3, category="tree")
    assert not ok
    assert "not a building placement" in reason


def test_gate2_rejects_muyang_with_routes_own_reason():
    p, ok, reason, route, kind, name, bakeable = _damage(MUYANG, 45.0, 45.0, 90.0)
    assert not ok
    assert "route refused" in reason
    assert "unburnable" in reason


def test_gate4_rejects_blacklisted_building_11():
    usd = gf.DTC_DIR + "Building_11.usdc"
    p, ok, reason, route, kind, name, bakeable = _damage(usd, 30.9, 35.4, 32.6)
    assert not ok
    assert "blacklisted" in reason
    assert "Building_11" in reason


def test_gate4_rejects_blacklisted_carved_03():
    usd = gf.DTC_DIR + "Carved_03.usdc"
    p, ok, reason, route, kind, name, bakeable = _damage(usd, 42.6, 44.4, 27.6)
    assert not ok
    assert "blacklisted" in reason
    assert "Carved_" in reason


def test_gate5_height_cap_rejects_a_302m_sm_building_31():
    usd = gf.GAC_DIR + "SM_Building_31.usd"
    p, ok, reason, route, kind, name, bakeable = _damage(usd, 60.3, 142.2, 302.2)
    assert not ok
    assert "taller than the tornado-height cap" in reason
    assert "302.2" in reason


def test_gate5_accepts_amar_tower_at_its_own_measured_height():
    usd = gf.DTC_DIR + "Amar_Tower.usdc"
    p, ok, reason, route, kind, name, bakeable = _damage(usd, 42.3, 48.8, 231.4)
    assert ok
    assert kind == "dtc"
    assert name == "Amar_Tower"
    assert route == "slice"
    assert bakeable is True


def test_aec_brownstone_accepted_with_bakeable_false():
    p, ok, reason, route, kind, name, bakeable = _damage(
        AEC_BROWNSTONE, 20.0, 15.0, 18.0)
    assert ok, reason
    assert route == "slice"
    assert bakeable is False
    assert name  # a real basename, not None


def test_kit_archetype_accepted_with_route_kit():
    # ROUND 2 (`_plans/urban_tornado_plan.md` §7): a kit-routed record's
    # `bakeable` tracks `disaster.tornado_kit`'s live importability
    # (`tornado_city._kit_damage_capable()`), not a hardcoded True -- see
    # `test_kit_bakeable_tracks_tornado_kit_importability` below for the
    # gate itself. Assert AGREEMENT with the live probe rather than a fixed
    # value, so this test does not start failing the moment stream K's
    # module lands.
    usd = KIT_ARCHETYPE_DIR + "bld_office_wide_DG0.usd"
    p, ok, reason, route, kind, name, bakeable = _damage(usd, 40.0, 30.0, 35.0)
    assert ok, reason
    assert route == "kit"
    assert kind == "kit"
    assert bakeable is tc._kit_damage_capable()


def test_kit_bakeable_tracks_tornado_kit_importability():
    # The gate itself: absent `disaster.tornado_kit` (true today -- stream K
    # has not landed it yet), a kit record is NOT damage-capable. This is a
    # deliberate reversal of round 1's behaviour, where `bakeable` was True
    # unconditionally for any `kind is not None` (see `tornado_city.
    # damageable`'s module-level comment on the gate for why that was an
    # overclaim for tornado specifically, inherited from the FIRE sense of
    # "bakeable"). If this test starts failing because `tornado_kit` now
    # importable, that is EXPECTED once stream K lands -- flip the assertion
    # per that module's docstring, do not delete the test.
    import importlib
    try:
        importlib.import_module("disaster.tornado_kit")
        tornado_kit_present = True
    except ImportError:
        tornado_kit_present = False
    assert tc._kit_damage_capable() is tornado_kit_present


# ---------------------------------------------------------------------------
# industrial (tilt-up / light-roof) gate -- R11, §8c
# ---------------------------------------------------------------------------
DMYTRO_FACTORY_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/"
                      "Stages/Dmytro/Assets/Game/FactoryDistrict/Meshes/")
# the four chosen sheds, measured (config/harvested/standalone_buildings.json)
INDUSTRIAL_SHED_C_A = (DMYTRO_FACTORY_DIR + "Building_TypeC_A.usd", 25.1, 25.1, 11.1)
INDUSTRIAL_SHED_D_A = (DMYTRO_FACTORY_DIR + "Building_TypeD_A.usd", 42.0, 31.5, 11.7)
INDUSTRIAL_SHED_B_C = (DMYTRO_FACTORY_DIR + "Building_TypeB_C.usd", 67.1, 45.1, 12.0)
INDUSTRIAL_SHED_C_D = (DMYTRO_FACTORY_DIR + "Building_TypeC_D.usd", 41.1, 41.1, 16.0)
# a REAL FactoryDistrict shed NOT in the chosen four -- confirms the suffix
# match is precise, not "anything under FactoryDistrict/Meshes/".
NON_CHOSEN_SHED = (DMYTRO_FACTORY_DIR + "Building_TypeA_A.usd", 25.0, 39.7, 11.9)


def test_industrial_shed_accepted_with_kind_industrial():
    for usd, W, D, H in (INDUSTRIAL_SHED_C_A, INDUSTRIAL_SHED_D_A,
                         INDUSTRIAL_SHED_B_C, INDUSTRIAL_SHED_C_D):
        p, ok, reason, route, kind, name, bakeable = _damage(usd, W, D, H)
        assert ok, (usd, reason)
        assert route == "slice"
        assert kind == "industrial"
        assert name  # a real basename, not None
        assert bakeable is tc._industrial_damage_capable()


def test_non_chosen_shed_stays_kind_slice_not_industrial():
    usd, W, D, H = NON_CHOSEN_SHED
    assert not tc.is_industrial_shed(usd)
    p, ok, reason, route, kind, name, bakeable = _damage(usd, W, D, H)
    assert ok, reason
    assert route == "slice"
    assert kind == "slice"          # NOT "industrial"
    assert bakeable is False        # the ordinary AEC/standalone dead end


def test_is_industrial_shed_matches_by_suffix_any_root():
    # the asset-set yaml's own RELATIVE form (no Nucleus root at all).
    assert tc.is_industrial_shed(
        "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeC_A.usd")
    # the harvested manifest's fully-resolved form.
    assert tc.is_industrial_shed(
        "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/Dmytro/"
        "Assets/Game/FactoryDistrict/Meshes/Building_TypeC_A.usd")
    assert not tc.is_industrial_shed(
        "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeA_A.usd")
    assert not tc.is_industrial_shed(None)
    assert not tc.is_industrial_shed("")


def test_industrial_damage_capable_tracks_tornado_collapse_importability():
    # Same "gate tracks live importability" discipline as
    # `test_kit_bakeable_tracks_tornado_kit_importability` -- `disaster.
    # tornado_collapse` DOES exist on disk as of this stream's own round
    # (unlike `tornado_kit` when that test was first written), so this is
    # expected to read True; if it ever starts failing, that means the
    # module import itself started raising -- update the assertion only
    # after confirming that is intentional, per that test's own rule.
    import importlib
    try:
        importlib.import_module("disaster.tornado_collapse")
        present = True
    except ImportError:
        present = False
    assert tc._industrial_damage_capable() is present


def test_industrial_shed_height_cap_still_applies():
    usd, W, D, _H = INDUSTRIAL_SHED_D_A
    p, ok, reason, route, kind, name, bakeable = _damage(
        usd, W, D, tc.TORNADO_MAX_H_M + 5.0)
    assert not ok
    assert "taller than the tornado-height cap" in reason
    assert kind == "industrial"     # kind/name still carried on a refusal


def test_industrial_shed_blacklist_gate_is_a_documented_no_op():
    # `gac_fire.PACKS` has no `"industrial"` entry at all -- gate 4 is a
    # structural no-op for this kind (same as `"kit"`/`"slice"`), asserted
    # directly so a future PACKS edit that adds one is not silently inert.
    assert ufc._pack_blacklist_reason("industrial", "Building_TypeD_A") is None


def test_height_cap_boundary_is_inclusive():
    usd = gf.GAC_DIR + "SM_Building_04.usd"
    p, ok, reason, route, kind, name, bakeable = _damage(
        usd, 31.5, 28.0, tc.TORNADO_MAX_H_M)
    assert ok
    p, ok, reason, route, kind, name, bakeable = _damage(
        usd, 31.5, 28.0, tc.TORNADO_MAX_H_M + 0.01)
    assert not ok
    assert "taller than the tornado-height cap" in reason


def test_unmeasured_height_never_trips_the_cap():
    assert tc._height_cap_reason(None, tc.TORNADO_MAX_H_M) is None


# ---------------------------------------------------------------------------
# height_class_for(): typology beats a contradicting height
# ---------------------------------------------------------------------------
def test_height_class_typology_beats_height():
    # A "lowrise"/"rowhouse" typology (fire's unambiguous "low" bucket, per
    # `urban_fire_spread.TYPOLOGY_HEIGHT_CLASS`) is trusted outright even
    # against a height that would otherwise read as "highrise" (90 m).
    assert tc.height_class_for(90.0, "lowrise") == "lowrise"
    assert tc.height_class_for(90.0, "rowhouse") == "lowrise"
    # the same height with NO typology falls through to the H bands.
    assert tc.height_class_for(90.0, None) == "highrise"


def test_height_class_h_fallback_bands():
    assert tc.height_class_for(9.0, None) == "lowrise"
    assert tc.height_class_for(30.0, None) == "midrise"
    assert tc.height_class_for(70.0, None) == "highrise"
    assert tc.height_class_for(150.0, None) == "tower"
    # boundaries, per §2.3: lowrise < 18, midrise 18-45, highrise 45-100,
    # tower >= 100
    assert tc.height_class_for(17.99, None) == "lowrise"
    assert tc.height_class_for(18.0, None) == "midrise"
    assert tc.height_class_for(44.99, None) == "midrise"
    assert tc.height_class_for(45.0, None) == "highrise"
    assert tc.height_class_for(99.99, None) == "highrise"
    assert tc.height_class_for(100.0, None) == "tower"


# ---------------------------------------------------------------------------
# level_for_intensity(): deterministic per (seed, i), monotone in
# expectation in intensity; the FALLBACK matches §2.6's cut table exactly.
# ---------------------------------------------------------------------------
def test_level_draw_deterministic_per_seed_and_index():
    seed = 4242
    for i in (0, 1, 17, 583):
        a = tc.level_for_intensity(0.55, random.Random(seed * 1000003 + i))
        b = tc.level_for_intensity(0.55, random.Random(seed * 1000003 + i))
        assert a == b
    # a different building index draws its OWN rng stream -- not required to
    # differ, but the SEED formula itself must be what actually varies.
    r17 = random.Random(seed * 1000003 + 17)
    r18 = random.Random(seed * 1000003 + 18)
    assert r17.random() != r18.random()


def test_level_draw_monotone_in_expectation_in_intensity():
    _RANK = {"T0": 0, "T1": 1, "T2": 2, "T3": 3, "T4": 4}
    intensities = [0.0, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.0]
    means = []
    for i_val in intensities:
        draws = [tc.level_for_intensity(i_val, random.Random(k))
                for k in range(300)]
        means.append(sum(_RANK[d] for d in draws) / float(len(draws)))
    # non-decreasing, allowing for sampling noise between ADJACENT points by
    # comparing the ends against the middle rather than every single step.
    assert means[0] < means[len(means) // 2] < means[-1]
    for a, b in zip(means, means[1:]):
        assert b >= a - 0.05          # small slack for sampling noise


def test_fallback_level_for_intensity_matches_the_cut_table_at_band_centres():
    # §2.6's cuts (2026-09-01 revision, `tornado_urban._URBAN_CUTS`): T0 <
    # 0.10, T1 0.10-0.36, T2 0.36-0.56, T3 0.56-0.74, T4 >= 0.74. jitter=0
    # makes this an exact, non-random check.
    centres = {"T0": 0.05, "T1": 0.23, "T2": 0.46, "T3": 0.65, "T4": 0.87}
    for level, i_val in centres.items():
        got = tc._fallback_level_for_intensity(i_val, random.Random(1), jitter=0.0)
        assert got == level, (i_val, got, level)


def test_fallback_level_for_intensity_cut_boundaries():
    rng = random.Random(1)
    assert tc._fallback_level_for_intensity(0.099999, rng, jitter=0.0) == "T0"
    assert tc._fallback_level_for_intensity(0.10, rng, jitter=0.0) == "T1"
    assert tc._fallback_level_for_intensity(0.359999, rng, jitter=0.0) == "T1"
    assert tc._fallback_level_for_intensity(0.36, rng, jitter=0.0) == "T2"
    assert tc._fallback_level_for_intensity(0.559999, rng, jitter=0.0) == "T2"
    assert tc._fallback_level_for_intensity(0.56, rng, jitter=0.0) == "T3"
    assert tc._fallback_level_for_intensity(0.739999, rng, jitter=0.0) == "T3"
    assert tc._fallback_level_for_intensity(0.74, rng, jitter=0.0) == "T4"
    assert tc._fallback_level_for_intensity(1.5, rng, jitter=0.0) == "T4"


# ---------------------------------------------------------------------------
# skyscraper_exposure(): plan §7 R1's hard-check data source
# ---------------------------------------------------------------------------
def test_skyscraper_exposure_protected_set_is_tower_or_h_ge_75():
    # class "tower" protects at ANY height (250 m here); "highrise" at
    # 80 m (>= SKYSCRAPER_PROTECTED_MIN_H_M) protects too; "highrise" at
    # 60 m -- the GAC brick_midrise pool's own 47-72 m range, the exact
    # regression the lead review found -- does NOT; "midrise" at 30 m never
    # did.
    recs = [
        tc.record(0, "/World/a", "u1", "gac", "n1", 10.0, 20.0, 0.0,
                  60.0, 60.0, 250.0, "rc", "tower", 0.5, "T2", None,
                  "slice", True, 1),
        tc.record(1, "/World/b", "u2", "gac", "n2", -5.0, 5.0, 0.0,
                  40.0, 40.0, 80.0, "rc", "highrise", 0.5, "T1", None,
                  "slice", True, 1),
        tc.record(2, "/World/c", "u3", "gac", "n3", 2.0, 2.0, 0.0,
                  20.0, 20.0, 60.0, "urm", "highrise", 0.5, "T1", None,
                  "slice", True, 1),
        tc.record(3, "/World/d", "u4", "gac", "n4", 1.0, 1.0, 0.0,
                  20.0, 20.0, 30.0, "urm", "midrise", 0.5, "T1", None,
                  "slice", True, 1),
    ]
    out = tc.skyscraper_exposure(recs, lambda x, y: 0.7)
    assert {o["name"] for o in out} == {"n1", "n2"}
    assert len(out) == 2
    assert all(o["i_raw"] == 0.7 for o in out)


def test_is_protected_skyscraper():
    assert tc.is_protected_skyscraper("tower", 33.0) is True     # class always protects
    assert tc.is_protected_skyscraper("tower", None) is True     # even with no H at all
    assert tc.is_protected_skyscraper("highrise", 75.0) is True  # boundary itself
    assert tc.is_protected_skyscraper("highrise", 74.99) is False
    assert tc.is_protected_skyscraper("highrise", None) is False  # unmeasured never protects alone
    assert tc.is_protected_skyscraper("midrise", 90.0) is True   # H alone is enough regardless of class
    assert tc.is_protected_skyscraper("lowrise", 10.0) is False


def test_skyscraper_exposure_corner_sampling_catches_a_wide_footprint():
    # SM_Building_16-shaped regression: a wide tower whose CENTRE sits
    # outside a narrow hot zone but whose near CORNER sits inside it.
    def to_track(x, y):
        return x, x                                   # cross == x, trivially

    def field(x, y):
        return 0.9 if abs(x) <= 40.0 else 0.1

    wide = tc.record(0, "/World/w", "u", "gac", "wide_tower", 50.0, 0.0, 0.0,
                     84.5, 56.9, 312.0, "rc", "tower", 0.2, "T1", None,
                     "slice", True, 1)
    centre_only = tc.skyscraper_exposure([wide], field)
    with_corner = tc.skyscraper_exposure([wide], field, to_track=to_track)
    assert centre_only[0]["i_raw"] < tc.SKYSCRAPER_MAX_I
    assert with_corner[0]["i_raw"] >= 0.9
    assert with_corner[0]["sample_x"] != wide["x"]


def test_skyscraper_exposure_samples_the_raw_field_not_a_jittered_draw():
    # The field passed in stands for `intensity_field`'s own raw sample --
    # skyscraper_exposure must call it with the item's OWN (x, y) and use
    # its return value verbatim, with no jitter/randomness of its own.
    calls = []

    def field(x, y):
        calls.append((x, y))
        return 0.55

    rec = tc.record(0, "/World/a", "u", "gac", "n", 3.0, -4.0, 0.0,
                    50.0, 50.0, 150.0, "rc", "highrise", 0.61, "T2", None,
                    "slice", True, 7)
    out = tc.skyscraper_exposure([rec], field)
    assert calls == [(3.0, -4.0)]
    assert out[0]["i_raw"] == 0.55


def test_skyscraper_exposure_accepts_raw_placements_via_H_and_typology():
    placements = [
        {"x_m": 0.0, "y_m": 0.0, "H": 300.0, "usd": "tall"},          # tower by H
        {"x_m": 5.0, "y_m": 5.0, "H": 12.0, "typology": "lowrise",
         "usd": "short"},                                             # not a skyscraper
    ]
    out = tc.skyscraper_exposure(placements, lambda x, y: 0.3)
    assert len(out) == 1
    assert out[0]["usd"] == "tall"
    assert out[0]["height_class"] == "tower"


def test_skyscraper_exposure_empty_for_no_skyscrapers():
    placements = [{"x_m": 0.0, "y_m": 0.0, "H": 10.0, "usd": "u"}]
    assert tc.skyscraper_exposure(placements, lambda x, y: 0.9) == []


# ---------------------------------------------------------------------------
# entry_string() <-> parse_entry() round trip
# ---------------------------------------------------------------------------
def test_entry_string_round_trips_through_parse_entry():
    rec = tc.record(3, "/World/x", gf.GAC_DIR + "SM_Building_04.usd", "gac",
                    "SM_Building_04", 10.0, -5.0, 90.0, 28.4, 42.4, 48.0,
                    "urm", "midrise", 0.61, "T3",
                    {"bearing_deg": 224.7, "speed_frac": 0.6,
                     "cross_frac": -0.2, "over": False}, "slice", True, 4123)
    s = tc.entry_string(rec)
    assert s == "tornado:gac:SM_Building_04:T3:225:4123"
    parsed = tc.parse_entry(s)
    assert parsed == {"kind": "gac", "name": "SM_Building_04", "level": "T3",
                      "bearing_deg": 225, "seed": 4123}


def test_entry_string_empty_bearing_round_trips_to_none():
    rec = tc.record(0, "/World/x", "u", "kit", "office", 0.0, 0.0, 0.0,
                    20.0, 20.0, 30.0, "urm", "lowrise", 0.5, "T2",
                    None, "kit", True, 7)
    parsed = tc.parse_entry(tc.entry_string(rec))
    assert parsed["bearing_deg"] is None
    assert parsed["kind"] == "kit"
    assert parsed["name"] == "office"


def test_parse_entry_rejects_a_non_tornado_string():
    import pytest
    with pytest.raises(ValueError):
        tc.parse_entry("gac:SM_Building_04:F4:2:S,E:1013")


# ---------------------------------------------------------------------------
# ONE integration test on the REAL _lvl2 dump -- skipped, not failed, when
# the dump is absent.
# ---------------------------------------------------------------------------
_LVL2_DUMP = os.path.join(_SCENE_GEN_DIR, "_plans",
                          "city_placements_downtown_fire_1500_lvl2.json")


def test_real_lvl2_dump_manifest_and_checks():
    import pytest
    if not os.path.isfile(_LVL2_DUMP):
        pytest.skip("no _plans/city_placements_downtown_fire_1500_lvl2.json "
                   "present on this machine")

    manifest, extras = tcdr.solve("downtown_tornado_1500_lvl2", _LVL2_DUMP,
                                  verbose=False)

    assert manifest["records"], "the real _lvl2 dump should produce T1+ records"

    # every record's (kind, name) must not be pack-blacklisted.
    for r in manifest["records"]:
        reason = ufc._pack_blacklist_reason(
            r["kind"], r["name"] if r["kind"] in ("gac", "dtc") else None)
        assert reason is None, (r["i"], r["kind"], r["name"], reason)

    # no record over the height cap.
    for r in manifest["records"]:
        if r["H"] is not None:
            assert r["H"] <= tc.TORNADO_MAX_H_M + 1e-6, (r["i"], r["H"])

    # §4 check 7: the refused list is complete -- every house placement is
    # accounted for exactly once (records / refused / gate-passed-T0).
    ok7, detail7 = tcdr.check_7_refused_complete(manifest)
    assert ok7, detail7

    # §4 check 1: corridor coverage EVALUATES (returns real numbers) -- not
    # asserted to pass; the presets are tuned by hand against this number.
    ok1, detail1 = tcdr.check_1_corridor_coverage(manifest)
    assert isinstance(detail1["frac_t1plus_in_window"], (int, float))
    assert isinstance(detail1["track_frac_in_window"], (int, float))


# ---------------------------------------------------------------------------
# ROUND 2 (`_plans/urban_tornado_plan.md` §7) — the four new checks and the
# whole-plate window generalisation, all pure over a hand-built manifest.
# ---------------------------------------------------------------------------
def _rec(i, name, x, y, level, height_class, btype="urm", bakeable=True,
        kind="gac", H=40.0, W=20.0, D=20.0):
    return tc.record(i, f"/World/h{i}", f"u{i}", kind, name, x, y, 0.0,
                     W, D, H, btype, height_class,
                     {"T0": 0.02, "T1": 0.2, "T2": 0.45, "T3": 0.6, "T4": 0.8}[level],
                     level, None, "slice", bakeable, 1)


def test_check_r1_skyscraper_exposure_pass_and_fail():
    # `_synthetic_tcfg` (defined below) -- straight along +x, width 40 m,
    # peak 0.95, no wobble/noise: dead centre (0, 0) reads ~peak, 200 m off
    # the centreline reads ~0. Passed explicitly (`tcfg=`) so this test
    # needs no preset compile.
    tcfg = _synthetic_tcfg(origin_m=[0.0, 0.0], heading_deg=0.0)

    # "tall_ok": class tower (protected at any H), parked WELL off the
    # centreline -- passes.
    ok_manifest = {"region_m": [500.0, 500.0], "seed": 1,
                   "records": [_rec(0, "tall_ok", 0.0, 200.0, "T1", "tower")]}
    ok, detail = tcdr.check_r1_skyscraper_exposure(ok_manifest, tcfg=tcfg)
    assert ok, detail
    assert detail["n_protected"] == 1 and detail["n_over_cap"] == 0

    # "tall_bad": `highrise` at H=90 (>= SKYSCRAPER_PROTECTED_MIN_H_M, so
    # PROTECTED under the corrected rule), parked dead on the centreline --
    # fails. A SMALL footprint (W=D=2) so the corner sample stays close to
    # the (already high, ~0.76) centre reading -- this test is about the
    # protected-set rule, not about corner-vs-centre geometry (that is
    # `test_check_r1_skyscraper_exposure_corner_sampling` below).
    bad_manifest = {"region_m": [500.0, 500.0], "seed": 1,
                    "records": [_rec(0, "tall_bad", 0.0, 0.0, "T4",
                                     "highrise", H=90.0, W=2.0, D=2.0)]}
    ok, detail = tcdr.check_r1_skyscraper_exposure(bad_manifest, tcfg=tcfg)
    assert not ok
    assert detail["n_over_cap"] == 1
    assert detail["over"][0]["name"] == "tall_bad"


def test_check_r1_skyscraper_exposure_scans_refused_and_t0_too():
    # LEAD REVIEW REGRESSION (2026-09-01): the first version of this check
    # only ever saw `manifest["records"]`. A height-capped REFUSED
    # supertall standing dead in the core -- pristine by construction,
    # never a record -- must still FAIL this check; so must a T0 one
    # (below the T1 threshold, also never a record). Same `_synthetic_tcfg`
    # as above: (0, 0) is dead centre, i ~ peak (0.95).
    tcfg = _synthetic_tcfg(origin_m=[0.0, 0.0], heading_deg=0.0)

    # small footprints, dead centre -- this test is "is refused/t0 scanned
    # at all", not corner-vs-centre geometry (covered separately).
    refused_manifest = {
        "region_m": [500.0, 500.0], "seed": 1, "records": [],
        "refused": [{"x": 0.0, "y": 0.0, "W": 4.0, "D": 4.0, "yaw": 0.0,
                     "H": 312.0, "height_class": "tower",
                     "name": "SM_Building_16_refused"}],
    }
    ok, detail = tcdr.check_r1_skyscraper_exposure(refused_manifest, tcfg=tcfg)
    assert not ok, detail
    assert detail["n_over_cap"] == 1
    assert detail["n_from_refused"] == 1
    assert detail["over"][0]["name"] == "SM_Building_16_refused"

    t0_manifest = {
        "region_m": [500.0, 500.0], "seed": 1, "records": [],
        "t0_footprints": [{"x": 0.0, "y": 0.0, "W": 4.0, "D": 4.0,
                           "yaw": 0.0, "H": 220.0, "height_class": "tower",
                           "name": "some_t0_tower"}],
    }
    ok, detail = tcdr.check_r1_skyscraper_exposure(t0_manifest, tcfg=tcfg)
    assert not ok, detail
    assert detail["n_from_t0"] == 1
    assert detail["over"][0]["name"] == "some_t0_tower"


def test_check_composition_pass_and_fail():
    good = {"typology_counts": {"rowhouse": 40, "lowrise": 30, "midrise": 20,
                                "tower": 5, "highrise": 5},
           "n_protected_sky": 10}
    ok, detail = tcdr.check_composition(good)
    assert ok, detail
    assert detail["frac_low_mid"] == 0.9 and detail["frac_sky"] == 0.1

    bad = {"typology_counts": {"tower": 50, "highrise": 30, "rowhouse": 20},
          "n_protected_sky": 80}
    ok, detail = tcdr.check_composition(bad)
    assert not ok
    assert detail["frac_sky"] == 0.8

    # "_none" (street/park/off-block) is excluded from the LOW/MID
    # denominator, but n_protected_sky is an independent tally, not derived
    # from typology_counts at all any more (lead review) -- 0 here means
    # "no protected skyscraper", not "excluded".
    with_none = {"typology_counts": {"rowhouse": 80, "lowrise": 20, "_none": 500},
                "n_protected_sky": 0}
    ok, detail = tcdr.check_composition(with_none)
    assert ok, detail
    assert detail["n_zoned"] == 100

    # a manifest with no `n_protected_sky` key at all (an older manifest,
    # or `--check-only` on one written before this fix) degrades to 0
    # rather than crashing.
    no_key = {"typology_counts": {"rowhouse": 10}}
    ok, detail = tcdr.check_composition(no_key)
    assert detail["n_sky_protected"] == 0


def test_check_damage_capable_coverage():
    # 4 of 5 capable = 80%, the boundary itself (>=0.80 passes).
    good = {"records": [_rec(k, f"a{k}", 0, 0, "T2", "midrise", bakeable=True)
                        for k in range(4)]
                       + [_rec(4, "e", 0, 0, "T2", "midrise", bakeable=False,
                              kind="slice")]}
    ok, detail = tcdr.check_damage_capable_coverage(good)
    assert ok, detail
    assert detail["frac_capable"] == 0.8

    bad = {"records": [_rec(0, "a", 0, 0, "T2", "midrise", bakeable=False,
                            kind="slice"),
                       _rec(1, "b", 0, 0, "T2", "midrise", bakeable=True)]}
    ok, detail = tcdr.check_damage_capable_coverage(bad)
    assert not ok
    assert detail["frac_capable"] == 0.5

    ok, detail = tcdr.check_damage_capable_coverage({"records": []})
    assert not ok


def test_check_partial_collapse_presence():
    # below the peak threshold: not applicable, always reports PASS.
    low_peak = {"tornado_cfg": {"peak": 0.7}, "records": []}
    ok, detail = tcdr.check_partial_collapse_presence(low_peak)
    assert ok and detail["applicable"] is False

    # at/above threshold, too few T4 urm lowrise/midrise records: FAILS.
    high_peak_few = {"tornado_cfg": {"peak": 0.95}, "records": [
        _rec(0, "a", 0, 0, "T4", "lowrise", btype="urm")]}
    ok, detail = tcdr.check_partial_collapse_presence(high_peak_few)
    assert not ok and detail["applicable"] is True

    # at/above threshold, enough: PASSES. A T4 highrise/tower or a T4 rc
    # record must NOT count toward the requirement.
    high_peak_enough = {"tornado_cfg": {"peak": 0.95}, "records": [
        _rec(0, "a", 0, 0, "T4", "lowrise", btype="urm"),
        _rec(1, "b", 0, 0, "T4", "midrise", btype="urm"),
        _rec(2, "c", 0, 0, "T4", "lowrise", btype="urm"),
        _rec(3, "d", 0, 0, "T4", "tower", btype="urm"),      # excluded: tower
        _rec(4, "e", 0, 0, "T4", "midrise", btype="rc"),     # excluded: rc
    ]}
    ok, detail = tcdr.check_partial_collapse_presence(high_peak_enough)
    assert ok, detail
    assert detail["n_t4_urm_lowmid"] == 3


def test_check_1_corridor_coverage_whole_plate_window_skips_the_band():
    # `level is None` (the bench preset's own signal) -> the 8-30% band is
    # NOT enforced, only the track-crosses-window (>= 60%) requirement.
    manifest = {"level": None, "window": (-100.0, -100.0, 100.0, 100.0),
               "n_house_in_window": 50, "records": [], "refused": [],
               "track_frac_in_window": 1.0}
    ok, detail = tcdr.check_1_corridor_coverage(manifest)
    assert ok, detail
    assert detail["whole_plate_window"] is True

    # the track-crosses requirement still applies even under a whole-plate
    # window -- a track that misses its own plate is still a broken track.
    manifest["track_frac_in_window"] = 0.1
    ok, detail = tcdr.check_1_corridor_coverage(manifest)
    assert not ok


def test_window_for_preset_falls_back_to_region_for_unknown_presets():
    level, window = tcdr.window_for_preset("downtown_tornado_bench_500",
                                           region=(-10.0, -20.0, 10.0, 20.0))
    assert level is None
    assert window == (-10.0, -20.0, 10.0, 20.0)

    # a known 1500 m level preset is UNCHANGED by passing region -- the
    # level's own crop window always wins.
    level, window = tcdr.window_for_preset(
        "downtown_tornado_1500", region=(-750.0, -750.0, 750.0, 750.0))
    assert level == 1
    assert window == (-680.0, -320.0, 320.0, 680.0)

    # no region and an unknown preset -> (None, None), the round-1 shape.
    assert tcdr.window_for_preset("some_unknown_preset") == (None, None)


def _synthetic_tcfg(**over):
    """A fully self-contained, DETERMINISTIC `tn.resolve_cfg`-shaped dict
    (`edge_noise_m: 0.0` -> `intensity_field` takes its no-noise branch, so
    these tests need no seeded rng and no real preset compile) -- decoupled
    from the real bench preset on purpose, so a later preset/districts edit
    cannot silently change what this test asserts about the SEARCH
    MACHINERY itself."""
    from disaster import tornado as tn
    cfg = dict(tn.DEFAULTS)
    cfg.update({"width_m": 40.0, "core_frac": 0.3, "wobble_m": 0.0,
               "curvature_deg_per_km": 0.0, "edge_noise_m": 0.0,
               "peak": 0.95})
    cfg.update(over)
    return cfg


def test_corridor_full_width_in_plate_frac():
    plate = (-250.0, -250.0, 250.0, 250.0)

    # a track running straight through the plate centre, along +x: its full
    # width stays inside a 500 m square for virtually its entire length.
    tcfg_centre = _synthetic_tcfg(origin_m=[0.0, 0.0], heading_deg=0.0)
    frac_centre = tcdr.corridor_full_width_in_plate_frac(tcfg_centre, plate)
    assert frac_centre > 0.9

    # a track entering at a corner and immediately leaving it clips the
    # plate for only a small fraction of its own length.
    tcfg_corner = _synthetic_tcfg(origin_m=[240.0, 240.0], heading_deg=0.0)
    frac_corner = tcdr.corridor_full_width_in_plate_frac(tcfg_corner, plate)
    assert frac_corner < frac_centre


def test_evaluate_track_hard_constraints():
    plate = (-250.0, -250.0, 250.0, 250.0)

    # a tower at the origin; intact fabric far out on both flanks (x=-200
    # and x=+200) of a track running along +y (heading 90).
    buildings = [{"i": 0, "x": 0.0, "y": 0.0, "ok": True, "name": "tower0",
                 "height_class": "tower", "bakeable": True, "btype": "rc"}]
    for k in range(15):
        y = -190.0 + k * 20.0
        buildings.append({"i": k + 1, "x": -200.0, "y": y, "ok": True,
                          "name": f"lowL{k}", "height_class": "lowrise",
                          "bakeable": True, "btype": "urm"})
        buildings.append({"i": k + 16, "x": 200.0, "y": y, "ok": True,
                          "name": f"lowR{k}", "height_class": "lowrise",
                          "bakeable": True, "btype": "urm"})

    # a track straight through the origin (heading 90, along +y) puts the
    # tower dead on the centreline at high peak -- HARD constraint 2
    # (skyscraper exposure) must reject it.
    tcfg_through = _synthetic_tcfg(origin_m=[0.0, 0.0], heading_deg=90.0)
    assert tcdr._evaluate_track(buildings, tcfg_through, plate, 1) is None

    # the SAME heading, offset 150 m in x (still along +y, full width stays
    # inside the plate, both flanks' fabric at x=+-200 stays outside a
    # 40 m-wide corridor at x=150): tower and fabric are all outside the
    # corridor, so every hard constraint clears and a real score comes back.
    tcfg_away = _synthetic_tcfg(origin_m=[150.0, 0.0], heading_deg=90.0)
    result = tcdr._evaluate_track(buildings, tcfg_away, plate, 1)
    assert result is not None
    assert "score" in result and result["max_sky_i_raw"] <= tc.SKYSCRAPER_MAX_I
    assert result["n_left"] > 0 and result["n_right"] > 0


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
