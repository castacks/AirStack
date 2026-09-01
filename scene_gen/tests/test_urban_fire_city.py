#!/usr/bin/env python3
"""test_urban_fire_city.py — the CITY-scale burnable-set predicate and
bake-kind mapper (`disaster/urban_fire_city.py`), work item #3 of
`scene_gen/_plans/urban_fire_city_plan.md`.

    python3 scene_gen/tests/test_urban_fire_city.py
    pytest -q scene_gen/tests/test_urban_fire_city.py

Pure python, host-side, no Kit, no Nucleus, no stage — every assertion here
runs against a SYNTHETIC layout (three blocks: `lowrise`, `brick_midrise`,
`tower`) and synthetic placements, exercising:

  * the four gates of `burnable()`, in order, with the EXACT refusal reason
    for each: not-a-house, off the block map (a street), inside a no-fire
    (tower) district, `kit_substitute.route()`'s own reason (Muyang
    DownTown), and the NEW gate `bake_kind()` adds that `route()` does not
    (an AEC brownstone routes to 'slice' but has no `fire_bake.KINDS` entry);
  * `bake_kind()` on one real asset per pack: GAC, downtowncity, an
    already-kit build, a matchable `same_art` MCE merged asset, and an
    unmatchable one (refused, never sliced);
  * the district rule, MUTATION-CHECKED: a placement that is burnable at one
    (x, y) is refused once moved into the tower block, with no other change;
  * `entry_string()` round-tripping through `fire_bake.parse_entry`, with and
    without `origin`/`sides`;
  * `damaged_manifest()`'s `seed = seed_base + 31 * i` (i = ordinal position
    in the OUTPUT manifest, not the placement index) and its `refused` list
    reporting both an out-of-range plan record and a district refusal, each
    with a reason;
  * `no_fire_assets()`'s live read of `usds.buildings.tower`/`highrise` vs.
    its `FALLBACK_NO_FIRE_ASSETS` when a config carries neither pool.

Every asset URL below is a plausible Nucleus path built the same way the real
presets build one, but none of it is opened — no `Usd.Stage`, no network.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import fire_bake as fb                      # noqa: E402
from disaster import gac_fire as gf                        # noqa: E402
from disaster import urban_fire_city as ufc                # noqa: E402

# ---------------------------------------------------------------------------
# Fixtures shared by every test
# ---------------------------------------------------------------------------
LOWRISE = (0.0, 0.0, 100.0, 100.0)
BRICK_MIDRISE = (100.0, 0.0, 250.0, 120.0)
TOWER = (250.0, 0.0, 400.0, 150.0)

LAYOUT = {"_typology_of": {
    LOWRISE: "lowrise",
    BRICK_MIDRISE: "brick_midrise",
    TOWER: "tower",
}}

GAC_USD = gf.GAC_DIR + "SM_Building_04.usd"        # 31.5 x 28.0 x 42.3 m
DTC_USD = gf.DTC_DIR + "Building_09.usdc"
DTC_BUILDING_11_USD = gf.DTC_DIR + "Building_11.usdc"       # user-blacklisted
DTC_CARVED_04_USD = gf.DTC_DIR + "Carved_04.usdc"            # blacklisted prefix
DTC_AMAR_TOWER_USD = gf.DTC_DIR + "Amar_Tower.usdc"          # NOT blacklisted
KIT_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "scene_gen/assets/archetype/bld_apartment_DG0.usd")
AEC_BROWNSTONE_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/"
                      "Demos/AEC/Buildings/Brownstone/Brownstone_01.usd")
MUYANG_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Muyang/"
             "DownTown/Assets/BG_Building_A.usd")
SAME_ART_OK_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                   "SEI-COA/ModernCityEnvironment/Collected_Building01/"
                   "SM_MERGED_BP_MBuilding01.usd")
SAME_ART_REFUSED_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                        "Projects/SEI-COA/ModernCityEnvironment/"
                        "Collected_Building02/SM_MERGED_BP_MBuilding02.usd")


def _house(usd, x, y, prim_path, yaw_deg=0.0, z_m=0.0):
    return {"category": "house", "usd": usd, "x_m": x, "y_m": y,
            "yaw_deg": yaw_deg, "z_m": z_m, "prim_path": prim_path}


# ---------------------------------------------------------------------------
# typology_at
# ---------------------------------------------------------------------------
def test_typology_at_finds_each_block():
    assert ufc.typology_at(LAYOUT, 50.0, 50.0) == "lowrise"
    assert ufc.typology_at(LAYOUT, 150.0, 60.0) == "brick_midrise"
    assert ufc.typology_at(LAYOUT, 300.0, 75.0) == "tower"


def test_typology_at_returns_none_off_the_block_map():
    assert ufc.typology_at(LAYOUT, 900.0, 900.0) is None
    assert ufc.typology_at({"_typology_of": {}}, 0.0, 0.0) is None
    assert ufc.typology_at({}, 0.0, 0.0) is None


# ---------------------------------------------------------------------------
# bake_kind -- one representative case per pack, by exact outcome
# ---------------------------------------------------------------------------
def test_bake_kind_gac():
    assert ufc.bake_kind(GAC_USD, 31.5, 28.0, 42.3, "urm") == \
        ("gac", "SM_Building_04")


def test_bake_kind_dtc():
    assert ufc.bake_kind(DTC_USD, 25.0, 25.0, 40.0, "rc") == \
        ("dtc", "Building_09")


def test_bake_kind_already_kit():
    assert ufc.bake_kind(KIT_USD, 20.0, 20.0, 30.0, "urm") == \
        ("kit", "apartment")


def test_bake_kind_same_art_matchable():
    # the exact measured triple `kit_substitute.check()` asserts routes to
    # 'commercial_mid' -- reused here rather than a fresh guess.
    assert ufc.bake_kind(SAME_ART_OK_USD, 28.5, 18.5, 29.0, None) == \
        ("kit", "commercial_mid")


def test_bake_kind_same_art_refused_never_slices():
    kind, reason = ufc.bake_kind(SAME_ART_REFUSED_USD, 60.0, 140.0, 302.0, None)
    assert kind is None
    assert reason and "kit style" in reason


def test_bake_kind_aec_brownstone_refused_with_reason():
    kind, reason = ufc.bake_kind(AEC_BROWNSTONE_USD, 20.0, 15.0, 18.0, "urm")
    assert kind is None
    assert "fire_bake.KINDS" in reason


def test_bake_kind_muyang_refused_as_unburnable():
    kind, reason = ufc.bake_kind(MUYANG_USD, 45.0, 45.0, 90.0, None)
    assert kind is None
    assert "unburnable" in reason


# ---------------------------------------------------------------------------
# burnable() -- the four gates, in order, with exact reasons
# ---------------------------------------------------------------------------
def test_burnable_gac_in_lowrise():
    p = _house(GAC_USD, 50.0, 50.0, "/World/stage/generated/house_0_10")
    ok, rec = ufc.burnable(LAYOUT, p, {})
    assert ok
    assert rec["kind"] == "gac"
    assert rec["asset"] == "SM_Building_04"
    assert rec["style"] is None
    assert rec["typology"] == "lowrise"
    assert rec["cell"] == "/World/stage/generated/house_0_10"
    assert rec["x"] == 50.0 and rec["y"] == 50.0


def test_burnable_dtc_in_brick_midrise():
    p = _house(DTC_USD, 150.0, 60.0, "/World/stage/generated/house_1_11")
    ok, rec = ufc.burnable(LAYOUT, p, {})
    assert ok
    assert rec["kind"] == "dtc"
    assert rec["asset"] == "Building_09"
    assert rec["style"] is None
    assert rec["typology"] == "brick_midrise"


def test_burnable_already_kit_asset():
    p = _house(KIT_USD, 160.0, 40.0, "/World/stage/generated/house_2_12")
    ok, rec = ufc.burnable(LAYOUT, p, {})
    assert ok
    assert rec["kind"] == "kit"
    assert rec["style"] == "apartment"
    assert rec["asset"] is None


def test_burnable_same_art_matchable():
    p = _house(SAME_ART_OK_USD, 20.0, 20.0, "/World/stage/generated/house_3_13")
    ok, rec = ufc.burnable(LAYOUT, p, {SAME_ART_OK_USD: (28.5, 18.5, 29.0)})
    assert ok
    assert rec["kind"] == "kit"
    assert rec["style"] == "commercial_mid"


def test_burnable_gate1_rejects_non_house():
    p = dict(_house(GAC_USD, 50.0, 50.0, "x"), category="tree")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "not a building placement" in reason


def test_burnable_gate2_rejects_a_street_placement():
    p = _house(GAC_USD, 900.0, 900.0, "x")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "outside every zoned block" in reason


def test_burnable_gate2_accepts_the_tower_district_now_burnable():
    """2026-08-31 policy: the blanket "no fire in a tower/highrise block"
    ban is LIFTED -- gate 2 only checks that a typology exists at all now.
    The collapse cap that replaces the ban is a LEVEL policy applied in
    `damaged_manifest`, not a candidacy gate -- see `test_urban_fire_
    spread.py`'s height-class tests and `test_damaged_manifest_caps_a_
    skyscraper_record_instead_of_refusing_it` below."""
    p = _house(GAC_USD, 300.0, 75.0, "x")
    ok, rec = ufc.burnable(LAYOUT, p, {})
    assert ok
    assert rec["typology"] == "tower"
    assert rec["kind"] == "gac"


def test_burnable_gate3_rejects_muyang_via_routes_own_reason():
    p = _house(MUYANG_USD, 60.0, 60.0, "x")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "route refused" in reason
    assert "unburnable" in reason


def test_burnable_gate3_rejects_unmatchable_same_art():
    p = _house(SAME_ART_REFUSED_USD, 70.0, 20.0, "x")
    ok, reason = ufc.burnable(LAYOUT, p, {SAME_ART_REFUSED_USD: (60.0, 140.0, 302.0)})
    assert not ok
    assert "route refused" in reason


def test_burnable_gate4_rejects_aec_brownstone():
    p = _house(AEC_BROWNSTONE_USD, 170.0, 70.0, "x")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "fire_bake.KINDS" in reason


# ---------------------------------------------------------------------------
# gate 5: the pack blacklist (2026-08-31) -- `gac_fire.PACKS["dtc"]
# ["blacklist"] = ("Carved_", "Building_11")` already existed for a BENCH
# row picker; the city path (`burnable()`) never consulted it, so three
# `dtc:Building_11` records reached a live bake manifest despite the user
# explicitly disliking that building. Blacklisted buildings are refused as
# CANDIDATES (firebreaks), never assigned a level at all.
# ---------------------------------------------------------------------------
def test_burnable_gate5_rejects_building_11():
    p = _house(DTC_BUILDING_11_USD, 150.0, 60.0, "/World/stage/generated/house_9_19")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "blacklisted" in reason
    assert "Building_11" in reason


def test_burnable_gate5_rejects_carved_prefix():
    p = _house(DTC_CARVED_04_USD, 150.0, 60.0, "/World/stage/generated/house_9_20")
    ok, reason = ufc.burnable(LAYOUT, p, {})
    assert not ok
    assert "blacklisted" in reason
    assert "Carved_" in reason


def test_burnable_gate5_accepts_non_blacklisted_dtc():
    for usd, name in ((DTC_USD, "Building_09"), (DTC_AMAR_TOWER_USD, "Amar_Tower")):
        p = _house(usd, 150.0, 60.0, "/World/stage/generated/house_9_21")
        ok, rec = ufc.burnable(LAYOUT, p, {})
        assert ok, (name, rec)
        assert rec["kind"] == "dtc"
        assert rec["asset"] == name


def test_pack_blacklist_reason_reads_gac_fire_packs_live():
    assert ufc._pack_blacklist_reason("dtc", "Building_11") is not None
    assert ufc._pack_blacklist_reason("dtc", "Carved_17") is not None
    assert ufc._pack_blacklist_reason("dtc", "Building_09") is None
    assert ufc._pack_blacklist_reason("dtc", "Amar_Tower") is None
    # kinds with no "blacklist" entry at all never match anything
    assert ufc._pack_blacklist_reason("gac", "Building_11") is None
    assert ufc._pack_blacklist_reason("kit", "tower") is None
    assert ufc._pack_blacklist_reason("dtc", None) is None


# ---------------------------------------------------------------------------
# gate 6: the max-fire-height cap (2026-08-31 user policy review of the live
# 500 m city -- "don't let anything taller than the amar tower be on fire").
# `FIRE_MAX_H_M` defaults to 232.0 m, set just above `Amar_Tower`'s own
# measured 231.4 m (`_plans/dtc_buildings.json`) so Amar itself stays
# burnable while the genuine monsters above it (GAC's `SM_Building_16` at
# 312.0 m, `SM_Building_31` at 302.2 m) are refused.
# ---------------------------------------------------------------------------
def test_height_cap_reason_never_fires_on_an_unmeasured_height():
    assert ufc._height_cap_reason(None) is None


def test_height_cap_reason_boundary_is_inclusive():
    assert ufc._height_cap_reason(ufc.FIRE_MAX_H_M) is None
    reason = ufc._height_cap_reason(ufc.FIRE_MAX_H_M + 0.01)
    assert reason is not None
    assert "taller than the fire-height cap" in reason


def test_burnable_gate6_accepts_amar_tower_at_its_own_measured_height():
    p = _house(DTC_AMAR_TOWER_USD, 150.0, 60.0,
              "/World/stage/generated/house_9_22")
    ok, rec = ufc.burnable(LAYOUT, p, {DTC_AMAR_TOWER_USD: (42.3, 48.8, 231.4)})
    assert ok, rec
    assert rec["kind"] == "dtc" and rec["asset"] == "Amar_Tower"
    assert rec["H"] == 231.4


def test_burnable_gate6_rejects_a_302m_gac_monster():
    """GAC's `SM_Building_31` (302.2 m) -- one of the "302 m and 250+ m
    monsters" the user flagged live in the 500 m city -- must be refused,
    with a reason naming both the measured height and the cap."""
    p = _house(GAC_USD, 150.0, 60.0, "/World/stage/generated/house_9_23")
    ok, reason = ufc.burnable(LAYOUT, p, {GAC_USD: (60.3, 142.2, 302.2)})
    assert not ok
    assert "taller than the fire-height cap" in reason
    assert "302.2" in reason
    assert str(ufc.FIRE_MAX_H_M) in reason or "232.0" in reason


def test_burnable_gate6_does_not_refuse_an_unmeasured_building():
    """An empty `size_of` (H unknown) is the common case in every OTHER
    test in this file -- gate 6 must never turn any of them into a false
    refusal."""
    p = _house(GAC_USD, 50.0, 50.0, "/World/stage/generated/house_0_10")
    ok, rec = ufc.burnable(LAYOUT, p, {})
    assert ok and rec["H"] is None


# ---------------------------------------------------------------------------
# the district no longer gates candidacy, MUTATION-CHECKED
# ---------------------------------------------------------------------------
def test_moving_a_burnable_building_into_the_tower_block_only_changes_its_typology():
    p = _house(GAC_USD, 50.0, 50.0, "/World/stage/generated/house_0_10")
    ok_before, rec_before = ufc.burnable(LAYOUT, p, {})
    assert ok_before and rec_before["kind"] == "gac"
    assert rec_before["typology"] == "lowrise"

    p["x_m"], p["y_m"] = 300.0, 75.0     # into the tower block, nothing else changes
    ok_after, rec_after = ufc.burnable(LAYOUT, p, {})
    assert ok_after
    assert rec_after["typology"] == "tower"
    assert rec_after["kind"] == rec_before["kind"]


# ---------------------------------------------------------------------------
# entry_string <-> fire_bake.parse_entry
# ---------------------------------------------------------------------------
def test_entry_string_round_trips_with_origin_and_sides():
    rec = {"kind": "gac", "asset": "SM_Building_04", "style": None,
           "level": "F4", "origin": 2, "sides": ["S", "E"], "seed": 1013}
    text = ufc.entry_string(rec)
    parsed = fb.parse_entry(text)
    assert parsed["kind"] == "gac"
    assert parsed["name"] == "SM_Building_04"
    assert parsed["level"] == "F4"
    assert parsed["origin"] == 2
    assert parsed["sides"] == ("S", "E")
    assert parsed["seed"] == 1013


def test_entry_string_round_trips_kit_with_no_origin_or_sides():
    rec = {"kind": "kit", "asset": None, "style": "commercial_mid",
           "level": "F1", "origin": None, "sides": None, "seed": 55}
    text = ufc.entry_string(rec)
    parsed = fb.parse_entry(text)
    assert parsed["kind"] == "kit"
    assert parsed["name"] == "commercial_mid"
    assert parsed["level"] == "F1"
    assert parsed["origin"] is None
    assert parsed["sides"] is None
    assert parsed["seed"] == 55


def test_entry_string_matches_out_stem_kind_naming():
    # `fire_bake.out_stem` and `entry_string` must agree on which kind token
    # names a bake -- both read straight off `record["kind"]`/`entry["kind"]`,
    # and `out_stem` folds `origin`/`sides` in when present (`o<origin>` then
    # the sides in ring order) -- see `fire_bake.out_stem`'s own docstring.
    rec = {"kind": "dtc", "asset": "Amar_Tower", "style": None,
           "level": "F5c", "origin": 0, "sides": ["S"], "seed": 7}
    parsed = fb.parse_entry(ufc.entry_string(rec))
    stem = fb.out_stem(parsed)
    assert stem == "dtc_Amar_Tower_F5c_o0_S_s7"


# ---------------------------------------------------------------------------
# damaged_manifest -- seeds, the schema, and refused-with-reasons
# ---------------------------------------------------------------------------
def _manifest_fixture():
    placements = [
        _house(GAC_USD, 50.0, 50.0, "/World/stage/generated/house_0_10"),
        _house(DTC_USD, 150.0, 60.0, "/World/stage/generated/house_1_11"),
        _house(GAC_USD, 300.0, 75.0, "/World/stage/generated/house_2_12"),
    ]
    plan_records = [
        {"i": 0, "level": "F5", "origin": 0, "sides": ["S"], "t_ignite_s": 0.0,
         "age_s": 900.0, "via": None, "how": "origin"},
        {"i": 1, "level": "F2", "origin": 0, "sides": ["W"], "t_ignite_s": 300.0,
         "age_s": 600.0, "via": 0, "how": "radiation"},
        {"i": 2, "level": "F1", "origin": 0, "sides": ["N"], "t_ignite_s": 400.0,
         "age_s": 500.0, "via": 0, "how": "attached"},          # in the tower block
        {"i": 99, "level": "F1", "origin": 0, "sides": [], "t_ignite_s": 10.0,
         "age_s": 10.0, "via": 0, "how": "spot"},                # out of range
    ]
    return placements, plan_records


def test_damaged_manifest_keeps_only_burnable_records_in_order():
    """Tower is burnable now (record 2), so only the out-of-range record
    (99) is refused."""
    placements, plan_records = _manifest_fixture()
    manifest, refused = ufc.damaged_manifest(LAYOUT, placements, plan_records, 1000)
    assert [m["i"] for m in manifest] == [0, 1, 2]
    assert manifest[0]["kind"] == "gac" and manifest[0]["asset"] == "SM_Building_04"
    assert manifest[1]["kind"] == "dtc" and manifest[1]["asset"] == "Building_09"
    assert manifest[2]["kind"] == "gac" and manifest[2]["typology"] == "tower"
    assert manifest[0]["level"] == "F5"
    assert manifest[0]["how"] == "origin"
    assert manifest[1]["how"] == "radiation"
    assert len(refused) == 1 and refused[0]["i"] == 99


def test_damaged_manifest_seeds_follow_seed_base_plus_31_times_i():
    placements, plan_records = _manifest_fixture()
    manifest, _refused = ufc.damaged_manifest(LAYOUT, placements, plan_records, 1000)
    assert [m["seed"] for m in manifest] == [1000, 1031, 1062]

    manifest2, _ = ufc.damaged_manifest(LAYOUT, placements, plan_records, 7)
    assert [m["seed"] for m in manifest2] == [7, 38, 69]


def test_damaged_manifest_refuses_only_the_out_of_range_record():
    placements, plan_records = _manifest_fixture()
    _manifest, refused = ufc.damaged_manifest(LAYOUT, placements, plan_records, 1000)
    assert len(refused) == 1
    by_i = {r["i"]: r["reason"] for r in refused}
    assert "out-of-range" in by_i[99]


def test_damaged_manifest_caps_a_skyscraper_record_instead_of_refusing_it():
    """2026-08-31 policy: the tower-block record (F6, in the old no-fire
    district) is now BURNABLE but CAPPED -- skyscraper: fire only, never
    any collapse -- rather than refused. `cap_level_for_class` degrades
    F6 -> F5c -> F5 (skyscraper bans both F5c and F6); F5 is not a
    `ROOF_LEVELS` outcome, so `enforce_roof_eligibility` is a no-op here."""
    placements, plan_records = _manifest_fixture()
    plan_records[2]["level"] = "F6"
    manifest, refused = ufc.damaged_manifest(LAYOUT, placements, plan_records, 1000)
    tower = next(m for m in manifest if m["i"] == 2)
    assert tower["height_class"] == "skyscraper"
    assert tower["level"] == "F5"
    assert not any(r["i"] == 2 for r in refused)


# ---------------------------------------------------------------------------
# the height-class layer of damaged_manifest: rank cap, roof eligibility,
# and the roof-outcome share budget (2026-08-31 policy). `LAYOUT` above has
# no `midrise`/`highrise` blocks, so this uses its own fixture.
# ---------------------------------------------------------------------------
ROWHOUSE = (0.0, 0.0, 100.0, 100.0)
MIDRISE = (100.0, 0.0, 250.0, 100.0)
HIGHRISE = (250.0, 0.0, 400.0, 100.0)
HC_LAYOUT = {"_typology_of": {ROWHOUSE: "rowhouse", MIDRISE: "midrise",
                              HIGHRISE: "highrise"}}


def _hc_fixture():
    """Six placements naming F6 (raw "full collapse"): four `rowhouse`
    (low, roof-eligible), one `midrise` (mid_high), one `highrise`
    (skyscraper). Whatever survives to the manifest is entirely the
    height-class policy's doing."""
    placements = [
        _house(GAC_USD, 50.0, 50.0, "/h0"), _house(GAC_USD, 20.0, 20.0, "/h1"),
        _house(GAC_USD, 70.0, 20.0, "/h2"), _house(GAC_USD, 20.0, 70.0, "/h3"),
        _house(GAC_USD, 150.0, 50.0, "/h4"), _house(GAC_USD, 300.0, 50.0, "/h5"),
    ]

    def _rec(i, via):
        return {"i": i, "level": "F6", "origin": 0, "sides": ["S"],
               "t_ignite_s": float(i), "age_s": 12000.0, "via": via,
               "how": "origin" if via is None else "attached"}

    plan_records = [_rec(0, None), _rec(1, 0), _rec(2, 0), _rec(3, 0),
                    _rec(4, 0), _rec(5, 0)]
    return placements, plan_records


def test_mid_high_and_skyscraper_never_show_a_roof_opening_outcome():
    """(3) mid/high/skyscraper classes: never a roof-opening outcome --
    both the midrise and the highrise record name F6, and both must end at
    F5, never F5c or F6, regardless of the roof-outcome share budget."""
    placements, plan_records = _hc_fixture()
    manifest, refused = ufc.damaged_manifest(HC_LAYOUT, placements,
                                             plan_records, 4242)
    assert not refused
    by_i = {m["i"]: m for m in manifest}
    assert by_i[4]["height_class"] == "mid_high"
    assert by_i[4]["level"] == "F5"
    assert by_i[5]["height_class"] == "skyscraper"
    assert by_i[5]["level"] == "F5"
    for m in manifest:
        if m["height_class"] != "low":
            assert m["level"] not in ("F5c", "F6"), m


def test_roof_outcomes_are_eligible_only_for_the_low_timber_class():
    """(1) roof-affecting outcomes are eligible ONLY for the low-rise/
    brownstone/timber class -- every non-low record above degrades away
    from F6/F5c regardless of the share budget being wide open."""
    placements, plan_records = _hc_fixture()
    manifest, _refused = ufc.damaged_manifest(HC_LAYOUT, placements,
                                              plan_records, 4242,
                                              roof_collapse_max=99)
    for m in manifest:
        if m["level"] in ("F5c", "F6"):
            assert m["height_class"] == "low", m


def test_roof_collapse_share_cap_limits_the_count_across_the_manifest():
    """(2) the SHARE cap: of the four eligible (low-class) records, at most
    `roof_collapse_max` may show a roof outcome at all -- the rest degrade
    to F5, deterministically for a given seed, and the fire's own origin is
    always one of the kept ones."""
    placements, plan_records = _hc_fixture()
    manifest, _refused = ufc.damaged_manifest(HC_LAYOUT, placements,
                                              plan_records, 4242,
                                              roof_collapse_max=2)
    rowhouse = [m for m in manifest if m["i"] in (0, 1, 2, 3)]
    kept = [m["i"] for m in rowhouse if m["level"] == "F6"]
    demoted = [m["i"] for m in rowhouse if m["level"] == "F5"]
    assert len(kept) == 2 and len(demoted) == 2
    assert 0 in kept, "the origin should always be kept over the shuffle"

    # deterministic given the same seed_base
    manifest2, _ = ufc.damaged_manifest(HC_LAYOUT, placements, plan_records,
                                        4242, roof_collapse_max=2)
    assert [(m["i"], m["level"]) for m in manifest2] == \
        [(m["i"], m["level"]) for m in manifest]

    # a wide-open budget keeps every eligible record
    manifest_big, _ = ufc.damaged_manifest(HC_LAYOUT, placements, plan_records,
                                           4242, roof_collapse_max=4)
    assert sum(1 for m in manifest_big if m["i"] in (0, 1, 2, 3)
              and m["level"] == "F6") == 4

    # a budget of exactly zero keeps NONE, including the origin
    manifest_zero, _ = ufc.damaged_manifest(HC_LAYOUT, placements, plan_records,
                                            4242, roof_collapse_max=0)
    assert not any(m["level"] == "F6" for m in manifest_zero
                  if m["i"] in (0, 1, 2, 3))

    # roof_collapse_max=None means "no budget at all" -- every eligible
    # record is left alone
    manifest_none, _ = ufc.damaged_manifest(HC_LAYOUT, placements, plan_records,
                                            4242, roof_collapse_max=None)
    assert sum(1 for m in manifest_none if m["i"] in (0, 1, 2, 3)
              and m["level"] == "F6") == 4


def test_roof_collapse_max_defaults_to_two():
    assert ufc.ROOF_COLLAPSE_MAX_DEFAULT == 2
    import inspect
    assert inspect.signature(ufc.damaged_manifest).parameters[
        "roof_collapse_max"].default == ufc.ROOF_COLLAPSE_MAX_DEFAULT


# ---------------------------------------------------------------------------
# no_fire_assets: live read vs. fallback
# ---------------------------------------------------------------------------
def test_no_fire_assets_reads_the_preset_pools_live():
    cfg = {"usds": {"buildings": {
        "tower": [{"usd": "a/b/office_tower.usdc"}, "a/b/SM_Building_09.usd"],
        "highrise": ["a/b/SM_Building_99.usd"],
    }}}
    got = ufc.no_fire_assets(cfg)
    assert got == frozenset({"office_tower", "SM_Building_09", "SM_Building_99"})


def test_no_fire_assets_falls_back_when_pools_are_absent():
    got = ufc.no_fire_assets({})
    assert got == frozenset(ufc.FALLBACK_NO_FIRE_ASSETS)
    assert "SM_Building_31" in got
    assert "Amar_Tower" in got


def test_no_fire_assets_ignores_stale_hardcoding():
    # a preset that has RETIRED a tower model must not still exclude it --
    # the whole point of reading live rather than hardcoding.
    cfg = {"usds": {"buildings": {"tower": ["a/b/brand_new_tower.usdc"],
                                  "highrise": []}}}
    got = ufc.no_fire_assets(cfg)
    assert got == frozenset({"brand_new_tower"})
    assert "SM_Building_31" not in got


# ---------------------------------------------------------------------------
# the module's own host-side check()
# ---------------------------------------------------------------------------
def test_module_self_check_passes():
    assert ufc.check(verbose=False) == []


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
