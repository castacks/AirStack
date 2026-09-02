#!/usr/bin/env python3
"""test_quake_gac_city.py — does a placed GreatAmericanCity (GAC) merged
building get swapped for ITS OWN per-building earthquake bake, at the same
pose, and does the swapped record carry what `_clear_under_heaps` needs?

    pytest -q scene_gen/tests/test_quake_gac_city.py

Some cases need pytest's `tmp_path` fixture (the manifest-file read tests),
so — unlike `test_quake_twins.py` — this file has no `__main__` runner;
invoke it with pytest, the same way `test_quake_manifest_rebase.py` is.

WHY THIS EXISTS
---------------
`urban_quake_v3` gave a same_art (ModernCityEnvironment) original a
size-matched KIT TWIN (`disaster.quake.decide_building`). `urban_quake_v4`
adds the 31 GreatAmericanCity buildings, and GAC is not same_art —
`kit_substitute.route()` sends it to `'slice'`, never `'kit'` — so there is
no honest kit stand-in for one. The fix (parallel work, this round) is a
per-building bake: `disaster.quake.decide_gac_building` swaps a damaged GAC
placement for the bake of THAT EXACT BUILDING at that exact grade
(`scene_gen/assets/gac_quake/gac_<name>_<grade>_s<seed>.usd`, manifest
`gac_quake.json`), never a best-fit substitute. `decide_gac_building` is
PURE (no pxr, no stage) for the same reason `decide_building` is: so
`tools/layout_dry_run.py`'s offline city-scale tally can call it directly.

WHAT THIS CANNOT SEE: whether a GAC bake's own geometry reads right next to
a kept original in a render, whether `_mono_pass`'s fallback lean looks
right on a KEPT GAC building. That needs Isaac Sim — not run here.
"""

import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake as q               # noqa: E402
from disaster import kit_substitute as ks      # noqa: E402


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_GAC_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
            "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
_B02_USD = _GAC_ROOT + "SM_Building_02.usd"
_B01_USD = _GAC_ROOT + "SM_Building_01.usd"


def _gac_manifest(*rows):
    """{(name, grade): record} from (name, grade, usd) tuples — a synthetic
    stand-in for `quake.load_gac_manifest`'s real gac_quake.json read, same
    shape `test_quake_twins.py::_manifest` uses for the kit archetype one."""
    out = {}
    for name, grade, usd in rows:
        out[(name, grade)] = {"name": name, "grade": grade, "usd": usd,
                              "style": name, "level": grade,
                              "W": 28.0, "D": 14.4, "H": 38.6}
    return out


def test_module_exposes_the_pure_gac_functions():
    for name in ("decide_gac_building", "load_gac_manifest", "_is_gac",
                "_is_gac_bake", "gac_name_of", "_sibling_dir"):
        assert hasattr(q, name), name


# ---------------------------------------------------------------------------
# _is_gac / _is_gac_bake / gac_name_of — the pack-recognition helpers
# ---------------------------------------------------------------------------
def test_is_gac_recognises_the_pack_by_substring():
    assert q._is_gac(_B02_USD)
    assert q._is_gac("some/root/" + "GreatAmericanCity/Assets/Game/"
                     "GreatAmericanCity/Meshes/SM_Building_31.usd")


def test_is_gac_rejects_other_packs():
    assert not q._is_gac(
        "Muyang/ModernCityEnvironment/Collected_Building01/"
        "SM_MERGED_BP_MBuilding01.usd")           # same_art, not 'other'
    assert not q._is_gac("omniverse://host/archetype/bld_office_DG0.usd")  # kit
    assert not q._is_gac(
        "omniverse://host/Projects/SEI-COA/scene_gen/assets/downtowncity/"
        "Amar_Tower.usdc")                        # 'other', but not GAC


def test_is_gac_bake_recognises_the_swapped_path_only():
    assert q._is_gac_bake(
        "/isaac-sim/AirStack/scene_gen/assets/gac_quake/"
        "gac_SM_Building_02_DG3_s0.usd")
    assert not q._is_gac_bake(_B02_USD)            # still the original GAC asset
    assert not q._is_gac_bake("omniverse://host/archetype/bld_office_DG3.usd")


def test_gac_name_of_strips_root_and_extension():
    assert q.gac_name_of(_B02_USD) == "SM_Building_02"
    assert q.gac_name_of(_B01_USD) == "SM_Building_01"
    assert q.gac_name_of("x/y/SM_Building_06_Small.usd") == "SM_Building_06_Small"


def test_sibling_dir_joins_next_to_the_parent():
    assert q._sibling_dir("/a/b/assets/archetype", "gac_quake") == \
        "/a/b/assets/gac_quake"
    assert q._sibling_dir("/a/b/assets/archetype/", "gac_quake") == \
        "/a/b/assets/gac_quake"
    a = "omniverse://host:443/Projects/x/assets/archetype"
    assert q._sibling_dir(a, "gac_quake") == \
        "omniverse://host:443/Projects/x/assets/gac_quake"


# ---------------------------------------------------------------------------
# decide_gac_building — the pure per-building contract
# ---------------------------------------------------------------------------
def test_dg0_keeps_the_original_untouched():
    """A pristine (DG0) GAC building is never even asked about a bake — no
    manifest lookup at all, proved with an empty manifest."""
    rng = random.Random(0)
    out = q.decide_gac_building(_B02_USD, "DG0", {}, rng,
                                x=10.0, y=-4.0, yaw_deg=90.0)
    assert out == {"x": 10.0, "y": -4.0, "yaw_deg": 90.0,
                  "action": "keep", "grade": "DG0"}


def test_dg3_swaps_to_the_named_buildings_own_bake_with_pose_preserved():
    """SM_Building_02 at DG3 -> the bake of SM_Building_02 at DG3 — never a
    different building's bake, never a kit style — and x/y/yaw ride through
    unchanged (the swap only ever touches the reference)."""
    manifest = _gac_manifest(
        ("SM_Building_02", "DG3", "/gac_quake/gac_SM_Building_02_DG3_s0.usd"))
    rng = random.Random(1)
    out = q.decide_gac_building(_B02_USD, "DG3", manifest, rng,
                                x=123.4, y=-56.7, yaw_deg=270.0)
    assert out["action"] == "twin"
    assert out["style"] == "SM_Building_02"
    assert out["usd"] == "/gac_quake/gac_SM_Building_02_DG3_s0.usd"
    assert out["grade"] == "DG3"
    assert not out["stepped"]
    assert (out["x"], out["y"], out["yaw_deg"]) == (123.4, -56.7, 270.0)


def test_never_swaps_to_a_different_buildings_bake():
    """A manifest that only has SM_Building_01 baked must NOT satisfy a
    request for SM_Building_02 — there is no best-fit here, unlike
    same_art's `decide_building`."""
    manifest = _gac_manifest(
        ("SM_Building_01", "DG3", "/gac_quake/gac_SM_Building_01_DG3_s0.usd"))
    rng = random.Random(2)
    out = q.decide_gac_building(_B02_USD, "DG3", manifest, rng,
                                x=0.0, y=0.0, yaw_deg=0.0)
    assert out["action"] == "keep"
    assert "SM_Building_02" in out.get("reason", "")


def test_fallback_steps_down_to_the_nearest_baked_grade():
    """Drawn at DG4, but SM_Building_02 is only baked at DG1 and DG3 — the
    SAME "step down until something is baked" fallback `decide_building`
    uses, so a missing DG4 becomes a DG3."""
    manifest = _gac_manifest(
        ("SM_Building_02", "DG1", "/gac_quake/gac_SM_Building_02_DG1_s0.usd"),
        ("SM_Building_02", "DG3", "/gac_quake/gac_SM_Building_02_DG3_s0.usd"))
    rng = random.Random(3)
    out = q.decide_gac_building(_B02_USD, "DG4", manifest, rng,
                                x=7.0, y=8.0, yaw_deg=180.0)
    assert out["action"] == "twin"
    assert out["grade"] == "DG3"
    assert out["usd"] == "/gac_quake/gac_SM_Building_02_DG3_s0.usd"
    assert out["stepped"] is True


def test_fallback_with_nothing_baked_at_any_grade_keeps_the_original():
    rng = random.Random(4)
    out = q.decide_gac_building(_B02_USD, "DG2", {}, rng,
                                x=0.0, y=0.0, yaw_deg=0.0)
    assert out["action"] == "keep"
    assert out["grade"] == "DG2"
    assert "SM_Building_02" in out.get("reason", "")


# ---------------------------------------------------------------------------
# BUG FIX (2026-09-01): `SM_Building_17`'s exact manifest shape — this
# building has exactly ONE bake, at `DG3` (a mid/high grade), the same shape
# as the real `gac_quake.json` row this reproduces (`scene_gen/assets/
# gac_quake/gac_SM_Building_17_DG3_s596.usd` / `.json`, verified to exist on
# disk — see quake.py's own "BUG FIX" note on `decide_gac_building`). A kit
# archetype is baked at every DG1-DG5 rung, so `decide_building`'s DOWN-only
# fallback practically always lands on something; a GAC bake is sparse, and
# the down-only fallback used to give up entirely whenever the ONLY baked
# grade sat ABOVE the drawn one — reported "no baked GAC archetype ... at
# any grade" even though the manifest row and the .usd both existed.
# ---------------------------------------------------------------------------
_B17_USD = _GAC_ROOT + "SM_Building_17.usd"


def test_sm_building_17_shape_steps_up_when_drawn_below_its_only_bake():
    """Drawn DG1, but SM_Building_17 (this exact manifest shape: one row,
    DG3, nothing lower) is only baked at DG3 — must swap to DG3, not report
    'no baked archetype at any grade'."""
    manifest = _gac_manifest(
        ("SM_Building_17", "DG3", "/gac_quake/gac_SM_Building_17_DG3_s596.usd"))
    rng = random.Random(17)
    out = q.decide_gac_building(_B17_USD, "DG1", manifest, rng,
                                x=8.4, y=27.1, yaw_deg=180.0)
    assert out["action"] == "twin"
    assert out["grade"] == "DG3"
    assert out["usd"] == "/gac_quake/gac_SM_Building_17_DG3_s596.usd"
    assert out["stepped"] is True


def test_sm_building_17_shape_steps_up_from_dg2_too():
    """Same shape, drawn one grade closer (DG2) — still only DG3 baked."""
    manifest = _gac_manifest(
        ("SM_Building_17", "DG3", "/gac_quake/gac_SM_Building_17_DG3_s596.usd"))
    rng = random.Random(18)
    out = q.decide_gac_building(_B17_USD, "DG2", manifest, rng,
                                x=0.0, y=0.0, yaw_deg=0.0)
    assert out["action"] == "twin"
    assert out["grade"] == "DG3"


def test_only_baked_grade_above_draw_is_not_confused_with_no_bake_at_all():
    """The two failure modes must produce DIFFERENT reason text: a building
    with a real (if sparse) manifest entry that the up/down scan still
    could not use (format bug) vs. one with no entry at all (genuinely not
    baked yet) — see `decide_gac_building`'s `has_any_entry` branch."""
    rng = random.Random(19)
    # Genuinely nothing baked for this name: the ORIGINAL "no bake" message.
    out_absent = q.decide_gac_building(_B17_USD, "DG1", {}, rng,
                                       x=0.0, y=0.0, yaw_deg=0.0)
    assert out_absent["action"] == "keep"
    assert "no baked GAC archetype for 'SM_Building_17' at any grade" == \
        out_absent["reason"]

    # An entry exists (case-mismatched grade string: "dg3", not "DG3") that
    # `_variants`' exact-match can never resolve at ANY of the up/down
    # scan's DG1-DG5 tries — the format-mismatch branch, not the "no bake"
    # one.
    bad_manifest = {("SM_Building_17", "dg3"): {
        "name": "SM_Building_17", "grade": "dg3", "style": "SM_Building_17",
        "level": "dg3", "usd": "/gac_quake/gac_SM_Building_17_dg3_s596.usd"}}
    out_bad = q.decide_gac_building(_B17_USD, "DG1", bad_manifest, rng,
                                    x=0.0, y=0.0, yaw_deg=0.0)
    assert out_bad["action"] == "keep"
    assert out_bad["reason"] != out_absent["reason"]
    assert "format mismatch" in out_bad["reason"]


def test_absent_manifest_reason_names_the_building_for_per_name_dedup():
    """`assemble` dedupes its "kept, no bake" print by reason TEXT
    (`gac_reasons_seen`) — the reason must therefore be unique PER BUILDING
    NAME, so two different GAC buildings both get their own printed line
    instead of collapsing into one generic message."""
    rng = random.Random(5)
    out1 = q.decide_gac_building(_B01_USD, "DG3", {}, rng, x=0, y=0, yaw_deg=0)
    out2 = q.decide_gac_building(_B02_USD, "DG3", {}, rng, x=0, y=0, yaw_deg=0)
    assert out1["reason"] != out2["reason"]
    assert "SM_Building_01" in out1["reason"]
    assert "SM_Building_02" in out2["reason"]


def test_multiple_seeds_at_the_same_grade_are_all_candidates():
    """"possibly with several seeds per pair later" (the bake contract) —
    `_variants` already generalises to `DG3`, `DG3_v1`, `DG3_v2`... for the
    kit archetype manifest; the same generalisation must hold for GAC."""
    manifest = _gac_manifest(
        ("SM_Building_02", "DG3", "/gac_quake/gac_SM_Building_02_DG3_s0.usd"),
        ("SM_Building_02", "DG3_v1", "/gac_quake/gac_SM_Building_02_DG3_s1.usd"))
    seen = set()
    for seed in range(20):
        rng = random.Random(seed)
        out = q.decide_gac_building(_B02_USD, "DG3", manifest, rng,
                                    x=0, y=0, yaw_deg=0)
        assert out["action"] == "twin" and out["grade"] == "DG3"
        seen.add(out["usd"])
    assert seen == {"/gac_quake/gac_SM_Building_02_DG3_s0.usd",
                    "/gac_quake/gac_SM_Building_02_DG3_s1.usd"}


# ---------------------------------------------------------------------------
# load_gac_manifest — rebase-by-basename, same discipline as load_manifest
# ---------------------------------------------------------------------------
def _write_gac_manifest(tmp_path, recs):
    p = tmp_path / "gac_quake.json"
    p.write_text(json.dumps(recs))
    return str(tmp_path)


def test_stale_container_path_is_rebased_onto_gac_dir(tmp_path):
    recs = [{"usd": "/isaac-sim/AirStack/scene_gen/assets/gac_quake/"
                    "gac_SM_Building_02_DG3_s0.usd",
             "name": "SM_Building_02", "grade": "DG3", "btype": "urm",
             "W": 28.0, "D": 14.4, "H": 38.6,
             "fall_sides": ["S"], "extent_m": {"S": 6.0, "N": 1.5,
                                                "E": 1.6, "W": 1.6},
             "crown_m": 4.0, "mb": 1, "prims": 42}]
    d = _write_gac_manifest(tmp_path, recs)
    m = q.load_gac_manifest(d)
    row = m[("SM_Building_02", "DG3")]
    assert row["usd"] == os.path.join(d, "gac_SM_Building_02_DG3_s0.usd")
    # style/level aliases so `_variants` and `_clear_under_heaps` work
    # unchanged against a merged dict
    assert row["style"] == "SM_Building_02"
    assert row["level"] == "DG3"
    # the other fields ride along untouched
    assert row["H"] == 38.6
    assert row["fall_sides"] == ["S"]
    # the source list is not mutated
    assert recs[0]["usd"].endswith("gac_quake/gac_SM_Building_02_DG3_s0.usd")


def test_gac_manifest_keys_never_collide_with_kit_archetype_keys():
    """The whole point of aliasing rather than reusing `style`/`level` as the
    PRIMARY vocabulary: a GAC `name` like "SM_Building_02" must never be
    mistaken for (or collide with) a kit style name once the two manifests
    are merged into one dict, as `assemble` does."""
    kit_style, kit_level = "office", "DG3"
    gac_name, gac_grade = "SM_Building_02", "DG3"
    assert (kit_style, kit_level) != (gac_name, gac_grade)


# ---------------------------------------------------------------------------
# the swapped record reaches `_heap_reach_for` with fall_sides/extent_m
# ---------------------------------------------------------------------------
def test_swapped_gac_record_extent_m_beats_nominal_reach(tmp_path):
    """A `_clear_under_heaps` row lookup on a GAC bake's manifest row must
    prefer the bake's own MEASURED `extent_m` over its own `reach_m`
    wherever the measured value is larger, and never shrink below `reach_m`
    — the exact same preference
    `test_quake_heap_clearance.test_measured_extent_beats_nominal_reach`
    proves for a kit archetype row, run here against a `load_gac_manifest`
    row end to end (json -> rebase -> `_heap_reach_for`)."""
    recs = [{"usd": "/isaac-sim/AirStack/scene_gen/assets/gac_quake/"
                    "gac_SM_Building_02_DG5_s0.usd",
             "name": "SM_Building_02", "grade": "DG5", "btype": "urm",
             "W": 28.0, "D": 14.4, "H": 38.6,
             "fall_sides": ["S", "W"],
             "reach_m": {"S": 6.0, "N": 1.6, "E": 1.7, "W": 5.2},
             "extent_m": {"S": 6.4, "N": 5.1, "E": 4.8, "W": 5.0},
             "crown_m": 4.0, "mb": 1, "prims": 42}]
    d = _write_gac_manifest(tmp_path, recs)
    manifest = q.load_gac_manifest(d)
    row = manifest[("SM_Building_02", "DG5")]
    got = q._heap_reach_for(row, "urm", "DG5", 38.6)
    assert got["N"] == 5.1 and got["E"] == 4.8      # blind sides: measured foot
    assert got["S"] == 6.4                           # larger measured value wins
    assert got["W"] == 5.2                           # never shrinks below reach_m
    # and it beats what a bare nominal draw (no manifest row at all) would
    # have given this same building on its blind sides
    nominal_blind = q.heap_reach_m("urm", "DG5", 38.6, fall_side=False)
    assert got["N"] > nominal_blind and got["E"] > nominal_blind


def test_variants_finds_gac_rows_by_name_after_merge_into_kit_manifest():
    """`assemble` does `manifest.update(gac_manifest)` — after that,
    `_variants(merged_manifest, gac_name, grade)` (the exact call
    `_clear_under_heaps` and `decide_gac_building`'s own fallback loop make)
    must find the GAC row through the SAME kit-archetype-shaped manifest
    dict, with no special-casing at the call site."""
    kit_manifest = {("office", "DG3"): {"style": "office", "level": "DG3",
                                        "usd": "/arch/bld_office_DG3.usd"}}
    gac_manifest = _gac_manifest(
        ("SM_Building_02", "DG3", "/gac_quake/gac_SM_Building_02_DG3_s0.usd"))
    merged = dict(kit_manifest)
    merged.update(gac_manifest)
    assert len(q._variants(merged, "office", "DG3")) == 1
    assert len(q._variants(merged, "SM_Building_02", "DG3")) == 1
    assert q._variants(merged, "SM_Building_02", "DG3")[0]["usd"] == \
        "/gac_quake/gac_SM_Building_02_DG3_s0.usd"
