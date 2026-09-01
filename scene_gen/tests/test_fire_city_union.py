#!/usr/bin/env python3
"""test_fire_city_union.py — the pure-python additions `tools/
fire_city_union.py` picked up for the baseline intensity ladder
(`scene_gen/_plans/baseline_fire_ladder.md`): `resolve_severity_target`,
`profile_roof_collapse_max`, `_parse_severity_spec`, the `PROFILES` table's
own structural shape, and the F6/F5c CEILING pass added to `rebalance_
severity` on 2026-09-01 (see that function's own docstring for the bug this
fixes — a target BELOW the union's natural F5c/F6 count silently did
nothing before this pass existed).

    python3 -m pytest scene_gen/tests/test_fire_city_union.py -q
    python3 scene_gen/tests/test_fire_city_union.py

Pure python, host-side, no `pxr`, no Kit, no Nucleus, and (unlike a real
`--profile` run against a full city dump) FAST: every test here uses either
no dump at all, or a tiny 4-8 building synthetic one built the same way
`test_fire_city_dry_run.py`'s own `_row_dump` does — the full `apply_
profile` pipeline against `_plans/fc_dump_500.json` costs ~90 s a profile
(three nested `auto_select_seeds` sweeps, each re-compiling the preset once
per candidate seed) and is deliberately NOT exercised here; it was run by
hand while building the ladder and its numbers are recorded in `scene_gen/
_plans/baseline_fire_ladder.md` instead.
"""
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _TOOLS_DIR)

import fire_city_union as fcu                              # noqa: E402
from disaster import gac_fire as _gf                        # noqa: E402

_DUMP_GAC = _gf.GAC_DIR + "SM_Building_04.usd"
LOWRISE_BLOCK = {"rect": [0.0, 0.0, 400.0, 200.0], "name": "lowrise"}


def _dump_placement(i, x, y, usd=_DUMP_GAC, W=20.0, D=15.0, H=42.3,
                    z=0.0, yaw=0.0, scale=0.01):
    return {"i": i, "cell": f"/World/stage/generated/house_{i}",
           "usd": usd, "x_m": x, "y_m": y, "z_m": z, "yaw_deg": yaw,
           "scale": scale, "category": "house", "axis_up": "Z",
           "W": W, "D": D, "H": H}


def _write_dump(path, placements, typology_blocks, n_total, seed=42,
               preset="downtown_fire_500", region_m=(500.0, 500.0)):
    doc = {
        "schema": "fire_city_placements_dump.v1",
        "preset": preset, "seed": seed,
        "region_m": list(region_m),
        "n_placements_total": n_total,
        "placements": placements,
        "typology": {"blocks": typology_blocks},
    }
    with open(path, "w") as fh:
        json.dump(doc, fh)
    return path


def _row_dump(path, seed=555, n_buildings=6, spacing=25.0):
    """A tiny row of GAC placements, well clear of each other -- just
    enough for `_burnable_geometry`/`load_placements_dump` to open
    successfully. `rebalance_severity`'s CEILING passes never touch `x`/`y`/
    `sides`, so the geometry only has to be VALID, not realistic."""
    xs = [20.0 + spacing * k for k in range(n_buildings)]
    placements = [_dump_placement(k, x, 20.0, W=20.0, D=15.0, H=42.3)
                 for k, x in enumerate(xs)]
    return _write_dump(path, placements, [LOWRISE_BLOCK],
                       n_total=n_buildings, seed=seed)


# ---------------------------------------------------------------------------
# resolve_severity_target
# ---------------------------------------------------------------------------
def test_resolve_severity_target_absolute_int_stays_absolute():
    target = fcu.resolve_severity_target({"F5c": 2}, n_achieved=100)
    assert target["F5c"] == 2   # NOT round(2 * 100)


def test_resolve_severity_target_fraction_rounds_against_n_achieved():
    target = fcu.resolve_severity_target({"F4": 0.15}, n_achieved=20)
    assert target["F4"] == 3    # round(0.15 * 20)


def test_resolve_severity_target_origin_only_is_always_exactly_one():
    for n in (1, 9, 500):
        target = fcu.resolve_severity_target({"F5": "origin_only"}, n_achieved=n)
        assert target["F5"] == 1


def test_resolve_severity_target_missing_keys_default_to_zero_or_one():
    # F4/F5c/F6 default to 0, F5 defaults to 1 (the mandatory origin) when
    # "collapse_visible" is not given -- see the function's own docstring.
    target = fcu.resolve_severity_target({}, n_achieved=50)
    assert target == {"F4": 0, "F5": 1, "F5c": 0, "F6": 0}


def test_resolve_severity_target_collapse_visible_splits_out_f5c_and_f6():
    target = fcu.resolve_severity_target(
        {"collapse_visible": 0.28, "F5c": 0.14, "F6": 0.05}, n_achieved=100)
    assert target["F5c"] == 14
    assert target["F6"] == 5
    assert target["F5"] == 28 - 14 - 5   # the remainder of the total


def test_resolve_severity_target_collapse_visible_floors_f5_at_one():
    # F5c + F6 alone already exceed (or equal) the collapse_visible total --
    # F5 must never go negative or to zero (the mandatory origin record).
    target = fcu.resolve_severity_target(
        {"collapse_visible": 0.10, "F5c": 3, "F6": 3}, n_achieved=10)
    assert target["F5"] == 1


def test_resolve_severity_target_rejects_bool_value():
    try:
        fcu.resolve_severity_target({"F4": True}, n_achieved=10)
        assert False, "expected TypeError"
    except TypeError:
        pass


# ---------------------------------------------------------------------------
# profile_roof_collapse_max
# ---------------------------------------------------------------------------
def test_profile_roof_collapse_max_explicit_wins():
    profile = {"roof_collapse_max": 2,
              "severity_frac": {"F5c": 0.9, "F6": 0.9}}   # would compute huge
    assert fcu.profile_roof_collapse_max(profile, n_target=100) == 2


def test_profile_roof_collapse_max_computed_from_severity_frac():
    profile = {"roof_collapse_max": None,
              "severity_frac": {"F5c": 0.14, "F6": 0.05}}
    # n_target=30 -> F5c round(4.2)=4, F6 round(1.5)=2 -> 6
    assert fcu.profile_roof_collapse_max(profile, n_target=30) == 6


# ---------------------------------------------------------------------------
# PROFILES table -- structural sanity (every level baseline_l1/l2/l3 needs)
# ---------------------------------------------------------------------------
def test_profiles_table_has_the_three_baseline_levels():
    assert set(fcu.PROFILES.keys()) == {"baseline_l1", "baseline_l2", "baseline_l3"}


def test_profiles_burn_frac_strictly_increases_l1_to_l3():
    l1, l2, l3 = (fcu.PROFILES[k]["burn_frac"] for k in
                 ("baseline_l1", "baseline_l2", "baseline_l3"))
    assert 0.0 < l1 < l2 < l3 < 1.0


def test_profiles_l1_has_no_roof_loss_at_all():
    l1 = fcu.PROFILES["baseline_l1"]
    assert l1["roof_collapse_max"] == 0
    assert l1["severity_frac"]["F5c"] == 0.0
    assert l1["severity_frac"]["F6"] == 0.0
    assert l1["severity_frac"]["F5"] == "origin_only"


def test_profiles_l2_matches_todays_measured_realism_mix():
    # F2:3 F3:18 F4:11 F5:2 F5c:4 F6:1 / 39 records, measured on the real
    # `fire_city_500m_39.json` manifest (`scene_gen/_plans/
    # fire_city_500m_progress.md`) -- F4 28%, collapse-visible (F5+F5c+F6)
    # 18%, F2/F3 the remaining 54%.
    l2 = fcu.PROFILES["baseline_l2"]["severity_frac"]
    assert l2["F4"] == 0.28
    assert l2["collapse_visible"] == 0.18


def test_profiles_l3_collapse_visible_stays_a_minority():
    l3 = fcu.PROFILES["baseline_l3"]["severity_frac"]
    assert l3["collapse_visible"] < 0.5


# ---------------------------------------------------------------------------
# _parse_severity_spec
# ---------------------------------------------------------------------------
def test_parse_severity_spec_mixed_int_and_float():
    spec = fcu._parse_severity_spec("F4:11,F5c:4,F6:1,collapse_visible:0.18")
    assert spec == {"F4": 11, "F5c": 4, "F6": 1, "collapse_visible": 0.18}
    assert isinstance(spec["F4"], int)
    assert isinstance(spec["collapse_visible"], float)


def test_parse_severity_spec_origin_only_kept_literal():
    spec = fcu._parse_severity_spec("F5:origin_only")
    assert spec == {"F5": "origin_only"}


def test_parse_severity_spec_ignores_blank_entries():
    spec = fcu._parse_severity_spec("F4:1,, F5c:2 ,")
    assert spec == {"F4": 1, "F5c": 2}


def test_parse_severity_spec_rejects_entry_without_colon():
    try:
        fcu._parse_severity_spec("F4:1,badentry")
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "badentry" in str(exc)


# ---------------------------------------------------------------------------
# rebalance_severity — the 2026-09-01 F6/F5c CEILING pass, regression-tested
#
# Before this pass, a `target` asking for FEWER F6/F5c than the union
# naturally already had (e.g. baseline Level 2's own `F6: 0` on top of a
# solve-time `roof_collapse_max` budget shared across F5c+F6 together) was
# silently ignored: pass 1 ("F6 then F5c GAIN") only ever ADDS when `want`
# exceeds the current count, never trims the other direction. Caught live
# against `fc_dump_500.json` (`baseline_l2`'s F6 stayed at 1 when the target
# said 0) before this test existed.
# ---------------------------------------------------------------------------
def _synthetic_records(levels, tmp_path_dump):
    """`levels` -> a minimal records list, one per level given, with just
    enough fields for the CEILING passes (`move`, sorted by `age_s`) --
    the GAIN pass's own `roof_eligible`/`street_facing` closures are never
    reached when `need <= 0` for every goal, which is true for every test
    below (targets only ever ask for FEWER F6/F5c than are already
    present)."""
    return [
        {"i": k, "x": 20.0 + 25.0 * k, "y": 20.0, "W": 20.0, "D": 15.0,
         "yaw_deg": 0.0, "level": lvl, "age_s": 1000.0 * (k + 1),
         "btype": "urm", "typology": "lowrise", "n_storeys": 4,
         "sides": ("S",)}
        for k, lvl in enumerate(levels)
    ]


def test_rebalance_severity_f6_ceiling_demotes_excess_to_f3(tmp_path):
    dump = _row_dump(str(tmp_path / "dump.json"), n_buildings=6)
    records = _synthetic_records(
        ["F6", "F6", "F5c", "F5c", "F4", "F3"], dump)

    diff = fcu.rebalance_severity(records, dump, {"F6": 0, "F5c": 1})

    from collections import Counter
    counts = Counter(r["level"] for r in records)
    assert counts["F6"] == 0, counts       # both F6 demoted
    assert counts["F5c"] == 1, counts      # one F5c demoted (excess of 1)
    assert counts["F3"] == 4, counts       # 2 (from F6) + 1 (from F5c) + the original F3
    assert len(diff) == 3


def test_rebalance_severity_leaves_f6_f5c_alone_when_target_omits_them(tmp_path):
    """Backward compatibility: a `target` dict that never mentions "F6"/
    "F5c" at all (the pre-2026-09-01 calling convention) must not touch
    either count -- `target.get(level)` is `None`, and both the gain pass
    and the new ceiling pass already skip on `None`."""
    dump = _row_dump(str(tmp_path / "dump2.json"), n_buildings=6)
    records = _synthetic_records(["F6", "F5c", "F4", "F4", "F3", "F3"], dump)

    diff = fcu.rebalance_severity(records, dump, {"F4": 1})

    from collections import Counter
    counts = Counter(r["level"] for r in records)
    assert counts["F6"] == 1, counts
    assert counts["F5c"] == 1, counts
    # F4 ceiling of 1 still fires -- unrelated to the F6/F5c omission
    assert counts["F4"] == 1, counts


def test_rebalance_severity_ceiling_prefers_youngest_excess():
    """Same rule the pre-existing F5/F4 ceiling already documents: the
    OLDEST record of the excess level survives, the YOUNGEST is demoted
    first."""
    dump_records = [
        {"i": 0, "level": "F6", "age_s": 9000.0},
        {"i": 1, "level": "F6", "age_s": 100.0},   # youngest -- demoted
    ]
    # rebalance_severity needs a real dump path only for _burnable_geometry
    # (never reached by the ceiling pass alone, since `need <= 0` for every
    # gain goal here) -- any file `load_placements_dump` can open works.
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        dump = _row_dump(os.path.join(d, "dump.json"), n_buildings=2)
        fcu.rebalance_severity(dump_records, dump, {"F6": 1})
    by_i = {r["i"]: r for r in dump_records}
    assert by_i[0]["level"] == "F6"    # oldest kept
    assert by_i[1]["level"] == "F3"    # youngest demoted


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
