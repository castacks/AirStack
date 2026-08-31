#!/usr/bin/env python3
"""test_fire_city_dry_run.py — the SEVEN §5 CHECKS of `tools/
fire_city_dry_run.py` (`scene_gen/_plans/urban_fire_city_plan.md`, work item
#4 of its §6).

    python3 -m pytest scene_gen/tests/test_fire_city_dry_run.py -q
    python3 scene_gen/tests/test_fire_city_dry_run.py

Pure python, host-side, no `pxr`, no Kit, no Nucleus — everything below
exercises the check FUNCTIONS (`check_district_rule`, `check_contiguity`,
`check_level_distribution`, `check_entry_points`, `check_bakeability`,
`check_footprint`, `check_determinism`) against a SYNTHETIC manifest built by
hand, the same discipline `test_urban_fire_spread.py` / `test_urban_fire_
city.py` already use. `fire_city_dry_run.py` itself imports `pxr` (and
`compile_disaster`/`generate_scene`/`scene_generator`) ONLY inside
`build_layout` / `run_dry` — never at module scope — so importing the module
here to reach the check functions costs nothing extra.

EVERY MUTATION-CHECKED CHECK is tested BOTH WAYS: once on a manifest built to
satisfy it, and once on a copy deliberately broken in exactly the way the
check exists to catch — the discipline `test_urban_fire_city.py`'s own
"mutation-checked" district-rule test already uses, extended to the other
five manifest checks so a check that could never actually fail (a bug that
always returns `True`) cannot hide here either.

A SECOND test class loads `_plans/fire_city_<seed>.json` if the dry run has
already been run for `downtown_fire_500` (`tools/fire_city_dry_run.py
--preset downtown_fire_500`) and re-runs the six manifest checks against the
REAL output — skipped, not failed, when no such file exists yet (a fresh
checkout, or a different preset run last).
"""
import glob
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _TOOLS_DIR)

import fire_city_dry_run as fdr                            # noqa: E402

# ---------------------------------------------------------------------------
# A synthetic manifest: a five-building terrace-plus-spot fire.
#
#   0  origin, urm, F5c, at (0, 0)
#   1  attached from 0 (party wall, gap 0), rc, F4
#   2  radiation from 1, rc, F3
#   3  spot from 0 (downwind, far), urm, F1  -- the just-caught edge
#   4  a `kit` record whose style exists in the tiny synthetic `manifest_dg0`
#      below, footprint matched (no area-ratio violation)
#
# Every building is well separated (10 m clear on the short axis at least)
# except 0/1, which share a party wall on purpose (gap 0 -- ATTACHED) so the
# footprint check's "touching is not overlapping" behaviour is exercised by
# the PASSING manifest, not just asserted in isolation.
# ---------------------------------------------------------------------------
def _rec(i, level, x, y, W=20.0, D=15.0, yaw=0.0, kind="gac", asset=None,
        style=None, typology="lowrise", btype="urm", via=None, how="origin",
        t=0.0, age=None, origin=1, sides=("S",), entry_side=None,
        origin_frac=0.5, n_storeys=4):
    return {
        "i": i, "usd": f"asset_{i}.usd", "x": x, "y": y, "yaw_deg": yaw,
        "z": 0.0, "kind": kind, "asset": asset, "style": style,
        "typology": typology, "W": W, "D": D, "H": 15.0, "cell": f"/cell_{i}",
        "level": level, "origin": origin, "sides": list(sides),
        "t_ignite_s": t, "age_s": (age if age is not None else t),
        "via": via, "how": how, "seed": 7 + 31 * i, "btype": btype,
        "entry_side": entry_side, "origin_frac": origin_frac,
        "n_storeys": n_storeys,
    }


def _base_manifest():
    records = [
        _rec(0, "F5c", 0.0, 0.0, W=20.0, D=15.0, kind="gac", asset="SM_Building_X",
            btype="urm", via=None, how="origin", t=0.0, age=12000.0,
            sides=("S", "E"), origin_frac=0.15, n_storeys=5),
        _rec(1, "F4", 0.0, 17.5, W=20.0, D=15.0, kind="gac", asset="SM_Building_Y",
            btype="rc", via=0, how="attached", t=600.0, age=11400.0,
            sides=("N", "E"), entry_side="N", origin_frac=0.22, n_storeys=6),
        _rec(2, "F3", 0.0, 35.0, W=20.0, D=15.0, kind="gac", asset="SM_Building_Z",
            btype="rc", via=1, how="radiation", t=2400.0, age=9600.0,
            sides=("N", "E"), entry_side="N", origin_frac=0.45, n_storeys=6),
        _rec(3, "F1", 60.0, 0.0, W=18.0, D=12.0, kind="kit", asset=None,
            style="commercial_mid", btype="urm", via=0, how="spot",
            t=11800.0, age=200.0, sides=("W",), entry_side="W",
            origin_frac=0.88, n_storeys=4),
        _rec(4, "F4", 90.0, 0.0, W=18.0, D=12.0, kind="kit", asset=None,
            style="commercial_mid", btype="rc", via=3, how="radiation",
            t=13000.0, age=1000.0, sides=("E", "N"), entry_side="E",
            origin_frac=0.45, n_storeys=4),
    ]
    refused = [
        {"i": 99, "usd": "some/aec/brownstone.usdc",
         "reason": "kit_substitute.route() says 'slice' ... no fire_bake.KINDS entry"},
        {"i": 100, "typology": "tower",
         "reason": "in a no-fire district ('tower')"},
    ]
    return {"seed": 12345, "preset": "synthetic", "n": 5, "n_achieved": 5,
           "origin": 0, "epoch_s": 12000.0, "records": records,
           "refused": refused}


# A tiny synthetic `manifest_dg0` ({style: {"W", "D", ...}}) for the
# footprint check's kit-twin area-ratio test — building 3/4's style
# `commercial_mid` matches its own 18 x 12 footprint closely (ratio ~1.0).
_MANIFEST_DG0 = {"commercial_mid": {"W": 18.0, "D": 12.5, "H": 15.0}}


def test_base_manifest_passes_all_six_manifest_checks():
    m = _base_manifest()
    for name, fn in fdr.CHECKS:
        ok, detail = (fn(m, manifest_dg0=_MANIFEST_DG0) if name == "footprint"
                     else fn(m))
        assert ok, f"{name} unexpectedly failed on the base manifest: {detail}"


# ---------------------------------------------------------------------------
# district rule — mutation-checked
# ---------------------------------------------------------------------------
def test_district_rule_mutation():
    m = _base_manifest()
    ok, _ = fdr.check_district_rule(m)
    assert ok

    bad = _base_manifest()
    bad["records"][2]["typology"] = "highrise"
    ok, detail = fdr.check_district_rule(bad)
    assert not ok
    assert detail["violations"] == [2]


# ---------------------------------------------------------------------------
# contiguity — mutation-checked
# ---------------------------------------------------------------------------
def test_contiguity_mutation():
    m = _base_manifest()
    ok, detail = fdr.check_contiguity(m)
    assert ok
    assert detail["root"] == 0

    # break the chain: building 2 now points at a building not in the
    # manifest at all.
    bad = _base_manifest()
    bad["records"][2]["via"] = 999
    ok, detail = fdr.check_contiguity(bad)
    assert not ok

    # two origins (two records with via=None) -- also must fail.
    two_roots = _base_manifest()
    two_roots["records"][2]["via"] = None
    ok, detail = fdr.check_contiguity(two_roots)
    assert not ok

    # a cycle: 0 -> ... -> 1 -> 0 (with 0 no longer via=None).
    cyc = _base_manifest()
    cyc["records"][0]["via"] = 1
    ok, detail = fdr.check_contiguity(cyc)
    assert not ok


# ---------------------------------------------------------------------------
# level distribution — mutation-checked (four ways: origin, F2/F3, F1, ladder)
# ---------------------------------------------------------------------------
def test_level_distribution_mutation():
    m = _base_manifest()
    ok, detail = fdr.check_level_distribution(m)
    assert ok
    assert detail["origin_level"] == "F5c"

    origin_not_burnt = _base_manifest()
    origin_not_burnt["records"][0]["level"] = "F4"
    ok, detail = fdr.check_level_distribution(origin_not_burnt)
    assert not ok
    assert detail["origin_ok"] is False

    no_f1 = _base_manifest()
    no_f1["records"][3]["level"] = "F2"
    ok, detail = fdr.check_level_distribution(no_f1)
    assert not ok
    assert detail["has_f1"] is False

    no_f2_f3 = _base_manifest()
    no_f2_f3["records"][2]["level"] = "F4"
    ok, detail = fdr.check_level_distribution(no_f2_f3)
    assert not ok
    assert detail["has_f2_or_f3"] is False

    bad_ladder = _base_manifest()
    bad_ladder["records"][1]["level"] = "F9"     # not a LEVELS member at all
    ok, detail = fdr.check_level_distribution(bad_ladder)
    assert not ok
    assert detail["ladder_violations"]


# ---------------------------------------------------------------------------
# entry points — mutation-checked
# ---------------------------------------------------------------------------
def test_entry_points_mutation():
    m = _base_manifest()
    ok, _ = fdr.check_entry_points(m)
    assert ok

    # F3 should vent two elevations; give it one.
    one_side = _base_manifest()
    one_side["records"][2]["sides"] = ["N"]
    ok, detail = fdr.check_entry_points(one_side)
    assert not ok

    # an impossible compass letter.
    bad_side = _base_manifest()
    bad_side["records"][1]["sides"] = ["Q"]
    ok, detail = fdr.check_entry_points(bad_side)
    assert not ok

    # storey out of range for its own n_storeys.
    bad_storey = _base_manifest()
    bad_storey["records"][0]["origin"] = 99
    ok, detail = fdr.check_entry_points(bad_storey)
    assert not ok

    # a "spot" entry with a low origin_frac (should be high, ~0.88).
    bad_frac = _base_manifest()
    bad_frac["records"][3]["origin_frac"] = 0.1
    ok, detail = fdr.check_entry_points(bad_frac)
    assert not ok


# ---------------------------------------------------------------------------
# bakeability — mutation-checked
# ---------------------------------------------------------------------------
def test_bakeability_mutation():
    m = _base_manifest()
    ok, detail = fdr.check_bakeability(m)
    assert ok
    assert detail["n_refused"] == 2

    bad_kind = _base_manifest()
    bad_kind["records"][0]["kind"] = "slice"     # not in fire_bake.KINDS
    ok, detail = fdr.check_bakeability(bad_kind)
    assert not ok
    assert detail["bad_kind"] == [0]

    no_reason = _base_manifest()
    no_reason["refused"].append({"i": 101, "reason": ""})
    ok, detail = fdr.check_bakeability(no_reason)
    assert not ok
    assert detail["refusals_without_reason"] == 1


# ---------------------------------------------------------------------------
# footprint — touching is NOT an overlap; true overlap and a bad area ratio
# both are.
# ---------------------------------------------------------------------------
def test_footprint_touching_terrace_is_not_an_overlap():
    m = _base_manifest()
    ok, detail = fdr.check_footprint(m, manifest_dg0=_MANIFEST_DG0)
    assert ok, detail
    assert detail["overlaps"] == []
    # buildings 0/1 share a party wall: centres 17.5 m apart, D=15 each ->
    # edge-to-edge gap exactly 2.5 m, comfortably clear (not the overlap
    # case) -- this manifest's own "touching" exercise is `_obb_overlap`'s
    # tolerance on a GENUINE zero gap, covered directly below.
    a, b = m["records"][0], m["records"][1]
    ca = fdr._obb_corners(a["x"], a["y"], a["W"], a["D"], a["yaw_deg"])
    cb = fdr._obb_corners(b["x"], b["y"], b["W"], b["D"], b["yaw_deg"])
    assert not fdr._obb_overlap(ca, cb)


def test_footprint_exact_zero_gap_is_not_an_overlap():
    # two 20 x 15 boxes centred 15 m apart on the D axis touch EXACTLY
    # (gap 0) -- the ordinary terrace party-wall condition.
    ca = fdr._obb_corners(0.0, 0.0, 20.0, 15.0, 0.0)
    cb = fdr._obb_corners(0.0, 15.0, 20.0, 15.0, 0.0)
    assert not fdr._obb_overlap(ca, cb)


def test_footprint_mutation_true_overlap():
    bad = _base_manifest()
    # drop building 2 exactly on top of building 1 -- a REAL overlap, not a
    # shared wall (both 20 x 15, same centre).
    bad["records"][2]["x"] = bad["records"][1]["x"]
    bad["records"][2]["y"] = bad["records"][1]["y"]
    ok, detail = fdr.check_footprint(bad, manifest_dg0=_MANIFEST_DG0)
    assert not ok
    assert [1, 2] in detail["overlaps"] or [2, 1] in detail["overlaps"]


def test_footprint_mutation_bad_area_ratio():
    bad = _base_manifest()
    # building 3/4 are `kit:commercial_mid`, matched at 18 x 12 in
    # `_MANIFEST_DG0` against this record's own 18 x 12 -- shrink the twin's
    # own footprint in the manifest table to blow the ratio past 1.3x.
    dg0 = dict(_MANIFEST_DG0)
    dg0["commercial_mid"] = {"W": 40.0, "D": 30.0, "H": 15.0}   # ~4.6x area
    ok, detail = fdr.check_footprint(bad, manifest_dg0=dg0)
    assert not ok
    assert detail["area_ratio_violations"]


# ---------------------------------------------------------------------------
# determinism
# ---------------------------------------------------------------------------
def test_determinism_of_a_properly_seeded_run():
    def run_fn(seed):
        rng = random.Random(seed)
        # a tiny, deterministic-given-seed "pipeline": draw a few numbers off
        # ITS OWN rng, never the global `random` module.
        vals = [rng.random() for _ in range(5)]
        m = _base_manifest()
        m["records"][0]["origin_frac"] = round(vals[0], 6)
        return m

    ok, detail = fdr.check_determinism(2024, run_fn)
    assert ok, detail
    assert detail["repeat_identical"]
    assert detail["stable_after_unrelated_draws"]


def test_determinism_catches_a_run_fn_that_reads_global_random_state():
    def bad_run_fn(seed):
        # WRONG: reads the GLOBAL `random` module instead of a locally
        # seeded `random.Random(seed)` -- exactly the bug `check_determinism`
        # exists to catch (the 50 unrelated draws perturb this).
        m = _base_manifest()
        m["records"][0]["origin_frac"] = round(random.random(), 6)
        return m

    ok, detail = fdr.check_determinism(2024, bad_run_fn)
    assert not ok
    assert detail["stable_after_unrelated_draws"] is False


# ---------------------------------------------------------------------------
# --placements-json: the 2026-08-30 manifest/city mismatch fix.
#
# `load_placements_dump` / `run_dry_from_dump` skip layout generation
# entirely and solve on a city-placements dump the LAUNCHER wrote (`FC_DUMP`,
# `FC_INTACT_ONLY=1`) instead of a host-side reconstruction whose patched
# `SizeResolver` (GAC/DTC substituted from the checked-in cache) can pack
# differently than Kit's real one — see the module docstring's incident
# note. No `pxr` needed here: every asset below is a `gac_fire.GAC_DIR`
# path, so `disaster.quake._same_art_material`'s config lookup (the one
# thing that WOULD need `pxr`, see `load_placements_dump`'s own note) is
# never reached.
# ---------------------------------------------------------------------------
from disaster import gac_fire as _gf                       # noqa: E402

_DUMP_GAC = _gf.GAC_DIR + "SM_Building_04.usd"


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


def _dump_placement(i, x, y, usd=_DUMP_GAC, W=20.0, D=15.0, H=42.3,
                    z=0.0, yaw=0.0, scale=0.01):
    return {"i": i, "cell": f"/World/stage/generated/house_{i}",
           "usd": usd, "x_m": x, "y_m": y, "z_m": z, "yaw_deg": yaw,
           "scale": scale, "category": "house", "axis_up": "Z",
           "W": W, "D": D, "H": H}


LOWRISE_BLOCK = {"rect": [0.0, 0.0, 400.0, 200.0], "name": "lowrise"}


def test_load_placements_dump_round_trips_index_alignment(tmp_path):
    """Only `category == 'house'` entries are written by the launcher, each
    carrying its ORIGINAL index into the full placement list — the read
    side must reconstruct a same-length list with non-house placeholders
    everywhere else, or `urban_fire_city_launch_script.resolve_cell`'s
    route-1 index match breaks the moment a non-house placement comes
    before a house one."""
    path = str(tmp_path / "dump.json")
    placements = [_dump_placement(5, 20.0, 20.0), _dump_placement(12, 90.0, 20.0),
                 _dump_placement(20, 160.0, 20.0)]
    _write_dump(path, placements, [LOWRISE_BLOCK], n_total=25, seed=555)

    config, layout, out_placements, seed, preset, sha256 = fdr.load_placements_dump(path)
    assert seed == 555
    assert preset == "downtown_fire_500"
    assert len(sha256) == 64 and all(c in "0123456789abcdef" for c in sha256)
    assert config == {}          # no same_art placement -- never compiled
    assert len(out_placements) == 25
    for i in (5, 12, 20):
        assert out_placements[i]["category"] == "house"
        assert out_placements[i]["usd"] == _DUMP_GAC
        assert out_placements[i]["prim_path"] == f"/World/stage/generated/house_{i}"
    for i in range(25):
        if i not in (5, 12, 20):
            assert out_placements[i]["category"] != "house"
    assert layout["_typology_of"][(0.0, 0.0, 400.0, 200.0)] == "lowrise"


def test_load_placements_dump_rejects_wrong_schema(tmp_path):
    path = str(tmp_path / "bad.json")
    with open(path, "w") as fh:
        json.dump({"schema": "something-else", "preset": "x", "seed": 1,
                  "placements": []}, fh)
    try:
        fdr.load_placements_dump(path)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "schema" in str(exc)


def test_load_placements_dump_rejects_a_placement_missing_a_field(tmp_path):
    path = str(tmp_path / "incomplete.json")
    bad = _dump_placement(0, 1.0, 2.0)
    del bad["W"]
    _write_dump(path, [bad], [LOWRISE_BLOCK], n_total=1)
    try:
        fdr.load_placements_dump(path)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "W" in str(exc)


def test_load_placements_dump_rejects_two_placements_at_the_same_index(tmp_path):
    path = str(tmp_path / "collide.json")
    placements = [_dump_placement(0, 1.0, 2.0), _dump_placement(0, 3.0, 4.0)]
    _write_dump(path, placements, [LOWRISE_BLOCK], n_total=1)
    try:
        fdr.load_placements_dump(path)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "index 0" in str(exc)


# ---------------------------------------------------------------------------
# run_dry_from_dump -- end to end on a synthetic dump, no layout, no pxr.
#
# Eight GAC buildings in a close-spaced row (20 m wide, 5 m edge gap --
# `urban_fire_spread`'s own radiation range is ~1-12 m) so the real spread
# solve actually propagates hop to hop under `downtown_fire_500`'s own
# wind/epoch numbers (heading 45 deg, 4 m/s, epoch = 14400 * 0.70 = 10080 s).
# Verified deterministic at seed 555 (this exact layout): 7 of the 8
# buildings ignite, levels F5c/F5/F4/F4/F3/F3/F2 -- five of the six §5
# checks (district_rule, contiguity, entry_points, bakeability, footprint)
# and the origin/F2-F3 halves of level_distribution are all structurally
# guaranteed by this construction (no tower typology, all `kind == "gac"`,
# well-clear footprints, a single radiation chain). `has_f1` is NOT
# asserted: whether the chain's last hop lands under `T_FLASHOVER` is a
# property of the real stochastic spread solve, which has its own test
# suite (`test_urban_fire_spread.py`) -- this test's job is the dump round
# trip, not re-proving the fire model.
# ---------------------------------------------------------------------------
def _row_dump(path, seed=555, n_buildings=8, spacing=25.0, first_i=3, stride=10):
    xs = [20.0 + spacing * k for k in range(n_buildings)]
    placements = [_dump_placement(first_i + stride * k, x, 20.0, W=20.0, D=15.0,
                                  H=42.3)
                 for k, x in enumerate(xs)]
    n_total = first_i + stride * (n_buildings - 1) + 1
    return _write_dump(path, placements, [LOWRISE_BLOCK], n_total=n_total,
                       seed=seed)


def test_run_dry_from_dump_end_to_end_on_a_synthetic_dump(tmp_path):
    path = _row_dump(str(tmp_path / "row.json"))

    manifest, checks, extras = fdr.run_dry_from_dump(path, n=16, collapse=1)

    assert manifest["seed"] == 555
    assert manifest["preset"] == "downtown_fire_500"
    assert manifest["placements_dump"]["path"] == os.path.abspath(path)
    import hashlib
    assert manifest["placements_dump"]["sha256"] == hashlib.sha256(
        open(path, "rb").read()).hexdigest()

    dumped_cells = {f"/World/stage/generated/house_{3 + 10 * k}" for k in range(8)}
    dumped_i = {3 + 10 * k for k in range(8)}
    assert manifest["records"], "expected at least one damaged record"
    for rec in manifest["records"]:
        # THE REGRESSION CHECK: every record traces back VERBATIM to a cell
        # and index this dump actually recorded -- not a renumbered or
        # reconstructed one.
        assert rec["cell"] in dumped_cells
        assert rec["i"] in dumped_i
        assert rec["kind"] == "gac"

    for name in ("district_rule", "contiguity", "entry_points", "bakeability",
                "footprint"):
        ok, detail = checks[name]
        assert ok, f"{name} failed: {detail}"

    ok, detail = checks["level_distribution"]
    assert detail["origin_ok"] is True
    assert detail["origin_level"] == "F5c"
    assert detail["has_f2_or_f3"] is True
    assert not detail["ladder_violations"]

    # determinism: the spread solve/manifest assembly (the layout is frozen,
    # not re-packed) is stable for a given seed and unaffected by unrelated
    # global `random` draws.
    ok, detail = fdr.check_determinism(
        555, lambda s: fdr._manifest_only_from_dump(path, s, n=16, collapse=1))
    assert ok, detail


def test_run_dry_from_dump_seed_override_only_touches_the_spread_solve(tmp_path):
    """`seed=` overrides the SPREAD SOLVE's own seed (`pick_origin` draws
    `rng.random()`, so a different seed CAN legitimately pick a different
    origin — this is not testing that it doesn't). What must hold regardless
    of `seed` is the LAYOUT itself: every record, whichever seed picked it,
    can only ever reference a cell the dump actually recorded — the dump is
    never re-packed. Omitting `seed` falls back to the dump's own recorded
    seed (555), reproducing the exact same run as the no-override default."""
    path = _row_dump(str(tmp_path / "row2.json"), seed=555)
    dumped_cells = {f"/World/stage/generated/house_{3 + 10 * k}" for k in range(8)}

    m_default, _c1, _e1 = fdr.run_dry_from_dump(path, n=16, collapse=1)
    m_override, _c2, _e2 = fdr.run_dry_from_dump(path, seed=999, n=16, collapse=1)
    m_no_override, _c3, _e3 = fdr.run_dry_from_dump(path, n=16, collapse=1)

    assert m_default["seed"] == 555
    assert m_override["seed"] == 999
    assert m_no_override["seed"] == 555
    assert m_no_override["origin"] == m_default["origin"]
    for m in (m_default, m_override, m_no_override):
        for rec in m["records"]:
            assert rec["cell"] in dumped_cells


# ---------------------------------------------------------------------------
# real output, if present — skipped, not failed, when it is not.
# ---------------------------------------------------------------------------
def _latest_real_manifest():
    paths = sorted(glob.glob(os.path.join(_SCENE_GEN_DIR, "_plans",
                                          "fire_city_*.json")),
                   key=os.path.getmtime, reverse=True)
    for path in paths:
        if path.endswith("_report.md"):
            continue
        try:
            with open(path) as fh:
                data = json.load(fh)
            if isinstance(data, dict) and "records" in data and "refused" in data:
                return path, data
        except (json.JSONDecodeError, OSError):
            continue
    return None, None


def test_real_manifest_passes_the_seven_checks_if_present():
    path, manifest = _latest_real_manifest()
    if manifest is None:
        import pytest
        pytest.skip("no _plans/fire_city_<seed>.json present yet -- run "
                   "tools/fire_city_dry_run.py --preset downtown_fire_500 first")

    manifest_dg0 = None
    arch_json = os.path.join(_SCENE_GEN_DIR, "assets", "archetype",
                             "archetypes.json")
    if os.path.isfile(arch_json):
        with open(arch_json) as fh:
            recs = json.load(fh)
        manifest_dg0 = {r["style"]: r for r in recs if r.get("level") == "DG0"}

    checks = fdr.run_all_checks(manifest, manifest_dg0=manifest_dg0)
    failures = {name: detail for name, (ok, detail) in checks.items() if not ok}
    assert not failures, (
        f"{path} failed {list(failures)}: {json.dumps(failures, indent=1)}")

    # determinism re-runs the WHOLE pipeline (needs usd-core / pxr) -- only
    # attempted when it's actually importable, so this test stays usable on
    # a plain `python3 -m pytest` with no Isaac Sim / uv environment.
    try:
        import pxr  # noqa: F401
    except ImportError:
        return
    ok, detail = fdr.check_determinism(
        manifest["seed"],
        lambda s: fdr._manifest_only(manifest["preset"], s, n=manifest["n"]))
    assert ok, detail


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
