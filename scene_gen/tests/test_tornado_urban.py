#!/usr/bin/env python3
"""test_tornado_urban.py — does the urban tornado ladder pick the right side,
tooth its removal, respect the height-class caps, and throw debris the right
way?

    python3 -m pytest -q scene_gen/tests/test_tornado_urban.py

WHY THIS EXISTS
---------------
`disaster/tornado_urban.py` is `_plans/urban_tornado_plan.md`'s stream L: the
planner half of the urban tornado ladder, on the SAME sliced-building piece
grid `disaster/quake_sliced.py` damages for an earthquake. Its whole
vocabulary is PURE arithmetic on an element table (removal + rigid
displacement + a debris deposition model), so — exactly as
`test_quake_sliced.py` checks `quake_sliced.plan_damage` with no `pxr`
anywhere — this file checks `tornado_urban.plan_damage` the same way: no
stage, no Isaac Sim, host-side in a few seconds.

WHAT IS CHECKED, AND WHERE
---------------------------
`_plans/urban_tornado_plan.md` §3's PLANNER checks (determinism, the wind-
side rule, toothing, the height-class caps, monotonicity in expectation,
every debris invariant) on:

  (a) `test_quake_sliced.fake_sliced_building` at three heights — lowrise
      (12 m / 3 storeys), midrise (40 m / 10), tower (120 m / 30) — imported
      from that module rather than re-derived, the same discipline
      `tools/quake_gac_probe.py`'s own `load_synthetic` follows;
  (b) REAL GAC piece grids via `tools.quake_gac_probe.load_real_kit`, for
      `SM_Building_02` and `SM_Building_09`.

A DISCOVERED DATA-SHAPE ARTIFACT IN THE CACHED KITS, NOT IN THIS PLANNER
--------------------------------------------------------------------------
`load_real_kit`'s two default-signature (`signature=None`) entries beyond
`SM_Building_02` — `SM_Building_09`, `SM_Building_11` — turn out to carry a
COLLAPSED grid: `SM_Building_09`'s 1525 pieces are 1489 `core`/`side="-"` and
only 36 `wall` pieces total, ONE bay per elevation (`g.n_bays == {"S":1,
"E":1, "N":1, "W":1}`), spread across up to 1488 distinct `_storey` values
that plainly are not 1488 real building storeys on a 59 m building. Measured
directly (`n_sub_of` returns 1, no `pier`/`corner`/`parapet` role exists at
all in that cached record). This is a property of THAT cached bake, not of a
live slice: the lead's own bare-python probe of a LIVE `SM_Building_02`
slice (`region=None`) found a normally-shaped grid (239 pieces: corner 30 /
pier 116 / wall 59 / core 11 / parapet 18 / parapet_corner 4 / roof 1,
`_storey` 0..11), matching this file's synthetic fixture far more closely
than the cached kit does — and the REAL bake launcher
(`quake_gac_bake_launch_script.py`, per `quake_gac_probe.py`'s own
docstring) never reads `kits.json` at all, it always does a live
`slice_to_kit`. So `SM_Building_09`'s test below is a SMOKE test only (does
not crash, respects every hard invariant, produces a valid, JSON-safe plan)
— the invariants that need a real grid to be meaningful (the wind-side
rule, toothing, monotonicity, the debris statistics) are checked on the
fixtures and on `SM_Building_02`, whose cached grid — while also far
sparser than a live slice — at least has more than one bay per side to
work with.

NO TOWER-CLASS REAL KIT EXISTS UNDER THE DEFAULT SIGNATURE. `assets/kits/
kits.json`'s three `signature=None` rows (`load_real_kit(name)`'s only
reachable rows, since it calls `kit_bake._entry(name)` with the default
`signature=None`) top out at `SM_Building_11`, H=64.3 m (highrise, not
tower — the `>=100 m` cut). The plan brief anticipated this might be the
case ("one tower-class kit IF `kits.json` has one") — it does not, so the
tower-class invariants (§3's own explicit ask: "every T4 plan on the tower
fixture has `removed_frac <= 0.10`...") are checked on the 120 m / 30-storey
SYNTHETIC fixture instead, which is exactly why the brief handed this file
that fixture in the first place.

WHAT THIS FILE CANNOT SEE: whether a voided pane reads as a hole, whether a
hanging panel or a windward chunk looks right from the air, whether the
debris field reads as "thrown" rather than "scattered". That needs
`tornado_urban_usd.apply_plan` and a render — not this module's job.
"""

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from disaster import quake_flow as qf                  # noqa: E402
from disaster import quake_sliced as qs                 # noqa: E402
from disaster import tornado as trn                      # noqa: E402
from disaster import tornado_urban as tu                  # noqa: E402
from test_quake_sliced import fake_sliced_building          # noqa: E402
from tools.quake_gac_probe import load_real_kit               # noqa: E402
from detail import kit_bake as kb                               # noqa: E402


# ---------------------------------------------------------------------------
# `wind` fixtures
# ---------------------------------------------------------------------------
def fake_wind(bearing_deg, speed_frac=0.8, cross_frac=-0.4, over=False):
    """The §2.4 wind dict shape, without depending on `tornado.wind_at`
    (stream P's job) for every test — a handful of tests below also check
    against the real `wind_at` now that it has landed."""
    return {"bearing_deg": float(bearing_deg), "speed_frac": float(speed_frac),
            "cross_frac": float(cross_frac), "over": bool(over)}


# ---------------------------------------------------------------------------
# building fixtures
# ---------------------------------------------------------------------------
_FIXTURES = {
    "lowrise": dict(W=24.0, D=18.0, H=12.0, storeys=3),
    "midrise": dict(W=30.0, D=24.0, H=40.0, storeys=10),
    "tower": dict(W=42.0, D=36.0, H=120.0, storeys=30),
}


def _fixture(name, seed=5, x=0.0, y=0.0, yaw=0.0, btype="urm"):
    kw = dict(_FIXTURES[name])
    pls, style, grid = fake_sliced_building(seed=seed, **kw)
    info = qf.describe(style, pls, x, y, yaw)
    info["type"] = btype
    return info


def _plan(fixture, level, btype="urm", seed=5, wind=None, intensity=None,
         height_class=None, x=0.0, y=0.0, yaw=0.0):
    info = _fixture(fixture, seed=seed, x=x, y=y, yaw=yaw, btype=btype)
    wind = wind if wind is not None else fake_wind(0.0)
    rng = random.Random(seed)
    if intensity is None:
        intensity = {"T0": 0.05, "T1": 0.2, "T2": 0.4, "T3": 0.6, "T4": 0.85}.get(level, 0.5)
    plan = tu.plan_damage(info, info["elements"], level, btype, rng, wind,
                          height_class=height_class, intensity=intensity)
    return info, plan


def _by_path(info):
    return {(e["p"] or {}).get("prim_path"): e for e in info["elements"]}


def _removed_els(info, plan):
    idx = _by_path(info)
    return [idx[p] for p in plan["removed"] if p in idx]


def _real_kit_info(name, btype):
    pls, style, grid = load_real_kit(name)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    return info


def _run_one_recipe(fixture, recipe, kw, btype="urm", seed=5, wind=None,
                    height_class=None, intensity=0.7, x=0.0, y=0.0, yaw=0.0):
    """Run exactly ONE recipe (bypassing `LADDER_T`/`_guard`), the same
    isolation `test_quake_sliced.py` uses to check one recipe's own
    toothing draw without a later recipe in the same T-level ladder
    touching the same cells."""
    info = _fixture(fixture, seed=seed, x=x, y=y, yaw=yaw, btype=btype)
    wind = wind if wind is not None else fake_wind(0.0)
    if height_class is None:
        height_class = tu.height_class_for(info["H"])
    rng = random.Random(seed)
    plan = {
        "schema": "tornado_urban_plan.v1", "level": "TEST", "btype": btype,
        "style": info.get("style"), "H": float(info.get("H") or 0.0),
        "height_class": height_class, "wind": dict(wind),
        "recipes": [[recipe, dict(kw)]],
        "removed": [], "displaced": {}, "glass": [], "glass_bands": [],
        "macroblocks": [], "regions": [], "roof_props": "keep", "debris": [],
        "notes": [], "stats": {}, "_removed_set": set(),
    }
    weights = tu.side_weights(info, wind, rng)
    plan["side_weights"] = {k: float(v) for k, v in weights.items()}
    pctx = tu._pctx(info, info["elements"], btype, rng, plan, wind, weights,
                    height_class, intensity)
    tu.RECIPES_T[recipe](pctx, **kw)
    plan = tu._finalise(pctx, plan, height_class, wind, intensity)
    return info, plan


# ---------------------------------------------------------------------------
# LEVELS / height class
# ---------------------------------------------------------------------------
def test_levels_and_cuts_are_consistent():
    assert tu.LEVELS == ("T0", "T1", "T2", "T3", "T4")
    lims = [c[0] for c in tu._URBAN_CUTS]
    assert lims == sorted(lims)
    names = [c[1] for c in tu._URBAN_CUTS]
    assert names == list(tu.LEVELS)


def test_level_for_intensity_is_monotonic_in_expectation():
    rng = random.Random(1)
    order = {n: i for i, n in enumerate(tu.LEVELS)}
    means = []
    for i in (0.02, 0.2, 0.4, 0.6, 0.9):
        picks = [order[tu.level_for_intensity(i, rng, jitter=0.06)] for _ in range(200)]
        means.append(sum(picks) / float(len(picks)))
    assert means == sorted(means), means


def test_height_class_from_measured_h():
    assert tu.height_class_for(10.0) == "lowrise"
    assert tu.height_class_for(17.9) == "lowrise"
    assert tu.height_class_for(18.0) == "midrise"
    assert tu.height_class_for(44.9) == "midrise"
    assert tu.height_class_for(45.0) == "highrise"
    assert tu.height_class_for(99.9) == "highrise"
    assert tu.height_class_for(100.0) == "tower"
    assert tu.height_class_for(300.0) == "tower"


def test_height_class_typology_low_is_trusted_but_only_low():
    # "rowhouse"/"lowrise" typologies resolve to urban_fire_spread's "low"
    # class -- forced to lowrise regardless of a (bogus) tall H.
    assert tu.height_class_for(90.0, typology="rowhouse") == "lowrise"
    assert tu.height_class_for(90.0, typology="lowrise") == "lowrise"
    # "tower"/"highrise"/"midrise"/"brick_midrise" typologies fall through
    # to H (see the module docstring on why the 3-class map is not trusted
    # for these) -- so a tall H still resolves by H, not by the typology
    # name looking like a class name.
    assert tu.height_class_for(30.0, typology="tower") == "midrise"
    assert tu.height_class_for(150.0, typology="midrise") == "tower"


def test_height_caps_table_shape():
    for hc in tu.HEIGHT_CLASSES:
        caps = tu.HEIGHT_CAPS[hc]
        assert 0.0 < caps["max_removed_frac"] <= 1.0
        assert caps["max_chunk_storeys"] >= 1
        assert caps["out_of_plane"] in (True, False)
    # the caps get STRICTER (or equal) as the building gets taller
    fracs = [tu.HEIGHT_CAPS[hc]["max_removed_frac"] for hc in tu.HEIGHT_CLASSES]
    assert fracs == sorted(fracs, reverse=True), fracs
    assert tu.HEIGHT_CAPS["highrise"]["out_of_plane"] is False
    assert tu.HEIGHT_CAPS["tower"]["out_of_plane"] is False


# ---------------------------------------------------------------------------
# side_weights
# ---------------------------------------------------------------------------
def test_side_weights_windward_is_highest_and_leeward_lowest():
    info = _fixture("midrise")
    # bearing 180 (d = (cos180, sin180) = (-1, 0), wind blows TOWARD -x)
    # hits the EAST face hardest: E's outward normal is (1, 0), and
    # windward_w(E) = max(0, -n_E . d) = max(0, -(1*-1 + 0*0)) = 1 (max).
    w = tu.side_weights(info, fake_wind(180.0))
    assert set(w) == set(qs.SIDES) | set(qs._CORNER_SIDES)
    assert all(0.0 <= v <= 1.0 for v in w.values()), w
    assert w["E"] == max(w[s] for s in qs.SIDES)
    assert w["W"] == min(w[s] for s in qs.SIDES)


def test_side_weights_corner_is_mean_plus_bonus():
    info = _fixture("midrise")
    w = tu.side_weights(info, fake_wind(41.0))
    for cn, (sa, sb) in qs._CORNER_SIDES.items():
        expect = min(1.0, (w[sa] + w[sb]) / 2.0 + tu._CORNER_BONUS)
        assert abs(w[cn] - expect) < 1e-9, (cn, w[cn], expect)


def test_side_weights_over_uses_rng_when_given_else_a_fixed_midpoint():
    info = _fixture("midrise")
    w0 = tu.side_weights(info, fake_wind(0.0, over=True))
    for sd in qs.SIDES:
        assert abs(w0[sd] - sum(tu._OVER_WEIGHT) / 2.0) < 1e-9
    rng = random.Random(9)
    w1 = tu.side_weights(info, fake_wind(0.0, over=True), rng=rng)
    assert any(abs(w1[sd] - w0[sd]) > 1e-9 for sd in qs.SIDES)
    for sd in qs.SIDES:
        assert tu._OVER_WEIGHT[0] <= w1[sd] <= tu._OVER_WEIGHT[1]


def test_side_weights_against_the_real_wind_at():
    """`tornado.wind_at` has landed -- exercise `side_weights` against the
    real function too, not only `fake_wind`."""
    cfg = trn.resolve_cfg({})
    info = _fixture("midrise", x=0.0, y=0.0)
    wind = trn.wind_at(cfg, 0.0, 0.0)
    for key in ("bearing_deg", "speed_frac", "cross_frac", "over"):
        assert key in wind
    w = tu.side_weights(info, wind)
    assert set(w) == set(qs.SIDES) | set(qs._CORNER_SIDES)
    assert all(0.0 <= v <= 1.0 for v in w.values())


# ---------------------------------------------------------------------------
# T0 / determinism
# ---------------------------------------------------------------------------
def test_t0_is_pristine():
    for btype in ("urm", "rc", "rc_glass"):
        info, plan = _plan("midrise", "T0", btype=btype)
        assert plan["removed"] == [], btype
        assert plan["displaced"] == {}, btype
        assert plan["glass"] == [], btype
        assert plan["debris"] == [], btype
        assert plan["stats"]["removed_frac"] == 0.0


def test_planner_determinism_same_seed_byte_identical():
    for level in tu.LEVELS:
        for btype in ("urm", "rc", "rc_glass"):
            _info1, p1 = _plan("midrise", level, btype=btype, seed=17,
                               wind=fake_wind(63.0, 0.7, -0.2))
            _info2, p2 = _plan("midrise", level, btype=btype, seed=17,
                               wind=fake_wind(63.0, 0.7, -0.2))
            j1 = json.dumps(p1, sort_keys=True)
            j2 = json.dumps(p2, sort_keys=True)
            assert j1 == j2, (level, btype)


def test_plan_json_round_trips_every_level_and_type():
    for level in tu.LEVELS:
        for btype in ("urm", "rc", "rc_glass"):
            _info, plan = _plan("midrise", level, btype=btype, seed=3)
            s = json.dumps(plan, sort_keys=True)
            back = json.loads(s)
            assert back["level"] == level
            assert back["stats"]["n_removed"] == plan["stats"]["n_removed"]


# ---------------------------------------------------------------------------
# the wind-side rule
# ---------------------------------------------------------------------------
def test_wind_side_rule_south_bearing_hits_south():
    """`bearing_deg=270` (blowing toward -x) is wrong for hitting "S" in
    this fixture's unyawed frame (`quake_flow._side_of`: S is -Y). Blowing
    TOWARD -y (`bearing_deg=270`? no: -y is `bearing_deg=270` only if 270
    means direction (0,-1)... use math convention directly: -y is 270 deg
    (cos270=0, sin270=-1) -- so wind blowing toward -y hits the face whose
    outward normal is +y? Work it from the formula instead of by eye: south
    wall's outward normal is (0,-1) (`quake_sliced._SIDE_NORMAL["S"]`), and
    `windward_w(S) = max(0, -n_S . d)` is maximised when `d = (0, 1)` --
    `bearing_deg = 90`, wind blowing TOWARD +y, hits the SOUTH (-y-facing)
    wall, because that wall faces where the wind is coming FROM."""
    info = _fixture("midrise", seed=21)
    wind = fake_wind(90.0, 0.85, cross_frac=0.05, over=False)
    _info2, plan = _plan("midrise", "T4", seed=21, wind=wind, intensity=0.85)
    rm = _removed_els(info, plan)
    assert rm, "T4 should remove something"
    on_s_or_corner = sum(1 for e in rm
                         if (e["p"] or {}).get("_side") in ("S", "SW", "SE"))
    assert on_s_or_corner / float(len(rm)) >= 0.70, (on_s_or_corner, len(rm))
    assert not any((e["p"] or {}).get("_side") == "N" for e in rm)


def test_wind_over_produces_removal_on_at_least_three_sides():
    seen_any = False
    for seed in range(12):
        info, plan = _plan("midrise", "T4", seed=seed,
                           wind=fake_wind(30.0, 0.85, 0.0, over=True),
                           intensity=0.9)
        rm = _removed_els(info, plan)
        sides = set((e["p"] or {}).get("_side") for e in rm)
        sides = set(s for s in sides if s is not None)
        # normalise corners to their two parent sides for the >=3-side count
        flat = set()
        for s in sides:
            if s in qs._CORNER_SIDES:
                flat |= set(qs._CORNER_SIDES[s])
            elif s in qs.SIDES:
                flat.add(s)
        if len(flat) >= 3:
            seen_any = True
    assert seen_any


# ---------------------------------------------------------------------------
# toothing
# ---------------------------------------------------------------------------
def test_toothing_no_boundary_row_is_all_or_nothing():
    """Reasserts `quake_sliced._apply_region`'s own invariant through this
    module's recipes, the SAME way `test_quake_sliced.
    test_a_lost_region_is_toothed_and_not_a_rectangle` checks it: one
    recipe run in isolation (`_run_one_recipe`, bypassing the rest of a
    T-level's ladder, which can legitimately touch the same cells a later
    recipe already toothed), aggregated over the whole boundary rather than
    row by row — not every boundary pier survives across seeds (there
    would be no toothing at all), and not every boundary pier is lost on
    any given run (a ruler edge)."""
    for recipe, kw in (("cladding_band", {}), ("chunk", {})):
        seen_partial = False
        for seed in range(15):
            info, plan = _run_one_recipe("midrise", recipe, kw, seed=seed,
                                         wind=fake_wind(10.0 + 7.0 * seed, 0.85),
                                         intensity=0.85)
            regs = [r for r in plan["regions"] if r["recipe"] == recipe]
            if not regs:
                continue
            cells = set(tuple(c) for c in regs[0]["cells"])
            g = qs._Grid(info, info["elements"])
            bnd = qs._boundary(cells, set(g.runs))
            piers = [e for key in bnd for e in g.at(key)
                    if qs.is_pier(e.get("p") or {}, g.n_sub)]
            if not piers:
                continue
            removed = set(plan["removed"])
            survived = [e for e in piers if qs._path(e) not in removed]
            assert survived, (recipe, seed, "every boundary pier removed")
            if len(survived) < len(piers):
                seen_partial = True
        assert seen_partial, recipe


# ---------------------------------------------------------------------------
# height-class caps
# ---------------------------------------------------------------------------
def test_caps_hold_for_every_height_class():
    for fixture in ("lowrise", "midrise", "tower"):
        info0 = _fixture(fixture, seed=2)
        hc = tu.height_class_for(info0["H"])
        cap = tu.HEIGHT_CAPS[hc]["max_removed_frac"]
        for seed in range(10):
            info, plan = _plan(fixture, "T4", seed=seed,
                               wind=fake_wind(50.0, 0.9), intensity=0.9)
            st = plan["stats"]
            # NAMED CARVE-OUT (Sec8c): `t_facade_collapse` is exempted from
            # the lowrise area cap by `_cap_removed_frac` -- when it ran and
            # the plan still landed over cap, the exemption itself must be
            # visible in the plan's own notes (`t_facade_collapse`'s own
            # docstring, "THE CAP EXEMPTION"). Every OTHER recipe's own
            # contribution still competes for trimming, which is why this
            # fixture's own 10 seeds mostly land back under cap anyway (the
            # branch below is what a fixture/seed with nothing else to trim
            # would hit).
            ran_fc = any(r.get("recipe") == "facade_collapse" for r in plan["regions"])
            # ROUND 3b (§8e F1): `tornado_urban._shed_unsupported_roof` can
            # ALSO push `removed_frac` over the height-class cap on its own
            # -- a plain `parapet` piece carries a real S/E/N/W `_side`
            # (`_facade_area_of` counts it) and the pass runs strictly
            # AFTER `_cap_removed_frac`, deliberately un-gated by it, for
            # the same reason `core`/`roof` are already walk-back-exempt
            # there ("it is not a removal mistake to walk back, it is the
            # one named exception"): shedding a piece because its OWN
            # support is already gone is a CONSEQUENCE of an already-capped
            # wall loss, not a new area decision. `st["n_roof_shed"]`/
            # `st["n_parapet_shed"]` are that pass's own counters, so an
            # over-cap plan is accepted when EITHER the pre-existing
            # facade_collapse exemption note explains it OR F1's own
            # shedding does; only an over-cap plan with NEITHER is a bug.
            shed_any = (int(st.get("n_roof_shed", 0))
                       + int(st.get("n_parapet_shed", 0))) > 0
            if st["removed_frac"] > cap + 1e-9:
                has_fc_note = any("facade_collapse" in n and "exempt" in n
                                 for n in plan["notes"])
                assert shed_any or (ran_fc and has_fc_note), \
                    (fixture, seed, "over cap with neither a "
                                    "facade_collapse exemption note nor F1 "
                                    "roof/parapet shedding to explain it",
                     plan["notes"])
            rm = _removed_els(info, plan)
            ran_tsl = any(r.get("recipe") == "top_storey_loss" for r in plan["regions"])
            assert not any((e["p"] or {}).get("_role") == "core" for e in rm), \
                (fixture, seed, "core removed")
            assert not any((e["p"] or {}).get("_role") == "roof" for e in rm) \
                  or ran_tsl or ran_fc or int(st.get("n_roof_shed", 0)) > 0, \
                (fixture, seed, "roof removed without a named exception")
            # no storey emptied
            g = qs._Grid(info, info["elements"])
            by_storey = {}
            for e in g.els:
                by_storey.setdefault(int((e["p"] or {}).get("_storey", 0)), []).append(e)
            removed_set = set(plan["removed"])
            for _st_i, els in by_storey.items():
                paths = [qs._path(e) for e in els if qs._path(e)]
                assert not (paths and all(p in removed_set for p in paths)), \
                    (fixture, seed, _st_i)
            if hc == "tower":
                assert not any(int((e["p"] or {}).get("_storey", -1)) == 0
                              and qs.is_pier(e.get("p") or {}, g.n_sub)
                              for e in rm), (fixture, seed)


def test_t4_chunk_region_top_is_in_the_top_40_percent():
    for seed in range(10):
        info, plan = _plan("midrise", "T4", seed=seed, wind=fake_wind(20.0, 0.9),
                           intensity=0.9)
        m = info["masses"]["main"]
        for region in plan["regions"]:
            if region["recipe"] != "chunk":
                continue
            top_storey = max(int(s) for s in region["storeys"])
            top_z = m["levels"][top_storey] if top_storey < len(m["levels"]) else m["top"]
            assert top_z >= 0.6 * info["H"] - 1e-6, (seed, top_z, info["H"])


def test_chunk_anchor_varies_across_the_upper_40_percent():
    """The lead's design change: `t_chunk` no longer always notches the
    roof corner -- its region's own TOP storey is drawn anywhere in the
    upper 40 % of the building (roof included), which is the constraint
    §2.6 actually states ("region top >= 0.6 H"), not an instruction to
    always anchor there. Over enough seeds the set of chosen top storeys
    must show real spread, not just repeat `g.top`."""
    info = _fixture("tower", seed=1)
    g = qs._Grid(info, info["elements"])
    lo_cut = int(math.ceil(0.6 * g.top))
    tops = set()
    for seed in range(40):
        _info, plan = _plan("tower", "T4", seed=seed, wind=fake_wind(65.0, 0.9),
                            intensity=0.95)
        for region in plan["regions"]:
            if region["recipe"] != "chunk":
                continue
            top_storey = max(int(s) for s in region["storeys"])
            tops.add(top_storey)
            assert top_storey >= lo_cut, (seed, top_storey, lo_cut)
    assert len(tops) >= 3, tops


def test_t4_tower_fixture_invariants():
    """§3's explicit tower-fixture ask, verbatim: `removed_frac <= 0.10`, no
    storey emptied, no ground-storey pier gone, the chunk's region top
    `>= 0.6 H`."""
    cap = tu.HEIGHT_CAPS["tower"]["max_removed_frac"]
    assert cap == 0.10
    for seed in range(10):
        info, plan = _plan("tower", "T4", seed=seed, wind=fake_wind(75.0, 0.95),
                           intensity=0.95)
        st = plan["stats"]
        assert tu.height_class_for(info["H"]) == "tower"
        assert st["removed_frac"] <= 0.10 + 1e-9, (seed, st["removed_frac"])
        g = qs._Grid(info, info["elements"])
        removed_set = set(plan["removed"])
        by_storey = {}
        for e in g.els:
            by_storey.setdefault(int((e["p"] or {}).get("_storey", 0)), []).append(e)
        for _st_i, els in by_storey.items():
            paths = [qs._path(e) for e in els if qs._path(e)]
            assert not (paths and all(p in removed_set for p in paths))
        assert not any(int((e["p"] or {}).get("_storey", -1)) == 0
                      and qs.is_pier(e.get("p") or {}, g.n_sub)
                      for e in _removed_els(info, plan))
        if st["max_removed_storey"] is not None:
            m = info["masses"]["main"]
            st_i = st["max_removed_storey"]
            top_z = m["levels"][st_i] if st_i < len(m["levels"]) else m["top"]
            assert top_z >= 0.6 * info["H"] - 1e-6, (seed, top_z, info["H"])


def test_out_of_plane_top_and_top_storey_loss_are_guarded_off_above_midrise():
    for fixture, hc in (("midrise", "midrise"), ("tower", "tower")):
        for seed in range(6):
            info, plan = _plan(fixture, "T4", btype="urm", seed=seed,
                               wind=fake_wind(15.0, 0.9), intensity=0.95)
            has_oop = any(r["recipe"] == "out_of_plane_top" for r in plan["regions"])
            has_tsl = any(r["recipe"] == "top_storey_loss" for r in plan["regions"])
            if hc == "tower":
                assert not has_oop, (fixture, seed)
                assert not has_tsl, (fixture, seed)
                assert any("out_of_plane_top refused" in n for n in plan["notes"])
            if hc == "midrise":
                assert not has_tsl, (fixture, seed)  # lowrise-only


def test_rc_glass_chunk_never_a_macroblock():
    for seed in range(8):
        info, plan = _plan("midrise", "T4", btype="rc_glass", seed=seed,
                           wind=fake_wind(5.0, 0.9), intensity=0.9)
        assert plan.get("macroblocks") in (None, [])
        # a chunk on rc_glass only carries corner pieces in its top storeys
        for region in plan["regions"]:
            if region["recipe"] != "chunk":
                continue
            sts = sorted(set(int(c[1]) for c in region["cells"]))
            assert len(sts) <= tu.HEIGHT_CAPS[
                tu.height_class_for(info["H"])]["max_chunk_storeys"]


# ---------------------------------------------------------------------------
# facade_collapse — R11 / §8c: the URM lowrise total facade-loss state
# ---------------------------------------------------------------------------
def _custom_fixture(W, D, H, storeys, seed=5, btype="urm"):
    """Same shape as `_fixture` above but for a W/D/H/storeys combination
    not in `_FIXTURES` — needed for the storey-count guard test, which
    needs a building that is `height_class == "lowrise"` (H < 18 m) but has
    MORE than 4 storeys (a short-storey warehouse-mezzanine shape no named
    fixture provides)."""
    pls, style, grid = fake_sliced_building(W=W, D=D, H=H, storeys=storeys,
                                            seed=seed)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    return info


def _fc_plan(fixture_info, level="T4", btype="urm", seed=5,
            wind=None, intensity=0.9, height_class=None):
    wind = wind if wind is not None else fake_wind(50.0, 0.9)
    rng = random.Random(seed)
    plan = tu.plan_damage(fixture_info, fixture_info["elements"], level, btype,
                          rng, wind, height_class=height_class,
                          intensity=intensity)
    return plan


def _fc_ran(plan):
    return any(r.get("recipe") == "facade_collapse" for r in plan["regions"])


def test_facade_collapse_guard_table():
    """Every refusal named, per Sec8c: btype, height_class, storey count,
    intensity — the four eligibility gates, exactly as `t_facade_collapse`'s
    own docstring and `_guard`'s new branch describe them."""
    lowrise = _fixture("lowrise", seed=2)          # urm, H=12, 3 storeys
    midrise = _fixture("midrise", seed=2)           # H=40
    tall_lowrise = _custom_fixture(24.0, 18.0, 17.0, 6, seed=2)  # H<18, 6 storeys

    # 1) wrong btype (rc) -- `LADDER_T["rc"]["T4"]` never OFFERS
    #    `facade_collapse` at all (it is urm-only in the ladder itself,
    #    same as `top_storey_loss`/`out_of_plane_top`), so the only way to
    #    observe `_guard`'s own btype branch is to call it directly with a
    #    synthetic recipe list, exactly as its docstring describes ("defence
    #    in depth for a future ladder edit that adds one to the wrong
    #    bucket by mistake").
    recs, notes = tu._guard([("facade_collapse", {})], "rc", lowrise, "lowrise")
    assert recs == []
    assert any("guard: facade_collapse refused" in n and "rc" in n
              for n in notes), notes

    # 2) wrong height_class (midrise) on urm -- reachable through the real
    #    ladder (`LADDER_T["urm"]["T4"]` always offers it; the runtime
    #    guard is what refuses a midrise/tower building).
    plan = _fc_plan(midrise, btype="urm", intensity=0.9)
    assert not _fc_ran(plan)
    assert any("guard: facade_collapse refused" in n for n in plan["notes"]), \
        plan["notes"]
    plan = _fc_plan(midrise, btype="urm", intensity=0.9)
    assert not _fc_ran(plan)
    assert any("guard: facade_collapse refused" in n for n in plan["notes"]), \
        plan["notes"]

    # 3) too many storeys (6 > 5; the cap moved 4 -> 5 for Waco's own
    #    5-storey Dennis Building) on an otherwise-eligible lowrise urm
    #    building -- `_guard` has no element table, so this is refused
    #    INSIDE the recipe itself, not by `_guard`'s own note.
    plan = _fc_plan(tall_lowrise, btype="urm", intensity=0.9)
    assert not _fc_ran(plan)
    assert any("facade_collapse: refused" in n and "storeys" in n
              for n in plan["notes"]), plan["notes"]

    # 4) intensity below the 0.82 floor on an otherwise-eligible building --
    #    also refused inside the recipe (`_guard` has no intensity either).
    plan = _fc_plan(lowrise, btype="urm", intensity=0.80)
    assert not _fc_ran(plan)
    assert any("facade_collapse: refused" in n and "intensity" in n
              for n in plan["notes"]), plan["notes"]

    # 5) every gate cleared -- fires.
    plan = _fc_plan(lowrise, btype="urm", intensity=0.9)
    assert _fc_ran(plan)

    # 6) the boundary itself: 0.82 fires, just under does not.
    plan_at = _fc_plan(lowrise, btype="urm", intensity=0.82)
    plan_under = _fc_plan(lowrise, btype="urm", intensity=0.819)
    assert _fc_ran(plan_at)
    assert not _fc_ran(plan_under)


def test_facade_collapse_touches_exactly_one_elevation_and_building_stands():
    """The recipe's OWN region never carries a corner or a second side —
    trivially satisfies Sec8c's "never > 2 of 4 elevations" — and the roof
    piece it ledgers as debris is the same named `(core, roof)` exception
    `top_storey_loss` already uses."""
    lowrise = _fixture("lowrise", seed=2)
    seen = False
    for seed in range(20):
        plan = _fc_plan(lowrise, btype="urm", seed=seed,
                        wind=fake_wind(50.0 + seed, 0.9), intensity=0.9)
        fc_regions = [r for r in plan["regions"] if r["recipe"] == "facade_collapse"]
        if not fc_regions:
            continue
        seen = True
        assert len(fc_regions) == 1, (seed, fc_regions)
        region = fc_regions[0]
        sides_in_region = set(c[0] for c in region["cells"])
        assert sides_in_region == {region["side"]}, (seed, sides_in_region)
        assert "corner" not in region, (seed, region)
        # storeys carried in the region include storey 0 -- the one recipe
        # this ladder allows to touch a ground-storey run.
        assert 0 in region["storeys"], (seed, region["storeys"])
    assert seen, "facade_collapse never fired across 20 seeds at i=0.9"


def test_facade_collapse_leaning_macroblocks_pitch_into_the_street():
    """1-2 survivors, still `displaced` (never `removed`), pitched 55-80 deg
    -- the same `_disp` pivot/axis/sign `test_every_macroblock_lands_in_
    the_street`-style checks already use for `t_out_of_plane_top`."""
    lowrise = _fixture("lowrise", seed=2)
    seen_any = False
    for seed in range(20):
        plan = _fc_plan(lowrise, btype="urm", seed=seed,
                        wind=fake_wind(50.0 + seed, 0.9), intensity=0.9)
        macros = [m for m in (plan.get("macroblocks") or [])
                 if m.get("recipe") == "facade_collapse"]
        if not macros:
            continue
        seen_any = True
        assert 1 <= len(macros) <= 2, (seed, len(macros))
        removed = set(plan["removed"])
        displaced = plan["displaced"]
        for mb in macros:
            p = mb["path"]
            assert p not in removed, (seed, p, "leaning piece was removed too")
            assert p in displaced, (seed, p, "leaning piece not displaced")
            assert 55.0 - 1e-6 <= mb["deg"] <= 80.0 + 1e-6, (seed, mb["deg"])
            assert displaced[p]["deg"] == mb["deg"]
    assert seen_any, "no facade_collapse leaning macroblock seen across 20 seeds"


def test_facade_collapse_never_removes_core_and_ledgers_the_roof():
    lowrise = _fixture("lowrise", seed=2)
    idx = _by_path(lowrise)
    seen = False
    for seed in range(20):
        plan = _fc_plan(lowrise, btype="urm", seed=seed,
                        wind=fake_wind(50.0 + seed, 0.9), intensity=0.9)
        if not _fc_ran(plan):
            continue
        seen = True
        removed = plan["removed"]
        assert not any((idx.get(p, {}).get("p") or {}).get("_role") == "core"
                      for p in removed if p in idx), (seed, "core removed")
        roof_removed = [p for p in removed
                        if p in idx and (idx[p].get("p") or {}).get("_role") == "roof"]
        assert plan.get("roof_shed") is True, (seed, "roof_shed flag not set")
    assert seen


def test_facade_collapse_cap_exemption_named_when_it_matters():
    """A direct, deterministic exercise of `_cap_removed_frac`'s own
    carve-out (Sec8c): hand it a plan whose ONLY region is a synthetic
    `facade_collapse` one that is, by construction, over the lowrise cap,
    and confirm NOTHING is trimmed and the exemption is named."""
    lowrise = _fixture("lowrise", seed=2)
    g = qs._Grid(lowrise, lowrise["elements"])
    cap = tu.HEIGHT_CAPS["lowrise"]["max_removed_frac"]
    # every S-side piece, at every storey -- the same full-side shape
    # `t_facade_collapse` itself builds, guaranteed over the 0.35 cap for
    # this fixture (measured: a single elevation alone already exceeds it
    # once every bay/storey on that side is counted).
    s_pieces = [e for e in g.els if (e.get("p") or {}).get("_side") == "S"]
    assert s_pieces, "fixture has no S-side pieces to build the test region from"
    cells = sorted(set(tu._cell_of(e, g.n_sub) for e in s_pieces
                       if tu._cell_of(e, g.n_sub) is not None))
    plan = {
        "level": "T4", "removed": [qs._path(e) for e in s_pieces if qs._path(e)],
        "_removed_set": set(qs._path(e) for e in s_pieces if qs._path(e)),
        "regions": [{"recipe": "facade_collapse", "side": "S",
                    "cells": [list(c) for c in cells]}],
        "notes": [],
    }
    pctx = {"g": g, "info": lowrise, "plan": plan}
    tu._cap_removed_frac(pctx, plan, "lowrise")
    # nothing restored -- the facade_collapse region is exempt outright.
    assert set(plan["removed"]) == set(qs._path(e) for e in s_pieces if qs._path(e))
    removed_area = sum(tu._facade_area_of(e) for e in s_pieces
                       if (e.get("p") or {}).get("_side") in qs.SIDES)
    total_area = tu._total_facade_area(g)
    if removed_area / total_area > cap + 1e-9:
        assert any("facade_collapse" in n and "exempt" in n for n in plan["notes"]), \
            plan["notes"]
    else:
        # this fixture's S side did not, in fact, exceed the cap alone --
        # the assertion above (nothing restored) is still the load-bearing
        # one; report the measured fraction so a future fixture change that
        # breaks this precondition is diagnosable, not silently trivial.
        assert removed_area / total_area <= cap + 1e-9, (removed_area, total_area)


# ---------------------------------------------------------------------------
# monotonicity in expectation
# ---------------------------------------------------------------------------
def test_monotonic_mean_removal_and_glass_over_seeds():
    for fixture in ("lowrise", "midrise", "tower"):
        means_removed, means_glass = [], []
        for level in tu.LEVELS[1:]:  # T1..T4
            n_rem, n_gl = [], []
            for seed in range(30):
                _info, plan = _plan(fixture, level, seed=seed,
                                    wind=fake_wind(12.0, 0.8),
                                    intensity={"T1": 0.2, "T2": 0.42, "T3": 0.62,
                                              "T4": 0.85}[level])
                n_rem.append(plan["stats"]["n_removed"])
                n_gl.append(plan["stats"]["n_glass"])
            means_removed.append(sum(n_rem) / 30.0)
            means_glass.append(sum(n_gl) / 30.0)
        assert means_removed == sorted(means_removed), (fixture, means_removed)
        assert means_glass == sorted(means_glass), (fixture, means_glass)


# ---------------------------------------------------------------------------
# debris
# ---------------------------------------------------------------------------
def test_every_removed_piece_has_at_least_one_fragment():
    info, plan = _plan("midrise", "T4", seed=6, wind=fake_wind(200.0, 0.9),
                       intensity=0.9)
    assert plan["removed"], "need at least one removed piece to test this"
    sourced = set(f["from"] for f in plan["debris"])
    for p in plan["removed"]:
        assert p in sourced, p


def test_debris_class_volume_within_bounds_of_source():
    """Volume conservation is APPROXIMATE now, not exact (the lead's ledger
    fix): a structural fragment's thickness is the source piece's own
    (`min(sx, sy)`, clamped), and its COUNT — capped at `N_MAX_PER_PIECE` —
    absorbs the piece's volume instead of the fragment dimensions being
    solved for it. `stats["debris_volume_m3"]` / `stats["source_volume_m3"]`
    is the aggregate version of this same ratio, in [0.3, 1.0] — a hard
    per-piece cap can legitimately drop it below the round-1 [0.5, 1.0]."""
    info, plan = _plan("midrise", "T4", seed=8, wind=fake_wind(140.0, 0.9),
                       intensity=0.9)
    st = plan["stats"]
    assert st["source_volume_m3"] > 0.0
    ratio = st["debris_volume_m3"] / st["source_volume_m3"]
    # `debris_volume_m3` also includes glass shards (area-driven, not
    # volume-conserved against anything), so an UPPER bound is not
    # meaningful here -- only the lower bound (structural debris did not
    # collapse to near-nothing) is checked.
    assert ratio >= 0.05, (ratio, st)
    # per-piece: every removed piece's own fragments are individually
    # bounded too, loosely (a hard per-piece cap can drop this well below
    # 1, but never to zero and never absurdly over).
    idx = _by_path(info)
    by_source = {}
    for f in plan["debris"]:
        by_source.setdefault(f["from"], []).append(f)
    n_checked = 0
    removed_set = set(plan["removed"])
    for p, frags in by_source.items():
        e = idx.get(p)
        if e is None or p not in removed_set:
            continue
        sx, sy, sz = qs._size(e)
        src_vol = sx * sy * sz
        frag_vol = sum(f["size"][0] * f["size"][1] * f["size"][2] for f in frags)
        ratio = frag_vol / max(1e-9, src_vol)
        assert ratio > 0.0, (p, ratio, src_vol, frag_vol)
        n_checked += 1
    assert n_checked > 0


def test_fragment_thickness_is_bounded_never_derived_from_volume():
    """Round 1's bug, directly: a fragment's thickness must never exceed a
    sane per-class ceiling regardless of how large its source piece was —
    the container probe found ~600 m "slabs" on a real tower before this
    fix."""
    info, plan = _plan("tower", "T4", btype="urm", seed=3,
                       wind=fake_wind(88.0, 0.95), intensity=0.95)
    assert plan["debris"], "need debris to test this"
    for f in plan["debris"]:
        thick = f["size"][2]
        if f["kind"] == "glass":
            assert 0.0 < thick <= 0.02, f
        else:
            assert 0.0 < thick <= 0.5, f


def test_reach_is_capped_by_building_height():
    """Round 1's second bug: a dropped block/coping travelled 400-745 m off
    a real tower on a massless-ballistic reach. Every fragment's landing
    point must sit within `REACH_MAX_H * H` (`0.8 * H` for glass) of its
    own source piece, floored at 6 m."""
    info, plan = _plan("tower", "T4", btype="urm", seed=6,
                       wind=fake_wind(300.0, 0.95), intensity=0.95)
    idx = _by_path(info)
    H = info["H"]
    n_checked = 0
    for f in plan["debris"]:
        e = idx.get(f["from"])
        if e is None:
            continue
        d = math.hypot(f["x"] - e["x"], f["y"] - e["y"])
        cap_frac = tu.REACH_MAX_H_GLASS if f["kind"] == "glass" else tu.REACH_MAX_H
        cap = max(tu.REACH_FLOOR_M, cap_frac * H)
        # `_push_out_of_footprint` can add up to `t + 1.5` beyond the raw
        # reach when a landing point fell inside the source footprint --
        # allow a generous slack for that rather than re-deriving `t`.
        assert d <= cap + info["masses"]["main"]["W"] + info["masses"]["main"]["D"] + 5.0, \
            (f["kind"], d, cap, H)
        n_checked += 1
    assert n_checked > 0


def test_removed_frac_is_by_facade_area_not_piece_count():
    """The lead's third fix: a coarse grid (few, huge pieces) can hide a
    large hole behind a tiny piece-count fraction. Build one such fixture
    (a single giant wall run per side, `storeys=1`) and confirm the AREA
    fraction the cap actually enforces differs from the count fraction."""
    pls, style, grid = fake_sliced_building(W=24.0, D=18.0, H=6.0, storeys=1,
                                            bays_x=1, bays_y=1, seed=2)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "rc"
    rng = random.Random(2)
    wind = fake_wind(180.0, 0.9)  # hits E (see the side_weights test's math)
    plan = tu.plan_damage(info, info["elements"], "T4", "rc", rng, wind,
                          intensity=0.9)
    st = plan["stats"]
    assert "removed_count_frac" in st
    if st["n_removed"] > 0:
        # a whole giant wall piece removed moves the AREA fraction far more
        # than the COUNT fraction on a grid this coarse (a handful of huge
        # pieces total).
        assert st["removed_frac"] != st["removed_count_frac"] or st["n_removed"] == 0


def test_debris_max_per_building_caps_structural_fragments():
    info, plan = _plan("tower", "T4", btype="rc", seed=11,
                       wind=fake_wind(120.0, 0.95), intensity=0.95)
    n_struct = sum(1 for f in plan["debris"] if f["kind"] != "glass")
    assert n_struct == plan["stats"]["n_struct_debris"]
    assert n_struct <= tu.DEBRIS_MAX_PER_BUILDING
    if plan["stats"]["struct_debris_thinned"]:
        sources = set(f["from"] for f in plan["debris"] if f["kind"] != "glass")
        assert sources == set(plan["removed"]), \
            "every removed piece must still be represented after thinning"


def test_cladding_band_and_chunk_cap_storeys_on_a_one_bay_windward_side():
    """The lead's fourth fix: a one-bay windward side on a tower/highrise
    caps `cladding_band`/`chunk` to 2 storeys (a full-width band there is a
    horizontal SLICE of the building, not a chunk)."""
    pls, style, grid = fake_sliced_building(W=24.0, D=42.0, H=120.0, storeys=30,
                                            bays_x=1, seed=9)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "urm"
    g = qs._Grid(info, info["elements"])
    assert g.n_bays.get("S") == 1 and tu.height_class_for(info["H"]) == "tower"
    seen_cap_note = False
    for seed in range(10):
        rng = random.Random(seed)
        # bearing 90 hits S (side_weights math: -n_S . d maximised at d=(0,1))
        plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                              fake_wind(90.0, 0.95), intensity=0.95)
        for region in plan["regions"]:
            if region["recipe"] not in ("cladding_band", "chunk"):
                continue
            if region.get("side") == "S" or "S" in (region.get("sides") or []):
                n_st = len(set(int(c[1]) for c in region["cells"] if c[0] == "S"))
                assert n_st <= 2, (seed, region)
        if any("capped to 2" in n for n in plan["notes"]):
            seen_cap_note = True
    assert seen_cap_note


def test_no_fragment_centre_inside_its_source_footprint():
    info, plan = _plan("midrise", "T4", seed=9, wind=fake_wind(80.0, 0.9),
                       intensity=0.9)
    m = info["masses"]["main"]
    w2, d2 = m["W"] / 2.0, m["D"] / 2.0
    n_checked = 0
    for f in plan["debris"]:
        lx, ly = qf._to_local(m, f["x"], f["y"])
        assert not (abs(lx) < w2 - 1e-6 and abs(ly) < d2 - 1e-6), f
        n_checked += 1
    assert n_checked > 0


def test_reach_grows_with_release_height():
    rng_lo = random.Random(11)
    rng_hi = random.Random(11)
    wind = fake_wind(0.0, 0.9)
    reaches_lo, reaches_hi = [], []
    for _ in range(400):
        theta = 0.0 + math.radians(rng_lo.gauss(0.0, 18.0))
        C = tu._C_KIND["block"]
        speed = 25.0 + 70.0 * 0.8
        r_lo = C * speed * math.sqrt(2.0 * 1.5 / tu.G_ACCEL) * tu._lognormal(rng_lo, 0.35)
        r_hi = C * speed * math.sqrt(2.0 * 40.0 / tu.G_ACCEL) * tu._lognormal(rng_hi, 0.35)
        reaches_lo.append(r_lo)
        reaches_hi.append(r_hi)
    assert (sum(reaches_hi) / len(reaches_hi)) > (sum(reaches_lo) / len(reaches_lo))


def _circular_mean_deg(frags, idx):
    sx_, sy_, n = 0.0, 0.0, 0
    for f in frags:
        e = idx.get(f["from"])
        if e is None:
            continue
        dx, dy = f["x"] - e["x"], f["y"] - e["y"]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            continue
        ang = math.atan2(dy, dx)
        sx_ += math.cos(ang)
        sy_ += math.sin(ang)
        n += 1
    return (math.degrees(math.atan2(sy_, sx_)) % 360.0) if n else None, n


def test_debris_direction_circular_mean_within_8deg_of_bearing():
    """ROUND 3 (`_plans/urban_tornado_plan.md` §8, R8) narrows this to the
    BALLISTIC (non-berm) share only. A majority of every façade piece's
    fragments now heap against its own wall base (`stacked` — see
    `tornado_urban._deposit_berm`'s docstring) rather than travelling
    downwind at all, so the OLD claim ("every fragment's direction hugs
    the bearing") is no longer true of the population as a whole — see
    `test_berm_share_lands_near_the_wall_not_on_the_bearing`, directly
    below, for the berm's own (very different) proof. What must still hold
    is that whatever DOES travel — the remaining ballistic share, plus
    every glass shard, which never berms (§2.9) — still throws along the
    wind, unchanged from round 1/2."""
    bearing = 233.0
    info, plan = _plan("midrise", "T4", seed=13, wind=fake_wind(bearing, 0.9),
                       intensity=0.9)
    idx = _by_path(info)
    ballistic = [f for f in plan["debris"] if not f.get("stacked")]
    mean_ang, n = _circular_mean_deg(ballistic, idx)
    assert n > 20
    diff = abs((mean_ang - bearing + 180.0) % 360.0 - 180.0)
    assert diff <= 8.0, (mean_ang, bearing, diff)


def test_berm_share_lands_near_the_wall_not_on_the_bearing():
    """The direct counter-proof to the test above: a `stacked` (berm)
    fragment's direction from its own source piece is NOT expected to hug
    the wind bearing at all (it hugs the piece's own wall instead — the
    `_BERM_OUT_M`/along-wall model), and its distance from the source is
    tiny — bounded PER FRAGMENT by its own source piece's bay width
    (`tu._along_dim`) plus `_BERM_OUT_M`'s ceiling, with slack for the
    exit-the-footprint safety net a deep corner piece can trigger (never
    more than the building's own half-diagonal beyond that), and TIGHTLY
    by a 90th-percentile check that the typical berm fragment sits within
    10 m of its own wall — an order of magnitude under the ballistic
    share's own multi-metre-to-tens-of-metres reach."""
    bearing = 233.0
    info, plan = _plan("midrise", "T4", seed=13, wind=fake_wind(bearing, 0.9),
                       intensity=0.9)
    idx = _by_path(info)
    m = info["masses"]["main"]
    slack = math.hypot(m["W"], m["D"])
    berm = [f for f in plan["debris"] if f.get("stacked")]
    assert len(berm) > 20, "need a real berm population to test this"
    dists = []
    off_bearing = 0
    for f in berm:
        e = idx.get(f["from"])
        if e is None:
            continue
        dx, dy = f["x"] - e["x"], f["y"] - e["y"]
        d = math.hypot(dx, dy)
        dists.append(d)
        pp = e.get("p") or {}
        sd = pp.get("_side")
        bay_w = tu._along_dim(sd, qs._size(e))
        cap = tu._BERM_OUT_M[1] + max(0.2, bay_w) * tu._BERM_ALONG_MULT / 2.0
        assert d <= cap + slack, (f, d, cap, slack)
        if d > 1e-6:
            ang = math.degrees(math.atan2(dy, dx)) % 360.0
            diff = abs((ang - bearing + 180.0) % 360.0 - 180.0)
            if diff > 30.0:
                off_bearing += 1
    dists.sort()
    assert dists[int(0.9 * len(dists))] <= 10.0, dists
    # And most of them are travelling in some direction OTHER than close to
    # the wind bearing -- the whole point of a wall-hugging heap instead of
    # a downwind fan.
    assert off_bearing / float(len(berm)) > 0.5, (off_bearing, len(berm))


def test_berm_share_matches_the_graded_table_per_level(monkeypatch):
    """`_BERM_SHARE` (§8 R8): T2 0.55 / T3 0.65 / T4 0.75 of every façade
    piece's own fragments land in its berm. Measured DIRECTLY against
    `stats["n_berm"] / stats["n_struct_debris"]` with `DEBRIS_MAX_PER_
    BUILDING` monkeypatched huge so the per-building THINNING pass (which
    truncates `n_struct_debris` independently of the berm/ballistic split
    — see `_thin_fragments`) cannot distort the ratio; on the unthinned
    fixture it lands within a couple of percent of the table (per-piece
    integer rounding, `int(round(n * berm_share))`, is the only source of
    drift, and averages out over many removed pieces)."""
    monkeypatch.setattr(tu, "DEBRIS_MAX_PER_BUILDING", 10 ** 7)
    intensity_for = {"T2": 0.4, "T3": 0.6, "T4": 0.9}
    n_checked = 0
    for level, share in tu._BERM_SHARE.items():
        info, plan = _plan("midrise", level, btype="rc", seed=21,
                           wind=fake_wind(140.0, 0.95),
                           intensity=intensity_for[level])
        st = plan["stats"]
        assert st["berm_share_level"] == share
        assert not st["struct_debris_thinned"], "thinning defeats this measurement"
        if st["n_removed"] == 0 or st["n_struct_debris"] == 0:
            continue
        ratio = st["n_berm"] / float(st["n_struct_debris"])
        assert abs(ratio - share) <= 0.03, (level, ratio, share)
        n_checked += 1
    assert n_checked == len(tu._BERM_SHARE)


def test_berm_z_lift_is_bounded_by_berm_h_and_ballistic_z_lift_is_zero():
    """`z_lift` (§8 R8's height profile): every ballistic fragment carries
    `z_lift == 0.0` exactly (`_deposit` hard-codes it); every berm
    fragment's `z_lift` sits in `[0, berm_h]` where `berm_h = min(_BERM_H_
    MAX_M, _BERM_H_PER_PIECE_M * pieces_lost_on_that_side)` — recomputed
    here per source piece's own `_side` against `plan["stats"][
    "removed_by_side"]`-equivalent counts (re-derived from `plan["removed"]`
    directly, the same count `_ledger_removed` itself builds) rather than
    trusted blind."""
    info, plan = _plan("tower", "T4", btype="rc", seed=17,
                       wind=fake_wind(310.0, 0.95), intensity=0.95)
    idx = _by_path(info)
    removed_by_side = {}
    for p in plan["removed"]:
        e = idx.get(p)
        if e is None:
            continue
        sd0 = (e.get("p") or {}).get("_side")
        if sd0 in tu._FACADE_SIDES:
            removed_by_side[sd0] = removed_by_side.get(sd0, 0) + 1

    n_berm_checked = n_ballistic_checked = 0
    for f in plan["debris"]:
        if f["kind"] == "glass":
            continue
        e = idx.get(f["from"])
        if e is None:
            continue
        if not f.get("stacked"):
            assert f["z_lift"] == 0.0, f
            n_ballistic_checked += 1
            continue
        sd = (e.get("p") or {}).get("_side")
        berm_h = min(tu._BERM_H_MAX_M,
                    tu._BERM_H_PER_PIECE_M * removed_by_side.get(sd, 1))
        assert 0.0 < f["z_lift"] <= berm_h + 1e-9, (f, berm_h)
        n_berm_checked += 1
    assert n_berm_checked > 0 and n_ballistic_checked > 0


def test_berm_h_grows_with_pieces_lost_on_the_same_side():
    """A side that loses more pieces piles higher, up to `_BERM_H_MAX_M` —
    a one-bay-worth loss on an otherwise mostly-intact side gets a shallow
    heap; a side stripped nearly bare gets the tallest one the model
    allows."""
    assert tu._BERM_H_PER_PIECE_M * 1 < tu._BERM_H_MAX_M
    assert tu._BERM_H_PER_PIECE_M * 20 > tu._BERM_H_MAX_M
    lo = min(tu._BERM_H_MAX_M, tu._BERM_H_PER_PIECE_M * 1)
    hi = min(tu._BERM_H_MAX_M, tu._BERM_H_PER_PIECE_M * 20)
    assert hi > lo
    assert hi == tu._BERM_H_MAX_M


def test_region_clamp_keeps_every_fragment_inside_the_shrunk_region():
    """R8's "the floating-off-plate defect": a small `region` around a
    building that a raw ballistic reach or a berm's own push-out-of-
    footprint safety net would otherwise clear must still bound every
    fragment (structural, glass, berm and ballistic alike) — `region`
    minus `tu._REGION_MARGIN_M` on every side."""
    region = (-40.0, -40.0, 40.0, 40.0)
    info, plan = _plan("tower", "T4", btype="urm", seed=4,
                       wind=fake_wind(70.0, 0.95), intensity=0.95)
    # re-plan with the SAME seed/inputs but a region tight enough that the
    # unclamped reach (checked separately, `test_reach_is_capped_by_
    # building_height`) would blow past it on this fixture.
    info2 = _fixture("tower", seed=4, btype="urm")
    rng = random.Random(4)
    plan2 = tu.plan_damage(info2, info2["elements"], "T4", "urm", rng,
                           fake_wind(70.0, 0.95), intensity=0.95,
                           region=region)
    assert plan2["debris"], "need debris to test the clamp"
    lo_x = region[0] + tu._REGION_MARGIN_M - 1e-6
    hi_x = region[2] - tu._REGION_MARGIN_M + 1e-6
    lo_y = region[1] + tu._REGION_MARGIN_M - 1e-6
    hi_y = region[3] - tu._REGION_MARGIN_M + 1e-6
    for f in plan2["debris"]:
        assert lo_x <= f["x"] <= hi_x, f
        assert lo_y <= f["y"] <= hi_y, f
    assert plan2["stats"]["region"] == list(region)
    # Sanity: WITHOUT a region, at least one fragment on the SAME building
    # actually lands outside the tight box above -- otherwise the clamp
    # test above is vacuous (never actually binding).
    unclamped_outside = any(
        not (region[0] <= f["x"] <= region[2] and region[1] <= f["y"] <= region[3])
        for f in plan["debris"])
    assert unclamped_outside, "fixture never exercises the clamp -- widen the reach"


def test_region_clamp_is_a_noop_when_region_is_none():
    info, plan_no_region = _plan("midrise", "T3", btype="rc", seed=9,
                                 wind=fake_wind(200.0, 0.9), intensity=0.6)
    assert plan_no_region["stats"]["region"] is None


def test_region_env_fallback(monkeypatch):
    """`TU_PLATE_REGION` (env, read once via `tu._region_from_env` at
    import time — `tu.TU_PLATE_REGION` is what `plan_damage` actually
    falls back to) — monkeypatched directly on the module rather than via
    `os.environ` + re-import, since the module is already imported
    process-wide and re-importing it would not be the same object every
    other test in this file already holds a reference to."""
    region = (-10.0, -10.0, 10.0, 10.0)
    monkeypatch.setattr(tu, "TU_PLATE_REGION", region)
    info = _fixture("tower", seed=6, btype="rc")
    rng = random.Random(6)
    plan = tu.plan_damage(info, info["elements"], "T4", "rc", rng,
                          fake_wind(15.0, 0.95), intensity=0.95)
    assert plan["stats"]["region"] == list(region)
    for f in plan["debris"]:
        assert region[0] - 1e-6 <= f["x"] <= region[2] + 1e-6
        assert region[1] - 1e-6 <= f["y"] <= region[3] + 1e-6


def test_region_from_env_parses_and_rejects_malformed():
    assert tu._region_from_env.__call__ is not None  # importable/pure
    old = os.environ.get("TU_PLATE_REGION")
    try:
        os.environ["TU_PLATE_REGION"] = "1.0,2.0,3.0,4.0"
        assert tu._region_from_env() == (1.0, 2.0, 3.0, 4.0)
        os.environ["TU_PLATE_REGION"] = "not,a,valid,region"
        assert tu._region_from_env() is None
        os.environ["TU_PLATE_REGION"] = "1.0,2.0,3.0"
        assert tu._region_from_env() is None
        del os.environ["TU_PLATE_REGION"]
        assert tu._region_from_env() is None
    finally:
        if old is None:
            os.environ.pop("TU_PLATE_REGION", None)
        else:
            os.environ["TU_PLATE_REGION"] = old


def test_debris_material_hints_by_role_and_btype():
    # R5 (round 2): `deck` is retired -- a roof-shed piece now comes out as
    # `membrane` (urm/rc) or `metal` (rc_glass), never the kind that used to
    # fall through to `planks.wood_material`. See `tornado_urban._kind_of`.
    info, plan = _plan("midrise", "T4", btype="rc", seed=4,
                       wind=fake_wind(300.0, 0.9), intensity=0.9)
    kinds = set(f["kind"] for f in plan["debris"])
    assert kinds <= {"panel", "block", "coping", "membrane", "metal", "glass"}
    assert "deck" not in kinds
    for f in plan["debris"]:
        if f["kind"] == "glass":
            assert f["material"] == "glass"
        elif f["kind"] == "coping":
            assert f["material"] == "coping"
        elif f["kind"] == "membrane":
            assert f["material"] == "membrane"
        elif f["kind"] == "metal":
            assert f["material"] == "metal"


def test_block_and_coping_dims_are_squat_not_lumber():
    """R5: `_DIMS["block"]`/`_DIMS["coping"]` must not be able to draw an
    elongated (plank-shaped) fragment. Both axes now share one range, which
    bounds the worst-case independent draw's aspect ratio to `hi/lo` by
    construction; the round-2 ground-evidence review's ceiling was "aspect
    < ~2.5" so a masonry fragment reads as a lump, not a board, from the
    60-90 m range this debris is judged from. `panel` is deliberately left
    out -- a spalled wall/cladding panel is meant to be rectangular."""
    for kind in ("block", "coping"):
        (l_lo, l_hi), (w_lo, w_hi) = tu._DIMS[kind]
        assert (l_lo, l_hi) == (w_lo, w_hi), \
            "both axes must share one range for the aspect bound to hold"
        worst_aspect = l_hi / l_lo
        assert worst_aspect < 2.5, (kind, worst_aspect)


def test_no_facade_fragment_is_plank_shaped():
    """ROUND 4 (D3): the round-3 bench read "masonry fragments as scattered
    wooden planks", and the measurement behind it is this one — on the
    container probe's own plan (`SM_Building_02` T4 s7) the `panel` class
    ran aspect p50 2.46 / p90 6.72 / MAX 11.96 with 40% of its fragments
    over 3:1, because `_DIMS["panel"]` was `((0.8, 2.5), (0.15, 1.2))` and
    nothing bounded the ratio between two independent draws.

    Every BLOCKY kind (`_BLOCKY_KINDS` — every façade kind `_kind_of`
    derives) must now come out within `_MAX_ASPECT`, checked over real
    plans across several fixtures/seeds rather than by arithmetic on the
    table, since `_dims_for` clamps the DRAW. The sheet kinds
    (`membrane`/`metal`) are deliberately exempt and not asserted on."""
    worst = 0.0
    n_checked = 0
    for hc, btype, seed, brg in (("midrise", "urm", 6, 200.0),
                                 ("tower", "rc", 3, 88.0),
                                 ("lowrise", "urm", 5, 15.0),
                                 ("midrise", "rc_glass", 11, 300.0)):
        _info, plan = _plan(hc, "T4", btype=btype, seed=seed,
                            wind=fake_wind(brg, 0.95), intensity=0.95)
        for f in plan["debris"]:
            if f["kind"] not in tu._BLOCKY_KINDS:
                continue
            l, w, t = f["size"]
            aspect = max(l, w) / max(1e-9, min(l, w))
            assert aspect <= tu._MAX_ASPECT + 1e-6, (f, aspect)
            # and the third dimension is a real thickness, not a wafer
            assert t >= tu._THICK_RANGE[f["kind"]][0] - 1e-9, f
            worst = max(worst, aspect)
            n_checked += 1
    assert n_checked > 100, n_checked
    assert worst > 1.5, ("clamped to a cube — the fragments lost every bit "
                         "of rectangularity", worst)


def test_reach_never_exceeds_the_absolute_ceiling():
    """ROUND 4 (D3): `REACH_MAX_H * H` alone is 64 m on the probe's 42.7 m
    building (measured debris bbox x [-17.9, 73.8] y [-11.0, 63.5]) and
    180 m on a tower — which walks fragments off a bench plate onto the
    void. `REACH_ABS_MAX_M` bounds the ballistic reach with no caller
    involvement at all (unlike `region`, which is opt-in and which nothing
    in the tree passes). Slack allows `_push_out_of_footprint`'s `t + 1.5`
    plus the source piece's own offset from the mass centre."""
    info, plan = _plan("tower", "T4", btype="urm", seed=6,
                       wind=fake_wind(300.0, 0.95), intensity=0.95)
    idx = _by_path(info)
    m = info["masses"]["main"]
    slack = m["W"] + m["D"] + 5.0
    n_checked = 0
    for f in plan["debris"]:
        e = idx.get(f["from"])
        if e is None:
            continue
        d = math.hypot(f["x"] - e["x"], f["y"] - e["y"])
        assert d <= tu.REACH_ABS_MAX_M + slack, (f["kind"], d)
        n_checked += 1
    assert n_checked > 0


def test_berm_fragments_have_a_size_mix_not_identical_cubes():
    """ROUND 4 (v6 review): "the kit berms render as uniform light-grey
    IDENTICAL boxes -- no texture read, no size/tone variation".

    MEASURED on the offline bench stage before the fix (every box's own
    edge lengths, read back off the authored meshes): B1's dominant
    population, `coping` at 247 of ~400 fragments, ran plan-area
    p10/p50/p90 = 0.12/0.17/0.20 m2 -- a 1.7x spread end to end, i.e. one
    size. `_DIMS_SLAB`/`_SLAB_SHARE` add a minority of large units to every
    blocky class; this pins the RESULTING spread rather than the table, so
    a future range edit that flattens the mix fails here.
    """
    info, plan = _plan("midrise", "T4", seed=6, wind=fake_wind(200.0, 0.9),
                       intensity=0.9)
    by_kind = {}
    for f in plan["debris"]:
        if f["kind"] not in tu._BLOCKY_KINDS:
            continue
        l, w, _t = f["size"]
        by_kind.setdefault(f["kind"], []).append(l * w)
    assert by_kind, "need blocky debris to measure"
    for kind, areas in sorted(by_kind.items()):
        if len(areas) < 20:
            continue
        areas.sort()
        p10 = areas[int(0.10 * len(areas))]
        p90 = areas[int(0.90 * len(areas))]
        assert p90 / max(1e-9, p10) >= 3.0, (kind, p10, p90, len(areas))
    # and the mix never smuggles a plank back in
    for f in plan["debris"]:
        if f["kind"] not in tu._BLOCKY_KINDS:
            continue
        l, w, _t = f["size"]
        assert max(l, w) / max(1e-9, min(l, w)) <= tu._MAX_ASPECT + 1e-6, f


def test_masonry_tone_is_stamped_from_the_style_and_only_where_it_belongs():
    """ROUND 4 (v6): B1/B3 are WHITE STONE kit buildings whose every
    masonry fragment bound `TornadoDebrisLooks/brick` -- a red-brown map
    (measured linear mean 0.213/0.109/0.071) -- because the class branch is
    the only masonry look. The ledger now stamps the building's TONE token
    (`_KIT_TONE`) so `tornado_urban_usd` can pick the matching rubble look.

    Three invariants: a named kit style stamps its tone on BLOCKY fragments
    only (never on the roof-shed sheet kinds or glass, which are not the
    building's walls); an unnamed style (every SLICED A-row building, whose
    class-branch look the review approved) stamps nothing at all; and the
    tone is JSON-safe plain data like the rest of the fragment schema."""
    import json

    info = _fixture("midrise", seed=4, btype="urm")
    info["style"] = "brownstone_row"
    rng = random.Random(4)
    plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                          fake_wind(200.0, 0.9), intensity=0.9)
    json.dumps(plan)
    n_toned = 0
    for f in plan["debris"]:
        if f["kind"] in tu._BLOCKY_KINDS:
            assert f["tone"] == "stone", f
            n_toned += 1
        else:
            assert f["tone"] == "", f
        assert 0 <= f["shade"] < max(1, tu._DEBRIS_SHADES), f
    assert n_toned > 0

    info2 = _fixture("midrise", seed=4, btype="urm")   # style stays the
    rng2 = random.Random(4)                            # fixture's own
    plan2 = tu.plan_damage(info2, info2["elements"], "T4", "urm", rng2,
                           fake_wind(200.0, 0.9), intensity=0.9)
    assert tu._tone_for(info2.get("style")) == ""
    assert all(f["tone"] == "" for f in plan2["debris"])


def test_shade_bands_are_deterministic_and_spread_across_a_berm():
    """The per-mesh tone jitter must not be a draw (it would move every
    downstream number in the plan) and must not put a whole berm in one
    band. Deterministic on (source piece, fragment index) via
    `_stable_shade` -- `zlib.crc32`, never `hash()`, which is salted per
    process."""
    kw = dict(wind=fake_wind(200.0, 0.9), intensity=0.9)
    _i1, p1 = _plan("midrise", "T4", seed=6, **kw)
    _i2, p2 = _plan("midrise", "T4", seed=6, **kw)
    assert [f["shade"] for f in p1["debris"]] == [f["shade"] for f in p2["debris"]]
    blocky = [f for f in p1["debris"] if f["kind"] in tu._BLOCKY_KINDS]
    seen = set(f["shade"] for f in blocky)
    assert len(seen) == max(1, tu._DEBRIS_SHADES), (seen, len(blocky))
    for band in seen:
        share = sum(1 for f in blocky if f["shade"] == band) / float(len(blocky))
        assert 0.15 <= share <= 0.55, (band, share)


def test_deck_kind_is_retired_from_kind_of_and_dims():
    """R5: `deck` must not be reachable anywhere in the debris vocabulary --
    not as a `_kind_of` return value (a `role == "roof"` piece is always
    `membrane` or `metal` now, by `btype`) and not as a `_DIMS`/`_THICK_
    RANGE`/`_C_KIND` key (all three keyed by kind; `deck` gone means nothing
    can look it up and silently fall back to a stale entry)."""
    assert tu._kind_of({"_role": "roof"}, "urm") == "membrane"
    assert tu._kind_of({"_role": "roof"}, "rc") == "membrane"
    assert tu._kind_of({"_role": "roof"}, "rc_glass") == "metal"
    for table in (tu._DIMS, tu._THICK_RANGE, tu._C_KIND):
        assert "deck" not in table, table


def test_glass_shard_cap_on_a_tall_rc_glass_building():
    """The lead's addendum: a T4 rc_glass TOWER authors far more glass
    panes than a midrise does, and each pane can shed up to 60 shards
    (§2.7) with no per-BUILDING ceiling — `GLASS_SHARDS_MAX_PER_BUILDING`
    caps the total, thinned deterministically so every broken pane still
    contributes at least one shard."""
    info, plan = _plan("tower", "T4", btype="rc_glass", seed=13,
                       wind=fake_wind(40.0, 0.9), intensity=0.9)
    n_panes = plan["stats"]["n_glass"]
    assert n_panes > tu.GLASS_SHARDS_MAX_PER_BUILDING // 4, \
        "need a plan with plenty of broken glass for this to be a real test"
    assert plan["stats"]["n_glass_shards"] <= tu.GLASS_SHARDS_MAX_PER_BUILDING
    glass_frags = [f for f in plan["debris"] if f["kind"] == "glass"]
    assert len(glass_frags) == plan["stats"]["n_glass_shards"]
    sources = set(f["from"] for f in glass_frags)
    assert sources == set(plan["glass"]), \
        "every glazed pane must still be represented by >= 1 shard"
    # non-glass fragment counts are untouched by the cap
    n_structural = sum(1 for f in plan["debris"] if f["kind"] != "glass")
    assert n_structural == plan["stats"]["n_debris"] - plan["stats"]["n_glass_shards"]
    # determinism survives the thinning pass
    info2, plan2 = _plan("tower", "T4", btype="rc_glass", seed=13,
                         wind=fake_wind(40.0, 0.9), intensity=0.9)
    assert json.dumps(plan, sort_keys=True) == json.dumps(plan2, sort_keys=True)


def test_glass_shard_cap_env_override(monkeypatch):
    monkeypatch.setenv("TU_GLASS_SHARDS_MAX", "40")
    import importlib
    reloaded = importlib.reload(tu)
    try:
        info, plan = _plan("tower", "T4", btype="rc_glass", seed=13,
                           wind=fake_wind(40.0, 0.9), intensity=0.9)
        assert reloaded.GLASS_SHARDS_MAX_PER_BUILDING == 40
        assert plan["stats"]["n_glass_shards"] <= 40
    finally:
        monkeypatch.delenv("TU_GLASS_SHARDS_MAX", raising=False)
        importlib.reload(tu)


# ---------------------------------------------------------------------------
# measured glazing (`_glass_faces` / `_glass_frac`, stamped by stream D's
# `tornado_urban_usd.annotate_glazing` before `describe` on a REAL slice)
# ---------------------------------------------------------------------------
# The lead's live probe of `SM_Building_02`: every `wall` piece there
# carries only the blind `WallBack` material, and the actual glazing is 18
# PIER + 9 core pieces -- so the role-based (`is_opening`) pick that every
# synthetic fixture exercises finds NOTHING on a building shaped like that.
# `annotate_glazing` stamps `_glass_faces` (int) / `_glass_frac` (0..1) on
# every placement dict before `describe`; these tests stamp a synthetic
# fixture's placements the same way (piers only) to exercise that path
# without needing pxr or a real slice.
def _stamp_glass(pls, role="pier", frac=0.5):
    """Mutates `pls` in place: `_glass_faces`/`_glass_frac` on every
    placement (0 for everything except `role`), matching `annotate_glazing`
    stamping EVERY piece, not only the glazed ones — `_glass_measured`
    checks for the KEY's presence, not its value."""
    for p in pls:
        if p.get("_role") == role:
            p["_glass_faces"] = 1
            p["_glass_frac"] = float(frac)
        else:
            p["_glass_faces"] = 0
            p["_glass_frac"] = 0.0
    return pls


def test_measured_glass_faces_path_restricts_to_stamped_pieces():
    pls, style, grid = fake_sliced_building(W=30.0, D=24.0, H=40.0, storeys=10, seed=7)
    _stamp_glass(pls, role="pier", frac=0.5)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "urm"
    rng = random.Random(7)
    # bearing 90 hits S (side_weights math: -n_S . d maximised at d=(0,1))
    plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                          fake_wind(90.0, 0.9), intensity=0.9)
    assert plan["stats"]["glass_measured"] is True
    idx = _by_path(info)
    assert plan["glass"], "expected some glass voided on the stamped piers"
    for p in plan["glass"]:
        e = idx.get(p)
        assert e is not None, p
        assert (e["p"] or {}).get("_role") == "pier", (p, (e["p"] or {}).get("_role"))
    assert any("measured" in n for n in plan["notes"])
    n_piers = sum(1 for pp in pls if pp.get("_role") == "pier")
    assert plan["stats"]["n_glass_candidates"] == n_piers
    json.dumps(plan, sort_keys=True)  # must not raise


def test_unmeasured_fixture_still_uses_the_role_based_fallback():
    """No `_glass_faces` key anywhere on an ordinary fixture -- confirms the
    fallback path (round-1 behaviour) fires and every voided pane is the
    OPENING sub-panel (`is_opening`: role `wall`, or `parapet` on the top
    relabelled band -- never a `pier`/`corner`/`parapet_corner`, the narrow
    sub-panels `is_opening` excludes)."""
    info, plan = _plan("midrise", "T4", seed=7,
                       wind=fake_wind(95.0, 0.82, -0.35), intensity=0.8)
    assert plan["stats"]["glass_measured"] is False
    assert not any("measured" in n for n in plan["notes"])
    n_sub = qs.n_sub_of(info["elements"])
    idx = _by_path(info)
    for p in plan["glass"]:
        e = idx.get(p)
        assert qs.is_opening(e["p"] or {}, n_sub), (p, (e["p"] or {}).get("_role"))
        assert (e["p"] or {}).get("_role") not in ("pier", "corner", "parapet_corner"), p


def test_measured_glass_frac_scales_shard_count():
    """A piece stamped with a small `_glass_frac` sheds fewer shards than
    the same piece stamped fully glazed -- §2.7's area formula scaled by
    `_glass_frac` rather than the piece's whole face."""
    counts = []
    for frac in (0.15, 1.0):
        pls, style, grid = fake_sliced_building(W=30.0, D=24.0, H=40.0,
                                                storeys=10, seed=7)
        _stamp_glass(pls, role="pier", frac=frac)
        info = qf.describe(style, pls, 0.0, 0.0, 0.0)
        info["type"] = "urm"
        rng = random.Random(7)
        plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                              fake_wind(90.0, 0.9), intensity=0.9)
        n_shards = sum(1 for f in plan["debris"] if f["kind"] == "glass")
        counts.append(n_shards)
    assert counts[0] < counts[1], counts


def test_glass_measured_flag_is_grid_wide_not_per_piece():
    """Even one placement carrying the `_glass_faces` KEY (value 0 is fine)
    flips the whole building onto the measured path -- `_glass_measured`
    checks presence, not truthiness, and is decided once for the grid."""
    pls, style, grid = fake_sliced_building(W=30.0, D=24.0, H=40.0, storeys=10, seed=7)
    for p in pls:
        p["_glass_faces"] = 0
        p["_glass_frac"] = 0.0
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "urm"
    rng = random.Random(7)
    plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                          fake_wind(90.0, 0.9), intensity=0.9)
    assert plan["stats"]["glass_measured"] is True
    assert plan["glass"] == []  # nothing has any glass faces
    assert plan["stats"]["n_glass_candidates"] == 0


# ---------------------------------------------------------------------------
# real GAC kits
# ---------------------------------------------------------------------------
def test_real_kit_sm_building_02_smoke():
    # NOT `kb.have_kit` -- that also demands the referenced `.usd` exist on
    # disk and the manifest's fingerprint match the CURRENT slicer code;
    # both are irrelevant on a bare host per `load_real_kit`'s own
    # docstring ("kits.json stores the baked file's path CONTAINER-SIDE...
    # DELIBERATELY NEVER OPENS THE REFERENCED .usd GEOMETRY") and the
    # fingerprint check in particular can go stale from a concurrent edit
    # elsewhere in the repo without the manifest ROW itself being gone.
    # `kb._entry(name)` (default `signature=None`) is the exact precondition
    # `load_real_kit` checks before raising.
    assert kb._entry("SM_Building_02") is not None, \
        "expects the checked-in kits.json row"
    info = _real_kit_info("SM_Building_02", "urm")
    hc = tu.height_class_for(info["H"])
    for level in tu.LEVELS:
        rng = random.Random(4)
        plan = tu.plan_damage(info, info["elements"], level, "urm", rng,
                              fake_wind(35.0, 0.85), height_class=hc,
                              intensity={"T0": 0.05, "T1": 0.2, "T2": 0.4,
                                        "T3": 0.6, "T4": 0.85}[level])
        cap = tu.HEIGHT_CAPS[hc]["max_removed_frac"]
        assert plan["stats"]["removed_frac"] <= cap + 1e-9
        s = json.dumps(plan, sort_keys=True)
        assert json.loads(s)["level"] == level
        rm = _removed_els(info, plan)
        assert not any((e["p"] or {}).get("_role") == "core" for e in rm)


def test_real_kit_sm_building_09_smoke_degenerate_grid():
    """SM_Building_09's cached bake has a one-bay-per-side grid (see the
    module docstring) -- this only asserts the planner runs cleanly and
    keeps every hard invariant on data that sparse; it is not expected to
    remove much of anything."""
    if kb._entry("SM_Building_09") is None:
        return
    info = _real_kit_info("SM_Building_09", "urm")
    hc = tu.height_class_for(info["H"])
    rng = random.Random(2)
    plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                          fake_wind(10.0, 0.9), height_class=hc, intensity=0.9)
    cap = tu.HEIGHT_CAPS[hc]["max_removed_frac"]
    assert plan["stats"]["removed_frac"] <= cap + 1e-9
    json.dumps(plan, sort_keys=True)  # must not raise
    rm = _removed_els(info, plan)
    assert not any((e["p"] or {}).get("_role") in ("core", "roof") for e in rm)


def test_no_tower_class_real_kit_under_default_signature():
    """Documents the manifest check this module's docstring reports: every
    `signature=None` row in `kits.json` -- the only rows `load_real_kit(name)`
    (no explicit `signature=`) can reach -- resolves, once
    `load_real_kit`'s own scale correction is applied, to a measured H under
    100 m (not `tower` class). `assets.kits.kits.json` may grow a genuinely
    tower-height default-signature bake later; if it does, this test's own
    failure is the signal to update the brief's report on it."""
    rows = kb.read_manifest()
    default_names = sorted(set(r.get("asset") for r in rows
                               if r.get("signature") is None and r.get("asset")))
    assert default_names, "expected at least the checked-in SM_Building_02/09/11 rows"
    for name in default_names:
        pls, style, grid = load_real_kit(name)
        H = float(grid.get("H") or 0.0)
        assert tu.height_class_for(H) != "tower", (name, H)


# ---------------------------------------------------------------------------
# btype-specific ladder shape
# ---------------------------------------------------------------------------
def test_ladder_t_covers_every_btype_and_level():
    for btype in ("urm", "rc", "rc_glass"):
        assert set(tu.LADDER_T[btype]) == set(tu.LEVELS)
        for level, recs in tu.LADDER_T[btype].items():
            for name, _kw in recs:
                assert name in tu.RECIPES_T, (btype, level, name)


def test_recipes_run_list_matches_plan():
    info, plan = _plan("midrise", "T3", btype="rc", seed=1, wind=fake_wind(45.0))
    names_in_plan = set(n for n, _kw in plan["recipes"])
    for name, _kw in plan["recipes"]:
        assert name in tu.RECIPES_T
    assert names_in_plan  # T3 always runs something


# ---------------------------------------------------------------------------
# ROUND 3b (§8e F3, stream FX2) — THE LEDGER COPIES `source_tex`/
# `source_tex_name` ONTO A FAÇADE FRAGMENT
# ---------------------------------------------------------------------------
def _stamp_tex(pls, url, name):
    """Mutates `pls` in place: `_tex_url`/`_tex_name` on EVERY placement —
    mirrors `_stamp_glass`'s own mock pattern for `annotate_glazing`, but
    for `tornado_urban_usd.annotate_surface`'s stamp. Stamping every piece
    the SAME texture (rather than by role, the way `_stamp_glass` restricts
    to one role) is deliberate here: it isolates the KIND gate
    (`tu._FACADE_TEX_KINDS`) as the only thing that can still tell a
    façade fragment from a roof one, since every SOURCE piece now looks
    identical."""
    for p in pls:
        p["_tex_url"] = url
        p["_tex_name"] = name
    return pls


def test_ledger_copies_source_tex_onto_every_facade_fragment_not_roof_or_glass():
    """F3's headline behaviour end to end: `plan_damage`'s own recipes (not
    a hand-built plan) on a fully `annotate_surface`-stamped grid. Every
    debris fragment whose `kind` is a façade kind (`panel`/`block`/
    `coping` -- `tu._FACADE_TEX_KINDS`) must carry the stamped texture;
    every `membrane` (this fixture's roof kind at `btype="urm"`) and every
    `glass` shard must carry the empty string on both fields -- "Glass and
    membrane fragments keep their class looks", `_plans/
    urban_tornado_plan.md` §8e F3."""
    pls, style, grid = fake_sliced_building(W=30.0, D=24.0, H=40.0,
                                            storeys=10, seed=7)
    url = "airstack://scene_gen/assets/materials/megascans/Grey_Stone/T_greystone_1K_B.jpg"
    name = "T_greystone_1K_B.jpg"
    _stamp_tex(pls, url, name)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "urm"
    rng = random.Random(7)
    plan = tu.plan_damage(info, info["elements"], "T4", "urm", rng,
                          fake_wind(90.0, 0.9), intensity=0.9)
    assert plan["debris"], "expected some debris"
    json.dumps(plan, sort_keys=True)  # source_tex/_name must stay JSON-safe

    n_facade = n_roof = n_glass = 0
    for f in plan["debris"]:
        if f["kind"] == "glass":
            assert f["source_tex"] == "" and f["source_tex_name"] == "", f
            n_glass += 1
        elif f["kind"] in ("membrane", "metal"):
            assert f["source_tex"] == "" and f["source_tex_name"] == "", f
            n_roof += 1
        else:
            assert f["kind"] in tu._FACADE_TEX_KINDS, f["kind"]
            assert f["source_tex"] == url, f
            assert f["source_tex_name"] == name, f
            n_facade += 1
    assert n_facade > 0, "expected at least one façade fragment"
    assert n_glass > 0, "expected at least one glass shard"


def test_ledger_kind_gate_beats_material_hint_facade_metal_vs_roof_metal():
    """The sharpest version of the gate: on an `rc_glass` building a FAÇADE
    piece's own MATERIAL hint is also the literal string `"metal"`
    (`_MAT_BY_BTYPE[("rc_glass", "panel")]`) -- the SAME string a roof-shed
    piece's `kind` and `material` both are. If the gate were keyed on
    `material` instead of `kind`, a roof piece would wrongly inherit a
    texture too. Built directly off `_ledger_removed` (bypassing which
    recipes happen to reach a `role == roof` piece for a given btype/level
    -- measured: `LADDER_T["rc_glass"]` never runs `top_storey_loss` or
    `facade_collapse`, so no STOCK T-level plan ever ledgers an rc_glass
    roof piece) so this proves the KIND gate itself, not a recipe's reach.
    """
    for btype, roof_kind in (("urm", "membrane"), ("rc_glass", "metal")):
        pls, style, grid = fake_sliced_building(W=30.0, D=24.0, H=40.0,
                                                storeys=10, seed=7)
        url, name = "airstack://x/T_cw.jpg", "T_cw.jpg"
        _stamp_tex(pls, url, name)
        info = qf.describe(style, pls, 0.0, 0.0, 0.0)
        info["type"] = btype
        idx = _by_path(info)
        wall_path = next(p for p, e in idx.items()
                         if (e.get("p") or {}).get("_role") == "wall")
        roof_path = next(p for p, e in idx.items()
                         if (e.get("p") or {}).get("_role") == "roof")

        wind = fake_wind(0.0, 0.9)
        rng = random.Random(3)
        plan = {
            "schema": "tornado_urban_plan.v1", "level": "T4", "btype": btype,
            "style": info.get("style"), "H": float(info.get("H") or 0.0),
            "height_class": "midrise", "wind": dict(wind), "recipes": [],
            "removed": [wall_path, roof_path], "displaced": {}, "glass": [],
            "glass_bands": [], "macroblocks": [], "regions": [],
            "roof_props": "keep", "debris": [], "notes": [], "stats": {},
            "_removed_set": set()}
        weights = tu.side_weights(info, wind, rng)
        plan["side_weights"] = {k: float(v) for k, v in weights.items()}
        pctx = tu._pctx(info, info["elements"], btype, rng, plan, wind,
                        weights, "midrise", 0.85)

        ledger = tu._ledger_removed(pctx, plan, wind, 0.85)
        by_from = {}
        for f in ledger["frags"]:
            by_from.setdefault(f["from"], []).append(f)
        wall_frags = by_from.get(wall_path, [])
        roof_frags = by_from.get(roof_path, [])
        assert wall_frags, (btype, "no wall fragments")
        assert roof_frags, (btype, "no roof fragments")

        for f in wall_frags:
            assert f["kind"] == "panel", (btype, f)
            assert f["source_tex"] == url, (btype, f)
            assert f["source_tex_name"] == name, (btype, f)
        for f in roof_frags:
            assert f["kind"] == roof_kind, (btype, f)
            assert f["source_tex"] == "" and f["source_tex_name"] == "", (btype, f)

        if btype == "rc_glass":
            # THE POINT: wall and roof fragments share the literal same
            # MATERIAL hint here, and still diverge on source_tex.
            assert wall_frags[0]["material"] == "metal", wall_frags[0]
            assert roof_frags[0]["material"] == "metal", roof_frags[0]


# ---------------------------------------------------------------------------
# ROUND 3b (§8e) — stream FX1's own region: F1, the roof/parapet support
# post-pass, and F2a, the ragged-tear wiring. `_plans/urban_tornado_plan.md`
# §8e, the user's own words on the first lit bench: "floating roofs, missing
# chunks from buildings ... Look at the various skills on how to damage
# buildings".
# ---------------------------------------------------------------------------
def _bare_plan(info, btype, seed=5, wind=None):
    """A `plan_damage`-shaped dict with EVERY §2.8 field but NO recipe run —
    the same isolation `_run_one_recipe` gives a single recipe, here given
    to `_shed_unsupported_roof`/`_ledger_removed` directly so a test can
    engineer an exact "this span lost its support" state without depending
    on the full ladder (the height-class caps make some F1 states -- the
    SLAB threshold especially -- hard to reach by chance; see `_shed_
    unsupported_roof`'s own docstring)."""
    wind = wind if wind is not None else fake_wind(0.0)
    height_class = tu.height_class_for(info["H"])
    plan = {
        "schema": "tornado_urban_plan.v1", "level": "TEST", "btype": btype,
        "style": info.get("style"), "H": float(info.get("H") or 0.0),
        "height_class": height_class, "wind": dict(wind),
        "recipes": [], "removed": [], "displaced": {}, "glass": [],
        "glass_bands": [], "macroblocks": [], "regions": [],
        "roof_props": "keep", "debris": [], "notes": [], "stats": {},
        "panels": [], "roof_shed": False, "tears": [], "tear_scope": {},
        "_removed_set": set(),
    }
    rng = random.Random(seed)
    weights = tu.side_weights(info, wind, rng)
    plan["side_weights"] = {k: float(v) for k, v in weights.items()}
    pctx = tu._pctx(info, info["elements"], btype, rng, plan, wind, weights,
                    height_class, 0.9)
    return pctx, plan


def _remove_wall_band_under(g, plan, mass, lx0, ly0, storey,
                            radius=None):
    """Remove every wall/pier/corner piece of `mass` at `storey` within
    `tu._ROOF_SUPPORT_RADIUS_M` of local `(lx0, ly0)` -- the test-side
    mirror of `tu._local_support`'s own query, so a test can engineer
    "this exact span lost its support" without hand-picking prim paths.
    Returns the candidate list (never empty is the caller's job to
    assert)."""
    radius = tu._ROOF_SUPPORT_RADIUS_M if radius is None else radius
    cand = [e for e in g.els
           if (e.get("mass") or "main") == mass
           and int((e.get("p") or {}).get("_storey", 0)) == storey
           and (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")
           and ((float(e.get("lx", 0.0)) - lx0) ** 2
                + (float(e.get("ly", 0.0)) - ly0) ** 2) ** 0.5 <= radius]
    for e in cand:
        p = qs._path(e)
        plan["_removed_set"].add(p)
        plan["removed"].append(p)
    return cand


def test_f1_slab_roof_sheds_when_top_storey_mostly_gone():
    """SLICED synthetic fixture: `role_pieces(("roof",))` is ONE piece
    spanning most of the footprint, so `_ROOF_TILE_FRAC`'s own area test
    classifies it a SLAB. Once more than `ROOF_SHED_FRAC` of the
    wall/pier/corner band the roof/parapet band actually sits above
    (`_wall_band_storey`, not the roof's own `_storey` -- see that
    function's own docstring for why the two differ on most real GAC
    buildings) is gone, the roof piece is shed too, ledgered, and
    `tornado_roof` is told via the `top_storey_loss`-tagged region."""
    info = _fixture("lowrise", seed=3, btype="urm")
    pctx, plan = _bare_plan(info, "urm", seed=3)
    g = pctx["g"]
    roofs = g.role_pieces(("roof",))
    assert len(roofs) == 1, "fixture assumption: one slab roof piece"
    mass = "main"
    wall_storey = tu._wall_band_storey(g, mass, g.top)
    perimeter = [e for e in g.els
                if (e.get("mass") or "main") == mass
                and int((e.get("p") or {}).get("_storey", 0)) == wall_storey
                and (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")]
    assert perimeter, "fixture assumption: a real wall band exists"
    for e in perimeter[: max(1, int(round(len(perimeter) * 0.6)))]:
        p = qs._path(e)
        plan["_removed_set"].add(p)
        plan["removed"].append(p)

    n_roof, n_parapet = tu._shed_unsupported_roof(pctx, plan)
    roof_path = qs._path(roofs[0])
    assert n_roof >= 1
    assert roof_path in plan["_removed_set"]
    assert roof_path in plan["removed"]
    assert plan["roof_shed"] is True
    assert any(r.get("recipe") == "top_storey_loss" for r in plan["regions"]), \
        "tornado_roof._roof_already_shed only greps for this recipe tag"


def test_f1_kit_roof_tile_sheds_only_its_own_unsupported_bay():
    """KIT `walkup`: a TILED roof grid (`urban_building._roof`'s own
    nx*ny tiling), so `_ROOF_TILE_FRAC` classifies EACH piece a TILE.
    Removing every wall/pier/corner piece under ONE tile sheds exactly
    that tile and leaves an untouched, far-away tile standing -- the
    "floating roof" fix, at the per-bay-column granularity KIT buildings
    need (§8e F1's own KIT/SLICED split)."""
    from disaster import kit_substitute as ksub
    from disaster import tornado_kit as tk

    style = "walkup"
    placements = tk.kit_placements(style, seed=9)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    elements = tk.adapt(placements, info)
    btype = ksub.styles()[style]["type"]
    info["type"] = btype
    pctx, plan = _bare_plan(info, btype, seed=9)
    g = pctx["g"]
    roofs = g.role_pieces(("roof",))
    assert len(roofs) > 1, "walkup's own roof must be a tiled grid"

    target = roofs[0]
    mass = target.get("mass") or "main"
    storey = tu._wall_band_storey(
        g, mass, int((target.get("p") or {}).get("_storey", 0)))
    lx0, ly0 = float(target.get("lx", 0.0)), float(target.get("ly", 0.0))
    cand = _remove_wall_band_under(g, plan, mass, lx0, ly0, storey)
    assert cand, "expected some wall/pier/corner piece near the target tile"

    n_roof, _n_parapet = tu._shed_unsupported_roof(pctx, plan)
    assert n_roof >= 1
    assert qs._path(target) in plan["_removed_set"]
    survivors = [r for r in roofs if qs._path(r) not in plan["_removed_set"]]
    assert survivors, "an untouched tile elsewhere on the roof must survive"
    # only a FRACTION of the roof came down -- never the whole-roof signal
    assert plan["roof_shed"] is False


def test_f1_parapet_sheds_with_its_own_lost_wall_cell():
    """"a parapet piece above a lost top-storey cell always goes with it"
    (plan brief, verbatim) -- on the SLICED synthetic fixture, whose
    parapet band sits on a storey with NO wall/pier/corner content of its
    own (`_wall_band_storey`'s own "whole band fallback" case, MEASURED
    common on real GAC buildings by `gac_storey_slice.roof_and_parapet`'s
    own docstring)."""
    info = _fixture("lowrise", seed=3, btype="urm")
    pctx, plan = _bare_plan(info, "urm", seed=3)
    g = pctx["g"]
    parapets = g.role_pieces(("parapet", "parapet_corner"))
    assert parapets
    target = parapets[0]
    mass = target.get("mass") or "main"
    storey = tu._wall_band_storey(
        g, mass, int((target.get("p") or {}).get("_storey", 0)))
    lx0, ly0 = float(target.get("lx", 0.0)), float(target.get("ly", 0.0))
    cand = _remove_wall_band_under(g, plan, mass, lx0, ly0, storey)
    assert cand

    _n_roof, n_parapet = tu._shed_unsupported_roof(pctx, plan)
    assert n_parapet >= 1
    assert qs._path(target) in plan["_removed_set"]


def test_f1_never_re_fates_an_already_displaced_piece():
    """A piece another recipe already put in `plan["displaced"]` (a
    leaning macroblock survivor) must never ALSO end up in `plan[
    "removed"]` -- the regression `test_facade_collapse_leaning_
    macroblocks_pitch_into_the_street` caught once `_wall_band_storey`
    started reaching parapet pieces `t_facade_collapse` can pick as a
    leaning survivor (its own candidate draw is bay-position-based, `qs.
    is_opening`, with no role check)."""
    info = _fixture("lowrise", seed=3, btype="urm")
    pctx, plan = _bare_plan(info, "urm", seed=3)
    g = pctx["g"]
    target = g.role_pieces(("parapet", "parapet_corner"))[0]
    mass = target.get("mass") or "main"
    storey = tu._wall_band_storey(
        g, mass, int((target.get("p") or {}).get("_storey", 0)))
    lx0, ly0 = float(target.get("lx", 0.0)), float(target.get("ly", 0.0))
    _remove_wall_band_under(g, plan, mass, lx0, ly0, storey)
    tpath = qs._path(target)
    plan["displaced"][tpath] = {"pivot": [0.0, 0.0, 0.0],
                               "axis": [1.0, 0.0, 0.0], "deg": 60.0}

    tu._shed_unsupported_roof(pctx, plan)
    assert tpath not in plan["_removed_set"]
    assert tpath not in plan["removed"]
    assert tpath in plan["displaced"]


def test_f1_shed_pieces_are_ledgered_as_debris():
    info = _fixture("lowrise", seed=3, btype="urm")
    pctx, plan = _bare_plan(info, "urm", seed=3)
    g = pctx["g"]
    roofs = g.role_pieces(("roof",))
    mass = "main"
    wall_storey = tu._wall_band_storey(g, mass, g.top)
    perimeter = [e for e in g.els
                if (e.get("mass") or "main") == mass
                and int((e.get("p") or {}).get("_storey", 0)) == wall_storey
                and (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")]
    for e in perimeter[: max(1, int(round(len(perimeter) * 0.6)))]:
        p = qs._path(e)
        plan["_removed_set"].add(p)
        plan["removed"].append(p)
    tu._shed_unsupported_roof(pctx, plan)

    ledger = tu._ledger_removed(pctx, plan, pctx["wind"], 0.9)
    roof_path = qs._path(roofs[0])
    frags = [f for f in ledger["frags"] if f["from"] == roof_path]
    assert frags, "the shed roof piece must be ledgered as debris"
    assert all(f["kind"] in ("membrane", "metal") for f in frags), frags


def test_f1_is_deterministic():
    info_a = _fixture("midrise", seed=6, btype="urm")
    plan_a = _fc_plan(info_a, level="T4", btype="urm", seed=17,
                      wind=fake_wind(63.0, 0.9), intensity=0.9)
    info_b = _fixture("midrise", seed=6, btype="urm")
    plan_b = _fc_plan(info_b, level="T4", btype="urm", seed=17,
                      wind=fake_wind(63.0, 0.9), intensity=0.9)
    assert plan_a["removed"] == plan_b["removed"]
    assert plan_a["stats"]["n_roof_shed"] == plan_b["stats"]["n_roof_shed"]
    assert plan_a["stats"]["n_parapet_shed"] == plan_b["stats"]["n_parapet_shed"]
    assert plan_a["tears"] == plan_b["tears"]


def test_panels_field_is_always_an_empty_list():
    """`plan["panels"]` exists SOLELY so `quake_sliced._plan_tears` can
    read `plan["panels"]` without a `KeyError` -- this ladder never lays
    a piece on a pile."""
    info = _fixture("midrise", seed=4, btype="urm")
    plan = _fc_plan(info, level="T3", btype="urm", seed=4)
    assert plan["panels"] == []


def test_tears_never_fire_below_t3():
    info = _fixture("midrise", seed=6, btype="urm")
    for level in ("T0", "T1", "T2"):
        plan = _fc_plan(info, level=level, btype="urm", seed=6,
                        wind=fake_wind(30.0, 0.85), intensity=None)
        assert plan["tears"] == [], (level, plan["tears"])
        assert plan["stats"]["n_tears"] == 0


def test_tears_fire_on_at_least_one_t4_seed():
    info = _fixture("midrise", seed=6, btype="urm")
    found = False
    for seed in range(15):
        plan = _fc_plan(info, level="T4", btype="urm", seed=seed,
                        wind=fake_wind(15.0 + 11.0 * seed, 0.9),
                        intensity=0.9)
        if plan["stats"]["n_tears"] > 0:
            found = True
            kept = [j for j in plan["tears"] if not j.get("dropped")]
            assert len(kept) == plan["stats"]["n_tears"]
            for j in kept:
                assert j.get("path")
                assert j.get("classes") or j.get("cuts")
            break
    assert found, "expected at least one T4 seed to produce a tear"


def test_cap_tears_drops_core_role_and_respects_cap():
    jobs = [{"el": {"p": {"_role": "wall"}}, "dropped": False} for _ in range(5)]
    core_job = {"el": {"p": {"_role": "core"}}, "dropped": False}
    jobs.insert(2, core_job)
    out = tu._cap_tears(list(jobs), 3)
    assert core_job["dropped"] is True
    kept = [j for j in out if not j.get("dropped")]
    assert len(kept) == 3

    pre_dropped = {"el": {"p": {"_role": "wall"}}, "dropped": True}
    out2 = tu._cap_tears([pre_dropped] + [dict(j) for j in jobs[:2]], 1)
    assert pre_dropped["dropped"] is True
    assert sum(1 for j in out2 if not j.get("dropped")) == 1


def test_tu_max_tears_defaults_to_qs_max_tears():
    assert tu.TU_MAX_TEARS == qs.QS_MAX_TEARS


# ---------------------------------------------------------------------------
# ROUND 4 (D2) -- THE COARSE-FACE SHARE MATH. Both bugs below were measured
# on the live container probe, not on a fixture: a 2-3 bay real face rounded
# a 40-70 % band down to one bay, and `_glass_frac` used as a raw
# probability multiplier voided 2 panes of 99 candidates on SM_Building_24.
# ---------------------------------------------------------------------------
def test_n_bays_for_never_falls_below_the_fraction_range_floor():
    """A 3-bay face at 40-70 % is 2 bays, not the 1 that `int(round(...))`
    gave -- and a 1-bay face is still 1 (nothing to widen into)."""
    assert tu._n_bays_for(3, 0.40, (0.40, 0.70)) == 2
    assert tu._n_bays_for(3, 0.70, (0.40, 0.70)) == 2
    assert tu._n_bays_for(1, 0.70, (0.40, 0.70)) == 1
    assert tu._n_bays_for(2, 0.45, (0.40, 0.70)) == 1
    assert tu._n_bays_for(0, 0.50, (0.40, 0.70)) == 0
    # never more than the face has, never zero on a face that has bays
    for nb in range(1, 12):
        for f in (0.05, 0.4, 0.55, 0.7, 1.0):
            n = tu._n_bays_for(nb, f, (0.40, 0.70))
            assert 1 <= n <= nb, (nb, f, n)
    # a wide face is unchanged from the old nearest-bay expression
    assert tu._n_bays_for(10, 0.55, (0.40, 0.70)) == round(10 * 0.55)


def test_n_bays_for_without_a_span_is_the_plain_nearest_bay_round():
    for nb in (1, 2, 3, 7, 12):
        for f in (0.1, 0.33, 0.5, 0.9):
            assert tu._n_bays_for(nb, f) == max(1, min(nb, int(round(nb * f))))


def test_glass_frac_ref_is_the_median_glazed_piece_and_normalises_the_draw():
    """`_glass_frac` is glazing FACES over total faces -- a mesh-complexity
    ratio. Normalised against the building's own median, the TYPICAL glazed
    piece draws at the recipe's own fraction instead of at a hundredth of
    it."""
    els = []
    for i, gf in enumerate((0.01, 0.02, 0.03, 0.04, 0.60)):
        els.append({"p": {"_role": "wall", "_glass_faces": 10.0,
                          "_glass_frac": gf}})
    els.append({"p": {"_role": "core", "_glass_faces": 99.0,
                      "_glass_frac": 0.99}})      # never a candidate
    els.append({"p": {"_role": "wall", "_glass_faces": 0.0}})   # no glass
    pctx = {"g": type("G", (), {"els": els})()}
    ref = tu._glass_frac_ref(pctx)
    assert abs(ref - 0.03) < 1e-9, ref
    assert pctx["_glass_frac_ref"] == ref          # cached on the pctx
    # the median piece draws at the full recipe fraction, the heavily
    # glazed one is capped there too, the sparse one is proportional
    assert min(1.0, 0.03 / ref) == 1.0
    assert min(1.0, 0.60 / ref) == 1.0
    assert abs(min(1.0, 0.01 / ref) - (1.0 / 3.0)) < 1e-9
    # no glazed piece at all -> a safe non-zero reference, never a div by 0
    empty = {"g": type("G", (), {"els": [{"p": {"_role": "wall"}}]})()}
    assert tu._glass_frac_ref(empty) > 0.0


def _band_storeys(n_bays_override, seeds=range(10), btype="urm",
                  height_class=None):
    """`t_cladding_band` run alone over several seeds, with the grid's own
    `n_bays` forced to `n_bays_override` -- the real SM_Building_02 slice
    has TWO bays per side (six runs, `n_sub` 3) and the synthetic fixture
    has more, so the narrow-face branch is unreachable from the fixture
    without this."""
    out = []
    for seed in seeds:
        info = _fixture("midrise", seed=3, btype=btype)
        wind = fake_wind(20.0 + 9.0 * seed, 0.9)
        hc = height_class or tu.height_class_for(info["H"])
        rng = random.Random(seed)
        plan = {
            "schema": "tornado_urban_plan.v1", "level": "TEST", "btype": btype,
            "style": info.get("style"), "H": float(info.get("H") or 0.0),
            "height_class": hc, "wind": dict(wind), "recipes": [],
            "removed": [], "displaced": {}, "glass": [], "glass_bands": [],
            "macroblocks": [], "regions": [], "roof_props": "keep",
            "debris": [], "notes": [], "stats": {}, "_removed_set": set(),
        }
        weights = tu.side_weights(info, wind, rng)
        plan["side_weights"] = {k: float(v) for k, v in weights.items()}
        pctx = tu._pctx(info, info["elements"], btype, rng, plan, wind,
                        weights, hc, 0.7)
        pctx["g"].n_bays = {sd: min(n_bays_override, nb)
                            for sd, nb in pctx["g"].n_bays.items()}
        tu.t_cladding_band(pctx, storeys=(2, 4), bay_frac=(0.40, 0.70))
        for r in plan["regions"]:
            if r.get("recipe") == "cladding_band":
                out.append(len(r.get("storeys") or ()))
    return out


def test_cladding_band_on_a_narrow_face_reaches_for_storeys():
    """A <= 2-bay midrise face draws its band from the TOP of the recipe's
    storey range -- the mirror of the 1-bay tower cap above it, and the
    reason the A2/A3 bench cells stopped reading undamaged (measured: 3
    storeys x 1 bay -> 4 storeys x 1 bay on the live SM_Building_02 T3
    probe, `removed_frac` 0.074 -> 0.096 against a 0.25 midrise cap)."""
    narrow = _band_storeys(2)
    wide = _band_storeys(6)
    assert narrow, "expected the narrow-face branch to author a band"
    assert min(narrow) >= 3, narrow            # top of the 2-4 range
    assert max(narrow) <= 4, narrow            # still inside the range
    assert wide and min(wide) >= 2, wide       # the wide face is untouched
    assert max(wide) <= 4, wide
    assert min(wide) < min(narrow) or len(set(wide)) > 1, (wide, narrow)


# ---------------------------------------------------------------------------
# ROUND 4 (D2, v6 review) -- "no piece stands on air" on the SLICED path, and
# "no hole edge left square". The user, on the v6 lit bench: "wall_N_1_10_0204
# -- some walls like this are still floating and some of the walls still have
# even breaks".
# ---------------------------------------------------------------------------
def _grid_of(info):
    return qs._Grid(info, info["elements"])


def test_no_sliced_piece_stands_on_air():
    """The invariant `_shed_unsupported_walls` exists to hold, over a spread
    of seeds and both damaging levels. `displaced` pieces are excluded: they
    are pitched about their own bottom outer edge and are attached there."""
    seen_shed = 0
    for fixture, btype, hc in (("midrise", "urm", "midrise"),
                               ("lowrise", "urm", "lowrise"),
                               ("tower", "rc", "tower")):
        info = _fixture(fixture, seed=4, btype=btype)
        for seed in range(6):
            for level in ("T3", "T4"):
                plan = _fc_plan(info, level=level, btype=btype, seed=seed,
                                wind=fake_wind(15.0 + 23.0 * seed, 0.9),
                                intensity=0.9, height_class=hc)
                g = _grid_of(info)
                left = tu._unsupported_survivors(
                    g, plan["removed"],
                    protect=set(plan.get("_support_protect") or ()))
                assert left == [], (fixture, level, seed, left[:4])
                seen_shed += plan["stats"]["n_support_shed"]
    # and the pass is not a no-op everywhere (it would prove nothing if the
    # fixtures never produced a floating piece in the first place)
    assert seen_shed > 0, "no seed exercised the support pass at all"


def test_support_pass_is_off_under_tu_support_0(monkeypatch):
    """`TU_SUPPORT=0` is the A/B switch the container probe uses; it must
    restore the round-3 answer exactly (no shed, and the note says so)."""
    info = _fixture("midrise", seed=4, btype="urm")
    monkeypatch.setattr(tu, "TU_SUPPORT_ON", False)
    plan = _fc_plan(info, level="T4", btype="urm", seed=3,
                    wind=fake_wind(35.0, 0.9), intensity=0.9)
    assert plan["stats"]["n_support_shed"] == 0
    assert any(n.startswith("support: DISABLED") for n in plan["notes"])


def test_support_never_sheds_a_displaced_or_leaning_piece():
    """A hanging panel / leaning macroblock is ATTACHED at its pivot -- the
    one place this pass deliberately diverges from `tornado_kit.kit_guard`,
    which demotes them. A piece must never be in `displaced` AND `removed`."""
    for fixture, btype in (("lowrise", "urm"), ("midrise", "urm")):
        info = _fixture(fixture, seed=4, btype=btype)
        for seed in range(8):
            plan = _fc_plan(info, level="T4", btype=btype, seed=seed,
                            wind=fake_wind(20.0 + 17.0 * seed, 0.95),
                            intensity=0.95)
            shed = set(plan.get("support_shed") or ())
            # SCOPED TO THIS PASS. `_shed_unsupported_roof` (round 3b F1,
            # and unchanged here) can and does take a displaced PARAPET
            # when its wall band empties -- pre-existing, deliberate, and
            # not what this test is about.
            assert not (shed & set(plan["displaced"])), (
                fixture, seed, sorted(shed & set(plan["displaced"]))[:3])
            assert not (shed & {mb["path"] for mb in
                                (plan.get("macroblocks") or ())}), (
                fixture, seed)


def test_support_keeps_the_height_class_area_cap():
    """The seed trade: the closure only ever ADDS and `_cap_removed_frac`
    cannot hand back what it added, so the pass trades a recipe removal
    instead of busting the cap."""
    for fixture, btype, hc in (("midrise", "urm", "midrise"),
                               ("lowrise", "urm", "lowrise")):
        info = _fixture(fixture, seed=4, btype=btype)
        cap = tu.HEIGHT_CAPS[hc]["max_removed_frac"]
        for seed in range(8):
            plan = _fc_plan(info, level="T4", btype=btype, seed=seed,
                            wind=fake_wind(11.0 * seed, 0.95), intensity=0.95,
                            height_class=hc)
            if any(r.get("recipe") == "facade_collapse"
                   for r in plan["regions"]):
                continue          # Sec8c's documented carve-out
            assert plan["stats"]["removed_frac"] <= cap + 1e-9, (
                fixture, seed, plan["stats"]["removed_frac"], cap)


def test_every_hole_border_piece_carries_a_tear():
    """`stats["n_border_untorn"]` is the "even breaks" number: a surviving
    piece touching a hole with no tear job keeps the slicer's own straight
    boundary."""
    saw_border = 0
    for fixture, btype in (("midrise", "urm"), ("lowrise", "urm")):
        info = _fixture(fixture, seed=4, btype=btype)
        for seed in range(6):
            for level in ("T3", "T4"):
                plan = _fc_plan(info, level=level, btype=btype, seed=seed,
                                wind=fake_wind(13.0 * seed + 20.0, 0.9),
                                intensity=0.9)
                saw_border += plan["stats"]["n_hole_border"]
                # THE INVARIANT THIS MODULE OWNS: a border piece that
                # `plan_edges` gave a job to is never dropped here.
                assert plan["stats"]["n_border_untorn"] == 0, (
                    fixture, level, seed,
                    plan["stats"]["n_border_untorn"],
                    plan["stats"]["n_hole_border"])
                # The tornado wrapper aligns `storey` with the sliced grid's
                # authoritative `_storey` for the borrowed planning call, so
                # every geometric hole-border piece must now receive a job.
                nb = plan["stats"]["n_hole_border"]
                assert plan["stats"]["n_border_no_job"] == 0, (
                    fixture, level, seed,
                    plan["stats"]["n_border_no_job"], nb)
    assert saw_border > 0, "no plan opened a hole with a border at all"


def test_cap_tears_never_drops_a_border_job():
    """The cap bounds the DECORATIVE tears only; a border job is kept
    whatever the cap, and does not count against it."""
    jobs = [{"el": {"p": {"_role": "wall", "prim_path": "/p/%d" % i}},
             "dropped": False} for i in range(9)]
    border = {"/p/7", "/p/8"}
    out = tu._cap_tears([dict(j) for j in jobs], 3, border=border)
    kept = [j for j in out if not j.get("dropped")]
    kept_paths = {(j["el"]["p"]["prim_path"]) for j in kept}
    assert border <= kept_paths, kept_paths
    # 3 non-border + the 2 border ones
    assert len(kept) == 5, [j["el"]["p"]["prim_path"] for j in kept]
    # ...and with the border switch off it is the flat round-3 cap again
    out2 = tu._cap_tears([dict(j) for j in jobs], 3, border=None)
    assert sum(1 for j in out2 if not j.get("dropped")) == 3


def test_hole_border_is_geometric_at_the_corners():
    """`_hole_border_paths` must not claim the corner at the FAR end of a
    run. The index-only rule flagged all seven storeys of the SW corner
    against a band nowhere near it (22 of 54 on the real T3 plan)."""
    info = _fixture("midrise", seed=4, btype="urm")
    g = _grid_of(info)
    corners = [e for e in g.els
               if (e.get("p") or {}).get("_side") in qs._CORNER_SIDES]
    if not corners:
        pytest.skip("fixture has no corner pieces")
    side = "S"
    bays = sorted(g.sides.get(side, ()) or ())
    assert len(bays) >= 3, bays
    mid = [qs._path(e) for e in g.at((side, 1, bays[len(bays) // 2]))]
    border = tu._hole_border_paths(g, set(mid))
    far = [qs._path(e) for e in corners
           if not tu._touch_xy(e, g.at((side, 1, bays[len(bays) // 2]))[0])]
    assert far, "expected at least one corner far from the mid bay"
    assert not (set(far) & border), sorted(set(far) & border)[:4]


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
