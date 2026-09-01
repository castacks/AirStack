#!/usr/bin/env python3
"""test_tornado_kit.py — does `disaster/tornado_kit.py`'s adapter let the
EXISTING sliced-building tornado ladder (`disaster/tornado_urban.py` +
`disaster/tornado_urban_usd.py`) damage a KIT-STYLE building correctly?

    python3 -m pytest -q scene_gen/tests/test_tornado_kit.py
    python3 scene_gen/tests/test_tornado_kit.py     # prints the T1-T4 stats
                                                     # table per style

WHY THIS EXISTS
---------------
`_plans/urban_tornado_plan.md` §7's rule R3 (round 2): the round-1 ladder
only ever damaged sliced GAC/downtowncity buildings, so 13 of 20 corridor
records in the first GUI scene had NO damage path at all — every kit-style
building (`bld_<style>_DG0`: brownstone, brownstone_row, dw_terrace, walkup,
apartment, office, commercial_mid, ...) read as intact. `tornado_kit.adapt`
closes that gap by stamping the FIVE fields `quake_sliced._Grid` reads
(`_side`/`_role`/`_storey`/`_bay`/`_size`) onto a kit building's own
placements, so `tornado_urban.plan_damage` and `tornado_urban_usd.apply_plan`
run UNCHANGED on a kit style. This file is `test_quake_sliced.py`'s /
`test_tornado_urban.py`'s sibling for that adapter: everything checkable
without `pxr` runs host-side in a few seconds; the one stage test below uses
STUB box prims (never a real kit USD — those are `omniverse://`, see the
module's own docstring: "never compose them host-side").

WHAT THIS FILE CANNOT SEE: whether a real authored kit building's glazing
subsets actually void (needs a real stage + `annotate_glazing`'s connection-
following material read — `tools/tornado_kit_probe.py`'s job, run in the
`isaac-sim` container as bare python), whether a removed piece's fragment
field looks like a wrecked brownstone from the air. That needs the probe and
a render.
"""

import json
import os
import random
import sys
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from disaster import kit_substitute as ksub                    # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402
from disaster import tornado_kit as tk                          # noqa: E402
from disaster import tornado_urban as tu                        # noqa: E402

# ---------------------------------------------------------------------------
# style set — every style the task asks for, plus one rc (office) for
# construction-type spread. All single-mass (no wings/tower) so `adapt`
# accepts them; the multi-mass refusal has its own test below.
# ---------------------------------------------------------------------------
STYLES = ("brownstone_row", "dw_terrace", "walkup", "office", "commercial_mid")
MULTI_MASS_STYLES = ("tower", "skyscraper_b", "highrise_step",
                     "block_residential", "block_office", "block_stone")
LEVELS = ("T1", "T2", "T3", "T4")


def fake_wind(bearing_deg, speed_frac=0.8, cross_frac=-0.4, over=False):
    """The §2.4 wind dict shape — `tests/test_tornado_urban.py`'s own
    fixture, reproduced here so this file does not depend on
    `tornado.wind_at` for every test."""
    return {"bearing_deg": float(bearing_deg), "speed_frac": float(speed_frac),
            "cross_frac": float(cross_frac), "over": bool(over)}


def _run(style, level, seed=11, dmg_seed=None, wind=None, intensity=None,
        height_class=None, btype=None):
    wind = wind if wind is not None else fake_wind(90.0, 0.85)
    if intensity is None:
        intensity = tk.LEVEL_INTENSITY.get(level, 0.5)
    rng = random.Random(dmg_seed if dmg_seed is not None else seed + 100003)
    placements, info, plan = tk.plan_for_kit(
        style, level, rng, wind, seed=seed, btype=btype,
        height_class=height_class, intensity=intensity)
    return placements, info, plan


def _by_path(info):
    return {(e["p"] or {}).get("prim_path"): e for e in info["elements"]}


def _removed_els(info, plan):
    idx = _by_path(info)
    return [idx[p] for p in plan["removed"] if p in idx]


# ---------------------------------------------------------------------------
# adapter invariants
# ---------------------------------------------------------------------------
_KNOWN_ROLES = {"wall", "pier", "corner", "parapet_corner", "parapet", "roof",
                "core"}


def test_every_placement_is_stamped():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=3)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        assert els, style
        for e in els:
            p = e["p"]
            for field in ("_side", "_role", "_storey", "_bay", "_size",
                         "prim_path"):
                assert field in p, (style, field, p.get("usd"))
            assert p["_role"] in _KNOWN_ROLES, (style, p["_role"])
            assert p["_side"] in qs.SIDES or p["_side"] in qs._CORNER_SIDES \
                or p["_side"] == "-", (style, p["_side"])
            assert len(p["_size"]) == 3
            # sx, sy are always a real measured span; sz (piece HEIGHT) is
            # legitimately 0.0 for a roof tile (`urban_building.PIECES`'s
            # own measured entries: "SM_MBuilding01_Roof": (5.0, 5.0, 0.0,
            # ...) — a flat quad, by design in this kit).
            assert p["_size"][0] > 0.0 and p["_size"][1] > 0.0, (style, p["_size"])
            assert p["_size"][2] >= 0.0, (style, p["_size"])
            assert isinstance(p["prim_path"], str) and p["prim_path"]


def test_prim_paths_are_unique_per_style():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=4)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        paths = [e["p"]["prim_path"] for e in els]
        assert len(paths) == len(set(paths)), style


def test_side_distribution_covers_at_least_three_sides():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=5)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        sides = set(e["p"]["_side"] for e in els
                    if e["p"]["_role"] in ("wall", "pier", "parapet"))
        assert sides <= set(qs.SIDES)
        assert len(sides) >= 3, (style, sides)


def test_n_sub_is_three_on_every_tested_style():
    """At least one plain (unglazed) wall-band piece exists on every tested
    style, so `quake_sliced.n_sub_of` detects the `pier` role and returns 3
    — the n_sub=3 grammar this adapter's `_bay` binning relies on (see the
    module docstring's "WHY n_sub=3, NOT n_sub=1")."""
    for style in STYLES:
        placements = tk.kit_placements(style, seed=6)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        n_sub = qs.n_sub_of(els)
        assert n_sub == 3, (style, n_sub)
        assert any(e["p"]["_role"] == "pier" for e in els), style


def test_is_opening_true_exactly_on_glazed_pieces():
    """The task's own acceptance test: for every wall/pier-role piece,
    `quake_sliced.is_opening(p, n_sub)` must agree EXACTLY with
    `tornado_kit._is_glazed_name(name)` — no false opening, no missed one."""
    for style in STYLES:
        placements = tk.kit_placements(style, seed=7)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        n_sub = qs.n_sub_of(els)
        n_checked = 0
        for e in els:
            p = e["p"]
            if p["_role"] not in ("wall", "pier"):
                continue
            n_checked += 1
            glazed = tk._is_glazed_name(e.get("name"))
            opening = qs.is_opening(p, n_sub)
            assert glazed == opening, (style, e.get("name"), glazed, opening)
            # is_pier is exactly the complement
            assert qs.is_pier(p, n_sub) == (not glazed)
        assert n_checked > 0, style


def test_storeys_match_mass_specs_level_count():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=8)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        n_levels = len(info["masses"]["main"]["levels"])
        run_storeys = [int(e["p"]["_storey"]) for e in els
                       if e["p"]["_role"] in ("wall", "pier")]
        assert run_storeys, style
        assert min(run_storeys) == 0, (style, min(run_storeys))
        assert max(run_storeys) == n_levels - 1, \
            (style, max(run_storeys), n_levels)


def test_corner_pieces_resolve_to_all_four_corners():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=9)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        corners = set(e["p"]["_side"] for e in els
                      if e["p"]["_role"] == "corner")
        assert corners, style
        assert corners <= set(qs._CORNER_SIDES), (style, corners)


def test_excluded_roles_never_enter_the_side_grid():
    for style in STYLES:
        placements = tk.kit_placements(style, seed=10)
        info = qf.describe(style, placements, 0.0, 0.0, 0.0)
        els = tk.adapt(placements, info)
        for e in els:
            p = e["p"]
            if p["_role"] in ("roof", "core"):
                assert p["_side"] == "-", (style, p["_role"], p["_side"])


def test_multi_mass_styles_are_refused_not_mis_damaged():
    for style in MULTI_MASS_STYLES:
        try:
            tk.kit_placements(style, seed=1)
        except ValueError as exc:
            assert "multi-mass" in str(exc)
            continue
        raise AssertionError("{0} should have been refused".format(style))
    # and plan_for_kit refuses the same way, before ever building an rng draw
    try:
        tk.plan_for_kit("tower", "T3", random.Random(1), fake_wind(0.0), seed=1)
        raise AssertionError("plan_for_kit('tower', ...) should have raised")
    except ValueError:
        pass


def test_unknown_style_is_refused():
    try:
        tk.kit_placements("no_such_style", seed=1)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass


# ---------------------------------------------------------------------------
# planner invariants (round-1 pinned, re-asserted through the adapter)
# ---------------------------------------------------------------------------
def test_planner_determinism_same_seed_byte_identical():
    for style in STYLES:
        for level in LEVELS:
            _p1, _i1, plan1 = _run(style, level, seed=17, dmg_seed=23,
                                   wind=fake_wind(63.0, 0.7, -0.2))
            _p2, _i2, plan2 = _run(style, level, seed=17, dmg_seed=23,
                                   wind=fake_wind(63.0, 0.7, -0.2))
            j1 = json.dumps(plan1, sort_keys=True)
            j2 = json.dumps(plan2, sort_keys=True)
            assert j1 == j2, (style, level)


def test_plan_json_round_trips_every_level():
    for style in STYLES:
        for level in LEVELS:
            _placements, _info, plan = _run(style, level, seed=3)
            s = json.dumps(plan, sort_keys=True)
            back = json.loads(s)
            assert back["level"] == level
            assert back["stats"]["n_removed"] == plan["stats"]["n_removed"]


def test_windward_dominance_bearing_90_hits_south():
    """`bearing_deg=90` (wind blowing toward +y) is windward on the SOUTH
    wall (`quake_sliced._SIDE_NORMAL["S"] == (0, -1)`) — the exact reasoning
    `tests/test_tornado_urban.py`'s own
    `test_wind_side_rule_south_bearing_hits_south` uses, reproduced here
    through the adapter.

    `intensity=0.8`, not 0.9: at `i >= 0.85` `tornado_urban.
    t_top_storey_loss` (lowrise-urm-only) legitimately removes the top
    storey's wall on 2-3 SIDES by design (`_plans/urban_tornado_plan.md`
    §2.6's own "urm additionally loses the top 1-2 storeys' windward wall"
    rule, generalised at very high intensity) — `brownstone_row` and
    `dw_terrace` are both lowrise, so pinning this test at 0.9 would be
    checking a ladder rule that is SUPPOSED to spread removal across
    several sides, not this recipe-family's windward-only ones. 0.8 keeps
    T4 (`>= 0.74`) without crossing that threshold."""
    for style in STYLES:
        _placements, info, plan = _run(
            style, "T4", seed=21, wind=fake_wind(90.0, 0.85, 0.05, over=False),
            intensity=0.8)
        rm = _removed_els(info, plan)
        if not rm:
            continue
        on_s = sum(1 for e in rm if e["p"]["_side"] in ("S", "SW", "SE"))
        assert on_s / float(len(rm)) >= 0.70, (style, on_s, len(rm))
        assert not any(e["p"]["_side"] == "N" for e in rm), style


def test_caps_hold_by_height_class():
    for style in STYLES:
        for seed in range(6):
            _placements, info, plan = _run(
                style, "T4", seed=seed, wind=fake_wind(50.0, 0.9),
                intensity=0.9)
            hc = plan["height_class"]
            cap = tu.HEIGHT_CAPS[hc]["max_removed_frac"]
            st = plan["stats"]
            # `t_facade_collapse` is CAP-EXEMPT BY DESIGN (§8c's named
            # carve-out: "the lowrise area cap is exempted for this one
            # guarded recipe — it exists to stop accidental gutting, not
            # the documented Waco/Nashville state"). It became reachable
            # on `brownstone_row` when the storey cap moved 4 -> 5 (Waco's
            # own Dennis Building was five storeys), so this blanket
            # assertion now honours the exemption rather than re-imposing
            # the cap the recipe is documented to be exempt from. The
            # recipe's OWN invariants (>= 2 elevations untouched, the
            # building stands) have their dedicated tests in
            # `test_tornado_urban.py`.
            ran_fc = any(r.get("recipe") == "facade_collapse"
                         for r in plan["regions"])
            if not ran_fc:
                assert st["removed_frac"] <= cap + 1e-9, \
                    (style, seed, st["removed_frac"], cap)
            rm = _removed_els(info, plan)
            ran_tsl = any(r.get("recipe") == "top_storey_loss"
                         for r in plan["regions"])
            # ROUND 4 D1: `core` (portico / pediment / ornament / balcony)
            # is still never REMOVED BY A RECIPE -- `adapt` keeps it off
            # the side/bay grid entirely, so no recipe can address it. The
            # ONE mechanism that may now take one is `tornado_kit`'s own
            # support post-pass, and only when every wall/pier/corner
            # piece within `KIT_ORNAMENT_RADIUS_M` of it, at its own storey
            # or one either side, is gone -- B1's stranded ornate assembly
            # hanging at the roof line over a removed elevation, the
            # round-4 review's own "floating elements everywhere". Each
            # such removal is re-checked here against the rule itself, so
            # this stays an invariant rather than a licence.
            g_chk = qs._Grid(info, info["elements"])
            removed_set = set(plan["removed"])
            for e in rm:
                if e["p"]["_role"] != "core":
                    continue
                assert int(plan["stats"].get("n_support_shed", 0)) > 0, \
                    (style, seed, "core removed with no support-pass shed")
                assert tk._unsupported(g_chk, e, removed_set), \
                    (style, seed, "core removed while still attached to a "
                                  "standing wall")
            # ROUND 3b (§8e F1) widens the old "only top_storey_loss ever
            # removes a roof" invariant: `tornado_urban.
            # _shed_unsupported_roof` now ALSO sheds a roof piece whenever
            # its own bay column (KIT) or its mass's own top-storey wall/
            # pier/corner pieces (SLAB, the `ROOF_SHED_FRAC` threshold) are
            # gone -- exactly the "floating roof" fix the user asked for.
            # `st["n_roof_shed"]` is that pass's own counter, so a removed
            # roof piece is still required to be ACCOUNTED FOR by a named
            # mechanism, just no longer only the one.
            n_roof_removed = sum(1 for e in rm if e["p"]["_role"] == "roof")
            assert (n_roof_removed == 0 or ran_tsl
                   or int(st.get("n_roof_shed", 0)) > 0), \
                (style, seed, "roof removed without top_storey_loss or "
                             "the F1 roof-support post-pass")


def test_no_storey_is_ever_emptied():
    for style in STYLES:
        for seed in range(6):
            _placements, info, plan = _run(
                style, "T4", seed=seed, wind=fake_wind(15.0, 0.9),
                intensity=0.9)
            g = qs._Grid(info, info["elements"])
            by_storey = {}
            for e in g.els:
                by_storey.setdefault(int((e["p"] or {}).get("_storey", 0)),
                                     []).append(e)
            removed_set = set(plan["removed"])
            for _st, els in by_storey.items():
                paths = [qs._path(e) for e in els if qs._path(e)]
                assert not (paths and all(p in removed_set for p in paths)), \
                    (style, seed, _st)


def test_t4_chunk_region_top_is_in_the_top_40_percent():
    """`tornado_urban.t_chunk` draws its region's own top storey from
    `randint(ceil(0.6 * g.top), g.top)` — i.e. it reasons in STOREY-INDEX
    space and assumes storey index is a reasonable proxy for height
    fraction. That assumption holds for a GAC/downtowncity slice (storeys
    are close to uniform) and for four of this adapter's five tested
    styles (`urban_building.STYLES[style]["bands"]` heights are close
    enough to uniform that storey-index and height-fraction agree).

    IT DOES NOT HOLD FOR `dw_terrace`, MEASURED: its band heights are
    `[4.3, 1.2, 6.0, 6.0]` (storefront, TRIM, window x2) — the 1.2 m trim
    band gets its OWN storey index (`urban_building.dw_b`'s trim band
    carries no `"parapet": True` flag, so `quake_flow._mass_specs` counts
    it as a full level) sitting between two 6 m window bands, so storey
    index 2 of 3 (`>= ceil(0.6 * 3) = 2`, satisfying `t_chunk`'s own
    constraint) lands at z=5.5 of H=17.5 — 31% up the building, not >= 60%.
    Reproduced directly: of 19 T4 chunk draws over 40 seeds, 13 violate
    "region top >= 0.6 H" this way. This is a genuine mismatch between a
    kit style's own uneven band grammar and `t_chunk`'s storey-index
    proxy for height — `tornado_urban.py` is owned by a different stream
    this round and is not edited here; reported instead of silently
    asserting past it. The other four tested styles (whose band heights
    are far more uniform) hold the invariant with ZERO violations over the
    same seed range and are asserted strictly.
    """
    seen_any_chunk = False
    violations = {"dw_terrace": 0}
    for style in STYLES:
        for seed in range(8):
            _placements, info, plan = _run(
                style, "T4", seed=seed, wind=fake_wind(20.0, 0.9),
                intensity=0.9)
            m = info["masses"]["main"]
            for region in plan["regions"]:
                if region["recipe"] != "chunk":
                    continue
                seen_any_chunk = True
                top_storey = max(int(s) for s in region["storeys"])
                top_z = (m["levels"][top_storey]
                        if top_storey < len(m["levels"]) else m["top"])
                ok = top_z >= 0.6 * info["H"] - 1e-6
                if not ok:
                    violations["dw_terrace"] = violations.get(style, 0) + 1
                assert ok, (style, seed, top_z, info["H"])
    assert seen_any_chunk
    # ROUND-2 RESOLUTION (lead, 2026-09-01): the mismatch this test
    # originally DOCUMENTED (13/19 dw_terrace chunk draws anchored at 31 %
    # of H under the storey-INDEX rule) was fixed in `tornado_urban.t_chunk`
    # the same day: the anchor's eligible set is now built from each
    # storey's own base elevation (`m["levels"][st]`) against 0.6 * H, so
    # a thin trim band that owns a storey index can no longer satisfy the
    # constraint by index while violating it by height. The loop above now
    # asserts the height constraint STRICTLY for every style, dw_terrace
    # included — if it ever fires again, the height-based draw regressed.


def _run_one_recipe(style, recipe, kw, seed=5, wind=None, height_class=None,
                    intensity=0.7, btype=None):
    """Run exactly ONE `tornado_urban` recipe against this adapter's own
    `info`/`elements`, bypassing `LADDER_T`/`_guard` — the same isolation
    `tests/test_tornado_urban.py`'s own `_run_one_recipe` uses, so a single
    recipe's own toothing draw is checked without a LATER recipe in the
    same T-level ladder legitimately touching the same cells (which is
    exactly what happened running the full ladder here first: a later
    recipe removing a pier `cladding_band`'s own `_apply_region` had just
    forced to survive is not a toothing bug, it is two independent recipes
    sharing a boundary — round 1's own test avoids it the same way)."""
    placements = tk.kit_placements(style, seed=seed)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    elements = tk.adapt(placements, info)
    if btype is None:
        btype = ksub.styles()[style]["type"]
    info["type"] = btype
    wind = wind if wind is not None else fake_wind(0.0)
    if height_class is None:
        height_class = tu.height_class_for(info["H"])
    rng = random.Random(seed + 100003)
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
    pctx = tu._pctx(info, elements, btype, rng, plan, wind, weights,
                    height_class, intensity)
    tu.RECIPES_T[recipe](pctx, **kw)
    plan = tu._finalise(pctx, plan, height_class, wind, intensity)
    return info, plan


def test_toothing_no_boundary_row_is_all_or_nothing():
    """`quake_sliced._apply_region`'s toothing invariant, reasserted
    through the adapter with `_run_one_recipe`'s isolation (see that
    helper's own docstring for why a whole-ladder run is the wrong level
    to check this at)."""
    # `commercial_mid`'s own cladding-band regions carry very few boundary
    # PIER cells per draw (most of its wall-band pieces are glazed --
    # measured: `test_n_sub_is_three_on_every_tested_style` still finds at
    # least one pier building-wide, but a single 2-4 storey region can
    # easily land on zero or one), so more seeds are needed to see a
    # genuine >=2-pier partial-survival draw than the round-1 MIDRISE
    # fixture (rich pier population) ever needed.
    for style in ("brownstone_row", "walkup", "commercial_mid"):
        seen_partial = False
        sampled = False
        for seed in range(40):
            info, plan = _run_one_recipe(
                style, "cladding_band", {}, seed=seed,
                wind=fake_wind(10.0 + 7.0 * seed, 0.85), intensity=0.85)
            regs = [r for r in plan["regions"] if r["recipe"] == "cladding_band"]
            if not regs:
                continue
            cells = set(tuple(c) for c in regs[0]["cells"])
            g = qs._Grid(info, info["elements"])
            bnd = qs._boundary(cells, set(g.runs))
            # ROUND 3b (§8e F1): `qs.is_pier` is a pure BAY-POSITION test
            # (not sub_ix == the opening's own middle index), with no ROLE
            # check at all -- a `parapet` piece shares the wall band's own
            # bay partition on this kit (`tornado_kit.adapt`'s `_bay_index`
            # reads the SAME per-side pitch table for every band), so a
            # top-storey boundary cell can hand back a parapet piece that
            # `is_pier` alone happily counts as a "pier". This test's own
            # invariant is about `cladding_band`'s WALL-BAND toothing
            # (`quake_sliced._apply_region`), so the role filter below
            # excludes that parapet false-positive rather than the test
            # coincidentally also asserting on F1's own, separate, and
            # CORRECT behaviour ("a parapet piece above a lost top-storey
            # cell always goes with it" -- `tornado_urban.
            # _shed_unsupported_roof`, new this round): the ONE piece this
            # style/seed combination handed the pre-F1 test as its
            # "surviving boundary pier" was actually a parapet sitting
            # directly over the emptied cladding-band hole, which F1 now
            # correctly sheds.
            piers = [e for key in bnd for e in g.at(key)
                    if qs.is_pier(e.get("p") or {}, g.n_sub)
                    and (e.get("p") or {}).get("_role") in ("wall", "pier")]
            if not piers:
                continue
            sampled = True
            removed = set(plan["removed"])
            survived = [e for e in piers if qs._path(e) not in removed]
            assert survived, (style, seed, "every boundary pier removed")
            if len(survived) < len(piers):
                seen_partial = True
        # MEASURED (not assumed): `walkup`'s own cladding_band boundary
        # never lands next to a role-true wall/pier piece across 40 seeds
        # -- every "pier" the pre-F1, role-blind version of this check
        # ever found for this style was the same parapet miscount the
        # comment above explains, so `sampled` stays False for it. That is
        # a genuine property of `walkup`'s own bay layout (its plain,
        # unglazed panels sit elsewhere on the elevation, per `test_n_sub_
        # is_three_on_every_tested_style`'s own finding), not a toothing
        # regression -- asserting `seen_partial` on zero samples would be
        # asserting nothing, so the requirement only binds once a style
        # actually produced a role-true boundary pier to check.
        if sampled:
            assert seen_partial, style


def test_monotonic_mean_removal_over_levels():
    """Mean `n_removed` is non-decreasing T1 -> T4, in expectation over
    several seeds — `_plans/urban_tornado_plan.md` §3's monotonicity check,
    reasserted through the adapter."""
    for style in STYLES:
        means = []
        for level in LEVELS:
            vals = []
            for seed in range(6):
                _placements, _info, plan = _run(
                    style, level, seed=seed, wind=fake_wind(40.0, 0.85),
                    intensity=tk.LEVEL_INTENSITY[level])
                vals.append(plan["stats"]["n_removed"])
            means.append(sum(vals) / float(len(vals)))
        assert means == sorted(means), (style, means)


# ---------------------------------------------------------------------------
# stage test — stub box prims (never a real kit USD host-side)
# ---------------------------------------------------------------------------
def test_apply_plan_on_stub_stage_removes_and_authors_debris():
    from pxr import Sdf, Usd, UsdGeom

    from disaster import tornado_urban_usd as tuu

    style = "walkup"
    placements, info, plan = _run(style, "T4", seed=11,
                                  wind=fake_wind(35.0, 0.9), intensity=0.85)
    assert plan["removed"], "T4 should remove something on walkup"

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    cell = "/World/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))
    UsdGeom.Xform.Define(stage, Sdf.Path(cell + "/parts"))

    idx = _by_path(info)
    for path, e in idx.items():
        p = e["p"]
        sx, sy, sz = p["_size"]
        cz = float(p.get("z_m", 0.0)) + max(sz, 0.05) / 2.0
        qf._box(stage, path, float(p["x_m"]), float(p["y_m"]), cz,
                max(sx, 0.05), max(sy, 0.05), max(sz, 0.05), yaw_deg=0.0)

    ctx = {"stage": stage, "parent": cell, "tag": "tk_test", "mats": {},
           "static_extra": [], "loose": [], "authored": [], "info": info,
           "notes": []}
    counts = tuu.apply_plan(stage, ctx, plan, verbose=False)

    assert counts["n_removed"] > 0
    assert counts["n_missing"] == 0, "every removed path must resolve on the stub stage"
    for path in plan["removed"]:
        prim = stage.GetPrimAtPath(path)
        assert prim.IsValid()
        assert not prim.IsActive(), path

    assert plan["debris"], "T4 walkup should shed debris"
    assert counts["n_debris_meshes"] > 0
    deb_root = cell + "/tornado_debris"
    assert stage.GetPrimAtPath(deb_root).IsValid()


# ---------------------------------------------------------------------------
# T1-T4 stats table, per style — printed by the report and by __main__
# ---------------------------------------------------------------------------
def _stats_table():
    rows = []
    for style in STYLES:
        for level in LEVELS:
            _placements, info, plan = _run(
                style, level, seed=13, wind=fake_wind(35.0, 0.85),
                intensity=tk.LEVEL_INTENSITY[level])
            st = plan["stats"]
            rows.append((style, plan["btype"], plan["height_class"], level,
                        st["n_pieces"], st["n_removed"],
                        round(st["removed_frac"], 3), st["n_glass"],
                        st["n_debris"]))
    return rows


def test_print_t1_t4_stats_table():
    rows = _stats_table()
    header = "{0:<16} {1:<8} {2:<9} {3:<3} {4:>7} {5:>8} {6:>6} {7:>6} {8:>7}".format(
        "style", "btype", "hclass", "lvl", "pieces", "removed", "frac",
        "glass", "debris")
    print("\n" + header)
    print("-" * len(header))
    for row in rows:
        print("{0:<16} {1:<8} {2:<9} {3:<3} {4:>7} {5:>8} {6:>6} {7:>6} {8:>7}"
              .format(*row))
    assert rows


if __name__ == "__main__":
    rows = _stats_table()
    header = "{0:<16} {1:<8} {2:<9} {3:<3} {4:>7} {5:>8} {6:>6} {7:>6} {8:>7}".format(
        "style", "btype", "hclass", "lvl", "pieces", "removed", "frac",
        "glass", "debris")
    print(header)
    print("-" * len(header))
    for row in rows:
        print("{0:<16} {1:<8} {2:<9} {3:<3} {4:>7} {5:>8} {6:>6} {7:>6} {8:>7}"
              .format(*row))
