#!/usr/bin/env python3
"""test_urban_fire_spread.py — the CITY-SCALE claims of the spread solver.

    python3 scene_gen/tests/test_urban_fire_spread.py
    pytest -q scene_gen/tests/test_urban_fire_spread.py

WHY THIS EXISTS
---------------
`disaster/urban_fire_spread.py` decides which buildings on a 500 m plate are
on fire, when each caught, and how badly each is burning right now. Every one
of those is a claim about arithmetic, and every one of them is asserted here
host-side — no `pxr`, no Kit, no Isaac Sim, well under a second — because the
city bake is an hour of GPU time per run and a spread plan that is wrong is
wrong in every single bake it drives.

The four properties that actually matter, and are hard to see by eye in a
built scene:

  * A BLOCKED BUILDING IS OUT OF THE GRAPH. The no-fire rule for tower and
    highrise districts is expressed by deleting nodes, so a blocked building
    must never ignite AND must never appear as somebody else's `via` — a
    tower that is not on fire but is still the stepping stone the fire used
    to cross a block is a silent lie about the plan.
  * THE BURNT SET IS ONE CONNECTED FIRE. `cap_to_prefix` trims the plan to N
    buildings by keeping the N earliest ignitions, and the whole argument
    that this stays contiguous is that a Dijkstra parent ignites strictly
    before its child. Asserted, not assumed.
  * EVERY LEVEL THE SOLVER EMITS IS A LEVEL `urban_fire` CAN BUILD. The two
    modules keep separate ladders; a drift means a bake driver asking for a
    recipe list that does not exist.
  * THE GRADIENT IS A WAVE. Origin burnt out, a ring of F4 behind the front,
    F2/F3 at the front, F1 on the just-caught edge, F0 beyond. That is the
    deliverable, and it falls out of the clock rather than being tuned — so
    it is checkable.

WHAT THIS TEST FOUND ABOUT THE PLAN (`_plans/urban_fire_city_plan.md` §2.5)
--------------------------------------------------------------------------
The plan's illustrative epoch — "elapsed_s at 0.55 of a 3-hour duration" —
CANNOT produce the gradient it describes, and the arithmetic is not close:
0.55 x 3 h = 99 min, while a building does not leave F4 until `T_OUT`
(140 min) and cannot reach F5/F6 until `T_COLD` (200 min). At that epoch the
origin is F4 and the ladder tops out one rung below where §2.5 says it does.
Both epochs are exercised below (`test_gradient_wave_60_line`): the 99 min
case for the monotone shape it does have, and 0.55 x 6.6 h = 217.8 min for
the full F1 -> F5 wave.
"""

import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import urban_fire_spread as ufs        # noqa: E402

# `urban_fire` is host-side importable (nothing under `pxr` at module scope),
# but it pulls in numpy through `soot_plume`, so the ladder cross-checks are
# skipped rather than failed if that is not available on this machine.
uf = pytest.importorskip("disaster.urban_fire",
                         reason="urban_fire (numpy) not importable host-side")

MIN = 60.0


# ---------------------------------------------------------------------------
# fixtures: the two geometries every test below uses
# ---------------------------------------------------------------------------
def _grid(rows=3, cols=5, size=20.0, gap=8.0):
    """A `rows` x `cols` block of `size` m square buildings `gap` m apart.

    8 m gaps are inside `RAD_REACH_M` (13 m) on the square AND on the
    diagonal (hypot(8, 8) = 11.3 m), so the grid is radiation-connected in
    every direction — which is what makes it a fair test of a firebreak: if
    the far side goes dark, it is because the block was removed and not
    because the grid was never connected there.
    """
    step = size + gap
    return [{"x": c * step, "y": r * step, "W": size, "D": size, "yaw": 0.0,
             "H": 18.0, "style": "r{0}c{1}".format(r, c), "row": r, "col": c}
            for r in range(rows) for c in range(cols)]


def _grid_index(bs):
    return {(b["row"], b["col"]): k for k, b in enumerate(bs)}


def _line(n=60, w=20.0, d=15.0, gap=0.5):
    """A terrace of `n` buildings in a row, `gap` m apart — a 1-D chain."""
    return [{"x": i * (w + gap), "y": 0.0, "W": w, "D": d, "yaw": 0.0,
             "H": 18.0, "style": "b{0}".format(i)} for i in range(n)]


def _lit(plan):
    return [p for p in plan if p["t_ignite"] is not None]


# ---------------------------------------------------------------------------
# 1. blocked buildings leave the graph
# ---------------------------------------------------------------------------
def test_blocked_never_ignites_and_is_never_a_via():
    bs = _grid()
    idx = _grid_index(bs)
    blocked = frozenset(idx[(1, c)] for c in range(5))     # the middle row
    plan = ufs.solve(bs, idx[(0, 0)], 400 * MIN, wind_dir=0.0, wind_mps=6.0,
                     rng=random.Random(11), blocked=blocked)
    for i in sorted(blocked):
        assert plan[i]["t_ignite"] is None, plan[i]
        assert plan[i]["age"] is None
        assert plan[i]["level"] == "F0"
        assert plan[i]["via"] is None and plan[i]["how"] is None
    assert not [p for p in plan if p["via"] in blocked], \
        "a blocked building was used as a stepping stone"
    # and it is out of the EDGE SET, not just out of the answer
    assert not [e for e in ufs.edges(bs, 0.0, 6.0, None, blocked=blocked)
                if e[0] in blocked or e[1] in blocked]


def test_fire_routes_around_a_blocked_row():
    """A firebreak delays the far side; it does not always sever it.

    `RAD_REACH_M` is 13 m and the blocked row is 20 m deep, so radiation
    cannot cross it at all — with the wind too weak to throw brands
    (`wind_mps < 1.0` switches spotting off in `edges`) the far row NEVER
    catches and the block is a true firebreak.

    Turn the wind up and it is crossed anyway, by the one mechanism that is
    supposed to cross it: a BRAND, lofted off the origin and landing two rows
    downwind. That is not a bug in the block, it is the model's whole point
    about spotting being what makes a conflagration jump a street — and the
    far side still ignites ~50 min LATER than with no block at all.
    """
    bs = _grid()
    idx = _grid_index(bs)
    blocked = frozenset(idx[(1, c)] for c in range(5))
    origin, far = idx[(0, 0)], idx[(2, 0)]

    def _t(blk, wind):
        # rng=None: midpoint delays and no `SPOT_P` coin, so the two runs
        # differ ONLY by the block and the comparison below means something
        return ufs.solve(bs, origin, 400 * MIN, wind_dir=0.0, wind_mps=wind,
                         rng=None, blocked=blk)

    open_plan = _t(frozenset(), 6.0)
    blk_plan = _t(blocked, 6.0)
    assert open_plan[far]["t_ignite"] is not None
    assert blk_plan[far]["t_ignite"] is not None, \
        "with wind, a brand should still cross the firebreak"
    assert blk_plan[far]["t_ignite"] > open_plan[far]["t_ignite"] + 30 * MIN, \
        (blk_plan[far]["t_ignite"], open_plan[far]["t_ignite"])
    # the crossing is a brand, and it starts from an UNBLOCKED building
    chain, k = [], far
    while k is not None:
        chain.append(k)
        k = blk_plan[k]["via"]
    assert chain[-1] == origin
    assert not (set(chain) & blocked)
    assert "spot" in [blk_plan[i]["how"] for i in chain]

    # no wind -> nothing crosses; only the origin's own row burns
    calm = _t(blocked, 0.5)
    assert calm[far]["t_ignite"] is None
    assert all(bs[p["i"]]["row"] == 0 for p in _lit(calm))


def test_origin_frac_cap_clamps_spot_but_not_the_others():
    """`ORIGIN_FRAC_CAP` -- no mechanism may start a fire above half a mass.

    Only `spot` (raw 0.88) is above the cap; `origin`/`attached`/`radiation`
    (0.15/0.22/0.45) are already under it and must come through unclamped,
    so the relative ORDER between mechanisms survives -- a spot fire still
    starts higher than everything else, it just cannot start above
    `ORIGIN_FRAC_CAP` of the mass.

    User-facing regression this guards (2026-08-31): every top-heavy
    record in the 39-record `fire_city_500m` manifest (origin far up a
    20-30 storey GAC tower) was `how == "spot"` -- `plan_fire`'s F3/F4/F5
    bands are windowed by, or run to the top from, the origin, so an
    origin near 0.88 on a tall mass leaves the whole lower building
    untouched and pins every flame/soot/smoke event to a handful of
    storeys under the roof ("fire still seems to be on the higher floors
    rather than all over").
    """
    bs = _grid()
    seen_how = set()
    for seed in range(80):
        plan = ufs.solve(bs, 0, 400 * MIN, wind_dir=0.0, wind_mps=6.0,
                         rng=random.Random(seed))
        for p in _lit(plan):
            seen_how.add(p["how"])
            assert p["origin_frac"] <= ufs.ORIGIN_FRAC_CAP + 1e-9, p
            if p["how"] == "spot":
                assert p["origin_frac"] == pytest.approx(
                    ufs.ORIGIN_FRAC_CAP)
            elif p["how"] == "radiation":
                assert p["origin_frac"] == pytest.approx(0.45)
            elif p["how"] == "attached":
                assert p["origin_frac"] == pytest.approx(0.22)
            elif p["how"] == "origin":
                assert p["origin_frac"] == pytest.approx(0.15)
    assert "spot" in seen_how, "this grid/wind never drew a spot ignition"
    # the cap must still sit above every other mechanism's frac, or a spot
    # fire stops reading as a distinct "brand on the roof" mechanism at all
    assert ufs.ORIGIN_FRAC_CAP > 0.45


def test_capped_spot_origin_keeps_the_lower_half_of_a_tall_mass_clean():
    """End to end: a capped spot fire on a tall tower still leaves the
    bottom half of the mass below `origin`. Mirrors `SM_Building_28`
    (o22 of 26, how=spot) from the pre-fix 39-record manifest, where the
    UNCAPPED 0.88 fraction put the origin at storey 22 of 26 and left
    storeys 0-21 completely untouched.
    """
    n_storeys = 26
    rec = {"origin_frac": ufs.ORIGIN_FRAC_CAP, "entry_side": "S",
           "level": "F4"}
    storey, _sides = ufs.entry_for_plan_fire(rec, n_storeys,
                                             rng=random.Random(1))
    assert storey <= (n_storeys - 1) // 2, (storey, n_storeys)


# ---------------------------------------------------------------------------
# 2. the origin draw
# ---------------------------------------------------------------------------
def test_pick_origin_skips_blocked_and_is_deterministic():
    bs = _grid(rows=4, cols=6)
    blocked = frozenset(range(0, len(bs), 2))
    seen = set()
    for seed in range(60):
        a = ufs.pick_origin(bs, blocked, random.Random(seed))
        b = ufs.pick_origin(bs, blocked, random.Random(seed))
        assert a == b, "pick_origin is not deterministic for a given rng"
        assert a not in blocked
        seen.add(a)
    assert len(seen) > 1, "the draw collapsed onto one building"
    with pytest.raises(ValueError):
        ufs.pick_origin(bs, frozenset(range(len(bs))), random.Random(0))


def test_pick_origin_is_biased_toward_the_epicenter():
    """`u ** 1.7` over a nearest-first ranking, so most draws land near it."""
    bs = _grid(rows=6, cols=6)
    ep = (bs[0]["x"], bs[0]["y"])            # the corner building's centre
    order = sorted(range(len(bs)), key=lambda i: (
        (bs[i]["x"] - ep[0]) ** 2 + (bs[i]["y"] - ep[1]) ** 2, i))
    rank = {i: r for r, i in enumerate(order)}
    draws = [rank[ufs.pick_origin(bs, frozenset(), random.Random(s), ep)]
             for s in range(400)]
    near = sum(1 for r in draws if r < len(bs) / 3.0) / float(len(draws))
    assert near > 0.5, "u**1.7 should put most draws in the nearest third"
    assert max(draws) > len(bs) / 2.0, "the tail should still reach far out"
    # the nearest building is by definition the epicentre's own
    assert ufs.pick_origin(bs, frozenset(), None, ep) == order[0]


# ---------------------------------------------------------------------------
# 3. the prefix cap
# ---------------------------------------------------------------------------
def _assert_connected(bs_plan, origin):
    kept = {p["i"] for p in _lit(bs_plan)}
    assert origin in kept
    reach, frontier = {origin}, [origin]
    children = {}
    for p in _lit(bs_plan):
        if p["via"] is not None:
            children.setdefault(p["via"], []).append(p["i"])
    while frontier:
        u = frontier.pop()
        for v in children.get(u, ()):
            if v not in reach:
                reach.add(v)
                frontier.append(v)
    assert reach == kept, "kept set is not one connected subtree: {0}".format(
        sorted(kept - reach))


def test_prefix_cap_is_sized_connected_and_deterministic():
    bs = _grid(rows=5, cols=6)
    origin = 0
    full = ufs.solve(bs, origin, 600 * MIN, wind_dir=0.0, wind_mps=6.0,
                     rng=random.Random(5))
    assert len(_lit(full)) == len(bs), "the whole grid should burn eventually"
    for n in (1, 2, 7, 16, len(bs) - 1):
        cap = ufs.cap_to_prefix(full, n)
        assert len(_lit(cap)) == n, (n, len(_lit(cap)))
        _assert_connected(cap, origin)
        # the kept set is exactly the N earliest, and nothing else moved
        keep = {p["i"] for p in _lit(cap)}
        order = sorted(_lit(full), key=lambda p: (p["t_ignite"], p["i"]))
        assert keep == {p["i"] for p in order[:n]}
        for p in cap:
            if p["i"] in keep:
                assert p == full[p["i"]]
            else:
                assert (p["t_ignite"], p["age"], p["level"], p["via"],
                        p["how"], p["entry_side"]) == (
                    None, None, "F0", None, None, None)
        assert ufs.cap_to_prefix(full, n) == cap        # deterministic
    assert full[origin]["t_ignite"] == 0.0
    assert _lit(ufs.cap_to_prefix(full, 1))[0]["i"] == origin


def test_prefix_cap_is_a_noop_when_n_covers_everything():
    bs = _grid(rows=3, cols=4)
    plan = ufs.solve(bs, 0, 600 * MIN, wind_dir=0.0, wind_mps=6.0,
                     rng=random.Random(2))
    n_lit = len(_lit(plan))
    assert ufs.cap_to_prefix(plan, n_lit) == plan
    assert ufs.cap_to_prefix(plan, n_lit + 50) == plan
    assert ufs.cap_to_prefix(plan, 10 ** 6) == plan


def test_max_burnt_kwarg_is_the_prefix_cap():
    bs = _grid(rows=4, cols=5)
    a = ufs.solve(bs, 0, 600 * MIN, wind_dir=0.0, wind_mps=6.0,
                  rng=random.Random(9), max_burnt=6)
    b = ufs.cap_to_prefix(
        ufs.solve(bs, 0, 600 * MIN, wind_dir=0.0, wind_mps=6.0,
                  rng=random.Random(9)), 6)
    assert a == b
    assert len(_lit(a)) == 6
    _assert_connected(a, 0)


# ---------------------------------------------------------------------------
# 4. the ladder: `level_for_age` and `urban_fire.LADDER`
# ---------------------------------------------------------------------------
def test_levels_are_in_sync_with_urban_fire():
    assert tuple(ufs.LEVELS) == tuple(uf.LEVELS)
    assert ufs.check_levels_sync() == []
    assert set(ufs.RANK) == set(ufs.LEVELS)
    for bt in ("urm", "rc", "rc_glass"):
        assert set(ufs.LEVELS) <= set(uf.LADDER[bt]), bt


def test_level_for_age_only_returns_ladder_keys():
    ages = [a * MIN for a in range(-30, 320, 1)]
    for bt in ("urm", "rc", "rc_glass"):
        keys = set(uf.LADDER[bt])
        got = set()
        for seed in range(25):
            rng = random.Random(seed)
            for a in ages:
                got.add(ufs.level_for_age(a, bt, rng))
            got.add(ufs.level_for_age(a, bt, None))
        assert got <= keys, (bt, sorted(got - keys))
        assert got <= set(ufs.LEVELS), (bt, sorted(got - set(ufs.LEVELS)))


def test_rc_glass_never_collapses():
    """A curtain-wall cage does not drop a wall — no F5, no F5c, ever."""
    for seed in range(50):
        rng = random.Random(seed)
        for a in range(0, 400):
            lvl = ufs.level_for_age(a * MIN, "rc_glass", rng)
            assert lvl not in ("F5", "F5c"), (a, lvl)


def test_level_for_age_bands_and_the_new_top_of_the_ladder():
    # deterministic without an rng: no collapse draw, no burnt-out draw
    assert ufs.level_for_age(-1.0, "urm", None) == "F0"
    assert ufs.level_for_age(0.0, "urm", None) == "F1"
    assert ufs.level_for_age(ufs.T_FLASHOVER, "urm", None) == "F2"
    assert ufs.level_for_age(ufs.T_FULL, "urm", None) == "F3"
    assert ufs.level_for_age(ufs.T_DECAY, "urm", None) == "F4"
    assert ufs.level_for_age(ufs.T_OUT, "urm", None) == "F4"
    assert ufs.level_for_age(ufs.T_COLD, "urm", None) == "F5"
    assert ufs.level_for_age(ufs.T_COLD, "rc", None) == "F4"
    # F5c is reachable from T_OUT for urm AND rc; F6 only for cold urm
    def _reach(bt, age, p):
        return {ufs.level_for_age(age, bt, random.Random(s), **p)
                for s in range(200)}
    assert "F5c" in _reach("urm", ufs.T_OUT + 1, {})
    assert "F5c" in _reach("rc", ufs.T_OUT + 1, {})
    assert "F6" in _reach("urm", ufs.T_COLD + 1, {})
    assert "F6" not in _reach("rc", ufs.T_COLD + 1, {})
    assert "F6" not in _reach("urm", ufs.T_OUT + 1, {})
    # the two probabilities are wired to the right draws
    assert _reach("urm", ufs.T_COLD + 1, {"collapse_p": 0.0,
                                          "burnt_out_p": 0.0}) == {"F5"}
    assert _reach("urm", ufs.T_COLD + 1, {"collapse_p": 1.0}) == {"F5c"}
    assert _reach("urm", ufs.T_COLD + 1, {"collapse_p": 0.0,
                                          "burnt_out_p": 1.0}) == {"F6"}


def test_solve_levels_are_ladder_keys_for_every_btype():
    bs = _grid(rows=4, cols=5)
    for bt in ("urm", "rc", "rc_glass"):
        for seed in range(8):
            plan = ufs.solve(bs, 0, 400 * MIN, wind_dir=0.3, wind_mps=6.0,
                             rng=random.Random(seed),
                             btype_of=lambda b, _t=bt: _t)
            for p in plan:
                assert p["level"] in uf.LADDER[bt], (bt, p)


# ---------------------------------------------------------------------------
# 5. the gradient (plan §2.5)
# ---------------------------------------------------------------------------
def _levels_along(bs, elapsed_s):
    """Levels down a terrace, origin first. `rng=None` on purpose.

    Without an rng `level_for_age` makes no F5c/F6 draw, so the level is a
    pure function of age and the gradient is strictly monotone — which is the
    property being asserted. Given an rng the AGES are still monotone but the
    LEVELS are not, because the collapse draw is independent per building.
    """
    plan = ufs.solve(bs, 0, elapsed_s, wind_dir=0.0, wind_mps=6.0, rng=None)
    return [p["level"] for p in plan], plan


def test_gradient_wave_60_line():
    bs = _line(60)

    # --- (a) the plan's illustrative epoch: 0.55 of a 3-hour duration ------
    # 99 min. Monotone, but the origin is only F4: a building does not leave
    # F4 until T_OUT (140 min). The plan's "origin F5/F5c/F6" is unreachable
    # here and this asserts the arithmetic rather than the wish.
    lv, plan = _levels_along(bs, 0.55 * 3 * 3600)
    rk = [ufs.RANK[x] for x in lv]
    assert all(rk[i] >= rk[i + 1] for i in range(len(rk) - 1)), lv[:12]
    assert lv[0] == "F4"
    assert 0.55 * 3 * 3600 < ufs.T_OUT           # ...and this is why
    assert "F3" in lv and lv[-1] == "F0"

    # --- (b) an epoch deep enough for the whole wave ----------------------
    # 0.55 x 6.6 h = 217.8 min, past T_COLD, so the origin is burnt out.
    lv, plan = _levels_along(bs, 0.55 * 6.6 * 3600)
    rk = [ufs.RANK[x] for x in lv]
    assert all(rk[i] >= rk[i + 1] for i in range(len(rk) - 1)), lv[:14]
    assert ufs.RANK[lv[0]] >= ufs.RANK["F5"], lv[0]       # origin burnt out
    # the whole ladder is on the plate at once, in order
    for want in ("F4", "F3", "F2", "F1"):
        assert want in lv, (want, lv[:14])
    assert lv.index("F4") < lv.index("F3") < lv.index("F2") < lv.index("F1")
    # THE FRONT IS WHERE THE LEVELS STOP, NOT WHERE THE PLAN STOPS. Every
    # building in a terrace eventually catches, so `t_ignite` is set the
    # whole way down the row; what makes the far end F0 is that its ignition
    # is still in the FUTURE at this epoch (negative age).
    front = max(i for i, x in enumerate(lv) if x != "F0")
    assert lv[front] == "F1", lv[front - 2:front + 2]
    assert set(lv[front + 1:]) == {"F0"}
    assert plan[front]["age"] >= 0 > plan[front + 1]["age"]
    # ages fall monotonically away from the origin, which is the real claim
    ages = [p["age"] for p in _lit(plan)]
    assert ages == sorted(ages, reverse=True)


def test_gradient_ages_are_monotone_even_with_an_rng():
    """The stochastic part is the LEVEL past T_OUT, never the arrival time."""
    bs = _line(40)
    plan = ufs.solve(bs, 0, 0.55 * 6.6 * 3600, wind_dir=0.0, wind_mps=6.0,
                     rng=random.Random(5))
    lit = _lit(plan)
    assert [p["i"] for p in lit] == sorted(p["i"] for p in lit)
    assert all(lit[i]["t_ignite"] <= lit[i + 1]["t_ignite"]
               for i in range(len(lit) - 1))


# ---------------------------------------------------------------------------
# 6. the join with `urban_fire.plan_fire`
# ---------------------------------------------------------------------------
def test_entry_for_plan_fire_storeys_sides_and_the_F3_corner():
    for n in (1, 2, 3, 5, 12, 30):
        for frac, side, lvl in ((0.15, None, "F1"), (0.22, "W", "F2"),
                                (0.45, "S", "F3"), (0.88, "N", "F4"),
                                (0.45, "E", "F5"), (0.45, "E", "F5c"),
                                (0.88, "S", "F6"), (0.25, "E", "F0")):
            rec = {"origin_frac": frac, "entry_side": side, "level": lvl}
            st, sides = ufs.entry_for_plan_fire(rec, n, random.Random(3))
            assert isinstance(st, int) and 0 <= st <= n - 1, (n, lvl, st)
            assert 1 <= len(sides) <= 2
            assert len(set(sides)) == len(sides)
            for s in sides:
                assert s in ufs._SIDE_RING
            if side is not None:
                assert sides[0] == side
            want = 2 if ufs.RANK[lvl] >= ufs.RANK["F3"] else 1
            assert len(sides) == want, (lvl, sides)
            if len(sides) == 2:
                assert sides[1] in ufs.side_neighbors(sides[0])
    # A BRAND LANDS HIGH AND A PARTY WALL LOW, at every building height.
    # `origin_frac` is a fraction of HEIGHT, so 0.88 is the top storey of a
    # 3-storey block and storey 21 of a 25-storey tower — the top eighth in
    # both cases, which is what "on the roof" has to mean once a building is
    # tall enough for "the top storey" and "where a brand landed" to differ.
    for n in (3, 8, 25):
        st, _ = ufs.entry_for_plan_fire(
            {"origin_frac": 0.88, "entry_side": "S", "level": "F3"}, n)
        assert st >= 0.8 * (n - 1), (n, st)
        st, _ = ufs.entry_for_plan_fire(
            {"origin_frac": 0.22, "entry_side": "S", "level": "F3"}, n)
        assert st <= 0.3 * (n - 1) + 0.5, (n, st)
    assert ufs.entry_for_plan_fire(
        {"origin_frac": 0.88, "entry_side": "S", "level": "F3"}, 3)[0] == 2


# ---------------------------------------------------------------------------
# 6b. MORE SIDES BURNING (2026-08-31 user policy): F3+ is now a draw over
# {2, 3} extra-elevation counts (2 or 3 total sides), shifted up by one
# option from the old fixed "always exactly 1 extra" (2 total). F1/F2 is
# untouched -- always exactly 1 side, with or without an rng.
# ---------------------------------------------------------------------------
def test_f3_plus_side_count_is_two_or_three_with_an_rng():
    # random.Random(3).randint(1, 2) == 1 -> the OLD default, 2 total sides.
    rec_f3 = {"origin_frac": 0.45, "entry_side": "S", "level": "F3"}
    _, sides = ufs.entry_for_plan_fire(rec_f3, 6, random.Random(3))
    assert sides == ("S", "W") or sides == ("S", "E")
    assert len(sides) == 2

    # random.Random(0).randint(1, 2) == 2 -> the NEW option, 3 total sides:
    # the entry side plus BOTH corner neighbours.
    _, sides3 = ufs.entry_for_plan_fire(rec_f3, 6, random.Random(0))
    assert len(sides3) == 3
    assert sides3[0] == "S"
    assert set(sides3[1:]) == set(ufs.side_neighbors("S"))
    assert len(set(sides3)) == 3          # no duplicate elevation


def test_f3_plus_side_count_without_an_rng_stays_at_the_old_default():
    """No `rng` -> no change in behaviour at all: still exactly 2 total
    sides, deterministically `nb[0]`."""
    rec_f3 = {"origin_frac": 0.45, "entry_side": "S", "level": "F3"}
    _, sides = ufs.entry_for_plan_fire(rec_f3, 6)
    assert sides == ("S", ufs.side_neighbors("S")[0])


def test_f1_f2_never_gain_a_second_side_regardless_of_rng():
    for lvl in ("F1", "F2"):
        rec = {"origin_frac": 0.2, "entry_side": "S", "level": lvl}
        for seed in range(10):
            _, sides = ufs.entry_for_plan_fire(rec, 5, random.Random(seed))
            assert sides == ("S",), (lvl, seed, sides)


def test_f3_plus_side_count_distribution_is_roughly_even():
    """Over many seeds, both the 2-side and the 3-side outcome actually
    occur -- not just reachable in principle for one hand-picked seed."""
    rec_f4 = {"origin_frac": 0.45, "entry_side": "N", "level": "F4"}
    counts = {2: 0, 3: 0}
    for seed in range(200):
        _, sides = ufs.entry_for_plan_fire(rec_f4, 8, random.Random(seed))
        counts[len(sides)] += 1
    assert counts[2] > 50 and counts[3] > 50, counts


# ---------------------------------------------------------------------------
# 6c. STREET-FACING SIDE PREFERENCE (2026-08-31 user policy): `street_
# side_score` scores S/E/N/W by clearance, and `entry_for_plan_fire` uses it
# ONLY where nothing about contagion has already fixed the answer -- the
# origin's own free choice, and F3+'s single extra corner side. It must
# NEVER override a real (non-origin) entry side.
# ---------------------------------------------------------------------------
def test_street_side_score_prefers_the_open_side():
    b0 = {"x": 0.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}
    b1 = {"x": 20.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}   # east of b0
    bl = [b0, b1]
    assert ufs.street_side_score(b0, "W", bl) > ufs.street_side_score(b0, "E", bl)
    # no block_rect -> exactly the neighbour clearance (gap_m)
    assert ufs.street_side_score(b0, "E", bl) == ufs.gap_m(b0, b1)
    # a side with no facing neighbour at all is capped, not unbounded
    assert ufs.street_side_score(b0, "W", bl) == ufs.STREET_SCORE_CAP_M


def test_street_side_score_with_block_rect_can_go_negative():
    """A neighbour closer than the block edge scores NEGATIVE, however far
    away the edge itself is -- that side looks at the neighbour, not the
    street."""
    b0 = {"x": 0.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}
    b1 = {"x": 20.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}
    rect = (-50.0, -50.0, 50.0, 50.0)
    sw = ufs.street_side_score(b0, "W", [b0, b1], block_rect=rect)
    se = ufs.street_side_score(b0, "E", [b0, b1], block_rect=rect)
    assert sw == pytest.approx(155.0)     # 200 (capped, no neighbour) - 45
    assert se == pytest.approx(-35.0)     # 10 (b1's clearance) - 45
    assert sw > 0 > se


def test_entry_for_plan_fire_origin_side_follows_street_score():
    rec_origin = {"origin_frac": 0.15, "entry_side": None, "level": "F1"}
    prefer_n = lambda s: 1.0 if s == "N" else 0.0
    _, sides = ufs.entry_for_plan_fire(rec_origin, 4, None, street_score=prefer_n)
    assert sides == ("N",)
    # no street_score -> the old fallback (rng draw, or "S" with no rng)
    _, sides_none = ufs.entry_for_plan_fire(rec_origin, 4, None)
    assert sides_none == ("S",)


def test_entry_for_plan_fire_never_overrides_a_real_entry_side():
    """Precedence: entry-side realism wins over street-facing preference
    whenever contagion has actually decided the side."""
    prefer_n = lambda s: 1.0 if s == "N" else 0.0
    for lvl in ("F1", "F2", "F3", "F4"):
        rec = {"origin_frac": 0.3, "entry_side": "W", "level": lvl}
        _, sides = ufs.entry_for_plan_fire(rec, 5, random.Random(1),
                                           street_score=prefer_n)
        assert sides[0] == "W"


def test_entry_for_plan_fire_f3_plus_tiebreak_follows_street_score():
    """Only the SINGLE extra corner side (2-total-sides draw) is subject to
    the street-facing tiebreak; when both are added (3-total) there is no
    choice left to make."""
    prefer_e = lambda s: 1.0 if s == "E" else 0.0
    rec_f3 = {"origin_frac": 0.45, "entry_side": "S", "level": "F3"}
    # random.Random(3) draws the 1-extra-side branch (2 total)
    _, sides = ufs.entry_for_plan_fire(rec_f3, 6, random.Random(3),
                                       street_score=prefer_e)
    assert sides == ("S", "E")
    # random.Random(0) draws the 2-extra-sides branch (3 total) -- street_
    # score has nothing left to decide, both neighbours are already in
    _, sides3 = ufs.entry_for_plan_fire(rec_f3, 6, random.Random(0),
                                        street_score=prefer_e)
    assert set(sides3) == {"S"} | set(ufs.side_neighbors("S"))


def test_entry_for_plan_fire_feeds_plan_fire():
    """The join actually composes: real building -> real `plan_fire`."""
    ub = pytest.importorskip("detail.urban_building")
    qf = pytest.importorskip("disaster.quake_flow")
    bs = _line(6)
    plan = ufs.solve(bs, 0, 0.55 * 6.6 * 3600, wind_dir=0.0, wind_mps=6.0,
                     rng=random.Random(4))
    for rec in _lit(plan):
        rng = random.Random(100 + rec["i"])
        pls = ub.build_building("apartment", 0.0, 0.0, 0.0, rng)
        info = qf.describe("apartment", pls, 0.0, 0.0, 0.0)
        mtag = max(info["masses"].items(),
                   key=lambda kv: (len(kv[1]["levels"]), kv[0] == "main"))[0]
        n = len(info["masses"][mtag]["levels"])
        st, sides = ufs.entry_for_plan_fire(rec, n, rng)
        f = uf.plan_fire(info, rec["level"], rng, origin=st, sides=sides)
        assert f["origin"] == st
        assert f["sides"] == tuple(sides)
        assert f["storeys"][0] == st and f["top"] < n


# ---------------------------------------------------------------------------
# 7. determinism
# ---------------------------------------------------------------------------
def test_same_seed_same_plan_and_unrelated_draws_do_not_leak():
    bs = _grid(rows=4, cols=6)
    blocked = frozenset((3, 9, 14))

    def _run():
        rng = random.Random(2024)
        origin = ufs.pick_origin(bs, blocked, rng, (30.0, 30.0))
        return origin, ufs.solve(bs, origin, 300 * MIN, wind_dir=0.4,
                                 wind_mps=6.0, rng=rng, blocked=blocked,
                                 max_burnt=9)

    o1, p1 = _run()
    # advance an UNRELATED generator: the plan may not move
    noise = random.Random(7)
    for _ in range(50):
        noise.random()
    random.seed(1234)
    for _ in range(50):
        random.random()
    o2, p2 = _run()
    assert o1 == o2
    assert p1 == p2
    assert len(_lit(p1)) == 9
    _assert_connected(p1, o1)


# ---------------------------------------------------------------------------
# 8. height class: who is allowed to collapse, and whose roof may open
#
# 2026-08-31 policy, superseding the earlier blanket "no fire in skyscraper
# districts": collapse eligibility is by HEIGHT CLASS, not a per-district
# fire ban --
#   low         rowhouse/lowrise      -- full collapse stands (F6, F5c ok)
#   mid_high    midrise/brick_midrise/tower/highrise -- PARTIAL collapse
#               only (cap F5c, never F6)
#   skyscraper  -- fire only, never any collapse (cap F5, never F5c/F6)
# `tower`/`highrise` are grouped as `skyscraper` here: they were BOTH
# `urban_fire_city.NO_FIRE_TYPOLOGIES` before this policy (fire banned
# outright), and their measured height pools genuinely overlap (tower
# 44.7-131 m, highrise 103.7-312 m) so splitting them by height instead of
# by typology name would be ambiguous exactly where it matters.
# ---------------------------------------------------------------------------
def test_height_class_typology_is_the_source_of_truth():
    assert ufs.height_class(typology="rowhouse") == ufs.HEIGHT_CLASS_LOW
    assert ufs.height_class(typology="lowrise") == ufs.HEIGHT_CLASS_LOW
    assert ufs.height_class(typology="midrise") == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(typology="brick_midrise") == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(typology="tower") == ufs.HEIGHT_CLASS_SKYSCRAPER
    assert ufs.height_class(typology="highrise") == ufs.HEIGHT_CLASS_SKYSCRAPER
    # typology wins even when storeys/H would disagree
    assert ufs.height_class(typology="rowhouse", n_storeys=90,
                            H_m=300.0) == ufs.HEIGHT_CLASS_LOW


def test_height_class_storey_and_metre_fallbacks_at_their_boundaries():
    assert ufs.height_class(n_storeys=ufs.STOREY_LOW_MAX) == ufs.HEIGHT_CLASS_LOW
    assert ufs.height_class(
        n_storeys=ufs.STOREY_LOW_MAX + 1) == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(
        n_storeys=ufs.STOREY_MIDHIGH_MAX) == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(
        n_storeys=ufs.STOREY_MIDHIGH_MAX + 1) == ufs.HEIGHT_CLASS_SKYSCRAPER
    assert ufs.height_class(H_m=ufs.LOW_H_MAX_M) == ufs.HEIGHT_CLASS_LOW
    assert ufs.height_class(H_m=ufs.LOW_H_MAX_M + 0.1) == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(H_m=ufs.MIDHIGH_H_MAX_M) == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(
        H_m=ufs.MIDHIGH_H_MAX_M + 0.1) == ufs.HEIGHT_CLASS_SKYSCRAPER
    # storeys wins over H when both are given, with no typology
    assert ufs.height_class(n_storeys=2, H_m=300.0) == ufs.HEIGHT_CLASS_LOW
    # nothing given at all -> the conservative middle, not "low"
    assert ufs.height_class() == ufs.HEIGHT_CLASS_MIDHIGH
    assert ufs.height_class(typology="not_a_real_typology") == ufs.HEIGHT_CLASS_MIDHIGH


def test_cap_level_for_class_degrades_never_drops_to_f0():
    # low: full collapse stands
    assert ufs.cap_level_for_class("F6", ufs.HEIGHT_CLASS_LOW) == "F6"
    assert ufs.cap_level_for_class("F5c", ufs.HEIGHT_CLASS_LOW) == "F5c"
    # mid_high: partial collapse only -- F6 degrades one step, to F5c
    assert ufs.cap_level_for_class("F6", ufs.HEIGHT_CLASS_MIDHIGH) == "F5c"
    assert ufs.cap_level_for_class("F5c", ufs.HEIGHT_CLASS_MIDHIGH) == "F5c"
    # skyscraper: fire only -- F6 degrades two steps, all the way to F5
    assert ufs.cap_level_for_class("F6", ufs.HEIGHT_CLASS_SKYSCRAPER) == "F5"
    assert ufs.cap_level_for_class("F5c", ufs.HEIGHT_CLASS_SKYSCRAPER) == "F5"
    # never touches anything below F5c, for any class
    for lvl in ("F0", "F1", "F2", "F3", "F4", "F5"):
        for cls in ufs.HEIGHT_CLASSES:
            assert ufs.cap_level_for_class(lvl, cls) == lvl


def test_roof_eligibility_is_low_only_and_degrades_to_f5():
    assert ufs.roof_eligible(ufs.HEIGHT_CLASS_LOW)
    assert not ufs.roof_eligible(ufs.HEIGHT_CLASS_MIDHIGH)
    assert not ufs.roof_eligible(ufs.HEIGHT_CLASS_SKYSCRAPER)
    # low keeps both roof-affecting outcomes
    assert ufs.enforce_roof_eligibility("F6", ufs.HEIGHT_CLASS_LOW) == "F6"
    assert ufs.enforce_roof_eligibility("F5c", ufs.HEIGHT_CLASS_LOW) == "F5c"
    # mid_high/skyscraper: NEVER a roof-opening outcome, regardless of what
    # the rank cap alone would have allowed (mid_high's own cap permits
    # F5c -- this is the policy that says it should not actually show it)
    for cls in (ufs.HEIGHT_CLASS_MIDHIGH, ufs.HEIGHT_CLASS_SKYSCRAPER):
        assert ufs.enforce_roof_eligibility("F5c", cls) == "F5"
        assert ufs.enforce_roof_eligibility("F6", cls) == "F5"
    # never touches a non-ROOF_LEVELS level
    for lvl in ("F0", "F1", "F2", "F3", "F4", "F5"):
        for cls in ufs.HEIGHT_CLASSES:
            assert ufs.enforce_roof_eligibility(lvl, cls) == lvl


def test_solve_applies_the_rank_cap_end_to_end():
    """An old-age skyscraper gets F5, an old-age midrise F5c, an old-age
    brownstone F6 -- `solve()` applies ONLY the rank cap (not roof
    eligibility, which is a manifest-generation concern -- see
    `urban_fire_city.damaged_manifest`), so this exercises `cap_level_for_
    class` wired through `height_class_of` rather than `enforce_roof_
    eligibility`. `collapse_p=0.0, burnt_out_p=1.0` forces the raw,
    uncapped level to F6 deterministically for any rng: the F5c coin can
    never hit 0.0 and the F6 coin can never miss 1.0."""
    bs = [{"x": 0.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
          "style": "brownstone"},
         {"x": 500.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
          "style": "midrise"},
         {"x": 1000.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
          "style": "skyscraper"}]
    cls_of = {"brownstone": ufs.HEIGHT_CLASS_LOW,
             "midrise": ufs.HEIGHT_CLASS_MIDHIGH,
             "skyscraper": ufs.HEIGHT_CLASS_SKYSCRAPER}
    for i, want_level, want_cls in (
            (0, "F6", ufs.HEIGHT_CLASS_LOW),
            (1, "F5c", ufs.HEIGHT_CLASS_MIDHIGH),
            (2, "F5", ufs.HEIGHT_CLASS_SKYSCRAPER)):
        plan = ufs.solve(bs, i, 220 * MIN, wind_dir=0.0, wind_mps=0.0,
                         rng=random.Random(1), collapse_p=0.0, burnt_out_p=1.0,
                         height_class_of=lambda b: cls_of[b["style"]])
        assert plan[i]["level"] == want_level, (i, plan[i])
        assert plan[i]["height_class"] == want_cls


def test_solve_height_class_default_fallback_uses_h_metres():
    """No `height_class_of` given -> `solve()` falls back to `height_class(
    H_m=b["H"])` per building, so a tall building still caps itself even
    when the caller has no typology to hand it."""
    bs = [{"x": 0.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0},
         {"x": 500.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 200.0}]
    plan = ufs.solve(bs, 1, 220 * MIN, wind_dir=0.0, wind_mps=0.0,
                     rng=random.Random(1), collapse_p=0.0, burnt_out_p=1.0)
    assert plan[1]["height_class"] == ufs.HEIGHT_CLASS_SKYSCRAPER
    assert plan[1]["level"] == "F5"


def test_module_check_is_clean():
    assert ufs.check(verbose=False) == []


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
