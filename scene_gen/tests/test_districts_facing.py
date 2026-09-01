#!/usr/bin/env python3
"""
test_districts_facing.py — the blank-wall-off-the-street mechanism, pinned
without Isaac.

`districts.py` carries per-asset `yaw-offset` and `blank:`/`front:`/`place`
metadata into the pool entry (`_pool_entries`), uses it to keep a blank
elevation off a street AND, since the 2026-08-29 veto-to-score rewrite, to
prefer the yaw that puts the asset's FRONT on a street over one that merely
isn't illegal (`_pack_free`'s `_yaw_score`, `_lay_terrace`'s per-strip filter
and `_order_run`), and to suppress a model repeating too close to itself.
Six claims are worth pinning here, because a wrong compass rotation or a
wrong street-side test looks identical to a correct one until a render shows
a blank wall facing traffic — or, since the rewrite, a windowless face
standing where a windowed one was available:

  1. `_rot_side`/`_rot_sides` implement the documented CCW convention (+90
     maps E->N, N->W, W->S, S->E) for all four letters at all four multiples
     of 90 degrees.
  2. `_pool_entries` SWAPS a footprint's sx/sy exactly when `yaw-offset` is an
     odd multiple of 90 — the rotation that turns a building a quarter turn
     also turns which extent lands along world X.
  3. `_street_sides` reports 2 sides for a footprint at a block corner, 1 for
     one flush against a single edge, 0 for one in the interior.
  4. `_pack_free` REFUSES a synthetic one-sided ("place_mid") asset at a
     corner slot (two street sides, one modelled flank cannot cover both) and
     ACCEPTS the same asset at a single-street-side slot where its modelled
     side lines up with the street; then, past that hard reject, `_yaw_score`
     ranks the survivors — front-on-street beats not, the longer of two
     available streets beats the shorter, more streets engaged beats fewer,
     and depth-into-the-block is the last tiebreak (section "4c" below,
     including `place_never_corner` and `_lay_terrace_end_caps`, the compact
     stock this rewrite lets a generous terrace alley's own short ends take).
  5. `_order_run` puts a `mid` entry in the run's interior, drops a run that
     cannot be laid legally (all-mid, or a lone `mid`), and — the case that
     must stay inert for a pool with no metadata — leaves an all-`any` run's
     order AND the caller's `rng` state untouched.

RUNS WITHOUT ISAAC. `districts` imports `scene_generator` at module scope,
which imports `pxr` at module scope; both are stubbed out below exactly as
`tools/plan_png.py` stubs them, so nothing here touches USD or a GPU.

USAGE
    python3 scene_gen/tests/test_districts_facing.py
    pytest -s scene_gen/tests/test_districts_facing.py
"""

import os
import random
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

# `scene_generator` imports pxr at module scope; nothing reached from here
# touches it. Same stub `tools/plan_png.py` installs.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, types.ModuleType(_n))

from detail import districts as dd                    # noqa: E402

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def strict(fn):
    """Make a recorded FAIL an actual pytest failure — same idiom as
    `test_car_toss.py`/`test_fence_rules.py`: `check` records rather than
    raises so a run prints the whole table, and under pytest each test
    re-reads the tally it started with and asserts nothing was added."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


def _entry(usd, sx, sy, sz=10.0, base=0.0, yaw_offset=0.0, place="any",
          front=None, blank=frozenset(), never_corner=False):
    """A synthetic 6-tuple pool entry, built the same way `_pool_entries`
    builds a real one — footprint UNROTATED (as the resolver would report
    it), `blank0`/`front0` derived from `blank`/`front` and `yaw_offset` via
    `_rot_sides`/`_rot_side`, exactly what `_pool_entries` does."""
    fp = {"sx": float(sx), "sy": float(sy), "sz": float(sz), "base": float(base)}
    meta = {"place": place, "front": front, "blank": frozenset(blank),
            "blank0": dd._rot_sides(frozenset(blank), yaw_offset),
            "front0": dd._rot_side(front, yaw_offset) if front else None,
            "never_corner": never_corner}
    return (usd, 1.0, "Z", fp, float(yaw_offset), meta)


# ---------------------------------------------------------------------------
# 1. _rot_side / _rot_sides
# ---------------------------------------------------------------------------

@strict
def test_rot_side_ccw_cycle():
    print("\n[1] _rot_side: +90 CCW walks E -> N -> W -> S -> E")
    cycle = ("E", "N", "W", "S")
    for i, letter in enumerate(cycle):
        for steps in range(-4, 5):
            want = cycle[(i + steps) % 4]
            got = dd._rot_side(letter, steps * 90.0)
            check(got == want,
                  f"_rot_side({letter!r}, {steps * 90:.0f}) = {got!r}, "
                  f"want {want!r}")
    # The four claims the module docstring and `tools/faces_to_yaml.py`'s
    # FRONT_TO_YAW table both make explicitly.
    for letter, deg, want in (("E", 90, "N"), ("N", 90, "W"),
                              ("W", 90, "S"), ("S", 90, "E")):
        got = dd._rot_side(letter, deg)
        check(got == want, f"+90 CCW: {letter} -> {got}, want {want}")
    # A full turn is a no-op, and 360 folds the same as 0.
    for letter in cycle:
        check(dd._rot_side(letter, 360.0) == letter, f"360 deg is a no-op on {letter}")
        check(dd._rot_side(letter, 0.0) == letter, f"0 deg is a no-op on {letter}")


@strict
def test_rot_sides_set():
    print("\n[1] _rot_sides: element-wise, empty set stays empty")
    check(dd._rot_sides(frozenset(), 90.0) == frozenset(),
          "empty in, empty out at every angle — the no-metadata no-op")
    got = dd._rot_sides(frozenset({"N", "E", "S"}), 90.0)
    want = frozenset({dd._rot_side("N", 90.0), dd._rot_side("E", 90.0),
                      dd._rot_side("S", 90.0)})
    check(got == want, f"_rot_sides({{'N','E','S'}}, 90) = {sorted(got)}")
    check(got == frozenset({"W", "N", "E"}),
          f"N,E,S rotated +90 = W,N,E (got {sorted(got)})")


# ---------------------------------------------------------------------------
# 2. the sx/sy swap, through _pool_entries itself
# ---------------------------------------------------------------------------

class _FakeResolver:
    """Answers every query with the SAME unrotated footprint, so the test is
    purely about what `_pool_entries` does to it, not about path resolution."""

    def __init__(self, sx, sy, sz=50.0, base=0.0):
        self.fp = {"sx": float(sx), "sy": float(sy), "sz": float(sz),
                  "base": float(base)}
        self.calls = 0

    def get(self, usd, category, scale=1.0, axis_up="Z", **_kw):
        self.calls += 1
        return dict(self.fp)                # a FRESH copy every call


@strict
def test_pool_entries_swaps_on_odd_90():
    print("\n[2] _pool_entries: sx/sy swap exactly on an odd multiple of 90")
    # SM_Building_24: 29.0 x 58.0 m, yaw-offset 90 — the concrete example
    # from the task brief. Depth in a terrace is the SWAPPED sx (58 m).
    for offset, want_swapped in ((0.0, False), (90.0, True), (180.0, False),
                                 (270.0, True), (-90.0, True), (450.0, True)):
        res = _FakeResolver(sx=29.0, sy=58.0)
        cfg = {
            "usds": {"buildings": {"pool": [
                {"usd": "SM_Building_24.usd", "scale": 1.0,
                 "yaw-offset": offset, "tags": ["place_end"]},
            ]}},
            "asset_scale": 1.0, "asset_root": "",
        }
        entries = dd._pool_entries(cfg, res, "pool")
        check(len(entries) == 1, f"offset {offset}: one entry back")
        usd, sc, au, fp, yaw_offset, meta = entries[0]
        # `_pool_entries` carries the offset through UNCHANGED — the modulo
        # only happens once, in `_new_placement`, when it is finally added to
        # a placement yaw.
        check(yaw_offset == offset,
              f"offset {offset}: yaw_offset carried through as-is "
              f"({yaw_offset})")
        if want_swapped:
            check(fp["sx"] == 58.0 and fp["sy"] == 29.0,
                  f"offset {offset}: EXPECTED swap, got sx={fp['sx']} "
                  f"sy={fp['sy']}")
        else:
            check(fp["sx"] == 29.0 and fp["sy"] == 58.0,
                  f"offset {offset}: expected NO swap, got sx={fp['sx']} "
                  f"sy={fp['sy']}")
        # sz/base never touch, regardless of the swap.
        check(fp["sz"] == 50.0 and fp["base"] == 0.0,
              f"offset {offset}: sz/base untouched")

    # The resolver's own dict must never be the one that gets mutated — two
    # pools sharing an asset with different yaw-offsets must not fight over
    # one cached footprint.
    res = _FakeResolver(sx=29.0, sy=58.0)
    cfg = {"usds": {"buildings": {
        "a": [{"usd": "X.usd", "yaw-offset": 90, "tags": ["place_end"]}],
        "b": [{"usd": "X.usd", "yaw-offset": 0, "tags": ["place_end"]}],
    }}, "asset_scale": 1.0, "asset_root": ""}
    ea = dd._pool_entries(cfg, res, "a")[0]
    eb = dd._pool_entries(cfg, res, "b")[0]
    check(ea[3]["sx"] == 58.0 and ea[3]["sy"] == 29.0, "pool a: swapped")
    check(eb[3]["sx"] == 29.0 and eb[3]["sy"] == 58.0,
          "pool b: NOT swapped (independent of pool a's mutation)")
    check(res.fp == {"sx": 29.0, "sy": 58.0, "sz": 50.0, "base": 0.0},
          "the resolver's own footprint dict was never mutated")


@strict
def test_pool_entries_meta_and_blank0():
    print("\n[2b] _pool_entries: place/front/blank parsed, blank0 rotated by "
          "yaw-offset")
    res = _FakeResolver(sx=29.0, sy=28.0)
    cfg = {"usds": {"buildings": {"pool": [
        {"usd": "SM_Building_01.usd", "yaw-offset": 180,
         "tags": ["place_mid", "front:E", "blank:N,W,S"]},
    ]}}, "asset_scale": 1.0, "asset_root": ""}
    usd, sc, au, fp, yaw_offset, meta = dd._pool_entries(cfg, res, "pool")[0]
    check(meta["place"] == "mid", f"place parsed: {meta['place']}")
    check(meta["front"] == "E", f"front parsed: {meta['front']}")
    check(meta["blank"] == frozenset({"N", "W", "S"}),
          f"blank parsed: {sorted(meta['blank'])}")
    check(meta["blank0"] == dd._rot_sides(frozenset({"N", "W", "S"}), 180.0),
          "blank0 = blank rotated by yaw_offset")

    # No tags at all -> the no-op defaults every consumer relies on.
    res2 = _FakeResolver(sx=10.0, sy=10.0)
    cfg2 = {"usds": {"buildings": {"pool": ["plain.usd"]}},
           "asset_scale": 1.0, "asset_root": ""}
    _u, _s, _a, _fp, yo, meta2 = dd._pool_entries(cfg2, res2, "pool")[0]
    check(yo == 0.0, "untagged entry: yaw_offset 0")
    check(meta2["place"] == "any", "untagged entry: place any")
    check(meta2["front"] is None, "untagged entry: no front")
    check(meta2["blank"] == frozenset() and meta2["blank0"] == frozenset(),
          "untagged entry: blank and blank0 both empty")


# ---------------------------------------------------------------------------
# 3. _street_sides
# ---------------------------------------------------------------------------

@strict
def test_street_sides_corner_edge_interior():
    print("\n[3] _street_sides: corner (2), single edge (1), interior (0)")
    block = (0.0, 0.0, 100.0, 100.0)
    # Corner: flush against the block's south-west corner.
    sides = dd._street_sides(block, 0.0, 0.0, 10.0, 6.0, tol_m=6.0)
    check(sides == frozenset({"W", "S"}),
          f"SW corner footprint: {sorted(sides)}, want W,S")
    # A different corner (north-east) to rule out an axis-only bug.
    sides = dd._street_sides(block, 90.0, 94.0, 10.0, 6.0, tol_m=6.0)
    check(sides == frozenset({"E", "N"}),
          f"NE corner footprint: {sorted(sides)}, want E,N")
    # Single edge: flush against west only, well clear of north/south.
    sides = dd._street_sides(block, 0.0, 45.0, 10.0, 6.0, tol_m=6.0)
    check(sides == frozenset({"W"}),
          f"west-edge-only footprint: {sorted(sides)}, want W")
    # Interior: nowhere near any block edge.
    sides = dd._street_sides(block, 45.0, 45.0, 10.0, 6.0, tol_m=6.0)
    check(sides == frozenset(),
          f"interior footprint: {sorted(sides)}, want none")
    # tol_m matters: just inside vs just outside the tolerance band.
    sides_in = dd._street_sides(block, 5.9, 45.0, 10.0, 6.0, tol_m=6.0)
    sides_out = dd._street_sides(block, 6.1, 45.0, 10.0, 6.0, tol_m=6.0)
    check("W" in sides_in, "5.9 m off the edge still counts as street-facing")
    check("W" not in sides_out, "6.1 m off the edge no longer counts")


# ---------------------------------------------------------------------------
# 4. _pack_free: corner refusal, single-side acceptance
# ---------------------------------------------------------------------------

@strict
def test_pack_free_corner_refused_edge_accepted():
    print("\n[4] _pack_free: a one-sided asset refused at a corner, accepted "
          "at a single street side its front matches")
    block = (0.0, 0.0, 100.0, 100.0)
    # ONE-SIDED: only its front (canonical W, per the module's own
    # yaw-offset convention) is modelled; N, E, S are all blank.
    pool = [_entry("one_sided.usd", sx=10.0, sy=6.0, yaw_offset=0.0,
                   place="mid", front="W", blank=frozenset({"N", "E", "S"}))]

    # (a) CORNER SLOT: the sub-rect IS the block's SW corner (2 street sides,
    # W and S). Neither yaw 0 (front W) nor 180 (front E) ever avoids "S" —
    # the asset never has a candidate front pointing S in this shape — so
    # BOTH orientations must be refused and the rect must come back empty.
    rng = random.Random(1)
    sky = dd._Skyline({}, rng)
    placed, refused = dd._pack_free(
        (0.0, 0.0, 10.0, 6.0), pool, gap=2.0, min_side=1.0, rng=rng, sky=sky,
        typ={}, block_rect=block, street_tol_m=6.0)
    check(placed == [], f"corner slot: expected nothing placed, got {placed}")
    check(refused == 1, f"corner slot: expected refused=1, got {refused}")

    # (b) WEST-EDGE-ONLY SLOT: one street side (W). At yaw 0 the asset's
    # front (W) faces exactly that street side, so it must be ACCEPTED.
    rng2 = random.Random(1)
    sky2 = dd._Skyline({}, rng2)
    placed2, refused2 = dd._pack_free(
        (0.0, 45.0, 10.0, 51.0), pool, gap=2.0, min_side=1.0, rng=rng2,
        sky=sky2, typ={}, block_rect=block, street_tol_m=6.0)
    check(len(placed2) == 1,
          f"west-edge slot: expected one placement, got {placed2}")
    if placed2:
        e, cx, cy, yaw = placed2[0]
        check(e[0] == "one_sided.usd", "west-edge slot: the one candidate")
        check(yaw % 360.0 == 0.0,
              f"west-edge slot: expected yaw 0 (front already faces W), "
              f"got {yaw}")
    check(refused2 == 0, f"west-edge slot: expected refused=0, got {refused2}")

    # (c) SANITY: with block_rect=None the facing test is skipped entirely —
    # the corner slot that was refused above must now place something,
    # proving the refusal above came from the facing filter and not from
    # some unrelated fit failure.
    rng3 = random.Random(1)
    sky3 = dd._Skyline({}, rng3)
    placed3, refused3 = dd._pack_free(
        (0.0, 0.0, 10.0, 6.0), pool, gap=2.0, min_side=1.0, rng=rng3,
        sky=sky3, typ={}, block_rect=None)
    check(len(placed3) == 1,
          "corner slot with block_rect=None: facing test skipped, "
          "something IS placed")
    check(refused3 == 0, "corner slot with block_rect=None: refused=0")


@strict
def test_pack_free_place_none_never_placed():
    print("\n[4b] _pack_free: place_none is dropped outright, block_rect or not")
    block = (0.0, 0.0, 100.0, 100.0)
    pool = [_entry("blank_all.usd", sx=10.0, sy=6.0, place="none")]
    for br in (block, None):
        rng = random.Random(3)
        sky = dd._Skyline({}, rng)
        placed, refused = dd._pack_free(
            (45.0, 45.0, 55.0, 51.0), pool, gap=2.0, min_side=1.0, rng=rng,
            sky=sky, typ={}, block_rect=br)
        check(placed == [],
              f"place_none never placed (block_rect={'set' if br else 'None'})")


# ---------------------------------------------------------------------------
# 4c. the SCORE, not the veto — 2026-08-29 rewrite, per four counter-examples
# the user gave reviewing a built scene: a building can be LEGAL at more than
# one yaw and still be WRONG at all but one of them, so `_pack_free` now
# scores every surviving candidate (`_yaw_score`) instead of taking the
# first one the old veto didn't drop. `_has_facing_pref` gates it (blank/
# front/place/never_corner), which is what keeps an untagged pool
# (`downtown`/`downtown_1000`) drawing byte-identical to before this existed
# — confirmed separately via `plan_png.py --json`, not repeated here.
# ---------------------------------------------------------------------------

@strict
def test_pool_entries_front0_and_never_corner():
    print("\n[4c-1] _pool_entries: front0 = front rotated by yaw-offset "
          "(the module's own convention makes it always 'W'), "
          "place_never_corner parsed")
    res = _FakeResolver(sx=29.0, sy=28.0)
    cfg = {"usds": {"buildings": {"pool": [
        {"usd": "A.usd", "yaw-offset": 180,
         "tags": ["front:E", "place_never_corner"]},
    ]}}, "asset_scale": 1.0, "asset_root": ""}
    _u, _s, _a, _fp, _yo, meta = dd._pool_entries(cfg, res, "pool")[0]
    check(meta["front0"] == dd._rot_side("E", 180.0),
          f"front0 = front rotated by yaw-offset: {meta['front0']}")
    check(meta["front0"] == "W",
          "a correctly-authored yaw-offset always turns front0 to W")
    check(meta["never_corner"] is True, "place_never_corner parsed")

    res2 = _FakeResolver(sx=10.0, sy=10.0)
    cfg2 = {"usds": {"buildings": {"pool": ["plain.usd"]}},
           "asset_scale": 1.0, "asset_root": ""}
    _u, _s, _a, _fp, _yo, meta2 = dd._pool_entries(cfg2, res2, "pool")[0]
    check(meta2["front0"] is None, "no front tag: front0 is None")
    check(meta2["never_corner"] is False, "no tag: never_corner False")


@strict
def test_has_facing_pref():
    print("\n[4c-2] _has_facing_pref: gates on front/blank/place/"
          "never_corner, false only for a fully untagged entry")
    check(dd._has_facing_pref({"place": "any", "front": None,
                               "blank": frozenset(), "never_corner": False})
          is False, "untagged entry: no preference to express")
    check(dd._has_facing_pref({"place": "any", "front": "W",
                               "blank": frozenset(), "never_corner": False}),
          "a front tag alone is enough")
    check(dd._has_facing_pref({"place": "any", "front": None,
                               "blank": frozenset({"N"}), "never_corner": False}),
          "a blank tag alone is enough")
    check(dd._has_facing_pref({"place": "mid", "front": None,
                               "blank": frozenset(), "never_corner": False}),
          "a non-'any' place alone is enough")
    check(dd._has_facing_pref({"place": "any", "front": None,
                               "blank": frozenset(), "never_corner": True}),
          "never_corner alone is enough")


@strict
def test_frontage_len_and_depth_into_block():
    print("\n[4c-3] _frontage_len / _depth_into_block: the tier 2 and "
          "tier 4 helpers")
    block = (0.0, 0.0, 200.0, 50.0)      # wide (200 m) and shallow (50 m)
    check(dd._frontage_len(block, "W") == 50.0, "W frontage = block's Y extent")
    check(dd._frontage_len(block, "E") == 50.0, "E frontage = block's Y extent")
    check(dd._frontage_len(block, "N") == 200.0, "N frontage = block's X extent")
    check(dd._frontage_len(block, "S") == 200.0, "S frontage = block's X extent")
    check(dd._frontage_len(block, None) == 0.0, "no side to measure: 0")

    # The SHORT footprint face should be the one meeting the street.
    check(dd._depth_into_block(28.0, 14.4, frozenset({"W"})) is True,
          "wide-in-X shape (28x14.4) at a W street (runs N-S): the long "
          "axis (X) is perpendicular to it -- depth into the block")
    check(dd._depth_into_block(14.4, 28.0, frozenset({"W"})) is False,
          "the same footprint rotated 90: now the LONG face meets the W "
          "street -- wrong")
    check(dd._depth_into_block(28.0, 14.4, frozenset({"N"})) is False,
          "a N street runs E-W; this shape's long axis (X) runs PARALLEL "
          "to it, not perpendicular")
    check(dd._depth_into_block(14.4, 28.0, frozenset({"N"})) is True,
          "rotated 90, the long axis (Y) is now perpendicular to the N "
          "street")
    check(dd._depth_into_block(10.0, 10.0, frozenset()) is False,
          "no street sides at all: nothing to be perpendicular to")


@strict
def test_yaw_score_tiers():
    print("\n[4c-4] _yaw_score: front-on-street beats not, longest "
          "frontage beats shortest, more streets engaged beats fewer, "
          "depth-into-block is the LAST tiebreak")
    block = (0.0, 0.0, 200.0, 50.0)      # N/S frontage 200 m, W/E 50 m
    meta_w = {"front0": "W"}

    # Tier 1.
    s_on = dd._yaw_score(10.0, 6.0, 0.0, block, meta_w, frozenset({"W"}))
    s_off = dd._yaw_score(10.0, 6.0, 90.0, block, meta_w, frozenset({"N"}))
    check(s_on > s_off,
          f"front-on-street ({s_on}) beats front-not-on-street ({s_off})")

    # Tier 2: SAME asset, front reaches the LONG (N, 200 m) street at one
    # yaw and the SHORT (W, 50 m) street at another (see the module
    # docstring: each of the 4 yaws gives a different world-facing
    # direction, so a single `front0` legitimately competes for both).
    score_w = dd._yaw_score(10.0, 6.0, 0.0, block, meta_w, frozenset({"W"}))
    score_n = dd._yaw_score(10.0, 6.0, 270.0, block, meta_w, frozenset({"N"}))
    check(score_w[1] == 50.0 and score_n[1] == 200.0,
          f"frontage term matches the block's own edge length: "
          f"W={score_w[1]}, N={score_n[1]}")
    check(score_n > score_w,
          "front on the 200 m N street outscores front on the 50 m W one")

    # Tier 3: no front at all (both False/0.0 on tiers 1-2) -- more street
    # sides actually engaged wins.
    meta_none = {"front0": None}
    s_one_side = dd._yaw_score(10.0, 6.0, 0.0, block, meta_none,
                               frozenset({"W"}))
    s_two_sides = dd._yaw_score(10.0, 6.0, 0.0, block, meta_none,
                                frozenset({"W", "S"}))
    check(s_two_sides > s_one_side,
          "tier 3: two street sides engaged beats one, once tiers 1-2 tie")

    # Tier 4: the LAST tiebreak, only visible once 1-3 all tie too.
    s_deep = dd._yaw_score(28.0, 14.4, 0.0, block, meta_none, frozenset({"W"}))
    s_wide = dd._yaw_score(14.4, 28.0, 0.0, block, meta_none, frozenset({"W"}))
    check(s_deep[:3] == s_wide[:3],
          f"tiers 1-3 tied by construction: {s_deep[:3]} vs {s_wide[:3]}")
    check(s_deep > s_wide,
          "tier 4 breaks the tie: depth-into-the-block wins")


@strict
def test_pack_free_front_only_entry_now_oriented():
    print("\n[4c-5] _pack_free: a front-ONLY entry (no `blank:` at all -- "
          "SM_Building_22's real tags) now gets its orientation chosen, "
          "where the old `flip = bool(meta.get('blank'))` gate meant its "
          "180-degree twin was never even generated")
    block = (0.0, 0.0, 100.0, 100.0)
    # Flush against the block's EAST edge only -- the one street side here
    # is E, and this asset's front (W at yaw 0) has to flip to E to face it.
    pool = [_entry("front_only.usd", sx=10.0, sy=6.0, yaw_offset=0.0,
                   place="any", front="W")]
    rng = random.Random(11)
    sky = dd._Skyline({}, rng)
    placed, refused = dd._pack_free(
        (90.0, 40.0, 100.0, 46.0), pool, gap=2.0, min_side=1.0, rng=rng,
        sky=sky, typ={}, block_rect=block, street_tol_m=6.0)
    check(len(placed) == 1, f"expected one placement, got {placed}")
    if placed:
        e, cx, cy, yaw = placed[0]
        check(yaw % 360.0 == 180.0,
              f"expected yaw 180 (front flips to face the E street), "
              f"got {yaw}")
    check(refused == 0, "the correct orientation was always legal")


@strict
def test_pack_free_prefers_longer_street_frontage():
    print("\n[4c-6] _pack_free: at a corner with two viable streets, front "
          "goes on the LONGER one -- house_26_707's case")
    block = (0.0, 0.0, 200.0, 40.0)      # S/N frontage 200 m, W/E 40 m
    pool = [_entry("corner26.usd", sx=10.0, sy=6.0, place="any", front="W")]
    rng = random.Random(7)
    sky = dd._Skyline({}, rng)
    placed, refused = dd._pack_free(
        (0.0, 0.0, 10.0, 10.0), pool, gap=2.0, min_side=1.0, rng=rng,
        sky=sky, typ={}, block_rect=block, street_tol_m=6.0)
    check(len(placed) == 1, f"one candidate placed, got {placed}")
    if placed:
        e, cx, cy, yaw = placed[0]
        check(yaw % 360.0 == 90.0,
              f"expected yaw 90 (front swings onto the 200 m S street, not "
              f"the 40 m W one), got {yaw}")
    check(refused == 0, "something legal was always available")


@strict
def test_pack_free_never_corner():
    print("\n[4c-7] _pack_free: place_never_corner refused at 2+ street "
          "sides, accepted elsewhere -- with no blank tag at all")
    block = (0.0, 0.0, 100.0, 100.0)
    pool = [_entry("nc.usd", sx=10.0, sy=6.0, place="any", never_corner=True)]
    rng = random.Random(5)
    sky = dd._Skyline({}, rng)
    placed, refused = dd._pack_free(
        (0.0, 0.0, 10.0, 6.0), pool, gap=2.0, min_side=1.0, rng=rng, sky=sky,
        typ={}, block_rect=block, street_tol_m=6.0)
    check(placed == [], "never_corner: refused at a 2-side corner")
    check(refused == 1, "never_corner: the refusal is counted")

    rng2 = random.Random(5)
    sky2 = dd._Skyline({}, rng2)
    placed2, refused2 = dd._pack_free(
        (0.0, 45.0, 10.0, 51.0), pool, gap=2.0, min_side=1.0, rng=rng2,
        sky=sky2, typ={}, block_rect=block, street_tol_m=6.0)
    check(len(placed2) == 1, "never_corner: accepted at a single street side")
    check(refused2 == 0, "never_corner: no refusal off a corner")


@strict
def test_pack_free_mid_refused_at_corner_without_blank():
    print("\n[4c-8] _pack_free: place_mid refused at a corner even with NO "
          "`blank:` tag -- the defensive, unconditional version of a rule "
          "that used to rely entirely on `blank:` being set")
    block = (0.0, 0.0, 100.0, 100.0)
    pool = [_entry("mid_no_blank.usd", sx=10.0, sy=6.0, place="mid")]
    rng = random.Random(6)
    sky = dd._Skyline({}, rng)
    placed, refused = dd._pack_free(
        (0.0, 0.0, 10.0, 6.0), pool, gap=2.0, min_side=1.0, rng=rng, sky=sky,
        typ={}, block_rect=block, street_tol_m=6.0)
    check(placed == [], "place_mid, no blank tag: still refused at a corner")
    check(refused == 1, "and counted")


@strict
def test_lay_terrace_end_caps_basic():
    print("\n[4c-9] _lay_terrace_end_caps: compact stock at both alley "
          "ends, each fronting its own short edge, the middle left open")
    rect = (0.0, 0.0, 100.0, 50.0)        # rows run along X, alley cross 20 m
    depth = 15.0
    pool = [_entry("cap.usd", sx=18.0, sy=8.0, place="any", front="W")]
    rng = random.Random(9)
    sky = dd._Skyline({}, rng)
    caps = dd._lay_terrace_end_caps(rect, depth, pool, rng, sky, typ={},
                                    area_band=0.55, max_depth_m=0.0, gap=2.0,
                                    reach=None, street_tol_m=6.0)
    # AT LEAST one cap per end, not EXACTLY one: the alley's cross-width
    # (20 m here) can hold more than one 8 m-facade building side by side
    # along the short edge, and `_pack_free`'s own guillotine step does
    # exactly that -- which is realistic (a real short edge holds more than
    # one small building too) and not a defect this test should chase.
    check(len(caps) >= 2, f"at least one cap at each end, got {len(caps)}")
    if caps:
        yaws = {round(c[3] % 360.0) for c in caps}
        check(yaws <= {0, 180},
              f"only west-facing (0) or east-facing (180) yaws: {yaws}")
        check(0 in yaws and 180 in yaws,
              f"both ends got at least one cap: {yaws}")
        for _e, cx, _cy, _yaw in caps:
            check(cx < 30.0 or cx > 70.0,
                  f"cap at x={cx} sits too close to the block's own middle "
                  f"(30-70) -- the alley must stay open there")


@strict
def test_lay_terrace_end_caps_needs_an_alley():
    print("\n[4c-10] _lay_terrace_end_caps: nothing placed when the block "
          "leaves no alley, or leaves one too narrow for the pool")
    pool = [_entry("cap.usd", sx=18.0, sy=8.0, place="any", front="W")]
    rng = random.Random(9)
    sky = dd._Skyline({}, rng)
    # depth * 2 == the block's own short side: the alley's cross-width is 0.
    tight = dd._lay_terrace_end_caps((0.0, 0.0, 100.0, 30.0), 15.0, pool, rng,
                                     sky, typ={}, area_band=0.55,
                                     max_depth_m=0.0, gap=2.0, reach=None)
    check(tight == [], "no alley at all: nothing placed")

    # Alley present (4 m) but narrower than the pool's shortest side (8 m).
    narrow = dd._lay_terrace_end_caps((0.0, 0.0, 100.0, 34.0), 15.0, pool,
                                      rng, sky, typ={}, area_band=0.55,
                                      max_depth_m=0.0, gap=2.0, reach=None)
    check(narrow == [], "alley narrower than anything in the pool: nothing "
                        "placed")


# ---------------------------------------------------------------------------
# 5. _order_run
# ---------------------------------------------------------------------------

@strict
def test_order_run_mid_in_interior():
    print("\n[5] _order_run: a mid piece never lands at either end")
    mid = _entry("mid.usd", 10, 10, place="mid")
    end_a = _entry("end_a.usd", 10, 10, place="end")
    end_b = _entry("end_b.usd", 10, 10, place="corner")
    for seed in range(20):
        rng = random.Random(seed)
        out = dd._order_run([mid, end_a, end_b], rng)
        check(len(out) == 3, f"seed {seed}: run kept all three ({len(out)})")
        if len(out) == 3:
            check(out[1][0] == "mid.usd",
                  f"seed {seed}: mid landed at index "
                  f"{[e[0] for e in out].index('mid.usd')}, want 1 (interior)")
            check(out[0][0] != "mid.usd" and out[-1][0] != "mid.usd",
                  f"seed {seed}: mid at an end ({[e[0] for e in out]})")


@strict
def test_order_run_drops_unlayable():
    print("\n[5b] _order_run: an unlayable run is dropped, not laid wrong")
    mid1 = _entry("mid1.usd", 10, 10, place="mid")
    mid2 = _entry("mid2.usd", 10, 10, place="mid")
    check(dd._order_run([mid1, mid2], random.Random(1)) == [],
          "two mid pieces, no end stock: run dropped")
    check(dd._order_run([mid1], random.Random(1)) == [],
          "a run of exactly ONE mid: dropped (only `any` may stand alone)")
    any_one = _entry("any.usd", 10, 10, place="any")
    out = dd._order_run([any_one], random.Random(1))
    check(out == [any_one],
          "a run of exactly ONE `any`: kept unchanged, both flanks shown")
    check(dd._order_run([], random.Random(1)) == [], "an empty run: empty out")


@strict
def test_order_run_noop_without_metadata():
    print("\n[5c] _order_run: a no-`mid` run is untouched — order AND rng "
          "state — the no-metadata no-op")
    a = _entry("a.usd", 10, 10, place="any")
    b = _entry("b.usd", 10, 10, place="end")
    c = _entry("c.usd", 10, 10, place="corner")
    chosen = [a, b, c]
    rng = random.Random(42)
    state_before = rng.getstate()
    out = dd._order_run(chosen, rng)
    check(out == chosen,
          f"order preserved exactly: {[e[0] for e in out]} vs "
          f"{[e[0] for e in chosen]}")
    check(rng.getstate() == state_before,
          "rng was never touched — no `mid` in the run means nothing to "
          "decide, so no `rng.shuffle` call, which is what keeps a "
          "metadata-less pool's terrace output byte-identical")


# ---------------------------------------------------------------------------
# 6. _tile_run's `no_repeat` — a HARD guarantee, pinned after a real bug
# ---------------------------------------------------------------------------

@strict
def test_tile_run_no_repeat_stops_rather_than_duplicates():
    print("\n[6] _tile_run(no_repeat=True): a forced repeat ends the run, "
          "it never places one")
    # A never fits (too long); only B does, twice over if allowed. The first
    # shipped version of `no_repeat` fell back to "place the repeat anyway"
    # here and produced Brownstone02-Brownstone02 pairs in 6 of 25 multi-house
    # `downtown` runs — this is that exact shape, minimised.
    a = _entry("A.usd", 10, 20.0)
    b = _entry("B.usd", 10, 6.0)
    out = dd._tile_run(13.0, [a, b], random.Random(1), no_repeat=True)
    bases = [e[0] for e in out]
    check(bases == ["B.usd"],
          f"only B ever fits; the run stops after one B rather than placing "
          f"a second — got {bases}")

    # Flag OFF must reproduce the historical forced-repeat behaviour exactly
    # (the whole point of gating this behind `districts.terrace_no_repeat`
    # is that OFF is byte-identical to before this feature existed).
    out_off = dd._tile_run(13.0, [a, b], random.Random(1), no_repeat=False)
    bases_off = [e[0] for e in out_off]
    check(bases_off == ["B.usd", "B.usd"],
          f"no_repeat=False: the original algorithm DOES place B twice "
          f"(nothing else fits) — got {bases_off}")

    # A pool of one variant is a degenerate no-repeat case: nothing to
    # prefer, but still never a SECOND placement once the first is the only
    # thing that ever fits again.
    only = _entry("ONLY.usd", 10, 6.0)
    out2 = dd._tile_run(13.0, [only], random.Random(2), no_repeat=True)
    check([e[0] for e in out2] == ["ONLY.usd"],
          f"single-variant pool: one placement, then the run stops rather "
          f"than repeating it — got {[e[0] for e in out2]}")


# ---------------------------------------------------------------------------
# 7. _Skyline: tall-building separation
# ---------------------------------------------------------------------------

@strict
def test_tall_ok_measures_footprint_gap_not_centres():
    print("\n[7] _Skyline._tall_ok: footprint-rectangle gap, short neighbours inert")
    cfg = {"tall_min_h_m": 120.0, "tall_min_gap_m": 50.0}
    sky = dd._Skyline(cfg, random.Random(1))
    # An existing 300 m tower, 60x60 footprint, centred at the origin —
    # occupies x in [-30, 30]. A 60x60 candidate's gap along x is
    # |dx| - 60 (30 m half-width each side).
    sky.record(0.0, 0.0, 300.0, (60.0, 60.0))
    check(not sky._tall_ok(60.0, 60.0, 100.0, 0.0),
          "gap 40 m (100 - 60) < 50 m min: too close")
    check(sky._tall_ok(60.0, 60.0, 130.0, 0.0),
          "gap 70 m (130 - 60) >= 50 m min: clear")
    check(not sky._tall_ok(60.0, 60.0, 109.9, 0.0),
          "gap 49.9 m, just under the 50 m minimum: still too close")
    check(sky._tall_ok(60.0, 60.0, 110.0, 0.0),
          "gap exactly 50 m (110 - 60): the minimum itself is acceptable")
    # A SHORT existing neighbour never constrains, at any distance — the
    # knob only ever compares TALL to TALL.
    sky2 = dd._Skyline(cfg, random.Random(1))
    sky2.record(0.0, 0.0, 50.0, (60.0, 60.0))     # below tall_min_h_m
    check(sky2._tall_ok(60.0, 60.0, 61.0, 0.0),
          "a short existing neighbour, even overlapping, never blocks a "
          "tall candidate")
    # A slender tower beside a broad one is not the mirror image of the
    # reverse under a CENTRE rule — confirm the footprint rule treats both
    # orderings the same, which is the whole reason for measuring the gap
    # off sx/sy instead of centre distance.
    sky3 = dd._Skyline(cfg, random.Random(1))
    sky3.record(0.0, 0.0, 300.0, (140.0, 40.0))   # broad, shallow tower
    slender_far = sky3._tall_ok(20.0, 20.0, 130.0, 0.0)     # gap = 130-70-10=50
    sky4 = dd._Skyline(cfg, random.Random(1))
    sky4.record(0.0, 0.0, 300.0, (20.0, 20.0))    # slender tower
    broad_far = sky4._tall_ok(140.0, 40.0, 130.0, 0.0)      # same 50 m gap
    check(slender_far == broad_far == True,
          f"the SAME 50 m footprint gap clears either way round "
          f"(slender-near-broad={slender_far}, broad-near-slender={broad_far})")


@strict
def test_pick_drops_close_tall_prefers_short():
    print("\n[7b] _pick: a too-close tall candidate is dropped, a short one "
         "at the same slot is not")
    cfg = {"tall_min_h_m": 120.0, "tall_min_gap_m": 50.0}
    sky = dd._Skyline(cfg, random.Random(3))
    sky.record(0.0, 0.0, 300.0, (60.0, 60.0))
    tall_close = _entry("tall_close.usd", 60, 60, sz=250.0)
    short_ok = _entry("short_ok.usd", 40, 40, sz=30.0)
    # Corner (70, 0): tall_close's own centre would be (100, 0), 40 m gap —
    # too close. short_ok is short and untouched by the rule regardless of
    # where it lands.
    chosen = sky.choose([tall_close, short_ok], 0.0, 70.0, 0.0)
    check(chosen[0] == "short_ok.usd",
          f"expected the short candidate (tall one dropped), got {chosen[0]}")


@strict
def test_pick_falls_through_when_every_tall_candidate_too_close():
    print("\n[7c] _pick: falls through and counts it when NOTHING survives "
         "the tall filter")
    cfg = {"tall_min_h_m": 120.0, "tall_min_gap_m": 50.0}
    sky = dd._Skyline(cfg, random.Random(4))
    sky.record(0.0, 0.0, 300.0, (60.0, 60.0))
    tall_close = _entry("tall_close.usd", 60, 60, sz=250.0)
    before = sky.tall_fallback
    chosen = sky.choose([tall_close], 0.0, 70.0, 0.0)
    check(chosen[0] == "tall_close.usd",
          "the only candidate is placed anyway — a slot must be filled")
    check(sky.tall_fallback == before + 1,
          f"tall_fallback incremented (before={before}, "
          f"after={sky.tall_fallback})")


@strict
def test_tall_separation_off_by_default():
    print("\n[7d] _Skyline: tall_min_h_m/tall_min_gap_m default OFF — a "
         "too-close tall candidate can still win")
    got_tall = False
    for seed in range(30):
        sky = dd._Skyline({}, random.Random(seed))   # no tall_* keys at all
        sky.record(0.0, 0.0, 300.0, (60.0, 60.0))
        tall_close = _entry("tall_close.usd", 60, 60, sz=250.0)
        short_ok = _entry("short_ok.usd", 40, 40, sz=30.0)
        c = sky.choose([tall_close, short_ok], 0.0, 70.0, 0.0)
        if c[0] == "tall_close.usd":
            got_tall = True
            break
    check(got_tall,
          "with both knobs absent (default 0 = off), the 'too close' tall "
          "candidate is still a live option — the filter never activates")
    check(dd._Skyline({}, random.Random(0)).tall_min_h == 0.0
          and dd._Skyline({}, random.Random(0)).tall_gap == 0.0,
          "tall_min_h_m and tall_min_gap_m both default to 0.0")


# 8. burnability-aware pool selection — "we aren't using some buildings for
# fire at all, we need to account for that in the layout gen" (user,
# 2026-08-31). `_is_unburnable`/`_is_landmark` are plain lookups against the
# checked-in table (`config/harvested/burnability_table.json`, KEYED BY
# TYPOLOGY since `usds.buildings` pool keys are not typology names -- see
# `tools/gen_burnability_table.py`'s 2026-08-31 review-fix docstring) and
# the `_LANDMARK_H_M` cutoff; `_burnable_substitute` finds a same-facing,
# same-footprint-class, burnable pool-mate WITHIN THE SAME TYPOLOGY;
# `_BurnabilityGuard` enforces "at most one unburnable per block" by
# swapping every draw after the first. `dd._burnability_table` is
# monkeypatched to a small two-typology fixture for every test in this
# section so none of it depends on the real, evolving checked-in file --
# restored in a `finally` so a failure here can't leak a stub into a later
# test.
# ---------------------------------------------------------------------------

_FIXTURE_TABLE = {
    "midrise": {"unburnable_a": False, "unburnable_b": False,
               "burnable_match": True, "burnable_wrong_front": True,
               "burnable_too_small": True, "burnable_c": True},
    # a SEPARATE typology with a DIFFERENT verdict for the same basename --
    # pins that the lookup is genuinely keyed by typology, not just by name
    # with a typology parameter nobody reads.
    "tower": {"unburnable_a": True, "tower_only_unburnable": False},
}


def _with_fixture_table(fn):
    """Same idea as `@strict`, layered underneath it: swap in the fixture
    table for the duration of *fn*, restore the real lookup afterward no
    matter what *fn* does."""
    def run(*a, **kw):
        orig = dd._burnability_table
        dd._burnability_table = lambda: {k: dict(v) for k, v in _FIXTURE_TABLE.items()}
        try:
            return fn(*a, **kw)
        finally:
            dd._burnability_table = orig
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


@strict
@_with_fixture_table
def test_is_unburnable_and_landmark():
    print("\n[8a] _is_unburnable/_is_landmark: table lookup by "
          "(typology, basename), unknown typology/name fails OPEN "
          "(burnable)")
    check(dd._is_unburnable("omniverse://x/unburnable_a.usd", "midrise") is True,
          "a (typology, name) the table marks False: unburnable")
    check(dd._is_unburnable("omniverse://x/burnable_c.usdc", "midrise") is False,
          "a (typology, name) the table marks True: burnable")
    check(dd._is_unburnable("omniverse://x/never_measured.usd", "midrise") is False,
          "a name the table has never seen under this typology: fails OPEN")
    check(dd._is_unburnable("omniverse://x/unburnable_a.usd", "rowhouse") is False,
          "the SAME name under a typology the table never audited it for: "
          "fails OPEN -- the lookup is genuinely per-typology")
    check(dd._is_unburnable("omniverse://x/unburnable_a.usd", "tower") is False,
          "the SAME name is unburnable under `midrise` (False, above) but "
          "the fixture marks it BURNABLE (True) under `tower` -- that "
          "typology's own answer wins, not `midrise`'s")
    tall = _entry("tall.usd", 10, 10, sz=300.0)
    short = _entry("short.usd", 10, 10, sz=50.0)
    check(dd._is_landmark(tall) is True, ">232 m: a landmark")
    check(dd._is_landmark(short) is False, "<=232 m: not a landmark")


@strict
@_with_fixture_table
def test_burnable_substitute_matching():
    print("\n[8b] _burnable_substitute: front0/blank0/place/area-band all "
          "have to agree, WITHIN the given typology; None when nothing in "
          "the pool qualifies")
    target = _entry("unburnable_a.usd", 20, 30, place="any", front="E",
                    yaw_offset=180)          # front0 == "W", blank0 == {}
    check(target[5]["front0"] == "W", "sanity: target's front0 is W")

    good = _entry("burnable_match.usd", 22, 28, place="any", front="E",
                 yaw_offset=180)              # same front0, ~similar area
    wrong_front = _entry("burnable_wrong_front.usd", 20, 30, place="any",
                         front="W", yaw_offset=180)   # front0 == "E"
    too_small = _entry("burnable_too_small.usd", 5, 5, place="any",
                       front="E", yaw_offset=180)     # front0 == "W", tiny
    still_unburnable = _entry("unburnable_b.usd", 20, 30, place="any",
                              front="E", yaw_offset=180)

    pool = [wrong_front, too_small, still_unburnable, good]
    sub = dd._burnable_substitute(target, pool, "midrise")
    check(sub is not None and sub[0] == "burnable_match.usd",
          f"picks the one candidate matching front0/place/area, got "
          f"{sub[0] if sub else None}")

    # blank0 must be a SUBSET of the target's — a candidate with MORE blank
    # sides than what already cleared the hard reject at this slot is never
    # an acceptable stand-in, even with a matching front0.
    blanker = _entry("blanker.usd", 20, 30, place="mid", front="E",
                     yaw_offset=180, blank={"N", "S"})
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"], **{"blanker": True})}
    check(dd._burnable_substitute(target, [blanker], "midrise") is None,
          "a candidate with MORE blank sides than the target: rejected")

    # No qualifying candidate at all -> None, not a crash or a bad guess.
    check(dd._burnable_substitute(target, [wrong_front, too_small],
                                  "midrise") is None,
          "no candidate clears every gate: None")
    check(dd._burnable_substitute(target, [], "midrise") is None,
          "empty pool: None")

    # A candidate that is only burnable under a DIFFERENT typology (the
    # fixture's "tower": {"unburnable_a": True}, i.e. burnable there) must
    # not be treated as qualifying for a `midrise` swap just because SOME
    # typology likes it.
    other_typ_only = _entry("unburnable_a.usd", 21, 29, place="any",
                            front="E", yaw_offset=180)
    check(dd._burnable_substitute(target, [other_typ_only], "midrise") is None,
          "a candidate that is only burnable under a DIFFERENT typology "
          "does not qualify here")

    # front0 IS None (no measured front at all -- most of `tower`'s "any"
    # filler stock) has nothing to preserve, so it must accept a candidate
    # WITH a front tag too -- `podium_highrise` (front0 None, real Nucleus
    # size 1,176 m2) finding no substitute in `SM_Building_27` (front0 "W",
    # 1,216 m2) purely because of an exact-None-match requirement was the
    # 2026-08-31 review bug this pins.
    untagged_target = _entry("unburnable_untagged.usd", 26.7, 44.0,
                             place="any")                     # front0 None
    tagged_candidate = _entry("burnable_tagged.usd", 42.5, 28.6, place="any",
                              front="E", yaw_offset=180)       # front0 "W"
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"],
                       **{"unburnable_untagged": False,
                          "burnable_tagged": True})}
    sub2 = dd._burnable_substitute(untagged_target, [tagged_candidate],
                                   "midrise")
    check(sub2 is not None and sub2[0] == "burnable_tagged.usd",
          f"an untagged (front0=None) target accepts a front-TAGGED "
          f"candidate, got {sub2[0] if sub2 else None}")
    # ...but the reverse still respects a REAL preference: a tagged target
    # does not accept an untagged candidate as if front0=None meant
    # "matches anything" symmetrically.
    check(dd._burnable_substitute(tagged_candidate, [untagged_target],
                                  "midrise") is None,
          "a target WITH a measured front does not accept an untagged "
          "(front0=None) candidate -- the relaxation is one-directional")


@strict
@_with_fixture_table
def test_burnable_substitute_rejects_oversized_candidate():
    print("\n[8b-3] _burnable_substitute: a candidate within the area band "
          "but wider or taller than the target (at the SAME reused yaw) is "
          "rejected -- the overlap defect, 2026-08-31 review: `Building_11` "
          "(35.4 x 30.9) swapped for `SM_Building_30` (28.4 x 42.4) at "
          "unchanged (cx, cy, yaw) overlapped a neighbour by 85.6 m2 in "
          "both the seeded host build and the Kit dump it was meant to "
          "match, despite passing the area-ratio check alone")
    target = _entry("unburnable_a.usd", 20.0, 30.0, place="any")  # 600 m2
    taller = _entry("burnable_taller.usd", 20.0, 40.0, place="any")  # 800 m2
    wider = _entry("burnable_wider.usd", 28.0, 22.0, place="any")   # 616 m2
    fits = _entry("burnable_fits.usd", 20.0, 28.0, place="any")     # 560 m2
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"],
                       **{"unburnable_a": False, "burnable_taller": True,
                          "burnable_wider": True, "burnable_fits": True})}

    # No position context (yaw=None): the fit check does not apply at all
    # (unchanged, pre-review behaviour) -- area ratio alone decides, and
    # the FIRST qualifying pool member wins.
    sub_no_ctx = dd._burnable_substitute(target, [taller], "midrise")
    check(sub_no_ctx is not None and sub_no_ctx[0] == "burnable_taller.usd",
          "no yaw given: the fit check is skipped, area ratio alone decides")

    # WITH position context: a candidate taller than the target in Y, or
    # wider in X, is refused even though its area is in-band.
    check(dd._burnable_substitute(target, [taller], "midrise",
                                  cx=0.0, cy=0.0, yaw=0.0) is None,
          "taller than the target (40 > 30): rejected once yaw is given")
    check(dd._burnable_substitute(target, [wider], "midrise",
                                  cx=0.0, cy=0.0, yaw=0.0) is None,
          "wider than the target (28 > 20): rejected once yaw is given")

    # A candidate that fits within BOTH of the target's own extents is
    # still accepted, and wins over the two oversized ones in the same
    # pool.
    pool = [taller, wider, fits]
    sub = dd._burnable_substitute(target, pool, "midrise",
                                  cx=0.0, cy=0.0, yaw=0.0)
    check(sub is not None and sub[0] == "burnable_fits.usd",
          f"the one candidate that fits both extents wins, got "
          f"{sub[0] if sub else None}")

    # Rotation is invariant to this check by construction: `_rotated_wh`
    # swaps BOTH the target's and the candidate's (sx, sy) by the same
    # yaw%180, so "does the candidate's rotated box fit the target's" comes
    # out identically at yaw 90/270 as at yaw 0/180 -- confirmed directly
    # rather than assumed.
    check(dd._burnable_substitute(target, [wider], "midrise",
                                  cx=0.0, cy=0.0, yaw=90.0) is None,
          "still rejected at yaw 90 -- the fit test is rotation-invariant")


@strict
@_with_fixture_table
def test_burnable_substitute_street_aware_for_untagged_target():
    print("\n[8b-2] _burnable_substitute: an untagged target's front0=None "
          "relaxation still checks a TAGGED candidate against the real "
          "street at the reused position -- reusing the original slot's "
          "candidate-yaw for a NEWLY front-tagged substitute was never "
          "validated for that candidate's own front, and blindly trusting "
          "it recreates `house_16_223` on different geometry")
    block = (0.0, 0.0, 100.0, 100.0)      # a 100x100 block
    untagged_target = _entry("unburnable_untagged.usd", 20.0, 20.0,
                             place="any")                      # front0 None
    front_w_ok = _entry("burnable_w.usd", 20.0, 20.0, place="any",
                        front="E", yaw_offset=180)              # front0 "W"
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"],
                       **{"unburnable_untagged": False,
                          "burnable_w": True})}

    # (a) Placed flush against the block's WEST edge, at candidate yaw 0 --
    # front0 "W" rotated by yaw 0 is "W", which IS the street here: safe,
    # the candidate qualifies.
    sub_ok = dd._burnable_substitute(untagged_target, [front_w_ok],
                                     "midrise", cx=10.0, cy=50.0, yaw=0.0,
                                     block_rect=block)
    check(sub_ok is not None and sub_ok[0] == "burnable_w.usd",
          f"front lands on the real street here: accepted, got "
          f"{sub_ok[0] if sub_ok else None}")

    # (b) Placed flush against the block's NORTH edge instead (yaw still
    # 0, front0 "W" -- now WRONG, the only street here is "N"): the exact
    # defect `_substitute_fits_street` exists to catch. No other candidate
    # in the pool -> None, not a silently-wrong swap.
    sub_bad = dd._burnable_substitute(untagged_target, [front_w_ok],
                                      "midrise", cx=50.0, cy=90.0, yaw=0.0,
                                      block_rect=block)
    check(sub_bad is None,
          f"front would NOT land on the real street (N) here: rejected, "
          f"got {sub_bad[0] if sub_bad else None}")

    # (c) Same NORTH-edge position, but no street context at all
    # (`block_rect=None`, the default every OTHER caller in this file's own
    # tests already relies on) -- the check is skipped, matching the
    # pre-street-aware behaviour exactly.
    sub_no_ctx = dd._burnable_substitute(untagged_target, [front_w_ok],
                                         "midrise")
    check(sub_no_ctx is not None and sub_no_ctx[0] == "burnable_w.usd",
          "no position/block_rect given at all: the street re-check is "
          "skipped, not treated as a failure")

    # (d) Interior position (no street on ANY side): nothing to violate,
    # so even the "wrong" front0 direction is fine.
    sub_interior = dd._burnable_substitute(untagged_target, [front_w_ok],
                                           "midrise", cx=50.0, cy=50.0,
                                           yaw=0.0, block_rect=block)
    check(sub_interior is not None and sub_interior[0] == "burnable_w.usd",
          "fully interior (0 street sides): front0 mismatch is moot, "
          "candidate accepted")

    # (e) A target that ALREADY has a matching front0 (front0 "W" itself),
    # reused at a position/yaw where that front DOES reach the street (west
    # edge, as in (a)) -- the same re-check still runs (it is unconditional
    # on the CANDIDATE having a front0, not on whether it differs from the
    # target's), but passes exactly because task 1's own facing fix
    # guarantees a real pipeline never hands this function a target whose
    # already-matching front0 sits somewhere its own street check would
    # fail; a hand-built test that deliberately violates that precondition
    # (a "wrong"-position same-front0 target) is testing an input the real
    # pipeline cannot produce, not a case this function needs to special-
    # case around.
    tagged_target = _entry("unburnable_tagged.usd", 20.0, 20.0, place="any",
                           front="E", yaw_offset=180)           # front0 "W"
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"],
                       **{"unburnable_tagged": False, "burnable_w": True})}
    sub_matched = dd._burnable_substitute(tagged_target, [front_w_ok],
                                          "midrise", cx=10.0, cy=50.0,
                                          yaw=0.0, block_rect=block)
    check(sub_matched is not None and sub_matched[0] == "burnable_w.usd",
          "target's own front0 already equals the candidate's, reused at "
          "a position/yaw the front genuinely reaches: accepted")


@strict
@_with_fixture_table
def test_burnability_guard_one_per_block():
    print("\n[8c] _BurnabilityGuard: first unburnable draw kept, a second "
          "swapped for a burnable pool-mate, landmarks exempt from both "
          "the count and the swap")
    unb1 = _entry("unburnable_a.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    unb2 = _entry("unburnable_b.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    sub = _entry("burnable_match.usd", 22, 28, place="any", front="E",
                yaw_offset=180)
    pool = [sub]

    guard = dd._BurnabilityGuard()
    first = guard.filter_one(unb1, pool, "midrise")
    check(first[0] == "unburnable_a.usd",
          "first unburnable draw in the block: kept as-is")
    second = guard.filter_one(unb2, pool, "midrise")
    check(second[0] == "burnable_match.usd",
          f"second unburnable draw: swapped for the burnable pool-mate, "
          f"got {second[0]}")

    # A landmark never counts against the allowance and is never swapped,
    # even after the block has already used its one "real" unburnable slot.
    landmark = _entry("SM_Building_16.usd", 20, 30, sz=312.0, place="any",
                      front="E", yaw_offset=180)
    dd._burnability_table = lambda: {
        "midrise": dict(_FIXTURE_TABLE["midrise"],
                       **{"SM_Building_16": False})}
    third = guard.filter_one(landmark, pool, "midrise")
    check(third[0] == "SM_Building_16.usd",
          "a >232 m landmark is never swapped, even as the block's second "
          "unburnable-looking draw")
    fourth = guard.filter_one(unb2, pool, "midrise")
    check(fourth[0] == "burnable_match.usd",
          "the landmark did not consume the block's one allowance -- a "
          "REAL second unburnable draw right after it is still swapped")

    # A fresh guard (a new block) resets the allowance.
    fresh = dd._BurnabilityGuard()
    check(fresh.filter_one(unb1, pool, "midrise")[0] == "unburnable_a.usd",
          "a new block's guard starts with its allowance unused")

    # Restore the FULL two-typology fixture -- the landmark sub-test above
    # replaced the table with a `midrise`-only one and never put `tower`
    # back, which would silently fail every `_is_unburnable(..., "tower")`
    # lookup below OPEN rather than testing anything.
    dd._burnability_table = lambda: {k: dict(v) for k, v in _FIXTURE_TABLE.items()}

    # A name that is BURNABLE under `midrise` (not in that sub-table at all
    # -- fails open) but the fixture marks unburnable under `tower`
    # ("tower_only_unburnable": False) -- the guard's verdict follows
    # whichever typology is passed at call time, per block, not a name-only
    # lookup.
    guard_tower = dd._BurnabilityGuard()
    tower_only = _entry("tower_only_unburnable.usd", 20, 30, place="any",
                        front="E", yaw_offset=180)
    check(guard_tower.filter_one(tower_only, [], "midrise") is tower_only,
          "burnable under `midrise` (unknown there, fails open): untouched")
    guard_tower2 = dd._BurnabilityGuard()
    kept_under_tower = guard_tower2.filter_one(tower_only, pool, "tower")
    check(kept_under_tower[0] == "tower_only_unburnable.usd",
          "unburnable under `tower`: still the first draw in THIS fresh "
          "block, so kept as-is regardless (the allowance, not the swap, "
          "is what fires first)")
    check(guard_tower2.filter_one(tower_only, pool, "tower")[0]
          == "burnable_match.usd",
          "a SECOND `tower`-unburnable draw in the same block: swapped, "
          "same as the `midrise` case above")


@strict
@_with_fixture_table
def test_burnability_guard_ignores_burnable_and_no_substitute():
    print("\n[8d] _BurnabilityGuard: a burnable entry always passes "
          "through untouched; an unswappable second draw is kept rather "
          "than dropped")
    guard = dd._BurnabilityGuard()
    ok = _entry("burnable_match.usd", 20, 30, place="any", front="E",
               yaw_offset=180)
    check(guard.filter_one(ok, [], "midrise") is ok,
          "a burnable entry is never touched, empty pool or not")
    check(guard.used is False,
          "a burnable entry never consumes the block's unburnable allowance")

    unb1 = _entry("unburnable_a.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    unb2 = _entry("unburnable_b.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    guard.filter_one(unb1, [], "midrise")          # consumes the allowance
    kept = guard.filter_one(unb2, [], "midrise")   # no pool -- nothing to swap to
    check(kept[0] == "unburnable_b.usd",
          "no substitute available: the unburnable draw is kept rather "
          "than the slot being dropped")

    # A name that is unburnable under `midrise` but the table has never
    # even seen under `rowhouse` -- fails OPEN, so this "unburnable" draw
    # is actually treated as burnable when audited under the wrong
    # typology, and does NOT consume the allowance.
    guard2 = dd._BurnabilityGuard()
    same_name_other_typ = guard2.filter_one(unb1, [], "rowhouse")
    check(same_name_other_typ is unb1,
          "unrecognised under THIS typology: fails open, passed through")
    check(guard2.used is False,
          "and does not consume the block's allowance either")


@strict
@_with_fixture_table
def test_burnability_guard_filter_laid():
    print("\n[8e] _BurnabilityGuard.filter_laid: same rule applied across a "
          "whole block's (entry, cx, cy, yaw) list, positions/yaws "
          "untouched")
    unb1 = _entry("unburnable_a.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    unb2 = _entry("unburnable_b.usd", 20, 30, place="any", front="E",
                 yaw_offset=180)
    # Both extents at or under unb2's own (20 x 30) -- filter_laid forwards
    # real cx/cy/yaw to `_burnable_substitute`, which now also rejects a
    # substitute wider or taller than the slot it is dropped into (the
    # overlap defect, 2026-08-31 review); 22 x 28 traded height for width
    # and used to pass on area ratio alone.
    sub = _entry("burnable_match.usd", 20, 28, place="any", front="E",
                yaw_offset=180)
    laid = [(unb1, 10.0, 20.0, 0.0), (unb2, 50.0, 20.0, 180.0)]
    out = dd._BurnabilityGuard().filter_laid(laid, [sub], "midrise")
    check(len(out) == 2, "filter_laid: same length in and out")
    check(out[0][0][0] == "unburnable_a.usd", "first entry: unchanged")
    check(out[0][1:] == (10.0, 20.0, 0.0), "first entry: position/yaw untouched")
    check(out[1][0][0] == "burnable_match.usd",
          "second entry: swapped for the burnable pool-mate")
    check(out[1][1:] == (50.0, 20.0, 180.0),
          "second entry: position/yaw untouched by the swap")


# ---------------------------------------------------------------------------

def main():
    print("=" * 72)
    print("DISTRICTS FACING (2D, no Isaac)")
    print("=" * 72)
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    broken = []
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print(f"    ERROR {name}: {exc}")
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print(f"  all checks passed ({len(tests)} tests)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
