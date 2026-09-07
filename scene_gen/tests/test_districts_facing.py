#!/usr/bin/env python3
"""
test_districts_facing.py — the blank-wall-off-the-street mechanism, pinned
without Isaac.

`districts.py` carries per-asset `yaw-offset` and `blank:`/`front:`/`place`
metadata into the pool entry (`_pool_entries`), uses it to keep a blank
elevation off a street, and — past that hard reject — to PREFER the yaw that
puts an asset's FRONT on a street over one that merely is not illegal
(`_pack_free`'s `_yaw_score`, `_lay_terrace`'s per-strip filter and
`_order_run`). Five claims are worth pinning, because a wrong compass
rotation or a wrong street-side test looks identical to a correct one until a
render shows a blank wall facing traffic, or a windowless face standing where
a windowed one was available:

  1. `_rot_side`/`_rot_sides` implement the documented CCW convention (+90
     maps E->N, N->W, W->S, S->E) for all four letters at all four multiples
     of 90 degrees.
  2. `_pool_entries` SWAPS a footprint's sx/sy exactly when `yaw-offset` is an
     odd multiple of 90 — the rotation that turns a building a quarter turn
     also turns which extent lands along world X — and derives
     `blank0`/`front0` from the raw tags through that same offset.
  3. `_street_sides` reports 2 sides for a footprint at a block corner, 1 for
     one flush against a single edge, 0 for one in the interior.
  4. `_pack_free` REFUSES a synthetic one-sided (`place_mid`) asset at a
     corner slot (one modelled flank cannot cover two streets) and ACCEPTS the
     same asset where its modelled side lines up with the single street; then
     `_yaw_score` ranks the survivors — front-on-street beats not, the longer
     of two available streets beats the shorter, more streets engaged beats
     fewer, and depth-into-the-block is the last tiebreak.
  5. `_order_run` puts a `mid` entry in the run's interior, drops a run that
     cannot be laid legally (all-mid, or a lone `mid`), and — the case that
     must stay inert for a pool with no metadata — leaves an all-`any` run's
     order AND the caller's `rng` state untouched.

RUNS WITHOUT ISAAC. `districts` imports `scene_generator` at module scope,
which imports `pxr` at module scope; that is stubbed below WHEN THE REAL ONE
IS MISSING, so nothing here needs USD or a GPU. See the stub for why the
"when" matters under pytest.

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
# touches it, so a stub is enough to import this module standalone.
#
# ONLY WHEN THE REAL THING IS ABSENT. `tools/plan_png.py` installs this stub
# unconditionally, which is safe for a one-shot CLI and NOT safe here: under
# pytest every test module shares one `sys.modules`, so stubbing a `pxr` that
# IS installed leaves the stub in place for every test collected afterwards.
# `test_layout_decoupling` then measures nothing, silently falls back to
# constant footprints, and its layout digests move — three extra baseline
# "failures" that are entirely this file's doing. Alphabetically this module
# happens to sort after several that import pxr first, which only means the
# damage depends on collection order. So: try the real import, stub solely on
# ImportError.
try:                                                       # noqa: SIM105
    import pxr  # noqa: F401
except ImportError:
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
# 6. _tile_run_ended — end-capacity as a CONSTRUCTION constraint
# ---------------------------------------------------------------------------

@strict
def test_tile_run_ended_brackets_with_end_capable_stock():
    print("\n[6] _tile_run_ended: both ends end-capable, interior `mid`")
    # 2 end-capable, 3 `mid` — the shape of a real terrace pool, where most
    # of the stock is one-sided.
    pool = ([_entry("any1", 12.0, 10.0, place="any"),
             _entry("end1", 12.0, 10.0, place="end")]
            + [_entry(f"mid{i}", 12.0, 10.0, place="mid") for i in range(3)])
    for seed in range(30):
        out = dd._tile_run_ended(50.0, pool, random.Random(seed))
        places = [e[5]["place"] for e in out]
        if len(places) == 1:
            check(places[0] == "any",
                  f"seed {seed}: a lone piece must be `any`, got {places}")
            continue
        check(places[0] != "mid" and places[-1] != "mid",
              f"seed {seed}: neither end may be `mid`, got {places}")
        check(sum(e[3]["sy"] for e in out) <= 50.0 + 1e-6,
              f"seed {seed}: run overruns its length budget: {places}")
    # The whole point: it reaches a REAL row, not just a bare bracket.
    lens = [len(dd._tile_run_ended(50.0, pool, random.Random(s)))
            for s in range(30)]
    check(max(lens) >= 3,
          f"a 50 m run of 10 m stock should reach 3+ pieces, max was {max(lens)}")
    check(all(dd._order_run(dd._tile_run_ended(50.0, pool, random.Random(s)),
                            random.Random(s)) != []
              for s in range(30)),
          "every run it builds survives `_order_run` — that is the contract")


@strict
def test_tile_run_ended_refuses_what_cannot_be_bracketed():
    print("\n[6b] _tile_run_ended: no legal bracket -> nothing, not a "
          "blank flank")
    only_mid = [_entry("m", 12.0, 10.0, place="mid")]
    check(dd._tile_run_ended(50.0, only_mid, random.Random(0)) == [],
          "an all-`mid` pool has nothing that may close a run: empty")
    # Room for one end-capable piece but not two, and no `any` to stand alone.
    pool = [_entry("e", 12.0, 30.0, place="end"),
            _entry("m", 12.0, 10.0, place="mid")]
    check(dd._tile_run_ended(35.0, pool, random.Random(0)) == [],
          "cannot bracket two ends in the length, and no `any`: empty")
    # The same length WITH an `any` falls back to the lone piece.
    out = dd._tile_run_ended(35.0, pool + [_entry("a", 12.0, 30.0)],
                             random.Random(0))
    check([e[5]["place"] for e in out] == ["any"],
          f"falls back to a lone `any`, got {[e[0] for e in out]}")


@strict
def test_face_runs_picks_the_tiler_by_pool():
    print("\n[6c] _face_runs: `_tile_run_ended` only when the pool has a "
          "`mid` — an untagged pool keeps the original tiler")
    calls = []
    orig_plain, orig_ended = dd._tile_run, dd._tile_run_ended
    dd._tile_run = lambda *a, **k: (calls.append("plain"),
                                    orig_plain(*a, **k))[1]
    dd._tile_run_ended = lambda *a, **k: (calls.append("ended"),
                                          orig_ended(*a, **k))[1]
    try:
        untagged = [_entry("a", 12.0, 10.0), _entry("b", 12.0, 10.0)]
        dd._face_runs(60.0, untagged, random.Random(0), 45.0, 0.7, 3.0, 9.0)
        check(calls and set(calls) == {"plain"},
              f"untagged pool must stay on `_tile_run`, saw {set(calls)}")
        calls.clear()
        tagged = untagged + [_entry("m", 12.0, 10.0, place="mid")]
        dd._face_runs(60.0, tagged, random.Random(0), 45.0, 0.7, 3.0, 9.0)
        check(calls and set(calls) == {"ended"},
              f"a pool with a `mid` must use `_tile_run_ended`, "
              f"saw {set(calls)}")
    finally:
        dd._tile_run, dd._tile_run_ended = orig_plain, orig_ended


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
