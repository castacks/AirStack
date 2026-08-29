#!/usr/bin/env python3
"""
test_districts_facing.py — the blank-wall-off-the-street mechanism, pinned
without Isaac.

`districts.py` carries per-asset `yaw-offset` and `blank:`/`place` metadata
into the pool entry (`_pool_entries`), uses it to keep a blank elevation off
a street (`_pack_free`'s facing filter, `_lay_terrace`'s per-strip filter and
`_order_run`), and to suppress a model repeating too close to itself. Five
claims are worth pinning here, because a wrong compass rotation or a wrong
street-side test looks identical to a correct one until a render shows a
blank wall facing traffic:

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
     side lines up with the street.
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
          front=None, blank=frozenset()):
    """A synthetic 6-tuple pool entry, built the same way `_pool_entries`
    builds a real one — footprint UNROTATED (as the resolver would report
    it), `blank0` derived from `blank` and `yaw_offset` via `_rot_sides`,
    exactly what `_pool_entries` does."""
    fp = {"sx": float(sx), "sy": float(sy), "sz": float(sz), "base": float(base)}
    meta = {"place": place, "front": front, "blank": frozenset(blank),
            "blank0": dd._rot_sides(frozenset(blank), yaw_offset)}
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
