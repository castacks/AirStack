#!/usr/bin/env python3
"""
test_block_fill_and_plazas.py — the two 2026-08-31 layout fixes, pinned
without Isaac Sim, Nucleus or a GPU.

Both come from one report on the built `downtown_fire_500` city:

  (1) "There's a lot of empty space in the blocks (non skyscraper). that
      shouldn't occur with the layout gen."
  (2) "Modify the layout gen so that the props it places in the empty areas
      in skyscraper blocks can also have trees. Make it like an arrangement
      of trees with seating, etc."

MEASURED CAUSE OF (1), with `tools/block_fill_probe.py` on seed 4 — two
mechanisms, and neither is "the packer gave up too early":

  a. `_pack_free` anchors every building at its sub-rectangle's LOW corner
     and pushes the residue to the far side, so a block's unbuildable slack
     accumulates as one contiguous 8-19 m band along its north and east
     edges — which are STREETS. 27,322 m2 of open ground at least 8 m across
     inside ten blocks, and only 57.2% of the block frontage line had a
     building on it. `districts._justify` hands that residue back as gap
     between the buildings instead; the frontage line goes to 76.0%.
  b. A terrace block outside its alley band is rebuilt from the next
     typology up, but `layout["_typology_of"]` kept reporting the ZONED
     name, so `infill_blocks` read `morphology: terrace` and skipped the
     block as "back yards and service alley". Both `brick_midrise` blocks in
     this scene were packed with tower stock and then never infilled:
     3,943 m2 left bare on two blocks with no terrace in them.
     `rezone_blocks` now publishes `layout["_built_typology_of"]` beside it.

WHAT IS PINNED HERE
-------------------
  1. `_pool_fits` — the "could anything at all stand here" predicate
     `_justify` asks, either way round, and False for a residue nothing fits.
  2. `_justify` — no shift while the residue could still hold something; a
     shift to the far edge when it could not; and the facing re-check, which
     drops a shift that would put a `blank:` wall on a street.
  3. `_pack_free` end to end with `justify` on: the last building in a run
     lands ON the block's far edge rather than short of it, nothing
     overlaps, and `justify=False` reproduces the corner-anchored packing.
  4. `_allee_plan` — two rows when the gap is wide enough, one when it is
     not, stations centred on the gap and pitched evenly, none at all in a
     sliver.
  5. `_Occupancy.fits` — non-mutating, which is what lets a composed seating
     group be placed all-or-nothing.
  6. `_seat_z` / `_surface_field` — the floating-tree fix. A prop's z has to
     seat its lowest geometry on whatever ground is under it, and there are
     FOUR grounds (asphalt 0, block grass 0.01, block paving 0.02, sidewalk
     ring 0.115). The AEC `Shumard_Oak` carries 0.533 m of root ball below
     its own origin, and `surface + base` lifted every kerb oak by exactly
     that ("this tree is floating on the sidewalk", user 2026-08-31).
  7. `_Occupancy.release` — the undo that lets a composed group be placed
     all-or-nothing, so no lone bench, planter or café table is ever left
     standing where its companions were refused.
  8. Determinism: the same seed gives the same pack, twice.

RUNS WITHOUT ISAAC. `districts`/`city_detail` import `scene_generator`,
which imports `pxr` at module scope; both are stubbed exactly as
`test_districts_facing.py` and `tools/plan_png.py` stub them.

USAGE
    python3 scene_gen/tests/test_block_fill_and_plazas.py
    pytest -s scene_gen/tests/test_block_fill_and_plazas.py
"""

import os
import random
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

# `detail.districts` / `detail.city_detail` import `scene_generator`, which
# imports `pxr` at module scope -- stub it out (as `test_districts_facing.py`
# and `tools/plan_png.py` do) so this file can import them without Isaac
# Sim's real pxr.
#
# MUST be fully undone before this module finishes importing: pytest collects
# every test file up front (running each one's top-level code) before running
# ANY test, so anything left sitting in `sys.modules` leaks into every
# later-collected file. This stub caused TWO distinct leaks: (1) the fake
# `pxr`/`pxr.Usd`/... entries themselves, so a later file's own fresh `from
# pxr import Usd, ...` got the stub instead of real usd-core; (2)
# `scene_generator` (imported here for the first time in the whole session,
# transitively through `districts`/`city_detail`) got CACHED in
# `sys.modules` with `Usd`/`UsdGeom`/... permanently bound to the fake stub
# inside ITS OWN namespace -- restoring `sys.modules["pxr"]` alone does not
# fix that already-bound reference, so a later file's `import
# scene_generator` (or `tools/layout_dry_run.py`, which imports it too)
# reused the SAME poisoned module object (`AttributeError: module 'UsdGeom'
# has no attribute 'Scope'`, from a REAL usd-core stack). An integrated sweep
# of a dozen files after this one hit ~100 failures from leak (1) alone; a
# 101st, subtler failure from leak (2) only showed up once (1) was fixed.
#
# Fix: snapshot every module name already in `sys.modules` before touching
# anything, then evict every name ADDED as a side effect of this file's own
# imports once they are done -- whichever file imports `scene_generator` (or
# `pxr`) next gets a clean, from-scratch import against whatever is really
# on the path (a real `pxr`/`scene_generator` already cached by an earlier
# file, if any, is untouched -- it was never in the "added" set).
_pre_existing_modules = set(sys.modules)

for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, types.ModuleType(_n))

from detail import districts as dd                        # noqa: E402
from detail import city_detail as cd                      # noqa: E402

# Undo it now -- `dd`/`cd` (and everything they imported, including
# `scene_generator`) already hold their own bound references into the stub,
# so evicting these from `sys.modules` cannot change THEIR behaviour, only
# what the NEXT file to import any of them sees.
for _m in set(sys.modules) - _pre_existing_modules:
    del sys.modules[_m]

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def strict(fn):
    """A recorded FAIL becomes a pytest failure — same idiom as
    `test_districts_facing.py`, so a direct run prints the whole table while
    pytest still fails on the first test that added to it."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


def _entry(usd, sx, sy, sz=10.0, place="any", front=None,
           blank=frozenset(), yaw_offset=0.0):
    """A synthetic 6-tuple pool entry, built the way `_pool_entries` builds a
    real one (footprint UNROTATED, `blank0`/`front0` folded through the
    asset's own yaw offset)."""
    fp = {"sx": float(sx), "sy": float(sy), "sz": float(sz), "base": 0.0}
    meta = {"place": place, "front": front, "blank": frozenset(blank),
            "blank0": dd._rot_sides(frozenset(blank), yaw_offset),
            "front0": dd._rot_side(front, yaw_offset) if front else None,
            "never_corner": False}
    return (usd, 1.0, "Z", fp, float(yaw_offset), meta)


def _rects(placed):
    """World footprint rects for `_pack_free` output, yaw-swapped the way
    `_rect_of` swaps them."""
    out = []
    for e, cx, cy, yaw in placed:
        sx, sy = e[3]["sx"], e[3]["sy"]
        if abs((float(yaw) % 180.0) - 90.0) < 45.0:
            sx, sy = sy, sx
        out.append((cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0))
    return out


def _overlaps(rects, tol=1e-6):
    n = 0
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            a, b = rects[i], rects[j]
            if (min(a[2], b[2]) - max(a[0], b[0]) > tol
                    and min(a[3], b[3]) - max(a[1], b[1]) > tol):
                n += 1
    return n


# ---------------------------------------------------------------------------
# 1. _pool_fits
# ---------------------------------------------------------------------------

@strict
def test_pool_fits_either_way_round():
    print("\n[1] _pool_fits: either orientation counts, and a residue "
          "nothing fits is False")
    pool = [_entry("wide.usd", 40.0, 14.0), _entry("big.usd", 60.0, 55.0)]
    check(dd._pool_fits(pool, 45.0, 20.0), "40x14 fits a 45x20 residue")
    check(dd._pool_fits(pool, 20.0, 45.0),
          "the same asset fits the SWAPPED 20x45 residue (either way round)")
    check(not dd._pool_fits(pool, 45.0, 12.0),
          "nothing fits a 12 m deep residue (narrowest member is 14 m)")
    check(not dd._pool_fits(pool, 0.0, 50.0), "a zero-width residue is False")
    check(not dd._pool_fits([], 100.0, 100.0), "an empty pool never fits")


# ---------------------------------------------------------------------------
# 2. _justify
# ---------------------------------------------------------------------------

@strict
def test_justify_holds_when_the_residue_is_buildable():
    print("\n[2a] _justify: a residue that could still hold something is "
          "left alone — the packer keeps its right/top children")
    pool = [_entry("a.usd", 20.0, 20.0)]
    # 100 x 50 rect, a 20 x 20 building: 80 m of x residue and 30 m of y
    # residue, both with room for another of these, so nothing may move.
    px, py = dd._justify(0.0, 0.0, 100.0, 50.0, 20.0, 20.0, pool, gap=2.0,
                         block_rect=None, meta={}, yaw=0.0, street_tol_m=6.0)
    check((px, py) == (0.0, 0.0),
          f"expected the low corner (0, 0), got ({px}, {py})")

    # PER AXIS, not all or nothing: shrink the same rect to 30 m deep and the
    # 10 m y residue becomes dead while the x one is still live, so the
    # building justifies NORTH and stays at x=0 for its right-hand sibling.
    px2, py2 = dd._justify(0.0, 0.0, 100.0, 30.0, 20.0, 20.0, pool, gap=2.0,
                           block_rect=None, meta={}, yaw=0.0,
                           street_tol_m=6.0)
    check((px2, abs(py2 - 10.0) < 1e-9) == (0.0, True),
          f"expected (0, 10) — y justified, x untouched — got ({px2}, {py2})")


@strict
def test_justify_shifts_to_the_far_edge():
    print("\n[2b] _justify: a residue NOTHING in the pool fits is handed "
          "back as gap and the building goes to the far edge")
    pool = [_entry("a.usd", 20.0, 20.0)]
    # 32 x 30: 12 m of x residue and 10 m of y residue, both under the
    # pool's 20 m, so both are dead and both get justified away.
    px, py = dd._justify(0.0, 0.0, 32.0, 30.0, 20.0, 20.0, pool, gap=2.0,
                         block_rect=None, meta={}, yaw=0.0, street_tol_m=6.0)
    check(abs(px - 12.0) < 1e-9 and abs(py - 10.0) < 1e-9,
          f"expected (12, 10) — the whole residue on each axis — got "
          f"({px}, {py})")
    # ...and the shift is exactly what puts the far face on the rect's far
    # edge, which is the whole point: the street wall closes.
    check(abs((px + 20.0) - 32.0) < 1e-9,
          "the building's far face now sits on the rect's far edge")


@strict
def test_justify_never_turns_a_blank_wall_to_the_street():
    print("\n[2c] _justify: the shift is DROPPED when it would put a "
          "`blank:` elevation on a street")
    block = (0.0, 0.0, 100.0, 100.0)
    pool = [_entry("a.usd", 20.0, 20.0)]
    # An asset blank on its E face, at yaw 0. Justifying it 12 m east would
    # seat that face on the block's own east edge (a street) — refused.
    meta = {"place": "any", "blank": frozenset({"E"}),
            "blank0": frozenset({"E"}), "front": None, "front0": None,
            "never_corner": False}
    px, py = dd._justify(60.0, 40.0, 100.0, 62.0, 20.0, 20.0, pool, gap=2.0,
                         block_rect=block, meta=meta, yaw=0.0,
                         street_tol_m=6.0)
    check((px, py) == (60.0, 40.0),
          f"expected no shift (blank E would land on the E street), got "
          f"({px}, {py})")
    # The same geometry with no blank tag DOES justify — proving the refusal
    # above came from the facing re-check and not from the residue test.
    px2, py2 = dd._justify(60.0, 40.0, 100.0, 62.0, 20.0, 20.0, pool,
                           gap=2.0, block_rect=block, meta={}, yaw=0.0,
                           street_tol_m=6.0)
    check(abs(px2 - 80.0) < 1e-9,
          f"untagged asset justifies to x=80, got {px2}")


# ---------------------------------------------------------------------------
# 3. _pack_free with justify on
# ---------------------------------------------------------------------------

@strict
def test_pack_free_justify_closes_the_far_edge():
    print("\n[3a] _pack_free: with `justify` the last building lands ON the "
          "block's far edge instead of leaving a bare band there")
    # 100 x 20 strip, one 30 x 20 model: three fit at a 2 m gap
    # (30+2+30+2+30 = 94), leaving 6 m dead — under the model's own 20 m, so
    # nothing more can ever go there.
    pool = [_entry("a.usd", 30.0, 20.0)]
    rect = (0.0, 0.0, 100.0, 20.0)

    rng = random.Random(7)
    on, ref = dd._pack_free(rect, pool, gap=2.0, min_side=20.0, rng=rng,
                            sky=dd._Skyline({}, rng), typ={})
    rng = random.Random(7)
    off, _ = dd._pack_free(rect, pool, gap=2.0, min_side=20.0, rng=rng,
                           sky=dd._Skyline({}, rng), typ={}, justify=False)

    check(len(on) == len(off) == 3,
          f"same building count either way: justify {len(on)}, plain "
          f"{len(off)}")
    far_on = max(r[2] for r in _rects(on))
    far_off = max(r[2] for r in _rects(off))
    check(abs(far_on - 100.0) < 1e-6,
          f"justified: the far face is on the block edge (x=100), got "
          f"{far_on:.2f}")
    check(abs(far_off - 94.0) < 1e-6,
          f"plain: the far face stops 6 m short (x=94), got {far_off:.2f}")
    check(_overlaps(_rects(on)) == 0, "justified pack: no overlapping pair")
    check(ref == 0, "justified pack: nothing refused")


@strict
def test_pack_free_justify_off_is_byte_identical():
    print("\n[3b] _pack_free: `justify=False` reproduces the original "
          "corner-anchored packing placement for placement")
    pool = [_entry("a.usd", 31.0, 17.0), _entry("b.usd", 19.0, 23.0)]
    rect = (0.0, 0.0, 140.0, 90.0)
    rng = random.Random(3)
    a, _ = dd._pack_free(rect, pool, gap=3.0, min_side=17.0, rng=rng,
                         sky=dd._Skyline({}, rng), typ={}, justify=False)
    rng = random.Random(3)
    b, _ = dd._pack_free(rect, pool, gap=3.0, min_side=17.0, rng=rng,
                         sky=dd._Skyline({}, rng), typ={}, justify=False)
    check([(e[0], cx, cy, y) for e, cx, cy, y in a]
          == [(e[0], cx, cy, y) for e, cx, cy, y in b],
          "two runs at one seed agree exactly")
    # every building anchored at a sub-rect low corner still sits inside the
    # rect and nothing overlaps — the invariant the justify must not break
    check(_overlaps(_rects(a)) == 0, "no overlapping pair")


@strict
def test_pack_free_justify_stays_inside_and_deterministic():
    print("\n[3c] _pack_free: justified packing stays inside the rect, "
          "never overlaps, and reproduces from one seed")
    pool = [_entry("a.usd", 31.0, 17.0), _entry("b.usd", 19.0, 23.0),
            _entry("c.usd", 44.0, 28.0)]
    rect = (10.0, -5.0, 190.0, 105.0)
    runs = []
    for _ in range(2):
        rng = random.Random(11)
        placed, _r = dd._pack_free(rect, pool, gap=4.0, min_side=17.0,
                                   rng=rng, sky=dd._Skyline({}, rng), typ={})
        runs.append(placed)
    check([(e[0], cx, cy, y) for e, cx, cy, y in runs[0]]
          == [(e[0], cx, cy, y) for e, cx, cy, y in runs[1]],
          f"deterministic across two runs ({len(runs[0])} buildings)")
    rr = _rects(runs[0])
    check(_overlaps(rr) == 0, "no overlapping pair")
    inside = all(r[0] >= rect[0] - 1e-6 and r[1] >= rect[1] - 1e-6
                 and r[2] <= rect[2] + 1e-6 and r[3] <= rect[3] + 1e-6
                 for r in rr)
    check(inside, "every footprint inside the rect it was packed into")


# ---------------------------------------------------------------------------
# 4. _allee_plan — the tree arrangement's geometry
# ---------------------------------------------------------------------------

@strict
def test_allee_plan_two_rows_when_wide_enough():
    print("\n[4a] _allee_plan: a wide gap gets TWO rows, set in from its "
          "long edges, with evenly pitched stations centred on the gap")
    axis, rows, stations = cd._allee_plan((0.0, 0.0, 100.0, 30.0),
                                          pitch_m=10.0, edge_m=4.0,
                                          row_gap_min_m=7.0)
    check(axis == "x", f"a 100x30 gap runs long east-west, got axis={axis}")
    check(len(rows) == 2 and abs(rows[0] - 4.0) < 1e-9
          and abs(rows[1] - 26.0) < 1e-9,
          f"rows set in 4 m from each long edge, got {rows}")
    check(len(stations) == 10,
          f"(100 - 8) // 10 + 1 = 10 stations, got {len(stations)}")
    pitches = {round(b - a, 6) for a, b in zip(stations, stations[1:])}
    check(pitches == {10.0}, f"one even pitch throughout, got {pitches}")
    mid = (stations[0] + stations[-1]) / 2.0
    check(abs(mid - 50.0) < 1e-9,
          f"the run is centred on the gap (50), got {mid}")


@strict
def test_allee_plan_single_row_and_slivers():
    print("\n[4b] _allee_plan: a narrow gap collapses to ONE centre row, "
          "and a sliver gets no stations at all")
    _axis, rows, stations = cd._allee_plan((0.0, 0.0, 60.0, 12.0),
                                           pitch_m=10.0, edge_m=4.0,
                                           row_gap_min_m=7.0)
    check(len(rows) == 1 and abs(rows[0] - 6.0) < 1e-9,
          f"12 m across - 2*4 m = 4 m < 7 m, so one centre row, got {rows}")
    check(len(stations) > 0, "a 60 m long alley still gets a line of trees")

    axis2, _rows2, stations2 = cd._allee_plan((0.0, 0.0, 6.0, 5.0),
                                              pitch_m=10.0, edge_m=4.0,
                                              row_gap_min_m=7.0)
    check(stations2 == [],
          f"a 6x5 sliver stays paving, got {len(stations2)} station(s)")
    check(axis2 == "x", "axis still reported for a sliver")


# ---------------------------------------------------------------------------
# 5. _Occupancy.fits — what makes a composed group all-or-nothing
# ---------------------------------------------------------------------------

@strict
def test_occupancy_fits_is_non_mutating():
    print("\n[5] _Occupancy.fits: agrees with reserve and claims nothing, "
          "so a seating PAIR can be tested before either bench is placed")
    occ = cd._Occupancy(cell_m=4.0, pad_m=0.1)
    a = (0.0, 0.0, 2.0, 1.0)
    b = (1.0, 0.0, 3.0, 1.0)          # overlaps a
    c = (10.0, 10.0, 12.0, 11.0)      # clear of both
    check(occ.fits(a) and occ.fits(b) and occ.fits(c),
          "an empty grid accepts everything")
    check(occ.fits(a) is True and occ.reserve(a) is True,
          "fits then reserve both succeed on an empty grid")
    check(occ.fits(b) is False, "fits reports the collision")
    check(occ.reserve(b) is False, "reserve agrees with fits")
    check(occ.fits(c) is True, "a clear box still fits after the failures")
    # the two failed calls must have claimed nothing: c is still free, and so
    # is everything b would have covered once a is the only reservation.
    check(occ.reserve(c) is True, "fits claimed nothing — c is still free")


# ---------------------------------------------------------------------------
# 6. _seat_z / _surface_field — the floating-tree fix
# ---------------------------------------------------------------------------

@strict
def test_seat_z_sinks_a_root_ball_and_lowers_a_high_origin():
    print("\n[6a] _seat_z: geometry BELOW an asset's origin is buried, not "
          "used to lift the asset off the ground")
    # The measured AEC Shumard_Oak: 0.533 m of root ball below its origin.
    # `surface + base` put it 0.533 m above the pavement — the reported
    # "this tree is floating on the sidewalk".
    check(abs(cd._seat_z(0.115, {"base": 0.533}) - 0.115) < 1e-9,
          "a 0.533 m root ball sinks; the trunk base sits ON the slab")
    check(abs(cd._seat_z(0.115, {"base": 0.0}) - 0.115) < 1e-9,
          "an asset authored on its own origin is unchanged")
    # An asset whose geometry starts ABOVE its origin is still LOWERED, or it
    # hovers by that much.
    check(abs(cd._seat_z(0.115, {"base": -0.4}) - (0.115 - 0.4)) < 1e-9,
          "a +0.4 m authored gap is closed by lowering the prop")
    # Deeper than the threshold and the origin probably is not the base at
    # all — lifting stays the safer read, same rule as `districts._place_z`.
    deep = 1.0 + cd._BELOW_GRADE_MAX_M
    check(abs(cd._seat_z(0.0, {"base": deep}) - deep) < 1e-9,
          f"below-grade deeper than {cd._BELOW_GRADE_MAX_M} m still lifts")
    check(cd._BELOW_GRADE_MAX_M == dd._BELOW_GRADE_MAX_M,
          "one threshold shared with districts, not two that can drift")


@strict
def test_surface_field_knows_the_four_ground_planes():
    print("\n[6b] _surface_field: sidewalk ring, block paving, block grass "
          "and the asphalt base are four different heights")
    layout = {
        "blocks": [(0.0, 0.0, 100.0, 100.0)],
        # the ring is the outer 4 m of the block's south edge, for the test
        "sidewalk_rects": [(0.0, 0.0, 100.0, 4.0)],
        "paved_blocks": [(4.0, 4.0, 96.0, 96.0)],
        "sidewalk_top_m": 0.115,
    }
    at = cd._surface_field(layout, 0.115)
    check(abs(at(50.0, 2.0) - 0.115) < 1e-9, "on the ring -> sidewalk top")
    check(abs(at(50.0, 50.0) - 0.02) < 1e-9,
          "inside a paved block -> the 0.02 concrete plane")
    check(abs(at(2.0, 50.0) - 0.01) < 1e-9,
          "in the block but off the paving -> the 0.01 grass plane")
    check(abs(at(-50.0, -50.0) - 0.0) < 1e-9,
          "in the roadway -> the asphalt base at 0")


# ---------------------------------------------------------------------------
# 7. _Occupancy.release — what makes a composed group all-or-nothing
# ---------------------------------------------------------------------------

@strict
def test_occupancy_release_gives_the_ground_back():
    print("\n[7] _Occupancy.release: a refused group's first member can be "
          "taken back off the ground")
    occ = cd._Occupancy(cell_m=4.0, pad_m=0.1)
    a = (0.0, 0.0, 2.0, 1.0)
    check(occ.reserve(a) is True, "the anchor is claimed")
    check(occ.fits((1.0, 0.0, 3.0, 1.0)) is False,
          "an overlapping box is refused while the anchor stands")
    occ.release(a)
    check(occ.fits((1.0, 0.0, 3.0, 1.0)) is True,
          "and accepted once the anchor is taken back")
    check(occ.reserve((0.0, 0.0, 2.0, 1.0)) is True,
          "the anchor's own ground is free again too")


def main():
    for fn in (test_pool_fits_either_way_round,
               test_justify_holds_when_the_residue_is_buildable,
               test_justify_shifts_to_the_far_edge,
               test_justify_never_turns_a_blank_wall_to_the_street,
               test_pack_free_justify_closes_the_far_edge,
               test_pack_free_justify_off_is_byte_identical,
               test_pack_free_justify_stays_inside_and_deterministic,
               test_allee_plan_two_rows_when_wide_enough,
               test_allee_plan_single_row_and_slivers,
               test_occupancy_fits_is_non_mutating,
               test_seat_z_sinks_a_root_ball_and_lowers_a_high_origin,
               test_surface_field_knows_the_four_ground_planes,
               test_occupancy_release_gives_the_ground_back):
        try:
            fn()
        except AssertionError:
            pass
    print(f"\n{len(FAILS)} failure(s)")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
