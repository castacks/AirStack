#!/usr/bin/env python3
"""What encloses a back yard, and what may stand in one.

`suburb_scene._yard_enclosed` is the predicate the whole enclosure rework turns
on. `build_placements` gates its all-or-nothing fence sweep on it, the seating
gate in `suburb_yardplan` asks the same question of trees, and
`tools/fence_check.py` asserts both. One function, three callers, and nothing
tested it directly -- so this does, against fixtures small enough to reason
about by hand:

  [1] the predicate      fenced / screened / open / PARTIAL / gardenless, and
                         the one that matters most is `partial`: a yard fenced
                         down both sides with the rear left open must FAIL.
                         A predicate that averaged its three edges scores that
                         0.67 and passes it, and 0.67 is a garden you walk into
                         off the block behind.
  [2] never pooled       an edge half covered by fence and half by trees must
                         FAIL, even though SOMETHING stands along all of it.
                         That is not an enclosed yard, it is a gappy fence next
                         to some trees -- and `_edge_cover` will happily answer
                         1.00 for it if you hand it both source kinds at once,
                         which is exactly the call `_yard_enclosed` must not
                         make. Asserted both ways round, because the bug is
                         invisible unless you show the pooled answer differing.
  [3] the mirrored bar   `suburb_yardplan` COPIES `_YARD_SCREEN_COVER`,
                         `_YARD_SCREEN_SLACK_M` and `_YARD_SAMPLE_M` into its
                         own `_SCREEN_COVER` / `_SCREEN_SLACK_M` /
                         `_EDGE_SAMPLE_M`, because importing back would be a
                         cycle (`suburb_scene` imports that module at module
                         scope). Two constants that must stay equal with
                         nothing enforcing it is a drift hazard and not a
                         style question: move the bar in `suburb_scene` alone
                         and the seating gate silently starts screening yards
                         the checker calls open. This reads both live values
                         and compares them.
  [4] attribution        a prop belongs to the lot whose LOCAL FRAME
                         contains it, not to the nearest house centre. Two lot
                         rectangles genuinely overlap in a block corner and a
                         bench at the back of a deep garden is nearer the
                         NEIGHBOUR's house than its own, so nearest-centre
                         reports furniture in a fenced yard as standing in an
                         open one -- which it did, twice, during the seating
                         work.
  [5] no seating in an   over a REAL suburb -- `parcel_blocks`, then
      open yard          `build_placements`, then `yp.plan` -- rather than over
                         a fixture, because the defect was never in the
                         predicate. It was in the ORDER: the patio slot ran
                         before the canopy slot, so a yard could not be
                         screened by trees that had not been planted yet, and
                         294 of 358 props on seed 3 stood in open ground.

PROVING EACH ONE CATCHES THE OLD BEHAVIOUR. The shipped code cannot be reverted
from inside a test, so every case here is stated as a failing INPUT rather than
as a passing run: `partial` and `pooled` are the fixtures the wrong predicate
passes, [4] is built so that the nearest-centre rule gets it wrong, and [5]
ends by placing a bench in a yard the pass refused to seat and
asserting the checker catches it -- which is the old slot's behaviour, one prop
at a time.

RUNS OFFLINE. `tools/fence_png` stubs `pxr` before importing `suburb_scene` and
answers every footprint question from measured bboxes, so the real
`build_placements` and the real `yp.plan` run here with no stage and no Isaac.

USAGE
    python3 scene_gen/tests/test_yard_enclosure.py
    pytest -q scene_gen/tests/test_yard_enclosure.py
"""

import collections
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_SCENE_GEN, os.path.join(_SCENE_GEN, "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# `fence_png` installs the pxr stubs on import; everything below it has to come
# after that, which is what the import order here is for.
import fence_png as fp                                            # noqa: E402
import suburb_scene as ss                                         # noqa: E402
from detail import suburb_yardplan as yp                          # noqa: E402

FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


# -- the fixture lot ---------------------------------------------------------
# A 20 x 40 m lot on the axes, frontage at the origin, `u` along the street and
# `n` into the block -- the same frame every house record carries. The house is
# 12 x 10 m with its FRONT WALL 8 m in, so `_building_line_depth` is 8.0 and the
# rear yard runs 8 m to 40 m: 22 m of it, of which 12 m is garden behind the
# back wall. Every number is round on purpose, so a coverage figure below can be
# read off by hand rather than trusted.
#
#   left edge   (-10, 8) -> (-10, 40)
#   right edge  ( 10, 8) -> ( 10, 40)
#   rear edge   (-10, 40) -> ( 10, 40)
LOT = {"frontage": (0.0, 0.0), "u": (1.0, 0.0), "n": (0.0, 1.0),
       "c": (0.0, 13.0), "w": 12.0, "d": 10.0,
       "lot_width": 20.0, "lot_depth": 40.0}


def _run(p0, p1, step=4.0):
    """*p0* -> *p1* as a chain of fence spans, `_fence_run`'s output shape.

    Chopped into panels rather than handed over as one long span because that
    is what `h["fence_drawn"]` really holds -- a run is tiled -- and a predicate
    that only worked on whole edges would pass a fixture built the lazy way and
    fail on the suburb.
    """
    ln = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(round(ln / step)))
    out = []
    for i in range(n):
        a, b = i / float(n), (i + 1) / float(n)
        out.append(((p0[0] + (p1[0] - p0[0]) * a, p0[1] + (p1[1] - p0[1]) * a),
                    (p0[0] + (p1[0] - p0[0]) * b, p0[1] + (p1[1] - p0[1]) * b)))
    return out


def _screen(p0, p1, r=2.0, step=3.0):
    """*p0* -> *p1* as a row of canopies, `(x, y, radius)`.

    2 m crowns at 3 m centres: with `_YARD_SCREEN_SLACK_M` (1.5 m) each disc
    reaches 3.5 m, so the row closes with room and the fixture is not sitting on
    the bar it is testing. A real screen row is exactly this -- `suburb_yardplan`
    advances by the crown it just planted plus the crown it is about to.
    """
    ln = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(2, int(ln / step) + 1)
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1.0),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1.0), r) for i in range(n)]


def _gappy(p0, p1, r=1.0, spacing=6.6):
    """A row that closes an edge to ~0.7 — over `_SCREEN_COVER`, under 0.95.

    Built for the drift demonstration in [3] and nothing else: it has to sit
    BETWEEN the shipped bar and a drifted one, so that moving the copy makes
    the two modules answer differently about the very same trees. 1 m crowns
    reach 2.5 m with the slack, so a 6.6 m step leaves 1.6 m of daylight
    between canopies — a real row of small trees, not a hedge.
    """
    ln = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    out = []
    d = 0.0
    while d <= ln:
        t = d / ln
        out.append((p0[0] + (p1[0] - p0[0]) * t,
                    p0[1] + (p1[1] - p0[1]) * t, r))
        d += spacing
    return out


def _half(p0, p1, lo, hi):
    """The sub-segment of *p0* -> *p1* between fractions *lo* and *hi*."""
    return ((p0[0] + (p1[0] - p0[0]) * lo, p0[1] + (p1[1] - p0[1]) * lo),
            (p0[0] + (p1[0] - p0[0]) * hi, p0[1] + (p1[1] - p0[1]) * hi))


def test_predicate():
    print('\n[1] what closes a back yard, and what does not')
    edges = ss._rear_yard_edges(LOT)
    check(len(edges) == 3, f'the fixture lot has three rear-yard edges '
                           f'({len(edges)})')
    check(abs(ss._building_line_depth(LOT) - 8.0) < 1e-9,
          f'its building line is 8.0 m in ({ss._building_line_depth(LOT):.2f})')
    left, right, rear = edges

    fenced = _run(*left) + _run(*right) + _run(*rear)
    screened = _screen(*left) + _screen(*right) + _screen(*rear)

    for name, kw, want in (
            ('a fence on all three edges closes it',
             {'fences': fenced}, True),
            ('a treeline on all three edges closes it',
             {'trees': screened}, True),
            ('bare ground does not',
             {}, False),
            # THE ONE THAT MATTERS. Both sides fenced, rear open. A mean over
            # the three edges is 0.67 and any bar low enough to reject it also
            # rejects a real yard with a gate in one side; the MINIMUM is 0.0
            # and rejects it for the right reason.
            ('two sides fenced and the rear open does NOT',
             {'fences': _run(*left) + _run(*right)}, False),
            ('two sides screened and the rear open does NOT',
             {'trees': _screen(*left) + _screen(*right)}, False),
            ('a fence down two sides plus a treeline on the rear closes it',
             {'fences': _run(*left) + _run(*right),
              'trees': _screen(*rear)}, True)):
        closed, cover = ss._yard_enclosed(LOT, **kw)
        check(closed is want,
              f'{name} (closed={closed}, min cover {cover:.2f})')

    # A LOT WITH NO GARDEN IS NOT AN OPEN YARD, and the predicate cannot tell
    # them apart -- both come back False. That is stated here because
    # `fence_check.fence_fragment` DEPENDS on it: a gardenless lot carrying
    # fence is a fragment, and it is caught precisely because this returns
    # False rather than raising or answering True vacuously.
    shallow = dict(LOT, lot_depth=20.0)      # 20 - 8 - 10 = 2 m of garden
    check(ss._rear_yard_edges(shallow) == [],
          'a lot with 2 m of garden has no rear-yard edges at all')
    check(ss._yard_enclosed(shallow, fences=fenced) == (False, 0.0),
          'and reads (False, 0.0) however much fence stands on it')

    assert not FAILS, FAILS


def test_sources_are_never_pooled():
    print('\n[2] fence cover and tree cover are never added together')
    left, right, rear = ss._rear_yard_edges(LOT)
    # The rear edge, fenced over its first 60% and planted over the rest.
    # Nothing is missing from the line -- and it is still not an enclosure.
    #
    # THE SPLIT IS 60/40 AND NOT 50/50 FOR A REASON WORTH READING. Each source
    # reaches PAST its own run -- a fence by `_YARD_FENCE_REACH_M` (1.2 m) and
    # a canopy by its crown plus `_YARD_SCREEN_SLACK_M` -- so half an edge of
    # 2 m crowns scores 0.67 against a tree bar of 0.60 and passes on its own.
    # That is not the case under test: the fixture has to be one where NEITHER
    # source clears its bar and the POOL does, or it proves nothing. Small
    # crowns at 60% of the way along put the two at 0.67 (fence, bar 0.85) and
    # 0.52 (trees, bar 0.60).
    part_f = _run(*_half(rear[0], rear[1], 0.0, 0.60))
    part_t = _screen(*_half(rear[0], rear[1], 0.60, 1.0), r=0.5, step=1.5)
    fc = ss._edge_cover(rear[0], rear[1], fences=part_f)
    tc = ss._edge_cover(rear[0], rear[1], trees=part_t)
    both = ss._edge_cover(rear[0], rear[1], fences=part_f, trees=part_t)
    print(f'        rear edge: fence {fc:.2f}, trees {tc:.2f}, '
          f'pooled {both:.2f}')
    check(both > 0.95, 'pooled, the rear edge reads as fully covered — this is '
                       'the answer the wrong predicate believes')
    check(fc < ss._YARD_FENCE_COVER and tc < ss._YARD_SCREEN_COVER,
          'each source alone is below its own bar')
    closed, _ = ss._yard_enclosed(LOT, fences=_run(*left) + _run(*right) + part_f,
                                  trees=part_t)
    check(closed is False,
          'so the yard is NOT closed, despite something standing along all of it')
    assert not FAILS, FAILS


def test_screen_bar_is_mirrored():
    print('\n[3] the duplicated screen constants have not drifted')
    # READ LIVE FROM BOTH MODULES. `suburb_yardplan` cannot import
    # `suburb_scene` -- that module imports THIS one at module scope, so the
    # arrow only points one way -- and a fourth pass inventing its own bar for
    # "screened" is the disease the enclosure work was written to cure. The
    # copies are labelled in `suburb_yardplan`'s own comment; this is what makes
    # the label enforceable.
    for a, av, b, bv in (('_YARD_SCREEN_COVER', ss._YARD_SCREEN_COVER,
                          '_SCREEN_COVER', yp._SCREEN_COVER),
                         ('_YARD_SCREEN_SLACK_M', ss._YARD_SCREEN_SLACK_M,
                          '_SCREEN_SLACK_M', yp._SCREEN_SLACK_M),
                         ('_YARD_SAMPLE_M', ss._YARD_SAMPLE_M,
                          '_EDGE_SAMPLE_M', yp._EDGE_SAMPLE_M)):
        check(av == bv, f'suburb_scene.{a} ({av}) == suburb_yardplan.{b} ({bv})')
    check(yp._Canopies().slack == ss._YARD_SCREEN_SLACK_M,
          f'and `_Canopies` grows a crown by that same slack '
          f'({yp._Canopies().slack})')
    # ...AND THE COMPARISON HAS TEETH. A test that only asserts two numbers are
    # equal today proves nothing about whether it would notice them diverging,
    # so this drifts one on purpose and checks the comparison and the SCREEN
    # ANSWER both move. 0.60 -> 0.95 is the change somebody makes when they
    # want screens to be rarer, and it is exactly the change that would leave
    # `suburb_scene` calling a yard screened that the seating gate refuses.
    _left, _right, _rear = ss._rear_yard_edges(LOT)
    row = _screen(*_left) + _screen(*_right) + _screen(*_rear)
    six = yp._Canopies(yp._SCREEN_SLACK_M)
    for (x, y, r) in row:
        six.add(x, y, r)
    cover = yp._screen_cover(ss._rear_yard_edges(LOT), six)
    was = yp._SCREEN_COVER
    try:
        yp._SCREEN_COVER = 0.95
        check(ss._YARD_SCREEN_COVER != yp._SCREEN_COVER,
              'drifting the copy makes the equality check fail, as it must')
        gappy = _gappy(*_left) + _gappy(*_right) + _gappy(*_rear)
        gix = yp._Canopies(yp._SCREEN_SLACK_M)
        for (x, y, r) in gappy:
            gix.add(x, y, r)
        gc = yp._screen_cover(ss._rear_yard_edges(LOT), gix)
        check(ss._yard_enclosed(LOT, trees=gappy)[0]
              and gc < yp._SCREEN_COVER,
              f'and the two modules then disagree about the same yard: '
              f'suburb_scene calls it screened, suburb_yardplan does not '
              f'(cover {gc:.2f})')
    finally:
        yp._SCREEN_COVER = was
    check(yp._SCREEN_COVER == ss._YARD_SCREEN_COVER,
          f'the bar is restored ({yp._SCREEN_COVER}); a solid row covers '
          f'{cover:.2f} of every edge')
    assert not FAILS, FAILS


def _rect(h):
    """`lot_corners` for a fixture lot, in `suburb_parcel`'s ring order."""
    p, u, n = h["frontage"], h["u"], h["n"]
    hw, ld = h["lot_width"] / 2.0, h["lot_depth"]

    def at(a, d):
        return (p[0] + u[0] * a + n[0] * d, p[1] + u[1] * a + n[1] * d)
    return [at(-hw, 0.0), at(hw, 0.0), at(hw, ld), at(-hw, ld)]


def test_attribution_is_by_lot_frame():
    print('\n[4] a prop is attributed to the lot it STANDS in')
    # TWO LOTS HUNG OFF TWO FRONTAGES OF ONE BLOCK, which is the corner
    # `fence_check.trespass` measures and the reason a lot rectangle is not a
    # partition of the ground. `a` fronts south and runs north; `b` fronts west
    # and runs east across `a`'s back garden, so the two rectangles overlap in
    # the block corner. This is not a contrived shape — it is 8 of the 392
    # seating props on seed 3.
    a = dict(LOT, lot_corners=None)
    a["lot_corners"] = _rect(a)
    b = {"frontage": (-30.0, 30.0), "u": (0.0, 1.0), "n": (1.0, 0.0),
         "c": (-17.0, 30.0), "w": 12.0, "d": 10.0,
         "lot_width": 20.0, "lot_depth": 40.0}
    b["lot_corners"] = _rect(b)
    hs = [a, b]

    # A bench at the back of `a`'s garden, off toward the side line — where
    # `patio_side_off_frac` puts a patio.
    pt = (-8.0, 34.0)
    owners = fp._LotIndex(hs).at(*pt)
    check(set(owners) == {0, 1},
          f'the point stands inside BOTH lot rectangles ({sorted(owners)})')

    # ...AND THE NEAREST HOUSE CENTRE IS THE WRONG ONE. `a`'s house is at the
    # front of a 40 m lot and the bench is at the back of it, so `b`'s centre —
    # a different lot, on a different street — is 9.8 m away against 22.5 m.
    da = math.dist(pt, a["c"])
    db = math.dist(pt, b["c"])
    print(f'        bench {da:.1f} m from its own house centre, '
          f'{db:.1f} m from the neighbour\'s')
    check(db < da,
          'nearest-house-centre attribution picks the WRONG lot for it')
    # So a checker written that way reports a bench in `a`'s fenced garden as
    # standing in `b`'s open one. That is not hypothetical: two such false
    # positives were reported during the seating work and both were this.
    check(min(range(len(hs)), key=lambda i: math.dist(pt, hs[i]["c"])) == 1,
          'i.e. it would score the bench to the lot it is not in')
    assert not FAILS, FAILS


# -- the real suburb ---------------------------------------------------------
# 1000 x 750 m of the shipped preset rather than the full 1600 x 1200 m. The
# street network is 14.4 s of the 16 s a full build costs and it is the only
# expensive part, so this is the same code path, the same records and the same
# invariants for 2.3 s. The FULL preset is gated by `tools/fence_check.py` over
# seeds 1/3/5/7; what a pytest run buys is that it is gated on every commit.
_REGION = (1000.0, 750.0)
_SCENES = {}


def _scene(seed):
    if seed not in _SCENES:
        _SCENES[seed] = fp.build(seed, region_m=_REGION)
    return _SCENES[seed]


def test_no_seating_without_an_enclosure():
    print('\n[5] no seating group stands in a yard nothing encloses')
    for seed in (3, 7):
        sc = _scene(seed)
        hs = fp.houses(sc)
        trees = fp.canopies(sc)
        state = fp.enclosure_state(sc, trees)
        seats = fp.seating(sc)
        bad = fp.seating_unenclosed(sc, trees, state)
        st = collections.Counter(state.values())
        print(f'        seed {seed}: {len(hs)} lots '
              f'({st["fenced"]} fenced, {st["screened"]} screened, '
              f'{st["open"]} open, {st[None]} gardenless), '
              f'{len(seats)} seating props, {len(bad)} in an open yard')
        # THE FIXTURE HAS TO EXERCISE THE RULE. A region that platted no
        # seating, or no open yards to put it in, passes this test for the
        # wrong reason -- and a region small enough to be fast is exactly the
        # kind that can.
        check(len(seats) >= 20, f'seed {seed} placed seating to test ({len(seats)})')
        check(st['open'] >= 5, f'seed {seed} has open back yards to misplace '
                               f'it in ({st["open"]})')
        check(not bad, f'seed {seed}: {len(bad)} seating props in an open yard')

        # ...AND THE CHECKER HAS TO BE ABLE TO SEE ONE. This is the old slot's
        # behaviour reproduced by hand: it made one attempt per lot with >= 4 m
        # of rear garden, with no enclosure test at all, so a bench landed in
        # the middle of open ground. Dropped in at the rear of a yard the pass
        # itself refused to seat, `seating_unenclosed` must find it and must
        # find only it.
        victim = next((h for h in hs if state.get(id(h)) == 'open'), None)
        check(victim is not None, f'seed {seed} has an open yard to plant in')
        if victim is None or not seats:
            continue
        p, u, n = victim['frontage'], victim['u'], victim['n']
        deep = float(victim['lot_depth']) - 2.0
        pt = (p[0] + n[0] * deep, p[1] + n[1] * deep)
        # Only meaningful if the point really is in this lot and in no
        # ENCLOSED one -- two lot rectangles genuinely overlap in a block
        # corner, and a prop excused by a second owner would prove nothing.
        owners = fp._LotIndex(hs).at(pt[0], pt[1])
        check(all(state.get(id(hs[i])) == 'open' for i in owners) and owners,
              f'seed {seed}: the planted point is in an open lot and no other')
        sc['yard'].append(dict(seats[0], x_m=pt[0], y_m=pt[1]))
        try:
            caught = fp.seating_unenclosed(sc, trees, state)
            check(len(caught) == 1 and
                  abs(caught[0][0][0] - pt[0]) < 1e-9,
                  f'seed {seed}: a bench dropped in an open yard is caught '
                  f'({len(caught)} flagged)')
        finally:
            sc['yard'].pop()
        check(not fp.seating_unenclosed(sc, trees, state),
              f'seed {seed}: and the scene is clean again once it is removed')
        # The same question [4] asks of a hand-built corner, asked of the real
        # plat. REPORTED AND NOT CHECKED, because it is a property of the
        # ground rather than of the code: this region is 1000 x 750 m and
        # throws up no overlapping corner with a seated lot on one side and an
        # open one on the other, so it reads 0 here and 1 on the full 1600 x
        # 1200 m preset. A number that is 0 for want of the geometry is not an
        # assertion, and [4] is where the rule is actually pinned.
        wrong = 0
        for pr in seats:
            q = (pr['x_m'], pr['y_m'])
            near = min(hs, key=lambda h: ((h['c'][0] - q[0]) ** 2
                                          + (h['c'][1] - q[1]) ** 2)
                       if h.get('c') else 1e18)
            if state.get(id(near)) == 'open':
                wrong += 1
        print(f'        seed {seed}: nearest-house-centre attribution would '
              f'report {wrong} of these {len(seats)} props as misplaced')
    assert not FAILS, FAILS


def main():
    print('=' * 70)
    print('YARD ENCLOSURE')
    print('=' * 70)
    for fn in (test_predicate, test_sources_are_never_pooled,
               test_screen_bar_is_mirrored, test_attribution_is_by_lot_frame,
               test_no_seating_without_an_enclosure):
        try:
            fn()
        except AssertionError:
            pass
    print('\n' + '=' * 70)
    if FAILS:
        print(f'{len(FAILS)} FAILED')
        for f in FAILS:
            print('   -', f)
        return 1
    print('all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
