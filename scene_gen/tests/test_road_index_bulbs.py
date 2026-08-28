#!/usr/bin/env python3
"""A cul-de-sac turnaround is road, and kerbside props must stand off it.

THE FAULT. `_RoadIndex` is the one POSITIVE "is this point on the road?" test
`build_frontage` has, and it knew only about centreline segments and their half
widths. A lollipop's turnaround is a **disc** — `apply_ground` lays it at
`bulb_radius_m` = 14.64 m against a ~5 m half carriageway — so a point out near
the edge of a bulb is far outside `half_w + margin` of the stem's last segment
and `on_road` said "not road". Meanwhile `_RoadIndex.verge` deliberately
declines to snap at a segment ENDPOINT (a bulb kerb is an arc the half-width
model cannot describe) and falls back to offsetting from the block ring — and
the block ring at a turnaround is the arc `_arc_cap_bulbs` spliced in. The two
together put street lights, hydrants and bins on the paving:
`test_streetlight_placement.py` section 6 records one measured at **14.62 m
from the bulb centre against a 14.64 m paved radius**. Reported on sight as
props standing on top of the cul-de-sac, 2026-08-27.

THE FIX, and both halves are needed:

  * `_RoadIndex(net, bulb_r=...)` registers each lollipop end as a
    ZERO-LENGTH segment with `half_w` = the bulb radius, which is exactly what
    a disc is to a segment-distance test. `on_road` then answers correctly
    inside a turnaround for every caller, not just this one.
  * `bulb_verge` PUSHES rather than drops. Deleting the prop would also satisfy
    `on_road`, and it is worse: a real turnaround is lit, and the street
    light's 2.79 m mast arm reaching over the paving is what the arm is for.

RUNS WITHOUT ISAAC. `suburb_scene` imports `pxr` at module scope, so the class
is sliced out of the source and exec'd against a stub `sn` — the same trick
`test_fence_geometry.py` uses for `_fence_run`.

USAGE
    python3 scene_gen/tests/test_road_index_bulbs.py
"""

import math
import os
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, "..", "suburb_scene.py")).read()

FAILS = []


def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


# --------------------------------------------------------------------------
# slice the real class out
# --------------------------------------------------------------------------

def _seg_seg_dist(p0, p1, a, b):
    """Enough of `suburb_net.seg_seg_dist` for a POINT query.

    `_RoadIndex` only ever calls it as `seg_seg_dist(p, p, a, b)` — a
    degenerate first segment — so point-to-segment is the whole contract, and
    stubbing it keeps this test free of the layout package.
    """
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 0.0 else ((p0[0] - a[0]) * vx + (p0[1] - a[1]) * vy) / L2
    t = max(0.0, min(1.0, t))
    return math.hypot(p0[0] - (a[0] + t * vx), p0[1] - (a[1] + t * vy))


class _StubSn:
    seg_seg_dist = staticmethod(_seg_seg_dist)

    @staticmethod
    def _add(p, q):
        return (p[0] + q[0], p[1] + q[1])

    @staticmethod
    def _mul(p, k):
        return (p[0] * k, p[1] * k)


_i = SRC.index("class _RoadIndex")
_j = SRC.index("\n# ---", _i)
_ns = {"math": math, "sn": _StubSn}
exec(textwrap.dedent(SRC[_i:_j]), _ns)
RoadIndex = _ns["_RoadIndex"]

BULB_R = 14.64          # sn.DEFAULTS["bulb_radius_m"]
VERGE = 1.6             # city_detail.zones.furnishing_inset_m default


class _E:
    def __init__(self, pts, half_w, street_type="street", road_class="local"):
        self.pts = pts
        self.half_w = half_w
        self.street_type = street_type
        self.road_class = road_class


class _Net:
    def __init__(self, edges):
        self.edges = {i: e for i, e in enumerate(edges)}


def _net():
    """A straight collector, and a lollipop stem whose bulb is at (100, 0)."""
    return _Net([_E([(0.0, 0.0), (60.0, 0.0)], 5.5),
                 _E([(60.0, 0.0), (100.0, 0.0)], 5.0, street_type="lollipop")])


BULB = (100.0, 0.0)


def _at(d, ang=0.0):
    return (BULB[0] + d * math.cos(ang), BULB[1] + d * math.sin(ang))


def _r(p):
    return math.hypot(p[0] - BULB[0], p[1] - BULB[1])


# --------------------------------------------------------------------------

def test_01_bulbs_are_road():
    print("\n[1] the turnaround reads as road, and the old index could not see it")
    old = RoadIndex(_net())
    new = RoadIndex(_net(), bulb_r=BULB_R)
    check(len(new.bulbs) == 1, "one lollipop end registered as a bulb")
    check(not old.bulbs, "no bulbs without the radius — old behaviour is unchanged")
    for d in (0.0, 5.0, 10.0, 14.0, 14.62):
        p = _at(d)
        check(new.on_road(p, margin=0.3),
              "%.2f m from the bulb centre is ON the turnaround" % d)
    check(not old.on_road(_at(14.62), margin=0.3),
          "the OLD index called the measured 14.62 m lamp position clear — "
          "the bug this fixes")


def test_02_off_the_paving_is_still_off():
    print("\n[2] and it does not swallow the verge around it")
    new = RoadIndex(_net(), bulb_r=BULB_R)
    for d in (BULB_R + 0.4, BULB_R + VERGE, BULB_R + 6.0):
        check(not new.on_road(_at(d), margin=0.3),
              "%.2f m out is verge, not road" % d)
    check(new.on_road((20.0, 0.0), margin=0.3),
          "the straight carriageway still tests positive")
    check(not new.on_road((20.0, 8.0), margin=0.3),
          "and its verge still tests negative")


def test_03_bulb_verge_pushes_out_not_in():
    print("\n[3] bulb_verge moves a prop radially out to the verge")
    new = RoadIndex(_net(), bulb_r=BULB_R)
    for d in (0.5, 6.0, 14.62, BULB_R + VERGE - 0.01):
        q, n = new.bulb_verge(_at(d), VERGE)
        check(abs(_r(q) - (BULB_R + VERGE)) < 1e-6,
              "a prop at %.2f m lands at %.2f m — the paved radius plus the "
              "furnishing inset" % (d, _r(q)))
        check(not new.on_road(q, margin=0.3),
              "  and the pushed point passes the road test that rejected it")
    # the normal points AWAY from the centre, so `-n` (which build_frontage
    # faces the prop along) points AT the turnaround: arm over the road.
    q, n = new.bulb_verge(_at(6.0, math.radians(37.0)), VERGE)
    ux = (q[0] - BULB[0]) / _r(q)
    uy = (q[1] - BULB[1]) / _r(q)
    check(abs(n[0] - ux) < 1e-9 and abs(n[1] - uy) < 1e-9,
          "the returned normal is the outward radial, so the lamp faces the bulb")


def test_04_ordinary_frontage_is_never_teleported():
    print("\n[4] a prop already clear of the bulb is left exactly where it was")
    new = RoadIndex(_net(), bulb_r=BULB_R)
    for d in (BULB_R + VERGE, BULB_R + VERGE + 0.01, BULB_R + 8.0, 40.0):
        check(new.bulb_verge(_at(d), VERGE) is None,
              "%.2f m from the centre needs no push" % d)
    # A lamp on the straight collector's verge, 20 m back from the bulb, must
    # not be snapped onto the bulb's radial ring — that would take street
    # furniture off a straight kerb and stand it on a circle.
    p = (20.0, 7.1)
    check(new.bulb_verge(p, VERGE) is None,
          "a collector-verge lamp 80 m from the turnaround is untouched")


def test_05_wired_into_build_frontage():
    print("\n[5] build_frontage actually uses both halves")
    check("road = _RoadIndex(net, bulb_r=float(" in SRC,
          "the frontage index is built WITH the bulb radius")
    check("_bq = road.bulb_verge(q, verge)" in SRC,
          "and every kerbside prop is pushed off a turnaround before the "
          "road test")
    i = SRC.index("_bq = road.bulb_verge(q, verge)")
    j = SRC.index("if road.on_road(q, margin=0.3):", i)
    check(0 < j - i < 400,
          "the push happens BEFORE on_road, so a bulb prop is relocated "
          "rather than dropped")


def main():
    test_01_bulbs_are_road()
    test_02_off_the_paving_is_still_off()
    test_03_bulb_verge_pushes_out_not_in()
    test_04_ordinary_frontage_is_never_teleported()
    test_05_wired_into_build_frontage()
    print("\n" + "=" * 68)
    if FAILS:
        print("FAILED %d check(s)" % len(FAILS))
        for m in FAILS:
            print("  - " + m)
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
