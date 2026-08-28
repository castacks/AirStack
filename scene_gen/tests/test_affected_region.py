#!/usr/bin/env python3
"""
test_affected_region.py — the disaster-affected polygon, pinned without Isaac.

`disaster/region.py` writes the burnt region out as an ordinary polygon so a
search planner can size its search area to the fire instead of to the whole
1 km plat. That polygon is an ANSWER KEY BOUND: a search area that does not
contain the survivors is a benchmark nobody can win, and the failure is silent
— the planner sweeps a perfectly good ellipse and simply never flies over the
people. Nothing in a render says so. Five claims are worth pinning:

  1. IT IS THE FIRE MODEL, NOT A PICTURE OF IT. `point_in_polygon` on
     `front_polygon(..., T)` agrees with `fire._ignition_time(u, v) <= T`
     everywhere except within a couple of metres of the boundary, which is the
     72-gon's own chord sag and nothing else.
  2. IT GROWS. The ellipse at a later T strictly contains the earlier one, and
     `affected` therefore contains `burn`.
  3. THE CLIP IS A CLIP. A polygon inside the plat comes back untouched; one
     hanging off the edge comes back smaller and entirely inside.
  4. THE PEOPLE SET THE SIZE. On the shipped 1 km wildfire the `affected`
     polygon contains ground-truth survivor positions taken from a real run —
     and a FIXED 0.20*span lead does NOT, which is the measurement the
     `people_lead_s` bound exists because of. Both are pinned: the people bound
     covers all five, the fixed lead covers one.
  5. THE SEAMS HOLD. `scene_api` really does compute the polygons and really
     does publish them, and the launcher really does write them out.

RUNS WITHOUT ISAAC. `region.py` is plain arithmetic — no `pxr`, no numpy, no
stage. `fire.py` is NOT: it imports `pxr` at module scope, so `_ignition_time`
cannot simply be imported here. It is SLICED OUT OF fire.py AS SOURCE and
exec'd (the trick `test_scour_relief.test_launcher_wiring` uses on the
launcher), so what the polygon is compared against is the repo's real spread
model rather than a copy of it. A copy is kept below anyway, and checked
against the sliced original — a reader should be able to see the twenty lines
the polygon is claiming to be.

USAGE
    cd scene_gen && python3 -m pytest tests/test_affected_region.py -q
    python3 scene_gen/tests/test_affected_region.py
"""

import math
import os
import random
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import region as rg                      # noqa: E402

FAILS = []
_GATED = 0


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def gate():
    """Turn everything `check` recorded in this test into a pytest failure.

    `check` collects rather than asserts, so a test prints its whole table
    before it stops — a single number out of place is far easier to read
    against the other nine than on its own. That only works if something
    eventually raises, and this is it: called last in every test, it fails on
    what THIS test recorded. The standalone runner below swallows the raise,
    because it prints the same list itself at the end.
    """
    global _GATED
    new = FAILS[_GATED:]
    _GATED = len(FAILS)
    assert not new, " | ".join(new)


# ---------------------------------------------------------------------------
# the spread model, borrowed from fire.py without importing pxr
# ---------------------------------------------------------------------------

_FIRE_PY = os.path.join(_SCENE_GEN, "disaster", "fire.py")


def _sliced_ignition_time():
    """`fire._ignition_time`, sliced out of fire.py as text and exec'd.

    `disaster/fire.py` does `from pxr import ...` at module scope, so importing
    it here would need Isaac. The function itself depends on nothing but
    `math`, so the twenty lines are cut out between their own `def` and the
    next top-level one and compiled on their own.
    """
    src = open(_FIRE_PY, encoding="utf-8").read()
    i = src.index("def _ignition_time(")
    j = src.index("\ndef ", i + 1)
    block = src[i:j]
    ns = {"math": math}
    exec(compile(block, "<fire._ignition_time>", "exec"), ns)
    return ns["_ignition_time"], block


# THE COPY. Verbatim from `fire._ignition_time` — kept here so the thing
# `front_polygon` claims to draw is readable in one place, and checked against
# the original above by `test_matches_the_fire_model`.
def _ignition_time_copy(u, v, head, flank, back):
    A = 0.5 * (head + back)
    C = 0.5 * (head - back)
    B = flank
    if A <= 0.0 or B <= 0.0:
        return float("inf")
    if abs(u) < 1e-9 and abs(v) < 1e-9:
        return 0.0

    qa = (u * u) / (A * A) + (v * v) / (B * B)
    qb = -2.0 * u * C / (A * A)
    qc = (C * C) / (A * A) - 1.0

    disc = qb * qb - 4.0 * qa * qc
    if qa <= 0.0 or disc < 0.0:
        return float("inf")
    s = (-qb + math.sqrt(disc)) / (2.0 * qa)
    if s <= 1e-12:
        return float("inf")
    return 1.0 / s


# ---------------------------------------------------------------------------
# the shipped 1 km wildfire
# ---------------------------------------------------------------------------
#
# `compile_disaster.compile_wildfire` at severity 0.6 on a 1000 m plat:
#   head  = lerp(0.25, 2.0, 0.6)      = 1.30 m/s
#   flank = head / 4                  = 0.325
#   back  = head / 12                 = 0.10833
#   origin = epicenter default        = (-0.35 * span, -0.35 * span)
KM_CFG = {
    "origin_m": [-350.0, -350.0],
    "heading_deg": 45.0,
    "head_mps": 1.30,
    "flank_mps": 0.325,
    "back_mps": 0.10833,
}
KM_REGION = (-500.0, -500.0, 500.0, 500.0)
KM_ELAPSED = 450.0
KM_SPAN = 430.0

# Ground truth out of a real run of that scene — survivor positions from the
# people pass, NOT points chosen to make this pass.
KM_PEOPLE = [(107.8, -78.1), (194.7, 85.8), (-34.3, -368.5),
             (80.3, -253.8), (-40.4, -369.0)]


def _wind_frame(x, y, cfg):
    """World xy -> the fire's (u, v), the same rotation `plan_ignition` uses."""
    ox, oy = cfg["origin_m"]
    th = math.radians(float(cfg["heading_deg"]))
    ct, st = math.cos(th), math.sin(th)
    dx, dy = x - ox, y - oy
    return dx * ct + dy * st, -dx * st + dy * ct


def _dist_to_ring(x, y, poly):
    """Shortest distance from a point to a ring's boundary, metres."""
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 <= 0.0 else ((x - ax) * vx + (y - ay) * vy) / L2
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(x - (ax + vx * t), y - (ay + vy * t)))
    return best


# ---------------------------------------------------------------------------
# 1. the polygon IS the fire model
# ---------------------------------------------------------------------------

def test_matches_the_fire_model():
    """`inside the polygon at T` == `_ignition_time <= T`, to a few metres."""
    print("\n[1] the polygon agrees with fire._ignition_time")
    live, block = _sliced_ignition_time()

    # The copy above and the original are the same function.
    rng = random.Random(5)
    worst = 0.0
    for _k in range(2000):
        u = rng.uniform(-800.0, 800.0)
        v = rng.uniform(-400.0, 400.0)
        a = live(u, v, 1.30, 0.325, 0.10833)
        b = _ignition_time_copy(u, v, 1.30, 0.325, 0.10833)
        if math.isinf(a) and math.isinf(b):
            continue
        worst = max(worst, abs(a - b))
    check(worst < 1e-9,
          "the copy in this file is fire._ignition_time (worst %.3g s)" % worst)
    # ...and the slice really is the quadratic, not some other twenty lines.
    check("disc = qb * qb - 4.0 * qa * qc" in block
          and re.search(r"def _ignition_time\(u, v, head, flank, back\)", block),
          "the sliced block is the arrival-time quadratic")

    TOL_M = 2.0        # the 72-gon's chord sag is ~0.4 m on this ellipse
    for label, T in (("burn", KM_ELAPSED), ("affected", 536.0),
                     ("early", 120.0)):
        poly = rg.front_polygon(KM_CFG["origin_m"], KM_CFG["heading_deg"],
                                KM_CFG["head_mps"], KM_CFG["flank_mps"],
                                KM_CFG["back_mps"], T)
        n_dis = 0
        worst_d = 0.0
        rng = random.Random(17)
        for _k in range(4000):
            x = rng.uniform(-700.0, 500.0)
            y = rng.uniform(-700.0, 500.0)
            u, v = _wind_frame(x, y, KM_CFG)
            t = live(u, v, KM_CFG["head_mps"], KM_CFG["flank_mps"],
                     KM_CFG["back_mps"])
            model = math.isfinite(t) and t <= T
            poly_in = rg.point_in_polygon(x, y, poly)
            if model != poly_in:
                n_dis += 1
                worst_d = max(worst_d, _dist_to_ring(x, y, poly))
        print("      T=%6.0f s: %d/4000 disagree, worst %.3f m from the edge"
              % (T, n_dis, worst_d))
        check(worst_d < TOL_M,
              "%s: every disagreement is inside the %.0f m boundary band "
              "(%.3f m)" % (label, TOL_M, worst_d))

    # A degenerate fire draws nothing rather than a line or a crash.
    check(rg.front_polygon([0, 0], 0.0, 1.0, 0.2, 0.0, 0.0) == [],
          "t=0 is an empty polygon")
    check(rg.front_polygon([0, 0], 0.0, 1.0, 0.0, 0.0, 100.0) == [],
          "a fire with no flank spread is an empty polygon")
    check(rg.polygon_area([]) == 0.0 and not rg.point_in_polygon(0, 0, []),
          "an empty polygon has no area and contains nothing")
    gate()


# ---------------------------------------------------------------------------
# 2. it grows, and `affected` contains `burn`
# ---------------------------------------------------------------------------

def test_grows_monotonically():
    """A later front contains an earlier one, so affected contains burn."""
    print("\n[2] the ellipse only ever grows")
    args = (KM_CFG["origin_m"], KM_CFG["heading_deg"], KM_CFG["head_mps"],
            KM_CFG["flank_mps"], KM_CFG["back_mps"])
    areas = []
    for T in (60.0, 120.0, 300.0, 450.0, 536.0, 900.0):
        areas.append(rg.polygon_area(rg.front_polygon(*args, t_s=T)))
    print("      areas m2: " + ", ".join("%.0f" % a for a in areas))
    check(all(b > a for a, b in zip(areas, areas[1:])),
          "the area is strictly increasing in T")
    # Area is quadratic in t, so doubling T quadruples it.
    r = areas[2] / areas[1]                       # 300 s over 120 s
    check(abs(r - (300.0 / 120.0) ** 2) < 1e-6,
          "the area goes as T^2, as a linearly-growing ellipse must (%.4f)" % r)

    early = rg.front_polygon(*args, t_s=KM_ELAPSED)
    late = rg.front_polygon(*args, t_s=536.0)
    outside = [p for p in early if not rg.point_in_polygon(p[0], p[1], late)]
    check(not outside,
          "every vertex of the t=450 front is inside the t=536 one (%d out)"
          % len(outside))

    reg = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION)
    check(reg["t_affected_s"] > reg["t_burn_s"],
          "affected is read LATER than burn (t+%.0f vs t+%.0f s)"
          % (reg["t_affected_s"], reg["t_burn_s"]))
    check(abs(reg["t_affected_s"] - (KM_ELAPSED + 0.20 * KM_SPAN)) < 1e-9,
          "the lead is lead_frac * span, 0.20 by default")
    a_burn = rg.polygon_area(reg["burn"])
    a_aff = rg.polygon_area(reg["affected"])
    print("      clipped: burn %.0f m2, affected %.0f m2 (+%.1f%%)"
          % (a_burn, a_aff, 100.0 * (a_aff / a_burn - 1.0)))
    check(a_aff > a_burn, "affected is bigger than burn once clipped too")
    out = [p for p in reg["burn"]
           if not rg.point_in_polygon(p[0], p[1], reg["affected"])
           and _dist_to_ring(p[0], p[1], reg["affected"]) > 1e-6]
    check(not out, "the clipped burn is inside the clipped affected (%d out)"
          % len(out))

    # A zero lead and a zero margin collapse the two, which is the honest way
    # to ask for "just the burn" and must not be a special case.
    same = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                                lead_frac=0.0, margin_s=0.0)
    check(same["affected"] == same["burn"],
          "a zero lead and a zero margin make affected the burn exactly")

    # THE MAX IS A MAX, IN BOTH DIRECTIONS.
    lo = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                              people_lead_s=10.0)
    check(lo["lead_bound"] == "lead_frac"
          and abs(lo["lead_s"] - 0.20 * KM_SPAN) < 1e-9,
          "a survivor closer than the floor leaves lead_frac in charge "
          "(%.0f s by %s)" % (lo["lead_s"], lo["lead_bound"]))
    hi = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                              people_lead_s=383.0)
    check(hi["lead_bound"] == "people"
          and abs(hi["lead_s"] - (383.0 + rg.DEFAULT_MARGIN_S)) < 1e-9,
          "a survivor beyond it takes over, plus the margin (%.0f s by %s)"
          % (hi["lead_s"], hi["lead_bound"]))
    check(rg.polygon_area(hi["affected"]) > rg.polygon_area(lo["affected"]),
          "...and the people-bound area is the bigger one")

    # `people_lead_s()` reads `burn_age_s` the way people.py writes it:
    # NEGATIVE is ahead of the front, and ahead is what has to be covered.
    recs = [{"burn_age_s": 40.0}, {"burn_age_s": -111.7}, {"burn_age_s": -383.0},
            {"nothing": 1}, {"burn_age_s": None}]
    check(rg.people_lead_s(recs) == 383.0,
          "people_lead_s is max(-burn_age_s) over the records (%.1f)"
          % rg.people_lead_s(recs))
    check(rg.people_lead_s([{"burn_age_s": 12.0}]) == 0.0,
          "a scene whose survivors are all in the black needs no lead")
    check(rg.people_lead_s([]) == 0.0 and rg.people_lead_s(None) == 0.0,
          "no records is no lead, not a crash")
    gate()


# ---------------------------------------------------------------------------
# 3. the clip is a clip
# ---------------------------------------------------------------------------

def test_clip_rect():
    """Inside is untouched; across the edge is smaller and inside."""
    print("\n[3] clipping to the plat")
    rect = (-100.0, -100.0, 100.0, 100.0)
    square = [[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]]
    check(rg.clip_rect(square, rect) == square,
          "a polygon wholly inside the rect comes back UNCHANGED")

    args = (KM_CFG["origin_m"], KM_CFG["heading_deg"], KM_CFG["head_mps"],
            KM_CFG["flank_mps"], KM_CFG["back_mps"])
    big = rg.front_polygon(*args, t_s=900.0)
    clipped = rg.clip_rect(big, rect)
    a_big, a_clip = rg.polygon_area(big), rg.polygon_area(clipped)
    print("      a %.0f s front is %.0f m2, %.0f m2 of it on a 200 m plate"
          % (900.0, a_big, a_clip))
    check(0.0 < a_clip < a_big, "clipping across the edge loses area")
    eps = 1e-9
    check(all(rect[0] - eps <= p[0] <= rect[2] + eps
              and rect[1] - eps <= p[1] <= rect[3] + eps for p in clipped),
          "every surviving vertex is inside the rect")
    # A rect wholly inside the front comes back AS the rect.
    check(abs(rg.polygon_area(rg.clip_rect(big, (-10.0, -10.0, 10.0, 10.0)))
              - 400.0) < 1e-6,
          "a rect swallowed by the front clips to the rect (400 m2)")
    # ...and a front that misses the plate entirely leaves nothing.
    far = rg.clip_rect(big, (2000.0, 2000.0, 2100.0, 2100.0))
    check(far == [], "a front that misses the plate clips to nothing")
    check(rg.clip_rect([], rect) == [], "clipping nothing gives nothing")

    # The rect's corners in either order mean the same rect — `binfo["region"]`
    # is a plat extent and nobody should have to remember its winding.
    check(rg.clip_rect(big, (100.0, 100.0, -100.0, -100.0)) == clipped,
          "the rect corners may be given in either order")
    gate()


# ---------------------------------------------------------------------------
# 4. the shipped 1 km scene contains its own survivors
# ---------------------------------------------------------------------------

def test_km_scene_covers_the_answer_key():
    """The `affected` polygon of the real scene contains its real people —
    and the fixed 0.20*span lead alone does NOT.

    THE POINTS ARE NOT NEGOTIABLE. They came out of a run of
    `suburb_wildfire` at severity 0.6. Measured against them, the fixed lead
    covers ONE of five: the other four are 26 to 69 m outside, and three of
    those are at v = -232..-236 m in the wind frame against a cross-wind
    semi-axis of only b = 0.325*T = 174 m. They are FLANK cases, and a lead
    expressed as a fraction of `span` is the most expensive possible way to buy
    flank width, because b grows only linearly in T.

    That is why `affected_polygons` takes `people_lead_s`: instead of guessing
    a lead, the front is run until it has actually reached the
    furthest-ahead survivor, plus `margin_s`. Both halves are pinned below —
    the people bound covers all five, the fixed lead covers one — so the day
    somebody is tempted to drop the people bound and keep the tidy constant,
    the number that says why is right here.
    """
    print("\n[4] the 1 km wildfire, against ground truth from a real run")
    live, _block = _sliced_ignition_time()

    # `people_lead_s` as `build_scene` computes it, but derived here from the
    # fire model rather than trusted from a record: `burn_age_s` IS
    # `elapsed - arrival`, so the lead of the furthest-ahead survivor is
    # max(arrival) - elapsed over the ground-truth points.
    arrivals = [live(*(_wind_frame(x, y, KM_CFG)), KM_CFG["head_mps"],
                     KM_CFG["flank_mps"], KM_CFG["back_mps"])
                for x, y in KM_PEOPLE]
    lead_s = max(0.0, max(arrivals) - KM_ELAPSED)
    reg = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                               people_lead_s=lead_s)
    aff, burn = reg["affected"], reg["burn"]
    bb = rg.polygon_bbox(aff)
    print("      people_lead_s %.1f s (+ %.0f s margin) vs a %.0f s floor "
          "(0.20 x %.0f) -> lead %.0f s, bound by %s"
          % (lead_s, reg["margin_s"], 0.20 * KM_SPAN, KM_SPAN, reg["lead_s"],
             reg["lead_bound"]))
    check(reg["lead_bound"] == "people",
          "the survivors, not the 0.20 floor, size this scene's search area")
    print("      t_burn %.0f s, t_affected %.0f s (elapsed %.0f + %.0f)"
          % (reg["t_burn_s"], reg["t_affected_s"], KM_ELAPSED, reg["lead_s"]))
    print("      affected %d pts, %.0f m2 (%.1f%% of the plat), bbox "
          "%.0f..%.0f, %.0f..%.0f"
          % (len(aff), rg.polygon_area(aff),
             100.0 * rg.polygon_area(aff) / 1.0e6,
             bb[0], bb[2], bb[1], bb[3]))
    print("      burn     %d pts, %.0f m2" % (len(burn),
                                              rg.polygon_area(burn)))

    n_burn = 0
    outside = []
    print("      %-19s %-8s %-5s %9s %8s %9s %8s"
          % ("survivor", "affected", "burn", "arrival s", "age/span",
             "lead need", "pad m"))
    for x, y in KM_PEOPLE:
        in_aff = rg.point_in_polygon(x, y, aff)
        in_burn = rg.point_in_polygon(x, y, burn)
        n_burn += 1 if in_burn else 0
        d = _dist_to_ring(x, y, aff)
        u, v = _wind_frame(x, y, KM_CFG)
        t = live(u, v, KM_CFG["head_mps"], KM_CFG["flank_mps"],
                 KM_CFG["back_mps"])
        # The lead this one point would have needed, as a fraction of span:
        # `affected` is the ellipse at elapsed + lead*span, so a point the
        # front reaches at `t` needs lead = (t - elapsed) / span.
        need = (t - KM_ELAPSED) / KM_SPAN
        print("      (%7.1f, %7.1f) %-8s %-5s %9.1f %+8.3f %9.3f %8.1f"
              % (x, y, "yes" if in_aff else "NO", "yes" if in_burn else "no",
                 t, (KM_ELAPSED - t) / KM_SPAN, need,
                 d if not in_aff else 0.0))
        if not in_aff:
            outside.append((x, y, d, need, v))
    if outside:
        b_axis = KM_CFG["flank_mps"] * reg["t_affected_s"]
        for x, y, d, need, v in outside:
            print("      FINDING: (%.1f, %.1f) is %.1f m OUTSIDE the affected "
                  "polygon — it needs lead_frac %.3f (cross-wind v = %.0f m "
                  "against a %.0f m semi-axis)" % (x, y, d, need, v, b_axis))
    check(not outside,
          "every ground-truth survivor is inside the affected area (%d/%d)"
          % (len(KM_PEOPLE) - len(outside), len(KM_PEOPLE)))
    print("      %d/%d of them are inside the BURN — the rest are the ones "
          "ahead of the front that `affected` exists for"
          % (n_burn, len(KM_PEOPLE)))

    # AND THE HALF THAT JUSTIFIES THE BOUND. Run the same scene on the fixed
    # 0.20*span lead alone — the rule this started as — and count what it
    # misses. If this ever comes out 5/5 the people bound has stopped earning
    # its keep and can go; until then it is the reason it is there.
    fixed = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                                 people_lead_s=0.0)
    miss = [(x, y, _dist_to_ring(x, y, fixed["affected"])) for x, y in KM_PEOPLE
            if not rg.point_in_polygon(x, y, fixed["affected"])]
    print("      on the 0.20 floor alone (t+%.0f s, %.0f m2): %d/%d inside"
          % (fixed["t_affected_s"], rg.polygon_area(fixed["affected"]),
             len(KM_PEOPLE) - len(miss), len(KM_PEOPLE)))
    for x, y, d in miss:
        print("        MISSED (%7.1f, %7.1f) by %5.1f m" % (x, y, d))
    check(miss,
          "the fixed 0.20 lead really does miss survivors (%d of %d) — this "
          "is why people_lead_s exists" % (len(miss), len(KM_PEOPLE)))
    check(rg.polygon_area(aff) > rg.polygon_area(fixed["affected"]),
          "...and the people bound is what makes the polygon big enough "
          "(%.0f m2 vs %.0f m2)" % (rg.polygon_area(aff),
                                    rg.polygon_area(fixed["affected"])))

    # The affected area has to be a SEARCH AREA, i.e. usefully smaller than
    # the plat. If it were not, none of this would buy the planner anything.
    frac = rg.polygon_area(aff) / 1.0e6
    check(0.05 < frac < 0.80,
          "the affected area is a real reduction on the 1 km plat (%.0f%%)"
          % (100.0 * frac))
    gate()


# ---------------------------------------------------------------------------
# 5. the seams
# ---------------------------------------------------------------------------

_SCENE_API = os.path.join(_SCENE_GEN, "scene_api.py")
_LAUNCHER = os.path.join(
    os.path.dirname(_SCENE_GEN), "simulation", "isaac-sim", "launch_scripts",
    "example_multi_drone_scene_import.py")


def test_wiring():
    """The polygons are computed where `age` is, and written where the GT is.

    Read as SOURCE rather than imported: `scene_api` needs `pxr` and the
    launcher's module scope starts Isaac. What is being pinned is that the two
    call sites exist at all and read from the right places — the numbers above
    are worth nothing if nothing ever calls them.
    """
    print("\n[5] the two seams")
    api = open(_SCENE_API, encoding="utf-8").read()
    check("from disaster import region as region_poly" in api,
          "scene_api imports the module (aliased — `region` is a local there)")
    check("region_poly.affected_polygons(" in api
          and "people_lead_s=region_poly.people_lead_s(p_recs)" in api,
          "build_scene computes the polygons from the merged fire cfg and "
          "sizes them off the survivor records")
    # ONE rect, not two. The ground scar used to compute it itself; if both
    # lines come back the two can drift apart.
    check(api.count('tuple(binfo.get("region") or (-800, -600, 800, 600))') == 1,
          "the plat rect is resolved exactly once")
    i = api.index("def build_scene(")
    check(api.index('tuple(binfo.get("region")', i)
          < api.index("# 5) GROUND SCAR", i),
          "...and it is resolved BEFORE the ground scar reads it")
    # THE ORDER THE PEOPLE BOUND FORCES. `p_recs` does not exist until the
    # people pass has run, so the polygons cannot be computed with `age` —
    # this is the constraint that moved them.
    check(api.index("ppl.plan_people(", i)
          < api.index("region_poly.affected_polygons(", i),
          "the polygons are computed AFTER the people pass, which is what "
          "makes p_recs readable")
    for key in ('"burn_xy"', '"affected_xy"', '"t_burn_s"', '"t_affected_s"',
                '"fire_origin_m"', '"fire_heading_deg"', '"lead_s"',
                '"lead_bound"', '"people_lead_s"', '"lead_frac"',
                '"margin_s"'):
        check(key in api[api.index("    return {", i):],
              "the stats dict carries %s" % key)

    lx = open(_LAUNCHER, encoding="utf-8").read()
    check('sa.write_annotations(scene + "_region"' in lx,
          "the launcher writes <scene>_region.json")
    for cls in ('"class": "burn"', '"class": "affected"', '"class": "region"',
                '"class": "meta"'):
        check(cls in lx, "the region file carries %s" % cls)
    check("region_poly.point_in_polygon(cx, cy, aff)" in lx,
          "...and CHECKS the people GT against the polygon it just wrote")
    check("OUTSIDE the affected polygon" in lx,
          "...and says so loudly when one is not")
    check("affected    {0:.0f} m2 ellipse at t+{1:.0f} s" in lx,
          "the scene banner reports the affected area")

    # ...AND THE BLOCK ITSELF, SLICED OUT AND RUN against stubs — the same
    # trick `test_scour_relief.test_launcher_wiring` uses, and for the same
    # reason: the launcher's module scope starts Isaac, so the only way to
    # execute the repo's real wiring rather than a copy of it is to exec the
    # source. Everything that can be wrong here is a KEY NAME, and a key name
    # that is wrong writes a file with a null in it and says nothing.
    import textwrap
    i = lx.index("        # THE DISASTER-AFFECTED AREA")
    j = lx.index("    def _print_scene_banner", i)
    block = textwrap.dedent(lx[i:j])

    wrote = {}

    class _Sa(object):
        @staticmethod
        def write_annotations(name, entries, dirs, quiet=False):
            wrote[name] = entries

        @staticmethod
        def annotation_dirs(repo):
            return ["/nowhere"]

    reg = rg.affected_polygons(KM_CFG, KM_ELAPSED, KM_SPAN, KM_REGION,
                               people_lead_s=383.0)
    st = {"affected_xy": reg["affected"], "burn_xy": reg["burn"],
          "t_burn_s": reg["t_burn_s"], "t_affected_s": reg["t_affected_s"],
          "region": KM_REGION, "scene_config": "suburb_wildfire", "seed": 11,
          "elapsed_s": KM_ELAPSED, "span_s": KM_SPAN,
          "fire_origin_m": list(KM_CFG["origin_m"]),
          "fire_heading_deg": KM_CFG["heading_deg"],
          "lead_s": reg["lead_s"], "lead_bound": reg["lead_bound"],
          "people_lead_s": reg["people_lead_s"],
          "lead_frac": reg["lead_frac"], "margin_s": reg["margin_s"]}
    boxes = [{"class": "person",
              "bbox_world": {"center_xyz_m": [x, y, 0.9]}}
             for x, y in KM_PEOPLE]
    exec(compile(block, "<region annotations>", "exec"),
         {"sa": _Sa, "st": st, "boxes": boxes, "scene": "scn", "repo": "/r",
          "region_poly": rg})

    check(list(wrote) == ["scn_region"],
          "the block writes exactly scn_region.json (%s)" % sorted(wrote))
    ent = wrote.get("scn_region") or []
    check([e["class"] for e in ent] == ["burn", "affected", "region", "meta"],
          "...with burn, affected, region and meta, in that order")
    by = {e["class"]: e for e in ent}
    check(by["affected"]["polygon_xy"] == reg["affected"]
          and by["burn"]["polygon_xy"] == reg["burn"],
          "the polygons written are the ones build_scene computed")
    check(by["affected"]["t_s"] == reg["t_affected_s"]
          and by["burn"]["t_s"] == reg["t_burn_s"],
          "each polygon carries its own T")
    check(rg.polygon_area(by["region"]["polygon_xy"]) == 1.0e6,
          "the region entry is the whole 1 km plat (%.0f m2)"
          % rg.polygon_area(by["region"]["polygon_xy"]))
    meta = by["meta"]
    check(meta["lead_frac"] == rg.DEFAULT_LEAD_FRAC and meta["seed"] == 11
          and meta["elapsed_s"] == KM_ELAPSED and meta["span_s"] == KM_SPAN
          and meta["fire_origin_m"] == list(KM_CFG["origin_m"])
          and meta["fire_heading_deg"] == KM_CFG["heading_deg"],
          "meta carries everything needed to rebuild the polygon")
    check(meta["lead_bound"] == "people" and meta["people_lead_s"] == 383.0
          and meta["lead_s"] == 403.0 and meta["margin_s"] == 20.0,
          "...including WHICH bound sized it and the lead it used (%s, %.0f s)"
          % (meta["lead_bound"], meta["lead_s"]))
    check(all(v is not None for k, v in meta.items() if k != "class"),
          "...and no key of it came out None (a renamed stat would)")
    gate()


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("affected region: %d tests, offline (no pxr, no GPU)" % len(tests))
    broken = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError:
            pass                       # `gate` — already recorded in FAILS
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print("    ERROR %s: %s" % (name, exc))
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
