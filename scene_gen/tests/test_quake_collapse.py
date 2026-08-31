#!/usr/bin/env python3
"""test_quake_collapse.py — does the EARTHQUAKE collapse family take away the
right part, tear every edge of what it leaves, and put the rubble outside the
wall it fell through?

    python3 scene_gen/tests/test_quake_collapse.py
    pytest -q scene_gen/tests/test_quake_collapse.py

WHY THIS EXISTS
---------------
`disaster/quake_collapse.py` rebuilds the earthquake's collapse grades on the
urban-fire partial-collapse mechanism (`disaster/fire_collapse.py`) — the
staircase kill list, `plan_edges` -> `_tear_perimeter` on every surviving
module that touches the hole, `_break(mode="uniform")` so a fragment keeps its
own cladding, and per-fragment outward throw — with the quake's own failure
modes on top and none of the fire's palette. Every decision in that sentence
is arithmetic on the element table, and none of it needs USD:

  * everything that comes away is on the intended elevation(s) and storey(s),
    and an elevation the mode does not name keeps EVERY module;
  * the break line is a STAIRCASE, widening upward, so what is left is a notch
    on the module grid and not a rectangle;
  * every surviving module that touches a dead one is in the tear list with
    the COMPLEMENTARY edge class — that is what stops the hole's outline being
    kit module seams ("the buildings still look like they make clean
    rectangular breaks", user, 2026-08-30);
  * a crush band is exactly ONE storey, bitten into the storeys above and
    below so it is not the clean rectangular slice the round-4 review
    photographed, and the block above lands on the residual heights
    `quake_flow._soft_storey_geometry` claims;
  * every windrow/fan pile is OUTSIDE its own wall line at every yaw — get the
    sign wrong and the rubble is inside a building whose wall went into the
    road;
  * a total collapse keeps a stub only where a neighbour braced the wall;
  * the plan is stable per building and takes ZERO draws off the ladder's
    shared rng, which is what lets these recipes be inserted into a ladder
    without moving any other recipe's outcome.

So the decisions are factored out of the USD authoring into
`quake_collapse.plan_collapse`, and this file checks THAT, on REAL kit
buildings — `urban_building.build_building` + `quake_flow.describe`, pure
placement math, no USD, no Kit, no Isaac Sim. Same shape as
`test_fire_collapse.py`, which is the file it was modelled on.

EVERY INVARIANT HAS A MUTATION CHECK. A test that passes because the thing it
measures cannot fail is a transcript, not a check — so each predicate below is
a module-level `_check_*` function used by BOTH the real test and a
`test_mutation_*` that breaks the plan (flips a heap's sign, flattens the
staircase, un-bites the band, moves a stub onto a street wall) and asserts the
same predicate reports it.

WHAT IT CANNOT SEE: everything downstream of the plan — whether the fan
actually lands where the plan says, whether the block above reads as crushed
rather than lifted, whether `_dust_loose`'s tint is deep enough at 40 m. That
needs a render.
"""

import math
import os
import random
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import urban_building as ub          # noqa: E402
from disaster import fire_collapse as fc         # noqa: E402
from disaster import quake_collapse as qc        # noqa: E402
from disaster import quake_flow as qf            # noqa: E402

# The same seven real kit buildings `test_fire_collapse.py` uses: every MCE
# kit twin plus the two masonry blocks the bench leans on. The families are
# 01/04 (urm), 02 (rc) and 05 (rc_glass), so this spans all three construction
# types the ladder keys on.
STYLES = ("commercial_mid", "commercial", "apartment", "block_residential",
          "highrise_step", "dw_terrace", "brownstone_row")

PARTIAL = ("elevation", "corner")
BAND = ("soft_storey", "mid_storey")
TOTALS = ("pancake", "total")

_PLACEMENTS = {}


def _placements(style, x=0.0, y=0.0, yaw=0.0, seed=7):
    """`ub.build_building` is the slow part of this file; the placements for
    one (style, pose, seed) are deterministic, so build each once."""
    key = (style, x, y, yaw, seed)
    if key not in _PLACEMENTS:
        _PLACEMENTS[key] = ub.build_building(style, x, y, yaw,
                                             random.Random(seed))
    return _PLACEMENTS[key]


def _ctx(style, seed=7, tag="b0", x=0.0, y=0.0, yaw=0.0, neighbours=None):
    """A real kit building, no stage.

    Nothing here is a fixture — the element table, the mass specs, the storey
    of every module and its side are all computed the way a scene computes
    them (`quake_flow.describe`, which is what `wreck_building` calls).
    `describe` is re-run per ctx because the recipes mutate the element
    records they are handed (`dead`), and a shared table would leak between
    tests.
    """
    info = qf.describe(style, _placements(style, x, y, yaw, seed), x, y, yaw)
    ctx = {"info": info, "rng": random.Random(seed),
           "nrng": np.random.default_rng(seed), "notes": [], "tag": tag}
    if neighbours is not None:
        ctx["neighbours"] = list(neighbours)
    return ctx


def _plan(style, mode, **kw):
    ctx = _ctx(style, tag=kw.pop("tag", "b0"))
    return ctx, qc.plan_collapse(ctx, mode=mode, **kw)


def _mass(ctx, plan):
    return ctx["info"]["masses"][plan["mass"]]


def _shell(ctx, plan, side=None, roles=qc.SHELL_ROLES):
    return [e for e in ctx["info"]["elements"]
            if e["mass"] == plan["mass"] and e["role"] in roles
            and (side is None or e["side"] == side)]


def _alive(ctx, plan, side, storey):
    """The modules that are STILL STANDING after the plan — the only ones a
    tear can be demanded of. A stub module is torn at its own height by a
    z-line split of its own, and a panel is carried away whole for the pile to
    lay; neither is a survivor and neither may be `_break_split` a second
    time (that call deactivates its source prim)."""
    killed = set(id(e) for e in plan["kill"])
    killed |= set(id(j["el"]) for j in plan["stub_jobs"])
    killed |= set(id(e) for e in plan["panels"])
    return [e for e in ctx["info"]["elements"]
            if e["mass"] == plan["mass"] and e["role"] in fc.SHELL_ROLES
            and e["side"] == side and int(e["storey"]) == storey
            and id(e) not in killed]


# ---------------------------------------------------------------------------
# THE PREDICATES. Each returns a list of failure strings; each is used by a
# real test AND by a `test_mutation_*` that breaks the plan and asserts the
# predicate reports it.
# ---------------------------------------------------------------------------
def _check_kill_is_on_plan(ctx, plan):
    """Everything that comes away is on a named elevation, at a storey the
    plan swept, inside the span drawn for that (side, storey)."""
    m = _mass(ctx, plan)
    bad = []
    if not plan["kill"]:
        bad.append("nothing taken away")
    for e in plan["kill"]:
        if e["mass"] != plan["mass"]:
            bad.append("killed {0} on mass {1}".format(e["name"], e["mass"]))
            continue
        if e["role"] in qc.ROOF_ROLES:
            # a roof tile is swept by ROLE, not by geometry (its `side` is
            # whichever wall its centre happens to be nearest)
            if plan["roof"] != "kill":
                bad.append("roof {0} killed in mode {1}".format(
                    e["name"], plan["mode"]))
            continue
        if e["role"] not in plan["roles"]:
            bad.append("killed {0}, role {1} not in {2}".format(
                e["name"], e["role"], plan["roles"]))
        if e["side"] not in plan["sides"]:
            bad.append("killed {0} on {1}, not in {2}".format(
                e["name"], e["side"], plan["sides"]))
            continue
        s = int(e["storey"])
        if s not in plan["storeys"]:
            bad.append("killed {0} at storey {1}, not in {2}".format(
                e["name"], s, plan["storeys"]))
            continue
        lo, hi = plan["span"][(e["side"], s)]
        t = qf._p_el_t(m, e["side"], e)
        if not (lo - plan["pad_m"] - 1e-6 <= t <= hi + plan["pad_m"] + 1e-6):
            bad.append("killed {0} at t={1:.2f}, span ({2:.2f}, {3:.2f})"
                       .format(e["name"], t, lo, hi))
    return bad


def _check_untouched_elevations(ctx, plan):
    """The whole point of a PARTIAL collapse: an elevation the mode does not
    name keeps every module, and the storeys below the failure line on the
    lost elevation keep theirs too."""
    bad = []
    killed = set(id(e) for e in plan["kill"])
    for sd in plan["keep_sides"]:
        on_side = _shell(ctx, plan, sd)
        if not on_side:
            continue
        lost = [e for e in on_side if id(e) in killed]
        if lost:
            bad.append("{0} module(s) taken off {1}, which is not lost"
                       .format(len(lost), sd))
    for sd in plan["sides"]:
        below = [e for e in _shell(ctx, plan, sd)
                 if int(e["storey"]) < plan["s0"]]
        if [e for e in below if id(e) in killed]:
            bad.append("a module below the failure line on {0} was taken"
                       .format(sd))
    return bad


def _check_staircase(ctx, plan):
    """`monolith_damage`'s rule, which `fire_collapse` carries and this family
    inherits: a stepped profile on the bay grid that WIDENS UPWARD — never a
    rectangle (`PROFILE_FOOT` = 1.0) and never one long diagonal."""
    m = _mass(ctx, plan)
    bad = []
    for sd in plan["sides"]:
        ws = [(s, plan["span"][(sd, s)][1] - plan["span"][(sd, s)][0])
              for s in plan["storeys"] if (sd, s) in plan["span"]]
        if len(ws) < 2:
            continue
        widths = [w for _s, w in ws]
        for a, b in zip(widths, widths[1:]):
            if b < a - 1e-6:
                bad.append("{0}: profile narrows upward {1}".format(sd, widths))
                break
        if widths[-1] <= widths[0] + 1e-6:
            bad.append("{0}: no step at all, {1}".format(sd, widths))
        L = fc.side_length(m, sd)
        if widths[-1] > L + 1e-6:
            bad.append("{0}: span {1:.2f} wider than the wall {2:.2f}".format(
                sd, widths[-1], L))
    return bad


def _check_edges(ctx, plan):
    """Walked from the DEAD modules outward instead of from the survivors
    inward: for every module that came away, each of its four grid neighbours
    that is still standing must be in the tear list with the COMPLEMENTARY
    class. A neighbour that is not is a kit module seam left as a dead
    straight line on the edge of the hole."""
    m = _mass(ctx, plan)
    byid = dict((id(j["el"]), j) for j in plan["tears"])
    bad = []
    for d in plan["kill"]:
        if d["role"] not in fc.SHELL_ROLES:
            continue
        sd, s = d["side"], int(d["storey"])
        d0, d1 = fc.el_span(m, d)
        for e in _alive(ctx, plan, sd, s):
            t0, t1 = fc.el_span(m, e)
            w = max(0.3, t1 - t0)
            if -0.25 * w <= (t0 - d1) <= 0.6:
                j = byid.get(id(e))
                if j is None or "right" not in j["classes"]:
                    bad.append("{0}: right neighbour of {1} not torn".format(
                        e["name"], d["name"]))
            if -0.25 * w <= (d0 - t1) <= 0.6:
                j = byid.get(id(e))
                if j is None or "left" not in j["classes"]:
                    bad.append("{0}: left neighbour of {1} not torn".format(
                        e["name"], d["name"]))
        for s2, cls in ((s - 1, "below"), (s + 1, "above")):
            for e in _alive(ctx, plan, sd, s2):
                t0, t1 = fc.el_span(m, e)
                over = min(1.2, 0.3 * max(0.3, t1 - t0))
                if min(d1, t1) - max(d0, t0) > over:
                    j = byid.get(id(e))
                    if j is None or cls not in j["classes"]:
                        bad.append("{0}: {1} neighbour of {2} not torn".format(
                            e["name"], cls, d["name"]))
    return bad


def _check_heaps(ctx, plan):
    """Every windrow/fan pile is OUTSIDE its own wall line, under the run that
    fell, and a dome sits on the mass's own centre.

    `quake_rubble._side_axes` places a strip at `+half + offset_m` and reaches
    further out still (and v1's `_heap` draws its windrow distance positive),
    so a plan whose centre comes out NEGATIVE here has buried the rubble in
    the building.
    """
    m = _mass(ctx, plan)
    bad = []
    if not plan["heaps"]:
        bad.append("no pile at all")
    for h in plan["heaps"]:
        if h["kind"] not in ("dome", "windrow", "fan"):
            bad.append("unknown _rubble kind {0!r}".format(h["kind"]))
        if h["kind"] == "dome":
            if h["centres"]:
                bad.append("a dome should have no per-side centre")
            if h["crown_m"] is None or h["crown_m"] <= 0.0:
                bad.append("dome with no crown height")
            continue
        if h["depth_m"] is None or h["depth_m"] <= 0.0:
            bad.append("{0} with no depth".format(h["kind"]))
        if not h["sides"]:
            bad.append("{0} with no side".format(h["kind"]))
        if set(h["centres"]) != set(h["sides"]):
            bad.append("{0}: centres {1} != sides {2}".format(
                h["kind"], sorted(h["centres"]), sorted(h["sides"])))
        for sd, c in h["centres"].items():
            lx, ly = c["local"]
            d = fc.outward_of(m, sd, lx, ly)
            if d <= 0.0:
                bad.append("{0} on {1} at {2:+.2f} m of the wall line, "
                           "wrong side".format(h["kind"], sd, d))
            if abs(d - c["outward_m"]) > 1e-9:
                bad.append("{0} on {1}: reported outward {2} != {3}".format(
                    h["kind"], sd, c["outward_m"], d))
            if d > max(14.0, 0.6 * (m["top"] - m["z0"])):
                bad.append("{0} on {1} at {2:.1f} m: in orbit".format(
                    h["kind"], sd, d))
            # ...and ALONG the wall it is under the part that fell
            t = fc.along_of(m, sd, lx, ly)
            lo, hi = h["along_m"][sd]
            if not (lo - 1e-6 <= t <= hi + 1e-6):
                bad.append("{0} on {1} at t={2:.2f}, run ({3:.2f}, {4:.2f})"
                           .format(h["kind"], sd, t, lo, hi))
            # the world position has to follow the building's yaw
            wx, wy = c["world"]
            blx, bly = qf._to_local(m, wx, wy)
            if math.hypot(blx - lx, bly - ly) > 1e-6:
                bad.append("{0} on {1}: world/local disagree".format(
                    h["kind"], sd))
    return bad


def _check_stub(ctx, plan):
    """A stub survives where the wall was BRACED — the party wall, the blind
    flank. `r_masonry_collapse` keeps one on every side and its DG5 reads as a
    bathtub."""
    bad = []
    for sd in plan["stub"]:
        if sd not in plan["blind_sides"]:
            bad.append("stub on {0}, which is not blind ({1})".format(
                sd, plan["blind_sides"]))
    killed = set(id(e) for e in plan["kill"])
    for j in plan["stub_jobs"]:
        if j["side"] not in plan["blind_sides"]:
            bad.append("stub job on {0}, not blind".format(j["side"]))
        if id(j["el"]) in killed:
            bad.append("{0} is BOTH stubbed and killed".format(
                j["el"]["name"]))
        za = float(j["el"]["z"])
        zb = za + float(j["el"].get("h") or 3.0)
        if not (za < j["z_line"] < zb):
            bad.append("{0}: stub line {1:.2f} outside ({2:.2f}, {3:.2f})"
                       .format(j["el"]["name"], j["z_line"], za, zb))
    return bad


def _check_band(ctx, plan):
    """The crush band is exactly ONE storey and it is BITTEN, not sliced."""
    bad = []
    c = plan["crush"]
    if not c:
        return ["no crush geometry on a band mode"]
    k = c["storey"]
    if plan["band"] != [k]:
        bad.append("band {0} != [{1}]".format(plan["band"], k))
    m = _mass(ctx, plan)
    for sd in plan["sides"]:
        if (sd, k) not in plan["span"]:
            bad.append("{0}: no span at the band storey".format(sd))
            continue
        lo, hi = plan["span"][(sd, k)]
        L = fc.side_length(m, sd)
        if not (lo <= 1e-6 and hi >= L - 1e-6):
            bad.append("{0}: the band does not take the whole wall".format(sd))
    # the band is bitten into a NEIGHBOURING storey, or the seam is a ruler
    bitten = [s for (_sd, s) in plan["span"] if s != k]
    if plan["n_levels"] > 1 and not bitten:
        bad.append("the band is a clean slice: no bite above or below")
    for s in set(bitten):
        if abs(s - k) != 1:
            bad.append("a bite at storey {0}, {1} away from the band".format(
                s, abs(s - k)))
    # ...and nothing outside k and its two neighbours is touched at all
    for e in plan["kill"]:
        if abs(int(e["storey"]) - k) > 1:
            bad.append("{0} killed at storey {1}, band is {2}".format(
                e["name"], e["storey"], k))
    if plan["mode"] == "mid_storey" and k < 1:
        bad.append("a mid-storey band at storey 0")
    return bad


def _check_budget(plan, allow_over=False):
    """Nothing is ever silently dropped, and on the partial modes the cap is
    a real limit the failure line rises to meet.

    On a band / total / pancake the cap is a COST WARNING instead: a total
    collapse that left half the building standing to save fracture time is not
    a total collapse, so the overrun is reported and the modules still go.
    """
    bad = []
    b = plan["budget"]
    if b["modules"] != len(plan["kill"]):
        bad.append("the budget reports {0} module(s), the kill list has "
                   "{1}".format(b["modules"], len(plan["kill"])))
    if b["over_modules"] and not allow_over:
        bad.append("{0} module(s) over the {1} cap".format(
            b["modules"] - b["module_cap"], b["module_cap"]))
    if b["over_edges"]:
        bad.append("{0} edge job(s) dropped".format(b["over_edges"]))
    if b["edges"] > b["edge_cap"]:
        bad.append("{0} edge job(s) over the {1} cap".format(
            b["edges"], b["edge_cap"]))
    return bad


# ---------------------------------------------------------------------------
# A rotation helper, so the band's rigid transform can be applied with no pxr.
# Row-vector convention (p' = p * M), matching `quake_flow._rot_about` /
# `_translate` / `_soft_storey_matrix` — the same helper
# `test_quake_flow_rubble_routing.py` arrived at independently.
# ---------------------------------------------------------------------------
def _rot3(axis, deg):
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = a
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (np.eye(3) * c + K * s + np.outer(a, a) * (1.0 - c)).T


def _apply_band(bt, pt):
    p = np.array(pt, dtype=float) + np.asarray(bt["translate"], dtype=float)
    if bt["pivot"] is not None and abs(bt["deg"]) > 1e-9:
        piv = np.asarray(bt["pivot"], dtype=float)
        p = (p - piv) @ _rot3(bt["axis"], bt["deg"]) + piv
    return p


# ---------------------------------------------------------------------------
# The tests
# ---------------------------------------------------------------------------
def test_the_module_self_check_passes():
    """`quake_collapse.check()` is the same invariants over every mode and
    every style; a launch script gates on it the way it gates on
    `quake_flow.check`."""
    bad = qc.check(verbose=False)
    assert not bad, "\n".join(bad)


@pytest.mark.parametrize("mode", qc.MODES)
def test_everything_killed_is_on_the_intended_sides_and_storeys(mode):
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        bad = _check_kill_is_on_plan(ctx, plan)
        assert not bad, "{0}/{1}: {2}".format(style, mode, bad)


@pytest.mark.parametrize("mode", PARTIAL)
def test_the_untouched_elevations_keep_every_module(mode):
    """`elevation` / `corner` only: the band and total modes go all the way
    round by definition and have no `keep_sides` to protect."""
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        assert plan["keep_sides"], (style, mode, "nothing left untouched")
        bad = _check_untouched_elevations(ctx, plan)
        assert not bad, "{0}/{1}: {2}".format(style, mode, bad)


@pytest.mark.parametrize("mode", PARTIAL + ("pancake",))
def test_the_break_line_is_a_staircase_that_widens_upward(mode):
    seen = 0
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        bad = _check_staircase(ctx, plan)
        assert not bad, "{0}/{1}: {2}".format(style, mode, bad)
        if len(plan["storeys"]) >= 2:
            seen += 1
    assert seen, "no multi-storey building in seven styles?"


def test_a_real_share_of_the_lost_elevation_actually_goes():
    """A "collapse" that takes two modules is a hole. On the elevation mode
    most of the wall above the failure line has to come away — and never all
    of it, or there is no stub left to tell the viewer how tall it was."""
    for style in STYLES:
        ctx, plan = _plan(style, "elevation")
        sd = plan["sides"][0]
        in_band = [e for e in _shell(ctx, plan, sd)
                   if int(e["storey"]) >= plan["s0"]]
        killed = set(id(e) for e in plan["kill"])
        got = len([e for e in in_band if id(e) in killed])
        assert got >= 0.4 * len(in_band), (style, got, len(in_band))


def test_the_construction_type_decides_where_the_failure_line_is():
    """A URM elevation peels from the PARAPET DOWN (the unrestrained top
    storeys); an RC frame does not peel at all — it loses its INFILL from the
    first floor up. Reusing one rule for both is how you build the wrong
    disaster in one step."""
    for style in STYLES:
        ctx, plan = _plan(style, "elevation")
        n = plan["n_levels"]
        if n < 3:
            continue
        if plan["btype"] == "urm":
            assert plan["s0"] >= n - 1 - max(qc.URM_TOP_DOWN), (style,
                                                                plan["s0"], n)
        else:
            assert plan["s0"] >= qc.RC_FROM_STOREY, (style, plan["s0"])
    # ...and the RC infill variant takes PANELS, not the frame
    ctx, plan = _plan("block_residential", "elevation", infill=True)
    assert plan["roles"] == ("wall",), plan["roles"]
    assert all(e["role"] == "wall" for e in plan["kill"])
    assert not plan["drop"], "an infill loss does not drop a floor plate"


@pytest.mark.parametrize("mode", PARTIAL + BAND + ("pancake",))
def test_every_module_touching_the_hole_is_torn_and_knows_which_edge(mode):
    """THE ANSWER TO "SHARP STRAIGHT OR RECTANGULAR CUTS", checked from the
    dead side. `total` is excluded because nothing survives it to tear."""
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        bad = _check_edges(ctx, plan)
        assert not bad, "{0}/{1}: {2}".format(style, mode, bad[:6])


@pytest.mark.parametrize("mode", PARTIAL + BAND)
def test_a_tear_never_takes_the_whole_module(mode):
    """`_break_split` returns statics as well as loose: the FAR portion of a
    torn module stays standing. A cut line outside the module's own extent is
    a module that was killed by accident, and the wall it was holding up then
    reads as floating."""
    for style in STYLES:
        _ctx_, plan = _plan(style, mode)
        for j in plan["tears"]:
            assert j["cuts"], (style, j["name"], "classed but not cut")
            for c in j["cuts"]:
                if c["kind"] == "v":
                    lo, hi = fc.EDGE_PEN
                    assert lo * j["w"] - 1e-9 <= c["pen"] <= hi * j["w"] + 1e-9
                    assert j["t0"] < c["line"] < j["t1"], (style, j["name"])
                elif c["kind"] == "z":
                    lo, hi = (fc.EDGE_PEN if c["cls"] == "below"
                              else fc.EDGE_PEN_ABOVE)
                    assert lo * j["h"] - 1e-9 <= c["pen"] <= hi * j["h"] + 1e-9
                    assert j["za"] < c["line"] < j["zb"], (style, j["name"])
                else:
                    assert 0.0 < c["frac"] <= 0.9, (style, j["name"])
                if c["cls"] == "above":
                    # a module standing ON the hole keeps its footing, or the
                    # storeys over it read as floating
                    assert c["pen"] <= fc.EDGE_PEN_ABOVE[1] * j["h"] + 1e-9


@pytest.mark.parametrize("mode", qc.MODES)
@pytest.mark.parametrize("yaw", (0.0, 37.0, 90.0, 180.0))
def test_the_heaps_are_on_the_right_side_of_the_wall_line_at_every_yaw(mode,
                                                                       yaw):
    """Everything in the plan is in the mass's LOCAL frame and the world
    position has to follow the building's yaw. A heap authored in world
    coordinates from a local offset is the class of bug that puts a windrow
    through a neighbour."""
    for style in ("commercial", "block_residential", "dw_terrace"):
        info = qf.describe(style, _placements(style, 12.0, -7.0, yaw),
                           12.0, -7.0, yaw)
        ctx = {"info": info, "rng": random.Random(5),
               "nrng": np.random.default_rng(5), "notes": [], "tag": "y"}
        plan = qc.plan_collapse(ctx, mode=mode)
        bad = _check_heaps(ctx, plan)
        assert not bad, "{0}/{1}/yaw {2}: {3}".format(style, mode, yaw, bad)


def test_the_heap_kind_and_sides_match_the_mode():
    """One `_rubble` call shape per mode, and it is the shape the recipe it
    replaces uses: a peeled elevation makes a FAN (wider at the toe than at
    the wall — `r_out_of_plane`), a crushed storey makes a WINDROW collar on
    all four sides (`r_soft_storey`), a building that came down into its own
    footprint makes a DOME (`r_masonry_collapse` / `r_pancake`)."""
    want = {"elevation": ("fan",), "corner": ("fan",),
            "soft_storey": ("windrow",), "mid_storey": ("windrow",),
            "pancake": ("dome",), "total": ("dome",)}
    for style in STYLES:
        for mode in qc.MODES:
            ctx, plan = _plan(style, mode)
            kinds = set(h["kind"] for h in plan["heaps"])
            assert kinds <= set(want[plan["mode"]]), (style, mode, kinds)
            if plan["mode"] in ("soft_storey", "mid_storey"):
                h = plan["heaps"][0]
                assert set(h["sides"]) == set("SENW"), (style, h["sides"])
                assert h["base_z"] == plan["crush"]["z_lo"], style
                assert h["elem_h_m"] == plan["crush"]["h_st"], style
            if plan["mode"] in ("elevation", "corner"):
                assert set(h["sides"][0] for h in plan["heaps"]) \
                    == set(plan["sides"]), (style, mode)
            if plan["mode"] == "total":
                h = plan["heaps"][0]
                # the dome's `sides` are the FALL sides: `quake_rubble` gives
                # them the long run-out and the blind sides the flat 0.10 x H
                assert not (set(h["sides"]) & set(plan["blind_sides"])), \
                    (style, h["sides"], plan["blind_sides"])
                assert abs(h["stub_h_m"]
                           - (sum(plan["stub"].values()) / len(plan["stub"])
                              if plan["stub"] else 0.0)) < 1e-9
    # the infill variant is a shallow windrow of dropped panels, not a fan
    _c, plan = _plan("block_residential", "elevation", infill=True)
    assert [h["kind"] for h in plan["heaps"]] == ["windrow"]
    assert qc.INFILL_DEPTH_M[0] <= plan["heaps"][0]["depth_m"] \
        <= qc.INFILL_DEPTH_M[1]


@pytest.mark.parametrize("mode", BAND)
def test_the_band_is_exactly_one_storey_and_is_bitten_not_sliced(mode):
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        if plan["mode"] not in BAND:
            continue          # a one- or two-level mass degrades to soft
        bad = _check_band(ctx, plan)
        assert not bad, "{0}/{1}: {2}".format(style, mode, bad)


def test_the_block_above_a_band_lands_on_the_residuals_the_geometry_claims():
    """`quake_flow._soft_storey_geometry` is REUSED, not re-derived — the
    round-4 fix (pivot the FAR base edge, let the lean side drop to its own
    lower residual) lives there, and the bug it replaced leaned buildings AWAY
    from the side they named. Applying the plan's own transform to the two
    base-wall midpoints has to land them on `r_lean` and `r_far`."""
    seen = 0
    for style in STYLES:
        for mode in BAND:
            ctx, plan = _plan(style, mode)
            c = plan["crush"]
            if c is None:
                continue
            geo = c["geo"]
            m = _mass(ctx, plan)
            bt = qc.band_transform(plan)
            assert bt is not None
            span = m["D"] if c["lean_side"] in ("S", "N") else m["W"]
            if geo["mode"] == "crush":
                want = math.degrees(math.asin(
                    min(1.0, (geo["r_far"] - geo["r_lean"]) / span)))
                assert abs(geo["lean_deg"] - want) < 1e-6, (style, geo)
                nx, ny = qf._SIDE_NORMAL[c["lean_side"]]
                lean_pt = qf._to_world(m, nx * m["W"] / 2.0, ny * m["D"] / 2.0)
                far_pt = qf._to_world(m, -nx * m["W"] / 2.0,
                                      -ny * m["D"] / 2.0)
                after_l = _apply_band(bt, (lean_pt[0], lean_pt[1], c["z_hi"]))
                after_f = _apply_band(bt, (far_pt[0], far_pt[1], c["z_hi"]))
                assert abs(after_l[2] - (c["z_lo"] + geo["r_lean"])) < 0.02, \
                    (style, mode, after_l, geo)
                assert abs(after_f[2] - (c["z_lo"] + geo["r_far"])) < 0.02, \
                    (style, mode, after_f, geo)
            else:
                assert bt["pivot"] is None and bt["deg"] == 0.0, (style, geo)
                ox, oy = qf._outward(m, c["lean_side"])
                for pt in ((m["cx"], m["cy"], c["z_hi"]),
                           (m["cx"] + 3.0, m["cy"] - 4.0, c["z_hi"] + 6.0)):
                    after = _apply_band(bt, pt)
                    want = (pt[0] + ox * geo["offset_m"],
                            pt[1] + oy * geo["offset_m"],
                            pt[2] - geo["drop_m"])
                    assert max(abs(after[i] - want[i]) for i in range(3)) < 1e-9
            # the block never sinks below the storey it crushed onto
            assert min(geo["r_lean"], geo["r_far"]) >= 0.0
            assert max(geo["r_lean"], geo["r_far"]) <= c["h_st"] + 1e-9
            seen += 1
    assert seen, "no band plan drawn at all"


def test_a_mid_storey_band_twists_in_plan_and_a_soft_storey_does_not():
    """The one thing that tells a mid-storey crush from a building that is
    merely shorter: the facade lines break at the seam (Kobe, Mexico City).
    `r_mid_storey`'s own numbers."""
    got_mid = 0
    for style in STYLES:
        _c, plan = _plan(style, "mid_storey")
        if plan["mode"] != "mid_storey":
            continue
        got_mid += 1
        c = plan["crush"]
        assert c["storey"] >= 1, style
        assert 2.0 <= abs(c["twist_deg"]) <= 8.0, (style, c["twist_deg"])
        assert 0.3 <= c["offset_m"] <= 2.0, (style, c["offset_m"])
        _c2, p2 = _plan(style, "soft_storey", storey=0)
        assert p2["crush"]["twist_deg"] == 0.0, style
        assert p2["crush"]["offset_m"] == 0.0, style
    assert got_mid, "no mid-storey band drawn on any style"


def test_pancake_stacks_every_slab_at_the_planned_pitch():
    """`r_pancake`'s own stacking math, reused: a stack of thin plates is
    exactly what PhysX does worst, so the slabs are RE-AUTHORED at a pitch
    rather than simulated — and never ALSO handed to the solver."""
    for style in STYLES:
        _c, plan = _plan(style, "pancake")
        st = plan["stack"]
        assert st, style
        assert qc.PANCAKE_PITCH[0] <= st["pitch_m"] <= qc.PANCAKE_PITCH[1]
        assert len(st["plates"]) == plan["n_levels"], (style, len(st["plates"]))
        zs = [p["z"] for p in st["plates"]]
        for a, b in zip(zs, zs[1:]):
            assert abs((b - a) - st["pitch_m"]) < 1e-9, (style, zs)
        assert zs[0] == st["base_z"] + 0.5 * st["pitch_m"]
        assert st["top_z"] > zs[-1]
        assert not plan["drop"], "a stacked slab must not also be dropped"
        for p in st["plates"]:
            assert 1 <= len(p["sides"]) <= 4
            assert qc.PANCAKE_TILT[0] <= p["tilt_deg"] <= qc.PANCAKE_TILT[1]
        # the TOP plate is the one a nadir camera sees, so it loses more sides
        tops = [p for p in st["plates"] if p["top"]]
        assert tops and all(len(p["sides"]) >= 3 for p in tops), style


def test_total_takes_the_roof_and_the_parapet_ring_too():
    """The one place this family must NOT copy `fire_collapse.SHELL_ROLES`: a
    burnt-out shell keeps its skyline, a collapsed one has no skyline left."""
    assert "roof" not in fc.SHELL_ROLES
    assert "roof" in qc.ALL_ROLES
    for style in STYLES:
        ctx, plan = _plan(style, "total")
        assert plan["roof"] == "kill", style
        spared = set(id(e) for e in plan["kill"])
        spared |= set(id(j["el"]) for j in plan["stub_jobs"])
        spared |= set(id(e) for e in plan["panels"])
        left = [e for e in ctx["info"]["elements"]
                if e["mass"] == plan["mass"] and e["role"] in qc.ALL_ROLES
                and id(e) not in spared]
        assert not left, (style, [e["name"] for e in left[:5]])


def test_the_pile_is_handed_whole_wall_modules_to_lay_on_it():
    """Round-4 design table row 2: "1-3 wall PANELS ... the building's own kit
    modules kept whole, half-buried". A pile made only of fracture cells has
    no large recognisable elements in it, which is one of the five reasons the
    round-3 heap had to go.

    A panel must be in NEITHER `kill` NOR `tears`: `quake_rubble_usd.
    _lay_existing` re-lays the prim itself, so a module that is also fractured
    or `_break_split` is destroyed before the pile ever gets it.
    """
    seen = 0
    for style in STYLES:
        for mode in TOTALS:
            ctx, plan = _plan(style, mode)
            killed = set(id(e) for e in plan["kill"])
            torn = set(id(j["el"]) for j in plan["tears"])
            stubbed = set(id(j["el"]) for j in plan["stub_jobs"])
            assert len(plan["panels"]) <= 2, (style, mode)
            for e in plan["panels"]:
                assert e["role"] == "wall", (style, e["name"])
                assert id(e) not in killed, (style, e["name"], "also killed")
                assert id(e) not in torn, (style, e["name"], "also torn")
                assert id(e) not in stubbed, (style, e["name"], "also stubbed")
                assert e["side"] not in plan["blind_sides"], (style, e["name"])
                seen += 1
            dome = [h for h in plan["heaps"] if h["kind"] == "dome"]
            assert len(dome) == 1
            assert dome[0]["panels"] == plan["panels"], (style, mode)
        # ...and no PARTIAL mode keeps one: a wall left whole there is a wall
        # that simply did not fall.
        for mode in PARTIAL + BAND:
            _c, plan = _plan(style, mode)
            assert not plan["panels"], (style, mode)
    assert seen, "no whole panel kept anywhere in seven styles"


def test_total_keeps_a_stub_only_on_a_blind_side():
    """`r_masonry_collapse(keep_stub=True)` keeps one on EVERY side and its
    DG5 reads as a bathtub. The record is the party wall — the one the
    neighbour braced."""
    seen = 0
    for style in STYLES:
        for mode in TOTALS:
            ctx, plan = _plan(style, mode)
            bad = _check_stub(ctx, plan)
            assert not bad, "{0}/{1}: {2}".format(style, mode, bad)
            seen += len(plan["stub_jobs"])
    assert seen, "no stub kept anywhere in seven styles"
    # ...and a REGISTERED neighbour is what makes a side blind
    ctx = _ctx("commercial", tag="nb",
               neighbours=[{"side": "W", "gap": 0.4, "H": 12.0}])
    plan = qc.plan_collapse(ctx, mode="total")
    assert plan["blind_sides"] == ("W",), plan["blind_sides"]
    assert plan["stub_jobs"] and set(j["side"] for j in plan["stub_jobs"]) \
        == {"W"}
    # a neighbour across a street is not a party wall
    ctx = _ctx("commercial", tag="nb2",
               neighbours=[{"side": "W", "gap": 9.0, "H": 12.0}])
    p2 = qc.plan_collapse(ctx, mode="total")
    assert "W" not in p2["blind_sides"] or not p2["stub"], p2["blind_sides"]


def test_a_total_collapse_and_a_pancake_take_the_wings_too():
    """`urban_building` builds a block as a main mass plus wings, and on the
    tall stock the WINGS are the building: `block_residential` is five masses
    and 2,377 elements with only 181 of them on `main`, `block_stone` a
    one-level base under two eight-storey wings. A "total collapse" that swept
    `main` alone would leave 584 modules of eight-storey wing standing on a
    heap — which is what `r_masonry_collapse` (`for mt, m in
    info["masses"].items()`) and `r_pancake` both already avoid."""
    multi = [s for s in ("block_residential", "block_stone", "highrise_step",
                         "block_office", "tower")
             if len(qf.describe(s, _placements(s), 0.0, 0.0, 0.0)["masses"])
             > 1]
    assert multi, "no multi-mass style to check"
    for style in multi:
        info = qf.describe(style, _placements(style), 0.0, 0.0, 0.0)
        for mode in TOTALS:
            tags = qc.collapse_masses(info, mode)
            assert set(tags) == set(info["masses"]), (style, mode, tags)
            assert tags[0] == "main"
            # an explicit mass always wins
            assert qc.collapse_masses(info, mode, "wing0") == ["wing0"]
        for mode in PARTIAL + BAND:
            assert qc.collapse_masses(info, mode) == ["main"], (style, mode)
        # ...and each mass draws its OWN plan: same tag, different seed
        ctx = _ctx(style)
        seeds = set()
        for mt in qc.collapse_masses(info, "total"):
            p = qc.plan_collapse(ctx, mode="total", mass=mt)
            assert p["mass"] == mt
            assert p["masses"] == [mt]
            seeds.add(p["seed"])
        assert len(seeds) == len(info["masses"]), (style, seeds)


@pytest.mark.parametrize("mode", qc.MODES)
def test_the_budgets_are_respected_and_an_overrun_is_reported(mode):
    band_or_total = mode in BAND + TOTALS
    for style in STYLES:
        _c, plan = _plan(style, mode)
        assert not _check_budget(plan, allow_over=band_or_total), \
            (style, mode, plan["budget"])
        assert len(plan["tears"]) <= qc.MAX_EDGE_MODULES, (style, mode)
    # EVERY MASS, on the modes that sweep them all: `wing3` of
    # `block_residential` is 1,194 elements on its own, so this is the one
    # place the cost of a DG5 in a city is visible offline.
    if mode in TOTALS:
        ctx = _ctx("block_residential")
        over = 0
        for mt in qc.collapse_masses(ctx["info"], mode):
            plan = qc.plan_collapse(ctx, mode=mode, mass=mt)
            assert not _check_budget(plan, allow_over=True), (mt,
                                                              plan["budget"])
            over += plan["budget"]["over_modules"]
        assert over > 0, ("the 5-mass block should trip the cost warning; "
                          "if it stopped doing so the cap moved")
    # ...and a cap small enough to bite RAISES the failure line rather than
    # dropping modules silently: the notch gets SHALLOWER, not narrower, so
    # the staircase profile survives the trim.
    ctx, plan = _plan("highrise_step", "elevation", from_storey=0,
                      max_modules=2)
    assert plan["budget"]["trimmed_storeys"] >= 1, plan["budget"]
    assert plan["s0"] > 0, plan["s0"]
    assert not _check_staircase(ctx, plan), "the trim flattened the profile"
    # ...and where it CANNOT rise, the overrun is REPORTED and the modules are
    # NOT silently dropped — an under-broken elevation is a hole with half its
    # wall still standing in it. Pinned with an explicit `from_storey` so the
    # case is the TWO-STOREY BAND (which has nowhere to rise to) whatever the
    # private draw would have chosen.
    ctx = _ctx("brownstone_row")
    n_lv = len(ctx["info"]["masses"]["main"]["levels"])
    plan = qc.plan_collapse(ctx, mode="elevation", from_storey=n_lv - 2,
                            max_modules=1)
    assert plan["budget"]["trimmed_storeys"] == 0, plan["budget"]
    assert plan["budget"]["over_modules"] > 0, plan["budget"]
    assert len(plan["kill"]) == plan["budget"]["modules"] > 1
    # ...and either way, a plan never quietly loses a module
    _c, plan = _plan("brownstone_row", "elevation", max_modules=4)
    b = plan["budget"]
    assert b["trimmed_storeys"] >= 1 or b["over_modules"] > 0, b
    assert len(plan["kill"]) == b["modules"]
    # same for the edge budget: over the cap the jobs nearest the hole are
    # kept, the rest are MARKED dropped and counted, never quietly missing
    _c, plan = _plan("commercial_mid", "mid_storey", max_edges=3)
    assert plan["budget"]["edge_cap"] == 3
    assert plan["budget"]["over_edges"] > 0, plan["budget"]
    assert len([j for j in plan["tears"] if not j.get("dropped")]) == 3


def test_the_plan_is_stable_and_takes_zero_shared_draws():
    """The whole reason these recipes may be inserted into a ladder: a plan
    drawn after the LADDER's rng has been advanced is the SAME plan."""
    for mode in qc.MODES:
        a = _ctx("commercial_mid", tag="b3")
        b = _ctx("commercial_mid", tag="b3")
        pa = qc.plan_collapse(a, mode=mode)
        for _ in range(50):
            b["rng"].random()
            b["nrng"].random()
        pb = qc.plan_collapse(b, mode=mode)
        assert pa["seed"] == pb["seed"], mode
        assert pa["mode"] == pb["mode"] and pa["s0"] == pb["s0"], mode
        assert pa["sides"] == pb["sides"] and pa["span"] == pb["span"], mode
        assert [e["name"] for e in pa["kill"]] \
            == [e["name"] for e in pb["kill"]], mode
        assert [q["name"] for q in pa["tears"]] \
            == [q["name"] for q in pb["tears"]], mode
        assert [[round(c["pen"], 9) for c in q["cuts"]] for q in pa["tears"]] \
            == [[round(c["pen"], 9) for c in q["cuts"]] for q in pb["tears"]]
        assert [(h["kind"], h["depth_m"], h["crown_m"]) for h in pa["heaps"]] \
            == [(h["kind"], h["depth_m"], h["crown_m"]) for h in pb["heaps"]]
        assert a["rng"].getstate() == random.Random(7).getstate(), \
            "{0}: the plan drew from the SHARED rng".format(mode)
    # a DIFFERENT building gets a different plan, and the seed is stable
    # ACROSS PROCESSES (crc32, never `hash()` — that is salted per process)
    c = _ctx("commercial_mid", tag="b9")
    assert qc.plan_seed(c, "elevation") != qc.plan_seed(
        _ctx("commercial_mid", tag="b3"), "elevation")
    assert qc.plan_seed(_ctx("commercial_mid", tag="b3"), "elevation") \
        == qc.plan_seed(_ctx("commercial_mid", tag="b3"), "elevation")


def test_the_family_is_registered_and_the_eq_ladder_switch_picks_the_table():
    """`EQ_LADDER=qc` (the default) selects `LADDER_QC`; `legacy` reproduces
    today. Every recipe either table names has to exist, or the bake raises
    fourteen seconds into a launch."""
    for name in ("qc_elevation", "qc_corner", "qc_soft_storey",
                 "qc_mid_storey", "qc_pancake", "qc_total", "qc_auto"):
        assert name in qc.RECIPES, name
        assert name in qf.RECIPES, name
    assert not qf.check(verbose=False)
    assert not qc.check(verbose=False)

    assert set(qf.LADDER_QC) == set(qf.LADDER)
    for btype, levels in qf.LADDER_QC.items():
        assert set(levels) == set(qf.LADDER[btype]), btype
        for grade, recs in levels.items():
            for name, kwargs in recs:
                assert name in qf.RECIPES, (btype, grade, name)
                assert isinstance(kwargs, dict)

    # the collapse grades actually route to this family...
    def names(table, btype, grade):
        return [n for n, _kw in table[btype][grade]]

    assert "qc_auto" in names(qf.LADDER_QC, "urm", "DG3")
    assert "qc_elevation" in names(qf.LADDER_QC, "urm", "DG4")
    assert names(qf.LADDER_QC, "urm", "DG5") == ["qc_total"]
    assert "qc_elevation" in names(qf.LADDER_QC, "rc", "DG3")
    assert "qc_soft_storey" in names(qf.LADDER_QC, "rc", "DG4")
    assert names(qf.LADDER_QC, "rc", "DG5") == ["qc_pancake"]
    assert "qc_soft_storey" in names(qf.LADDER_QC, "rc_glass", "DG4")
    # ...and the tower's DG5 is STILL a tilt. No curtain-wall tower has come
    # down in the reviewed record and this family must not change that.
    assert "tilt_sink" in names(qf.LADDER_QC, "rc_glass", "DG5")
    assert not [n for n in names(qf.LADDER_QC, "rc_glass", "DG5")
                if n.startswith("qc_")]

    # THE FREEZE: `LADDER` itself is untouched, so `EQ_LADDER=legacy`
    # reproduces today byte-for-byte, and no `qc_*` name reaches it.
    for btype, levels in qf.LADDER.items():
        for grade, recs in levels.items():
            assert not [n for n, _kw in recs if n.startswith("qc_")], \
                (btype, grade)
    for grade in ("DG0", "DG1", "DG2"):
        for btype in qf.LADDER:
            assert names(qf.LADDER_QC, btype, grade) \
                == names(qf.LADDER, btype, grade), (btype, grade)
    for lvl in ("SETTLE", "TILT", "OV"):
        for btype in qf.LADDER:
            assert names(qf.LADDER_QC, btype, lvl) \
                == names(qf.LADDER, btype, lvl), (btype, lvl)

    prev = qf._LADDER_MODE
    try:
        qf._LADDER_MODE = "qc"
        assert qf.active_ladder() is qf.LADDER_QC
        qf._LADDER_MODE = "legacy"
        assert qf.active_ladder() is qf.LADDER
    finally:
        qf._LADDER_MODE = prev


def test_the_recipes_take_no_fire_semantics_and_no_dust_palette():
    """The freeze on the other side: this family may not reach for the fire's
    palette or for the quake's own PALETTE-SWAP duster.

    `_a_dustify` swaps a known palette identity for a `_dusty` twin and its
    table has no entry for `brick` or for a cladding photo at all, so it
    cannot touch the two things a kit fragment actually carries; `HEAP_MIX` is
    mortar fines over pale brick, which is what made the round-4 heaps read as
    a palette swap rather than as the building that broke. `_dust_loose` —
    a dusted COPY of the fragment's OWN bound material — is the replacement,
    and it is what the module is allowed to call.
    """
    import ast
    import inspect
    tree = ast.parse(inspect.getsource(qc))
    used = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            used.add(node.attr)
        elif isinstance(node, ast.Name):
            used.add(node.id)
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            continue                       # a docstring is not a call
    # AST, not a substring sweep: every one of these names appears in a
    # COMMENT in that module explaining why it is not called, and a grep-style
    # check would fail on its own documentation.
    for banned in ("_a_dustify", "HEAP_MIX", "soot_plume", "_refire",
                   "bind_break", "_heap", "_debris_mat", "_burn_mat",
                   "urban_fire", "burn_zone"):
        assert banned not in used, banned
    assert "_dust_loose" in used
    assert "_rubble" in used
    assert "fire" not in used            # no `ctx["fire"]`, no fire kwargs


# ---------------------------------------------------------------------------
# MUTATION CHECKS — break the plan, assert the named predicate reports it.
# ---------------------------------------------------------------------------
def test_mutation_a_heap_pushed_inside_the_wall_is_caught():
    ctx, plan = _plan("commercial", "elevation")
    assert not _check_heaps(ctx, plan)
    m = _mass(ctx, plan)
    for h in plan["heaps"]:
        for sd, c in h["centres"].items():
            lx, ly = c["local"]
            # mirror the centre back INSIDE the wall line: the sign flip
            nx, ny = qf._SIDE_NORMAL[sd]
            d = fc.outward_of(m, sd, lx, ly)
            c["local"] = (lx - 2.0 * nx * d, ly - 2.0 * ny * d)
            c["world"] = qf._to_world(m, *c["local"])
            c["outward_m"] = fc.outward_of(m, sd, *c["local"])
    bad = _check_heaps(ctx, plan)
    assert bad and any("wrong side" in b for b in bad), bad


def test_mutation_a_rectangular_profile_is_caught():
    """`PROFILE_FOOT = 1.0` is the rectangle the staircase exists to avoid."""
    prev = qc.PROFILE_FOOT
    try:
        qc.PROFILE_FOOT = 1.0
        ctx, plan = _plan("commercial_mid", "elevation")
        bad = _check_staircase(ctx, plan)
        assert bad and any("no step at all" in b for b in bad), bad
    finally:
        qc.PROFILE_FOOT = prev
    ctx, plan = _plan("commercial_mid", "elevation")
    assert not _check_staircase(ctx, plan)


def test_mutation_an_unbitten_band_is_caught():
    """A kill list of exactly `storey == k` is the clean rectangular slice the
    round-4 review photographed."""
    prev = (qc.BAND_BITE_UP, qc.BAND_BITE_DOWN)
    try:
        qc.BAND_BITE_UP = (0.0, 0.0)
        qc.BAND_BITE_DOWN = (0.0, 0.0)
        ctx, plan = _plan("commercial_mid", "soft_storey", storey=1)
        bad = _check_band(ctx, plan)
        assert bad and any("clean slice" in b for b in bad), bad
    finally:
        qc.BAND_BITE_UP, qc.BAND_BITE_DOWN = prev
    ctx, plan = _plan("commercial_mid", "soft_storey", storey=1)
    assert not _check_band(ctx, plan)


def test_mutation_a_stub_on_a_street_wall_is_caught():
    ctx, plan = _plan("commercial", "total")
    assert not _check_stub(ctx, plan)
    street = [s for s in ("S", "E", "N", "W") if s not in plan["blind_sides"]]
    plan["stub"][street[0]] = 1.4
    bad = _check_stub(ctx, plan)
    assert bad and any("not blind" in b for b in bad), bad


def test_mutation_an_untorn_neighbour_is_caught():
    """Drop one tear job and the dead-side walk has to name the module that
    kept a factory edge."""
    ctx, plan = _plan("commercial_mid", "elevation")
    assert not _check_edges(ctx, plan)
    assert plan["tears"], "nothing torn at all?"
    plan["tears"] = plan["tears"][1:]
    bad = _check_edges(ctx, plan)
    assert bad and any("not torn" in b for b in bad), bad


def test_mutation_a_module_taken_off_a_cold_elevation_is_caught():
    ctx, plan = _plan("commercial", "elevation")
    assert not _check_untouched_elevations(ctx, plan)
    sd = plan["keep_sides"][0]
    victim = _shell(ctx, plan, sd)[0]
    plan["kill"].append(victim)
    bad = _check_untouched_elevations(ctx, plan)
    assert bad and any("not lost" in b for b in bad), bad
    assert _check_kill_is_on_plan(ctx, plan)


def test_mutation_a_module_outside_its_own_span_is_caught():
    ctx, plan = _plan("commercial", "elevation")
    assert not _check_kill_is_on_plan(ctx, plan)
    sd = plan["sides"][0]
    lo, hi = plan["span"][(sd, plan["top_storey"])]
    plan["span"][(sd, plan["top_storey"])] = (lo, lo + 0.01)
    bad = _check_kill_is_on_plan(ctx, plan)
    assert bad and any("span" in b for b in bad), bad


if __name__ == "__main__":
    import inspect as _inspect
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        marks = getattr(fn, "pytestmark", [])
        if marks:
            continue                     # parametrized: run under pytest
        if _inspect.signature(fn).parameters:
            continue
        fn()
        print("ok  " + name)
