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


def test_infill_removal_still_rags_the_exposed_floor_edge():
    """rect-cutouts review (2026-08-31): the slab an infill panel used to
    hide behind is authored by `fit_interior` exactly like any other floor
    slab — a plain rectangular box — so it needs `quake_flow._ragged_slabs`
    on the failed side just as much as a masonry elevation/corner loss does.
    `_author_one` used to skip the break entirely when `plan["infill"]` was
    set (`elif ... and not plan["infill"]:`), leaving a machined-straight
    slab edge behind every RC DG3 infill opening
    (`LADDER_QC["rc"]["DG3"]` -> `qc_elevation(infill=True)`). The fix reads
    the decision off the PLAN (`floor_ragged`) instead of re-deriving
    `mode`/`infill` at authoring time, so this is checkable with no stage."""
    _ctx_i, plan_i = _plan("block_residential", "elevation", infill=True)
    assert plan_i["floor_ragged"] is True, (
        "an infill elevation plan must still ask for a ragged floor edge")
    # ...and the non-infill path is unchanged: still ragged.
    _ctx_p, plan_p = _plan("block_residential", "elevation", infill=False)
    assert plan_p["floor_ragged"] is True
    _ctx_c, plan_c = _plan("apartment", "corner")
    assert plan_c["floor_ragged"] is True
    # band/total/pancake modes handle their own exposed slabs a different
    # way (`_a_slab_rim` for a crush band; every slab is broken into boards
    # or re-authored as a stack for total/pancake) and must NOT also run
    # `_ragged_slabs` a second time on the same slab.
    for mode in BAND + TOTALS:
        _ctx_o, plan_o = _plan("block_residential", mode)
        assert plan_o["floor_ragged"] is False, (mode, plan_o["floor_ragged"])


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
                # The SECOND RING draws from its own, smaller ranges (it
                # cracks and spalls, it does not peel) — a ring cut judged
                # against `fc.EDGE_PEN` would be "too small", which is the one
                # direction this test does not care about. Every other
                # invariant is the same for both rings.
                ring = bool(c.get("ring"))
                if c["kind"] == "v":
                    lo, hi = ((min(qc.RING_PEN2[0], qc.RING_DIAG_PEN[0],
                                   qc.RING_CORNER_PEN[0], qc.RING_SPALL_PEN[0]),
                               max(qc.RING_PEN2[1], qc.RING_DIAG_PEN[1],
                                   qc.RING_CORNER_PEN[1], qc.RING_SPALL_PEN[1]))
                              if ring else fc.EDGE_PEN)
                    assert lo * j["w"] - 1e-9 <= c["pen"] <= hi * j["w"] + 1e-9
                    assert j["t0"] < c["line"] < j["t1"], (style, j["name"])
                elif c["kind"] == "z":
                    lo, hi = ((min(qc.RING_DIAG_PEN[0], qc.RING_TOP_PEN[0]),
                               max(qc.RING_DIAG_PEN[1], qc.RING_TOP_PEN[1]))
                              if ring else
                              (fc.EDGE_PEN if c["cls"] == "below"
                               else fc.EDGE_PEN_ABOVE))
                    assert lo * j["h"] - 1e-9 <= c["pen"] <= hi * j["h"] + 1e-9
                    assert j["za"] < c["line"] < j["zb"], (style, j["name"])
                else:
                    assert 0.0 < c["frac"] <= 0.9, (style, j["name"])
                if c["cls"] == "above":
                    # a module standing ON the hole keeps its footing, or the
                    # storeys over it read as floating
                    cap_a = (qc.RING_DIAG_PEN[1] if ring
                             else fc.EDGE_PEN_ABOVE[1])
                    assert c["pen"] <= cap_a * j["h"] + 1e-9


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
        assert (plan["budget"]["edges"] <= qc.MAX_EDGE_MODULES
                and plan["budget"]["ring_new"] <= plan["budget"]["ring_cap"]
                and len(plan["tears"]) == (plan["budget"]["edges"]
                                           + plan["budget"]["ring_new"])), \
            (style, mode, plan["budget"])
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
    # ...the FIRST ring only. `max_edges` is `fire_collapse.plan_edges`'s cap;
    # the second ring answers to `QC_RING_MAX` and is counted separately, and
    # it never picks up a module `plan_edges` dropped (that prim would be
    # split twice).
    first = [j for j in plan["tears"]
             if not j.get("ring") and not j.get("dropped")]
    assert len(first) == 3, len(first)
    dropped = set(id(j["el"]) for j in plan["tears"] if j.get("dropped"))
    assert not [j for j in plan["tears"]
                if j.get("ring") and id(j["el"]) in dropped]


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


# ---------------------------------------------------------------------------
# ROOF PROPS THAT LOSE THEIR ROOF — `quake_collapse.roof_prop_footprint_lost`
#
# The AUTHORING half (`_sweep_roof_props` / `_deck_support_z`: real
# `ctx["roof_plant"]` paths, `UsdGeom.BBoxCache`, the geometric support probe,
# `quake_flow._a_bury_props`) needs a stage — the `test_deck_support_z_*` and
# `test_sweep_roof_props_*` tests below build a real (in-memory) one, exactly
# because round 6 shipped a real bug (`_deck_support_z` missing a wide slab
# entirely) that no purely-planner test could have caught. What IS pure — no
# `pxr`, no ctx even — is the DECISION `roof_prop_footprint_lost` makes: does
# a prop's own footprint sit over roof area this plan just killed. That
# decision is what a real footprint (measured off the stage) and a synthetic
# one in this file are handed to identically.
# ---------------------------------------------------------------------------
def _region_pts(plan, m):
    """One WORLD point safely INSIDE the plan's lost region (half its depth
    in from the wall, the middle of its span) and one safely OUTSIDE it (well
    past the region's own depth) — `elevation` / `corner` only."""
    sd = plan["sides"][0]
    lo, hi, dep = plan["region"][sd]
    t_mid = 0.5 * (lo + hi)
    inside = qf._to_world(m, *fc.wall_point(m, sd, t_mid, out_m=-(dep * 0.5)))
    outside = qf._to_world(m, *fc.wall_point(m, sd, t_mid, out_m=-(dep + 20.0)))
    return inside, outside


@pytest.mark.parametrize("mode", PARTIAL)
def test_roof_prop_over_the_lost_region_falls_the_rest_does_not(mode):
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        m = ctx["info"]["masses"][plan["mass"]]
        inside, outside = _region_pts(plan, m)
        assert qc.roof_prop_footprint_lost(plan, m, [inside] * 5) is True, \
            (style, mode)
        assert qc.roof_prop_footprint_lost(plan, m, [outside] * 5) is False, \
            (style, mode)


@pytest.mark.parametrize("mode", PARTIAL)
def test_roof_prop_majority_footprint_not_centre_decides(mode):
    """A footprint of 5 points where 3 sit one way and 2 the other has to
    follow the 3, whichever position in the list plays "the centre" — the
    fire's own `_mostly_in_hole` lesson (bug 9): the centre ALONE is never
    enough to decide for a prop whose footprint spans the boundary."""
    ctx, plan = _plan("commercial_mid", mode)
    m = ctx["info"]["masses"][plan["mass"]]
    inside, outside = _region_pts(plan, m)
    # 3 inside + a "centre" outside: majority wins, the prop still falls
    assert qc.roof_prop_footprint_lost(
        plan, m, [inside, inside, inside, outside, outside]) is True
    # 3 outside + a "centre" inside: majority wins, the prop stays put
    assert qc.roof_prop_footprint_lost(
        plan, m, [outside, outside, outside, inside, inside]) is False
    # a literal tie (2 of 4, ignoring the 5th) must not read as a majority
    assert qc.roof_prop_footprint_lost(
        plan, m, [inside, inside, outside, outside]) is False


@pytest.mark.parametrize("mode", TOTALS)
def test_roof_prop_always_falls_on_total_or_pancake(mode):
    """The whole roof of this mass is gone either way, so ANY point on it —
    there is no surviving part of THIS mass's roof left to spare one."""
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        m = ctx["info"]["masses"][plan["mass"]]
        far_corner = qf._to_world(m, 0.49 * m["W"], 0.49 * m["D"])
        centre = qf._to_world(m, 0.0, 0.0)
        assert qc.roof_prop_footprint_lost(plan, m, [far_corner]) is True, \
            (style, mode)
        assert qc.roof_prop_footprint_lost(plan, m, [centre] * 5) is True, \
            (style, mode)


@pytest.mark.parametrize("mode", BAND)
def test_roof_prop_never_resolved_on_a_band_mode(mode):
    """`soft_storey` / `mid_storey` are NOT in `ROOF_PROP_MODES`:
    `_author_band` already carries the roof (and whatever `_els` finds on
    it) down with the block above when the band reaches that high, so this
    family's own roof-prop sweep has nothing to add and must stay out of the
    way — every point, anywhere on the roof, stays put."""
    assert mode not in qc.ROOF_PROP_MODES
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        m = ctx["info"]["masses"][plan["mass"]]
        centre = qf._to_world(m, 0.0, 0.0)
        assert qc.roof_prop_footprint_lost(plan, m, [centre] * 5) is False, \
            (style, mode)


def test_roof_prop_fall_key_present_and_empty_on_a_bare_plan():
    """`plan_collapse` never touches a stage, so it cannot know a single real
    prop path — the key is present and empty in EVERY mode, never absent,
    exactly like every other key `plan_collapse`'s own docstring promises."""
    for mode in qc.MODES:
        ctx, plan = _plan("commercial_mid", mode)
        assert plan["roof_prop_fall"] == [], mode


def test_roof_prop_modes_are_exactly_the_ones_that_kill_the_roof():
    assert set(qc.ROOF_PROP_MODES) == {"elevation", "corner", "total",
                                        "pancake"}
    assert not set(qc.ROOF_PROP_MODES) & set(BAND)


def test_roof_prop_classification_takes_no_shared_draws():
    """`roof_prop_footprint_lost` takes a `plan` and an `m`, never a `ctx` —
    so calling it, however many times, cannot move a shared rng that it was
    never handed in the first place."""
    ctx = _ctx("commercial_mid")
    for mode in qc.MODES:
        plan = qc.plan_collapse(ctx, mode=mode)
        m = ctx["info"]["masses"][plan["mass"]]
        pt = qf._to_world(m, 0.0, 0.0)
        for _ in range(5):
            qc.roof_prop_footprint_lost(plan, m, [pt, pt, pt, pt, pt])
    assert ctx["rng"].getstate() == random.Random(7).getstate(), \
        "the roof-prop classifier drew from the SHARED rng"


# ---------------------------------------------------------------------------
# ROUND 6: `_deck_support_z` and `_sweep_roof_props`'s GEOMETRIC placement —
# a real stage, on purpose (see the section note above).
# ---------------------------------------------------------------------------
def _stage_box(stage, path, cx, cy, cz, sx, sy, sz):
    """One box Mesh, world-axis-aligned, no external reference — the same
    shape every kit slab/wall piece in this codebase authors
    (`quake_flow._box`'s own construction), so a synthetic stage exercises
    the SAME "one big quad per face" case a real bake does."""
    from pxr import Gf, Sdf, UsdGeom
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            pts.append(Gf.Vec3f(cx + dx, cy + dy, cz + dz))
    faces, counts = [], []
    for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
              (2, 3, 7, 6), (3, 0, 4, 7)):
        faces.extend(f)
        counts.append(4)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh.GetPrim()


def _empty_stage(root="/World/b0"):
    from pxr import Sdf, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    return stage, root


def _partition_ctx(stage, root, seed=13):
    """Minimal real-material context for the fallen-partition fracture."""
    from pxr import Sdf, UsdShade
    mats = {}
    for key in ("plaster", "plaster_dusty"):
        mats[key] = UsdShade.Material.Define(
            stage, Sdf.Path(root + "/Looks/" + key))
    return {"stage": stage, "parent": root, "mats": mats,
            "rng": random.Random(seed),
            "nrng": np.random.default_rng(seed), "notes": [],
            "loose": [], "static_extra": [], "velocity": {},
            "fit": {"slabs": {}, "columns": {}, "partitions": [],
                    "all": []},
            "info": {"elements": []}}


def test_fallen_partition_is_fractured_before_physics_not_dropped_as_a_sheet():
    """Regression for the measured highrise_04 DG5 white-wall defect.

    A 10 x 3 m, 0.12 m-thick divider used to enter settle as this exact
    eight-vertex box.  It must now leave several bounded, non-box fragments;
    supported partitions are safe because only `_fall_fitout` calls the
    helper after its support/region test passes.
    """
    from pxr import UsdGeom
    stage, root = _empty_stage()
    path = root + "/fit_b0/part_main_3_1"
    _stage_box(stage, path, 0.0, 0.0, 6.0, 0.12, 10.0, 3.0)
    ctx = _partition_ctx(stage, root)

    made, fractured = qc._fracture_fallen_partition(ctx, path)

    assert fractured, ctx["notes"]
    assert 4 <= len(made) <= 18, len(made)
    assert not stage.GetPrimAtPath(path).IsActive(), \
        "the complete room wall remained active behind its fragments"
    point_counts = []
    longest = []
    for p in made:
        prim = stage.GetPrimAtPath(p)
        assert prim.IsValid() and prim.IsActive(), p
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        assert pts and mesh.GetFaceVertexIndicesAttr().Get(), p
        point_counts.append(len(pts))
        xyz = np.asarray([[float(q[0]), float(q[1]), float(q[2])]
                          for q in pts])
        longest.append(float(np.max(xyz.max(0) - xyz.min(0))))
    assert max(longest) < 3.5, \
        "a near-whole partition sheet survived: {0}".format(max(longest))
    assert any(n != 8 for n in point_counts), \
        "all outputs are still perfect eight-corner boxes"


def test_fallen_partition_fracture_does_not_consume_shared_rngs():
    """The appearance fix may not move any later collapse decision."""
    stage, root = _empty_stage()
    path = root + "/fit_b0/part_main_3_1"
    _stage_box(stage, path, 0.0, 0.0, 6.0, 0.12, 8.0, 3.0)
    ctx = _partition_ctx(stage, root, seed=31)
    py_before = ctx["rng"].getstate()
    np_before = repr(ctx["nrng"].bit_generator.state)

    made, fractured = qc._fracture_fallen_partition(ctx, path)

    assert fractured and made
    assert ctx["rng"].getstate() == py_before
    assert repr(ctx["nrng"].bit_generator.state) == np_before


def test_detached_rectangle_gate_never_breaks_a_supported_slab(monkeypatch):
    """The global rule applies to DETACHED members, not sound structure."""
    stage, root = _empty_stage()
    path = root + "/fit_b0/slab_main_9"
    _stage_box(stage, path, 0.0, 0.0, 12.0, 32.0, 18.0, 0.28)
    ctx = _partition_ctx(stage, root)
    ctx["fit"]["slabs"][("main", 9)] = path
    ctx["fit"]["all"].append(path)

    report = qc.normalize_detached_rectangles(ctx)

    assert report["replaced"] == 0
    assert report["violations"] == []
    assert stage.GetPrimAtPath(path).IsActive(), \
        "a supported floor was fractured merely because it is rectangular"


def test_detached_rectangle_gate_fractures_a_whole_fallen_slab(monkeypatch):
    """A building-width plate may not reach PhysX as one pristine box."""
    stage, root = _empty_stage()
    path = root + "/fit_b0/slab_main_9"
    _stage_box(stage, path, 0.0, 0.0, 12.0, 32.0, 18.0, 0.28)
    ctx = _partition_ctx(stage, root, seed=47)
    ctx["fit"]["slabs"][("main", 9)] = path
    ctx["fit"]["all"].append(path)
    ctx["loose"].append(path)
    # Chipping is separately covered.  This test isolates the mandatory
    # replacement and avoids making an optional cosmetic mesh pass its oracle.
    monkeypatch.setattr(qc, "_chip_pieces", lambda *a, **k: 0)
    py_before = ctx["rng"].getstate()
    np_before = repr(ctx["nrng"].bit_generator.state)

    report = qc.normalize_detached_rectangles(ctx)

    assert report["replaced"] == 1, report
    assert report["pieces"] >= 4, report
    assert report["violations"] == [], report
    assert not stage.GetPrimAtPath(path).IsActive()
    assert path not in ctx["loose"] and path not in ctx["fit"]["all"]
    assert len(ctx["loose"]) == report["pieces"]
    assert all(stage.GetPrimAtPath(p).IsActive() for p in ctx["loose"])
    assert all(qc._simple_rect_info(ctx, p) is None for p in ctx["loose"]), \
        "the gate replaced one box with more pristine box topology"
    assert ctx["rng"].getstate() == py_before
    assert repr(ctx["nrng"].bit_generator.state) == np_before


def test_real_kit_roof_name_is_classified_from_element_provenance():
    """Regression for bld_office_plain_roof_7_210 from the 250 m review."""
    stage, root = _empty_stage()
    path = root + "/bld_office_plain_roof_7_210"
    _stage_box(stage, path, 0.0, 0.0, 25.0, 8.0, 8.0, 0.0)
    ctx = _partition_ctx(stage, root)
    ctx["info"]["elements"].append(
        {"role": "roof", "p": {"prim_path": path}})
    ctx["loose"].append(path)

    bad = qc._oversized_detached_rect(ctx, path)

    assert bad is not None and bad[0] == "roof"


def test_foundation_tilt_does_not_author_a_pristine_frame_on_every_storey():
    """Only the locally distressed lower bay needs synthetic fit-out."""
    assert qf._fit_scope_for_recipes(qf.FOUNDATION["TILT"]) == (1,)
    assert qf._fit_scope_for_recipes(qf.FOUNDATION["SETTLE"]) == ()
    assert qf._fit_scope_for_recipes(qf.FOUNDATION["OV"]) is None
    assert qf._fit_scope_for_recipes(qf.LADDER["rc"]["DG3"]) is None
    assert qf._fit_scope_for_recipes(qf.FOUNDATION["TILT"], (4, 5)) == (4, 5)


def test_deck_support_z_finds_a_wide_slab_not_just_its_triangle_centroid():
    """THE BUG THIS PINS: a kit slab is ONE big quad (`quake_flow._box`'s own
    shape), and a naive "is the TRIANGLE'S CENTROID inside the query
    rectangle" test never fires for a small prop over the middle of a wide
    one — a fan triangulation of a 30 x 10 m quad centres its two triangles a
    quarter and three-quarters along the long axis, never at the middle.
    `_deck_support_z` samples the QUERY rectangle's own corners + centre
    against each candidate triangle instead (point-in-triangle), so it has to
    find a 30 m-wide slab from a 2 m-wide query centred on it."""
    from pxr import Usd, UsdGeom
    stage, root = _empty_stage()
    _stage_box(stage, root + "/roof_west", -10, 0, 10, 10, 10, 0.3)
    _stage_box(stage, root + "/roof_east", 10, 0, 10, 10, 10, 0.3)
    _stage_box(stage, root + "/floor_below", 0, 0, 6.5, 30, 10, 0.3)
    tank = root + "/tank_x_1"
    _stage_box(stage, tank, 0, 0, 11.02, 1.6, 1.6, 2.0)   # base at z=10.02

    support = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=1.0,
                                 half_d=1.0, z_ceiling=10.02, exclude={tank})
    assert support is not None, \
        "missed the wide floor slab entirely (the round-6 bug)"
    assert abs(support - 6.65) < 0.05, support     # NOT the roof at ~10.3

    # over the surviving west roof instead: a NARROWER candidate this time
    tank2 = root + "/tank_x_2"
    _stage_box(stage, tank2, -10, 0, 11.15, 1.6, 1.6, 2.0)
    support2 = qc._deck_support_z(stage, root, cx=-10.0, cy=0.0, half_w=1.0,
                                  half_d=1.0, z_ceiling=10.15, exclude={tank2})
    assert support2 is not None and abs(support2 - 10.15) < 0.05, support2


def test_deck_support_z_ignores_what_is_above_and_returns_none_over_a_true_gap():
    stage, root = _empty_stage()
    _stage_box(stage, root + "/floor_below", 0, 0, 6.5, 30, 10, 0.3)
    # a slab ABOVE the query ceiling must never count as "support"
    _stage_box(stage, root + "/roof_overhead", 0, 0, 20.0, 30, 10, 0.3)
    support = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=1.0,
                                 half_d=1.0, z_ceiling=10.02)
    assert support is not None and abs(support - 6.65) < 0.05, support
    # nothing at all under a spot far from every authored box
    gone = qc._deck_support_z(stage, root, cx=500.0, cy=500.0, half_w=1.0,
                              half_d=1.0, z_ceiling=10.02)
    assert gone is None


def _office_wide_elevation_plan():
    ctx0 = _ctx("office_wide", tag="b0")
    plan = qc.plan_collapse(ctx0, mode="elevation")
    m = ctx0["info"]["masses"][plan["mass"]]
    return ctx0["info"], plan, m


def _rig_stage_with_tank(info, plan, m, root="/World/stage/generated/b0"):
    """A stage with ONE tank sitting where `dress_roof` would have put it
    (roof height + 2 cm), over a WORLD point this plan's own region test
    already calls "lost" (`_region_pts`'s own construction), plus a
    surviving floor one storey down and the ground further below — enough
    for `_sweep_roof_props` to have something real to resolve against."""
    stage, _ = _empty_stage(root)
    sd = plan["sides"][0]
    lo, hi, dep = plan["region"][sd]
    t_mid = 0.5 * (lo + hi)
    ix, iy = qf._to_world(m, *fc.wall_point(m, sd, t_mid, out_m=-(dep * 0.5)))
    tank_z0 = float(m["top"]) + 0.02
    tank = root + "/tank_b0_1"
    _stage_box(stage, tank, ix, iy, tank_z0 + 1.0, 1.6, 1.6, 2.0)
    floor_z = float(m["levels"][-1])
    _stage_box(stage, root + "/floor_below", m["cx"], m["cy"], floor_z,
              m["W"] * 1.2, m["D"] * 1.2, 0.3)
    _stage_box(stage, root + "/ground", m["cx"], m["cy"], float(m["z0"]),
              m["W"] * 3.0, m["D"] * 3.0, 0.2)
    return stage, tank, tank_z0, floor_z


def _rig_ctx(stage, tank, seed=11, parent="/World/stage/generated/b0"):
    return {"stage": stage, "parent": parent,
           "rng": random.Random(seed), "nrng": np.random.default_rng(seed),
           "roof_plant": [tank], "roof_fixed": [], "roof_plant_mass": "main",
           "loose": [], "static_extra": [], "velocity": {}, "notes": [],
           "tag": "b0"}


def test_sweep_roof_props_places_a_fallen_tank_on_the_real_deck_not_a_kick():
    """THE PRODUCTION BUG, reproduced and pinned: round 5's velocity-kick
    mechanism froze `bld_brownstone_row_DG3.usd`'s tank ~1 m into a required
    ~3 m fall and `bld_office_wide_DG4.usd`'s ~2-4.7 m short. This building
    (`office_wide`, `elevation`) has the tank fall a full storey (>half of
    `ROOF_PROP_BIG_TIP_STOREY_FRAC`), so it must land on the real floor
    slab — not the kick's 0.3-0.9 m/s nudge and a hopeful settle — get a
    real tip/roll (not stay bolt upright), come out as STATIC geometry (no
    rigid body / step-budget dependency), and land in the SAME place for the
    same seed."""
    from pxr import Gf, Usd, UsdGeom
    info, plan, m = _office_wide_elevation_plan()
    stage, tank, tank_z0, floor_z = _rig_stage_with_tank(info, plan, m)
    ctx = _rig_ctx(stage, tank)

    n_fall, n_buried = qc._sweep_roof_props(ctx, plan, m, random.Random(99))
    assert n_fall == 1 and n_buried == 0
    assert tank not in ctx["roof_plant"], "not removed from roof_plant"
    assert ctx["loose"] == [], "must not be a rigid body — no settle dependency"
    assert tank in ctx["static_extra"]

    pr = stage.GetPrimAtPath(tank)
    assert pr.IsValid() and pr.IsActive(), "must not be deactivated (keep=1.0)"
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    assert r.GetMin()[2] < tank_z0 - 1.0, "did not fall at all"
    assert abs(r.GetMin()[2] - (floor_z + 0.15)) < 0.6, \
        ("landed far from the real floor slab", r.GetMin()[2], floor_z)

    xf = UsdGeom.XformCache()
    tr = Gf.Transform(xf.GetLocalToWorldTransform(pr))
    assert tr.GetRotation().GetAngle() > 5.0, \
        "landed bolt upright on a > half-storey drop"

    # determinism: same building, same seeds -> the same landing
    stage2, tank2, _, _ = _rig_stage_with_tank(info, plan, m)
    ctx2 = _rig_ctx(stage2, tank2)
    qc._sweep_roof_props(ctx2, plan, m, random.Random(99))
    pr2 = stage2.GetPrimAtPath(tank2)
    t1 = Gf.Transform(xf.GetLocalToWorldTransform(pr)).GetTranslation()
    t2 = Gf.Transform(xf.GetLocalToWorldTransform(pr2)).GetTranslation()
    assert (t1 - t2).GetLength() < 1e-6, "same seed must land the prop identically"


def test_sweep_roof_props_small_drop_gets_the_idle_tip_not_the_bury_dressing():
    """A drop shallower than `ROOF_PROP_BIG_TIP_STOREY_FRAC` of the top
    storey's own height is not a "fell off the building" drop — it gets the
    same small idle tip `quake_flow._b_settle_roof_plant` gives a prop
    nobody kicked over (`quake_flow.B_ROOF_PLANT_TIP_DEG`), not the DG5
    bury path's 20-75 deg roll."""
    from pxr import Gf, Usd, UsdGeom
    info, plan, m = _office_wide_elevation_plan()
    root = "/World/stage/generated/b0"
    stage, root = _empty_stage(root)
    sd = plan["sides"][0]
    lo, hi, dep = plan["region"][sd]
    t_mid = 0.5 * (lo + hi)
    ix, iy = qf._to_world(m, *fc.wall_point(m, sd, t_mid, out_m=-(dep * 0.5)))
    tank_z0 = float(m["top"]) + 0.02
    tank = root + "/tank_b0_1"
    _stage_box(stage, tank, ix, iy, tank_z0 + 1.0, 1.6, 1.6, 2.0)
    # a deck only 0.3 m under the tank's own base — a shallow drop
    _stage_box(stage, root + "/deck_just_under", m["cx"], m["cy"],
              tank_z0 - 0.3, m["W"] * 1.2, m["D"] * 1.2, 0.3)
    ctx = _rig_ctx(stage, tank, parent=root)

    n_fall, _ = qc._sweep_roof_props(ctx, plan, m, random.Random(5))
    assert n_fall == 1
    pr = stage.GetPrimAtPath(tank)
    xf = UsdGeom.XformCache()
    tr = Gf.Transform(xf.GetLocalToWorldTransform(pr))
    assert tr.GetRotation().GetAngle() <= qf.B_ROOF_PLANT_TIP_DEG + 1e-6, \
        "a barely-fallen prop should barely lean"


# ---------------------------------------------------------------------------
# ROUND 7: `_deck_support_z`'s per-prim AABB prune must never reject a mesh
# on its TOP — only on its BOTTOM. See `_deck_support_z`'s own "ROUND 7" note
# and `disaster/quake_sliced.py::_reseat_roof_plant`'s docstring (the
# reference for the same class of bug, on the reseat side): a merged
# GAC/kit prim can legitimately span multiple heights when a coping run or
# raised section elsewhere on the SAME prim sits well above a genuine deck
# under THIS footprint. Pruning on `hi[2]` threw the whole candidate mesh
# out whenever ANY part of it (however far from the query XY) cleared the
# ceiling; the fix prunes only on `lo[2]` — a mesh is a candidate whenever
# its z-range could contain a triangle at or below `z_ceiling + margin`,
# regardless of how much higher the rest of it reaches.
# ---------------------------------------------------------------------------
def _stage_merged_boxes(stage, path, boxes):
    """ONE Mesh prim built from several axis-aligned boxes concatenated into
    a single points/faceVertexCounts/faceVertexIndices triple — the same
    shape a real bake's merge produces when a coping run and a flat deck,
    authored as separate pieces, end up sharing one exported prim. `boxes`
    is an iterable of `(cx, cy, cz, sx, sy, sz)`, each turned into 8 points
    and 6 quad faces exactly like `_stage_box`, offset into one shared index
    space so the whole thing is a SINGLE mesh (a single world AABB), not
    several."""
    from pxr import Gf, Sdf, UsdGeom
    pts, faces, counts = [], [], []
    for cx, cy, cz, sx, sy, sz in boxes:
        base = len(pts)
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        for dz in (-hz, hz):
            for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                pts.append(Gf.Vec3f(cx + dx, cy + dy, cz + dz))
        for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
                  (2, 3, 7, 6), (3, 0, 4, 7)):
            faces.extend(base + i for i in f)
            counts.append(4)
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh.GetPrim()


def _stage_capless_shaft(stage, path, cx, cy, cz, sx, sy, sz):
    """A tall, purely VERTICAL box — the 4 side-wall quads only, no top or
    bottom cap — so it contributes zero up-facing triangles anywhere,
    however tall it is or however much of its AABB sits under a query
    footprint. Stands in for a chimney/shaft/parapet-stub whose own top is
    nowhere near horizontal: a candidate that survives the (correct, XY +
    lo[2]) prune but must still resolve to NO support once the triangle
    test runs."""
    from pxr import Gf, Sdf, UsdGeom
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            pts.append(Gf.Vec3f(cx + dx, cy + dy, cz + dz))
    faces, counts = [], []
    for f in ((0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        faces.extend(f)
        counts.append(4)
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh.GetPrim()


def test_deck_support_z_finds_the_deck_under_a_merged_prim_with_a_far_coping_run():
    """THE ROUND-7 BUG, PINNED: a single merged mesh carries a real,
    up-facing deck at (0, 0) — top at z=69.276, exactly the real bake's own
    measurement (`gac_SM_Building_19_DG1_*.usd`'s `roof_x_0_14_0271`) — and,
    20 m away in X on the SAME prim, a raised coping run reaching z=70.196.
    The merged prim's own world AABB is therefore z=[68.7, 70.196]: taller
    than `z_ceiling + margin` by a wide margin. The OLD prune
    (`hi[2] > z_ceiling + margin`) discarded the WHOLE mesh because of the
    distant coping run and returned `None` (or fell through to a lower,
    wrong candidate); the fix must find the real deck under the query's own
    footprint regardless of what else, elsewhere, shares its prim."""
    stage, root = _empty_stage()
    _stage_merged_boxes(stage, root + "/roof_x_0_14_0271", [
        (0.0, 0.0, 69.138, 6.0, 6.0, 0.276),          # the real deck: top 69.276
        (20.0, 0.0, 70.048, 1.0, 1.0, 0.296),          # a coping run far away: top 70.196
    ])
    tank = root + "/tank_x_1"
    _stage_box(stage, tank, 0.0, 0.0, 69.30, 1.6, 1.6, 2.0)   # base at 69.30

    support = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=1.0,
                                 half_d=1.0, z_ceiling=69.30, exclude={tank})
    assert support is not None, \
        "the merged prim's far coping run pruned out its own real deck"
    assert abs(support - 69.276) < 0.01, support


def test_deck_support_z_none_when_a_tall_neighbour_has_no_up_face_under_the_footprint():
    """A capless vertical shaft directly under the query footprint spans
    z=[0, 45] — tall, and (after the fix) NOT pruned by `lo[2]` since its
    bottom is at grade — but it is side-walls only, so it contributes zero
    up-facing triangles anywhere. With nothing else in the scene, the
    search must resolve to `None`, not a spurious "support" borrowed from
    the shaft's own bounding box."""
    stage, root = _empty_stage()
    _stage_capless_shaft(stage, root + "/shaft_0", 0.0, 0.0, 22.5, 1.0, 1.0, 45.0)
    support = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=1.0,
                                 half_d=1.0, z_ceiling=40.0)
    assert support is None, support

    # the SAME shaft, plus a genuine deck lower down and off to the side —
    # the shaft must still contribute nothing, and the real deck must still
    # be found under ITS OWN footprint.
    _stage_box(stage, root + "/deck_real", 10.0, 0.0, 5.0, 4.0, 4.0, 0.3)
    support2 = qc._deck_support_z(stage, root, cx=10.0, cy=0.0, half_w=1.0,
                                  half_d=1.0, z_ceiling=40.0)
    assert support2 is not None and abs(support2 - 5.15) < 0.05, support2


def test_deck_support_z_performance_smoke_many_meshes_stays_bounded():
    """Not a benchmark — a guard against the fix accidentally turning an
    O(candidates) prune into an O(candidates) full triangle pass over every
    mesh in the scope. 400 small boxes scattered over a wide field (the
    scale of one building's own fractured piece count) plus the query prop;
    a bare linear sweep over that many tiny meshes, each with a handful of
    triangles, has no business taking more than a couple of seconds even on
    a loaded CI box."""
    import time
    stage, root = _empty_stage()
    rng = random.Random(11)
    for i in range(400):
        x = rng.uniform(-100.0, 100.0)
        y = rng.uniform(-100.0, 100.0)
        z = rng.uniform(0.0, 30.0)
        _stage_box(stage, "{0}/piece_{1}".format(root, i), x, y, z,
                  0.8, 0.8, 0.3)
    # the genuine target, dead centre, findable regardless of the noise
    _stage_box(stage, root + "/deck_target", 0.0, 0.0, 10.0, 4.0, 4.0, 0.3)
    tank = root + "/tank_x_1"
    _stage_box(stage, tank, 0.0, 0.0, 11.12, 1.6, 1.6, 2.0)

    t0 = time.perf_counter()
    support = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=1.0,
                                 half_d=1.0, z_ceiling=11.12, exclude={tank})
    dt = time.perf_counter() - t0

    assert support is not None and abs(support - 10.15) < 0.05, support
    assert dt < 5.0, "401-mesh sweep took {0:.2f}s — the prune regressed".format(dt)


# ---------------------------------------------------------------------------
# ROUND 7 — THE SECOND RING
#
# "Look at how we do partial collapse in the urban fire. There we figured out
#  how to do non-straight/rectangular break-offs from buildings. I like what
#  you're doing with partial collapse but the break-aways should be like
#  that." (user, 2026-08-31, on the staged city)
#
# MEASURED FIRST (`tools/qc_edge_probe.py`, on the archetype bake's own pose
# and seed): `fire_collapse.plan_edges` -> `_tear_perimeter` tears 100 % of
# what it is handed — 15/15, 55/55, 10/10 on the three styles probed, zero
# failures. The machinery was never broken; the RING WAS TOO SMALL, because
# the quake ladder's notch is 2-3 storeys of one wall (URM peels from the
# parapet down) where the fire's F5c hole is five storeys of nearly the whole
# elevation. So everything below is about what is left PRISTINE beside a
# correctly-torn hole, which is what the review actually photographed.
# ---------------------------------------------------------------------------
def _boundary_ring(ctx, plan, m):
    """Every surviving shell module that TOUCHES the hole — orthogonally or
    diagonally — as `{id(e): e}`. Deliberately re-derived here from the kill
    list and the element table rather than read off `plan["tears"]`: a test
    that asks the plan which modules it decided to tear and then checks it
    tore them is a transcript.
    """
    killed = set(id(e) for e in plan["kill"])
    killed |= set(id(j["el"]) for j in plan["stub_jobs"])
    killed |= set(id(e) for e in plan["panels"])
    dead = {}
    for e in plan["kill"]:
        if e["role"] not in fc.SHELL_ROLES:
            continue
        dead.setdefault((e["side"], int(e["storey"])), []).append(
            fc.el_span(m, e))
    out = {}
    for e in ctx["info"]["elements"]:
        if e["mass"] != plan["mass"] or e["role"] not in fc.SHELL_ROLES:
            continue
        if e.get("dead") or id(e) in killed:
            continue
        s = int(e["storey"])
        t0, t1 = fc.el_span(m, e)
        w = max(0.3, t1 - t0)
        tol = fc.edge_gap_tol(e, w, 0.6)
        for key in (s - 1, s, s + 1):
            for a, b in dead.get((e["side"], key), ()):
                touch = (min(b, t1) - max(a, t0) > min(1.2, 0.3 * w)
                         if key != s else False)
                side_by = (-0.25 * w <= (a - t1) <= tol
                           or -0.25 * w <= (t0 - b) <= tol)
                if touch or side_by:
                    out[id(e)] = e
    return out


def _check_boundary(ctx, plan, m):
    """Names of boundary modules that survive with no cut on them at all."""
    jobs = dict((id(j["el"]), j) for j in plan["tears"]
                if not j.get("dropped"))
    bad = []
    for k, e in _boundary_ring(ctx, plan, m).items():
        j = jobs.get(k)
        if j is None or not j["cuts"]:
            bad.append("{0} {1} st{2}".format(e.get("name"), e["side"],
                                              e["storey"]))
    return bad


@pytest.mark.parametrize("mode", PARTIAL + BAND)
def test_no_module_at_a_break_boundary_survives_untorn(mode):
    """THE ROUND-7 INVARIANT. Not "the jobs the plan wrote were cut" (which is
    what `test_every_module_touching_the_hole_is_torn_and_knows_which_edge`
    already asserts) but "no module TOUCHING the hole was left without a job"
    — including the DIAGONAL ones, the re-entrant corner of every staircase
    step, which `plan_edges` cannot see and which is the sharpest right angle
    in the whole notch.
    """
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        bad = _check_boundary(ctx, plan, _mass(ctx, plan))
        assert not bad, (style, mode, bad[:8])


def test_mutation_an_untorn_diagonal_neighbour_is_caught():
    """`_check_boundary` has to FAIL when a boundary module loses its job —
    otherwise the test above passes because it cannot see anything."""
    ctx, plan = _plan("commercial_mid", "elevation")
    m = _mass(ctx, plan)
    assert not _check_boundary(ctx, plan, m)
    ring = _boundary_ring(ctx, plan, m)
    assert ring, "no boundary modules at all — the fixture stopped biting"
    plan["tears"] = [j for j in plan["tears"] if id(j["el"]) not in ring]
    assert _check_boundary(ctx, plan, m), "a stripped ring reported clean"


@pytest.mark.parametrize("mode", PARTIAL + BAND)
def test_the_ring_never_cuts_the_top_off_a_wall_carrying_a_parapet(mode):
    """A parapet / cornice band has NOTHING else holding it up — it is a band
    sitting on the top course of the wall — so a `z` cut taken from that
    wall's top leaves the band in the air. Same class of bug as soot baked
    onto a wall a later recipe takes away.

    A full storey module above is deliberately NOT covered: it bears on the
    floor slab, and a bite at the slab line is the same thing
    `fire_collapse.plan_edges`'s own `below` class authors under a hole.

    This is also what forced the roofline pass to group a bay by OVERLAP in t
    rather than by a rounded midpoint: a corner's wall piece and the
    parapet_corner on it are the same bay at different lengths (1.25 m against
    1.81 m on `apartment_long`), and a midpoint bucket split them and cut the
    wall with the parapet still standing on it.
    """
    for style in STYLES:
        ctx, plan = _plan(style, mode)
        m = _mass(ctx, plan)
        killed = set(id(e) for e in plan["kill"])
        killed |= set(id(j["el"]) for j in plan["stub_jobs"])
        bands = [e for e in ctx["info"]["elements"]
                 if e["mass"] == plan["mass"]
                 and e["role"] in ("parapet", "parapet_corner")
                 and not e.get("dead") and id(e) not in killed]
        for j in plan["tears"]:
            if j.get("dropped"):
                continue
            if not [c for c in j["cuts"]
                    if c["kind"] == "z" and c.get("loose_above")]:
                continue
            for e in bands:
                if e is j["el"] or e["side"] != j["side"]:
                    continue
                t0, t1 = fc.el_span(m, e)
                za, _zb = fc.el_z_span(m, e)
                if abs(za - j["zb"]) > 0.15:
                    continue
                ov = min(t1, j["t1"]) - max(t0, j["t0"])
                if ov <= 0.5 * min(max(0.3, t1 - t0), j["w"]):
                    continue
                assert False, (style, mode, j["name"], "top cut with",
                               e.get("name"), "standing on it")


def test_the_second_ring_is_deterministic_and_takes_zero_shared_draws():
    """Same building, same seed, same ring — and a ladder that has already
    drawn from the shared rng gets the identical one, which is what lets this
    family sit anywhere in a recipe list."""
    for mode in ("elevation", "corner", "mid_storey"):
        a = _plan("apartment", mode)[1]
        ctx = _ctx("apartment")
        for _ in range(37):
            ctx["rng"].random()
            ctx["nrng"].random()
        b = qc.plan_collapse(ctx, mode=mode)
        key = lambda p: [(j["name"], j["side"], j["storey"],
                          tuple(j["classes"]),
                          tuple(round(float(c.get("line", c.get("frac", 0.0))), 9)
                                for c in j["cuts"]),
                          bool(j.get("ring")))
                         for j in p["tears"]]
        assert key(a) == key(b), mode
        assert a["budget"]["ring"] == b["budget"]["ring"], mode


def test_the_second_ring_answers_to_its_own_cap_and_never_to_the_first():
    """`QC_RING_MAX=0` gives back the round-6 look exactly: the first ring is
    untouched, byte for byte, and nothing else in the plan moves."""
    import os as _os
    style, mode = "commercial_mid", "elevation"
    on = _plan(style, mode)[1]
    _os.environ["QC_RING_MAX"] = "0"
    try:
        off = _plan(style, mode)[1]
    finally:
        _os.environ.pop("QC_RING_MAX", None)
    assert off["budget"]["ring"] == 0 and on["budget"]["ring"] > 0
    assert not [j for j in off["tears"] if j.get("ring")]
    first_on = [(j["name"], j["side"], j["storey"], tuple(j["classes"]))
                for j in on["tears"] if not j.get("ring")]
    # the first ring's own classes are `plan_edges`'s; the second ring may add
    # a class to a job it merges into, so compare the modules and the cuts
    # `plan_edges` itself drew
    first_off = [(j["name"], j["side"], j["storey"], tuple(j["classes"]))
                 for j in off["tears"]]
    assert [q[:3] for q in first_on] == [q[:3] for q in first_off]
    assert len(off["kill"]) == len(on["kill"])
    assert off["s0"] == on["s0"] and off["sides"] == on["sides"]


@pytest.mark.parametrize("mode", PARTIAL + BAND)
def test_the_boundary_counter_line_reports_every_treated_module(mode):
    """`[qc] boundary tears: N/M modules treated (...)` is the line a bake log
    is read for, so it has to be true before anything is authored: M is every
    live job, and the two ring counts add up to it."""
    for style in ("commercial_mid", "apartment", "dw_terrace"):
        _c, plan = _plan(style, mode)
        line = qc.boundary_line(plan)
        assert line.startswith("[qc] boundary tears: ")
        live = [j for j in plan["tears"] if not j.get("dropped")]
        assert "0/{0} modules treated".format(len(live)) in line, line
        assert "{0} first ring".format(
            len(live) - plan["budget"]["ring_new"]) in line, line
        assert "{0} second ring".format(plan["budget"]["ring"]) in line, line
        # nothing is authored yet, so nothing may claim to be torn
        assert "0/" in line


@pytest.mark.parametrize("mode", PARTIAL + BAND)
def test_no_tear_job_is_ever_written_without_a_cut(mode):
    """`_tear_perimeter` skips a job with no judges (`if not judges:
    continue`) but `boundary_line` counts it, so an empty job is a boundary
    counter that can never reach N/N. The ring therefore creates its job LAST,
    after the cut is known to be going in."""
    for style in STYLES:
        _c, plan = _plan(style, mode)
        for j in plan["tears"]:
            assert j["cuts"], (style, mode, j["name"], j["classes"])
        # ...and no module ever carries two jobs (`_break_split` deactivates
        # the source prim, so the second one would fracture a dead prim)
        ids = [id(j["el"]) for j in plan["tears"]]
        assert len(ids) == len(set(ids)), (style, mode)


def test_parapet_fall_spends_its_side_budget_on_sides_that_still_have_one():
    """The 2026-08-31 review's "the top floor has a bunch of perfect
    rectangular wall pieces all intact".

    `_pick_sides` returns S FIRST, and in the urm ladders a collapse recipe
    runs before `parapet_fall` and normally takes S — so the old loop spent
    one of its `sides` slots on a wall whose parapets were already `dead`,
    silently, and a whole elevation kept its pristine parapet ring. Measured
    on the baked `bld_apartment_long_DG4`: eight untouched 5.12 x 2.17 m
    parapet modules on N.
    """
    ctx = _ctx("apartment_long")
    m = ctx["info"]["masses"]["main"]
    top = len(m["levels"]) - 1
    # the collapse already took S, exactly as `qc_elevation` does
    for e in ctx["info"]["elements"]:
        if e["side"] == "S" and e["mass"] == "main" and (
                e["role"] in ("parapet", "parapet_corner")
                or int(e["storey"]) == top):
            e["dead"] = True
    want, jobs, empty = qf._parapet_sides(ctx, "main", 3, 0.9)
    assert want == 3
    assert "S" in empty, empty
    assert len(jobs) == 3, [q[0] for q in jobs]
    assert "S" not in [q[0] for q in jobs]
    assert set(q[0] for q in jobs) == {"E", "W", "N"}
    # ...and `frac` is per side: one parapet-less side must not boost the next
    ctx2 = _ctx("commercial_mid")
    _w2, jobs2, _e2 = qf._parapet_sides(ctx2, "main", 3, 0.5)
    for _sd, _run, partial, frac_s in jobs2:
        assert abs(frac_s - (0.8 if partial else 0.5)) < 1e-9, (partial, frac_s)


def test_a_parapet_ring_left_standing_is_reported_not_silent():
    """A role lookup that comes back empty used to be an invisible
    `continue`. If the budget cannot be spent the note has to say so."""
    ctx = _ctx("apartment")
    for e in ctx["info"]["elements"]:
        e["dead"] = True
    want, jobs, empty = qf._parapet_sides(ctx, "main", 3, 0.9)
    assert want == 3 and not jobs and len(empty) == 4


# ---------------------------------------------------------------------------
# Z-OUTLIER SWEEP (`quake_collapse.z_outlier_sweep`) — a real, measured defect:
# `bld_brownstone_row_DG4.usd`'s `/Baked/LOD0_108` (2762 pts / 2754 faces)
# settled at z~=17, on the roof deck, while its five topology-identical
# siblings (`LOD0_105`/`106`/`107`/`109`/`110` — same signature, one
# wall-break event's own cells) landed at z~=11, at the actual break line.
# The gap from `LOD0_108` to the roof deck it stopped on is a real -0.07 m,
# so `fire_bake.deactivate_airborne`'s points-based "is this seated" test
# passes it outright — this sweep is the "does this body agree with its own
# family" question nothing upstream ever asks.
#
# `_stage_box` (this file's own helper, above) gives every fixture chip the
# SAME (8 pt / 6 face) topology, which is exactly what puts them in one
# z-outlier GROUP. A support surface is authored as a single flat quad
# (`_zo_plane`, 4 pts / 1 face) instead of another `_stage_box` for exactly
# the opposite reason: a real deck's own topology never happens to collide
# with a real fracture chip's (2762 pts / 2754 faces on the measured
# defect), and a fixture where every prim is a generic box would collide
# them by accident, testing an artefact of the fixture instead of the sweep.
# ---------------------------------------------------------------------------
def _zo_plane(stage, path, cx, cy, z, size=6.0):
    """One flat, upward-facing quad (4 pts / 1 face) — a support surface
    shaped for `_deck_support_z`, topologically distinct from the
    `_stage_box`-shaped chips above it (see the section note)."""
    from pxr import Gf, Sdf, UsdGeom
    h = size / 2.0
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr([Gf.Vec3f(cx - h, cy - h, z), Gf.Vec3f(cx + h, cy - h, z),
                          Gf.Vec3f(cx + h, cy + h, z), Gf.Vec3f(cx - h, cy + h, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return path


def _zo_chip(stage, root, name, cx, cy, bottom_z, size=1.0):
    """One (8 pt / 6 face) fracture-cell-shaped box, `_stage_box`'s own
    construction, at an explicit BOTTOM z (not centre — the world AABB
    bottom is the measure `z_outlier_sweep` itself groups and thresholds
    on)."""
    p = "{0}/{1}".format(root, name)
    _stage_box(stage, p, cx, cy, bottom_z + size / 2.0, size, size, size)
    return p


# Five siblings, one wall-break event's own cells: bottoms within 0.3 m of
# each other, median (sorted index 2) exactly 11.05 — deliberately not all
# identical, the same small real-world spread the measured defect's own
# siblings show (`LOD0_105`..`110` span z=[10.999, 11.486]).
_ZO_SIBLING_BOTTOMS = (10.90, 11.00, 11.05, 11.10, 11.20)


def _zo_family_fixture(stage, root, outlier_bottom=17.05, deck_low=True,
                       deck_high=True):
    """The full scenario: five real siblings at the break line, ONE climber
    at `outlier_bottom` (default 6.0 m above the siblings' median — past
    the default 1.5 x 3.4 m = 5.1 m detection threshold, the same order of
    magnitude as the measured defect's own 5.68 m), a real deck at the
    siblings' own band (`deck_low`, the correct landing spot) and/or a
    second deck exactly where the climber currently rests (`deck_high` —
    the wrong "roof" it stopped on, gap = 0, matching the measured -0.07 m
    the points-based audit already passes)."""
    siblings = [_zo_chip(stage, root, "sib_{0}".format(i), i * 2.0, 0.0, bz)
               for i, bz in enumerate(_ZO_SIBLING_BOTTOMS)]
    outlier = _zo_chip(stage, root, "climber", 100.0, 100.0, outlier_bottom)
    if deck_low:
        _zo_plane(stage, root + "/deck_low", 100.0, 100.0, 11.05)
    if deck_high:
        _zo_plane(stage, root + "/deck_high", 100.0, 100.0, outlier_bottom)
    return siblings, outlier


def _bbox_z_range(stage, path):
    from pxr import Usd, UsdGeom
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    pr = stage.GetPrimAtPath(path)
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    return float(r.GetMin()[2]), float(r.GetMax()[2])


def test_z_outlier_enabled_env_reader(monkeypatch):
    monkeypatch.delenv(qc.Z_OUTLIER_ENV, raising=False)
    assert qc.z_outlier_enabled() is True

    monkeypatch.setenv(qc.Z_OUTLIER_ENV, "")
    assert qc.z_outlier_enabled() is True

    monkeypatch.setenv(qc.Z_OUTLIER_ENV, "0")
    assert qc.z_outlier_enabled() is False

    monkeypatch.setenv(qc.Z_OUTLIER_ENV, "false")
    assert qc.z_outlier_enabled() is False

    monkeypatch.setenv(qc.Z_OUTLIER_ENV, "1")
    assert qc.z_outlier_enabled() is True


def test_z_outlier_detected_and_reseated_onto_real_support():
    """THE MEASURED DEFECT, reproduced: a climber past the threshold, with
    both the wrong (high) deck it actually stopped on and the right (low)
    deck its siblings share both present. The reseat must find the LOW
    deck — the search ceiling (`median + 1 storey`) is deliberately below
    the high deck, so it can never just be handed back the same roof."""
    stage, root = _empty_stage()
    siblings, outlier = _zo_family_fixture(stage, root)

    before_lo, before_hi = _bbox_z_range(stage, outlier)
    assert abs(before_lo - 17.05) < 1e-6, "fixture must start on the high deck"

    result = qc.z_outlier_sweep(stage, root, rng=random.Random(5), verbose=False)

    assert result["reseated"] == 1, result
    assert result["deactivated"] == 0, result
    assert result["outliers"] == [outlier]

    pr = stage.GetPrimAtPath(outlier)
    assert pr.IsValid() and pr.IsActive(), "a re-seated body must stay active"
    after_lo, _after_hi = _bbox_z_range(stage, outlier)
    assert after_lo < 13.0, ("did not leave the high deck", after_lo)
    assert abs(after_lo - 11.05) < 0.3, \
        ("did not land on the real (low) deck", after_lo)


def test_z_outlier_no_support_is_deactivated():
    """The same climber, but its own footprint has nothing real under it
    within `median + 1 storey` (only the high deck it wrongly stopped on
    exists) — `_deck_support_z` must return `None`, and the sweep's answer
    to a genuine hole all the way down is `fire_bake.deactivate_airborne`'s
    own: turn it off."""
    stage, root = _empty_stage()
    _siblings, outlier = _zo_family_fixture(stage, root, deck_low=False)

    result = qc.z_outlier_sweep(stage, root, rng=random.Random(5), verbose=False)

    assert result["reseated"] == 0, result
    assert result["deactivated"] == 1, result
    assert result["outliers"] == [outlier]
    pr = stage.GetPrimAtPath(outlier)
    assert pr.IsValid() and not pr.IsActive(), \
        "no real support anywhere under it -> deactivated, not left floating"


def test_z_outlier_small_group_untouched():
    """Two topology-identical chips, one 10 m above the other: with no
    third member there is no majority to be an outlier FROM
    (`min_group`, default 3) — the sweep must not even look at their Z,
    and the stage must come back byte-for-byte."""
    stage, root = _empty_stage()
    _zo_chip(stage, root, "a", 0.0, 0.0, 11.0)
    _zo_chip(stage, root, "b", 5.0, 0.0, 21.0)
    before = stage.GetRootLayer().ExportToString()

    result = qc.z_outlier_sweep(stage, root, rng=random.Random(1), verbose=False)

    # `groups` counts every distinct topology signature FOUND (1 — both
    # chips share one), regardless of whether it cleared `min_group`; only
    # `outliers` (and the stage itself) prove nothing was acted on.
    assert result["reseated"] == 0 and result["deactivated"] == 0
    assert result["outliers"] == []
    assert stage.GetRootLayer().ExportToString() == before, \
        "a group under min_group must not be touched at all"


def test_z_outlier_non_outliers_untouched():
    """The five real siblings must come out of a sweep exactly where they
    went in — only the climber may move."""
    stage, root = _empty_stage()
    siblings, outlier = _zo_family_fixture(stage, root)
    before = {p: _bbox_z_range(stage, p) for p in siblings}

    qc.z_outlier_sweep(stage, root, rng=random.Random(5), verbose=False)

    for p in siblings:
        assert _bbox_z_range(stage, p) == before[p], (p, "a non-outlier moved")
        pr = stage.GetPrimAtPath(p)
        assert pr.IsValid() and pr.IsActive(), (p, "a non-outlier was deactivated")


# ---------------------------------------------------------------------------
# `paths=` — THE MEASURED FALSE POSITIVE (real archetype file, unrestricted
# scan): `prop_main_4_7` (a floor-4 fixture) shares its (180, 312) topology
# with `prop_main_1_3`/`1_5`/`1_6` (floor 1) and `prop_main_2_0`/`2_3`
# (floor 2) — one legitimate prop, repeated once per storey BY DESIGN. An
# unrestricted scan of the WHOLE building sees one group of 6, the floor-4
# copy 5.99 m above the floor-1/2 median, past the 5.1 m line — flagged and
# moved, though nothing ever settled wrong. `paths=` is the fix: the
# production caller (`bake_quake_archetypes_launch_script.py`) passes the
# building's own `res["loose"]`, which a static per-floor prop is never
# part of, so it never enters the candidate pool at all.
# ---------------------------------------------------------------------------
def test_z_outlier_unrestricted_scan_can_false_positive_on_a_per_storey_prop():
    """Reproduces the measured false positive on a minimal fixture: three
    floors' worth of one legitimate, unrelated prop, sharing one topology.
    Without `paths=`, the sweep cannot tell "one wall-break event's cells"
    from "one prop asset repeated once per storey" — this is not a bug in
    the assertion, it is the documented reason `paths=` exists."""
    stage, root = _empty_stage()
    floor1 = _zo_chip(stage, root, "prop_floor1", 0.0, 0.0, 5.0)
    floor2 = _zo_chip(stage, root, "prop_floor2", 3.0, 0.0, 8.0)
    floor4 = _zo_chip(stage, root, "prop_floor4", 6.0, 0.0, 14.0)

    result = qc.z_outlier_sweep(stage, root, rng=random.Random(2), verbose=False)

    assert floor4 in result["outliers"], \
        "the false positive this fixture demonstrates did not reproduce"
    assert _bbox_z_range(stage, floor1) == (5.0, 6.0)
    assert _bbox_z_range(stage, floor2) == (8.0, 9.0)
    pr4 = stage.GetPrimAtPath(floor4)
    moved = (not pr4.IsActive()) or _bbox_z_range(stage, floor4) != (14.0, 15.0)
    assert moved, "flagged as an outlier but never actually acted on"


def test_z_outlier_paths_allowlist_prevents_the_false_positive():
    """The SAME three props, but the caller passes `paths=` naming only the
    two that were ever actually loose (`floor4` stands in for a static prop
    that never went through settle at all) — `floor4` must never even be
    grouped, let alone flagged or moved."""
    stage, root = _empty_stage()
    floor1 = _zo_chip(stage, root, "prop_floor1", 0.0, 0.0, 5.0)
    floor2 = _zo_chip(stage, root, "prop_floor2", 3.0, 0.0, 8.0)
    floor4 = _zo_chip(stage, root, "prop_floor4", 6.0, 0.0, 14.0)
    before_floor4 = _bbox_z_range(stage, floor4)

    result = qc.z_outlier_sweep(stage, root, paths=[floor1, floor2],
                                rng=random.Random(2), verbose=False)

    assert result["reseated"] == 0 and result["deactivated"] == 0
    assert result["outliers"] == []
    assert _bbox_z_range(stage, floor4) == before_floor4
    pr = stage.GetPrimAtPath(floor4)
    assert pr.IsValid() and pr.IsActive()


def test_z_outlier_deterministic_same_seed_same_placement():
    def _run():
        stage, root = _empty_stage()
        _siblings, outlier = _zo_family_fixture(stage, root)
        result = qc.z_outlier_sweep(stage, root, rng=random.Random(77),
                                    verbose=False)
        return result, _bbox_z_range(stage, outlier)

    result_a, z_a = _run()
    result_b, z_b = _run()
    assert result_a == result_b
    assert z_a == z_b, "same seed against the same stage content must land " \
                       "the climber identically"


def test_z_outlier_env_off_is_a_noop(monkeypatch):
    """`EQ_Z_OUTLIER=0` disables the sweep entirely — no traversal, no
    stage mutation, byte-for-byte, exactly `apply_settle_budget`'s own
    `budget=None` contract."""
    monkeypatch.setenv(qc.Z_OUTLIER_ENV, "0")
    stage, root = _empty_stage()
    _zo_family_fixture(stage, root)
    before = stage.GetRootLayer().ExportToString()

    result = qc.z_outlier_sweep(stage, root, rng=random.Random(5), verbose=False)

    after = stage.GetRootLayer().ExportToString()
    assert result == {"reseated": 0, "deactivated": 0, "groups": 0, "outliers": []}
    assert after == before, "EQ_Z_OUTLIER=0 must not touch the stage at all"


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
