#!/usr/bin/env python3
"""test_fire_collapse.py — does the PARTIAL collapse take away the right part?

    python3 scene_gen/tests/test_fire_collapse.py
    pytest -q scene_gen/tests/test_fire_collapse.py

WHY THIS EXISTS
---------------
`disaster/fire_collapse.py` takes PART of a burnt-out shell away — one burning
elevation from the fire floor up, or the corner two burning elevations share —
and leaves the rest of the building standing. Every claim in that sentence is
arithmetic on the element table, and none of it needs USD:

  * the modules that come away are all on an elevation the FIRE was venting
    through, and all at or above the fire's ORIGIN storey (the clean band
    under a black stripe is `urban_fire`'s strongest signature and a collapse
    may not eat into it);
  * the elevations that are not lost keep EVERY module — this is what makes it
    a partial collapse rather than `r_fire_collapse` with a bigger number;
  * the heap a wall that fell outward makes is OUTSIDE the wall line, in the
    street, and the smaller one is INSIDE — get the sign wrong and the rubble
    is inside a building whose wall went into the road;
  * the break line is a STAIRCASE, widening upward, so what is left is a notch
    on the module grid and not a rectangle;
  * the plan is stable per building (its own seed), and does not touch the
    ladder's shared rng.

So the decisions are factored out of the USD authoring into
`fire_collapse.plan_partial_collapse`, and this file checks THAT, on REAL kit
buildings — `urban_building.build_building` + `quake_flow.describe`, pure
placement math, no USD, no Kit, no Isaac Sim — through the real
`plan_fire` -> `plan_partial_collapse` path a scene takes. Same shape as
`test_soot_plume.py`, which is the file it was modelled on.

It runs host-side in a couple of seconds.

WHAT IT CANNOT SEE: everything downstream of the plan — whether the fractured
wall actually lands in the windrow, whether the drooped floors read, whether
the rebound fragments are dark enough. That needs a render.
"""

import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import urban_building as ub          # noqa: E402
from disaster import fire_collapse as fc         # noqa: E402
from disaster import quake_flow as qf            # noqa: E402
from disaster import urban_fire as uf            # noqa: E402

# Every MCE kit twin plus the two masonry blocks the bench leans on. The
# families are 01/04 (urm), 02 (rc) and 05 (rc_glass), so this spans all three
# construction types the ladder keys on.
STYLES = ("commercial_mid", "commercial", "apartment", "block_residential",
          "highrise_step", "dw_terrace", "brownstone_row")


def _ctx(style, seed=7, level="F5c", sides=("S", "E"), origin=1, tag="b0"):
    """A real kit building, a real fire plan, no stage.

    `plan_fire` is `urban_fire`'s own; `describe` is `quake_flow`'s. Nothing
    here is a fixture — the element table, the mass specs, the storey of every
    module and its side are all computed the way a scene computes them.
    """
    rng = random.Random(seed)
    placements = ub.build_building(style, 0.0, 0.0, 0.0, rng)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    ctx = {"info": info, "rng": rng, "nrng": np.random.default_rng(seed),
           "notes": [], "tag": tag}
    ctx["fire"] = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
    return ctx


def _mass(ctx, plan):
    return ctx["info"]["masses"][plan["mass"]]


def _shell(ctx, plan, side=None):
    """Every shell element of the burning mass (optionally on one side)."""
    return [e for e in ctx["info"]["elements"]
            if e["mass"] == plan["mass"] and e["role"] in fc.SHELL_ROLES
            and (side is None or e["side"] == side)]


# ---------------------------------------------------------------------------
def test_the_module_self_check_passes():
    """`fire_collapse.check()` is the same invariants over both modes; a
    launch script gates on it the way it gates on `urban_fire.check`."""
    bad = fc.check(verbose=False)
    assert not bad, "\n".join(bad)


def test_the_ladder_can_select_it():
    """F5c has to be a real level everywhere `urban_fire` looks, or the bench
    raises fourteen seconds into a launch."""
    assert not uf.check(verbose=False)
    assert fc.FIRE_LEVEL in uf.LEVELS
    assert "partial_collapse" in uf.RECIPES
    for btype in ("urm", "rc", "rc_glass"):
        assert fc.FIRE_LEVEL in uf.LADDER[btype]
    assert uf.ACTIVE[fc.FIRE_LEVEL] == "residual"
    assert uf.FINISH[fc.FIRE_LEVEL] == "ash"
    # urm and rc actually run the recipe; a curtain-wall tower deliberately
    # does not (no tower has lost part of its shell in a fire on the record)
    for btype in ("urm", "rc"):
        names = [n for n, _kw in uf.LADDER[btype][fc.FIRE_LEVEL]]
        assert "partial_collapse" in names, btype
        # COLLAPSE FIRST: anything that takes a wall away must run before the
        # passes that author art on walls, or the art is left in the sky.
        i = names.index("partial_collapse")
        for art in ("window_burnout", "smoke_stain", "char_facade",
                    "expose_interior", "street_debris", "flames"):
            if art in names:
                assert i < names.index(art), (btype, art)
    assert "partial_collapse" not in [
        n for n, _kw in uf.LADDER["rc_glass"][fc.FIRE_LEVEL]]
    # THE FREEZE, stated as a test: F5c is the ONLY level that runs the
    # recipe, so nothing `fire_collapse` does can reach F0-F6. Everything
    # round the hole — the edge tears and the burn zone — is downstream of
    # `r_partial_collapse`, and `soot_plume.skin`'s `burn_zone` is `None`
    # for every other level by construction.
    for btype, levels in uf.LADDER.items():
        for lv, recipes in levels.items():
            if lv == fc.FIRE_LEVEL:
                continue
            assert "partial_collapse" not in [n for n, _kw in recipes], \
                (btype, lv)


def test_soot_plume_knows_the_level():
    """A level `soot_plume.DURATION_S` has never heard of returns NO fire
    events at all — no soot skin, no flames — and says nothing about it."""
    from disaster import soot_plume as sp
    assert fc.FIRE_LEVEL in sp.DURATION_S
    ctx = _ctx("commercial_mid")
    ev = sp.plan_events(ctx, uf._severity)
    assert ev, "F5c planned no fire events at all"
    assert all(e["side"] in ctx["fire"]["sides"] for e in ev)


# ---------------------------------------------------------------------------
def test_everything_lost_is_on_a_burning_elevation_at_or_above_the_origin():
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            f = ctx["fire"]
            assert plan["kill"], (style, mode, "nothing taken away")
            assert plan["s0"] >= f["origin"], (style, mode, plan["s0"],
                                               f["origin"])
            for e in plan["kill"]:
                assert e["side"] in plan["sides"], (style, mode, e["name"],
                                                    e["side"], plan["sides"])
                assert e["side"] in f["sides"] or plan["mode"] == "corner", \
                    (style, mode, e["side"], f["sides"])
                assert int(e["storey"]) >= plan["s0"], (style, mode,
                                                        e["name"],
                                                        e["storey"])
                assert e["mass"] == plan["mass"], (style, mode, e["name"])


def test_the_untouched_elevations_keep_every_module():
    """The whole point of PARTIAL. `r_fire_collapse` sweeps every side of the
    top storeys; this must not touch a cold wall at all."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            assert plan["keep_sides"], (style, mode)
            killed = set(id(e) for e in plan["kill"])
            for sd in plan["keep_sides"]:
                on_side = _shell(ctx, plan, sd)
                assert on_side, (style, mode, sd, "no modules on this side?")
                lost = [e for e in on_side if id(e) in killed]
                assert not lost, (style, mode, sd,
                                  "{0} module(s) taken off an elevation that "
                                  "is not lost".format(len(lost)))
            # ...and the storeys BELOW the failure line keep every module on
            # the lost elevation too — the clean band under the fire.
            for sd in plan["sides"]:
                below = [e for e in _shell(ctx, plan, sd)
                         if int(e["storey"]) < plan["s0"]]
                assert not [e for e in below if id(e) in killed], (style, mode)
                if plan["s0"] > 0:
                    assert below, (style, mode, "no clean band under the "
                                                "failure line")


def test_a_real_share_of_the_lost_elevation_actually_goes():
    """A "collapse" that takes two modules is a hole. On the elevation mode
    most of the burning wall above the failure line has to come away."""
    for style in ("commercial_mid", "commercial", "apartment", "dw_terrace",
                  "brownstone_row"):
        ctx = _ctx(style)
        plan = fc.plan_partial_collapse(ctx, mode="elevation")
        sd = plan["sides"][0]
        in_band = [e for e in _shell(ctx, plan, sd)
                   if int(e["storey"]) >= plan["s0"]]
        killed = set(id(e) for e in plan["kill"])
        got = len([e for e in in_band if id(e) in killed])
        assert got >= 0.5 * len(in_band), (style, got, len(in_band))
        assert got < len(in_band) or plan["span_frac"] >= 0.99, \
            (style, "the whole elevation went — no stub left standing")


def test_the_break_line_is_a_staircase_that_widens_upward():
    """`monolith_damage`'s rule: a stepped profile on the bay grid, never a
    rectangle (`PROFILE_FOOT` = 1.0) and never one long diagonal."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            if len(plan["storeys"]) < 2:
                continue
            for sd in plan["sides"]:
                widths = [plan["span"][(sd, s)][1] - plan["span"][(sd, s)][0]
                          for s in plan["storeys"]]
                for a, b in zip(widths, widths[1:]):
                    assert b >= a - 1e-6, (style, mode, sd, widths)
                assert widths[-1] > widths[0] + 1e-6, (style, mode, sd,
                                                       "no step at all")
                # ...and never wider than the wall
                L = fc.side_length(_mass(ctx, plan), sd)
                assert widths[-1] <= L + 1e-6, (style, mode, sd, widths[-1], L)


def test_the_heaps_are_on_the_right_side_of_the_wall_line():
    """A wall that failed out of plane lands in the STREET. The windrow's
    centre must therefore be strictly outside the wall line and the smaller
    heap strictly inside it — `quake_flow._heap(fill=False)` places its
    chunks outward from the wall, so a plan that put the windrow inside would
    bury it in the building."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            m = _mass(ctx, plan)
            outs = [h for h in plan["heaps"] if h["where"] == "outside"]
            ins = [h for h in plan["heaps"] if h["where"] == "inside"]
            assert len(outs) == len(plan["sides"]), (style, mode)
            assert len(ins) == 1, (style, mode)
            for h in outs:
                lx, ly = h["centre_local"]
                d = fc.outward_of(m, h["side"], lx, ly)
                assert d > 0.0, (style, mode, h["side"], d)
                assert abs(d - h["outward_m"]) < 1e-9
                # in the street, not in orbit: within a couple of building
                # heights of the wall
                assert d < max(12.0, 0.5 * (m["top"] - m["z0"])), (style, d)
                # ...and along the wall it is under the part that fell
                t = fc.along_of(m, h["side"], lx, ly)
                lo, hi = h["along_m"]
                assert lo - 1e-6 <= t <= hi + 1e-6, (style, mode, t, lo, hi)
            for h in ins:
                lx, ly = h["centre_local"]
                d = fc.outward_of(m, h["side"], lx, ly)
                assert d < 0.0, (style, mode, h["side"], d)
                # inside the footprint, not through the far wall
                assert -d < fc.side_depth(m, h["side"]) * 0.5, (style, d)


def test_the_region_is_only_the_lost_strip():
    """The position sweep hands a static prim to the solver when it is above
    the failure line AND inside the plan footprint of the loss. If that
    footprint covered the mass, the sweep would be `r_fire_collapse`'s and
    would drop the whole roof deck and its plant."""
    for style in ("commercial_mid", "block_residential", "highrise_step"):
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            m = _mass(ctx, plan)
            # the centre of the mass is NOT in the lost region
            assert not fc.in_region(plan, m, 0.0, 0.0), (style, mode)
            # a point just inside the middle of the lost wall IS
            sd = plan["sides"][0]
            lo, hi = plan["region"][sd][:2]
            lx, ly = fc.wall_point(m, sd, 0.5 * (lo + hi), out_m=-0.5)
            assert fc.in_region(plan, m, lx, ly), (style, mode)
            # ...and the same point on the OPPOSITE elevation is not
            from disaster.quake_flow import _opposite
            olx, oly = fc.wall_point(m, _opposite(sd), 0.5 * (lo + hi),
                                     out_m=-0.5)
            assert not fc.in_region(plan, m, olx, oly), (style, mode)


def test_corner_mode_takes_a_corner_and_not_half_the_building():
    """`CORNER_REACH_MODULES` is in MODULES on purpose: a share of the side
    is 41 m of an 88 m elevation, which is not a corner (measured on
    `block_residential` before it was bounded)."""
    for style in STYLES:
        ctx = _ctx(style)
        plan = fc.plan_partial_collapse(ctx, mode="corner")
        m = _mass(ctx, plan)
        assert plan["corner"] and len(plan["sides"]) == 2, style
        assert set(plan["sides"]) == set(fc.corner_sides(plan["corner"]))
        for sd in plan["sides"]:
            L = fc.side_length(m, sd)
            lo, hi = plan["span"][(sd, plan["top_storey"])]
            assert (hi - lo) <= fc.CORNER_REACH_MAX_FRAC * L + 1e-6, (style, sd)
            # anchored AT the corner, i.e. at one end of the wall
            at_hi = fc.corner_at_high_end(sd, plan["corner"])
            assert (abs(hi - L) < 1e-6) if at_hi else (abs(lo) < 1e-6), \
                (style, sd, plan["corner"], lo, hi, L)


def test_the_hole_has_an_owner_for_both_of_its_vertical_edges():
    """Every vertical edge of the loss is either a building CORNER (and then
    `quake_flow._ragged_neighbours` tears the return wall) or a mid-wall edge
    (and then `_tear_edge` tears the surviving bay on a wandering vertical
    crack). An edge with neither is a dead-straight kit module seam up the
    wall — the vertical half of the "unnatural rectangular parts broken off"
    complaint."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            m = _mass(ctx, plan)
            for sd in plan["sides"]:
                at_lo, at_hi = plan["reaches_end"][sd]
                L = fc.side_length(m, sd)
                lo, hi = plan["span"][(sd, plan["top_storey"])]
                assert at_lo == (lo <= plan["pad_m"])
                assert at_hi == (hi >= L - plan["pad_m"])
                if mode == "corner":
                    # anchored at the corner: exactly one end is a corner,
                    # unless the reach swallowed the whole wall
                    assert at_lo or at_hi, (style, sd)
                # A MID-WALL EDGE NEEDS A SURVIVING BAY TO CRACK, and the
                # crack has to land INSIDE that bay — not on the span's own
                # edge, which is within centimetres of a module seam and
                # would shave a sliver off instead of moving the visible
                # edge off the kit grid.
                killed = set(id(e) for e in plan["kill"])
                # `edge_neighbour` walks `_els`, which skips elements a
                # recipe has marked `dead` — so it only answers correctly
                # once the kill loop has run. Do exactly what
                # `r_partial_collapse` does before it calls `_tear_edge`.
                for e in plan["kill"]:
                    e["dead"] = True
                for low_edge, at_end in ((True, at_lo), (False, at_hi)):
                    if at_end:
                        continue
                    for st in plan["storeys"]:
                        e, bay = fc.edge_neighbour(ctx, plan, m, sd, st,
                                                   low_edge)
                        if e is None:
                            # legitimate: the loss took every bay on this
                            # side at this storey (a 15 m return wall with
                            # ONE 5 m module — `highrise_step`'s tower), so
                            # there is no mid-wall edge to tear and the end
                            # of the wall is the edge.
                            live = [q for q in _shell(ctx, plan, sd)
                                    if int(q["storey"]) == st
                                    and q["role"] in ("wall", "corner")
                                    and id(q) not in killed]
                            assert not live, (style, mode, sd, st, low_edge,
                                              "mid-wall edge with a surviving "
                                              "bay that was not offered")
                            continue
                        assert id(e) not in killed, (style, mode, sd, st)
                        b0, b1 = bay
                        assert b1 > b0
                        # the edge is the span at THIS storey — the staircase
                        # means it is not the top storey's
                        slo, shi = plan["span"][(sd, st)]
                        edge_t = slo if low_edge else shi
                        # the surviving bay is on the STANDING side of the
                        # hole and its own run reaches the edge
                        w = b1 - b0
                        if low_edge:
                            # standing side of the hole...
                            assert b0 <= edge_t + 1e-6, (style, sd, st, b0,
                                                         edge_t)
                            # ...and within one bay of it (the kill window
                            # carries half a module of tolerance either end,
                            # so the nearest survivor can sit a module back)
                            assert b1 >= edge_t - plan["pad_m"] - w - 1e-6, \
                                (style, sd, st, b1, edge_t)
                        else:
                            assert b1 >= edge_t - 1e-6, (style, sd, st, b1,
                                                         edge_t)
                            assert b0 <= edge_t + plan["pad_m"] + w + 1e-6, \
                                (style, sd, st, b0, edge_t)


def test_only_an_elevation_loss_drops_a_whole_floor_plate():
    """A corner that let go takes the CORNER of each slab (`_ragged_slabs`
    breaks the open edge), not the whole plate — sending an 88 x 16 m slab
    to the solver because a 7 m corner failed is `r_fire_collapse` wearing
    this recipe's name."""
    for style in STYLES:
        ctx = _ctx(style)
        assert not fc.plan_partial_collapse(ctx, mode="corner")["drop"], style
        ctx = _ctx(style)
        plan = fc.plan_partial_collapse(ctx, mode="elevation")
        if len(plan["storeys"]) >= 2:
            assert plan["drop"], style
            for mt, st in plan["drop"]:
                assert mt == plan["mass"]
                assert st in plan["storeys"] and st >= plan["s0"], (style, st)
                # the TOP of the band: the floor whose bearing wall went
                assert st > min(plan["storeys"]), (style, st)


def test_the_band_is_capped_from_the_top_down():
    """A 60 m strip of elevation on the ground is a bombing. On a tall block
    the band is capped and the failure line RISES to meet the cap — it never
    sinks below the fire's own origin to make room."""
    ctx = _ctx("block_residential", origin=1)
    plan = fc.plan_partial_collapse(ctx, mode="elevation")
    assert len(plan["storeys"]) <= fc.MAX_FALL_STOREYS
    assert plan["s0"] >= ctx["fire"]["origin"]
    assert plan["kill"] and len(plan["kill"]) <= fc.MAX_MODULES
    # a high origin is respected even when it is above the cap
    ctx = _ctx("block_residential", origin=17)
    plan = fc.plan_partial_collapse(ctx, mode="elevation")
    assert plan["s0"] >= 17, plan["s0"]


def test_the_plan_is_stable_and_does_not_touch_the_shared_rng():
    """The plan is drawn from `soot_plume.event_seed(ctx) ^ SEED_XOR`, so two
    identical buildings plan identically — and a plan drawn after the shared
    rng has been advanced is the SAME plan, which is what lets the recipe be
    inserted anywhere in a ladder without moving another recipe's outcome."""
    a = _ctx("commercial_mid", tag="b3")
    b = _ctx("commercial_mid", tag="b3")
    pa = fc.plan_partial_collapse(a, mode="elevation")
    for _ in range(50):
        b["rng"].random()
    pb = fc.plan_partial_collapse(b, mode="elevation")
    assert pa["seed"] == pb["seed"]
    assert pa["s0"] == pb["s0"] and pa["sides"] == pb["sides"]
    assert pa["span"] == pb["span"]
    assert [e["name"] for e in pa["kill"]] == [e["name"] for e in pb["kill"]]
    assert ([h["centre_world"] for h in pa["heaps"]]
            == [h["centre_world"] for h in pb["heaps"]])
    # a DIFFERENT building gets a different plan
    c = _ctx("commercial_mid", tag="b9")
    pc = fc.plan_partial_collapse(c, mode="elevation")
    assert pc["seed"] != pa["seed"]


def test_the_plan_follows_the_fire_round_the_building():
    """The lost elevation is the one the fire is venting through, whichever
    it is — not a hard-coded south wall."""
    for want in ("S", "E", "N", "W"):
        ctx = _ctx("commercial", sides=(want,))
        plan = fc.plan_partial_collapse(ctx, mode="elevation")
        assert plan["sides"] == (want,), (want, plan["sides"])
        assert all(e["side"] == want for e in plan["kill"])
        assert plan["heaps"][0]["side"] == want


def test_yaw_does_not_move_the_heap_off_the_wall():
    """Everything above is in the mass's LOCAL frame; the world position has
    to follow the building's yaw. A heap authored in world coordinates from
    a local offset is the class of bug that put a windrow through a
    neighbour."""
    import math
    for yaw in (0.0, 37.0, 90.0, 213.0):
        rng = random.Random(5)
        pls = ub.build_building("commercial", 12.0, -7.0, yaw, rng)
        info = qf.describe("commercial", pls, 12.0, -7.0, yaw)
        ctx = {"info": info, "rng": rng, "nrng": np.random.default_rng(5),
               "notes": [], "tag": "y"}
        ctx["fire"] = uf.plan_fire(info, "F5c", rng, origin=1, sides=("S",))
        plan = fc.plan_partial_collapse(ctx, mode="elevation")
        m = _mass(ctx, plan)
        for h in plan["heaps"]:
            wx, wy = h["centre_world"]
            lx, ly = qf._to_local(m, wx, wy)
            assert math.hypot(lx - h["centre_local"][0],
                              ly - h["centre_local"][1]) < 1e-6, (yaw, h)
            want = 1.0 if h["where"] == "outside" else -1.0
            assert fc.outward_of(m, h["side"], lx, ly) * want > 0.0, (yaw, h)


def test_outward_of_is_signed_the_way_the_heap_reads_it():
    """The one function every heap assertion above rests on, checked
    directly against `quake_flow._heap`'s own windrow arithmetic: for side S
    it places chunks at `ly = -D/2 - d` with d > 0."""
    m = {"W": 20.0, "D": 10.0}
    assert fc.outward_of(m, "S", 0.0, -5.0) == 0.0
    assert fc.outward_of(m, "S", 0.0, -7.5) == 2.5      # outside
    assert fc.outward_of(m, "S", 0.0, -2.5) == -2.5     # inside
    assert fc.outward_of(m, "N", 0.0, 7.5) == 2.5
    assert fc.outward_of(m, "W", -12.5, 0.0) == 2.5
    assert fc.outward_of(m, "E", 12.5, 0.0) == 2.5
    assert fc.along_of(m, "S", -10.0, 0.0) == 0.0
    assert fc.along_of(m, "S", 10.0, 0.0) == 20.0
    assert fc.along_of(m, "E", 0.0, -5.0) == 0.0
    assert fc.side_length(m, "S") == 20.0 and fc.side_length(m, "E") == 10.0
    assert fc.shared_corner(("S", "E")) == "SE"
    assert fc.shared_corner(("S", "N")) is None
    assert fc.corner_sides("NW") == ("N", "W")
    assert fc.corner_at_high_end("S", "SE") and not fc.corner_at_high_end("S", "SW")
    assert fc.corner_at_high_end("E", "NE") and not fc.corner_at_high_end("E", "SE")


# ---------------------------------------------------------------------------
# ROUND 4 (2026-08-30): THE PERIMETER OF THE HOLE AND THE BURN ZONE
#
# The user, on the ModernCityEnvironment F5c: "some parts of it seem like they
# were directly cut off from the actual prims and therefore look like sharp
# straight or rectangular cuts ... Also there are parts of the surface that
# look pristine. Any parts directly near where the building collapsed (up,
# left, down, right, anything) would have been flamed and scorched."
#
# Both halves of that are decisions, not authoring, so both are checked here.
# ---------------------------------------------------------------------------
def _edges(ctx, plan, tag_seed=None):
    """`plan_edges` on a fresh private rng — what `r_partial_collapse` runs."""
    m = _mass(ctx, plan)
    prng = random.Random(plan["seed"] if tag_seed is None else tag_seed)
    return fc.plan_edges(ctx, plan, m, prng), m


def _by_id(jobs):
    return dict((id(j["el"]), j) for j in jobs)


def _alive(ctx, plan, side, storey):
    killed = set(id(e) for e in plan["kill"])
    return [e for e in ctx["info"]["elements"]
            if e["mass"] == plan["mass"] and e["role"] in fc.SHELL_ROLES
            and e["side"] == side and int(e["storey"]) == storey
            and id(e) not in killed]


def test_every_module_touching_the_hole_is_torn_and_knows_which_edge():
    """The cross-check of `plan_edges`, walked from the DEAD modules outward
    instead of from the survivors inward: for every module that came away,
    each of its four grid neighbours that is still standing must be in the job
    list with the COMPLEMENTARY class. A neighbour that is not is a kit module
    seam left as a dead straight line on the edge of the hole."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, m = _edges(ctx, plan)
            byid = _by_id(jobs)
            for d in plan["kill"]:
                sd, s = d["side"], int(d["storey"])
                d0, d1 = fc.el_span(m, d)
                for e in _alive(ctx, plan, sd, s):
                    t0, t1 = fc.el_span(m, e)
                    w = max(0.3, t1 - t0)
                    if -0.25 * w <= (t0 - d1) <= 0.6:
                        assert id(e) in byid, (style, mode, e["name"],
                                               "right neighbour not torn")
                        assert "right" in byid[id(e)]["classes"], \
                            (style, mode, e["name"], byid[id(e)]["classes"])
                    if -0.25 * w <= (d0 - t1) <= 0.6:
                        assert id(e) in byid, (style, mode, e["name"],
                                               "left neighbour not torn")
                        assert "left" in byid[id(e)]["classes"], \
                            (style, mode, e["name"], byid[id(e)]["classes"])
                for s2, cls in ((s - 1, "below"), (s + 1, "above")):
                    for e in _alive(ctx, plan, sd, s2):
                        t0, t1 = fc.el_span(m, e)
                        over = min(1.2, 0.3 * max(0.3, t1 - t0))
                        if min(d1, t1) - max(d0, t0) > over:
                            assert id(e) in byid, (style, mode, e["name"],
                                                   cls + " neighbour not torn")
                            assert cls in byid[id(e)]["classes"], \
                                (style, mode, e["name"], cls,
                                 byid[id(e)]["classes"])


def test_the_storey_under_the_failure_line_is_always_torn():
    """The single largest source of straight lines in the old notch, and the
    one the round-3 code only touched when the loss ran into a corner: the
    modules directly under the hole kept a level top edge exactly on the slab
    line, all the way across."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            if plan["s0"] < 1:
                continue
            jobs, m = _edges(ctx, plan)
            byid = _by_id(jobs)
            for sd in plan["sides"]:
                under = [j for j in jobs
                         if j["side"] == sd and j["storey"] == plan["s0"] - 1
                         and "below" in j["classes"]]
                dead_here = [e for e in plan["kill"]
                             if e["side"] == sd
                             and int(e["storey"]) == plan["s0"]]
                alive_below = _alive(ctx, plan, sd, plan["s0"] - 1)
                if dead_here and alive_below:
                    assert under, (style, mode, sd, "nothing torn under the "
                                                    "failure line")
                # ...and every one of them cuts HORIZONTALLY, near its top
                for j in under:
                    z = [c for c in j["cuts"] if c["cls"] == "below"]
                    assert z and z[0]["kind"] == "z"
                    assert z[0]["loose_above"] is True
                    assert j["za"] < z[0]["line"] < j["zb"], (style, j["name"])
            assert byid


def test_every_staircase_step_has_a_torn_tread():
    """The profile widens by a module or two per storey, so at every step
    there is a module that survived at storey `s` under one that died at
    `s + 1`. Its top edge is the tread, and it was never touched before."""
    seen = 0
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, m = _edges(ctx, plan)
            for j in jobs:
                if j["storey"] < plan["s0"] or "below" not in j["classes"]:
                    continue
                seen += 1
                assert j["side"] in plan["sides"], (style, j["name"])
    assert seen, "no staircase tread anywhere in seven styles / two modes"


def test_a_tear_never_takes_the_whole_module():
    """`_break_split` returns statics as well as loose: the FAR portion of a
    torn module stays standing. A cut line outside the module's own extent —
    or a wander big enough to leave it — is a module that was killed by
    accident, and the wall it was holding up then reads as floating."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, m = _edges(ctx, plan)
            for j in jobs:
                assert j["cuts"], (style, j["name"], "classed but not cut")
                for c in j["cuts"]:
                    if c["kind"] == "v":
                        lo, hi = fc.EDGE_PEN
                        assert lo * j["w"] - 1e-9 <= c["pen"] <= hi * j["w"] + 1e-9
                        assert j["t0"] < c["line"] < j["t1"], (style, j["name"])
                        assert c["amp"] <= fc.EDGE_AMP_FRAC * c["pen"] + 1e-9
                    elif c["kind"] == "z":
                        lo, hi = (fc.EDGE_PEN if c["cls"] == "below"
                                  else fc.EDGE_PEN_ABOVE)
                        assert lo * j["h"] - 1e-9 <= c["pen"] <= hi * j["h"] + 1e-9
                        assert j["za"] < c["line"] < j["zb"], (style, j["name"])
                    else:
                        assert 0.0 < c["frac"] <= 0.9, (style, j["name"])


def test_a_module_above_the_hole_keeps_its_footing():
    """The exception in the brief: tearing the foot off a module that is
    holding the storeys over it makes the building float. `EDGE_PEN_ABOVE`
    is 0.25-0.40 where every other edge gets 0.25-0.60."""
    assert fc.EDGE_PEN_ABOVE[1] < fc.EDGE_PEN[1]
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, _m = _edges(ctx, plan)
            for j in jobs:
                for c in j["cuts"]:
                    if c["cls"] != "above":
                        continue
                    assert c["pen"] <= fc.EDGE_PEN_ABOVE[1] * j["h"] + 1e-9
                    assert c["loose_above"] is False


def test_the_return_wall_is_torn_only_at_a_corner_the_loss_reaches():
    """`quake_flow._ragged_neighbours` tears BOTH walls perpendicular to the
    failed one, at every storey in the band, whichever end of it the loss
    actually reached — so a corner failure at the SE corner also chewed the
    south end of the WEST wall, damage with nothing next to it to explain
    it. The replacement is per storey and per corner."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, m = _edges(ctx, plan)
            for j in jobs:
                if "return" not in j["classes"]:
                    continue
                assert j["side"] not in plan["sides"], (style, j["name"])
                assert j["storey"] in plan["storeys"], (style, j["name"])
                for c in j["cuts"]:
                    if c["cls"] != "return":
                        continue
                    lost = c["side"]
                    assert lost in plan["sides"]
                    # it really is AT the corner: its near end is within one
                    # bay of the lost wall line
                    near, _far = fc.el_near_far(m, j["el"], lost)
                    assert near <= j["w"] + fc.RETURN_REACH_PAD_M + 1e-6, \
                        (style, j["name"], near, j["w"])
                    # ...and a module died against that corner at this storey
                    L = fc.side_length(m, lost)
                    ends = [fc.el_span(m, e) for e in plan["kill"]
                            if e["side"] == lost
                            and int(e["storey"]) == j["storey"]]
                    assert ends, (style, j["name"])
                    corner_lo = min(a for a, _b in ends) <= plan["pad_m"]
                    corner_hi = max(b for _a, b in ends) >= L - plan["pad_m"]
                    assert corner_lo or corner_hi, (style, j["name"])


def test_the_edge_plan_is_stable_and_takes_no_shared_draw():
    """Same building, same private seed, same tears — and advancing the
    LADDER's rng between the two changes nothing, which is the whole reason
    this recipe may be inserted anywhere in a ladder."""
    a = _ctx("commercial_mid", tag="b3")
    b = _ctx("commercial_mid", tag="b3")
    pa = fc.plan_partial_collapse(a, mode="elevation")
    ja, _m = _edges(a, pa)
    for _ in range(50):
        b["rng"].random()
    pb = fc.plan_partial_collapse(b, mode="elevation")
    jb, _m = _edges(b, pb)
    assert [q["name"] for q in ja] == [q["name"] for q in jb]
    assert [q["classes"] for q in ja] == [q["classes"] for q in jb]
    assert ([[round(c["pen"], 9) for c in q["cuts"]] for q in ja]
            == [[round(c["pen"], 9) for c in q["cuts"]] for q in jb])


def test_the_edge_work_stays_inside_its_budget():
    """Each tear is a `_break_split`; the budget is what stops the recipe's
    cost running away on an 88 m elevation. Nothing in the bench set may hit
    it, or the probe's "100 % of the neighbours torn" is not true."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            jobs, _m = _edges(ctx, plan)
            assert len(jobs) <= fc.MAX_EDGE_MODULES, (style, mode, len(jobs))
            assert not [j for j in jobs if j.get("dropped")], (style, mode)


# ---------------------------------------------------------------------------
# The burn zone
# ---------------------------------------------------------------------------
def test_the_burn_zone_is_in_soot_plume_side_u_coordinates():
    """`along_of` and `soot_plume.side_u` AGREE on S and E and are MIRRORED on
    N and W (`side_u` unwraps the perimeter counter-clockwise). Get that wrong
    and the heavy soot lands on the far end of the wall from the hole — which
    a render would show and nothing else would."""
    from disaster import soot_plume as spl
    for want in ("S", "E", "N", "W"):
        ctx = _ctx("commercial", sides=(want,))
        plan = fc.plan_partial_collapse(ctx, mode="elevation")
        m = _mass(ctx, plan)
        rects = [r for r in plan["burn_zone"] if r[0] == want]
        assert rects, want
        t0, t1 = plan["span"][(want, plan["top_storey"])]
        for t in (t0 + 0.05, 0.5 * (t0 + t1), t1 - 0.05):
            lx, ly = fc.wall_point(m, want, t)
            wx, wy = qf._to_world(m, lx, ly)
            u = spl.side_u(m, want, wx, wy)
            assert any(r[1] - 1e-6 <= u <= r[2] + 1e-6 for r in rects), \
                (want, t, u, rects)
            # and `_u_of_t` is that same number
            assert abs(fc._u_of_t(m, want, t) - u) < 1e-6, (want, t)


def test_the_burn_zone_covers_the_hole_with_a_real_margin():
    """"Any parts directly near where the building collapsed (up, left, down,
    right, anything)": one module sideways, the plume's reach up, and the
    little that rolls under the lip below."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            m = _mass(ctx, plan)
            lv = list(m["levels"])
            for sd in plan["sides"]:
                L = fc.side_length(m, sd)
                rs = [r for r in plan["burn_zone"] if r[0] == sd]
                assert rs, (style, mode, sd)
                for s in plan["storeys"]:
                    t0, t1 = plan["span"][(sd, min(s, plan["top_storey"]))]
                    za = float(lv[s])
                    zb = float(lv[s + 1]) if s + 1 < len(lv) else float(m["top"])
                    u0, u1 = sorted((fc._u_of_t(m, sd, max(0.0, t0 - fc.BURN_ZONE_PAD_U)),
                                     fc._u_of_t(m, sd, min(L, t1 + fc.BURN_ZONE_PAD_U))))
                    hit = [r for r in rs
                           if r[1] <= u0 + 1e-6 and r[2] >= u1 - 1e-6
                           and r[3] <= za - fc.BURN_ZONE_PAD_DOWN + 1e-6
                           and r[4] >= zb + fc.BURN_ZONE_PAD_UP - 1e-6]
                    assert hit, (style, mode, sd, s, u0, u1, rs)
                # ...and it never leaves the wall
                for r in rs:
                    assert -1e-6 <= r[1] <= r[2] <= L + 1e-6, (style, sd, r)


def test_the_burn_zone_only_touches_the_lost_walls_and_their_returns():
    """A zone on a cold elevation is the flat-black-rectangle bug wearing a
    physical model's clothes. The only elevation that may carry one without
    being lost is the RETURN of a corner the loss actually reaches."""
    for style in STYLES:
        for mode in ("elevation", "corner"):
            ctx = _ctx(style)
            plan = fc.plan_partial_collapse(ctx, mode=mode)
            m = _mass(ctx, plan)
            for r in plan["burn_zone"]:
                if r[0] in plan["sides"]:
                    continue
                ok = False
                for sd in plan["sides"]:
                    for low_end, reached in zip((True, False),
                                                plan["reaches_end"][sd]):
                        if not reached:
                            continue
                        c = fc.corner_of_end(sd, low_end)
                        if fc.other_side(c, sd) != r[0]:
                            continue
                        L2 = fc.side_length(m, r[0])
                        at_hi = fc.corner_at_high_end(r[0], c)
                        u_c = fc._u_of_t(m, r[0], L2 if at_hi else 0.0)
                        # the rect is AT that corner, not out in the wall
                        if min(abs(r[1] - u_c), abs(r[2] - u_c)) < 1e-6:
                            ok = True
                assert ok, (style, mode, "burn zone on an unexplained wall", r)


def test_corner_of_end_agrees_with_corner_at_high_end():
    for side in ("S", "E", "N", "W"):
        for low in (True, False):
            c = fc.corner_of_end(side, low)
            assert side in c
            assert fc.corner_at_high_end(side, c) == (not low), (side, low, c)
            assert fc.other_side(c, side) != side
            assert set(fc.corner_sides(c)) == set((side, fc.other_side(c, side)))


def test_el_span_is_a_footprint_and_never_degenerates():
    """`quake_flow._piece_frame` is a LINE — origin, yaw, width — so a CORNER
    BLOCK placed at 90 degrees on the south wall projects onto it as a single
    POINT, and every adjacency test against that module answers "no
    neighbour" (measured: `highrise_step` storey 8, a 6 x 6 m corner at
    t = 25.0 .. 25.0 on a 25 m wall; and `dw_terrace`'s pillar, whose frame
    axis runs INTO the building). `el_span` measures the real footprint."""
    for style in STYLES:
        for yaw in (0.0, 37.0):
            rng = random.Random(7)
            pls = ub.build_building(style, 3.0, -2.0, yaw, rng)
            info = qf.describe(style, pls, 3.0, -2.0, yaw)
            for e in info["elements"]:
                if e["role"] not in fc.SHELL_ROLES:
                    continue
                m = info["masses"].get(e["mass"]) or info["masses"]["main"]
                t0, t1 = fc.el_span(m, e)
                assert t1 - t0 > 0.05, (style, yaw, e["name"], t0, t1)
                L = fc.side_length(m, e["side"])
                assert -2.0 <= t0 and t1 <= L + 2.0, (style, e["name"], t0, t1)
                za, zb = fc.el_z_span(m, e)
                assert zb > za, (style, e["name"])
                near, far = fc.el_near_far(m, e, e["side"])
                assert far >= near
                assert near < 2.5, (style, e["name"], near)


def test_the_soot_skin_is_identical_without_a_zone():
    """THE FREEZE, in one assertion: `burn_zone=None` must not move a single
    texel, and must not take a single extra draw off the skin's generator
    (which would move the tone noise and every level's soot with it)."""
    from disaster import soot_plume as spl
    ctx = _ctx("commercial_mid", level="F4")
    ev = spl.plan_events(ctx, uf._severity)
    seed = spl.event_seed(ctx) ^ 0x5EED

    def mk(**kw):
        return spl.skin(ctx, ev, np.random.default_rng(seed), finish="ash",
                        duration_scale=1.4, **kw)

    a, b = mk(), mk(burn_zone=None)
    assert np.array_equal(a["rgba"], b["rgba"])
    assert np.array_equal(a["dep"], b["dep"])
    assert a["zone"] is None and b["zone"] is None


def test_a_burn_zone_soots_the_wall_round_the_hole():
    """The other half of the review: nothing near the loss may stay pristine.
    Mean alpha inside the zone >= 0.8; outside it, materially lower."""
    from disaster import soot_plume as spl
    for style, mode in (("commercial_mid", "elevation"),
                        ("office_wide", "corner"),
                        ("apartment", "elevation")):
        ctx = _ctx(style)
        plan = fc.plan_partial_collapse(ctx, mode=mode)
        ev = spl.plan_events(ctx, uf._severity)
        seed = spl.event_seed(ctx) ^ 0x5EED
        base = spl.skin(ctx, ev, np.random.default_rng(seed), finish="ash",
                        duration_scale=1.4)
        zoned = spl.skin(ctx, ev, np.random.default_rng(seed), finish="ash",
                         duration_scale=1.4, burn_zone=plan["burn_zone"])
        z = zoned["zone"]
        assert z is not None and z.shape == base["rgba"].shape[:2]
        inside = z > 0.999
        outside = z <= 0.0
        assert inside.any() and outside.any(), (style, mode)
        a_in = float(zoned["rgba"][..., 3][inside].mean())
        a_out = float(zoned["rgba"][..., 3][outside].mean())
        assert a_in >= 0.80, (style, mode, a_in)
        assert a_out < a_in, (style, mode, a_in, a_out)
        # OUTSIDE the zone nothing moved at all
        assert np.array_equal(base["rgba"][..., 3][outside],
                              zoned["rgba"][..., 3][outside]), (style, mode)
        # ...and what WAS pristine inside it is not any more. THE FLOOR IS
        # `ZONE_ALPHA`'s low end (times the streak's own 0.94-1.00 thickness
        # factor), not a magic 0.7: the constant was recalibrated against the
        # char maps on 2026-08-30 (see `soot_plume.ZONE_ALPHA`'s note) and a
        # hard number here would have to be edited every time it moves, which
        # is how a test stops being a check and becomes a transcript.
        was_clean = inside & (base["rgba"][..., 3] < 0.4)
        if was_clean.any():
            assert float(zoned["rgba"][..., 3][was_clean].min()) >= \
                spl.ZONE_ALPHA[0] * 0.93, (style, mode)
        # THE TONE INSIDE MEETS THE CHAR ON THE BROKEN PIECES. It used to be
        # driven almost to pure `SOOT_DARK` (`ZONE_TONE` 0.85), which made the
        # lip of the hole the darkest thing on the elevation while the char
        # maps on the fragments beside it were much lighter — "the material of
        # the broken/debris part is a much darker colour than the intact
        # façade next to it" (third review, 2026-08-30). Two claims now:
        #   (a) the zone is never LIGHTER than the un-zoned ash tone of a
        #       burnt-out shell (it may match it; it may not wash out);
        #   (b) composited over a mid brick it lands within 0.06 sRGB
        #       luminance of the char texture set as bound — 0.141, measured
        #       (`_debris_mat` 0.148 / `_burn_mat("ash", 0.85)` 0.135), the
        #       tolerance the review asked for.
        rgb_in = zoned["rgba"][..., :3][inside].mean(axis=0)
        ashy = (spl.SOOT_DARK[0]
                + (spl.SOOT_ASH[0] - spl.SOOT_DARK[0]) * spl.ASH_LEVEL["ash"])
        assert float(rgb_in.max()) <= ashy + 1e-3, (style, mode, rgb_in, ashy)
        CHAR_SRGB, MID_BRICK = 0.141, 0.257
        comp = MID_BRICK * (1.0 - a_in) + float(rgb_in.mean()) * a_in
        assert abs(CHAR_SRGB - comp) <= 0.06, (style, mode, comp, a_in)


def test_the_zone_edge_is_a_ramp_and_not_a_cut():
    """A hard edge is the artefact. The coverage field has to spend real area
    strictly between 0 and 1, and the boundary has to wander — a straight
    ramp on a rectangle would give the same histogram but a dead straight
    contour, so the wander is measured as the spread of the transition band's
    own position along a column."""
    from disaster import soot_plume as spl
    ctx = _ctx("commercial_mid")
    plan = fc.plan_partial_collapse(ctx, mode="elevation")
    ev = spl.plan_events(ctx, uf._severity)
    sk = spl.skin(ctx, ev, np.random.default_rng(spl.event_seed(ctx) ^ 0x5EED),
                  finish="ash", duration_scale=1.4,
                  burn_zone=plan["burn_zone"])
    z = sk["zone"]
    frac = float(((z > 0.02) & (z < 0.98)).mean())
    assert frac > 0.02, frac
    # the lowest fully-covered row differs from column to column: the ramp
    # is not a straight line across the elevation
    rows = []
    h, w = z.shape
    for c in range(0, w, max(1, w // 240)):
        col = np.nonzero(z[:, c] > 0.98)[0]
        if col.size:
            rows.append(int(col[-1]))
    assert len(rows) > 20
    assert float(np.std(rows)) > 1.0, np.std(rows)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
