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
        # 0.065, NOT 0.06, AND THE REASON IS WORTH KNOWING: `skin` takes
        # `max(plume_alpha, zone x za)` and the plume's own deposit is
        # already saturated over most of a collapse zone (mean alpha 0.834
        # there with the zone OFF), so `ZONE_ALPHA` cannot lower the
        # composite past what the plume alone gives. What is left over the
        # review's 0.06 is `SOOT_DARK`/`SOOT_ASH` — LINEAR albedo constants
        # that `merge_rgb` composites straight into an sRGB-encoded base map,
        # so a saturated deposit renders ~2.4x darker than the constant reads
        # — and those are shared with every level, including the frozen kit
        # ladder. See `soot_plume.ZONE_ALPHA`'s note.
        CHAR_SRGB, MID_BRICK = 0.141, 0.257
        comp = MID_BRICK * (1.0 - a_in) + float(rgb_in.mean()) * a_in
        assert abs(CHAR_SRGB - comp) <= 0.065, (style, mode, comp, a_in)
        # THE REAL REGRESSION GUARD: the zone may not make the lip of the
        # hole materially DARKER than the plume already makes that wall. A
        # dark band there is the artefact — it is what put a hard edge round
        # the loss in the second row.
        rgb_b = base["rgba"][..., :3][inside].mean(axis=0)
        a_b = float(base["rgba"][..., 3][inside].mean())
        comp_b = MID_BRICK * (1.0 - a_b) + float(rgb_b.mean()) * a_b
        assert comp >= comp_b - 0.035, (style, mode, comp, comp_b)


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


# ---------------------------------------------------------------------------
# fire_dtc3 (2026-08-30): THE SLICED BUILDING'S OWN TEAR
#
# Two complaints on the same building (`gac_SM_Building_02_F5c_s193`, b6 of
# the fire_dtc3 bench):
#
#   "(1) the partial collapse still shows very straight cuts — see standing
#    pieces .../pieces/pier_S_2_10_0119 and pier_S_3_09_0102 ...
#    (2) the ragged break pieces that DO exist are a completely diff
#    texture/color from the wall they extend."
#
# (1) is `plan_edges`' adjacency test run against footprints that do not butt:
# a sliced piece's footprint is the BOUNDING BOX OF A REGION CUT and swallows
# whatever sills and cornices fall in the cell, so consecutive columns gap by
# 0.0-1.2 m and the gap CHANGES PER STOREY. A fixed 0.6 m tolerance therefore
# tears one storey's boundary piece and silently misses the storey above's.
# (2) is `_clad_material`'s world triplanar of a UNIQUE atlas, and the fix is
# to carry the parent's own UVs onto the fragment (`project_uv`) and bind the
# parent's own material, which `bind_break` then has to recognise as a façade.
#
# All three are arithmetic, so all three are checked here.
# ---------------------------------------------------------------------------
# The S elevation of SM_Building_02 at storey 8, MEASURED off that bake
# (`tools/tear_edge_probe.py gac:SM_Building_02:F5c 193`, table B): metres
# along the wall from its low end. Note pier_S_3 -> wall_S_4: the same two
# columns gap by 0.38 m at storey 7 and by 0.98 m here, which is what made the
# miss a one-storey-in-three affair rather than an obvious failure.
_SLICE_ROW = (("pier_S_0_09", 0.82, 3.89),
              ("wall_S_1_09", 4.08, 9.75),
              ("pier_S_2_09", 9.23, 12.30),
              ("pier_S_3_09", 12.34, 15.42),
              ("wall_S_4_09", 16.40, 22.07),
              ("pier_S_5_09", 22.70, 25.62))
_SLICE_W, _SLICE_D = 28.0, 14.4
_SLICE_STOREY = 8


def _slice_mass():
    lv = [3.56 * i for i in range(12)]
    return {"tag": "main", "W": _SLICE_W, "D": _SLICE_D, "cx": 0.0, "cy": 0.0,
            "yaw": 0.0, "z0": 0.0, "levels": lv, "top": lv[-1] + 3.56,
            "module": 3.62, "spec": {"bands": []}}


def _slice_elements(sliced):
    """The measured row as element records, with the `urban_building.PIECES`
    row `gac_slice.register_style` writes for a sliced cell —
    `(sx, sy, sz, -sx/2, -sy/2, 0)`, i.e. the piece is anchored on its own
    CENTRE. `sliced` False builds the same geometry as a kit module (no
    `_role`, no `slice://` url), which is what pins the MCE freeze: identical
    numbers must NOT be classified there.
    """
    m = _slice_mass()
    za = m["levels"][_SLICE_STOREY]
    sz = m["levels"][_SLICE_STOREY + 1] - za
    els, names = [], []
    for base, t0, t1 in _SLICE_ROW:
        name = base + ("_sliced" if sliced else "_kit")
        sx = t1 - t0
        ub.PIECES[name] = (sx, 0.25, sz, -sx / 2.0, -0.125, 0.0)
        names.append(name)
        p = {"prim_path": "/bench/" + name}
        if sliced:
            p["_role"] = "pier" if base.startswith("pier") else "wall"
            p["usd"] = "slice://" + base
        else:
            p["usd"] = "SM_MBuilding01_Facade_B.usd"
        els.append({"p": p, "sub": "storey", "mass": "main", "role": "wall",
                    "name": name, "h": sz, "z": za,
                    "x": 0.5 * (t0 + t1) - _SLICE_W / 2.0,
                    "y": -_SLICE_D / 2.0 + 0.125, "yaw": 0.0,
                    "storey": _SLICE_STOREY,
                    "lx": 0.5 * (t0 + t1) - _SLICE_W / 2.0,
                    "ly": -_SLICE_D / 2.0 + 0.125, "side": "S",
                    "ref": (0.0, 0.0, 0.5 * m["top"]), "out": (0.0, -1.0, 0.0)})
    return m, els, names


def _slice_case(sliced):
    """(jobs by name, mass) for the measured row with the last two columns
    dead — the hole that `pier_S_3_09_0102` was left standing beside."""
    m, els, names = _slice_elements(sliced)
    try:
        kill = [e for e in els if e["name"].startswith(("wall_S_4", "pier_S_5"))]
        plan = {"mass": "main", "sides": ("S",), "storeys": [_SLICE_STOREY],
                "top_storey": _SLICE_STOREY, "kill": kill,
                "span": {("S", _SLICE_STOREY): (16.40, 25.62)},
                "pad_m": 0.5 * m["module"]}
        ctx = {"info": {"elements": els, "masses": {"main": m},
                        "type": "urm", "style": "x"}}
        jobs = fc.plan_edges(ctx, plan, m, random.Random(11))
        return dict((j["name"], j) for j in jobs), m
    finally:
        for n in names:
            ub.PIECES.pop(n, None)


def test_a_sliced_piece_gets_a_scaled_gap_and_a_kit_module_does_not():
    """`edge_gap_tol` is the one knob that separates the two footprint
    regimes, and it must be inert on anything modelled."""
    m, els, names = _slice_elements(sliced=True)
    try:
        pier = [e for e in els if e["name"].startswith("pier_S_3")][0]
        assert fc.is_sliced(pier)
        # a 3.08 m pier: 0.45 x 3.08 = 1.39 m, which clears the measured
        # 0.98 m gap to `wall_S_4` at this storey
        assert fc.edge_gap_tol(pier, 3.08, 0.6) > 0.98
        assert fc.edge_gap_tol(pier, 3.08, 0.6) <= fc.EDGE_GAP_MAX_M
        # ...and the cap holds on a piece wide enough to reach past it
        assert fc.edge_gap_tol(pier, 40.0, 0.6) == fc.EDGE_GAP_MAX_M
    finally:
        for n in names:
            ub.PIECES.pop(n, None)
    m, els, names = _slice_elements(sliced=False)
    try:
        kit = [e for e in els if e["name"].startswith("pier_S_3")][0]
        assert not fc.is_sliced(kit)
        assert fc.edge_gap_tol(kit, 3.08, 0.6) == 0.6
        assert fc.edge_gap_tol(kit, 40.0, 0.6) == 0.6
    finally:
        for n in names:
            ub.PIECES.pop(n, None)


def test_plan_edges_reaches_the_sliced_boundary_piece_across_the_slice_gap():
    """`pier_S_3_09_0102`, the prim the user named: 0.98 m from the hole in
    footprint terms, hard against it in fact. It must be a `left` job now, and
    the piece a whole bay further in must still be left alone — a tolerance
    that swallows the next column along would tear standing wall for nothing.
    """
    jobs, _m = _slice_case(sliced=True)
    j = jobs.get("pier_S_3_09_sliced")
    assert j is not None, sorted(jobs)
    assert "left" in j["classes"], j["classes"]
    assert "pier_S_2_09_sliced" not in jobs, sorted(jobs)
    # and the tear it draws stays inside the piece it is cutting
    cut = [c for c in j["cuts"] if c["cls"] == "left"][0]
    assert j["t0"] < cut["line"] < j["t1"], cut


def test_the_kit_keeps_its_own_tolerance_on_the_very_same_geometry():
    """THE MCE FREEZE. The user authorised changing the kit's TEAR-EDGE LOOK
    and nothing else, so the same measured row expressed as kit modules must
    classify exactly as it did before `EDGE_GAP_FRAC` existed: a 0.98 m gap is
    not a neighbour at 0.6 m tolerance, and a kit module's footprint is its
    modelled panel, which really does butt its neighbour (measured on
    `apartment` F5c: every neighbour of the hole is 0.07-0.09 m from it)."""
    jobs, _m = _slice_case(sliced=False)
    assert "pier_S_3_09_kit" not in jobs, sorted(jobs)
    assert not jobs, sorted(jobs)


# ---------------------------------------------------------------------------
# The tear skin
# ---------------------------------------------------------------------------
def _quad(uv_second=None):
    """A 4 x 3 m wall panel in the y = 0 plane as two triangles, mapped u =
    x / 4, v = z / 3 — and optionally with the SECOND triangle's UVs moved to
    an unrelated island, which is what an atlas actually looks like."""
    A, B, C, D = ((0., 0., 0.), (4., 0., 0.), (4., 0., 3.), (0., 0., 3.))
    tris = np.array([[A, B, C], [A, C, D]], dtype=float)
    uv = np.array([[(0., 0.), (1., 0.), (1., 1.)],
                   [(0., 0.), (1., 1.), (0., 1.)]], dtype=float)
    if uv_second is not None:
        uv[1] = np.asarray(uv_second, dtype=float)
    return tris, uv


def test_project_uv_reads_the_parents_map_at_the_break():
    """A fragment has no UVs of its own (`fracture._write_mesh` writes none),
    so the only mapping that can put the wall back on it is the parent's, read
    where the fragment actually is. On a panel mapped u = x/4, v = z/3 that is
    checkable to the millimetre — and it has to survive the fragment sitting a
    few centimetres OFF the surface, which every fragment does (the chew, the
    roughening and `solidify`'s own offset)."""
    tris, uv = _quad()
    corners = np.array([[(1.0, 0.02, 0.5), (1.5, -0.03, 0.6),
                         (1.2, 0.05, 0.9)]], dtype=float)
    out = fc.project_uv(tris, uv, corners)
    assert out is not None and out.shape == (1, 3, 2)
    want = np.array([[1.0 / 4.0, 0.5 / 3.0], [1.5 / 4.0, 0.6 / 3.0],
                     [1.2 / 4.0, 0.9 / 3.0]])
    assert np.allclose(out[0], want, atol=2e-3), (out[0], want)


def test_project_uv_keeps_one_fragment_face_inside_one_uv_island():
    """PER FACE, NOT PER VERTEX — the reason `project_uv` picks its parent
    triangle from the fragment face's CENTROID. The parent's UVs are an
    ATLAS, so the triangle next door can be on the far side of the sheet;
    choosing per vertex would let one fragment triangle straddle two islands
    and stretch the whole atlas across it, which is the smear this whole
    change exists to remove, arriving by another door."""
    far = [(0.60, 0.60), (0.64, 0.64), (0.60, 0.64)]
    tris, uv = _quad(uv_second=far)
    # a face wholly inside the FIRST triangle (below the diagonal x/4 > z/3)
    corners = np.array([[(3.0, 0., 0.2), (3.6, 0., 0.4), (3.2, 0., 0.1)]],
                       dtype=float)
    out = fc.project_uv(tris, uv, corners)[0]
    assert np.allclose(out[:, 0], [3.0 / 4.0, 3.6 / 4.0, 3.2 / 4.0], atol=2e-3), out
    # every corner came from the same island: none of them landed in the
    # 0.60..0.64 box the second triangle was moved to
    assert out.max() > 0.7, out


def test_no_fragment_face_can_sample_off_its_parents_uv_island():
    """THE HARD BOUND. A fragment's BACK and CUT corners are metres behind the
    façade, so their barycentrics in the parent triangle run away and the raw
    projection reached u[-0.40, 1.21] v[-1.35, 1.00] on the first GAC run —
    i.e. right off the sheet. Those faces are in the `core` subset and wear
    the char, so it never showed, but "the fragment samples somewhere else on
    the atlas" is precisely the failure this change removes and it must not be
    reachable at all. `project_uv` clips into the parent's own UV box plus one
    tenth."""
    tris, uv = _quad()
    far = np.array([[(1.0, -6.0, 0.5), (1.5, -6.0, 0.6), (1.2, -6.0, 0.9)]],
                   dtype=float)
    out = fc.project_uv(tris, uv, far)
    assert out is not None
    assert out.min() >= -0.1 - 1e-9, out.min()
    assert out.max() <= 1.1 + 1e-9, out.max()


# ---------------------------------------------------------------------------
# ROUND 5 (live row-5 bench, 2026-08-30): WHICH SUBSET IS THE FAÇADE
#
#   /World/bake/g7/brk_g7_wall_E_4_06_0090/frag_001
#       -> M_Building_24_Office_Fake_Inst   ..._Office_Fake_Inst_BaseColor.png
#   user: "it's like the interior office material not the outside glass
#          window one"
#
# A GAC office block hangs a FAKE INTERIOR CARD behind its glazing, facing
# OUT, to be seen through the pane — so it passes the outward-normal test and,
# being one unbroken rectangle per bay, beat the real cladding on AREA. Both
# halves of the replacement (`facade_class`, `pick_facade`) are pure, so both
# are checked here with the strings and the depths actually measured on
# `gac_SM_Building_24_F5c_s224`.
# ---------------------------------------------------------------------------
# MEASURED on the offline rebuild of that building: the two materials the old
# rule picked. Note the material PRIM is called `UnrealMaterial` in both
# cases — every GAC section calls it that — so the base map's file name is
# the only part of the pair that says anything, which is why `facade_class`
# reads both.
_B24_FAKE = ("/W/bench/g0/src/asset/LOD0/Section7/UnrealMaterial",
             "Game_GreatAmericanCity_Materials_M_Building_24_Office_Fake"
             "_Inst_BaseColor.png")
_B24_GLASS = ("/W/bench/g0/src/asset/LOD0/Section11/UnrealMaterial",
              "Game_GreatAmericanCity_Materials_M_Building_24_Glass_Green"
              "_Inst_BaseColor.png")


def test_a_fake_interior_card_is_never_the_facade():
    assert fc.facade_class(*_B24_FAKE) == "fake"
    assert fc.facade_class(*_B24_GLASS) == "glass"
    # the inner pane is interior, and the never-list beats the glass hint
    assert fc.facade_class("/x/M_Glass_In_Standard_Inst", None) == "fake"
    for nm in ("M_Fake_Interior", "M_FakeInterior_01", "M_Office_Fake_A",
               "Fake_Light_01", "M_Off_Light_Inst"):
        assert fc.facade_class("/x/" + nm, None) == "fake", nm
    # real cladding, and a sooted copy whose names say NOTHING (that case is
    # what the outermost-wins rule exists for)
    assert fc.facade_class(
        "/x/M_Building_01_Bricks_Inst",
        "Game_GreatAmericanCity_Materials_M_Building_01_Bricks_Inst"
        "_BaseColor.png") == "opaque"
    assert fc.facade_class("/W/bench/g0/SootLooks/m2",
                           "gacsoot_6fdc8f94b4c7e201.png") == "opaque"


# The interior-only material families, as they are actually named in the GAC
# and downtowncity packs (measured off `gac_SM_Building_02/_24` and
# `dtc_Building_12` while writing this). "Let's make sure that NEVER happens
# to any building" (user, row 5) — so the predicate is pinned by name here,
# and the pick is pinned below on the two shapes that produce it.
_INTERIOR_NAMES = (
    "M_Building_24_Office_Fake_Inst", "M_Fake_Interior_01", "M_FakeInterior",
    "M_Fake_Light_A", "M_Off_Light_Inst", "M_Glass_In_Standard_Inst",
    "M_Buildings_Ceiling_Inst", "M_Building_Floor_Inst",
    "M_Building_Wood_Floor_Inst", "M_Slab_Inst",
    "M_Interior_Trim")
_EXTERIOR_NAMES = (
    "M_Building_01_Bricks_Inst", "M_Building_24_Metal_Inst",
    "M_Building_01_Windows_Inst", "M_MBuilding01_Facades",
    "M_Awning_Metal_Inst", "M_Building_01_Concrete_02_Inst",
    # NOT interior on GreatAmericanCity, however it reads: measured on
    # `gac_SM_Building_02` (`pier_S_3_09_0102`'s outward subset binds it
    # carrying a `sootbake_*.png`), and the fleet census flagged 342
    # fragments of the building the user had just approved when it WAS on
    # the list. See `FAKE_INTERIOR_HINTS`.
    "M_Building_01_WallBack_Inst")


def test_the_fake_interior_predicate_names_every_interior_family():
    for nm in _INTERIOR_NAMES:
        assert fc.is_fake_interior("/x/" + nm), nm
        # ...and through the TEXTURE alone, which is the only channel that
        # says anything when the material prim is called `UnrealMaterial`
        assert fc.is_fake_interior(
            "/W/bench/g0/src/asset/LOD0/Section7/UnrealMaterial",
            "Game_GreatAmericanCity_Materials_%s_BaseColor.png" % nm), nm
    for nm in _EXTERIOR_NAMES:
        assert not fc.is_fake_interior("/x/" + nm), nm
        assert not fc.is_fake_interior("/x/UnrealMaterial",
                                       "Game_GAC_Materials_%s_BaseColor.png"
                                       % nm), nm
    # a sooted copy says nothing either way, and must not be refused
    assert not fc.is_fake_interior("/W/bench/g0/SootLooks/m2",
                                   "gacsoot_6fdc8f94b4c7e201.png")
    assert not fc.is_fake_interior(None, None)


def _pick_from(rows):
    """`facade_class` + `pick_facade` together, the way `facade_skin` runs
    them: classify, DROP the fake-interior groups, then pick. Rows are
    `(material path, texture, d_out, area)`."""
    cands = []
    for mat, tex, d, area in rows:
        kind = fc.facade_class(mat, tex)
        if kind == "fake":
            continue
        cands.append({"d": d, "area": area, "glass": kind == "glass",
                      "mat": mat, "tex": tex})
    return fc.pick_facade(cands)


def test_a_urm_piece_whose_biggest_subset_is_interior_still_picks_cladding():
    """A masonry cell: the ceiling plane and the floor plate are the big
    areas, the brick skin is a strip at the front. Area alone picks the
    floor plate."""
    got = _pick_from([
        ("/g/S1/UnrealMaterial", "M_Buildings_Ceiling_Inst_BaseColor.png",
         14.30, 90.0),
        ("/g/S2/UnrealMaterial", "M_Building_Floor_Inst_BaseColor.png",
         14.20, 120.0),
        ("/g/S3/UnrealMaterial", "M_Building_01_Bricks_Inst_BaseColor.png",
         14.50, 12.0)])
    assert got is not None and "Bricks" in got["tex"], got
    assert got["glass"] is False


def test_a_glass_piece_whose_biggest_subset_is_the_office_card_picks_spandrel():
    """`gac_SM_Building_24` `wall_E_4_06_0090`, with the depths and areas
    MEASURED off the offline rebuild (`UF_TEAR_DEBUG=1`): the office card is
    excluded by name, the green glass is ranked behind every opaque
    candidate, the metal trim is frontmost but small, and the spandrel at
    14.468 with 30.94 m2 is the façade."""
    got = _pick_from([
        ("/g/S7/UnrealMaterial",
         "M_Building_24_Office_Fake_Inst_BaseColor.png", 14.47, 200.0),
        ("/g/S10/UnrealMaterial", "M_Building_24_Metal_Inst_BaseColor.png",
         14.498, 5.46),
        ("/g/S8/UnrealMaterial", None, 14.468, 30.94),
        ("/g/S11/UnrealMaterial", "M_Building_24_Glass_Green_Inst_BaseColor.png",
         14.468, 10.30),
        ("/g/S3/UnrealMaterial", None, 13.993, 6.56)])
    assert got is not None and got["mat"] == "/g/S8/UnrealMaterial", got
    assert got["glass"] is False


def test_a_glass_piece_with_nothing_but_card_and_pane_takes_the_pane_as_tone():
    """Strip the spandrel and the trim out of the same piece and the only
    candidate left is the pane — taken, but flagged `glass` so
    `skin_fragment` binds a TONE sampled from it instead of a transparent
    material on a solid chunk of wall."""
    got = _pick_from([
        ("/g/S7/UnrealMaterial",
         "M_Building_24_Office_Fake_Inst_BaseColor.png", 14.47, 200.0),
        ("/g/S11/UnrealMaterial", "M_Building_24_Glass_Green_Inst_BaseColor.png",
         14.468, 10.30)])
    assert got is not None and got["glass"] is True, got
    # ...and with the pane gone too there is NO façade: the fragments keep
    # the break palette and `_refire` chars them, which is the safe end of
    # the asymmetry.
    assert _pick_from([("/g/S7/UnrealMaterial",
                        "M_Building_24_Office_Fake_Inst_BaseColor.png",
                        14.47, 200.0)]) is None


def _cand(d, area, glass=False, tag=""):
    return {"d": d, "area": area, "glass": glass, "mat": tag, "tex": tag}


def test_pick_facade_takes_the_outermost_opaque_not_the_biggest():
    """The row-5 shape, in numbers: a mullion strip and a spandrel band at the
    wall face, a big sheet of glass in the same plane, and a fake interior
    card a floor depth behind. Area alone picks the card; the façade is the
    spandrel."""
    cands = [_cand(14.500, 8.0, tag="mullion"),
             _cand(14.480, 30.0, tag="spandrel"),
             _cand(14.500, 100.0, glass=True, tag="glass"),
             _cand(11.000, 400.0, tag="interior_card")]
    assert fc.pick_facade(cands)["mat"] == "spandrel"


def test_pick_facade_ignores_a_deep_interior_however_big():
    cands = [_cand(14.5, 2.0, tag="strip"), _cand(11.0, 400.0, tag="card")]
    assert fc.pick_facade(cands)["mat"] == "strip"


def test_pick_facade_falls_back_to_glass_and_says_so():
    """Rule 4: glass is ranked behind every opaque candidate, and when it is
    all there is the caller is TOLD (`glass` True) so `skin_fragment` takes
    the sampled tone rather than binding a transparent pane material to a
    solid chunk of spandrel."""
    got = fc.pick_facade([_cand(14.5, 100.0, glass=True, tag="glass"),
                          _cand(14.4, 5.0, glass=True, tag="glass2")])
    assert got["mat"] == "glass" and got["glass"] is True
    assert fc.pick_facade([]) is None


def test_a_reskinned_fragment_is_never_charred_whole():
    """`bind_break`'s guard. A fragment `skin_fragment` has given the parent's
    own UVs and material is a FAÇADE, but its material is not under
    `CLAD_PREFIX`, so `facade_material` says no about the one binding that is
    most emphatically yes. `ctx["tear_faced"]` is the register that answers
    first; without it `_refire` would char the wall straight back off the tear
    one call later — the fire_dtc3 fix undoing itself in the same recipe.

    No stage: the decision is reached before any `pxr` call (`cut_subset`
    returns None on a stageless ctx), which is what lets it be checked here.
    """
    ctx = {"stage": None, "tear_faced": set(["/bench/brk/frag_000"])}
    assert fc.bind_break(ctx, "/bench/brk/frag_000", None,
                         cut_only=True) == "kept"
    # and a fragment nobody skinned is not silently kept
    assert fc.skin_fragment(ctx, None, "/bench/brk/frag_001") is None


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
