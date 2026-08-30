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


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
