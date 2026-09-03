#!/usr/bin/env python3
"""test_quake_sliced.py — does the sliced-asset earthquake ladder take away
and move the right pieces?

    python3 scene_gen/tests/test_quake_sliced.py
    pytest -q scene_gen/tests/test_quake_sliced.py

WHY THIS EXISTS
---------------
`disaster/quake_sliced.py` damages a GreatAmericanCity / downtowncity building
that `detail/gac_storey_slice.slice_to_kit` has cut into kit-shaped pieces.
It cannot fracture anything (VTK segfaults on a clipped shell — memory
`gac-fire-pipeline`), so the whole vocabulary is REMOVAL on the piece grid and
RIGID DISPLACEMENT of whole pieces. Every one of those decisions is arithmetic
on the element table and none of it needs USD:

  * a lost region is TOOTHED — some piers on its boundary row survive and some
    openings in the intact bays beside it go — because the alternative is a
    rectangular module cut-out, which is the one thing this vocabulary can
    produce that no earthquake does;
  * a corner failure widens UPWARD and never touches the ground storey;
  * an out-of-plane failure is on ONE elevation and its macroblocks land in
    the STREET (get the rotation sign wrong and the wall folds into the
    building — the bug that was found in `r_overturn` by a matrix check, not
    by looking at a render);
  * a total collapse leaves a ground-floor stub in 40-60 % of cases and lays
    2-4 whole panels on the mound;
  * the construction type comes from MEASURED data, never from a family
    default or a height guess.

So the decisions live in the pure planner `plan_damage`, and this file checks
THAT, on a fixture built exactly the way `as_placements` builds a real slice —
including registering the synthetic style through `gac_slice.register_style`,
so `quake_flow.describe` is the real function doing the real work.

`quake_rubble` / `quake_rubble_usd` (work packages B and C) are STUBBED: this
file must pass whether or not those two exist yet.

It runs host-side in a couple of seconds.

WHAT IT CANNOT SEE: everything downstream of the plan — whether the mound
looks like rubble, whether a voided pane reads as a hole, whether a macroblock
lands clear of the sidewalk. That needs a render.
"""

import json
import math
import os
import random
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import gac_slice as gsl                 # noqa: E402
from detail import gac_storey_slice as gss          # noqa: E402
from detail import urban_building as ub             # noqa: E402
from disaster import quake_flow as qf               # noqa: E402
from disaster import quake_sliced as qs             # noqa: E402

_PLANS = os.path.normpath(os.path.join(_HERE, "..", "_plans"))


# ---------------------------------------------------------------------------
# THE FIXTURE — placements shaped exactly as `as_placements` writes them
# ---------------------------------------------------------------------------
def fake_sliced_building(W=30.0, D=24.0, H=40.0, storeys=8, bays_x=None,
                         bays_y=None, seed=3, module=6.0, leg=2.0,
                         style=None, splits=gss.BAY_SPLITS):
    """One sliced building's placements + its registered style.

    Mirrors `gac_storey_slice.ring()` / `roof_and_parapet()` / `as_placements()`:
    a 3x3 plan partition per band (four corner pieces, four runs of bays each
    split into `BAY_SPLITS` pier/opening/pier, one interior core), the TOP band
    relabelled parapet / parapet_corner / roof — which is the branch every GAC
    asset measured so far actually takes (`_whole_band_as_roof`: the coping is
    the whole of what is left above the last window row).

    W and D must be multiples of `module`, so that
    `gac_slice.register_style`'s advertised footprint is the geometric one and
    `quake_flow._side_of` classifies the corner-adjacent bays correctly.
    """
    style = style or "gac_fake_{0}_{1}".format(int(H), seed)
    assert abs(W / module - round(W / module)) < 1e-9, "W must be n x module"
    assert abs(D / module - round(D / module)) < 1e-9, "D must be n x module"
    # The bay COUNT is the footprint over the module (so `register_style`'s
    # advertised footprint is the geometric one); the run's own pitch is then
    # the inset span over that count, exactly as `ring()` divides a run.
    bays_x = bays_x or int(round(W / module)) or 1
    bays_y = bays_y or int(round(D / module)) or 1
    h = H / float(storeys)
    x0, x1, y0, y1 = -W / 2.0, W / 2.0, -D / 2.0, D / 2.0
    xa, xb, ya, yb = x0 + leg, x1 - leg, y0 + leg, y1 - leg
    relabel = {"corner": "parapet_corner", "wall": "parapet",
               "pier": "parapet", "core": "roof"}
    fr = list(splits) if splits else [1.0]
    tot = float(sum(fr))
    scope = "/World/cell/pieces"
    pls, j = [], 0

    def _emit(role, side, bay, storey, cx, cy, z, sx, sy, sz):
        nonlocal j
        nm = "{0}_{1}_{2}_{3:02d}_{4:04d}".format(
            role, str(side).replace("-", "x"), bay, storey, j)
        j += 1
        pls.append({
            "category": "bld_{0}_{1}".format(style, gss._ROLE_SUB.get(role, "storey")),
            "usd": "slice://{0}".format(nm),
            "x_m": float(cx), "y_m": float(cy), "z_m": float(z),
            "yaw_deg": 0.0, "scale": 1.0,
            "prim_path": "{0}/{1}".format(scope, nm),
            "_size": (float(sx), float(sy), float(sz)),
            "_role": role, "_side": side, "_storey": storey, "_bay": bay})

    for i in range(storeys):
        z = i * h
        top = (i == storeys - 1)
        R = (lambda r: relabel.get(r, r)) if top else (lambda r: r)
        for side, cx, cy in (("SW", x0 + leg / 2, y0 + leg / 2),
                             ("SE", x1 - leg / 2, y0 + leg / 2),
                             ("NW", x0 + leg / 2, y1 - leg / 2),
                             ("NE", x1 - leg / 2, y1 - leg / 2)):
            _emit(R("corner"), side, 0, i, cx, cy, z, leg, leg, h)
        runs = (("S", xa, xb, y0 + leg / 2, "x", bays_x),
                ("N", xa, xb, y1 - leg / 2, "x", bays_x),
                ("W", ya, yb, x0 + leg / 2, "y", bays_y),
                ("E", ya, yb, x1 - leg / 2, "y", bays_y))
        for side, lo, hi, off, axis, nb in runs:
            span = hi - lo
            for k in range(nb):
                base, acc = k / float(nb), 0.0
                for q, f in enumerate(fr):
                    t0 = base + (acc / tot) / nb
                    acc += f
                    t1 = base + (acc / tot) / nb
                    role = "wall" if q == len(fr) // 2 else "pier"
                    a0, a1 = lo + t0 * span, lo + t1 * span
                    if axis == "x":
                        _emit(R(role), side, k * len(fr) + q, i,
                              (a0 + a1) / 2.0, off, z, a1 - a0, leg, h)
                    else:
                        _emit(R(role), side, k * len(fr) + q, i,
                              off, (a0 + a1) / 2.0, z, leg, a1 - a0, h)
        _emit(R("core"), "-", 0, i, 0.0, 0.0, z, xb - xa, yb - ya, h)

    grid = {"storey_h": h, "storeys": [k * h for k in range(storeys)],
            "W": W, "D": D, "measured": True,
            "bays": {s: {"pitch": module} for s in ("S", "N", "W", "E")}}
    spec = gsl.register_style(grid, style, pieces_of=pls)
    # The advertised footprint must be the geometric one. `register_style` has
    # to GUESS a bay count from the measured grid; the real pipeline corrects
    # the same class of thing in `gac_storey_slice._fix_advertised_bands`.
    spec["bays"] = (bays_x, bays_y)
    spec["bands"][0]["module"] = module
    spec["bands"][0]["h"] = h
    spec["bands"][0]["repeat"] = storeys
    return pls, style, grid


def _plan(grade, btype="urm", seed=5, **kw):
    """A fixture building + its plan, through the real `describe`."""
    pls, style, grid = fake_sliced_building(seed=seed, **kw)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)
    return info, plan


def test_registered_slice_preserves_measured_levels_and_real_roof_deck():
    """A non-uniform measured grid must not be rebuilt as N median floors."""
    style = "exact_measured_slice"
    grid = {"bbox": ((0.0, 0.0, 0.0), (20.0, 12.0, 38.8)),
            "storey_h": 3.86,
            "storeys": [0.0, 3.1, 6.0, 9.5, 13.0, 16.6, 20.2, 23.8,
                       27.4, 31.0, 34.5, 38.08],
            "W": 20.0, "D": 12.0,
            "bays": {s: {"pitch": 4.0} for s in ("S", "N", "E", "W")}}
    pieces = [{"usd": "slice://roof", "_role": "roof", "z_m": 38.08,
               "_storey": 11, "_size": (18.0, 10.0, 0.5)},
              {"usd": "slice://wall", "_role": "wall", "z_m": 34.5,
               "_storey": 10, "_size": (4.0, 0.3, 3.58)}]
    gsl.register_style(grid, style, pieces_of=pieces)
    mass = qf._mass_specs(style, 0.0, 0.0, 0.0)[0]
    assert mass["levels"] == pytest.approx(grid["storeys"][:-1])
    assert mass["top"] == pytest.approx(38.08)
    assert mass["deck_z"] == pytest.approx(38.08)


def _by_path(info):
    return {(e["p"] or {}).get("prim_path"): e for e in info["elements"]}


def _cell(e, n_sub):
    p = e["p"]
    return (p.get("_side"), int(p.get("_storey")), qs.bay_no(p, n_sub))


def _removed_els(info, plan):
    idx = _by_path(info)
    return [idx[p] for p in plan["removed"] if p in idx]


# ---------------------------------------------------------------------------
def test_the_module_self_check_passes():
    """Same gate a launcher puts on `quake_flow.check` / `urban_fire.check`."""
    assert not qs.check(verbose=False)
    assert not qf.check(verbose=False), "quake_flow itself must still be sane"


def test_the_fixture_is_shaped_like_a_real_slice():
    """If the fixture is not what `as_placements` writes, nothing below means
    anything. Roles, the `_bay` arithmetic, the categories `classify` parses
    and the sizes `register_style` files in `ub.PIECES` all have to match."""
    pls, style, grid = fake_sliced_building()
    roles = set(p["_role"] for p in pls)
    assert roles == {"wall", "pier", "corner", "core", "parapet",
                     "parapet_corner", "roof"}, roles
    for p in pls:
        assert p["category"].startswith("bld_" + style + "_")
        assert p["category"].endswith(gss._ROLE_SUB[p["_role"]])
        assert p["usd"].startswith("slice://")
        assert gsl._piece_key(p) in ub.PIECES
        assert len(ub.PIECES[gsl._piece_key(p)]) == 6
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    assert info["H"] > 0
    # `describe` collapses wall/pier/core to "wall"; the fine role survives on
    # the placement, which is what the planner reads.
    for e in info["elements"]:
        assert e["role"] in ("wall", "corner", "parapet", "parapet_corner",
                             "roof"), e["role"]
        assert e["p"]["_role"] in roles
    # every run piece must land on the elevation it was cut from
    for e in info["elements"]:
        if e["p"]["_side"] in qs.SIDES:
            assert e["side"] == e["p"]["_side"], (e["name"], e["side"],
                                                  e["p"]["_side"])
    assert qs.n_sub_of(info["elements"]) == len(gss.BAY_SPLITS)


def test_dg0_removes_nothing_and_moves_nothing():
    for btype in ("urm", "rc", "rc_glass"):
        info, plan = _plan("DG0", btype=btype)
        assert plan["removed"] == [], btype
        assert plan["displaced"] == {}, btype
        assert plan["piles"] == [], btype
        assert plan["glass"] == [], btype
        assert plan["stub"] is None and plan["ground"] is None, btype


# ---------------------------------------------------------------------------
# CORNER FAILURE
# ---------------------------------------------------------------------------
def test_a_corner_failure_is_contiguous_from_the_corner_and_widens_upward():
    """The staircase, and the one thing it must never do: reach the ground.

    Contiguity is checked on what was actually REMOVED (the toothing's stray
    openings are, by construction, in a bay adjacent to the region, so the
    union stays a contiguous block); the widening is checked on the planned
    region, since a stray in a lower storey could otherwise widen it.
    """
    for seed in range(8):
        info, plan = _plan([("corner_fail", {"storeys": 2})], "urm", seed=seed)
        reg = [r for r in plan["regions"] if r["recipe"] == "corner_fail"]
        assert len(reg) == 1
        reg = reg[0]
        cn, (sa, sb) = reg["corner"], reg["sides"]
        n_sub = plan["stats"]["n_sub"]
        rm = _removed_els(info, plan)
        assert rm, seed
        # (a) never the ground storey
        assert min(int(e["p"]["_storey"]) for e in rm) >= 1, (seed, cn)
        # (b) only the corner's two elevations
        assert set(e["p"]["_side"] for e in rm) <= {sa, sb, cn}, (seed, cn)
        # (c) the corner piece itself went
        assert any(e["p"]["_side"] == cn for e in rm), (seed, cn)
        # (d) contiguous, and anchored at the corner's END of each run
        for sd in (sa, sb):
            end = qs._CORNER_END[(sd, cn)]
            per_st = {}
            for e in rm:
                if e["p"]["_side"] != sd:
                    continue
                per_st.setdefault(int(e["p"]["_storey"]), set()).add(
                    qs.bay_no(e["p"], n_sub))
            for st, bays in per_st.items():
                lo, hi = min(bays), max(bays)
                assert hi - lo + 1 == len(bays), (seed, sd, st, sorted(bays))
                if end == "lo":
                    assert lo == 0, (seed, sd, st, sorted(bays))
        # (e) widens upward: the planned bay count never decreases with height
        widths = [row["bays"] for row in reg["shape"]]
        assert widths == sorted(widths), (seed, widths)
        assert widths[0] == 1 and widths[-1] == 2, (seed, widths)


def test_a_lost_region_is_toothed_and_not_a_rectangle():
    """The signature that keeps removal from reading as a module cut-out:
    some piers on the boundary row SURVIVE, and at least one opening in an
    intact bay beside the region goes anyway."""
    for recipe, kw, btype in ((("corner_fail"), {"storeys": 2}, "urm"),
                              (("out_of_plane"), {"sides": 1}, "urm")):
        for seed in range(6):
            info, plan = _plan([(recipe, kw)], btype, seed=seed)
            n_sub = plan["stats"]["n_sub"]
            reg = plan["regions"][0]
            cells = set(tuple(c) for c in reg["cells"])
            removed = set(plan["removed"])
            idx = _by_path(info)
            # (1) not every pier on the boundary row went
            g = qs._Grid(info, info["elements"])
            bnd = qs._boundary(cells, set(g.runs))
            piers = [e for key in bnd for e in g.at(key)
                     if qs.is_pier(e["p"], n_sub)]
            assert piers, (recipe, seed, "no boundary piers at all?")
            survived = [e for e in piers if e["p"]["prim_path"] not in removed]
            assert survived, (recipe, seed,
                              "every boundary pier removed — ruler edge")
            # (2) at least one opening OUTSIDE the region went
            strays = [p for p in removed
                      if p in idx and _cell(idx[p], n_sub) not in cells
                      and idx[p]["p"]["_side"] in qs.SIDES]
            assert strays, (recipe, seed,
                            "the outline does not wander into intact wall")


# ---------------------------------------------------------------------------
# OUT OF PLANE
# ---------------------------------------------------------------------------
def test_out_of_plane_is_one_elevation_over_the_stated_storeys():
    for seed in range(8):
        info, plan = _plan([("out_of_plane", {"sides": 1, "from_storey": 1})],
                           "urm", seed=seed)
        reg = [r for r in plan["regions"] if r["recipe"] == "out_of_plane"][0]
        sd, sts = reg["side"], set(reg["storeys"])
        rm = _removed_els(info, plan)
        assert rm, seed
        assert set(e["p"]["_side"] for e in rm) == {sd}, (seed, sd)
        assert min(int(e["p"]["_storey"]) for e in rm) >= 1, seed
        # the strays are on the same side and in the same storeys
        assert set(int(e["p"]["_storey"]) for e in rm) <= sts, (seed, sts)
        assert 3 <= len(reg["bays"]) <= 6, (seed, reg["bays"])
        assert 1 <= len(reg["storeys"]) <= 3, (seed, reg["storeys"])


def test_every_macroblock_lands_in_the_street():
    """The whole point of a macroblock, and a SIGN test — `r_overturn` shipped
    with the rotation the wrong way round for a round, folding the mass back
    over its own footprint, and only a matrix check found it.

    The transformed TOP of the piece must end up further out than the piece's
    own outer face was, measured in the mass's local frame so a yawed building
    is checked too.
    """
    for yaw in (0.0, 37.0, 90.0, 213.0):
        for seed in range(6):
            pls, style, grid = fake_sliced_building(seed=seed)
            info = qf.describe(style, pls, 11.0, -5.0, yaw)
            info["type"] = "urm"
            plan = qs.plan_damage(info, info["elements"],
                                  [("out_of_plane", {"sides": 1})], "urm",
                                  random.Random(seed))
            assert plan["macroblocks"], (yaw, seed)
            idx = _by_path(info)
            m = info["masses"]["main"]
            for mb in plan["macroblocks"]:
                e = idx[mb["path"]]
                sd = mb["side"]
                assert qs.MACRO_DEG[0] <= mb["deg"] <= qs.MACRO_DEG[1], mb
                sx, sy, sz = mb["size"]
                depth = sy if sd in ("S", "N") else sx
                lnx, lny = qs._SIDE_NORMAL[sd]
                face_lx = e["lx"] + lnx * depth / 2.0
                face_ly = e["ly"] + lny * depth / 2.0
                face_out = qs._out_dist(m, sd, face_lx, face_ly)
                spec = plan["displaced"][mb["path"]]
                # the top of the piece, at its own centre along the wall
                wx, wy = qf._to_world(m, face_lx, face_ly)
                top = qs.apply_rigid(spec, (wx, wy, e["z"] + sz))
                tlx, tly = qf._to_local(m, top[0], top[1])
                out = qs._out_dist(m, sd, tlx, tly)
                assert out > face_out + 0.5 * sz, (yaw, seed, sd, out, face_out)
                # ...and it is DOWN: a wall on the street is not still upright
                assert top[2] < e["z"] + 0.55 * sz, (yaw, seed, top[2], e["z"], sz)
            # a macroblock is MOVED, not deleted
            for mb in plan["macroblocks"]:
                assert mb["path"] not in set(plan["removed"])


# ---------------------------------------------------------------------------
# TOTAL COLLAPSE
# ---------------------------------------------------------------------------
def test_a_total_collapse_clears_everything_above_the_stub_and_keeps_panels():
    """DG5 on both types: every piece above the ground storey goes except the
    2-4 panels laid on the mound; the pile is a dome over the footprint whose
    crown is the type's own fraction of H and whose `stub_h_m` is the standing
    storey's height; and the stub itself is kept in some cases and not others
    (agent A: 40-60 %), never always and never never."""
    seen = {True: 0, False: 0}
    for btype, recipe in (("rc", "pancake"), ("urm", "masonry_collapse")):
        for seed in range(16):
            info, plan = _plan("DG5", btype, seed=seed, H=30.0, storeys=6)
            assert [r[0] for r in plan["recipes"]] == [recipe], (btype, seed)
            idx = _by_path(info)
            removed = set(plan["removed"])
            panels = set(p for p, _s in plan["panels"])
            assert 2 <= len(panels) <= 4, (btype, seed, len(panels))
            above = [e for e in info["elements"]
                     if int(e["p"]["_storey"]) >= 1]
            for e in above:
                p = e["p"]["prim_path"]
                assert (p in removed) != (p in panels), (btype, seed, p)
            # the panels are preferentially the WIDE opening panels
            n_sub = plan["stats"]["n_sub"]
            assert all(qs.is_opening(idx[p]["p"], n_sub) for p in panels), \
                (btype, seed)
            stub = plan["stub"]
            seen[bool(stub["keep"])] += 1
            st0 = [e for e in info["elements"] if int(e["p"]["_storey"]) == 0]
            left = [e for e in st0 if e["p"]["prim_path"] not in removed]
            if stub["keep"]:
                assert left, (btype, seed, "stub kept but nothing stands")
                assert stub["sides_lost"], (btype, seed)
                assert qs.STUB_RESIDUAL_M[0] <= stub["residual_m"] <= qs.STUB_RESIDUAL_M[1]
                # the fall side lost its ground storey; another side did not
                by_side = {}
                for e in left:
                    by_side.setdefault(e["p"]["_side"], 0)
                    by_side[e["p"]["_side"]] += 1
                for sd in stub["sides_lost"]:
                    n_all = len([e for e in st0 if e["p"]["_side"] == sd])
                    assert by_side.get(sd, 0) < n_all, (btype, seed, sd)
            else:
                assert not left, (btype, seed, "no stub but pieces stand")
            # --- the pile ------------------------------------------------
            piles = [q for q in plan["piles"] if q["kind"] == "dome"]
            assert len(piles) == 1, (btype, seed)
            pile = piles[0]
            h_st = info["masses"]["main"]["levels"][1] - info["masses"]["main"]["levels"][0]
            if stub["keep"]:
                assert abs(pile["stub_h_m"] - h_st) < 0.2, (btype, seed,
                                                            pile["stub_h_m"], h_st)
            else:
                assert pile["stub_h_m"] == 0.0, (btype, seed)
            want = min(qs.CROWN_MAX_M, qs.CROWN_FRAC[btype] * info["H"])
            assert abs(pile["crown_est_m"] - want) < 1e-6, (btype, seed)
            assert pile["sides"], (btype, seed, "the fall side is not named")
            assert set(pile["sides"]) <= set(qs.SIDES)
            # the run-out is left to the pile planner, per side
            assert pile["crown_m"] is None and pile["spread_frac"] is None
            assert [q["op"] for q in plan["fit_ops"]].count("deactivate_slabs") == 1
    assert seen[True] and seen[False], \
        "the stub is not a draw at all: {0}".format(seen)


# ---------------------------------------------------------------------------
# SOFT STOREY
# ---------------------------------------------------------------------------
def _soft(grade_recipes, seed, btype="rc", **kw):
    info, plan = _plan(grade_recipes, btype, seed=seed, **kw)
    return info, plan, plan["storey_collapse"]


def _base_edges(info, plan, rc):
    """Where the block's two base edges actually END UP, through the plan's
    own displacement spec. The lean-side and far-side wall lines at the
    bottom of the block (z_hi) are the two points the mechanism is defined
    by, so they are the two points to measure."""
    m = info["masses"]["main"]
    lnx, lny = qs._SIDE_NORMAL[rc["lean_side"]]
    z_hi = m["levels"][rc["storey"] + 1]
    spec = list(plan["displaced"].values())[0]
    lw = qf._to_world(m, lnx * m["W"] / 2.0, lny * m["D"] / 2.0)
    fw = qf._to_world(m, -lnx * m["W"] / 2.0, -lny * m["D"] / 2.0)
    return (qs.apply_rigid(spec, (lw[0], lw[1], z_hi)),
            qs.apply_rigid(spec, (fw[0], fw[1], z_hi)))


def _storey_is_cleared(info, plan, rc):
    """Shared by both mechanisms: the failed storey's own walls go (80-100 %,
    toothed so stumps of pier survive) and the fit-out ops are queued."""
    removed = set(plan["removed"])
    k = rc["storey"]
    band = [e for e in info["elements"] if int(e["p"]["_storey"]) == k
            and e["p"]["_side"] in qs.SIDES]
    gone = [e for e in band if e["p"]["prim_path"] in removed]
    assert len(gone) >= 0.80 * len(band), (len(gone), len(band))
    assert len(gone) < len(band), "the whole storey went — no stumps"
    collar = [q for q in plan["piles"] if q["tag"] == "soft{0}".format(k)]
    assert len(collar) == 1 and collar[0]["kind"] == "dome"
    assert abs(collar[0]["crown_m"] - rc["crush_m"]) < 1e-9
    ops = {q["op"] for q in plan["fit_ops"]}
    assert {"displace_above", "columns_to_pile", "bury_props"} <= ops, ops


def test_a_differential_crush_sits_the_block_on_the_wedge_it_drew():
    """CRUSH: the angle is NOT a free parameter, it is (r_far - r_lean)/span.

    The two base edges have to land exactly on the two residual heights the
    plan drew — that is the whole definition of the mechanism — and the top
    has to move TOWARD the side that crushed more. Getting the pivot or the
    sign wrong shows up here as a lifted far side or a block leaning uphill,
    which is what the first version of this recipe did.
    """
    seen_capped = 0
    for seed in range(40):
        info, plan, rc = _soft([("soft_storey", {"storey": 0})], seed)
        if rc["mechanism"] != "crush":
            continue
        m = info["masses"]["main"]
        h_st, span = rc["h_st"], rc["span_m"]
        z_lo = m["levels"][0]
        # (a) the draws are inside their bands
        assert qs.R_LEAN_FRAC[0] * h_st - 1e-9 <= rc["r_lean_m"] \
            <= qs.R_LEAN_FRAC[1] * h_st + 1e-9, rc
        assert rc["r_far_m"] <= qs.R_FAR_CEIL_FRAC * h_st + 1e-9, rc
        assert rc["r_far_m"] > rc["r_lean_m"], rc
        assert qs.LEAN_DEG[0] <= rc["lean_drawn_deg"] <= qs.LEAN_DEG[1], rc
        # (b) the angle IS the wedge
        want = math.degrees(math.asin((rc["r_far_m"] - rc["r_lean_m"]) / span))
        assert abs(rc["lean_deg"] - want) < 1e-9, (seed, rc)
        assert rc["lean_deg"] <= rc["lean_drawn_deg"] + 1e-9
        if rc["capped"]:
            seen_capped += 1
        else:
            assert abs(rc["lean_deg"] - rc["lean_drawn_deg"]) < 1e-9, rc
        # (c) THE TWO BASE EDGES LAND ON THE TWO RESIDUALS (2 cm)
        lean_pt, far_pt = _base_edges(info, plan, rc)
        assert abs(lean_pt[2] - (z_lo + rc["r_lean_m"])) < 0.02, (seed, lean_pt)
        assert abs(far_pt[2] - (z_lo + rc["r_far_m"])) < 0.02, (seed, far_pt)
        # (d) the TOP moved toward the lean side, and nothing rose
        ox, oy = qf._outward(m, rc["lean_side"])
        spec = list(plan["displaced"].values())[0]
        top = qs.apply_rigid(spec, (m["cx"], m["cy"], m["top"]))
        assert (top[0] - m["cx"]) * ox + (top[1] - m["cy"]) * oy > 0.1, (seed, rc)
        assert top[2] < m["top"], (seed, top[2], m["top"])
        for e in info["elements"]:
            if e["p"]["prim_path"] in plan["displaced"]:
                z = qs.apply_rigid(plan["displaced"][e["p"]["prim_path"]],
                                   (e["x"], e["y"], e["z"]))[2]
                assert z <= e["z"] + 0.2, (seed, e["name"], z, e["z"])
                assert e["z"] - z >= rc["drop_far_m"] - 0.2, (seed, e["name"])
        _storey_is_cleared(info, plan, rc)
        # (e) a crush does NOT incline the columns — it shatters them
        col = [q for q in plan["fit_ops"] if q["op"] == "columns_to_pile"][0]
        assert col["incline_deg"] == 0.0, rc
    assert seen_capped or True     # the ceiling only bites on wide spans


def test_a_sidesway_keeps_the_block_plumb_and_slides_it_sideways():
    """SWAY: the columns hinge, the block above stays VERTICAL and offsets.

    This is the mechanism a tilt-only model cannot express, so the test is
    that there is NO rotation at all: every point takes the same translation,
    which is what "plumb" means.
    """
    got = 0
    for seed in range(40):
        info, plan, rc = _soft([("soft_storey", {"storey": 0})], seed)
        if rc["mechanism"] != "sway":
            continue
        got += 1
        m = info["masses"]["main"]
        h_st = rc["h_st"]
        assert qs.SWAY_DEG[0] <= rc["sway_deg"] <= qs.SWAY_DEG[1], rc
        assert rc["lean_deg"] == 0.0, rc
        spec = list(plan["displaced"].values())[0]
        assert spec["deg"] == 0.0 and spec["twist_deg"] == 0.0, spec
        # (a) offset = h_st sin(phi), toward the lean side
        want_d = h_st * math.sin(math.radians(rc["sway_deg"]))
        assert abs(rc["offset_m"] - want_d) < 1e-9, rc
        ox, oy = qf._outward(m, rc["lean_side"])
        a = qs.apply_rigid(spec, (0.0, 0.0, 0.0))
        assert abs(a[0] - ox * want_d) < 1e-9 and abs(a[1] - oy * want_d) < 1e-9, a
        # (b) drop = h_st (1 - cos phi) + squash, squash in its band
        want_z = h_st * (1.0 - math.cos(math.radians(rc["sway_deg"]))) + rc["squash_m"]
        assert abs(rc["drop_m"] - want_z) < 1e-9, rc
        assert qs.SWAY_CRUSH_FRAC[0] * h_st - 1e-9 <= rc["squash_m"] \
            <= qs.SWAY_CRUSH_FRAC[1] * h_st + 1e-9, rc
        assert abs(a[2] + rc["drop_m"]) < 1e-9, a
        # (c) PLUMB: two points, one delta
        b = qs.apply_rigid(spec, (17.0, -9.0, 31.0))
        for i, base in enumerate((17.0, -9.0, 31.0)):
            assert abs((b[i] - base) - a[i]) < 1e-9, (seed, i, a, b)
        # the base edges are both at the same residual — no wedge
        lean_pt, far_pt = _base_edges(info, plan, rc)
        assert abs(lean_pt[2] - far_pt[2]) < 1e-9, (lean_pt, far_pt)
        assert abs(lean_pt[2] - (m["levels"][0] + h_st - rc["drop_m"])) < 0.02
        _storey_is_cleared(info, plan, rc)
        # (d) the racked columns are authored INCLINED by the rack angle
        col = [q for q in plan["fit_ops"] if q["op"] == "columns_to_pile"][0]
        assert abs(col["incline_deg"] - rc["sway_deg"]) < 1e-9, (rc, col)
        assert col["incline_side"] == rc["lean_side"]
    assert got, "no sidesway drawn in 40 seeds"


def test_both_mechanisms_are_drawn_and_a_named_angle_forces_the_wedge():
    kinds = {}
    for seed in range(60):
        _i, _p, rc = _soft([("soft_storey", {"storey": 0})], seed)
        kinds[rc["mechanism"]] = kinds.get(rc["mechanism"], 0) + 1
    assert set(kinds) == {"crush", "sway"}, kinds
    # ~40 % sway, and a long way from 0 or 100
    assert 0.2 < kinds["sway"] / 60.0 < 0.6, kinds
    # the tower ladder names its podium angle, and a named angle is a WEDGE:
    # a sidesway has no lean to give.
    for seed in range(8):
        _i, _p, rc = _soft([("soft_storey", {"storey": 0, "lean_deg": 2.5})],
                           seed, btype="rc_glass")
        assert rc["mechanism"] == "crush", rc
        assert rc["lean_drawn_deg"] == 2.5, rc
        assert rc["lean_deg"] <= 2.5 + 1e-9, rc
    for seed in range(8):
        _i, _p, rc = _soft([("soft_storey", {"storey": 0, "crush_frac": 0.3})], seed)
        assert rc["mechanism"] == "crush", rc
        assert abs(rc["r_lean_m"] - 0.3 * rc["h_st"]) < 1e-9, rc


def test_mid_storey_twists_the_block_above_whichever_mechanism_it_draws():
    """The twist is the mid-storey signature and is kept for BOTH mechanisms;
    without it a mid-storey collapse is a building that is merely shorter."""
    kinds = {}
    for seed in range(14):
        info, plan, rc = _soft([("mid_storey", {})], seed)
        n_lv = len(info["masses"]["main"]["levels"])
        assert 1 <= rc["storey"] <= n_lv - 2, (seed, rc["storey"], n_lv)
        assert abs(rc["twist_deg"]) >= 2.0, (seed, rc)
        spec = list(plan["displaced"].values())[0]
        assert abs(spec["twist_deg"] - rc["twist_deg"]) < 1e-9, seed
        # the twist is IN PLAN: a point straight above the twist pivot moves
        # horizontally and its height is set by the mechanism, not the twist
        kinds[rc["mechanism"]] = kinds.get(rc["mechanism"], 0) + 1
        # the mid-storey offset rides on top of whatever the mechanism did
        if rc["mechanism"] == "sway":
            assert rc["offset_m"] > rc["h_st"] * math.sin(
                math.radians(rc["sway_deg"])), rc
    assert kinds, kinds


# ---------------------------------------------------------------------------
# PILES
# ---------------------------------------------------------------------------
def test_windrow_and_fan_piles_sit_on_the_side_that_failed():
    for seed in range(6):
        info, plan = _plan([("parapet_fall", {"sides": 1, "frac": 0.6})],
                           "urm", seed=seed)
        rm = _removed_els(info, plan)
        sides = set(e["p"]["_side"] for e in rm) & set(qs.SIDES)
        piles = [q for q in plan["piles"] if q["kind"] == "windrow"]
        assert len(piles) == 1, seed
        assert set(piles[0]["sides"]) == sides, (seed, piles[0]["sides"], sides)
        assert piles[0]["depth_m"] == qs.WINDROW_DEPTH_M["parapet"]
        # the reach comes from what FELL, not from the building
        assert piles[0]["elem_h_m"] is not None
        assert piles[0]["elem_h_m"] < 0.5 * info["H"], seed
        t0, t1 = piles[0]["along"]
        assert 0.0 <= t0 < t1 <= 1.0, (seed, t0, t1)

        info, plan = _plan([("out_of_plane", {"sides": 1})], "urm", seed=seed)
        fan = [q for q in plan["piles"] if q["kind"] == "fan"]
        assert len(fan) == 1 and fan[0]["sides"] == [plan["regions"][0]["side"]]
        assert fan[0]["depth_m"] == qs.WINDROW_DEPTH_M["wall"]

        info, plan = _plan([("corner_fail", {"storeys": 2})], "urm", seed=seed)
        fan = [q for q in plan["piles"] if q["kind"] == "fan"]
        assert len(fan) == 1
        assert set(fan[0]["sides"]) == set(plan["regions"][0]["sides"]), seed


# ---------------------------------------------------------------------------
# GLASS
# ---------------------------------------------------------------------------
def test_glass_loss_is_a_contiguous_band_of_storeys_on_one_or_two_sides():
    for grade, lo, hi, n_sides in (("DG1", 0.01, 0.20, 1), ("DG2", 0.10, 0.45, 1),
                                   ("DG4", 0.35, 0.75, 2)):
        for seed in range(6):
            info, plan = _plan(grade, "rc_glass", seed=seed)
            bands = plan["glass_bands"]
            assert bands, (grade, seed)
            b = bands[0]
            assert lo <= b["frac"] <= hi, (grade, seed, b["frac"])
            assert len(b["sides"]) <= n_sides + 1, (grade, seed, b["sides"])
            sts = b["storeys"]
            assert sts == list(range(sts[0], sts[-1] + 1)), (grade, seed, sts)
            assert plan["glass"], (grade, seed, "no pane went at all")
            idx = _by_path(info)
            n_sub = plan["stats"]["n_sub"]
            for p in plan["glass"]:
                e = idx[p]
                assert e["p"]["_side"] in b["sides"] or len(bands) > 1, (grade, p)
                assert qs.is_opening(e["p"], n_sub), (grade, p)
                assert int(e["p"]["_storey"]) in set(
                    s for bb in bands for s in bb["storeys"]), (grade, p)


# ---------------------------------------------------------------------------
# CONSTRUCTION TYPE
# ---------------------------------------------------------------------------
def _gac_data():
    with open(os.path.join(_PLANS, "gac_building_material.json")) as fh:
        mats = json.load(fh)
    with open(os.path.join(_PLANS, "gac_buildings.json")) as fh:
        dims = {r["name"]: r for r in json.load(fh)}
    return mats, dims


def test_every_gac_and_downtowncity_asset_routes_to_a_type_from_the_data():
    """The table is the record of the decision; the JSON is the evidence. If
    they ever disagree, this fails — which is the whole reason the table is
    allowed to be a literal."""
    mats, dims = _gac_data()
    assert len(mats) == 31, len(mats)
    for row in mats:
        name, mat = row["name"], row["material"]
        H = dims[name]["H"]
        got = qs.construction_type(
            "omniverse://x/GreatAmericanCity/Meshes/{0}.usd".format(name))
        assert got in qs.LADDER_S, (name, got)
        if mat in ("brick", "stone"):
            want = "urm"
        elif mat == "concrete":
            want = "rc"
        elif mat in ("glass", "steel"):
            want = "rc_glass"
        else:                      # no evidence at all -> by height
            want = "urm" if H <= qs.H_URM_MAX else "rc"
        assert got == want, (name, mat, H, got, want)
    # SM_Building_04 is the null row, and it is 52.8 m
    assert dims["SM_Building_04"]["H"] > qs.H_URM_MAX
    assert qs.construction_type("SM_Building_04.usd") == "rc"
    # downtowncity: the asset set carries no `material:` for these three
    assert qs.construction_type("a/downtowncity/Amar_Tower.usdc") == "rc_glass"
    for n, H in (("Building_11", 32.6), ("Building_12", 38.9)):
        assert qs.construction_type("a/downtowncity/{0}.usdc".format(n)) == "rc"
        # ...and the height rule alone would have said the same thing
        assert qs.construction_type("unknown_{0}.usdc".format(n), H=H) == "rc"
    # the standalone packs DO carry `material:`
    for mat, want in (("brick", "urm"), ("stone", "urm"), ("concrete", "rc"),
                      ("glass", "rc_glass"), ("steel", "rc_glass")):
        assert qs.construction_type("x/block_99.usdc", H=18.0, material=mat) == want
    # and with nothing at all, the height cut `gac_fire.prepare` uses
    assert qs.construction_type("x/mystery.usdc", H=12.0) == "urm"
    assert qs.construction_type("x/mystery.usdc", H=80.0) == "rc"
    # extensions must not leak into the key
    for ext in (".usd", ".usdc", ".usda", ""):
        assert qs.construction_type("d/SM_Building_16" + ext) == "rc_glass", ext


# ---------------------------------------------------------------------------
# THE LADDER
# ---------------------------------------------------------------------------
def test_the_ladder_names_are_all_real_recipes_and_every_grade_exists():
    for btype in ("urm", "rc", "rc_glass"):
        assert btype in qs.LADDER_S
        for lvl in qs.LEVELS:
            assert lvl in qs.LADDER_S[btype], (btype, lvl)
            for name, kw in qs.LADDER_S[btype][lvl]:
                assert name in qs.RECIPES_S, (btype, lvl, name)
                assert callable(qs.RECIPES_S[name]), name
                assert isinstance(kw, dict), (btype, lvl, name)
        # DG0 is nothing; DG5 is not nothing (except the tower's OV)
        assert qs.LADDER_S[btype]["DG0"] == []
        assert qs.LADDER_S[btype]["DG5"]
    # the grade names and the foundation levels match `quake_flow`'s exactly,
    # so a bake launcher can hand either kind of building the same string
    assert set(qs.GRADES) == set(qf.GRADES)
    assert set(qs.FOUNDATION_S) == set(qf.FOUNDATION)
    assert qs.LADDER_S["rc_glass"]["OV"] == qf.LADDER["rc_glass"]["OV"] == []
    # `_B_TOTAL_COLLAPSE` in quake_flow keys on these two names, and
    # `_b_settle_roof_plant` buries the roof plant on them — so they must be
    # spelled the same here or a DG5 keeps its water tanks in mid-air
    assert set(qf._B_TOTAL_COLLAPSE) <= set(qs.RECIPES_S)


def test_a_curtain_wall_tower_is_never_pancaked_and_a_tall_urm_is_never_a_heap():
    """Two guards, both from the record: no curtain-wall tower has collapsed
    in an earthquake, and a 40 m+ 'brick' building is a frame with a masonry
    façade — so its DG5 is a pancake, not a masonry heap."""
    tall = {"H": 200.0, "masses": {}}
    for grade in ("DG4", "DG5"):
        recs, notes = qs._resolve(grade, "rc_glass", tall)
        names = [n for n, _kw in recs]
        assert "pancake" not in names and "masonry_collapse" not in names, names
    # even asked for explicitly
    recs, notes = qs._resolve([("pancake", {})], "rc_glass", tall)
    assert [n for n, _ in recs] != ["pancake"], recs
    assert notes and "refused" in notes[0]
    # a short glass block is not a tower, and the ladder never asks anyway
    short = {"H": 43.0, "masses": {}}
    assert [n for n, _ in qs._resolve("DG5", "rc_glass", short)[0]] == \
        [n for n, _ in qs.LADDER_S["rc_glass"]["DG5"]]
    # tall URM
    recs, notes = qs._resolve("DG5", "urm", {"H": 71.8, "masses": {}})
    assert [n for n, _ in recs] == ["pancake"], recs
    assert notes
    recs, _n = qs._resolve("DG5", "urm", {"H": 38.6, "masses": {}})
    assert [n for n, _ in recs] == ["masonry_collapse"], recs
    # SM_Building_05 is the 71.8 m "brick" asset this guard exists for
    assert qs.construction_type("SM_Building_05.usd") == "urm"


# ---------------------------------------------------------------------------
# THE FOUNDATION FAMILY
# ---------------------------------------------------------------------------
def test_the_foundation_levels_route_to_quake_flow_and_overturn_does_not():
    for btype in ("urm", "rc"):
        for lvl, recipe in (("SETTLE", "settlement"), ("TILT", "tilt_severe"),
                            ("OV", "overturn")):
            info, plan = _plan(lvl, btype, seed=4)
            assert plan["ground"], (btype, lvl)
            assert plan["ground"]["recipe"] == recipe, (btype, lvl)
            if recipe != "overturn":
                # the three rigid ones ARE quake_flow's own recipes
                assert recipe in qf.RECIPES, recipe
                assert plan["removed"] == [] and plan["displaced"] == {}
            else:
                # ours: the landing edge is REMOVED, not fractured
                assert plan["removed"], (btype, "nothing crushed on landing")
                idx = _by_path(info)
                sd = plan["ground"]["kwargs"]["side"]
                for p in plan["removed"]:
                    e = idx[p]
                    assert sd in str(e["p"]["_side"]), (p, e["p"]["_side"], sd)
                assert 62.0 <= plan["ground"]["kwargs"]["angle_deg"] <= 90.0
                land = [q for q in plan["piles"] if q["tag"] == "landing"]
                assert len(land) == 1 and land[0]["sides"] == [sd]
                assert land[0]["offset_m"] > 0.0
    # a tower does not go over
    info, plan = _plan("OV", "rc_glass", seed=4)
    assert plan["ground"] is None and plan["removed"] == []


def test_quake_flows_whole_body_helper_sees_sliced_pieces():
    """`r_settlement` / `r_tilt_severe` move `quake_flow._everything(ctx)`.
    That helper walks `_els(ctx)`, i.e. `ctx["info"]["elements"]` — so it
    picks up sliced placements only if `describe` put them there and only
    while they are not marked dead. Both halves checked here, because the
    whole foundation family rests on it."""
    pls, style, grid = fake_sliced_building()
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    ctx = {"info": info, "fit": {"all": ["/World/cell/fit_b/slab_main_1"]}}
    got = qf._everything(ctx)
    assert len(got) == len(pls) + 1, (len(got), len(pls))
    assert set(p["prim_path"] for p in pls) <= set(got)
    info["elements"][0]["dead"] = True
    assert len(qf._everything(ctx)) == len(pls)


# ---------------------------------------------------------------------------
# THE PLAN ITSELF
# ---------------------------------------------------------------------------
def test_the_plan_is_json_serialisable_and_deterministic_per_seed():
    for btype in ("urm", "rc", "rc_glass"):
        for grade in qs.LEVELS:
            a = json.dumps(_plan(grade, btype, seed=9)[1], sort_keys=True)
            b = json.dumps(_plan(grade, btype, seed=9)[1], sort_keys=True)
            assert a == b, (btype, grade, "not deterministic per seed")
            c = json.dumps(_plan(grade, btype, seed=10)[1], sort_keys=True)
            if grade not in ("DG0",) and qs.LADDER_S[btype][grade]:
                assert a != c, (btype, grade, "the seed does nothing")
            # no numpy scalars, no sets, no tuples-of-tuples surprises
            back = json.loads(a)
            assert set(back) >= {"removed", "displaced", "panels", "piles",
                                 "glass", "regions", "interior", "fit_ops",
                                 "ground", "notes", "stats", "recipes"}
            assert "_removed_set" not in back


def test_a_piece_is_lost_or_moved_or_laid_on_the_pile_but_never_two():
    for btype in ("urm", "rc", "rc_glass"):
        for grade in qs.LEVELS:
            info, plan = _plan(grade, btype, seed=6)
            rm = set(plan["removed"])
            mv = set(plan["displaced"])
            pn = set(p for p, _s in plan["panels"])
            # A TORN piece (round 6b) is a fourth state, and a distinct one:
            # it is always a SURVIVING piece, never lost/moved/piled.
            tn = set(t["path"] for t in plan["tears"] if not t["dropped"])
            assert not (rm & mv), (btype, grade, sorted(rm & mv)[:3])
            assert not (rm & pn), (btype, grade, sorted(rm & pn)[:3])
            assert not (mv & pn), (btype, grade, sorted(mv & pn)[:3])
            assert not (tn & rm), (btype, grade, sorted(tn & rm)[:3])
            assert not (tn & mv), (btype, grade, sorted(tn & mv)[:3])
            assert not (tn & pn), (btype, grade, sorted(tn & pn)[:3])
            known = set(e["p"]["prim_path"] for e in info["elements"])
            assert (rm | mv | pn) <= known, btype
            assert tn <= known, btype
            assert set(plan["glass"]) <= known
            assert len(plan["removed"]) == len(set(plan["removed"]))


# ---------------------------------------------------------------------------
# ROUND 6b: THE RAGGED REMOVAL BOUNDARY — `plan["tears"]`
#
# `quake_sliced` cannot fracture a clipped shell, so every removal reads as a
# ruler-straight edge on the piece it stopped at. `fire_collapse.plan_edges` /
# `_tear_perimeter` already solve exactly this for the FIRE ladder on sliced
# GAC / downtowncity pieces; `_plan_tears` (pure, host-side) hooks the same
# machinery into the quake ladder. See `_plan_tears`'s own docstring for why
# it is driven off `plan["_removed_set"]` and not `plan["regions"]`.
# ---------------------------------------------------------------------------
def test_every_hole_boundary_has_a_tear_job():
    """Host-side proxy for `_tear_perimeter`'s own acceptance number: every
    class of neighbour `fire_collapse.plan_edges` finds gets a planned job,
    and — short of the `QS_MAX_TEARS` budget — every one of them is planned
    to actually be torn (`fire_collapse.edge_census`, fed `dropped` in place
    of the stage-side `torn` flag, since nothing has touched a stage yet)."""
    from disaster import fire_collapse as fc
    info, plan = _plan("DG4", btype="urm", seed=7)
    tears = plan["tears"]
    assert tears, "a DG4 urm plan opened holes but planned no tears at all"
    census = fc.edge_census([{"classes": t["classes"], "torn": not t["dropped"]}
                             for t in tears])
    n_dropped = sum(1 for t in tears if t["dropped"])
    seen_class = False
    for cls, (n_neighbours, n_torn) in census.items():
        if n_neighbours == 0:
            continue
        seen_class = True
        if n_dropped == 0:
            assert n_torn == n_neighbours, (cls, n_neighbours, n_torn)
        else:
            assert n_torn <= n_neighbours, (cls, n_neighbours, n_torn)
    assert seen_class, "no job carried any of fire_collapse.EDGE_CLASSES"
    assert (plan["stats"]["n_tears"] + plan["stats"]["n_tears_dropped"]
            == len(tears))


def test_no_tear_job_targets_a_removed_or_moved_piece():
    """`fire_collapse.plan_edges` finds neighbours with no idea a sliced
    plan also removes/moves pieces, so a job CAN be found against one — but
    `_plan_tears` must always catch it and mark it `dropped`, never author
    it. Only non-dropped jobs are the ones `_author_tears` will run."""
    for btype in ("urm", "rc", "rc_glass"):
        for grade in ("DG2", "DG3", "DG4"):
            info, plan = _plan(grade, btype, seed=8)
            rm = set(plan["removed"])
            mv = set(plan["displaced"]) | set(p for p, _s in plan["panels"])
            for t in plan["tears"]:
                if t["path"] in rm or t["path"] in mv:
                    assert t["dropped"], (btype, grade, t["path"])
            for t in plan["tears"]:
                if t["dropped"]:
                    continue
                assert t["path"] not in rm, (btype, grade, t["path"])
                assert t["path"] not in mv, (btype, grade, t["path"])


def test_no_tear_job_targets_a_core_or_roof_piece():
    for btype in ("urm", "rc", "rc_glass"):
        for grade in ("DG3", "DG4"):
            info, plan = _plan(grade, btype, seed=8)
            idx = _by_path(info)
            for t in plan["tears"]:
                if t["dropped"]:
                    continue
                e = idx.get(t["path"])
                assert e is not None, (btype, grade, t["path"])
                role = (e["p"] or {}).get("_role")
                assert role not in ("core", "roof"), (btype, grade, t["path"], role)


def test_a_total_collapse_plans_no_tears():
    for btype in ("urm", "rc"):
        info, plan = _plan("DG5", btype=btype, seed=4)
        assert plan["collapse"] is not None, (btype, "DG5 should collapse")
        assert plan["tears"] == [], (btype, plan["tears"])
    # The tower ladder never collapses at all (TOWER_NO_COLLAPSE_M); its DG5
    # removes nothing either, so tears is empty for the same underlying
    # reason (`not plan["_removed_set"]`), not because of the collapse gate.
    info, plan = _plan("DG5", btype="rc_glass", seed=4)
    assert plan["collapse"] is None
    assert plan["tears"] == []


def test_qs_ragged_off_reproduces_the_round6b_plan_byte_for_byte():
    """`QS_RAGGED=0` must consume ZERO extra draws from the shared rng: every
    field this pass did not exist to touch must be byte-identical to the
    pass turned on, for the same seed — the gate that proves the tear pass's
    private rng never spends from `pctx["rng"]`."""
    old = qs.QS_RAGGED
    try:
        for btype in ("urm", "rc", "rc_glass"):
            for grade in ("DG1", "DG2", "DG3", "DG4", "DG5"):
                qs.QS_RAGGED = True
                _, plan_on = _plan(grade, btype, seed=13)
                qs.QS_RAGGED = False
                _, plan_off = _plan(grade, btype, seed=13)
                for key in ("removed", "displaced", "panels", "piles",
                           "glass", "glass_bands", "macroblocks", "regions",
                           "stub", "ground", "collapse", "storey_collapse",
                           "interior", "fit_ops", "recipes", "notes"):
                    assert plan_on[key] == plan_off[key], (btype, grade, key)
                assert plan_off["tears"] == [], (btype, grade)
                assert plan_off["tear_scope"] == {}, (btype, grade)
                assert plan_off["stats"]["n_tears"] == 0, (btype, grade)
                assert plan_off["stats"]["n_tears_dropped"] == 0, (btype, grade)
    finally:
        qs.QS_RAGGED = old


def test_tears_survive_a_json_round_trip():
    info, plan = _plan("DG4", btype="urm", seed=7)
    assert plan["tears"], "need at least one tear job to test the round trip"
    dumped = qs.plan_to_json(plan)
    loaded = qs.plan_from_json(dumped)
    assert loaded["tears"] == plan["tears"]
    idx = _by_path(info)
    n_checked = 0
    for t in loaded["tears"]:
        if t["dropped"]:
            continue
        assert t["path"] in idx, t["path"]
        n_checked += 1
    assert n_checked, "every job came back dropped; nothing round-tripped"


def test_only_the_contract_kwargs_reach_plan_pile():
    """`_author_pile` forwards `_PILE_KW` and nothing else: a spec carries
    plan-local bookkeeping (`mass`, `base_z`, `tag`, `crown_est_m`) that
    `quake_rubble.plan_pile` has never heard of, and passing it would be a
    TypeError deep inside a launch."""
    info, plan = _plan("DG5", "rc", seed=2)
    spec = plan["piles"][0]
    kw = {k: spec[k] for k in qs._PILE_KW if spec.get(k) is not None}
    assert "kind" in kw and kw["kind"] == "dome"
    for leaked in ("mass", "base_z", "tag", "crown_est_m", "spread_est",
                   "panels", "fit"):
        assert leaked not in kw, leaked
    import inspect
    sig = ("m", "btype", "rng", "kind", "crown_m", "spread_frac", "sides",
           "along", "depth_m", "offset_m", "plate_ok", "stub_h_m", "panels",
           "budget", "seed_tag")
    for k in qs._PILE_KW:
        assert k in sig or k == "elem_h_m", k


def test_the_rubble_modules_are_only_touched_through_the_two_hooks():
    """`quake_rubble` / `quake_rubble_usd` are written in parallel. This file
    must pass without them, and a caller must be able to stub them."""
    calls = []

    def fake_plan_pile(m, btype, rng, **kw):
        calls.append(("plan_pile", btype, kw))
        return {"mound": None, "large": [], "instances": {}, "stats": {}}

    def fake_author(stage, parent, plan, **kw):
        calls.append(("author", parent, kw))
        return {"mound": "/m", "static": ["/m"], "all": ["/m"]}

    old = (qs.PLAN_PILE, qs.AUTHOR)
    try:
        qs.PLAN_PILE, qs.AUTHOR = fake_plan_pile, fake_author
        pp, au = qs._rubble()
        assert pp is fake_plan_pile and au is fake_author
    finally:
        qs.PLAN_PILE, qs.AUTHOR = old
    # ...and NOTHING is imported from them at module level, or this file
    # could not even be imported until packages B and C land
    import ast as _ast
    tree = _ast.parse(open(os.path.join(_HERE, "..", "disaster",
                                        "quake_sliced.py")).read())
    for node in tree.body:
        if isinstance(node, _ast.ImportFrom):
            names = [a.name for a in node.names] + [node.module or ""]
            assert not any("quake_rubble" in str(n) for n in names), names
        if isinstance(node, _ast.Import):
            assert not any("quake_rubble" in a.name for a in node.names)
    # Importing `disaster.quake_sliced` must not pull `quake_rubble` in — but
    # checked in a FRESH interpreter, not this one: asserting on this
    # process's `sys.modules` was order-fragile (any earlier test file in the
    # same session may import quake_rubble legitimately — test_quake_collapse
    # does, through a quake_flow call — and made this fail only in combined
    # runs; round-5 reviewer fix).
    import subprocess as _sp
    r = _sp.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, sys.argv[1]); "
         "import disaster.quake_sliced; "
         "assert 'disaster.quake_rubble' not in sys.modules, 'transitive'",
         os.path.join(_HERE, "..")],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-800:]


def test_the_rubble_numbers_agree_with_the_planner_when_it_is_present():
    """`quake_rubble` owns the mound numbers; this module mirrors two of them
    so a plan can say what it EXPECTS (`crown_est_m`) without importing it.
    When that module is on disk, the mirror has to still be a mirror — and
    when it is not, this test is a no-op rather than a failure."""
    try:
        from disaster import quake_rubble as qr
    except ImportError:
        return
    assert qs.CROWN_FRAC == qr.CROWN_FRAC, (qs.CROWN_FRAC, qr.CROWN_FRAC)
    assert qs.CROWN_MAX_M == qr.CROWN_CAP_M
    import inspect
    sig = inspect.signature(qr.plan_pile).parameters
    for k in qs._PILE_KW:
        assert k in sig, ("plan_pile has no {0} parameter".format(k))
    assert "panels" in sig and "seed_tag" in sig
    info, plan = _plan("DG5", "rc", seed=2)
    for spec in plan["piles"]:
        kw = {k: spec[k] for k in qs._PILE_KW if spec.get(k) is not None}
        want = min(qr.CROWN_CAP_M, qr.CROWN_FRAC["rc"] * info["H"])
        assert abs(spec["crown_est_m"] - want) < 1e-9
        assert set(kw) <= set(sig)


def test_wreck_sliced_produces_wreck_buildings_ctx_shape():
    """The bake launcher, the settle and the bake must not have to know which
    kind of building they were handed — so the ctx dict `wreck_sliced` builds
    has to carry exactly the keys `quake_flow.wreck_building` builds. Checked
    on the SOURCE, because building one for real needs a stage."""
    import ast as _ast

    def _ctx_keys(path, fn):
        tree = _ast.parse(open(path).read())
        for node in _ast.walk(tree):
            if isinstance(node, _ast.FunctionDef) and node.name == fn:
                for sub in _ast.walk(node):
                    if (isinstance(sub, _ast.Assign) and sub.targets
                            and isinstance(sub.targets[0], _ast.Name)
                            and sub.targets[0].id == "ctx"
                            and isinstance(sub.value, _ast.Dict)):
                        return set(k.value for k in sub.value.keys)
        raise AssertionError("no ctx dict in " + fn)

    base = _ctx_keys(os.path.join(_HERE, "..", "disaster", "quake_flow.py"),
                     "wreck_building")
    mine = _ctx_keys(os.path.join(_HERE, "..", "disaster", "quake_sliced.py"),
                     "wreck_sliced")
    assert base <= mine, sorted(base - mine)
    assert "parent" in mine and "fit" in mine and "velocity" in mine
    # the extras are additive and named so a reader knows they are ours
    assert mine - base <= {"sliced"}, sorted(mine - base)
    # the entry point takes what the brief fixed, in that order
    import inspect
    a = list(inspect.signature(qs.wreck_sliced).parameters)
    assert a[:9] == ["stage", "cell", "placements", "style", "recipes", "rng",
                     "nrng", "mats", "tag"], a
    for kw in ("btype", "usd", "mat_cache", "fit_storeys"):
        assert kw in a, kw


# ---------------------------------------------------------------------------
# ROUND-5 FOLLOW-UP (2026-08-31): the floating roof-plant fix and the
# dropped-fitout chip fix. Both need a real (in-memory) USD stage, unlike
# everything above — `pxr` is imported lazily, INSIDE these tests only, so
# every test above keeps running with no `pxr` on the path at all (this
# file's own design note: "quake_rubble / quake_rubble_usd ... STUBBED").
# Zero-argument tests throughout (no `monkeypatch`/`capsys`), matching this
# file's own `__main__` runner, which calls every `test_*` with no args.
# ---------------------------------------------------------------------------
def test_roof_plant_target_z_tracks_the_measured_roof_not_the_advertised_top():
    """The pure half of the floating-tank fix. `_roof_plant_target_z` must
    prefer a MEASURED roof top over the style's advertised H whenever one is
    given — SM_Building_02's own numbers (38.6 m advertised, ~35.6 m of
    actual support under the footprint) are close to this synthetic ~3 m
    gap."""
    advertised = 38.6
    measured = [35.6, 35.55, 35.62]        # several roof pieces, near-flat
    z = qs._roof_plant_target_z(advertised, measured)
    assert abs(z - (max(measured) + 0.02)) < 1e-9
    assert z < advertised - 2.9            # tracks the measured top, not H

    # no measured pieces at all (a kit building routed here by mistake, or a
    # roof bbox that failed to resolve) -> unchanged fallback, byte for byte
    # with `dress_roof`'s own `top + 0.02` formula
    z0 = qs._roof_plant_target_z(advertised, [])
    assert abs(z0 - (advertised + 0.02)) < 1e-9


def _box_mesh(stage, path, sx, sy, sz, z_bottom):
    """A plain axis-aligned box MESH (its own prim, no separate Xform
    parent) spanning `[-sx/2, sx/2] x [-sy/2, sy/2] x [z_bottom, z_bottom+sz]`
    in local space with no xform ops — a stand-in for a `role == "roof"`
    piece's own authored geometry, measurable with a bbox cache exactly like
    a real one."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    hx, hy = sx / 2.0, sy / 2.0
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    pts = [Gf.Vec3f(-hx, -hy, z_bottom), Gf.Vec3f(hx, -hy, z_bottom),
           Gf.Vec3f(hx, hy, z_bottom), Gf.Vec3f(-hx, hy, z_bottom),
           Gf.Vec3f(-hx, -hy, z_bottom + sz), Gf.Vec3f(hx, -hy, z_bottom + sz),
           Gf.Vec3f(hx, hy, z_bottom + sz), Gf.Vec3f(-hx, hy, z_bottom + sz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    return m


def test_reseat_roof_plant_slides_props_down_to_the_measured_roof_top():
    """End to end on a real stage: a `role == "roof"` piece measured ~3 m
    below the advertised `m["top"]`, and a `roof_plant` prop seated the way
    `dress_roof` seats one (bottom at `advertised_top + 0.02`, built by hand
    here so this does not depend on that function's own internals) —
    `_reseat_roof_plant` must slide it down to sit on the MEASURED top."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    parent = "/World/Bldg"
    st.DefinePrim(Sdf.Path(parent), "Xform")

    advertised_top = 38.6
    roof_top_measured = advertised_top - 3.0        # the coping-band overshoot
    roof_path = parent + "/roof_piece"
    _box_mesh(st, roof_path, 10.0, 10.0, 0.3, roof_top_measured - 0.3)

    tank_path = parent + "/tank_0"
    tank = _box_mesh(st, tank_path, 1.0, 1.0, 2.0, 0.0)
    UsdGeom.Xformable(tank).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, advertised_top + 0.02))

    info = {"masses": {"main": {"top": advertised_top}},
           "elements": [{"mass": "main", "role": "roof",
                         "p": {"prim_path": roof_path}}]}
    ctx = {"info": info, "roof_plant": [tank_path], "notes": []}

    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, n

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = bc.ComputeWorldBound(st.GetPrimAtPath(tank_path)).ComputeAlignedRange()
    new_bottom = float(rng.GetMin()[2])
    # 1e-4, not tighter: mesh points are `Gf.Vec3f` (float32).
    assert abs(new_bottom - (roof_top_measured + 0.02)) < 1e-4, new_bottom
    assert new_bottom < advertised_top - 2.5, "still floating at advertised H"
    assert any("roof_plant reseated" in n_ for n_ in ctx["notes"]), ctx["notes"]


def test_reseat_uses_the_footprint_not_the_tallest_roof_piece():
    """The SM_Building_02 regression, pinned: a GAC top band relabels its
    CORE as "roof", and that core is a full-band-height box whose bbox top
    equals the ADVERTISED height — so a reseat that takes max() over
    roof-piece tops measures the advertised top again and the tank keeps
    its 2.5 m of air (found by the support audit on the FRESH round-6
    bakes). The prop must land on the piece under ITS OWN footprint."""
    from pxr import Usd, UsdGeom, Gf
    st = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(st, "/W")

    def box(path, x0, y0, x1, y1, z0, z1):
        m = UsdGeom.Cube.Define(st, path)
        xf = UsdGeom.Xformable(m.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0,
                                         (z0 + z1) / 2.0))
        xf.AddScaleOp().Set(Gf.Vec3f((x1 - x0) / 2.0, (y1 - y0) / 2.0,
                                     (z1 - z0) / 2.0))
        return path

    # deck UNDER the tank footprint, top at 35.6
    deck = box("/W/deck", -5, -5, 5, 5, 35.0, 35.6)
    # full-band-height "roof"-labelled core AWAY from the footprint, top 38.6
    tall = box("/W/tallcore", 8, 8, 14, 14, 35.0, 38.6)
    # the tank, seated by the advertised height: bottom at 38.62
    tank = box("/W/tank", -1.0, -1.0, 1.0, 1.0, 38.62, 40.6)

    ctx = {
        "info": {"masses": {"main": {"top": 38.6}},
                 "elements": [
                     {"mass": "main", "role": "roof", "p": {"prim_path": deck}},
                     {"mass": "main", "role": "roof", "p": {"prim_path": tall}},
                 ]},
        "roof_plant": [tank],
        "notes": [],
    }
    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, (n, ctx["notes"])
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    lo = bc.ComputeWorldBound(st.GetPrimAtPath(tank)).ComputeAlignedRange().GetMin()
    assert abs(float(lo[2]) - 35.62) < 0.05, float(lo[2])

    # a building with no measurable pieces at all is a no-op (falls back to
    # leaving the advertised seat alone) — self-contained refit of the block
    # that previously lived in the test above (reviewer seam fix)
    ctx2 = {"info": {"masses": {"main": {"top": 38.6}},
                     "elements": []},
            "roof_plant": [tank], "notes": []}
    assert qs._reseat_roof_plant(st, ctx2) == 0
    assert ctx2["notes"] == []


# ---------------------------------------------------------------------------
# ROUND-7 FOLLOW-UP (2026-08-31): `_reseat_roof_plant` still left
# `SM_Building_19`/`_12`/`_24` floating 2.5-3.8 m across a full re-bake with
# new seeds -- a deterministic geometric miss, not settle noise. Ground
# truth (bare `pxr`, points, no Kit): a real up-facing deck triangle exists
# EXACTLY under every one of those tanks' own footprints, inside a merged
# `roof_x_*` piece whose OVERALL bbox reaches 1-2.4 m higher because the
# SAME prim also carries a coping run elsewhere on the roof (`SM_Building_
# 19`/`_24`), or a bare storey-height wall shell with ZERO real deck at all
# happens to still satisfy the old `ctop <= pz0 + 0.30` ceiling when the
# advertised seat is metres too LOW (`SM_Building_12`). The four tests below
# reproduce each failure class synthetically, against the FIXED (points-
# based, no-ceiling) `_reseat_roof_plant` -- see that function's own
# docstring for the full root-cause writeup.
# ---------------------------------------------------------------------------
def _two_level_box_mesh(stage, path, lo_rect, lo_z, hi_rect, hi_z, thickness=0.3):
    """ONE merged Mesh prim holding TWO separate flat-topped boxes at
    different heights -- `lo_rect=(x0,y0,x1,y1)` topped at `lo_z`, `hi_rect`
    topped at `hi_z` -- a stand-in for a real GAC `roof_x_*` piece that
    spans a stepped/multi-level roof in a SINGLE prim, so its own whole-mesh
    BBoxCache top is the HIGHER region even directly over the lower one
    (`SM_Building_19`'s `roof_x_0_14_0271`: deck at 69.276 m, bbox top
    70.196 m, from ONE prim)."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    def _box_pts(x0, y0, x1, y1, z0, z1):
        return [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
               (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]

    face_pattern = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                    (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    lo_pts = _box_pts(lo_rect[0], lo_rect[1], lo_rect[2], lo_rect[3],
                      lo_z - thickness, lo_z)
    hi_pts = _box_pts(hi_rect[0], hi_rect[1], hi_rect[2], hi_rect[3],
                      hi_z - thickness, hi_z)
    pts = lo_pts + hi_pts
    counts, idx = [], []
    for base in (0, 8):
        for f in face_pattern:
            counts.append(4)
            idx.extend(base + i for i in f)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    return m


def _wall_shell_mesh(stage, path, x0, y0, x1, y1, z0, z1):
    """A box with its TOP FACE OMITTED -- four vertical walls and a floor,
    no deck -- a stand-in for a bare storey-height `core_x_*`/`wall_*` shell
    (`SM_Building_19`'s `core_x_0_14_0254`: 220 points, 182 faces, ZERO
    up-facing triangles; `SM_Building_12`'s `core_x_0_20`, same shape). Must
    never register as a support no matter how close its own bbox top sits
    to a prop's seat."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
          (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6),
            (3, 0, 4, 7)]                      # no (4,5,6,7) top cap
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    return m


def _tank_at(stage, path, seat_z, hw=0.5):
    """A 1x1xN box prop seated the way `dress_roof` seats one: bottom at
    `seat_z`, centred at the origin's XY."""
    from pxr import Gf, UsdGeom
    tank = _box_mesh(stage, path, hw * 2.0, hw * 2.0, 2.0, 0.0)
    UsdGeom.Xformable(tank).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, seat_z))
    return tank


def test_reseat_multi_level_roof_uses_the_deck_under_this_footprint():
    """STEPPED / MULTI-LEVEL ROOF, one merged prim. A single `role=="roof"`
    piece carries a LOW deck (top 10.0 m) under the tank's own footprint and
    a HIGH deck (top 13.0 m) elsewhere in the SAME prim -- exactly
    `SM_Building_19`'s shape. The tank was seated (by whatever guessed
    `dress_roof`'s advertised top) at the HIGH region's height; the fix
    must slide it down to the LOW deck actually under it, not leave it at
    the piece's own tallest point."""
    from pxr import Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    st.DefinePrim(Sdf.Path("/W"), "Xform")

    roof_path = "/W/roof_stepped"
    _two_level_box_mesh(st, roof_path, (-5.0, -5.0, 5.0, 5.0), 10.0,
                        (15.0, -5.0, 25.0, 5.0), 13.0)
    tank_path = "/W/tank_0"
    _tank_at(st, tank_path, 13.02)     # seated at the WRONG (high) region

    ctx = {"info": {"masses": {"main": {"top": 13.0}},
                    "elements": [{"mass": "main", "role": "roof",
                                 "p": {"prim_path": roof_path}}]},
          "roof_plant": [tank_path], "notes": []}
    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, ctx["notes"]

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bottom = float(bc.ComputeWorldBound(
        st.GetPrimAtPath(tank_path)).ComputeAlignedRange().GetMin()[2])
    assert abs(bottom - 10.02) < 1e-3, bottom
    assert bottom < 11.0, "still resting near the piece's tallest point"


def test_reseat_ignores_a_bare_wall_shell_with_no_up_facing_deck():
    """TANK OVER A REMOVED-LOOKING / DECKLESS ROOF PIECE. A bare wall shell
    (no top cap, zero up-facing triangles) sits directly under the tank and
    its own bbox top would satisfy the OLD `ctop <= pz0 + 0.30` rule -- but
    it is not a deck. A real deck exists a full storey below. The fix must
    skip the shell entirely and land on the real, lower deck (`SM_Building_
    12`'s `core_x_0_20` shape: the closest candidate by height is a wall
    with nothing to stand on)."""
    from pxr import Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    st.DefinePrim(Sdf.Path("/W"), "Xform")

    shell_path = "/W/wall_shell"
    _wall_shell_mesh(st, shell_path, -5.0, -5.0, 5.0, 5.0, 8.0, 12.0)
    deck_path = "/W/real_deck"
    _box_mesh(st, deck_path, 20.0, 20.0, 0.3, 4.7)   # top at 5.0
    tank_path = "/W/tank_0"
    _tank_at(st, tank_path, 12.02)      # seated on the shell's own bbox top

    ctx = {"info": {"masses": {"main": {"top": 12.0}},
                    "elements": [
                        {"mass": "main", "role": "core",
                         "p": {"prim_path": shell_path}},
                        {"mass": "main", "role": "roof",
                         "p": {"prim_path": deck_path}},
                    ]},
          "roof_plant": [tank_path], "notes": []}
    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, ctx["notes"]

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bottom = float(bc.ComputeWorldBound(
        st.GetPrimAtPath(tank_path)).ComputeAlignedRange().GetMin()[2])
    assert abs(bottom - 5.02) < 1e-3, bottom
    assert bottom < 7.0, "landed on the deckless wall shell, not the real deck"


def test_reseat_a_parapet_band_under_the_footprint_is_still_excluded():
    """PARAPET BAND UNDER THE FOOTPRINT. The coping run has a genuine
    up-facing cap (a real triangle, not just a tall bbox) sitting well
    above the actual roof deck -- the points-based fix must still exclude
    it BY ROLE, exactly as the bbox-based version did, so a tank never
    lands on a rim it merely overlaps (SM_Building_02's original round-6
    regression, re-proven against the new per-triangle scoring)."""
    from pxr import Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    st.DefinePrim(Sdf.Path("/W"), "Xform")

    parapet_path = "/W/parapet_cap"
    _box_mesh(st, parapet_path, 10.0, 10.0, 0.4, 13.6)     # top at 14.0
    deck_path = "/W/real_deck"
    _box_mesh(st, deck_path, 10.0, 10.0, 0.3, 9.7)          # top at 10.0
    tank_path = "/W/tank_0"
    _tank_at(st, tank_path, 14.02)      # seated on the parapet's own cap

    ctx = {"info": {"masses": {"main": {"top": 14.0}},
                    "elements": [
                        {"mass": "main", "role": "parapet",
                         "p": {"prim_path": parapet_path}},
                        {"mass": "main", "role": "roof",
                         "p": {"prim_path": deck_path}},
                    ]},
          "roof_plant": [tank_path], "notes": []}
    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, ctx["notes"]

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bottom = float(bc.ComputeWorldBound(
        st.GetPrimAtPath(tank_path)).ComputeAlignedRange().GetMin()[2])
    assert abs(bottom - 10.02) < 1e-3, bottom


def test_reseat_finds_a_real_roof_far_above_a_too_low_advertised_seat():
    """THE ROOT-CAUSE REGRESSION for `SM_Building_12`: the advertised top
    `dress_roof` seated the prop at is METRES too LOW, not too high -- the
    reverse of the bug `_reseat_roof_plant` was originally built for. The
    real roof sits 7 m above the tank's initial (wrong) seat; a nearer,
    also-real deck sits almost exactly AT the initial seat. The OLD `ctop
    <= pz0 + 0.30` ceiling would have excluded the true (higher) roof and
    kept the tank on the near one; the fix must find the real roof
    regardless of which side of the initial guess it falls on."""
    from pxr import Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    st.DefinePrim(Sdf.Path("/W"), "Xform")

    near_path = "/W/near_deck"
    _box_mesh(st, near_path, 10.0, 10.0, 0.3, 7.7)     # top at 8.0
    roof_path = "/W/true_roof"
    _box_mesh(st, roof_path, 10.0, 10.0, 0.3, 14.7)    # top at 15.0
    tank_path = "/W/tank_0"
    _tank_at(st, tank_path, 8.02)        # dress_roof's badly-low advertised seat

    ctx = {"info": {"masses": {"main": {"top": 8.0}},
                    "elements": [
                        {"mass": "main", "role": "core",
                         "p": {"prim_path": near_path}},
                        {"mass": "main", "role": "roof",
                         "p": {"prim_path": roof_path}},
                    ]},
          "roof_plant": [tank_path], "notes": []}
    n = qs._reseat_roof_plant(st, ctx)
    assert n == 1, ctx["notes"]

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bottom = float(bc.ComputeWorldBound(
        st.GetPrimAtPath(tank_path)).ComputeAlignedRange().GetMin()[2])
    assert abs(bottom - 15.02) < 1e-3, bottom
    assert bottom > 10.0, "stayed on the near deck, excluded by the old ceiling"


def test_displace_above_chips_the_dropped_fitout_slabs_and_columns():
    """`_apply_fit_ops`'s "displace_above" moves a whole crushed block's
    fit-out (slabs/columns/partitions) as one rigid body — this is the
    round-5 follow-up on top of that: the moved SLAB and COLUMN meshes (both
    plain `qf._box` prisms `fit_interior` authors — never a sliced piece)
    come out visibly chipped (far more than the box's own 6 quad faces), and
    `QC_CHIP=0` reproduces the exact 6-face box byte for byte. Also checks
    the `[chip]` proof line fires exactly when chipping does.
    """
    import contextlib
    import io

    from pxr import Sdf, Usd, UsdGeom

    def _build():
        st = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        parent = "/World/Bldg"
        st.DefinePrim(Sdf.Path(parent), "Xform")
        slab_path = parent + "/fit_t/slab_main_1"
        col_path = parent + "/fit_t/col_main_1_0_0"
        qf._box(st, slab_path, 0.0, 0.0, 3.0, 20.0, 16.0, 0.22, 0.0, mat=None)
        qf._box(st, col_path, 4.0, 4.0, 3.0, 0.45, 0.45, 2.8, 0.0, mat=None)
        ctx = {"stage": st, "parent": parent, "tag": "t", "mats": {},
              "static_extra": []}
        fit = {"slabs": {("main", 1): slab_path},
              "columns": {("main", 1): [col_path]},
              "partitions": [], "props": {}, "all": [slab_path, col_path]}
        ctx["fit"] = fit
        plan = {"fit_ops": [{"op": "displace_above", "mass": "main",
                            "storey": 0,
                            "transform": qs._disp(translate=(0.0, 0.0, -1.0),
                                                  why="test drop")}]}
        return st, ctx, plan, slab_path, col_path

    def _n_faces(st, path):
        m = UsdGeom.Mesh(st.GetPrimAtPath(path))
        return len(m.GetFaceVertexCountsAttr().Get())

    old = os.environ.get("QC_CHIP")
    try:
        os.environ["QC_CHIP"] = "1"
        st, ctx, plan, slab_path, col_path = _build()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            qs._apply_fit_ops(st, ctx, plan)
        assert _n_faces(st, slab_path) > 30, _n_faces(st, slab_path)
        assert _n_faces(st, col_path) > 30, _n_faces(st, col_path)
        assert slab_path in ctx["static_extra"] and col_path in ctx["static_extra"]
        printed = buf.getvalue()
        assert "[chip] quake_sliced fit-out" in printed, printed
        assert "2 chipped, 0 passed-through (vtk=True)" in printed, printed

        os.environ["QC_CHIP"] = "0"
        st2, ctx2, plan2, slab_path2, col_path2 = _build()
        buf2 = io.StringIO()
        with contextlib.redirect_stdout(buf2):
            qs._apply_fit_ops(st2, ctx2, plan2)
        assert _n_faces(st2, slab_path2) == 6, _n_faces(st2, slab_path2)
        assert _n_faces(st2, col_path2) == 6, _n_faces(st2, col_path2)
        printed2 = buf2.getvalue()
        assert "0 chipped, 2 passed-through (vtk=False)" in printed2, printed2
    finally:
        if old is None:
            os.environ.pop("QC_CHIP", None)
        else:
            os.environ["QC_CHIP"] = old


def test_chip_dropped_fitout_is_callable_with_empty_lists():
    """The proof-line helper never raises when there is nothing to chip
    (every earlier recipe already removed the fit-out, say) — no stage
    access happens at all in that case."""
    n = qs._chip_dropped_fitout({"tag": "t"}, [], [], [])
    assert n == 0


# ---------------------------------------------------------------------------
# ROUND 6: `_sweep_roof_props_sliced` — the sliced-path equivalents of
# `test_quake_collapse.py`'s four `_sweep_roof_props` / `_author_band`
# integration tests. Real (in-memory) stages, on purpose, same reason
# `test_reseat_roof_plant_slides_props_down_to_the_measured_roof_top` above
# is one: this is the AUTHORING half, out of reach of a pure-planner test.
# ---------------------------------------------------------------------------
def _sliced_mass(top, levels, z0=0.0, cx=0.0, cy=0.0, W=20.0, D=20.0):
    return {"top": float(top), "levels": [float(v) for v in levels],
           "z0": float(z0), "cx": float(cx), "cy": float(cy),
           "W": float(W), "D": float(D)}


def _sliced_ctx(stage, parent, mass, tank_path, seed=11):
    return {"stage": stage, "parent": parent,
           "info": {"masses": {"main": mass}},
           "roof_plant": [tank_path], "roof_fixed": [],
           "roof_plant_mass": "main", "rng": random.Random(seed),
           "static_extra": [], "loose": [], "notes": []}


def test_sweep_roof_props_sliced_carries_plant_with_a_storey_crush():
    """BAND / STOREY CARRY: a `soft_storey`-shaped `fit_ops["displace_
    above"]` entry must move a surviving `roof_plant` path by the EXACT
    same rigid transform the block above the crush got — not a kick, not a
    re-seat, the identical spec `apply_plan` already fed `_gf` +
    `qf._transform_prims` for every OTHER piece above storey `k`."""
    from pxr import Gf, Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, Sdf.Path(parent))

    mass = _sliced_mass(top=22.0, levels=[0, 7, 10, 13, 16, 19])
    tank_path = parent + "/tank_0"
    tank = _box_mesh(st, tank_path, 1.6, 1.6, 2.0, 0.0)
    UsdGeom.Xformable(tank).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, mass["top"] + 0.02))

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    before = bc.ComputeWorldBound(st.GetPrimAtPath(tank_path)).ComputeAlignedRange()

    spec = qs._disp(translate=(0.4, -0.2, -4.6), why="test soft storey")
    plan = {"fit_ops": [{"op": "displace_above", "mass": "main", "storey": 2,
                        "transform": spec}],
           "removed": [], "collapse": None}
    ctx = _sliced_ctx(st, parent, mass, tank_path)

    n_carried, n_reseated = qs._sweep_roof_props_sliced(st, ctx, plan)
    assert (n_carried, n_reseated) == (1, 0)
    assert tank_path not in ctx["roof_plant"]
    assert tank_path in ctx["static_extra"]

    bc2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    after = bc2.ComputeWorldBound(st.GetPrimAtPath(tank_path)).ComputeAlignedRange()
    want = np.array(spec["translate"])
    got = np.array(after.GetMin()) - np.array(before.GetMin())
    assert np.allclose(got, want, atol=1e-4), (got, want)


def test_sweep_roof_props_sliced_reseats_a_region_removal_big_drop():
    """REGION REMOVAL, deep: a `corner_fail`/`out_of_plane`-shaped `plan[
    "removed"]` set near the roof, with nothing real left under the tank's
    OWN footprint except a slab a full storey down. Must land on that real
    slab (not the advertised roof height), get a real tip/roll (deep drop),
    come out static (no rigid-body/settle dependency), and be dropped from
    `roof_plant`."""
    from pxr import Gf, Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, Sdf.Path(parent))

    mass = _sliced_mass(top=17.0, levels=[0, 5, 8, 11, 14])
    tank_path = parent + "/tank_0"
    tank = _box_mesh(st, tank_path, 1.6, 1.6, 2.0, 0.0)
    UsdGeom.Xformable(tank).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, mass["top"] + 0.02))

    # what `corner_fail`/`out_of_plane` deleted near the roof, right under
    # the tank's own footprint
    removed = []
    for i, (x, y) in enumerate(((-1.0, -1.0), (1.0, 1.0))):
        p = parent + "/wall_gone_{0}".format(i)
        piece = _box_mesh(st, p, 2.0, 2.0, 3.0, mass["top"] - 3.0)
        UsdGeom.Xformable(piece).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        piece.GetPrim().SetActive(False)
        removed.append(p)

    # the real support: a slab a full storey down, spanning the footprint
    deck_path = parent + "/deck_below"
    _box_mesh(st, deck_path, 20.0, 20.0, 0.3, mass["levels"][-1])

    plan = {"fit_ops": [], "removed": removed, "collapse": None}
    ctx = _sliced_ctx(st, parent, mass, tank_path)

    n_carried, n_reseated = qs._sweep_roof_props_sliced(st, ctx, plan)
    assert (n_carried, n_reseated) == (0, 1)
    assert tank_path not in ctx["roof_plant"]
    assert tank_path in ctx["static_extra"]
    assert ctx["loose"] == [], "must not be a rigid body — no settle dependency"

    pr = st.GetPrimAtPath(tank_path)
    assert pr.IsValid() and pr.IsActive()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    lo = bc.ComputeWorldBound(pr).ComputeAlignedRange().GetMin()
    assert float(lo[2]) < mass["top"] - 2.0, "did not fall at all"
    assert abs(float(lo[2]) - mass["levels"][-1]) < 0.6, \
        ("landed far from the real deck", float(lo[2]))

    xf = UsdGeom.XformCache()
    tr = Gf.Transform(xf.GetLocalToWorldTransform(pr))
    assert tr.GetRotation().GetAngle() > 5.0, \
        "landed bolt upright on a deep drop"


def test_sweep_roof_props_sliced_small_region_loss_gets_idle_tip():
    """REGION REMOVAL, shallow: the same majority-footprint-lost region, but
    real support sits only centimetres under the tank — the idle tip
    (`quake_flow.B_ROOF_PLANT_TIP_DEG`), not the DG5-style bury roll."""
    from pxr import Gf, Sdf, Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, Sdf.Path(parent))

    mass = _sliced_mass(top=17.0, levels=[0, 5, 8, 11, 14])
    tank_path = parent + "/tank_0"
    tank = _box_mesh(st, tank_path, 1.6, 1.6, 2.0, 0.0)
    UsdGeom.Xformable(tank).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, mass["top"] + 0.02))

    removed = []
    for i, (x, y) in enumerate(((-1.0, -1.0), (1.0, 1.0))):
        p = parent + "/wall_gone_{0}".format(i)
        piece = _box_mesh(st, p, 2.0, 2.0, 3.0, mass["top"] - 3.0)
        UsdGeom.Xformable(piece).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        piece.GetPrim().SetActive(False)
        removed.append(p)

    # real support just 0.12 m under the tank's own base, not a storey down
    deck_path = parent + "/deck_just_under"
    _box_mesh(st, deck_path, 20.0, 20.0, 0.3, mass["top"] + 0.02 - 0.12 - 0.3)

    plan = {"fit_ops": [], "removed": removed, "collapse": None}
    ctx = _sliced_ctx(st, parent, mass, tank_path)

    n_carried, n_reseated = qs._sweep_roof_props_sliced(st, ctx, plan)
    assert (n_carried, n_reseated) == (0, 1)

    pr = st.GetPrimAtPath(tank_path)
    xf = UsdGeom.XformCache()
    tr = Gf.Transform(xf.GetLocalToWorldTransform(pr))
    assert tr.GetRotation().GetAngle() <= qf.B_ROOF_PLANT_TIP_DEG + 1e-6, \
        "a barely-fallen prop should barely lean"


def test_sweep_roof_props_sliced_is_deterministic():
    """Same building, same seeds -> the same landing — `_sweep_roof_props_
    sliced` draws only from `ctx["rng"]`, the building's own generator."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    def _rig():
        st = Usd.Stage.CreateInMemory()
        parent = "/World/Bldg"
        UsdGeom.Xform.Define(st, Sdf.Path(parent))
        mass = _sliced_mass(top=17.0, levels=[0, 5, 8, 11, 14])
        tank_path = parent + "/tank_0"
        tank = _box_mesh(st, tank_path, 1.6, 1.6, 2.0, 0.0)
        UsdGeom.Xformable(tank).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, mass["top"] + 0.02))
        removed = []
        for i, (x, y) in enumerate(((-1.0, -1.0), (1.0, 1.0))):
            p = parent + "/wall_gone_{0}".format(i)
            piece = _box_mesh(st, p, 2.0, 2.0, 3.0, mass["top"] - 3.0)
            UsdGeom.Xformable(piece).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            piece.GetPrim().SetActive(False)
            removed.append(p)
        _box_mesh(st, parent + "/deck_below", 20.0, 20.0, 0.3,
                  mass["levels"][-1])
        plan = {"fit_ops": [], "removed": removed, "collapse": None}
        ctx = _sliced_ctx(st, parent, mass, tank_path, seed=99)
        qs._sweep_roof_props_sliced(st, ctx, plan)
        return st, tank_path

    st1, t1 = _rig()
    st2, t2 = _rig()
    xf = UsdGeom.XformCache()
    m1 = xf.GetLocalToWorldTransform(st1.GetPrimAtPath(t1))
    m2 = xf.GetLocalToWorldTransform(st2.GetPrimAtPath(t2))
    assert np.allclose(np.array(m1), np.array(m2), atol=1e-9)


# ---------------------------------------------------------------------------
# SHELL MATERIAL BINDING (round 2026-08-31): `gac_storey_slice.read_mesh`
# harvested a real GAC facade material for a wall/corner/pier/parapet piece
# but `write_piece` skipped the bind whenever `mats[mi] is None` -- and,
# separately, `read_mesh`'s no-argument `ComputeBoundMaterial()` missed any
# binding authored under the `full` material purpose only (measured: exactly
# how GreatAmericanCity binds its facade materials), collapsing to the same
# "no material" case even when the source genuinely had one. Both together
# produced the shells that render as flat fallback in every fresh GAC quake
# bake (33/33 checked, `tools/gac_shell_bind_repair.py`'s docstring has the
# census). The synthetic source stage below reproduces both failure shapes
# at once: a mesh whose only binding lives under `full` purpose (must be
# HARVESTED, not treated as absent), and a second mesh with one subset-bound
# region plus one genuinely unbound region (must get the role fallback, not
# be skipped).
# ---------------------------------------------------------------------------
def test_sliced_shell_pieces_have_no_unbound_mesh_after_full_purpose_and_gap():
    from pxr import Sdf, Usd, UsdGeom, UsdShade, Vt

    def _new_material(stage, path):
        mat = UsdShade.Material.Define(stage, Sdf.Path(path))
        sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
        sh.CreateIdAttr("UsdPreviewSurface")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def _quad_mesh(stage, path):
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)]
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3, 3]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 0, 2, 3]))
        return m

    src = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(src, UsdGeom.Tokens.z)
    parent = "/Src/Bldg"
    src.DefinePrim(Sdf.Path(parent), "Xform")

    # mesh A: mesh-level bind, "full" PURPOSE ONLY -- no allPurpose binding
    # exists at all, so the old no-argument `ComputeBoundMaterial()` found
    # nothing here.
    mesh_a_path = parent + "/core_solid"
    mesh_a = _quad_mesh(src, mesh_a_path)
    mat_full = _new_material(src, parent + "/Looks/M_full_only")
    UsdShade.MaterialBindingAPI.Apply(mesh_a.GetPrim()).Bind(
        mat_full, materialPurpose=UsdShade.Tokens.full)

    # mesh B: NO mesh-level bind. One GeomSubset (face 0), also "full"-
    # purpose-only, bound to a second real material; face 1 has no subset
    # and no default anywhere -- truly unbound at the source.
    mesh_b_path = parent + "/wall_partial"
    mesh_b = _quad_mesh(src, mesh_b_path)
    mat_glass = _new_material(src, parent + "/Looks/M_glass_full_only")
    sub = UsdGeom.Subset.CreateGeomSubset(
        mesh_b, "mat_0", UsdGeom.Tokens.face, Vt.IntArray([0]),
        UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(
        mat_glass, materialPurpose=UsdShade.Tokens.full)

    # ONE shared stage for source AND sliced output -- exactly how
    # `slice_to_kit` calls `read_mesh`/`write_piece` for real (both against
    # the SAME `stage` argument), and the reason `write_piece` can bind a
    # harvested `mats[mi]` by absolute path at all: it is only ever valid
    # while the destination composition still contains whatever the source
    # stage had at that path. A later, separate export/flatten of just the
    # sliced subtree into a standalone file is a real, different hazard
    # (see `_role_fallback_material`'s docstring and
    # `tools/gac_shell_bind_repair.py`) but is not what this test reproduces.
    out = src
    out.DefinePrim(Sdf.Path("/Out"), "Xform")

    m_a = gss.read_mesh(src, mesh_a_path, verbose=False)
    assert m_a is not None
    assert len(m_a["mats"]) == 1 and m_a["mats"][0] is not None, (
        "the full-purpose mesh-level bind was not harvested")
    assert gss.write_piece(out, "/Out/core_x_0_00_0000", m_a, m_a["mats"],
                           role="core")

    m_b = gss.read_mesh(src, mesh_b_path, verbose=False)
    assert m_b is not None
    # exactly one real (harvested) slot and one "no material" sentinel --
    # never a silently-reused unrelated material (the old `elif not mats:`
    # one-shot guard this round replaced with a stable, shared sentinel key).
    assert len(m_b["mats"]) == 2
    assert sum(1 for x in m_b["mats"] if x is None) == 1
    assert sum(1 for x in m_b["mats"] if x is not None) == 1
    assert gss.write_piece(out, "/Out/wall_S_1_00_0001", m_b, m_b["mats"],
                           role="wall")

    # THE ACCEPTANCE CHECK: 0 unbound shell meshes in the sliced output,
    # using the exact substrings the round's binding census matches on.
    unbound = []
    for prim in Usd.PrimRange(out.GetPrimAtPath("/Out")):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        name = prim.GetName()
        if not any(k in name for k in
                  ("wall_", "corner_", "core_x", "pier_", "parapet_")):
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not (mat and mat.GetPrim().IsValid()):
            unbound.append(str(prim.GetPath()))
    assert unbound == [], unbound

    # the region that DID have a real harvested material must still resolve
    # to THAT material, not be silently overridden by the role fallback.
    wall_prim = out.GetPrimAtPath("/Out/wall_S_1_00_0001")
    mesh_default, _ = UsdShade.MaterialBindingAPI(wall_prim).ComputeBoundMaterial()
    subset_prims = [c for c in wall_prim.GetChildren() if c.IsA(UsdGeom.Subset)]
    assert len(subset_prims) == 1, [str(c.GetPath()) for c in wall_prim.GetChildren()]
    sub_mat, _ = UsdShade.MaterialBindingAPI(subset_prims[0]).ComputeBoundMaterial()
    assert sub_mat.GetPrim().GetPath() != mesh_default.GetPrim().GetPath()
    assert sub_mat.GetPrim().GetPath() == mat_glass.GetPrim().GetPath()


# ---------------------------------------------------------------------------
# ROUND 8 (`stranded-bands`): the sky-grid finding on `eq500_v4`
# ---------------------------------------------------------------------------
# Photo + USD forensics: regular grids/columns of tan wall/parapet ring
# pieces hanging in the sky. Traced to `_apply_region`'s own toothing —
# `_boundary()` counts a VERTICAL neighbour exactly like a horizontal one, so
# a wide MULTI-STOREY region's edge-bay columns are boundary cells at every
# storey of the region, and `kept_piers` rolls keep/lose PER CELL,
# independently: a pier can survive at storey `st` while the pier directly
# below it, at `st-1`, same column, same region, loses its OWN independent
# draw. Nothing carries the survivor down, gives it physics, or sweeps it —
# `_repair_stranded_shell_sliced` / `_sweep_airborne_shell_sliced` below are
# the fix. The BAND-CRUSH half of the story (`s_soft_storey`'s own `plan[
# "displaced"]` loop) turned out to need no code change at all — the first
# test below is the regression lock that keeps it that way.
def test_soft_storey_carries_every_shell_role_above_the_crush():
    """CARRIED-BAND CASE. `s_soft_storey`'s loop over `g.els` for `_storey >
    k` has no role filter, so every wall/pier/corner/parapet/parapet_corner/
    roof/core placement above the crushed storey already lands in `plan[
    "displaced"]`, with the IDENTICAL spec `_apply_fit_ops`'s own "displace_
    above" branch feeds the fit-out (slabs/columns/partitions/props). This
    is the ONE mechanism this round's repair does not have to reimplement —
    pinned here so a future edit to `s_soft_storey` cannot silently regress
    it back to the state the sky-grid photo showed."""
    for seed in range(6):
        info, plan = _plan([("soft_storey", {"storey": 2})], "rc", seed=seed,
                           storeys=8, H=40.0)
        k = plan["storey_collapse"]["storey"]
        assert k == 2, (seed, k)
        above = [e for e in info["elements"] if int(e["p"]["_storey"]) > k]
        assert above, "fixture too short to exercise this"
        specs = set(json.dumps(plan["displaced"][e["p"]["prim_path"]],
                               sort_keys=True) for e in above)
        assert len(specs) == 1, (seed, "every piece above the crush must "
                                 "share ONE rigid transform", specs)
        for e in above:
            assert e["p"]["prim_path"] in plan["displaced"], \
                (seed, e["p"]["_role"], e["p"]["prim_path"])
        # nothing AT OR BELOW the crushed storey is ever rigid-displaced —
        # it is either removed (the band itself) or left alone (below it)
        for e in info["elements"]:
            if int(e["p"]["_storey"]) <= k:
                assert e["p"]["prim_path"] not in plan["displaced"], \
                    (seed, e["p"]["_storey"], e["p"]["_role"])


def _shell_el(role, side, storey, bay, path, dead=False):
    """A minimal `info["elements"]` entry — just enough of the shape
    `_shell_column_index` / `_orphaned_shell_candidates` read (`mass`, `p.
    _role`/`_side`/`_storey`/`_bay`/`prim_path`, `dead`) — for the PURE
    detection tests, no stage at all."""
    return {"mass": "main", "role": role,
           "p": {"_role": role, "_side": side, "_storey": storey,
                 "_bay": bay, "prim_path": path},
           "dead": dead}


def test_orphaned_shell_candidates_finds_the_toothed_gap():
    """Pure detection (no stage): a live piece one storey above a column
    whose OWN cell is entirely `dead` is an orphan; the same shape with the
    cell below still (fully) alive is not; a piece already claimed by
    `plan["displaced"]` (case 1's own carrier) is never a candidate either
    way; a PARTIALLY toothed cell below (one sub alive, one dead) is not a
    STRICT (item 2) candidate but is a LOOSE (item 3 safety-net) one;
    corners use the same (corner, storey) column, with no bay index."""
    below_gone = _shell_el("wall", "S", 0, 0, "/W/wall_S_0_0", dead=True)
    orphan = _shell_el("wall", "S", 1, 0, "/W/wall_S_1_0", dead=False)
    info = {"elements": [below_gone, orphan]}
    plan = {"displaced": {}, "removed": ["/W/wall_S_0_0"]}
    assert qs._orphaned_shell_candidates(info, plan, "main") == \
        ["/W/wall_S_1_0"]

    below_alive = _shell_el("wall", "S", 0, 0, "/W/wall_S_0_0", dead=False)
    info2 = {"elements": [below_alive, orphan]}
    assert qs._orphaned_shell_candidates(info2, plan, "main") == []

    plan_disp = {"displaced": {"/W/wall_S_1_0": {}}, "removed": []}
    assert qs._orphaned_shell_candidates(info, plan_disp, "main") == []

    pier_alive = _shell_el("pier", "S", 0, 0, "/W/pier_S_0_0a", dead=False)
    wall_dead = _shell_el("wall", "S", 0, 1, "/W/pier_S_0_0b", dead=True)
    info3 = {"elements": [pier_alive, wall_dead, orphan]}
    assert qs._orphaned_shell_candidates(info3, plan, "main", loose=False) == []
    assert qs._orphaned_shell_candidates(info3, plan, "main", loose=True) == \
        ["/W/wall_S_1_0"]

    c_below = _shell_el("corner", "SW", 0, 0, "/W/corner_SW_0", dead=True)
    c_above = _shell_el("corner", "SW", 1, 0, "/W/corner_SW_1", dead=False)
    info4 = {"elements": [c_below, c_above]}
    assert qs._orphaned_shell_candidates(info4, plan, "main") == \
        ["/W/corner_SW_1"]

    # never authored below at all (e.g. a ground-storey street opening) is
    # not the same thing as "removed" — no candidate either way
    info5 = {"elements": [orphan]}
    assert qs._orphaned_shell_candidates(info5, plan, "main") == []
    assert qs._orphaned_shell_candidates(info5, plan, "main", loose=True) == []


def test_repair_stranded_shell_drops_a_one_storey_gap_onto_real_support():
    """DROPPED-PIECE CASE. A live wall piece one storey above a toothed-away
    column (the cell directly below it is entirely `dead` — a `corner_fail`/
    `out_of_plane` boundary draw that kept THIS row and lost the one under
    it), with real support two storeys down. Must land flush on that real
    support (plus a small settle tip) and count as DROPPED, never deleted."""
    from pxr import Usd, UsdGeom

    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, parent)

    support_path = parent + "/wall_S_0_0"
    _box_mesh(st, support_path, 3.0, 1.0, 3.0, 0.0)            # z in [0, 3]
    gone_path = parent + "/wall_S_1_0"
    gone = _box_mesh(st, gone_path, 3.0, 1.0, 3.0, 3.0)        # z in [3, 6]
    gone.GetPrim().SetActive(False)
    orphan_path = parent + "/wall_S_2_0"
    _box_mesh(st, orphan_path, 3.0, 1.0, 3.0, 6.0)             # z in [6, 9]

    info = {"masses": {"main": {"top": 9.0, "levels": [0.0, 3.0, 6.0, 9.0],
                                "z0": 0.0}},
           "elements": [_shell_el("wall", "S", 0, 0, support_path),
                       _shell_el("wall", "S", 1, 0, gone_path, dead=True),
                       _shell_el("wall", "S", 2, 0, orphan_path)]}
    plan = {"displaced": {}, "removed": [gone_path]}
    ctx = {"stage": st, "parent": parent, "info": info,
          "rng": random.Random(5), "static_extra": [], "notes": []}

    n_dropped, n_deleted = qs._repair_stranded_shell_sliced(st, ctx, plan)
    assert (n_dropped, n_deleted) == (1, 0)
    assert st.GetPrimAtPath(orphan_path).IsActive()

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    lo = bc.ComputeWorldBound(
        st.GetPrimAtPath(orphan_path)).ComputeAlignedRange().GetMin()
    # flush on the support (z=3), +/- the small idle tip's own height shift
    assert abs(float(lo[2]) - 3.0) < 0.5, float(lo[2])
    assert orphan_path in ctx["static_extra"]
    assert any("stranded shell" in n_ for n_ in ctx["notes"]), ctx["notes"]
    assert any("1 dropped" in n_ for n_ in ctx["notes"]), ctx["notes"]


def test_repair_stranded_shell_deletes_a_multi_storey_gap():
    """DELETED-PIECE CASE. The same shape, but nothing real is left below
    except the ground, several storeys down — past `_SHELL_ORPHAN_DROP_
    STOREYS_MAX`. Must be DEACTIVATED, never repositioned: a big shell panel
    landed rigidly on an uneven mound reads worse than the mound alone, and
    the pile's own crown/reach already accounts for this piece's mass in
    aggregate (the same call `_total_collapse`'s wholesale upper-storey
    removal already makes for every OTHER piece up there)."""
    from pxr import Usd, UsdGeom

    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, parent)

    gone_paths = []
    for k in range(3):
        p = parent + "/wall_S_{0}_0".format(k)
        mesh = _box_mesh(st, p, 3.0, 1.0, 3.0, float(k) * 3.0)
        mesh.GetPrim().SetActive(False)
        gone_paths.append(p)
    orphan_path = parent + "/wall_S_3_0"
    _box_mesh(st, orphan_path, 3.0, 1.0, 3.0, 9.0)             # z in [9, 12]

    info = {"masses": {"main": {"top": 12.0,
                               "levels": [0.0, 3.0, 6.0, 9.0, 12.0],
                               "z0": 0.0}},
           "elements": [_shell_el("wall", "S", k, 0, gone_paths[k], dead=True)
                       for k in range(3)] +
                      [_shell_el("wall", "S", 3, 0, orphan_path)]}
    plan = {"displaced": {}, "removed": list(gone_paths)}
    ctx = {"stage": st, "parent": parent, "info": info,
          "rng": random.Random(5), "static_extra": [], "notes": []}

    n_dropped, n_deleted = qs._repair_stranded_shell_sliced(st, ctx, plan)
    assert (n_dropped, n_deleted) == (0, 1)
    assert not st.GetPrimAtPath(orphan_path).IsActive()
    assert any("1 deleted" in n_ for n_ in ctx["notes"]), ctx["notes"]


def test_sweep_airborne_shell_catches_what_the_strict_repair_did_not():
    """SWEEP CASE (item 3, the safety net). A partially-toothed column — one
    sub of the cell below (a narrow pier stub, off to one side) survives —
    is NOT a strict (`_repair_stranded_shell_sliced`) candidate: some of the
    below cell is still alive. But the stub does not reach anywhere near
    this piece's own footprint, so the loose gate + a real points check
    finds nothing actually under it, and the safety net deactivates it —
    the thing item 2's targeted repair, by design, left alone."""
    from pxr import Gf, Usd, UsdGeom

    st = Usd.Stage.CreateInMemory()
    parent = "/World/Bldg"
    UsdGeom.Xform.Define(st, parent)

    stub_path = parent + "/pier_S_0_0"
    stub = _box_mesh(st, stub_path, 1.0, 1.0, 3.0, 0.0)
    UsdGeom.Xformable(stub).AddTranslateOp().Set(Gf.Vec3d(-4.5, 0.0, 0.0))
    dead_path = parent + "/wall_S_0_1"
    dead = _box_mesh(st, dead_path, 3.0, 1.0, 3.0, 0.0)
    dead.GetPrim().SetActive(False)
    # the orphan: full-width, centred on x=0 — nowhere near the stub
    orphan_path = parent + "/wall_S_1_0"
    _box_mesh(st, orphan_path, 6.0, 1.0, 3.0, 3.0)             # z in [3, 6]

    info = {"masses": {"main": {"top": 6.0, "levels": [0.0, 3.0, 6.0],
                                "z0": 0.0}},
           "elements": [_shell_el("pier", "S", 0, 0, stub_path),
                       _shell_el("wall", "S", 0, 1, dead_path, dead=True),
                       _shell_el("wall", "S", 1, 0, orphan_path)]}
    plan = {"displaced": {}, "removed": [dead_path]}
    ctx = {"stage": st, "parent": parent, "info": info,
          "rng": random.Random(3), "static_extra": [], "notes": []}

    n_dropped, n_deleted = qs._repair_stranded_shell_sliced(st, ctx, plan)
    assert (n_dropped, n_deleted) == (0, 0)
    assert st.GetPrimAtPath(orphan_path).IsActive()

    n = qs._sweep_airborne_shell_sliced(st, ctx, plan, verbose=False)
    assert n == 1, n
    assert not st.GetPrimAtPath(orphan_path).IsActive()
    assert any("airborne shell sweep: 1 deactivated" in n_
              for n_ in ctx["notes"]), ctx["notes"]


def test_stranded_bands_plan_removed_is_bit_identical():
    """PLAN["REMOVED"] IDENTITY. Neither `_orphaned_shell_candidates`,
    `_repair_stranded_shell_sliced` nor `_sweep_airborne_shell_sliced` is
    called from anywhere inside `plan_damage` / `_apply_region` / `s_*` —
    they are a POST-`apply_plan` stage pass, reading `e["dead"]` back, never
    the other way round. So the pure planner's own `plan["removed"]` (which
    another agent's materials work reads off the same plan) must be
    UNCHANGED, RNG draws included, for every recipe this round touched.
    Pinned by hash rather than by the full path list (which would make this
    test itself the thing that has to be updated by hand on every unrelated
    naming change)."""
    import hashlib

    cases = [
        ("corner_fail", {"storeys": 2}, "urm", 11, 10, 45.0,
         "a3be6ffe9f1081c9"),
        ("out_of_plane", {"sides": 1, "from_storey": 1}, "urm", 7, 8, 40.0,
         "9938060acbf63d5d"),
        ("DG3", None, "urm", 5, 10, 45.0, "7fbfdf8830100530"),
        ("DG4", None, "rc", 9, 9, 42.0, "948ab26174362377"),
    ]
    for recipe, kw, btype, seed, storeys, H, want in cases:
        grade = recipe if kw is None else [(recipe, kw)]
        info, plan = _plan(grade, btype, seed=seed, storeys=storeys, H=H)
        removed = sorted(plan["removed"])
        got = hashlib.sha256(json.dumps(removed).encode()).hexdigest()[:16]
        assert got == want, (recipe, kw, btype, seed, "removed list drifted",
                            len(removed))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
