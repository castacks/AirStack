#!/usr/bin/env python3
"""test_quake_heap_clearance.py — does the pile bury what stands under it?

    python3 scene_gen/tests/test_quake_heap_clearance.py
    pytest -q scene_gen/tests/test_quake_heap_clearance.py

WHY THIS EXISTS
---------------
Round-3 review (`final_r3/b0_apartment_tall_DG5_obl.png`,
`b3_highrise_04_DG5_obl.png`): street trees stood untouched THROUGH the
rubble pile of a totally collapsed building, a bus sat clean at the toe,
lampposts stood inside the pile. `disaster/quake._clear_under_heaps` is the
fix — for every DG4/DG5 building (and every monolith swapped for a ruin) it
removes, leans, tips or buries the street furniture and cars a heap would
actually reach.

Everything this file checks is arithmetic that needs no USD at all:

  * `heap_reach_m` / `heap_reach_sides` — how far a heap reaches past the
    wall line, per side, from agent A's memo
    (`_plans/eq_round4_rubble_research.md` SS1b/SS6): a run-out fraction of H
    on the street/fall side that DECREASES with height (0.65 at H<=4m down
    to 0.35 at H>=20m, URM x0.85, rc_glass DG5 x0.5 because only the
    cladding sheds), a flat 0.10 H on a blind/party-wall side, DG4 halving
    whichever applies, floor 1.5 m, cap 18 m;
  * `in_reach` — the yaw-aware, PER-SIDE footprint + reach test: returns
    which side a point is nearest and the actual metre distance from that
    side's wall line (0 on the wall, negative inside the footprint);
  * `_heap_action` — what a tree / pole-like prop / car does at that
    distance, banded in METRES OF H (0-0.3 H buried/crushed, 0.3 H out to
    the side's own reach lighter, beyond untouched) rather than as a
    fraction of the reach — so a blind side whose reach is under 0.3 H has
    no outer band at all, per the memo;
  * `_dedupe_actions` — when two overlapping heaps reach the same prop, the
    strongest action wins (remove > tip > lean > bury), once.

These are all PURE (no pxr import anywhere in their call chain), which is
also what this file proves just by importing `disaster.quake` on the host:
the module used to `from pxr import Gf, Sdf, Usd, UsdGeom` at module scope,
which made it unimportable here. That import is now lazy, per-function, the
same convention `quake_flow.py` already holds.

WHAT THIS CANNOT SEE: whether a leaned tree or a buried car actually reads in
a render, whether the PhysX collider on a tipped lamppost misbehaves, whether
`_clear_under_heaps` picks up the right prims on a real stage. That needs
Isaac Sim — not run here.
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake as q                  # noqa: E402


# ---------------------------------------------------------------------------
def test_module_is_importable_without_pxr():
    """The whole point of the lazy-import restructure: getting this far in
    the file means `import disaster.quake` did not need `pxr` at all."""
    for name in ("heap_reach_m", "heap_reach_sides", "in_reach",
                "_heap_action", "_dedupe_actions", "_clear_under_heaps",
                "_lean_matrix"):
        assert hasattr(q, name), name


# ---------------------------------------------------------------------------
# `heap_reach_m` / `heap_reach_sides` — agent A's run-out law
def test_fall_frac_knots_and_flats():
    assert q._fall_frac(4.0) == 0.65
    assert q._fall_frac(1.0) == 0.65           # flat below the first knot
    assert q._fall_frac(12.0) == 0.40
    assert q._fall_frac(20.0) == 0.35
    assert q._fall_frac(30.0) == 0.35          # flat above the last knot
    # linear between knots: H=8 is halfway from 4 to 12
    assert abs(q._fall_frac(8.0) - (0.65 + 0.40) / 2.0) < 1e-9
    # H=16 is halfway from 12 to 20
    assert abs(q._fall_frac(16.0) - (0.40 + 0.35) / 2.0) < 1e-9


def test_heap_reach_fall_side_by_construction_type():
    """URM travels x0.85 of the rc baseline; rc_glass at DG5 sheds only its
    cladding, x0.5. H=15 (no floor/cap in play): base fraction 0.38125."""
    H = 15.0
    base = q._fall_frac(H)
    assert abs(q.heap_reach_m("rc", "DG5", H) - base * H) < 1e-9
    assert abs(q.heap_reach_m("urm", "DG5", H) - base * 0.85 * H) < 1e-9
    assert abs(q.heap_reach_m("rc_glass", "DG5", H) - base * 0.5 * H) < 1e-9
    # the worked example in the brief: a 15 m URM building's front reach
    assert abs(q.heap_reach_m("urm", "DG5", H) - 4.86) < 0.02


def test_heap_reach_dg4_halves_dg5():
    """On the FALL side H=25 keeps every type away from both the 1.5 m floor
    and the 10 m cap (rc 8.75 m, urm 7.4 m, rc_glass 4.4 m), so the halving is
    visible rather than swallowed by a clamp. On the BLIND side the 3 m cap and
    the 1.5 m floor bracket every realistic value, so only the ordering and the
    bracket are asserted there."""
    H = 25.0
    for btype in ("urm", "rc", "rc_glass"):
        dg5 = q.heap_reach_m(btype, "DG5", H, fall_side=True)
        dg4 = q.heap_reach_m(btype, "DG4", H, fall_side=True)
        assert 1.5 < dg4 < 10.0 and 1.5 < dg5 < 10.0, btype
        assert abs(dg4 - 0.5 * dg5) < 1e-9, btype
        b5 = q.heap_reach_m(btype, "DG5", H, fall_side=False)
        b4 = q.heap_reach_m(btype, "DG4", H, fall_side=False)
        assert 1.5 <= b4 <= b5 <= 3.0, (btype, b4, b5)


def test_heap_reach_blind_side_is_flat_and_type_independent():
    """0.10 H, the same for every construction type — only the fall side
    knows what the building is made of."""
    H = 15.0
    for btype in ("urm", "rc", "rc_glass"):
        blind = q.heap_reach_m(btype, "DG5", H, fall_side=False)
        assert abs(blind - 0.10 * H) < 1e-9, btype


def test_heap_reach_floor_and_cap():
    # a small building's blind reach (0.10 * 5 = 0.5 m) is floored at 1.5 m
    assert q.heap_reach_m("urm", "DG5", 5.0, fall_side=False) == 1.5
    # the 15 m URM worked example's BLIND side lands exactly on the floor
    assert q.heap_reach_m("urm", "DG5", 15.0, fall_side=False) == 1.5
    # a tall tower's reach is capped like the planner's run-out: 10 m fall, 3 m blind
    assert q.heap_reach_m("rc", "DG5", 100.0, fall_side=True) == 10.0
    assert q.heap_reach_m("urm", "DG5", 100.0, fall_side=True) == 10.0
    assert q.heap_reach_m("rc", "DG5", 100.0, fall_side=False) == 3.0


def test_heap_reach_sides_front_is_fall_two_sides_are_blind():
    H = 15.0
    fall = q.heap_reach_m("urm", "DG5", H, fall_side=True)
    blind = q.heap_reach_m("urm", "DG5", H, fall_side=False)
    sides = q.heap_reach_sides("urm", "DG5", H, "/World/city/bld_42")
    assert set(sides) == {"S", "E", "N", "W"}
    assert sides["S"] == fall                  # the front always gets it
    values = list(sides.values())
    assert values.count(fall) == 2             # S + exactly one more side
    assert values.count(blind) == 2


def test_heap_reach_sides_is_deterministic_per_prim_path():
    """Same building, same run: the "extra" fall side must not flip between
    two calls (or two identical scenes would throw debris differently)."""
    a = q.heap_reach_sides("rc", "DG5", 20.0, "/World/city/bld_7")
    b = q.heap_reach_sides("rc", "DG5", 20.0, "/World/city/bld_7")
    assert a == b


def test_heap_reach_sides_varies_across_buildings():
    """Not every building throws its extra debris the same direction —
    otherwise every DG5 building in a scene would bury its E side and none
    of its N/W sides."""
    seen = set()
    for i in range(12):
        sides = q.heap_reach_sides("rc", "DG5", 20.0, "/World/city/bld_{0}".format(i))
        extra = [s for s in ("E", "N", "W")
                if sides[s] == q.heap_reach_m("rc", "DG5", 20.0, fall_side=True)]
        seen.add(extra[0])
    assert len(seen) > 1, "every building picked the same extra side"


# ---------------------------------------------------------------------------
# `in_reach` — a 20 x 10 m footprint (W=20 along local x, D=10 along local y)
def _forward(cx, cy, yaw_deg, lx, ly):
    """World point for a LOCAL offset (lx, ly) from centre (cx, cy), rotated
    FORWARD by +yaw — the inverse of what `in_reach` does internally (it
    rotates incoming world points by -yaw), so this is an independent check
    rather than a restatement of the function under test."""
    a = math.radians(yaw_deg)
    return (cx + lx * math.cos(a) - ly * math.sin(a),
            cy + lx * math.sin(a) + ly * math.cos(a))


def test_in_reach_isotropic_wall_toe_beyond_every_yaw():
    """A single-number `reach` is the isotropic convenience wrapper: every
    side reads the same, so this exercises the geometry independent of the
    per-side law above."""
    W, D, reach = 20.0, 10.0, 5.0
    cx, cy = 3.0, -4.0
    for yaw in (0.0, 90.0, 37.0):
        x, y = _forward(cx, cy, yaw, 0.0, 0.0)
        inside, d, _side = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
        assert inside, (yaw, "centre")
        assert d == -5.0, (yaw, d)             # nearest wall is D/2 = 5 m in

        x, y = _forward(cx, cy, yaw, 10.0, 0.0)
        inside, d, _side = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
        assert inside and abs(d) < 1e-9, (yaw, "W wall", d)

        x, y = _forward(cx, cy, yaw, 0.0, 5.0)
        inside, d, _side = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
        assert inside and abs(d) < 1e-9, (yaw, "D wall", d)

        x, y = _forward(cx, cy, yaw, 10.0 + reach, 0.0)
        inside, d, _side = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
        assert inside, (yaw, "toe")
        assert abs(d - reach) < 1e-9, (yaw, d)

        x, y = _forward(cx, cy, yaw, 10.0 + reach + 1.0, 0.0)
        inside, d, _side = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
        assert not inside, (yaw, "beyond")
        assert d > reach, (yaw, d)


def test_in_reach_side_naming_at_yaw_zero():
    W, D, reach = 20.0, 10.0, 5.0
    assert q.in_reach(0, 0, W, D, 0.0, 12.0, 0.0, reach)[2] == "E"
    assert q.in_reach(0, 0, W, D, 0.0, -12.0, 0.0, reach)[2] == "W"
    assert q.in_reach(0, 0, W, D, 0.0, 0.0, 7.0, reach)[2] == "N"
    assert q.in_reach(0, 0, W, D, 0.0, 0.0, -7.0, reach)[2] == "S"


def test_in_reach_dist_is_symmetric_through_the_centre_isotropic():
    """`dist_m` is symmetric under mirroring even though `side` is not (the
    mirrored point names the OPPOSITE side) — only `dist_m` and, for an
    isotropic reach, `inside` are guaranteed equal."""
    W, D, reach = 20.0, 10.0, 6.0
    cx, cy = 8.0, -2.0
    for yaw in (0.0, 15.0, 37.0, 90.0, 213.0):
        for lx, ly in ((7.0, 3.0), (-4.0, 6.5), (11.5, -1.0), (0.0, 0.0)):
            x, y = _forward(cx, cy, yaw, lx, ly)
            mx, my = 2 * cx - x, 2 * cy - y
            i1, d1, _s1 = q.in_reach(cx, cy, W, D, yaw, x, y, reach)
            i2, d2, _s2 = q.in_reach(cx, cy, W, D, yaw, mx, my, reach)
            assert i1 == i2, (yaw, lx, ly)
            assert abs(d1 - d2) < 1e-9, (yaw, lx, ly, d1, d2)


def test_in_reach_corner_is_the_pythagorean_distance():
    W, D, reach = 20.0, 10.0, 5.0
    inside, d, side = q.in_reach(0.0, 0.0, W, D, 0.0, 11.0, 6.0, reach)
    assert inside
    assert abs(d - math.hypot(1.0, 1.0)) < 1e-9
    assert side == "E"                         # a tie on ox/oy breaks to side_x


def test_in_reach_per_side_dict_is_anisotropic():
    """The entire reason `in_reach` takes a dict: the SAME offset distance
    reads as in-reach on the fall side and out-of-reach on a blind side."""
    W, D = 20.0, 10.0
    sides = {"S": 10.0, "N": 1.0, "E": 1.0, "W": 1.0}
    # 8 m out from the S wall (fall side, reach 10): in reach
    inside, d, side = q.in_reach(0.0, 0.0, W, D, 0.0, 0.0, -13.0, sides)
    assert side == "S" and inside and abs(d - 8.0) < 1e-9
    # mirrored: 8 m out from the N wall (blind side, reach 1.0): NOT in reach
    inside, d, side = q.in_reach(0.0, 0.0, W, D, 0.0, 0.0, 13.0, sides)
    assert side == "N" and not inside and abs(d - 8.0) < 1e-9


# ---------------------------------------------------------------------------
# `_heap_action` — bands in METRES OF H, not fractions of the reach
def test_heap_action_tree_buried_vs_lean_vs_untouched():
    H, reach = 20.0, 10.0          # buried band = min(0.3*20, 10) = 6.0
    assert q._heap_action("tree", 0.0, reach, H) == "remove"
    assert q._heap_action("tree", 5.999, reach, H) == "remove"
    assert q._heap_action("tree", 6.0, reach, H) == "lean"
    assert q._heap_action("tree", 10.0, reach, H) == "lean"       # at the toe
    assert q._heap_action("tree", 10.001, reach, H) is None
    assert q._heap_action("tree", -3.0, reach, H) == "remove"     # inside


def test_heap_action_pole_tips_across_the_whole_reach():
    H, reach = 20.0, 10.0
    assert q._heap_action("pole", 0.0, reach, H) == "tip"
    assert q._heap_action("pole", 6.0, reach, H) == "tip"          # past buried
    assert q._heap_action("pole", 10.0, reach, H) == "tip"         # at the toe
    assert q._heap_action("pole", 10.001, reach, H) is None
    assert q._heap_action("pole", -5.0, reach, H) == "tip"


def test_heap_action_car_buried_only_inside_the_buried_band():
    H, reach = 20.0, 10.0          # buried band = 6.0
    assert q._heap_action("car", 0.0, reach, H) == "bury"
    assert q._heap_action("car", 5.999, reach, H) == "bury"
    assert q._heap_action("car", 6.0, reach, H) is None            # lighter band
    assert q._heap_action("car", 9.0, reach, H) is None
    assert q._heap_action("car", 10.001, reach, H) is None


def test_heap_action_blind_side_whole_reach_is_buried():
    """When the side's reach is under 0.3 H (a blind side, typically), the
    outer 'lighter' band vanishes and the whole reach counts as buried."""
    H, reach = 20.0, 2.0           # 0.3*H = 6.0 > reach: buried = min(6, 2) = 2
    assert q._heap_action("tree", 1.0, reach, H) == "remove"       # not "lean"
    assert q._heap_action("car", 1.0, reach, H) == "bury"
    assert q._heap_action("tree", 2.5, reach, H) is None           # past reach
    assert q._heap_action("pole", 1.9, reach, H) == "tip"


def test_heap_action_unknown_kind_is_inert():
    for d in (-1.0, 0.0, 5.0, 20.0):
        assert q._heap_action("bench", d, 10.0, 20.0) is None
        assert q._heap_action(None, d, 10.0, 20.0) is None


def test_prop_kind_covers_both_generators_vocabularies():
    """The legacy `scene_generator.build_city` packer says "tree"; the newer
    `detail/city_detail.py` says "street_tree" — both must resolve the same
    way, or one generator's scenes get no clearance at all."""
    assert q._prop_kind("tree") == "tree"
    assert q._prop_kind("street_tree") == "tree"
    assert q._prop_kind("car") == "car"
    for cat in ("streetlight", "traffic_light", "sign", "fire_hydrant",
                "trash_can", "dumpster", "bollard", "utility_pole"):
        assert q._prop_kind(cat) == "pole", cat
    for cat in ("bench", "cafe_set", "bus_stop", "manhole", "storm_drain",
                None, "human"):
        assert q._prop_kind(cat) is None, cat


# ---------------------------------------------------------------------------
def test_dedupe_strongest_action_wins():
    entries = [
        ("a", "lean", 1), ("a", "remove", 2),        # a: remove beats lean
        ("b", "bury", 3), ("b", "tip", 4),            # b: tip beats bury
        ("c", "bury", 5),                             # c: only one entry
        ("d", "lean", 6), ("d", "tip", 7), ("d", "remove", 8),
    ]
    best = q._dedupe_actions(entries)
    assert best["a"] == ("remove", 2)
    assert best["b"] == ("tip", 4)
    assert best["c"] == ("bury", 5)
    assert best["d"] == ("remove", 8)


def test_dedupe_ignores_falsy_paths_and_bad_actions():
    entries = [("", "remove", 1), (None, "tip", 2), ("e", "sparkle", 3),
              ("e", "lean", 4)]
    best = q._dedupe_actions(entries)
    assert list(best.keys()) == ["e"]
    assert best["e"] == ("lean", 4)


def test_dedupe_first_seen_breaks_a_tie():
    """Two entries for the same prop with the SAME action: the first payload
    is kept (no reason to prefer the second)."""
    best = q._dedupe_actions([("f", "tip", "first"), ("f", "tip", "second")])
    assert best["f"] == ("tip", "first")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)


def test_measured_extent_beats_nominal_reach():
    """Round-4 Isaac pass: a manifest row carrying the trimmed mound's
    measured per-side `extent_m` wins over the nominal `reach_m` wherever it
    is larger (the relaxed blind-side foot), and never shrinks a side."""
    r = {"reach_m": {"S": 6.0, "N": 1.6, "E": 1.7, "W": 5.2}, "fall_sides": ["S", "W"],
         "extent_m": {"S": 6.4, "N": 5.1, "E": 4.8, "W": 5.0}}
    got = q._heap_reach_for(r, "urm", "DG5", 20.0)
    assert got["N"] == 5.1 and got["E"] == 4.8      # blind sides: measured foot
    assert got["S"] == 6.4                           # larger measured value wins
    assert got["W"] == 5.2                           # never shrinks below nominal
    # extent alone (no reach_m) is enough
    r2 = {"extent_m": {"S": 3.0, "N": 2.0, "E": 2.5, "W": 2.0}, "fall_sides": ["S"]}
    got2 = q._heap_reach_for(r2, "urm", "DG5", 20.0)
    assert got2 == {"S": 3.0, "N": 2.0, "E": 2.5, "W": 2.0}


# ---------------------------------------------------------------------------
# round-5 addendum: `_fall_sides_for` — the STREET DEBRIS pass (`_street_
# debris_pass` / `quake_rubble.plan_street_scatter`) has to agree with heap
# clearance on which side is "the street", or it could scatter debris on a
# building's blind/party-wall side instead. `_street_debris_pass` itself
# needs pxr (it authors a PointInstancer); `_fall_sides_for` is its pure
# core, so it gets a `test_quake_heap_clearance`-style pure test here rather
# than a stage-authoring one.
# ---------------------------------------------------------------------------
def test_fall_sides_for_uses_measured_fall_sides_when_present():
    r = {"fall_sides": ["S", "E"]}
    assert q._fall_sides_for(r, "/World/city/bld_1") == {"S", "E"}
    # a single measured side is respected too, not padded to a second one
    assert q._fall_sides_for({"fall_sides": ["N"]}, "/World/city/bld_1") == {"N"}


def test_fall_sides_for_matches_heap_reach_sides_stable_hash_when_absent():
    """No measured `fall_sides` on the record: falls back to EXACTLY the
    same stable-hashed second side `heap_reach_sides` itself draws for the
    same `prim_path` — the two passes must pick the same "street" side for
    a DG3 building neither pass has a manifest row to measure from."""
    for path in ("/World/city/bld_0", "/World/city/bld_7", "/World/city/bld_42"):
        sides = q.heap_reach_sides("rc", "DG5", 20.0, path)
        fall = q.heap_reach_m("rc", "DG5", 20.0, fall_side=True)
        expect_extra = next(s for s in ("E", "N", "W") if sides[s] == fall)
        got = q._fall_sides_for({}, path)
        assert got == {"S", expect_extra}, (path, got, expect_extra)


def test_fall_sides_for_is_deterministic_per_prim_path():
    a = q._fall_sides_for({}, "/World/city/bld_9")
    b = q._fall_sides_for({}, "/World/city/bld_9")
    assert a == b


# ---------------------------------------------------------------------------
# round-5 addendum: the street-debris pass's env gate. `_street_debris_
# enabled` reads `QUAKE_STREET_DEBRIS` live (not cached at import time, so
# it can be exercised here without reimporting the module) — default ON,
# per the brief ("gated QUAKE_STREET_DEBRIS=1 default on").
# ---------------------------------------------------------------------------
def test_street_debris_enabled_default_on_and_env_gated():
    saved = os.environ.pop("QUAKE_STREET_DEBRIS", None)
    try:
        assert q._street_debris_enabled() is True
        for off in ("0", "false", "False", "no"):
            os.environ["QUAKE_STREET_DEBRIS"] = off
            assert q._street_debris_enabled() is False, off
        os.environ["QUAKE_STREET_DEBRIS"] = "1"
        assert q._street_debris_enabled() is True
    finally:
        if saved is None:
            os.environ.pop("QUAKE_STREET_DEBRIS", None)
        else:
            os.environ["QUAKE_STREET_DEBRIS"] = saved
