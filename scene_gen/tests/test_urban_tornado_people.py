#!/usr/bin/env python3
"""
test_urban_tornado_people.py — pure placement math for
`disaster/urban_tornado_people.py` (stream B, round 3, the R12 bench's
D-row). Pins the anchor geometry, the berm burial-fraction model, the
rule-4 macroblock gate, the rule-1 T3/T4 casualty gate, and determinism —
everything the module's own docstring claims, without touching `pxr`.

`disaster/urban_tornado_people.py` imports only `math` at module scope (see
its own docstring, "WHY NOT REIMPLEMENT") — `to_placement` is the one
function that reaches for `disaster.people` (and through it, eventually,
`pxr`), and this file does not call it: every class-placement function
under test returns a plain dict, checked as data.

USAGE
    python3 scene_gen/tests/test_urban_tornado_people.py
    pytest -s scene_gen/tests/test_urban_tornado_people.py
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import urban_tornado_people as utp      # noqa: E402


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def _wall_anchor(along_m=12.0, bearing_deg=270.0):
    """A windward wall anchor the way the bench launcher measures one: a
    point at the wall's own base, outward bearing 270 (world -Y, the
    bearing every cell in the bench shares when the wind is drawn at the
    probe's own bearing 35 -> `wind_at` bearing ~57.6, S windward)."""
    return utp.anchor(100.0, -20.0, bearing_deg, along_m=along_m, cell="B1")


# ---------------------------------------------------------------------------
# anchor geometry
# ---------------------------------------------------------------------------

def test_anchor_outward_offset_moves_away_from_wall():
    """`_along_wall` at `along_frac=0.5` (dead centre) and increasing
    `offset_m` should move monotonically FURTHER from the wall point along
    the anchor's own outward bearing — never sideways only."""
    a = _wall_anchor()
    base = _dist(a["x"], a["y"], *utp._along_wall(a, 0.5, 0.0))
    assert base < 1e-9, "offset 0 must sit exactly at the wall anchor point"
    prev = 0.0
    for off in (0.5, 1.0, 2.0, 3.0, 4.0):
        x, y = utp._along_wall(a, 0.5, off)
        d = _dist(a["x"], a["y"], x, y)
        assert abs(d - off) < 1e-9, (
            "offset %.2f produced distance %.4f from the wall, not equal to "
            "the offset itself" % (off, d))
        assert d > prev
        prev = d


def test_anchor_along_frac_spans_the_wall_run():
    """`along_frac` 0 and 1 should land `along_m` apart (the wall's full
    run), symmetric about the anchor's own centre point at 0.5."""
    a = _wall_anchor(along_m=12.0)
    x0, y0 = utp._along_wall(a, 0.0, 0.0)
    x1, y1 = utp._along_wall(a, 1.0, 0.0)
    assert abs(_dist(x0, y0, x1, y1) - 12.0) < 1e-9
    xc, yc = utp._along_wall(a, 0.5, 0.0)
    assert abs(xc - a["x"]) < 1e-9 and abs(yc - a["y"]) < 1e-9


def test_anchor_offset_is_perpendicular_to_along():
    """Moving along the wall (offset fixed at 0) must not change the
    distance-from-wall-line component; moving outward (along fixed at 0.5)
    must not drift sideways. Checked by decomposing into the anchor's own
    along/outward unit vectors."""
    a = _wall_anchor(bearing_deg=270.0)
    r = math.radians(a["bearing_deg"])
    ax, ay = -math.sin(r), math.cos(r)
    ox, oy = math.cos(r), math.sin(r)
    x, y = utp._along_wall(a, 0.75, 2.5)
    dx, dy = x - a["x"], y - a["y"]
    along_component = dx * ax + dy * ay
    out_component = dx * ox + dy * oy
    assert abs(along_component - (0.75 - 0.5) * a["along_m"]) < 1e-9
    assert abs(out_component - 2.5) < 1e-9


# ---------------------------------------------------------------------------
# berm profile / burial fraction
# ---------------------------------------------------------------------------

def test_berm_profile_zero_at_wall_and_past_span():
    assert utp.berm_profile(0.0) == 0.0
    assert utp.berm_profile(-1.0) == 0.0
    assert utp.berm_profile(utp.BERM_SPAN_M) == 0.0
    assert utp.berm_profile(utp.BERM_SPAN_M + 1.0) == 0.0


def test_berm_profile_peaks_at_peak_m():
    at_peak = utp.berm_profile(utp.BERM_PEAK_M)
    assert abs(at_peak - utp.BERM_PEAK_FRAC) < 1e-9
    for off in (0.2, 0.5, 1.5, 2.0, 3.0, 3.9):
        assert utp.berm_profile(off) <= at_peak + 1e-9, (
            "offset %.2f exceeds the profile's own peak" % off)


def test_berm_profile_monotonic_each_side_of_peak():
    rising = [utp.berm_profile(d) for d in (0.1, 0.3, 0.5, 0.7, 0.9)]
    assert rising == sorted(rising), "not monotonically rising to the peak"
    falling = [utp.berm_profile(d) for d in (1.1, 2.0, 3.0, 3.9)]
    assert falling == sorted(falling, reverse=True), (
        "not monotonically falling past the peak")


def test_burial_fraction_clamped_and_noiseless_without_rng():
    for off in (-5.0, 0.0, 1.0, 4.0, 50.0):
        f = utp.burial_fraction(off)
        assert 0.0 <= f <= 1.0
    # no rng -> exactly the profile, no jitter
    assert utp.burial_fraction(1.0) == utp.berm_profile(1.0)


def test_burial_fraction_jitter_stays_clamped():
    rng = random.Random(3)
    for _ in range(200):
        off = rng.uniform(-1.0, 5.0)
        f = utp.burial_fraction(off, rng=rng, jitter=0.5)
        assert 0.0 <= f <= 1.0


def test_burial_fraction_from_debris_empty_is_zero():
    assert utp.burial_fraction_from_debris(0.0, 0.0, []) == 0.0
    assert utp.burial_fraction_from_debris(0.0, 0.0, None) == 0.0


def test_burial_fraction_from_debris_scales_with_nearby_mass():
    far = [{"x": 100.0, "y": 100.0, "l": 1.0, "w": 1.0}]
    near_light = [{"x": 0.05, "y": 0.0, "l": 0.3, "w": 0.2}]
    near_heavy = [{"x": 0.0, "y": 0.0, "l": 1.0, "w": 1.0}] * 4
    f_far = utp.burial_fraction_from_debris(0.0, 0.0, far, radius_m=1.0)
    f_light = utp.burial_fraction_from_debris(0.0, 0.0, near_light, radius_m=1.0)
    f_heavy = utp.burial_fraction_from_debris(0.0, 0.0, near_heavy, radius_m=1.0)
    assert f_far == 0.0
    assert 0.0 < f_light < f_heavy <= 1.0


# ---------------------------------------------------------------------------
# rule 1 — the T3/T4 casualty gate
# ---------------------------------------------------------------------------

def test_casualty_gate_t3_t4_only():
    assert utp.casualty_gate("T3") is True
    assert utp.casualty_gate("T4") is True
    for lvl in ("T0", "T1", "T2", "t3", "T5", "", None):
        assert utp.casualty_gate(lvl) is False, (
            "level %r must NOT pass the casualty gate" % (lvl,))


# ---------------------------------------------------------------------------
# rule 4 — the macroblock-gated crush victim
# ---------------------------------------------------------------------------

def test_crush_victim_gated_off_without_macroblock():
    a = _wall_anchor()
    rng = random.Random(1)
    assert utp.crush_victim(a, rng, macroblock_present=False) is None
    assert utp.crush_victim(a, rng, macroblock_present=[]) is None


def test_crush_victim_present_with_macroblock():
    a = _wall_anchor()
    rng = random.Random(1)
    rec = utp.crush_victim(a, rng, macroblock_present=True)
    assert rec is not None
    assert rec["class"] == "crush_victim"
    assert utp.CRUSH_OFFSET_RANGE_M[0] <= rec["offset_m"] <= \
        utp.CRUSH_OFFSET_RANGE_M[1]
    # 3-6 m is past the berm's own 0-4 m band and past a car's own reach
    assert rec["offset_m"] >= utp.BERM_OFFSET_RANGE_M[1] - 1.0


def test_crush_victim_offset_is_clearly_past_the_berm():
    a = _wall_anchor()
    rng = random.Random(4)
    for _ in range(40):
        rec = utp.crush_victim(a, rng, macroblock_present=[{"x": 1}])
        assert rec["offset_m"] >= 3.0 - 1e-9
        assert rec["offset_m"] <= 6.0 + 1e-9


# ---------------------------------------------------------------------------
# clustering (rule 11) and counts
# ---------------------------------------------------------------------------

def test_digger_pair_clusters_tightly():
    a = _wall_anchor(along_m=20.0)
    rng = random.Random(7)
    figs = utp.digger_pair(a, rng, n=2, cluster_span_m=3.0)
    assert len(figs) == 2
    d = _dist(figs[0]["x"], figs[0]["y"], figs[1]["x"], figs[1]["y"])
    # generous bound: two figures within a few berm-widths of each other,
    # never spread across the whole 20 m wall run
    assert d < 8.0, "diggers %.2f m apart -- not a tight cluster" % d
    for f in figs:
        assert f["class"] == "digger"
        assert f["prone"] is False
        assert f["pose"] in dict(utp._DIGGER_POSES)


def test_digger_pair_n_is_honoured():
    a = _wall_anchor()
    figs = utp.digger_pair(a, random.Random(2), n=3)
    assert len(figs) == 3


def test_pile_edge_digger_clusters_and_faces_the_pile():
    p = utp.anchor(50.0, 50.0, 90.0, along_m=0.0)
    figs = utp.pile_edge_digger(p, random.Random(9), n=2, spread_m=2.0)
    assert len(figs) == 2
    d = _dist(figs[0]["x"], figs[0]["y"], figs[1]["x"], figs[1]["y"])
    assert d < 6.0
    for f in figs:
        # faces back toward the pile (opposite the pile's own outward
        # bearing), matching digger_pair's own convention
        assert abs(f["yaw_deg"] - ((p["bearing_deg"] + 180.0) % 360.0)) < 1e-6


def test_doorway_threshold_stays_at_the_door_not_the_interior():
    door = utp.anchor(10.0, 10.0, 0.0)
    rng = random.Random(5)
    for _ in range(50):
        figs = utp.doorway_threshold(door, rng)
        assert 1 <= len(figs) <= 2
        for f in figs:
            assert _dist(f["x"], f["y"], door["x"], door["y"]) <= 0.7
            assert f["anchor"] == "doorway"


def test_doorway_threshold_n_override():
    door = utp.anchor(0.0, 0.0, 0.0)
    assert len(utp.doorway_threshold(door, random.Random(1), n=1)) == 1
    assert len(utp.doorway_threshold(door, random.Random(1), n=2)) == 2


# ---------------------------------------------------------------------------
# trapped / covered casualties: burial-driven pose selection
# ---------------------------------------------------------------------------

def test_trapped_in_berm_prone_and_within_band():
    lying_names = set(dict(utp._BURIED_POSES)) | set(
        dict(utp._VISIBLE_LYING_POSES))
    a = _wall_anchor()
    rng = random.Random(11)
    for _ in range(30):
        rec = utp.trapped_in_berm(a, rng)
        assert rec["prone"] is True
        assert utp.BERM_OFFSET_RANGE_M[0] <= rec["offset_m"] <= \
            utp.BERM_OFFSET_RANGE_M[1]
        assert 0.0 <= rec["burial_frac"] <= 1.0
        assert rec["pose"] in lying_names


def test_trapped_in_berm_pose_tracks_burial_threshold():
    """Force a shallow and a deep burial via a fixed offset (through
    `along_frac`/monkeypatched offset is not exposed, so drive it via
    `berm_offset`'s own range at the profile's extremes) and check the pose
    selection follows `BURIAL_THRESHOLD`."""
    a = _wall_anchor()
    heavy = dict((n, w) for n, w in utp._BURIED_POSES)
    light = dict((n, w) for n, w in utp._VISIBLE_LYING_POSES)
    # deterministic burial fraction path: call the pure pieces directly
    rng = random.Random(0)
    deep_pose = utp._lying_pose_for_burial(0.9, rng)
    shallow_pose = utp._lying_pose_for_burial(0.05, rng)
    assert deep_pose in heavy
    assert shallow_pose in light


def test_covered_casualty_prone_with_debris_overlap():
    center = utp.anchor(0.0, 0.0, 45.0)
    debris = [{"x": 0.5, "y": 0.5, "l": 1.0, "w": 1.0}] * 6
    rec = utp.covered_casualty(center, random.Random(2), debris=debris)
    assert rec["prone"] is True
    assert rec["class"] == "covered_casualty"
    assert 0.0 <= rec["burial_frac"] <= 1.0


# ---------------------------------------------------------------------------
# car occupant / evacuee pair
# ---------------------------------------------------------------------------

def test_car_occupant_sits_at_the_car_and_faces_its_heading():
    car = utp.anchor(5.0, -5.0, 123.0)
    rec = utp.car_occupant(car, random.Random(3))
    assert rec["x"] == car["x"] and rec["y"] == car["y"]
    assert rec["yaw_deg"] == car["bearing_deg"]
    assert rec["prone"] is False
    assert rec["pose"] in dict(utp._SEATED_POSES)


def test_evacuee_pair_two_figures_offset_and_separated():
    entry = utp.anchor(0.0, 0.0, 0.0, along_m=0.0)
    figs = utp.evacuee_pair(entry, random.Random(6))
    assert len(figs) == 2
    for f in figs:
        assert f["class"] == "evacuee"
        assert f["prone"] is False
        d = _dist(f["x"], f["y"], entry["x"], entry["y"])
        assert d > 0.5, "an evacuee must stand OFF the door threshold itself"
    d = _dist(figs[0]["x"], figs[0]["y"], figs[1]["x"], figs[1]["y"])
    assert d > 0.1, "the pair collapsed onto the same point"


# ---------------------------------------------------------------------------
# determinism
# ---------------------------------------------------------------------------

def _run_all(seed):
    berm = _wall_anchor()
    pile = utp.anchor(80.0, -30.0, 200.0, along_m=0.0)
    door = utp.anchor(80.0, -33.0, 200.0)
    car = utp.anchor(40.0, -10.0, 15.0)
    entry = utp.anchor(0.0, 40.0, 270.0)
    rng = random.Random(seed)
    out = []
    out += utp.digger_pair(berm, rng)
    out.append(utp.trapped_in_berm(berm, rng))
    out.append(utp.crush_victim(berm, rng, macroblock_present=True))
    out += utp.pile_edge_digger(pile, rng)
    out.append(utp.covered_casualty(pile, rng))
    out += utp.doorway_threshold(door, rng)
    out.append(utp.car_occupant(car, rng))
    out += utp.evacuee_pair(entry, rng)
    return out


def test_determinism_same_seed_same_output():
    a = _run_all(42)
    b = _run_all(42)
    assert a == b, "the same seed through the same call sequence must " \
        "reproduce byte-identical placements"


def test_determinism_different_seed_usually_differs():
    a = _run_all(1)
    b = _run_all(2)
    assert a != b, "two different seeds produced identical output -- " \
        "something is not actually consuming the rng"


# ---------------------------------------------------------------------------
# rule 8 — the open-space-pedestrian cap is a constant this bench honours
# by construction (no class here IS one); pin the constant exists and is 1.
# ---------------------------------------------------------------------------

def test_open_pedestrian_cap_constant():
    assert utp.MAX_OPEN_PEDESTRIAN == 1


def test_vehicle_share_constant_documented():
    # rule 3: roughly double the suburb's incidental 0.10 street share
    assert 0.15 <= utp.VEHICLE_SHARE <= 0.25


def main():
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    bad = 0
    for name, fn in fns:
        try:
            fn()
            print("PASS %s" % name)
        except AssertionError as exc:
            bad += 1
            print("FAIL %s\n     %s" % (name, exc))
    print("\n%d/%d passed" % (len(fns) - bad, len(fns)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
