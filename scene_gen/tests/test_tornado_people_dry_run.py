"""test_tornado_people_dry_run.py — pins the pure geometry/pick-logic half of
`tools/tornado_people_dry_run.py`: the spawn-point math (distance, facing
quaternion yaw) and the exposed/partial pick logic, all on synthetic data.

NO SCENE BUILD IN THIS FILE. `build_ctx`/`run_plan_people`/`draw_overlay`
(the halves that actually compile a suburb layout) are exercised by running
the tool itself (see `_plans/raven_test_scene_runbook.md` §1b/1c), not here.

    python3 -m pytest scene_gen/tests/test_tornado_people_dry_run.py -q
    python3 scene_gen/tests/test_tornado_people_dry_run.py
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_TOOLS = os.path.join(_SCENE_GEN, "tools")
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import tornado_people_dry_run as tpdr                    # noqa: E402


# ---------------------------------------------------------------------------
# yaw_facing — distance/yaw is the caller's own job (hypot), this is the
# YAW + QUATERNION half
# ---------------------------------------------------------------------------

def test_facing_due_east():
    yaw, q = tpdr.yaw_facing(0.0, 0.0, 10.0, 0.0)
    assert abs(yaw - 0.0) < 1e-9
    assert q == [0.0, 0.0, 0.0, 1.0]


def test_facing_due_north():
    yaw, q = tpdr.yaw_facing(0.0, 0.0, 0.0, 10.0)
    assert abs(yaw - 90.0) < 1e-9
    assert abs(q[2] - math.sin(math.radians(45.0))) < 1e-9
    assert abs(q[3] - math.cos(math.radians(45.0))) < 1e-9


def test_facing_due_west():
    yaw, q = tpdr.yaw_facing(0.0, 0.0, -10.0, 0.0)
    assert abs(abs(yaw) - 180.0) < 1e-9


def test_facing_is_independent_of_distance():
    """Only the DIRECTION should matter, not how far the target is."""
    yaw_near, _q1 = tpdr.yaw_facing(0.0, 0.0, 1.0, 1.0)
    yaw_far, _q2 = tpdr.yaw_facing(0.0, 0.0, 100.0, 100.0)
    assert abs(yaw_near - yaw_far) < 1e-9
    assert abs(yaw_near - 45.0) < 1e-9


def test_facing_quaternion_is_unit_length():
    for (px, py, tx, ty) in ((0, 0, 5, 3), (-4, 7, 2, -8), (10, 10, 10, 10.001)):
        _yaw, q = tpdr.yaw_facing(px, py, tx, ty)
        norm = math.sqrt(sum(c * c for c in q))
        assert abs(norm - 1.0) < 1e-9


def test_facing_matches_manual_atan2_and_distance():
    """The combined 'spawn-point math' the CLI reports: hypot for distance,
    `yaw_facing` for the quaternion — checked together the way `main()`
    computes and prints them."""
    px, py = -2.26, -5.04
    tx, ty = -4.866, -24.865
    dist = math.hypot(tx - px, ty - py)
    yaw, q = tpdr.yaw_facing(px, py, tx, ty)
    expected_yaw = math.degrees(math.atan2(ty - py, tx - px))
    assert abs(yaw - expected_yaw) < 1e-9
    assert dist > 0.0
    # reconstruct yaw from the quaternion and check round-trip
    rebuilt = math.degrees(2.0 * math.atan2(q[2], q[3]))
    assert abs(((rebuilt - yaw + 180.0) % 360.0) - 180.0) < 1e-6


# ---------------------------------------------------------------------------
# spawn_config_entry
# ---------------------------------------------------------------------------

def test_spawn_config_entry_shape():
    e = tpdr.spawn_config_entry(1.23456, -7.891, 90.0)
    assert set(e.keys()) == {"x_m", "y_m", "orient"}
    assert e["x_m"] == 1.23
    assert e["y_m"] == -7.89
    assert len(e["orient"]) == 4
    assert e["orient"][0] == 0.0 and e["orient"][1] == 0.0


def test_spawn_config_entry_orient_matches_yaw_facing():
    yaw, q = tpdr.yaw_facing(0.0, 0.0, 3.0, 4.0)
    e = tpdr.spawn_config_entry(0.0, 0.0, yaw)
    assert abs(e["orient"][2] - round(q[2], 4)) < 1e-6
    assert abs(e["orient"][3] - round(q[3], 4)) < 1e-6


# ---------------------------------------------------------------------------
# pick_casualties — synthetic record list, no scene
# ---------------------------------------------------------------------------

def _people():
    """Mirrors the shape of a real `plan_people` record, minimal fields."""
    return [
        {"x": 100.0, "y": 100.0, "occlusion": "none", "covered_frac": 0.0},   # 0 far, exposed
        {"x": 5.0, "y": 5.0, "occlusion": "none", "covered_frac": 0.0},       # 1 near, exposed
        {"x": 3.0, "y": -4.0, "occlusion": "midriff", "covered_frac": 0.28},  # 2 near, too little cover
        {"x": -6.0, "y": 2.0, "occlusion": "flank", "covered_frac": 0.40},    # 3 near, partial (in range)
        {"x": 90.0, "y": -90.0, "occlusion": "torso_head", "covered_frac": 0.48},  # 4 far, partial (in range)
    ]


def test_autopick_prefers_nearest_to_centre_for_both():
    people = _people()
    ie, ip = tpdr.pick_casualties(people, region_center=(0.0, 0.0))
    assert ie == 1          # nearer exposed candidate, not idx 0
    assert ip == 3          # nearer partial candidate, not idx 4


def test_autopick_partial_range_is_exclusive_of_out_of_band_cover():
    """idx 2's covered_frac (0.28) sits just under the default [0.30, 0.55]
    band and must never be auto-picked over idx 3 (0.40, in-band) even
    though idx 2 is nearer the centre."""
    people = _people()
    _ie, ip = tpdr.pick_casualties(people, region_center=(0.0, 0.0))
    assert ip != 2


def test_explicit_picks_are_honoured_verbatim():
    people = _people()
    ie, ip = tpdr.pick_casualties(people, region_center=(0.0, 0.0),
                                  pick_exposed=0, pick_partial=4)
    assert (ie, ip) == (0, 4)


def test_explicit_pick_out_of_range_raises():
    people = _people()
    try:
        tpdr.pick_casualties(people, pick_exposed=99)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_explicit_mismatched_pick_is_honoured_with_a_warning(capsys):
    """Picking a casualty that is NOT occlusion=none for --pick-exposed must
    still go through (the caller asked for it on purpose) — just warns."""
    people = _people()
    ie, ip = tpdr.pick_casualties(people, pick_exposed=2, pick_partial=3)
    assert (ie, ip) == (2, 3)
    out = capsys.readouterr().out
    assert "WARNING" in out


def test_no_exposed_candidate_raises_with_reason():
    people = [{"x": 0.0, "y": 0.0, "occlusion": "midriff", "covered_frac": 0.3}]
    try:
        tpdr.pick_casualties(people)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "none" in str(exc)


def test_no_partial_candidate_raises_with_reason():
    people = [{"x": 0.0, "y": 0.0, "occlusion": "none", "covered_frac": 0.0}]
    try:
        tpdr.pick_casualties(people)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "covered_frac" in str(exc)


def test_custom_partial_band_is_respected():
    people = _people()
    # widen the band so idx 2 (0.28) now qualifies and is nearer centre than idx 3
    _ie, ip = tpdr.pick_casualties(people, region_center=(0.0, 0.0),
                                   partial_lo=0.20, partial_hi=0.55)
    assert ip == 2


# ---------------------------------------------------------------------------
# pick_spawn_point — synthetic geometry callbacks, still no scene build
# ---------------------------------------------------------------------------

def test_pick_spawn_point_finds_open_ground_away_from_a_house():
    """A single house sits due east of the casualty; with open ground
    everywhere else and uniform intensity, the search must not return a
    point inside the house's footprint."""
    region = (-100.0, -100.0, 100.0, 100.0)

    def in_any_house(x, y):
        return abs(x - 20.0) <= 5.0 and abs(y - 0.0) <= 5.0   # a 10x10 house at (20,0)

    def cross_offset(x, y):
        return y   # trivial "centreline" = the x axis

    def intensity_at(x, y):
        return 0.5   # uniform — every candidate ties on intensity

    sp = tpdr.pick_spawn_point(0.0, 0.0, 20.0, region, in_any_house,
                               cross_offset, intensity_at, n_angles=72)
    assert sp is not None
    assert not in_any_house(sp["x"], sp["y"])
    assert abs(math.hypot(sp["x"], sp["y"]) - 20.0) < 1e-6


def test_pick_spawn_point_prefers_lower_intensity():
    """Two candidate directions at the same radius, different intensity —
    the lower-intensity one must win."""
    region = (-100.0, -100.0, 100.0, 100.0)

    def in_any_house(x, y):
        return False

    def cross_offset(x, y):
        return 0.0

    def intensity_at(x, y):
        # East (angle 0) is hot, everywhere else is cold.
        return 0.9 if x > 19.0 and abs(y) < 1.0 else 0.1

    sp = tpdr.pick_spawn_point(0.0, 0.0, 20.0, region, in_any_house,
                               cross_offset, intensity_at, n_angles=72)
    assert sp is not None
    assert sp["intensity"] < 0.5


def test_pick_spawn_point_returns_none_when_the_whole_ring_is_blocked():
    region = (-100.0, -100.0, 100.0, 100.0)

    def in_any_house(x, y):
        return True   # everything is a house

    sp = tpdr.pick_spawn_point(0.0, 0.0, 20.0, region, in_any_house,
                               lambda x, y: 0.0, lambda x, y: 0.0, n_angles=16)
    assert sp is None


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
