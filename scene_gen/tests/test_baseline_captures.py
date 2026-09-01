#!/usr/bin/env python3
"""test_baseline_captures.py -- the offline plan-math check for
`disaster/baseline_captures.py` (the FINAL urban-fire baseline review pass:
overviews / districts / buildings / people / groups, one capture plan per
scene -- see that module's own docstring for the user directive it answers).

    python3 scene_gen/tests/test_baseline_captures.py
    pytest -q scene_gen/tests/test_baseline_captures.py

MOSTLY HOST-SIDE, NO `pxr`. Every `plan_*` function in the module under test
except `plan_building_shots` needs nothing but `math` -- no stage, no `pxr`,
no Kit -- so this file imports the module directly and exercises the real
functions (unlike `test_urban_fire_city_launch.py`, which has to go through
`ast` because the launcher it tests builds a live `SimulationApp`). The one
exception, `test_plan_building_shots_*`, DOES need `pxr` (it exercises
`fire_assembly_lib.fire_view_params`/`clear_oblique` for real, the same way
several other `scene_gen/tests/*.py` files already do) and is skipped
outright if `pxr` cannot be imported, so this file still runs clean on a
host that lacks it.
"""
import math
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TESTS)
for _p in (_SCENE_GEN, os.path.join(_SCENE_GEN, "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from disaster import baseline_captures as bc               # noqa: E402

try:
    import pxr as _pxr                                     # noqa: F401
    _HAVE_PXR = True
except Exception:
    _HAVE_PXR = False


# ---------------------------------------------------------------------------
# helpers to build synthetic records with no launcher/stage context at all
# ---------------------------------------------------------------------------
def _building(i, x, y, W=20.0, D=15.0, yaw=0.0, H=24.0):
    return {"i": i, "x": x, "y": y,
           "bbox": [x - W / 2.0, y - D / 2.0, 0.0, x + W / 2.0, y + D / 2.0, H],
           "rec": {"W": W, "D": D, "yaw_deg": yaw},
           "stem": "bldg{0}".format(i)}


def _person(i, cls, x, y, z=0.0, yaw_deg=0.0, building_i=1, group=None,
           side=None, prone=False):
    r = {"id": i, "cls": cls, "x": x, "y": y, "z": z, "yaw_deg": yaw_deg,
        "building_i": building_i, "prone": prone}
    if group is not None:
        r["group"] = group
    if side is not None:
        r["side"] = side
    return r


# ===========================================================================
# 0) shared helpers
# ===========================================================================
def test_connected_components_basic():
    # 3 points: 0-1 close, 2 far from both -> two components, biggest first
    pts = [(0.0, 0.0), (5.0, 0.0), (500.0, 500.0)]

    def gap(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    comps = bc.connected_components(pts, gap, 25.0)
    assert comps == [[0, 1], [2]]


def test_connected_components_empty():
    assert bc.connected_components([], lambda a, b: 0.0, 25.0) == []


def test_connected_components_transitive_chain():
    # a-b close, b-c close, a-c far: still ONE component via b
    pts = [(0.0, 0.0), (20.0, 0.0), (40.0, 0.0)]

    def gap(a, b):
        return abs(a[0] - b[0])

    comps = bc.connected_components(pts, gap, 25.0)
    assert len(comps) == 1
    assert sorted(comps[0]) == [0, 1, 2]


def test_mean_bearing_average_and_fallback():
    assert abs(bc._mean_bearing_deg([0.0, 0.0]) - 0.0) < 1e-6
    assert abs(bc._mean_bearing_deg([0.0, 90.0]) - 45.0) < 1e-6
    # opposite bearings cancel -> fallback
    assert bc._mean_bearing_deg([0.0, 180.0], fallback=225.0) == 225.0
    assert bc._mean_bearing_deg([], fallback=225.0) == 225.0


def test_assert_unique_names_passes_and_raises():
    a = bc.Shot("x", (0, 0, 0), (0, 0, 0), 18.0)
    b = bc.Shot("y", (0, 0, 0), (0, 0, 0), 18.0)
    assert bc.assert_unique_names([a, b]) is True
    dup = bc.Shot("x", (1, 1, 1), (0, 0, 0), 18.0)
    try:
        bc.assert_unique_names([a, dup])
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "x" in str(exc)


# ===========================================================================
# 1) overviews
# ===========================================================================
def test_plan_overviews_shape_and_names():
    shots = bc.plan_overviews(1000.0)
    assert [s.name for s in shots] == [
        "overviews/city_top", "overviews/city_corner_ne",
        "overviews/city_corner_sw"]
    bc.assert_unique_names(shots)


def test_plan_overviews_top_is_true_plumb():
    shots = bc.plan_overviews(1000.0)
    top = shots[0]
    assert top.eye[0] == 0.0 and top.eye[1] == 0.0
    assert top.target == (0.0, 0.0, 0.0)
    assert abs(top.eye[2] - 1000.0 * bc.OVERVIEW_TOP_FRAC) < 1e-6


def test_plan_overviews_corners_are_opposite_and_scale_with_span():
    small = bc.plan_overviews(400.0)
    big = bc.plan_overviews(1000.0)
    ne_small, sw_small = small[1], small[2]
    ne_big = big[1]
    # opposite bearings: 225 - 45 == 180
    assert abs((bc.OVERVIEW_CORNER_AZIMUTHS_DEG[1]
               - bc.OVERVIEW_CORNER_AZIMUTHS_DEG[0]) % 360.0 - 180.0) < 1e-6
    # a bigger plate pushes the corner cameras further out and higher
    d_small = math.hypot(ne_small.eye[0], ne_small.eye[1])
    d_big = math.hypot(ne_big.eye[0], ne_big.eye[1])
    assert d_big > d_small
    assert ne_big.eye[2] > ne_small.eye[2]
    # the two corners are mirror images through the origin
    assert abs(ne_small.eye[0] + sw_small.eye[0]) < 1e-6
    assert abs(ne_small.eye[1] + sw_small.eye[1]) < 1e-6


# ===========================================================================
# 2) districts
# ===========================================================================
def test_footprint_normalizes_launcher_shape():
    f = bc._footprint(_building(1, 100.0, -50.0, W=10.0, D=8.0, yaw=30.0, H=12.0))
    assert f == {"x": 100.0, "y": -50.0, "W": 10.0, "D": 8.0, "yaw": 30.0, "H": 12.0}


def test_footprint_normalizes_flat_synthetic_shape():
    f = bc._footprint({"x": 1.0, "y": 2.0, "W": 5.0, "D": 5.0, "yaw": 0.0, "H": 9.0})
    assert f["H"] == 9.0 and f["W"] == 5.0


def test_cluster_buildings_adjacency_and_isolation():
    # two buildings 5 m apart (walls closer once W/D is subtracted) cluster;
    # a third 200 m away stays its own component.
    placed = [
        _building(1, 0.0, 0.0, W=10.0, D=10.0),
        _building(2, 20.0, 0.0, W=10.0, D=10.0),   # gap ~10 m < 25 m
        _building(3, 300.0, 300.0, W=10.0, D=10.0),
    ]
    clusters = bc.cluster_buildings(placed, adjacency_m=25.0)
    sizes = sorted(len(c) for c in clusters)
    assert sizes == [1, 2]
    big = [c for c in clusters if len(c) == 2][0]
    assert {r["i"] for r in big} == {1, 2}


def test_cluster_buildings_single_building_is_its_own_component():
    placed = [_building(1, 0.0, 0.0)]
    clusters = bc.cluster_buildings(placed)
    assert len(clusters) == 1 and len(clusters[0]) == 1


def test_district_orbit_azimuths_count_and_spacing():
    two = bc.district_orbit_azimuths(1)
    assert len(two) == 2
    also_two = bc.district_orbit_azimuths(2)
    assert len(also_two) == 2
    three = bc.district_orbit_azimuths(3, base_deg=0.0)
    assert len(three) == 3
    assert three == [0.0, 120.0, 240.0]
    many = bc.district_orbit_azimuths(20)
    assert len(many) == 3


def test_district_view_params_scales_with_span_and_height():
    small = bc.district_view_params((0.0, 0.0, 20.0, 20.0), 10.0)
    big = bc.district_view_params((0.0, 0.0, 200.0, 200.0), 10.0)
    assert big["dist"] > small["dist"]
    tall = bc.district_view_params((0.0, 0.0, 20.0, 20.0), 300.0)
    assert tall["dist"] > small["dist"]
    assert tall["obl_h"] > small["obl_h"]


def test_plan_district_shots_names_unique_and_counts_match_azimuths():
    placed = [
        _building(1, 0.0, 0.0), _building(2, 15.0, 0.0),   # cluster A (2)
        _building(3, 15.0, 15.0),                          # still close enough -> A (3)
        _building(4, 1000.0, 1000.0),                      # isolated -> B (1)
    ]
    shots = bc.plan_district_shots(placed, adjacency_m=25.0)
    bc.assert_unique_names(shots)
    # cluster A has 3 buildings -> 3 angles; cluster B has 1 -> 2 angles
    assert len(shots) == 3 + 2
    for s in shots:
        assert s.name.startswith("districts/d")
        assert s.focal_mm == bc.DISTRICT_FOCAL_MM


def test_plan_district_shots_empty_input():
    assert bc.plan_district_shots([]) == []


# ===========================================================================
# 3) buildings -- pure helpers only here; the pxr-dependent function below
# ===========================================================================
def test_building_tag_format():
    assert bc.building_tag(12, "towerA", "F4") == "d12_towerA_F4"
    assert bc.building_tag(3, None, "") == "d3_bldg"


def test_second_oblique_azimuth_wraps():
    assert bc.second_oblique_azimuth(0.0) == 120.0
    assert bc.second_oblique_azimuth(300.0) == 60.0
    assert 0.0 <= bc.second_oblique_azimuth(-40.0) < 360.0


def test_plan_building_shots_needs_bbox_or_is_skipped():
    # no bbox -> contributes nothing (mirrors the launcher's own `if not b:
    # continue`), and this path needs no pxr import at all since it returns
    # before ever reaching `fire_view_params`.
    if not _HAVE_PXR:
        return
    out = bc.plan_building_shots([{"i": 1, "x": 0.0, "y": 0.0}])
    assert out == []


def test_plan_building_shots_full():
    if not _HAVE_PXR:
        return
    placed = [{
        "i": 7, "x": 100.0, "y": 200.0, "stem": "towerA",
        "bbox": [90.0, 190.0, 0.0, 110.0, 210.0, 40.0],
        "doc": {"level": "F3", "name": "towerA",
               "fire": {"sides": ["E"], "mass": "main", "storeys": [1, 2]}},
        "masses": {"main": {"levels": [0.0, 10.0, 20.0, 30.0, 40.0]}},
    }]
    shots = bc.plan_building_shots(placed)
    assert len(shots) == 3
    bc.assert_unique_names(shots)
    names = [s.name for s in shots]
    assert names == ["buildings/d7_towerA_F3_top",
                     "buildings/d7_towerA_F3_obl000",
                     "buildings/d7_towerA_F3_obl120"]
    obl0, obl120 = shots[1], shots[2]
    # the two obliques are NOT the same shot -- "diff angles"
    assert obl0.eye != obl120.eye


def test_plan_building_shots_second_oblique_is_120_off_the_first():
    if not _HAVE_PXR:
        return
    from disaster import fire_assembly_lib as fal
    placed = [{
        "i": 1, "x": 0.0, "y": 0.0, "stem": "b",
        "bbox": [-5.0, -5.0, 0.0, 5.0, 5.0, 10.0],
        "doc": {"level": "F2", "fire": {"sides": [], "mass": "main"}},
        "masses": None,
    }]
    vp = fal.fire_view_params(placed[0]["doc"], None, placed[0]["bbox"])
    shots = bc.plan_building_shots(placed)
    obl0, obl120 = shots[1], shots[2]

    def az_of(shot):
        dx = shot.eye[0] - shot.target[0]
        dy = shot.eye[1] - shot.target[1]
        return math.degrees(math.atan2(dy, dx)) % 360.0

    assert abs((az_of(obl120) - az_of(obl0)) % 360.0 - 120.0) < 1e-3


# ===========================================================================
# 4) people
# ===========================================================================
def test_person_shot_params_default_tight_portrait():
    rec = _person(1, "evacuee", 10.0, 0.0, z=0.0, yaw_deg=0.0)
    eye, target, focal = bc.person_shot_params(rec, 0.0)
    assert focal == bc._TIGHT_FOCAL_MM
    assert abs(math.hypot(eye[0] - 10.0, eye[1] - 0.0) - bc._TIGHT_DIST_M) < 1e-6
    assert abs(target[2] - bc._STAND_AIM_M) < 1e-6


def test_person_shot_params_prone_uses_prone_aim():
    rec = _person(1, "evacuee", 10.0, 0.0, z=0.0, yaw_deg=0.0, prone=True)
    _eye, target, _focal = bc.person_shot_params(rec, 0.0)
    assert abs(target[2] - bc._PRONE_AIM_M) < 1e-6


def test_person_shot_params_window_is_street_level_both_angles():
    rec = _person(1, "window", 0.0, 0.0, z=20.0, yaw_deg=90.0)
    for off in bc.PEOPLE_ANGLE_OFFSETS_DEG:
        eye, _target, focal = bc.person_shot_params(rec, off)
        assert focal == bc._WINDOW_FOCAL_MM
        assert abs(eye[2] - bc._WINDOW_EYE_M) < 1e-6
        dist = math.hypot(eye[0], eye[1])
        assert abs(dist - bc._WINDOW_DIST_M) < 1e-6


def test_person_shot_params_burial_is_elevated_both_angles():
    for cls in bc._BURIAL_CLASSES:
        rec = _person(1, cls, 0.0, 0.0, z=0.0, yaw_deg=0.0)
        for off in bc.PEOPLE_ANGLE_OFFSETS_DEG:
            eye, target, _focal = bc.person_shot_params(rec, off)
            assert eye[2] > target[2], "burial closeup must look down, not level"


def test_plan_people_shots_two_per_record_and_unique_names():
    recs = [_person(1, "evacuee", 0.0, 0.0), _person(2, "window", 5.0, 5.0, side="E")]
    shots = bc.plan_people_shots(recs)
    assert len(shots) == 4
    bc.assert_unique_names(shots)
    names = sorted(s.name for s in shots)
    assert names == ["people/evacuee_1_a000", "people/evacuee_1_a090",
                     "people/window_2_a000", "people/window_2_a090"]


def test_plan_people_shots_missing_id_falls_back_to_index_and_stays_unique():
    recs = [_person(None, "onlooker", 0.0, 0.0), _person(None, "onlooker", 1.0, 1.0)]
    for r in recs:
        del r["id"]
    shots = bc.plan_people_shots(recs)
    bc.assert_unique_names(shots)
    assert len(shots) == 4


def test_plan_people_shots_empty():
    assert bc.plan_people_shots([]) == []


# ===========================================================================
# 5) groups
# ===========================================================================
def test_group_key_window_vs_other_classes():
    win = _person(1, "window", 0, 0, building_i=3, side="E")
    assert bc.group_key(win) == ("window", 3, "E")
    roof = _person(2, "roof", 0, 0, building_i=3, group=44)
    assert bc.group_key(roof) == ("roof", 44)
    # no group id and not a window -> unkeyable
    stray = _person(3, "roof", 0, 0, building_i=3)
    assert bc.group_key(stray) is None
    # a window with no side -> unkeyable
    win_no_side = _person(4, "window", 0, 0, building_i=3)
    assert bc.group_key(win_no_side) is None


def test_cluster_people_drops_singletons():
    recs = [
        _person(1, "roof", 0, 0, building_i=1, group=1),
        _person(2, "roof", 1, 0, building_i=1, group=1),
        _person(3, "casualty_apron", 0, 0, building_i=1, group=2),  # alone
    ]
    clusters = bc.cluster_people(recs)
    assert list(clusters.keys()) == [("roof", 1)]
    assert len(clusters[("roof", 1)]) == 2


def test_group_aabb_and_eye_target_sane():
    recs = [_person(1, "roof", 0.0, 0.0, z=30.0, yaw_deg=0.0),
           _person(2, "roof", 4.0, 2.0, z=30.0, yaw_deg=0.0)]
    aabb = bc.group_aabb(recs)
    assert aabb == (0.0, 0.0, 30.0, 4.0, 2.0, 30.0)
    eye, target = bc.group_eye_target(aabb, 0.0)
    # camera must stand OUTSIDE the group's own footprint
    dist = math.hypot(eye[0] - target[0], eye[1] - target[1])
    assert dist >= bc.GROUP_MIN_DIST_M - 1e-6
    assert eye[2] > target[2]        # an oblique, not a level shot


def test_plan_group_shots_naming_matches_the_work_order_examples():
    recs = []
    # a 3-person roof crowd on building 7 -> group_roof_b7_01
    for k in range(3):
        recs.append(_person(100 + k, "roof", float(k), 0.0, z=30.0,
                            yaw_deg=0.0, building_i=7, group=900))
    # a 4-window run on building 7's east face -> group_windows_b7_E
    for k in range(4):
        recs.append(_person(200 + k, "window", 10.0, float(k), z=12.0,
                            yaw_deg=90.0, building_i=7, side="E"))
    shots = bc.plan_group_shots(recs)
    names = sorted(s.name for s in shots)
    assert names == ["groups/group_roof_b7_01", "groups/group_windows_b7_E"]
    bc.assert_unique_names(shots)


def test_plan_group_shots_sequence_numbers_per_building_and_class():
    recs = []
    for group_id, bi in ((1, 7), (2, 7), (3, 9)):
        for k in range(2):
            recs.append(_person(1000 * group_id + k, "roof_victim", float(k),
                                0.0, z=25.0, building_i=bi, group=group_id))
    shots = bc.plan_group_shots(recs)
    names = sorted(s.name for s in shots)
    assert names == ["groups/group_roof_victim_b7_01",
                     "groups/group_roof_victim_b7_02",
                     "groups/group_roof_victim_b9_01"]


def test_plan_group_shots_empty():
    assert bc.plan_group_shots([]) == []


# ===========================================================================
# 6) sightline clearance no-op fast path (never imports fire_assembly_lib)
# ===========================================================================
def test_clear_shot_sightline_noop_without_obstacles():
    s = bc.Shot("x", (10.0, 0.0, 5.0), (0.0, 0.0, 1.0), 18.0)
    assert bc.clear_shot_sightline(s, None) is s
    assert bc.clear_shot_sightline(s, []) is s


def test_apply_sightline_clearance_empty_obstacles_is_identity():
    shots = [bc.Shot("a", (1, 0, 1), (0, 0, 0), 18.0),
            bc.Shot("b", (0, 1, 1), (0, 0, 0), 18.0)]
    out = bc.apply_sightline_clearance(shots, [])
    assert out == shots


# ===========================================================================
# 7) plan_stats / print_plan_summary
# ===========================================================================
def test_plan_stats_totals_and_eta():
    families = {
        "overviews": [bc.Shot("o1", (0, 0, 0), (0, 0, 0), 18.0)] * 3,
        "districts": [bc.Shot("d1", (0, 0, 0), (0, 0, 0), 18.0)] * 5,
        "buildings": [bc.Shot("b1", (0, 0, 0), (0, 0, 0), 18.0)] * 30,
        "people": [bc.Shot("p1", (0, 0, 0), (0, 0, 0), 18.0)] * 40,
        "groups": [bc.Shot("g1", (0, 0, 0), (0, 0, 0), 18.0)] * 6,
        "_all": [],
    }
    counts = bc.plan_stats(families)
    assert counts["total"] == 3 + 5 + 30 + 40 + 6
    assert "_all" not in counts
    assert counts["est_wall_clock_s_lo"] == counts["total"] * bc.SEC_PER_SHOT_LO
    assert counts["est_wall_clock_s_hi"] == counts["total"] * bc.SEC_PER_SHOT_HI


def test_print_plan_summary_prints_and_returns_counts(capsys):
    families = {"overviews": [bc.Shot("o1", (0, 0, 0), (0, 0, 0), 18.0)]}
    counts = bc.print_plan_summary(families, prefix="test")
    out = capsys.readouterr().out
    assert "[test] capture plan:" in out
    assert "estimated wall clock" in out
    assert counts["total"] == 1


# ===========================================================================
# 8) integration sanity: a city-scale synthetic scene lands "roughly
#    600-900" shots, per the work order's own worked example.
# ===========================================================================
def _synthetic_city(n_buildings=75, n_people=200):
    """A cluster-friendly building layout (several tight knots + a couple of
    isolated fires) and a class-realistic people population WITH proper
    `group`/`side` semantics, built with no launcher/stage context -- the
    same normalized shapes `_building`/`_person` produce above."""
    placed = []
    i = 1
    knot = 0
    while len(placed) < n_buildings - 3:
        bx, by = knot * 400.0, 0.0
        for k in range(8):
            placed.append(_building(i, bx + 12.0 * k, by, W=10.0, D=10.0,
                                    H=20.0 + 5.0 * (k % 3)))
            i += 1
        knot += 1
    for _ in range(3):
        placed.append(_building(i, 5000.0 + 500.0 * i, 5000.0, W=10.0, D=10.0))
        i += 1
    placed = placed[:n_buildings]

    people = []
    pid = 1
    gid = 1
    building_ids = [r["i"] for r in placed]

    def bi():
        return building_ids[pid % len(building_ids)]

    # window facades: groups of 3-4 on one (building, side)
    while len(people) < int(0.16 * n_people):
        b, side = bi(), ["N", "E", "S", "W"][gid % 4]
        for _k in range(3):
            people.append(_person(pid, "window", float(pid), 0.0, z=12.0,
                                  yaw_deg=0.0, building_i=b, side=side))
            pid += 1
        gid += 1
    # roof crowds: groups of 3
    while len(people) < int(0.28 * n_people):
        b = bi()
        for _k in range(3):
            people.append(_person(pid, "roof", float(pid), 0.0, z=30.0,
                                  yaw_deg=0.0, building_i=b, group=gid))
            pid += 1
        gid += 1
    # roof_victim: groups of 2
    while len(people) < int(0.36 * n_people):
        b = bi()
        for _k in range(2):
            people.append(_person(pid, "roof_victim", float(pid), 0.0, z=30.0,
                                  building_i=b, group=gid))
            pid += 1
        gid += 1
    # casualty_apron + roof_debris: groups of 2
    while len(people) < int(0.46 * n_people):
        b = bi()
        cls = "casualty_apron" if gid % 3 else "roof_debris"
        for _k in range(2):
            people.append(_person(pid, cls, float(pid), 0.0, z=0.0,
                                  building_i=b, group=gid))
            pid += 1
        gid += 1
    # interior_trapped: groups of 2
    while len(people) < int(0.56 * n_people):
        b = bi()
        for _k in range(2):
            people.append(_person(pid, "interior_trapped", float(pid), 0.0,
                                  z=0.0, building_i=b, group=gid))
            pid += 1
        gid += 1
    # street classes (evacuee/onlooker/at_car): the remainder, in groups of 3
    cls_cycle = ["evacuee", "onlooker", "at_car"]
    while len(people) < n_people:
        b = bi()
        cls = cls_cycle[gid % 3]
        for _k in range(3):
            if len(people) >= n_people:
                break
            people.append(_person(pid, cls, float(pid), 0.0, z=0.0,
                                  building_i=b, group=gid))
            pid += 1
        gid += 1
    return placed[:n_buildings], people[:n_people]


def test_city_scale_shot_count_lands_in_the_worked_example_range():
    placed, people = _synthetic_city(75, 200)
    overviews = bc.plan_overviews(1000.0)
    districts = bc.plan_district_shots(placed)
    people_shots = bc.plan_people_shots(people)
    group_shots = bc.plan_group_shots(people)
    # BUILDINGS needs pxr for real framing; the shot COUNT (3/building,
    # always, no filtering -- see `plan_building_shots`) does not.
    n_building_shots = 3 * len(placed)

    total = (len(overviews) + len(districts) + n_building_shots
            + len(people_shots) + len(group_shots))
    assert len(people_shots) == 2 * len(people)
    # the work order's own worked example: "roughly 600-900 shots" for
    # ~60-90 burning and ~200 people -- a loose band, not a tuned target.
    assert 500 <= total <= 1000, total

    # name uniqueness across every family at once, exactly what a real
    # launcher's `build_capture_plan` checks before it authors anything.
    all_shots = overviews + districts + people_shots + group_shots
    bc.assert_unique_names(all_shots)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
