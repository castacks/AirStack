"""Pure contract tests for the earthquake casualty-only population."""

import os
import random
import sys

import numpy as np


ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from disaster import quake, quake_people as qp  # noqa: E402


def _building(**kw):
    out = {"style": "office", "grade": "DG4", "prim": "/World/g/house_1",
           "x": 10.0, "y": 20.0, "yaw_deg": 90.0,
           "W": 30.0, "D": 18.0, "H": 24.0,
           "fall_sides": ["S", "E"],
           "reach_m": {"S": 3.0, "E": 7.0}, "type": "rc"}
    out.update(kw)
    return out


def test_damage_side_prefers_exact_failure_and_largest_reach():
    rec = _building(failure_sides=["S", "E"])
    assert qp.damage_sides(rec) == ["E", "S"]


def test_rotated_face_and_footprint_use_building_yaw():
    rec = _building()
    cx, cy, nx, ny, _tx, _ty, half = qp.face_center(rec, "S")
    # Local -Y rotated +90 degrees points world +X.
    assert round(cx, 6) == 19.0
    assert round(cy, 6) == 20.0
    assert (round(nx, 6), round(ny, 6)) == (1.0, 0.0)
    assert half == 15.0
    assert qp.point_in_footprint(10.0, 20.0, rec)
    assert not qp.point_in_footprint(30.0, 20.0, rec)


def test_budget_excludes_mild_mono_and_directionless_buildings():
    rows = [_building(prim="a"), _building(prim="b", grade="DG2"),
            _building(prim="c", mono=True),
            _building(prim="d", fall_sides=[], failure_sides=[])]
    assert qp.population_budget(rows, requested=99) == 2
    assert qp.population_budget(rows) == 2


def test_disable_generic_population_prevents_city_people_authoring():
    config = {
        "usds": {"humans": ["a.usd", "b.usd"], "cars": ["car.usd"]},
        "disaster": {"humans_strewn": [20, 30],
                     "humans_prone_fraction": 0.4},
    }
    assert qp.disable_generic_population(config) == 2
    assert config["usds"]["humans"] == []
    assert config["usds"]["cars"] == ["car.usd"]
    assert config["disaster"]["humans_strewn"] == [0, 0]
    assert config["disaster"]["humans_prone_fraction"] == 0.0


def test_rubble_cover_is_partial_contact_solved_and_two_to_four_pieces():
    rec = {"id": "eqp_0000", "pose": "buried_reach",
           "yaw_deg": 33.0, "x": 2.0, "y": -4.0, "z": 0.2,
           "construction_type": "urm"}
    specs = qp._cover_specs(rec, random.Random(7))
    assert 2 <= len(specs) <= 4
    assert 0.0 < rec["covered_frac"] <= qp.MAX_COVERED_FRAC
    assert rec["cover_piece_count"] == len(specs)
    assert all(s["over_record_id"] == rec["id"] for s in specs)
    assert all(s["class"] in {"brick_chunk", "concrete_chunk", "beam_chunk"}
               for s in specs)
    assert all(s["z"] >= rec["z"] for s in specs)


def test_attach_damage_metadata_uses_exact_final_variant():
    rows = {
        ("office", "DG4"): {"style": "office", "level": "DG4",
                              "usd": "/b/bld_office_DG4.usd",
                              "fall_sides": ["S"], "type": "rc",
                              "reach_m": {"S": 4.2}},
        ("office", "DG4_v1"): {"style": "office", "level": "DG4_v1",
                                 "usd": "/b/bld_office_DG4_v1.usd",
                                 "fall_sides": ["E"], "type": "rc",
                                 "reach_m": {"E": 6.4}},
    }
    recs = [{"style": "office", "grade": "DG4",
             "prim": "/World/g/house"}]
    placements = [{"prim_path": "/World/g/house",
                   "usd": "/b/bld_office_DG4_v1.usd"}]
    quake.attach_damage_metadata(recs, placements, rows)
    assert recs[0]["damage_level"] == "DG4_v1"
    assert recs[0]["failure_sides"] == ["E"]
    assert recs[0]["failure_sides_source"] == "legacy_fall_sides"
    assert recs[0]["reach_m"] == {"E": 6.4}


def test_segment_triangle_gate_distinguishes_wall_from_opening():
    # A square wall in the YZ plane at x=1 blocks a segment through its
    # middle, but not a parallel segment above the wall opening.
    A = np.array([[1.0, -1.0, 0.0], [1.0, -1.0, 0.0]])
    B = np.array([[1.0, 1.0, 0.0], [1.0, 1.0, 2.0]])
    C = np.array([[1.0, 1.0, 2.0], [1.0, -1.0, 2.0]])
    assert qp._segment_triangle_hits(
        (0.0, 0.0, 1.0), (2.0, 0.0, 1.0), A, B, C).any()
    assert not qp._segment_triangle_hits(
        (0.0, 0.0, 3.0), (2.0, 0.0, 3.0), A, B, C).any()


def test_interior_review_eyes_are_drone_distance_outside_failed_face():
    rec = _building(yaw_deg=0.0)
    target, eyes = qp._interior_review_geometry(
        rec, 24.0, 20.0, 6.0, "E", 1.0)
    assert target == (24.0, 20.0, 6.35)
    # Both eyes are well outside the local east wall at x=25, rather than
    # close interior inspection cameras; the second is materially higher.
    assert eyes[0][0] > 35.0
    assert eyes[1][0] > eyes[0][0]
    assert eyes[1][2] > eyes[0][2]


def test_rubble_review_eyes_are_outside_and_above_debris_apron():
    rec = _building(yaw_deg=0.0)
    target, eyes = qp._rubble_review_geometry(
        rec, 30.0, 20.0, 1.5, "E", tangent_sign=-1.0)
    assert target == (30.0, 20.0, 1.92)
    assert eyes[0][0] == 40.0
    assert eyes[1][0] == 44.0
    assert eyes[1][1] < eyes[0][1]
    assert eyes[0][2] > target[2] + 6.0
    assert eyes[1][2] > eyes[0][2]


def test_prone_support_points_follow_root_at_soles_pose_axis():
    # Positive-roll prone points from the sole root toward world +X at yaw 0.
    prone = qp._prone_support_points(
        4.0, 7.0, "lying_prone", 0.0, height_m=2.0)
    assert min(x for x, _y in prone) >= 4.0
    assert max(x for x, _y in prone) > 5.8
    # Negative-roll supine points toward -X. This is deliberately not a
    # symmetric footprint around the placement origin.
    supine = qp._prone_support_points(
        4.0, 7.0, "lying_supine", 0.0, height_m=2.0)
    assert max(x for x, _y in supine) <= 4.0
    assert min(x for x, _y in supine) < 2.2
    assert len(prone) == 9
