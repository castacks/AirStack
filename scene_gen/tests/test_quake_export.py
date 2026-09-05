#!/usr/bin/env python3
"""Pure-Python gates for the urban-earthquake dataset adapter."""

import os
import sys


sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..")))

from disaster import quake_export as qe  # noqa: E402


def test_people_document_refuses_ordinary_population():
    try:
        qe.people_document([{"active": True, "state": "standing"}], {})
    except ValueError as exc:
        assert "standing/walking" in str(exc)
    else:
        raise AssertionError("ordinary person was accepted")


def test_people_document_keeps_only_active_casualties():
    rows = [
        {"id": "a", "active": True, "state": "rubble_casualty"},
        {"id": "b", "active": False, "state": "interior_casualty"},
    ]
    got = qe.people_document(rows, {
        "interior_casualties": 0, "rubble_casualties": 1,
        "generic_humans_deactivated": 11, "cover_pieces": 2,
        "underfilled": 0,
    })
    assert got["count"] == 1
    assert got["standing_or_walking"] == 0
    assert [r["id"] for r in got["people"]] == ["a"]


def test_house_objects_canonicalise_only_intact_grade():
    got = qe.house_objects([
        {"prim": "/a", "style": "office", "grade": "DG0",
         "yaw_deg": 12.0},
        {"prim": "/b", "style": "brick", "grade": "AEC_DG4+tilt",
         "yaw_deg": 23.0},
    ])
    assert got[0]["level"] == "pristine"
    assert got[1]["level"] == "AEC_DG4+tilt"
    assert qe.base_grade(got[1]["level"]) == "DG4"


def test_vehicle_adapter_does_not_label_buildings_as_cars():
    got = qe.cars_from_placements([
        {"category": "house", "prim_path": "/house", "usd": "h.usd"},
        {"category": "car", "prim_path": "/car", "usd": "pickup.usd",
         "yaw_deg": 90.0},
    ])
    assert len(got) == 1
    assert got[0]["prim_path"] == "/car"
    assert got[0]["occupied"] is False
