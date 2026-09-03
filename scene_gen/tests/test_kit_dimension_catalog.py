import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "scene_gen/tools/kit_dump_to_dimension_catalog.py"
SPEC = importlib.util.spec_from_file_location("kit_dims", TOOL)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def test_catalog_keys_full_url_scale_axis_and_unrotates_world_dump():
    doc = {
        "preset": "p", "seed": 3, "dimensions_space": "world_xy",
        "placements": [{
            "category": "house", "usd": "omniverse://n/a.usd",
            "scale": 0.01, "axis_up": "Z", "yaw_deg": 90,
            "W": 20, "D": 10, "H": 30,
        }],
    }
    out = MOD.build_catalog([("kit.json", doc)])
    assert out["records"] == [{
        "usd": "omniverse://n/a.usd", "scale": 0.01, "axis_up": "Z",
        "W": 10.0, "D": 20.0, "H": 30.0,
    }]


def test_catalog_refuses_inconsistent_measurements():
    row = {"category": "house", "usd": "u", "scale": 1,
           "axis_up": "Z", "yaw_deg": 0, "W": 10, "D": 20, "H": 30}
    doc1 = {"placements": [row]}
    doc2 = {"placements": [dict(row, W=11)]}
    try:
        MOD.build_catalog([("a", doc1), ("b", doc2)])
    except ValueError as exc:
        assert "inconsistent Kit dimensions" in str(exc)
    else:
        raise AssertionError("inconsistent dimensions were accepted")


def test_catalog_merges_existing_records():
    existing = {"sources": [{"path": "old"}], "records": [{
        "usd": "old.usd", "scale": 1, "axis_up": "Z",
        "W": 1, "D": 2, "H": 3,
    }]}
    out = MOD.build_catalog([], existing=existing)
    assert out["sources"] == [{"path": "old"}]
    assert out["records"][0]["usd"] == "old.usd"
