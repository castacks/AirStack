import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "scene_gen/tools/verify_dump_matches_kit.py"
SPEC = importlib.util.spec_from_file_location("verify_dump", TOOL)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def _p(i, usd, x, y):
    return {"i": i, "usd": usd, "x_m": x, "y_m": y, "category": "house"}


def test_geometry_match_ignores_full_list_index_drift():
    left = {10: _p(10, "a.usd", 1, 2), 30: _p(30, "b.usd", 8, 9)}
    right = {700: _p(700, "b.usd", 8.1, 9), 900: _p(900, "a.usd", 1, 2.1)}
    matched, missing, extra = MOD._geometry_match(left, right, 0.5)
    assert len(matched) == 2
    assert not missing
    assert not extra


def test_geometry_match_reports_real_model_or_position_difference():
    left = {1: _p(1, "a.usd", 0, 0)}
    right = {2: _p(2, "a.usd", 5, 0), 3: _p(3, "b.usd", 0, 0)}
    matched, missing, extra = MOD._geometry_match(left, right, 0.5)
    assert not matched
    assert missing == [left[1]]
    assert len(extra) == 2
