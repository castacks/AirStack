#!/usr/bin/env python3
"""Offline checks for the Isaac-only earthquake review-camera helpers."""
import ast
import math
from pathlib import Path


LAUNCHER = (Path(__file__).resolve().parents[2] / "simulation" / "isaac-sim"
            / "launch_scripts" / "downtown_quake_launch_script.py")
_NAMES = {"_footprint_radius", "_building_blocks", "_camera_eye_blocked",
          "_review_camera_pose", "_building_review_pose",
          "_review_camera_clearance", "_review_camera_azimuth"}


def _helpers():
    tree = ast.parse(LAUNCHER.read_text(), filename=str(LAUNCHER))
    body = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == "TILT_DEG_MAX"
                for t in node.targets):
            body.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in _NAMES:
            body.append(node)
    ns = {"math": math}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(LAUNCHER), "exec"), ns)
    return ns


def test_building_review_pose_clears_roof_and_scales_with_asset():
    fn = _helpers()["_building_review_pose"]
    low = fn({"W": 22.0, "D": 14.0, "H": 14.0})
    tall = fn({"W": 60.5, "D": 36.5, "H": 131.0})
    assert low[0] > 14.0
    assert tall[0] > 131.0
    assert tall[0] > low[0]
    assert tall[1] > low[1]
    assert 0.35 * 131.0 < tall[3] < 0.55 * 131.0


def test_old_top_height_was_inside_tower_new_pose_is_not():
    ns = _helpers()
    rec = {"x": 0.0, "y": 0.0, "W": 60.5, "D": 36.5,
           "H": 131.0, "grade": "DG4+tilt"}
    assert ns["_camera_eye_blocked"]([rec], 0.0, 0.0, 70.0)
    top_h, _d, _h, _aim = ns["_building_review_pose"](rec)
    assert not ns["_camera_eye_blocked"]([rec], 0.0, 0.0, top_h)


def test_building_review_pose_handles_bad_record_without_kit():
    top_h, dist, obl_h, aim_h = _helpers()["_building_review_pose"](
        {"W": None, "D": "bad", "H": object()})
    assert top_h > 12.0
    assert dist >= 28.0
    assert obl_h > aim_h >= 2.0


def test_launcher_no_longer_groups_all_buildings_at_fixed_pose():
    src = LAUNCHER.read_text()
    assert "_building_review_pose(r)" in src
    assert "top_h=70.0, obl_dist=55.0, obl_h=28.0" not in src


def test_resolved_snapshot_directory_is_shared_with_review_helper():
    src = LAUNCHER.read_text()
    assert 'os.environ["SNAP_DIR"] = SNAP_DIR' in src


def test_yawed_record_uses_its_real_rectangular_footprint():
    blocks = _helpers()["_building_blocks"]
    rec = {"x": 0.0, "y": 0.0, "W": 60.0, "D": 12.0, "H": 80.0,
           "yaw_deg": 45.0, "grade": "DG3"}
    # Local x=25, y=0 rotated by +45 deg: inside the long rectangle but far
    # outside its old six-metre inscribed circle.
    x = 25.0 / math.sqrt(2.0)
    assert blocks(rec, x, x, 20.0)
    # Local y=10 is outside the narrow axis.
    x2, y2 = -10.0 / math.sqrt(2.0), 10.0 / math.sqrt(2.0)
    assert not blocks(rec, x2, y2, 20.0)


def test_targeted_azimuth_avoids_tall_foreground_building():
    ns = _helpers()
    target = {"prim": "/target", "x": 0.0, "y": 0.0, "W": 12.0,
              "D": 10.0, "H": 14.0, "yaw_deg": 0.0, "grade": "DG5"}
    obstacle = {"prim": "/tower", "x": -14.0, "y": -14.0, "W": 22.0,
                "D": 42.0, "H": 100.0, "yaw_deg": 45.0,
                "grade": "DG4+tilt"}
    preferred = 225.0
    az = ns["_review_camera_azimuth"](
        [target, obstacle], target, 28.0, 13.0, 6.0,
        preferred=preferred)
    assert az != preferred
    a = math.radians(az)
    eye = (28.0 * math.cos(a), 28.0 * math.sin(a), 13.0)
    assert not ns["_camera_eye_blocked"]([obstacle], *eye, margin_m=2.0)
