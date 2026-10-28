"""CPU checks of the launcher's collider-box geometry: the rotation, the box corners, the
horizontal footprint distance, and the box measured from iris.usd."""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

AIRSTACK = Path(__file__).resolve().parents[2]
LAUNCHER = AIRSTACK / "simulation" / "isaac-sim" / "launch_scripts" / "safe_eval_launch_script.py"
IRIS = (AIRSTACK / "simulation" / "isaac-sim" / "extensions" / "PegasusSimulator" / "extensions"
        / "pegasus.simulator" / "pegasus" / "simulator" / "assets" / "Robots" / "Iris" / "iris.usd")
METHODS = ("_drone_box", "_rotation", "_box_corners", "_footprint_distance")


def _box_class(**names):
    """ScenarioManager's collider-box methods on a bare class, since the launcher imports Kit."""
    tree = ast.parse(LAUNCHER.read_text())
    manager = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ScenarioManager")
    body = [n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    assert sorted(n.name for n in body) == sorted(METHODS)
    module = ast.Module(body=[ast.ClassDef(name="Box", bases=[], keywords=[], body=body,
                                           decorator_list=[])], type_ignores=[])
    namespace = {"np": np, **names}
    exec(compile(ast.fix_missing_locations(module), str(LAUNCHER), "exec"), namespace)
    return namespace["Box"]


def _box(centre=(0.0, 0.0, 0.0), half=(0.3, 0.2, 0.05)):
    box = _box_class()()
    box._box = (np.asarray(centre, dtype=np.float64), np.asarray(half, dtype=np.float64))
    return box


def _quat(axis, angle):
    axis = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    return [np.cos(angle / 2.0), *(np.sin(angle / 2.0) * axis)]


def test_rotation_is_proper_and_turns_x_towards_y_on_positive_yaw():
    rot = _box_class()._rotation
    for q in (_quat([0, 0, 1], 0.7), _quat([1, 2, 3], -1.1), _quat([0, 1, 0], np.pi / 2)):
        r = rot(q)
        assert np.allclose(r @ r.T, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)
    assert np.allclose(rot(_quat([0, 0, 1], np.pi / 2)) @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(rot([1, 0, 0, 0]), np.eye(3))


def test_level_box_corners_are_the_offset_box():
    corners = _box(centre=(0.1, 0.0, -0.01))._box_corners([1.0, 2.0, 3.0], [1, 0, 0, 0])
    assert np.allclose(corners.min(axis=0), [0.8, 1.8, 2.94])
    assert np.allclose(corners.max(axis=0), [1.4, 2.2, 3.04])


def test_yawed_box_corners_turn_about_the_body_origin():
    corners = _box(centre=(0.1, 0.0, 0.0))._box_corners([0, 0, 0], _quat([0, 0, 1], np.pi / 2))
    # The centre offset along body x lands on world y, and the long side turns onto y.
    assert np.allclose(corners.min(axis=0), [-0.2, -0.2, -0.05])
    assert np.allclose(corners.max(axis=0), [0.2, 0.4, 0.05])


def test_level_footprint_distance_inside_edge_and_corner():
    box = _box()
    corners = box._box_corners([0, 0, 1], [1, 0, 0, 0])[:, :2]
    d = box._footprint_distance([[0, 0], [0.3, 0.0], [1.3, 0.0], [0.0, -0.7], [0.6, 0.6]], corners)
    assert np.allclose(d, [0.0, 0.0, 1.0, 0.5, np.hypot(0.3, 0.4)])


def test_yawed_footprint_distance():
    box = _box(half=(0.5, 0.5, 0.05))
    corners = box._box_corners([0, 0, 1], _quat([0, 0, 1], np.pi / 4))[:, :2]
    # A square turned 45 degrees reaches its corner at sqrt(0.5) along x.
    d = box._footprint_distance([[1.0, 0.0], [0.5, 0.5], [0.69, 0.0]], corners)
    assert np.allclose(d, [1.0 - np.sqrt(0.5), np.sqrt(0.5) - 0.5, 0.0])


def test_tilted_footprint_grows_with_the_box_height():
    box = _box(half=(0.3, 0.2, 0.05))
    pitch = np.pi / 2
    corners = box._box_corners([0, 0, 1], _quat([0, 1, 0], pitch))
    # Pitched upright, the box's height lies along world x and its length along world z.
    assert np.allclose(np.ptp(corners, axis=0), [0.1, 0.4, 0.6])
    d = box._footprint_distance([[1.05, 0.0], [0.0, 0.0]], corners[:, :2])
    assert np.allclose(d, [1.0, 0.0])


def test_iris_box_covers_the_body_and_the_swept_rotor_disks():
    pytest.importorskip("pxr")
    from pxr import Gf, Usd, UsdGeom
    stage = Usd.Stage.Open(str(IRIS))
    box = _box_class(UsdGeom=UsdGeom, Gf=Gf, DRONE_PRIM="/vehicle")()
    box._box, box.stage = None, stage
    centre, half = box._drone_box()
    lo, hi = centre - half, centre + half
    # The body mesh alone spans these bounds in its own frame.
    assert lo[2] < -0.067 + 1e-3 and hi[2] > 0.046
    # Each rotor sweeps a disk of its blade's reach about the rotor origin.
    for i in range(4):
        rotor = stage.GetPrimAtPath(f"/vehicle/rotor{i}")
        x, y, _ = rotor.GetAttribute("xformOp:translate").Get()
        assert lo[0] < x - 0.12 and hi[0] > x + 0.12 and lo[1] < y - 0.12 and hi[1] > y + 0.12
    assert np.all(half < 0.4)
