"""Hermetic request and same-render payload checks; no Isaac or ROS runtime."""
import importlib.util
import ast
import os
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np

spec = importlib.util.spec_from_file_location("office_camera_teacher", Path(__file__).parents[1] /
    "office_camera_teacher.py")
teacher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teacher)


class Requests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name)
        self.gate = teacher.CaptureRequest(self.path)

    def request(self, capture_id="first", enabled=True):
        (self.path / "office-camera-request.json").write_text(json.dumps(
            dict(capture_id=capture_id, enabled=enabled)))

    def test_idle_does_not_create_capture(self):
        self.gate.poll(0)
        self.assertIsNone(self.gate.active)
        self.assertEqual(list(self.path.iterdir()), [])

    def test_frame_cap_and_new_id_recovery(self):
        self.request(); self.gate.poll(10)
        self.gate.count = 3; self.gate.poll(11)
        self.assertIsNone(self.gate.active)
        self.assertEqual(self.gate.reason, "frame_limit")
        self.gate.poll(12)
        self.assertIsNone(self.gate.active)
        self.request("second"); self.gate.poll(13)
        self.assertEqual(self.gate.active, "second")
        self.assertEqual(self.gate.count, 0)

    def test_duration_cap(self):
        self.request(); self.gate.poll(10); self.gate.poll(25)
        self.assertIsNone(self.gate.active)
        self.assertEqual(self.gate.reason, "duration_limit")

    def test_disabled_and_missing_request_stop(self):
        self.request(); self.gate.poll(10)
        self.request(enabled=False); self.gate.poll(11)
        self.assertIsNone(self.gate.active)
        self.request("second"); self.gate.poll(12)
        (self.path / "office-camera-request.json").unlink(); self.gate.poll(13)
        self.assertIsNone(self.gate.active)

    def test_existing_capture_preserved(self):
        output = self.path / "office-camera-first"
        output.mkdir(); (output / "retained").write_text("old")
        self.request()
        with self.assertRaises(FileExistsError):
            self.gate.poll(10)
        self.assertEqual((output / "retained").read_text(), "old")

    def test_invalid_and_oversized_request(self):
        for name, enabled in [("../escape", True), ("valid", 1), ("", True)]:
            self.request(name, enabled)
            with self.assertRaises(ValueError):
                self.gate.poll(10)
        (self.path / "office-camera-request.json").write_text(" " * 4097)
        with self.assertRaises(ValueError):
            self.gate.poll(10)

    def test_poll_detaches_at_cap_without_new_capture(self):
        observer = teacher.OfficeCameraTeacher(self.path, None, launcher_sha256="source")
        writer = Mock(capture_id="first")
        observer._attach = Mock(side_effect=lambda: setattr(observer, "writer", writer))
        self.request()
        with patch.object(teacher.time, "monotonic", return_value=10):
            observer.poll()
        observer.requests.count = 3
        with patch.object(teacher.time, "monotonic", return_value=11):
            observer.poll()
        writer.detach.assert_called_once()
        self.assertIsNone(observer.writer)
        self.assertEqual(observer.requests.reason, "frame_limit")

    def test_attach_failure_contained_and_same_id_not_retried(self):
        observer = teacher.OfficeCameraTeacher(self.path, None, launcher_sha256="source", warn=Mock())
        observer._attach = Mock(side_effect=RuntimeError("native attach failed"))
        self.request()
        with patch.object(teacher.time, "monotonic", return_value=10):
            observer.poll()
        self.assertIsNone(observer.requests.active)
        self.assertEqual(observer.requests.reason, "error")
        self.assertEqual(observer.error, "native attach failed")
        with patch.object(teacher.time, "monotonic", return_value=11):
            observer.poll()
        observer._attach.assert_called_once()


class Payload(unittest.TestCase):
    def payload(self):
        return dict(rgb=np.zeros((2, 3, 4), dtype=np.uint8),
            semantic_segmentation=dict(data=np.zeros((2, 3), dtype=np.uint32),
                info=dict(idToLabels={0: {"class": "BACKGROUND"}})),
            ReferenceTime=dict(referenceTimeNumerator=1, referenceTimeDenominator=3),
            IsaacReadSimulationTime=dict(simulationTime=.25),
            camera_params=dict(cameraViewTransform=np.eye(4).flatten()))

    def test_integer_reference_and_rgb_hash(self):
        value = self.payload()
        rgb, mask, record = teacher.payload_record(value)
        self.assertEqual(record["source_stamp_ns"], 333333333)
        self.assertEqual(record["ros_source_stamp_ns"], 250000000)
        self.assertEqual(record["ros_simulation_time_s"], .25)
        self.assertEqual(rgb.shape, (2, 3, 3))
        self.assertEqual(mask.shape, (2, 3))
        value["rgb"][:, :, 3] = 255
        self.assertEqual(teacher.payload_record(value)[2]["rgb_sha256"], record["rgb_sha256"])

    def test_invalid_rgb_segmentation_reference_and_pose_rejected(self):
        for field in ("rgb", "mask", "time", "pose"):
            value = self.payload()
            if field == "rgb": value["rgb"] = np.zeros((2, 3, 3), dtype=float)
            if field == "mask": value["semantic_segmentation"]["data"] = np.zeros((3, 3), dtype=np.uint32)
            if field == "time": value["ReferenceTime"]["referenceTimeDenominator"] = 0
            if field == "pose": value["camera_params"]["cameraViewTransform"][0] = np.nan
            with self.assertRaises(ValueError):
                teacher.payload_record(value)

    def test_ros_clock_missing_invalid_and_distinct_from_render_reference(self):
        for clock in (True, "1", -1., float("nan"), float("inf")):
            value = self.payload()
            value["IsaacReadSimulationTime"]["simulationTime"] = clock
            with self.assertRaises(ValueError):
                teacher.payload_record(value)
        value = self.payload()
        del value["IsaacReadSimulationTime"]
        with self.assertRaises(KeyError):
            teacher.payload_record(value)


class Poses(unittest.TestCase):
    def test_usd_row_pose_and_optical_axes(self):
        row = np.eye(4)
        row[3, :3] = [20, 6, -4]
        result = teacher.usd_pose_matrix(row, .01, optical=True)
        np.testing.assert_allclose(result[:3, 3], [.2, .06, -.04])
        np.testing.assert_allclose(result[:3, :3], np.diag([1, -1, -1]))
        np.testing.assert_allclose(teacher.usd_pose_matrix(row, .01)[:3, :3], np.eye(3))

    def test_invalid_usd_pose_rejected(self):
        for matrix, scale in [(np.eye(3), 1), (np.full((4, 4), np.nan), 1),
                              (np.eye(4), 0), (np.eye(4), float("nan"))]:
            with self.assertRaises(ValueError):
                teacher.usd_pose_matrix(matrix, scale)

    def test_mount_composition_separates_estimate_from_extrinsics(self):
        true_base = teacher.pose_matrix([0, 0, .07], [0, 0, 0, 1])
        estimated_base = teacher.pose_matrix([0, 0, 0], [0, 0, 0, 1])
        mount = teacher.pose_matrix([.2, .06, -.04], [0, 0, 0, 1])
        native_camera, ros_camera = true_base @ mount, estimated_base @ mount
        self.assertAlmostEqual(teacher.pose_errors(native_camera, ros_camera)["translation_error_m"], .07)
        self.assertAlmostEqual(teacher.pose_errors(np.linalg.inv(true_base) @ native_camera, mount)["translation_error_m"], 0)

    def test_rotation_and_sign_equivalence(self):
        identity = teacher.pose_matrix([0, 0, 0], [0, 0, 0, 1])
        quarter = teacher.pose_matrix([0, 0, 0], [0, 0, 2**-.5, 2**-.5])
        self.assertAlmostEqual(teacher.pose_errors(identity, quarter)["rotation_error_degrees"], 90)
        opposite = teacher.pose_matrix([0, 0, 0], [0, 0, -2**-.5, -2**-.5])
        self.assertAlmostEqual(teacher.pose_errors(quarter, opposite)["rotation_error_degrees"], 0)

    def test_bad_pose_rejected(self):
        for position, quaternion in [([0, 0], [0, 0, 0, 1]), ([0, 0, float("nan")], [0, 0, 0, 1]),
                                     ([0, 0, 0], [0, 0, 0, 0])]:
            with self.assertRaises(ValueError):
                teacher.pose_matrix(position, quaternion)


class AuthoredBodyFrame(unittest.TestCase):
    def check_constructor_frame(self, authored, expected, *, existing=False):
        # Execute the real constructor's frame-cache and Robot-init statements,
        # stubbing only native boundaries. Robot initialization deliberately tilts
        # the body: the regression used to capture that live transform as authored.
        from scipy.spatial.transform import Rotation
        path = Path(__file__).parents[2] / "extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/vehicle.py"
        module = ast.parse(path.read_text())
        vehicle = next(item for item in module.body if isinstance(item, ast.ClassDef) and item.name == "Vehicle")
        constructor = next(item for item in vehicle.body if isinstance(item, ast.FunctionDef) and item.name == "__init__")
        selected = []
        for statement in constructor.body:
            text = ast.unparse(statement)
            if (text.startswith("self._body_local_rotation_inv =") or text.startswith("body_prim =")
                    or text.startswith("if body_prim.IsValid()") or text.startswith("if not spawn_prim and usd_path:")
                    or text.startswith("super().__init__(")):
                selected.append(statement)
        constructor.body = selected
        vehicle.body = [constructor]
        vehicle.bases = [ast.Name(id="Robot", ctx=ast.Load())]
        unit = ast.fix_missing_locations(ast.Module(body=[vehicle], type_ignores=[]))
        body = SimpleNamespace(quaternion=np.asarray(authored))
        source_body = SimpleNamespace(quaternion=np.asarray(authored))
        body.IsValid = lambda: True
        source_body.IsValid = lambda: True
        def local_transform(prim):
            quaternion = prim.quaternion.copy()
            quat = SimpleNamespace(GetImaginary=lambda: quaternion[:3], GetReal=lambda: quaternion[3])
            return SimpleNamespace(ExtractRotation=lambda: SimpleNamespace(GetQuaternion=lambda: quat))
        stage = SimpleNamespace(GetPrimAtPath=lambda _: body)
        root = SimpleNamespace(IsValid=lambda: True,
            GetPath=lambda: SimpleNamespace(AppendChild=lambda _: "/vehicle/body"))
        source_stage = SimpleNamespace(GetDefaultPrim=lambda: root, GetPrimAtPath=lambda _: source_body)
        class Robot:
            def __init__(self, **kwargs):
                body.quaternion = Rotation.from_euler("y", 8, degrees=True).as_quat()
        scope = dict(Robot=Robot, Rotation=Rotation, np=np, carb=Mock(), os=os,
                     Usd=SimpleNamespace(Stage=SimpleNamespace(Open=lambda _: source_stage)),
                     UsdGeom=SimpleNamespace(Xformable=lambda prim:
                         SimpleNamespace(GetLocalTransformation=lambda: local_transform(prim))))
        exec(compile(unit, str(path), "exec"), scope)
        instance = scope["Vehicle"].__new__(scope["Vehicle"])
        instance._current_stage, instance._stage_prefix = stage, "/vehicle"
        if existing:
            body.quaternion = Rotation.from_euler("y", 8, degrees=True).as_quat()
        instance.__init__("/vehicle", usd_path="authored.usd", spawn_prim=not existing)
        np.testing.assert_allclose(instance._body_local_rotation_inv.as_quat(), expected, atol=1e-12)

    def test_live_tilt_never_becomes_authored_correction(self):
        self.check_constructor_frame([0, 0, 0, 1], [0, 0, 0, 1])

    def test_real_authored_rotation_still_compensated(self):
        self.check_constructor_frame([0, 0, 2**-.5, 2**-.5], [0, 0, -2**-.5, 2**-.5])

    def test_existing_simulated_prim_uses_source_asset_not_live_tilt(self):
        self.check_constructor_frame([0, 0, 0, 1], [0, 0, 0, 1], existing=True)


if __name__ == "__main__":
    unittest.main()
