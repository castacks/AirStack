"""CPU-only tests for R4 camera lookup and pre-capture verification."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as Namespace
import unittest

from capture_r4 import initialize_camera
from recorder import phase

FIXTURE = json.loads((Path(__file__).resolve().parent.parent / "static-isaac-receipt/fixture.json").read_text())
ROOT = Path("/capture-input/stage.usda")
PRODUCT = "/Render/Geometry"
USD_MATRIX = [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 1, 1]]


class Camera:
    def __init__(self, *, matrix=None, focal=20, apertures=(33, 25), offsets=(0, 0)):
        self.valid = True
        self.reads = 0
        self.matrix = deepcopy(USD_MATRIX if matrix is None else matrix)
        self.fields = {"Projection": "perspective", "FocalLength": focal,
                       "HorizontalAperture": apertures[0], "VerticalAperture": apertures[1],
                       "HorizontalApertureOffset": offsets[0], "VerticalApertureOffset": offsets[1]}

    def GetPrim(self):
        return Namespace(IsValid=lambda: self.valid, IsActive=lambda: self.valid,
                         GetAppliedSchemas=lambda: [])

    def TransformMightBeTimeVarying(self):
        return False

    def ComputeLocalToWorldTransform(self, _time):
        if not self.valid:
            raise RuntimeError("stale USD camera")
        self.reads += 1
        return self.matrix

    def GetOrderedXformOps(self):
        raise AssertionError("camera setup accessed a transform-operation handle")

    def __getattr__(self, name):
        if name.startswith("Get") and name.endswith("Attr"):
            key = name[3:-4]
            return lambda: Namespace(Get=lambda: self.fields[key])
        raise AttributeError(name)


class CameraSetupTests(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.output = Path(directory.name)
        self.authored = Camera()
        self.active = Namespace(camera=self.authored, GetRootLayer=lambda: Namespace(realPath=str(ROOT)))
        self.after_create = lambda: None
        self.resolution = (33, 25)
        self.attached = FIXTURE["camera_prim"]
        self.product = Namespace(path=PRODUCT)
        self.lookups = []

    def lookup(self, stage, path):
        self.lookups.append((stage, path))
        return stage.camera

    def create(self, path, resolution):
        self.assertEqual(path, FIXTURE["camera_prim"])
        self.assertEqual(resolution, (33, 25))
        self.after_create()
        return self.product

    def setup_camera(self):
        with phase(self.output, "stage_camera_writers"):
            return initialize_camera(
                self.output, FIXTURE, ROOT, lambda: self.active, self.lookup, self.create,
                lambda _product: self.attached, lambda _product: self.resolution,
            )

    def journal(self, directory="camera-setup"):
        path = self.output / directory / "phases.jsonl"
        return [json.loads(line) for line in path.read_text().splitlines()]

    def test_valid_prim_is_read_again_after_initialization(self):
        stage, camera, product, intrinsics = self.setup_camera()
        self.assertIs(stage, self.active)
        self.assertIs(camera, self.authored)
        self.assertIs(product, self.product)
        self.assertEqual(self.authored.reads, 2)
        self.assertEqual(intrinsics, [[20, 0, 16.5], [0, 20, 12.5], [0, 0, 1]])
        self.assertEqual(len(self.lookups), 2)

    def test_replaced_prim_with_same_pose_is_reacquired(self):
        replacement = Camera()

        def replace():
            self.authored.valid = False
            self.active = Namespace(camera=replacement, GetRootLayer=lambda: Namespace(realPath=str(ROOT)))

        self.after_create = replace
        stage, camera, _, _ = self.setup_camera()
        self.assertIs(stage, self.active)
        self.assertIs(camera, replacement)
        self.assertEqual(self.authored.reads, 1)
        self.assertEqual(replacement.reads, 1)

    def test_stale_transform_op_is_never_read_or_written(self):
        def stale_ops():
            raise AssertionError("invalidated xform ops must not be used")

        self.authored.GetOrderedXformOps = stale_ops
        self.authored.SetXformOpOrder = stale_ops
        self.authored.GetXformOpOrderAttr = stale_ops
        self.setup_camera()

    def test_missing_or_invalid_effective_camera_is_rejected(self):
        for candidate in (None, Camera()):
            with self.subTest(candidate=candidate), TemporaryDirectory() as directory:
                self.output = Path(directory)
                if candidate is not None:
                    candidate.valid = False
                self.after_create = lambda: setattr(self.active, "camera", candidate)
                self.active.camera = self.authored
                with self.assertRaisesRegex(RuntimeError, "missing or invalid"):
                    self.setup_camera()
                self.assertEqual(self.journal()[-1]["phase"], "effective_camera_pose")

    def test_missing_authored_camera_does_not_initialize(self):
        self.active.camera = None
        self.after_create = lambda: self.fail("unexpected render product initialization")
        with self.assertRaisesRegex(RuntimeError, "missing or invalid"):
            self.setup_camera()
        self.assertEqual(self.journal()[-1]["phase"], "authored_camera")

    def test_stage_with_different_root_is_rejected(self):
        self.after_create = lambda: setattr(self.active, "GetRootLayer",
                                            lambda: Namespace(realPath="/other.usda"))
        with self.assertRaisesRegex(RuntimeError, "pinned root layer"):
            self.setup_camera()

    def test_pose_change_is_rejected_without_reapplying_it(self):
        for row, column, delta in ((3, 0, .001), (1, 1, 3.42e-8)):
            with self.subTest(row=row), TemporaryDirectory() as directory:
                self.output = Path(directory)
                self.authored.matrix = deepcopy(USD_MATRIX)
                self.after_create = lambda: self.authored.matrix[row].__setitem__(column, delta)
                with self.assertRaisesRegex(ValueError, "effective camera pose differs"):
                    self.setup_camera()
                observed = json.loads((self.output / "camera-setup/effective-pose.json").read_text())
                self.assertEqual(observed["usd_local_to_world"][row][column], delta)
                self.assertEqual(self.authored.matrix[row][column], delta)

    def test_invalid_or_inconsistent_intrinsics_are_rejected(self):
        cases = [(Camera(focal=float("nan")), (33, 25), FIXTURE["camera_prim"], "invalid stationary"),
                 (Camera(apertures=(33, 0)), (33, 25), FIXTURE["camera_prim"], "invalid stationary"),
                 (Camera(offsets=(.1, 0)), (33, 25), FIXTURE["camera_prim"], "invalid stationary"),
                 (Camera(focal=21), (33, 25), FIXTURE["camera_prim"], "effective camera intrinsics"),
                 (Camera(), (33, 24), FIXTURE["camera_prim"], "render product camera or resolution"),
                 (Camera(), (33, 25), "/WrongCamera", "render product camera or resolution")]
        for camera, resolution, attached, message in cases:
            with self.subTest(message=message, attached=attached), TemporaryDirectory() as directory:
                self.output = Path(directory)
                self.active.camera = camera
                self.attached = attached
                self.resolution = resolution
                with self.assertRaisesRegex(ValueError, message):
                    self.setup_camera()
                self.assertEqual(self.journal()[-1]["phase"], "effective_camera_intrinsics")

    def test_camera_setup_exception_is_persisted_at_both_levels(self):
        def failed_product():
            raise RuntimeError("injected render product exception")

        self.after_create = failed_product
        with self.assertRaisesRegex(RuntimeError, "injected render product exception"):
            self.setup_camera()
        last = self.journal()[-1]
        self.assertEqual((last["phase"], last["status"]), ("initialize_render_product", "failed"))
        self.assertIn("Traceback (most recent call last)", last["traceback"])
        self.assertIn("injected render product exception", last["traceback"])
        outer = self.journal(".")[-1]
        self.assertEqual((outer["phase"], outer["status"]), ("stage_camera_writers", "failed"))
        self.assertTrue((self.output / "camera-setup/phases.jsonl").read_bytes().endswith(b"\n"))

    def test_success_persists_authored_pose_and_intrinsics(self):
        self.setup_camera()
        setup = self.output / "camera-setup"
        authored = json.loads((setup / "authored.json").read_text())
        effective = json.loads((setup / "effective-pose.json").read_text())
        self.assertEqual(authored["usd_local_to_world"], effective["usd_local_to_world"])
        self.assertEqual(authored["expected_usd_local_to_world"], USD_MATRIX)
        verified = self.journal()[-2]
        self.assertEqual(verified["status"], "verified")
        self.assertEqual(verified["intrinsics_native"][0], [20, 0, 16.5])


if __name__ == "__main__":
    unittest.main()
