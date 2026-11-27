import copy
from pathlib import Path
import struct
import tempfile
import unittest

import verify as audit


class ReceiptTest(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.folder = Path(scratch.name)
        self.lock_path = self.folder / "lock.json"
        self.receipt_path = self.folder / "receipt.json"
        self.fixture = f = audit.read_json(audit.FIXTURE)
        self.lock = {
            "schema_version": 1, "kind": "synthetic_test",
            "fixture_sha256": audit.digest(audit.FIXTURE.read_bytes()),
            "isaac_version": "5.1.0", "image_sha256": "1" * 64,
            "airstack_commit": "2" * 40, "pegasus_commit": "3" * 40,
            "renderer": "synthetic placeholder",
            "stage": {key: f[key] for key in ("meters_per_unit", "up_axis", "camera_prim",
                    "target_center_m", "target_normal", "target_half_extents_xz_m")}
                    | {"only_renderable_prim": f["target_prim"]},
            "control_axial_m": [5.0, 15.0],
            "files": self.files(("stage", "exporter", "preflight")),
        }
        self.receipt = {
            "schema_version": 1, "kind": "synthetic_test",
            "clock": {
                "source": "IsaacReadSimulationTime:/clock", "epoch_ns": 0,
                "period_ns": 100_000_000, "samples_ns": [i * 100_000_000 for i in range(10, 31)],
            },
            "depth_annotator": "distance_to_image_plane",
            "issue": self.observation(10), "target": self.observation(30),
            "files": self.files(("clock", "issue", "target")),
        }

    def files(self, roles):
        result = {}
        for role in roles:
            path = self.folder / f"{role}.txt"
            data = f"synthetic placeholder for {role}\n".encode()
            path.write_bytes(data)
            result[role] = {"path": path.name, "sha256": audit.digest(data)}
        return result

    def observation(self, tick):
        f = self.fixture
        stamp = tick * f["tick_period_ns"]
        pose = {"origin_m": f["origin_m"], "optical_to_stage": f["optical_to_stage"]}
        return {
            "tick": tick, "pose_stamp_ns": stamp, "depth_stamp_ns": stamp,
            "depth_frame": f["camera_frame"],
            "tf": {"stamp_ns": stamp, "parent": f["stage_frame"], "child": f["camera_frame"], "pose": pose},
            "pose": pose, "ray_origin_m": f["origin_m"], "ray_direction": [0, 1, 0],
            "camera_info": {"stamp_ns": stamp, "frame": f["camera_frame"], "intrinsics": f["intrinsics"]},
            "pixels": [{"uv": uv, "axial_m": 10.0, "first_hit": f["target_prim"]} for uv in f["pixels"]],
        }

    def write(self, receipt=None, lock=None):
        lock = self.lock if lock is None else lock
        receipt = self.receipt if receipt is None else receipt
        self.lock_path.write_bytes(audit.canonical(lock))
        pin = audit.digest(self.lock_path.read_bytes())
        self.receipt_path.write_bytes(audit.canonical(receipt | {"lock_sha256": pin}))
        return pin

    def run_verify(self, receipt=None, *, synthetic=True):
        return audit.verify(self.receipt_path, self.lock_path, self.write(receipt), synthetic=synthetic)

    def test_synthetic_pass_and_exact_adapter_binding(self):
        result = self.run_verify()
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["kind"], "synthetic_test")
        self.assertEqual(result["target_ns"] - result["issue_binding"]["issue_ns"], 2_000_000_000)
        self.assertEqual(result["issue_binding"]["geometry"]["source_m"], [10, 0, 0])
        self.assertAlmostEqual(result["issue_ranges"][1]["analytic_range_m"], (101)**.5)
        self.assertEqual(result, self.run_verify())
        with self.assertRaisesRegex(ValueError, "receipt kind not allowed"):
            self.run_verify(synthetic=False)

    def test_target_only_change_does_not_change_issue_binding(self):
        a = self.run_verify()
        altered = copy.deepcopy(self.receipt)
        altered["target"]["pixels"][0]["axial_m"] = struct.unpack("<f", struct.pack("<I", 0x41200001))[0]
        b = self.run_verify(altered)
        self.assertNotEqual(a["canonical_receipt_sha256"], b["canonical_receipt_sha256"])
        self.assertEqual(a["issue_binding_sha256"], b["issue_binding_sha256"])

    def test_wrong_pin_and_changed_stage_bytes(self):
        pin = self.write()
        with self.assertRaisesRegex(ValueError, "lock hash"):
            audit.verify(self.receipt_path, self.lock_path, "0"*64, synthetic=True)
        (self.folder / "stage.txt").write_text("tampered stage")
        with self.assertRaisesRegex(ValueError, "digest mismatch"):
            audit.verify(self.receipt_path, self.lock_path, pin, synthetic=True)

    def test_reference_range_guards(self):
        for bad, error in (
            ({"first_hit": "/World/Occluder"}, "first-hit"),
            ({"axial_m": 10.05}, "binary32"),
            ({"axial_m": struct.unpack("<f", struct.pack("<f", 101**.5))[0]}, "disagrees with geometry"),
        ):
            with self.subTest(bad=bad):
                altered = copy.deepcopy(self.receipt)
                altered["issue"]["pixels"][1].update(bad)
                with self.assertRaisesRegex(ValueError, error):
                    self.run_verify(altered)

    def test_bad_clock_pose_intrinsics_and_tf(self):
        for field, value, error in (
            ("depth_stamp_ns", 999_999_999, "not tied"),
            ("ray_direction", [0, 2, 0], "unit length"),
            ("ray_direction", [1, 0, 0], "disagrees with camera"),
            ("depth_frame", "camera_left", "depth frame"),
        ):
            with self.subTest(field=field, value=value):
                altered = copy.deepcopy(self.receipt)
                altered["issue"][field] = value
                with self.assertRaisesRegex(ValueError, error):
                    self.run_verify(altered)
        altered = copy.deepcopy(self.receipt)
        altered["target"]["tf"]["stamp_ns"] -= 1
        with self.assertRaisesRegex(ValueError, "exact-time camera TF"):
            self.run_verify(altered)
        altered = copy.deepcopy(self.receipt)
        altered["clock"]["samples_ns"].pop(5)
        with self.assertRaisesRegex(ValueError, "simulation stamps"):
            self.run_verify(altered)
        altered = copy.deepcopy(self.receipt)
        altered["issue"]["pose"] = {"origin_m": [0, 0, 1], "optical_to_stage": [[1, 0, 0],
                                        [0, 1, 0], [0, 0, 1]]}
        with self.assertRaisesRegex(ValueError, "optical-to-stage"):
            self.run_verify(altered)

    def test_unsupported_depth_and_bad_calibration(self):
        altered = self.receipt | {"depth_annotator": "distance_to_camera"}
        with self.assertRaisesRegex(ValueError, "depth convention"):
            self.run_verify(altered)
        self.lock["control_axial_m"][1] = 15.125
        with self.assertRaisesRegex(ValueError, "M11 range ceiling"):
            self.run_verify()

    def test_duplicate_key_and_file_escape_rejected(self):
        self.receipt_path.write_text('{"kind": "first", "kind": "second"}')
        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            audit.read_json(self.receipt_path)
        self.receipt["files"]["issue"]["path"] = "../outside"
        with self.assertRaisesRegex(ValueError, "inside bundle"):
            self.run_verify()


if __name__ == "__main__":
    unittest.main()
