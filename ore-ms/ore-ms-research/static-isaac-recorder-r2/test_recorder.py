"""CPU-only subprocess tests. No Isaac modules are loaded from disk."""

import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import ModuleType
import unittest

import recorder


def mock_worker(output, fault):
    print(f"mock child: {fault}", flush=True)
    if fault == "startup-exception":
        fake_isaac = ModuleType("isaacsim")

        def fail(_settings):
            raise RuntimeError("injected SimulationApp initialization failure")

        fake_isaac.SimulationApp = fail
        sys.modules["isaacsim"] = fake_isaac
        import capture_one

        capture_one.OUTPUT = output
        capture_one.capture()
        return

    if fault == "masked-exception":
        try:
            with recorder.phase(output, "isaac_startup"):
                raise RuntimeError("exception before simulated native shutdown")
        finally:
            os._exit(0)

    for name in recorder.PHASES:
        with recorder.phase(output, name):
            if name == "camera_initialize" and fault == "signal":
                os.kill(os.getpid(), signal.SIGTERM)
            if name == "camera_initialize" and fault == "timeout":
                signal.pause()
            if name == "camera_initialize" and fault == "early":
                os._exit(0)
            if name == "frame_readout" and fault == "readout-exception":
                raise RuntimeError("injected annotator failure")
            if name == "write_artifacts":
                write_mock_frame(output, fault)
            if name == "shutdown" and fault == "shutdown-exit":
                os._exit(0)


def write_mock_frame(output, fault):
    metadata = {
        "schema": 1, "source": "mock", "width": recorder.WIDTH, "height": recorder.HEIGHT,
        "rendering_frame": 3, "rendering_time_s": .3,
        "camera_prim": recorder.CAMERA, "target_prim": recorder.TARGET,
        "stage_meters_per_unit": 1, "stage_up_axis": "Z",
        "intrinsics_native": [[20, 0, 16.5], [0, 20, 12.5], [0, 0, 1]],
        "camera_local_to_world_usd": [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 1, 1]],
        "depth_annotator": "distance_to_image_plane", "instance_annotator": "instance_id_segmentation",
        "depth_native_dtype": "float32", "instance_native_dtype": "uint32",
        "id_to_labels": {"8": recorder.TARGET},
    }
    if fault == "bad-stamp":
        metadata["rendering_time_s"] = "wall-clock"
    if fault == "bad-labels":
        metadata["id_to_labels"] = {}
    if fault != "no-metadata":
        recorder.write_json(output / "frame.json", metadata)
    size = recorder.WIDTH * recorder.HEIGHT
    if fault != "no-depth":
        depth = struct.pack("<f", float("inf")) * size
        if fault != "all-invalid":
            depth = struct.pack("<f", 10.0) + depth[4:]
        recorder.write_artifact(output / "depth.f32le", b"" if fault == "empty-depth" else depth)
    if fault != "no-instance":
        ids = struct.pack("<I", 8) + struct.pack("<I", 0) * (size - 1)
        recorder.write_artifact(output / "instances.u32le", ids[:-1] if fault == "short-instance" else ids)


class RecorderTests(unittest.TestCase):
    def run_worker(self, fault, mode="mock", timeout="5"):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        output = Path(directory.name) / "run"
        command = [sys.executable, recorder.__file__, "--output", str(output),
                   "--timeout", timeout, "--mode", mode, "--", sys.executable, "-u",
                   __file__, "--mock-worker", str(output / "capture"), fault]
        child = subprocess.run(command, capture_output=True, timeout=10, check=False)
        self.assertIn(f"mock child: {fault}", (output / "child.log").read_text())
        return child, output, command

    def assert_failed(self, fault, message, mode="mock", timeout="5"):
        child, output, _ = self.run_worker(fault, mode, timeout)
        self.assertEqual(child.returncode, 1, child.stderr.decode())
        self.assertFalse((output / "COMPLETED.json").exists())
        result = json.loads((output / "FAILED.json").read_text())
        self.assertIn(message, result["traceback"])
        return output, result

    def test_valid_mock_and_hashes(self):
        child, output, command = self.run_worker("valid")
        self.assertEqual(child.returncode, 0, child.stderr.decode())
        completion = (output / "COMPLETED.json").read_bytes()
        result = json.loads(completion)
        self.assertEqual(result["mode"], "mock")
        self.assertEqual(len(result["sha256"]), 5)
        for name, digest in result["sha256"].items():
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
        self.assertFalse((output / "FAILED.json").exists())
        repeated = subprocess.run(command, capture_output=True, timeout=10, check=False)
        self.assertNotEqual(repeated.returncode, 0)
        self.assertEqual((output / "COMPLETED.json").read_bytes(), completion)

    def test_mock_cannot_complete_in_isaac_mode(self):
        self.assert_failed("valid", "unexpected frame schema or source", mode="isaac")

    def test_clean_early_exit(self):
        output, result = self.assert_failed("early", "did not complete every phase")
        self.assertEqual(result["returncode"], 0)
        last_event = json.loads((output / "capture/phases.jsonl").read_text().splitlines()[-1])
        self.assertEqual((last_event["phase"], last_event["status"]), ("camera_initialize", "started"))

    def test_exit_with_all_frames_before_shutdown_returns(self):
        output, result = self.assert_failed("shutdown-exit", "did not complete every phase")
        self.assertEqual(result["returncode"], 0)
        self.assertTrue((output / "capture/depth.f32le").exists())
        self.assertTrue((output / "capture/instances.u32le").exists())

    def test_missing_artifacts(self):
        for fault, name in (("no-depth", "depth.f32le"), ("no-instance", "instances.u32le"),
                            ("no-metadata", "frame.json")):
            with self.subTest(fault=fault):
                self.assert_failed(fault, name)

    def test_invalid_artifacts(self):
        for fault, message in (("empty-depth", "complete frame"), ("short-instance", "complete frame"),
                               ("all-invalid", "no finite positive depth pixel"),
                               ("bad-labels", "no finite positive depth pixel"),
                               ("bad-stamp", "invalid native render timestamp")):
            with self.subTest(fault=fault):
                self.assert_failed(fault, message)

    def test_exceptions_persist_in_worker_journal(self):
        for fault, phase, message in (
            ("startup-exception", "isaac_startup", "injected SimulationApp initialization failure"),
            ("readout-exception", "frame_readout", "injected annotator failure"),
        ):
            with self.subTest(fault=fault):
                output, _ = self.assert_failed(fault, "worker command exited with status 1")
                events = (output / "capture/phases.jsonl").read_text().splitlines()
                last_event = json.loads(events[-1])
                self.assertEqual((last_event["phase"], last_event["status"]), (phase, "failed"))
                self.assertIn("Traceback (most recent call last)", last_event["traceback"])
                self.assertIn(message, last_event["traceback"])

    def test_exception_survives_clean_native_exit(self):
        output, result = self.assert_failed("masked-exception", "did not complete every phase")
        self.assertEqual(result["returncode"], 0)
        events = (output / "capture/phases.jsonl").read_text().splitlines()
        failure = json.loads(events[-1])
        self.assertEqual(failure["status"], "failed")
        self.assertIn("Traceback (most recent call last)", failure["traceback"])
        self.assertIn("exception before simulated native shutdown", failure["traceback"])

    def test_signal(self):
        _, result = self.assert_failed("signal", "worker command exited with status -15")
        self.assertEqual(result["returncode"], -signal.SIGTERM)

    def test_timeout(self):
        _, result = self.assert_failed("timeout", "TimeoutExpired", timeout="1")
        self.assertIsNone(result["returncode"])


if __name__ == "__main__":
    if sys.argv[1:2] == ["--mock-worker"]:
        mock_worker(Path(sys.argv[2]), sys.argv[3])
    else:
        unittest.main()
