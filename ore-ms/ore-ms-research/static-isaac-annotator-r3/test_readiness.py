"""Exercise R3 with CPU-only writer and array test doubles."""

import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace as Namespace
import unittest

from capture_one_r3 import attach_writers
from readiness import DEPTH, INSTANCE, MAX_ATTEMPTS, UNVERIFIED, acquire, writer_packet
import recorder
from supervise_r3 import PHASES


class Array:
    def __init__(self, dtype, data, shape=(recorder.HEIGHT, recorder.WIDTH)):
        self.dtype = Namespace(name=dtype)
        self.shape = shape
        self.data = data
        self.size = len(data) // 4

    def astype(self, dtype, *, copy):
        assert dtype == {"float32": "<f4", "uint32": "<u4"}[self.dtype.name]
        assert copy is False
        return self

    def tobytes(self):
        return self.data


def payload(annotator, index=9):
    native = {"referenceTimeNumerator": index, "referenceTimeDenominator": 60}
    count = recorder.WIDTH * recorder.HEIGHT
    if annotator == DEPTH:
        data = Array("float32", struct.pack("<f", 10) * count)
    else:
        data = {"data": Array("uint32", struct.pack("<I", 1) * count),
                "info": {"idToLabels": json.dumps({"1": recorder.TARGET})}}
    return {"ReferenceTime": native, annotator: data}


def packets(depth_index=9, instance_index=9):
    depth = {} if depth_index is None else payload(DEPTH, depth_index)
    instances = {} if instance_index is None else payload(INSTANCE, instance_index)
    return writer_packet(depth, DEPTH), writer_packet(instances, INSTANCE)


class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.output = Path(scratch.name)
        self.calls = 0

    def run_acquire(self, samples, sim_time=lambda reference: reference[0] / reference[1]):
        samples = iter(samples)

        def step():
            self.calls += 1
            return next(samples)

        return acquire(self.output, step, sim_time)

    def events(self):
        stream = self.output / "readiness.jsonl"
        return [json.loads(line) for line in stream.read_text().splitlines()]

    def test_immediate_native_writer_pair(self):
        class Writer:
            def add_annotator(self, annotator, *, name):
                self.__dict__.setdefault("annotators", {})[name] = annotator

            def initialize(self):
                pass

            def attach(self, products):
                self.products = products

        classes = {}
        rep = Namespace(
            Writer=Writer,
            WriterRegistry=Namespace(register=lambda cls: classes.update({cls.__name__: cls}),
                                     get=lambda name: classes[name]()),
            AnnotatorRegistry=Namespace(get_annotator=lambda name, **params: (name, params)),
        )
        depth, ids = attach_writers(rep, "/Render/R3")
        self.assertEqual(depth.products, ids.products)
        self.assertEqual(set(depth.annotators), {DEPTH, "ReferenceTime"})
        self.assertEqual(ids.annotators[INSTANCE][1]["init_params"], {"colorize": False})
        self.assertEqual(set(ids.annotators), {INSTANCE, "ReferenceTime"})
        depth.write(payload(DEPTH))
        ids.write(payload(INSTANCE))
        frame = self.run_acquire([(writer_packet(depth.packet, DEPTH),
                                   writer_packet(ids.packet, INSTANCE))])
        self.assertEqual(frame.association["depth_reference"], [9, 60])
        self.assertEqual(frame.association["instance_reference"], [9, 60])
        self.assertEqual(frame.association["rendering_time_s"], .15)
        self.assertEqual(self.calls, 1)

    def test_delayed_instances(self):
        result = self.run_acquire([packets(7, None), packets(8, None), packets()])
        self.assertEqual(result.association["attempt"], 3)
        self.assertEqual([event["instances"]["status"] for event in self.events()[1::2]],
                         ["missing", "missing", "valid"])

    def test_delayed_depth(self):
        result = self.run_acquire([packets(None, 7), packets(None, 8), packets()])
        self.assertEqual(result.association["attempt"], 3)
        self.assertEqual([event["depth"]["status"] for event in self.events()[1::2]],
                         ["missing", "missing", "valid"])

    def test_different_frames_never_combined(self):
        samples = [packets(8, None), packets(None, 8)] + [packets(8, 9)] * (MAX_ATTEMPTS - 2)
        with self.assertRaisesRegex(RuntimeError, UNVERIFIED):
            self.run_acquire(samples)
        self.assertEqual(self.events()[-1]["status"], "FRAME_MISMATCH")

    def test_permanently_missing_annotator(self):
        with self.assertRaisesRegex(RuntimeError, "READINESS_BOUND_EXHAUSTED"):
            self.run_acquire([packets(9, None)] * MAX_ATTEMPTS)
        self.assertEqual(self.events()[-1]["instances"]["status"], "missing")
        self.assertEqual(self.calls, MAX_ATTEMPTS)

    def test_empty_and_malformed_payloads(self):
        empty, valid_ids = packets()
        empty.array = Array("float32", b"")
        malformed, _ = packets()
        malformed.array = "not a native array"
        self.run_acquire([(empty, valid_ids), (malformed, valid_ids), packets()])
        for event in self.events()[1:4:2]:
            self.assertEqual(event["depth"]["status"], "empty_or_malformed")

    def test_mismatched_dimensions(self):
        depth, ids = packets()
        ids.array.shape = (recorder.HEIGHT, recorder.WIDTH - 1)
        with self.assertRaisesRegex(RuntimeError, "READINESS_BOUND_EXHAUSTED"):
            self.run_acquire([(depth, ids)] * MAX_ATTEMPTS)
        self.assertEqual(self.events()[-1]["instances"]["status"], "unexpected_dimensions")

    def test_exhaustion_after_exact_bound(self):
        with self.assertRaisesRegex(RuntimeError, "after 8 render/update attempts"):
            self.run_acquire([packets(None, None)] * MAX_ATTEMPTS + [packets()])
        self.assertEqual(self.calls, MAX_ATTEMPTS)
        self.assertEqual(len(self.events()), 2 * MAX_ATTEMPTS)

    def test_missing_stamp_rejects_nonempty_buffers(self):
        untagged = payload(INSTANCE)
        del untagged["ReferenceTime"]
        untagged["rendering_time_s"] = .15
        pair = (packets()[0], writer_packet(untagged, INSTANCE))
        with self.assertRaisesRegex(RuntimeError, UNVERIFIED):
            self.run_acquire([pair] * MAX_ATTEMPTS)
        self.assertIsNone(self.events()[-1]["instances"]["reference"])

    def test_unmapped_native_time_rejected(self):
        with self.assertRaisesRegex(RuntimeError, UNVERIFIED):
            self.run_acquire([packets()] * MAX_ATTEMPTS, sim_time=lambda _reference: -1)

    def test_association_and_finite_target_depth_both_required(self):
        depth, ids = packets()
        depth.array.data = struct.pack("<f", float("inf")) * depth.array.size
        frame = self.run_acquire([packets(7, 8), (depth, ids), packets()])
        statuses = [event["status"] for event in self.events()[1::2]]
        self.assertEqual(statuses, ["FRAME_MISMATCH", "NO_VALID_TARGET_DEPTH", "READY"])
        self.assertEqual(frame.association["attempt"], 3)

    def test_readout_exception_persists(self):
        def broken_source():
            raise ValueError("mock acquisition error")

        with self.assertRaisesRegex(ValueError, "mock acquisition error"):
            acquire(self.output, broken_source, lambda _reference: .15)
        last = self.events()[-1]
        self.assertEqual(last["status"], "EXCEPTION")
        self.assertIn("Traceback (most recent call last)", last["traceback"])
        self.assertIn("mock acquisition error", last["traceback"])


def mock_worker(output, fault):
    for name in PHASES:
        with recorder.phase(output, name):
            if name == "frame_readiness":
                if fault == "association-fail":
                    try:
                        acquire(output, lambda: packets(8, 9), lambda _reference: .15)
                    except RuntimeError:
                        os._exit(0)
                frame = acquire(output, packets, lambda _reference: .15)
            if name == "write_artifacts":
                write_mock_frame(output, frame, fault)
            if name == "shutdown" and fault == "shutdown-exit":
                os._exit(0)


def write_mock_frame(output, frame, fault):
    association = dict(frame.association)
    if fault == "tampered-reference":
        association["instance_reference"] = [10, 60]
    if fault == "tampered-journal":
        association["attempt"] = 2
    metadata = {
        "schema": 2, "source": "mock", "width": recorder.WIDTH, "height": recorder.HEIGHT,
        "rendering_time_s": .15, "frame_association": association, "render_product": "/Render/R3",
        "camera_prim": recorder.CAMERA, "target_prim": recorder.TARGET,
        "stage_meters_per_unit": 1, "stage_up_axis": "Z",
        "intrinsics_native": [[20, 0, 16.5], [0, 20, 12.5], [0, 0, 1]],
        "camera_local_to_world_usd": [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 1, 1]],
        "depth_annotator": DEPTH, "instance_annotator": INSTANCE,
        "depth_native_dtype": "float32", "instance_native_dtype": "uint32", "id_to_labels": frame.labels,
    }
    recorder.write_json(output / "frame.json", metadata)
    recorder.write_artifact(output / "depth.f32le", frame.depth)
    recorder.write_artifact(output / "instances.u32le", frame.instances)


class CompletionTests(unittest.TestCase):
    def run_supervisor(self, fault):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        root = Path(scratch.name) / "run"
        entry = Path(__file__).with_name("supervise_r3.py")
        command = [sys.executable, str(entry), "--output", str(root), "--timeout", "5",
                   "--mode", "mock", "--", sys.executable, __file__,
                   "--mock-worker", str(root / "capture"), fault]
        result = subprocess.run(command, capture_output=True, timeout=10, check=False)
        return result, root

    def assert_failed(self, fault, reason):
        result, root = self.run_supervisor(fault)
        self.assertEqual(result.returncode, 1, result.stderr.decode())
        self.assertFalse((root / "COMPLETED.json").exists())
        self.assertIn(reason, (root / "FAILED.json").read_text())
        return root

    def test_ready_pair_and_verified_shutdown_can_complete(self):
        result, root = self.run_supervisor("valid")
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        marker = json.loads((root / "COMPLETED.json").read_text())
        self.assertIn("capture/readiness.jsonl", marker["sha256"])

    def test_successful_acquisition_followed_by_incomplete_shutdown(self):
        root = self.assert_failed("shutdown-exit", "did not complete every phase through shutdown")
        self.assertTrue((root / "capture/depth.f32le").exists())
        self.assertIn('"status": "READY"', (root / "capture/readiness.jsonl").read_text())

    def test_unverified_acquisition_with_clean_exit_cannot_complete(self):
        root = self.assert_failed("association-fail", "did not complete every phase")
        self.assertIn('"returncode": 0', (root / "FAILED.json").read_text())

    def test_metadata_mismatch_rejected_after_clean_shutdown(self):
        self.assert_failed("tampered-reference", UNVERIFIED)

    def test_metadata_must_match_attempt_journal(self):
        self.assert_failed("tampered-journal", "incomplete readiness attempts")


if __name__ == "__main__":
    if sys.argv[1:2] == ["--mock-worker"]:
        mock_worker(Path(sys.argv[2]), sys.argv[3])
    else:
        unittest.main()
