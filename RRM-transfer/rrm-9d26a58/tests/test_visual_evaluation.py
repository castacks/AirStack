"""Synthetic pairing contracts; no live acquisition, model or dispatch evidence."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

from rrm.visual_evaluation import canonical_sha256, validate_visual_pair


ROOT = Path(__file__).parents[1]


def fixture():
    def chunk(kind, value):
        return (struct.pack(">I", len(value)) + kind + value
                + struct.pack(">I", zlib.crc32(kind + value)))
    image = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 48, 30, 8, 2, 0, 0, 0))
             + chunk(b"IDAT", zlib.compress((b"\0" + b"\0" * 144) * 30)) + chunk(b"IEND", b""))
    manifest = json.loads((ROOT / "examples/office_visual_eval/scene_manifest.json").read_text())
    entities = [{"entity_id": key, "scene_prim": value["scene_prim"], "kind": value["kind"]}
                for key, value in manifest["markers"].items()]
    frame = dict(episode_id="seed-1-run-1", scene_revision="stage-1", source_stamp_ns=1000,
                 sha256=hashlib.sha256(image).hexdigest(), topic=manifest["camera_topic"],
                 frame_id=manifest["camera_frame_id"], width=48, height=30)
    registration = dict(schema_version="rrm-visual-fixture/v1", fixture_id=manifest["scene_id"],
                        episode_id=frame["episode_id"], scene_revision=frame["scene_revision"],
                        manifest_sha256=canonical_sha256(manifest), launcher_sha256="a" * 64,
                        producer_ref="trusted-test-exporter/v1", camera_topic=frame["topic"],
                        camera_frame_id=frame["frame_id"], entities=entities)
    teacher = dict(schema_version="rrm-visual-teacher-frame/v1", provenance="SIMULATOR",
                   producer_ref=registration["producer_ref"], fixture_id=registration["fixture_id"],
                   manifest_sha256=registration["manifest_sha256"], frame=copy.deepcopy(frame),
                   labels=[dict(e, exists=True, visibility="VISIBLE", image_region=[1, 2, 10, 20])
                           for e in entities])
    return dict(image=image, frame=frame, registration=registration, teacher=teacher,
                manifest=manifest, launcher_sha256="a" * 64,
                expected_teacher_source_ref=registration["producer_ref"],
                assessment_episode_id=frame["episode_id"], assessment_source_stamp_ns=1010,
                max_age_ns=20)


class VisualPairingTests(unittest.TestCase):
    def assert_blocked(self, value, reason=None):
        result = validate_visual_pair(**value)
        self.assertEqual(result["status"], "BLOCKED")
        if reason:
            self.assertEqual(result["reason"], reason)
        self.assertFalse(result["execution_dispatch"])
        self.assertFalse(result["scored"])
        return result

    def test_bound_records_are_not_scored_or_dispatched_or_mutated(self):
        value = fixture()
        original = copy.deepcopy(value)
        result = validate_visual_pair(**value)
        self.assertEqual(result["status"], "BOUND_FOR_ASSESSMENT")
        self.assertEqual(result["observation_age_ns"], 10)
        self.assertEqual(result["label_count"], 2)
        self.assertFalse(result["scored"])
        self.assertFalse(result["execution_dispatch"])
        self.assertFalse(result["teacher_sent_to_candidate"])
        self.assertEqual(value, original)

    def test_missing_registration_and_independent_teacher(self):
        for key, reason in (("registration", "fixture_registration_missing"),
                            ("teacher", "independent_teacher_missing")):
            value = fixture(); value[key] = None
            self.assert_blocked(value, reason)

    def test_fixture_and_source_identity_mismatches(self):
        for key in ("fixture_id", "manifest_sha256", "launcher_sha256", "camera_topic", "camera_frame_id"):
            with self.subTest(key=key):
                value = fixture()
                value["registration"][key] = "b" * 64 if "sha256" in key else "wrong"
                self.assert_blocked(value, "fixture_identity_mismatch")
        for record in ("registration", "teacher"):
            value = fixture(); value[record]["producer_ref"] = "candidate-model"
            self.assert_blocked(value, "teacher_source_mismatch")

    def test_frame_and_epoch_identity_mismatches(self):
        for key, wrong in (("source_stamp_ns", 999), ("sha256", "b" * 64),
                           ("topic", "wrong"), ("frame_id", "wrong"), ("width", 49),
                           ("height", 31), ("episode_id", "old-epoch"), ("scene_revision", "stage-2")):
            with self.subTest(key=key):
                value = fixture(); value["teacher"]["frame"][key] = wrong
                self.assert_blocked(value, "teacher_frame_mismatch")
        for key in ("episode_id", "scene_revision"):
            value = fixture(); value["registration"][key] = "old"
            self.assert_blocked(value, "scene_epoch_or_revision_mismatch")
        value = fixture(); value["assessment_episode_id"] = "another-run"
        self.assert_blocked(value, "scene_epoch_or_revision_mismatch")

    def test_catalog_and_prim_labels_must_be_complete_exact_and_unique(self):
        for record, key in (("registration", "entities"), ("teacher", "labels")):
            for change in ("missing", "duplicate", "unknown", "prim", "kind"):
                with self.subTest(record=record, change=change):
                    value = fixture(); rows = value[record][key]
                    if change == "missing": rows.pop()
                    elif change == "duplicate": rows.append(copy.deepcopy(rows[0]))
                    else: rows[0][{"unknown": "entity_id", "prim": "scene_prim", "kind": "kind"}[change]] = "/wrong"
                    self.assert_blocked(value, "entity_labels_incomplete_or_mismatched")

    def test_age_boundary_future_stale_and_invalid_clock(self):
        for now in (1000, 1020):
            value = fixture(); value["assessment_source_stamp_ns"] = now
            self.assertEqual(validate_visual_pair(**value)["status"], "BOUND_FOR_ASSESSMENT")
        for now in (999, 1021):
            value = fixture(); value["assessment_source_stamp_ns"] = now
            self.assert_blocked(value, "future_or_stale_frame")
        for key, wrong in (("max_age_ns", 0), ("max_age_ns", float("nan")),
                           ("assessment_source_stamp_ns", True), ("assessment_source_stamp_ns", -1),
                           ("assessment_source_stamp_ns", float("inf"))):
            value = fixture(); value[key] = wrong
            self.assert_blocked(value, "invalid_assessment_context")

    def test_image_hash_dimensions_and_invalid_header(self):
        value = fixture(); value["image"] += b"changed"
        self.assert_blocked(value, "image_integrity_mismatch")
        value = fixture()
        for record in (value["frame"], value["teacher"]["frame"]): record["width"] = 49
        self.assert_blocked(value, "image_dimensions_mismatch")
        value = fixture(); value["image"] = b"not a png"
        for record in (value["frame"], value["teacher"]["frame"]):
            record["sha256"] = hashlib.sha256(value["image"]).hexdigest()
        self.assert_blocked(value, "invalid_png_header")

    def test_visibility_regions_and_uncertainty(self):
        for visible in ("OCCLUDED", "UNKNOWN", "ABSENT"):
            value = fixture(); label = value["teacher"]["labels"][0]
            label.update(visibility=visible, exists=visible != "ABSENT", image_region=None)
            self.assertEqual(validate_visual_pair(**value)["status"], "BOUND_FOR_ASSESSMENT")
        for region in ([-1, 0, 5, 5], [0, 0, 49, 10], [5, 5, 5, 10], [0, 0, 10, 31]):
            value = fixture(); value["teacher"]["labels"][0]["image_region"] = region
            self.assert_blocked(value, "invalid_image_region")
        for changes in ({"exists": False}, {"visibility": "OCCLUDED"},
                        {"image_region": None}, {"image_region": [0, 0, float("nan"), 10]}):
            value = fixture(); value["teacher"]["labels"][0].update(changes)
            self.assert_blocked(value, "malformed_pairing_evidence")

    def test_malformed_and_candidate_provenance_are_blocked(self):
        for record in ("frame", "registration", "teacher"):
            value = fixture(); value[record]["execution_dispatch"] = True
            self.assert_blocked(value, "malformed_pairing_evidence")
        for wrong in (True, "1000", 1000.5):
            value = fixture(); value["frame"]["source_stamp_ns"] = wrong
            self.assert_blocked(value, "malformed_pairing_evidence")
        value = fixture(); value["teacher"]["provenance"] = "INFERRED"
        self.assert_blocked(value, "malformed_pairing_evidence")

    def test_offline_cli_missing_registration_returns_report_exit_2(self):
        value = fixture()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "image.png").write_bytes(value["image"])
            for key in ("frame", "manifest"):
                (root / (key + ".json")).write_text(json.dumps(value[key]))
            result = subprocess.run([sys.executable, str(ROOT / "scripts/rrm_validate_visual_pair.py"),
                "--image", str(root / "image.png"), "--frame", str(root / "frame.json"),
                "--manifest", str(root / "manifest.json"), "--launcher-sha256", "a" * 64,
                "--teacher-source-ref", "trusted-test-exporter/v1", "--assessment-episode-id", "seed-1-run-1",
                "--assessment-source-stamp-ns", "1010", "--max-age-ns", "20"],
                cwd=ROOT, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(json.loads(result.stdout)["reason"], "fixture_registration_missing")


if __name__ == "__main__":
    unittest.main()
