"""Tests for the no-action tabletop C02 export boundary."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from export_c02_facts import build_export, load_probe
from score_candidate import load_image_only_candidate
from rrm.state_contracts import FactKey, StateSnapshot


def probe() -> dict:
    return {
        "schema_version": "rrm-hand-tabletop-probe/v1",
        "scene_recipe_sha256": "a" * 64,
        "scene_entities": {
            "red_block": "/World/red_block",
            "blue_block": "/World/blue_block",
            "tray_1": "/World/Tray",
            "table": "/World/Table",
        },
        "controller_command_sent": False,
        "ros_connected": False,
        "execution_dispatch": False,
        "reset_hashes_match": True,
        "reset_samples": [{
            "reset_index": 0,
            "episode_id": "episode-1",
            "observed_monotonic_s": 10.0,
            "camera_capture": {
                "episode_id": "episode-1",
                "observed_monotonic_s": 10.0,
                "path": "overhead_0.png",
                "sha256": "b" * 64,
            },
        }],
    }


class HandTabletopExportTests(unittest.TestCase):
    def test_export_is_camera_paired_and_limited_to_supported_teacher_facts(self):
        record = build_export(probe(), probe_sha256="c" * 64, reset_index=0,
                              task_id="hand-task-001")
        snapshot = StateSnapshot.model_validate(record["snapshot"])
        self.assertFalse(record["execution_dispatch"])
        self.assertEqual(record["sample"]["camera_sha256"], "b" * 64)
        self.assertEqual(len(snapshot.evidence), 12)
        self.assertEqual(
            snapshot.resolve(FactKey(subject="red_block", predicate="kind", obj="block"),
                             now_monotonic_s=10.0).value,
            "TRUE",
        )
        self.assertEqual(
            snapshot.resolve(FactKey(subject="tray_1", predicate="kind", obj="tray"),
                             now_monotonic_s=10.0).value,
            "TRUE",
        )
        self.assertEqual(
            snapshot.resolve(FactKey(subject="red_block", predicate="graspable"),
                             now_monotonic_s=10.0).value,
            "UNKNOWN",
        )
        self.assertTrue(all(item.provenance.value == "SIMULATOR" for item in snapshot.evidence))

    def test_export_rejects_action_or_unpaired_probe_data(self):
        unsafe = probe()
        unsafe["execution_dispatch"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-probe.json"
            path.write_text(json.dumps(unsafe), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no-action"):
                load_probe(path)
        paired = deepcopy(probe())
        paired["reset_samples"][0]["camera_capture"]["episode_id"] = "wrong-episode"
        with self.assertRaisesRegex(ValueError, "camera capture"):
            build_export(paired, probe_sha256="c" * 64, reset_index=0,
                         task_id="hand-task-001")

    def test_image_only_candidate_must_bind_to_the_teacher_frame(self):
        teacher = build_export(probe(), probe_sha256="c" * 64, reset_index=0,
                               task_id="hand-task-001")
        candidate = {
            "image_sha256": "b" * 64,
            "execution_dispatch": False,
            "snapshot": {
                "snapshot_id": "candidate-1", "revision": "candidate-1",
                "task_id": "hand-task-001", "episode_id": "episode-1",
                "complete_domains": [], "evidence": [],
            },
        }
        self.assertEqual(load_image_only_candidate(candidate, teacher).episode_id, "episode-1")
        candidate["image_sha256"] = "d" * 64
        with self.assertRaisesRegex(ValueError, "camera image"):
            load_image_only_candidate(candidate, teacher)


if __name__ == "__main__":
    unittest.main()
