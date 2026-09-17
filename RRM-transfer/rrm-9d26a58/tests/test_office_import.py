"""Synthetic transport tests, not Cosmos inference or flight evidence."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from rrm_import_office import import_bundle
from rrm_cosmos_reason2 import load_context
from rrm.cosmos_reason2 import parse_cosmos_candidate, render_cosmos_prompt


class OfficeImportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.bundle = Path(self.temp.name)
        fixture = ROOT / "examples" / "office_visual_eval"
        shutil.copy(fixture / "navigation_context.json", self.bundle / "input.json")
        shutil.copy(fixture / "scene_manifest.json", self.bundle / "scene_manifest.json")
        (self.bundle / "input.png").write_bytes(b"synthetic test media, not an actual image")
        context = load_context(self.bundle / "input.json")
        raw = json.dumps({
            "status": "READY", "grounded_entities": ["blue_marker"],
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
            "ambiguity_refs": [], "explanation": "synthetic test fixture",
            "actions": [{"id": "model-selected-nav", "verb": "NAVIGATE_TO",
                         "targets": ["blue_marker"], "dependencies": []}], "recovery_budget": 0,
        })
        self.record = {
            "raw_response": raw, "prompt": render_cosmos_prompt(context),
            "candidate": parse_cosmos_candidate(raw, context).model_dump(mode="json"),
            "input_sha256": hashlib.sha256((self.bundle / "input.json").read_bytes()).hexdigest(),
            "media_sha256": hashlib.sha256((self.bundle / "input.png").read_bytes()).hexdigest(),
        }
        self.save()

    def save(self):
        (self.bundle / "result.json").write_text(json.dumps(self.record))

    def test_actual_plan_ids_survive_import(self):
        result = import_bundle(self.bundle)
        self.assertEqual(result.proposal.action_id, "model-selected-nav")
        self.assertEqual(result.proposal.waypoints[-1].x, 3.2)

    def test_changed_image_rejected(self):
        (self.bundle / "input.png").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            import_bundle(self.bundle)

    def test_tampered_candidate_rejected(self):
        self.record["candidate"]["plan"]["actions"][0]["action"]["id"] = "substituted"
        self.save()
        with self.assertRaisesRegex(ValueError, "reparsed"):
            import_bundle(self.bundle)

    def test_wrong_prompt_rejected(self):
        self.record["prompt"] = "unrelated context"
        self.save()
        with self.assertRaisesRegex(ValueError, "prompt/context"):
            import_bundle(self.bundle)

    def test_changed_binding_rejected(self):
        path = self.bundle / "scene_manifest.json"
        scene = json.loads(path.read_text())
        scene["markers"]["blue_marker"]["map_waypoint"]["x"] = 999
        path.write_text(json.dumps(scene))
        with self.assertRaisesRegex(ValueError, "local reviewed fixture"):
            import_bundle(self.bundle)


if __name__ == "__main__":
    unittest.main()
