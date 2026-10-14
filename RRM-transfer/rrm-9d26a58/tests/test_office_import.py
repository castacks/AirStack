"""Synthetic transport tests, not Cosmos inference or flight evidence."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from pydantic import ValidationError
from rrm.airstack_drone import DroneTaskProposal

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
        self.assertEqual(result.proposal.source_scope, "FIXTURE_REPLAY")
        self.assertEqual(result.proposal.action_id, "model-selected-nav")
        self.assertEqual(result.proposal.waypoints[-1].x, 3.2)

    def test_cli_exports_audit_report_without_executable_proposal(self):
        output = self.bundle / "imported"
        script = ROOT / "scripts" / "rrm_import_office.py"
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join((str(ROOT), str(ROOT / "scripts"),
                                              env.get("PYTHONPATH", "")))
        result = subprocess.run([sys.executable, str(script), "--bundle", str(self.bundle),
                                 "--output-dir", str(output)], capture_output=True,
                                text=True, env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual({path.name for path in output.iterdir()}, {"replay_report.json"})
        report = json.loads((output / "replay_report.json").read_text())
        self.assertEqual(report["authority"], "FIXTURE_REPLAY")
        self.assertFalse(report["execution_allowed"])
        self.assertEqual(report["decision"]["plan"]["actions"][0]["action"]["id"],
                         "model-selected-nav")
        self.assertEqual(report["proposal_preview"]["goal"]["global_plan"]["poses"][-1]
                         ["pose"]["position"]["x"], 3.2)
        with self.assertRaises(ValidationError):
            DroneTaskProposal.model_validate(report)

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
