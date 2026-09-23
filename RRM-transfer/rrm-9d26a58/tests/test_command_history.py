"""Offline takeoff repeatability reporting tests."""
import json
from pathlib import Path
import tempfile
import unittest

from rrm.command_history import summarize_takeoffs


class CommandHistoryTests(unittest.TestCase):
    def test_legacy_and_current_takeoff_evidence_are_aggregated_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "run-a" / "command-mission-evidence"
            second = root / "run-b" / "command-mission-evidence"
            first.mkdir(parents=True)
            second.mkdir(parents=True)
            (first / "0000-takeoff-0-outcome.json").write_text(json.dumps({
                "kind": "TAKEOFF", "action_id": "takeoff-0", "action_success": False,
                "action_message": "horizontal displacement limit exceeded",
                "verdict": "UNCONFIRMED",
                "reasons": ["takeoff_horizontal_displacement_mismatch"],
                "pre_odometry": {"x": 0, "y": 0, "z": 0.02},
                "post_odometry": {"x": 0.3, "y": 0.4, "z": 0.1},
            }), encoding="utf-8")
            (second / "0000-takeoff-0-outcome.json").write_text(json.dumps({
                "kind": "TAKEOFF", "action_id": "takeoff-0", "action_success": True,
                "verdict": "VERIFIED", "metrics": {"horizontal_displacement_m": 0.1},
                "diagnostics": [],
            }), encoding="utf-8")

            report = summarize_takeoffs(root)

            self.assertEqual(report["attempt_count"], 2)
            self.assertEqual(report["verified_rate"], 0.5)
            self.assertAlmostEqual(report["mean_horizontal_displacement_m"], 0.3)
            self.assertEqual(report["max_horizontal_displacement_m"], 0.5)
            self.assertIn("LATERAL_INSTABILITY_OBSERVED",
                          report["attempts"][0]["diagnostics"])
            self.assertFalse(report["execution_dispatch"])


if __name__ == "__main__":
    unittest.main()
