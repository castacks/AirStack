"""Paired all-attempt accounting, immutable input binding and comparison integrity."""

import copy
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rrm.acceptance_campaign import ROOT, run_campaign, verify_campaign, write_json
from rrm.benchmark_evidence import _sha256
from rrm.campaign_comparison import (ComparisonError, _load_campaign, compare_campaigns,
                                     distribution, paired_measurements, verify_comparison)


def save(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n")


def rehash(directory):
    path = directory / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["artifacts"] = {p.relative_to(directory).as_posix():
                             {"sha256": _sha256(p), "bytes": p.stat().st_size}
                             for p in directory.rglob("*") if p.is_file() and p != path}
    save(path, manifest)


class PairedMeasurementsTests(unittest.TestCase):
    def attempt(self, index, goal, duration, complete=None):
        return {"attempt_id": str(index), "scenario_id": "case", "repetition": index + 1,
                "goal_met": goal, "evidence_complete": goal is not None if complete is None else complete,
                "expectation_match": True, "harness": {"status": "EXITED", "elapsed_s": duration}}

    def test_all_nine_goal_outcomes_retain_unknown_pairs(self):
        outcomes = [(left, right) for left in (True, False, None) for right in (True, False, None)]
        left = [self.attempt(i, value[0], i + 1) for i, value in enumerate(outcomes)]
        right = [self.attempt(i, value[1], i + 2) for i, value in enumerate(outcomes)]
        report = paired_measurements(left, right)
        self.assertEqual(report["attempted_pairs"], 9)
        self.assertEqual(report["both_goal_reports_available"], 4)
        self.assertEqual(report["pairs_with_unknown_goal"], 5)
        self.assertEqual(report["candidate_goal_gains"], 1)
        self.assertEqual(report["candidate_goal_losses"], 1)
        self.assertTrue(all(count == 1 for row in report["goal_outcome_matrix"].values() for count in row.values()))
        latency = report["harness_latency_s"]
        self.assertEqual(latency["all_attempts"]["reference"], {"count": 9, "median": 5, "p95": 9, "max": 9})
        self.assertEqual(latency["jointly_qualified_pairs"]["reference"]["count"], 4)
        self.assertEqual(latency["all_attempts"]["candidate_minus_reference"]["median"], 1)

    def test_false_goal_report_does_not_assert_verified_negative_truth(self):
        report = paired_measurements([self.attempt(0, False, 1)], [self.attempt(0, None, 1)])
        self.assertEqual(report["pairs"][0]["reference_goal"], "GOAL_NOT_VERIFIED")
        self.assertEqual(report["pairs"][0]["candidate_goal"], "UNKNOWN")
        self.assertEqual(report["both_goal_reports_available"], 0)
        self.assertEqual(report["candidate_goal_losses"], 0)

    def test_invalid_identity_types_outcomes_and_latencies_fail(self):
        original = self.attempt(0, True, 1)
        for field, value in (("repetition", True), ("goal_met", 1), ("evidence_complete", 1),
                             ("evidence_complete", False), ("attempt_id", "different")):
            right = copy.deepcopy(original)
            right[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ComparisonError):
                paired_measurements([original], [right])
        for value in (-1, True, float("nan"), float("inf")):
            right = copy.deepcopy(original)
            right["harness"]["elapsed_s"] = value
            with self.subTest(value=value), self.assertRaises(ComparisonError):
                paired_measurements([original], [right])
        for right in ([], [original, original]):
            with self.assertRaises(ComparisonError):
                paired_measurements([original], right)

    def test_empty_qualified_subset_and_nearest_rank(self):
        report = paired_measurements([self.attempt(0, None, 5)], [self.attempt(0, None, 3)])
        self.assertEqual(report["jointly_qualified_pairs"], 0)
        self.assertEqual(report["harness_latency_s"]["jointly_qualified_pairs"]["reference"],
                         {"count": 0, "median": None, "p95": None, "max": None})
        self.assertEqual(report["harness_latency_s"]["all_attempts"]["candidate_minus_reference"]["max"], -2)
        self.assertEqual(distribution(list(range(1, 21)))["p95"], 19)


class CampaignComparisonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.base = Path(cls.temp.name)
        cls.left, cls.right = cls.base / "left", cls.base / "right"
        cls.cases = ["nominal-pick", "persistent-effect", "evidence-write-loss"]
        run_campaign(cls.left, case_ids=cls.cases)
        run_campaign(cls.right, case_ids=cls.cases)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_real_bundles_pair_all_attempts_and_reconstruct_report(self):
        report = compare_campaigns(self.left, self.right)
        self.assertEqual(report["attempted_pairs"], 3)
        self.assertEqual(report["jointly_qualified_pairs"], 2)
        self.assertEqual(report["goal_outcome_matrix"]["UNKNOWN"]["UNKNOWN"], 1)
        self.assertEqual(report["pairs_with_unknown_goal"], 1)
        self.assertEqual(report["candidate_goal_gains"], 0)
        self.assertEqual(report["candidate_goal_losses"], 0)
        self.assertFalse(report["performance_claim_authorized"])
        self.assertEqual(report["scope"], "same_implementation_mock_repeatability")
        self.assertEqual(report["arms"]["reference"]["metrics"]["verified_goal_rate"]["numerator"], 1)
        self.assertEqual(report["harness_latency_s"]["all_attempts"]["candidate"]["count"], 3)
        self.assertEqual(report["arms"]["candidate"]["safety_adjudication"]["unqualified_attempts"], 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            write_json(path, report)
            self.assertTrue(verify_comparison(self.left, self.right, path)["valid"])
            for field, value in (("candidate_goal_gains", False), ("attempted_pairs", 99), ("scope", "performance")):
                changed = copy.deepcopy(report)
                changed[field] = value
                save(path, changed)
                self.assertFalse(verify_comparison(self.left, self.right, path)["valid"])

    def test_alias_clone_and_reused_run_identity_rejected(self):
        with self.assertRaisesRegex(ComparisonError, "same_campaign_directory"):
            compare_campaigns(self.left, self.left)
        with tempfile.TemporaryDirectory() as directory:
            alias = Path(directory) / "alias"
            alias.symlink_to(self.left, target_is_directory=True)
            with self.assertRaisesRegex(ComparisonError, "same_campaign_directory"):
                compare_campaigns(self.left, alias)
            clone = Path(directory) / "clone"
            shutil.copytree(self.left, clone)
            with self.assertRaisesRegex(ComparisonError, "cloned_campaign_bundle"):
                compare_campaigns(self.left, clone)
            manifest = json.loads((clone / "manifest.json").read_text())
            manifest["created_at"] = "2026-10-02T00:00:00+00:00"
            save(clone / "manifest.json", manifest)
            self.assertTrue(verify_campaign(clone)["valid"])
            with self.assertRaisesRegex(ComparisonError, "cloned_campaign_bundle"):
                compare_campaigns(self.left, clone)
            # Different artifact bytes with reused run IDs still cannot qualify.
            (clone / "attempts/nominal-pick-r001/worker.log").write_text("extra log\n")
            rehash(clone)
            with self.assertRaisesRegex(ComparisonError, "reused_run_identity_across_campaigns"):
                compare_campaigns(self.left, clone)

    def test_reused_run_identity_within_bundle_rejected(self):
        # Preserve manifest binding while isolating the extra independence check.
        with tempfile.TemporaryDirectory() as directory:
            clone = Path(directory) / "clone"
            shutil.copytree(self.left, clone)
            for trace in clone.glob("attempts/*/events.jsonl"):
                events = [json.loads(line) for line in trace.read_text().splitlines()]
                events[0]["run_id"] = "reused"
                trace.write_text("".join(json.dumps(e) + "\n" for e in events))
            rehash(clone)
            with patch("rrm.campaign_comparison.verify_campaign", return_value={"valid": True}), \
                    self.assertRaisesRegex(ComparisonError, "reused_run_identity"):
                _load_campaign(clone)

    def test_configuration_source_drift_and_unsealed_bundle_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            other = Path(directory) / "other"
            run_campaign(other, case_ids=["nominal-pick"])
            with self.assertRaisesRegex(ComparisonError, "frozen_configuration_mismatch"):
                compare_campaigns(self.left, other)
            drift = Path(directory) / "drift"
            shutil.copytree(self.right, drift)
            manifest = json.loads((drift / "manifest.json").read_text())
            manifest["final_source"]["git_commit"] = "changed"
            save(drift / "manifest.json", manifest)
            summary = json.loads((drift / "summary.json").read_text())
            summary["source_unchanged"] = False
            summary["campaign_expectations_met"] = False
            save(drift / "summary.json", summary)
            rehash(drift)
            self.assertTrue(verify_campaign(drift)["valid"])
            with self.assertRaisesRegex(ComparisonError, "campaign_source_drift"):
                compare_campaigns(self.left, drift)
            (drift / "manifest.json").unlink()
            with self.assertRaises(ComparisonError):
                compare_campaigns(self.left, drift)

    def test_crashed_campaign_remains_all_attempts_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            crash = Path(directory) / "crash"
            run_campaign(crash, case_ids=self.cases,
                         command_factory=lambda attempt, target: [sys.executable, "-c", "raise SystemExit(7)"])
            report = compare_campaigns(self.left, crash)
            self.assertEqual(report["attempted_pairs"], 3)
            self.assertEqual(report["pairs_with_unknown_goal"], 3)
            self.assertEqual(report["jointly_qualified_pairs"], 0)
            self.assertEqual(report["candidate_goal_losses"], 0)
            self.assertEqual(report["all_attempt_rate_differences"]["verified_goal_rate"]["numerator_difference"], -1)
            self.assertEqual(report["arms"]["candidate"]["metrics"]["evidence_complete_rate"]["numerator"], 0)
            self.assertTrue(all(pair["candidate_run_id"] is None for pair in report["pairs"]))
            self.assertEqual(report["harness_latency_s"]["all_attempts"]["candidate"]["count"], 3)
            self.assertEqual(report["arms"]["candidate"]["attempts_without_run_identity"], 3)
            clone = Path(directory) / "clone"
            shutil.copytree(crash, clone)
            manifest = json.loads((clone / "manifest.json").read_text())
            manifest["created_at"] = "2026-10-02T00:00:00+00:00"
            save(clone / "manifest.json", manifest)
            with self.assertRaisesRegex(ComparisonError, "cloned_campaign_bundle"):
                compare_campaigns(crash, clone)

    def test_bound_inputs_changed_during_loading_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            clone = Path(directory) / "clone"
            shutil.copytree(self.right, clone)
            def mutate(target):
                summary = json.loads((target / "summary.json").read_text())
                summary["attempted_runs"] = 99
                save(target / "summary.json", summary)
                return {"valid": True}
            with patch("rrm.campaign_comparison.verify_campaign", side_effect=mutate), \
                    self.assertRaisesRegex(ComparisonError, "input_changed:summary.json"):
                _load_campaign(clone)

    def test_cli_exclusive_export_verification_and_no_input_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report.json"
            command = [sys.executable, str(ROOT / "scripts/core_comparison.py"),
                       "--reference", str(self.left), "--candidate", str(self.right), "--output", str(report)]
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 0)
            before = report.read_bytes()
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 1)
            self.assertEqual(report.read_bytes(), before)
            self.assertEqual(subprocess.run(command + ["--verify"], capture_output=True).returncode, 0)
            command[-1] = str(self.left / "report.json")
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 1)
            self.assertFalse((self.left / "report.json").exists())
            self.assertTrue(verify_campaign(self.left)["valid"])
            self.assertTrue(verify_campaign(self.right)["valid"])


if __name__ == "__main__":
    unittest.main()
