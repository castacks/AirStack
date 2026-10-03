"""All-attempt denominators, fault expectations and adversarial bundle verification."""

import copy
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

from rrm.acceptance_campaign import (digest, evaluate, run_campaign, specification,
                                     verify_campaign)
from rrm.acceptance_fixtures import SCENARIOS
from rrm.benchmark_evidence import _sha256


def save(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n")


def rehash(output):
    path = output / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["artifacts"] = {p.relative_to(output).as_posix():
                             {"sha256": _sha256(p), "bytes": p.stat().st_size}
                             for p in output.rglob("*") if p.is_file() and p != path}
    save(path, manifest)


class AcceptanceCampaignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temp.name) / "matrix"
        cls.summary = run_campaign(cls.output)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_fixed_matrix_separates_acceptance_goals_and_incomplete_evidence(self):
        summary = self.summary
        self.assertEqual(summary["attempted_runs"], len(SCENARIOS))
        self.assertEqual(summary["expectation_match_rate"]["numerator"], 17)
        self.assertEqual(summary["evidence_complete_rate"]["numerator"], 16)
        self.assertEqual(summary["replay_qualified_acceptance_rate"]["numerator"], 16)
        self.assertEqual(summary["verified_goal_rate"]["numerator"], 3)
        self.assertEqual(summary["verified_goal_rate"]["denominator"], 17)
        self.assertEqual(summary["unknown_goal_outcomes"], 1)
        self.assertFalse(summary["performance_claim_authorized"])
        self.assertTrue(verify_campaign(self.output)["valid"])
        persistent = next(a for a in summary["attempts"] if a["scenario_id"] == "persistent-effect")
        self.assertTrue(persistent["legacy_task_success"])
        self.assertFalse(persistent["goal_met"])
        lost = next(a for a in summary["attempts"] if a["scenario_id"] == "evidence-write-loss")
        self.assertTrue(lost["expectation_match"])
        self.assertFalse(lost["acceptance_pass"])
        self.assertIsNone(lost["goal_met"])
        self.assertEqual(lost["stop_status"], "UNKNOWN")
        adjudication = summary["safety_adjudication"]
        self.assertEqual(adjudication["attempt_label_coverage"]["numerator"], 16)
        self.assertEqual(adjudication["attempt_label_coverage"]["denominator"], 17)
        self.assertEqual(adjudication["unqualified_attempts"], 1)
        self.assertEqual(adjudication["unqualified_observed_decisions"], 2)
        self.assertEqual(adjudication["stages"]["dynamic_symbolic"]["confusion_matrix"]["true_positive"], 2)
        self.assertEqual(adjudication["stages"]["symbolic"]["confusion_matrix"]["true_positive"], 1)
        self.assertEqual(adjudication["stages"]["numeric"]["confusion_matrix"]["true_positive"], 1)

    def test_configuration_is_validated_and_existing_campaign_is_never_overwritten(self):
        for kwargs in ({"repetitions": 0}, {"repetitions": True}, {"attempt_timeout_s": 0},
                       {"attempt_timeout_s": float("nan")}, {"case_ids": []},
                       {"case_ids": ["unknown"]}, {"case_ids": ["nominal-pick"] * 2}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                specification(**kwargs)
        with self.assertRaises(FileExistsError):
            run_campaign(self.output)

    def test_crash_timeout_launch_error_and_missing_result_remain_attempts(self):
        cases = {
            "crash": ([sys.executable, "-c", "raise SystemExit(7)"], "EXITED"),
            "timeout": ([sys.executable, "-c", "import time; time.sleep(10)"], "TIMEOUT"),
            "launch": (["/missing/rrm-fixture-executable"], "LAUNCH_ERROR"),
            "missing": ([sys.executable, "-c", "pass"], "EXITED"),
            "corrupt": ([sys.executable, "-c",
                         "import pathlib,sys; pathlib.Path(sys.argv[1]).write_text('{')"], "EXITED"),
        }
        with tempfile.TemporaryDirectory() as directory:
            for mode, (command, expected) in cases.items():
                def factory(attempt, target):
                    return command + ([str(target / "result.json")] if mode == "corrupt" else [])
                output = Path(directory) / mode
                summary = run_campaign(output, case_ids=["nominal-pick"],
                                       attempt_timeout_s=0.3, command_factory=factory)
                self.assertEqual(summary["attempted_runs"], 1)
                self.assertEqual(summary["evidence_complete_rate"]["denominator"], 1)
                self.assertEqual(summary["evidence_complete_rate"]["numerator"], 0)
                self.assertFalse(summary["campaign_expectations_met"])
                self.assertEqual(summary["attempts"][0]["harness"]["status"], expected)
                self.assertEqual(summary["attempts"][0]["stop_status"], "UNKNOWN")
                # A faithfully recorded unsuccessful campaign is an intact bundle.
                self.assertTrue(verify_campaign(output)["valid"])

    def test_missing_terminal_and_forged_trace_binding_cannot_count_as_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            for mode in ("terminal", "scenario", "configuration", "limits", "profile", "result"):
                target = Path(directory) / mode
                shutil.copytree(self.output, target)
                attempt = specification()["attempts"][0]
                folder = target / "attempts" / attempt["attempt_id"]
                path = folder / "events.jsonl"
                events = [json.loads(line) for line in path.read_text().splitlines()]
                if mode == "terminal":
                    events.pop()
                elif mode in {"scenario", "configuration"}:
                    events[0]["scenario_id" if mode == "scenario" else "configuration_sha256"] = "forged"
                elif mode == "limits":
                    next(e for e in events if e["kind"] == "call_limits_declaration")["limits"]["reasoner_s"] = 99
                elif mode == "profile":
                    next(e for e in events if e["kind"] == "numeric_profile_declaration")["profile"]["max_velocity"] = 99
                else:
                    result = json.loads((folder / "result.json").read_text())
                    result["metrics"] = {}
                    save(folder / "result.json", result)
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                config = json.loads((target / "campaign.json").read_text())
                harness = json.loads((folder / "harness.json").read_text())
                report = evaluate(folder, attempt, config["sha256"], harness)
                self.assertFalse(report["evidence_complete"], mode)
                self.assertIsNone(report["goal_met"], mode)
                self.assertFalse(report["expectation_match"], mode)

    def test_timeout_preserves_partial_trace_without_claiming_core_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "partial-timeout"
            def factory(attempt, target):
                code = ("import pathlib,sys,time; "
                        "pathlib.Path(sys.argv[1]).write_text(sys.argv[2]); time.sleep(10)")
                return [sys.executable, "-c", code, str(target / "events.jsonl"),
                        json.dumps({"kind": "run_start", "attempt_id": attempt["attempt_id"]}) + "\n"]
            summary = run_campaign(output, case_ids=["nominal-pick"],
                                   attempt_timeout_s=0.3, command_factory=factory)
            attempt = summary["attempts"][0]
            self.assertEqual(attempt["harness"]["status"], "TIMEOUT")
            self.assertEqual(attempt["stop_status"], "UNKNOWN")
            self.assertFalse(attempt["evidence_complete"])
            self.assertIsNone(attempt["goal_met"])
            self.assertTrue((output / "attempts/nominal-pick-r001/events.jsonl").is_file())
            self.assertTrue(verify_campaign(output)["valid"])

    def test_rehashed_bundle_tampering_is_rejected(self):
        for mode in ("ledger_drop", "ledger_duplicate", "result_id", "result_counter", "source",
                     "summary", "matrix", "assessment", "trace", "extra"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                target = Path(directory) / "tamper"
                shutil.copytree(self.output, target)
                folder = target / "attempts" / "nominal-pick-r001"
                if mode.startswith("ledger"):
                    path = target / "attempts.jsonl"
                    records = [json.loads(line) for line in path.read_text().splitlines()]
                    if mode == "ledger_drop":
                        records.pop()
                    else:
                        records[2:4] = copy.deepcopy(records[:2])
                    path.write_text("".join(json.dumps(r) + "\n" for r in records))
                elif mode.startswith("result"):
                    path = folder / "result.json"
                    result = json.loads(path.read_text())
                    if mode == "result_id":
                        result["attempt_id"] = "another"
                    else:
                        result["metrics"]["action_count"] = 0
                    save(path, result)
                elif mode == "source":
                    path = target / "manifest.json"
                    manifest = json.loads(path.read_text())
                    manifest["source"]["runtime_sha256"] = "0" * 64
                    save(path, manifest)
                elif mode == "summary":
                    path = target / "summary.json"
                    summary = json.loads(path.read_text())
                    summary["verified_goal_rate"]["numerator"] = 17
                    save(path, summary)
                elif mode == "matrix":
                    path = target / "campaign.json"
                    config = json.loads(path.read_text())
                    config["spec"]["cases"][0]["applies"] = 0
                    config["sha256"] = digest(config["spec"])
                    save(path, config)
                elif mode == "assessment":
                    path = folder / "assessment.json"
                    report = json.loads(path.read_text())
                    report["acceptance_pass"] = False
                    save(path, report)
                elif mode == "trace":
                    path = folder / "events.jsonl"
                    events = [json.loads(line) for line in path.read_text().splitlines()]
                    events[-1]["goal_met"] = False
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                else:
                    # An unlisted artifact cannot disappear silently.
                    (target / "extra.txt").write_text("unexpected")
                    self.assertFalse(verify_campaign(target)["valid"])
                    continue
                rehash(target)
                self.assertFalse(verify_campaign(target)["valid"])

    def test_rehashed_bool_integer_substitutions_are_rejected(self):
        for mode in ("summary", "assessment", "ledger", "artifact_size", "source_unchanged"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                target = Path(directory) / mode
                shutil.copytree(self.output, target)
                if mode == "ledger":
                    path = target / "attempts.jsonl"
                    records = [json.loads(line) for line in path.read_text().splitlines()]
                    records[0]["repetition"] = True
                    path.write_text("".join(json.dumps(record) + "\n" for record in records))
                elif mode == "assessment":
                    path = target / "attempts/nominal-pick-r001/assessment.json"
                    report = json.loads(path.read_text())
                    report["expectation_match"] = 1
                    save(path, report)
                elif mode == "artifact_size":
                    rehash(target)
                    path = target / "manifest.json"
                    manifest = json.loads(path.read_text())
                    manifest["artifacts"]["attempts/nominal-pick-r001/worker.log"]["bytes"] = False
                    save(path, manifest)
                    self.assertFalse(verify_campaign(target)["valid"])
                    continue
                else:
                    path = target / "summary.json"
                    summary = json.loads(path.read_text())
                    if mode == "summary":
                        summary["safety_adjudication"]["stages"]["symbolic"]["confusion_matrix"]["false_positive"] = False
                    else:
                        summary["source_unchanged"] = 1
                    save(path, summary)
                rehash(target)
                self.assertFalse(verify_campaign(target)["valid"])

    def test_partial_prefix_and_invalid_harness_cannot_match_expectations(self):
        for mode in ("prefix", "dispatch", "negative_time", "bool_returncode", "missing_time", "scalar", "sequence"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                target = Path(directory) / "altered"
                shutil.copytree(self.output, target)
                attempt = next(a for a in specification()["attempts"]
                               if a["scenario_id"] == "evidence-write-loss")
                folder = target / "attempts" / attempt["attempt_id"]
                path = folder / "events.jsonl"
                events = [json.loads(line) for line in path.read_text().splitlines()]
                harness = json.loads((folder / "harness.json").read_text())
                if mode == "prefix":
                    events = events[:6]
                elif mode == "dispatch":
                    events[-1]["dispatch_id"] = "forged"
                elif mode == "negative_time":
                    harness["elapsed_s"] = -99
                elif mode == "bool_returncode":
                    harness["returncode"] = False
                elif mode == "missing_time":
                    del harness["elapsed_s"]
                elif mode == "scalar":
                    events = [17]
                else:
                    events[-1]["sequence"] = 999
                path.write_text("".join(json.dumps(e) + "\n" for e in events))
                config = json.loads((target / "campaign.json").read_text())
                report = evaluate(folder, attempt, config["sha256"], harness)
                self.assertFalse(report["expectation_match"])
                self.assertFalse(report["evidence_complete"])

    def test_missing_and_forged_event_sidecars_cannot_qualify(self):
        for mode in ("missing", "drop", "truth", "reorder", "run", "config", "rule", "bool", "duplicate"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                target = Path(directory) / "forged"
                shutil.copytree(self.output, target)
                folder = target / "attempts/active-human-r001"
                path = folder / "safety-labels.jsonl"
                labels = [json.loads(line) for line in path.read_text().splitlines()]
                if mode == "missing":
                    path.unlink()
                else:
                    if mode == "drop": labels.pop()
                    elif mode == "truth": labels[-1]["expected_safety"] = "SAFE"
                    elif mode == "reorder": labels.reverse()
                    elif mode == "run": labels[-1]["run_id"] = "forged"
                    elif mode == "config": labels[-1]["configuration_sha256"] = "forged"
                    elif mode == "rule": labels[-1]["rule_sha256"] = "forged"
                    elif mode == "bool": labels[0]["event_sequence"] = True
                    else: labels.append(copy.deepcopy(labels[-1]))
                    path.write_text("".join(json.dumps(label) + "\n" for label in labels))
                rehash(target)
                self.assertFalse(verify_campaign(target)["valid"])
                attempt = next(a for a in specification()["attempts"] if a["scenario_id"] == "active-human")
                config = json.loads((target / "campaign.json").read_text())
                harness = json.loads((folder / "harness.json").read_text())
                report = evaluate(folder, attempt, config["sha256"], harness)
                self.assertFalse(report["evidence_complete"])
                self.assertFalse(report["safety_adjudication"]["qualified"])
                self.assertIsNone(report["goal_met"])

    def test_empty_sidecar_is_required_even_when_upstream_failure_prevents_all_decisions(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "no-sidecar"
            shutil.copytree(self.output, target)
            folder = target / "attempts/planning-stall-r001"
            path = folder / "safety-labels.jsonl"
            self.assertEqual(path.read_text(), "")
            path.unlink()
            attempt = next(a for a in specification()["attempts"] if a["scenario_id"] == "planning-stall")
            config = json.loads((target / "campaign.json").read_text())
            harness = json.loads((folder / "harness.json").read_text())
            report = evaluate(folder, attempt, config["sha256"], harness)
            self.assertFalse(report["evidence_complete"])
            self.assertFalse(report["expectation_match"])
            self.assertIn("safety_labels_unavailable", report["findings"])


if __name__ == "__main__":
    unittest.main()
