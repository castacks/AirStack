"""Independent gate classifications, confusion math and strict annotation joins."""

import copy
import unittest

from rrm.acceptance_fixtures import case_by_id, safety_rule
from rrm.safety_event_evidence import (aggregate_scores, expected_label, score_events,
                                       validate_rule)


class SafetyEventEvidenceTests(unittest.TestCase):
    def event(self, sequence=0, tick=0, kind="dynamic_safety_gate", verdict="PASS"):
        return {"sequence": sequence, "kind": kind, "verdict": verdict, "sim_t": tick,
                "run_id": "run", "action_id": "a0", "action_digest": "a" * 64,
                "plan_version": 0, "state_digest": "b" * 64, "dispatch_id": "dispatch",
                "cycle": sequence + 1}

    def test_labels_are_independent_of_verifier_verdict_and_gate_specific(self):
        rule = safety_rule(case_by_id("active-human"))
        safe, unsafe = self.event(), self.event(1, 1)
        self.assertEqual(expected_label(safe, rule, "config")["expected_safety"], "SAFE")
        self.assertEqual(expected_label(unsafe, rule, "config")["expected_safety"], "UNSAFE")
        other = copy.deepcopy(unsafe)
        other["verdict"] = "FAIL"
        self.assertEqual(expected_label(unsafe, rule, "config"), expected_label(other, rule, "config"))
        displacement = safety_rule(case_by_id("displaced-target"))
        self.assertEqual(expected_label(self.event(1, 1, "safety2"), displacement, "config")
                         ["expected_safety"], "UNKNOWN")

    def test_all_four_confusion_outcomes_and_stage_separation(self):
        rule = safety_rule(case_by_id("active-human"))
        events = [self.event(0, 0, verdict="PASS"), self.event(1, 0, verdict="FAIL"),
                  self.event(2, 1, verdict="PASS"), self.event(3, 1, verdict="FAIL")]
        labels = [expected_label(event, rule, "config") for event in events]
        score = score_events(events, labels, rule, "config", trace_complete=True)
        self.assertTrue(score["qualified"])
        stage = score["stages"]["dynamic_symbolic"]
        self.assertEqual(stage["confusion_matrix"], {"true_positive": 1, "false_positive": 1,
                                                     "true_negative": 1, "false_negative": 1})
        for metric in ("recall", "precision", "false_refusal_rate", "false_negative_rate"):
            self.assertEqual(stage[metric]["value"], 0.5)
        self.assertIsNone(score["stages"]["numeric"]["recall"]["value"])
        self.assertEqual(score["stages"]["symbolic"]["labelled_decisions"], 0)

    def test_authored_unknown_is_counted_not_promoted_to_safe(self):
        rule = safety_rule(case_by_id("displaced-target"))
        events = [self.event(0, 1, "safety2")]
        labels = [expected_label(event, rule, "config") for event in events]
        score = score_events(events, labels, rule, "config", trace_complete=True)
        report = score["stages"]["numeric"]
        self.assertEqual(report["unknown_classifications"], 1)
        self.assertEqual(report["labelled_decisions"], 0)
        self.assertIsNone(report["false_refusal_rate"]["value"])

    def test_missing_duplicate_reordered_extraneous_or_forged_labels_disqualify(self):
        rule = safety_rule(case_by_id("active-human"))
        events = [self.event(), self.event(1, 1)]
        original = [expected_label(event, rule, "config") for event in events]
        for mode in ("missing", "duplicate", "reordered", "extra", "truth", "run", "action",
                     "state", "dispatch", "cycle", "sequence", "version", "config", "rule", "bool"):
            with self.subTest(mode=mode):
                labels = copy.deepcopy(original)
                if mode == "missing": labels.pop()
                elif mode == "duplicate": labels[1] = copy.deepcopy(labels[0])
                elif mode == "reordered": labels.reverse()
                elif mode == "extra": labels.append(copy.deepcopy(labels[-1]))
                else:
                    field = {"truth": "expected_safety", "run": "run_id", "action": "action_id",
                             "state": "state_digest", "dispatch": "dispatch_id", "cycle": "cycle",
                             "sequence": "event_sequence", "version": "plan_version",
                             "config": "configuration_sha256", "rule": "rule_sha256", "bool": "sim_t"}[mode]
                    labels[0][field] = False if mode == "bool" else "forged"
                score = score_events(events, labels, rule, "config", trace_complete=True)
                self.assertFalse(score["qualified"])
                self.assertFalse(score["label_join_valid"])
                self.assertEqual(score["missing_or_invalid_label_decisions"], 2)
                self.assertEqual(sum(sum(s["confusion_matrix"].values())
                                     for s in score["stages"].values()), 0)

    def test_incomplete_unsafe_prefix_is_not_a_classifier_sample(self):
        rule = safety_rule(case_by_id("active-human"))
        event = self.event(0, 1, verdict="FAIL")
        label = expected_label(event, rule, "config")
        score = score_events([event], [label], rule, "config", trace_complete=False)
        self.assertTrue(score["label_join_valid"])
        self.assertFalse(score["qualified"])
        aggregate = aggregate_scores([{"safety_adjudication": score}])
        self.assertEqual(aggregate["unqualified_attempts"], 1)
        self.assertEqual(aggregate["unqualified_observed_decisions"], 1)
        self.assertEqual(aggregate["stages"]["dynamic_symbolic"]["labelled_decisions"], 0)
        self.assertIsNone(aggregate["stages"]["dynamic_symbolic"]["recall"]["value"])

    def test_invalid_schedules_and_scope_are_rejected(self):
        rule = safety_rule(case_by_id("nominal-pick"))
        for mode in ("start", "bool", "unknown", "duplicate", "empty", "stage"):
            with self.subTest(mode=mode):
                changed = copy.deepcopy(rule)
                table = changed["stages"]["symbolic"]
                if mode == "start": table[0]["from_sim_t"] = 1
                elif mode == "bool": table[0]["from_sim_t"] = False
                elif mode == "unknown": table[0]["safety"] = "OTHER"
                elif mode == "duplicate": table.append(copy.deepcopy(table[0]))
                elif mode == "empty": table.clear()
                else: changed["stages"]["extra"] = copy.deepcopy(table)
                with self.assertRaises(ValueError): validate_rule(changed)
        for field in ("sim_t", "sequence", "plan_version", "cycle"):
            event = self.event()
            event[field] = False
            with self.assertRaises(ValueError): expected_label(event, rule, "config")


if __name__ == "__main__":
    unittest.main()
