from dataclasses import replace
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

from human_detector import BODY_PARTS, Detector, DetectorConfig, Observation, from_counts, from_demo_evidence


def full_view(**changes):
    return replace(Observation(80, 1.0, {p: 1.0 for p in BODY_PARTS}, 1.0, 0.0), **changes)


class ProbabilityTests(unittest.TestCase):
    def setUp(self):
        self.detector = Detector()

    def test_reference_threshold(self):
        self.assertEqual(self.detector.probability(full_view()), 0.5)

    def test_hidden_is_impossible_for_every_profile(self):
        for name in ("baseline", "stronger", "weaker", "small_target", "occlusion_tolerant"):
            d = Detector(DetectorConfig.profile(name))
            result = d.detect(Observation(0, 0), seed=7, key="hidden")
            self.assertEqual(result["probability"], 0)
            self.assertFalse(result["detected"])

    def test_monotonic_response_to_each_measurement(self):
        p = self.detector.probability
        for sequence in (
            [full_view(visible_pixels=n) for n in (1, 10, 80, 400, 1000)],
            [full_view(visible_body_fraction=f) for f in (.01, .1, .5, 1)],
            [full_view(visible_parts={p: f for p in BODY_PARTS}) for f in (.01, .1, .5, 1)],
            [full_view(contrast=f) for f in (0, .3, .7, 1)],
            [full_view(clutter=f) for f in (1, .7, .3, 0)],
        ):
            scores = [p(o) for o in sequence]
            self.assertTrue(all(a < b for a, b in zip(scores, scores[1:])), scores)

    def test_parts_distinguish_equal_size_and_fraction(self):
        parts = {p: 0.0 for p in BODY_PARTS}
        head = full_view(visible_parts={**parts, "head": 1.0}, visible_body_fraction=.1)
        arm = replace(head, visible_parts={**parts, "left_arm": 1.0})
        self.assertGreater(self.detector.probability(head), self.detector.probability(arm))
        d = Detector(replace(DetectorConfig(), parts_weight=0))
        self.assertEqual(d.probability(head), d.probability(arm))

    def test_disable_all_penalties_gives_size_only(self):
        d = Detector(DetectorConfig(visibility_weight=0, parts_weight=0, contrast_weight=0, clutter_weight=0))
        hard = full_view(visible_body_fraction=.01, visible_parts={p: .01 for p in BODY_PARTS}, contrast=0, clutter=1)
        self.assertEqual(d.probability(hard), .5)

    def test_unknown_features_are_reported_and_neutral(self):
        result = self.detector.explain(Observation(80, 1))
        self.assertEqual(result["probability"], .5)
        self.assertEqual(result["missing_features"], ["visible_parts", "contrast", "clutter"])

    def test_profiles_order_over_many_views_and_samples(self):
        rng = random.Random(42)
        detectors = [Detector(DetectorConfig.profile(n)) for n in ("weaker", "baseline", "stronger")]
        for i in range(300):
            o = full_view(visible_pixels=rng.randint(1, 10000), visible_body_fraction=rng.uniform(.001, 1),
                          visible_parts={p: rng.random() for p in BODY_PARTS}, contrast=rng.random(), clutter=rng.random())
            results = [d.detect(o, seed=17, key=str(i)) for d in detectors]
            ps = [r["probability"] for r in results]
            self.assertEqual(ps, sorted(ps))
            outcomes = [r["detected"] for r in results]
            self.assertEqual(outcomes, sorted(outcomes))

    def test_same_seed_key_is_stateless_and_order_independent(self):
        first = self.detector.detect(full_view(), seed=3, key="person/view")
        self.detector.detect(full_view(), seed=99, key="unrelated")
        self.assertEqual(first, Detector().detect(full_view(), seed=3, key="person/view"))
        self.assertNotEqual(first["draw"], self.detector.detect(full_view(), seed=3, key="different")["draw"])

    def test_sample_frequency_agrees_with_probability(self):
        count = sum(self.detector.detect(full_view(), seed=4, key=str(i))["detected"] for i in range(5000))
        self.assertLess(abs(count / 5000 - .5), .03)


class ValidationTests(unittest.TestCase):
    def test_invalid_observations(self):
        for kwargs in ({"visible_pixels": -1}, {"visible_pixels": True}, {"visible_pixels": 1.2},
                       {"visible_pixels": 300000}, {"visible_body_fraction": 1.1},
                       {"visible_body_fraction": 0}, {"visible_body_fraction": math.nan},
                       {"contrast": math.inf}, {"clutter": -.1}, {"visible_parts": {"head": 1}},
                       {"visible_parts": {p: 0 for p in BODY_PARTS}}, {"image_width": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                full_view(**kwargs)

    def test_reject_removed_factors_and_unknown_settings(self):
        with self.assertRaises(ValueError):
            Observation.from_dict({"visible_pixels": 80, "visible_body_fraction": 1, "pose": "standing"})
        with self.assertRaises(ValueError):
            DetectorConfig.from_dict({"pixel_50": 40})

    def test_invalid_configuration(self):
        for kwargs in ({"pixels_50": 0}, {"pixel_slope": -1}, {"visibility_weight": -1},
                       {"pixel_slope": 1e308}, {"contrast_weight": 1e308},
                       {"parts_weight": math.nan}, {"part_weights": {p: 0 for p in BODY_PARTS}}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DetectorConfig(**kwargs)

    def test_resolution_mismatch(self):
        with self.assertRaisesRegex(ValueError, "resolution"):
            Detector().probability(full_view(image_width=1280))

    def test_config_round_trip_and_immutable_weights(self):
        c = DetectorConfig.profile("stronger")
        self.assertEqual(c, DetectorConfig.from_dict(json.loads(json.dumps(c.to_dict()))))
        with self.assertRaises(TypeError):
            c.part_weights["head"] = .9

    def test_invalid_sampling_identifiers(self):
        for seed, key in ((True, "x"), (1.5, "x"), (1, ""), (1, 4)):
            with self.assertRaises(ValueError):
                Detector().detect(full_view(), seed=seed, key=key)


class AdapterTests(unittest.TestCase):
    def test_counts_and_empty_view(self):
        self.assertEqual(from_counts(visible_pixels=30, unoccluded_pixels=120).visible_body_fraction, .25)
        self.assertEqual(Detector().probability(from_counts(visible_pixels=0, unoccluded_pixels=0)), 0)
        with self.assertRaises(ValueError):
            from_counts(visible_pixels=121, unoccluded_pixels=120)

    def test_actual_recording_format_ignores_cover_and_range(self):
        row = {"pixels": 16, "visibleBodyFraction": .5333333333333333,
               "expectedBodyPixels": 30, "rangeMeters": 82.414, "knownCoverFraction": 0}
        a = from_demo_evidence(row)
        b = from_demo_evidence({**row, "rangeMeters": 1, "knownCoverFraction": .9})
        self.assertEqual(a, b)
        self.assertIsNone(a.visible_parts)
        with self.assertRaises(ValueError):
            from_demo_evidence({"pixels": 16, "knownCoverFraction": .2})


class CliTests(unittest.TestCase):
    def run_cli(self, *args, input=None):
        return subprocess.run([sys.executable, "-m", "human_detector", *args],
                              input=input, text=True, capture_output=True)

    def test_examples_and_cross_process_reproducibility(self):
        path = Path(__file__).resolve().parents[1] / "examples" / "observations.jsonl"
        a = self.run_cli("--input", str(path), "--seed", "7")
        b = self.run_cli("--input", str(path), "--seed", "7")
        self.assertEqual(a.returncode, 0, a.stderr)
        self.assertEqual(a.stdout, b.stdout)
        self.assertEqual(len(a.stdout.splitlines()), 5)
        self.assertEqual(json.loads(a.stdout.splitlines()[-1])["probability"], 0)

    def test_config_export_and_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "config.json")
            self.assertEqual(self.run_cli("--profile", "stronger", "--write-config", path).returncode, 0)
            result = self.run_cli("--config", path, input='{"key":"x","observation":{"visible_pixels":80,"visible_body_fraction":1}}\n')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(json.loads(result.stdout)["probability"], .5)

    def test_bad_input_is_nonzero_with_line_number(self):
        result = self.run_cli(input='{"key":"x","observation":{"visible_pixels":80,"visible_body_fraction":1,"pose":"lying"}}\n')
        self.assertEqual(result.returncode, 2)
        self.assertIn("line 1", result.stderr)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
