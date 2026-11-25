"""Verify thermal transfer and audit the frozen experiment output."""
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import random
import statistics
import sys
import unittest

from experiment import render
from reference import (STEFAN_BOLTZMANN, combined_band_reference, integrate_simpson,
                       interpolate, planck_reference, rk4_layers)
from thermal_model import Detector, Slab, atmosphere, planck_um, surface_components

CONFIG = json.loads(Path(__file__).with_name("scene.json").read_text())


class Verification(unittest.TestCase):
    def close(self, actual, expected, relative=2e-12, absolute=2e-12):
        self.assertTrue(math.isclose(actual, expected, rel_tol=relative, abs_tol=absolute),
                        (actual, expected))

    def test_planck(self):
        errors = []
        for wavelength in (8, 10, 14):
            previous = 0
            for temperature in (280, 300, 320, 340):
                value = planck_um(wavelength, temperature)
                truth = planck_reference(wavelength, temperature)
                errors.append(abs(value / truth - 1))
                self.close(value, truth, absolute=0)
                self.assertGreater(value, previous)
                previous = value
        peak = planck_um(9.659, 300)
        self.assertGreater(peak, max(planck_um(9.5, 300), planck_um(9.8, 300)))
        for wavelength in (.05, 10000):
            value = planck_um(wavelength, 280)
            self.assertTrue(value >= 0 and math.isfinite(value))
        with self.assertRaises(ValueError):
            planck_um(10, 0)
        print(f"maximum pointwise relative error={max(errors):.10g}", flush=True)

    def test_stefan_boltzmann(self):
        low, high = math.log(.05), math.log(10000)
        for temperature in (280, 300, 320, 340):
            def integrand(log_wavelength):
                wavelength = min(10000, max(.05, math.exp(log_wavelength)))
                return planck_um(wavelength, temperature) * wavelength
            a, b = (integrate_simpson(integrand, low, high, count) for count in (2048, 4096))
            target = STEFAN_BOLTZMANN * temperature**4 / math.pi
            first_error, second_error = (abs(value / target - 1) for value in (a, b))
            self.assertLessEqual(max(first_error, second_error), 2e-8)
            self.assertLessEqual(abs(a - b) / target, 2e-9)
            print(f"T={temperature}: Stefan-Boltzmann rel. errors={first_error:.10g},"
                  f"{second_error:.10g}; refinement={abs(a-b)/target:.10g}", flush=True)

    def test_surface_and_gray_atmosphere(self):
        self.assertEqual(surface_components(12, 0, 7), (0, 7))
        self.assertEqual(surface_components(12, 1, 7), (12, 0))
        for eps in (.2, .5, .8, 1):
            self.close(sum(surface_components(7, eps, 7)), 7)
        for given in (0, .25, .5, .75, 1):
            t, path = atmosphere([Slab.gray_fixture(10, given, 292)])
            self.close(t, given)
            self.close(t * 12 + path, given * 12 + (1 - given) * planck_reference(10, 292))
        for arguments in ((12, -1, 7), (12, 1.1, 7), (12, .5, -7)):
            with self.assertRaises(ValueError):
                surface_components(*arguments)
        with self.assertRaises(ValueError):
            Slab.absorbing(10, 1, -.1, 300)

    def test_layered_reference(self):
        largest, refinement = 0, 0
        for wavelength in (8, 10, 14):
            slabs = []
            conditions = []
            for layer in CONFIG["layers"]:
                length = layer["length_m"]
                coefficient = interpolate(wavelength, layer["absorption_m_inv"])
                temperature = layer["temperature_K"]
                slabs.append(Slab.absorbing(wavelength, length, coefficient, temperature))
                conditions.append((length, coefficient, planck_reference(wavelength, temperature)))
            for reverse in (False, True):
                selected = list(reversed(slabs)) if reverse else slabs
                condition = list(reversed(conditions)) if reverse else conditions
                t, path = atmosphere(selected)
                a, b = (rk4_layers(12, condition, steps) for steps in (128, 256))
                largest = max(largest, abs(12 * t + path - b))
                refinement = max(refinement, abs(a - b))
            forward, reverse = atmosphere(slabs), atmosphere(reversed(slabs))
            self.close(forward[0], reverse[0])
            self.assertNotEqual(forward[1], reverse[1])
        print(f"layer RK4 max absolute error={largest:.10g}; refinement={refinement:.10g}",
              flush=True)
        self.assertLessEqual(largest, 2e-8)
        self.assertLessEqual(refinement, 2e-8)

    def test_integration_convergence(self):
        case = next(case for case in CONFIG["controls"] if case["name"] == "combined")
        a, b = (combined_band_reference(CONFIG, case, count) for count in (1536, 3072))
        self.assertLessEqual(abs(a - b), 2e-8)
        errors = [abs(render(CONFIG, case, "target_ray", count)[-1][-1] - b)
                  for count in (48, 96, 192)]
        ratios = [first / second for first, second in zip(errors, errors[1:])]
        print(f"Simpson band reference={b:.12g}; refinement={abs(a-b):.10g}; "
              f"trapezoid errors={errors}; ratios={ratios}", flush=True)
        self.assertLessEqual(errors[-1], 2e-4)
        self.assertTrue(all(3.7 <= ratio <= 4.3 for ratio in ratios))

    def test_detector(self):
        detector = Detector(0, 100, 12, .3)
        _, code, low, high = detector.measure(50)
        self.assertFalse(low or high)
        self.assertLessEqual(abs(detector.decode(code) - 50), detector.lsb / 2)
        self.assertEqual(detector.measure(110)[1:], (4095, False, True))
        self.assertEqual(Detector(5, 100, 12, 0).measure(0)[1:], (0, True, False))
        with self.assertRaises(ValueError):
            Detector(100, 5, 12, 0)
        samples = []
        for seed in (101, 101, 102):
            rng = random.Random(seed)
            samples.append([detector.measure(50, rng)[0] - 50 for _ in range(8192)])
        self.assertEqual(samples[0], samples[1])
        self.assertNotEqual(samples[1], samples[2])
        mean, variance = statistics.fmean(samples[0]), statistics.variance(samples[0])
        self.assertLessEqual(abs(mean), .05 * detector.sigma)
        self.assertLessEqual(abs(variance / detector.sigma**2 - 1), .05)
        print(f"noise sample mean={mean:.10g}; variance={variance:.10g}", flush=True)

    def test_captures_and_crossover(self):
        def read(name):
            with (CAPTURE / name).open(newline="") as source:
                return list(csv.DictReader(source))
        spectra, detector, summaries = (read(name) for name in ("spectra.csv", "detector.csv", "metrics.csv"))
        summary = {row["case"]: row for row in summaries}
        count = sum(len(values) for values in CONFIG["sweeps"].values()) + len(CONFIG["controls"])
        self.assertEqual(len(summaries), count)
        expected_cases = {f"{sweep}_{value}" for sweep, values in CONFIG["sweeps"].items()
                          for value in values} | {case["name"] for case in CONFIG["controls"]}
        self.assertEqual(set(summary), expected_cases)
        self.assertEqual(len(spectra), 2 * count * (CONFIG["intervals"] + 1))
        self.assertEqual(len(detector), 2 * count * CONFIG["detector"]["repeats"])
        spectra_by_ray, readings = defaultdict(list), defaultdict(list)
        for row in spectra:
            self.assertTrue(0 <= float(row["transmission"]) <= 1)
            self.assertTrue(all(math.isfinite(float(value)) and float(value) >= 0
                                for key, value in row.items() if key.endswith("_W_m2_sr_um")))
            self.close(float(row["observed_W_m2_sr_um"]), sum(float(row[key + "_W_m2_sr_um"])
                       for key in ("direct_emission", "direct_reflection", "path")))
            spectra_by_ray[row["case"], row["ray"]].append(row)
        for row in detector:
            readings[row["case"], row["ray"]].append(row)
        for case, metrics in summary.items():
            for ray, label in (("target_ray", "first"), ("background_ray", "background")):
                band = float(metrics[label + "_band"])
                group = spectra_by_ray[case, ray]
                self.assertEqual(len(group), CONFIG["intervals"] + 1)
                self.assertEqual(group[0]["surface"], metrics[label + "_surface"])
                for component in ("direct_emission", "direct_reflection", "path", "observed"):
                    field = component + "_W_m2_sr_um"
                    trapezoids = [
                        (float(b["wavelength_um"]) - float(a["wavelength_um"]))
                        * (float(a["response"]) * float(a[field])
                           + float(b["response"]) * float(b[field])) / 2
                        for a, b in zip(group, group[1:])]
                    expected_value = band if component == "observed" else float(metrics[f"{label}_{component}"])
                    self.close(expected_value, sum(trapezoids))
                self.close(band, sum(float(metrics[f"{label}_{key}"])
                                     for key in ("direct_emission", "direct_reflection", "path")))
                group = readings[case, ray]
                self.assertEqual([int(row["sample"]) for row in group],
                                 list(range(CONFIG["detector"]["repeats"])))
                decoded = [float(row["decoded"]) for row in group]
                self.close(float(metrics[label + "_decoded_sd"]), statistics.stdev(decoded))
                self.close(float(metrics[label + "_decoded_rmse"]),
                           math.sqrt(sum((value - band)**2 for value in decoded) / len(decoded)))
                self.close(float(metrics[label + "_clip_fraction"]),
                           sum(int(row["high_clip"]) + int(row["low_clip"]) for row in group)
                           / len(group))
            if metrics["target_visible"] == "1":
                first, background = (float(metrics[name + "_band"])
                                     for name in ("first", "background"))
                delta = first - background
                self.close(float(metrics["target_delta"]), delta)
                if first + background:
                    self.close(float(metrics["normalized_target_delta"]), delta / (first + background))
                else:
                    self.assertEqual(metrics["normalized_target_delta"], "")
                self.assertEqual(int(metrics["within_threshold"]),
                                 abs(delta) <= float(metrics["detector_threshold"]))
        for case in ("equal_radiance_visible", "gray_transmission_0"):
            self.assertEqual(summary[case]["target_visible"], "1")
            self.close(float(summary[case]["target_delta"]), 0)
            self.assertEqual(summary[case]["within_threshold"], "1")
        hidden = summary["foreground_hidden_target"]
        self.assertEqual((hidden["target_visible"], hidden["target_delta"], hidden["within_threshold"],
                          hidden["first_surface"]), ("0", "", "", "foreground"))
        for case in ("high_saturation", "low_saturation"):
            self.assertEqual(float(summary[case]["first_clip_fraction"]), 1)
        scan = []
        for row in summaries:
            if row["sweep"] == "crossover_temperature_K":
                self.assertEqual(row["target_visible"], "1")
                self.assertEqual(float(row["first_clip_fraction"]), 0)
                self.assertEqual(float(row["background_clip_fraction"]), 0)
                scan.append((float(row["first_temperature_K"]), float(row["target_delta"])))
        brackets = [(a, b) for a, b in zip(scan, scan[1:]) if a[1] * b[1] < 0]
        self.assertEqual(len(brackets), 1)
        ((low, delta_low), (high, delta_high)), = brackets
        result = low - delta_low * (high - low) / (delta_high - delta_low)
        print(f"{count} cases; {len(spectra)} spectra; {len(detector)} detector readings; "
              f"crossover {result:.8f} K bracketed [{low:g},{high:g}] K; "
              f"contrast at endpoints={delta_low:.8f}, {delta_high:.8f}", flush=True)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
