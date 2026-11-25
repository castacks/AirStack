"""Verify OU moments, transport, model interfaces and captured output."""
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import random
import sys
import unittest

from environment import advance, physical_sample, transition
from reference import confidence, forced_mean, spatial_covariance, temporal_covariance

ROOT = Path(__file__).resolve().parent.parent
CONFIG = json.loads(Path(__file__).with_name("config.json").read_text())


def read(name):
    with (CAPTURE / name).open(newline="") as source:
        yield from csv.DictReader(source)


class Verification(unittest.TestCase):
    def check(self, actual, reference, tolerance=2e-12):
        self.assertLessEqual(abs(actual - reference), tolerance, (actual, reference))

    def estimate(self, group, metric, expected, values, tolerance):
        estimate, lower, upper = confidence(values)
        STATS.writerow((group, metric, len(values), expected, estimate, estimate - expected,
                        lower, upper, tolerance))
        self.check(estimate, expected, tolerance)
        return estimate

    def test_stationary_temporal_and_spatial(self):
        # Co-moving correlations avoid mixing decay with advection.
        arms = defaultdict(list)
        for row in read("ensemble.csv"):
            arms[row["arm"]].append(row)
        summaries = {}
        largest = {"mean": 0, "variance": 0, "temporal": 0, "spatial": 0}
        n = CONFIG["stationary_size"]
        self.assertEqual(len(arms), len(CONFIG["stationary_arms"]))
        for arm in CONFIG["stationary_arms"]:
            name = arm["name"]
            rows = arms[name]
            self.assertEqual([int(row["realization"]) for row in rows], list(range(n)))
            normalized = [float(row["lag_0"]) / arm["sigma"] for row in rows]
            self.assertEqual([row["space_0"] for row in rows], [row["lag_0"] for row in rows])
            mean = self.estimate(name, "mean_over_sigma", 0, normalized, .075)
            variance = self.estimate(name, "variance_over_sigma2", 1,
                                     [x*x for x in normalized], .11)
            largest["mean"] = max(largest["mean"], abs(mean))
            largest["variance"] = max(largest["variance"], abs(variance - 1))
            self.estimate(name, "burden_temperature_normalized_cov", 0, [
                x * float(row["temperature_latent"]) / CONFIG["temperature"]["sigma"]
                for x, row in zip(normalized, rows, strict=True)
            ], .10)
            temporal = []
            for lag in CONFIG["lags_s"][1:]:
                reference = temporal_covariance(lag, arm["tau_s"])
                value = self.estimate(name, f"normalized_cov_lag_{lag}", reference,
                                      [x * float(row[f"lag_{lag}"]) / arm["sigma"]
                                       for x, row in zip(normalized, rows, strict=True)], .11)
                largest["temporal"] = max(largest["temporal"], abs(value - reference))
                temporal.append(value)
            for h in CONFIG["separations_m"]:
                reference = spatial_covariance(h, CONFIG["period_m"])
                value = self.estimate(name, f"normalized_cov_separation_{h}", reference,
                                      [x * float(row[f"space_{h}"]) / arm["sigma"]
                                       for x, row in zip(normalized, rows, strict=True)], .11)
                largest["spatial"] = max(largest["spatial"], abs(value - reference))
            summaries[name] = temporal, variance * arm["sigma"]**2
        coarse, fine = summaries["baseline"][0], summaries["fine"][0]
        difference = max(abs(a - b) for a, b in zip(coarse, fine, strict=True))
        self.assertLessEqual(difference, .14)
        self.assertGreater(summaries["slow"][0][1], coarse[1])
        self.assertLess(summaries["low_amplitude"][1], summaries["baseline"][1])
        self.assertNotEqual([r["lag_0"] for r in arms["baseline"]],
                            [r["lag_0"] for r in arms["alternate"]])
        # First positive e-fold crossing follows a quadratic in cos(θ).
        cosine = (-.5 + math.sqrt(.25 + 2.4 * (.1 + math.exp(-1)))) / 1.2
        length = math.acos(cosine) * CONFIG["period_m"] / (2 * math.pi)
        self.check(spatial_covariance(length, CONFIG["period_m"]), math.exp(-1))
        print(f"{n} realizations per arm; max errors={largest}; coarse/fine={difference:.9g}; "
              f"L_e={length:.9g} m; lag6 baseline/slow={coarse[1]:.9g}/{summaries['slow'][0][1]:.9g}; "
              f"raw variance baseline/low={summaries['baseline'][1]:.9g}/"
              f"{summaries['low_amplitude'][1]:.9g}", flush=True)

    def test_coupled_refinement_and_deterministic_forcing(self):
        config = CONFIG["forced"]
        max_error = 0
        for dt in (2, .5):
            state = (config["clear_mean"],)
            for i in range(round(config["end_s"] / dt) + 1):
                time = i * dt
                max_error = max(max_error, abs(state[0] - forced_mean(time, config)))
                clear, driven = config["clear_mean"], config["driven_mean"] + config["driven_cosine"]
                target = driven if config["switch_s"][0] <= time < config["switch_s"][1] else clear
                state = advance(state, (target,), transition(dt, config["tau_s"], 0))
        self.assertLessEqual(max_error, 2e-12)

        class FixedInnovation:
            def __init__(self, value):
                self.value = value

            def gauss(self, mean, scale):
                return self.value

        rng = random.Random(90001)
        small = transition(.5, config["tau_s"], 1.3)
        large = transition(1, config["tau_s"], 1.3)
        coupling_error = 0
        for _ in range(100):
            start, first, second = (rng.gauss(0, 1) for _ in range(3))
            fine = advance((start,), (.7,), small, FixedInnovation(first))
            fine = advance(fine, (.7,), small, FixedInnovation(second))[0]
            combined = (small[0] * first + second) / math.sqrt(1 + small[0]**2)
            coarse = advance((start,), (.7,), large, FixedInnovation(combined))[0]
            coupling_error = max(coupling_error, abs(fine - coarse))
        self.assertLessEqual(coupling_error, 2e-12)
        values = defaultdict(list)
        for row in read("forced.csv"):
            values[int(float(row["time_s"]))].append(float(row["burden_latent"]))
        self.assertEqual(sorted(values), config["check_s"])
        errors = []
        for time, data in values.items():
            self.assertEqual(len(data), CONFIG["forced_size"])
            expected = forced_mean(time, config)
            observed = self.estimate("forced", f"latent_mean_t{time}", expected, data, .14)
            errors.append(abs(observed - expected))
        print(f"deterministic max error={max_error:.9g}; coupled refinement={coupling_error:.9g}; "
              f"forced mean max error={max(errors):.9g}", flush=True)

    def test_transport_and_bounds(self):
        paths = defaultdict(list)
        terrain = CONFIG["media"]
        max_mapping_error = 0
        for row in read("trajectories.csv"):
            name = row["case"]
            path_key = name, int(float(row["time_s"]))
            paths[path_key].append(row)
            x = {k: float(v) for k, v in row.items() if k != "case"}
            self.assertTrue(all(map(math.isfinite, x.values())))
            self.assertTrue(0 <= x["burden"] <= 1 and 250 <= x["air_K"] <= 330)
            self.assertTrue(0 <= x["scattering_fraction"] <= 1)
            coefficients = ("rgb_extinction_m_inv", "rgb_absorption_m_inv", "rgb_scattering_m_inv",
                            "lwir_8_m_inv", "lwir_11_m_inv", "lwir_14_m_inv")
            self.assertGreaterEqual(min(x[k] for k in coefficients), 0)
            residual = x["rgb_extinction_m_inv"] - x["rgb_absorption_m_inv"] - x["rgb_scattering_m_inv"]
            self.check(residual, 0)
            optical = terrain["dust" if name == "synthetic_dust" else "fog"]
            burden_ref = 1 / (1 + math.exp(-x["burden_latent"]))
            lw = [x[f"lwir_{nm}_m_inv"] for nm in (8, 11, 14)]
            self.assertGreater(len(set(lw)), 1)
            for a, floor, span in zip(lw, optical["lwir_floor"], optical["lwir_span"], strict=True):
                expected = floor + span * burden_ref
                self.check(a, expected)
                max_mapping_error = max(max_mapping_error, abs(a - expected))
            self.check(x["rgb_extinction_m_inv"], optical["rgb_floor"] + optical["rgb_span"]*burden_ref)
            self.check(x["rgb_exponent"], optical["rgb_exponent"])
            self.check(x["scattering_fraction"], optical["scatter_fraction"])
        self.assertEqual(len(paths), 3 * 73 + 10)
        self.assertTrue(all(len(rows) == CONFIG["grid_cells"] for rows in paths.values()))
        position_errors = []
        for t in range(0, 73, 8):
            rows = paths["periodic_transport", t]
            peak = max(rows, key=lambda row: float(row["burden"]))
            error = abs(float(peak["position_m"]) - t % 64)
            self.assertLessEqual(error, 1e-10)
            position_errors.append(error)
            self.check(float(peak["burden"]), 1 / (1 + math.exp(-2)))
        values = []
        for time in CONFIG["forced"]["check_s"]:
            probe = next(row for row in paths["forced_clear_cycle", time]
                         if float(row["position_m"]) == time % 64)
            reference = forced_mean(time, CONFIG["forced"])
            self.check(float(probe["burden_latent"]), reference)
            values.append(float(probe["burden"]))
        self.assertGreaterEqual(values[2] - values[0], .4)
        self.assertLessEqual(abs(values[3] - values[0]), .015)
        for fog, dust in zip(paths["synthetic_fog", 36], paths["synthetic_dust", 36], strict=True):
            self.assertEqual(fog["burden_latent"], dust["burden_latent"])
            self.assertNotEqual(fog["lwir_11_m_inv"], dust["lwir_11_m_inv"])
        stress = list(read("stress.csv"))
        self.assertEqual(len(stress), 16)
        self.assertEqual([int(row["evaluations"]) for row in stress], [8192] * 16)
        self.assertEqual([int(row["invalid"]) for row in stress], [0] * 16)
        print(f"{sum(len(rows) for rows in paths.values())} field samples; 131072 stress samples, "
              f"0 invalid; peak error={max(position_errors)} m; mapping error={max_mapping_error:.9g}; "
              f"clear/driven/recovery={values[0]:.9g}/{values[2]:.9g}/{values[3]:.9g}", flush=True)

    def test_interfaces_and_admissibility(self):
        sys.path.extend(str(ROOT / name) for name in ("m02", "m03", "m04"))
        from rgb_model import Layer, spectral_coefficient, transfer
        from thermal_model import Slab, atmosphere
        from geometry import Camera, Rectangle, hits

        case = CONFIG["media"]["fog"]
        sample = physical_sample(.7, -.5, 1, case, CONFIG["temperature"])
        length, wavelength = 5, 450
        rgb_absorption, rgb_scattering = (spectral_coefficient(k, wavelength, 550, sample.rgb_exponent)
            for k in (sample.rgb_absorption_550_m_inv, sample.rgb_scattering_550_m_inv))
        actual = transfer(1, [Layer(length, rgb_absorption, rgb_scattering, 0, 0)], 0).transmission
        reference = math.exp(-length * sample.rgb_extinction_550_m_inv * (550/450)**case["rgb_exponent"])
        self.check(actual, reference)
        self.check(transfer(1, [Layer(2*length, rgb_absorption/2, rgb_scattering/2, 0, 0)], 0)
                   .transmission, actual)
        lwir = sample.lwir_absorption_m_inv[1]
        result = atmosphere([Slab.absorbing(11, length, lwir, sample.temperature_K)])[0]
        self.check(result, math.exp(-length * lwir))
        camera = Camera(1, 1, 1, 1, 0, 0, (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
        target = Rectangle("target", (0, 0, 5), (1, 1), "target", True)
        for velocity in (sample.velocity_m_s, -sample.velocity_m_s):
            physical_sample(.7, -.5, velocity, case, CONFIG["temperature"])
            _, observed, distance = hits(camera, camera.ray(0, 0), [(target, target.at(0))])
            self.assertEqual((observed.id, distance), ("target", 5))
        self.assertNotEqual(actual, result)
        for invalid in ((0, 6, 1), (2, 0, 1), (2, 6, -1)):
            with self.assertRaises(ValueError):
                transition(*invalid)
        with self.assertRaises(ValueError):
            physical_sample(math.nan, 0, 1, case, CONFIG["temperature"])
        print(f"isolated 5 m kernel transmissions RGB={actual:.9g}, LWIR={result:.9g}; "
              f"abs errors={abs(actual-reference):.3g}/{abs(result-math.exp(-length*lwir)):.3g}; "
              "unchanged M04 target hit and equal-depth ambiguity", flush=True)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    with Path(sys.argv.pop(1)).open("w", newline="") as artifact:
        STATS = csv.writer(artifact, lineterminator="\n")
        STATS.writerow(("ensemble", "metric", "n", "reference", "estimate", "error",
                        "lower_99", "upper_99", "tolerance"))
        unittest.main(verbosity=2)
