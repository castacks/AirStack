"""Execute M02 verification against closed forms, alternate solvers and captures."""
import csv
import json
import math
from pathlib import Path
import random
import statistics
import sys
import unittest

from metrics import local_ssim, michelson, rmse, snr_db
from reference import (analytic_slab, continuous_reference, manufactured_coefficients,
                       piecewise_projection)
from rgb_model import (Layer, Sensor, henyey_greenstein, integrate_response,
                       lambertian, spectral_coefficient, transfer, triangle_response)


def load_rows(name):
    with (CAPTURE / name).open(newline="") as file:
        return list(csv.DictReader(file))


class Verification(unittest.TestCase):
    def close(self, value, reference):
        self.assertTrue(math.isclose(value, reference, rel_tol=2e-12, abs_tol=2e-12),
                        (value, reference))

    def test_analytic_transfer(self):
        for a, s, length, source in ((0, 0, 12, 1), (.08, 0, 12, 1), (.1, .2, 8000, .4),
                                      (0, .12, 10, .4), (.3, .5, 1e-14, 1)):
            actual = transfer(.4, [Layer(length, a, s, source, 1)], 1)
            transmission, radiance = analytic_slab(.4, a, s, length, source)
            self.close(actual.total, radiance)
            self.close(actual.transmission, transmission)
            self.assertTrue(0 <= actual.transmission <= 1)
            self.assertGreaterEqual(actual.path, 0)
            self.assertGreaterEqual(actual.direct, 0)
        a, b = Layer(4, .2, 0, 1, 1), Layer(6, .3, 0, 1, 1)
        self.close(transfer(1, [a, b], 1).transmission, math.exp(-2.6))
        first, second = Layer(4, .02, .03, .5, 1), Layer(6, .07, .02, .1, 1)
        for layers in ((first, second), (second, first)):
            value = .4
            for layer in layers:
                _, value = analytic_slab(value, layer.absorption_m_inv,
                                         layer.scattering_m_inv, layer.length_m,
                                         layer.incident_radiance)
            self.close(transfer(.4, layers, 1).total, value)
        self.close(spectral_coefficient(.2, 400, 600, 2), .45)
        self.close(transfer(.4, [Layer(9, 0, .12, .4, 4 * math.pi)],
                            henyey_greenstein(1, 0)).total, .4)

    def test_spatial_convergence(self):
        baseline = continuous_reference(4096)
        alternate = continuous_reference(2048)
        self.assertLessEqual(abs(baseline - alternate), 1e-12)
        errors = []
        for count in (16, 32, 64):
            width = 12 / count
            layers = []
            for i in range(count):
                a, s, source = manufactured_coefficients(width * (i + .5))
                layers.append(Layer(width, a, s, source, 1))
            errors.append(abs(transfer(.008, layers, 1).total - baseline))
        self.assertLessEqual(errors[-1], 5e-6)
        ratios = [first / second for first, second in zip(errors, errors[1:])]
        self.assertTrue(all(3.7 <= ratio <= 4.3 for ratio in ratios))
        print(f"RK4 reference={baseline:.14g}; refinement errors={errors}; ratios={ratios}", flush=True)

    def test_projection_and_illumination(self):
        grid = [400, 450, 500]
        values = [1, 1.5, 2]
        weights = [0, .5, 1]
        self.close(integrate_response(grid, values, weights), 250 / 3)
        self.close(integrate_response(grid, values, weights),
                   piecewise_projection(grid, values, weights))
        self.close(lambertian(3 * math.pi, .4), 1.2)
        self.close(triangle_response(550, 500, 550, 650), 1)

    def test_phase_and_admissibility(self):
        self.close(henyey_greenstein(-1, 0), 1 / (4 * math.pi))
        size = 10000
        integral = 4 * math.pi / size * math.fsum(
            henyey_greenstein(-1 + (2 * i + 1) / size, .65) for i in range(size))
        self.assertLess(abs(integral - 1), .0001)
        print(f"HG g=.65 angular integral={integral:.10g}", flush=True)
        for call, args in ((Layer, (-1, 0, 0, 0, 0)), (Layer, (1, 0, -1, 0, 0)),
                           (Layer, (1, 0, 0, 0, 20)), (lambertian, (1, 1.01)),
                           (integrate_response, ([1, 2], [0, 1], [0, 2])),
                           (integrate_response, ([2, 1], [0, 1], [0, 1])),
                           (Sensor, (0, 100, 12)), (Sensor, (1, 100, 0)),
                           (henyey_greenstein, (1, 1))):
            with self.assertRaises(ValueError):
                call(*args)

    def test_sensor_and_noise(self):
        sensor = Sensor(4, 1000, 10)
        code, clipped = sensor.capture(100)
        self.assertFalse(clipped)
        self.assertLessEqual(abs(code - .4 * 1023), .5)
        self.assertEqual(sensor.capture(300), (1023, True))
        self.assertEqual(sensor.capture(0), (0, False))
        for read, shot, expected_variance in ((12, False, 144), (0, True, 500)):
            model = Sensor(1, 1000, 24, read, shot)
            runs = []
            for seed in (112, 112, 113):
                rng = random.Random(seed)
                runs.append([model.capture(500, rng)[0] / model.max_code * 1000 - 500
                             for _ in range(8192)])
            self.assertEqual(runs[0], runs[1])
            self.assertNotEqual(runs[1], runs[2])
            mean = statistics.fmean(runs[0])
            variance = statistics.variance(runs[0])
            self.assertLess(abs(mean), .05 * math.sqrt(expected_variance))
            self.assertLess(abs(variance / expected_variance - 1), .05)
            print(f"noise read={read} shot={shot} sample mean={mean:.6g} variance={variance:.6g}",
                  flush=True)

    def test_metrics(self):
        self.close(michelson(3, 1), .5)
        self.assertIsNone(michelson(0, 0))
        self.close(rmse([3, 2], [1, 4]), 2)
        self.close(snr_db([110, 110], [100, 100]), 20)
        self.assertIsNone(snr_db([100], [100]))
        values = [i / 8 for i in range(9)]
        self.close(local_ssim(values, values, 3, 3, 3), 1)
        expected = (2 * .2 * .4 + .01**2) / (.2**2 + .4**2 + .01**2)
        self.close(local_ssim([.2] * 9, [.4] * 9, 3, 3, 3), expected)

    def test_capture_and_trends(self):
        rows = load_rows("pixels.csv")
        metrics = load_rows("metrics.csv")
        summary = {row["case"]: row for row in metrics}
        config = json.loads(Path(__file__).with_name("scene.json").read_text())
        expected = {
            (case["name"], y, x, channel)
            for case in config["cases"] for y in range(config["height"])
            for x in range(config["width"]) for channel in "RGB"
        }
        pixels = {(row["case"], int(row["row"]), int(row["column"]), row["channel"]): row
                  for row in rows}
        self.assertEqual(len(rows), len(expected))
        self.assertEqual(set(pixels), expected)
        self.assertEqual(len(summary), len(config["cases"]))
        self.assertEqual(set(summary), {key[0] for key in expected})
        for row in rows:
            self.assertTrue(0 <= int(row["code"]) <= 2**config["bits"] - 1)
            direct = float(row["direct_W_m2_sr"])
            boundary = float(row["boundary_W_m2_sr"])
            path = float(row["path_W_m2_sr"])
            self.assertTrue(0 <= direct <= boundary and path >= 0 and math.isfinite(path))
            self.assertEqual(row["material"], "light" if (
                int(row["row"]) // config["checker_px"]
                + int(row["column"]) // config["checker_px"]) % 2 else "dark")
            if row["case"] == "clear":
                self.close(path, 0)
                self.close(direct, boundary)

        def sample(case, channel, field):
            return float(pixels[case, 0, 0, channel][field])

        for channel in "RGB":
            self.assertGreater(sample("low_extinction", channel, "direct_W_m2_sr"),
                               sample("medium_extinction", channel, "direct_W_m2_sr"))
            self.assertGreater(sample("medium_extinction", channel, "direct_W_m2_sr"),
                               sample("high_extinction", channel, "direct_W_m2_sr"))
            self.assertGreater(sample("short_range", channel, "direct_W_m2_sr"),
                               sample("long_range", channel, "direct_W_m2_sr"))
            self.assertGreater(float(summary["out_scattering"][f"contrast_{channel}"]),
                               float(summary["isotropic_path"][f"contrast_{channel}"]))
            self.close(sample("dim", channel, "boundary_W_m2_sr"),
                       .25 * sample("clear", channel, "boundary_W_m2_sr"))
            self.close(sample("layered", channel, "direct_W_m2_sr"),
                       sample("layered_reversed", channel, "direct_W_m2_sr"))
            self.assertNotEqual(sample("layered", channel, "path_W_m2_sr"),
                                sample("layered_reversed", channel, "path_W_m2_sr"))
        self.close(float(summary["clear"]["SSIM"]), 1)
        self.assertGreater(float(summary["bright_haze_clip"]["high_clip_fraction"]), 0)
        print(f"{len(summary)} cases; {len(rows)} channel records", flush=True)
        for case in metrics:
            matching = [row for row in rows if row["case"] == case["case"]]
            scale = config["full_well_e"] / ((2**config["bits"] - 1) * config["gain_e_per_W_m2_sr"])
            error = math.sqrt(sum((int(row["code"]) * scale - float(row["clear_W_m2_sr"]))**2
                                  for row in matching) / len(matching))
            self.close(float(case["radiance_RMSE_W_m2_sr"]), error)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
