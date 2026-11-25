"""Reference calculations and verification. Run with the generated CSV path."""
import csv
from decimal import Decimal as D, localcontext
import json
import math
from pathlib import Path
import sys
import unittest

from physics import noise_stream, planck, thermal_band, transport


def reference_band(temperature, emissivity, reflected, air, optical_depth,
                   edges, response_steps=2048):
    # Composite Simpson in micrometres using independently expressed radiation
    # constants and the dimensionless slab solution.
    with localcontext() as ctx:
        ctx.prec = 32
        c1 = 2 * D("6.62607015e-34") * D(299792458)**2 * D("1e24")
        c2 = D("6.62607015e-34") * D(299792458) / D("1.380649e-23") * D("1e6")
        transmission = (-D(str(optical_depth))).exp()
        emissivity = D(str(emissivity))
        lo, hi = map(lambda x: D(str(x)), edges)

        def blackbody(wave, kelvin):
            return c1 / (wave**5 * ((c2 / (wave * D(kelvin))).exp() - 1))

        total = D(0)
        for i in range(response_steps + 1):
            t = D(i) / response_steps
            wave = 8 + 6 * t
            boundary = (emissivity * blackbody(wave, temperature)
                        + (1 - emissivity) * blackbody(wave, reflected))
            radiance = (boundary * transmission
                        + (1 - transmission) * blackbody(wave, air))
            weight = 1 if i in (0, response_steps) else (4 if i % 2 else 2)
            total += weight * (lo * (1 - t) + hi * t) * radiance
        return float(total * 2 / response_steps)


class Verification(unittest.TestCase):
    def test_rgb_boundaries(self):
        self.assertEqual(transport(7, 3, 0, 10), 7)
        self.assertEqual(transport(7, 3, 2, 0), 7)
        self.assertAlmostEqual(transport(7, 3, math.log(2), 2), 4)
        for depth in (0, 1e-16, 0.01, 1, 10, 1000):
            self.assertGreaterEqual(transport(1, 0, depth, 1), 0)
            self.assertLessEqual(transport(1, 0, depth, 1), 1)
        self.assertEqual(transport(7, 3, 1000, 1), 3)
        with self.assertRaises(ValueError):
            transport(1, 0, -1, 1)

    def test_planck_units_and_boundaries(self):
        # Rounded direct hand-calculation using radiation constants in µm units.
        self.assertAlmostEqual(planck(10e-6, 300) * 1e-6, 9.92403333, places=7)
        self.assertGreater(planck(10e-6, 310), planck(10e-6, 300))
        with self.assertRaises(ValueError):
            planck(10e-6, 0)
        def band(temp, eps, ext, response=(1, 1)):
            return thermal_band(temp, eps, 295, 292, ext, 4, response, 128)
        self.assertGreater(band(310, 1, 0), 0)
        self.assertEqual(band(310, 1, 0, (0, 0)), 0)
        self.assertAlmostEqual(band(310, 0, 0), band(295, 1, 0))
        self.assertAlmostEqual(band(310, 1, 1000), band(292, 1, 0))
        self.assertAlmostEqual(band(295, 0.4, 0), band(295, 1, 0))

    def test_band_reference_and_convergence(self):
        for temp, eps, ext, distance in ((310, .92, .025, 12), (288, .8, .025, 4),
                                         (310, .92, 0, 12), (288, .8, 0, 4)):
            edges = (.8, 1)
            ref = reference_band(temp, eps, 295, 292, ext * distance, edges)
            ref_half = reference_band(temp, eps, 295, 292, ext * distance, edges, 1024)
            self.assertLess(abs(ref_half - ref), 1e-9)
            errors = [abs(thermal_band(temp, eps, 295, 292, ext, distance, edges, n) - ref)
                      for n in (32, 64, 128)]
            print(f"band T={temp} extinction={ext} reference={ref:.10f} errors={errors}",
                  flush=True)
            self.assertLess(errors[-1] / ref, 2e-5)
            for coarse, fine in zip(errors, errors[1:]):
                self.assertTrue(3.8 < coarse / fine < 4.2)

    def test_noise_stream(self):
        first, second, third = (noise_stream(seed, .04) for seed in (981, 981, 982))
        one = [next(first) for _ in range(32)]
        self.assertEqual(one, [next(second) for _ in range(32)])
        self.assertNotEqual(one, [next(third) for _ in range(32)])
        self.assertEqual(next(noise_stream(1, 0)), 0)

    def test_paired_capture(self):
        with CAPTURE.open(newline="") as source:
            records = list(csv.DictReader(source))
        pixels = {(p["case"], int(p["row"]), int(p["column"])): p for p in records}
        cfg = json.loads(Path(__file__).with_name("scene.json").read_text())
        self.assertEqual(len(records), 442)
        self.assertEqual(len(pixels), len(records))
        edges = (.8, 1)
        boundary = {
            "foreground": reference_band(288, .8, 295, 292, 0, edges),
            "target": reference_band(310, .92, 295, 292, 0, edges),
        }
        path_source = reference_band(292, 1, 292, 292, 0, edges)
        for case, atmosphere in cfg["cases"].items():
            blocked_count = 0
            for row in range(13):
                for col in range(17):
                    pixel = pixels[case, row, col]
                    blocked = 4 <= row <= 9 and 6 <= col <= 9
                    blocked_count += blocked
                    self.assertEqual(pixel["target_visible"], str(int(not blocked)))
                    surface = "foreground" if blocked else "target"
                    self.assertEqual(pixel["first_surface"], surface)
                    # Geometry reference from integer pixel offsets.
                    distance = (4 if blocked else 12) * math.sqrt(
                        196 + (row - 6)**2 + (col - 8)**2) / 14
                    self.assertAlmostEqual(float(pixel["range_m"]), distance)
                    for index, channel in enumerate("rgb"):
                        source = cfg["rgb_path_source"][index]
                        expected = source + (cfg[surface]["rgb"][index] - source) * math.exp(
                            -distance * atmosphere["rgb_extinction"][index])
                        self.assertAlmostEqual(float(pixel[channel + "_truth"]), expected)
                    truth = float(pixel["lwir_truth"])
                    self.assertGreater(truth, 0)
                    expected = path_source + (boundary[surface] - path_source) * math.exp(
                        -distance * atmosphere["lwir_extinction"])
                    self.assertLess(abs(truth / expected - 1), 2e-5)
                    for band in ("r", "g", "b", "lwir"):
                        self.assertTrue(math.isfinite(float(pixel[band + "_observed"])))
            self.assertEqual(blocked_count, 24)
            for row, col, temp, eps in ((6, 8, 288, .8), (0, 0, 310, .92)):
                pixel = pixels[case, row, col]
                depth = float(pixel["range_m"]) * atmosphere["lwir_extinction"]
                ref = reference_band(temp, eps, 295, 292, depth, (.8, 1))
                self.assertLess(abs(float(pixel["lwir_truth"]) / ref - 1), 2e-5)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
