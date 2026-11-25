"""Compare M04 outputs against independently projected bounds and radiometry."""
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import sys
import unittest

import analytical
from geometry import Camera, Rectangle, hits, intersect

CONFIG = json.loads(Path(__file__).with_name("scene.json").read_text())


def read(name):
    with (CAPTURE / name).open(newline="") as source:
        return list(csv.DictReader(source))


def key(row):
    return int(row["time_s"]), int(row["row"]), int(row["column"])


def select(table, case):
    return {key(row): {field: value for field, value in row.items() if field != "case"}
            for row in table if row["case"] == case}


class Verification(unittest.TestCase):
    def check_close(self, actual, expected, atol=1e-10, rtol=0):
        self.assertTrue(math.isclose(actual, expected, abs_tol=atol, rel_tol=rtol), (actual, expected))

    def test_camera_and_jacobian(self):
        axes = ((0, 1, 0), (-1, 0, 0), (0, 0, 1))
        camera = Camera(17, 11, 10, 12, 8, 5, (4, -1, 2), axes)
        world = (2, 0, 7)
        self.assertEqual(camera.world_to_camera(world), (1, 2, 5))
        u, v = camera.project(world)
        self.check_close(u, 10)
        self.check_close(v, 9.8)
        direction = camera.ray(u, v)
        range_m = math.sqrt(30)
        for component, wanted in zip(direction, (-2/range_m, 1/range_m, 5/range_m), strict=True):
            self.check_close(component, wanted)
        self.check_close(intersect(camera.origin, direction, world, (1, 1)), range_m)
        derivatives = camera.projection_jacobian(world)
        shift = 1e-5
        errors = []
        for index, axis in enumerate(axes):
            plus = camera.project(tuple(x + shift * a for x, a in zip(world, axis, strict=True)))
            minus = camera.project(tuple(x - shift * a for x, a in zip(world, axis, strict=True)))
            for dim in range(2):
                errors.append(abs((plus[dim] - minus[dim]) / (2 * shift) - derivatives[dim][index]))
        self.assertLessEqual(max(errors), 1e-7)
        print(f"maximum camera Jacobian error={max(errors):.10g} px/m", flush=True)
        with self.assertRaises(ValueError):
            camera.project((4, -1, 1))
        with self.assertRaises(ValueError):
            Camera(17, 11, 10, 12, 8, 5, (0, 0, 0), ((1,0,0), (0,1,0), (0,0,-1)))

    def test_boundary_parallel_and_ties(self):
        origin = (0, 0, 0)
        self.assertEqual(intersect(origin, (0, 0, 1), (1, 0, 5), (1, 1)), 5)
        self.assertIsNone(intersect(origin, (0, 0, 1), (1 + 1e-11, 0, 5), (1, 1)))
        self.assertIsNone(intersect(origin, (1, 0, 1e-12), (5e12, 0, 5), (1, 1)))
        almost_parallel = (math.sqrt(1 - 4e-24), 0, 2e-12)
        self.check_close(intersect(origin, almost_parallel, (2.5e12, 0, 5), (1, 1)), 2.5e12)
        for center in ((0, 0, -1), (0, 0, 1e-9)):
            self.assertIsNone(intersect(origin, (0, 0, 1), center, (1, 1)))
        with self.assertRaises(ValueError):
            Rectangle("bad", origin, (0, 1), "target")
        camera = Camera(**CONFIG["camera"])
        target = Rectangle("target", (0, 0, 5), (1, 1), "target", True)
        for offset, expected in ((0, "a"), (5e-10, "a"), (2e-9, "target")):
            objects = [target, *(Rectangle(name, (0, 0, 5 + offset), (1, 1), "foreground")
                                 for name in ("z", "a"))]
            posed = [(r, r.at(0)) for r in objects]
            for order in (posed, posed[::-1]):
                target_range, first, _ = hits(camera, (0, 0, 1), order)
                self.assertEqual((target_range, first.id), (5, expected))

    def test_masks_ranges_and_timing(self):
        rows, frames = read("geometry.csv"), read("frames.csv")
        rows_by_frame = defaultdict(dict)
        for row in rows:
            frame_key = row["case"], int(row["time_s"])
            index = int(row["row"]), int(row["column"])
            self.assertNotIn(index, rows_by_frame[frame_key])
            rows_by_frame[frame_key][index] = row
        expected_frames = [(case["name"], time) for case in CONFIG["cases"]
                           for time in case.get("times", [0])]
        self.assertEqual([(r["case"], int(r["time_s"])) for r in frames], expected_frames)
        self.assertEqual(set(rows_by_frame), set(expected_frames))
        metrics = {(r["case"], int(r["time_s"])): r for r in frames}
        maximum = 0
        for case in CONFIG["cases"]:
            for time in case.get("times", [0]):
                frame_key = case["name"], time
                frame_rows = rows_by_frame[frame_key]
                support = visible = foreground = 0
                reference = list(analytical.frame(CONFIG, case, time))
                self.assertEqual(len(frame_rows), len(reference))
                for row, col, target_range, first, depth, hit_range in reference:
                    actual = frame_rows[row, col]
                    expected_visible = first == "target"
                    self.assertEqual(actual["target_projected"], str(int(target_range is not None)))
                    self.assertEqual(actual["target_visible"], str(int(expected_visible)))
                    self.assertEqual(actual["first_id"], first)
                    support += target_range is not None
                    visible += expected_visible
                    foreground += bool(first) and not expected_visible
                    for field, value in (("target_only_range_m", target_range),
                                         ("first_range_m", hit_range), ("first_camera_depth_m", depth)):
                        if value is None:
                            self.assertEqual(actual[field], "")
                        else:
                            error = abs(float(actual[field]) - value)
                            maximum = max(maximum, error)
                            self.assertLessEqual(error, 1e-10)
                values = metrics[frame_key]
                self.assertEqual([int(values[k]) for k in
                                  ("projected_target", "visible_target", "foreground_first_hits")],
                                 [support, visible, foreground])
                if support:
                    self.check_close(float(values["visible_fraction"]), visible / support)
                else:
                    self.assertEqual(values["visible_fraction"], "")
        for name, pixels in (("clear", 35), ("partial_1", 30), ("partial_3", 20),
                             ("partial_5", 10), ("complete_distinct", 0), ("far_only", 35)):
            self.assertEqual(int(metrics[name, 0]["visible_target"]), pixels)
        foreground = [metrics["moving_foreground", i] for i in range(9)]
        target = [metrics["moving_target", i] for i in range(7)]
        self.assertEqual([int(foreground[i]["foreground_first_hits"]) for i in (0, 4, 8)],
                         [0, 15, 0])
        self.assertEqual([int(foreground[i]["visible_target"]) for i in (0, 4, 8)],
                         [35, 20, 35])
        self.assertEqual(target[-1]["projected_target"], "0")
        self.assertEqual(select(rows, "overlap"), select(rows, "overlap_reordered"))
        print(f"{len(frames)} frames; {len(rows)} rays; 0 label mismatches; "
              f"maximum range/depth error={maximum:.10g} m", flush=True)

    def test_surface_replacement_and_radiometric_invariance(self):
        geometry, radiance = read("geometry.csv"), read("radiometry.csv")
        geometry_hits = {(row["case"], *key(row)): row for row in geometry if row["first_id"]}
        self.assertEqual(len(geometry_hits), len(radiance))
        self.assertEqual(len({(row["case"], *key(row)) for row in radiance}), len(radiance))
        for result in radiance:
            ray = geometry_hits[result["case"], *key(result)]
            self.assertEqual((ray["first_id"], ray["first_range_m"]),
                             (result["first_id"], result["first_range_m"]))
            case = next(case for case in CONFIG["cases"] if case["name"] == result["case"])
            expected_material = case.get("target_material", "target") if ray["first_id"] == "target" else next(
                CONFIG["obstacles"][name]["material"] for name in case.get("obstacles", [])
                if CONFIG["obstacles"][name]["id"] == ray["first_id"])
            self.assertEqual(result["material"], expected_material)
        self.assertEqual(select(radiance, "complete_distinct"), select(radiance, "complete_target_changed"))
        self.assertEqual(select(radiance, "overlap"), select(radiance, "overlap_reordered"))
        baseline = select(geometry, "partial_3")
        for case in ("matched_radiance", "clipped", "attenuated"):
            self.assertEqual(select(geometry, case), baseline)

        rgb_error = lwir_error = atten_error = 0
        for case, coordinate, material, depth in (("clear", (0, 5, 8), "target", 0),
                ("complete_distinct", (0, 5, 8), "distinct", 0),
                ("attenuated", (0, 3, 5), "target", math.sqrt(113)),
                ("attenuated", (0, 3, 8), "foreground", math.sqrt(26))):
            pixel = select(radiance, case)[coordinate]
            self.assertEqual(pixel["material"], material)
            properties = CONFIG["materials"][material]
            medium = CONFIG["medium"]
            if depth:
                self.check_close(float(pixel["first_range_m"]), depth)
            for channel in "RGB":
                coefficient = medium["rgb_absorption_m_inv"] + medium["rgb_scattering_m_inv"]
                source = medium["rgb_ambient"] * medium["rgb_solid_angle_sr"] / (4 * math.pi)
                source *= medium["rgb_scattering_m_inv"] / coefficient
                expected = analytical.rgb_integral(properties, CONFIG["rgb_response"][channel],
                                                   coefficient * depth, source)
                actual = [float(pixel[f"{channel}_{term}"]) for term in ("direct", "path")]
                rgb_error = max(rgb_error, *(abs(a - b) for a, b in zip(actual, expected, strict=True)))
                for a, b in zip(actual, expected, strict=True):
                    self.check_close(a, b, 2e-12)
            for value, field in zip(analytical.lwir_integrals(CONFIG, properties, depth),
                                    ("lwir_emission", "lwir_reflection", "lwir_path"), strict=True):
                actual = float(pixel[field])
                if depth:
                    self.check_close(actual, value, 0, 2e-5)
                    atten_error = max(atten_error, abs(actual / value - 1))
                else:
                    self.check_close(actual, value, 5e-4)
                    lwir_error = max(lwir_error, abs(actual - value))
        matched = select(radiance, "matched_radiance")
        near, open_pixel = matched[0, 5, 8], matched[0, 5, 5]
        fields = ["R_direct", "G_direct", "B_direct", "lwir_emission", "lwir_reflection"]
        matched_fields = [*fields, "R_code", "G_code", "B_code", "lwir_code"]
        self.assertEqual([near[field] for field in matched_fields],
                         [open_pixel[field] for field in matched_fields])
        self.assertEqual((near["first_id"], open_pixel["first_id"]), ("mid", "target"))
        clipped = select(radiance, "clipped")[0, 5, 5]
        unclipped = select(radiance, "partial_3")[0, 5, 5]
        attenuated = select(radiance, "attenuated")[0, 5, 5]
        self.assertLess(float(attenuated["R_direct"]), float(unclipped["R_direct"]))
        self.assertLess(float(attenuated["lwir_emission"]), float(unclipped["lwir_emission"]))
        self.assertGreater(float(attenuated["R_path"]), 0)
        self.assertGreater(float(attenuated["lwir_path"]), 0)
        self.assertEqual([clipped[k] for k in fields], [unclipped[k] for k in fields])
        self.assertEqual((clipped["R_clipped"], clipped["lwir_clipped"]), ("1", "1"))
        self.assertEqual((clipped["R_code"], clipped["lwir_code"]), ("4095", "4095"))
        self.assertNotEqual((clipped["R_code"], clipped["lwir_code"]),
                            (unclipped["R_code"], unclipped["lwir_code"]))
        print(f"{len(radiance)} first-hit observations; RGB max abs error={rgb_error:.10g}; "
              f"LWIR clear max component error={lwir_error:.10g}; "
              f"attenuated LWIR max relative error={atten_error:.10g}", flush=True)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
