"""Qualify the synchronized corpus and verify its scientific controls."""
import csv
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import unittest

from adapters import KERNELS, ROOT, optical_depth, segments
from experiment import FRAME_FIELDS, OBS_FIELDS, PAIR_FIELDS, SEGMENT_FIELDS
import reference

HERE = Path(__file__).resolve().parent
CFG = json.loads((HERE / "config.json").read_text())
CASES = {case["id"]: case for case in CFG["cases"]}
CASES.update(ensemble_fixed={}, ensemble_covered={"obstacle": "partial"})
BANDS = ("R", "G", "B", "lwir")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(capture, filename, fields):
    with (capture / filename).open(newline="") as f:
        reader = csv.DictReader(f)
        require(tuple(reader.fieldnames) == fields, "invalid CSV schema")
        return list(reader)


def frame_key(row):
    return row["case"], int(row["realization"]), int(row["time_s"])


def index(rows, key):
    result = {key(row): row for row in rows}
    require(len(result) == len(rows), "duplicate key")
    return result


def check_finite(row, fields):
    require(all(math.isfinite(float(row[field])) for field in fields), "nonfinite value")


def qualify(frames, pairs, paths):
    frame_by_id = index(frames, frame_key)
    pixel_by_id = index(pairs, lambda row: (*frame_key(row), int(row["row"]), int(row["column"])))
    path_by_id = defaultdict(list)
    for (pid, _), row in index(paths, lambda row: (row["path_id"], int(row["segment"]))).items():
        path_by_id[pid].append(row)
        check_finite(row, SEGMENT_FIELDS[3:])
    by_frame = defaultdict(list)
    for row in pixel_by_id.values():
        key = frame_key(row)
        require(key in frame_by_id, "missing synchronized frame")
        fr = frame_by_id[key]
        require(row["environment"] == fr["environment"] and row["scene"] == fr["scene"],
                "broken synchronization")
        check_finite(row, ("dx", "dy", "dz"))
        require(row["target_visible"] in ("0", "1") and row["target_projected"] in ("0", "1"),
                "nonbinary mask")
        by_frame[key].append(row)
        if row["status"] == "no_hit":
            require(all(not row[field] for field in PAIR_FIELDS[13:]), "nonempty no-hit values")
            continue
        require(row["path_id"] in path_by_id
                and path_by_id[row["path_id"]][0]["environment"] == fr["environment"],
                "missing or mismatched environment path")
        check_finite(row, PAIR_FIELDS[15:])
    require(set(by_frame) == set(frame_by_id), "incomplete frame capture")
    require({row["path_id"] for row in pairs if row["path_id"]} == set(path_by_id),
            "unused atmospheric path")
    return frame_by_id, pixel_by_id, path_by_id, by_frame


def write_statistics(capture):
    with (capture / "statistics.csv").open("w", newline="") as file:
        csv.writer(file, lineterminator="\n").writerows(STAT_ROWS)


class Verification(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frames = load(CAPTURE, "frames.csv", FRAME_FIELDS)
        cls.pairs = load(CAPTURE, "pairs.csv", PAIR_FIELDS)
        cls.segments = load(CAPTURE, "segments.csv", SEGMENT_FIELDS)
        cls.frame, cls.pixel, cls.path, cls.by_frame = qualify(cls.frames, cls.pairs, cls.segments)

    def close(self, actual, expected, tolerance=2e-12):
        self.assertLessEqual(abs(float(actual) - expected), tolerance)

    def test_01_shared_geometry(self):
        max_error = 0
        scenes = set()
        for (name, realization, time), frame in self.frame.items():
            case = CASES[name]
            rows = self.by_frame[name, realization, time]
            expected_coords = (
                [tuple(CFG["ensemble"]["probe"])] if name.startswith("ensemble_") else
                [(r, c) for r in range(CFG["camera"]["height"])
                 for c in range(CFG["camera"]["width"])])
            self.assertEqual([(int(row["row"]), int(row["column"])) for row in rows], expected_coords)
            projected = visible = 0
            scenes.add(frame["scene"])
            for row in rows:
                pr, first, material, hit = reference.expected_hit(
                    CFG, case, time, int(row["row"]), int(row["column"]))
                status = ("no_hit" if not first else "target_visible" if first == "target" else
                          "target_occluded" if pr else "other_hit")
                self.assertEqual((row["first_id"], row["material"], row["status"]),
                                 (first, material, status))
                projected += pr
                visible += first == "target"
                self.assertEqual((int(row["target_projected"]), int(row["target_visible"])),
                                 (int(pr), int(first == "target")))
                if hit:
                    for field, expected in zip(("first_x_m", "first_y_m", "first_z_m", "first_range_m"),
                                               hit, strict=True):
                        error = abs(float(row[field]) - expected)
                        max_error = max(max_error, error)
                        self.assertLessEqual(error, 2e-12)
                    self.close(row["first_camera_depth_m"], hit[2])
            self.assertEqual((int(frame["projected"]), int(frame["visible"])), (projected, visible))
            if projected:
                self.close(frame["visible_fraction"], visible / projected)
            else:
                self.assertEqual(frame["visible_fraction"], "")
        print(f"{len(scenes)} layouts, {len(self.frames)} frames, {len(self.pairs)} ray rows; "
              f"0 label or timestamp mismatches; max geometry error={max_error:.10g} m", flush=True)

    def test_02_legacy_probes_and_modal_divergence(self):
        old = json.loads((HERE / "legacy_probe.json").read_text())
        largest = 0
        for before, current in (("clear", "clear"), ("attenuated", "homogeneous_dual")):
            sample = self.pixel[current, 0, 0, 3, 1]
            for key, expected in old[before].items():
                if key.endswith("code"):
                    self.assertEqual(int(sample[key]), int(expected))
                else:
                    self.close(sample[key], expected)
                    largest = max(largest, abs(float(sample[key]) - expected))
        for arm, degraded, clear in (("rgb_dominant", "rgb_tau550", "lwir_tau11"),
                                     ("lwir_dominant", "lwir_tau11", "rgb_tau550")):
            sample = self.pixel[arm, 0, 0, 3, 4]
            self.assertLess(float(sample[degraded]), .8)
            self.close(sample[clear], 1)
        target = self.pixel["crossover", 0, 0, 3, 4]
        background = self.pixel["crossover", 0, 0, 0, 4]
        lwir = ("lwir_emission", "lwir_reflection", "lwir_path")
        thermal_delta = sum(float(target[k]) - float(background[k]) for k in lwir)
        visible_delta = sum(float(target[k]) - float(background[k]) for k in ("R_direct", "R_path"))
        self.close(thermal_delta, 0)
        self.assertGreater(visible_delta, 0)
        self.assertEqual(target["target_visible"], "1")
        noisy = self.pixel["noise_and_clip", 0, 0, 3, 4]
        clear = self.pixel["clear", 0, 0, 3, 4]
        self.assertEqual((noisy["target_visible"], noisy["R_high_clip"], noisy["lwir_high_clip"]),
                         ("1", "1", "1"))
        self.assertEqual(noisy["R_direct"], clear["R_direct"])
        self.assertNotEqual(noisy["R_analog"], clear["R_analog"])
        noisy_frame = self.by_frame["noise_and_clip", 0, 0]
        STAT_ROWS.extend([
            ("crossover_R_target_minus_background_W_m2_sr", 1, visible_delta, "", ""),
            ("crossover_lwir_target_minus_background_W_m2_sr", 1, thermal_delta, "", ""),
            ("noisy_frame_R_high_clip_fraction", len(noisy_frame),
             statistics.fmean(int(row["R_high_clip"]) for row in noisy_frame), "", ""),
            ("noisy_frame_lwir_high_clip_fraction", len(noisy_frame),
             statistics.fmean(int(row["lwir_high_clip"]) for row in noisy_frame), "", ""),
        ])
        for mode, key in (("rgb_dominant", "rgb_tau550"), ("lwir_dominant", "lwir_tau11")):
            self.assertLess(float(self.pixel[mode, 0, 0, 3, 1][key]),
                            float(self.pixel[mode, 0, 0, 3, 4][key]))
        print(f"max M04 probe difference={largest:.10g}; codes exact; LWIR crossover error=0", flush=True)

    def test_03_midpoints_and_heterogeneous_integration(self):
        frames = {row["environment"]: row for row in self.frames}
        for row in self.frames:
            prior = frames[row["environment"]]
            for key in ("burden_coefficients", "temperature_coefficients", "environment_seed"):
                self.assertEqual(row[key], prior[key])
        largest = 0
        for group in self.path.values():
            frame = frames[group[0]["environment"]]
            time = int(frame["time_s"])
            sampled = frame["profile"] in ("stochastic", "advected")
            self.assertEqual(len(group), 16 if sampled else 1)
            self.close(group[-1]["lo_m"], 0)
            for a, b in zip(group, group[1:]):
                self.close(a["lo_m"], float(b["hi_m"]))
            coeffs = json.loads(frame["burden_coefficients"])
            heat = json.loads(frame["temperature_coefficients"])
            for row in group:
                self.assertEqual(row["environment"], frame["environment"])
                if not sampled:
                    continue
                burden, air, rgb, lwir = reference.physical(CFG, coeffs, heat, float(row["z_m"]), time)
                optics = CFG["field"]["optical"]
                expected = (burden, air, rgb*(1-optics["scatter_fraction"]),
                            rgb*optics["scatter_fraction"], optics["rgb_exponent"], *lwir)
                actual = [float(row[key]) for key in SEGMENT_FIELDS[6:]]
                for a, b in zip(actual, expected, strict=True):
                    self.close(a, b)
                    largest = max(largest, abs(a-b))
        self.assertNotEqual(self.frame["advected", 0, 0]["environment"],
                            self.frame["advected", 0, 4]["environment"])
        self.assertNotEqual(self.pixel["advected", 0, 0, 3, 1]["burden_avg"],
                            self.pixel["advected", 0, 4, 3, 1]["burden_avg"])
        frame = self.frame["heterogeneous", 0, 6]
        probe = self.pixel["heterogeneous", 0, 6, 3, 1]
        burden = json.loads(frame["burden_coefficients"])
        heat = json.loads(frame["temperature_coefficients"])
        extent = float(probe["first_range_m"])
        expected = reference.integrated_depths(CFG, burden, heat, float(probe["first_z_m"]), extent, 6)
        errors = []
        for n in (8, 16, 32):
            slabs = segments(CFG, "stochastic", 6, burden, heat, 0, float(probe["dz"]), extent, n)
            depth = [optical_depth(slabs, mode, w)
                     for mode, w in (("RGB", 450), ("RGB", 550), ("LWIR", 8), ("LWIR", 11))]
            errors.append([abs(a-b) for a, b in zip(depth, expected, strict=True)])
        self.assertLessEqual(max(errors[1]), 5e-4)
        self.assertTrue(all(a > b for a, b in zip(errors[1], errors[2], strict=True)))
        print(f"all heterogeneous midpoint mappings max absolute difference={largest:.10g}; "
              f"depth reference={expected}; errors for 8/16/32 segments={errors}", flush=True)

    def test_04_foreground_replacement(self):
        for arm, count in (("partial", 20), ("complete", 0), ("combined_hidden", 0)):
            frame = self.frame[arm, 0, 0]
            self.assertEqual((int(frame["projected"]), int(frame["visible"])), (35, count))
        only_observation = ("first_id", "material", "first_range_m", "rgb_tau550", "lwir_tau11", *OBS_FIELDS)
        for r in self.by_frame["combined_hidden", 0, 0]:
            p = self.pixel["combined_hidden_perturbed", 0, 0, int(r["row"]), int(r["column"])]
            self.assertEqual([p[key] for key in only_observation], [r[key] for key in only_observation])
        center = self.pixel["combined_hidden", 0, 0, 3, 4]
        self.assertEqual((center["material"], float(center["first_range_m"])), ("foreground", 5))
        self.assertEqual(float(self.pixel["clear", 0, 0, 3, 4]["first_range_m"]), 10)
        observed = [int(self.frame["moving_combined", 0, t]["visible"]) for t in range(7)]
        self.assertEqual((observed[0], observed[3], observed[-1]), (35, 20, 35))
        print(f"combined motion visible target counts={observed}; 0 hidden-material differences", flush=True)

    def test_05_joint_statistics(self):
        ensemble = CFG["ensemble"]
        probe_row, probe_col = ensemble["probe"]
        n = ensemble["size"]
        time = ensemble["analysis_time_s"]
        fixed = [self.pixel["ensemble_fixed", i, time, probe_row, probe_col] for i in range(n)]
        covered = [self.pixel["ensemble_covered", i, time, probe_row, probe_col] for i in range(n)]
        rgb, lwir = [[float(row[key]) for row in fixed] for key in ("rgb_tau550", "lwir_tau11")]
        self.assertGreater(len(set(rgb)), 100)
        self.assertGreater(len(set(lwir)), 100)
        self.assertEqual(len({row["environment"] for row in fixed}), n)
        self.assertNotEqual(self.frame["ensemble_fixed", 0, time]["burden_coefficients"],
                            self.frame["ensemble_fixed", 1, time]["burden_coefficients"])
        optical = CFG["field"]["optical"]
        slope = optical["lwir_span"][1] / optical["rgb_span"]
        errors = []
        for row, masked, t_rgb, t_lwir in zip(fixed, covered, rgb, lwir, strict=True):
            self.assertEqual(row["environment"], masked["environment"])
            self.assertGreater(float(masked["rgb_tau550"]), t_rgb)
            self.assertGreater(float(masked["lwir_tau11"]), t_lwir)
            intercept = float(row["first_range_m"]) * (
                slope * optical["rgb_floor"] - optical["lwir_floor"][1])
            error = abs(math.log(t_lwir) - slope * math.log(t_rgb) - intercept)
            errors.append(error)
            self.assertLessEqual(error, 2e-12)
        correlation = statistics.correlation(rgb, lwir)
        covariance = statistics.covariance(rgb, lwir)
        shifted = statistics.correlation(rgb, lwir[1:] + lwir[:1])
        self.assertGreaterEqual(correlation, .95)
        self.assertLessEqual(abs(shifted), .35)
        fisher = math.atanh(correlation)
        half = 1.96 / math.sqrt(n - 3)
        STAT_ROWS.extend([
            ("fixed_geometry_pearson_r", n, correlation, math.tanh(fisher-half), math.tanh(fisher+half)),
            ("fixed_geometry_covariance", n, covariance, "", ""),
            ("different_realization_comparator_r", n, shifted, "", ""),
            ("log_transmission_affine_max_error", n, max(errors), "", ""),
        ])
        latent = [reference.latent(CFG["field"], json.loads(
            self.frame["ensemble_fixed", i, time]["burden_coefficients"]), 0, time) for i in range(n)]
        STAT_ROWS.extend([("ou_probe_latent_mean_reference_0", n, *reference.interval(latent)),
                          ("ou_probe_latent_second_moment_reference_1", n,
                           *reference.interval([x*x for x in latent]))])
        for sensor in ("rgb_tau550", "lwir_tau11"):
            low = [float(row[sensor]) for row in fixed if float(row["burden_avg"]) < .5]
            high = [float(row[sensor]) for row in fixed if float(row["burden_avg"]) >= .5]
            self.assertGreaterEqual(min(len(low), len(high)), 30)
            self.assertGreater(statistics.fmean(low), statistics.fmean(high))
            for name, values in (("low_burden", low), ("high_burden", high)):
                STAT_ROWS.append((f"{sensor}_{name}_mean", len(values), *reference.interval(values)))
            geometry = [float(b[sensor])-float(a[sensor]) for a, b in zip(fixed, covered, strict=True)]
            STAT_ROWS.append((f"{sensor}_matched_geometry_change", n, *reference.interval(geometry)))
            changes = [float(self.pixel["ensemble_fixed", i, 12, probe_row, probe_col][sensor])
                       - float(self.pixel["ensemble_fixed", i, 0, probe_row, probe_col][sensor])
                       for i in range(n)]
            STAT_ROWS.append((f"{sensor}_within_realization_change_0_to_12s", n,
                              *reference.interval(changes)))
        print(f"{n} fixed-geometry realizations: r={correlation:.10g}, covariance={covariance:.10g}, "
              f"cyclic comparison={shifted:.10g}, max exact log-relation error={max(errors):.10g}; "
              f"low/high populations={len(low)}/{len(high)}", flush=True)

    def test_06_qualification_and_rejections(self):
        expected_frames = {(c["id"], 0, t) for c in CFG["cases"] for t in c.get("times_s", [0])}
        for i in range(CFG["ensemble"]["size"]):
            for name in ("ensemble_fixed", "ensemble_covered"):
                expected_frames.update((name, i, t) for t in CFG["ensemble"]["times_s"])
        self.assertEqual(set(self.frame), expected_frames)
        counts = Counter(row["split"] for row in self.frames)
        self.assertEqual(counts, {"controls": 25, "development": 576, "holdout": 192})
        for (name, realization, time), frame in self.frame.items():
            if name.startswith("ensemble_"):
                self.assertEqual(int(frame["environment_seed"]),
                                 CFG["ensemble"]["seed_start"] + realization)
                split = "development" if realization < CFG["ensemble"]["dev_size"] else "holdout"
                self.assertEqual(frame["split"], split)
            self.assertEqual(int(frame["lwir_seed"]), int(frame["rgb_seed"]) + 1)
        metadata = json.loads((CAPTURE / "metadata.json").read_text())
        self.assertEqual((metadata["configuration"], metadata["starting_revision"]), (CFG, CFG["starting_revision"]))
        self.assertEqual(metadata["table_counts"], dict(frames=len(self.frames), pairs=len(self.pairs),
                                                       segments=len(self.segments)))
        self.assertEqual(metadata["path_count"], len(self.path))
        self.assertEqual(metadata["configuration_sha256"], hashlib.sha256(
            (HERE / "config.json").read_bytes()).hexdigest())
        self.assertEqual(metadata["kernel_sha256"], {name: hashlib.sha256(
            (ROOT / name).read_bytes()).hexdigest() for name in KERNELS})
        self.assertEqual((metadata["time_units"], metadata["range_units"], metadata["band_radiance_units"]),
                         ("s", "m", "W m^-2 sr^-1"))
        self.assertTrue(metadata["synthetic_co_registration"])
        for pair in self.pairs:
            if not pair["path_id"]:
                self.assertEqual(pair["status"], "no_hit")
                continue
            settings = CFG["sensor"] | CASES[pair["case"]].get("sensor", {})
            chain = self.path[pair["path_id"]]
            self.close(chain[0]["hi_m"], float(pair["first_range_m"]))
            for segment in chain:
                midpoint = (float(segment["lo_m"]) + float(segment["hi_m"])) / 2
                self.close(segment["z_m"], midpoint * float(pair["dz"]))
                self.assertTrue(0 <= float(segment["burden"]) <= 1)
                self.assertTrue(250 <= float(segment["air_K"]) <= 330)
                self.assertTrue(all(float(segment[key]) >= 0 for key in SEGMENT_FIELDS[8:]))
            extents = [(float(s["hi_m"])-float(s["lo_m"]), s) for s in chain]
            depth_rgb = sum(length * (float(s["rgb_absorption_550_m_inv"])
                            + float(s["rgb_scattering_550_m_inv"])) for length, s in extents)
            depth_lwir = sum(length * float(s["lwir_absorption_11_m_inv"]) for length, s in extents)
            self.close(pair["rgb_tau550"], math.exp(-depth_rgb))
            self.close(pair["lwir_tau11"], math.exp(-depth_lwir))
            self.close(pair["burden_avg"], sum(length * float(s["burden"]) for length, s in extents)
                       / float(pair["first_range_m"]))
            max_code = 2**settings["bits"] - 1
            for band in BANDS:
                analog = float(pair[f"{band}_analog"])
                high = (settings["lwir_high"] if band == "lwir"
                        else settings["rgb_well"] / settings["rgb_gain"])
                self.assertEqual(int(pair[f"{band}_low_clip"]), analog < 0)
                self.assertEqual(int(pair[f"{band}_high_clip"]),
                                 analog > high if band == "lwir" else analog >= high)
                code = int(pair[f"{band}_code"])
                self.assertTrue(0 <= code <= max_code)
                self.assertLessEqual(abs(code - min(high, max(0, analog))/high*max_code), .50000000001)
            self.assertTrue(all(float(pair[key]) >= 0 for key in OBS_FIELDS
                                if key.endswith(("_emission", "_reflection", "_direct", "_path"))))
        for invalid, expected in (
            (self.pairs + [self.pairs[0]], "duplicate"),
            ([self.pairs[0] | {"dx": "NaN"}, *self.pairs[1:]], "nonfinite"),
            ([self.pairs[0] | {"environment": "incorrect"}, *self.pairs[1:]], "synchronization"),
        ):
            with self.assertRaisesRegex(ValueError, expected):
                qualify(self.frames, invalid, self.segments)
        no_hit = sum(row["status"] == "no_hit" for row in self.pairs)
        print(f"{len(self.pairs) - no_hit} valid paired observations, {no_hit} no-hits; "
              f"{len(self.path)} paths, {len(self.segments)} segments; 3 corruption probes rejected",
              flush=True)


if __name__ == "__main__":
    CAPTURE = Path(sys.argv.pop(1))
    STAT_ROWS = [("metric", "n", "estimate", "lower_95", "upper_95")]
    program = unittest.main(verbosity=2, exit=False)
    write_statistics(CAPTURE)
    sys.exit(not program.result.wasSuccessful())
