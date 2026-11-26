#!/usr/bin/env python3
"""Audit the retained M11 packet without importing or executing its harness."""

import ast
import csv
import hashlib
import json
import math
import statistics
import subprocess
from pathlib import Path

M11 = Path(__file__).resolve().parent.parent / "m11"
REPO = M11.parents[2]
SUBJECT = "ff64941152ad10b0656a2c68d80eb4121b070b97"
SUBMISSION = "1f04d8bd04a257defc000c4cb92b046e77570bb8"
PLAN_CHAIN = ("3e5693ecd", "f04ff3a1b", "3ac3f0d9e", "cb9940625")
LIVE_FILES = {
    "ore-ms/docs/research/INDEX.md",
    "ore-ms/ore-ms-research/18_RESEARCH_ROADMAP.md",
    "ore-ms/ore-ms-research/ACCEPTANCE_CONDITIONS_REGISTER.md",
    "ore-ms/ore-ms-research/RESEARCH_ROADMAP_MANDATE.md",
}


def git(*args):
    return subprocess.check_output(["git", *args], cwd=REPO)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify_manifest(name, folder, allowed_missing=(), verify_subject=False):
    checked, missing = 0, []
    for row in (M11 / name).read_text().splitlines():
        expected, path = row.split(maxsplit=1)
        local_path = (folder / path).resolve()
        if not local_path.is_file() and path in allowed_missing:
            missing.append({"file": path, "sha256": expected})
            continue
        assert digest(local_path.read_bytes()) == expected, path
        if verify_subject:
            relative = local_path.relative_to(REPO).as_posix()
            assert digest(git("show", f"{SUBJECT}:{relative}")) == expected, path
        checked += 1
    return {"verified": checked, "expected_but_not_retained": missing}


def table(name):
    return json.loads((M11 / "evidence" / f"{name}_checks.json").read_text())


def wilson(count, total):
    z = 1.959963984540054
    p = count / total
    scale = 1 + z * z / total
    center = (p + z * z / (2 * total)) / scale
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / scale
    return [center - half, center + half]


def validate_coverage(coverage, total, ci):
    count = round(coverage * total)
    independently_calculated = wilson(count, total)
    assert abs(count / total - coverage) < 1e-12
    assert max(abs(a - b) for a, b in zip(ci, independently_calculated, strict=True)) < 1e-12
    return {"covered": count, "total": total, "wilson_95": independently_calculated}


def transfer_checks():
    rgb, legacy, lwir = (table(key) for key in ("rgb", "legacy", "lwir"))
    rows = rgb["rows"]
    assert {tuple(row[:3]) for row in rows} == {
        (wave, depth, light)
        for wave in (450, 550, 650)
        for depth in (0, .2, 1, 4)
        for light in (0., .8)
    }
    assert len(rows) == 24
    rgb_max = max(abs(row[3]) for row in rows)
    assert rgb_max == rgb["max_absolute"] and max(rgb_max, rgb["spectral_error"]) <= 1e-11
    assert max(legacy["rgb_absolute_errors"]) < 1e-11
    assert abs(legacy["lwir_actual"] - legacy["lwir_reference"]) == legacy["lwir_absolute_error"] < .005
    for row in rgb["noise"]:
        variance = row["level_e"] + 9
        assert abs(row["mean_e"] - row["level_e"]) <= 5 * math.sqrt(variance / 20000) + 150 / 4095
        assert abs(row["variance_e2"] / variance - 1) <= .08
    assert rgb["detector_rails"] == [[0, False], [4081, False], [4095, True], [4095, True]]
    assert all(row["energy_residual"] < 1e-12 for row in rgb["two_stream"])

    def planck(wavelength_um, temperature):
        wavelength = wavelength_um * 1e-6
        h, c, k = 6.62607015e-34, 299792458, 1.380649e-23
        return (2 * h * c**2 / wavelength**5) / math.expm1(h * c / (wavelength * k * temperature)) * 1e-6

    planck_relative = max(abs(delta) / planck(wave, temp) for wave, temp, delta in lwir["planck_rows"])
    assert len(lwir["planck_rows"]) == 12 and planck_relative < 1e-12
    errors = [max(abs(a - b) for a, b in zip(row["band"], lwir["reference_terms"], strict=True))
              for row in lwir["refinements"]]
    assert [row["intervals"] for row in lwir["refinements"]] == [12, 24, 48, 96]
    assert errors[-1] < .005 and errors[-1] / errors[-2] < .30
    crossover_max = max(abs(a - b) for _, a, b in lwir["crossover"])
    assert crossover_max < 1e-10
    for analog, code, low, high in lwir["detector_rails"]:
        assert code == round((min(50, max(1, analog)) - 1) * 4095 / 49)
        assert (low, high) == (analog < 1, analog > 50)
    return {"rgb_cases": len(rows), "rgb_max_residual": rgb_max,
            "lwir_planck_relative_max": planck_relative, "lwir_band_errors": errors,
            "lwir_final_refinement_ratio": errors[-1] / errors[-2],
            "lwir_crossover_max": crossover_max}


def environment_checks():
    geometry, ou, multi = (table(key) for key in ("geometry", "stochastic", "multispectral"))
    assert geometry["rays"] == 33 * 25 * len(geometry["visibility"]) == 3300
    assert sum(not case["tied"] for case in geometry["visibility"]) == 3
    assert geometry["first_hit_mismatch"] == 0
    assert max(geometry["max_range_m"], geometry["max_projection_px"]) < 1e-10
    a = math.exp(-2 / 6)
    variance = 1.1**2 * -math.expm1(-4 / 6)
    deviations = {
        "conditional_mean": abs(ou["means"][3] - a) / math.sqrt(variance),
        "conditional_variance": abs(ou["conditional_variance"] / variance - 1),
        "covariance": max(abs(a - b) for a, b in zip(
            ou["covariance_row"], ou["reference_covariance"], strict=True)) / 1.1**2,
    }
    assert ou["n"] >= 20_000 and max(deviations.values()) < .04
    assert max(max(row[1:]) for row in ou["transition_errors"]) < 1e-12
    euler_biases = [abs(row[1] - variance) for row in ou["euler"]]
    assert all(a > b for a, b in zip(euler_biases, euler_biases[1:]))
    assert math.isfinite(ou["long_run_max_abs"])
    first, degraded, last = ou["clear_degraded_clear_burden"]
    assert first == last and degraded > first

    code_delta, band_budget_fraction = 0, 0
    assert len(multi["cases"]) == 7
    for row in multi["cases"]:
        assert row["length_m"] == row["reference_hit_depth_m"]
        assert row["material"] == row["reference_material"]
        for actual, expected in zip(row["actual"], row["reference"], strict=True):
            band_budget_fraction = max(band_budget_fraction, abs(actual - expected) / max(.005, .003 * expected))
        for actual, expected in zip(row["detector_codes"], row["independent_detector_codes"], strict=True):
            code_delta = max(code_delta, abs(actual - expected))
    assert code_delta <= 1 and band_budget_fraction < 1
    return {"rays_including_tie_grid": geometry["rays"], "ou_standardized_errors": deviations,
            "euler_biases": euler_biases, "coupled_matched_paths": len(multi["cases"]),
            "coupled_max_band_budget_fraction": band_budget_fraction, "coupled_max_code_error": code_delta}


def forecast_checks():
    posterior, forecast, learned = (table(key) for key in ("posterior", "forecast", "learned"))
    posterior_intervals = {}
    for row in posterior["rows"]:
        assert row["scored"] >= 128 and row["max_burden_discrepancy"] < .015
        posterior_intervals[row["regime"]] = {
            label: validate_coverage(row[f"{label}_coverage"], row["scored"], row[f"{label}_coverage_ci"])
            for label in ("existing", "reference")
        }
    assert forecast["n"] == 4096
    assert max(forecast["horizon_covariance_error"], forecast["projection_error"]) < 1e-11
    forecast_intervals = {}
    for regime, horizons in forecast["scores"].items():
        forecast_intervals[regime] = {}
        for horizon, row in horizons.items():
            result = validate_coverage(row["coverage"], row["n"], row["coverage_ci"])
            assert result["covered"] == row["covered"]
            forecast_intervals[regime][horizon] = result
    checkpoint = M11.parent / "m09/checkpoints/physics-768-901.json"
    assert digest(checkpoint.read_bytes()) == learned["checkpoint_sha256"]
    assert learned["same_issue_scores_before_future_shift"] is True
    learned_intervals = {}
    for regime, values in learned["scores"].items():
        assert values["generated"] == 256 and values["issued"] == 256
        learned_intervals[regime] = {
            horizon: validate_coverage(row["coverage"], values["issued"], row["coverage_ci"])
            for horizon, row in values["horizons"].items()
        }
    parameter_budgets = []
    for row in forecast["parameter_budget"]:
        horizon = row["horizon_s"]
        between = statistics.pvariance(math.exp(-horizon / tau) for tau in (4, 6, 10))
        process = statistics.mean(sigma**2 * -math.expm1(-2 * horizon / tau)
                                  for tau in (4, 6, 10) for sigma in (.8, 1, 1.2))
        assert math.isclose(between, row["parameter_variance_at_initial_state_1"], rel_tol=1e-12)
        assert math.isclose(process, row["mean_conditional_aleatoric_variance"], rel_tol=1e-12)
        parameter_budgets.append({"seconds": horizon, "process": process, "between_parameter": between})
    return {"posterior_intervals": posterior_intervals, "forecast_intervals": forecast_intervals,
            "learned_intervals": learned_intervals, "parameter_budgets": parameter_budgets}


def fusion_checks():
    fusion = table("fusion")
    assert fusion["frames"] == fusion["independent_clusters"] * 3 * 9 * 4 == 5184
    for arm, summary in fusion["summary"].items():
        for strata in (fusion["offset_control_strata"][arm], fusion["size_strata"][arm]):
            for field in ("n", "feasible", "issued", "correct_issued", "negative_scene_issues"):
                assert sum(row[field] for row in strata.values()) == summary[field]
        assert sum(summary["reason_counts"].values()) == summary["n"]
        assert math.isclose(summary["selective_risk"], 1 - summary["correct_issued"] / summary["issued"])
        assert math.isclose(summary["coverage"], summary["issued"] / summary["n"])
        assert sum(row["n"] for row in summary["reliability_bins"]) == summary["calibration_n"]
    a, b = (fusion["summary"][key] for key in ("A01", "B03"))
    assert a["feasible"] == b["feasible"] == 2448
    gain = (a["correct_issued"] - b["correct_issued"]) / a["feasible"]
    assert math.isclose(gain, fusion["A01_minus_B03_feasible_gain"])
    return {"gain": gain, "a01_feasible_correct": a["correct_issued"], "b03_feasible_correct": b["correct_issued"],
            "a01_false_support": a["negative_scene_issues"], "a01_negative_scenes": a["negative_scenes"],
            "a01_calibration_n": a["calibration_n"], "b03_calibration_n": b["calibration_n"],
            "reported_cluster_interval_not_recomputed": fusion["paired_cluster_interval"]}


def main():
    manifests = {
        "package": verify_manifest("PACKAGE_SHA256SUMS.txt", M11),
        "input_including_subject_bytes": verify_manifest("INPUT_SHA256SUMS.txt", M11, verify_subject=True),
        "deterministic_output": verify_manifest("EVIDENCE_SHA256SUMS.txt", M11 / "evidence",
                                                allowed_missing=("fusion_decisions.csv",)),
    }
    for earlier, later in zip((SUBJECT, *PLAN_CHAIN), (*PLAN_CHAIN, SUBMISSION), strict=True):
        git("merge-base", "--is-ancestor", earlier, later)
    m11_path = M11.relative_to(REPO).as_posix()
    assert not git("diff", "--name-only", SUBMISSION, "--", m11_path)
    changed = git("diff", "--name-only", SUBJECT, SUBMISSION).decode().splitlines()
    assert {path for path in changed if not path.startswith(m11_path + "/")} == LIVE_FILES
    for name in ("14_WORK_BREAKDOWN_STRUCTURE.csv", "17_TASK_DEPENDENCIES.csv"):
        catalog = M11.parent / name
        relative = catalog.relative_to(REPO).as_posix()
        assert catalog.read_bytes() == git("show", f"{SUBJECT}:{relative}")
    with (M11.parent / "14_WORK_BREAKDOWN_STRUCTURE.csv").open(newline="") as file:
        tasks = list(csv.reader(file))[1:]
    assert len(tasks) == len({row[0] for row in tasks}) == 105
    task_counts = {milestone: sum(row[2] == milestone for row in tasks)
                   for milestone in ("M11", "M12", "M13")}
    assert all(task_counts.values())
    modules = set()
    for node in ast.walk(ast.parse((M11 / "reference.py").read_text())):
        if isinstance(node, ast.Import):
            modules.update(alias.name.split(".")[0] for alias in node.names)
        if isinstance(node, ast.ImportFrom):
            assert node.level == 0
            modules.add(node.module.split(".")[0])
    assert modules <= {"math", "numpy", "scipy"}
    timing = json.loads((M11 / "evidence/timing.json").read_text())
    assert timing["fusion_latency"]["frontend"]["calls"] == timing["fusion_latency"]["policy"]["calls"] == 5184
    result = {
        "review_checkout": git("rev-parse", "HEAD").decode().strip(),
        "subject": SUBJECT, "submission": SUBMISSION, "plan_commits": PLAN_CHAIN,
        "manifests": manifests, "permitted_non_m11_subject_changes": sorted(LIVE_FILES),
        "historical_total_tasks": len(tasks), "late_milestone_task_counts": task_counts,
        "reference_imports": sorted(modules), "transfer": transfer_checks(),
        "environment": environment_checks(), "forecast": forecast_checks(), "fusion": fusion_checks(),
        "timing_derived": {"frontend_calls_per_s": 5184 / timing["fusion_latency"]["frontend"]["sum_s"],
                           "policy_calls_per_s": 5184 / timing["fusion_latency"]["policy"]["sum_s"],
                           "fusion_opportunities_per_s": 5184 / timing["fusion_checks"],
                           "peak_rss_mib": timing["peak_rss_kib"] / 1024},
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
