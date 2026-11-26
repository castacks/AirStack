#!/usr/bin/env python3
"""Run a bounded offline qualification with no simulator or model fitting."""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import random
import statistics

import adapter
from adapter import contract, dynamics
from forward import CONFIG as INVERSE_CONFIG
from reference import independent_burden, ou_reference

FIXTURE = adapter.HERE / "fixture.json"
COUNTERFACTUAL_SEED = 536000


def frozen_hashes():
    counts = {}
    for model in ("m05", "m06", "m07", "m08"):
        folder = adapter.M08.parent / model
        counts[model] = 0
        for name in ("PACKAGE_SHA256SUMS.txt", "EXPERIMENT_PLAN.sha256"):
            for record in (folder / name).read_text().splitlines():
                expected, relative = record.split(maxsplit=1)
                assert adapter.sha256(folder / relative) == expected, (model, relative)
                counts[model] += 1
    return counts


def rejects(path, data, message):
    path.write_text(json.dumps(data))
    try:
        adapter.load_fixture(path)
    except ValueError as error:
        assert message in str(error)
    else:
        raise AssertionError(f"unexpected admission for {message}")


def geometry_checks(folder):
    fixture, length, _ = adapter.load_fixture(FIXTURE)
    assert length == 10 and fixture["geometry"]["source_m"] == [10, 0, 0]
    schedule = list(adapter.schedule_rows(fixture, length))
    for row in schedule:
        _, _, _, path, issue, target, issue_ns, target_ns, mask = row
        assert (path, target - issue, target_ns - issue_ns, mask) == (10, 20, 2_000_000_000, 1)
    alternative = copy.deepcopy(fixture)
    alternative["tick_period_ns"] = 250_000_000
    path = folder / "alternative-ticks.json"
    path.write_text(json.dumps(alternative))
    alternative, _, _ = adapter.load_fixture(path)
    assert all(row[5] - row[4] == 8 and row[7] - row[6] == 2_000_000_000
               for row in adapter.schedule_rows(alternative, length))
    alternative["tick_period_ns"] = 300_000_000
    rejects(path, alternative, "divide")
    invalid = copy.deepcopy(fixture)
    invalid["geometry"]["source_m"] = [6, 8, 0]
    rejects(path, invalid, "positive X")
    invalid["geometry"]["source_m"] = [9, 0, 0]
    rejects(path, invalid, "incompatible")
    invalid = copy.deepcopy(fixture)
    invalid["issues"][0]["tick"] = -1
    rejects(path, invalid, "nonnegative")
    invalid["issues"][0]["tick"] = 0
    invalid["issues"][1]["realization"] = invalid["issues"][0]["realization"]
    rejects(path, invalid, "unique")
    return {"path_m": length, "issue_ticks": [row[4] for row in schedule],
            "target_ticks": [row[5] for row in schedule], "delta_ns": 2_000_000_000,
            "alternate_period_ns": 250_000_000}


def reference_checks(folder):
    transmission = adapter.transmission
    vacuum = transmission([10], [0])
    constant = transmission([2, 3, 5], [.1, .1, .1])
    known_depth = transmission([2, 3, 5], [.05, .2, .06])
    assert vacuum == 1
    assert math.isclose(constant, math.exp(-1), rel_tol=1e-15)
    assert math.isclose(known_depth, math.exp(-1), rel_tol=1e-15)
    assert math.isclose(transmission([20], [.1]), constant**2, rel_tol=1e-15)
    assert 0 <= transmission([10], [100]) <= constant <= vacuum
    for lengths, values in (([10], [-.1]), ([0], [.1]), ([10], [math.nan]), ([10, 2], [.1])):
        try:
            transmission(lengths, values)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid optical inputs were admitted")

    config, _, _ = adapter.load_fixture(FIXTURE)
    truth = {(row["realization"], int(row["horizon_s"])): row
             for row in contract.rows(folder / "truth.csv", contract.LABELS)}
    manifest = json.loads((folder / "manifest.json").read_text())
    optics = INVERSE_CONFIG["optical"]
    max_error = 0.0
    for issue, seed in zip(config["issues"], manifest["future_seeds"], strict=True):
        initial = adapter.initial_state(issue)
        rng = random.Random(seed)
        future = []
        for coefficient in initial:
            mean, variance = ou_reference(coefficient, 2, dynamics.CONFIG["tau_s"], dynamics.CONFIG["sigma"])
            future.append(mean + math.sqrt(variance) * rng.gauss(0, 1))
        for horizon, state in ((0, initial), (2, future)):
            row = truth[issue["realization"], horizon]
            burden = independent_burden(state, horizon, dynamics.CONFIG)
            expected = (burden, math.exp(-10 * (optics["rgb_floor"] + optics["rgb_span"] * burden)),
                        math.exp(-10 * (optics["lwir_floor"][1] + optics["lwir_span"][1] * burden)))
            values = tuple(float(row[field]) for field in contract.TARGETS)
            assert all(0 <= value <= 1 for value in values)
            max_error = max(max_error, *(abs(a - b) for a, b in zip(values, expected, strict=True)))
    assert len(truth) == 2 * len(config["issues"]) and max_error <= 2e-12
    return {"vacuum": vacuum, "constant_tau_1": constant, "piecewise_tau_1": known_depth,
            "double_length": transmission([20], [.1]), "reference_rows": len(truth),
            "max_vs_frozen_reference_path": max_error}


def matched_checks(folder):
    issue_rows = list(contract.rows(folder / "issues.csv", contract.FEATURES))
    forecast_rows = list(contract.rows(folder / "forecasts.csv", contract.FORECASTS))
    issues = {adapter.key(row): row for row in issue_rows}
    predictions = {(adapter.key(row), row["method"], int(row["horizon_s"])): row
                   for row in forecast_rows}
    assert len(issues) == len(issue_rows) and len(predictions) == len(forecast_rows)
    evaluations = list(contract.rows(folder / "evaluation.csv", adapter.EVALUATION))
    assert len(predictions) == len(issues) * 6 and len(evaluations) == len(issues) * 3
    legal_codes = range(1, 2**INVERSE_CONFIG["bits"] - 1)
    for key, issue in issues.items():
        assert all(int(issue[name]) in legal_codes for name in contract.FEATURES[3:])
        arms = [row for row in evaluations if adapter.key(row) == key]
        assert {row["method"] for row in arms} == set(adapter.METHODS)
        assert len({tuple(row[field] for field in (*adapter.SCHEDULE, "rgb_t550", "lwir_t11"))
                    for row in arms}) == 1
        assert all(row["evaluate"] == "1" and row["horizon_s"] == "2" for row in arms)
        issue_draws = predictions[key, "joint", 0]["burden_samples"]
        assert issue_draws == predictions[key, "persistence", 0]["burden_samples"]
        assert issue_draws == predictions[key, "persistence", 2]["burden_samples"]
        masked = issue | {"lwir_code": ""}
        # RGB's numerical outputs must be invariant; its declared mode may report exclusions.
        for a, b in zip(adapter.forecast(issue, "RGB"), adapter.forecast(masked, "RGB"), strict=True):
            assert (a[0], a[1], a[3:]) == (b[0], b[1], b[3:])
        for row in arms:
            forecast = predictions[key, row["method"], 2]
            burden = json.loads(forecast["burden_samples"])
            assert len(burden) == int(row["draw_count"]) == dynamics.CONFIG["marginal_size"]
            assert all(0 <= value <= 1 for value in burden)
            draws = [dynamics.targets(value)[1] for value in burden]
            truth = float(row["rgb_t550"])
            independent_crps = statistics.fmean(abs(value - truth) for value in draws) - (
                math.fsum(abs(a - b) for a in draws for b in draws) / (2 * len(draws)**2))
            assert math.isclose(independent_crps, float(row["rgb_t550_crps"]), abs_tol=2e-12)
    rail_case = next(iter(issues.values())) | dict.fromkeys(contract.FEATURES[3:], "0")
    _, state, flags, _, _ = next(adapter.forecast(rail_case, "joint"))
    assert state == "prior_fallback" and all(f"{channel}_rail" in flags for channel in contract.FEATURES[3:])
    return {"issues": len(issues), "forecast_rows": len(predictions), "evaluations": len(evaluations),
            "output_status_counts": dict(Counter(row["status"] for row in evaluations)),
            "crps_mean_by_arm": {method: statistics.fmean(float(row["rgb_t550_crps"]) for row in evaluations
                                                         if row["method"] == method)
                                 for method in adapter.METHODS}}


def verify_manifest(folder, fixture_path=FIXTURE):
    metadata = json.loads((folder / "manifest.json").read_text())
    for name, expected in metadata["artifacts_sha256"].items():
        assert adapter.sha256(folder / name) == expected, name
    assert set(metadata["artifacts_sha256"]) == {
        "issues.csv", "schedule.csv", "truth.csv", "forecasts.csv", "evaluation.csv"}
    assert metadata["fixture_sha256"] == adapter.sha256(fixture_path)
    assert metadata["adapter_sha256"] == adapter.sha256(adapter.HERE / "adapter.py")
    for model, expected in metadata["frozen_package_manifest_sha256"].items():
        assert adapter.sha256(adapter.M08.parent / model / "PACKAGE_SHA256SUMS.txt") == expected
    return metadata


def counterfactual_checks(first, shifted):
    unchanged = ("issues.csv", "schedule.csv", "forecasts.csv")
    assert all((first / name).read_bytes() == (shifted / name).read_bytes() for name in unchanged)
    truth = [list(contract.rows(folder / "truth.csv", contract.LABELS)) for folder in (first, shifted)]
    changed = 0
    for a, b in zip(*truth, strict=True):
        assert adapter.key(a) == adapter.key(b) and a["horizon_s"] == b["horizon_s"]
        if a["horizon_s"] == "0":
            assert a == b
        else:
            assert a["rgb_t550"] != b["rgb_t550"] and a["lwir_t11"] != b["lwir_t11"]
            changed += 1
    controls = [list(contract.rows(folder / "evaluation.csv", adapter.EVALUATION))
                for folder in (first, shifted)]
    score_columns = {"rgb_t550", "lwir_t11", "rgb_t550_crps"}
    for a, b in zip(*controls, strict=True):
        assert {name for name in a if a[name] != b[name]} == score_columns
    return {"byte_identical": unchanged, "changed_future_label_rows": changed,
            "changed_score_rows": len(controls[0]), "counterfactual_future_seed_base": COUNTERFACTUAL_SEED}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path, help="new artifact directory outside Git")
    args = parser.parse_args()
    counts = frozen_hashes()
    args.out.mkdir()
    baseline, repeat, changed = (args.out / name for name in ("baseline", "repeat", "future_changed"))
    adapter.run(FIXTURE, baseline)
    adapter.run(FIXTURE, repeat)
    alternate_fixture = args.out / "future-fixture.json"
    fixture_data = json.loads(FIXTURE.read_text())
    fixture_data["future_seed_base"] = COUNTERFACTUAL_SEED
    alternate_fixture.write_text(json.dumps(fixture_data, indent=2) + "\n")
    adapter.run(alternate_fixture, changed)
    manifest = verify_manifest(baseline)
    assert manifest == verify_manifest(repeat)
    alternate = verify_manifest(changed, alternate_fixture)
    assert set(manifest["future_seeds"]).isdisjoint(alternate["future_seeds"])
    result = {
        "geometry_and_timing": geometry_checks(args.out),
        "independent_references": reference_checks(baseline),
        "issue_schema_and_matched_controls": matched_checks(baseline),
        "leakage_counterfactual": counterfactual_checks(baseline, changed),
        "manifest_integrity": {"frozen_hashes_checked": counts, "baseline_manifest": manifest,
                               "counterfactual_manifest": alternate, "repeat_verified": True},
        "gates": dict.fromkeys(("geometry_and_exact_timing", "issue_schema_and_replay", "references",
                                "matched_controls_and_scoring", "future_leakage", "manifest_integrity"), "PASS"),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
