#!/usr/bin/env python3
"""Bind a stationary analytic ray to the frozen M08 issue and score contracts."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
M08 = HERE.parent / "m08"
sys.path.insert(0, str(M08))

import contract
import dynamics
from environment import advance, evaluate, logistic, stationary, transition
from forward import AnalogNoise, CASES, code_observation, context
from predict import forecast
from scores import crps, quantile

CASE = "matched"
SPLIT = "qualification"
METHODS = ("joint", "RGB", "persistence")
HORIZON_S = 2
HORIZON_NS = 2_000_000_000
SCHEDULE = (*contract.IDS, "path_m", "issue_tick", "target_tick", "issue_ns", "target_ns", "evaluate")
EVALUATION = (*SCHEDULE, "method", "horizon_s", "rgb_t550", "lwir_t11", "status", "flags",
              "ess", "draw_count", "rgb_t550_crps", "median_rgb_t550", "lower95", "upper95")


def path_length(geometry):
    receiver = geometry["receiver_m"]
    source = geometry["source_m"]
    if (geometry["frame"] != "receiver_origin_x_forward" or receiver != [0, 0, 0]
            or any(type(x) not in (int, float) for x in receiver)):
        raise ValueError("expected receiver origin and +X path convention")
    if len(source) != 3 or not all(type(x) in (int, float) and math.isfinite(x) for x in source):
        raise ValueError("invalid source coordinates")
    if source[0] <= 0 or source[1:] != [0, 0]:
        raise ValueError("source must lie on the positive X axis")
    return math.dist(receiver, source)


def load_fixture(path):
    fixture = json.loads(path.read_text())
    if fixture["schema_version"] != 1:
        raise ValueError("unsupported fixture version")
    length = path_length(fixture["geometry"])
    inverse = CASES[dynamics.CONFIG["cases"][CASE]["inverse"]]
    if length != dynamics.LENGTH or length != context(inverse)[1]:
        raise ValueError("path length incompatible with frozen M07/M08")
    period = fixture["tick_period_ns"]
    if type(period) is not int or period <= 0 or HORIZON_NS % period:
        raise ValueError("tick period must divide the two-second horizon exactly")
    if HORIZON_S not in dynamics.HORIZONS:
        raise ValueError("two-second horizon unavailable")
    issues = fixture["issues"]
    if not 1 <= len(issues) <= 8:
        raise ValueError("offline qualification supports one to eight issues")
    ids = [row["realization"] for row in issues]
    if any(not isinstance(key, str) or not key for key in ids) or len(set(ids)) != len(ids):
        raise ValueError("realization IDs must be nonempty and unique")
    for row in issues:
        if type(row["tick"]) is not int or row["tick"] < 0:
            raise ValueError("issue ticks must be nonnegative integers")
    return fixture, length, inverse


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_rows(path, fields, rows):
    with path.open("w", newline="") as handle:
        contract.output(handle, fields).writerows(rows)


def initial_state(row):
    return stationary(random.Random(row["initial_seed"]), dynamics.CONFIG["sigma"], dynamics.MODEL.size)


def issue_rows(issues, length, inverse):
    material, _, air = context(inverse)
    for row in issues:
        burden = dynamics.burden(initial_state(row), 0, length)
        codes = code_observation(inverse, (material, length, air), burden,
                                 AnalogNoise(row["rgb_seed"]), random.Random(row["lwir_seed"]))
        yield CASE, row["realization"], SPLIT, *codes


def schedule_rows(fixture, length):
    period = fixture["tick_period_ns"]
    for row in fixture["issues"]:
        issue = row["tick"]
        target = issue + HORIZON_NS // period
        yield CASE, row["realization"], SPLIT, length, issue, target, issue * period, target * period, 1


def transmission(lengths_m, extinction_m_inv):
    terms = []
    for length, coefficient in zip(lengths_m, extinction_m_inv, strict=True):
        if not (math.isfinite(length) and math.isfinite(coefficient) and length > 0 and coefficient >= 0):
            raise ValueError("invalid segment length or extinction")
        terms.append(length * coefficient)
    if not terms:
        raise ValueError("empty optical path")
    return math.exp(-math.fsum(terms))


def reference_targets(coefficients, time_s, length):
    """Calculate synthetic truth directly from segment depths, without M08 targets()."""
    count = dynamics.CONFIG["segments"]
    width = length / count
    velocity = dynamics.CONFIG["velocity_m_s"]
    burdens = [logistic(evaluate(coefficients, dynamics.MODEL.basis((i + .5) * width, time_s, velocity)))
               for i in range(count)]
    optics = dynamics.OPTICS
    bands = ((optics["rgb_floor"], optics["rgb_span"]),
             (optics["lwir_floor"][1], optics["lwir_span"][1]))
    return (math.fsum(burdens) / count, *(transmission([width] * count,
            (floor + span * b for b in burdens)) for floor, span in bands))


def label_rows(fixture, length, future_seeds):
    update = transition(HORIZON_S, dynamics.CONFIG["tau_s"], dynamics.CONFIG["sigma"])
    for row, seed in zip(fixture["issues"], future_seeds, strict=True):
        initial = initial_state(row)
        target = advance(initial, dynamics.ZERO, update, random.Random(seed))
        for horizon, state in ((0, initial), (HORIZON_S, target)):
            yield CASE, row["realization"], SPLIT, horizon, *reference_targets(state, horizon, length)


def forecast_rows(issue_path):
    for row in contract.rows(issue_path, contract.FEATURES):
        for method in METHODS:
            for result in forecast(row, method):
                if result[0] in (0, HORIZON_S):
                    yield (*key(row), method, *result)


def key(row):
    return tuple(row[name] for name in contract.IDS)


def evaluation_rows(folder):
    schedule = {key(row): row for row in contract.rows(folder / "schedule.csv", SCHEDULE)}
    labels = {key(row): row for row in contract.rows(folder / "truth.csv", contract.LABELS)
              if int(row["horizon_s"]) == HORIZON_S}
    for row in contract.rows(folder / "forecasts.csv", contract.FORECASTS):
        if int(row["horizon_s"]) != HORIZON_S:
            continue
        truth = labels[key(row)]
        burden_samples = json.loads(row["burden_samples"]) if row["burden_samples"] else []
        draws = sorted(dynamics.targets(value)[1] for value in burden_samples)
        score = ("", "", "", "")
        if draws:
            score = (crps(draws, float(truth["rgb_t550"])),
                     *(quantile(draws, q) for q in (.5, .025, .975)))
        yield (*(schedule[key(row)][field] for field in SCHEDULE), row["method"], HORIZON_S,
               truth["rgb_t550"], truth["lwir_t11"], row["status"], row["flags"], row["ess"],
               len(draws), *score)


def run(fixture_path, folder):
    fixture, length, inverse = load_fixture(fixture_path)
    base = fixture["future_seed_base"]
    if type(base) is not int or base < 0:
        raise ValueError("future seed base must be a nonnegative integer")
    future_seeds = [base + index for index in range(len(fixture["issues"]))]
    seeds = future_seeds + [row[k] for row in fixture["issues"]
                           for k in ("initial_seed", "rgb_seed", "lwir_seed")]
    if any(type(seed) is not int or seed < 0 for seed in seeds) or len(set(seeds)) != len(seeds):
        raise ValueError("all random streams must use distinct nonnegative integer seeds")

    folder.mkdir()
    write_rows(folder / "schedule.csv", SCHEDULE, schedule_rows(fixture, length))
    write_rows(folder / "issues.csv", contract.FEATURES, issue_rows(fixture["issues"], length, inverse))
    write_rows(folder / "forecasts.csv", contract.FORECASTS, forecast_rows(folder / "issues.csv"))
    write_rows(folder / "truth.csv", contract.LABELS, label_rows(fixture, length, future_seeds))
    write_rows(folder / "evaluation.csv", EVALUATION, evaluation_rows(folder))
    manifest = {
        "schema_version": 1, "fixture_id": fixture["fixture_id"], "python": sys.version.split()[0],
        "geometry": fixture["geometry"], "tick_period_ns": fixture["tick_period_ns"],
        "horizon_s": HORIZON_S, "issue_specs": fixture["issues"], "future_seeds": future_seeds,
        "fixture_sha256": sha256(fixture_path), "adapter_sha256": sha256(Path(__file__)),
        "frozen_package_manifest_sha256": {
            model: sha256(M08.parent / model / "PACKAGE_SHA256SUMS.txt")
            for model in ("m05", "m06", "m07", "m08")
        },
        "artifacts_sha256": {path.name: sha256(path) for path in sorted(folder.glob("*.csv"))},
    }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path, help="new output directory")
    parser.add_argument("--fixture", type=Path, default=HERE / "fixture.json")
    args = parser.parse_args()
    run(args.fixture, args.out)
