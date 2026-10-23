"""Describe recorded timestamp structure without fitting or applying a clock fix."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def summary_ns(values):
    values = np.asarray(values, dtype=np.int64)
    return {
        "count": int(values.size),
        "min_ms": int(values.min()) / 1e6,
        "p10_median_p90_ms": (np.percentile(values, [10, 50, 90]) / 1e6).tolist(),
        "max_ms": int(values.max()) / 1e6,
        "negative_count": int((values < 0).sum()),
        "positive_count": int((values > 0).sum()),
    }


def analyze(directory, output):
    directory, output = Path(directory), Path(output)
    output.mkdir(exist_ok=False, parents=True)
    times, hashes = {}, {}
    for sensor in ["rgb", "thr"]:
        for side in ["left", "right"]:
            key = f"{sensor}_{side}"
            path = directory / (key + ".txt")
            times[key] = np.loadtxt(path, dtype=np.int64)
            hashes[key] = hashlib.file_digest(path.open("rb"), "sha256").hexdigest()
            if times[key].ndim != 1 or not (np.diff(times[key]) > 0).all():
                raise ValueError("Expected strictly increasing integer timestamp rows")
    if len({len(x) for x in times.values()}) != 1:
        raise ValueError("Different timestamp row counts")
    n = len(times["rgb_left"])
    offsets = times["thr_right"] - times["thr_left"]
    elapsed = (times["thr_left"] - times["thr_left"][0]) / 1e9
    shifts = {}
    for shift in [-1, 0, 1]:
        left = np.arange(max(0, -shift), min(n, n - shift))
        delta = times["thr_right"][left + shift] - times["thr_left"][left]
        shifts[str(shift)] = {
            "right_minus_left": summary_ns(delta),
            "absolute_min_ms": int(np.abs(delta).min()) / 1e6,
            "absolute_within_2ms": int((np.abs(delta) <= 2_000_000).sum()),
        }
    bins = []
    for indices in np.array_split(np.arange(n), 10):
        bins.append({
            "first_index": int(indices[0]), "last_index": int(indices[-1]),
            "elapsed_start_s": float(elapsed[indices[0]]),
            "elapsed_end_s": float(elapsed[indices[-1]]),
            "signed_thermal_right_minus_left": summary_ns(offsets[indices]),
        })
    result = {
        "scope": "Recorded source times only; no re-pairing, clock fit or correction",
        "rows": n, "input_sha256": hashes,
        "signed_thermal_right_minus_left": summary_ns(offsets),
        "offset_change_between_adjacent_ids": summary_ns(np.diff(offsets)),
        "offset_sign_changes": int(((offsets[1:] > 0) != (offsets[:-1] > 0)).sum()),
        "constant_shift_worst_case_lower_bound_ms": int(offsets.max() - offsets.min()) / 2e6,
        "native_intervals": {k: summary_ns(np.diff(v)) for k, v in times.items()},
        "fixed_right_index_shifts": shifts,
        "ten_fixed_index_bins": bins,
    }
    with (output / "signed-offsets.csv").open("w") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "thermal_left_ns", "thermal_right_ns", "right_minus_left_ns"])
        writer.writerows(zip(range(n), times["thr_left"], times["thr_right"], offsets))
    (output / "analysis.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: result[k] for k in ["rows", "signed_thermal_right_minus_left", "fixed_right_index_shifts"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    analyze(args.input, args.output)
