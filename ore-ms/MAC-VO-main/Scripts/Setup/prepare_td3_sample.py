"""Build a one-to-one TD3 image manifest using acquisition timestamps."""

import argparse
from bisect import bisect_left, bisect_right
from decimal import Decimal
import json
from pathlib import Path


STREAMS = {
    "rgb_left": "multisense_left_rect_color",
    "rgb_right": "multisense_right_rect",
    "thermal_left": "thermal_left_processed",
    "thermal_right": "thermal_right_processed",
}


def load_stream(dataset, name):
    """Keep one existing file per timestamp; retain original sparse row indices."""
    directory = Path(dataset) / "sensors" / name
    records, seen = [], set()
    stats = {"missing": 0, "duplicates": 0, "rows": 0}
    for index, line in enumerate((directory / "timestamps.txt").read_text().splitlines()):
        stats["rows"] += 1
        stamp = Decimal(line.strip())
        file = directory / f"{index:08d}.png"
        if not stamp.is_finite() or not file.is_file():
            stats["missing"] += 1
            continue
        ns = int((stamp * 1_000_000_000).to_integral_value())
        if ns in seen:
            stats["duplicates"] += 1
            continue
        seen.add(ns)
        records.append({"index": index, "time_ns": ns,
                        "path": str(file.relative_to(dataset))})
    stats["out_of_order_steps"] = sum(
        b["time_ns"] < a["time_ns"] for a, b in zip(records, records[1:]))
    records.sort(key=lambda r: (r["time_ns"], r["index"]))
    stats["unique_acquisitions"] = len(records)
    return records, stats


def pair_streams(streams, stereo_ns, cross_ns):
    """Greedy chronological nearest pairing, with no reuse or time reversal.

    Ties choose the earlier acquisition. This policy is explicit and does not
    claim to globally optimize the number of matched samples.
    """
    if stereo_ns < 0 or cross_ns < 0:
        raise ValueError("Timestamp tolerances must be nonnegative")
    times = {k: [r["time_ns"] for r in v] for k, v in streams.items()}
    last = {k: -1 for k in streams}
    paired = []

    def nearest(key, stamp, tolerance, lower=None, upper=None):
        ts = times[key]
        lo = max(stamp - tolerance, lower if lower is not None else stamp - tolerance)
        hi = min(stamp + tolerance, upper if upper is not None else stamp + tolerance)
        begin = max(last[key] + 1, bisect_left(ts, lo))
        end = bisect_right(ts, hi)
        if begin >= end:
            return None
        return min(range(begin, end), key=lambda i: (abs(ts[i] - stamp), ts[i], i))

    for anchor in streams["rgb_left"]:
        stamp = anchor["time_ns"]
        ri = nearest("rgb_right", stamp, stereo_ns)
        ti = nearest("thermal_left", stamp, cross_ns)
        if ri is None or ti is None:
            continue
        rgb_right = streams["rgb_right"][ri]
        thermal_left = streams["thermal_left"][ti]
        # Every thermal acquisition must be within cross_ns of both RGB times.
        lower = max(stamp, rgb_right["time_ns"]) - cross_ns
        upper = min(stamp, rgb_right["time_ns"]) + cross_ns
        if not lower <= thermal_left["time_ns"] <= upper:
            continue
        tr = nearest("thermal_right", thermal_left["time_ns"], stereo_ns, lower, upper)
        if tr is None:
            continue
        row = {"rgb_left": anchor, "rgb_right": rgb_right,
               "thermal_left": thermal_left, "thermal_right": streams["thermal_right"][tr]}
        row["offsets_ns"] = {
            "rgb_stereo": rgb_right["time_ns"] - stamp,
            "thermal_stereo": row["thermal_right"]["time_ns"] - thermal_left["time_ns"],
            "thermal_left_minus_rgb_left": thermal_left["time_ns"] - stamp,
        }
        paired.append(row)
        last.update(rgb_right=ri, thermal_left=ti, thermal_right=tr)
    return paired


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stereo-max-ms", type=Decimal, required=True)
    parser.add_argument("--cross-max-ms", type=Decimal, required=True)
    args = parser.parse_args()
    stereo_ns = int(args.stereo_max_ms * 1_000_000)
    cross_ns = int(args.cross_max_ms * 1_000_000)
    streams, stats = {}, {}
    for key, name in STREAMS.items():
        streams[key], stats[key] = load_stream(args.dataset, name)
    rows = pair_streams(streams, stereo_ns, cross_ns)
    manifest = {"dataset": str(args.dataset.resolve()), "streams": STREAMS,
                "policy": {"name": "chronological-nearest-without-reuse",
                           "stereo_max_ns": stereo_ns, "cross_max_ns": cross_ns,
                           "cross_bound": "each RGB/thermal acquisition pair",
                           "status": "exploratory thresholds; not an accuracy protocol"},
                "stream_stats": stats, "pair_count": len(rows), "pairs": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"pairs": len(rows), "stream_stats": stats,
                      "output": str(args.output)}))
    if not rows:
        raise SystemExit("No pairs satisfy the specified tolerances")


if __name__ == "__main__":
    main()
