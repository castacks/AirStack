"""Fixed task frontends. The only data input is the observation store."""
import argparse
import csv
import json
from pathlib import Path
from time import perf_counter

import numpy as np

CONFIG = json.loads(Path(__file__).with_name("config.json").read_text())
W, H = CONFIG["grid"]
SPEC = CONFIG["frontends"]
OBS = ("id", "split", "rgb_ms", "lwir_ms", "registration_valid", "range_m", "material_qualified")
FEATURES = OBS[2:]
PRED = ("id", "arm", "available_rgb", "available_lwir", "x", "y", "support", "score", "reason")
ARMS = ("B01", "B02", "B03")


def component(contrast):
    remaining = set(np.flatnonzero(contrast.ravel() >= 1).tolist())
    flat = contrast.ravel()
    best = None
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        group = [first]
        for i in group:
            y, x = divmod(i, W)
            for adjacent in (i-1 if x else -1, i+1 if x < W-1 else -1,
                             i-W if y else -1, i+W if y < H-1 else -1):
                if adjacent in remaining:
                    remaining.remove(adjacent)
                    group.append(adjacent)
        if len(group) >= SPEC["minimum_component"]:
            peak = float(max(flat[i] for i in group))
            if best is None or peak > best[0]:
                best = (peak, sum(i % W for i in group)/len(group),
                        sum(i // W for i in group)/len(group), len(group))
    return best


def timely(stamp):
    return 0 <= CONFIG["decision_ms"] - int(stamp) <= SPEC["max_age_ms"]


def map_evidence(signal, valid, threshold):
    if not np.any(valid):
        return np.zeros((H, W))
    delta = np.maximum(signal - np.median(signal[valid]), 0) / threshold
    return np.where(valid, delta, 0)


def predict_frame(metadata, codes, masks):
    if tuple(metadata) != FEATURES:
        raise ValueError("unapproved inference schema")
    rgb_valid = (masks[..., :3] == 1).all(axis=-1) & timely(metadata["rgb_ms"])
    lwir_valid = (masks[..., 3] == 1) & timely(metadata["lwir_ms"]) & (metadata["registration_valid"] == "1")
    rgb = map_evidence(codes[..., :3].mean(axis=-1)/4095, rgb_valid, SPEC["rgb_min_delta"])
    lw = map_evidence(codes[..., 3]/4095, lwir_valid, SPEC["lwir_min_delta"])
    a, b = component(rgb), component(lw)
    available = (int(rgb_valid.any()), int(lwir_valid.any()))
    if a is not None and b is not None:
        if np.hypot(a[1]-b[1], a[2]-b[2]) <= SPEC["candidate_agreement_px"]:
            fused = component(np.where(rgb_valid & lwir_valid, (rgb+lw)/2, 0))
            mode = "equal" if fused else "no_shared_support"
        else:
            fused, mode = None, "conflict"
    elif a is not None or b is not None:
        fused, mode = (a, "fallback_rgb") if a is not None else (b, "fallback_lwir")
    else:
        fused, mode = None, "no_candidate"
    reasons = ["candidate" if candidate else "no_component" if valid else "unavailable"
               for candidate, valid in zip((a, b), available)]
    rows = []
    for arm, candidate, reason in zip(ARMS, (a, b, fused), (*reasons, mode)):
        peak, x, y, count = candidate or (0, "", "", 0)
        rows.append((arm, *available, x, y, count, min(peak/4, 1), reason))
    return rows


def run(source, destination, runtime=None):
    start = perf_counter()
    codes, masks = [np.load(source/f"{name}.npy", mmap_mode="r", allow_pickle=False)
                    for name in ("codes", "masks")]
    count = 0
    durations = []
    with (source/"frames.csv").open() as frame, destination.open("w", newline="") as out:
        reader = csv.DictReader(frame)
        if tuple(reader.fieldnames or ()) != OBS:
            raise ValueError("unapproved inference schema")
        if codes.shape != masks.shape or codes.shape[1:] != (H, W, 4):
            raise ValueError("unexpected observation shape")
        if codes.dtype != np.uint16 or masks.dtype != np.uint8:
            raise ValueError("unexpected observation type")
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(PRED)
        for i, row in enumerate(reader):
            if int(row["id"]) != i or row["split"] not in CONFIG["split_bounds"]:
                raise ValueError("unexpected observation index")
            t = perf_counter()
            evidence = predict_frame({field: row[field] for field in FEATURES}, codes[i], masks[i])
            writer.writerows((row["id"], *prediction) for prediction in evidence)
            durations.append(perf_counter()-t)
            count += 1
    if count != len(codes):
        raise ValueError("incomplete observation index")
    if runtime is not None:
        p50, p95, p99 = np.percentile(durations, [50, 95, 99])
        runtime.write_text(json.dumps({"frames": count, "elapsed_s": perf_counter()-start,
                                       "p50_ms": 1000*p50, "p95_ms": 1000*p95, "p99_ms": 1000*p99},
                                      indent=2)+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--runtime", type=Path)
    args = parser.parse_args()
    run(args.source, args.destination, args.runtime)
