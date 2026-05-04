#!/usr/bin/env python3
"""Parse test metrics. Renders a markdown report for one run, or a diff
between two runs when --baseline is supplied.

Reads results.xml (JUnit XML) for test durations and metrics.json for custom
metrics. In diff mode, exits 1 on regression; in single mode, always exits 0.

Usage:
    python parse_metrics.py --current tests/results/<run>/
    python parse_metrics.py --current tests/results/<run>/ --baseline tests/results/<run>/
    python parse_metrics.py --current tests/results/<run>/ --baseline tests/results/<run>/ --threshold 30
"""
import argparse
import json
import re
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from tabulate import tabulate

FLAG_SUFFIX = {"regression": " :red_circle:", "improved": " :green_circle:"}

ITER_RE = re.compile(r"-iter(\d+)(?=\])")
ROBOT_RE = re.compile(r"\brobot_\d+\b")
AGGS = ("mean", "start_mean", "end_mean", "min", "max")

# Time-series metrics recorded as `{prefix}.{type}_samples`. Each entry here
# declares the display unit + regression direction for the derived aggregates.
SAMPLE_TYPES = {
    "hz": {"unit": "Hz", "direction": "higher_is_better"},
    "cpu_pct": {"unit": "%", "direction": "lower_is_better"},
    "mem_mb": {"unit": "MB", "direction": "lower_is_better"},
    "disk_io_mb": {"unit": "MB", "direction": "lower_is_better"},
    "net_io_mb": {"unit": "MB", "direction": "lower_is_better"},
    "gpu_pct": {"unit": "%", "direction": "lower_is_better"},
    "vram_mb": {"unit": "MB", "direction": "lower_is_better"},
    "gpu_temp_c": {"unit": "°C", "direction": "lower_is_better"},
    "gpu_power_w": {"unit": "W", "direction": "lower_is_better"},
    "realtime_factor": {"unit": "", "direction": "higher_is_better"},
}
COMPUTE_TYPES = tuple(k for k in SAMPLE_TYPES if k != "hz")

HZ_METRIC_RE = re.compile(rf"^(.+)\.hz_({'|'.join(AGGS + ('samples',))})$")
COMPUTE_METRIC_RE = re.compile(
    rf"^(.+)\.({'|'.join(COMPUTE_TYPES)})_({'|'.join(AGGS + ('samples',))})$"
)


def _split_test_name(name):
    """`test_liveliness.TestLiveliness.test_foo[id]` →
    (module="test_liveliness", display="test_foo[id]"). Drops the Class segment
    for display since there's one class per module."""
    parts = name.split(".", 2)
    if len(parts) == 3:
        return parts[0], parts[2]
    return parts[0], name


def _aggregate_samples(series_list):
    """Align per-iteration sample lists by `t` and return per-step mean/std.

    Input: [[{"t": 10, "value": 19.27}, ...], [{"t": 10, "value": 45.67}, ...], ...]
    Output: [{"t": 10, "mean": 32.5, "std": 13.2, "n": 2}, ...]
    """
    by_t = defaultdict(list)
    for series in series_list:
        for s in series:
            t, v = s.get("t"), s.get("value", s.get("hz"))
            if t is None or v is None:
                continue
            by_t[t].append(v)
    out = []
    for t in sorted(by_t):
        vals = by_t[t]
        out.append({
            "t": t,
            "mean": round(statistics.mean(vals), 2),
            "std": round(statistics.pstdev(vals), 2) if len(vals) > 1 else 0.0,
            "n": len(vals),
        })
    return out


def _collapse_robot_topic_names(key):
    """`robot_1.sensors.foo` → `robot.sensors.foo`. No-op for keys without a
    topic-style `robot_N` segment."""
    return ROBOT_RE.sub("robot", key)


def _collapse_robot_container_names(key):
    """`airstack-robot-desktop-1.cpu_pct` → `airstack-robot-desktop.cpu_pct`.
    Strips a docker-compose `-N` suffix from the first dotted segment only,
    which is where a container name lives in our metric keys."""
    first, dot, rest = key.partition(".")
    first = re.sub(r"-\d+$", "", first)
    return f"{first}{dot}{rest}"


def _collapse_robots(merged):
    """Merge per-robot metric keys into robot-agnostic ones (homogeneous robots).
    Both topic-style (`robot_N`) and container-style (`-N` replica suffix)
    naming schemes collapse to the same base. Sample lists are concatenated on
    collision."""
    for metrics in merged.values():
        merged_samples = {}
        for key, val in list(metrics.items()):
            new_key = _collapse_robot_container_names(
                _collapse_robot_topic_names(key))
            if new_key == key:
                continue
            metrics.pop(key)
            if isinstance(val, dict) and "samples" in val:
                merged_samples.setdefault(new_key, []).extend(val["samples"])
            else:
                metrics.setdefault(new_key, val)
        for new_key, samples in merged_samples.items():
            metrics[new_key] = {"samples": samples}


def _expand_time_series(merged):
    """For each `{prefix}.{type}_samples` time series (type ∈ SAMPLE_TYPES),
    synthesize scalar aggregates ({type}_{mean,min,max,start_mean,end_mean})
    as peer metrics. Mutates `merged` in place."""
    for metrics in merged.values():
        for key, val in list(metrics.items()):
            if not (isinstance(val, dict) and "samples" in val
                    and key.endswith("_samples")):
                continue
            stem = key.removesuffix("_samples")
            sample_type = next(
                (t for t in SAMPLE_TYPES if stem.endswith(f".{t}")), None)
            if sample_type is None:
                continue
            samples = val["samples"]
            if not samples:
                continue
            # Post-collapse, samples from different robots/containers may
            # interleave. Sort by t so start_mean/end_mean land on clean halves.
            samples = sorted(samples, key=lambda s: s["t"])
            meta = SAMPLE_TYPES[sample_type]
            vals = [s.get("value", s.get("hz")) for s in samples]
            ts = [s["t"] for s in samples]
            half = len(samples) // 2 or 1
            aggs = {
                "mean": {"value": round(statistics.mean(vals), 2), **meta},
                "min": {"value": min(vals), **meta},
                "max": {"value": max(vals), **meta},
                "start_mean": {
                    "value": round(statistics.mean(vals[:half]), 2),
                    "t_start": ts[0], "t_end": ts[half - 1], **meta,
                },
            }
            if len(vals) > half:
                aggs["end_mean"] = {
                    "value": round(statistics.mean(vals[half:]), 2),
                    "t_start": ts[half], "t_end": ts[-1], **meta,
                }
            for agg_name, entry in aggs.items():
                metrics.setdefault(f"{stem}_{agg_name}", entry)


def parse_results_xml(path):
    tree = ET.parse(path)
    metrics = {}
    for tc in tree.iter("testcase"):
        name = f"{tc.get('classname')}.{tc.get('name')}"
        failed = tc.find("failure") is not None
        metrics[name] = {
            "duration_s": {
                "value": float(tc.get("time", 0)),
                "unit": "s",
                "direction": "lower_is_better",
            },
            "status": "failed" if failed else "passed",
        }
    return metrics


def parse_passrates(path):
    """Per (module, base_test) pass/fail/skip counts aggregated across -iterN
    iterations. results.xml is the authoritative source — metrics.json can't
    distinguish early-fail from skipped (neither produces entries)."""
    if not path.exists():
        return {}
    counts = {}
    for tc in ET.parse(path).iter("testcase"):
        full = f"{tc.get('classname')}.{tc.get('name')}"
        module, display = _split_test_name(full)
        base = ITER_RE.sub("", display)
        if tc.find("failure") is not None or tc.find("error") is not None:
            outcome = "fail"
        elif tc.find("skipped") is not None:
            outcome = "skip"
        else:
            outcome = "pass"
        counts.setdefault((module, base), {"pass": 0, "fail": 0, "skip": 0})[outcome] += 1
    return counts


def parse_metrics_json(path):
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def merge_metrics(run_dir):
    merged = {}
    results_xml = run_dir / "results.xml"
    if results_xml.exists():
        merged.update(parse_results_xml(results_xml))
    metrics_json = run_dir / "metrics.json"
    for test_name, test_metrics in parse_metrics_json(metrics_json).items():
        if test_name not in merged:
            merged[test_name] = {}
        merged[test_name].update(test_metrics)
    _collapse_robots(merged)
    _expand_time_series(merged)
    return _collapse_iterations(merged)


def _collapse_iterations(merged):
    """Strip -iterN from test keys and aggregate metrics across iterations.

    - Numeric scalars → `value` = mean, `stddev`, `n` (success count),
      `total` (iter count), `failures` (sentinel count), `missing` (neither).
      If all iterations produced sentinels (e.g. all timeout), value = sentinel.
    - Time-series `samples` → aligned per `t` across iterations; output list
      has `hz_mean`, `hz_std`, `n` per step.
    """
    numeric = {}  # (base, key) -> {"values": [...], "sentinels": [...], "meta": {...}}
    series = {}   # (base, key) -> [samples_list, ...]
    out = {}
    iters_per_test = {}

    for name, metrics in merged.items():
        m = ITER_RE.search(name)
        iter_n = int(m.group(1)) if m else 0
        base = ITER_RE.sub("", name)
        iters_per_test.setdefault(base, set()).add(iter_n)
        bucket = out.setdefault(base, {})
        for key, val in metrics.items():
            if key == "status":
                if val == "failed" or bucket.get("status") == "failed":
                    bucket["status"] = "failed"
                else:
                    bucket["status"] = val
                continue
            if isinstance(val, dict) and "samples" in val:
                series.setdefault((base, key), []).append(val["samples"])
                continue
            if isinstance(val, dict) and "value" in val:
                acc = numeric.setdefault((base, key), {
                    "values": [], "sentinels": [], "meta": {
                        "unit": val.get("unit", ""),
                        "direction": val.get("direction", "lower_is_better"),
                    }})
                for k, v2 in val.items():
                    if k not in ("value", "unit", "direction", "samples"):
                        acc["meta"].setdefault(k, v2)
                v = val["value"]
                (acc["values"] if isinstance(v, (int, float)) else acc["sentinels"]).append(v)
                continue
            bucket.setdefault(key, val)

    for (base, key), acc in numeric.items():
        total = len(iters_per_test[base])
        nums, sentinels = acc["values"], acc["sentinels"]
        entry = dict(acc["meta"])
        entry["total"] = total
        entry["failures"] = len(sentinels)
        entry["missing"] = total - len(nums) - len(sentinels)
        if nums:
            entry["value"] = round(statistics.mean(nums), 3)
            entry["stddev"] = round(statistics.pstdev(nums), 3) if len(nums) > 1 else 0.0
            entry["n"] = len(nums)
        elif sentinels:
            entry["value"] = sentinels[0]
            entry["n"] = 0
        else:
            continue
        out[base][key] = entry

    for (base, key), series_list in series.items():
        out[base][key] = {
            "samples": _aggregate_samples(series_list),
            "n": len(series_list),
        }
    return out


def _is_scored(entry):
    """True if this metric entry can be numerically compared.

    Skip time-series (list-valued, key 'samples'), non-numeric sentinels ('timeout'),
    and any dict that lacks a 'value' field.
    """
    if not isinstance(entry, dict):
        return False
    if "samples" in entry:
        return False
    if "value" not in entry:
        return False
    v = entry["value"]
    return isinstance(v, (int, float))


def _fmt(entry):
    """Format a metric entry for display. For aggregated numeric metrics,
    shows `mean ± stddev (n=success/total, F fail, M miss)`. Flags (failures,
    missing) are only appended when non-zero."""
    if not isinstance(entry, dict):
        return str(entry)
    if "samples" in entry:
        label = f"[{len(entry['samples'])} steps"
        if entry.get("n", 1) > 1:
            label += f" × n={entry['n']}"
        return label + "]"
    if "value" not in entry:
        return "—"
    v = entry["value"]
    unit = entry.get("unit", "")
    n = entry.get("n", 0)
    total = entry.get("total", n)
    failures = entry.get("failures", 0)
    missing = entry.get("missing", 0)
    stddev = entry.get("stddev")

    def _context():
        parts = [f"n={n}/{total}"] if total > 1 else []
        if failures:
            parts.append(f"{failures} fail")
        if missing:
            parts.append(f"{missing} miss")
        return f" ({', '.join(parts)})" if parts else ""

    if isinstance(v, (int, float)):
        base = f"{v:.4g}{unit}"
        if total > 1 and stddev is not None:
            base = f"{v:.4g}{unit} ± {stddev:.2g}"
        return base + _context()
    body = f"{v}{unit}" if unit else str(v)
    return body + _context()


def _score(c, b, threshold):
    """Compute change% and regression flag for a metric pair. Returns
    (change_str, flag). flag ∈ {"", "regression", "improved"}. When either
    entry is missing/sentinel/time-series, returns a stub with an empty flag
    (except: `timeout` current after numeric baseline → regression)."""
    if not c or not b:
        return ("new" if c and not b else "removed"), ""
    if not _is_scored(c) or not _is_scored(b):
        cv = c.get("value") if isinstance(c, dict) else None
        bv = b.get("value") if isinstance(b, dict) else None
        flag = "regression" if (cv == "timeout" and isinstance(bv, (int, float))) else ""
        return "—", flag
    cv, bv = c["value"], b["value"]
    direction = c.get("direction", "lower_is_better")
    change_pct = ((cv - bv) / bv) * 100 if bv != 0 else 0
    regressed = (direction == "lower_is_better" and change_pct > threshold) or \
                (direction == "higher_is_better" and change_pct < -threshold)
    improved = (direction == "lower_is_better" and change_pct < -threshold) or \
               (direction == "higher_is_better" and change_pct > threshold)
    flag = "regression" if regressed else ("improved" if improved else "")
    return f"{change_pct:+.1f}%", flag


def _pivot_cell(c, b, threshold, diff_mode):
    """Render a pivot-table cell. Diff mode: `b_short → c_short (Δ%[, t=...])`
    + regression flag suffix. Single mode: just the current value. Returns
    (text, flag)."""
    def t_window(entry):
        ts, te = (entry or {}).get("t_start"), (entry or {}).get("t_end")
        return f"t={ts}-{te}s" if ts is not None and te is not None else ""

    if not diff_mode or not (c and b):
        entry = c or b
        if not entry:
            return "—", ""
        if not _is_scored(entry):
            return _fmt(entry), ""
        text = f"{entry['value']:.4g}{entry.get('unit', '')}"
        t = t_window(entry)
        return (f"{text} ({t})" if t else text), ""

    if not _is_scored(c) or not _is_scored(b):
        return f"{_fmt(b)} → {_fmt(c)}", ""

    change, flag = _score(c, b, threshold)
    t = t_window(c)
    annotations = ([t] if t else []) + [change]
    return f"{b['value']:.4g} → {c['value']:.4g} ({', '.join(annotations)})", flag


def build_rows(current, baseline):
    """Route metrics into three groups: flat rows, hz pivot rows, compute pivot
    rows. Rows carry raw metric entries; rendering/scoring happens in
    format_markdown. `baseline` may be an empty dict for single-input mode.
    Test execution order from `current` is preserved for grouping."""
    main_rows = []
    hz_data = {}       # (test, module, display, topic) → {agg: (c, b)}
    compute_data = {}  # (test, module, display, entity, metric_type) → {agg: (c, b)}
    iter_counts = {}   # module → (baseline_n, current_n)

    def note_iters(module, c, b):
        if module not in iter_counts:
            b_n = b.get("total") if isinstance(b, dict) else None
            c_n = c.get("total") if isinstance(c, dict) else None
            iter_counts[module] = (b_n, c_n)

    ordered_tests = list(current) + [t for t in baseline if t not in current]
    for test in ordered_tests:
        module, display = _split_test_name(test)
        curr = current.get(test, {})
        base = baseline.get(test, {})
        metric_keys = [k for k in curr if k != "status"] + \
                      [k for k in base if k != "status" and k not in curr]
        for key in metric_keys:
            c, b = curr.get(key), base.get(key)

            hz_m = HZ_METRIC_RE.match(key)
            if hz_m:
                topic, agg = hz_m.group(1), hz_m.group(2)
                if agg == "samples":
                    continue
                hz_data.setdefault((test, module, display, topic), {})[agg] = (c, b)
                note_iters(module, c, b)
                continue

            compute_m = COMPUTE_METRIC_RE.match(key)
            if compute_m:
                entity, metric_type, agg = compute_m.group(1), compute_m.group(2), compute_m.group(3)
                if agg == "samples":
                    continue
                compute_data.setdefault(
                    (test, module, display, entity, metric_type), {}
                )[agg] = (c, b)
                note_iters(module, c, b)
                continue

            main_rows.append({
                "module": module, "test": display, "metric": key,
                "current_entry": c, "baseline_entry": b,
            })

    hz_rows = [
        {"module": module, "test": display, "topic": topic, "aggs": aggs}
        for (_, module, display, topic), aggs in hz_data.items()
    ]
    compute_rows = [
        {"module": module, "test": display, "entity": entity,
         "metric_type": metric_type, "aggs": aggs}
        for (_, module, display, entity, metric_type), aggs in compute_data.items()
    ]
    return main_rows, hz_rows, compute_rows, iter_counts


def _iter_annotation(baseline_n, current_n, diff_mode):
    if not diff_mode:
        return f"n={current_n} iterations; " if current_n else ""
    if baseline_n and current_n and baseline_n == current_n:
        return f"n={baseline_n} iterations; "
    if baseline_n or current_n:
        return f"baseline n={baseline_n}, current n={current_n}; "
    return ""


def _group_by_module(rows):
    modules = []
    grouped = {}
    for r in rows:
        mod = r["module"]
        if mod not in grouped:
            grouped[mod] = []
            if mod not in modules:
                modules.append(mod)
        grouped[mod].append(r)
    return modules, grouped


def format_markdown(main_rows, hz_rows, compute_rows, iter_counts,
                    current_pr, baseline_pr, threshold, diff_mode):
    regressions = [False]

    def pivot_cell(pair):
        if not pair:
            return "—"
        text, flag = _pivot_cell(*pair, threshold=threshold, diff_mode=diff_mode)
        if flag == "regression":
            regressions[0] = True
        return text + FLAG_SUFFIX.get(flag, "")

    def render_main(rows):
        if diff_mode:
            out = []
            for r in rows:
                c, b = r["current_entry"], r["baseline_entry"]
                change, flag = _score(c, b, threshold)
                if flag == "regression":
                    regressions[0] = True
                out.append([
                    r["test"], r["metric"],
                    _fmt(b) if b else "—",
                    _fmt(c) if c else "—",
                    change + FLAG_SUFFIX.get(flag, ""),
                ])
            return out, ["Test", "Metric", "Baseline", "Current", "Change"]
        return ([[r["test"], r["metric"], _fmt(r["current_entry"])] for r in rows],
                ["Test", "Metric", "Value"])

    def render_pivot(rows, leading):
        return [leading(r) + [pivot_cell(r["aggs"].get(agg)) for agg in AGGS]
                for r in rows]

    def _rate(c):
        considered = c["pass"] + c["fail"]
        return f"{c['pass'] * 100 / considered:.0f}%" if considered else "—"

    def render_passrates(mod):
        bases = sorted({b for (m, b) in current_pr if m == mod}
                       | {b for (m, b) in baseline_pr if m == mod})
        if not bases:
            return None
        rows = []
        empty = {"pass": 0, "fail": 0, "skip": 0}
        if diff_mode:
            for b in bases:
                cur, bl = current_pr.get((mod, b), empty), baseline_pr.get((mod, b), empty)
                rows.append([
                    b,
                    f"{bl['pass']} → {cur['pass']}",
                    f"{bl['fail']} → {cur['fail']}",
                    f"{bl['skip']} → {cur['skip']}",
                    f"{_rate(bl)} → {_rate(cur)}",
                ])
            headers = ["Test", "Pass", "Fail", "Skip", "Rate (baseline → current)"]
        else:
            for b in bases:
                c = current_pr.get((mod, b), empty)
                rows.append([b, c["pass"], c["fail"], c["skip"], _rate(c)])
            headers = ["Test", "Pass", "Fail", "Skip", "Rate"]
        return tabulate(rows, headers=headers, tablefmt="github")

    main_mods, main_by_module = _group_by_module(main_rows)
    hz_mods, hz_by_module = _group_by_module(hz_rows)
    compute_mods, compute_by_module = _group_by_module(compute_rows)
    pr_mods = list(dict.fromkeys(m for (m, _) in list(current_pr) + list(baseline_pr)))
    modules = []
    for m in main_mods + hz_mods + compute_mods + pr_mods:
        if m not in modules:
            modules.append(m)

    hz_suffix = "baseline → current, per-topic" if diff_mode else "per-topic"
    compute_suffix = ("baseline → current, per-container and global" if diff_mode
                      else "per-container and global")

    sections = []
    for mod in modules:
        sub = [f"## {mod}"]
        b_n, c_n = iter_counts.get(mod, (None, None))
        annotation = _iter_annotation(b_n, c_n, diff_mode)

        pr_table = render_passrates(mod)
        if pr_table is not None:
            sub.append("### Pass rates\n\n" + pr_table)

        main = main_by_module.get(mod, [])
        if main:
            rows, headers = render_main(main)
            sub.append("### Metrics\n\n" + tabulate(rows, headers=headers, tablefmt="github"))

        hz = hz_by_module.get(mod, [])
        if hz:
            sub.append(
                f"### Sim publishing rates ({annotation}{hz_suffix})\n\n"
                + tabulate(render_pivot(hz, lambda r: [r["test"], r["topic"]]),
                           headers=["Test", "Topic", *AGGS], tablefmt="github"))

        compute = compute_by_module.get(mod, [])
        if compute:
            sub.append(
                f"### Compute usage ({annotation}{compute_suffix})\n\n"
                + tabulate(render_pivot(
                    compute, lambda r: [r["test"], r["entity"], r["metric_type"]]),
                    headers=["Test", "Entity", "Metric", *AGGS], tablefmt="github"))

        sections.append("\n\n".join(sub))

    has_regression = regressions[0]
    if diff_mode and has_regression:
        sections.append("**Regression detected** — some metrics exceeded the threshold.")

    return "\n\n".join(sections), has_regression


def main():
    parser = argparse.ArgumentParser(
        description="Render a markdown report for a test run, or a diff if --baseline is supplied.")
    parser.add_argument("--current", required=True, help="Current run directory")
    parser.add_argument("--baseline", help="Baseline run directory (enables diff mode)")
    parser.add_argument("--threshold", type=float, default=20, help="Regression threshold (%%)")
    parser.add_argument("--output", help="Write markdown report to file")
    args = parser.parse_args()

    current = merge_metrics(Path(args.current))
    baseline = merge_metrics(Path(args.baseline)) if args.baseline else {}
    current_pr = parse_passrates(Path(args.current) / "results.xml")
    baseline_pr = (parse_passrates(Path(args.baseline) / "results.xml")
                   if args.baseline else {})
    diff_mode = bool(args.baseline)

    main_rows, hz_rows, compute_rows, iter_counts = build_rows(current, baseline)
    md, has_regression = format_markdown(
        main_rows, hz_rows, compute_rows, iter_counts,
        current_pr, baseline_pr, args.threshold, diff_mode)

    print(md)
    if args.output:
        Path(args.output).write_text(md)

    sys.exit(1 if diff_mode and has_regression else 0)


if __name__ == "__main__":
    main()
