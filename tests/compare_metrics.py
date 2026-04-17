#!/usr/bin/env python3
"""Compare test metrics between two runs.

Reads results.xml (JUnit XML) for test durations and metrics.json for custom
metrics (image sizes, etc.). Outputs a markdown table and exits 1 on regression.

Usage:
    python compare_metrics.py --current tests/results/<run>/ --baseline tests/results/<run>/
    python compare_metrics.py --current tests/results/<run>/ --baseline tests/results/<run>/ --threshold 30
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


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
    return merged


def compare(current, baseline, threshold):
    rows = []
    has_regression = False

    all_tests = sorted(set(list(current.keys()) + list(baseline.keys())))
    for test in all_tests:
        curr = current.get(test, {})
        base = baseline.get(test, {})

        # Collect all metric keys (skip 'status')
        metric_keys = sorted(set(
            [k for k in curr if k != "status"] +
            [k for k in base if k != "status"]
        ))
        for key in metric_keys:
            c = curr.get(key)
            b = base.get(key)
            if not c or not b:
                rows.append({
                    "test": test, "metric": key,
                    "baseline": f"{b['value']}{b.get('unit', '')}" if b else "—",
                    "current": f"{c['value']}{c.get('unit', '')}" if c else "—",
                    "change": "new" if c and not b else "removed",
                    "flag": "",
                })
                continue

            cv, bv = c["value"], b["value"]
            direction = c.get("direction", "lower_is_better")

            if bv != 0:
                change_pct = ((cv - bv) / bv) * 100
            else:
                change_pct = 0

            # Determine if this is a regression
            regressed = (direction == "lower_is_better" and change_pct > threshold) or \
                        (direction == "higher_is_better" and change_pct < -threshold)
            improved = (direction == "lower_is_better" and change_pct < -threshold) or \
                       (direction == "higher_is_better" and change_pct > threshold)

            if regressed:
                has_regression = True

            rows.append({
                "test": test, "metric": key,
                "baseline": f"{bv:.1f}{b.get('unit', '')}",
                "current": f"{cv:.1f}{c.get('unit', '')}",
                "change": f"{change_pct:+.1f}%",
                "flag": "regression" if regressed else ("improved" if improved else ""),
            })

    return rows, has_regression


def format_markdown(rows, has_regression):
    lines = [
        "| Test | Metric | Baseline | Current | Change |",
        "|------|--------|----------|---------|--------|",
    ]
    for r in rows:
        change = r["change"]
        if r["flag"] == "regression":
            change += " :red_circle:"
        elif r["flag"] == "improved":
            change += " :green_circle:"
        lines.append(f"| {r['test']} | {r['metric']} | {r['baseline']} | {r['current']} | {change} |")

    if has_regression:
        lines += ["", "**Regression detected** — some metrics exceeded the threshold."]

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Compare test metrics between runs")
    parser.add_argument("--current", required=True, help="Current run directory")
    parser.add_argument("--baseline", required=True, help="Baseline run directory")
    parser.add_argument("--threshold", type=float, default=20, help="Regression threshold (%%)")
    parser.add_argument("--output", help="Write markdown report to file")
    args = parser.parse_args()

    current = merge_metrics(Path(args.current))
    baseline = merge_metrics(Path(args.baseline))
    rows, has_regression = compare(current, baseline, args.threshold)
    md = format_markdown(rows, has_regression)

    print(md)
    if args.output:
        Path(args.output).write_text(md)

    sys.exit(1 if has_regression else 0)


if __name__ == "__main__":
    main()
