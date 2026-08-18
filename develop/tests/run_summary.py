"""Write a human-readable summary.txt for each test run.

Called automatically at pytest session end (see conftest.py). Users can also
regenerate manually:

    python3 tests/run_summary.py tests/results/<run-dir>/
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

from harness.run_meta import classify_run
from harness.test_ids import canonical_test_id

PARAM_RE = re.compile(r"\[(.+)\]$")
ITER_RE = re.compile(r"-iter\d+$")
ROBOT_METRIC_RE = re.compile(r"^robot_\d+\.(.+)$")
# pytest nodeid path prefix (metrics.json) vs JUnit classname (results.xml)
MODULE_RE = re.compile(r"(?:^|\.)(test_\w+)\.Test")
PHASE_RE = re.compile(r"\.Test[A-Za-z0-9_]+\.(test_\w+)(?:\[|$)")

# Ordered (metric_key, label) groups per test module. Only scalar metrics with
# a numeric "value" field are emitted.
FLIGHT_METRICS = [
    ("ready_duration_sys_s", "PX4 ready time"),
    ("takeoff_duration_sim_s", "Takeoff duration"),
    ("altitude_error_m", "Altitude error after takeoff"),
    ("overshoot_m", "Takeoff overshoot"),
    ("trajectory_success", "Trajectory success"),
    ("trajectory_execution_time_sim_s", "Trajectory duration"),
    ("cross_track_error_mean_m", "Cross-track error (mean)"),
    ("cross_track_error_max_m", "Cross-track error (max)"),
    ("path_rmse_m", "Path RMSE"),
    ("hover_duration_sim_s", "Hover duration"),
    ("hover_altitude_error_m", "Hover altitude error"),
    ("land_duration_sim_s", "Landing duration"),
    ("final_altitude_m", "Final altitude"),
]

LIVELINESS_METRICS = [
    ("sim_ready_duration_s", "Sim ready time"),
    ("sensors_sim_ready_duration_s", "Sensors sim ready time"),
]

# Some metrics were recorded with wrong units before METRIC_UNITS was updated.
UNIT_OVERRIDES = {
    "ready_duration_sys_s": "s",
    "airstack_up_duration_s": "s",
    "airstack_down_duration_s": "s",
}

PHASE_ORDER = {
    "test_px4_ready": 0,
    "test_takeoff": 1,
    "test_fixed_trajectory": 2,
    "test_hover": 2,
    "test_landing": 3,
    "test_land": 3,
}


def _normalize_keyed_map(raw: dict) -> dict:
    """Merge entries that differ only by path-slash vs dot classname form."""
    out: dict = {}
    for key, value in raw.items():
        out[canonical_test_id(key)] = value
    return out


def _parse_results_xml(path: Path) -> tuple[dict[str, str], dict[str, float]]:
    """Return ({full_test_name: status}, {full_test_name: wall_time_s})."""
    if not path.exists():
        return {}, {}
    try:
        testcases = ET.parse(path).iter("testcase")
    except (OSError, ET.ParseError):
        return {}, {}
    statuses: dict[str, str] = {}
    durations: dict[str, float] = {}
    for tc in testcases:
        full = canonical_test_id(f"{tc.get('classname')}.{tc.get('name')}")
        if tc.find("failure") is not None or tc.find("error") is not None:
            statuses[full] = "FAILED"
        elif tc.find("skipped") is not None:
            statuses[full] = "SKIPPED"
        else:
            statuses[full] = "PASSED"
        if tc.get("time"):
            try:
                durations[full] = float(tc.get("time"))
            except ValueError:
                pass
    return statuses, durations


def _load_metrics(path: Path) -> dict:
    if not path.exists():
        return {}
    return _normalize_keyed_map(json.loads(path.read_text()))


def _param_id(test_name: str) -> str:
    m = PARAM_RE.search(test_name)
    return m.group(1) if m else test_name


def _module_name(test_name: str) -> str:
    canonical = canonical_test_id(test_name)
    match = MODULE_RE.search(canonical)
    if match:
        return match.group(1)
    return canonical.split(".", 1)[0]


def _phase_name(test_name: str) -> str:
    """test_fixed_trajectory.TestFixedTrajectory.test_takeoff[...] -> test_takeoff"""
    canonical = canonical_test_id(test_name)
    match = PHASE_RE.search(canonical)
    if match:
        return match.group(1)
    return canonical.split("[", 1)[0]


def _base_param_id(param: str) -> str:
    """isaacsim-rob#1-trajCircle-iter3 -> isaacsim-rob#1-trajCircle"""
    return ITER_RE.sub("", param)


def _format_scalar(key: str, value: float | int, unit: str) -> str:
    if key == "trajectory_success":
        if value == 1.0:
            return "yes"
        if value == 0.0:
            return "no"
    text = f"{value:g}"
    return f"{text} {unit}".strip() if unit else text


def _format_value(key: str, entry: dict) -> str:
    value = entry.get("value")
    unit = UNIT_OVERRIDES.get(key, entry.get("unit", ""))
    if isinstance(value, (int, float)):
        return _format_scalar(key, value, unit)
    if value is None:
        return "n/a"
    return str(value)


def _format_aggregated(key: str, values: list[float], unit: str) -> str:
    if not values:
        return "n/a"
    if key == "trajectory_success":
        passed = sum(1 for v in values if v >= 1.0)
        return f"{passed}/{len(values)} passed"
    mean = statistics.mean(values)
    if len(values) == 1:
        return _format_scalar(key, round(mean, 3), unit)
    std = statistics.pstdev(values)
    base = _format_scalar(key, round(mean, 3), unit)
    return f"{base} ± {std:.3g} {unit}".strip() if unit else f"{base} ± {std:.3g} (n={len(values)})"


def _collect_scalar_metrics(metrics_blob: dict) -> dict[str, list[dict]]:
    """Flatten per-test metrics.json into {metric_key: [entries]}.

    Preserves one entry per robot for multi-robot runs (robot_N.metric_key).
    """
    out: dict[str, list[dict]] = {}
    for key, entry in metrics_blob.items():
        if not isinstance(entry, dict) or "value" not in entry:
            continue
        match = ROBOT_METRIC_RE.match(key)
        metric_key = match.group(1) if match else key
        out.setdefault(metric_key, []).append(entry)
    return out


def _metrics_blob(metrics: dict, test_name: str) -> dict:
    canonical = canonical_test_id(test_name)
    return metrics.get(canonical, {})


def _aggregate_metrics(
    test_names: list[str],
    metrics: dict,
    schema: list[tuple[str, str]],
) -> dict[str, list[float]]:
    """Collect numeric metric values across all test phases / iterations."""
    buckets: dict[str, list[float]] = {key: [] for key, _ in schema}
    for name in test_names:
        for metric_key, entries in _collect_scalar_metrics(_metrics_blob(metrics, name)).items():
            if metric_key not in buckets:
                continue
            for entry in entries:
                value = entry.get("value")
                if isinstance(value, (int, float)):
                    buckets[metric_key].append(float(value))
    return buckets


def _chain_title(module: str, param: str) -> str:
    if module == "test_fixed_trajectory":
        traj = re.search(r"traj(\w+)", param)
        traj_label = traj.group(1) if traj else "trajectory"
        sim = param.split("-", 1)[0]
        robots = re.search(r"rob#(\d+)", param)
        n_robots = robots.group(1) if robots else "?"
        return f"{traj_label} | {sim} | {n_robots} robot(s)"
    if module == "test_takeoff_hover_land":
        vel = re.search(r"v([\d.]+)", param)
        vel_label = f"{vel.group(1)} m/s" if vel else param
        sim = param.split("-", 1)[0]
        return f"takeoff-hover-land @ {vel_label} | {sim}"
    return param


def _metric_schema(module: str) -> list[tuple[str, str]]:
    if module in ("test_fixed_trajectory", "test_takeoff_hover_land"):
        if module == "test_takeoff_hover_land":
            return [m for m in FLIGHT_METRICS if m[0] != "trajectory_success"
                    and not m[0].startswith("cross_track")
                    and m[0] != "path_rmse_m"
                    and m[0] != "trajectory_execution_time_sim_s"]
        return FLIGHT_METRICS
    if module in ("test_liveliness", "test_sensors"):
        return LIVELINESS_METRICS
    return []


def _group_tests(
    metrics: dict,
    statuses: dict[str, str],
    durations: dict[str, float],
) -> dict[tuple[str, str], list[str]]:
    """Group full test names by (module, base_param_id) across stress iterations."""
    groups: dict[tuple[str, str], list[str]] = {}
    all_names = {canonical_test_id(name) for name in set(metrics) | set(statuses)}
    for name in sorted(all_names):
        module = _module_name(name)
        param = _base_param_id(_param_id(name))
        groups.setdefault((module, param), []).append(name)
    for names in groups.values():
        names.sort(key=lambda n: (
            int(ITER_RE.search(_param_id(n)).group(0).replace("-iter", ""))
            if ITER_RE.search(_param_id(n)) else 0,
            PHASE_ORDER.get(_phase_name(n), 99),
        ))
    return groups


def _iteration_count(test_names: list[str]) -> int:
    iters = set()
    for name in test_names:
        m = ITER_RE.search(_param_id(name))
        if m:
            iters.add(m.group(0))
    return len(iters) or 1


def _chain_status(test_names: list[str], statuses: dict[str, str]) -> str:
    n_iter = _iteration_count(test_names)
    if n_iter > 1:
        landing_phases = [n for n in test_names if _phase_name(n) in ("test_landing", "test_land")]
        check = landing_phases or test_names
        passed = sum(1 for n in check if statuses.get(n) == "PASSED")
        total = len(check)
        return f"{passed}/{total} flight cycles passed ({n_iter} iterations)"
    if any(statuses.get(n) == "FAILED" for n in test_names):
        return "FAILED"
    if test_names and all(statuses.get(n) == "PASSED" for n in test_names):
        return "PASSED"
    if any(statuses.get(n) == "SKIPPED" for n in test_names):
        return "SKIPPED"
    return "UNKNOWN"


def build_summary_lines(run_dir: Path) -> list[str]:
    metrics_path = run_dir / "metrics.json"
    results_path = run_dir / "results.xml"
    run_meta = classify_run(run_dir)
    statuses, durations = _parse_results_xml(results_path)
    metrics = _load_metrics(metrics_path)

    passed = sum(1 for s in statuses.values() if s == "PASSED")
    failed = sum(1 for s in statuses.values() if s == "FAILED")
    skipped = sum(1 for s in statuses.values() if s == "SKIPPED")
    total = len(statuses)

    lines = [
        "AirStack Test Run Summary",
        f"Run directory: {run_dir.name}",
        f"Overall: {passed} passed, {failed} failed, {skipped} skipped ({total} tests)",
        "",
    ]
    if run_meta.get("outcome") not in ("simulation", "non_simulation"):
        reason = run_meta.get("reason", run_meta.get("outcome", "unknown"))
        lines.extend([
            f"Run status: {run_meta.get('outcome', 'unknown')}",
            f"Simulation metrics are not comparable: {reason}.",
            "",
        ])

    groups = _group_tests(metrics, statuses, durations)
    if not groups:
        lines.append("No metrics or test results recorded for this run.")
        return lines

    for (module, param), test_names in sorted(groups.items()):
        title = _chain_title(module, param)
        chain_status = _chain_status(test_names, statuses)
        lines.append(f"── {title} ──")
        lines.append(f"Result: {chain_status}")
        lines.append("")

        schema = _metric_schema(module)
        aggregated = _aggregate_metrics(test_names, metrics, schema)
        n_iter = _iteration_count(test_names)
        emitted = False
        for metric_key, label in schema:
            values = aggregated.get(metric_key, [])
            if not values:
                continue
            unit = UNIT_OVERRIDES.get(
                metric_key,
                next(
                    (entry.get("unit", "")
                     for name in test_names
                     for entry in _collect_scalar_metrics(_metrics_blob(metrics, name)).get(
                         metric_key, [])),
                    "",
                ),
            )
            if n_iter > 1 or len(values) > 1:
                lines.append(f"{label}: {_format_aggregated(metric_key, values, unit)}")
            else:
                entry = {"value": values[-1], "unit": unit}
                lines.append(f"{label}: {_format_value(metric_key, entry)}")
            emitted = True

        if not emitted:
            lines.append("(no key metrics recorded)")

        if n_iter > 1:
            lines.append("")
            lines.append(f"Aggregated over {n_iter} stress iterations (mean ± stddev).")

        # Phase wall times help debugging without opening results.xml.
        phase_wall: dict[str, list[float]] = {}
        for name in test_names:
            phase = _phase_name(name)
            wall = durations.get(name)
            if wall is not None:
                phase_wall.setdefault(phase, []).append(wall)
        if phase_wall:
            lines.append("")
            lines.append("Phase wall times:")
            for phase, walls in sorted(phase_wall.items(), key=lambda x: PHASE_ORDER.get(x[0], 99)):
                if n_iter > 1 and len(walls) > 1:
                    mean = statistics.mean(walls)
                    std = statistics.pstdev(walls)
                    lines.append(f"  {phase}: {mean:.1f}s ± {std:.1f}s (n={len(walls)})")
                else:
                    status = statuses.get(
                        next((n for n in test_names if _phase_name(n) == phase), ""),
                        "?",
                    )
                    lines.append(f"  {phase}: {walls[0]:.1f}s ({status})")

        lines.append("")

    # Trim trailing blank line
    if lines and lines[-1] == "":
        lines.pop()
    return lines


def write_summary(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    out_path = run_dir / "summary.txt"
    lines = build_summary_lines(run_dir)
    out_path.write_text("\n".join(lines) + "\n")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate summary.txt for a test run")
    parser.add_argument("run_dir", type=Path, help="Path to tests/results/<timestamp>/")
    args = parser.parse_args()
    out = write_summary(args.run_dir)
    print(out.read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
