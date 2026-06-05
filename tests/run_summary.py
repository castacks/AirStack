"""Write a human-readable summary.txt for each test run.

Called automatically at pytest session end (see conftest.py). Users can also
regenerate manually:

    python3 tests/run_summary.py tests/results/<run-dir>/
"""
from __future__ import annotations

import argparse
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

PARAM_RE = re.compile(r"\[(.+)\]$")
ROBOT_METRIC_RE = re.compile(r"^robot_\d+\.(.+)$")

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


def _parse_results_xml(path: Path) -> tuple[dict[str, str], dict[str, float]]:
    """Return ({full_test_name: status}, {full_test_name: wall_time_s})."""
    if not path.exists():
        return {}, {}
    statuses: dict[str, str] = {}
    durations: dict[str, float] = {}
    for tc in ET.parse(path).iter("testcase"):
        full = f"{tc.get('classname')}.{tc.get('name')}"
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
    return json.loads(path.read_text())


def _param_id(test_name: str) -> str:
    m = PARAM_RE.search(test_name)
    return m.group(1) if m else test_name


def _module_name(test_name: str) -> str:
    return test_name.split(".", 1)[0]


def _phase_name(test_name: str) -> str:
    """test_fixed_trajectory.TestFixedTrajectory.test_takeoff[...] -> test_takeoff"""
    parts = test_name.split(".")
    if len(parts) >= 3:
        phase = parts[2]
        return phase.split("[", 1)[0]
    return test_name


def _format_value(key: str, entry: dict) -> str:
    value = entry.get("value")
    unit = UNIT_OVERRIDES.get(key, entry.get("unit", ""))
    if key == "trajectory_success":
        if value == 1.0:
            return "yes"
        if value == 0.0:
            return "no"
    if isinstance(value, (int, float)):
        text = f"{value:g}"
        return f"{text} {unit}".strip() if unit else text
    if value is None:
        return "n/a"
    return str(value)


def _collect_scalar_metrics(metrics_blob: dict) -> dict[str, dict]:
    """Flatten per-test metrics.json into {metric_key: entry}."""
    out: dict[str, dict] = {}
    for key, entry in metrics_blob.items():
        if not isinstance(entry, dict) or "value" not in entry:
            continue
        m = ROBOT_METRIC_RE.match(key)
        metric_key = m.group(1) if m else key
        out[metric_key] = entry
    return out


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
    """Group full test names by (module, param_id)."""
    groups: dict[tuple[str, str], list[str]] = {}
    all_names = set(metrics) | set(statuses)
    for name in sorted(all_names):
        module = _module_name(name)
        param = _param_id(name)
        groups.setdefault((module, param), []).append(name)
    for names in groups.values():
        names.sort(key=lambda n: PHASE_ORDER.get(_phase_name(n), 99))
    return groups


def _chain_status(test_names: list[str], statuses: dict[str, str]) -> str:
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

        combined: dict[str, dict] = {}
        for name in test_names:
            combined.update(_collect_scalar_metrics(metrics.get(name, {})))

        schema = _metric_schema(module)
        emitted = False
        for metric_key, label in schema:
            entry = combined.get(metric_key)
            if entry is None:
                continue
            lines.append(f"{label}: {_format_value(metric_key, entry)}")
            emitted = True

        if not emitted:
            lines.append("(no key metrics recorded)")

        # Phase wall times help debugging without opening results.xml.
        phase_times = []
        for name in test_names:
            phase = _phase_name(name)
            wall = durations.get(name)
            status = statuses.get(name, "?")
            if wall is not None:
                phase_times.append(f"  {phase}: {wall:.1f}s ({status})")
        if phase_times:
            lines.append("")
            lines.append("Phase wall times:")
            lines.extend(phase_times)

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
