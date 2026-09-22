"""Combined machine-readable and tabular reports for WS2 campaigns."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from mission import SUCCESSES, EXCLUDED


REPORT_METRICS = (
    "mission_progress_percent",
    "minimum_obstacle_clearance_m",
    "planner_wall_duration_s",
    "path_length_m",
    "time_to_goal_s",
    "mission_duration_sim_s",
    "mean_speed_m_s",
    "stationary_fraction",
    "mean_obstacle_clearance_m",
)


def build_campaign_report(manifest: Mapping[str, Any]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    comparisons: list[dict[str, Any]] = []
    verdicts: Counter[str] = Counter()
    for entry in manifest.get("trials", []):
        pairs = entry.get("pairs")
        if isinstance(pairs, list):
            for pair in pairs:
                verdicts[str(pair.get("verdict"))] += 1
                clean = _row(entry, pair, "clean")
                perturbed = _row(entry, pair, "perturbed")
                rows.extend((clean, perturbed))
                comparisons.append(
                    {
                        "scenario_id": entry.get("scenario_id"),
                        "pair_id": pair.get("pair_id"),
                        "repetition": pair.get("repetition"),
                        "verdict": pair.get("verdict"),
                        "metric_delta_perturbed_minus_clean": {
                            name: _difference(
                                perturbed.get(name), clean.get(name)
                            )
                            for name in REPORT_METRICS
                        },
                    }
                )
        else:
            rows.append(_single_row(entry))

    outcomes = Counter(str(row.get("outcome")) for row in rows)
    evaluable = sum(
        count for outcome, count in outcomes.items() if outcome not in EXCLUDED
    )
    successes = sum(outcomes.get(outcome, 0) for outcome in SUCCESSES)
    collisions = outcomes.get("collision", 0)
    return {
        "schema_version": 1,
        "campaign_id": manifest.get("campaign_id"),
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace(
            "+00:00", "Z"
        ),
        "summary": {
            "trial_count": len(rows),
            "pair_count": len(comparisons),
            "evaluable_trial_count": evaluable,
            "infrastructure_error_count": outcomes.get("infrastructure_error", 0),
            "operator_stopped_count": outcomes.get("user_stopped", 0),
            "success_percent": _percent(successes, evaluable),
            "collision_percent": _percent(collisions, evaluable),
            "outcomes": dict(sorted(outcomes.items())),
            "pair_verdicts": dict(sorted(verdicts.items())),
        },
        "trials": rows,
        "comparisons": comparisons,
    }


def write_campaign_report(manifest: Mapping[str, Any], campaign_dir: Path) -> None:
    report = build_campaign_report(manifest)
    _atomic_text(
        campaign_dir / "report.json",
        json.dumps(report, indent=2, sort_keys=False) + "\n",
    )
    csv_path = campaign_dir / "trials.csv"
    temporary = csv_path.with_suffix(".csv.tmp")
    fields = (
        "scenario_id",
        "pair_id",
        "repetition",
        "role",
        "verdict",
        "outcome",
        *REPORT_METRICS,
        "result_dir",
    )
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["trials"])
    temporary.replace(csv_path)


def _row(
    entry: Mapping[str, Any], pair: Mapping[str, Any], role: str
) -> dict[str, Any]:
    trial = pair[role]
    metrics = trial.get("metrics", {})
    return {
        "scenario_id": entry.get("scenario_id"),
        "pair_id": pair.get("pair_id"),
        "repetition": pair.get("repetition"),
        "role": role,
        "verdict": pair.get("verdict"),
        "outcome": trial.get("outcome"),
        **{name: metrics.get(name) for name in REPORT_METRICS},
        "result_dir": trial.get("result_dir"),
    }


def _single_row(entry: Mapping[str, Any]) -> dict[str, Any]:
    metrics = entry.get("metrics", {})
    return {
        "scenario_id": entry.get("scenario_id"),
        "pair_id": None,
        "repetition": None,
        "role": "single",
        "verdict": None,
        "outcome": entry.get("outcome"),
        **{name: metrics.get(name) for name in REPORT_METRICS},
        "result_dir": entry.get("result_dir"),
    }


def _difference(perturbed: Any, clean: Any) -> float | None:
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        for value in (perturbed, clean)
    ):
        return None
    return float(perturbed) - float(clean)


def _percent(numerator: int, denominator: int) -> float | None:
    return None if denominator == 0 else 100.0 * numerator / denominator


def _atomic_text(path: Path, text: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)
