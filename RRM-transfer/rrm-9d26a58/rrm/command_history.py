"""Read-only aggregation of immutable command-mission evidence."""
from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import mean


def summarize_takeoffs(command_root: Path) -> dict:
    """Summarize takeoff repeatability without issuing or modifying any command."""
    attempts = []
    for outcome_path in sorted(command_root.glob(
            "*/command-mission-evidence/*-takeoff-*-outcome.json")):
        try:
            record = json.loads(outcome_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if record.get("kind") not in (None, "TAKEOFF"):
            continue
        metrics = dict(record.get("metrics") or {})
        pre = record.get("pre_odometry")
        post = record.get("post_odometry")
        if isinstance(pre, dict) and isinstance(post, dict):
            try:
                metrics.setdefault("horizontal_displacement_m", math.hypot(
                    float(post["x"]) - float(pre["x"]),
                    float(post["y"]) - float(pre["y"]),
                ))
                metrics.setdefault(
                    "vertical_displacement_m", float(post["z"]) - float(pre["z"])
                )
            except (KeyError, TypeError, ValueError):
                pass
        reasons = list(record.get("reasons") or [])
        diagnostics = list(record.get("diagnostics") or [])
        if "takeoff_horizontal_displacement_mismatch" in reasons:
            diagnostics.append("LATERAL_INSTABILITY_OBSERVED")
        if "takeoff_altitude_mismatch" in reasons:
            diagnostics.append("ALTITUDE_TARGET_MISMATCH")
        if record.get("action_success") is False:
            diagnostics.append("TASK_REPORTED_FAILURE")
        attempts.append({
            "request_id": outcome_path.parents[1].name,
            "action_id": record.get("action_id"),
            "verdict": record.get("verdict", "UNCONFIRMED"),
            "action_success": record.get("action_success"),
            "message": record.get("action_message", record.get("message", "")),
            "metrics": metrics,
            "diagnostics": sorted(set(diagnostics)),
            "evidence_path": str(outcome_path),
        })
    lateral = [
        item["metrics"]["horizontal_displacement_m"] for item in attempts
        if isinstance(item["metrics"].get("horizontal_displacement_m"), (int, float))
    ]
    verified = sum(item["verdict"] == "VERIFIED" for item in attempts)
    return {
        "schema_version": "rrm-takeoff-repeatability/v1",
        "attempt_count": len(attempts),
        "verified_count": verified,
        "verified_rate": verified / len(attempts) if attempts else None,
        "mean_horizontal_displacement_m": mean(lateral) if lateral else None,
        "max_horizontal_displacement_m": max(lateral) if lateral else None,
        "attempts": attempts,
        "execution_dispatch": False,
    }
