"""Validation for one read-only, live Isaac observation.

The checks here deliberately prove provenance and transport freshness only. They do
not infer that a semantic entity is visible in an image or authorize execution.
"""
from __future__ import annotations

from datetime import datetime, timezone
import math


def validate_live_observation(metadata: dict, *, expected_camera_frame: str,
                              now: datetime | None = None,
                              max_capture_age_s: float = 15.0) -> dict:
    """Return a normalized observation or reject before a PSC job can be queued."""
    required = {
        "captured_at", "source_stamp_ns", "source_stamp_advanced", "frame_id",
        "sha256", "vehicle", "capture_mode",
    }
    if not isinstance(metadata, dict) or not required.issubset(metadata):
        raise ValueError("Live observation metadata is incomplete.")
    if metadata["capture_mode"] != "read_only":
        raise ValueError("Live observation must be read-only.")
    if not isinstance(expected_camera_frame, str) or not expected_camera_frame:
        raise ValueError("Expected camera frame is not configured.")
    if metadata["frame_id"] != expected_camera_frame:
        raise ValueError("Live camera frame does not match the reviewed scene binding.")
    if type(metadata["source_stamp_ns"]) is not int or metadata["source_stamp_ns"] <= 0:
        raise ValueError("Live camera timestamp is invalid.")
    if metadata["source_stamp_advanced"] is not True:
        raise ValueError("Live camera timestamp has not advanced; refresh after Isaac is running.")
    if not isinstance(metadata["sha256"], str) or len(metadata["sha256"]) != 64:
        raise ValueError("Live camera checksum is invalid.")
    try:
        captured_at = datetime.fromisoformat(metadata["captured_at"])
        now = now or datetime.now(timezone.utc)
        age_s = (now - captured_at).total_seconds()
    except (TypeError, ValueError):
        raise ValueError("Live capture time is invalid.") from None
    if captured_at.tzinfo is None or age_s < -1 or age_s > max_capture_age_s:
        raise ValueError("Live camera capture is stale.")
    vehicle = metadata["vehicle"]
    vehicle_required = {
        "odometry_frame_id", "odometry_child_frame_id",
        "odometry_stamp_ns", "x", "y", "z", "linear_speed_m_s",
    }
    if not isinstance(vehicle, dict) or not vehicle_required.issubset(vehicle):
        raise ValueError("Matched vehicle state is incomplete.")
    # connected may be None when MAVROS is not running (pure Isaac Sim);
    # connected=False is still rejected (means MAVROS is up but disconnected).
    if vehicle.get("connected") is False:
        raise ValueError("Vehicle is not connected.")
    if vehicle["odometry_frame_id"] != "map" or vehicle["odometry_child_frame_id"] != "base_link":
        raise ValueError("Vehicle odometry frame does not match map to base_link.")
    if type(vehicle["odometry_stamp_ns"]) is not int or vehicle["odometry_stamp_ns"] <= 0:
        raise ValueError("Vehicle odometry timestamp is invalid.")
    numeric = (vehicle["x"], vehicle["y"], vehicle["z"], vehicle["linear_speed_m_s"])
    if not all(type(value) in {int, float} and math.isfinite(value) for value in numeric):
        raise ValueError("Vehicle state contains non-finite values.")
    return {**metadata, "capture_age_s": round(age_s, 3)}
