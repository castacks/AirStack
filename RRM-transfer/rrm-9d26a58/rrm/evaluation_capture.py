"""Read-only evaluation session policy; no dispatch or ROS dependencies."""
from __future__ import annotations
import math

GROUND_CHANNELS = ('state', 'landed', 'armed', 'authority', 'odom')


def grounded_snapshot(rows: dict, now: float, max_age_s: float = 2.0) -> bool:
    """Require current, mutually consistent ROS evidence and low odometry speed."""
    try:
        ages = [now - rows[c]['receipt_monotonic_s'] for c in GROUND_CHANNELS]
        if any(not math.isfinite(age) or not 0 <= age < max_age_s for age in ages):
            return False
        state = rows['state']['message']
        velocity = rows['odom']['message']['twist']['twist']['linear']
        speed2 = sum(float(velocity[k]) ** 2 for k in ('x', 'y', 'z'))
        return (state['connected'] is True and state['armed'] is False
                and rows['landed']['message']['landed_state'] == 1
                and rows['armed']['message']['data'] is False
                and rows['authority']['message']['data'] is False
                and math.isfinite(speed2) and speed2 < 0.01)
    except (KeyError, TypeError, ValueError):
        return False


class GroundCompletion:
    """Finish only by explicit request, inactive mission and distinct ground receipts."""
    def __init__(self):
        self.previous_receipt = None

    def update(self, *, requested: bool, mission_active: bool | None,
               rows: dict, now: float) -> bool:
        if not requested or mission_active is not False or not grounded_snapshot(rows, now):
            self.previous_receipt = None
            return False
        receipt = rows['state']['receipt_monotonic_s']
        if self.previous_receipt is not None and receipt > self.previous_receipt:
            return True
        self.previous_receipt = receipt
        return False


def physical_rollover(*, elapsed_s: float, bytes_written: int, landing_active: bool,
                      target_s: float = 120.0, target_bytes: int = 48 * 1024**2) -> bool:
    """Defer target rollover through landing, with margin below hard recorder caps."""
    target = elapsed_s >= target_s or bytes_written >= target_bytes
    if not target:
        return False
    return (not landing_active or elapsed_s >= 240.0
            or bytes_written >= 56 * 1024**2)


def landing_protection(mission: dict) -> bool:
    """Protect a reviewed LAND before feedback; unknown mission state is conservative."""
    if mission.get('active') is None:
        return True
    if mission.get('active') is not True:
        return False
    # Protect the entire active plan when it includes LAND, including before its
    # first feedback. This is intentionally broader than the current action.
    return any(action.get('kind') == 'LAND'
               for action in (mission.get('plan') or {}).get('actions', []))


def validate_control_finalization(return_code: int, summary: dict, source_hash: str) -> None:
    if (return_code != 0 or summary.get('stop_reason') != 'interrupted'
            or summary.get('events', 0) <= 0 or summary.get('missing_channels')
            or len(summary.get('channels', {})) != 18
            or summary.get('recorder_sha256') != source_hash):
        raise RuntimeError('Control finalization incomplete or failed')
