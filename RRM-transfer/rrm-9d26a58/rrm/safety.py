"""Both safety verifiers. See docs/architecture.md §3.5.

Safety #1 is symbolic and runs before the policy; Safety #2 is numeric and runs
on the emitted trajectory. Neither subsumes the other. Both may only reject."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Truth
from .schema import (
    AbstractAction, NumericLimitProfile, SafetyVerdict, Trajectory, Violation, WorldState,
)
from .verbs import _p, holds, predicate_truth, preconditions_of

# ---------------------------------------------------------------------------
# Safety verifier  (docs/architecture.md §3.5) — deterministic, may only reject
# ---------------------------------------------------------------------------

HUMAN_CLEARANCE_M = 0.75


def _distance_to_segment(point: tuple[float, float, float],
                         start: tuple[float, float, float],
                         end: tuple[float, float, float]) -> float:
    delta = tuple(b - a for a, b in zip(start, end))
    length_sq = sum(value * value for value in delta)
    fraction = min(1.0, max(0.0, sum((p - a) * d for p, a, d in zip(
        point, start, delta)) / length_sq)) if length_sq else 0.0
    nearest = tuple(a + fraction * d for a, d in zip(start, delta))
    return sum((p - n) ** 2 for p, n in zip(point, nearest)) ** 0.5


class SafetyVerifier:
    def verify(self, action: AbstractAction, ws: WorldState) -> SafetyVerdict:
        checks: list[str] = []
        violations: list[Violation] = []

        checks.append("target_exists")
        for t in action.targets:
            if ws.get(t) is None:
                violations.append(Violation(check="target_exists", severity="hard",
                                            detail=f"{t} not in world model"))

        checks.append("workspace")
        for t in action.targets:
            o = ws.get(t)
            if o and o.pose is not None and not holds(_p("reachable", t), ws):
                violations.append(Violation(check="workspace", severity="hard",
                                            detail=f"{t} outside reach envelope"))

        checks.append("human_proximity")
        for person in (o for o in ws.objects if o.cls == "person"):
            for t in action.targets:
                o = ws.get(t)
                if not (o and o.pose and person.pose):
                    continue
                dist = sum((a - b) ** 2 for a, b in zip(o.pose, person.pose)) ** 0.5
                if dist < HUMAN_CLEARANCE_M:
                    violations.append(Violation(
                        check="human_proximity", severity="hard",
                        detail=f"{person.id} within {dist:.2f}m of {t} "
                               f"(min {HUMAN_CLEARANCE_M}m)"))

        checks.append("preconditions")
        for pre in preconditions_of(action):
            truth = predicate_truth(pre, ws)
            if truth is Truth.TRUE:
                continue
            check = "precondition_unknown" if truth is Truth.UNKNOWN \
                else "precondition_false"
            label = "unknown" if truth is Truth.UNKNOWN else "unsatisfied"
            violations.append(Violation(check=check, severity="hard",
                                        detail=f"{label}: {pre}"))

        return SafetyVerdict(
            action_id=action.id,
            verdict="FAIL" if any(v.severity == "hard" for v in violations) else "PASS",
            violations=violations,
            checked=checks,
        )


# ---------------------------------------------------------------------------
# Safety #2 — numeric. Runs on trajectories, after the policy, before ROS 2.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class NumericSafetyVerifier:
    """Checks the policy's numeric output. Cannot run before a trajectory exists.

    Symbolic safety (SafetyVerifier) asks "is this action permitted?". This asks
    "are these typed targets within the resolved adapter envelope?". Both are
    required; neither subsumes the other.
    """

    profile: NumericLimitProfile

    def __post_init__(self) -> None:
        if not isinstance(self.profile, NumericLimitProfile):
            raise TypeError("numeric safety requires a resolved NumericLimitProfile")

    def verify(self, traj: Trajectory, ws: WorldState, *,
               expected_action_id: str | None = None) -> SafetyVerdict:
        checks = ["numeric_profile", "state_pose", "axis_limits", "velocity_limit"]
        violations: list[Violation] = []

        profile = self.profile
        names = tuple(axis.axis for axis in profile.axes)
        if (expected_action_id is not None and traj.action_id != expected_action_id) \
                or ws.robot.embodiment_id != profile.embodiment_id or traj.kind != profile.kind \
                or traj.frame != profile.frame or traj.axes != names \
                or traj.position_unit != profile.position_unit \
                or traj.velocity_unit != profile.velocity_unit:
            violations.append(Violation(
                check="numeric_profile", severity="hard",
                detail="trajectory action, kind, frame, units, axes, or embodiment mismatch",
            ))
            return SafetyVerdict(action_id=traj.action_id, verdict="FAIL",
                                 violations=violations, checked=checks)

        if traj.kind == "cartesian_position" \
                and not all(math.isfinite(value) for value in ws.robot.base_pose):
            violations.append(Violation(check="state_pose", severity="hard",
                                        detail="robot base pose nonfinite"))
            return SafetyVerdict(action_id=traj.action_id, verdict="FAIL",
                                 violations=violations, checked=checks)

        for wp in traj.waypoints:
            for axis, value in zip(profile.axes, wp):
                if not axis.minimum <= value <= axis.maximum:
                    violations.append(Violation(
                        check="axis_limits", severity="hard",
                        detail=f"{axis.axis} target {value:.2f} outside "
                               f"[{axis.minimum}, {axis.maximum}]"))

        if traj.max_velocity > profile.max_velocity:
            violations.append(Violation(
                check="velocity_limit", severity="hard",
                detail=f"{traj.max_velocity:.2f} exceeds {profile.max_velocity}"))

        # Humans are checked again here, against the actual path rather than the
        # symbolic action. TODO(phase-6): full swept-volume collision against
        # ws.objects once the trajectory carries link geometry.
        checks.append("human_clearance_path")
        for person in (o for o in ws.objects if o.cls == "person"):
            if person.pose is None or not all(math.isfinite(v) for v in person.pose):
                violations.append(Violation(check="human_clearance_path", severity="hard",
                                            detail=f"{person.id} pose unknown or nonfinite"))
                continue
            if traj.kind != "cartesian_position":
                violations.append(Violation(
                    check="human_clearance_path", severity="hard",
                    detail="joint trajectory lacks Cartesian swept-path evidence",
                ))
                break
            points = [ws.robot.base_pose, *traj.waypoints]
            for start, end in zip(points, points[1:]):
                dist = _distance_to_segment(person.pose, start, end)
                if dist < HUMAN_CLEARANCE_M:
                    violations.append(Violation(
                        check="human_clearance_path", severity="hard",
                        detail=f"path within {dist:.2f}m of {person.id}"))
                    break

        return SafetyVerdict(
            action_id=traj.action_id,
            verdict="FAIL" if any(v.severity == "hard" for v in violations) else "PASS",
            violations=violations,
            checked=checks,
        )
