"""Both safety verifiers. See docs/architecture.md §3.5.

Safety #1 is symbolic and runs before the policy; Safety #2 is numeric and runs
on the emitted trajectory. Neither subsumes the other. Both may only reject."""

from __future__ import annotations

from .schema import (
    AbstractAction, SafetyVerdict, Trajectory, Violation, WorldState,
)
from .verbs import _p, holds, preconditions_of

# ---------------------------------------------------------------------------
# Safety verifier  (docs/architecture.md §3.5) — deterministic, may only reject
# ---------------------------------------------------------------------------

HUMAN_CLEARANCE_M = 0.75


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
            if not holds(pre, ws):
                violations.append(Violation(check="preconditions", severity="hard",
                                            detail=f"unsatisfied: {pre}"))

        return SafetyVerdict(
            action_id=action.id,
            verdict="FAIL" if any(v.severity == "hard" for v in violations) else "PASS",
            violations=violations,
            checked=checks,
        )


# ---------------------------------------------------------------------------
# Safety #2 — numeric. Runs on trajectories, after the policy, before ROS 2.
# ---------------------------------------------------------------------------

JOINT_LIMITS = (-3.14, 3.14)
VELOCITY_LIMIT = 1.5


class NumericSafetyVerifier:
    """Checks the policy's numeric output. Cannot run before a trajectory exists.

    Symbolic safety (SafetyVerifier) asks "is this action permitted?". This asks
    "are these joint targets physically admissible?". Both are required; neither
    subsumes the other.
    """

    def verify(self, traj: Trajectory, ws: WorldState) -> SafetyVerdict:
        checks = ["joint_limits", "velocity_limit"]
        violations: list[Violation] = []

        for wp in traj.waypoints:
            for j, value in enumerate(wp):
                if not JOINT_LIMITS[0] <= value <= JOINT_LIMITS[1]:
                    violations.append(Violation(
                        check="joint_limits", severity="hard",
                        detail=f"joint {j} target {value:.2f} outside {JOINT_LIMITS}"))

        if traj.max_velocity > VELOCITY_LIMIT:
            violations.append(Violation(
                check="velocity_limit", severity="hard",
                detail=f"{traj.max_velocity:.2f} exceeds {VELOCITY_LIMIT}"))

        # Humans are checked again here, against the actual path rather than the
        # symbolic action. TODO(phase-6): full swept-volume collision against
        # ws.objects once the trajectory carries link geometry.
        checks.append("human_clearance_path")
        for person in (o for o in ws.objects if o.cls == "person"):
            if person.pose is None:
                continue
            for wp in traj.waypoints:
                dist = sum((a - b) ** 2 for a, b in zip(wp[:3], person.pose)) ** 0.5
                if dist < HUMAN_CLEARANCE_M:
                    violations.append(Violation(
                        check="human_clearance_path", severity="hard",
                        detail=f"waypoint within {dist:.2f}m of {person.id}"))
                    break

        return SafetyVerdict(
            action_id=traj.action_id,
            verdict="FAIL" if any(v.severity == "hard" for v in violations) else "PASS",
            violations=violations,
            checked=checks,
        )


