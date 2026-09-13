"""SCRUM-8 contract primitives; not wired into the legacy control loop.

Trusted, in-process callers only. This module neither evaluates physical safety
nor authenticates operators, stops actuators, or implements distributed admission.
See docs/scrum-8/interfaces.md for the full integration contract.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
import math
from threading import Lock
from uuid import uuid4


class Truth(str, Enum):
    TRUE = "TRUE"
    FALSE = "FALSE"
    UNKNOWN = "UNKNOWN"

    def negate(self) -> Truth:
        if self is Truth.UNKNOWN:
            return self
        return Truth.FALSE if self is Truth.TRUE else Truth.TRUE


def evidence_truth(value: Truth | None, *, fresh: bool, negated: bool = False) -> Truth:
    """Missing/stale evidence cannot establish either polarity of a fact (C02)."""
    truth = value if fresh and isinstance(value, Truth) else Truth.UNKNOWN
    return truth.negate() if negated else truth


@dataclass(frozen=True)
class CapabilityDeclaration:
    """Semantic support only; numeric feasibility and permission are separate."""

    embodiment_id: str
    revision: str
    operations: frozenset[str]
    resources: frozenset[str]
    available_resources: frozenset[str]
    limits_ref: str

    def __post_init__(self) -> None:
        for name in ("operations", "resources", "available_resources"):
            values = frozenset(getattr(self, name))
            if any(not isinstance(v, str) or not v.strip() for v in values):
                raise ValueError(f"invalid {name}")
            object.__setattr__(self, name, values)
        if not self.embodiment_id.strip() or not self.revision.strip():
            raise ValueError("embodiment identity and revision are required")
        if not self.available_resources <= self.resources:
            raise ValueError("available resources must be declared")

    def rejection_reasons(self, operation: str, resources: frozenset[str]) -> tuple[str, ...]:
        """No reasons means semantic support, never an authorization (C03)."""
        reasons = []
        if operation not in self.operations:
            reasons.append("unsupported_operation")
        if not resources <= self.resources:
            reasons.append("undeclared_resource")
        elif not resources <= self.available_resources:
            reasons.append("unavailable_resource")
        if not self.limits_ref.strip():
            reasons.append("unknown_limits")
        return tuple(reasons)


@dataclass(frozen=True)
class DispatchContext:
    """Immutable references to every checked payload; caller stores the payloads."""

    run_id: str
    task_revision: str
    plan_revision: str
    action_id: str
    dispatch_id: str
    action_digest: str
    state_revision: str
    capability_revision: str
    permission_revision: str
    approval_revision: str
    constraints_revision: str
    authority_epoch: str
    stop_generation: int

    def __post_init__(self) -> None:
        for field in fields(self):
            if field.name == "stop_generation":
                continue
            value = getattr(self, field.name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field.name} must identify an immutable payload")
        if type(self.stop_generation) is not int or self.stop_generation < 0:
            raise ValueError("invalid stop generation")


@dataclass(frozen=True)
class SafetyDecision:
    decision_id: str
    context: DispatchContext
    verdict: str
    issued_at: float
    expires_at: float

    def __post_init__(self) -> None:
        if not self.decision_id.strip():
            raise ValueError("decision ID is required")
        if self.verdict not in {"ALLOW", "DENY", "NEEDS_APPROVAL", "UNKNOWN"}:
            raise ValueError("unknown verdict")
        if not all(math.isfinite(t) for t in (self.issued_at, self.expires_at)):
            raise ValueError("decision clock must be finite")
        if not 0 <= self.issued_at < self.expires_at:
            raise ValueError("invalid validity interval")


class AdmissionGuard:
    """Single-process C06/C08 prototype, initially inhibited on every restart.

    consume() atomically reserves an admission in this process. The caller must
    serialize context updates with the actual execution boundary, persist its
    dispatch ledger, and implement adapter deduplication and stop generation
    checks. Returning None is NOT permission to bypass those remaining checks.
    """

    def __init__(self) -> None:
        self._lock = Lock()
        self._epoch = uuid4().hex
        self._stopped = True
        self._generation = 0
        self._used_decisions: set[str] = set()
        self._used_dispatches: set[tuple[str, str]] = set()

    @property
    def epoch(self) -> str:
        return self._epoch

    @property
    def generation(self) -> int:
        with self._lock:
            return self._generation

    def stop(self) -> int:
        """Inhibit new admissions; does not claim actuator stop acknowledgment."""
        with self._lock:
            self._stopped = True
            self._generation += 1
            return self._generation

    def reset(self, *, generation: int, authorized: bool,
              safe_confirmed: bool, evidence_ref: str) -> bool:
        """Inputs must come from trusted supervision and fresh adapter evidence."""
        with self._lock:
            if (generation != self._generation or authorized is not True
                    or safe_confirmed is not True or not evidence_ref.strip()):
                return False
            if self._stopped:
                # Reset must not revive permits issued while admission was closed.
                self._generation += 1
                self._stopped = False
            return True

    def consume(self, decision: SafetyDecision, current: DispatchContext,
                *, now: float) -> str | None:
        """Return a denial reason or atomically consume this one-use decision."""
        with self._lock:
            if self._stopped:
                return "stopped"
            if not math.isfinite(now) or not decision.issued_at <= now < decision.expires_at:
                return "outside_validity_window"
            if decision.verdict != "ALLOW":
                return "not_allowed"
            if current.authority_epoch != self._epoch:
                return "stale_authority_epoch"
            if current.stop_generation != self._generation:
                return "stale_stop_generation"
            if decision.context != current:
                return "stale_context"
            if decision.decision_id in self._used_decisions:
                return "decision_consumed"
            dispatch_key = (current.run_id, current.dispatch_id)
            if dispatch_key in self._used_dispatches:
                return "dispatch_consumed"
            self._used_decisions.add(decision.decision_id)
            self._used_dispatches.add(dispatch_key)
            return None
