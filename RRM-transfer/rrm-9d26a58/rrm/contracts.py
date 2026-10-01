"""SCRUM-8 contract primitives; only scoped admission subsets enter the core loop.

Trusted, in-process callers only. This module neither evaluates physical safety
nor authenticates operators, stops actuators, or implements distributed admission.
See docs/scrum-8/interfaces.md for the full integration contract.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from enum import Enum
import math
from threading import Lock
from typing import Literal, Protocol
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
class PermissionDeclaration:
    """Policy scope only; this is neither approval nor dispatch authorization."""

    authority_id: str
    revision: str
    task_id: str
    embodiment_id: str
    operations: frozenset[str]
    resources: frozenset[str]

    def __post_init__(self) -> None:
        for name in ("authority_id", "revision", "task_id", "embodiment_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        for name in ("operations", "resources"):
            raw_values = getattr(self, name)
            if isinstance(raw_values, (str, bytes)):
                raise ValueError(f"invalid permitted {name}")
            try:
                values = frozenset(raw_values)
            except TypeError as exc:
                raise ValueError(f"invalid permitted {name}") from exc
            if any(not isinstance(value, str) or not value.strip() for value in values):
                raise ValueError(f"invalid permitted {name}")
            object.__setattr__(self, name, values)

    def rejection_reasons(self, *, task_id: str, embodiment_id: str,
                          operation: str,
                          resources: frozenset[str]) -> tuple[str, ...]:
        """Return every mismatch; no reasons means policy scope only allows evaluation."""
        reasons = []
        if task_id != self.task_id:
            reasons.append("task_not_permitted")
        if embodiment_id != self.embodiment_id:
            reasons.append("embodiment_not_permitted")
        if operation not in self.operations:
            reasons.append("operation_not_permitted")
        if not resources <= self.resources:
            reasons.append("resource_not_permitted")
        return tuple(reasons)


@dataclass(frozen=True)
class ApprovalScope:
    """One run/task/ordered-plan/action scope, not a dispatch authorization."""

    run_id: str
    task_id: str
    task_revision: str
    task_digest: str
    plan_id: str
    plan_version: int
    plan_digest: str
    action_id: str
    action_digest: str

    def __post_init__(self) -> None:
        for field in fields(self):
            if field.name == "plan_version":
                continue
            value = getattr(self, field.name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field.name} is required")
        if type(self.plan_version) is not int or self.plan_version < 0:
            raise ValueError("invalid approval plan version")

    def as_record(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class ApprovalDecision:
    """Explicit approval state supplied by a caller; identity is not authenticated here."""

    decision_id: str
    approver_id: str
    revision: str
    evidence_kind: Literal["operator_assertion", "synthetic_fixture"]
    verdict: Literal["APPROVE", "DENY"]
    scope: ApprovalScope

    def __post_init__(self) -> None:
        for name in ("decision_id", "approver_id", "revision"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        if self.evidence_kind not in {"operator_assertion", "synthetic_fixture"}:
            raise ValueError("invalid approval evidence kind")
        if self.verdict not in {"APPROVE", "DENY"}:
            raise ValueError("invalid approval verdict")
        if not isinstance(self.scope, ApprovalScope):
            raise ValueError("approval scope is required")

    def rejection_reasons(self, expected: ApprovalScope) -> tuple[str, ...]:
        reasons = []
        if self.scope != expected:
            reasons.append("approval_scope_mismatch")
        if self.verdict != "APPROVE":
            reasons.append("approval_denied")
        return tuple(reasons)

    def as_record(self) -> dict:
        return asdict(self)


class ApprovalProvider(Protocol):
    def decide(self, scope: ApprovalScope) -> ApprovalDecision | None: ...


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


def admission_rejection_reason(decision: SafetyDecision, current: DispatchContext,
                               *, now: float, stopped: bool, authority_epoch: str,
                               stop_generation: int, decision_used: bool,
                               dispatch_used: bool) -> str | None:
    """Pure form of the guard's precedence, also used by offline replay."""
    if stopped:
        return "stopped"
    if not math.isfinite(now) or not decision.issued_at <= now < decision.expires_at:
        return "outside_validity_window"
    if decision.verdict != "ALLOW":
        return "not_allowed"
    if current.authority_epoch != authority_epoch:
        return "stale_authority_epoch"
    if current.stop_generation != stop_generation:
        return "stale_stop_generation"
    if decision.context != current:
        return "stale_context"
    if decision_used:
        return "decision_consumed"
    if dispatch_used:
        return "dispatch_consumed"
    return None


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
        reason, _ = self.consume_evidenced(decision, current, now=now)
        return reason

    def snapshot(self, *, decision_id: str, run_id: str,
                 dispatch_id: str) -> dict:
        """Evidence for a missing/invalid decision; no admission occurs."""
        with self._lock:
            return {
                "stopped": self._stopped,
                "authority_epoch": self._epoch,
                "stop_generation": self._generation,
                "decision_used": decision_id in self._used_decisions,
                "dispatch_used": (run_id, dispatch_id) in self._used_dispatches,
            }

    def consume_evidenced(self, decision: SafetyDecision, current: DispatchContext,
                         *, now: float) -> tuple[str | None, dict]:
        """Atomically consume once and return the guard state used for the decision."""
        with self._lock:
            evidence = {
                "stopped": self._stopped,
                "authority_epoch": self._epoch,
                "stop_generation": self._generation,
                "decision_used": decision.decision_id in self._used_decisions,
                "dispatch_used": (current.run_id, current.dispatch_id) in self._used_dispatches,
            }
            reason = admission_rejection_reason(
                decision, current, now=now, **evidence,
            )
            if reason is None:
                self._used_decisions.add(decision.decision_id)
                self._used_dispatches.add((current.run_id, current.dispatch_id))
            return reason, evidence
