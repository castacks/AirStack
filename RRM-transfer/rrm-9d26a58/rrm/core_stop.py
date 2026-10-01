"""C08-shaped mock stop evidence; not a physical emergency-stop implementation."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from threading import Lock
from typing import Literal, Protocol
from uuid import uuid4

from .contracts import AdmissionGuard
from .trace import Tracer


@dataclass(frozen=True)
class StopStateEvidence:
    dispatch_id: str
    generation: int
    observed_t: int
    observed_at_monotonic: float
    motion_stopped: bool
    active_motion_command: bool
    safe_condition_met: bool
    controller_mode: str
    evidence_kind: Literal["synthetic_mock", "adapter_report"]

    def __post_init__(self) -> None:
        if not self.dispatch_id.strip() or not self.controller_mode.strip():
            raise ValueError("stop evidence identity is required")
        if type(self.generation) is not int or self.generation < 0 \
                or type(self.observed_t) is not int or self.observed_t < 0 \
                or not math.isfinite(self.observed_at_monotonic):
            raise ValueError("invalid stop evidence clock or generation")
        if any(type(value) is not bool for value in (
                self.motion_stopped, self.active_motion_command, self.safe_condition_met)):
            raise ValueError("invalid stop evidence truth value")
        if self.evidence_kind not in {"synthetic_mock", "adapter_report"}:
            raise ValueError("invalid stop evidence kind")


class StoppableWorld(Protocol):
    def cancel_dispatch(self, dispatch_id: str, generation: int) -> bool: ...
    def observe_safe_state(self, dispatch_id: str,
                           generation: int) -> StopStateEvidence | None: ...


@dataclass(frozen=True)
class StopOutcome:
    intervention_id: str
    dispatch_id: str
    reason: str
    generation: int
    cancel_acknowledged: bool
    safe_state: StopStateEvidence | None
    status: Literal["SAFE_CONFIRMED_MOCK", "SAFE_UNCONFIRMED"]
    trace_complete: bool


class CoreStopSupervisor:
    """Latch admission before mock cancellation; record each distinct C08 fact."""

    def __init__(self, guard: AdmissionGuard) -> None:
        self.guard = guard
        self._world: StoppableWorld | None = None
        self._tracer: Tracer | None = None
        self._lock = Lock()
        self._outcome: StopOutcome | None = None

    def bind(self, world: StoppableWorld, tracer: Tracer) -> None:
        with self._lock:
            if self._world is not None or self._tracer is not None:
                raise ValueError("stop supervisor is already bound to a run")
            self._world = world
            self._tracer = tracer

    @property
    def outcome(self) -> StopOutcome | None:
        with self._lock:
            return self._outcome

    def request_stop(self, *, dispatch_id: str, reason: str) -> StopOutcome:
        if not isinstance(dispatch_id, str) or not dispatch_id.strip() \
                or not isinstance(reason, str) or not reason.strip():
            raise ValueError("stop requires dispatch ID and reason")
        with self._lock:
            if self._world is None or self._tracer is None:
                raise RuntimeError("stop supervisor is not bound")
            if self._outcome is not None:
                return self._outcome
            intervention_id = uuid4().hex
            previous_generation = self.guard.generation
            generation = self.guard.stop()
            trace_complete = True

            def record(kind: str, **fields) -> None:
                nonlocal trace_complete
                try:
                    self._tracer.event(kind, intervention_id=intervention_id,
                                       dispatch_id=dispatch_id, generation=generation,
                                       **fields)
                except Exception:
                    trace_complete = False

            record("stop_request", reason=reason,
                   previous_generation=previous_generation)
            try:
                acknowledged = self._world.cancel_dispatch(dispatch_id, generation) is True
            except Exception:
                acknowledged = False
            record("stop_cancel", acknowledged=acknowledged)
            try:
                safe_state = self._world.observe_safe_state(dispatch_id, generation)
            except Exception:
                safe_state = None
            if not isinstance(safe_state, StopStateEvidence) \
                    or safe_state.dispatch_id != dispatch_id \
                    or safe_state.generation != generation:
                safe_state = None
            confirmed = bool(
                acknowledged and safe_state is not None
                and safe_state.motion_stopped
                and not safe_state.active_motion_command
                and safe_state.safe_condition_met
                and safe_state.evidence_kind == "synthetic_mock"
            )
            status = "SAFE_CONFIRMED_MOCK" if confirmed else "SAFE_UNCONFIRMED"
            record("stop_safe_state", evidence=(asdict(safe_state)
                                                if safe_state is not None else None),
                   status=status)
            self._outcome = StopOutcome(
                intervention_id, dispatch_id, reason, generation, acknowledged,
                safe_state, status, trace_complete,
            )
            return self._outcome
