"""Reference-loop C06 admission dependency; never a live execution boundary."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Literal, Protocol

from .contracts import AdmissionGuard, DispatchContext, SafetyDecision
from .core_stop import CoreStopSupervisor


class SafetyDecisionProvider(Protocol):
    def decide(self, context: DispatchContext, *, now: float) -> SafetyDecision | None: ...


@dataclass
class CoreAdmission:
    """An explicitly supplied guard/provider pair; guard starts inhibited by default."""

    guard: AdmissionGuard
    provider: SafetyDecisionProvider
    evidence_kind: Literal["trusted_in_process", "synthetic_fixture"]
    clock: Callable[[], float] = field(default=time.monotonic)
    stop: CoreStopSupervisor = field(init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.guard, AdmissionGuard) or self.provider is None \
                or not callable(getattr(self.provider, "decide", None)) \
                or self.evidence_kind not in {"trusted_in_process", "synthetic_fixture"} \
                or not callable(self.clock):
            raise ValueError("invalid core admission dependency")
        self.stop = CoreStopSupervisor(self.guard)
