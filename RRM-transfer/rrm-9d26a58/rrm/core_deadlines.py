"""Bounded in-process callbacks; timeout does not terminate a Python thread."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from threading import Event, Lock, Thread
from typing import Callable, TypeVar

T = TypeVar("T")


class PendingActuation:
    """A callback remains pending until it returns, even if its caller timed out."""

    def __init__(self):
        self._lock = Lock()
        self._calls: set[Event] = set()

    def add(self, event: Event) -> None:
        with self._lock:
            self._calls.add(event)

    def remove(self, event: Event) -> None:
        with self._lock:
            self._calls.discard(event)

    def pending(self) -> bool:
        with self._lock:
            return any(not event.is_set() for event in self._calls)


@dataclass(frozen=True)
class CoreCallLimits:
    policy_s: float = 5.0
    observation_s: float = 2.0
    adapter_s: float = 2.0
    stop_s: float = 1.0
    evidence_s: float = 1.0

    def __post_init__(self) -> None:
        if any(type(value) not in {int, float} or not math.isfinite(value)
               or value <= 0 for value in asdict(self).values()):
            raise ValueError("call deadlines must be finite positive seconds")

    def record(self) -> dict:
        return {"version": "core-call-limits/v1", **asdict(self)}


class CoreCallTimeout(TimeoutError):
    def __init__(self, label: str, timeout_s: float, elapsed_s: float, completion: Event):
        super().__init__(f"{label} exceeded its callback deadline")
        self.label, self.timeout_s, self.elapsed_s = label, timeout_s, elapsed_s
        self.completion = completion

    @property
    def pending(self) -> bool:
        return not self.completion.is_set()

    def record(self) -> dict:
        return {"call": self.label, "timeout_s": self.timeout_s,
                "elapsed_s": self.elapsed_s, "operation_pending": self.pending}


def bounded_call(label: str, timeout_s: float, callback: Callable[[], T],
                 pending: PendingActuation | None = None) -> T:
    """Discard a late result; the callback may still be running after timeout."""
    completed = Event()
    result: list[T] = []
    errors: list[BaseException] = []
    finished_at: list[float] = []

    def invoke() -> None:
        try:
            result.append(callback())
        except BaseException as error:
            errors.append(error)
        finally:
            finished_at.append(time.monotonic())
            completed.set()
            if pending is not None:
                pending.remove(completed)

    started = time.monotonic()
    if pending is not None:
        pending.add(completed)
    try:
        Thread(target=invoke, name=f"rrm-{label}", daemon=True).start()
    except Exception:
        completed.set()
        if pending is not None:
            pending.remove(completed)
        raise
    if not completed.wait(max(0.0, started + timeout_s - time.monotonic())) \
            or finished_at[0] > started + timeout_s:
        raise CoreCallTimeout(label, timeout_s, time.monotonic() - started, completed)
    if errors:
        if isinstance(errors[0], Exception):
            raise errors[0]
        raise RuntimeError("callback terminated abnormally") from errors[0]
    return result[0]


class BoundedTracer:
    """Bound caller waits without granting a late writer any execution authority."""

    def __init__(self, tracer, timeout_s: float):
        self.tracer, self.timeout_s = tracer, timeout_s
        self.run_id = tracer.run_id

    def event(self, kind: str, **fields) -> None:
        bounded_call("trace_write", self.timeout_s,
                     lambda: self.tracer.event(kind, **fields))
