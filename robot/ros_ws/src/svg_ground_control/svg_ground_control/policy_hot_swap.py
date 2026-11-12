"""Load a new policy checkpoint off the control loop and hand it over between policy ticks.

Loading an SB3 archive takes long enough (hundreds of ms) to stall a 10 Hz control
timer, so `PolicyHotSwap.request` loads in a background thread and the control loop
calls `take_ready` at the start of a tick to adopt the newest loaded policy. A request
made while another is loading supersedes it: only the latest requested path is adopted.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class LoadedPolicy:
    """A policy loaded for a request, ready to be swapped in."""

    path: str
    policy: Any


class PolicyHotSwap:
    """Background policy loading with a lock-protected hand-over to the control loop."""

    def __init__(self, load: Callable[[str], Any]) -> None:
        """Remember how to load a checkpoint path into an object exposing ``predict``."""
        self._load = load
        self._lock = threading.Lock()
        self._latest_request = 0
        self._ready: LoadedPolicy | None = None
        self._errors: list[str] = []

    def request(self, path: str) -> threading.Thread:
        """Start loading ``path`` in a daemon thread; returns the thread (for tests)."""
        with self._lock:
            self._latest_request += 1
            request_id = self._latest_request
        thread = threading.Thread(target=self._load_in_background, args=(path, request_id), daemon=True)
        thread.start()
        return thread

    def take_ready(self) -> LoadedPolicy | None:
        """The newest loaded policy not yet taken, or None."""
        with self._lock:
            ready, self._ready = self._ready, None
        return ready

    def take_errors(self) -> list[str]:
        """Load failures since the last call, as messages."""
        with self._lock:
            errors, self._errors = self._errors, []
        return errors

    def _load_in_background(self, path: str, request_id: int) -> None:
        """Load one request; keep it only if no newer request was made meanwhile."""
        try:
            policy = self._load(path)
        except Exception as error:  # reported to the node, which keeps flying the current policy
            with self._lock:
                self._errors.append(f'{path}: {type(error).__name__}: {error}')
            return
        with self._lock:
            if request_id == self._latest_request:
                self._ready = LoadedPolicy(path, policy)
