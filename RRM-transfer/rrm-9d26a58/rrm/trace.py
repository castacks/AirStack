"""Append-only JSONL tracing. See docs/benchmarks.md §5."""

from __future__ import annotations

import json
import time
from pathlib import Path
from threading import Event, Lock
from typing import Any
from uuid import uuid4

# ---------------------------------------------------------------------------
# Trace logging  (docs/benchmarks.md §5)
# ---------------------------------------------------------------------------

TRACE_EVENT_SCHEMA = "rrm-trace-event/v2"


class Tracer:
    """Append-only JSONL. Every reported number must be reconstructible from this.

    Written before the first real run on purpose: traces collected without full
    context cannot be re-scored offline, and retrofitting means discarding every
    episode recorded before the change.
    """

    def __init__(self, path: Path | None, run_meta: dict[str, Any]) -> None:
        self._fh = None
        self._sequence = 0
        self._lock = Lock()
        selected = run_meta["run_id"] if "run_id" in run_meta else uuid4().hex
        if not isinstance(selected, str) or not selected.strip():
            raise ValueError("run_id must be a nonempty string")
        self.run_id = selected
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            # Benchmark evidence is immutable. Reusing a trace path would silently
            # remove a failed attempt from the denominator.
            self._fh = path.open("x", encoding="utf-8")
            self.event("run_start", **run_meta)

    def event(self, kind: str, **fields: Any) -> int | None:
        commit_event = fields.pop("_commit_event", None)
        if commit_event is not None and not isinstance(commit_event, Event):
            raise TypeError("_commit_event must be a threading.Event")
        if "run_id" in fields and fields["run_id"] != self.run_id:
            raise ValueError("event run_id does not match tracer")
        with self._lock:
            if self._fh is None:
                if commit_event is not None:
                    commit_event.set()
                return None
            sequence = self._sequence
            rec = {
                "schema_version": TRACE_EVENT_SCHEMA,
                "sequence": sequence,
                "wall_time_s": round(time.time(), 6),
                "monotonic_time_s": round(time.monotonic(), 6),
                "kind": kind,
                **fields,
                "run_id": self.run_id,
            }
            self._fh.write(json.dumps(rec, default=str) + "\n")
            self._fh.flush()
            self._sequence += 1
            if commit_event is not None:
                commit_event.set()
            return sequence

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                self._fh.close()
                self._fh = None
