"""Append-only JSONL tracing. See docs/benchmarks.md §5."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

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
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            # Benchmark evidence is immutable. Reusing a trace path would silently
            # remove a failed attempt from the denominator.
            self._fh = path.open("x", encoding="utf-8")
            self.event("run_start", **run_meta)

    def event(self, kind: str, **fields: Any) -> None:
        if self._fh is None:
            return
        rec = {
            "schema_version": TRACE_EVENT_SCHEMA,
            "sequence": self._sequence,
            "wall_time_s": round(time.time(), 6),
            "monotonic_time_s": round(time.monotonic(), 6),
            "kind": kind,
            **fields,
        }
        self._fh.write(json.dumps(rec, default=str) + "\n")
        self._fh.flush()
        self._sequence += 1

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
