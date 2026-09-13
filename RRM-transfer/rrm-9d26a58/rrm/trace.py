"""Append-only JSONL tracing. See docs/benchmarks.md §5."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Trace logging  (docs/benchmarks.md §5)
# ---------------------------------------------------------------------------

class Tracer:
    """Append-only JSONL. Every reported number must be reconstructible from this.

    Written before the first real run on purpose: traces collected without full
    context cannot be re-scored offline, and retrofitting means discarding every
    episode recorded before the change.
    """

    def __init__(self, path: Path | None, run_meta: dict[str, Any]) -> None:
        self._fh = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = path.open("w", encoding="utf-8")
            self.event("run_start", **run_meta)

    def event(self, kind: str, **fields: Any) -> None:
        if self._fh is None:
            return
        rec = {"wall": round(time.time(), 6), "kind": kind, **fields}
        self._fh.write(json.dumps(rec, default=str) + "\n")

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


