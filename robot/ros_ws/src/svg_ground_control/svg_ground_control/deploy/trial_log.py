"""Append-only raw trial capture shared by simulated and physical flights.

A log is evidence for later SAFE scoring, not a score: contact and obstacle
geometry must come from the simulator or an independent real-world source.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


class TrialLog:
    def __init__(self, root: str, metadata: dict):
        base = Path(root).expanduser()
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        self.path = base / f'{stamp}_{uuid4().hex[:8]}.jsonl'
        self._stream = self.path.open('x', encoding='utf-8', buffering=1)
        self.write('start', metadata=metadata)

    def write(self, kind: str, *, time_s: float | None = None, **fields) -> None:
        record = {'kind': kind, **fields}
        if time_s is not None:
            record['time_s'] = float(time_s)
        self._stream.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')

    def close(self, reason: str, *, time_s: float | None = None) -> None:
        if self._stream.closed:
            return
        self.write('end', time_s=time_s, reason=reason)
        self._stream.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close('exception' if exc else 'closed')
