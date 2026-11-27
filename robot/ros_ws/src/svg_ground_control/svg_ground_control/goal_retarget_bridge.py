"""File protocol between the policy commander and the online goal-retargeting learner.

The learner (``scripts/goal_retarget_learner.py``) needs a CUDA GPU and the drone_soccer
``gpu`` environment, which has no rclpy, so it runs on the host beside the robot
container and the two talk through one run directory under the shared ``/bags`` mount:

    <run_dir>/session.json     commander settings the learner checks (written once)
    <run_dir>/ticks.jsonl      one JSON row per flown policy tick (commander appends)
    <run_dir>/policies/        policy_<n>.zip, one per learner update (learner writes)
    <run_dir>/updates.jsonl    the learner's diagnostics of every update
    <run_dir>/manifest.json    the newest iterate; the commander hot-swaps on change
    <run_dir>/commander.log    the commander's detailed log of segments and swaps
    <run_dir>/learner.log      the learner's detailed log of every pass and update

Ticks are grouped into segments: runs of consecutive policy ticks with no gap. A
segment ends when the policy is stopped or a tick is skipped for stale inputs, because
the learner compares each tick's ball plan with the real ball over the following ticks.
Each row also names the learner iterate that flew it: a learner batch holds only ticks
of the iterate it updates.

Both sides import this module, so it uses only the standard library and NumPy.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

# 2: session.json names the policy frame (`policy_frame`).
FORMAT_VERSION = 2
SESSION_FILE = 'session.json'
TICKS_FILE = 'ticks.jsonl'
MANIFEST_FILE = 'manifest.json'
UPDATES_FILE = 'updates.jsonl'
POLICIES_DIR = 'policies'
# Detailed logs; the consoles show one line per event.
COMMANDER_LOG_FILE = 'commander.log'
LEARNER_LOG_FILE = 'learner.log'

# Per-tick vector fields and their lengths, in row order.
TICK_VECTOR_FIELDS = {
    'goal_xy': 2,
    'drone_position': 3,
    'drone_velocity': 3,
    'drone_quat_wxyz': 4,
    'drone_angular_velocity_body': 3,
    'ball_position': 3,
    'ball_velocity': 3,
    'action': 3,
    'waypoint': 3,
}


def policy_file_name(iterate: int) -> str:
    """Archive name of learner iterate ``iterate`` inside ``policies/``."""
    return f'policy_{iterate:03d}.zip'


def write_json_atomic(path: Path, payload: dict) -> None:
    """Write JSON so a concurrent reader sees the old file or the new one, never a mix."""
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(json.dumps(payload, indent=2, default=json_default) + '\n')
    os.replace(temporary, path)


def json_default(value: Any) -> Any:
    """Serialise NumPy scalars and arrays."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'{type(value).__name__} is not JSON serialisable')


def _make_shared_dir(path: Path) -> None:
    """Create ``path`` writable by every user.

    The commander runs as root in the container and the learner as the host user, and
    both write into the run directory.
    """
    path.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(path, 0o777)
    except PermissionError:
        pass  # created earlier by another user; writable if that user made it so


class TickRecorder:
    """Commander side: creates the run directory and appends one row per flown tick."""

    def __init__(self, root: str | Path, run_name: str, session: dict) -> None:
        """Create ``root/run_name`` with its ``session.json`` and an empty tick log.

        Args:
            root: directory holding all runs (shared with the host).
            run_name: new run directory name; must not exist yet.
            session: commander settings recorded for the learner.
        """
        root = Path(root)
        _make_shared_dir(root)
        self.run_dir = root / run_name
        if self.run_dir.exists():
            raise FileExistsError(f'goal-retarget run directory exists: {self.run_dir}')
        _make_shared_dir(self.run_dir)
        write_json_atomic(self.run_dir / SESSION_FILE, {'format_version': FORMAT_VERSION, **session})
        # Line buffered: every row reaches the file when its newline is written.
        self._file = open(self.run_dir / TICKS_FILE, 'a', buffering=1)
        self.segment = -1
        self._next_tick: int | None = None
        self.rows_written = 0

    @property
    def segment_open(self) -> bool:
        """Whether the next row continues the current segment."""
        return self._next_tick is not None

    @property
    def next_position(self) -> tuple[int, int]:
        """``(segment, tick)`` the next recorded row will get."""
        if self._next_tick is None:
            return self.segment + 1, 0
        return self.segment, self._next_tick

    def end_segment(self) -> bool:
        """Close the current segment; returns whether one was open."""
        was_open = self.segment_open
        self._next_tick = None
        return was_open

    def record(self, time_s: float, policy_iterate: int, **vectors: np.ndarray) -> tuple[int, int]:
        """Append one tick; opens a new segment if none is open.

        Args:
            time_s: tick time (s).
            policy_iterate: learner iterate of the policy that acted (0 = the starting policy).
            **vectors: every field of `TICK_VECTOR_FIELDS`.

        Returns:
            ``(segment, tick)`` of the row, ``tick`` counted within the segment.
        """
        missing = TICK_VECTOR_FIELDS.keys() - vectors.keys()
        if missing:
            raise ValueError(f'tick row is missing {sorted(missing)}')
        if self._next_tick is None:
            self.segment += 1
            self._next_tick = 0
        row = {'segment': self.segment, 'tick': self._next_tick, 't': float(time_s),
               'policy': int(policy_iterate)}
        for name, size in TICK_VECTOR_FIELDS.items():
            values = np.asarray(vectors[name], dtype=float).reshape(-1)
            if values.size != size:
                raise ValueError(f'{name} must have {size} values, got {values.size}')
            row[name] = [round(float(value), 6) for value in values]
        self._file.write(json.dumps(row, separators=(',', ':')) + '\n')
        position = (self.segment, self._next_tick)
        self._next_tick += 1
        self.rows_written += 1
        return position

    def close(self) -> None:
        """Close the tick log."""
        self._file.close()


class TickLogReader:
    """Learner side: incrementally read complete rows appended to ``ticks.jsonl``."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._offset = 0
        self._partial = b''

    def poll(self) -> list[dict]:
        """Rows completed since the last call (a half-written last line waits)."""
        if not self.path.exists():
            return []
        with open(self.path, 'rb') as file:
            file.seek(self._offset)
            data = file.read()
        self._offset += len(data)
        lines = (self._partial + data).split(b'\n')
        self._partial = lines.pop()
        return [json.loads(line) for line in lines if line.strip()]


def rows_to_arrays(rows: list[dict]) -> dict[str, np.ndarray]:
    """Stack tick rows into ``(T, n)`` float arrays per vector field, plus ``t`` and ``policy``."""
    arrays = {name: np.array([row[name] for row in rows], dtype=float).reshape(len(rows), size)
              for name, size in TICK_VECTOR_FIELDS.items()}
    arrays['t'] = np.array([row['t'] for row in rows], dtype=float)
    arrays['policy'] = np.array([row['policy'] for row in rows], dtype=int)
    return arrays


@dataclass
class PolicyManifestWatcher:
    """Commander side: notice when the learner publishes a new iterate in ``manifest.json``."""

    run_dir: Path
    last_iterate: int = 0
    _signature: tuple[int, int] | None = field(default=None, repr=False)

    def poll(self) -> dict | None:
        """The manifest if it names a newer iterate than seen so far, else None.

        The returned dict gains ``policy_path``, the absolute archive path. A manifest
        that cannot be parsed is ignored and read again on its next change.
        """
        path = Path(self.run_dir) / MANIFEST_FILE
        try:
            stat = path.stat()
        except FileNotFoundError:
            return None
        signature = (stat.st_mtime_ns, stat.st_size)
        if signature == self._signature:
            return None
        try:
            manifest = json.loads(path.read_text())
            iterate = int(manifest['iterate'])
            relative_policy = str(manifest['policy'])
        except (OSError, ValueError, KeyError, TypeError):
            return None
        self._signature = signature
        if iterate <= self.last_iterate:
            return None
        self.last_iterate = iterate
        manifest['policy_path'] = str(Path(self.run_dir) / relative_policy)
        return manifest


class DetailLog:
    """Timestamped, line-buffered log file for the details kept off the console."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._file = open(self.path, 'a', buffering=1)

    def write(self, message: str) -> None:
        stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
        for line in message.splitlines() or ['']:
            self._file.write(f'{stamp} {line}\n')

    def close(self) -> None:
        self._file.close()


def format_update_summary(update: dict) -> str:
    """One line of the learner diagnostics that tell whether an update was healthy."""
    parts = []
    if 'rows_fit' in update and 'samples' in update:
        parts.append(f"fit {update['rows_fit']} of {update['samples']} accepted ticks")
    if 'loss_before' in update and 'loss_after' in update:
        parts.append(f"loss {update['loss_before']:.4f} -> {update['loss_after']:.4f}")
    if 'rms_action_change_m' in update:
        parts.append(f"batch action change {update['rms_action_change_m']:.3f} m RMS")
    # Medians of the goal step this batch (dg_curr), the shift pi_F already applied
    # (dg_cum) and the shift its labels use (dg).
    if 'median_current_offset_m' in update:
        parts.append(f"goal shift: step {update['median_current_offset_m']:.2f} m, "
                     f"before {update['median_accumulated_offset_m']:.2f} m, "
                     f"labels {update['median_applied_offset_m']:.2f} m (medians)")
    if 'flight_time_s' in update:
        parts.append(f"at {update['flight_time_s']:.0f} s of flight")
    return ' | '.join(parts)
