"""Bounded opt-in loop observations; never changes simulation execution."""
from collections import deque
from contextlib import contextmanager
import copy
import hashlib
import math
from pathlib import Path
import time
import uuid


class LoopTiming:
    def __init__(self, monotonic=time.monotonic, thread_cpu=time.thread_time):
        self.monotonic, self.thread_cpu = monotonic, thread_cpu
        self.observer_id = uuid.uuid4().hex
        self.source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.sequence = 0
        self.current = self.completed = None
        self.slow_loops = deque(maxlen=32)
        self.slow_count = 0
        self.max_wall_s = 0.0
        self.max_between_loop_wall_s = 0.0
        self.max_phase_wall_s = {}
        self.max_subphase_wall_s = {}
        self.max_subphase_records = {}
        self.latest_sampling_completed = None
        self.sampling_loop_count = 0
        self.sampling_retention_enabled = True
        self.sampling_retention_error = None
        self.sampling_retention_error_count = 0
        self.subphase_count = self.subphase_error_count = 0
        self.subphases_enabled = True
        self.subphase_error = None

    def _point(self, physics):
        wall, cpu = self.monotonic(), self.thread_cpu()
        if not math.isfinite(wall) or not math.isfinite(cpu):
            raise ValueError("nonfinite loop observation clock")
        return {"monotonic_s": wall, "thread_cpu_s": cpu,
                "physics_observer_id": getattr(physics, "observer_id", None) if physics else None,
                "physics_callback_count": physics.count if physics else None,
                "last_physics_callback_monotonic_s": physics.last_receipt_wall_s if physics else None}

    def _subphase_failed(self, error):
        self.subphases_enabled = False
        self.subphase_error_count += 1
        self.subphase_error = str(error)

    def _start_subphase(self, name, physics):
        if not self.subphases_enabled:
            return None
        try:
            if self.current is None or self.current['active_subphase'] is not None:
                raise ValueError("subphase requires an active loop without nested spans")
            token = {"name": name, "loop_sequence": self.current['sequence'],
                     "complete": False, "start": self._point(physics)}
            self.current['active_subphase'] = token
            return token
        except Exception as error:
            self._subphase_failed(error)
            return None

    def _end_subphase(self, token, physics, operation_failed):
        if token is None:
            return
        try:
            if self.current is None or self.current['sequence'] != token['loop_sequence']:
                raise ValueError("subphase crossed loop boundary")
            end = self._point(physics)
            wall = end['monotonic_s'] - token['start']['monotonic_s']
            cpu = end['thread_cpu_s'] - token['start']['thread_cpu_s']
            if wall < 0 or cpu < 0:
                raise ValueError("subphase observation clock moved backwards")
            record = {**token, "complete": True, "end": end, "wall_s": wall,
                      "thread_cpu_s": cpu, "operation_failed": operation_failed}
            spans = self.current['subphases']
            spans.append(record)
            if len(spans) > 32:
                del spans[0]
                self.current['subphase_history_dropped'] += 1
            self.current['subphase_count'] += 1
            self.subphase_count += 1
            if token['name'] not in self.max_subphase_records or wall > self.max_subphase_wall_s[token['name']]:
                self.max_subphase_wall_s[token['name']] = wall
                self.max_subphase_records[token['name']] = {"observer_id": self.observer_id, **record}
        except Exception as error:
            self._subphase_failed(error)
        finally:
            if self.current is not None and self.current.get('active_subphase') is token:
                self.current['active_subphase'] = None

    @contextmanager
    def span(self, name, physics=None):
        token = self._start_subphase(name, physics)
        operation_failed = True
        try:
            yield
            operation_failed = False
        finally:
            self._end_subphase(token, physics, operation_failed)

    def note_sample_written(self, capture_id, record_sequence):
        """Contained metadata only, called after a successful physical record flush."""
        if not self.sampling_retention_enabled:
            return
        try:
            if self.current is None:
                raise ValueError("sampling record requires an active loop")
            previous = self.current['sampling_record']
            if previous and previous['capture_id'] != capture_id:
                raise ValueError("multiple capture identities in one sampling loop")
            self.current['sampling_record'] = {
                "capture_id": capture_id,
                "first_record_sequence": previous['first_record_sequence'] if previous else record_sequence,
                "last_record_sequence": record_sequence,
                "records": previous['records'] + 1 if previous else 1}
        except Exception as error:
            self.sampling_retention_enabled = False
            self.sampling_retention_error = str(error)
            self.sampling_retention_error_count += 1

    def begin(self, physics=None):
        start = self._point(physics)
        between = None
        if self.completed:
            previous_end = self.completed['phases'][-1]['end']
            between = {"wall_s": start['monotonic_s'] - previous_end['monotonic_s'],
                       "thread_cpu_s": start['thread_cpu_s'] - previous_end['thread_cpu_s'],
                       "before_monotonic_s": previous_end['monotonic_s'],
                       "after_monotonic_s": start['monotonic_s']}
            if between['wall_s'] < 0 or between['thread_cpu_s'] < 0:
                raise ValueError("loop observation clock moved backwards")
        self.current = {"sequence": self.sequence, "complete": False,
                        "start": start, "between_loops": between, "phases": [],
                        "subphases": [], "active_subphase": None,
                        "subphase_count": 0, "subphase_history_dropped": 0, "sampling_record": None}
        self.sequence += 1

    def mark(self, name, physics=None):
        end = self._point(physics)
        previous = self.current['phases'][-1]['end'] if self.current['phases'] else self.current['start']
        wall = end['monotonic_s'] - previous['monotonic_s']
        cpu = end['thread_cpu_s'] - previous['thread_cpu_s']
        if wall < 0 or cpu < 0:
            raise ValueError("loop observation clock moved backwards")
        self.current['phases'].append({"name": name, "start": previous, "end": end,
                                       "wall_s": wall, "thread_cpu_s": cpu})
        self.max_phase_wall_s[name] = max(self.max_phase_wall_s.get(name, 0.0), wall)

    def finish(self):
        if not self.current or not self.current['phases']:
            raise ValueError("loop observation missing phase boundaries")
        self.current['complete'] = True
        end = self.current['phases'][-1]['end']
        self.current['wall_s'] = end['monotonic_s'] - self.current['start']['monotonic_s']
        self.current['thread_cpu_s'] = end['thread_cpu_s'] - self.current['start']['thread_cpu_s']
        self.completed = self.current
        self.current = None
        if self.sampling_retention_enabled and self.completed['sampling_record']:
            self.latest_sampling_completed = self.completed
            self.sampling_loop_count += 1
        self.max_wall_s = max(self.max_wall_s, self.completed['wall_s'])
        between = self.completed['between_loops']
        between_wall = between['wall_s'] if between else 0.0
        self.max_between_loop_wall_s = max(self.max_between_loop_wall_s, between_wall)
        if self.completed['wall_s'] > .1 or between_wall > .1:
            self.slow_count += 1
            self.slow_loops.append(self.completed)

    def snapshot(self, *, since_sequence=None, sampling_capture_id=None, sampling_since_sequence=-1):
        return copy.deepcopy({"schema": "airstack-loop-timing/v1",
            "observer_id": self.observer_id, "source_sha256": self.source_sha256,
            "current_partial": self.current, "previous_completed": self.completed,
            "slow_loops": [r for r in self.slow_loops if since_sequence is None or r["sequence"] > since_sequence],
            "history_since_sequence": since_sequence, "slow_loop_count": self.slow_count,
            "slow_history_dropped": max(0, self.slow_count - len(self.slow_loops)),
            "sampling_scope": "current loop partial; successful sampling-loop completion retained separately from immediate prior engine loop",
            "slow_threshold_s": .1, "slow_history_capacity": 32,
            "max_loop_wall_s": self.max_wall_s,
            "max_between_loop_wall_s": self.max_between_loop_wall_s,
            "max_phase_wall_s": self.max_phase_wall_s,
            "max_subphase_wall_s": self.max_subphase_wall_s,
            "max_subphase_records": self.max_subphase_records,
            "latest_sampling_completed": self.latest_sampling_completed
                if self.latest_sampling_completed
                and (sampling_capture_id is None or self.latest_sampling_completed['sampling_record']['capture_id'] == sampling_capture_id)
                and self.latest_sampling_completed['sequence'] > sampling_since_sequence else None,
            "sampling_loop_count": self.sampling_loop_count,
            "sampling_retention_capacity": 1,
            "sampling_retention_scope": "latest successful sampling loop only; final tail requires a subsequent successful record; single-vehicle delivery qualified",
            "sampling_retention_enabled": self.sampling_retention_enabled,
            "sampling_retention_error": self.sampling_retention_error,
            "sampling_retention_error_count": self.sampling_retention_error_count,
            "sampling_history_since_sequence": sampling_since_sequence,
            "subphase_count": self.subphase_count,
            "subphases_enabled": self.subphases_enabled,
            "subphase_error_count": self.subphase_error_count,
            "subphase_error": self.subphase_error,
            "subphase_history_capacity_per_loop": 32,
            "subphase_scope": "separate non-nested operation brackets; maxima cover successfully measured completed spans, not total overhead",
            "current_record_visibility": "active loop_snapshot; subsequent encode/write/status completion is visible only in later completed-loop evidence",
            "scope": "world_step includes render/physics/bridge; thread CPU is current thread only"})
