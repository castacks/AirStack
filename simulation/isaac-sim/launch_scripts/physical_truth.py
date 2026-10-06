"""Opt-in, observation-only rigid-body capture owned by the AirStack checkout."""
from collections import deque
from contextlib import nullcontext
import hashlib
import json
import math
import inspect
import uuid
from pathlib import Path
import re
import time


def body_record(vehicle, path):
    """Read PhysX and retain vehicle.state physical input to simulated sensors."""
    dc = vehicle.get_dc_interface()
    body = dc.get_rigid_body(path.rstrip("/") + "/body")
    if not body:
        raise ValueError("rigid-body handle unavailable")
    pose = dc.get_rigid_body_pose(body)
    velocity = dc.get_rigid_body_linear_velocity(body)
    angular = dc.get_rigid_body_angular_velocity(body)
    return {"vehicle_path": path,
            "rigid_body_position_m": [float(getattr(pose.p, c)) for c in "xyz"],
            "rigid_body_quaternion_xyzw": [float(getattr(pose.r, c)) for c in "xyzw"],
            "rigid_body_linear_velocity_m_s": [float(getattr(velocity, c)) for c in "xyz"],
            "rigid_body_angular_velocity_rad_s": [float(getattr(angular, c)) for c in "xyz"],
            "sensor_state_position_m": [float(x) for x in vehicle.state.position],
            "sensor_state_quaternion_xyzw": [float(x) for x in vehicle.state.attitude]}


class ClockObservation:
    """Read engine callback dt without wrapping or writing to backend state."""
    callback_name = "rrm_physics_clock_observer"

    def __init__(self, warn=print):
        self.warn = warn
        self.world = None
        self.attached = False
        self.next_retry = 0.0
        self.error = None
        self.callback_max_wall_s = 0.0
        self._reset()

    def _reset(self):
        self.observer_id = uuid.uuid4().hex
        self.count = self.sum_dt_s = self.sum_truncated_us = 0
        self.min_dt_s = self.max_dt_s = self.last_dt_s = None
        self.callback_max_wall_s = 0.0
        self.last_receipt_wall_s = None
        self.callback_long_gaps = deque(maxlen=32)
        self.callback_long_gap_count = 0

    def bind(self, world):
        if world is not self.world:
            self.close()
            self.world = world
            self._reset()
            self.error = None
            self.next_retry = 0.0
        if self.attached or time.monotonic() < self.next_retry:
            return
        self.next_retry = time.monotonic() + 1.0
        try:
            bound_id = self.observer_id
            world.add_physics_callback(self.callback_name,
                lambda dt: self.on_step(dt) if self.observer_id == bound_id else None)
            self.attached = True
            self.error = None
        except Exception as exc:
            self._error(exc)

    def _error(self, exc):
        message = str(exc)
        if message != self.error:
            self.warn("[physical_clock] observation error: " + message)
        self.error = message

    def on_step(self, dt):
        started = time.monotonic()
        try:
            if type(dt) not in (float, int) or not math.isfinite(dt) or dt <= 0:
                raise ValueError("invalid actual physics callback dt")
            if self.last_receipt_wall_s is not None:
                gap = started - self.last_receipt_wall_s
                if gap > .1:
                    self.callback_long_gap_count += 1
                    self.callback_long_gaps.append({"before_monotonic_s": self.last_receipt_wall_s,
                        "after_monotonic_s": started, "wall_gap_s": gap,
                        "callback_count_after": self.count + 1, "callback_dt_s": dt})
            self.last_receipt_wall_s = started
            self.count += 1
            self.sum_dt_s += dt
            self.sum_truncated_us += int(dt * 1_000_000)
            self.last_dt_s = dt
            self.min_dt_s = dt if self.min_dt_s is None else min(self.min_dt_s, dt)
            self.max_dt_s = dt if self.max_dt_s is None else max(self.max_dt_s, dt)
        except Exception as exc:
            self._error(exc)
        finally:
            self.callback_max_wall_s = max(self.callback_max_wall_s, time.monotonic()-started)

    def snapshot(self):
        return {"observer_id": self.observer_id, "attached": self.attached,
                "observation_error": self.error, "callback_count": self.count,
                "sum_callback_dt_s": self.sum_dt_s,
                "sum_truncated_callback_dt_us": self.sum_truncated_us,
                "last_callback_dt_s": self.last_dt_s, "min_callback_dt_s": self.min_dt_s,
                "max_callback_dt_s": self.max_dt_s,
                "max_callback_wall_s": self.callback_max_wall_s,
                "last_callback_receipt_monotonic_s": self.last_receipt_wall_s,
                "long_callback_gaps": [dict(r) for r in self.callback_long_gaps],
                "long_callback_gap_history_dropped": max(0, self.callback_long_gap_count - len(self.callback_long_gaps)),
                "long_callback_gap_count": self.callback_long_gap_count,
                "long_callback_gap_threshold_s": .1,
                "long_callback_gap_history_capacity": 32,
                "phase": "engine callback totals read after world.step",
                "backend_argument_equivalence": "source-backed forwarding, not direct backend-call instrumentation"}

    def close(self):
        if self.world is not None and self.attached:
            try:
                self.world.remove_physics_callback(self.callback_name)
            except Exception as exc:
                self._error(exc)
        self.attached = False


def backend_clocks(vehicle, source_cache):
    result = []
    for index, backend in enumerate(getattr(vehicle, "_backends", [])):
        if not hasattr(backend, "_current_utime"):
            continue
        kind = type(backend)
        if kind not in source_cache:
            path = inspect.getsourcefile(kind)
            source_cache[kind] = {"source_path": path,
                "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest() if path else None}
        sensors = getattr(backend, "_sensor_data", None)
        result.append({"backend_index": index, "backend_instance_id": hex(id(backend)), "backend_class": kind.__module__+"."+kind.__qualname__,
                       "current_utime_us": backend._current_utime,
                       "utime_remainder_us": getattr(backend, "_utime_remainder_us", None),
                       "running": getattr(backend, "_is_running", None),
                       "received_first_heartbeat": getattr(backend, "_received_first_hearbeat", None),
                       "received_first_imu": getattr(sensors, "received_first_imu", None),
                       "new_imu_data": getattr(sensors, "new_imu_data", None),
                       "gate_phase": "after world.step; does not identify prior update branches",
                       **source_cache[kind]})
    return result


class PhysicalTruthCapture:
    def __init__(self, directory, warn=print, *, max_records=10_000,
                 max_bytes=64 * 1024 * 1024, max_duration_s=300):
        self.directory = Path(directory)
        self.warn = warn
        self.max_records, self.max_bytes = max_records, max_bytes
        self.max_duration_s = max_duration_s
        self.file = None
        self.capture_id = None
        self.seen = set()
        self.next_poll = self.next_sample = self.next_status = 0.0
        self.published_state = None
        self.status_error = None
        self.started = self.records = self.bytes = 0
        self.max_sampling_wall_s = 0.0
        self.reason = "idle"
        self.last_error = None
        self.backend_source_cache = {}
        self.loop_history_observer_id = None
        self.loop_history_cursor = -1
        self.sampling_delivery_observer_id = None
        self.sampling_delivery_cursor = -1
        self.last_written_sampling_loop = None
        self.source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

    def close(self, reason):
        if self.file is not None:
            self.file.close()
            self.file = None
        self.reason = reason

    def status(self, now, loop_observation=None):
        latest = loop_observation.latest_sampling_completed if loop_observation else None
        pending_completed = None
        if (latest and loop_observation.sampling_retention_enabled and latest["sampling_record"]["capture_id"] == self.capture_id
                and loop_observation.observer_id == self.sampling_delivery_observer_id
                and latest["sequence"] > self.sampling_delivery_cursor):
            pending_completed = latest["sequence"]
        return {"schema": "airstack-physical-truth-status/v1",
                "receipt_monotonic_s": now, "wall_time_unix_s": time.time(),
                "capture_id": self.capture_id, "recording": self.file is not None,
                "records": self.records, "bytes": self.bytes, "reason": self.reason,
                "elapsed_wall_s": now - self.started if self.capture_id else None,
                "max_sampling_wall_s": self.max_sampling_wall_s,
                "last_error": self.last_error, "execution_dispatch": False,
                "sampling_delivery": {
                    "observer_id": self.sampling_delivery_observer_id,
                    "delivered_through_loop_sequence": self.sampling_delivery_cursor,
                    "last_written_loop_sequence": self.last_written_sampling_loop,
                    "pending_written_loop_sequence": self.last_written_sampling_loop
                        if self.last_written_sampling_loop is not None
                        and self.last_written_sampling_loop > self.sampling_delivery_cursor else None,
                    "pending_completed_loop_sequence": pending_completed,
                    "retention_observer_id": loop_observation.observer_id if loop_observation else None,
                    "retention_available": loop_observation is not None,
                    "retention_enabled": loop_observation.sampling_retention_enabled if loop_observation else None,
                    "retention_error": loop_observation.sampling_retention_error if loop_observation else None,
                    "retention_error_count": loop_observation.sampling_retention_error_count if loop_observation else None,
                    "scope": "pending written loop may be incomplete; pending completed loop is known complete; no extra final-tail write"}}

    def _poll(self, now):
        if now < self.next_poll:
            return
        self.next_poll = now + 1.0
        request_path = self.directory / "capture.json"
        if not request_path.exists():
            self.close("request_absent")
        else:
            request = json.loads(request_path.read_text())
            name = request.get("capture_id")
            if (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name)
                    or type(request.get("enabled")) is not bool):
                raise ValueError("invalid capture request")
            if not request["enabled"]:
                self.close("request_disabled")
            elif name != self.capture_id:
                self.close("capture_replaced")
                if name in self.seen:
                    raise ValueError("capture IDs cannot be reused")
                self.seen.add(name)
                self.capture_id = name
                self.loop_history_cursor = -1
                self.sampling_delivery_observer_id = None
                self.sampling_delivery_cursor = -1
                self.last_written_sampling_loop = None
                self.started, self.records, self.bytes = now, 0, 0
                self.max_sampling_wall_s = 0.0
                self.last_error = None
                self.file = (self.directory / (name + ".jsonl")).open("x")
                self.reason = "recording"
    def _publish_status(self, now, loop_observation=None, clock_observation=None):
        state = (self.reason, self.file is not None, self.last_error)
        if now < self.next_status and state == self.published_state:
            return
        self.next_status = now + 1.0
        self.published_state = state
        status_path = self.directory / "status.json"
        temporary = self.directory / "status.json.tmp"
        with self._measure("status_build_encode", loop_observation, clock_observation):
            encoded = json.dumps(self.status(now, loop_observation), indent=2) + "\n"
        with self._measure("status_write_replace", loop_observation, clock_observation):
            temporary.write_text(encoded)
            temporary.replace(status_path)
        self.status_error = None

    @staticmethod
    def _measure(name, loop_observation, clock_observation):
        return loop_observation.span(name, clock_observation) if loop_observation else nullcontext()

    def sample(self, sim_time_s, vehicles, *, playing=True, now=None, clock_observation=None, loop_observation=None):
        """Called after a world step; errors disable observation, never the simulator."""
        now = time.monotonic() if now is None else now
        try:
            with self._measure("request_poll", loop_observation, clock_observation):
                self._poll(now)
            if self.file is None:
                return
            if now - self.started >= self.max_duration_s:
                self.close("duration_limit")
                return
            if not playing or now < self.next_sample:
                return
            self.next_sample = now + 0.1
            started = time.monotonic()
            for path, vehicle in vehicles.items():
                if self.records >= self.max_records:
                    self.close("record_limit")
                    break
                if not math.isfinite(sim_time_s):
                    raise ValueError("nonfinite simulation time")
                with self._measure("body_read", loop_observation, clock_observation):
                    body = body_record(vehicle, path)
                with self._measure("clock_snapshot", loop_observation, clock_observation):
                    physics_clock = clock_observation.snapshot() if clock_observation else None
                with self._measure("backend_metadata", loop_observation, clock_observation):
                    clocks = backend_clocks(vehicle, self.backend_source_cache)
                with self._measure("loop_snapshot", loop_observation, clock_observation):
                    timing = loop_observation.snapshot(
                        since_sequence=self.loop_history_cursor
                        if self.loop_history_observer_id == loop_observation.observer_id else -1,
                        sampling_capture_id=self.capture_id,
                        sampling_since_sequence=self.sampling_delivery_cursor
                        if self.sampling_delivery_observer_id == loop_observation.observer_id else -1
                    ) if loop_observation else None
                record = {"schema": "airstack-physical-truth/v1",
                          "capture_id": self.capture_id, "sequence": self.records,
                          "sim_time_s": sim_time_s, "receipt_monotonic_s": now,
                          "wall_time_unix_s": time.time(),
                          "recorder_sha256": self.source_sha256, **body,
                          "physics_clock": physics_clock, "backend_clocks": clocks,
                          "loop_timing": timing}
                with self._measure("record_encode", loop_observation, clock_observation):
                    encoded = (json.dumps(record, allow_nan=False) + "\n").encode()
                if self.bytes + len(encoded) > self.max_bytes:
                    self.close("byte_limit")
                    break
                with self._measure("record_write_flush", loop_observation, clock_observation):
                    self.file.write(encoded.decode())
                    self.file.flush()
                self.bytes += len(encoded)
                self.records += 1
                if loop_observation:
                    if self.sampling_delivery_observer_id != loop_observation.observer_id:
                        self.sampling_delivery_cursor = -1
                    self.sampling_delivery_observer_id = loop_observation.observer_id
                    self.last_written_sampling_loop = timing['current_partial']['sequence'] if timing['current_partial'] else None
                    delivered = timing['latest_sampling_completed']
                    if delivered:
                        self.sampling_delivery_cursor = delivered['sequence']
                    loop_observation.note_sample_written(self.capture_id, self.records - 1)
                if loop_observation and record['loop_timing']['previous_completed']:
                    self.loop_history_observer_id = loop_observation.observer_id
                    self.loop_history_cursor = record['loop_timing']['previous_completed']['sequence']
            self.max_sampling_wall_s = max(self.max_sampling_wall_s,
                                           time.monotonic() - started)
        except Exception as exc:
            message = str(exc)
            try:
                self.close("error")
            except OSError:
                self.file = None
            if message != self.last_error:
                self.warn("[physical_truth] capture disabled: " + message)
            self.last_error = message
        finally:
            try:
                self._publish_status(now, loop_observation, clock_observation)
            except Exception as exc:
                # Unavailable storage cannot be made trustworthy; contain the
                # observer failure and require readers to check status freshness.
                message = str(exc)
                if message != self.status_error:
                    self.warn("[physical_truth] status unavailable: " + message)
                self.status_error = message
