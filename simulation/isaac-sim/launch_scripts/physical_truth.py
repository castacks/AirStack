"""Opt-in, observation-only rigid-body capture owned by the AirStack checkout."""
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
        self.source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

    def close(self, reason):
        if self.file is not None:
            self.file.close()
            self.file = None
        self.reason = reason

    def status(self, now):
        return {"schema": "airstack-physical-truth-status/v1",
                "receipt_monotonic_s": now, "wall_time_unix_s": time.time(),
                "capture_id": self.capture_id, "recording": self.file is not None,
                "records": self.records, "bytes": self.bytes, "reason": self.reason,
                "elapsed_wall_s": now - self.started if self.capture_id else None,
                "max_sampling_wall_s": self.max_sampling_wall_s,
                "last_error": self.last_error, "execution_dispatch": False}

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
                self.started, self.records, self.bytes = now, 0, 0
                self.max_sampling_wall_s = 0.0
                self.last_error = None
                self.file = (self.directory / (name + ".jsonl")).open("x")
                self.reason = "recording"
    def _publish_status(self, now):
        state = (self.reason, self.file is not None, self.last_error)
        if now < self.next_status and state == self.published_state:
            return
        self.next_status = now + 1.0
        self.published_state = state
        status_path = self.directory / "status.json"
        temporary = self.directory / "status.json.tmp"
        temporary.write_text(json.dumps(self.status(now), indent=2) + "\n")
        temporary.replace(status_path)
        self.status_error = None

    def sample(self, sim_time_s, vehicles, *, playing=True, now=None, clock_observation=None):
        """Called after a world step; errors disable observation, never the simulator."""
        now = time.monotonic() if now is None else now
        try:
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
                record = {"schema": "airstack-physical-truth/v1",
                          "capture_id": self.capture_id, "sequence": self.records,
                          "sim_time_s": sim_time_s, "receipt_monotonic_s": now,
                          "wall_time_unix_s": time.time(),
                          "recorder_sha256": self.source_sha256,
                          **body_record(vehicle, path),
                          "physics_clock": clock_observation.snapshot() if clock_observation else None,
                          "backend_clocks": backend_clocks(vehicle, self.backend_source_cache)}
                encoded = (json.dumps(record, allow_nan=False) + "\n").encode()
                if self.bytes + len(encoded) > self.max_bytes:
                    self.close("byte_limit")
                    break
                self.file.write(encoded.decode())
                self.file.flush()
                self.bytes += len(encoded)
                self.records += 1
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
                self._publish_status(now)
            except Exception as exc:
                # Unavailable storage cannot be made trustworthy; contain the
                # observer failure and require readers to check status freshness.
                message = str(exc)
                if message != self.status_error:
                    self.warn("[physical_truth] status unavailable: " + message)
                self.status_error = message
