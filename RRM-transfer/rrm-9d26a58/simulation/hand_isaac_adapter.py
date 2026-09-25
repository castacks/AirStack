"""Isolated, simulator-thread hand gateway; never starts Isaac or sends motion itself.

The caller must invoke tick()/sample() on the Isaac physics thread. This is an
integration prototype, not a qualified independent stop implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from threading import Event, Lock, RLock
import time
from typing import Callable

from rrm.hand_execution_boundary import DurableJournal, HandCommand, SafeStateEvidence


class GatewayError(RuntimeError):
    pass


@dataclass(frozen=True)
class _Pending:
    dispatch_id: str
    generation: int
    command: HandCommand


@dataclass(frozen=True)
class GatewayLivenessEvidence:
    episode_id: str
    generation: int
    observed_at_monotonic: float
    tick_count: int
    last_tick_started_at: float | None
    last_tick_completed_at: float | None
    last_tick_duration_s: float | None
    max_tick_duration_s: float
    heartbeat_age_s: float
    max_tick_gap_s: float
    stop_pending: bool
    stop_requested_at: float | None
    hold_applied_at: float | None
    stop_to_hold_s: float | None
    max_stop_to_hold_s: float
    motion_enabled: bool
    healthy: bool
    reason: str


class IsaacHandAdapter:
    """One-slot, fail-closed bridge to a passed-in Isaac articulation.

    `action_factory` is normally `isaacsim.core.utils.types.ArticulationAction`.
    Importing this module does not import Isaac, start a simulator, or command it.
    `motion_enabled` is a deliberate deployment gate, false by default. The caller
    owns authentication, scene reset, stepping, and gateway-thread liveness.
    """

    ARM_JOINTS = tuple(f"iiwa7_joint_{i}" for i in range(1, 8))

    def __init__(self, *, articulation: object, action_factory: Callable,
                 ledger_path: Path, episode_id: str, scene_recipe_sha256: str,
                 joint_names: tuple[str, ...], lower_rad: tuple[float, ...],
                 upper_rad: tuple[float, ...], max_velocity_rad_s: tuple[float, ...],
                 object_speeds: Callable[[], tuple[float, ...]],
                 controller_mode: Callable[[], str],
                 external_motion_active: Callable[[], bool],
                 motion_enabled: bool = False, max_delta_rad: float = 0.02,
                 observation_age_s: float = 0.25,
                 joint_safe_threshold_rad_s: float = 0.10,
                 object_safe_threshold_m_s: float = 0.01,
                 safe_window: int = 5, max_tick_gap_s: float = 0.10,
                 max_stop_to_hold_s: float = 0.10,
                 clock: Callable[[], float] = time.monotonic):
        if not episode_id or not scene_recipe_sha256 or len(joint_names) != 23 or \
                len(set(joint_names)) != 23 or tuple(joint_names[:7]) != self.ARM_JOINTS or \
                len(lower_rad) != 23 or len(upper_rad) != 23 or \
                len(max_velocity_rad_s) != 23 or \
                not all(math.isfinite(lo) and math.isfinite(hi) and lo < hi and
                        math.isfinite(vel) and vel > 0 for lo, hi, vel in
                        zip(lower_rad, upper_rad, max_velocity_rad_s)) or \
                not math.isfinite(max_delta_rad) or not 0 < max_delta_rad <= 0.02 or \
                not math.isfinite(observation_age_s) or observation_age_s <= 0 or \
                safe_window < 1 or \
                not isinstance(max_tick_gap_s, (int, float)) or isinstance(max_tick_gap_s, bool) or \
                not math.isfinite(max_tick_gap_s) or max_tick_gap_s <= 0 or \
                not isinstance(max_stop_to_hold_s, (int, float)) or \
                isinstance(max_stop_to_hold_s, bool) or \
                not math.isfinite(max_stop_to_hold_s) or max_stop_to_hold_s <= 0 or \
                not callable(clock):
            raise GatewayError("invalid_gateway_profile")
        self.articulation = articulation
        self.action_factory = action_factory
        self.object_speeds = object_speeds
        self.controller_mode = controller_mode
        self.external_motion_active = external_motion_active
        self.episode_id = episode_id
        self.scene_recipe_sha256 = scene_recipe_sha256
        self.joint_names, self.lower_rad, self.upper_rad = joint_names, lower_rad, upper_rad
        self.max_velocity_rad_s = max_velocity_rad_s
        self.max_delta_rad, self.observation_age_s = max_delta_rad, observation_age_s
        self.joint_safe_threshold_rad_s = joint_safe_threshold_rad_s
        self.object_safe_threshold_m_s, self.safe_window = object_safe_threshold_m_s, safe_window
        self.max_tick_gap_s = float(max_tick_gap_s)
        self.max_stop_to_hold_s = float(max_stop_to_hold_s)
        self._clock = clock
        self._created_at = float(clock())
        if not math.isfinite(self._created_at):
            raise GatewayError("invalid_gateway_clock")
        self._lock = RLock()
        self._heartbeat_lock = Lock()
        self._stop_lock = Lock()
        self._journal_lock = Lock()
        self._watchdog_fence = Event()
        self._watchdog_reason: str | None = None
        self._stop_fence = Event()
        self._async_stop_generation: int | None = None
        self._reconciled_stop_generation: int | None = None
        self._ledger = DurableJournal(ledger_path)
        self._enabled = bool(motion_enabled)
        self._generation = self._ledger.latest_generation
        self._pending: _Pending | None = None
        self._hold_pending = bool(self._ledger.outstanding_dispatch_ids)
        self._active = False
        self._safe_count = 0
        self._last_safe: SafeStateEvidence | None = None
        self._tick_count = 0
        self._last_tick_started_at: float | None = None
        self._last_tick_completed_at: float | None = None
        self._last_tick_duration_s: float | None = None
        self._max_tick_duration_s = 0.0
        self._stop_requested_at: float | None = None
        self._stop_pending = False
        self._hold_applied_at: float | None = None
        self._last_stop_to_hold_s: float | None = None
        self._liveness_fault: str | None = None
        self._seen: dict[str, str] = {}
        self._restart_dispatch_ids = set(self._ledger.outstanding_dispatch_ids)
        recovered_stop: dict | None = None
        if Path(ledger_path).exists():
            for line in Path(ledger_path).read_text().splitlines():
                record = json.loads(line)
                if record["event"] == "C09_ADAPTER_ENQUEUE":
                    payload = record["payload"]
                    self._seen[payload["dispatch_id"]] = payload["command_digest"]
                elif record["event"] == "C08_ADAPTER_LIVENESS_FAULT":
                    self._liveness_fault = record["payload"].get("reason", "LIVENESS_FAULT")
                elif record["event"] == "C08_ADAPTER_STOP_REQUEST":
                    recovered_stop = record["payload"]
                elif record["event"] == "C08_ADAPTER_HOLD_APPLIED":
                    payload = record["payload"]
                    if recovered_stop is not None and payload.get("generation") == \
                            recovered_stop.get("generation"):
                        recovered_stop = None
                    if payload.get("deadline_met") is False:
                        self._liveness_fault = "STOP_DEADLINE_EXCEEDED"
        if recovered_stop is not None:
            self._async_stop_generation = int(recovered_stop["generation"])
            self._stop_requested_at = float(recovered_stop["requested_at_monotonic"])
            self._stop_pending = True
            self._stop_fence.set()
            self._hold_pending = True
        if self._restart_dispatch_ids or recovered_stop is not None or \
                self._liveness_fault is not None:
            self._enabled = False  # Recovered dispatches or liveness faults keep motion closed.
        if self._liveness_fault is not None:
            self._set_watchdog_fence(self._liveness_fault)

    @property
    def motion_enabled(self) -> bool:
        return self._enabled and not self._watchdog_fence.is_set() and \
            not self._stop_fence.is_set()

    def _append_ledger(self, event: str, payload: dict) -> str:
        with self._journal_lock:
            return self._ledger.append(event, payload)

    def _append_motion_if_unfenced(self, event: str, payload: dict) -> bool:
        with self._journal_lock:
            if self._watchdog_fence.is_set() or self._stop_fence.is_set():
                return False
            self._ledger.append(event, payload)
            return True

    def _set_watchdog_fence(self, reason: str) -> None:
        with self._heartbeat_lock:
            if self._watchdog_reason is None:
                self._watchdog_reason = reason
            self._watchdog_fence.set()

    def _positions_velocities(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        positions = tuple(float(v) for v in self.articulation.get_joint_positions())
        velocities = tuple(float(v) for v in self.articulation.get_joint_velocities())
        if len(positions) != 23 or len(velocities) != 23 or \
                not all(math.isfinite(v) for v in positions + velocities):
            raise GatewayError("invalid_joint_observation")
        return positions, velocities

    def _validate(self, command: HandCommand, *, now: float) -> int:
        if command.operation != "ARM_JOINT_CALIBRATION" or \
                command.episode_id != self.episode_id or \
                command.scene_recipe_sha256 != self.scene_recipe_sha256 or \
                command.joint_name not in self.ARM_JOINTS:
            raise GatewayError("wrong_command_or_episode")
        if not all(math.isfinite(v) for v in (now, command.observed_at_monotonic,
                command.observed_position_rad, command.target_position_rad)) or \
                not 0 <= now - command.observed_at_monotonic <= self.observation_age_s:
            raise GatewayError("stale_command")
        positions, velocities = self._positions_velocities()
        index = self.joint_names.index(command.joint_name)
        if any(not lo <= pos <= hi for lo, pos, hi in
               zip(self.lower_rad, positions, self.upper_rad)) or \
                abs(positions[index] - command.observed_position_rad) > 1e-3 or \
                abs(command.target_position_rad - positions[index]) > self.max_delta_rad or \
                not self.lower_rad[index] <= command.target_position_rad <= self.upper_rad[index] or \
                any(abs(v) > limit for v, limit in zip(velocities, self.max_velocity_rad_s)):
            raise GatewayError("numeric_or_state_mismatch")
        return index

    def submit(self, dispatch_id: str, generation: int, command: HandCommand) -> None:
        """Durably reserve one dispatch; does not apply the action on this thread."""
        with self._lock:
            if not self.motion_enabled or self._pending is not None or self._active or \
                    self._hold_pending or generation < self._generation:
                raise GatewayError("gateway_inhibited_or_busy")
            if generation > self._generation:
                self._generation = generation
            if not dispatch_id or dispatch_id in self._seen:
                raise GatewayError("duplicate_dispatch_id")
            self._validate(command, now=self._clock())
            try:
                appended = self._append_motion_if_unfenced("C09_ADAPTER_ENQUEUE", {
                    "dispatch_id": dispatch_id,
                    "generation": generation, "episode_id": self.episode_id,
                    "command_digest": command.digest})
            except OSError as exc:
                self._enabled = False
                raise GatewayError("adapter_ledger_unavailable") from exc
            if not appended:
                raise GatewayError("gateway_inhibited_or_busy")
            self._seen[dispatch_id] = command.digest
            self._pending = _Pending(dispatch_id, generation, command)
            self._safe_count = 0
            self._last_safe = None

    def request_stop(self, generation: int, *, now: float | None = None) -> bool:
        """Fence new work immediately; acceptance is not physical stop proof."""
        now = self._clock() if now is None else now
        if type(generation) is not int or generation < 0 or \
                not isinstance(now, (int, float)) or isinstance(now, bool) or \
                not math.isfinite(now):
            return False
        with self._stop_lock:
            base_generation = self._generation if self._async_stop_generation is None else \
                max(self._generation, self._async_stop_generation)
            accepted_generation = max(base_generation + 1, generation)
            self._async_stop_generation = accepted_generation
            self._stop_requested_at = float(now)
            self._stop_pending = True
            self._hold_applied_at = None
            self._last_stop_to_hold_s = None
            self._stop_fence.set()
            try:
                self._append_ledger("C08_ADAPTER_STOP_REQUEST", {
                    "generation": accepted_generation,
                    "episode_id": self.episode_id, "requested_at_monotonic": now,
                    "max_stop_to_hold_s": self.max_stop_to_hold_s})
            except OSError:
                self._enabled = False
                return False
        return True

    def tick(self, *, now: float | None = None) -> str:
        """Call once on the physics thread before stepping the next frame."""
        started_at = self._clock() if now is None else now
        if not isinstance(started_at, (int, float)) or isinstance(started_at, bool) or \
                not math.isfinite(started_at):
            raise GatewayError("invalid_gateway_clock")
        with self._lock:
            try:
                with self._heartbeat_lock:
                    self._last_tick_started_at = float(started_at)
                result = self._tick_locked(now=float(started_at))
            except Exception:
                self._complete_tick(float(started_at), "ERROR")
                raise
            return self._complete_tick(float(started_at), result)

    def _tick_locked(self, *, now: float) -> str:
        self._reconcile_stop_locked()
        if self._hold_pending:
            positions, _ = self._positions_velocities()
            action = self.action_factory(joint_positions=positions[:7],
                joint_velocities=(0.0,) * 7, joint_indices=tuple(range(7)))
            self.articulation.apply_action(action)
            self._hold_pending = False
            self._active = False
            return "HOLD_APPLIED"
        if self._watchdog_fence.is_set() or self._stop_fence.is_set():
            self._pending = None
            return "INHIBITED"
        pending = self._pending
        if pending is None:
            return "IDLE"
        if not self._enabled or pending.generation != self._generation:
            self._pending = None
            return "INHIBITED"
        try:
            index = self._validate(pending.command, now=now)
        except GatewayError:
            self._pending = None
            self._enabled = False
            self._hold_pending = True
            raise
        try:
            appended = self._append_motion_if_unfenced("C09_ADAPTER_APPLY_INTENT", {
                "dispatch_id": pending.dispatch_id,
                "generation": pending.generation, "episode_id": self.episode_id})
        except OSError as exc:
            self._enabled = False
            self._pending = None
            raise GatewayError("adapter_ledger_unavailable") from exc
        self._pending = None
        if not appended or self._watchdog_fence.is_set() or self._stop_fence.is_set():
            self._hold_pending = True
            return "INHIBITED"
        self._active = True
        action = self.action_factory(joint_positions=(pending.command.target_position_rad,),
                                     joint_indices=(index,))
        try:
            self.articulation.apply_action(action)
        except Exception as exc:
            self._enabled = False
            self._hold_pending = True
            raise GatewayError("apply_uncertain_hold_required") from exc
        return "TARGET_APPLIED"

    def _complete_tick(self, started_at: float, result: str) -> str:
        try:
            completed_at = float(self._clock())
        except (TypeError, ValueError) as exc:
            self._latch_liveness_fault_locked("CLOCK_REGRESSION", started_at)
            raise GatewayError("invalid_gateway_clock") from exc
        if not math.isfinite(completed_at) or completed_at < started_at:
            self._latch_liveness_fault_locked("CLOCK_REGRESSION", started_at)
            raise GatewayError("invalid_gateway_clock")
        duration = completed_at - started_at
        with self._heartbeat_lock:
            self._tick_count += 1
            self._last_tick_completed_at = completed_at
            self._last_tick_duration_s = duration
            self._max_tick_duration_s = max(self._max_tick_duration_s, duration)
        stop_reconciled = self._reconcile_stop_locked()
        with self._stop_lock:
            stop_requested_at = self._stop_requested_at
        if result == "HOLD_APPLIED" and stop_requested_at is not None:
            stop_to_hold_s = completed_at - stop_requested_at
            deadline_met = stop_to_hold_s <= self.max_stop_to_hold_s
            try:
                self._append_ledger("C08_ADAPTER_HOLD_APPLIED", {
                    "generation": self._generation, "episode_id": self.episode_id,
                    "stop_requested_at_monotonic": stop_requested_at,
                    "hold_applied_at_monotonic": completed_at,
                    "stop_to_hold_s": stop_to_hold_s,
                    "max_stop_to_hold_s": self.max_stop_to_hold_s,
                    "deadline_met": deadline_met})
            except OSError as exc:
                self._enabled = False
                raise GatewayError("adapter_ledger_unavailable") from exc
            with self._stop_lock:
                self._hold_applied_at = completed_at
                self._last_stop_to_hold_s = stop_to_hold_s
                self._stop_pending = False
                self._async_stop_generation = None
                self._reconciled_stop_generation = None
                self._stop_fence.clear()
            if not deadline_met:
                self._enabled = False
                self._liveness_fault = "STOP_DEADLINE_EXCEEDED"
                self._set_watchdog_fence(self._liveness_fault)
        with self._heartbeat_lock:
            asynchronous_reason = self._watchdog_reason
        if asynchronous_reason is not None and self._liveness_fault is None:
            self._latch_liveness_fault_locked(asynchronous_reason, completed_at,
                                              heartbeat_age_s=duration)
            raise GatewayError("watchdog_fenced")
        if duration > self.max_tick_gap_s and self._liveness_fault is None:
            self._latch_liveness_fault_locked("TICK_DEADLINE_EXCEEDED", completed_at,
                                              heartbeat_age_s=0.0)
            raise GatewayError("tick_deadline_exceeded")
        if stop_reconciled and result != "HOLD_APPLIED":
            raise GatewayError("stop_fenced")
        return result

    def _reconcile_stop_locked(self) -> bool:
        """Consume async stop state while holding the simulator/action lock."""
        if not self._stop_fence.is_set():
            return False
        with self._stop_lock:
            stop_generation = self._async_stop_generation
            if stop_generation is None or stop_generation == self._reconciled_stop_generation:
                return False
            self._generation = max(self._generation, stop_generation)
            self._reconciled_stop_generation = stop_generation
        self._pending = None
        self._hold_pending = True
        self._safe_count = 0
        self._last_safe = None
        return True

    def _latch_liveness_fault_locked(self, reason: str, observed_at: float, *,
                                     heartbeat_age_s: float | None = None) -> None:
        """Fence and persist the first liveness fault; never touches the articulation."""
        first_fault = self._liveness_fault is None
        if first_fault:
            self._liveness_fault = reason
        self._set_watchdog_fence(reason)
        self._enabled = False
        possible_motion = self._pending is not None or self._active or self._stop_pending
        self._pending = None
        if possible_motion:
            self._hold_pending = True
        self._safe_count = 0
        self._last_safe = None
        if not first_fault:
            return
        try:
            self._append_ledger("C08_ADAPTER_LIVENESS_FAULT", {
                "generation": self._generation, "episode_id": self.episode_id,
                "reason": reason, "observed_at_monotonic": observed_at,
                "heartbeat_age_s": heartbeat_age_s,
                "last_tick_duration_s": self._last_tick_duration_s,
                "max_tick_gap_s": self.max_tick_gap_s,
                "stop_requested_at_monotonic": self._stop_requested_at,
                "max_stop_to_hold_s": self.max_stop_to_hold_s})
        except OSError:
            pass

    def _liveness_locked(self, now: float) -> GatewayLivenessEvidence:
        with self._heartbeat_lock:
            tick_count = self._tick_count
            last_tick_started_at = self._last_tick_started_at
            last_tick_completed_at = self._last_tick_completed_at
            last_tick_duration_s = self._last_tick_duration_s
            max_tick_duration_s = self._max_tick_duration_s
            asynchronous_reason = self._watchdog_reason
        with self._stop_lock:
            stop_generation = self._async_stop_generation
            stop_pending = self._stop_pending
            stop_requested_at = self._stop_requested_at
            hold_applied_at = self._hold_applied_at
            last_stop_to_hold_s = self._last_stop_to_hold_s
        generation = self._generation if stop_generation is None else \
            max(self._generation, stop_generation)
        heartbeat_origin = last_tick_completed_at
        heartbeat_age = now - (self._created_at if heartbeat_origin is None else heartbeat_origin)
        reason = self._liveness_fault or asynchronous_reason
        recorded_times = [self._created_at]
        recorded_times.extend(value for value in
            (last_tick_started_at, last_tick_completed_at, stop_requested_at)
            if value is not None)
        if reason is None and any(now < value for value in recorded_times):
            reason = "CLOCK_REGRESSION"
        if reason is None and stop_pending and stop_requested_at is not None and \
                now - stop_requested_at > self.max_stop_to_hold_s:
            reason = "STOP_DEADLINE_EXCEEDED"
        if reason is None and heartbeat_age > self.max_tick_gap_s:
            reason = "TICK_STALE"
        if reason is None:
            reason = "HEALTHY"
        return GatewayLivenessEvidence(self.episode_id, generation, now,
            tick_count, last_tick_started_at, last_tick_completed_at,
            last_tick_duration_s, max_tick_duration_s, heartbeat_age,
            self.max_tick_gap_s, stop_pending,
            stop_requested_at, hold_applied_at, last_stop_to_hold_s,
            self.max_stop_to_hold_s, self.motion_enabled, reason == "HEALTHY", reason)

    def watchdog_fence(self, *, now: float | None = None) -> str:
        """Trip a logical fence without waiting for the simulator/action lock.

        This path never calls the articulation or journal. The simulator thread
        durably reconciles the fence when its current tick returns.
        """
        now = self._clock() if now is None else now
        if not isinstance(now, (int, float)) or isinstance(now, bool) or not math.isfinite(now):
            raise GatewayError("invalid_gateway_clock")
        with self._heartbeat_lock:
            if self._watchdog_reason is not None:
                return self._watchdog_reason
            heartbeat_origin = self._last_tick_completed_at
            origin = self._created_at if heartbeat_origin is None else heartbeat_origin
            recorded_times = [self._created_at]
            recorded_times.extend(value for value in
                (self._last_tick_started_at, self._last_tick_completed_at)
                if value is not None)
            if any(now < value for value in recorded_times):
                reason = "CLOCK_REGRESSION"
            elif now - origin > self.max_tick_gap_s:
                reason = "TICK_STALE"
            else:
                return "HEALTHY"
            self._watchdog_reason = reason
            self._watchdog_fence.set()
            return reason

    def liveness(self, *, now: float | None = None) -> GatewayLivenessEvidence:
        now = self._clock() if now is None else now
        if not isinstance(now, (int, float)) or isinstance(now, bool) or not math.isfinite(now):
            raise GatewayError("invalid_gateway_clock")
        with self._lock:
            return self._liveness_locked(float(now))

    def watchdog_check(self, *, now: float | None = None) -> GatewayLivenessEvidence:
        """Fence on stale tick/stop timing; never calls the articulation."""
        now = self._clock() if now is None else now
        if not isinstance(now, (int, float)) or isinstance(now, bool) or not math.isfinite(now):
            raise GatewayError("invalid_gateway_clock")
        with self._lock:
            evidence = self._liveness_locked(float(now))
            if evidence.healthy:
                return evidence
            self._latch_liveness_fault_locked(evidence.reason, float(now),
                                               heartbeat_age_s=evidence.heartbeat_age_s)
            return self._liveness_locked(float(now))

    def sample(self, generation: int, *, now: float | None = None) -> SafeStateEvidence | None:
        """Sample measured safe state after a physics step; no stepping occurs here."""
        now = self._clock() if now is None else now
        with self._lock:
            positions, velocities = self._positions_velocities()
            if any(not lo <= pos <= hi for lo, pos, hi in
                   zip(self.lower_rad, positions, self.upper_rad)) or \
                    any(abs(vel) > limit for vel, limit in
                        zip(velocities, self.max_velocity_rad_s)):
                self._enabled = False
                self._pending = None
                self._hold_pending = True
                self._safe_count = 0
                self._last_safe = None
                return None
            if generation != self._generation or self._pending or self._hold_pending or self._active:
                self._safe_count = 0
                self._last_safe = None
                return None
            speeds = tuple(float(v) for v in self.object_speeds())
            if not speeds or not all(math.isfinite(v) and v >= 0 for v in speeds):
                self._safe_count = 0
                self._last_safe = None
                return None
            joint_speed, object_speed = max(abs(v) for v in velocities), max(speeds)
            if self.controller_mode() == "POSITION_HOLD" and \
                    self.external_motion_active() is False and \
                    joint_speed <= self.joint_safe_threshold_rad_s and \
                    object_speed <= self.object_safe_threshold_m_s:
                self._safe_count += 1
            else:
                self._safe_count = 0
                self._last_safe = None
            if self._safe_count < self.safe_window:
                return None
            payload = f"{self.episode_id}:{generation}:{now}:{joint_speed}:{object_speed}:{self._safe_count}"
            evidence = SafeStateEvidence(hashlib.sha256(payload.encode()).hexdigest(),
                self.episode_id, generation, now, True, "POSITION_HOLD", False,
                joint_speed, object_speed, self._safe_count)
            if self._restart_dispatch_ids:
                dispatch_ids = sorted(self._restart_dispatch_ids)
                try:
                    self._append_ledger("C08_ADAPTER_SAFE_RECONCILED", {
                        "generation": generation, "episode_id": self.episode_id,
                        "dispatch_ids": dispatch_ids, "evidence_ref": evidence.evidence_ref})
                except OSError:
                    self._enabled = False
                    self._last_safe = None
                    return None
                self._restart_dispatch_ids.clear()
            self._last_safe = evidence
            return evidence

    def safe_state(self, generation: int) -> SafeStateEvidence:
        with self._lock:
            if generation != self._generation or self._last_safe is None:
                raise GatewayError("safe_state_unconfirmed")
            return self._last_safe
