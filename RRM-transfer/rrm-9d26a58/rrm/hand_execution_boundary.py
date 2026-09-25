"""Fail-closed, transport-free hand admission prototype.

This module has no Isaac/ROS adapter. A real gateway must additionally enforce
dispatch-ID deduplication, generation fencing, measured stop and chunk limits.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from threading import RLock
import time
from typing import Protocol
from uuid import uuid4

from .contracts import DispatchContext, SafetyDecision


class BoundaryError(RuntimeError):
    pass


def _digest(data: object) -> str:
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class HandCommand:
    """One small arm-joint calibration target, never a hand/grasp command."""

    joint_name: str
    observed_position_rad: float
    target_position_rad: float
    observed_at_monotonic: float
    episode_id: str
    scene_recipe_sha256: str
    probe_sha256: str
    qualification_sha256: str
    operation: str = "ARM_JOINT_CALIBRATION"

    @property
    def digest(self) -> str:
        return _digest(self.__dict__)


class HandAdapter(Protocol):
    """Injected gateway. submit and request_stop must be bounded/nonblocking."""

    def submit(self, dispatch_id: str, generation: int, command: HandCommand) -> None: ...

    def request_stop(self, generation: int) -> bool: ...

    def safe_state(self, generation: int) -> SafeStateEvidence: ...


@dataclass(frozen=True)
class SafeStateEvidence:
    evidence_ref: str
    episode_id: str
    generation: int
    observed_at_monotonic: float
    motion_stopped: bool
    controller_mode: str
    active_motion_command: bool
    max_joint_velocity_rad_s: float
    max_object_velocity_m_s: float
    consecutive_safe_count: int


class DurableJournal:
    """Synchronous append-only JSONL; fsync is part of append success."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.has_dispatch_intent = False
        if self.path.exists() and self.path.stat().st_size:
            # Reopening is intentionally inhibited by the boundary. Reject malformed
            # tails; do not silently treat an incomplete evidence chain as valid.
            previous = "0" * 64
            sequence = 0
            for line in self.path.read_bytes().splitlines():
                item = json.loads(line)
                body = {k: v for k, v in item.items() if k != "hash"}
                if item["previous_hash"] != previous or item["sequence"] != sequence + 1 \
                        or item["hash"] != _digest(body):
                    raise BoundaryError("journal_chain_invalid")
                if item["event"] == "C09_DISPATCH_INTENT":
                    self.has_dispatch_intent = True
                previous, sequence = item["hash"], sequence + 1
            self.previous_hash, self.sequence = previous, sequence
        else:
            self.previous_hash, self.sequence = "0" * 64, 0

    def append(self, event: str, payload: dict) -> str:
        body = {"schema_version": "rrm-hand-c09/v1", "sequence": self.sequence + 1,
                "previous_hash": self.previous_hash, "event": event, "payload": payload,
                "wall_time_ns": time.time_ns()}
        record_hash = _digest(body)
        encoded = (json.dumps({**body, "hash": record_hash}, sort_keys=True,
                              separators=(",", ":"), allow_nan=False) + "\n").encode()
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        try:
            written = 0
            while written < len(encoded):
                count = os.write(fd, encoded[written:])
                if count <= 0:
                    raise OSError("short journal write")
                written += count
            os.fsync(fd)
        finally:
            os.close(fd)
        self.sequence += 1
        self.previous_hash = record_hash
        if event == "C09_DISPATCH_INTENT":
            self.has_dispatch_intent = True
        return record_hash


class HandExecutionBoundary:
    """Local C06/C08/C09 boundary, initially inhibited on every construction.

    A real deployment must keep this lock order and put an independent generation
    fence at the adapter. No remote exactly-once or physical safety claim is made.
    """

    def __init__(self, *, adapter: HandAdapter, journal: DurableJournal,
                 qualification_bytes: bytes, probe_bytes: bytes,
                 max_observation_age_s: float = 0.25, max_delta_rad: float = 0.02):
        self._lock = RLock()
        self.adapter, self.journal = adapter, journal
        self.qualification = json.loads(qualification_bytes)
        self.probe = json.loads(probe_bytes)
        qualification, probe = self.qualification, self.probe
        self.qualification_sha256 = hashlib.sha256(qualification_bytes).hexdigest()
        self.max_observation_age_s, self.max_delta_rad = max_observation_age_s, max_delta_rad
        self.epoch = uuid4().hex
        self.generation = 0
        self.inhibited = True
        self.safe_confirmed = False
        self._used_decisions: set[str] = set()
        self._used_dispatches: set[str] = set()
        self._active_dispatch: str | None = None
        self._episode_id: str | None = None
        self._restart_reconciliation_required = journal.has_dispatch_intent
        self._journal_fault = False
        if qualification.get("schema_version") != "rrm-hand-qualification/v1" or \
                qualification.get("probe_sha256") != hashlib.sha256(probe_bytes).hexdigest() or \
                qualification.get("scene_recipe_sha256") != probe.get("scene_recipe_sha256") or \
                set(qualification.get("gates", {})) != {"controller_limits", "post_command_reset",
                    "contact_observer", "safe_state", "independent_stop"} or \
                not all(g.get("status") == "PASS" for g in qualification["gates"].values()) or \
                qualification.get("ready_for_single_bounded_contact_trial") is not True or \
                qualification.get("execution_dispatch") is not False:
            raise BoundaryError("qualification_invalid")

    def validate_command(self, command: HandCommand, *, now: float) -> None:
        if command.operation != "ARM_JOINT_CALIBRATION":
            raise BoundaryError("unsupported_operation")
        if not command.episode_id or command.episode_id != self._episode_id or \
                not command.scene_recipe_sha256 or \
                command.scene_recipe_sha256 != self.probe.get("scene_recipe_sha256") or \
                command.probe_sha256 != self.qualification["probe_sha256"] or \
                command.qualification_sha256 != self.qualification_sha256:
            raise BoundaryError("stale_qualification_or_episode")
        numbers = (now, command.observed_at_monotonic, command.observed_position_rad,
                   command.target_position_rad)
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in numbers) or \
                not 0 <= now - command.observed_at_monotonic <= self.max_observation_age_s:
            raise BoundaryError("stale_joint_observation")
        limits = self.probe.get("joint_limits", [])
        matches = [item for item in limits if item.get("name") == command.joint_name]
        if len(matches) != 1 or command.joint_name not in \
                {f"iiwa7_joint_{index}" for index in range(1, 8)}:
            raise BoundaryError("unsupported_joint")
        limit = matches[0]
        lower, upper = limit["lower_rad"], limit["upper_rad"]
        if not lower <= command.observed_position_rad <= upper or \
                not lower <= command.target_position_rad <= upper or \
                abs(command.target_position_rad - command.observed_position_rad) > self.max_delta_rad:
            raise BoundaryError("numeric_envelope_exceeded")

    def _safe_evidence(self, generation: int, now: float) -> SafeStateEvidence | None:
        try:
            evidence = self.adapter.safe_state(generation)
        except Exception:
            return None
        safe = self.probe["safe_state"]
        if not isinstance(evidence, SafeStateEvidence) or not evidence.evidence_ref or \
                not evidence.episode_id or \
                evidence.generation != generation or \
                not math.isfinite(now) or not math.isfinite(evidence.observed_at_monotonic) or \
                not 0 <= now - evidence.observed_at_monotonic <= self.max_observation_age_s or \
                evidence.motion_stopped is not True or \
                evidence.controller_mode != "POSITION_HOLD" or \
                evidence.active_motion_command is not False or \
                not all(math.isfinite(v) and v >= 0 for v in
                        (evidence.max_joint_velocity_rad_s, evidence.max_object_velocity_m_s)) or \
                evidence.max_joint_velocity_rad_s > safe["joint_velocity_threshold_rad_s"] or \
                evidence.max_object_velocity_m_s > safe["object_velocity_threshold_m_s"] or \
                evidence.consecutive_safe_count < safe["consecutive_window_required"]:
            return None
        return evidence

    def reset(self, *, authorized: bool, generation: int, now: float) -> bool:
        with self._lock:
            if not (authorized is True and generation == self.generation and
                    not self._restart_reconciliation_required and not self._journal_fault and
                    self._active_dispatch is None):
                return False
            evidence = self._safe_evidence(generation, now)
            if evidence is None:
                return False
            try:
                self.journal.append("C08_RESET", {"epoch": self.epoch,
                    "generation": self.generation + 1, "evidence_ref": evidence.evidence_ref})
            except OSError:
                self.inhibited = True
                self._journal_fault = True
                return False
            self.generation += 1
            self.inhibited = False
            self.safe_confirmed = True
            self._episode_id = evidence.episode_id
            return True

    def dispatch(self, decision: SafetyDecision, current: DispatchContext,
                 command: HandCommand, *, now: float) -> str:
        with self._lock:
            if self.inhibited or self._active_dispatch is not None:
                raise BoundaryError("admission_closed")
            self.validate_command(command, now=now)
            if decision.verdict != "ALLOW" or decision.context != current or \
                    current.authority_epoch != self.epoch or \
                    current.stop_generation != self.generation or \
                    current.action_digest != command.digest or \
                    not math.isfinite(now) or \
                    not decision.issued_at <= now < decision.expires_at:
                raise BoundaryError("stale_or_denied_decision")
            if decision.decision_id in self._used_decisions or \
                    current.dispatch_id in self._used_dispatches:
                raise BoundaryError("consumed_decision_or_dispatch")
            payload = {"decision_id": decision.decision_id, "dispatch_id": current.dispatch_id,
                       "context": current.__dict__, "command": command.__dict__,
                       "command_digest": command.digest, "qualification_sha256": self.qualification_sha256}
            try:
                intent_hash = self.journal.append("C09_DISPATCH_INTENT", payload)
            except OSError as exc:
                self.inhibited = True
                self._journal_fault = True
                raise BoundaryError("journal_unavailable") from exc
            self._used_decisions.add(decision.decision_id)
            self._used_dispatches.add(current.dispatch_id)
            self._active_dispatch = current.dispatch_id
            self.safe_confirmed = False
            try:
                self.adapter.submit(current.dispatch_id, self.generation, command)
            except Exception as exc:
                self.inhibited = True
                self.generation += 1
                try:
                    self.adapter.request_stop(self.generation)
                except Exception:
                    pass
                try:
                    self.journal.append("C09_DISPATCH_UNCERTAIN", {"dispatch_id": current.dispatch_id,
                                                                   "intent_hash": intent_hash,
                                                                   "stop_generation": self.generation})
                except OSError:
                    self._journal_fault = True
                    pass
                raise BoundaryError("adapter_submit_uncertain") from exc
            return intent_hash

    def stop(self, *, intervention_id: str, reason: str) -> bool:
        """Close admission before independent cancellation; unknown ack is not safe."""
        with self._lock:
            self.inhibited = True
            self.safe_confirmed = False
            self._episode_id = None
            self.generation += 1
            generation = self.generation
            try:
                self.journal.append("C08_STOP_RECEIVED", {"intervention_id": intervention_id,
                    "reason": reason, "generation": generation,
                    "dispatch_id": self._active_dispatch})
            except OSError:
                self._journal_fault = True  # Stop delivery cannot depend on trace storage.
        try:
            accepted = self.adapter.request_stop(generation) is True
        except Exception:
            accepted = False
        if accepted:
            try:
                self.journal.append("C08_CANCEL_ACCEPTED", {"intervention_id": intervention_id,
                    "generation": generation})
            except OSError:
                self._journal_fault = True
                pass
        return accepted

    def confirm_stopped(self, *, generation: int, now: float) -> bool:
        with self._lock:
            if generation != self.generation:
                return False
            evidence = self._safe_evidence(generation, now)
            if evidence is None:
                return False
            try:
                self.journal.append("C08_MOTION_STOPPED", {"generation": generation,
                    "evidence_ref": evidence.evidence_ref, "dispatch_id": self._active_dispatch})
                self.journal.append("C08_SAFE_CONFIRMED", {"generation": generation,
                    "evidence_ref": evidence.evidence_ref})
            except OSError:
                self.inhibited = True
                self._journal_fault = True
                return False
            self._active_dispatch = None
            self.safe_confirmed = True
            return True
