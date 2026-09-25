"""Fail-closed, transport-free hand admission prototype.

This module has no Isaac/ROS transport. The injected gateway must enforce its own
dispatch-ID deduplication, generation fencing, measured stop and chunk limits.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
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


def _is_sha256(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value)


@dataclass(frozen=True)
class HandAuthorization:
    """Signed, short-lived authority for exactly one boundary operation."""

    authorization_id: str
    issuer_id: str
    subject_id: str
    role: str
    purpose: str
    authority_epoch: str
    stop_generation: int
    scope_digest: str
    issued_at_monotonic: float
    expires_at_monotonic: float
    signature: str
    schema_version: str = "rrm-hand-authorization/v1"

    @property
    def unsigned_payload(self) -> dict:
        return {key: value for key, value in self.__dict__.items() if key != "signature"}

    @property
    def digest(self) -> str:
        return _digest(self.__dict__)


def issue_hand_authorization(*, signing_key: bytes, authorization_id: str,
                             issuer_id: str, subject_id: str, role: str,
                             purpose: str, authority_epoch: str,
                             stop_generation: int, scope_digest: str,
                             issued_at_monotonic: float,
                             expires_at_monotonic: float) -> HandAuthorization:
    """Issuer-side helper; production keys must remain outside the boundary process."""
    if not isinstance(signing_key, bytes) or len(signing_key) < 32:
        raise ValueError("authorization signing key must contain at least 32 bytes")
    grant = HandAuthorization(authorization_id, issuer_id, subject_id, role, purpose,
        authority_epoch, stop_generation, scope_digest, issued_at_monotonic,
        expires_at_monotonic, "")
    signature = hmac.new(signing_key,
        json.dumps(grant.unsigned_payload, sort_keys=True, separators=(",", ":"),
                   allow_nan=False).encode(), hashlib.sha256).hexdigest()
    return HandAuthorization(**{**grant.__dict__, "signature": signature})


class HandAuthorityVerifier:
    """Verify trusted issuers and roles without exposing an authorization shortcut."""

    PURPOSES = frozenset({"RESET", "RECONCILE", "DISPATCH"})

    def __init__(self, *, issuer_keys: dict[str, bytes], allowed_roles: frozenset[str],
                 max_lifetime_s: float = 30.0):
        if not issuer_keys or any(not isinstance(name, str) or not name or
                not isinstance(key, bytes) or len(key) < 32
                for name, key in issuer_keys.items()) or \
                not allowed_roles or any(not isinstance(role, str) or not role
                                         for role in allowed_roles) or \
                not isinstance(max_lifetime_s, (int, float)) or \
                isinstance(max_lifetime_s, bool) or not math.isfinite(max_lifetime_s) or \
                max_lifetime_s <= 0:
            raise ValueError("invalid hand authority verifier configuration")
        self._issuer_keys = dict(issuer_keys)
        self._allowed_roles = frozenset(allowed_roles)
        self._max_lifetime_s = max_lifetime_s

    def verify(self, grant: HandAuthorization, *, purpose: str, authority_epoch: str,
               stop_generation: int, scope_digest: str, now: float) -> None:
        if not isinstance(grant, HandAuthorization) or \
                grant.schema_version != "rrm-hand-authorization/v1" or \
                any(not isinstance(value, str) or not value for value in
                    (grant.authorization_id, grant.issuer_id, grant.subject_id,
                     grant.role, grant.purpose, grant.authority_epoch,
                     grant.scope_digest, grant.signature)) or \
                grant.purpose not in self.PURPOSES or grant.role not in self._allowed_roles or \
                not _is_sha256(grant.scope_digest) or not _is_sha256(grant.signature) or \
                grant.purpose != purpose or grant.authority_epoch != authority_epoch or \
                type(grant.stop_generation) is not int or \
                grant.stop_generation != stop_generation or grant.scope_digest != scope_digest or \
                not all(isinstance(value, (int, float)) and not isinstance(value, bool) and
                        math.isfinite(value) for value in
                        (grant.issued_at_monotonic, grant.expires_at_monotonic, now)) or \
                not 0 <= grant.issued_at_monotonic <= now < grant.expires_at_monotonic or \
                grant.expires_at_monotonic - grant.issued_at_monotonic > self._max_lifetime_s:
            raise BoundaryError("authorization_scope_or_time_invalid")
        key = self._issuer_keys.get(grant.issuer_id)
        if key is None:
            raise BoundaryError("authorization_issuer_unknown")
        expected = hmac.new(key,
            json.dumps(grant.unsigned_payload, sort_keys=True, separators=(",", ":"),
                       allow_nan=False).encode(), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(grant.signature, expected):
            raise BoundaryError("authorization_signature_invalid")


def reset_authorization_scope(*, generation: int, evidence: SafeStateEvidence,
                              qualification_sha256: str, probe_sha256: str) -> str:
    return _digest({"purpose": "RESET", "generation": generation,
        "next_generation": generation + 1, "evidence": evidence.__dict__,
        "qualification_sha256": qualification_sha256, "probe_sha256": probe_sha256})


def reconciliation_authorization_scope(*, generation: int,
        dispatch_ids: tuple[str, ...], evidence: SafeStateEvidence,
        qualification_sha256: str, probe_sha256: str) -> str:
    return _digest({"purpose": "RECONCILE", "generation": generation,
        "dispatch_ids": dispatch_ids, "evidence": evidence.__dict__,
        "qualification_sha256": qualification_sha256, "probe_sha256": probe_sha256})


def dispatch_authorization_scope(*, decision: SafetyDecision, current: DispatchContext,
                                 command: HandCommand, qualification_sha256: str,
                                 probe_sha256: str) -> str:
    return _digest({"purpose": "DISPATCH", "decision": {
        "decision_id": decision.decision_id, "context": decision.context.__dict__,
        "verdict": decision.verdict, "issued_at": decision.issued_at,
        "expires_at": decision.expires_at},
        "context": current.__dict__, "command": command.__dict__,
        "qualification_sha256": qualification_sha256, "probe_sha256": probe_sha256})


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
        self.latest_generation = 0
        self.outstanding_dispatch_ids: set[str] = set()
        self.seen_dispatch_ids: set[str] = set()
        self.seen_decision_ids: set[str] = set()
        self.used_authorization_ids: set[str] = set()
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
                self._observe(item["event"], item["payload"])
                previous, sequence = item["hash"], sequence + 1
            self.previous_hash, self.sequence = previous, sequence
        else:
            self.previous_hash, self.sequence = "0" * 64, 0

    def _observe(self, event: str, payload: dict) -> None:
        if not isinstance(payload, dict):
            raise BoundaryError("journal_payload_invalid")
        generations = [payload.get("generation"), payload.get("stop_generation")]
        context = payload.get("context")
        if isinstance(context, dict):
            generations.append(context.get("stop_generation"))
        for generation in generations:
            if isinstance(generation, int) and not isinstance(generation, bool) and generation >= 0:
                self.latest_generation = max(self.latest_generation, generation)
        if event in {"C09_DISPATCH_INTENT", "C09_ADAPTER_ENQUEUE"}:
            dispatch_id = payload.get("dispatch_id")
            if not isinstance(dispatch_id, str) or not dispatch_id:
                raise BoundaryError("journal_dispatch_id_invalid")
            if dispatch_id in self.seen_dispatch_ids:
                raise BoundaryError("journal_duplicate_dispatch_id")
            self.seen_dispatch_ids.add(dispatch_id)
            self.outstanding_dispatch_ids.add(dispatch_id)
            decision_id = payload.get("decision_id")
            if decision_id is not None:
                if not isinstance(decision_id, str) or not decision_id or \
                        decision_id in self.seen_decision_ids:
                    raise BoundaryError("journal_decision_id_invalid")
                self.seen_decision_ids.add(decision_id)
        elif event in {"C09_DISPATCH_RECONCILED", "C09_RESTART_RECONCILED",
                       "C08_ADAPTER_SAFE_RECONCILED"}:
            dispatch_ids = payload.get("dispatch_ids")
            if not isinstance(dispatch_ids, list) or not all(
                    isinstance(value, str) and value for value in dispatch_ids):
                raise BoundaryError("journal_reconciliation_invalid")
            if len(dispatch_ids) != len(set(dispatch_ids)):
                raise BoundaryError("journal_reconciliation_invalid")
            if not set(dispatch_ids) <= self.outstanding_dispatch_ids:
                raise BoundaryError("journal_reconciliation_unknown_dispatch")
            self.outstanding_dispatch_ids.difference_update(dispatch_ids)
        elif event == "C06_AUTHORIZATION_CONSUMED":
            authorization_id = payload.get("authorization_id")
            if not isinstance(authorization_id, str) or not authorization_id or \
                    authorization_id in self.used_authorization_ids:
                raise BoundaryError("journal_authorization_invalid")
            self.used_authorization_ids.add(authorization_id)
        self.has_dispatch_intent = bool(self.outstanding_dispatch_ids)

    def append(self, event: str, payload: dict) -> str:
        if event in {"C09_DISPATCH_RECONCILED", "C09_RESTART_RECONCILED",
                     "C08_ADAPTER_SAFE_RECONCILED"}:
            dispatch_ids = payload.get("dispatch_ids") if isinstance(payload, dict) else None
            if not isinstance(dispatch_ids, list) or len(dispatch_ids) != len(set(dispatch_ids)) or \
                    not all(isinstance(value, str) and value for value in dispatch_ids) or \
                    not set(dispatch_ids) <= self.outstanding_dispatch_ids:
                raise BoundaryError("journal_reconciliation_invalid")
        if event == "C06_AUTHORIZATION_CONSUMED":
            authorization_id = payload.get("authorization_id") if isinstance(payload, dict) else None
            if not isinstance(authorization_id, str) or not authorization_id or \
                    authorization_id in self.used_authorization_ids:
                raise BoundaryError("journal_authorization_invalid")
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
        self._observe(event, payload)
        return record_hash


class HandExecutionBoundary:
    """Local C06/C08/C09 boundary, initially inhibited on every construction.

    A real deployment must keep this lock order and put an independent generation
    fence at the adapter. No remote exactly-once or physical safety claim is made.
    """

    def __init__(self, *, adapter: HandAdapter, journal: DurableJournal,
                 authority_verifier: HandAuthorityVerifier,
                 qualification_bytes: bytes, probe_bytes: bytes,
                 max_observation_age_s: float = 0.25, max_delta_rad: float = 0.02):
        self._lock = RLock()
        if not isinstance(authority_verifier, HandAuthorityVerifier):
            raise BoundaryError("authority_verifier_required")
        self.adapter, self.journal = adapter, journal
        self.authority_verifier = authority_verifier
        self.qualification = json.loads(qualification_bytes)
        self.probe = json.loads(probe_bytes)
        qualification, probe = self.qualification, self.probe
        self.qualification_sha256 = hashlib.sha256(qualification_bytes).hexdigest()
        self.max_observation_age_s, self.max_delta_rad = max_observation_age_s, max_delta_rad
        self.epoch = uuid4().hex
        self.generation = journal.latest_generation
        self.inhibited = True
        self.safe_confirmed = False
        self._used_decisions = set(journal.seen_decision_ids)
        self._used_dispatches = set(journal.seen_dispatch_ids)
        self._used_authorizations = set(journal.used_authorization_ids)
        self._active_dispatch: str | None = None
        self._episode_id: str | None = None
        self._restart_reconciliation_required = journal.has_dispatch_intent
        self._restart_dispatch_ids = set(journal.outstanding_dispatch_ids)
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

    def _consume_authorization(self, grant: HandAuthorization, *, purpose: str,
                               scope_digest: str, now: float) -> None:
        if grant.authorization_id in self._used_authorizations:
            raise BoundaryError("authorization_consumed")
        self.authority_verifier.verify(grant, purpose=purpose,
            authority_epoch=self.epoch, stop_generation=self.generation,
            scope_digest=scope_digest, now=now)
        try:
            self.journal.append("C06_AUTHORIZATION_CONSUMED", {
                "authorization_id": grant.authorization_id,
                "authorization_digest": grant.digest, "issuer_id": grant.issuer_id,
                "subject_id": grant.subject_id, "role": grant.role,
                "purpose": purpose, "scope_digest": scope_digest,
                "generation": self.generation, "epoch": self.epoch})
        except OSError as exc:
            self.inhibited = True
            self._journal_fault = True
            raise BoundaryError("authorization_journal_unavailable") from exc
        self._used_authorizations.add(grant.authorization_id)

    def reset(self, *, authorization: HandAuthorization,
              generation: int, now: float) -> bool:
        with self._lock:
            if not (self.inhibited and generation == self.generation and
                    not self._restart_reconciliation_required and not self._journal_fault and
                    self._active_dispatch is None):
                return False
            evidence = self._safe_evidence(generation, now)
            if evidence is None:
                return False
            try:
                self._consume_authorization(authorization, purpose="RESET",
                    scope_digest=reset_authorization_scope(generation=generation,
                        evidence=evidence, qualification_sha256=self.qualification_sha256,
                        probe_sha256=self.qualification["probe_sha256"]), now=now)
            except BoundaryError:
                return False
            try:
                self.journal.append("C08_RESET", {"epoch": self.epoch,
                    "generation": self.generation + 1, "evidence_ref": evidence.evidence_ref,
                    "authorization_id": authorization.authorization_id})
            except OSError:
                self.inhibited = True
                self._journal_fault = True
                return False
            self.generation += 1
            self.inhibited = False
            self.safe_confirmed = True
            self._episode_id = evidence.episode_id
            return True

    def reconcile_restart(self, *, authorization: HandAuthorization,
                          generation: int, now: float) -> bool:
        """Close recovered dispatch intents from fresh safe evidence; keep inhibited."""
        with self._lock:
            if not (self.inhibited and
                    self._restart_reconciliation_required and
                    generation == self.generation and not self._journal_fault and
                    self._active_dispatch is None and self._restart_dispatch_ids):
                return False
            evidence = self._safe_evidence(generation, now)
            if evidence is None:
                return False
            dispatch_ids = sorted(self._restart_dispatch_ids)
            try:
                self._consume_authorization(authorization, purpose="RECONCILE",
                    scope_digest=reconciliation_authorization_scope(generation=generation,
                        dispatch_ids=tuple(dispatch_ids), evidence=evidence,
                        qualification_sha256=self.qualification_sha256,
                        probe_sha256=self.qualification["probe_sha256"]), now=now)
            except BoundaryError:
                return False
            try:
                self.journal.append("C09_RESTART_RECONCILED", {
                    "generation": generation, "dispatch_ids": dispatch_ids,
                    "evidence_ref": evidence.evidence_ref,
                    "episode_id": evidence.episode_id,
                    "authorization_id": authorization.authorization_id})
            except OSError:
                self._journal_fault = True
                return False
            self._restart_dispatch_ids.clear()
            self._restart_reconciliation_required = False
            self.safe_confirmed = True
            self._episode_id = evidence.episode_id
            return True

    def dispatch(self, decision: SafetyDecision, current: DispatchContext,
                 command: HandCommand, *, authorization: HandAuthorization,
                 now: float) -> str:
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
                       "command_digest": command.digest,
                       "qualification_sha256": self.qualification_sha256,
                       "authorization_id": authorization.authorization_id,
                       "authorization_digest": authorization.digest}
            self._consume_authorization(authorization, purpose="DISPATCH",
                scope_digest=dispatch_authorization_scope(decision=decision,
                    current=current, command=command,
                    qualification_sha256=self.qualification_sha256,
                    probe_sha256=self.qualification["probe_sha256"]), now=now)
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
                if self._active_dispatch is not None:
                    self.journal.append("C09_DISPATCH_RECONCILED", {"generation": generation,
                        "dispatch_ids": [self._active_dispatch],
                        "evidence_ref": evidence.evidence_ref})
            except OSError:
                self.inhibited = True
                self._journal_fault = True
                return False
            self._active_dispatch = None
            self.safe_confirmed = True
            return True
