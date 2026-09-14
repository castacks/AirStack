"""Read-only AirStack observation adapter for RRM shadow-mode evaluation.

This module deliberately has no ROS dependency.  It accepts normalized inputs from a
transport-specific observer, applies conservative freshness rules, and writes evidence
that can be replayed without granting any execution authority.  The optional rclpy
runner lives in ``scripts/airstack_shadow.py``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
from pathlib import Path
import time
from typing import Any, Mapping
from uuid import uuid4


SCHEMA_VERSION = "rrm-shadow/v1"


class EvidenceStatus(str, Enum):
    """Availability of an observation at the time a snapshot is assessed."""

    FRESH = "FRESH"
    STALE = "STALE"
    MISSING = "MISSING"


@dataclass(frozen=True)
class Position:
    """A frame-qualified position.  No absent frame or pose is fabricated."""

    x: float
    y: float
    z: float
    frame_id: str
    child_frame_id: str
    source_stamp_ns: int | None


@dataclass(frozen=True)
class ChannelState:
    status: EvidenceStatus
    received_monotonic_s: float | None
    age_s: float | None
    source: str
    detail: str | None = None


@dataclass(frozen=True)
class ShadowSnapshot:
    """Immutable, transport-neutral state available to RRM reasoning.

    ``execution_inhibited`` is intentionally permanent in this increment.  A fresh
    snapshot is evidence that RRM may reason in shadow mode, never evidence that it may
    dispatch a drone task.
    """

    schema_version: str
    run_id: str
    sequence: int
    wall_time_ns: int
    robot_name: str
    position: Position | None
    mavros_connected: bool | None
    mavros_mode: str | None
    armed: bool | None
    odometry: ChannelState
    mavros: ChannelState
    transform: ChannelState
    execution_inhibited: bool
    readiness: str
    reasons: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return _jsonable(asdict(self))


@dataclass(frozen=True)
class TaskStatusEvent:
    """Observed action-status event; never an action request."""

    action: str
    status_codes: tuple[int, ...]
    received_monotonic_s: float


def _jsonable(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    return value


class JsonlEvidenceWriter:
    """Small append-only evidence writer with explicit run metadata."""

    def __init__(self, output_dir: Path, manifest: Mapping[str, Any]) -> None:
        self.output_dir = output_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        self._events = (output_dir / "events.jsonl").open("w", encoding="utf-8")
        complete_manifest = {
            "schema_version": SCHEMA_VERSION,
            "created_wall_time_ns": time.time_ns(),
            "observer_only": True,
            "execution_dispatch_enabled": False,
            **dict(manifest),
        }
        (output_dir / "manifest.json").write_text(
            json.dumps(_jsonable(complete_manifest), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.event("run_start", manifest=complete_manifest)

    def event(self, kind: str, **fields: Any) -> None:
        record = {
            "schema_version": SCHEMA_VERSION,
            "event_id": uuid4().hex,
            "wall_time_ns": time.time_ns(),
            "kind": kind,
            **fields,
        }
        self._events.write(json.dumps(_jsonable(record), sort_keys=True) + "\n")
        self._events.flush()

    def close(self) -> None:
        if not self._events.closed:
            self.event("run_end")
            self._events.close()


class AirStackShadowAdapter:
    """Conservatively assess AirStack observations without a control surface.

    The public API has only ingestion, snapshot, and evidence methods.  It has no
    method that creates a ROS action, service, publisher, trajectory, or PX4 command.
    """

    def __init__(self, *, run_id: str, robot_name: str,
                 max_observation_age_s: float = 1.0,
                 writer: JsonlEvidenceWriter | None = None) -> None:
        if not run_id.strip() or not robot_name.strip():
            raise ValueError("run_id and robot_name are required")
        if max_observation_age_s <= 0:
            raise ValueError("max_observation_age_s must be positive")
        self.run_id = run_id
        self.robot_name = robot_name
        self.max_observation_age_s = max_observation_age_s
        self.writer = writer
        self._sequence = 0
        self._position: Position | None = None
        self._odom_received: float | None = None
        self._mavros_connected: bool | None = None
        self._mavros_mode: str | None = None
        self._armed: bool | None = None
        self._mavros_received: float | None = None
        self._tf_received: float | None = None
        self._task_events: list[TaskStatusEvent] = []

    @property
    def observer_only(self) -> bool:
        return True

    def ingest_odometry(self, *, x: float, y: float, z: float, frame_id: str,
                        child_frame_id: str, source_stamp_ns: int | None,
                        received_monotonic_s: float | None = None) -> None:
        if not frame_id or not child_frame_id:
            raise ValueError("odometry frame IDs are required")
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        self._position = Position(x, y, z, frame_id, child_frame_id, source_stamp_ns)
        self._odom_received = received
        self._event("odometry_received", position=asdict(self._position),
                    received_monotonic_s=received)

    def ingest_mavros_state(self, *, connected: bool, armed: bool, mode: str,
                            received_monotonic_s: float | None = None) -> None:
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        self._mavros_connected = bool(connected)
        self._armed = bool(armed)
        self._mavros_mode = mode or None
        self._mavros_received = received
        self._event("mavros_state_received", connected=self._mavros_connected,
                    armed=self._armed, mode=self._mavros_mode,
                    received_monotonic_s=received)

    def ingest_map_to_base_link_transform(self, *, received_monotonic_s: float | None = None) -> None:
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        self._tf_received = received
        self._event("map_to_base_link_received", received_monotonic_s=received)

    def ingest_task_status(self, *, action: str, status_codes: list[int] | tuple[int, ...],
                           received_monotonic_s: float | None = None) -> None:
        if not action.strip():
            raise ValueError("action name is required")
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        event = TaskStatusEvent(action, tuple(int(code) for code in status_codes), received)
        self._task_events.append(event)
        self._event("task_status_observed", task_status=asdict(event))

    def snapshot(self, *, now_monotonic_s: float | None = None,
                 wall_time_ns: int | None = None) -> ShadowSnapshot:
        now = time.monotonic() if now_monotonic_s is None else now_monotonic_s
        self._sequence += 1
        odometry = self._channel("odometry", self._odom_received, now)
        mavros = self._channel("mavros_state", self._mavros_received, now)
        transform = self._channel("map_to_base_link_tf", self._tf_received, now)
        reasons: list[str] = []
        if odometry.status is not EvidenceStatus.FRESH:
            reasons.append(f"odometry_{odometry.status.value.lower()}")
        if mavros.status is not EvidenceStatus.FRESH:
            reasons.append(f"mavros_{mavros.status.value.lower()}")
        elif self._mavros_connected is not True:
            reasons.append("mavros_disconnected")
        if transform.status is not EvidenceStatus.FRESH:
            reasons.append(f"transform_{transform.status.value.lower()}")
        readiness = "OBSERVATION_READY" if not reasons else "OBSERVATION_INCOMPLETE"
        snapshot = ShadowSnapshot(
            schema_version=SCHEMA_VERSION,
            run_id=self.run_id,
            sequence=self._sequence,
            wall_time_ns=time.time_ns() if wall_time_ns is None else wall_time_ns,
            robot_name=self.robot_name,
            position=self._position if odometry.status is EvidenceStatus.FRESH else None,
            mavros_connected=self._mavros_connected if mavros.status is EvidenceStatus.FRESH else None,
            mavros_mode=self._mavros_mode if mavros.status is EvidenceStatus.FRESH else None,
            armed=self._armed if mavros.status is EvidenceStatus.FRESH else None,
            odometry=odometry,
            mavros=mavros,
            transform=transform,
            execution_inhibited=True,
            readiness=readiness,
            reasons=tuple(reasons),
        )
        self._event("shadow_snapshot", snapshot=snapshot.as_dict())
        return snapshot

    def completeness_report(self, snapshot: ShadowSnapshot) -> dict[str, Any]:
        """Report evidence completeness; never translate it into dispatch permission."""
        channels = (snapshot.odometry, snapshot.mavros, snapshot.transform)
        complete = all(channel.status is EvidenceStatus.FRESH for channel in channels)
        return {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "snapshot_sequence": snapshot.sequence,
            "observation_complete": complete and snapshot.mavros_connected is True,
            "execution_dispatch_enabled": False,
            "reasons": list(snapshot.reasons),
        }

    def _channel(self, source: str, received: float | None, now: float) -> ChannelState:
        if received is None:
            return ChannelState(EvidenceStatus.MISSING, None, None, source)
        age = max(0.0, now - received)
        status = EvidenceStatus.FRESH if age <= self.max_observation_age_s else EvidenceStatus.STALE
        return ChannelState(status, received, age, source)

    def _event(self, kind: str, **fields: Any) -> None:
        if self.writer is not None:
            self.writer.event(kind, run_id=self.run_id, robot_name=self.robot_name, **fields)
