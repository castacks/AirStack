"""Narrow, fail-closed admission and stop state for the Office demo console."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
import time
from typing import Callable, Protocol
import uuid

from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal


class ProcessLike(Protocol):
    def poll(self) -> int | None: ...
    def wait(self) -> int: ...


@dataclass(frozen=True)
class RunningDispatch:
    process: ProcessLike
    request_stop: Callable[[], None]
    finalize: Callable[[int], dict]


Launcher = Callable[[str, Path, Path], RunningDispatch]


class ExecutionSupervisor:
    """Bind one exact proposal to explicit approval and an independent stop latch.

    This is deliberately a single-process demo boundary, not a distributed C06/C08
    implementation. It records intent before launch, serializes admission with stop,
    and never promotes cancellation acknowledgement to physical safety evidence.
    """

    def __init__(self, proposal: DroneTaskProposal, artifact_root: Path, launcher: Launcher):
        self.proposal = proposal
        self.artifact_root = artifact_root.resolve()
        self.launcher = launcher
        self.lock = threading.Lock()
        self.stop_generation = 0
        self.stop_latched = False
        self.state = "READY_FOR_APPROVAL"
        self.dispatch_id: str | None = None
        self.running: RunningDispatch | None = None
        self.active_is_land = False
        self.pending_land: tuple[str, Path, Path] | None = None
        self.last_result: dict | None = None
        self.proposal_bytes = (
            json.dumps(proposal.model_dump(mode="json"), sort_keys=True,
                       separators=(",", ":")) + "\n"
        ).encode()
        self.proposal_sha256 = hashlib.sha256(self.proposal_bytes).hexdigest()
        if self.artifact_root.exists() and (
            any(self.artifact_root.glob("*/admission.json"))
            or any(self.artifact_root.glob("*/operator-land.json"))
            or any(self.artifact_root.glob("*/stop-*.json"))
        ):
            # A restarted web process cannot prove that a prior remote action is idle.
            self.stop_latched = True
            self.state = "RECONCILIATION_REQUIRED"

    def status(self) -> dict:
        with self.lock:
            return self._status_locked()

    def _status_locked(self) -> dict:
        return {
            "state": self.state,
            "proposal_sha256": self.proposal_sha256,
            "proposal": self.proposal.model_dump(mode="json"),
            "preview": self.proposal.preview(),
            "stop_generation": self.stop_generation,
            "stop_latched": self.stop_latched,
            "dispatch_id": self.dispatch_id,
            "active": bool(self.running and self.running.process.poll() is None),
            "active_command": "LAND" if self.active_is_land else (
                self.proposal.kind.value if self.running else None
            ),
            "land_pending": self.pending_land is not None,
            "last_result": self.last_result,
            "safety_claim": "SAFE_UNCONFIRMED" if self.stop_latched else None,
        }

    def decide(self, decision: str, proposal_sha256: str) -> dict:
        if decision not in {"APPROVE", "REJECT"}:
            raise ValueError("Decision must be APPROVE or REJECT.")
        with self.lock:
            if proposal_sha256 != self.proposal_sha256:
                raise ValueError("Proposal changed; reload before deciding.")
            if self.stop_latched:
                raise ValueError("Admission is stopped; restart and reconcile before dispatch.")
            if self.running and self.running.process.poll() is None:
                raise ValueError("A dispatch is already active.")
            if self.state not in {"READY_FOR_APPROVAL"}:
                raise ValueError("This proposal already has a terminal admission decision.")

            dispatch_id = uuid.uuid4().hex
            run_dir = self.artifact_root / dispatch_id
            run_dir.mkdir(parents=True, exist_ok=False)
            proposal_path = run_dir / "proposal.json"
            proposal_path.write_bytes(self.proposal_bytes)
            event = {
                "schema_version": "rrm-office-admission/v1",
                "recorded_at": datetime.now(timezone.utc).isoformat(),
                "dispatch_id": dispatch_id,
                "decision": decision,
                "proposal_sha256": self.proposal_sha256,
                "stop_generation": self.stop_generation,
                "execution_requested": decision == "APPROVE",
            }
            # The admission record must exist before an execution process can start.
            (run_dir / "admission.json").write_text(
                json.dumps(event, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            self.dispatch_id = dispatch_id
            if decision == "REJECT":
                self.state = "REJECTED"
                self.last_result = {"verdict": "NOT_DISPATCHED", "reason": "operator_rejected"}
                return self._status_locked()

            self.state = "STARTING"
            try:
                self.running = self.launcher(dispatch_id, run_dir, proposal_path)
            except Exception as exc:
                self.state = "LAUNCH_FAILED"
                self.stop_latched = True
                self.stop_generation += 1
                self.last_result = {"verdict": "NOT_DISPATCHED", "reason": type(exc).__name__}
                raise RuntimeError("Dispatcher could not be started; admission is now stopped.") from exc
            self.state = "RUNNING"
            self.active_is_land = False
            threading.Thread(target=self._monitor, args=(dispatch_id, self.running),
                             daemon=True, name=f"rrm-dispatch-{dispatch_id[:8]}").start()
            return self._status_locked()

    def request_stop(self) -> dict:
        request_started_s = time.monotonic()
        with self.lock:
            if self.stop_latched and self.pending_land is None and not self.active_is_land:
                return self._status_locked()
            self.stop_generation += 1
            stop_generation = self.stop_generation
            self.stop_latched = True
            self.state = "STOP_REQUESTED"
            self.pending_land = None
            running = self.running
            dispatch_id = self.dispatch_id
            event = {
                "schema_version": "rrm-office-stop/v1",
                "recorded_at": datetime.now(timezone.utc).isoformat(),
                "dispatch_id": dispatch_id,
                "stop_generation": stop_generation,
                "cancel_requested": bool(running and running.process.poll() is None),
                "physical_stop_verified": False,
                "safety_claim": "SAFE_UNCONFIRMED",
            }
            stop_dir = self.artifact_root / (dispatch_id or "no-active-dispatch")
            stop_dir.mkdir(parents=True, exist_ok=True)
            (stop_dir / f"stop-{stop_generation}.json").write_text(
                json.dumps(event, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
        if running and running.process.poll() is None:
            try:
                running.request_stop()
                delivery = {
                    "schema_version": "rrm-office-stop-delivery/v1",
                    "recorded_at": datetime.now(timezone.utc).isoformat(),
                    "dispatch_id": dispatch_id,
                    "stop_generation": stop_generation,
                    "signal_delivery_latency_ms": round(
                        (time.monotonic() - request_started_s) * 1000, 3
                    ),
                }
                stop_dir = self.artifact_root / dispatch_id
                (stop_dir / f"stop-delivery-{stop_generation}.json").write_text(
                    json.dumps(delivery, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
            except Exception as exc:
                with self.lock:
                    self.last_result = {
                        "verdict": "SAFE_UNCONFIRMED",
                        "reason": f"cancel_delivery_failed:{type(exc).__name__}",
                    }
                raise RuntimeError("Stop latched, but cancellation delivery failed.") from exc
        return self.status()

    def request_land(self) -> dict:
        """Cancel an active command, then use the public LAND action as an override."""
        with self.lock:
            if self.pending_land is not None or self.active_is_land:
                return self._status_locked()
            if not self.stop_latched:
                self.stop_generation += 1
            self.stop_latched = True
            land_id = uuid.uuid4().hex
            land = DroneTaskProposal(
                task_id=f"operator-land-{land_id}",
                action_id="LAND_NOW",
                robot_name=self.proposal.robot_name,
                kind=DroneTaskKind.LAND,
                velocity_m_s=1.0,
            )
            run_dir = self.artifact_root / land_id
            run_dir.mkdir(parents=True, exist_ok=False)
            proposal_path = run_dir / "proposal.json"
            proposal_path.write_text(
                json.dumps(land.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            event = {
                "schema_version": "rrm-office-operator-land/v1",
                "recorded_at": datetime.now(timezone.utc).isoformat(),
                "dispatch_id": land_id,
                "decision": "LAND_NOW",
                "stop_generation": self.stop_generation,
                "normal_admission_blocked": True,
                "execution_requested": True,
            }
            (run_dir / "operator-land.json").write_text(
                json.dumps(event, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            active = self.running if self.running and self.running.process.poll() is None else None
            if active is not None:
                self.pending_land = (land_id, run_dir, proposal_path)
                self.state = "LAND_CANCELING_ACTIVE"
            else:
                self.dispatch_id = land_id
                self.state = "LAND_STARTING"
                self._launch_land_locked(land_id, run_dir, proposal_path)
                return self._status_locked()
        try:
            active.request_stop()
        except Exception as exc:
            with self.lock:
                self.pending_land = None
                self.state = "LAND_BLOCKED_UNCONFIRMED"
                self.last_result = {
                    "verdict": "UNCONFIRMED",
                    "reason": f"cancel_delivery_failed:{type(exc).__name__}",
                }
            raise RuntimeError("Landing blocked because active-command cancellation failed.") from exc
        return self.status()

    def _launch_land_locked(self, dispatch_id: str, run_dir: Path,
                            proposal_path: Path) -> None:
        try:
            running = self.launcher(dispatch_id, run_dir, proposal_path)
        except Exception as exc:
            self.state = "LAND_LAUNCH_FAILED"
            self.last_result = {"verdict": "NOT_DISPATCHED", "reason": type(exc).__name__}
            raise RuntimeError("Landing command could not be started.") from exc
        self.running = running
        self.active_is_land = True
        self.state = "LANDING"
        threading.Thread(target=self._monitor, args=(dispatch_id, running),
                         daemon=True, name=f"rrm-land-{dispatch_id[:8]}").start()

    def _monitor(self, dispatch_id: str, running: RunningDispatch) -> None:
        return_code = running.process.wait()
        try:
            result = running.finalize(return_code)
        except Exception as exc:
            result = {"return_code": return_code, "verdict": "UNCONFIRMED",
                      "reason": f"finalize_failed:{type(exc).__name__}"}
        with self.lock:
            if self.dispatch_id != dispatch_id:
                return
            self.running = None
            self.last_result = result
            completed_land = self.active_is_land
            self.active_is_land = False
            pending_land = self.pending_land
            self.pending_land = None
            if pending_land is not None:
                if result.get("cancel_acknowledged") is not True:
                    self.state = "LAND_BLOCKED_UNCONFIRMED"
                    return
                land_id, run_dir, proposal_path = pending_land
                self.dispatch_id = land_id
                self.state = "LAND_STARTING"
                self._launch_land_locked(land_id, run_dir, proposal_path)
            elif completed_land and self.state == "STOP_REQUESTED":
                self.state = "STOPPED_UNCONFIRMED"
            elif completed_land:
                self.state = "LAND_FINISHED"
            else:
                self.state = "STOPPED_UNCONFIRMED" if self.stop_latched else "FINISHED"
