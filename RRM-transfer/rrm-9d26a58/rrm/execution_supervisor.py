"""Narrow, fail-closed admission and stop state for the Office demo console."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
from typing import Callable, Protocol
import uuid

from rrm.airstack_drone import DroneTaskProposal


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
        self.last_result: dict | None = None
        self.proposal_bytes = (
            json.dumps(proposal.model_dump(mode="json"), sort_keys=True,
                       separators=(",", ":")) + "\n"
        ).encode()
        self.proposal_sha256 = hashlib.sha256(self.proposal_bytes).hexdigest()
        if self.artifact_root.exists() and any(self.artifact_root.glob("*/admission.json")):
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
            threading.Thread(target=self._monitor, args=(dispatch_id, self.running),
                             daemon=True, name=f"rrm-dispatch-{dispatch_id[:8]}").start()
            return self._status_locked()

    def request_stop(self) -> dict:
        with self.lock:
            if self.stop_latched:
                return self._status_locked()
            self.stop_generation += 1
            self.stop_latched = True
            self.state = "STOP_REQUESTED"
            running = self.running
            dispatch_id = self.dispatch_id
            event = {
                "schema_version": "rrm-office-stop/v1",
                "recorded_at": datetime.now(timezone.utc).isoformat(),
                "dispatch_id": dispatch_id,
                "stop_generation": self.stop_generation,
                "cancel_requested": bool(running and running.process.poll() is None),
                "physical_stop_verified": False,
                "safety_claim": "SAFE_UNCONFIRMED",
            }
            stop_dir = self.artifact_root / (dispatch_id or "no-active-dispatch")
            stop_dir.mkdir(parents=True, exist_ok=True)
            (stop_dir / f"stop-{self.stop_generation}.json").write_text(
                json.dumps(event, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
        if running and running.process.poll() is None:
            try:
                running.request_stop()
            except Exception as exc:
                with self.lock:
                    self.last_result = {
                        "verdict": "SAFE_UNCONFIRMED",
                        "reason": f"cancel_delivery_failed:{type(exc).__name__}",
                    }
                raise RuntimeError("Stop latched, but cancellation delivery failed.") from exc
        return self.status()

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
            self.state = "STOPPED_UNCONFIRMED" if self.stop_latched else "FINISHED"
