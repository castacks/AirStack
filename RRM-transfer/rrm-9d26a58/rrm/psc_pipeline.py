"""Asynchronous, no-credential PSC inference boundary for one immutable run.

The configured bridge runs outside the browser security boundary. It receives a single
request directory and returns JSON only after it has staged, run, fetched and verified
one PSC bundle. This module never imports ROS or launches a vehicle action.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Protocol

from rrm.task_store import TaskStore


class PscBridge(Protocol):
    def run(self, request_dir: Path) -> dict: ...


class CommandPscBridge:
    """Run an operator-provided non-interactive bridge without a shell.

    The command receives the immutable request directory as its final argument and
    writes one JSON object to stdout containing `job_id` and `bundle_dir`. Credentials
    belong to the configured process/agent, never to the HTTP API or its database.
    """
    def __init__(self, argv: list[str]):
        if not argv or any(not isinstance(item, str) or not item for item in argv):
            raise ValueError("PSC bridge command is invalid.")
        self.argv = tuple(argv)

    def run(self, request_dir: Path) -> dict:
        completed = subprocess.run([*self.argv, str(request_dir)], check=True,
                                   capture_output=True, text=True, timeout=3600)
        lines = [line for line in completed.stdout.splitlines() if line.strip()]
        if not lines:
            raise RuntimeError("PSC bridge returned no result record.")
        result = json.loads(lines[-1])
        if not isinstance(result, dict):
            raise RuntimeError("PSC bridge result is not an object.")
        return result


class InferenceQueue:
    """One process-local worker per saved run; remote PSC jobs remain durable evidence."""
    def __init__(self, store: TaskStore, *, trusted_scene: Path, bridge: PscBridge | None):
        self.store = store
        self.trusted_scene = trusted_scene.resolve()
        self.bridge = bridge
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="rrm-psc")

    def submit(self, run_id: str) -> dict:
        if self.bridge is None:
            raise RuntimeError("PSC bridge is not configured.")
        run = self.store.get_run(run_id)
        if run is None:
            raise ValueError("Saved run not found.")
        if run["status"] != "SAVED_NOT_SUBMITTED":
            raise ValueError("Only a newly saved run can be submitted to PSC.")
        request_dir = Path(run["artifact_dir"]).resolve()
        if not (request_dir / "request.json").is_file():
            raise ValueError("Saved request evidence is incomplete.")
        self.store.set_lifecycle(run_id, status="INFERENCE_QUEUED")
        submission = request_dir / "psc-submission.json"
        submission.write_text(json.dumps({
            "schema_version": "rrm-psc-submission/v1", "run_id": run_id,
            "input_sha256": _sha256(request_dir / "input.json"),
            "media_sha256": _sha256(request_dir / "input.png"),
            "execution_dispatch": False,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.store.record_event(run_id, kind="psc_submission", artifact_path=submission,
                                summary={"state": "QUEUED", "execution_dispatch": False})
        self.executor.submit(self._run, run_id, request_dir)
        return self.store.get_run(run_id)

    def _run(self, run_id: str, request_dir: Path) -> None:
        self.store.set_lifecycle(run_id, status="INFERENCE_RUNNING")
        try:
            response = self.bridge.run(request_dir)
            job_id = response.get("job_id")
            bundle_dir = response.get("bundle_dir")
            if not isinstance(job_id, str) or not job_id.strip():
                raise RuntimeError("PSC bridge result lacks job_id.")
            if not isinstance(bundle_dir, str) or not bundle_dir:
                raise RuntimeError("PSC bridge result lacks bundle_dir.")
            self.store.set_lifecycle(run_id, status="INFERENCE_RUNNING", psc_job_id=job_id)
            receipt = request_dir / "psc-receipt.json"
            receipt.write_text(json.dumps({
                "schema_version": "rrm-psc-receipt/v1", "run_id": run_id,
                "job_id": job_id, "bundle_dir": bundle_dir, "execution_dispatch": False,
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            self.store.record_event(run_id, kind="psc_receipt", artifact_path=receipt,
                                    summary={"job_id": job_id, "state": "RESULT_RECEIVED"})
            self._import_result(run_id, request_dir, Path(bundle_dir))
        except Exception as error:
            self.record_failure(run_id, request_dir, error)

    def record_failure(self, run_id: str, request_dir: Path, error: Exception) -> None:
        """Persist an import failure so a completed PSC job never looks in-progress."""
        failure = request_dir / "psc-failure.json"
        reason = _safe_error_detail(error)
        failure.write_text(json.dumps({
            "schema_version": "rrm-psc-failure/v2", "run_id": run_id,
            "error": type(error).__name__, "reason": reason,
            "execution_dispatch": False,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.store.record_event(run_id, kind="psc_failure", artifact_path=failure,
                                summary={"state": "FAILED", "reason": reason})
        self.store.set_lifecycle(run_id, status="INFERENCE_FAILED",
                                 execution_state="NOT_DISPATCHED")

    def _record_rejection(self, run_id: str, request_dir: Path, *, candidate_status,
                          reason: str, detail: str | None = None) -> None:
        rejected = request_dir / "psc-rejected.json"
        record = {
            "schema_version": "rrm-psc-rejected/v2", "run_id": run_id,
            "candidate_status": candidate_status, "reason": reason,
            "execution_dispatch": False,
        }
        if detail:
            record["detail"] = detail
        rejected.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.store.record_event(run_id, kind="psc_rejected", artifact_path=rejected,
                                summary={"state": "REJECTED", "reason": reason})
        self.store.set_lifecycle(run_id, status="CANDIDATE_REJECTED",
                                 execution_state="NOT_DISPATCHED")

    def _import_result(self, run_id: str, request_dir: Path, source: Path) -> None:
        source = source.resolve()
        required = ("input.json", "input.png", "scene_manifest.json", "result.json")
        if not source.is_dir() or any(not (source / item).is_file() for item in required):
            raise ValueError("PSC result bundle is incomplete.")
        if _sha256(source / "input.json") != _sha256(request_dir / "input.json"):
            raise ValueError("PSC result input does not match the saved run.")
        if _sha256(source / "input.png") != _sha256(request_dir / "input.png"):
            raise ValueError("PSC result image does not match the saved run.")
        source_scene = json.loads((source / "scene_manifest.json").read_text())
        trusted_scene = json.loads(self.trusted_scene.read_text())
        # Older reviewed reference bundles predate the additive camera frame field.
        # Frame identity is independently bound by live_observation validation; it
        # must not make an otherwise identical reviewed scene fail import.
        source_scene.pop("camera_frame_id", None)
        trusted_scene.pop("camera_frame_id", None)
        if source_scene != trusted_scene:
            raise ValueError("PSC result scene binding is not the reviewed scene.")
        result = json.loads((source / "result.json").read_text())
        if result.get("execution_dispatch") is not False:
            raise ValueError("PSC result must not request execution.")
        if result.get("candidate", {}).get("status") != "ACCEPTED":
            self._record_rejection(run_id, request_dir,
                                   candidate_status=result.get("candidate", {}).get("status"),
                                   reason="model_candidate_not_accepted")
            return
        destination = request_dir / "psc-result"
        from rrm_import_office import import_bundle

        # Compile before retaining any new evidence. This prevents an adapter refusal
        # from leaving a raw partial `psc-result` that later looks like an import bug.
        try:
            decision = import_bundle(source)
        except ValueError as error:
            detail = str(error)
            if detail.startswith("adapter refused plan:"):
                self._record_rejection(run_id, request_dir, candidate_status="ACCEPTED",
                                       reason="adapter_refused_plan", detail=detail)
                return
            raise

        if destination.exists():
            if not destination.is_dir():
                raise ValueError("Retained PSC result path is not a directory.")
            required_retained = (*required, "decision.json", "proposal.json")
            if all((destination / item).is_file() for item in required_retained):
                if _sha256(destination / "result.json") != _sha256(source / "result.json"):
                    raise ValueError("Retained PSC result conflicts with this immutable result.")
                expected_proposal = decision.proposal.model_dump_json(indent=2) + "\n"
                if (destination / "proposal.json").read_text(encoding="utf-8") != expected_proposal:
                    raise ValueError("Retained PSC proposal conflicts with this immutable result.")
                self._record_accepted_result(run_id, destination)
                return
            # An old interrupted version may have retained only raw files. It is safe
            # to finish only when every immutable file exactly matches this fetch.
            if any(not (destination / item).is_file() or
                   _sha256(destination / item) != _sha256(source / item) for item in required):
                raise ValueError("Partial retained PSC result conflicts with this immutable result.")
            retained = destination
        else:
            stage_root = Path(tempfile.mkdtemp(prefix=".psc-result-", dir=request_dir))
            retained = stage_root / "bundle"
            try:
                shutil.copytree(source, retained)
                self._write_accepted_result(run_id, retained, decision)
                retained.replace(destination)
            finally:
                shutil.rmtree(stage_root, ignore_errors=True)
            return
        self._write_accepted_result(run_id, retained, decision)

    def _write_accepted_result(self, run_id: str, destination: Path, decision) -> None:
        decision_path = destination / "decision.json"
        proposal_path = destination / "proposal.json"
        decision_path.write_text(decision.model_dump_json(indent=2) + "\n", encoding="utf-8")
        proposal_path.write_text(decision.proposal.model_dump_json(indent=2) + "\n", encoding="utf-8")
        self._record_accepted_result(run_id, destination)

    def _record_accepted_result(self, run_id: str, destination: Path) -> None:
        proposal_path = destination / "proposal.json"
        candidate_digest = _sha256(destination / "result.json")
        proposal_digest = _sha256(proposal_path)
        self.store.set_lifecycle(run_id, status="CANDIDATE_ACCEPTED",
                                 execution_state="REVIEW_REQUIRED",
                                 candidate_sha256=candidate_digest,
                                 proposal_sha256=proposal_digest)
        self.store.record_event(run_id, kind="psc_result", artifact_path=destination / "result.json",
                                summary={"state": "CANDIDATE_ACCEPTED", "candidate_sha256": candidate_digest})
        self.store.record_event(run_id, kind="proposal", artifact_path=proposal_path,
                                summary={"proposal_sha256": proposal_digest, "review_required": True})


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _safe_error_detail(error: Exception) -> str:
    """Keep a bounded, operator-useful failure explanation out of untrusted output."""
    detail = str(error).strip().replace("\n", " ")
    return detail[:240] if detail else type(error).__name__
