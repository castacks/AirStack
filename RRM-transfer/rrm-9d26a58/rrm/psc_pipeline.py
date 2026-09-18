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
    def __init__(self, store: TaskStore, *, trusted_scene: Path, bridge: PscBridge):
        self.store = store
        self.trusted_scene = trusted_scene.resolve()
        self.bridge = bridge
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="rrm-psc")

    def submit(self, run_id: str) -> dict:
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
            failure = request_dir / "psc-failure.json"
            failure.write_text(json.dumps({
                "schema_version": "rrm-psc-failure/v1", "run_id": run_id,
                "error": type(error).__name__, "execution_dispatch": False,
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            self.store.record_event(run_id, kind="psc_failure", artifact_path=failure,
                                    summary={"state": "FAILED", "reason": type(error).__name__})
            self.store.set_lifecycle(run_id, status="INFERENCE_FAILED")

    def _import_result(self, run_id: str, request_dir: Path, source: Path) -> None:
        source = source.resolve()
        required = ("input.json", "input.png", "scene_manifest.json", "result.json")
        if not source.is_dir() or any(not (source / item).is_file() for item in required):
            raise ValueError("PSC result bundle is incomplete.")
        if _sha256(source / "input.json") != _sha256(request_dir / "input.json"):
            raise ValueError("PSC result input does not match the saved run.")
        if _sha256(source / "input.png") != _sha256(request_dir / "input.png"):
            raise ValueError("PSC result image does not match the saved run.")
        if json.loads((source / "scene_manifest.json").read_text()) != json.loads(self.trusted_scene.read_text()):
            raise ValueError("PSC result scene binding is not the reviewed scene.")
        result = json.loads((source / "result.json").read_text())
        if result.get("execution_dispatch") is not False:
            raise ValueError("PSC result must not request execution.")
        if result.get("candidate", {}).get("status") != "ACCEPTED":
            rejected = request_dir / "psc-rejected.json"
            rejected.write_text(json.dumps({
                "schema_version": "rrm-psc-rejected/v1", "run_id": run_id,
                "candidate_status": result.get("candidate", {}).get("status"),
                "execution_dispatch": False,
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            self.store.record_event(run_id, kind="psc_rejected", artifact_path=rejected,
                                    summary={"state": "REJECTED"})
            self.store.set_lifecycle(run_id, status="CANDIDATE_REJECTED")
            return
        destination = request_dir / "psc-result"
        if destination.exists():
            raise ValueError("PSC result already exists for this immutable run.")
        shutil.copytree(source, destination)
        # Import after copy: the evidence indexed below is the retained local copy.
        from rrm_import_office import import_bundle
        decision = import_bundle(destination)
        decision_path = destination / "decision.json"
        proposal_path = destination / "proposal.json"
        decision_path.write_text(decision.model_dump_json(indent=2) + "\n", encoding="utf-8")
        proposal_path.write_text(decision.proposal.model_dump_json(indent=2) + "\n", encoding="utf-8")
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
