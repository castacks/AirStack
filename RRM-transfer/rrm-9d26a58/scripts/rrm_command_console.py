#!/usr/bin/env python3
"""Local task intake, observation, and explicitly admitted Office demo dispatch."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import shutil
import sqlite3
import subprocess
import threading
from urllib.parse import urlsplit
import uuid

from rrm_cosmos_reason2 import load_context
from rrm_import_office import import_bundle
from rrm.execution_supervisor import ExecutionSupervisor, RunningDispatch
from rrm.live_observation import validate_live_observation
from rrm.psc_pipeline import CommandPscBridge, InferenceQueue, PscBridge
from rrm.task_store import TaskStore


def save_request(bundle: Path, output: Path, objective: str, goal_id: str | None = None,
                 *, image: bytes | None = None, observation: dict | None = None) -> dict:
    if not isinstance(objective, str) or not objective.strip() or len(objective) > 5000:
        raise ValueError("Enter a task between 1 and 5000 characters.")
    if goal_id is not None and (not isinstance(goal_id, str) or not re.fullmatch(r"[0-9a-f]{32}", goal_id)):
        raise ValueError("Invalid goal ID.")
    payload = json.loads((bundle / "input.json").read_text())
    request_id = uuid.uuid4().hex
    task_id = f"office-command-{request_id}"
    payload["task"].update(task_id=task_id, revision=f"{task_id}/v1",
                           objective=objective.strip(), issuer_id="command-console",
                           permission_revision="inference-only")
    # New task linkage; static scene catalog facts remain explicitly separate from
    # an optional, timestamped live image/vehicle observation.
    payload["snapshot"].update(task_id=task_id,
        snapshot_id=f"{payload['snapshot']['snapshot_id']}/{request_id}",
        revision=f"{payload['snapshot']['revision']}/{request_id}")
    destination = output / request_id
    destination.mkdir(parents=True, exist_ok=False)
    if observation is not None:
        payload["live_observation"] = observation
        payload["task"]["context_refs"] = [
            *payload["task"].get("context_refs", []),
            f"live-observation:{observation['sha256']}",
        ]
    (destination / "input.json").write_text(json.dumps(payload, indent=2) + "\n")
    load_context(destination / "input.json")
    if image is None:
        shutil.copyfile(bundle / "input.png", destination / "input.png")
    else:
        (destination / "input.png").write_bytes(image)
    if observation is not None:
        (destination / "observation.json").write_text(
            json.dumps(observation, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    manifest = {
        "request_id": request_id, "task_id": task_id,
        "goal_id": goal_id or uuid.uuid4().hex,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "SAVED_NOT_SUBMITTED", "execution_dispatch": False,
        "context_mode": "live-isaac-observation" if observation is not None else "frozen-office-replay",
        "source_bundle": str(bundle),
        "input_sha256": hashlib.sha256((destination / "input.json").read_bytes()).hexdigest(),
        "media_sha256": hashlib.sha256((destination / "input.png").read_bytes()).hexdigest(),
    }
    if observation is not None:
        manifest["observation_sha256"] = hashlib.sha256(
            (destination / "observation.json").read_bytes()
        ).hexdigest()
    (destination / "request.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


class Console:
    def __init__(self, bundle: Path, output: Path, camera_script: str,
                 psc_bridge: PscBridge | None = None):
        self.bundle = bundle.resolve()
        self.output = output.resolve()
        self.decision = import_bundle(self.bundle)
        self.context = json.loads((self.bundle / "input.json").read_text())
        self.store = TaskStore(self.output / "tasks.sqlite3")
        self.storage_lock = threading.Lock()
        self.reference_run_id = self.store.record_reference(self.bundle)
        self.recover_requests()
        self.token = secrets.token_urlsafe(32)
        self.camera_script = camera_script
        self.camera_lock = threading.Lock()
        self.last_stamp = None
        self.latest_camera = None
        self.latest_camera_metadata = None
        scene = json.loads((self.bundle / "scene_manifest.json").read_text())
        self.expected_camera_frame = scene.get("camera_frame_id", "camera_left")
        self.queue = (InferenceQueue(self.store, trusted_scene=self.bundle / "scene_manifest.json",
                                     bridge=psc_bridge) if psc_bridge is not None else None)
        self.execution = ExecutionSupervisor(
            self.decision.proposal,
            self.output / "execution",
            self._launch_dispatch,
        )
        self.store.set_lifecycle(self.reference_run_id,
                                 proposal_sha256=self.execution.proposal_sha256)
        self.index_execution_evidence()

    @staticmethod
    def _canonical_proposal_sha256(path: Path) -> str:
        value = json.loads(path.read_text(encoding="utf-8"))
        return hashlib.sha256((json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()).hexdigest()

    def index_execution_evidence(self) -> None:
        """Index only records cryptographically bound to the reference proposal.

        Reconciliation is a global safety gate for this console's single current
        proposal, so it remains linked to the reference run until the per-run dispatcher
        is enabled in a later step. Nothing is copied or rewritten.
        """
        root = self.output / "execution"
        if not root.is_dir():
            return
        for reconciliation in sorted((root / "reconciliations").glob("*.json")) if (root / "reconciliations").is_dir() else []:
            record = json.loads(reconciliation.read_text())
            self.store.record_event(self.reference_run_id, kind="reconciliation",
                                    artifact_path=reconciliation,
                                    summary={"state": "GROUNDED_RECONCILED",
                                             "reconciliation_id": record.get("reconciliation_id")})
        for directory in sorted(path for path in root.iterdir() if path.is_dir() and path.name != "reconciliations"):
            proposal = directory / "proposal.json"
            if not proposal.is_file() or self._canonical_proposal_sha256(proposal) != self.execution.proposal_sha256:
                continue
            for filename, kind, fields in (
                ("admission.json", "admission", ("dispatch_id", "decision", "execution_requested")),
                ("outcome.json", "dispatch_outcome", ("verdict", "return_code", "physical_stop_verified")),
                ("operator-land.json", "landing_admission", ("dispatch_id", "decision", "execution_requested")),
            ):
                path = directory / filename
                if path.is_file():
                    record = json.loads(path.read_text())
                    self.store.record_event(self.reference_run_id, kind=kind, artifact_path=path,
                                            summary={field: record.get(field) for field in fields})
            for path in sorted(directory.glob("stop-*.json")):
                record = json.loads(path.read_text())
                self.store.record_event(self.reference_run_id, kind="stop_request", artifact_path=path,
                                        summary={key: record.get(key) for key in
                                                 ("stop_generation", "cancel_requested", "physical_stop_verified", "safety_claim")})
            for path in sorted(directory.glob("stop-delivery-*.json")):
                record = json.loads(path.read_text())
                self.store.record_event(self.reference_run_id, kind="stop_delivery", artifact_path=path,
                                        summary={key: record.get(key) for key in
                                                 ("stop_generation", "signal_delivery_latency_ms")})

    def _launch_dispatch(self, dispatch_id: str, run_dir: Path,
                         proposal_path: Path) -> RunningDispatch:
        """Stage and launch the existing ActionClient-only adapter in the robot container."""
        container = "airstack-robot-desktop-1"
        remote_root = f"/tmp/rrm-console-dispatch-{dispatch_id}"
        remote_source = remote_root + "/source"
        remote_proposal = remote_root + "/proposal.json"
        remote_outcome = remote_root + "/outcome.json"
        remote_pid = remote_root + "/dispatcher.pid"
        source_root = Path(__file__).resolve().parents[1]
        subprocess.run(["docker", "exec", container, "mkdir", "-p", remote_source],
                       check=True, capture_output=True, timeout=10)
        subprocess.run(["docker", "cp", str(source_root) + "/.",
                        f"{container}:{remote_source}"],
                       check=True, capture_output=True, timeout=30)
        subprocess.run(["docker", "cp", str(proposal_path),
                        f"{container}:{remote_proposal}"],
                       check=True, capture_output=True, timeout=10)
        command = (
            "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
            f"echo \"$$\" > {remote_pid}; "
            f"export PYTHONPATH=/tmp/rrm-canonical-deps:{remote_source}:$PYTHONPATH; "
            "exec python3 "
            f"{remote_source}/scripts/airstack_drone_dispatch.py "
            f"--proposal-json {remote_proposal} --execute --verify-observation "
            f"--observation-timeout-s 10 --max-observation-age-s 2 "
            f"--action-timeout-s 120 --outcome-json {remote_outcome}"
        )
        log_handle = (run_dir / "dispatcher.log").open("wb")
        try:
            process = subprocess.Popen(
                ["docker", "exec", "-e", "ROS_DOMAIN_ID=1", container,
                 "bash", "-lc", command],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
            )
        except Exception:
            log_handle.close()
            raise

        def request_stop() -> None:
            pid_result = subprocess.run(
                ["docker", "exec", container, "cat", remote_pid],
                check=True, capture_output=True, text=True, timeout=5,
            )
            pid = pid_result.stdout.strip()
            if not pid.isdigit():
                raise RuntimeError("dispatcher PID record is invalid")
            subprocess.run(["docker", "exec", container, "kill", "-INT", pid],
                           check=True, capture_output=True, timeout=5)

        def finalize(return_code: int) -> dict:
            log_handle.close()
            outcome_path = run_dir / "outcome.json"
            exists = subprocess.run(
                ["docker", "exec", container, "test", "-r", remote_outcome],
                capture_output=True, timeout=5,
            ).returncode == 0
            if exists:
                subprocess.run(["docker", "cp", f"{container}:{remote_outcome}",
                                str(outcome_path)],
                               check=True, capture_output=True, timeout=10)
                result = json.loads(outcome_path.read_text(encoding="utf-8"))
                result["return_code"] = return_code
                return result
            return {"return_code": return_code, "verdict": "UNCONFIRMED",
                    "reason": "dispatcher_produced_no_outcome_record"}

        return RunningDispatch(process=process, request_stop=request_stop, finalize=finalize)

    def recover_requests(self):
        """Idempotently index older folders or requests saved before an interrupted DB write."""
        for path in sorted(self.output.glob("*/request.json")):
            if not re.fullmatch(r"[0-9a-f]{32}", path.parent.name):
                continue
            manifest = json.loads(path.read_text())
            if manifest["request_id"] != path.parent.name:
                raise ValueError("Request folder/manifest mismatch.")
            load_context(path.parent / "input.json")
            for key, filename in (("input_sha256", "input.json"), ("media_sha256", "input.png")):
                if hashlib.sha256((path.parent / filename).read_bytes()).hexdigest() != manifest[key]:
                    raise ValueError(f"Request checksum mismatch: {path.parent.name}/{filename}")
            if manifest.get("context_mode") == "live-isaac-observation":
                observation = path.parent / "observation.json"
                if not observation.is_file() or hashlib.sha256(observation.read_bytes()).hexdigest() != manifest.get("observation_sha256"):
                    raise ValueError(f"Request checksum mismatch: {path.parent.name}/observation.json")
            self.store.record_request(manifest, json.loads((path.parent / "input.json").read_text()), path.parent)

    def save(self, objective, goal_id=None):
        with self.storage_lock:
            if goal_id is not None:
                goal = self.store.get_goal(goal_id)
                if not isinstance(objective, str) or objective.strip() != goal["objective"]:
                    raise ValueError("Edited instructions must be saved as a new goal.")
                task = self.context["task"]
                if (goal["constraints_revision"] != task["constraints_revision"] or
                        goal["embodiment_id"] != task["requested_embodiment_id"]):
                    raise ValueError("Saved goal context differs; save a new goal for this context.")
            if self.latest_camera is None or self.latest_camera_metadata is None:
                raise ValueError("Refresh the live Isaac camera twice before saving an inference request.")
            observation = validate_live_observation(
                self.latest_camera_metadata, expected_camera_frame=self.expected_camera_frame
            )
            if hashlib.sha256(self.latest_camera).hexdigest() != observation["sha256"]:
                raise ValueError("Live camera image does not match its capture metadata.")
            manifest = save_request(self.bundle, self.output, objective, goal_id,
                                    image=self.latest_camera, observation=observation)
            directory = self.output / manifest["request_id"]
            self.store.record_request(manifest, json.loads((directory / "input.json").read_text()), directory)
            self.store.record_event(manifest["request_id"], kind="live_observation",
                                    artifact_path=directory / "observation.json",
                                    summary={
                                        "frame_id": observation["frame_id"],
                                        "source_stamp_ns": observation["source_stamp_ns"],
                                        "connected": observation["vehicle"]["connected"],
                                    })
            return manifest

    def submit_to_psc(self, run_id: str) -> dict:
        if self.queue is None:
            raise RuntimeError("PSC bridge is not configured; no credentials are accepted by this console.")
        run = self.store.get_run(run_id)
        if run is None:
            raise ValueError("Saved run not found.")
        directory = Path(run["artifact_dir"])
        manifest = json.loads((directory / "request.json").read_text())
        observation_path = directory / "observation.json"
        if manifest.get("context_mode") != "live-isaac-observation" or not observation_path.is_file():
            raise ValueError("PSC submission requires a fresh live Isaac observation, not a frozen replay.")
        observation = json.loads(observation_path.read_text())
        if hashlib.sha256(observation_path.read_bytes()).hexdigest() != manifest.get("observation_sha256"):
            raise ValueError("Live observation evidence checksum mismatch.")
        if hashlib.sha256((directory / "input.png").read_bytes()).hexdigest() != observation.get("sha256"):
            raise ValueError("Live observation image checksum mismatch.")
        validate_live_observation(observation, expected_camera_frame=self.expected_camera_frame)
        return self.queue.submit(run_id)

    def approve_candidate(self, run_id: str, proposal_sha256: str) -> dict:
        """Record explicit review of one validated model result; never dispatch here."""
        run = self.store.get_run(run_id)
        if run is None:
            raise ValueError("Saved run not found.")
        if run["status"] != "CANDIDATE_ACCEPTED" or run["execution_state"] != "REVIEW_REQUIRED":
            raise ValueError("Only a validated, unreviewed candidate can be approved.")
        proposal = Path(run["artifact_dir"]) / "psc-result" / "proposal.json"
        if not proposal.is_file():
            raise ValueError("Validated proposal evidence is missing.")
        actual = hashlib.sha256(proposal.read_bytes()).hexdigest()
        if proposal_sha256 != actual or actual != run["proposal_sha256"]:
            raise ValueError("Proposal changed; reload the candidate before approving.")
        approval = proposal.with_name("approval.json")
        approval.write_text(json.dumps({
            "schema_version": "rrm-run-approval/v1", "run_id": run_id,
            "approved_at": datetime.now(timezone.utc).isoformat(),
            "proposal_sha256": actual, "decision": "APPROVE_FOR_PUBLIC_ACTION_REVIEW",
            "execution_dispatch": False,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.store.set_lifecycle(run_id, execution_state="APPROVED")
        self.store.record_event(run_id, kind="candidate_approval", artifact_path=approval,
                                summary={"proposal_sha256": actual, "execution_dispatch": False})
        return self.store.get_run(run_id)

    def capture(self):
        if not self.camera_lock.acquire(blocking=False):
            raise ValueError("A camera capture is already in progress.")
        try:
            capture_id = uuid.uuid4().hex
            remote = f"/tmp/rrm-console-{capture_id}.png"
            command = ["docker", "exec", "airstack-robot-desktop-1", "bash", "-c",
                'source /root/AirStack/robot/ros_ws/install/local_setup.bash; '
                'exec timeout 12 python3 "$1" --topic '
                '/robot_1/sensors/front_stereo/left/image_rect '
                '--odometry-topic /robot_1/odometry_conversion/odometry '
                '--output "$2" --timeout-s 8',
                "rrm-camera", self.camera_script, remote]
            subprocess.run(command, check=True, capture_output=True, timeout=16)
            image = subprocess.run(["docker", "exec", "airstack-robot-desktop-1",
                "cat", remote], check=True, capture_output=True, timeout=5).stdout
            metadata = json.loads(subprocess.run(["docker", "exec", "airstack-robot-desktop-1",
                "cat", remote + ".json"], check=True, capture_output=True, timeout=5).stdout)
            if hashlib.sha256(image).hexdigest() != metadata["sha256"]:
                raise ValueError("Camera image checksum mismatch.")
            metadata["source_stamp_advanced"] = (None if self.last_stamp is None else
                metadata["source_stamp_ns"] != self.last_stamp)
            self.last_stamp = metadata["source_stamp_ns"]
            metadata["captured_at"] = datetime.now(timezone.utc).isoformat()
            self.latest_camera = image
            self.latest_camera_metadata = metadata
            return metadata
        finally:
            self.camera_lock.release()

    def reconcile_grounded(self):
        observer = "/tmp/rrm-grounded-state-observer.py"
        source = Path(__file__).with_name("airstack_vehicle_observe.py")
        subprocess.run(
            ["docker", "cp", str(source), f"airstack-robot-desktop-1:{observer}"],
            check=True, capture_output=True, timeout=10,
        )
        command = (
            "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
            f"exec python3 {observer} --timeout-s 10 --minimum-odometry-samples 3"
        )
        completed = subprocess.run(
            ["docker", "exec", "-e", "ROS_DOMAIN_ID=1", "airstack-robot-desktop-1",
             "bash", "-lc", command],
            check=True, capture_output=True, text=True, timeout=15,
        )
        lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
        if not lines:
            raise RuntimeError("The drone state could not be read.")
        return self.execution.reconcile_grounded(json.loads(lines[-1]))


def make_handler(app: Console):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, data, mime="application/json", status=200):
            if not isinstance(data, bytes):
                data = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                return self.respond((Path(__file__).parent / "ui/command_console.html").read_bytes(),
                                    "text/html; charset=utf-8")
            if path == "/api/state":
                return self.respond({"token": app.token, "context": app.context,
                                     "decision": app.decision.model_dump(mode="json")})
            if path == "/api/goals":
                app.index_execution_evidence()
                return self.respond({"goals": app.store.history()})
            if path == "/api/execution":
                return self.respond(app.execution.status())
            if path == "/reference.png":
                return self.respond((app.bundle / "input.png").read_bytes(), "image/png")
            if path == "/camera.png" and app.latest_camera is not None:
                return self.respond(app.latest_camera, "image/png")
            run_match = re.fullmatch(r"/runs/([0-9a-f]{32})/(input\.json|input\.png|request\.json|observation\.json|result\.json)", path)
            if run_match:
                run = app.store.get_run(run_match[1])
                if run:
                    target = Path(run["artifact_dir"]) / run_match[2]
                    if run_match[2] == "result.json" and not target.is_file():
                        target = Path(run["artifact_dir"]) / "psc-result" / "result.json"
                    if target.is_file():
                        return self.respond(target.read_bytes(),
                            "image/png" if target.suffix == ".png" else "application/json")
            evidence_match = re.fullmatch(r"/runs/([0-9a-f]{32})/evidence/([0-9a-f]{32})", path)
            if evidence_match:
                event = app.store.get_event(evidence_match[1], evidence_match[2])
                if event:
                    target = Path(event["artifact_path"])
                    if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == event["sha256"]:
                        return self.respond(target.read_bytes(), "application/json")
            match = re.fullmatch(r"/requests/([0-9a-f]{32})/(input\.json|input\.png|request\.json)", path)
            if match:
                target = app.output / match[1] / match[2]
                if target.is_file():
                    return self.respond(target.read_bytes(),
                        "image/png" if target.suffix == ".png" else "application/json")
            self.respond({"error": "Not found"}, status=404)

        def do_POST(self):
            # manual-import endpoint does not require the X-RRM-Token because it comes from a local terminal
            path = urlsplit(self.path).path
            if path == "/api/requests/manual-import":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    payload = json.loads(self.rfile.read(length))
                    run_id = payload.get("run_id")
                    job_id = payload.get("job_id")
                    bundle_dir = payload.get("bundle_dir")
                    if not all(isinstance(x, str) and x for x in (run_id, job_id, bundle_dir)):
                        return self.respond({"error": "Missing manual import fields"}, status=400)
                    if app.queue is None:
                        return self.respond({"error": "PSC bridge disabled"}, status=400)
                    run = app.store.get_run(run_id)
                    if run is None:
                        return self.respond({"error": "Saved run not found"}, status=400)
                    request_dir = Path(run["artifact_dir"]).resolve()
                    app.store.set_lifecycle(run_id, status="INFERENCE_RUNNING", psc_job_id=job_id)
                    app.queue._import_result(run_id, request_dir, Path(bundle_dir))
                    return self.respond({"status": "imported"})
                except Exception as e:
                    return self.respond({"error": str(e)}, status=400)

            if not secrets.compare_digest(self.headers.get("X-RRM-Token", ""), app.token):
                return self.respond({"error": "Reload this page before submitting."}, status=403)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 24000:
                    raise ValueError("Invalid request size.")
                value = json.loads(self.rfile.read(length))
                if not isinstance(value, dict):
                    raise ValueError("Request must be an object.")
                if self.path == "/api/requests":
                    return self.respond(app.save(value.get("objective"), value.get("goal_id")), status=201)
                submit_match = re.fullmatch(r"/api/runs/([0-9a-f]{32})/submit", self.path)
                if submit_match:
                    return self.respond(app.submit_to_psc(submit_match[1]), status=202)
                approval_match = re.fullmatch(r"/api/runs/([0-9a-f]{32})/approval", self.path)
                if approval_match:
                    return self.respond(app.approve_candidate(
                        approval_match[1], value.get("proposal_sha256")
                    ))
                if self.path == "/api/camera":
                    return self.respond(app.capture())
                if self.path == "/api/admission":
                    return self.respond(app.execution.decide(
                        value.get("decision"), value.get("proposal_sha256")
                    ))
                if self.path == "/api/stop":
                    return self.respond(app.execution.request_stop())
                if self.path == "/api/land":
                    return self.respond(app.execution.request_land())
                if self.path == "/api/reconcile":
                    return self.respond(app.reconcile_grounded())
                if self.path == "/api/reset":
                    import subprocess
                    try:
                        subprocess.run(["./airstack.sh", "down", "isaac-sim-livestream", "robot-desktop"], cwd="/root/AirStack", check=True)
                        subprocess.run(["./airstack.sh", "up", "--sim", "isaac", "--scene", "office", "--wait"], cwd="/root/AirStack", check=True)
                        return self.respond({"status": "reset complete"})
                    except subprocess.SubprocessError as e:
                        return self.respond({"error": f"Failed to reset: {str(e)}"}, status=500)
                self.respond({"error": "Not found"}, status=404)
            except (ValueError, TypeError) as error:
                self.respond({"error": str(error)}, status=400)
            except RuntimeError as error:
                self.respond({"error": str(error)}, status=409)
            except sqlite3.Error:
                self.respond({"error": "Task database unavailable. Saved artifacts are retained; restart to reindex."}, status=503)
            except (subprocess.SubprocessError, OSError):
                self.respond({"error": "Camera or storage unavailable. Use Foxglove and try again."}, status=503)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--camera-script", required=True, help="Capture utility path inside robot container")
    parser.add_argument("--psc-bridge", nargs="+", default=None,
                        help="non-interactive bridge command; appends the immutable request directory")
    parser.add_argument("--port", type=int, default=8787)
    args = parser.parse_args()
    bridge = CommandPscBridge(args.psc_bridge) if args.psc_bridge else None
    app = Console(args.bundle, args.output_dir, args.camera_script, psc_bridge=bridge)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(app))
    print(f"RRM console: http://127.0.0.1:{server.server_port} — intake, review, and gated execution", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
