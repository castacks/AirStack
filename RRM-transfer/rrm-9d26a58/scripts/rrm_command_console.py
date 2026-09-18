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
from rrm.task_store import TaskStore


def save_request(bundle: Path, output: Path, objective: str, goal_id: str | None = None) -> dict:
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
    # New task linkage, same frozen observation content and replay clock.
    payload["snapshot"].update(task_id=task_id,
        snapshot_id=f"{payload['snapshot']['snapshot_id']}/{request_id}",
        revision=f"{payload['snapshot']['revision']}/{request_id}")
    destination = output / request_id
    destination.mkdir(parents=True, exist_ok=False)
    (destination / "input.json").write_text(json.dumps(payload, indent=2) + "\n")
    load_context(destination / "input.json")
    shutil.copyfile(bundle / "input.png", destination / "input.png")
    manifest = {
        "request_id": request_id, "task_id": task_id,
        "goal_id": goal_id or uuid.uuid4().hex,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "SAVED_NOT_SUBMITTED", "execution_dispatch": False,
        "context_mode": "frozen-office-replay", "source_bundle": str(bundle),
        "input_sha256": hashlib.sha256((destination / "input.json").read_bytes()).hexdigest(),
        "media_sha256": hashlib.sha256((destination / "input.png").read_bytes()).hexdigest(),
    }
    (destination / "request.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


class Console:
    def __init__(self, bundle: Path, output: Path, camera_script: str):
        self.bundle = bundle.resolve()
        self.output = output.resolve()
        self.decision = import_bundle(self.bundle)
        self.context = json.loads((self.bundle / "input.json").read_text())
        self.store = TaskStore(self.output / "tasks.sqlite3")
        self.storage_lock = threading.Lock()
        self.store.record_reference(self.bundle)
        self.recover_requests()
        self.token = secrets.token_urlsafe(32)
        self.camera_script = camera_script
        self.camera_lock = threading.Lock()
        self.last_stamp = None
        self.latest_camera = None
        self.execution = ExecutionSupervisor(
            self.decision.proposal,
            self.output / "execution",
            self._launch_dispatch,
        )

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
            manifest = save_request(self.bundle, self.output, objective, goal_id)
            directory = self.output / manifest["request_id"]
            self.store.record_request(manifest, json.loads((directory / "input.json").read_text()), directory)
            return manifest

    def capture(self):
        if not self.camera_lock.acquire(blocking=False):
            raise ValueError("A camera capture is already in progress.")
        try:
            capture_id = uuid.uuid4().hex
            remote = f"/tmp/rrm-console-{capture_id}.png"
            command = ["docker", "exec", "airstack-robot-desktop-1", "bash", "-c",
                'source /root/AirStack/robot/ros_ws/install/local_setup.bash; '
                'exec timeout 12 python3 "$1" --topic '
                '/robot_1/sensors/front_stereo/left/image_rect --output "$2" --timeout-s 8',
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
                return self.respond({"goals": app.store.history()})
            if path == "/api/execution":
                return self.respond(app.execution.status())
            if path == "/reference.png":
                return self.respond((app.bundle / "input.png").read_bytes(), "image/png")
            if path == "/camera.png" and app.latest_camera is not None:
                return self.respond(app.latest_camera, "image/png")
            run_match = re.fullmatch(r"/runs/([0-9a-f]{32})/(input\.json|input\.png|request\.json|result\.json)", path)
            if run_match:
                run = app.store.get_run(run_match[1])
                if run:
                    target = Path(run["artifact_dir"]) / run_match[2]
                    if target.is_file():
                        return self.respond(target.read_bytes(),
                            "image/png" if target.suffix == ".png" else "application/json")
            match = re.fullmatch(r"/requests/([0-9a-f]{32})/(input\.json|input\.png|request\.json)", path)
            if match:
                target = app.output / match[1] / match[2]
                if target.is_file():
                    return self.respond(target.read_bytes(),
                        "image/png" if target.suffix == ".png" else "application/json")
            self.respond({"error": "Not found"}, status=404)

        def do_POST(self):
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
    parser.add_argument("--port", type=int, default=8787)
    args = parser.parse_args()
    app = Console(args.bundle, args.output_dir, args.camera_script)
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
