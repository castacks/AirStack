"""TCP client for the Isaac-side ScenarioManager (safe_eval_launch_script.py).

The Kit interpreter cannot host an rclpy node (typesupport assert in the
vendored rclpy), so scenario control and the ground-truth stream ride a
JSON-lines TCP socket on the docker network — the same style of link PX4
SITL itself uses to reach the sim. Real stack topics still go through ROS
via the robot-container bridge.
"""

from __future__ import annotations

import json
import socket
import subprocess
import threading
import time
import uuid

SERVER_PORT = 8899


class ScenarioError(RuntimeError):
    pass


def isaac_container_ip(container: str = "isaac-sim") -> str:
    result = subprocess.run(
        ["docker", "inspect", "-f",
         "{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}", container],
        capture_output=True, text=True, timeout=15)
    ip = result.stdout.strip()
    if result.returncode != 0 or not ip:
        raise ScenarioError(f"cannot resolve {container} IP: {result.stderr.strip()}")
    return ip


class ScenarioClient:

    def __init__(self, host: str, port: int = SERVER_PORT):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None
        self._lock = threading.Condition()
        self._state: dict | None = None
        self._responses: dict[str, dict] = {}
        self._closed = False

    # ── lifecycle ─────────────────────────────────────────────────────────

    def connect(self, timeout: float = 300.0) -> None:
        """Retry until the sim's server is up (Isaac boot takes minutes)."""
        deadline = time.monotonic() + timeout
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                self._sock = socket.create_connection((self.host, self.port), timeout=10)
                self._sock.settimeout(None)
                threading.Thread(target=self._reader_loop, daemon=True).start()
                return
            except OSError as e:
                last_err = e
                time.sleep(3.0)
        raise ScenarioError(
            f"ScenarioManager not reachable at {self.host}:{self.port} after "
            f"{timeout:.0f}s ({last_err}) — is safe_eval_launch_script.py the "
            f"ISAAC_SIM_SCRIPT_NAME and still booting/crashed?")

    def close(self) -> None:
        self._closed = True
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def alive(self) -> bool:
        return self._sock is not None and not self._closed

    def _reader_loop(self) -> None:
        assert self._sock is not None
        f = self._sock.makefile("r")
        try:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except (ValueError, TypeError):
                    continue
                with self._lock:
                    if msg.get("kind") == "state":
                        self._state = msg
                    elif msg.get("kind") == "response":
                        self._responses[msg.get("id", "")] = msg
                    self._lock.notify_all()
        except (OSError, ValueError):
            pass
        finally:
            with self._lock:
                self._closed = True
                self._lock.notify_all()

    # ── RPC ───────────────────────────────────────────────────────────────

    def command(self, cmd: str, args: dict | None = None, timeout: float = 30.0) -> dict:
        if not self.alive:
            raise ScenarioError("scenario connection is closed")
        cmd_id = uuid.uuid4().hex[:12]
        payload = json.dumps({"id": cmd_id, "cmd": cmd, "args": args or {}}) + "\n"
        self._sock.sendall(payload.encode())
        deadline = time.monotonic() + timeout
        with self._lock:
            while True:
                resp = self._responses.pop(cmd_id, None)
                if resp is not None:
                    if not resp.get("ok"):
                        raise ScenarioError(f"scenario cmd '{cmd}' failed: {resp.get('error')}")
                    return resp.get("data") or {}
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._closed:
                    raise ScenarioError(f"scenario cmd '{cmd}' timed out after {timeout:.0f}s")
                self._lock.wait(timeout=min(1.0, remaining))

    # ── state access ──────────────────────────────────────────────────────

    def latest_state(self) -> dict | None:
        with self._lock:
            return dict(self._state) if self._state else None

    def wait_state(self, timeout: float = 60.0) -> dict:
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._state is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ScenarioError("no ground-truth state received from the sim")
                self._lock.wait(timeout=min(1.0, remaining))
            return dict(self._state)

    def wait_sim_time(self, target_t: float, timeout: float = 60.0) -> dict:
        deadline = time.monotonic() + timeout
        with self._lock:
            while True:
                if self._state is not None and self._state.get("t", 0.0) >= target_t:
                    return dict(self._state)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ScenarioError(
                        f"sim time did not reach {target_t:.2f}s within {timeout:.0f}s "
                        f"(last t={self._state.get('t') if self._state else None}) — "
                        "sim paused or RTF collapsed")
                self._lock.wait(timeout=min(1.0, remaining))
