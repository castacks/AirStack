"""Host-side handle on the in-container bridge (container_bridge.py).

One `docker exec -i` process per stack bring-up. JSON lines out via stdin,
JSON lines in via stdout — see container_bridge.py for the message kinds.
Publishes to the stack's real topics (global_plan, trajectory_override) and
relays EKF odometry; ground truth and scenario control come from
scenario_client.py instead.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading

from .ros import ros2_env


class BridgeError(RuntimeError):
    pass


class StackBridge:

    def __init__(self, container: str, robot: str = "robot_1", domain_id: int = 1):
        self.container = container
        self.robot = robot
        self.domain_id = domain_id
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Condition()
        self._odom_frame: str | None = None
        self._odom_pos: list | None = None
        self._ready = False
        self._stderr_tail: list[str] = []

    # ── lifecycle ─────────────────────────────────────────────────────────

    def start(self, timeout: float = 60.0) -> None:
        # eval/ is not mounted in the robot container — copy the script in.
        script = os.path.join(os.path.dirname(__file__), "container_bridge.py")
        subprocess.run(["docker", "cp", script, f"{self.container}:/tmp/container_bridge.py"],
                       check=True, capture_output=True, timeout=30)
        cmd = (f"{ros2_env(self.domain_id)} && "
               f"python3 /tmp/container_bridge.py --robot {self.robot}")
        self._proc = subprocess.Popen(
            ["docker", "exec", "-i", self.container, "bash", "-c", cmd],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        threading.Thread(target=self._stdout_loop, daemon=True).start()
        threading.Thread(target=self._stderr_loop, daemon=True).start()
        with self._lock:
            self._lock.wait_for(lambda: self._ready or self._proc.poll() is not None,
                                timeout=timeout)
            if not self._ready:
                raise BridgeError(
                    "container bridge failed to start: "
                    + "".join(self._stderr_tail[-20:]))

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None

    @property
    def alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    # ── reader threads ────────────────────────────────────────────────────

    def _stdout_loop(self):
        assert self._proc is not None
        for line in self._proc.stdout:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except (ValueError, TypeError):
                continue
            kind = msg.get("kind")
            with self._lock:
                if kind == "odom":
                    self._odom_frame = msg.get("frame")
                    self._odom_pos = msg.get("pos")
                elif kind == "ready":
                    self._ready = True
                self._lock.notify_all()

    def _stderr_loop(self):
        assert self._proc is not None
        for line in self._proc.stderr:
            self._stderr_tail.append(line)
            del self._stderr_tail[:-50]

    # ── writes ────────────────────────────────────────────────────────────

    def _send(self, obj: dict) -> None:
        if not self.alive:
            raise BridgeError("container bridge is not running")
        self._proc.stdin.write(json.dumps(obj) + "\n")
        self._proc.stdin.flush()

    def publish_global_plan(self, waypoints) -> None:
        self._send({"kind": "global_plan",
                    "waypoints": [[float(v) for v in wp] for wp in waypoints]})

    def publish_traj_override(self, waypoints, velocity: float) -> None:
        self._send({"kind": "traj_override",
                    "waypoints": [[float(v) for v in wp] for wp in waypoints],
                    "velocity": float(velocity)})

    # ── state access ──────────────────────────────────────────────────────

    def latest_odom_pos(self) -> list | None:
        with self._lock:
            return list(self._odom_pos) if self._odom_pos else None
