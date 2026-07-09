"""AirStack × S.A.F.E. benchmark agents.

Each Policy subclass corresponds to a different AirStack planner configuration.
The AirstackDROAN and AirstackSuperPlanner agents are ROS bridge policies: they
publish SAFE's observations as ROS topics, then block waiting for the planner's
velocity output and return that as the SAFE action.

The bridge communicates with a sidecar process (eval/agent_hooks/*.sh) running
the AirStack Docker containers for that planner.  The sidecar protocol is
newline-delimited JSON over TCP (same as aerial_nav DYNUS/EgoSwarm patterns).

Pure-Python baselines (Random, Aggressive, Conservative) are also provided
for sanity checks and comparison without any Docker sidecar.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from typing import Any

import numpy as np

from safe_core.core.policy import Policy

# Heuristic baselines are shared with aerial_nav — import, don't duplicate.
from integrations.aerial_nav.agents.baselines import (  # noqa: F401
    Random, Aggressive, Conservative, Adaptive, AdaptiveApf,
)


# ── TCP sidecar helpers ────────────────────────────────────────────────────────

class _TCPSidecarClient:
    """Minimal newline-JSON client for communicating with the ROS bridge sidecar.

    Connection is lazy: the first call() attempt triggers the connect loop so
    that instantiating the agent does not block even if the sidecar hook script
    hasn't started yet (the benchmark framework builds agents before running hooks).
    """

    def __init__(self, host: str, port: int, connect_timeout: float = 60.0,
                 call_timeout: float = 1.0):
        self._host = host
        self._port = port
        self._connect_timeout = connect_timeout
        self._call_timeout = call_timeout
        self._sock: socket.socket | None = None
        self._rfile = None
        self._wfile = None
        self._lock = threading.Lock()
        self._connected = False

    def _connect(self) -> None:
        """Block until connected or timeout. Called lazily on first call()."""
        deadline = time.monotonic() + self._connect_timeout
        print(
            f"[AirstackAgent] Connecting to sidecar at {self._host}:{self._port} "
            f"(timeout {self._connect_timeout:.0f}s) …",
            flush=True,
        )
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection((self._host, self._port), timeout=5.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sock = s
                self._rfile = s.makefile("rb")
                self._wfile = s.makefile("wb")
                self._connected = True
                print(f"[AirstackAgent] Connected to sidecar at {self._host}:{self._port}", flush=True)
                return
            except OSError:
                time.sleep(1.0)
        raise RuntimeError(
            f"[AirstackAgent] Could not connect to sidecar at {self._host}:{self._port} "
            f"within {self._connect_timeout:.0f}s — start the hook script first:\n"
            f"  bash eval/agent_hooks/<agent>.sh"
        )

    def call(self, request: dict) -> dict:
        with self._lock:
            if not self._connected:
                self._connect()
            try:
                self._sock.settimeout(self._call_timeout)
                line = (json.dumps(request, separators=(",", ":")) + "\n").encode()
                self._wfile.write(line)
                self._wfile.flush()
                resp_line = self._rfile.readline()
                if not resp_line:
                    return {"error": "sidecar closed connection"}
                return json.loads(resp_line.decode("utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                return {"error": str(exc)}

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None



class _AirstackROSBridgePolicy(Policy):
    """Base for AirStack planner agents that communicate via a TCP ROS sidecar.

    The sidecar (eval/agent_hooks/*.sh) starts the AirStack Docker compose profile
    for the planner and exposes a newline-JSON server on SIDECAR_HOST:SIDECAR_PORT.

    Protocol (same as aerial_nav DYNUS sidecar):
        reset  → {"type": "reset"}
        step   → {"type": "step", "pos": [...], "target": [...], "obstacles": [...]}
        ← {"cmd": {"velocity": [vx, vy, vz]}} or {"cmd": null} (not yet ready → zero vel)
    """

    sensors = ["full_state"]

    # Subclasses set these
    SIDECAR_HOST_ENV: str = "AIRSTACK_SIDECAR_HOST"
    SIDECAR_PORT_ENV: str = "AIRSTACK_SIDECAR_PORT"
    SIDECAR_PORT_DEFAULT: int = 8780

    def __init__(self, connect_timeout: float = 60.0, call_timeout: float = 1.0):
        host = os.environ.get(self.SIDECAR_HOST_ENV, "127.0.0.1")
        port = int(os.environ.get(self.SIDECAR_PORT_ENV, str(self.SIDECAR_PORT_DEFAULT)))
        self._client = _TCPSidecarClient(host, port, connect_timeout, call_timeout)

    def reset(self) -> None:
        self._client.call({"type": "reset"})

    def act(self, obs: dict) -> np.ndarray:
        pos    = obs["agent"].tolist()
        target = obs["target"].tolist()

        # Flatten all obstacle positions into a list of [x, y, z] triples
        obstacle_pts = []
        for key in ("dynamic_obstacles", "static_obstacles", "static_cylinders"):
            arr = obs.get(key)
            if arr is not None and len(arr) > 0:
                for row in np.asarray(arr):
                    obstacle_pts.append(row[:3].tolist())

        env_bounds = obs.get("env_bounds")

        request = {
            "type":       "step",
            "pos":        pos,
            "target":     target,
            "obstacles":  obstacle_pts,
            "env_bounds": env_bounds.tolist() if env_bounds is not None else None,
        }

        resp = self._client.call(request)
        cmd = resp.get("cmd")
        if cmd is None:
            # Planner not ready yet — return zero velocity (hold position)
            return np.zeros(len(pos), dtype=np.float32)

        vel = cmd.get("velocity", [0.0, 0.0, 0.0])
        return np.asarray(vel[:len(pos)], dtype=np.float32)

    def __del__(self):
        try:
            self._client.close()
        except Exception:
            pass


class droan(_AirstackROSBridgePolicy):
    """DROAN local planner running inside the AirStack Docker container.

    Hook: eval/agent_hooks/airstackdroan.sh
    Sidecar port: AIRSTACK_DROAN_PORT (default 8780)
    """
    sensors = ["full_state"]
    SIDECAR_HOST_ENV = "AIRSTACK_SIDECAR_HOST"
    SIDECAR_PORT_ENV = "AIRSTACK_DROAN_PORT"
    SIDECAR_PORT_DEFAULT = 8780


class super(_AirstackROSBridgePolicy):
    """Super planner running inside the AirStack Docker container.

    Hook: eval/agent_hooks/airstacksuperplanner.sh
    Sidecar port: AIRSTACK_SUPER_PORT (default 8781)
    """
    sensors = ["full_state"]
    SIDECAR_HOST_ENV = "AIRSTACK_SIDECAR_HOST"
    SIDECAR_PORT_ENV = "AIRSTACK_SUPER_PORT"
    SIDECAR_PORT_DEFAULT = 8781
