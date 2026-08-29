"""Bounded, best-effort simulator failure diagnostics."""

from __future__ import annotations

import json
import os
import re
import subprocess
from pathlib import Path

from harness import session

MAX_OUTPUT_CHARS = 16_000
SAFE_ENV_KEYS = (
    "COMPOSE_PROFILES",
    "NUM_ROBOTS",
    "URDF_FILE",
    "AUTOLAUNCH",
    "PLAY_SIM_ON_START",
    "ISAAC_SIM_SCRIPT_NAME",
    "ISAAC_SIM_HEADLESS",
    "MS_AIRSIM_HEADLESS",
    "MS_AIRSIM_ENV_DIR",
    "MS_AIRSIM_BINARY_PATH",
    "PX4_PARAM_SET",
)


def _bounded_run(args, timeout=10) -> dict:
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = (result.stdout or "") + (result.stderr or "")
        return {
            "returncode": result.returncode,
            "output": output[-MAX_OUTPUT_CHARS:],
        }
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)[:180]


def collect_failure_diagnostics(
    env: dict | None = None,
    reason: str = "",
    test_id: str = "session",
) -> Path | None:
    """Persist a bounded JSON bundle; diagnostic failures never mask the test."""
    run_dir = session.run_dir()
    if run_dir is None:
        return None
    env = env or {}
    containers_result = _bounded_run(
        ["docker", "ps", "--format", "{{.Names}}"], timeout=10
    )
    containers = containers_result.get("output", "").splitlines()[:20]
    container_data = {}
    pane_cmd = (
        "tmux list-panes -a -F "
        "'#{session_name}:#{window_name}|#{pane_pid}|#{pane_title}|#{pane_dead}'"
    )
    for container in containers:
        container_data[container] = {
            "logs": _bounded_run(
                ["docker", "logs", "--tail", "200", container], timeout=15
            ),
            "tmux": _bounded_run(
                ["docker", "exec", container, "bash", "-c", pane_cmd], timeout=10
            ),
        }
    robot_graph = {}
    for index, container in enumerate(
        [name for name in containers if "robot" in name and "desktop" in name],
        start=1,
    ):
        robot_graph[container] = _bounded_run(
            [
                "docker", "exec", "-e", f"ROS_DOMAIN_ID={index}", container,
                "bash", "-lc",
                "source /opt/ros/jazzy/setup.bash 2>/dev/null; "
                "source /root/AirStack/robot/ros_ws/install/setup.bash 2>/dev/null; "
                "echo NODES; ros2 node list 2>&1; "
                "echo TOPICS; ros2 topic list 2>&1",
            ],
            timeout=15,
        )
    payload = {
        "schema_version": 1,
        "reason": reason[:4000],
        "effective_config": {
            key: str(env.get(key, os.environ.get(key, "")))[:2000]
            for key in SAFE_ENV_KEYS
            if env.get(key, os.environ.get(key)) is not None
        },
        "containers": container_data,
        "ros_graph": robot_graph,
        "gpu": _bounded_run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,utilization.gpu,memory.used,memory.total",
                "--format=csv,noheader",
            ],
            timeout=10,
        ),
        "command_ring": [
            {
                "command": str(entry.get("command", ""))[:1000],
                "log_name": str(entry.get("log_name", ""))[:300],
                "output": str(entry.get("output", ""))[-12000:],
            }
            for entry in session.recent_cmd_outputs()[-30:]
        ],
    }
    diagnostics_dir = Path(run_dir) / "diagnostics"
    diagnostics_dir.mkdir(parents=True, exist_ok=True)
    path = diagnostics_dir / f"{_safe_name(test_id)}.json"
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return path
