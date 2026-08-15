"""Bring the AirStack stack up/down for evaluation, AirStack's own way.

Uses `airstack up` / `airstack down` (the same entry point the system tests
use), with the safe_eval Isaac launch script selected via
ISAAC_SIM_SCRIPT_NAME and the scenario parameters passed via SAFE_EVAL_CONFIG.
Readiness gates mirror tests/test_takeoff_hover_land.py: MAVROS heartbeat,
then EKF odometry, then the ScenarioManager answering a ping.
"""

from __future__ import annotations

import json
import os
import subprocess

from safe_core.utils import console

from . import ros
from .bridge import StackBridge
from .scenario_client import ScenarioClient, isaac_container_ip

AIRSTACK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

ROBOT = "robot_1"
ROBOT_DOMAIN = 1


def airstack_cmd(*args: str, env_overrides: dict | None = None,
                 timeout: float = 900.0) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.update({k: str(v) for k, v in (env_overrides or {}).items()})
    result = subprocess.run(
        [os.path.join(AIRSTACK_ROOT, "airstack.sh"), *args],
        cwd=AIRSTACK_ROOT, env=env, capture_output=True, text=True, timeout=timeout,
    )
    return result


class StackHandle:
    """One running stack (Isaac sim + robot container) plus its bridge."""

    def __init__(self, headless: bool = True, keep_up: bool = False,
                 ready_timeout: float = 600.0):
        self.headless = headless
        self.keep_up = keep_up
        self.ready_timeout = ready_timeout
        self.bridge: StackBridge | None = None
        self.scenario: ScenarioClient | None = None
        self.container: str | None = None
        self._up_config: str | None = None

    # ── bring-up / teardown ───────────────────────────────────────────────

    def _env_for(self, safe_eval_config: dict, agent_env: dict) -> dict:
        env = {
            "AUTOLAUNCH": "true",
            "NUM_ROBOTS": "1",
            "COMPOSE_PROFILES": "desktop,isaac-sim",
            "ISAAC_SIM_USE_STANDALONE": "true",
            "ISAAC_SIM_SCRIPT_NAME": "safe_eval_launch_script.py",
            "ISAAC_SIM_HEADLESS": "true" if self.headless else "false",
            "PLAY_SIM_ON_START": "true",
            "SAFE_EVAL_CONFIG": json.dumps(safe_eval_config, sort_keys=True),
        }
        env.update(agent_env)
        return env

    def ensure_up(self, safe_eval_config: dict, agent_env: dict | None = None,
                  post_up_cmds: list[str] | None = None) -> None:
        """Idempotent: (re)starts the stack only when the config changed."""
        env = self._env_for(safe_eval_config, agent_env or {})
        fingerprint = json.dumps(env, sort_keys=True)
        if (self._up_config == fingerprint and self.bridge is not None
                and self.bridge.alive and self.scenario is not None
                and self.scenario.alive):
            return

        self.down()
        scene = safe_eval_config.get("env_url") or "default grid"
        line = console.LiveLine(f"airstack up  ({scene})").start_auto()
        # Only the services the eval needs — the GCS is irrelevant here and its
        # host ports may be taken by other stacks on a shared machine.
        result = airstack_cmd("up", "robot-desktop", "isaac-sim",
                              env_overrides=env, timeout=600)
        if result.returncode != 0:
            line.fail("airstack up failed")
            raise RuntimeError(f"airstack up failed:\n{result.stdout[-3000:]}\n{result.stderr[-3000:]}")
        line.done(f"airstack up  ({scene})")
        self._up_config = fingerprint

        self._wait_ready()
        for cmd in post_up_cmds or []:
            console.detail(f"[stack] post-up: {cmd}")
            ros.docker_exec(self.container, cmd, timeout=600, check=True)
        self._start_bridge()

    def down(self) -> None:
        if self.scenario is not None:
            self.scenario.close()
            self.scenario = None
        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None
        if self._up_config is not None or ros.find_containers("robot"):
            airstack_cmd("down", timeout=300)
        self._up_config = None
        self.container = None

    def close(self) -> None:
        if self.keep_up:
            console.info("[stack] --keep-up: leaving the stack running")
            if self.scenario is not None:
                self.scenario.close()
                self.scenario = None
            if self.bridge is not None:
                self.bridge.stop()
                self.bridge = None
            return
        self.down()

    # ── readiness ─────────────────────────────────────────────────────────

    def _wait_ready(self) -> None:
        ros.wait_until(lambda: bool(ros.find_containers("robot")),
                       timeout=180, desc="robot container")
        self.container = ros.robot_container(1)

        def mavros_connected():
            r = ros.ros2_exec(
                self.container,
                f"timeout 5 ros2 topic echo --once --csv --field connected "
                f"/{ROBOT}/interface/mavros/state",
                domain_id=ROBOT_DOMAIN, timeout=15)
            return "true" in r.stdout.lower()

        # Isaac + PX4 SITL boot is the long pole; the system tests allow 300s
        line = console.LiveLine("Isaac Sim boot + PX4 SITL (MAVROS heartbeat)").start_auto()
        try:
            ros.wait_until(mavros_connected, timeout=self.ready_timeout,
                           desc="MAVROS heartbeat (PX4 SITL up)")
        except TimeoutError:
            line.fail()
            raise
        line.done("Isaac Sim + PX4 SITL up")

        def ekf_ready():
            r = ros.topic_echo_once(
                self.container, f"/{ROBOT}/interface/mavros/local_position/odom",
                domain_id=ROBOT_DOMAIN, timeout=5)
            return r.returncode == 0 and "pose:" in r.stdout

        line = console.LiveLine("EKF convergence (arm-ready)").start_auto()
        try:
            ros.wait_until(ekf_ready, timeout=180, desc="EKF odometry (arm-ready)")
        except TimeoutError:
            line.fail()
            raise
        line.done("EKF converged")

    def _start_bridge(self) -> None:
        line = console.LiveLine("ROS bridge + ScenarioManager handshake").start_auto()
        try:
            self.bridge = StackBridge(self.container, robot=ROBOT, domain_id=ROBOT_DOMAIN)
            self.bridge.start()
            self.scenario = ScenarioClient(isaac_container_ip())
            self.scenario.connect(timeout=180)
            self.scenario.command("ping", timeout=60)
            self.scenario.wait_state(timeout=60)
        except Exception:
            line.fail()
            raise
        line.done("stack ready — ScenarioManager responding, ground-truth stream live")

    # ── flight commands (the stack's real action interfaces) ──────────────

    def takeoff(self, altitude_m: float, velocity_m_s: float = 1.0,
                attempts: int = 4) -> None:
        # PX4 refuses to arm until the EKF re-converges after a teleport —
        # the same window the system tests observe after boot. Retry with
        # backoff instead of failing the episode on the first "failed to arm".
        out = ""
        for attempt in range(attempts):
            ok, out = ros.action_send_goal(
                self.container, f"/{ROBOT}/tasks/takeoff", "task_msgs/action/TakeoffTask",
                f"{{target_altitude_m: {altitude_m}, velocity_m_s: {velocity_m_s}}}",
                domain_id=ROBOT_DOMAIN, timeout=120)
            if ok:
                return
            if attempt < attempts - 1:
                console.detail(f"[stack] takeoff attempt {attempt + 1} failed "
                               f"(likely EKF settling) — retrying in 10s")
                import time
                time.sleep(10.0)
        raise RuntimeError(f"takeoff failed after {attempts} attempts:\n{out[-2000:]}")

    def land(self, velocity_m_s: float = 0.5, timeout: float = 120.0) -> bool:
        ok, _ = ros.action_send_goal(
            self.container, f"/{ROBOT}/tasks/land", "task_msgs/action/LandTask",
            f"{{velocity_m_s: {velocity_m_s}}}",
            domain_id=ROBOT_DOMAIN, timeout=timeout)
        return ok

    def disarm(self) -> None:
        ros.service_call(
            self.container, f"/{ROBOT}/interface/robot_command",
            "airstack_msgs/srv/RobotCommand", "{command: 2}",  # DISARM
            domain_id=ROBOT_DOMAIN)
