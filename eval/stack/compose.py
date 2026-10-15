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

    # The compose services the eval starts and stops; the GCS and other stacks are untouched.
    SERVICES = ("robot-desktop", "isaac-sim")
    PROFILES = "desktop,isaac-sim"

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

    @staticmethod
    def _safe_source_root() -> str:
        from pathlib import Path
        import safe_core
        root = Path(safe_core.__file__).resolve().parent.parent
        required = (root / "integrations" / "navigation" / "common" / "config"
                    / "domains_isaac.py")
        if not required.is_file():
            raise RuntimeError(
                "AirStack-native SAFE worlds need a SAFE source checkout, not the "
                "core-only wheel. Install SAFE editable from source or add its "
                "checkout to PYTHONPATH."
            )
        return str(root)

    @staticmethod
    def _safe_assets_root() -> str:
        """Host folder of SAFE's baked stages, stage exports, skies and UrbanVerse cities,
        which the Isaac container mounts read-only at the same path."""
        from integrations.navigation.common.config.domains_isaac import BAKED_DIR
        return os.path.dirname(os.path.abspath(BAKED_DIR))

    def _env_for(self, safe_eval_config: dict, agent_env: dict) -> dict:
        env = {
            "SAFE_BENCHMARK_SOURCE": self._safe_source_root(),
            "SAFE_BENCHMARK_ASSETS": self._safe_assets_root(),
            "AUTOLAUNCH": "true",
            "NUM_ROBOTS": "1",
            "COMPOSE_PROFILES": self.PROFILES,
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
        result = airstack_cmd("up", *self.SERVICES, env_overrides=env, timeout=600)
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
            # Only the services ensure_up started, never the whole compose project.
            airstack_cmd("down", *self.SERVICES, timeout=300,
                         env_overrides={"COMPOSE_PROFILES": self.PROFILES})
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

    def set_trajectory_mode(self, mode: int) -> None:
        """airstack_msgs/TrajectoryMode: PAUSE=0 ROBOT_POSE=1 TRACK=2 ADD_SEGMENT=3 REWIND=4."""
        ros.service_call(
            self.container, f"/{ROBOT}/trajectory_controller/set_trajectory_mode",
            "airstack_msgs/srv/TrajectoryMode", f"{{mode: {int(mode)}}}",
            domain_id=ROBOT_DOMAIN)

    def set_pid_output(self, enabled: bool) -> None:
        """Give external SAFE setpoints exclusive PX4 control after AirStack takeoff."""
        result = ros.service_call(
            self.container, f"/{ROBOT}/control/safe_eval_pid_output_enabled",
            "std_srvs/srv/SetBool", f"{{data: {str(bool(enabled)).lower()}}}",
            domain_id=ROBOT_DOMAIN)
        if result.returncode != 0 or "success=true" not in result.stdout.replace(" ", "").lower():
            raise RuntimeError(
                f"failed to {'enable' if enabled else 'disable'} AirStack PID output: "
                f"{result.stdout[-800:]} {result.stderr[-800:]}")
