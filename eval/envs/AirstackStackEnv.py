"""AirstackStackEnv — the real AirStack stack as a S.A.F.E. environment backend.

The "environment" here is the full AirStack system: Isaac Sim (via AirStack's
own Pegasus launch path, running safe_eval_launch_script.py), PX4 SITL, and
the robot container running the complete autonomy stack (perception →
planner → trajectory controller → MAVROS). SAFE does not fly the drone —
the stack does. This env:

  reset()  brings the stack up (once), lands/teleports the drone to the
           scenario start, places obstacles via the Isaac ScenarioManager,
           takes off through /robot_1/tasks/takeoff, and hands the goal to
           the planner-under-test by publishing /robot_1/global_plan.
  step()   waits env_dt of *sim* time, then samples ground truth from the
           ScenarioManager stream and reports (obs, terminated, truncated,
           info) with info["collision"] from the sim's latched contact check.

Agents with control_mode == "stack" (DROAN, ...) get a passthrough act();
velocity baselines instead have their normalized action turned into a short
trajectory_override segment on the REAL trajectory controller, so baselines
and planners fly through the same interface layer.

Frames: obstacle layout, goals, and scoring all live in Isaac world
coordinates (ENU). The stack's local frame origin is wherever PX4's EKF
initialized; the offset between the two is measured once per bring-up from
(ground-truth pose − EKF odometry) and applied to everything we publish.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from safe_core.core.env_backend import BaseEnvBackend

from ..sim import scenario_config
from ..stack.compose import StackHandle


class _BoxActionSpace:
    """Minimal action space: uniform samples in [-1, 1]^3 (BaseEnvBackend only
    requires .sample())."""

    def __init__(self, dim: int = 3):
        self.dim = dim

    def sample(self) -> np.ndarray:
        return np.random.uniform(-1.0, 1.0, size=self.dim).astype(np.float32)


class AirstackStackEnv(BaseEnvBackend):

    def __init__(self, env_cfg, agent_profile: dict, *,
                 max_episode_time: float = 60.0, dt: float = 0.5,
                 max_steps: Optional[int] = None, max_speed: float = 3.0,
                 epsilon_unsafe: float = 0.3, r_failure: float = 0.31,
                 r_unsafe: float = 0.61, goal_tolerance: float = 1.0,
                 takeoff_velocity: float = 1.0, headless: bool = True,
                 keep_up: bool = False):
        self.env_cfg = env_cfg
        self.agent_profile = dict(agent_profile)
        self.dimension = 3
        self.dt = float(dt)
        self.max_episode_time = float(max_episode_time)
        self.max_steps = int(max_steps if max_steps else max_episode_time / dt)
        self.max_speed = float(max_speed)
        self.epsilon_unsafe = float(epsilon_unsafe)
        self.r_failure = float(r_failure)
        self.r_unsafe = float(r_unsafe)
        self.goal_tolerance = float(goal_tolerance)
        self.takeoff_velocity = float(takeoff_velocity)

        self.agent_radius = 0.15
        self.obstacle_radius = 0.3
        self.v_threshold = 0.05

        self.stack = StackHandle(headless=headless, keep_up=keep_up)
        self._action_space = _BoxActionSpace(3)
        self.np_random = np.random.RandomState(0)

        # Populated by switch_domain() / _apply_domain_cfg()
        self._apply_domain_cfg(env_cfg)

        # Scenario state
        self._layout: dict | None = None
        self._scenario_seed: int | None = None
        self._dynamic_obstacle_velocities = np.zeros((0, 3), dtype=np.float32)
        self._initial_dynamic_obstacle_velocities = np.zeros((0, 3), dtype=np.float32)
        self.static_obstacle_aabbs = np.zeros((0, 4), dtype=np.float32)

        # Episode state
        self._target = np.zeros(3, dtype=np.float32)
        self._start = np.zeros(3, dtype=np.float32)
        self._step_count = 0
        self._last_obs: dict | None = None
        self._last_state_t = 0.0
        self._ros_offset = np.zeros(3)
        self._airborne = False

    # ── domain plumbing (called by sim.environment_config.switch_domain) ──

    def _apply_domain_cfg(self, cfg) -> None:
        self.env_cfg = cfg
        self.size_x = float(cfg.size_x)
        self.size_y = float(cfg.size_y)
        self.size_z = float(cfg.size_z)
        self.origin_x = float(cfg.origin_x)
        self.origin_y = float(cfg.origin_y)
        self.num_dynamic_obstacles = int(cfg.num_dynamic_obstacles)
        self.num_static_cylinders = int(cfg.num_static_cylinders)
        self.num_obstacles = 0  # scene-mesh obstacles: not tracked in v1
        self.dynamic_obstacle_speed = float(cfg.dynamic_obstacle_speed)
        self.flight_z = float(cfg.flight_z)
        self._active_dynamic_obstacles = self.num_dynamic_obstacles
        self._layout = None
        self._scenario_seed = None

    def _arena(self) -> list[float]:
        return [self.origin_x, self.origin_y,
                self.origin_x + self.size_x, self.origin_y + self.size_y]

    def _safe_eval_config(self) -> dict:
        cx = self.origin_x + self.size_x / 2.0
        cy = self.origin_y + self.size_y / 2.0
        return {
            "env_url": self.env_cfg.usd_path,
            "stage_scale": float(getattr(self.env_cfg, "stage_scale", 1.0)),
            "spawn": [cx, cy],
            "max_dynamic": max(1, self.num_dynamic_obstacles),
            "max_static": max(1, self.num_static_cylinders),
            "obstacle_radius": self.obstacle_radius,
            "drone_radius": self.r_failure,
            "arena": self._arena(),
        }

    # ── reset ─────────────────────────────────────────────────────────────

    def prepare_scenario(self, seed: int) -> None:
        """Sample the scenario layout without touching the stack (called by
        Adapter.apply_scenario; the flight itself happens in reset())."""
        self.np_random = np.random.RandomState(seed)
        self._scenario_seed = seed
        self._layout = scenario_config.configure_scenario(self)
        self._dynamic_obstacle_velocities = self._layout["dynamic_obstacle_velocities"]
        self._initial_dynamic_obstacle_velocities = \
            self._layout["dynamic_obstacle_velocities"].copy()

    def reset(self, seed: Optional[int] = None) -> Tuple[Dict, Dict]:
        self.stack.ensure_up(
            self._safe_eval_config(),
            agent_env=self.agent_profile.get("agent_env") or {},
            post_up_cmds=self.agent_profile.get("post_up_cmds") or [],
        )
        bridge = self.stack.bridge  # noqa: F841 — real-topic publisher
        scenario = self.stack.scenario

        if self._layout is None or (seed is not None and seed != self._scenario_seed):
            self.prepare_scenario(seed if seed is not None else 0)

        layout = self._layout
        self._start = np.asarray(layout["agent_location"], dtype=np.float32).copy()
        self._start[2] = self.flight_z
        self._target = np.asarray(layout["target_location"], dtype=np.float32).copy()
        self._target[2] = self.flight_z

        # 1. Get the drone on the ground at the scenario start.
        self._ensure_landed_at(self._start)

        # 2. Place the obstacle field (episodes restore perturbed velocities).
        n_active = self._active_dynamic_obstacles
        cylinders = layout["static_cylinder_locations"]
        scenario.command("set_scenario", {
            "static": cylinders.tolist(),
            "dynamic": layout["dynamic_obstacle_locations"][:n_active].tolist(),
            "dynamic_vel": self._initial_dynamic_obstacle_velocities[:n_active].tolist(),
            "waypoints": layout["dynamic_obstacle_waypoints"][:n_active].tolist(),
            "target": self._target.tolist(),
            "seed": int(self._scenario_seed or 0),
        })

        # 3. Take off through the stack's own action server.
        self.stack.takeoff(self.flight_z, self.takeoff_velocity)
        self._airborne = True
        self._measure_ros_offset()

        # 4. Hand the goal to the planner-under-test: a real global plan.
        if self.agent_profile.get("control_mode", "stack") == "stack":
            self._publish_goal_plan()

        scenario.command("clear_collision")
        state = scenario.wait_state()
        self._last_state_t = float(state.get("t", 0.0))
        self._step_count = 0
        obs = self._build_obs(state)
        self._last_obs = obs
        return obs, {"distance": self._goal_distance(obs)}

    def _ensure_landed_at(self, start_world: np.ndarray) -> None:
        """Land wherever we are, then teleport (disarmed) to the episode start."""
        scenario = self.stack.scenario
        state = scenario.wait_state()
        if self._airborne or float(state["drone_pos"][2]) > 0.3:
            self.stack.land()
            time.sleep(2.0)
        self.stack.disarm()
        self._airborne = False
        scenario.command("reset_vehicle", {
            "pos": [float(start_world[0]), float(start_world[1]), 0.07],
            "yaw": 0.0,
        })
        # Let the EKF settle on the new (GPS-consistent) position before arming.
        time.sleep(10.0)

    def _measure_ros_offset(self) -> None:
        state = self.stack.scenario.wait_state()
        odom = self.stack.bridge.latest_odom_pos()
        if odom is None:
            # Odometry passthrough not seen yet — fall back to spawn-relative.
            self._ros_offset = np.zeros(3)
            return
        world = np.asarray(state["drone_pos"], dtype=np.float64)
        self._ros_offset = world - np.asarray(odom, dtype=np.float64)

    def _world_to_ros(self, p) -> list[float]:
        q = np.asarray(p, dtype=np.float64) - self._ros_offset
        return [float(q[0]), float(q[1]), float(q[2])]

    def _publish_goal_plan(self) -> None:
        state = self.stack.scenario.wait_state()
        current = np.asarray(state["drone_pos"], dtype=np.float64)
        current[2] = self.flight_z
        self.stack.bridge.publish_global_plan([
            self._world_to_ros(current),
            self._world_to_ros(self._target),
        ])

    # ── step ──────────────────────────────────────────────────────────────

    def step(self, action: Any) -> Tuple[Dict, bool, bool, Dict]:
        if self.agent_profile.get("control_mode", "stack") == "velocity":
            self._apply_velocity_action(np.asarray(action, dtype=np.float64))

        target_t = self._last_state_t + self.dt
        state = self.stack.scenario.wait_sim_time(target_t, timeout=max(60.0, self.dt * 30))
        self._last_state_t = float(state.get("t", target_t))

        obs = self._build_obs(state)
        self._last_obs = obs
        self._step_count += 1

        distance = self._goal_distance(obs)
        collision = bool(state.get("collision", False))
        reached = distance < self.goal_tolerance
        terminated = collision or reached
        truncated = (not terminated) and self._step_count >= self.max_steps
        info = {
            "collision": collision,
            "collision_with": state.get("collision_with", ""),
            "distance": distance,
        }
        return obs, terminated, truncated, info

    def _apply_velocity_action(self, action: np.ndarray) -> None:
        """Normalized velocity → short trajectory_override segment on the real
        trajectory controller (same interface layer the planners command)."""
        state = self.stack.scenario.latest_state()
        if state is None:
            return
        norm = float(np.linalg.norm(action))
        speed = float(np.clip(norm, 0.0, 1.0) * self.max_speed)
        current = np.asarray(state["drone_pos"], dtype=np.float64)
        if speed < 1e-3:
            hover = current.copy()
            hover[2] = max(hover[2], self.flight_z * 0.5)
            self.stack.bridge.publish_traj_override(
                [self._world_to_ros(hover)], velocity=0.1)
            return
        direction = action / max(norm, 1e-8)
        ahead = current + direction * speed * max(self.dt * 2.0, 1.0)
        ahead[2] = float(np.clip(ahead[2], 0.5, self.size_z))
        self.stack.bridge.publish_traj_override(
            [self._world_to_ros(current), self._world_to_ros(ahead)], velocity=speed)

    # ── observations / contract ───────────────────────────────────────────

    def _build_obs(self, state: dict) -> Dict[str, Any]:
        n_active = self._active_dynamic_obstacles
        dyn_pos = np.asarray(state.get("dynamic", []), dtype=np.float32).reshape(-1, 3)
        dyn_vel = np.asarray(state.get("dynamic_vel", []), dtype=np.float32).reshape(-1, 3)
        if len(dyn_pos) < n_active:  # stream may briefly lag a scenario change
            pad = n_active - len(dyn_pos)
            dyn_pos = np.vstack([dyn_pos, np.zeros((pad, 3), dtype=np.float32)]) \
                if len(dyn_pos) else np.zeros((n_active, 3), dtype=np.float32)
            dyn_vel = np.vstack([dyn_vel, np.zeros((pad, 3), dtype=np.float32)]) \
                if len(dyn_vel) else np.zeros((n_active, 3), dtype=np.float32)
        cylinders = (self._layout["static_cylinder_locations"]
                     if self._layout is not None
                     else np.zeros((0, 3), dtype=np.float32))
        arena = self._arena()
        return {
            "agent": np.asarray(state["drone_pos"], dtype=np.float32),
            "agent_vel": np.asarray(state.get("drone_vel", [0, 0, 0]), dtype=np.float32),
            "target": self._target.copy(),
            "static_obstacles": np.zeros((0, 3), dtype=np.float32),
            "static_cylinders": np.asarray(cylinders, dtype=np.float32),
            "dynamic_obstacles": np.concatenate(
                [dyn_pos[:n_active], dyn_vel[:n_active]], axis=1),
            "static_obstacle_aabbs": self.static_obstacle_aabbs,
            "env_bounds": np.array(
                [arena[0], arena[1], 0.0, arena[2], arena[3], self.size_z],
                dtype=np.float32),
            "r_unsafe": self.r_unsafe,
            "max_speed": self.max_speed,
            "t": float(state.get("t", 0.0)),
        }

    def _goal_distance(self, obs: dict) -> float:
        return float(np.linalg.norm(obs["agent"][:2] - self._target[:2]))

    def get_evaluator_state(self) -> Dict[str, Any]:
        return dict(self._last_obs) if self._last_obs is not None else {}

    @property
    def action_space(self):
        return self._action_space

    def get_env_params(self) -> Dict[str, Any]:
        arena = self._arena()
        return {
            "dt": self.dt,
            "max_speed": self.max_speed,
            "epsilon_unsafe": self.epsilon_unsafe,
            "obstacle_radius": self.obstacle_radius,
            "r_failure": self.r_failure,
            "r_unsafe": self.r_unsafe,
            "v_threshold": self.v_threshold,
            "x_min": arena[0], "y_min": arena[1],
            "x_max": arena[2], "y_max": arena[3],
            "size_x": self.size_x, "size_y": self.size_y,
            "flight_z": self.flight_z,
            "domain": self.env_cfg.domain,
            "num_dynamic_obstacles": self._active_dynamic_obstacles,
            "num_static_cylinders": self.num_static_cylinders,
            "agent_profile": self.agent_profile.get("name", "unknown"),
        }

    def get_runtime_state(self) -> Dict[str, Any]:
        s = self.stack.scenario.latest_state() if self.stack.scenario else None
        return {"sim_time": s.get("t") if s else None, "steps": self._step_count}

    def close(self) -> None:
        try:
            if self._airborne:
                self.stack.land()
        except Exception:
            pass
        self.stack.close()
