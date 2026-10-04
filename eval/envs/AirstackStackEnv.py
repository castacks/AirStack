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
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from safe_core.core.env_backend import BaseEnvBackend
from safe_core.utils import console

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
                 epsilon_unsafe: float = 0.3, r_failure: float = 0.61,
                 r_unsafe: float = 0.91, goal_tolerance: float = 1.0,
                 takeoff_velocity: float = 1.0, headless: bool = True,
                 keep_up: bool = False, humans: bool = False,
                 goal_distance: float | None = None,
                 static_only: bool = False):
        self.env_cfg = env_cfg
        self.agent_profile = dict(agent_profile)
        self.humans = bool(humans)
        self.goal_distance = goal_distance
        self.static_only = bool(static_only)
        self._ground_z = 0.0
        self.dimension = 3
        self.dt = float(dt)
        self.max_episode_time = float(max_episode_time)
        self.max_steps = int(max_steps if max_steps else max_episode_time / dt)
        # SAFE source velocity policies command physical m/s; their profile
        # cap must not silently cut a 10 m/s source action down to 3 m/s.
        self.max_speed = float(self.agent_profile.get("max_speed", max_speed))
        self.epsilon_unsafe = float(epsilon_unsafe)
        self.r_failure = float(r_failure)
        self.r_unsafe = float(r_unsafe)
        self.goal_tolerance = float(goal_tolerance)
        self.takeoff_velocity = float(takeoff_velocity)

        self.agent_radius = 0.15
        self.obstacle_radius = 0.3
        self.collision_radius = 0.31  # ScenarioManager drone-body contact radius
        self.v_threshold = 0.05
        # Run-up flown before each episode: stereo planners need two airborne
        # viewpoints more than a metre apart before they will certify a path.
        self.warmup_runup_m = 1.5
        # Start and goal stay this far from the arena walls, which are physical
        # here: the stack planner inflates them by 2 m and looks 6 m ahead, so a
        # goal closer than that is never approached within tolerance.
        self.wall_margin = 8.0
        self.min_goal_distance = 12.0

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
        self._static_mesh_positions = np.zeros((0, 3), dtype=np.float32)

        # Episode state
        self._target = np.zeros(3, dtype=np.float32)
        self._start = np.zeros(3, dtype=np.float32)
        self._step_count = 0
        self._last_obs: dict | None = None
        self._last_state_t = 0.0
        self._episode_start_t = 0.0
        self._ros_offset = np.zeros(3)
        self._airborne = False
        self._external_control_active = False

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
        self.dynamic_obstacle_speed = (1.5 if self.humans else float(cfg.dynamic_obstacle_speed))
        self.flight_z = float(cfg.flight_z)
        self._active_dynamic_obstacles = self.num_dynamic_obstacles
        self._layout = None
        self._scenario_seed = None

    def _arena(self) -> list[float]:
        return [self.origin_x, self.origin_y,
                self.origin_x + self.size_x, self.origin_y + self.size_y]

    def _safe_eval_config(self) -> dict:
        if getattr(self.env_cfg, 'source_domain', None):
            # The source stage measures its own bounds after load. Do not put
            # those measured values into the stack fingerprint: that would
            # restart Isaac on the next episode for the same domain.
            return {
                'safe_domain': self.env_cfg.source_domain,
                'humans': self.humans,
                'static_only': self.static_only,
                'goal_distance': self.goal_distance,
                'env_url': None,
                'stage_scale': 1.0,
                'spawn': [0.0, 0.0],
                'max_dynamic': self.num_dynamic_obstacles,
                'max_static': self.num_static_cylinders,
                'obstacle_radius': self.obstacle_radius,
                'drone_radius': self.collision_radius,
                'arena': [0.0, 0.0, 1.0, 1.0],
            }
        cx = self.origin_x + self.size_x / 2.0
        cy = self.origin_y + self.size_y / 2.0
        return {
            "env_url": self.env_cfg.usd_path,
            "safe_domain": getattr(self.env_cfg, "source_domain", None),
            "humans": self.humans,
            "stage_scale": float(getattr(self.env_cfg, "stage_scale", 1.0)),
            "spawn": [cx, cy],
            "max_dynamic": max(1, self.num_dynamic_obstacles),
            "max_static": max(1, self.num_static_cylinders),
            "obstacle_radius": self.obstacle_radius,
            "drone_radius": self.collision_radius,
            "arena": self._arena(),
            "size_z": float(self.size_z),
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
        if getattr(self.env_cfg, 'source_domain', None):
            info = scenario.command('get_info')['domain_info']
            if not info:
                raise RuntimeError('SAFE source world did not report measured bounds')
            # SAFE reports benchmark X/Y; AirStack operates in Isaac world XY.
            self.size_x = float(info['size_y'])
            self.size_y = float(info['size_x'])
            self.size_z = float(info['size_z'])
            self.origin_x = float(info['origin_y'])
            self.origin_y = float(info['origin_x'])
            self._ground_z = float(info['origin_z'])
            self.flight_z = 2.5  # AirStack TakeoffTask altitude above local ground
            self.env_cfg.size_x, self.env_cfg.size_y = self.size_x, self.size_y
            self.env_cfg.origin_x, self.env_cfg.origin_y = self.origin_x, self.origin_y
            self._static_mesh_positions = np.asarray(
                info.get('static_mesh_positions', []), dtype=np.float32).reshape(-1, 3)
            self.static_obstacle_aabbs = np.asarray(
                info.get('static_mesh_aabbs', []), dtype=np.float32).reshape(-1, 4)
            self.num_obstacles = len(self._static_mesh_positions)

        source_domain = getattr(self.env_cfg, 'source_domain', None)
        if source_domain:
            generated = scenario.command('generate_safe_scenario', {
                'seed': int(self._scenario_seed if self._scenario_seed is not None else 0),
                'goal_distance': self.goal_distance,
                'perturbation_seed': getattr(self, '_pending_perturbation_seed', None),
            }, timeout=120)
            self._layout = {
                'agent_location': np.asarray(generated['start'], dtype=np.float32),
                'target_location': np.asarray(generated['target'], dtype=np.float32),
                'static_obstacle_locations': np.asarray(
                    generated['static_obstacle_locations'], dtype=np.float32).reshape(-1, 3),
                'static_cylinder_locations': np.asarray(
                    generated['static_cylinder_locations'], dtype=np.float32).reshape(-1, 3),
                'dynamic_obstacle_locations': np.asarray(
                    generated['dynamic_obstacle_locations'], dtype=np.float32).reshape(-1, 3),
                'dynamic_obstacle_velocities': np.asarray(
                    generated['dynamic_obstacle_velocities'], dtype=np.float32).reshape(-1, 3),
                'dynamic_obstacle_waypoints': np.asarray(
                    generated['dynamic_obstacle_waypoints'], dtype=np.float32).reshape(-1, 3),
            }
            self._initial_dynamic_obstacle_velocities = (
                self._layout['dynamic_obstacle_velocities'].copy())
        else:
            if self._layout is None:
                self.prepare_scenario(self._scenario_seed if self._scenario_seed is not None else 0)
            if getattr(self, '_pending_perturbation_seed', None) is not None:
                scenario_config.perturb_scenario(
                    self, self._pending_perturbation_seed, scale_range=(0.8, 1.2))

        layout = self._layout
        self._start = np.asarray(layout["agent_location"], dtype=np.float32).copy()
        self._target = np.asarray(layout["target_location"], dtype=np.float32).copy()
        if not source_domain:
            self._start[2] = self.flight_z
            self._target[2] = self.flight_z

        # 1. Get the drone on the ground a short run-up behind the start, facing
        #    the goal, so the warm-up flight to the start seeds the planner's
        #    view of the space ahead before the episode clock starts.
        heading = self._target[:2] - self._start[:2]
        heading = heading / max(float(np.linalg.norm(heading)), 1e-6)
        yaw = float(np.arctan2(heading[1], heading[0]))
        runup = self._start.copy()
        runup[:2] -= heading * self.warmup_runup_m
        arena = self._arena()
        runup[0] = float(np.clip(runup[0], arena[0] + 0.4, arena[2] - 0.4))
        runup[1] = float(np.clip(runup[1], arena[1] + 0.4, arena[3] - 0.4))
        self._ensure_landed_at(runup, yaw)

        # 2. Place the obstacle field (episodes restore perturbed velocities).
        n_active = self._active_dynamic_obstacles
        cylinders = layout["static_cylinder_locations"]
        if not source_domain:
            scenario.command("set_scenario", {
                "static": cylinders.tolist(),
                "dynamic": layout["dynamic_obstacle_locations"][:n_active].tolist(),
                "dynamic_vel": self._initial_dynamic_obstacle_velocities[:n_active].tolist(),
                "waypoints": layout["dynamic_obstacle_waypoints"][:n_active].tolist(),
                "target": self._target.tolist(),
                "seed": int(self._scenario_seed or 0),
            })

        # 3. Take off through the stack's own action server, then fly the run-up
        #    to the start on the trajectory controller.
        self.stack.takeoff(self.flight_z, self.takeoff_velocity)
        self._airborne = True
        self._measure_ros_offset()
        self._fly_to(self._start)

        # 4. Hand the goal to the planner-under-test through its own task action.
        if self.agent_profile.get("control_mode", "stack") == "stack":
            self._send_navigate_goal()

        scenario.command("clear_collision")
        state = scenario.wait_state()
        self._last_state_t = float(state.get("t", 0.0))
        self._episode_start_t = self._last_state_t
        self._step_count = 0
        obs = self._build_obs(state)
        self._last_obs = obs
        return obs, {"distance": self._goal_distance(obs)}

    def _ensure_landed_at(self, start_world: np.ndarray, yaw: float = 0.0) -> None:
        """Land wherever we are, then teleport (disarmed) to the given pose."""
        scenario = self.stack.scenario
        if self._external_control_active:
            self.stack.bridge.clear_external_command()
            self.stack.set_pid_output(True)
            self._external_control_active = False
        state = scenario.wait_state()
        if self._airborne or float(state["drone_pos"][2]) > 0.3:
            self.stack.bridge.cancel_navigate()
            self.stack.land()
            time.sleep(2.0)
        self.stack.disarm()
        self._airborne = False
        scenario.command("reset_vehicle", {
            "pos": [float(start_world[0]), float(start_world[1]),
                    float(self._ground_z + 0.07)],
            "yaw": float(yaw),
        })
        # Let the EKF settle on the new (GPS-consistent) position before arming.
        time.sleep(10.0)

    def _measure_ros_offset(self) -> None:
        state = self.stack.scenario.wait_state()
        odom = self.stack.bridge.latest_odom_pos()
        if odom is None:
            # Odometry passthrough not seen yet — fall back to spawn-relative.
            console.detail("[stack] no EKF odometry seen on the bridge; assuming zero offset")
            self._ros_offset = np.zeros(3)
            return
        world = np.asarray(state["drone_pos"], dtype=np.float64)
        self._ros_offset = world - np.asarray(odom, dtype=np.float64)
        console.detail(f"[stack] world→ros offset {np.round(self._ros_offset, 2).tolist()} "
                       f"(world {np.round(world, 2).tolist()}, odom {np.round(odom, 2).tolist()})")

    def _world_to_ros(self, p) -> list[float]:
        q = np.asarray(p, dtype=np.float64) - self._ros_offset
        return [float(q[0]), float(q[1]), float(q[2])]

    def _fly_to(self, point_world: np.ndarray, speed: float = 0.7,
                tolerance: float = 0.4, timeout: float = 40.0) -> None:
        """Fly a straight override segment on the trajectory controller and wait
        for arrival. TRACK mode is required: takeoff leaves the controller
        holding pose, where overrides are ignored."""
        TRACK = 2
        goal = np.asarray(point_world, dtype=np.float64)
        goal[2] = self.flight_z
        deadline = time.monotonic() + timeout
        # The takeoff task switches the controller back to pose-hold as it
        # finishes, which can land after our TRACK switch; re-issue until the
        # drone is actually moving toward the point.
        while time.monotonic() < deadline:
            self.stack.set_trajectory_mode(TRACK)
            pos = np.asarray(self.stack.scenario.wait_state()["drone_pos"], dtype=np.float64)
            self.stack.bridge.publish_traj_override(
                [self._world_to_ros(pos), self._world_to_ros(goal)], velocity=speed)
            issued = np.linalg.norm(pos[:2] - goal[:2])
            for _ in range(20):
                time.sleep(0.5)
                pos = np.asarray(self.stack.scenario.wait_state()["drone_pos"], dtype=np.float64)
                remaining = float(np.linalg.norm(pos[:2] - goal[:2]))
                if remaining < tolerance:
                    console.detail(f"[stack] run-up complete at {np.round(pos, 2).tolist()}")
                    return
                if issued - remaining > 0.3:
                    break  # moving; keep waiting on the same override
            else:
                console.detail("[stack] run-up override not followed; re-issuing")
                continue
            while time.monotonic() < deadline:
                pos = np.asarray(self.stack.scenario.wait_state()["drone_pos"], dtype=np.float64)
                if float(np.linalg.norm(pos[:2] - goal[:2])) < tolerance:
                    console.detail(f"[stack] run-up complete at {np.round(pos, 2).tolist()}")
                    return
                time.sleep(0.5)
        console.detail("[stack] run-up did not reach the start in time; continuing from "
                       f"{np.round(pos, 2).tolist()}")

    def _hold_position(self) -> None:
        """End the planner's task and park the controller on the current pose."""
        ROBOT_POSE = 1
        try:
            if self._external_control_active:
                self.stack.bridge.clear_external_command()
                self.stack.set_pid_output(True)
                self._external_control_active = False
            self.stack.bridge.cancel_navigate()
            self.stack.set_trajectory_mode(ROBOT_POSE)
        except Exception as e:
            console.detail(f"[stack] hold after episode failed: {e}")

    def _send_navigate_goal(self) -> None:
        """Start the planner on a straight two-pose plan to the target. The
        planner rejects a goal while one is active, so a leftover goal is
        cancelled and the send retried once."""
        state = self.stack.scenario.wait_state()
        current = np.asarray(state["drone_pos"], dtype=np.float64)
        current[2] = self.flight_z
        plan = [self._world_to_ros(current), self._world_to_ros(self._target)]
        for attempt in range(3):
            if self.stack.bridge.navigate(plan, self.goal_tolerance):
                console.detail(f"[stack] navigate goal accepted: {np.round(plan, 2).tolist()}")
                return
            console.detail("[stack] navigate goal rejected — cancelling stale goal and retrying")
            self.stack.bridge.cancel_navigate()
            time.sleep(4.0)  # the planner's executor polls cancel at 1 Hz sim time
        raise RuntimeError("planner rejected the NavigateTask goal three times")

    # ── step ──────────────────────────────────────────────────────────────

    def step(self, action: Any) -> Tuple[Dict, bool, bool, Dict]:
        mode = self.agent_profile.get("control_mode", "stack")
        try:
            if mode in ("setpoint", "attitude_thrust") and not self._external_control_active:
                self.stack.set_pid_output(False)
                self._external_control_active = True
            if mode == "velocity":
                self._apply_velocity_action(np.asarray(action, dtype=np.float64))
            elif mode == "setpoint":
                self._apply_setpoint_action(action)
            elif mode == "attitude_thrust":
                self._apply_attitude_action(action)
            elif mode != "stack":
                raise ValueError(f"unknown AirStack SAFE control_mode {mode!r}")
        except Exception:
            if self._external_control_active:
                self.stack.bridge.clear_external_command()
                self.stack.set_pid_output(True)
                self._external_control_active = False
            raise

        target_t = self._last_state_t + self.dt
        state = self.stack.scenario.wait_sim_time(target_t, timeout=max(60.0, self.dt * 30))
        observed_t = float(state.get("t", target_t))
        elapsed = observed_t - self._last_state_t
        if (str(self.agent_profile.get("name", "")).startswith("safe")
                and elapsed > 1.5 * self.dt + 1e-6):
            raise RuntimeError(
                f"SAFE policy missed physics ticks: observed {elapsed:.5f}s between "
                f"states, expected {self.dt:.5f}s. AirStack's free-running sim "
                "cannot claim the SAFE source policy cadence for this episode.")
        self._last_state_t = observed_t

        obs = self._build_obs(state)
        self._last_obs = obs
        self._step_count += 1

        distance = self._goal_distance(obs)
        collision = bool(state.get("collision", False))
        reached = distance < self.goal_tolerance
        terminated = collision or reached
        truncated = (not terminated) and self._step_count >= self.max_steps
        if terminated or truncated:
            self._hold_position()
        info = {
            "collision": collision,
            "collision_with": state.get("collision_with", ""),
            "distance": distance,
            "elapsed_time": self._last_state_t - self._episode_start_t,
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

    def _apply_setpoint_action(self, action: dict) -> None:
        """Forward the planner wrapper's complete world-ENU reference to PX4.

        The AirStack planner wrapper has already converted SAFE benchmark
        X/Y and compass heading into ENU. Only position needs the EKF offset.
        """
        if not isinstance(action, dict):
            # SAFE sidecars return a zero velocity fallback when no reference
            # is available. Command a PX4 local position hold because the
            # AirStack PID is gated off while this planner owns the vehicle.
            fallback = np.asarray(action, dtype=np.float64).reshape(-1)
            if fallback.size != 3 or not np.allclose(fallback, 0.0):
                raise ValueError("planner fallback must be a zero velocity")
            state = self.stack.scenario.latest_state()
            if state is None:
                raise RuntimeError("cannot issue planner hover without drone state")
            hover = self._world_to_ros(state["drone_pos"])
            self.stack.bridge.publish_setpoint(
                hover, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, None)
            return
        from integrations.aerial_nav.envs.setpoint import parse

        sp = parse(action)
        ros_pos = self._world_to_ros(sp.position)
        yaw = math.atan2(math.sin(sp.heading), math.cos(sp.heading))
        self.stack.bridge.publish_setpoint(
            ros_pos, sp.velocity.tolist(), sp.acceleration.tolist(), yaw, sp.heading_rate)

    def _apply_attitude_action(self, action: dict) -> None:
        """Convert SAFE's PX4 NED/FRD quaternion to the AirStack ENU/FLU topic."""
        if not isinstance(action, dict) or "attitude" not in action or "thrust" not in action:
            raise ValueError("attitude_thrust action needs attitude and thrust")
        q = np.asarray(action["attitude"], dtype=np.float64).reshape(-1)
        if q.size != 4 or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
            raise ValueError("attitude must be four finite PX4 NED/FRD wxyz values")
        thrust = float(action["thrust"])
        if not math.isfinite(thrust) or not 0.0 <= thrust <= 1.0:
            raise ValueError("normalized thrust must be finite in [0, 1]")
        ned_to_enu = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        frd_to_flu = np.diag([1., -1., -1.])
        rotation_ned_frd = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        rotation_enu_flu = ned_to_enu @ rotation_ned_frd.as_matrix() @ frd_to_flu
        q_enu_flu_xyzw = Rotation.from_matrix(rotation_enu_flu).as_quat()
        self.stack.bridge.publish_attitude_thrust(q_enu_flu_xyzw.tolist(), thrust)

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
        obs = {
            "agent": np.asarray(state["drone_pos"], dtype=np.float32),
            "agent_vel": np.asarray(state.get("drone_vel", [0, 0, 0]), dtype=np.float32),
            "velocity": np.asarray(state.get("drone_vel", [0, 0, 0]), dtype=np.float32),
            "target": self._target.copy(),
            "static_obstacles": self._static_mesh_positions.copy(),
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
        sensor_obs = self.stack.bridge.latest_sensor_obs()
        if sensor_obs:
            obs.update(sensor_obs)
        return obs

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
                self._hold_position()
                self.stack.land()
        except Exception:
            pass
        self.stack.close()
