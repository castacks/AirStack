"""AirStack × S.A.F.E. task adapter.

Implements the BaseTaskAdapter contract for the AirStack integration.
Safety level-set functions are identical to those in aerial_nav/adapter.py
(clearance distance minus threshold radius), since the task structure is the same:
aerial navigation among dynamic and static obstacles.

Usage:
    Full SAFE mode (default) — define l_failure, l_unsafe, delta, run K×N×P×T.
    Simple mode (--simple)   — no level-set functions required; reports success/collision/timeout.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from safe_core.core.task_adapter import BaseTaskAdapter
from safe_core.core.scenario import ScenarioCfg, PerturbationCfg
from safe_core.core.scenario import Disturbance

from .sim.environment_config import (
    get_domain_registry,
    max_dynamic_obstacles,
    max_static_obstacles,
)
from .sim.scenario_config import perturb_scenario


class Adapter(BaseTaskAdapter):
    """Task adapter for AirStack aerial navigation evaluation.

    robot_states stored as dicts with keys:
        agent_pos              — (3,) drone position in benchmark frame
        all_obstacle_positions — (N, 3) combined dynamic + static positions at time t
        num_dynamic            — int, count of dynamic obstacles in all_obstacle_positions[:num_dynamic]
        static_aabbs           — (M, 4) XY axis-aligned bboxes for static mesh obstacles
        x_min/x_max/y_min/y_max — environment wall bounds

    Safety level sets:
        l_failure = min(obstacle_SDF, wall_SDF) - r_failure
        l_unsafe  = min(obstacle_SDF, wall_SDF) - r_unsafe
    """

    def __init__(self):
        self._delta:           float = 0.3
        self._obstacle_radius: float = 0.3
        self._r_failure:       float = 0.31
        self._r_unsafe:        float = 0.61

    # ------------------------------------------------------------------
    # 1) Environment lifecycle
    # ------------------------------------------------------------------

    def build_env(self, ctx: dict) -> Any:
        """The environment is the real AirStack stack (Isaac + robot container),
        orchestrated by AirstackStackEnv. Bring-up is lazy: the stack starts on
        the first reset(), once the first domain is known."""
        from .envs.AirstackStackEnv import AirstackStackEnv
        from .sim.environment_config import get_domain_registry

        registry = get_domain_registry()
        first_domain = (ctx.get("domains") or list(registry))[0]

        return AirstackStackEnv(
            env_cfg=registry[first_domain],
            agent_profile=ctx["agent_profile"],
            max_episode_time=float(ctx.get("max_episode_time", 60.0)),
            dt=float(ctx.get("env_dt", 0.5)),
            max_steps=int(ctx.get("max_steps", 0)) or None,
            max_speed=float(os.environ.get("SAFE_AIRSTACK_MAX_SPEED", "3.0")),
            epsilon_unsafe=float(ctx.get("epsilon_unsafe", 0.3)),
            goal_tolerance=float(ctx.get("goal_tolerance", 1.0)),
            takeoff_velocity=float(ctx.get("takeoff_velocity", 1.0)),
            headless=bool(ctx.get("headless", True)),
            keep_up=bool(ctx.get("keep_up", False)),
        )

    def close_env(self, env: Any) -> None:
        if hasattr(env, "close"):
            env.close()

    # ------------------------------------------------------------------
    # 2) Scenario / perturbation protocol
    # ------------------------------------------------------------------

    def sample_scenario(self, env: Any, domain_idx: int, scenario_idx: int,
                         scenario_seed: int) -> ScenarioCfg:
        return ScenarioCfg(
            domain_idx=domain_idx,
            scenario_idx=scenario_idx,
            scenario_seed=scenario_seed,
        )

    def apply_scenario(self, env: Any, scenario_cfg: ScenarioCfg) -> None:
        env._active_dynamic_obstacles = env.num_dynamic_obstacles
        # Layout only — the stack bring-up + flight happen in reset_episode.
        env.prepare_scenario(scenario_cfg.scenario_seed)

    def disturbance(self) -> Disturbance:
        return Disturbance(
            name="dynamic_obstacle_speed",
            element="moving obstacles (pedestrians and service robots)",
            variable="obstacle_speed_m_s",
            perturb_range=(0.80, 1.20),
            description=(
                "Independently moving pedestrians and service robots are the primary "
                "collision threat in AirStack's indoor and outdoor scenes. "
                "Their speed is resampled each perturbation within ±20% of the domain "
                "nominal, reflecting realistic crowd-density fluctuations and robot "
                "operational speed variance. Faster obstacles reduce time-to-collision "
                "from any given clearance distance, raising the tail risk of impacts."
            ),
        )

    def apply_perturbation(self, env: Any, scenario_cfg: ScenarioCfg,
                            perturbation_seed: int) -> PerturbationCfg:
        scale_range = self.disturbance().perturb_range
        perturb_scenario(env, perturbation_seed, scale_range=scale_range)
        return PerturbationCfg(perturbation_seed=int(perturbation_seed))

    # ------------------------------------------------------------------
    # 3) Episode rollout
    # ------------------------------------------------------------------

    def reset_episode(self, env: Any, episode_seed: Optional[int]) -> Tuple[Dict, Dict]:
        if episode_seed is None:
            return env.reset()
        return env.reset(seed=episode_seed)

    def step_env(self, env: Any, action: Any) -> Tuple[Dict, bool, bool, Dict]:
        return env.step(action)

    # ------------------------------------------------------------------
    # 4) Safety definitions
    # ------------------------------------------------------------------

    @property
    def delta(self) -> float:
        return self._delta

    @staticmethod
    def _wall_sdf(agent_pos, state: dict) -> float:
        """Signed distance to nearest wall face (positive = inside bounds)."""
        if "x_min" not in state:
            return float("inf")
        p = agent_pos
        return float(min(
            p[0] - state["x_min"],
            state["x_max"] - p[0],
            p[1] - state["y_min"],
            state["y_max"] - p[1],
        ))

    @staticmethod
    def _aabb_sdf_2d(p2d, aabbs) -> float:
        if len(aabbs) == 0:
            return float("inf")
        best = float("inf")
        for xmin, ymin, xmax, ymax in aabbs:
            mn = np.array([xmin, ymin])
            mx = np.array([xmax, ymax])
            d_out = np.linalg.norm(np.maximum(mn - p2d, 0) + np.maximum(p2d - mx, 0))
            if d_out == 0:
                d = min(p2d[0] - xmin, xmax - p2d[0], p2d[1] - ymin, ymax - p2d[1])
                dist = -float(d)
            else:
                dist = float(d_out)
            if dist < best:
                best = dist
        return best

    def l_failure(self, state: Any) -> float:
        positions = np.asarray(state["all_obstacle_positions"])
        p = state["agent_pos"]
        p2d = p[:2]
        n_dyn = state.get("num_dynamic", len(positions))
        aabbs = state.get("static_aabbs", np.zeros((0, 4), dtype=np.float32))

        l_dyn = (
            float(np.linalg.norm(positions[:n_dyn, :2] - p2d, axis=1).min()) - self._r_failure
            if n_dyn > 0 and len(positions) >= n_dyn
            else float("inf")
        )

        n_static = len(positions) - n_dyn
        # In 3D the drone flies above floor obstacles — use sphere SDF not AABB projection
        l_static = (
            float(np.linalg.norm(positions[n_dyn:, :2] - p2d, axis=1).min()) - self._r_failure
            if n_static > 0
            else float("inf")
        )

        # Subtract agent body radius from wall SDF so l_failure goes negative when body hits wall
        l_wall = self._wall_sdf(p, state) - 0.15
        return min(l_dyn, l_static, l_wall)

    def l_unsafe(self, state: Any) -> float:
        positions = np.asarray(state["all_obstacle_positions"])
        p = state["agent_pos"]
        p2d = p[:2]
        n_dyn = state.get("num_dynamic", len(positions))
        shell = self._r_unsafe - self._obstacle_radius

        l_dyn = (
            float(np.linalg.norm(positions[:n_dyn, :2] - p2d, axis=1).min()) - self._r_unsafe
            if n_dyn > 0 and len(positions) >= n_dyn
            else float("inf")
        )

        n_static = len(positions) - n_dyn
        l_static = (
            float(np.linalg.norm(positions[n_dyn:, :2] - p2d, axis=1).min()) - self._r_unsafe
            if n_static > 0
            else float("inf")
        )

        l_wall = self._wall_sdf(p, state) - shell - 0.15
        return min(l_dyn, l_static, l_wall)

    # ------------------------------------------------------------------
    # 5) Episode context extraction
    # ------------------------------------------------------------------

    def extract_episode_context(
        self,
        rollout_buffer: Dict[str, Any],
        nominal_outcome: str,
        info: Dict[str, Any],
    ) -> Dict[str, Any]:
        obs_list    = rollout_buffer.get("evaluator_state_trace", rollout_buffer["obs_list"])
        action_list = rollout_buffer["action_list"]
        env         = rollout_buffer["env"]

        obs0 = obs_list[0]
        dim = len(obs0["agent"])
        static_pos = obs0["static_obstacles"].copy()

        # Include static cylinders in the obstacle set for post-hoc scoring
        cylinders = obs0.get("static_cylinders")
        if cylinders is not None and len(cylinders):
            cyl_pos = np.asarray(cylinders, dtype=np.float32)[:, :dim]
            static_pos = np.concatenate([static_pos, cyl_pos], axis=0) if len(static_pos) else cyl_pos

        num_dynamic = env.num_dynamic_obstacles

        # Build per-obstacle trajectories
        dynamic_obs_trajectories = [[] for _ in range(num_dynamic)]
        for obs in obs_list:
            if num_dynamic > 0 and "dynamic_obstacles" in obs:
                for i in range(num_dynamic):
                    pos = obs["dynamic_obstacles"][i, :dim]
                    vel = obs["dynamic_obstacles"][i, dim:]
                    dynamic_obs_trajectories[i].append(np.concatenate([pos.copy(), vel.copy()]))

        static_aabbs = getattr(env, "static_obstacle_aabbs", np.zeros((0, 4), dtype=np.float32))

        # Per-timestep states for l_failure / l_unsafe
        robot_states = []
        for t, obs in enumerate(obs_list):
            agent_pos = obs["agent"].copy()
            if num_dynamic > 0 and dynamic_obs_trajectories[0]:
                dyn_positions = np.array([
                    dynamic_obs_trajectories[i][t][:dim] for i in range(num_dynamic)
                ])
                all_positions = np.concatenate([dyn_positions, static_pos], axis=0)
            else:
                dyn_positions = np.zeros((0, dim), dtype=np.float32)
                all_positions = static_pos
            robot_states.append({
                "agent_pos":              agent_pos,
                "all_obstacle_positions": all_positions,
                "num_dynamic":            num_dynamic,
                "static_aabbs":           static_aabbs,
            })

        env_params = env.get_env_params()

        # Set adapter params before trajectory_scoring calls l_failure / l_unsafe
        self._delta           = env_params["epsilon_unsafe"]
        self._obstacle_radius = env_params["obstacle_radius"]
        self._r_failure       = env_params["r_failure"]
        self._r_unsafe        = env_params["r_unsafe"]

        # Inject env bounds so wall SDF is computed correctly
        bounds = {k: env_params[k] for k in ("x_min", "x_max", "y_min", "y_max") if k in env_params}
        for rs in robot_states:
            rs.update(bounds)

        # Identify first-failure obstacle (dynamic → HVC candidate, static/wall → LVC)
        collision_prim = None
        for rs in robot_states:
            p = rs["agent_pos"]
            positions = rs["all_obstacle_positions"]
            l_wall = self._wall_sdf(p, rs)
            if len(positions) > 0:
                dists = np.linalg.norm(np.asarray(positions) - p, axis=1)
                argmin = int(np.argmin(dists))
                l_cyl = float(dists[argmin]) - self._r_failure
            else:
                l_cyl, argmin = float("inf"), -1
            if min(l_cyl, l_wall) < 0:
                if l_wall < l_cyl:
                    collision_prim = "wall"
                elif argmin < num_dynamic:
                    collision_prim = f"dynamic_obstacle_{argmin}"
                else:
                    collision_prim = f"static_obstacle_{argmin - num_dynamic}"
                break

        return {
            "robot_trajectory": {
                "states":  robot_states,
                "actions": action_list,
            },
            "start_state": {
                "agent_position":            obs0["agent"].copy(),
                "target_position":           obs0["target"].copy(),
                "static_obstacles":          static_pos,
                "dynamic_obstacles_initial": obs0.get("dynamic_obstacles"),
            },
            "dynamic_obstacle_trajectories": dynamic_obs_trajectories,
            "env_params":                    env_params,
            "goal_distance":                 info.get("distance", 0.0),
            "collision_prim":                collision_prim,
        }

    # ------------------------------------------------------------------
    # 6) Taxonomy + cost
    # ------------------------------------------------------------------

    def failure_severity_cost(self, episode_record: dict) -> float:
        if episode_record["nominal_outcome"] != "catastrophic_failure":
            return 0.0
        env_params = episode_record["env_params"]
        max_speed  = env_params["max_speed"]
        states  = episode_record["robot_trajectory"]["states"]
        actions = episode_record["robot_trajectory"]["actions"]
        dt      = env_params.get("dt", 0.1)
        if len(states) >= 2:
            dp = np.asarray(states[-1]["agent_pos"]) - np.asarray(states[-2]["agent_pos"])
            impact_vel = float(np.linalg.norm(dp)) / dt
        else:
            impact_vel = float(np.linalg.norm(actions[-1])) * max_speed if actions else 0.0
        return float(np.clip(impact_vel / max_speed, 0.0, 1.0))

    def extract_performance_metrics(self, episode_record: dict) -> Dict[str, float]:
        states    = episode_record["robot_trajectory"]["states"]
        dt        = episode_record["env_params"].get("dt", 0.1)
        num_steps = episode_record.get("num_steps", len(states))

        positions = np.array([s["agent_pos"] for s in states], dtype=np.float64)
        if len(positions) >= 2:
            diffs      = np.linalg.norm(np.diff(positions, axis=0), axis=1)
            speeds     = diffs / dt
            mean_speed = float(speeds.mean())
            mean_accel = float(np.abs(np.diff(speeds)).mean() / dt) if len(speeds) >= 2 else 0.0
        else:
            mean_speed = mean_accel = 0.0

        return {
            "mean_speed_m_s":  mean_speed,
            "mean_accel_m_s2": mean_accel,
            "duration_s":      num_steps * dt,
        }

    def task_progress_cost(self, episode_record: dict) -> float:
        start_state   = episode_record["start_state"]
        goal_distance = episode_record.get("goal_distance", 0.0)
        initial_dist  = np.linalg.norm(
            start_state["agent_position"] - start_state["target_position"]
        )
        return float(np.clip(goal_distance / initial_dist, 0.0, 1.0)) if initial_dist > 0 else 0.0

    def taxonomy_extensions(self) -> Dict[str, List[str]]:
        return {
            "Safe Failure":         ["SHP", "SR", "SF-O"],
            "Catastrophic Failure": ["LVC", "HVC", "AR-LVC", "AR-HVC"],
            "Unsafe Failure":       ["UF-O"],
        }

    def classify_node(self, episode_record: dict, node_label: str) -> str:
        if node_label == "Catastrophic Failure":
            prim = episode_record.get("collision_prim", "")
            high_value = bool(prim and prim.startswith("dynamic_"))
            retreating, inaction = self._detect_risk_aware_behaviors(episode_record)
            if retreating or inaction:
                return "AR-HVC" if high_value else "AR-LVC"
            return "HVC" if high_value else "LVC"

        if node_label == "Safe Failure":
            return self._classify_safe_failure(episode_record)

        if node_label == "Unsafe Failure":
            return "UF-O"

        return node_label

    def _detect_risk_aware_behaviors(self, episode_record: dict) -> tuple[bool, bool]:
        risk_metrics   = episode_record.get("risk_metrics", {})
        safety_metrics = episode_record.get("safety_metrics", [])
        states         = episode_record.get("robot_trajectory", {}).get("states", [])
        actions        = episode_record.get("robot_trajectory", {}).get("actions", [])
        env_params     = episode_record.get("env_params", {})
        v_threshold    = float(env_params.get("v_threshold", 0.01))

        rpt = getattr(risk_metrics, "rpt", None)
        if rpt is None:
            rpt = risk_metrics.get("rpt", 0.0) if isinstance(risk_metrics, dict) else 0.0
        was_risk_proximate = float(rpt) > 0.0

        inaction = retreating = False

        for t, sm in enumerate(safety_metrics):
            in_shell = (
                sm.get("in_risk_proximate_shell", False)
                if isinstance(sm, dict)
                else getattr(sm, "in_risk_proximate_shell", False)
            )
            if not in_shell:
                continue

            action = actions[t] if t < len(actions) else None
            if action is None:
                continue
            v = np.asarray(action, dtype=np.float64)
            speed = float(np.linalg.norm(v))

            if speed < v_threshold:
                inaction = True

            if t < len(states):
                p         = np.asarray(states[t]["agent_pos"], dtype=np.float64)
                positions = np.asarray(states[t]["all_obstacle_positions"], dtype=np.float64)

                closest = None
                min_dist = float("inf")

                if len(positions) > 0:
                    dists    = np.linalg.norm(positions - p, axis=1)
                    idx      = int(np.argmin(dists))
                    min_dist = float(dists[idx])
                    closest  = positions[idx]

                for key, axis in (("x_min", 0), ("x_max", 0), ("y_min", 1), ("y_max", 1)):
                    coord = states[t].get(key)
                    if coord is None:
                        continue
                    wall_dist = float(abs(p[axis] - coord))
                    if wall_dist < min_dist:
                        min_dist = wall_dist
                        face = p.copy()
                        face[axis] = coord
                        closest = face

                if closest is not None and float(np.dot(closest - p, v)) < 0.0:
                    retreating = True

        return (retreating and was_risk_proximate, inaction and was_risk_proximate)

    def has_risk_aware_behaviors(self, episode_record: dict) -> bool:
        retreating, inaction = self._detect_risk_aware_behaviors(episode_record)
        return retreating or inaction

    def _detect_progressed(self, episode_record: dict) -> bool:
        start_state = episode_record.get("start_state", {})
        p0 = np.asarray(start_state.get("agent_position", []), dtype=np.float64)
        pg = np.asarray(start_state.get("target_position", []), dtype=np.float64)
        if p0.size == 0 or pg.size == 0:
            return True
        initial_dist = float(np.linalg.norm(p0 - pg))
        if initial_dist < 1e-6:
            return True
        for s in episode_record.get("robot_trajectory", {}).get("states", []):
            p = np.asarray(s.get("agent_pos", []), dtype=np.float64)
            if p.size > 0 and float(np.linalg.norm(p - pg)) < initial_dist:
                return True
        return False

    def _classify_safe_failure(self, episode_record: dict) -> str:
        retreating, inaction = self._detect_risk_aware_behaviors(episode_record)
        progressed = self._detect_progressed(episode_record)
        if inaction and progressed:
            return "SHP"
        if retreating and progressed:
            return "SR"
        return "SF-O"

    def adapter_metadata(self) -> Dict[str, Any]:
        return {
            "adapter_name": "AirstackAdapter",
            "task":         "aerial_navigation",
            "sim_backend":  "airstack_isaac",
            "domains":      list(get_domain_registry()),
        }
