"""AirStack × S.A.F.E. task adapter.

Implements the BaseTaskAdapter contract for the AirStack integration.
Safety level-set functions follow the point-obstacle path of SAFE's
safe_navigation/aerial/adapter.py (centre distance minus contact radius),
since the task structure is the same: aerial navigation among dynamic and static
obstacles.

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


class Adapter(BaseTaskAdapter):
    """Task adapter for AirStack aerial navigation evaluation.

    robot_states stored as dicts with keys:
        agent_pos              — (3,) drone position in benchmark frame
        all_obstacle_positions — (N, 3) combined dynamic + static positions at time t
        num_dynamic            — int, count of dynamic obstacles in all_obstacle_positions[:num_dynamic]
        static_aabbs           — (M, 4) XY axis-aligned bboxes for static mesh obstacles
        x_min/x_max/y_min/y_max — environment wall bounds
        body_clearance         — clearance from the drone's collider box, measured in Isaac

    Safety level sets:
        l_failure = body_clearance
        l_unsafe  = l_failure - r_unsafe_offset_m
    Records without body_clearance take clearance - body_radius from the drone's centre to a
    person's capsule in 3-D, a column at its own radius, or an arena wall.
    """

    # SAFE's walker collision capsule: its radius, and half the length of its straight section.
    PERSON_RADIUS = 0.3
    PERSON_HALF_LENGTH = 0.6

    def __init__(self):
        self._delta:           float = 0.3
        self._obstacle_radius: float = 0.3

    @staticmethod
    def _body_radius() -> float:
        """The flown robot's body radius from SAFE's robot registry."""
        import safe_navigation.common.config.robot_spec  # noqa: F401
        from safe_core.core.robot_spec import get_active_robot
        return float(get_active_robot().body_radius)

    @staticmethod
    def _band() -> float:
        """Width of the unsafe band beyond the failure surface, from the rulebook."""
        from safe_core.core.records import rulebook
        return float(rulebook().r_unsafe_offset_m)

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
            humans=bool(ctx.get("humans", False)),
            goal_distance=ctx.get("goal_distance"),
            static_only=bool(ctx.get("static_only", False)),
            sensors=list(ctx.get("sensors") or []),
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
        # SAFE USD bounds are measured inside Isaac at boot. Defer the layout
        # until reset_episode, after that measured geometry reaches the host.
        env._scenario_seed = scenario_cfg.scenario_seed
        # As SAFE's aerial adapter does: the index picks a photoreal stage's density band.
        env._scenario_index = scenario_cfg.scenario_idx
        env._layout = None
        env._pending_perturbation_seed = None

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
        # SAFE's sampler inside Isaac draws the movers' speeds from this seed on the next reset.
        env._pending_perturbation_seed = int(perturbation_seed)
        return PerturbationCfg(perturbation_seed=int(perturbation_seed))

    def resolved_domain(self, env: Any) -> Optional[Dict[str, Any]]:
        """The arena the loaded domain measured, recorded in provenance.json."""
        env_cfg = getattr(env, "env_cfg", None)
        if env_cfg is None:
            return None
        fields = ("size_x", "size_y", "origin_x", "origin_y", "origin_z", "floor_z",
                  "num_static_cylinders", "num_dynamic_obstacles", "dynamic_obstacle_speed")
        return {f: getattr(env_cfg, f, None) for f in fields}

    def perturbations_vary_episodes(self, domain_cfg: Any) -> bool:
        """A perturbation resamples only the movers, so a domain without them replays."""
        return bool(getattr(domain_cfg, "num_dynamic_obstacles", 0))

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

    def _as_state(self, state: dict) -> dict:
        """A scoring state from a raw env observation, which SAFE's per-step video risk
        feed passes; a state built by extract_episode_context passes through unchanged."""
        if "all_obstacle_positions" in state:
            return state
        p = np.asarray(state["agent"], dtype=np.float64)
        dim = len(p)
        static = [np.asarray(state.get(k, np.zeros((0, dim))), dtype=np.float64).reshape(-1, dim)
                  for k in ("static_obstacles", "static_cylinders")]
        dyn = np.asarray(state.get("dynamic_obstacles", np.zeros((0, 2 * dim))),
                         dtype=np.float64).reshape(-1, 2 * dim)[:, :dim]
        radii = [np.full(len(static[0]), self._obstacle_radius),
                 np.asarray(state.get("static_cylinder_radii", np.full(len(static[1]), self._obstacle_radius)),
                            dtype=np.float64).reshape(-1)]
        out = {"agent_pos": p, "all_obstacle_positions": np.concatenate([dyn, *static]),
               "num_dynamic": len(dyn), "static_radii": np.concatenate(radii),
               "people": bool(state.get("humans", False))}
        bounds = state.get("env_bounds")
        if bounds is not None:
            out.update(x_min=bounds[0], y_min=bounds[1], x_max=bounds[3], y_max=bounds[4])
        return out

    def _nearest(self, state: dict) -> Tuple[float, str]:
        """(clearance, what it is) from the drone's centre to the nearest surface: a person's
        capsule in 3-D, a moving pool cylinder or a column at its radius, or a wall."""
        p = np.asarray(state["agent_pos"], dtype=np.float64)
        positions = np.asarray(state["all_obstacle_positions"], dtype=np.float64).reshape(-1, len(p))
        n_dyn = int(state.get("num_dynamic", 0))
        best = (self._wall_sdf(p, state), "wall")
        if n_dyn > 0:
            centres = positions[:n_dyn]
            d = np.linalg.norm(centres[:, :2] - p[:2], axis=1)
            people = bool(state.get("people", False))
            if people and len(p) > 2:
                # A drone above a head is clear: distance to the capsule's axis segment.
                dz = p[2] - np.clip(p[2], centres[:, 2] - self.PERSON_HALF_LENGTH,
                                    centres[:, 2] + self.PERSON_HALF_LENGTH)
                d = np.hypot(d, dz)
            clear = d - (self.PERSON_RADIUS if people else self._obstacle_radius)
            i = int(np.argmin(clear))
            if clear[i] < best[0]:
                best = (float(clear[i]), f"dynamic_obstacle_{i}")
        static = positions[n_dyn:]
        if len(static):
            radii = np.asarray(state.get("static_radii", np.full(len(static), self._obstacle_radius)),
                               dtype=np.float64)
            clear = np.linalg.norm(static[:, :2] - p[:2], axis=1) - radii
            i = int(np.argmin(clear))
            if clear[i] < best[0]:
                best = (float(clear[i]), f"static_obstacle_{i}")
        return best

    def l_failure(self, state: Any) -> float:
        # Isaac measures from the drone's collider box itself, so no body radius comes off it.
        # Records flown before that measurement existed fall back to the centre geometry.
        if "body_clearance" in state:
            return float(state["body_clearance"])
        return self._nearest(self._as_state(state))[0] - self._body_radius()

    def l_unsafe(self, state: Any) -> float:
        return self.l_failure(state) - self._band()

    @property
    def risk_latency(self) -> float:
        from safe_core.core.records import rulebook
        return rulebook().risk_latency_s

    @property
    def risk_deceleration(self) -> float:
        from safe_core.core.records import rulebook
        return rulebook().risk_deceleration_m_s2

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
        static_radii = np.full(len(static_pos), self._obstacle_radius)

        # Include static cylinders in the obstacle set for post-hoc scoring, each at its own radius
        cylinders = obs0.get("static_cylinders")
        if cylinders is not None and len(cylinders):
            cyl_pos = np.asarray(cylinders, dtype=np.float32)[:, :dim]
            static_pos = np.concatenate([static_pos, cyl_pos], axis=0) if len(static_pos) else cyl_pos
            static_radii = np.concatenate([static_radii, np.asarray(
                obs0.get("static_cylinder_radii", np.full(len(cyl_pos), self._obstacle_radius)),
                dtype=np.float64).reshape(-1)])

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
                "static_radii":           static_radii,
                "people":                 bool(getattr(env, "humans", False)),
                "body_clearance":         float(obs["body_clearance"]),
            })

        env_params = env.get_env_params()

        # Set adapter params before trajectory_scoring calls l_failure / l_unsafe
        self._delta           = env_params["epsilon_unsafe"]
        self._obstacle_radius = env_params["obstacle_radius"]

        # Inject env bounds so wall SDF is computed correctly
        bounds = {k: env_params[k] for k in ("x_min", "x_max", "y_min", "y_max") if k in env_params}
        for rs in robot_states:
            rs.update(bounds)

        # Identify first-failure obstacle (dynamic → HVC candidate, static/wall → LVC)
        collision_prim = info.get("collision_with") or None
        body = self._body_radius()
        for rs in robot_states if not collision_prim else ():
            clearance, what = self._nearest(rs)
            if clearance - body < 0:
                collision_prim = what
                break

        return {
            # Computed inside Isaac by SAFE's scenario_bands; None off photoreal stages.
            "scenario_difficulty": getattr(env, "_scenario_difficulty", None),
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
            # The sim latches one contact per episode, so the count is 0 or 1.
            "n_collisions":                  int(info.get(
                "n_collisions", 1 if nominal_outcome == "catastrophic_failure" else 0)),
        }

    def trajectory_arrays(self, rollout_buffer: Dict[str, Any]) -> Dict[str, Any]:
        """SAFE's navigation npz schema, which replay and rescore read. The measured box clearance
        is written as SAFE's physx_clearance, a centre clearance, so SAFE's replay renderer
        colours the trail by this integration's l_failure."""
        from safe_core.records.episodes import navigation_trajectory_arrays
        out = navigation_trajectory_arrays(rollout_buffer)
        trace = rollout_buffer["evaluator_state_trace"]
        if trace and "body_clearance" in trace[0]:
            out["physx_clearance"] = (np.asarray([s.get("body_clearance", np.inf) for s in trace],
                                                 dtype=np.float32) + self._body_radius())
        return out

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

    def outcome_display_names(self) -> Dict[str, str]:
        return {
            "SHP":    "Safe hold position",
            "SR":     "Safe retreat",
            "SF-O":   "Other safe failure",
            "LVC":    "Low-value collision",
            "HVC":    "High-value collision",
            "AR-LVC": "Low-value collision after risk-aware behavior",
            "AR-HVC": "High-value collision after risk-aware behavior",
            "TO-RD":  "Risk-distant timeout",
            "TO-RP":  "Risk-proximate timeout",
            "TO-US":  "Unsafe timeout",
            "UF-O":   "Other unsafe failure",
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
