"""Velocity baselines, vendored from the S.A.F.E. reference integration
(integrations/aerial_nav/agents/baselines.py) so this integration depends on
the installed safe_core package only — integrations are not installed.

Actions are normalized velocities in [-1, 1]^3; AirstackStackEnv executes
them as trajectory_override segments on the real trajectory controller.
"""

from __future__ import annotations

import numpy as np

from safe_core.core.policy import Policy


def _wall_repulsion(pos, env_bounds, repulsion_radius=1.0, repulsion_gain=1.5):
    """APF repulsion from the bounding planes.
    env_bounds: [x_min, y_min, z_min, x_max, y_max, z_max]."""
    dim = len(pos)
    force = np.zeros(dim)
    b = env_bounds
    walls = [
        (0, +1.0, b[0]),
        (0, -1.0, b[3]),
        (1, +1.0, b[1]),
        (1, -1.0, b[4]),
    ]
    if dim > 2:
        walls += [
            (2, +1.0, b[2]),
            (2, -1.0, b[5]),
        ]
    for axis, sign, wall_coord in walls:
        dist = max(sign * (pos[axis] - wall_coord), 1e-6)
        if dist < repulsion_radius:
            mag = repulsion_gain * (1.0 / dist - 1.0 / repulsion_radius) / dist
            normal = np.zeros(dim)
            normal[axis] = sign
            force += mag * normal
    return force


class Random(Policy):
    """Uniform random baseline — samples a velocity direction each step."""

    sensors = ["full_state"]

    def reset(self):
        pass

    def act(self, obs):
        dim = len(obs["agent"])
        return np.random.uniform(-1.0, 1.0, size=dim).astype(np.float32)


class Aggressive(Policy):
    """Full-speed beeline to the goal, no obstacle avoidance — establishes
    the worst-case risk exposure / collision-rate reference."""

    sensors = ["full_state"]

    def reset(self):
        pass

    def act(self, obs):
        direction = obs["target"] - obs["agent"]
        dist = np.linalg.norm(direction)
        if dist < 1e-8:
            return np.zeros(len(obs["agent"]), dtype=np.float32)
        goal_dir = direction / dist
        if obs.get("env_bounds") is not None:
            combined = goal_dir + _wall_repulsion(obs["agent"], obs["env_bounds"])
            norm = np.linalg.norm(combined)
            action = combined / norm if norm > 1e-8 else goal_dir
        else:
            action = goal_dir
        return action.astype(np.float32)


class Conservative(Policy):
    """Moves toward the goal but slows proportionally as obstacles approach,
    and retreats inside stop_radius."""

    sensors = ["full_state"]

    def __init__(self, safe_radius=1.0, stop_radius=0.5):
        self.safe_radius = safe_radius
        self.stop_radius = stop_radius

    def reset(self):
        pass

    def _all_obstacle_positions(self, obs):
        dim = len(obs["agent"])
        positions = list(obs["static_obstacles"])
        cylinders = obs.get("static_cylinders")
        if cylinders is not None and len(cylinders):
            positions.extend(np.asarray(cylinders)[:, :dim])
        if obs.get("dynamic_obstacles") is not None and len(obs["dynamic_obstacles"]):
            positions.extend(obs["dynamic_obstacles"][:, :dim])
        return positions

    def act(self, obs):
        pos = obs["agent"]
        target = obs["target"]
        dim = len(pos)

        to_goal = target - pos
        dist_to_goal = np.linalg.norm(to_goal)
        if dist_to_goal < 1e-8:
            return np.zeros(dim, dtype=np.float32)
        goal_dir = to_goal / dist_to_goal

        min_dist = float("inf")
        closest_pos = None
        for obs_pos in self._all_obstacle_positions(obs):
            d = np.linalg.norm(pos - np.asarray(obs_pos))
            if d < min_dist:
                min_dist = d
                closest_pos = np.asarray(obs_pos)

        if obs.get("env_bounds") is not None:
            b = obs["env_bounds"]
            wall_dists = [
                (pos[0] - b[0], np.array([b[0], pos[1], pos[2]] if dim > 2 else [b[0], pos[1]])),
                (b[3] - pos[0], np.array([b[3], pos[1], pos[2]] if dim > 2 else [b[3], pos[1]])),
                (pos[1] - b[1], np.array([pos[0], b[1], pos[2]] if dim > 2 else [pos[0], b[1]])),
                (b[4] - pos[1], np.array([pos[0], b[4], pos[2]] if dim > 2 else [pos[0], b[4]])),
            ]
            for wd, wp in wall_dists:
                wd = max(wd, 1e-6)
                if wd < min_dist:
                    min_dist = wd
                    closest_pos = wp[:dim]

        if closest_pos is None or min_dist >= self.safe_radius:
            return goal_dir.astype(np.float32)

        if min_dist < self.stop_radius:
            away = pos - closest_pos
            away_norm = np.linalg.norm(away)
            if away_norm > 1e-8:
                return (away / away_norm).astype(np.float32)
            return np.zeros(dim, dtype=np.float32)

        speed = (min_dist - self.stop_radius) / (self.safe_radius - self.stop_radius)
        return (goal_dir * float(np.clip(speed, 0.0, 1.0))).astype(np.float32)
