"""Scenario configuration for AirStack × S.A.F.E.

Copied and adapted from integrations/aerial_nav/sim/scenario_config.py.
The obstacle generation logic is identical; the AirStack-specific part is
making configure_scenario work with AirStack's USD prim structure.

Functions exposed:
    configure_scenario(env) -> dict      — called by AirstackEnv.reset()
    perturb_scenario(env, seed, range)   — resamples obstacle speeds per perturbation
"""

from __future__ import annotations

import numpy as np
from typing import Tuple


# ── Helpers ───────────────────────────────────────────────────────────────────

def _benchmark_floor_bounds(env):
    """Return (origin_x, origin_y, extent_x, extent_y) for navigable floor sampling."""
    ox = getattr(env.env_cfg, "origin_x", 0.0) if env.env_cfg else 0.0
    oy = getattr(env.env_cfg, "origin_y", 0.0) if env.env_cfg else 0.0
    return float(ox), float(oy), float(env.size_x), float(env.size_y)


def sample_random_velocity(speed, dimension, rng):
    vec = rng.standard_normal(dimension)
    norm = np.linalg.norm(vec)
    if norm < 1e-10:
        vec = np.zeros(dimension)
        vec[0] = 1.0
        norm = 1.0
    return ((vec / norm) * speed).astype(np.float32)


def perturb_scenario(env, perturbation_seed: int, scale_range: Tuple[float, float]) -> None:
    """Resample dynamic obstacle speeds and directions for a perturbation.

    Modifies both _dynamic_obstacle_velocities and _initial_dynamic_obstacle_velocities
    so that subsequent reset(same_seed) calls restore the perturbed (not original) speeds.
    """
    if not hasattr(env, "num_dynamic_obstacles") or env.num_dynamic_obstacles == 0:
        return

    n_active = getattr(env, "_active_dynamic_obstacles", env.num_dynamic_obstacles)
    rng = np.random.RandomState(perturbation_seed)

    nominal = float(env.dynamic_obstacle_speed)
    m = float(rng.uniform(scale_range[0], scale_range[1]))
    speed = max(1e-6, nominal * m)

    dim = env._dynamic_obstacle_velocities.shape[1]
    for i in range(n_active):
        vel = sample_random_velocity(speed, dim, rng)
        if dim > 2:
            vel[2] = 0.0
        env._dynamic_obstacle_velocities[i] = vel
        env._initial_dynamic_obstacle_velocities[i] = vel.copy()


def _apply_obstacle_separation(positions, velocities, n_active, radius, k_rep=0.8):
    d0 = radius * 2.0
    for i in range(n_active):
        for j in range(i + 1, n_active):
            sep = positions[i, :2] - positions[j, :2]
            dist = float(np.linalg.norm(sep))
            if dist < d0 and dist > 1e-6:
                push = k_rep * (1.0 - dist / d0) * (sep / dist)
                velocities[i, :2] += push
                velocities[j, :2] -= push


def update_dynamic_obstacles(
    positions,
    velocities,
    size_x,
    size_y,
    radius,
    origin_x=0.0,
    origin_y=0.0,
    n_active=None,
    waypoints=None,
    rng=None,
    dt=0.1,
):
    """Advance dynamic obstacle positions one timestep (waypoint motion)."""
    positions = np.asarray(positions, dtype=np.float32)
    velocities = np.asarray(velocities, dtype=np.float32)
    n = len(positions)
    if n_active is None:
        n_active = n
    n_active = max(0, min(int(n_active), n))

    if waypoints is None:
        # bounce fallback
        new_positions = positions.copy()
        if n_active > 0:
            new_positions[:n_active] = positions[:n_active] + velocities[:n_active] * dt
        for i in range(n_active):
            for d in range(min(2, positions.shape[1])):
                mn = (origin_x if d == 0 else origin_y) + radius
                mx = (origin_x + size_x if d == 0 else origin_y + size_y) - radius
                if new_positions[i, d] < mn:
                    new_positions[i, d] = mn
                    velocities[i, d] *= -1
                elif new_positions[i, d] > mx:
                    new_positions[i, d] = mx
                    velocities[i, d] *= -1
        _apply_obstacle_separation(new_positions, velocities, n_active, radius)
        return new_positions.astype(np.float32), velocities

    _rng = rng if rng is not None else np.random
    new_positions = positions.copy()
    arrival_thresh = max(float(radius) * 2.0, 1.0)
    mn_x, mx_x = origin_x + radius, origin_x + size_x - radius
    mn_y, mx_y = origin_y + radius, origin_y + size_y - radius

    for i in range(n_active):
        pos = positions[i]
        diff = waypoints[i, :2] - pos[:2]
        dist = float(np.linalg.norm(diff))

        if dist < arrival_thresh:
            waypoints[i, 0] = float(_rng.uniform(mn_x, mx_x))
            waypoints[i, 1] = float(_rng.uniform(mn_y, mx_y))
            if waypoints.shape[1] > 2:
                waypoints[i, 2] = 0.0
            diff = waypoints[i, :2] - pos[:2]
            dist = float(np.linalg.norm(diff))

        curr_speed = float(np.linalg.norm(velocities[i, :2]))
        if curr_speed < 1e-8:
            curr_speed = float(np.linalg.norm(velocities[i]))
        if curr_speed < 1e-8:
            curr_speed = 0.05

        if dist > 1e-6:
            velocities[i, 0] = (diff[0] / dist) * curr_speed
            velocities[i, 1] = (diff[1] / dist) * curr_speed
            if velocities.shape[1] > 2:
                velocities[i, 2] = 0.0

        new_positions[i] = positions[i] + velocities[i] * dt
        new_positions[i, 0] = np.clip(new_positions[i, 0], mn_x, mx_x)
        new_positions[i, 1] = np.clip(new_positions[i, 1], mn_y, mx_y)

    _apply_obstacle_separation(new_positions, velocities, n_active, radius)
    return new_positions.astype(np.float32), velocities


def generate_agent_start(env, static_obstacle_locations=None):
    margin = env.agent_radius
    ox, oy, sx, sy = _benchmark_floor_bounds(env)
    low = np.array([ox + margin, oy + margin] + [margin] * (env.dimension - 2), dtype=np.float32)
    high = np.array(
        [ox + sx - margin, oy + sy - margin] + [max(sx, sy) - margin] * (env.dimension - 2),
        dtype=np.float32,
    )
    r_unsafe = getattr(env, "r_unsafe", env.agent_radius)
    static_aabbs = getattr(env, "static_obstacle_aabbs", np.zeros((0, 4), dtype=np.float32))
    for _ in range(1000):
        pos = env.np_random.uniform(low, high).astype(np.float32)
        if env.dimension > 2:
            pos[2] = env.env_cfg.flight_z if env.env_cfg else 1.5
        if static_obstacle_locations is not None and len(static_obstacle_locations) > 0:
            dists = np.linalg.norm(static_obstacle_locations - pos, axis=1)
            if dists.min() <= r_unsafe:
                continue
        if len(static_aabbs) > 0:
            p2d = pos[:2]
            if np.any(
                (static_aabbs[:, 0] <= p2d[0]) & (p2d[0] <= static_aabbs[:, 2]) &
                (static_aabbs[:, 1] <= p2d[1]) & (p2d[1] <= static_aabbs[:, 3])
            ):
                continue
        return pos
    return pos


def generate_target(env, agent_location):
    margin = env.agent_radius
    min_distance = min(env.size_x, env.size_y) * 0.6
    ox, oy, sx, sy = _benchmark_floor_bounds(env)
    low = np.array([ox + margin, oy + margin] + [margin] * (env.dimension - 2), dtype=np.float32)
    high = np.array(
        [ox + sx - margin, oy + sy - margin] + [max(sx, sy) - margin] * (env.dimension - 2),
        dtype=np.float32,
    )
    target = agent_location.copy()
    while np.linalg.norm(target - agent_location) < min_distance:
        target = env.np_random.uniform(low, high).astype(np.float32)
        if env.dimension > 2:
            target[2] = 0.0
    return target


def generate_dynamic_obstacles(env, agent_location, target_location, static_obstacle_locations,
                                flight_z=1.5):
    min_spacing = env.obstacle_radius * 2 + 0.2
    margin = env.obstacle_radius
    n_active = getattr(env, "_active_dynamic_obstacles", env.num_dynamic_obstacles)
    dim = env.dimension
    dynamic_locs = np.zeros((env.num_dynamic_obstacles, dim), dtype=np.float32)
    dynamic_vels = np.zeros((env.num_dynamic_obstacles, dim), dtype=np.float32)

    ox, oy, sx, sy = _benchmark_floor_bounds(env)
    low = np.array([ox + margin, oy + margin] + [0.0] * (dim - 2), dtype=np.float32)
    high = np.array([ox + sx - margin, oy + sy - margin] + [0.0] * (dim - 2), dtype=np.float32)

    for i in range(n_active):
        for attempt in range(100):
            candidate = env.np_random.uniform(low, high).astype(np.float32)
            if dim > 2:
                candidate[2] = 0.0  # ground level — obstacles are pedestrians, not airborne

            cand_xy = candidate[:2]
            if np.linalg.norm(cand_xy - agent_location[:2]) < min_spacing:
                continue
            if np.linalg.norm(cand_xy - target_location[:2]) < min_spacing:
                continue
            if any(np.linalg.norm(cand_xy - s[:2]) < min_spacing for s in static_obstacle_locations):
                continue
            if any(np.linalg.norm(cand_xy - dynamic_locs[j, :2]) < min_spacing for j in range(i)):
                continue

            dynamic_locs[i] = candidate
            vel = sample_random_velocity(env.dynamic_obstacle_speed, dim, env.np_random)
            if dim > 2:
                vel[2] = 0.0
            dynamic_vels[i] = vel
            break
        else:
            # fallback
            for _ in range(200):
                loc = env.np_random.uniform(low, high).astype(np.float32)
                if dim > 2:
                    loc[2] = 0.0
                if np.linalg.norm(loc[:2] - agent_location[:2]) >= min_spacing:
                    dynamic_locs[i] = loc
                    break
            vel = sample_random_velocity(env.dynamic_obstacle_speed, dim, env.np_random)
            if dim > 2:
                vel[2] = 0.0
            dynamic_vels[i] = vel

    # Park inactive prims underground
    for i in range(n_active, env.num_dynamic_obstacles):
        dynamic_locs[i] = 0.0
        if dim > 2:
            dynamic_locs[i, 2] = -100.0
        dynamic_vels[i] = 0.0

    return dynamic_locs, dynamic_vels


def _read_obstacle_positions_from_stage(prim_paths, dimension, origin_z=0.0,
                                         world_offset_isaac=(0.0, 0.0)):
    """Read world-space benchmark-frame positions for named USD prims."""
    from isaacsim.core.prims import XFormPrim

    ox, oy = world_offset_isaac
    positions = []
    for path in prim_paths:
        xform = XFormPrim(path)
        pos, _ = xform.get_world_poses()
        p = pos[0]
        positions.append([float(p[1]) - oy, float(p[0]) - ox, float(p[2]) - origin_z][:dimension])
    return np.array(positions, dtype=np.float32)


def _read_obstacle_aabbs_from_stage(prim_paths, world_offset_isaac=(0.0, 0.0)):
    """Read 2D XY axis-aligned bounding boxes (benchmark coords) for static obstacle prims."""
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    ox, oy = world_offset_isaac
    aabbs = []
    for path in prim_paths:
        prim = stage.GetPrimAtPath(path)
        try:
            bbox = cache.ComputeWorldBound(prim)
            rng = bbox.GetRange()
            mn, mx = rng.GetMin(), rng.GetMax()
            bx_min = float(mn[1]) - oy
            bx_max = float(mx[1]) - oy
            by_min = float(mn[0]) - ox
            by_max = float(mx[0]) - ox
            if not all(np.isfinite([bx_min, by_min, bx_max, by_max])):
                continue
            if bx_max <= bx_min or by_max <= by_min:
                continue
            if any(abs(v) > 1e9 for v in [bx_min, by_min, bx_max, by_max]):
                continue
            aabbs.append([bx_min, by_min, bx_max, by_max])
        except Exception:
            pass
    return np.array(aabbs, dtype=np.float32) if aabbs else np.zeros((0, 4), dtype=np.float32)


def generate_static_cylinders(env, num_cylinders, agent_location, target_location,
                               static_obstacle_locations):
    if num_cylinders <= 0:
        return np.zeros((0, env.dimension), dtype=np.float32)

    margin = env.obstacle_radius
    min_spacing = env.obstacle_radius * 2 + 0.2
    ox, oy, sx, sy = _benchmark_floor_bounds(env)
    low = np.array([ox + margin, oy + margin] + [0.0] * (env.dimension - 2), dtype=np.float32)
    high = np.array([ox + sx - margin, oy + sy - margin] + [0.0] * (env.dimension - 2), dtype=np.float32)

    locs = np.zeros((num_cylinders, env.dimension), dtype=np.float32)
    placed = 0

    for _ in range(num_cylinders * 50):
        if placed >= num_cylinders:
            break
        candidate = env.np_random.uniform(low, high).astype(np.float32)
        candidate[2:] = 0.0
        if np.linalg.norm(candidate[:2] - agent_location[:2]) < min_spacing:
            continue
        if np.linalg.norm(candidate[:2] - target_location[:2]) < min_spacing:
            continue
        if any(np.linalg.norm(candidate[:2] - locs[j, :2]) < min_spacing for j in range(placed)):
            continue
        locs[placed] = candidate
        placed += 1

    for i in range(placed, num_cylinders):
        locs[i] = env.np_random.uniform(low, high).astype(np.float32)
        locs[i, 2:] = 0.0

    return locs


def configure_scenario(env):
    """Place drone start, goal, static obstacles, and dynamic obstacles for one scenario seed.

    Returns a dict consumed by AirstackEnv.reset().
    """
    cfg = getattr(env, "env_cfg", None)
    flight_z = cfg.flight_z if cfg else 1.5

    sim = getattr(env, "sim", None)
    env_prim_paths = getattr(sim, "_env_obstacle_prim_paths", None) or (
        cfg.static_obstacle_prim_paths if cfg else None
    )
    world_offset = getattr(sim, "_world_offset_isaac", (0.0, 0.0)) if sim is not None else (0.0, 0.0)

    if env_prim_paths:
        oz = float(getattr(cfg, "origin_z", 0.0))
        static_locs = _read_obstacle_positions_from_stage(
            env_prim_paths, env.dimension, origin_z=oz, world_offset_isaac=world_offset
        )
        env.num_obstacles = len(static_locs)
        env.static_obstacle_aabbs = _read_obstacle_aabbs_from_stage(
            env_prim_paths, world_offset_isaac=world_offset
        )
    else:
        static_locs = np.zeros((0, env.dimension), dtype=np.float32)
        env.num_obstacles = 0
        env.static_obstacle_aabbs = np.zeros((0, 4), dtype=np.float32)

    agent = generate_agent_start(env, static_obstacle_locations=static_locs)
    target = generate_target(env, agent)

    dynamic_locs, dynamic_vels = generate_dynamic_obstacles(
        env, agent, target, static_locs, flight_z=flight_z
    )

    n_dyn = env.num_dynamic_obstacles
    n_active = getattr(env, "_active_dynamic_obstacles", n_dyn)
    ox, oy, sx, sy = _benchmark_floor_bounds(env)
    margin = env.obstacle_radius
    dynamic_waypoints = np.zeros((n_dyn, env.dimension), dtype=np.float32)
    for i in range(n_active):
        dynamic_waypoints[i, 0] = float(env.np_random.uniform(ox + margin, ox + sx - margin))
        dynamic_waypoints[i, 1] = float(env.np_random.uniform(oy + margin, oy + sy - margin))

    num_scyl = int(getattr(cfg, "num_static_cylinders", 0)) if cfg else 0
    static_cylinder_locs = generate_static_cylinders(env, num_scyl, agent, target, static_locs)

    return {
        "agent_location": agent,
        "target_location": target,
        "static_obstacle_locations": static_locs,
        "dynamic_obstacle_locations": dynamic_locs,
        "dynamic_obstacle_velocities": dynamic_vels,
        "dynamic_obstacle_waypoints": dynamic_waypoints,
        "static_cylinder_locations": static_cylinder_locs,
    }
