"""Load SAFE's authored Isaac world and people without owning its drone.

AirStack remains responsible for the Pegasus PX4/ROS vehicle. All stage,
ground-map, and character construction below calls the mounted SAFE checkout.
This module is imported only after Isaac's SimulationApp has started.
"""

from __future__ import annotations


def load_domain(name: str, goal_distance: float | None = None):
    # Importing the registrations fills SAFE's canonical Isaac registry.
    import integrations.aerial_nav.config.domains_isaac  # noqa: F401
    from integrations.aerial_nav.config.domains import (
        get_domain_registry, scale_procedural_domains)
    scale_procedural_domains(goal_distance, sim_backend='isaac')
    from safe_isaac.world_builder import (
        add_ground_collider, load_environment_usd, setup_lighting, setup_sky,
        setup_outdoor_ground, setup_room_bounds, spawn_blocker_wall,
    )

    domains = get_domain_registry('isaac')
    if name not in domains:
        raise ValueError(f"SAFE has no Isaac domain {name!r}")
    cfg = domains[name]
    prefix = 'safe_eval'
    if cfg.usd_path:
        load_environment_usd(cfg, env_prefix=prefix)
    else:
        if cfg.outdoor:
            setup_sky(cfg.size_x, cfg.size_y)
        else:
            setup_lighting(cfg.size_x, cfg.size_y)
        add_ground_collider(prefix, cfg)
        if cfg.outdoor:
            setup_outdoor_ground(
                cfg.size_x, cfg.size_y, env_prefix=prefix,
                origin_x=cfg.origin_x, origin_y=cfg.origin_y,
                ground_material=cfg.ground_material)
        else:
            setup_room_bounds(cfg.size_x, cfg.size_y, env_prefix=prefix,
                              origin_x=cfg.origin_x, origin_y=cfg.origin_y)
            if cfg.wall_x is not None:
                spawn_blocker_wall(cfg.wall_x, cfg.size_y, env_prefix=prefix)
    return cfg


def create_humans(cfg, *, seed: int = 0):
    from isaacsim.storage.native import get_assets_root_path
    from safe_isaac.controllers.cct_human_controller import CctHumanController

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError('SAFE human actors require the Isaac asset root')
    humans = CctHumanController(
        num_humans=cfg.num_dynamic_obstacles,
        assets_root=assets_root,
        env_size_x=cfg.size_x,
        env_size_y=cfg.size_y,
        env_origin_x=cfg.origin_x,
        env_origin_y=cfg.origin_y,
        env_origin_z=cfg.origin_z,
        nav_collision_slug='airstack',
        character_usd=cfg.dynamic_obstacle_usd,
        ground_map=getattr(cfg, '_ground_map', None),
        floor_z=cfg.floor_z,
        distinct_characters=cfg.distinct_characters,
        seed=seed,
    )
    humans.setup()
    return humans


def sample_scenario(cfg, *, seed: int, goal_distance: float | None,
                    perturbation_seed: int | None, humans: bool,
                    agent_radius: float, obstacle_radius: float):
    """Call SAFE's canonical seeded aerial scenario sampler inside Isaac.

    It must run in the Isaac process: photoreal scenes inspect live USD prims
    and ground maps while choosing starts, goals and static AABBs.
    """
    from types import SimpleNamespace

    import numpy as np

    from integrations.aerial_nav.config.scenarios import (
        configure_scenario, perturb_scenario)
    from safe_isaac import sim_config

    sim_config.USE_HUMANS = bool(humans)
    sim = SimpleNamespace(
        _env_obstacle_prim_paths=cfg.static_obstacle_prim_paths,
        _world_offset_isaac=(0.0, 0.0),
    )
    env = SimpleNamespace(
        env_cfg=cfg,
        sim=sim,
        dimension=3,
        size_x=float(cfg.size_x),
        size_y=float(cfg.size_y),
        np_random=np.random.default_rng(int(seed)),
        goal_distance=goal_distance,
        agent_radius=float(agent_radius),
        obstacle_radius=float(obstacle_radius),
        r_unsafe=float(agent_radius + obstacle_radius + 0.3),
        num_dynamic_obstacles=int(cfg.num_dynamic_obstacles),
        dynamic_obstacle_speed=(float(sim_config.HUMAN_WALK_SPEED) if humans
                                else float(cfg.dynamic_obstacle_speed)),
        _active_dynamic_obstacles=int(cfg.num_dynamic_obstacles),
        _human_behavior=None,
        static_obstacle_aabbs=np.zeros((0, 4), dtype=np.float32),
    )
    layout = configure_scenario(env)
    env._dynamic_obstacle_velocities = layout['dynamic_obstacle_velocities'].copy()
    env._initial_dynamic_obstacle_velocities = layout['dynamic_obstacle_velocities'].copy()
    if perturbation_seed is not None:
        perturb_scenario(env, int(perturbation_seed), (0.8, 1.2))
        layout['dynamic_obstacle_velocities'] = env._initial_dynamic_obstacle_velocities.copy()
    return layout, env._human_behavior
