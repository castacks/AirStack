"""Domain registry for AirStack × S.A.F.E.

Maps domain names to AirStack scene configurations (USD paths from Isaac Sim Nucleus,
obstacle counts, flight height).  One entry per scene from AirStack's simulation/isaac-sim/
that you want to evaluate.

switch_domain() updates a live AirstackEnv when the benchmark moves to a new domain.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class AirstackDomainCfg:
    domain: str
    # USD path relative to the Isaac Nucleus root (same paths used by AirStack's Pegasus scenes)
    usd_path: str | None = None
    size_x: float = 20.0
    size_y: float = 20.0
    size_z: float = 5.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0
    flight_z: float = 1.5
    num_dynamic_obstacles: int = 20
    num_static_cylinders: int = 8
    dynamic_obstacle_speed: float = 0.9
    # Prim paths for static mesh obstacles; populated after USD load via auto-discovery
    static_obstacle_prim_paths: list[str] | None = None
    # USD asset for dynamic obstacle actors (pedestrians / robots)
    dynamic_obstacle_usd: str | None = None
    obstacle_layout: str = "random"
    outdoor: bool = False


_REGISTRY: dict[str, AirstackDomainCfg] = {}


def register_domain(cfg: AirstackDomainCfg) -> None:
    _REGISTRY[cfg.domain] = cfg


def get_domain_registry(sim_backend: str | None = None) -> dict[str, AirstackDomainCfg]:
    """Return the full registry.  sim_backend is accepted for API compatibility but ignored
    (AirStack always uses Isaac Sim)."""
    return dict(_REGISTRY)


def max_dynamic_obstacles() -> int:
    return max((cfg.num_dynamic_obstacles for cfg in _REGISTRY.values()), default=0)


def max_static_obstacles() -> int:
    return max((cfg.num_static_cylinders for cfg in _REGISTRY.values()), default=0)


def switch_domain(env, domain_cfg: AirstackDomainCfg) -> None:
    """Update a live AirstackEnv when the benchmark advances to a new domain."""
    envs = env if isinstance(env, list) else [env]

    if envs and getattr(envs[0], "env_cfg", None) is not None:
        current = getattr(envs[0].env_cfg, "domain", None)
        if current != domain_cfg.domain:
            print(
                f"[airstack.environment_config] Switching to domain '{domain_cfg.domain}'...",
                flush=True,
            )
            for e in envs:
                e.sim.reload_domain(domain_cfg)

    for e in envs:
        e.env_cfg = domain_cfg
        e.size_x = domain_cfg.size_x
        e.size_y = domain_cfg.size_y
        e.size_z = domain_cfg.size_z
        e.origin_x = domain_cfg.origin_x
        e.origin_y = domain_cfg.origin_y
        e.num_dynamic_obstacles = domain_cfg.num_dynamic_obstacles
        e.num_static_cylinders = int(getattr(domain_cfg, "num_static_cylinders", 0))
        e.num_obstacles = len(domain_cfg.static_obstacle_prim_paths or [])
        e.obstacle_values = ["high"] * e.num_dynamic_obstacles + ["low"] * e.num_obstacles
        e._camera_ready = False


# ── Domain registrations ──────────────────────────────────────────────────────
# These mirror AirStack's existing Isaac Sim launch scripts.
# USD paths match what Pegasus / AirStack's warehouse_px4_pegasus_launch_script.py uses.

register_domain(AirstackDomainCfg(
    domain="warehouse",
    usd_path="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/male_adult_construction_05/male_adult_construction_05.usd",
    flight_z=1.5,
    num_dynamic_obstacles=30,
    num_static_cylinders=8,
    dynamic_obstacle_speed=0.9,
))

register_domain(AirstackDomainCfg(
    domain="hospital",
    usd_path="/Isaac/Environments/Hospital/hospital.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/female_adult_police_01/female_adult_police_01.usd",
    flight_z=1.5,
    num_dynamic_obstacles=25,
    num_static_cylinders=8,
    dynamic_obstacle_speed=0.75,
))

register_domain(AirstackDomainCfg(
    domain="office",
    usd_path="/Isaac/Environments/Office/office.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/male_adult_business_02/male_adult_business_02.usd",
    flight_z=1.5,
    num_dynamic_obstacles=20,
    num_static_cylinders=6,
    dynamic_obstacle_speed=0.75,
))

register_domain(AirstackDomainCfg(
    domain="outdoor",
    usd_path="/Isaac/Environments/Outdoor/Rivermark/rivermark_community.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/male_adult_business_02/male_adult_business_02.usd",
    flight_z=2.5,
    size_x=40.0,
    size_y=40.0,
    num_dynamic_obstacles=40,
    num_static_cylinders=12,
    dynamic_obstacle_speed=0.9,
    outdoor=True,
))

# Sanity-check domain: no obstacles, just the goal.
register_domain(AirstackDomainCfg(
    domain="empty",
    usd_path=None,
    flight_z=1.5,
    size_x=15.0,
    size_y=15.0,
    num_static_cylinders=0,
    num_dynamic_obstacles=0,
    dynamic_obstacle_speed=0.0,
))
