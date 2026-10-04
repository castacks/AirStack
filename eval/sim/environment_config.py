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
    source_domain: str | None = None
    # USD path relative to the Isaac Nucleus root (same paths used by AirStack's Pegasus scenes)
    usd_path: str | None = None
    size_x: float = 20.0
    size_y: float = 20.0
    size_z: float = 5.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0
    origin_z: float = 0.0
    flight_z: float = 1.5
    stage_scale: float = 1.0  # /World/stage scale (0.01 for cm-unit Nucleus assets)
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
    """Project SAFE's canonical Isaac domain registry into AirStack coordinates.

    SAFE writes domain extents in its X/Y-swapped benchmark frame. AirStack
    positions are Isaac-world ENU, so the projection swaps X/Y once. USD
    bounds are measured again by SAFE's world builder inside Isaac at boot.
    """
    if sim_backend not in (None, 'isaac'):
        return {}
    if __import__('os').environ.get('AIRSTACK_SAFE_LEGACY_DOMAINS') == '1':
        return dict(_REGISTRY)
    try:
        import integrations.aerial_nav.config.domains_isaac  # noqa: F401
        from integrations.aerial_nav.config.domains import get_domain_registry as source_registry
    except ImportError as exc:
        raise RuntimeError('AirStack SAFE domains require the SAFE source checkout') from exc
    result = {}
    for name, src in source_registry('isaac').items():
        result[name] = AirstackDomainCfg(
            domain=name, source_domain=name,
            usd_path=src.usd_path,
            size_x=float(src.size_y), size_y=float(src.size_x),
            size_z=float(src.size_z),
            origin_x=float(src.origin_y), origin_y=float(src.origin_x),
            origin_z=float(src.origin_z),
            stage_scale=float(src.stage_scale or 1.0),
            num_dynamic_obstacles=int(src.num_dynamic_obstacles),
            num_static_cylinders=int(src.num_static_cylinders),
            dynamic_obstacle_speed=float(src.dynamic_obstacle_speed),
            dynamic_obstacle_usd=src.dynamic_obstacle_usd,
            obstacle_layout=src.obstacle_layout,
            outdoor=bool(src.outdoor),
            flight_z=2.5,
        )
    return result


def max_dynamic_obstacles() -> int:
    return max((cfg.num_dynamic_obstacles for cfg in get_domain_registry().values()), default=0)


def max_static_obstacles() -> int:
    return max((cfg.num_static_cylinders for cfg in get_domain_registry().values()), default=0)


def switch_domain(env, domain_cfg: AirstackDomainCfg) -> None:
    """Advance a live AirstackStackEnv to a new domain.

    Domain scenes are baked into the Isaac bring-up (SAFE_EVAL_CONFIG env),
    so a *different* domain means a stack restart: _apply_domain_cfg updates
    the env's config, and the next reset()'s ensure_up() sees a changed
    fingerprint and cycles `airstack down` / `airstack up` automatically.
    """
    current = getattr(getattr(env, "env_cfg", None), "domain", None)
    if current == domain_cfg.domain:
        return
    print(f"[airstack.environment_config] Switching to domain '{domain_cfg.domain}' "
          f"(stack restart on next reset)...", flush=True)
    env._apply_domain_cfg(domain_cfg)


# ── Domain registrations ──────────────────────────────────────────────────────
# These mirror AirStack's existing Isaac Sim launch scripts.
# USD paths match what Pegasus / AirStack's warehouse_px4_pegasus_launch_script.py uses.

register_domain(AirstackDomainCfg(
    domain="warehouse",
    usd_path="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/male_adult_construction_05/male_adult_construction_05.usd",
    flight_z=1.5,
    stage_scale=0.01,
    origin_x=-10.0,
    origin_y=-10.0,
    num_dynamic_obstacles=30,
    num_static_cylinders=8,
    dynamic_obstacle_speed=0.9,
))

register_domain(AirstackDomainCfg(
    domain="hospital",
    usd_path="/Isaac/Environments/Hospital/hospital.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/female_adult_police_01/female_adult_police_01.usd",
    flight_z=1.5,
    origin_x=-10.0,
    origin_y=-10.0,
    num_dynamic_obstacles=25,
    num_static_cylinders=8,
    dynamic_obstacle_speed=0.75,
))

register_domain(AirstackDomainCfg(
    domain="office",
    usd_path="/Isaac/Environments/Office/office.usd",
    dynamic_obstacle_usd="/Isaac/People/Characters/male_adult_business_02/male_adult_business_02.usd",
    flight_z=1.5,
    origin_x=-10.0,
    origin_y=-10.0,
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
    origin_x=-20.0,
    origin_y=-20.0,
    num_dynamic_obstacles=40,
    num_static_cylinders=12,
    dynamic_obstacle_speed=0.9,
    outdoor=True,
))

# Sanity-check domain: no obstacles, just the goal. Flight altitude clears the
# stack planner's 2 m obstacle inflation of the floor, and the arena is wide
# enough that its 7 s trajectory horizon clears the wall inflation past a goal.
register_domain(AirstackDomainCfg(
    domain="empty",
    usd_path=None,
    flight_z=2.5,
    size_x=30.0,
    size_y=30.0,
    origin_x=-15.0,
    origin_y=-15.0,
    num_static_cylinders=0,
    num_dynamic_obstacles=0,
    dynamic_obstacle_speed=0.0,
))
