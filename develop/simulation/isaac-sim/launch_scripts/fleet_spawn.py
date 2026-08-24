#!/usr/bin/env python
"""Generic Isaac Sim fleet spawner (RFC #380 §2).

Replaces the hardcoded one-/multi-drone example launch scripts when a fleet is
selected: spawn positions, per-robot vehicle (pass-through → the Pegasus Iris
asset today), sensor toggles, and the scene all come from the fleet file named
by ``FLEET_CONFIG_FILE`` (``airstack up --fleet <name>`` exports it and
switches ``ISAAC_SIM_SCRIPT_NAME`` to this script when the default was
untouched).

``FLEET_CONFIG_FILE`` carries the ROBOT-container path
(``/root/AirStack/config/fleets/<name>.yaml``); this container mounts the
checkout at ``/isaac-sim/AirStack``, so the path is remapped onto that mount.

Fleet fields consumed:
 - ``robots.<name>.spawn``  — [x, y, z] in meters (default [0, 0, 0.07])
 - ``robots.<name>``        — order defines domain_id / vehicle_id (robot N →
   domain N, ``network.domain_policy: auto``)
 - vehicle manifests (``config/vehicles/<v>/vehicle.yaml``) — any ``lidar*``
   sensor entry enables the RTX lidar subgraph for that robot (the
   ``ENABLE_LIDAR`` env-var equivalent, but per vehicle); any ``stereo_cam``
   entry enables the ZED camera subgraph.
 - ``sim.scene`` — ``default`` (or unset) = the Pegasus "Default Environment"
   (matching the example scripts); any other value: a
   ``SIMULATION_ENVIRONMENTS`` key, or a ``.usd`` path/URL used verbatim.

Env (see pegasus_app.py): ISAAC_SIM_LIVESTREAM, ISAAC_SIM_HEADLESS,
PLAY_SIM_ON_START.

Import-order contract: everything Isaac/Pegasus (including pegasus_app, which
imports ``carb`` at module scope) is imported inside ``main()`` — the
fleet-parsing helpers below are stdlib+PyYAML only, so this module parses and
imports without an Isaac install (unit tests exercise the mapping directly).
"""

import os
import sys

import yaml

# Robot-container checkout root → this container's checkout mount.
ROBOT_CONTAINER_ROOT = "/root/AirStack"
ISAAC_CONTAINER_ROOT = "/isaac-sim/AirStack"

DEFAULT_SPAWN_Z = 0.07


def remap_fleet_path(fleet_config_file, isaac_root=ISAAC_CONTAINER_ROOT):
    """Map the robot-container FLEET_CONFIG_FILE path onto this container."""
    if fleet_config_file.startswith(ROBOT_CONTAINER_ROOT + "/"):
        return isaac_root + fleet_config_file[len(ROBOT_CONTAINER_ROOT):]
    return fleet_config_file


def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def vehicle_sensor_flags(project_root, vehicle_name):
    """(has_stereo_cam, has_lidar) from the vehicle manifest's sensor list.

    A missing/invalid manifest keeps the permissive defaults (camera on,
    lidar on) so a mis-mounted config degrades loudly-visibly, not silently
    sensor-less.
    """
    manifest = os.path.join(project_root, "config", "vehicles", vehicle_name, "vehicle.yaml")
    if not os.path.isfile(manifest):
        print(f"[fleet_spawn] WARNING: no vehicle manifest at {manifest} — "
              f"defaulting to camera+lidar on")
        return True, True
    sensors = load_yaml(manifest).get("sensors") or []
    has_cam = any("cam" in str(s.get("type", "")) for s in sensors if isinstance(s, dict))
    has_lidar = any("lidar" in str(s.get("type", "")) for s in sensors if isinstance(s, dict))
    return has_cam, has_lidar


def fleet_to_drone_configs(fleet, project_root):
    """Fleet dict → PegasusApp drone_configs (pure function; unit-tested).

    Robot N (1-based file order) gets domain_id N — the ``domain_policy: auto``
    rule, matching the legacy resolver and ``row_spawn_configs``.
    """
    robots = fleet.get("robots") or {}
    if not robots:
        raise ValueError("fleet has no robots: — nothing to spawn")
    defaults = fleet.get("defaults") or {}
    configs = []
    for i, (name, entry) in enumerate(robots.items(), start=1):
        entry = entry or {}
        spawn = entry.get("spawn", [0.0, 0.0, DEFAULT_SPAWN_Z])
        vehicle = entry.get("vehicle", defaults.get("vehicle", ""))
        has_cam, has_lidar = vehicle_sensor_flags(project_root, vehicle)
        configs.append({
            "domain_id": i,           # MAVLink port = 14540 + vehicle_id (= domain_id)
            "robot_name": name,
            "x_m": float(spawn[0]),
            "y_m": float(spawn[1]),
            "z_m": float(spawn[2]),
            "camera": has_cam,
            "lidar": has_lidar,
        })
    if len(configs) == 1:
        # Single-robot fleets must be byte-equivalent to the validated
        # example_one script, including the historical single-drone prim and
        # node names (multi-style names are for multi-drone scenes).
        configs[0]["prim"] = "/World/base_link"
        configs[0]["node_name"] = "PX4Multirotor"
    return configs


def fleet_env_url(fleet, simulation_environments):
    """Resolve ``sim.scene`` against Pegasus SIMULATION_ENVIRONMENTS."""
    scene = (fleet.get("sim") or {}).get("scene", "default")
    if scene in (None, "", "default"):
        return simulation_environments["Default Environment"]
    if scene in simulation_environments:
        return simulation_environments[scene]
    if str(scene).endswith(".usd"):
        return scene
    raise ValueError(
        f"sim.scene '{scene}' is neither a SIMULATION_ENVIRONMENTS key nor a .usd path "
        f"(keys: {', '.join(sorted(simulation_environments))})"
    )


def main():
    fleet_config_file = os.environ.get("FLEET_CONFIG_FILE", "")
    if not fleet_config_file:
        print("[fleet_spawn] ERROR: FLEET_CONFIG_FILE is not set — this script is "
              "selected by `airstack up --fleet <name>`; use the example launch "
              "scripts for fleetless runs.", file=sys.stderr)
        return 1
    fleet_path = remap_fleet_path(fleet_config_file)
    if not os.path.isfile(fleet_path):
        print(f"[fleet_spawn] ERROR: fleet file not found: {fleet_path} "
              f"(from FLEET_CONFIG_FILE={fleet_config_file})", file=sys.stderr)
        return 1
    # <root>/config/fleets/<name>.yaml → <root>
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(fleet_path)))

    fleet = load_yaml(fleet_path)
    drone_configs = fleet_to_drone_configs(fleet, project_root)
    print(f"[fleet_spawn] {os.path.basename(fleet_path)}: spawning "
          f"{len(drone_configs)} drone(s): "
          + ", ".join(
              f"{c['robot_name']}@({c['x_m']:g},{c['y_m']:g},{c['z_m']:g})"
              f"{' +cam' if c['camera'] else ''}{' +lidar' if c['lidar'] else ''}"
              for c in drone_configs))

    # ── Isaac/Pegasus imports — deferred (see module docstring) ──────────────
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from pegasus_app import create_simulation_app

    # Must be created before any omni/pegasus imports.
    create_simulation_app()

    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
    from pegasus_app import PegasusApp  # noqa: E402

    PegasusApp(
        env_url=fleet_env_url(fleet, SIMULATION_ENVIRONMENTS),
        stage_scale=1.0,
        drone_configs=drone_configs,
        # Per-robot "camera"/"lidar" keys above override the app-level
        # defaults (enable_camera defaults True in PegasusApp).
        enable_lidar=False,
    ).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
