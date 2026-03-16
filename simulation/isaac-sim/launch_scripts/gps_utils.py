"""
GPS origin utilities for AirStack multi-drone simulation.

Computes per-robot GPS origins from Isaac Sim world-frame spawn positions and
sets them as process environment variables so the OmniGraph PX4 node picks them
up when building each vehicle's PX4MavlinkBackendConfig.

Usage in a scene script:
    from utils.gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

    DRONE_CONFIGS = [
        {"domain_id": 1, "x_m": -3.0, "y_m":  3.5, "z_m": 0.15, "orient": [0, 0, 0, 1]},
        {"domain_id": 2, "x_m":  3.0, "y_m":  3.0, "z_m": 0.15, "orient": [0, 0, 0, 1]},
    ]
    set_gps_origins(DRONE_CONFIGS)
"""

import math
import os

# GPS coordinates that correspond to world origin (0, 0, 0) in the simulation.
# Zurich default matches the existing px4_config.yaml hardcoded values.
DEFAULT_WORLD_ORIGIN = (47.3667, 8.5500, 408.0)  # (lat_deg, lon_deg, alt_m WGS-84)


def compute_gps_origin(x_m, y_m, z_m, world_origin=DEFAULT_WORLD_ORIGIN):
    """
    Convert a world-frame spawn position (meters) to a GPS geo_origin.

    Axis convention assumed (Isaac Sim stage, Z-up ENU):
        +X = East  →  affects longitude
        +Y = North →  affects latitude
        +Z = Up    →  affects altitude

    NOTE: If drones appear shifted in the wrong compass direction after testing,
    swap the x_m and y_m assignments in delta_lat / delta_lon below.

    Args:
        x_m: spawn X position in world-frame meters
        y_m: spawn Y position in world-frame meters
        z_m: spawn Z position in world-frame meters (height above world origin)
        world_origin: (lat_deg, lon_deg, alt_m) that world (0,0,0) maps to

    Returns:
        (lat_deg, lon_deg, alt_m) — the GPS home position for this robot
    """
    base_lat, base_lon, base_alt = world_origin
    delta_lat = y_m / 111320.0
    delta_lon = x_m / (111320.0 * math.cos(math.radians(base_lat)))
    return (base_lat + delta_lat, base_lon + delta_lon, base_alt + z_m)


def set_gps_origins(drone_configs, world_origin=DEFAULT_WORLD_ORIGIN):
    """
    Compute GPS origins for all drones and set them as process environment variables.

    Called once at the start of a scene script before spawning vehicles.
    Sets PX4_HOME_LAT_<domain_id>, PX4_HOME_LON_<domain_id>, PX4_HOME_ALT_<domain_id>
    in the current process so the OmniGraph PX4 node reads them when building
    PX4MavlinkBackendConfig, which passes them to PX4LaunchTool as PX4_HOME_LAT/LON/ALT
    on the PX4 subprocess environment.

    Args:
        drone_configs: list of dicts, one per drone:
            [
                {
                    "domain_id": int,      # matches vehicle_id passed to spawn_px4_multirotor_node
                    "x_m":       float,    # world-frame spawn East  (meters)
                    "y_m":       float,    # world-frame spawn North (meters)
                    "z_m":       float,    # world-frame spawn Up    (meters)
                    "orient":    list,     # quaternion [x, y, z, w]
                },
                ...
            ]
        world_origin: (lat_deg, lon_deg, alt_m) for world (0, 0, 0).
    """
    print(f"[gps_utils] World anchor: lat={world_origin[0]}, lon={world_origin[1]}, alt={world_origin[2]}")

    for cfg in drone_configs:
        domain_id = cfg["domain_id"]
        lat, lon, alt = compute_gps_origin(cfg["x_m"], cfg["y_m"], cfg["z_m"], world_origin)

        os.environ[f"PX4_HOME_LAT_{domain_id}"] = str(lat)
        os.environ[f"PX4_HOME_LON_{domain_id}"] = str(lon)
        os.environ[f"PX4_HOME_ALT_{domain_id}"] = str(alt)

        print(f"[gps_utils]   domain_id={domain_id}: "
              f"lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}")
