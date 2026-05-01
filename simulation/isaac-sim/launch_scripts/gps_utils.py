"""GPS origin utilities for multi-drone simulation.

Computes per-drone GPS origins from world-frame spawn positions and sets them
as PX4_HOME_LAT_<id>/LON_<id>/ALT_<id> env vars before the Pegasus PX4 node
spawns each vehicle.
"""

import math
import os

# Lisbon — matches the Pegasus configs.yaml default.
DEFAULT_WORLD_ORIGIN = (38.736832, -9.137977, 90.0)


def compute_gps_origin(x_m, y_m, z_m, world_origin=DEFAULT_WORLD_ORIGIN):
    """Convert a world-frame spawn (+X=East, +Y=North, +Z=Up, metres) to (lat, lon, alt).

    Flat-Earth approximation; accurate at scene scales (a few hundred metres).
    """
    base_lat, base_lon, base_alt = world_origin
    delta_lat = y_m / 111320.0
    delta_lon = x_m / (111320.0 * math.cos(math.radians(base_lat)))
    return (base_lat + delta_lat, base_lon + delta_lon, base_alt + z_m)


def set_gps_origins(drone_configs, world_origin=DEFAULT_WORLD_ORIGIN):
    """Set PX4_HOME_*_<domain_id> for each drone in drone_configs.

    Each entry needs ``domain_id``, ``x_m``, ``y_m``, ``z_m``. Call before
    spawning vehicles so the Pegasus OmniGraph PX4 node picks the values up.
    """
    print(f"[gps_utils] World anchor: lat={world_origin[0]}, lon={world_origin[1]}, alt={world_origin[2]}")

    for cfg in drone_configs:
        domain_id = cfg["domain_id"]
        lat, lon, alt = compute_gps_origin(cfg["x_m"], cfg["y_m"], cfg["z_m"], world_origin)
        os.environ[f"PX4_HOME_LAT_{domain_id}"] = str(lat)
        os.environ[f"PX4_HOME_LON_{domain_id}"] = str(lon)
        os.environ[f"PX4_HOME_ALT_{domain_id}"] = str(alt)
        print(f"[gps_utils]   domain_id={domain_id}: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}")
