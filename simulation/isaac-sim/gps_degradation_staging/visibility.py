"""
visibility.py

Per-satellite LOS/NLOS check using PhysX raycasting.

Each satellite's ENU unit vector (from constellation.py) is fired as a ray
from the drone position. A hit means the signal is NLOS; no hit means LOS.

Must be called from the Isaac Sim physics thread (after world.step()).
Falls back to all-LOS when PhysX is unavailable (testing/standalone use).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from .constellation import SatelliteView
    from .config import GpsDegradationConfig


@dataclass
class SatVisibility:
    """Constellation satellite + visibility classification."""
    sat: "SatelliteView"      # position data including enu_unit
    is_los: bool              # True = direct line-of-sight
    multipath_extra_m: float  # extra path length of reflected signal (m); 0 if LOS


def _get_physx():
    try:
        from omni.physx import get_physx_scene_query_interface
        return get_physx_scene_query_interface()
    except Exception:
        return None


class VisibilityChecker:
    def __init__(self, cfg: "GpsDegradationConfig"):
        self._cfg = cfg

    def check(
        self,
        sats: List["SatelliteView"],
        world_pos,  # (x, y, z) in Isaac Sim world coords — Z-up, metres
    ) -> List[SatVisibility]:
        physx = _get_physx()
        return [self._classify(physx, sat, world_pos) for sat in sats]

    def _classify(self, physx, sat, world_pos) -> SatVisibility:
        if physx is None:
            # No Isaac Sim physics — assume all LOS (testing/standalone)
            return SatVisibility(sat=sat, is_los=True, multipath_extra_m=0.0)

        cfg = self._cfg
        origin = (float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))

        # sat.enu_unit is [East, North, Up]; Isaac Sim Z-up maps directly
        direction = (
            float(sat.enu_unit[0]),  # East  → X
            float(sat.enu_unit[1]),  # North → Y
            float(sat.enu_unit[2]),  # Up    → Z
        )

        hit = physx.raycast_closest(origin, direction, cfg.raycast_max_distance_m)
        if not hit or not hit.get("hit", False):
            return SatVisibility(sat=sat, is_los=True, multipath_extra_m=0.0)

        hit_dist = float(hit.get("distance", cfg.raycast_max_distance_m))
        if hit_dist < cfg.raycast_min_hit_distance_m:
            # Self-hit on the drone body — treat as clear sky
            return SatVisibility(sat=sat, is_los=True, multipath_extra_m=0.0)

        normal = hit.get("normal")
        position = hit.get("position")
        if normal is None or position is None:
            return SatVisibility(sat=sat, is_los=False, multipath_extra_m=0.0)

        dot = sum(direction[i] * float(normal[i]) for i in range(3))
        reflected = tuple(direction[i] - 2.0 * dot * float(normal[i]) for i in range(3))
        reflected_origin = tuple(
            float(position[i]) + cfg.raycast_min_hit_distance_m * reflected[i]
            for i in range(3)
        )
        reflected_hit = physx.raycast_closest(
            reflected_origin,
            reflected,
            cfg.raycast_max_distance_m,
        )
        if reflected_hit and reflected_hit.get("hit", False):
            return SatVisibility(sat=sat, is_los=False, multipath_extra_m=0.0)

        mp_extra = float(min(2.0 * hit_dist, cfg.multipath_max_extra_m))
        return SatVisibility(sat=sat, is_los=False, multipath_extra_m=mp_extra)
