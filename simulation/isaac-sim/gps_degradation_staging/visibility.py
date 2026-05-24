"""
LOS/NLOS classification via PhysX raycasting.

Each satellite direction is tested with a single raycast_closest() call against
the scene's physics collision geometry. A hit means the signal path is blocked
or reflected by a building/terrain prim.

Requirements:
  - Simulation must be playing (physics scene active).
  - Building/terrain prims must have PhysicsCollisionAPI applied.
    See GPS_DEGRADATION_VM_BRIDGE.md Part 2 for how to verify and apply.
  - AirStack world frame convention: +X=East, +Y=North, +Z=Up.
    If a scene uses a different convention, adjust the ray_dir mapping below.
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import carb
from omni.physx import get_physx_scene_query_interface

from .config import GpsDegradationConfig
from .constellation import SatelliteView


@dataclass
class SatelliteVisibility:
    sat: SatelliteView
    is_los: bool
    # Extra path length from specular reflection (metres). 0.0 if LOS or
    # fully blocked (no reflected path reaches sky).
    multipath_extra_m: float = 0.0
    # Effective C/N0 after fading — populated by signal_model.py.
    effective_cn0: float = 45.0


class VisibilityEngine:
    """
    Classifies each satellite as LOS, NLOS-reflected, or NLOS-blocked
    using PhysX raycasting from the drone's world-frame position.
    """

    def __init__(self, cfg: GpsDegradationConfig):
        self._cfg = cfg
        self._physx = None  # lazy init: only available once simulation plays

    def _get_physx(self):
        if self._physx is None:
            self._physx = get_physx_scene_query_interface()
        return self._physx

    def classify(
        self,
        drone_pos_world: Tuple[float, float, float],
        satellites: List[SatelliteView],
    ) -> List[SatelliteVisibility]:
        """
        Args:
            drone_pos_world: (x, y, z) in Isaac Sim world frame (metres).
                             AirStack convention: X=East, Y=North, Z=Up.
            satellites:      Visible satellites from ConstellationModel.
        Returns:
            One SatelliteVisibility per input satellite.
        """
        physx = self._get_physx()
        ox, oy, oz = drone_pos_world
        origin = carb.Float3(ox, oy, oz)
        max_dist = self._cfg.raycast_max_distance_m
        min_hit = self._cfg.raycast_min_hit_distance_m
        multipath_cap = self._cfg.multipath_max_extra_m

        results = []
        for sat in satellites:
            # ENU unit vector maps directly to Isaac Sim world axes:
            # east→X, north→Y, up→Z (AirStack convention).
            e, n, u = sat.enu_unit
            ray_dir = carb.Float3(float(e), float(n), float(u))

            hit = physx.raycast_closest(origin, ray_dir, max_dist)

            if not hit.get("hit", False):
                results.append(SatelliteVisibility(sat=sat, is_los=True))
                continue

            hit_dist = hit.get("distance", 0.0)
            if hit_dist < min_hit:
                # Self-hit against drone mesh — treat as LOS.
                results.append(SatelliteVisibility(sat=sat, is_los=True))
                continue

            # --- Attempt to compute reflected path for multipath offset ---
            normal = hit.get("normal", None)
            multipath_extra = 0.0

            if normal is not None:
                nx, ny, nz = normal.x, normal.y, normal.z
                n_vec = np.array([nx, ny, nz])
                n_norm = np.linalg.norm(n_vec)
                if n_norm > 1e-6:
                    n_hat = n_vec / n_norm
                    d_vec = np.array([e, n, u])
                    refl = d_vec - 2.0 * np.dot(d_vec, n_hat) * n_hat
                    refl_norm = np.linalg.norm(refl)
                    if refl_norm > 1e-6:
                        refl_hat = refl / refl_norm
                        hx = ox + e * hit_dist
                        hy = oy + n * hit_dist
                        hz = oz + u * hit_dist
                        refl_origin = carb.Float3(hx, hy, hz)
                        refl_dir = carb.Float3(*refl_hat.tolist())
                        refl_hit = physx.raycast_closest(
                            refl_origin, refl_dir, max_dist)

                        if not refl_hit.get("hit", False):
                            # Reflected ray reaches sky: multipath signal present.
                            # Extra path ≈ distance to wall (conservative estimate
                            # for near-normal incidence). Capped at multipath_cap
                            # to prevent absurd biases from distant obstructions.
                            multipath_extra = min(hit_dist, multipath_cap)

            if multipath_extra > 0.0:
                results.append(SatelliteVisibility(
                    sat=sat,
                    is_los=False,
                    multipath_extra_m=multipath_extra,
                ))
            else:
                # Fully blocked — no usable signal.
                results.append(SatelliteVisibility(
                    sat=sat,
                    is_los=False,
                    multipath_extra_m=0.0,
                ))

        return results
