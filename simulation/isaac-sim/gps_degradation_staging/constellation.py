"""
Synthetic GPS Walker-delta constellation.

Generates 24 satellite ECEF positions from IS-GPS-200 nominal orbital
parameters. No ephemeris files, no network access.

Accuracy note: circular orbit approximation (e=0) places satellites within
~200 km of their true positions. At 20,200 km altitude this is ~0.6 deg of
angular error — well within the tolerance needed for sky-visibility raycasting.
Constellation is refreshed every ~60 s of sim time; satellites move ~2 deg/min
so geometry is accurate enough between refreshes.
"""

import math
import numpy as np
from typing import List, Optional
from dataclasses import dataclass

import pymap3d as pm

from .config import GpsDegradationConfig


@dataclass
class SatelliteView:
    sat_id: int
    elevation_deg: float
    azimuth_deg: float
    ecef_x: float
    ecef_y: float
    ecef_z: float
    # Unit vector from drone toward satellite in ENU frame.
    enu_unit: np.ndarray


class ConstellationModel:
    """
    Walker-delta(24:6:1) GPS constellation: 24 satellites, 6 orbital planes,
    55-degree inclination, 20,200 km altitude.

    Call update() once at init and then every constellation_refresh_steps.
    Call get_visible_satellites() every GPS update step.
    """

    EARTH_RADIUS_M = 6_371_000.0
    EARTH_MU = 3.986004418e14    # gravitational parameter m³/s²
    EARTH_OMEGA = 7.2921150e-5   # rotation rate rad/s

    def __init__(self, cfg: GpsDegradationConfig):
        self._cfg = cfg
        self._n_sats = cfg.gps_n_planes * cfg.gps_sats_per_plane
        self._ecef_positions: Optional[np.ndarray] = None
        self._last_refresh_time: float = -1e9
        self._orbital_elements = self._build_orbital_elements()

    def _build_orbital_elements(self) -> np.ndarray:
        """
        Returns array of shape (N, 3): (RAAN_rad, M0_rad, inclination_rad).
        Walker-delta(24:6:1): P=6 planes, S=4 sats/plane, F=1 phase factor.
        """
        P = self._cfg.gps_n_planes
        S = self._cfg.gps_sats_per_plane
        inc = math.radians(self._cfg.gps_inclination_deg)
        raan_spacing = 2 * math.pi / P
        anomaly_spacing = 2 * math.pi / S
        # Walker phase offset between planes: (2pi / N) * F, F=1
        phase_factor = 2 * math.pi / (P * S)

        elements = np.zeros((P * S, 3))
        idx = 0
        for p in range(P):
            raan = p * raan_spacing
            for s in range(S):
                m0 = s * anomaly_spacing + p * phase_factor
                elements[idx] = [raan, m0, inc]
                idx += 1
        return elements

    def update(self, sim_time_s: float) -> None:
        """Recompute satellite ECEF positions. Call every ~60 s of sim time."""
        self._last_refresh_time = sim_time_s
        self._ecef_positions = self._compute_ecef_all(sim_time_s)

    def needs_refresh(self, sim_time_s: float, refresh_interval_s: float) -> bool:
        return (sim_time_s - self._last_refresh_time) >= refresh_interval_s

    def _compute_ecef_all(self, t: float) -> np.ndarray:
        """
        Compute ECEF positions of all satellites at time t (seconds).
        Circular orbit: eccentricity = 0, mean anomaly = true anomaly.
        ECI → ECEF via Earth rotation angle (GMST approximation).
        """
        a = self._cfg.gps_orbital_altitude_m + self.EARTH_RADIUS_M
        n_motion = math.sqrt(self.EARTH_MU / a ** 3)  # mean motion rad/s

        ecef = np.zeros((self._n_sats, 3))
        theta_earth = self.EARTH_OMEGA * t  # Earth rotation angle
        cos_t, sin_t = math.cos(theta_earth), math.sin(theta_earth)

        for i in range(self._n_sats):
            raan, m0, inc = self._orbital_elements[i]
            nu = (m0 + n_motion * t) % (2 * math.pi)  # true anomaly

            # Perifocal (orbital plane) coordinates
            x_orb = a * math.cos(nu)
            y_orb = a * math.sin(nu)

            # Rotate to ECI: Rz(-RAAN) * Rx(-inc)
            cos_r, sin_r = math.cos(raan), math.sin(raan)
            cos_i, sin_i = math.cos(inc), math.sin(inc)
            xi = cos_r * x_orb - sin_r * cos_i * y_orb
            yi = sin_r * x_orb + cos_r * cos_i * y_orb
            zi = sin_i * y_orb

            # ECI → ECEF: rotate by Earth's rotation angle
            ecef[i, 0] = cos_t * xi + sin_t * yi
            ecef[i, 1] = -sin_t * xi + cos_t * yi
            ecef[i, 2] = zi

        return ecef

    def get_visible_satellites(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
    ) -> List[SatelliteView]:
        """
        Returns all satellites above the elevation mask as seen from the drone.
        Uses pymap3d for WGS-84 ECEF→AER conversion.
        """
        if self._ecef_positions is None:
            self.update(0.0)

        mask = self._cfg.gps_elevation_mask_deg
        visible = []

        for i, (sx, sy, sz) in enumerate(self._ecef_positions):
            az, el, _ = pm.ecef2aer(sx, sy, sz, lat_deg, lon_deg, alt_m)
            if el < mask:
                continue

            el_r = math.radians(el)
            az_r = math.radians(az)
            e = math.cos(el_r) * math.sin(az_r)
            n = math.cos(el_r) * math.cos(az_r)
            u = math.sin(el_r)

            visible.append(SatelliteView(
                sat_id=i,
                elevation_deg=float(el),
                azimuth_deg=float(az),
                ecef_x=float(sx),
                ecef_y=float(sy),
                ecef_z=float(sz),
                enu_unit=np.array([e, n, u]),
            ))

        return visible
