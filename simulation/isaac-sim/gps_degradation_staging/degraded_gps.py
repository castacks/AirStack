"""Pegasus GPS sensor adapter for the Tier 2 degradation pipeline."""
from __future__ import annotations

import numpy as np

from pegasus.simulator.logic.sensors.geo_mag_utils import reprojection
from pegasus.simulator.logic.sensors.gps import GPS

from .config import GpsDegradationConfig
from .model import GpsDegradationModel


class DegradedGPS(GPS):
    """GPS sensor whose PX4 HIL_GPS output reflects satellite visibility."""

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("update_rate", 5.0)
        super().__init__(config)

        self._degradation = GpsDegradationModel(
            cfg=GpsDegradationConfig.from_dict(config),
            include_ou_drift=False,
        )
        self._base_xy_noise_density = self._gps_xy_noise_density
        self._base_z_noise_density = self._gps_z_noise_density
        self._base_xy_random_walk = self._gps_xy_random_walk
        self._base_z_random_walk = self._gps_z_random_walk
        self._step_count = 0
        self._sim_time_s = 0.0

    def update(self, state, dt: float):
        self._step_count += 1
        self._sim_time_s += dt

        latitude, longitude = reprojection(
            state.position,
            np.radians(self._origin_lat),
            np.radians(self._origin_lon),
        )
        output = self._degradation.step(
            sim_time_s=self._sim_time_s,
            step_count=self._step_count,
            lat_deg=float(np.degrees(latitude)),
            lon_deg=float(np.degrees(longitude)),
            alt_m=float(state.position[2] + self._origin_alt),
            world_pos=state.position,
        )

        # Parent GPS applies its OU drift during this call. Scale only from the
        # saved baseline values so the factors do not compound over time.
        self._gps_xy_noise_density = self._base_xy_noise_density * max(output.hdop, 1.0)
        self._gps_z_noise_density = self._base_z_noise_density * max(output.vdop, 1.0)
        self._gps_xy_random_walk = self._base_xy_random_walk * max(output.hdop, 1.0)
        self._gps_z_random_walk = self._base_z_random_walk * max(output.vdop, 1.0)

        data = super().update(state, dt)
        if data is None:
            return None

        data["latitude"] += output.delta_lat
        data["longitude"] += output.delta_lon
        data["altitude"] += output.delta_alt
        data["eph"] = min(int(output.eph_m * 100.0), 65535)
        data["epv"] = min(int(output.epv_m * 100.0), 65535)
        data["fix_type"] = output.fix_type
        data["sattelites_visible"] = output.n_los
        self._state = data
        return data
