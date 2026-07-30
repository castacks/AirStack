"""Pegasus GPS sensor adapter for the Tier 2 degradation pipeline."""
from __future__ import annotations

from pegasus.simulator.logic.sensors.gps import GPS

from .runtime_output import get_latest_output


class DegradedGPS(GPS):
    """GPS sensor whose PX4 HIL_GPS output reflects satellite visibility."""

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("update_rate", 5.0)
        config.update({
            "gps_xy_random_walk": 0.0,
            "gps_z_random_walk": 0.0,
            "gps_xy_noise_density": 0.0,
            "gps_z_noise_density": 0.0,
        })
        super().__init__(config)

    def update(self, state, dt: float):
        data = super().update(state, dt)
        if data is None:
            return None

        output = get_latest_output()
        if output is None:
            return data

        data["latitude"] += output.delta_lat
        data["longitude"] += output.delta_lon
        data["altitude"] += output.delta_alt
        data["eph"] = min(int(output.eph_m * 100.0), 65535)
        data["epv"] = min(int(output.epv_m * 100.0), 65535)
        data["fix_type"] = output.fix_type
        data["sattelites_visible"] = output.n_los
        self._state = data
        return data
