"""Focused offline checks for the Tier 2 GPS degradation pipeline."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from gps_degradation_staging.config import GpsDegradationConfig
from gps_degradation_staging.constellation import ConstellationModel, SatelliteView
from gps_degradation_staging.dop import DopCalculator
from gps_degradation_staging.signal_model import SatelliteSignal
from gps_degradation_staging.state_machine import DegradationStateMachine, GpsState
import gps_degradation_staging.visibility as visibility


def _sat(azimuth_deg: float, elevation_deg: float) -> SatelliteView:
    az = np.radians(azimuth_deg)
    el = np.radians(elevation_deg)
    c = np.cos(el)
    return SatelliteView(
        sat_id=0,
        name="test",
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
        enu_unit=np.array([c * np.sin(az), c * np.cos(az), np.sin(el)]),
    )


def test_tle_constellation() -> None:
    tle = Path(__file__).parent / "data" / "gps_ops.tle"
    model = ConstellationModel(str(tle))
    sats = model.get_visible_satellites(40.4422, -79.9430, 100.0, sim_time_s=0.0)
    assert model.using_tle
    assert sats
    assert all(not sat.name.startswith("Walker-") for sat in sats)


def test_dop_and_state() -> None:
    signals = [
        SatelliteSignal(s.elevation_deg, s.azimuth_deg, True, 0.0, 40.0)
        for s in [_sat(0, 60), _sat(90, 45), _sat(180, 55), _sat(270, 40), _sat(35, 25)]
    ]
    hdop, vdop, _ = DopCalculator().compute(signals)
    assert 0.0 < hdop < 99.0
    assert 0.0 < vdop < 99.0

    machine = DegradationStateMachine(GpsDegradationConfig())
    denied = machine.update(3, 99.0, 99.0, 0.0, 0.0, 0.0, 40.0, -79.0)
    assert denied.state == GpsState.DENIED
    assert denied.fix_type == 0


def test_reflection_classification() -> None:
    class Physx:
        def __init__(self):
            self.calls = 0

        def raycast_closest(self, origin, direction, max_distance):
            self.calls += 1
            if self.calls == 1:
                return {
                    "hit": True,
                    "distance": 10.0,
                    "normal": (1.0, 0.0, 0.0),
                    "position": (10.0, 0.0, 5.0),
                }
            return {"hit": False}

    cfg = GpsDegradationConfig()
    original = visibility._get_physx
    visibility._get_physx = lambda: Physx()
    try:
        result = visibility.VisibilityChecker(cfg).check([_sat(45, 45)], (0.0, 0.0, 5.0))
    finally:
        visibility._get_physx = original

    assert len(result) == 1
    assert not result[0].is_los
    assert result[0].multipath_extra_m == 20.0


if __name__ == "__main__":
    test_tle_constellation()
    test_dop_and_state()
    test_reflection_classification()
    print("GPS degradation checks passed")
