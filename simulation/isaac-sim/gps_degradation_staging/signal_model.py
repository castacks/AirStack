"""
signal_model.py

Elevation-dependent C/N0 and correlated shadow fading model.

C/N0 formula (correct sign):
    cn0 = cn0_zenith + 10·log10(sin(elevation))
    → lower elevation → lower C/N0 (physically correct)

Shadow fading is correlated within 30° azimuth sectors to model
building clusters rather than per-satellite independent noise.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from .visibility import SatVisibility
    from .config import GpsDegradationConfig


@dataclass
class SatelliteSignal:
    """Satellite after signal quality filtering — input to DOP."""
    elevation_deg: float
    azimuth_deg: float
    is_los: bool
    cn0_dbhz: float


class SignalQualityModel:
    def __init__(self, cfg: "GpsDegradationConfig", rng=None):
        self._cfg = cfg
        self._rng = rng or random.Random()
        # 30° azimuth sector → correlated fading value (dB)
        self._sector_fading: dict = {}

    def compute(self, sv_visibilities: List["SatVisibility"]) -> List[SatelliteSignal]:
        cfg = self._cfg
        sigma_db = cfg.shadow_fading_sigma_db.get(cfg.scenario, 3.5)
        result = []

        for sv in sv_visibilities:
            if not sv.is_los:
                continue

            # C/N0: zenith reference, correct sign → lower at low elevation
            cn0 = cfg.cn0_zenith_dbhz + 10.0 * math.log10(
                max(math.sin(math.radians(sv.sat.elevation_deg)), 0.05)
            )

            # Correlated fading per 30° azimuth sector
            sector = int(sv.sat.azimuth_deg / 30.0) % 12
            if sector not in self._sector_fading:
                self._sector_fading[sector] = self._rng.gauss(0.0, sigma_db)
            cn0 += self._sector_fading[sector]

            if cn0 < cfg.cn0_floor_dbhz:
                continue  # below receiver tracking threshold

            result.append(SatelliteSignal(
                elevation_deg=sv.sat.elevation_deg,
                azimuth_deg=sv.sat.azimuth_deg,
                is_los=sv.is_los,
                cn0_dbhz=cn0,
            ))

        # Evict oldest sector entry to allow slow fading evolution over time
        if len(self._sector_fading) > 12:
            del self._sector_fading[next(iter(self._sector_fading))]

        return result
