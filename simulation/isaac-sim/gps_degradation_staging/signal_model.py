"""
Signal quality model: elevation-dependent C/N0 + lognormal shadow fading.

Filters the visibility list by dropping satellites whose effective C/N0 falls
below the acquisition floor. Satellites in the same azimuth sector share a
correlated fading component (70% common, 30% independent per satellite).

Fix applied vs original spec: fully NLOS-blocked satellites (is_los=False and
multipath_extra_m=0.0) are immediately dropped before any C/N0 computation.
A physically blocked signal has no carrier to track regardless of elevation.

References:
  Ni et al. 2024 — Path loss and shadowing for UAV-to-Ground channels
  Min et al. 2022 — DNN multipath: elevation and SNR as key features
"""

import math
import numpy as np
from typing import List

from .visibility import SatelliteVisibility
from .config import GpsDegradationConfig


def apply_signal_quality(
    visibility: List[SatelliteVisibility],
    cfg: GpsDegradationConfig,
    rng: np.random.Generator,
) -> List[SatelliteVisibility]:
    """
    Returns a filtered list with weak/blocked satellites removed and
    effective_cn0 populated on survivors.

    Removal criteria:
      1. Fully NLOS-blocked (is_los=False, multipath_extra_m=0) — always dropped.
      2. Effective C/N0 below cn0_floor_dbhz after fading — dropped.
    """
    sigma_sf_db = cfg.shadow_fading_sigma_db.get(cfg.scenario, 6.0)

    # Correlated fading: one common draw per 30-degree azimuth sector.
    n_sectors = 12
    sector_fading = rng.normal(0.0, sigma_sf_db * 0.7, n_sectors)

    surviving = []
    for sv in visibility:
        # Drop fully blocked satellites immediately — no signal to model.
        if not sv.is_los and sv.multipath_extra_m == 0.0:
            continue

        el_deg = sv.sat.elevation_deg
        az_deg = sv.sat.azimuth_deg

        # Elevation-dependent nominal C/N0
        cn0_nominal = cfg.cn0_zenith_dbhz - 10.0 * math.log10(
            max(math.sin(math.radians(el_deg)), 0.05))

        # Individual (independent) fading component per satellite
        sf_individual = rng.normal(0.0, sigma_sf_db * 0.3)
        # Correlated sector fading
        sector = int(az_deg / 30.0) % n_sectors
        sf_corr = sector_fading[sector]

        effective_cn0 = cn0_nominal - (sf_individual + sf_corr)
        sv.effective_cn0 = effective_cn0

        if effective_cn0 < cfg.cn0_floor_dbhz:
            continue

        surviving.append(sv)

    return surviving
