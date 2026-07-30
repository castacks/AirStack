"""GPS degradation pipeline configuration."""
from __future__ import annotations
import datetime
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class GpsDegradationConfig:
    # GPS update rate — gates pipeline at N simulation steps (100 Hz physics → 5 Hz GPS)
    gps_update_every_n_steps: int = 20

    # How often to recompute the full visible-satellite list
    constellation_refresh_steps: int = 6000  # every 60 s at 100 Hz

    # Elevation mask — satellites below this angle are ignored
    gps_elevation_mask_deg: float = 5.0

    # PhysX raycasting limits
    raycast_max_distance_m: float = 50_000.0
    raycast_min_hit_distance_m: float = 0.5  # ignore self-hits closer than this

    # Accuracy reported to PX4 before DOP amplification.
    # Civilian L1 multi-constellation effective UERE (RSS of satellite clock,
    # ephemeris, ionosphere w/ broadcast correction, troposphere, multipath,
    # receiver noise) = 2–4 m. Mid-range 2.5 m used here.
    # Source: GPS SPS Performance Standard (NAVCEN), NovAtel GNSS Error Sources.
    uere_accuracy_m: float = 2.5

    # Shadow fading sigma (dB) per scenario
    shadow_fading_sigma_db: Dict[str, float] = field(default_factory=lambda: {
        "open_sky": 1.5,
        "suburban": 3.5,
        "urban": 6.0,
        "dense_urban": 9.0,
    })
    cn0_floor_dbhz: float = 30.0    # tracking threshold
    cn0_zenith_dbhz: float = 45.0   # C/N0 at zenith

    # Ornstein-Uhlenbeck correlated drift (atmospheric delay + slowly-changing NLOS bias).
    #
    # Physical model: GPS total error = OU drift (correlated, slow) + multipath scatter (fast).
    # The OU process captures only the correlated component (~40–60% of total error).
    # EPH = uere_accuracy_m * HDOP captures the full uncertainty reported to PX4 EKF.
    #
    # Correlation time (tau):
    #   Urban multipath (moving drone): 25–60 s  [RTCA DO-229 MOPS: 25 s standard;
    #                                              DLR airborne study 2024: median 14 s]
    #   Atmospheric (ionosphere/tropo): 5–30 min  [use for open-sky / suburban scenarios]
    #   Default 60 s covers both: dominant multipath regime for urban, upper-end for atmos.
    #
    # Steady-state RMS = ou_base_xy * max(HDOP,1) * sqrt(tau/2):
    #   HDOP=1.0 (OPEN_SKY) : 0.27 * sqrt(30) = 1.48 m   [lit: 1–4 m]       ✓
    #   HDOP=3.0 (MARGINAL)  : 0.27*3 * sqrt(30) = 4.44 m  [lit: 5–15 m]      ✓ (correlated portion)
    #   HDOP=5.0 (DENIED)    : 0.27*5 * sqrt(30) = 7.39 m  [lit: 10–30 m]     ✓ (correlated portion)
    ou_correlation_time_s: float = 60.0
    ou_base_xy_noise_m_sqrts: float = 0.27   # σ in m/√s at HDOP=1; gives 1.48 m SS-RMS
    ou_base_z_noise_m_sqrts: float = 0.40    # σ in m/√s at VDOP=1; vertical ~1.5x horizontal

    # State-machine hysteresis thresholds: (enter_threshold, exit_threshold)
    state_thresholds: Dict = field(default_factory=lambda: {
        "denied_nsats":   (3, 4),
        "marginal_nsats": (5, 6),
        "denied_hdop":    (5.0, 4.5),
        "marginal_hdop":  (3.0, 2.5),
    })

    # Recovery and position/accuracy ramp limiting
    recovery_steps: int = 25
    max_position_step_m: float = 0.3
    max_eph_step_m: float = 0.5

    # Jamming / forced DENIED override
    jamming_fix_type: int = 0
    jamming_satellites: int = 0
    jamming_eph_cm: int = 9999
    jamming_epv_cm: int = 9999

    # Scenario name — selects shadow_fading_sigma_db entry
    scenario: str = "urban"

    # Path to GPS TLE file; empty string → auto-detect data/gps_ops.tle relative to package
    tle_file_path: str = ""

    # UTC epoch for TLE propagation; None → use system clock at model creation
    sim_epoch_utc: Optional[datetime.datetime] = None

    @classmethod
    def from_dict(cls, values: dict) -> "GpsDegradationConfig":
        """Build from Pegasus sensor config while ignoring parent GPS keys."""
        names = cls.__dataclass_fields__
        return cls(**{key: value for key, value in values.items() if key in names})
