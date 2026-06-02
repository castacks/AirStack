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

    # Multipath geometry cap
    multipath_max_extra_m: float = 300.0

    # Pseudorange thermal noise only (atmospheric drift handled by OU process)
    uere_base_m: float = 0.3
    # Accuracy reported to PX4 before geometry amplification
    uere_accuracy_m: float = 0.8

    # Low-elevation noise amplification
    low_elevation_threshold_deg: float = 15.0
    low_elevation_noise_factor: float = 2.5

    # Shadow fading sigma (dB) per scenario
    shadow_fading_sigma_db: Dict[str, float] = field(default_factory=lambda: {
        "open_sky": 1.5,
        "suburban": 3.5,
        "urban": 6.0,
        "dense_urban": 9.0,
    })
    cn0_floor_dbhz: float = 30.0    # tracking threshold
    cn0_zenith_dbhz: float = 45.0   # C/N0 at zenith

    # Ornstein-Uhlenbeck atmospheric + clock drift
    # Steady-state RMS at HDOP=1 = ou_base_xy_noise * sqrt(ou_correlation_time / 2)
    # = 0.08 * sqrt(150) ≈ 0.98 m (realistic L1 open-sky)
    ou_correlation_time_s: float = 60.0
    ou_base_xy_noise_m_sqrts: float = 0.08   # σ in m/√s at HDOP=1
    ou_base_z_noise_m_sqrts: float = 0.12    # σ in m/√s at VDOP=1

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
