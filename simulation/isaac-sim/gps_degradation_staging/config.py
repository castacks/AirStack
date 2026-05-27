from dataclasses import dataclass, field
from typing import Dict


@dataclass
class GpsDegradationConfig:
    # --- Timing ---
    # Physics runs at PX4_PHYSICS_HZ (default 100 Hz).
    # GPS at 5 Hz = every 20 physics steps.
    gps_update_every_n_steps: int = 20
    # Satellites move ~2 deg/min; recomputing every 60 s is sufficient.
    constellation_refresh_steps: int = 6000

    # --- Constellation ---
    # GPS Walker-delta(24:6:1): IS-GPS-200M Section 2.5 nominal parameters.
    gps_orbital_altitude_m: float = 20_200_000.0
    gps_inclination_deg: float = 55.0
    gps_n_planes: int = 6
    gps_sats_per_plane: int = 4
    gps_elevation_mask_deg: float = 5.0

    # --- PhysX raycasting ---
    # 50 km covers any urban scene; satellites are much farther but rays
    # only need to clear scene geometry.
    raycast_max_distance_m: float = 50_000.0
    # Avoid self-hit against the drone's own collision mesh.
    raycast_min_hit_distance_m: float = 0.5
    # Cap on reflected extra path length. Real multipath rarely exceeds ~300 m.
    multipath_max_extra_m: float = 300.0

    # --- Pseudorange thermal noise ---
    # Represents C/A-code receiver thermal noise only (~0.3 m for a narrow
    # correlator DLL at typical tracking loop bandwidth). Atmospheric delay
    # variability is handled by the OU process below — do NOT include it here.
    uere_base_m: float = 0.3
    # Elevation-dependent noise floor: sigma(theta) = uere_base / sin(theta).
    low_elevation_threshold_deg: float = 15.0
    low_elevation_noise_factor: float = 2.5

    # --- Signal quality / shadow fading ---
    # Lognormal shadow fading std dev (dB) by scenario type.
    shadow_fading_sigma_db: Dict[str, float] = field(default_factory=lambda: {
        "open_sky":    1.5,
        "suburban":    3.5,
        "urban":       6.0,
        "dense_urban": 9.0,
    })
    # C/N0 acquisition floor (dB·Hz). Satellites below this are dropped.
    cn0_floor_dbhz: float = 30.0
    # Nominal C/N0 for a satellite at zenith (dB·Hz).
    # Elevation dependence: cn0(el) = cn0_zenith + 10*log10(sin(el))
    cn0_zenith_dbhz: float = 45.0

    # --- OU (Gauss-Markov) position drift process ---
    # Models temporal GPS position drift from ionospheric and tropospheric delay
    # variability and receiver clock drift. The OU noise is HDOP/VDOP-scaled so
    # that poor constellation geometry → larger drift amplitude.
    #
    # Steady-state RMS = ou_base_noise × sqrt(ou_correlation_time_s / 2)
    # Examples at HDOP=1:
    #   horiz RMS = 0.08 × sqrt(150) ≈ 0.98 m   (open sky, L1 single-frequency)
    #   vert  RMS = 0.12 × sqrt(150) ≈ 1.47 m
    # At HDOP=3 (urban):
    #   horiz RMS ≈ 2.9 m (before adding WLS multipath bias)
    #
    # Save these at __init__ time in GpsDegradationModel and scale by HDOP/VDOP
    # per GPS step. Never scale from an already-scaled value.
    ou_correlation_time_s: float = 300.0        # ~5-minute ionospheric drift timescale
    ou_base_xy_noise_m_sqrts: float = 0.08      # horizontal diffusion  [m/sqrt(s)]
    ou_base_z_noise_m_sqrts: float = 0.12       # vertical diffusion    [m/sqrt(s)]

    # --- State machine thresholds (hysteresis) ---
    state_thresholds: Dict = field(default_factory=lambda: {
        # n_los: (enter_state_below, exit_state_above)
        "denied_nsats":   (4, 5),
        "marginal_nsats": (6, 7),
        # hdop: (enter_state_above, exit_state_below)
        "denied_hdop":    (5.0, 4.5),
        "marginal_hdop":  (3.0, 2.5),
    })
    # GPS updates to ramp from DENIED back to MARGINAL (~5 s at 5 Hz).
    recovery_steps: int = 25

    # Ramp-rate limiting on eph/epv and position to prevent EKF2 innovation
    # gate rejection (fires when position jumps more than ~sqrt(eph²+noise²)).
    max_position_step_m: float = 0.3
    max_eph_step_m: float = 0.5

    # --- Jamming override ---
    jamming_fix_type: int = 0
    jamming_satellites: int = 0
    jamming_eph_cm: int = 9999
    jamming_epv_cm: int = 9999

    # --- Scenario ---
    # Set per scene. Keys must match shadow_fading_sigma_db above.
    scenario: str = "urban"
