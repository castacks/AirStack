"""
GPS degradation state machine with hysteresis and ramp-rate limiting.

States:
  OPEN_SKY  — good geometry, full fix, low eph
  DEGRADED  — 3D fix but elevated eph (still EKF2-acceptable)
  MARGINAL  — 2D fix or borderline geometry (EKF2 may flag hacc/vacc)
  DENIED    — no fix (fix_type=0); EKF2 switches to dead-reckoning
  RECOVERY  — timed ramp back from DENIED to MARGINAL

Hysteresis prevents chattering at transition boundaries.
Ramp-rate limiting on eph and position prevents EKF2 innovation gate rejection,
which fires when position jumps more than ~sqrt(eph²+process_noise²) in one step.
"""

import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from .config import GpsDegradationConfig


class GpsState(IntEnum):
    OPEN_SKY = 0
    DEGRADED = 1
    MARGINAL = 2
    DENIED   = 3
    RECOVERY = 4


@dataclass
class DegradationOutput:
    fix_type: int            # 0=no fix, 2=2D fix, 3=3D fix
    satellites_visible: int
    eph_m: float             # horizontal position uncertainty (metres)
    epv_m: float             # vertical position uncertainty (metres)
    delta_lat_deg: float     # position error: latitude offset to apply
    delta_lon_deg: float     # position error: longitude offset to apply
    delta_alt_m: float       # position error: altitude offset to apply
    state: GpsState
    hdop: float
    vdop: float


class DegradationStateMachine:

    def __init__(self, cfg: GpsDegradationConfig):
        self._cfg = cfg
        self._state = GpsState.OPEN_SKY
        self._recovery_counter = 0
        self._prev_eph_m = 1.0
        self._prev_epv_m = 1.5
        self._prev_delta_enu = np.zeros(3)

    def update(
        self,
        n_los: int,
        hdop: float,
        vdop: float,
        delta_east_m: float,
        delta_north_m: float,
        delta_up_m: float,
        lat_deg: float,
        lon_deg: float,
    ) -> DegradationOutput:
        thr = self._cfg.state_thresholds

        # --- State transitions ---
        if self._state == GpsState.RECOVERY:
            self._recovery_counter += 1
            if self._recovery_counter >= self._cfg.recovery_steps:
                self._state = GpsState.MARGINAL
                self._recovery_counter = 0

        elif n_los < thr["denied_nsats"][0] or hdop > thr["denied_hdop"][0]:
            if self._state != GpsState.DENIED:
                self._state = GpsState.DENIED

        elif self._state == GpsState.DENIED:
            if n_los >= thr["denied_nsats"][1] and hdop <= thr["denied_hdop"][1]:
                self._state = GpsState.RECOVERY
                self._recovery_counter = 0

        elif n_los < thr["marginal_nsats"][0] or hdop > thr["marginal_hdop"][0]:
            self._state = GpsState.MARGINAL

        elif n_los >= thr["marginal_nsats"][1] and hdop <= thr["marginal_hdop"][1]:
            if self._state in (GpsState.MARGINAL, GpsState.DEGRADED):
                if hdop < 1.5 and n_los >= 8:
                    self._state = GpsState.OPEN_SKY
                else:
                    self._state = GpsState.DEGRADED
        else:
            self._state = GpsState.DEGRADED

        # --- Map state to fix_type and eph/epv targets ---
        if self._state == GpsState.DENIED:
            fix_type = 0
            sats_out = max(0, n_los)
            eph_target = 50.0
            epv_target = 80.0
        elif self._state == GpsState.RECOVERY:
            fix_type = 1
            sats_out = max(0, n_los)
            eph_target = 50.0
            epv_target = 80.0
        elif self._state == GpsState.MARGINAL:
            fix_type = 2
            sats_out = n_los
            eph_target = max(3.5, hdop * self._cfg.uere_base_m * 2)
            epv_target = max(6.0, vdop * self._cfg.uere_base_m * 2)
        elif self._state == GpsState.DEGRADED:
            fix_type = 3
            sats_out = n_los
            eph_target = max(1.5, hdop * self._cfg.uere_base_m)
            epv_target = max(2.5, vdop * self._cfg.uere_base_m)
        else:  # OPEN_SKY
            fix_type = 3
            sats_out = n_los
            eph_target = hdop * self._cfg.uere_base_m
            epv_target = vdop * self._cfg.uere_base_m

        # --- Ramp-rate limiting on eph/epv ---
        s = self._cfg.max_eph_step_m
        eph_out = self._prev_eph_m + float(
            np.clip(eph_target - self._prev_eph_m, -s, s))
        epv_out = self._prev_epv_m + float(
            np.clip(epv_target - self._prev_epv_m, -s * 1.5, s * 1.5))
        self._prev_eph_m = eph_out
        self._prev_epv_m = epv_out

        # --- Ramp-rate limiting on position error ---
        p = self._cfg.max_position_step_m
        target_enu = np.array([delta_east_m, delta_north_m, delta_up_m])
        delta_enu = self._prev_delta_enu + np.clip(
            target_enu - self._prev_delta_enu, -p, p)
        self._prev_delta_enu = delta_enu

        # --- ENU delta → lat/lon/alt delta ---
        # Flat-Earth approximation: 1 deg lat ≈ 111,320 m
        lat_r = math.radians(lat_deg)
        d_lat = delta_enu[1] / 111_320.0
        d_lon = delta_enu[0] / (111_320.0 * math.cos(lat_r) + 1e-9)
        d_alt = delta_enu[2]

        return DegradationOutput(
            fix_type=fix_type,
            satellites_visible=sats_out,
            eph_m=float(eph_out),
            epv_m=float(epv_out),
            delta_lat_deg=float(d_lat),
            delta_lon_deg=float(d_lon),
            delta_alt_m=float(d_alt),
            state=self._state,
            hdop=float(hdop),
            vdop=float(vdop),
        )

    def force_jamming(self) -> DegradationOutput:
        """Hard override for active jamming — all fields forced to no-fix values."""
        cfg = self._cfg
        self._state = GpsState.DENIED
        return DegradationOutput(
            fix_type=cfg.jamming_fix_type,
            satellites_visible=cfg.jamming_satellites,
            eph_m=cfg.jamming_eph_cm / 100.0,
            epv_m=cfg.jamming_epv_cm / 100.0,
            delta_lat_deg=0.0,
            delta_lon_deg=0.0,
            delta_alt_m=0.0,
            state=GpsState.DENIED,
            hdop=99.0,
            vdop=99.0,
        )
