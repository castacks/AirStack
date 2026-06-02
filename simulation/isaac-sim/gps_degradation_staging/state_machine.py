"""GPS degradation state machine with hysteresis and ramp-rate limiting."""
from __future__ import annotations
import math
from dataclasses import dataclass
from enum import IntEnum


class GpsState(IntEnum):
    OPEN_SKY = 0
    DEGRADED = 1
    MARGINAL = 2
    DENIED   = 3
    RECOVERY = 4


@dataclass
class DegradationOutput:
    state: GpsState = GpsState.OPEN_SKY
    n_los: int = 0
    hdop: float = 1.0
    vdop: float = 1.0
    eph_m: float = 1.5
    epv_m: float = 2.5
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_m: float = 0.0
    delta_lat: float = 0.0
    delta_lon: float = 0.0
    delta_alt: float = 0.0

    @property
    def fix_type(self) -> int:
        return {
            GpsState.OPEN_SKY: 3,
            GpsState.DEGRADED: 3,
            GpsState.MARGINAL: 2,
            GpsState.DENIED: 0,
            GpsState.RECOVERY: 1,
        }[self.state]


class DegradationStateMachine:
    # Base eph/epv per state (before DOP scaling)
    _EPH = {GpsState.OPEN_SKY: 1.5, GpsState.DEGRADED: 4.0,
            GpsState.MARGINAL: 12.0, GpsState.DENIED: 99.0, GpsState.RECOVERY: 6.0}
    _EPV = {GpsState.OPEN_SKY: 2.5, GpsState.DEGRADED: 7.0,
            GpsState.MARGINAL: 20.0, GpsState.DENIED: 99.0, GpsState.RECOVERY: 10.0}

    def __init__(self, cfg):
        self._cfg = cfg
        self._state = GpsState.OPEN_SKY
        self._eph = 1.5
        self._epv = 2.5
        self._recovery_count = 0
        # Ramp-limited delta position (degrees lat/lon, meters alt)
        self._dlat = 0.0
        self._dlon = 0.0
        self._dalt = 0.0

    def update(
        self,
        n_los: int,
        hdop: float,
        vdop: float,
        total_east: float,
        total_north: float,
        total_up: float,
        base_lat: float,
        base_lon: float,
    ) -> DegradationOutput:
        self._state = self._next_state(n_los, hdop)

        # PX4 HIL_GPS reports uncertainty in centimetres. Position uncertainty
        # grows linearly with DOP times the receiver's baseline UERE.
        eph_tgt = self._cfg.uere_accuracy_m * max(hdop, 1.0)
        epv_tgt = self._cfg.uere_accuracy_m * max(vdop, 1.0)

        # Ramp-rate limit accuracy estimates
        s = self._cfg.max_eph_step_m
        self._eph += max(-s, min(s, eph_tgt - self._eph))
        self._epv += max(-s, min(s, epv_tgt - self._epv))

        # ENU offset → geodetic delta
        R = 6_371_000.0
        cos_lat = math.cos(math.radians(base_lat))
        if self._state == GpsState.DENIED:
            dlat_tgt = dlon_tgt = dalt_tgt = 0.0
        else:
            dlat_tgt = math.degrees(total_north / R)
            dlon_tgt = math.degrees(total_east / (R * cos_lat + 1e-9))
            dalt_tgt = total_up

        # Ramp-rate limit position delta (convert max_position_step_m to degrees)
        max_deg = math.degrees(self._cfg.max_position_step_m / R)
        max_alt = self._cfg.max_position_step_m
        self._dlat += max(-max_deg, min(max_deg, dlat_tgt - self._dlat))
        self._dlon += max(-max_deg, min(max_deg, dlon_tgt - self._dlon))
        self._dalt += max(-max_alt, min(max_alt, dalt_tgt - self._dalt))

        return DegradationOutput(
            state=self._state,
            n_los=n_los,
            hdop=hdop,
            vdop=vdop,
            eph_m=self._eph,
            epv_m=self._epv,
            lat_deg=base_lat + self._dlat,
            lon_deg=base_lon + self._dlon,
            alt_m=self._dalt,  # delta alt (added to raw alt by publisher)
            delta_lat=self._dlat,
            delta_lon=self._dlon,
            delta_alt=self._dalt,
        )

    def _next_state(self, n_los: int, hdop: float) -> GpsState:
        thr = self._cfg.state_thresholds
        cur = self._state

        d_n_enter, d_n_exit = thr.get("denied_nsats",   (4, 5))
        d_h_enter, d_h_exit = thr.get("denied_hdop",    (5.0, 4.5))
        m_n_enter, m_n_exit = thr.get("marginal_nsats", (6, 7))
        m_h_enter, m_h_exit = thr.get("marginal_hdop",  (3.0, 2.5))

        # Enter DENIED
        if n_los <= d_n_enter or hdop >= d_h_enter:
            self._recovery_count = 0
            return GpsState.DENIED

        # Exit DENIED via RECOVERY
        if cur == GpsState.DENIED:
            if n_los >= d_n_exit and hdop <= d_h_exit:
                self._recovery_count += 1
                if self._recovery_count >= self._cfg.recovery_steps:
                    self._recovery_count = 0
                    return GpsState.RECOVERY
            else:
                self._recovery_count = 0
            return GpsState.DENIED

        # Exit RECOVERY
        if cur == GpsState.RECOVERY:
            if n_los >= m_n_exit and hdop <= m_h_exit:
                return GpsState.OPEN_SKY
            return GpsState.RECOVERY

        # Enter/exit MARGINAL
        if n_los <= m_n_enter or hdop >= m_h_enter:
            return GpsState.MARGINAL
        if cur == GpsState.MARGINAL:
            if n_los >= m_n_exit and hdop <= m_h_exit:
                return GpsState.DEGRADED
            return GpsState.MARGINAL

        # DEGRADED vs OPEN_SKY
        if n_los < 8 or hdop >= 1.5:
            return GpsState.DEGRADED
        return GpsState.OPEN_SKY
