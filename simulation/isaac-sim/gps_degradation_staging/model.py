"""GPS degradation pipeline — orchestrates all subsystems."""
from __future__ import annotations

import math
import random
from typing import Optional, Tuple

import numpy as np

from .config import GpsDegradationConfig
from .constellation import ConstellationModel
from .visibility import VisibilityChecker
from .signal_model import SignalQualityModel
from .dop import DopCalculator
from .pseudorange import PseudorangeSolver
from .state_machine import DegradationStateMachine, DegradationOutput, GpsState
from .ros2_publisher import Ros2StatePublisher

# Satellite positions change ~2°/min. At 30s refresh the geometry shifts <1°,
# which is orders of magnitude too small to matter for building-scale raycasting.
_CONSTELLATION_REFRESH_S = 30.0


class GpsDegradationModel:
    def __init__(
        self,
        cfg: Optional[GpsDegradationConfig] = None,
        ros2_node=None,
        robot_id: int = 1,
        seed: Optional[int] = None,
        include_ou_drift: bool = True,
    ):
        self._cfg = cfg or GpsDegradationConfig()
        self._rng    = random.Random(seed)
        self._np_rng = np.random.default_rng(seed)
        self._include_ou_drift = include_ou_drift

        self._constellation = ConstellationModel(self._cfg.tle_file_path)
        self._visibility    = VisibilityChecker(self._cfg)
        self._signal_model  = SignalQualityModel(self._cfg, self._rng)
        self._dop           = DopCalculator()
        self._pseudorange   = PseudorangeSolver(self._cfg, self._rng)
        self._state_machine = DegradationStateMachine(self._cfg)
        self._state_pub     = Ros2StatePublisher(ros2_node, robot_id)

        # OU process — save baseline noise ONCE; never scale from already-scaled value
        self._base_xy = self._cfg.ou_base_xy_noise_m_sqrts
        self._base_z  = self._cfg.ou_base_z_noise_m_sqrts
        self._theta   = 1.0 / self._cfg.ou_correlation_time_s
        self._ou_dt   = self._cfg.gps_update_every_n_steps / 100.0
        self._ou_enu  = np.zeros(3)

        self._sats: list = []
        self._last_output = DegradationOutput()
        self._last_constellation_update = -_CONSTELLATION_REFRESH_S  # force first refresh

    def step(
        self,
        sim_time_s: float,
        step_count: int,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        world_pos: Tuple[float, float, float],
        jamming: bool = False,
    ) -> DegradationOutput:
        if step_count % self._cfg.gps_update_every_n_steps != 0 and self._sats:
            return self._last_output

        # Jamming override — bypass pipeline entirely
        if jamming:
            out = DegradationOutput(
                state=GpsState.DENIED,
                n_los=self._cfg.jamming_satellites,
                hdop=99.0, vdop=99.0,
                eph_m=self._cfg.jamming_eph_cm / 100.0,
                epv_m=self._cfg.jamming_epv_cm / 100.0,
                lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m,
            )
            self._last_output = out
            self._state_pub.publish(out)
            return out

        # Time-based constellation refresh (every 30 s of simulation time)
        if (sim_time_s - self._last_constellation_update) >= _CONSTELLATION_REFRESH_S or not self._sats:
            self._sats = self._constellation.get_visible_satellites(
                lat_deg, lon_deg, alt_m, sim_time_s=sim_time_s
            )
            self._last_constellation_update = sim_time_s

        # ── Pipeline ──────────────────────────────────────────────────────────

        # 1. LOS/NLOS via PhysX raycasting (sat.enu_unit → ray direction)
        sv_vis = self._visibility.check(self._sats, world_pos)

        # 2. C/N0 and shadow fading → filter below tracking floor
        sv_sigs = self._signal_model.compute(sv_vis)

        # 3. DOP from LOS-only geometry matrix
        hdop, vdop, _ = self._dop.compute(sv_sigs)
        n_los = sum(1 for s in sv_sigs if s.is_los)

        # 4. WLS multipath bias (instantaneous, geometry-driven)
        de_wls, dn_wls, du_wls = self._pseudorange.solve(sv_sigs)

        # 5. Exact discrete-time OU drift (slow, atmosphere/clock-driven)
        #    x(t+dt) = x(t)·exp(-θ·dt) + σ·√((1-exp(-2θ·dt))/(2θ))·N(0,1)
        theta   = self._theta
        dt      = self._ou_dt
        sig_xy  = self._base_xy * max(hdop, 1.0)   # scale by DOP
        sig_z   = self._base_z  * max(vdop, 1.0)
        decay   = math.exp(-theta * dt)
        diffuse = math.sqrt((1.0 - decay * decay) / (2.0 * theta))
        noise   = self._np_rng.standard_normal(3)
        if self._include_ou_drift:
            self._ou_enu[0] = self._ou_enu[0] * decay + sig_xy * diffuse * noise[0]
            self._ou_enu[1] = self._ou_enu[1] * decay + sig_xy * diffuse * noise[1]
            self._ou_enu[2] = self._ou_enu[2] * decay + sig_z  * diffuse * noise[2]

        # 6. Combined error: OU temporal drift + WLS multipath bias
        east  = float(self._ou_enu[0]) + de_wls
        north = float(self._ou_enu[1]) + dn_wls
        up    = float(self._ou_enu[2]) + du_wls

        # 7. State machine with hysteresis + ramp-rate limiting
        out = self._state_machine.update(n_los, hdop, vdop, east, north, up, lat_deg, lon_deg)
        self._last_output = out
        self._state_pub.publish(out)
        return out

    @property
    def state(self) -> GpsState:
        return self._last_output.state

    @property
    def last_output(self) -> DegradationOutput:
        return self._last_output
