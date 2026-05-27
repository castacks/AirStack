"""
GpsDegradationModel — public API.

Create one instance per drone. Call step() every physics step.
Only runs the full pipeline every gps_update_every_n_steps steps; returns
the cached output on non-GPS steps.

Position error is the sum of two physically distinct components:

  1. WLS multipath bias  — instantaneous, geometry-driven.
     The pseudorange WLS solve projects NLOS reflected-signal path offsets
     into ENU position space. This changes as satellites move in/out of LOS.

  2. OU temporal drift  — slow, correlated.
     An Ornstein-Uhlenbeck (Gauss-Markov) process that models ionospheric
     and tropospheric delay variability plus receiver clock drift. The
     diffusion coefficient is scaled by HDOP/VDOP so that good geometry
     keeps drift small, and urban canyons (high DOP) produce large drift.

     Steady-state RMS = ou_base_noise × HDOP × sqrt(ou_correlation_time_s/2)

     The base noise values are saved once at __init__ and multiplied by the
     current HDOP/VDOP at each GPS step. Never scale from an already-scaled
     value — doing so would compound the geometry factor across time steps.

Usage in px4_mavlink_backend.py
--------------------------------
See GPS_DEGRADATION_VM_BRIDGE.md Part 3 for the exact hook location.
Once variable names are known from the VM, the hook pattern is:

    from pegasus.simulator.logic.gps_degradation import (
        GpsDegradationModel, GpsDegradationConfig)

    # In __init__:
    self._gps_degradation = GpsDegradationModel(vehicle_id=self._vehicle_id)
    self._step_count = 0

    # At top of per-step callback:
    self._step_count += 1

    # Just before hil_gps_send:
    _deg = self._gps_degradation.step(
        sim_time_s=self._step_count / 100.0,
        step_count=self._step_count,
        lat_deg=<lat_var> * 1e-7,
        lon_deg=<lon_var> * 1e-7,
        alt_m=<alt_var> * 1e-3,
        world_pos=(<x>, <y>, <z>),
    )
    <lat_var> = int((_deg.delta_lat_deg + <lat_var> * 1e-7) * 1e7)
    <lon_var> = int((_deg.delta_lon_deg + <lon_var> * 1e-7) * 1e7)
    <alt_var> = int((_deg.delta_alt_m   + <alt_var> * 1e-3) * 1e3)
    <eph_var> = min(int(_deg.eph_m * 100), 65535)
    <epv_var> = min(int(_deg.epv_m * 100), 65535)
    <sats_var> = _deg.satellites_visible
    <fix_var>  = _deg.fix_type
"""

import math
from typing import Optional, Tuple

import numpy as np

from .config import GpsDegradationConfig
from .constellation import ConstellationModel
from .visibility import VisibilityEngine
from .dop import compute_dop
from .pseudorange import compute_position_error
from .signal_model import apply_signal_quality
from .state_machine import DegradationStateMachine, DegradationOutput


class GpsDegradationModel:
    """
    Stateful GPS degradation pipeline. One instance per drone.

    Thread safety: not thread-safe. Isaac Sim calls the physics callback
    on a single thread, so no locking is needed in normal use.
    """

    def __init__(
        self,
        vehicle_id: int = 0,
        cfg: Optional[GpsDegradationConfig] = None,
        ros2_node=None,
    ):
        self._cfg = cfg or GpsDegradationConfig()
        self._vehicle_id = vehicle_id
        self._constellation = ConstellationModel(self._cfg)
        self._visibility = VisibilityEngine(self._cfg)
        self._state_machine = DegradationStateMachine(self._cfg)
        self._rng = np.random.default_rng(seed=vehicle_id)

        # ------------------------------------------------------------------ #
        # OU (Gauss-Markov) process for temporal position drift.              #
        # Save base noise values ONCE — scaled by HDOP/VDOP per step.        #
        # Scaling from an already-scaled value would compound the geometry    #
        # factor and grow unboundedly; always multiply from the true baseline. #
        # ------------------------------------------------------------------ #
        self._base_xy_noise = self._cfg.ou_base_xy_noise_m_sqrts
        self._base_z_noise  = self._cfg.ou_base_z_noise_m_sqrts
        self._ou_theta      = 1.0 / self._cfg.ou_correlation_time_s
        # GPS update interval in seconds (assumes 100 Hz physics rate).
        self._ou_dt         = self._cfg.gps_update_every_n_steps / 100.0
        # OU state: [east, north, up] drift in metres.
        self._ou_enu        = np.zeros(3)

        # Constellation refresh interval in seconds.
        self._const_refresh_s = self._cfg.constellation_refresh_steps / 100.0

        # ROS 2 publisher — optional, silently skipped if unavailable.
        self._publisher = None
        if ros2_node is not None:
            try:
                from .ros2_publisher import GpsDegradationPublisher
                self._publisher = GpsDegradationPublisher(vehicle_id, ros2_node)
            except Exception:
                pass

        self._last_output: Optional[DegradationOutput] = None

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
        """
        Call every physics step.

        Args:
            sim_time_s:   Simulation elapsed time in seconds.
            step_count:   Physics step counter (for GPS update gating).
            lat_deg:      Ground-truth drone latitude (degrees).
            lon_deg:      Ground-truth drone longitude (degrees).
            alt_m:        Ground-truth drone altitude MSL (metres).
            world_pos:    Isaac Sim world-frame (x, y, z) in metres.
                          AirStack convention: X=East, Y=North, Z=Up.
            jamming:      If True, return jamming override (no-fix) output.

        Returns:
            DegradationOutput with all fields ready to apply to HIL_GPS.
        """
        if jamming:
            return self._state_machine.force_jamming()

        # Non-GPS steps return the last computed output unchanged.
        if step_count % self._cfg.gps_update_every_n_steps != 0:
            if self._last_output is not None:
                return self._last_output
            # First step before any output: return benign open-sky default.
            return self._state_machine.update(12, 1.0, 1.5, 0.0, 0.0, 0.0,
                                              lat_deg, lon_deg)

        # ------------------------------------------------------------------
        # Step 1: refresh satellite constellation if interval elapsed
        # ------------------------------------------------------------------
        if self._constellation.needs_refresh(sim_time_s, self._const_refresh_s):
            self._constellation.update(sim_time_s)

        # ------------------------------------------------------------------
        # Step 2: satellites above the elevation mask
        # ------------------------------------------------------------------
        sats = self._constellation.get_visible_satellites(lat_deg, lon_deg, alt_m)

        # ------------------------------------------------------------------
        # Step 3: per-satellite LOS/NLOS classification via PhysX raycasting
        # ------------------------------------------------------------------
        visibility = self._visibility.classify(world_pos, sats)

        # ------------------------------------------------------------------
        # Step 4: signal quality filter
        #   - Fully blocked sats (is_los=False, multipath_extra_m=0) dropped
        #   - Sats below cn0_floor_dbhz after elevation + fading model dropped
        # ------------------------------------------------------------------
        visibility = apply_signal_quality(visibility, self._cfg, self._rng)

        # ------------------------------------------------------------------
        # Step 5: DOP from LOS-only geometry matrix
        # ------------------------------------------------------------------
        hdop, vdop, _ = compute_dop(visibility)
        n_los = sum(1 for sv in visibility if sv.is_los)

        # ------------------------------------------------------------------
        # Step 6: WLS pseudorange solve → instantaneous multipath bias
        # ------------------------------------------------------------------
        de_wls, dn_wls, du_wls = compute_position_error(
            visibility, self._cfg, self._rng)

        # ------------------------------------------------------------------
        # Step 7: OU drift — scale diffusion by HDOP/VDOP from step 5.
        #
        # sigma_xy = base_xy_noise × HDOP  (scale from saved baseline)
        # sigma_z  = base_z_noise  × VDOP
        #
        # Exact discrete-time OU (avoids drift for large dt):
        #   x(t+dt) = x(t) × exp(-θ dt) + σ × sqrt((1−exp(−2θ dt))/(2θ)) × N(0,1)
        #
        # The OU term captures atmospheric delay variability and clock drift.
        # The WLS term captures multipath geometry bias.
        # Neither should be filtered again after this point.
        # ------------------------------------------------------------------
        theta = self._ou_theta
        dt    = self._ou_dt

        # Scale diffusion by geometry — always from the saved baseline.
        sigma_xy = self._base_xy_noise * hdop
        sigma_z  = self._base_z_noise  * vdop

        e_decay     = math.exp(-theta * dt)
        # sqrt((1 − e²) / (2θ)) is the exact noise std of the OU increment.
        ou_noise_factor = math.sqrt((1.0 - e_decay * e_decay) / (2.0 * theta))
        noise = self._rng.standard_normal(3)

        self._ou_enu[0] = self._ou_enu[0] * e_decay + sigma_xy * ou_noise_factor * noise[0]
        self._ou_enu[1] = self._ou_enu[1] * e_decay + sigma_xy * ou_noise_factor * noise[1]
        self._ou_enu[2] = self._ou_enu[2] * e_decay + sigma_z  * ou_noise_factor * noise[2]

        # ------------------------------------------------------------------
        # Step 8: combine OU drift with WLS multipath bias
        #
        # total = WLS_multipath_bias + OU_drift
        #   WLS: changes with satellite-building geometry (seconds timescale)
        #   OU:  changes with atmospheric variability (minutes timescale)
        # No additional noise added here — the OU process already carries it.
        # ------------------------------------------------------------------
        total_east  = float(self._ou_enu[0]) + de_wls
        total_north = float(self._ou_enu[1]) + dn_wls
        total_up    = float(self._ou_enu[2]) + du_wls

        # ------------------------------------------------------------------
        # Step 9: state machine + ramp-rate limiting on eph/epv and position
        # ------------------------------------------------------------------
        output = self._state_machine.update(
            n_los=n_los,
            hdop=hdop,
            vdop=vdop,
            delta_east_m=total_east,
            delta_north_m=total_north,
            delta_up_m=total_up,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
        )

        # ------------------------------------------------------------------
        # Step 10: publish ROS 2 status (optional)
        # ------------------------------------------------------------------
        if self._publisher is not None:
            self._publisher.publish(output, n_los)

        self._last_output = output
        return output
