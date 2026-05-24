"""
GpsDegradationModel — public API.

Create one instance per drone. Call step() every physics step.
Only runs the full pipeline every gps_update_every_n_steps steps; returns
the cached output on non-GPS steps.

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

        # IIR low-pass filter on position error.
        # alpha = 1/(tau+1): higher tau = slower response = smoother output.
        tau = self._cfg.position_error_tau_steps
        self._iir_alpha = 1.0 / (tau + 1.0)
        self._smooth_enu = np.zeros(3)

        # Constellation refresh interval in seconds.
        # Assumes physics at 100 Hz; adjust if PX4_PHYSICS_HZ differs.
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
            # First step before any output exists: return a benign open-sky default.
            return self._state_machine.update(12, 1.0, 1.5, 0.0, 0.0, 0.0,
                                              lat_deg, lon_deg)

        # Step 1: refresh satellite positions if interval has elapsed
        if self._constellation.needs_refresh(sim_time_s, self._const_refresh_s):
            self._constellation.update(sim_time_s)

        # Step 2: visible satellites (above elevation mask)
        sats = self._constellation.get_visible_satellites(lat_deg, lon_deg, alt_m)

        # Step 3: LOS/NLOS classification via PhysX raycasting
        visibility = self._visibility.classify(world_pos, sats)

        # Step 4: signal quality filtering (drops blocked + faded satellites)
        visibility = apply_signal_quality(visibility, self._cfg, self._rng)

        # Step 5: DOP from surviving LOS satellites
        hdop, vdop, _ = compute_dop(visibility)
        n_los = sum(1 for sv in visibility if sv.is_los)

        # Step 6: pseudorange WLS → raw position error
        de, dn, du = compute_position_error(visibility, self._cfg, self._rng)

        # Step 7: IIR smoothing on position error
        raw = np.array([de, dn, du])
        self._smooth_enu = (
            (1.0 - self._iir_alpha) * self._smooth_enu
            + self._iir_alpha * raw
        )

        # Step 8: state machine + ramp-rate limiting
        output = self._state_machine.update(
            n_los=n_los,
            hdop=hdop,
            vdop=vdop,
            delta_east_m=float(self._smooth_enu[0]),
            delta_north_m=float(self._smooth_enu[1]),
            delta_up_m=float(self._smooth_enu[2]),
            lat_deg=lat_deg,
            lon_deg=lon_deg,
        )

        # Step 9: publish ROS 2 status
        if self._publisher is not None:
            self._publisher.publish(output, n_los)

        self._last_output = output
        return output
