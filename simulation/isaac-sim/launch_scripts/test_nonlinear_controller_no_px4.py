#!/usr/bin/env python
"""
Benchmark 1 — Pegasus Multirotor with in-process nonlinear controller, no PX4, no OmniGraph sensors.

Isolates the cost of the Pegasus Multirotor object itself (physics body + rotor
thrust simulation) compared to the other scripts that add PX4 and RTX sensors.

The drone follows a sinusoidal back-and-forth trajectory at 3 m altitude.
Two rigid-body cubes fall from above to provide a fixed physics probe workload.

Wall-clock / simulation-time ratio is printed every 5 s.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import sys
import math

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS, WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import add_dome_light, spawn_falling_cubes, SimTimer


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DRONE_USD = os.path.expanduser(
    "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator"
    "/pegasus/simulator/assets/Robots/Iris/iris.usd"
)
CRUISE_ALT = 3.0        # m above ground
HALF_RANGE = 4.0        # m — sinusoid amplitude
PERIOD_S   = 12.0       # s — time for one back-and-forth cycle


# ---------------------------------------------------------------------------
# Back-and-forth nonlinear controller
# ---------------------------------------------------------------------------

class BackAndForthController(Backend):
    """SE(3) geometric controller that tracks a sinusoidal back-and-forth trajectory."""

    def __init__(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.R = Rotation.identity()
        self.w = np.zeros(3)

        self.Kp = np.diag([10.0, 10.0, 10.0])
        self.Kd = np.diag([8.5,  8.5,  8.5])
        self.Ki = np.diag([1.5,  1.5,  1.5])
        self.Kr = np.diag([3.5,  3.5,  3.5])
        self.Kw = np.diag([0.5,  0.5,  0.5])

        self.integral = np.zeros(3)
        self.m  = 1.50
        self.g  = 9.81
        self.t  = -3.0   # start negative → takeoff phase before the sinusoid

        # Rotor mixing (Iris quad-X)
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------
    def update_state(self, state: State):
        self.p = state.position
        self.v = state.linear_velocity
        self.R = Rotation.from_matrix(state.attitude)
        self.w = state.angular_velocity

    def input_reference(self):
        return self.input_ref

    def update(self, dt: float):
        self.t += dt

        # Target position
        if self.t < 0.0:
            # Takeoff: rise to cruise altitude
            frac = max(0.0, min(1.0, 1.0 + self.t / 3.0))
            pd = np.array([0.0, 0.0, CRUISE_ALT * frac])
            vd = np.zeros(3)
            ad = np.zeros(3)
        else:
            phase = 2.0 * math.pi * self.t / PERIOD_S
            pd = np.array([HALF_RANGE * math.sin(phase), 0.0, CRUISE_ALT])
            vd = np.array([HALF_RANGE * (2.0 * math.pi / PERIOD_S) * math.cos(phase), 0.0, 0.0])
            ad = np.array([-HALF_RANGE * (2.0 * math.pi / PERIOD_S)**2 * math.sin(phase), 0.0, 0.0])

        # Position / velocity error
        ep = self.p - pd
        ev = self.v - vd

        self.integral = np.clip(self.integral + ep * dt, -2.0, 2.0)

        # Desired force in world frame
        F_des = (
            -self.Kp @ ep
            - self.Kd @ ev
            - self.Ki @ self.integral
            + self.m * np.array([0.0, 0.0, self.g])
            + self.m * ad
        )

        # Desired heading — always face +X
        b1d = np.array([1.0, 0.0, 0.0])
        b3d = F_des / (np.linalg.norm(F_des) + 1e-6)
        b2d = np.cross(b3d, b1d); b2d /= np.linalg.norm(b2d) + 1e-6
        b1d = np.cross(b2d, b3d)
        Rd = np.column_stack([b1d, b2d, b3d])

        R_cur = self.R.as_matrix()
        eR = 0.5 * (Rd.T @ R_cur - R_cur.T @ Rd)
        eR_vec = np.array([eR[2, 1], eR[0, 2], eR[1, 0]])

        # Torque
        tau = -self.Kr @ eR_vec - self.Kw @ self.w

        thrust = float(np.dot(F_des, R_cur[:, 2]))
        thrust = max(0.0, thrust)

        # Convert thrust + torque → per-rotor speed² (Iris geometry)
        ct, l, cq = 8.54858e-6, 0.17, 0.016
        A = np.array([
            [ ct,   ct,   ct,   ct],
            [ 0,   -ct*l,  0,   ct*l],
            [ ct*l,  0,  -ct*l,  0],
            [-cq,   cq,  -cq,   cq],
        ])
        b = np.array([thrust, tau[0], tau[1], tau[2]])
        try:
            w2 = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            w2 = np.zeros(4)

        w2 = np.clip(w2, 0.0, None)
        self.input_ref = list(np.sqrt(w2))

    def start(self): pass
    def stop(self):  pass
    def reset(self): self.t = -3.0; self.integral[:] = 0.0


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])

        stage = omni.usd.get_context().get_stage()
        add_dome_light(stage)
        spawn_falling_cubes(stage)

        # Spawn drone via Python API — no OmniGraph, no PX4
        controller = BackAndForthController()
        config = MultirotorConfig()
        config.backends = [controller]
        config.sensors = []          # suppress default sensor suite

        self.vehicle = Multirotor(
            "/World/Iris",
            DRONE_USD,
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0]).as_quat(),
            config=config,
        )

        self.timer = SimTimer("no-px4-nonlinear", interval_s=5.0)

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                self.timer.tick(self.world.current_time)
            else:
                omni.kit.app.get_app().update()

        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        simulation_app.close()


def main():
    PegasusApp().run()


if __name__ == "__main__":
    main()
