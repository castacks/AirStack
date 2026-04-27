#!/usr/bin/env python
"""
Benchmark 2 — Full OmniGraph + RTX sensors (ZED + Ouster), PX4 node ticking,
but drone physics driven by an in-process nonlinear controller (no PX4 process).

Purpose
-------
Isolates the cost of the OmniGraph sensor pipeline.  Compare with:
  • test_nonlinear_controller_no_px4.py — cost of bare Multirotor (no sensors)
  • example_one_px4_pegasus_launch_script.py — cost of full PX4 lockstep

The Multirotor is spawned via the Python API so that it begins simulating
immediately at Play.  The OmniGraph PX4 node is present but its vehicle-
creation block (compute_base) will fire on the first tick — the two Python
objects share the same USD prim and coexist without spawning a second body.

Two rigid-body cubes fall from above to provide a fixed physics probe.
Wall-clock / simulation-time ratio is printed every 5 s.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import sys
import math
import time

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

# Enable required extensions before any Pegasus / OmniGraph imports
_ext_manager = omni.kit.app.get_app().get_extension_manager()
for _ext in [
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.action_nodes",
    "isaacsim.core.nodes",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.ui_nodes",
    "pegasus.simulator",
]:
    if not _ext_manager.is_extension_enabled(_ext):
        _ext_manager.set_extension_enabled_immediate(_ext, True)

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import add_dome_light, spawn_falling_cubes, SimTimer


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DRONE_PRIM = "/World/base_link"
DRONE_USD  = os.path.expanduser(
    "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator"
    "/pegasus/simulator/assets/Robots/Iris/iris.usd"
)
CRUISE_ALT = 3.0
HALF_RANGE = 4.0
PERIOD_S   = 12.0


# ---------------------------------------------------------------------------
# Back-and-forth nonlinear controller (identical to test_nonlinear_controller)
# ---------------------------------------------------------------------------

class BackAndForthController(Backend):
    """SE(3) geometric controller — sinusoidal back-and-forth at CRUISE_ALT."""

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
        self.t  = -3.0
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

    def update_state(self, state: State):
        self.p = state.position
        self.v = state.linear_velocity
        self.R = Rotation.from_matrix(state.attitude)
        self.w = state.angular_velocity

    def input_reference(self):
        return self.input_ref

    def update(self, dt: float):
        self.t += dt

        if self.t < 0.0:
            frac = max(0.0, min(1.0, 1.0 + self.t / 3.0))
            pd = np.array([0.0, 0.0, CRUISE_ALT * frac])
            vd = np.zeros(3)
            ad = np.zeros(3)
        else:
            phase = 2.0 * math.pi * self.t / PERIOD_S
            pd = np.array([HALF_RANGE * math.sin(phase), 0.0, CRUISE_ALT])
            vd = np.array([HALF_RANGE * (2.0 * math.pi / PERIOD_S) * math.cos(phase), 0.0, 0.0])
            ad = np.array([-HALF_RANGE * (2.0 * math.pi / PERIOD_S)**2 * math.sin(phase), 0.0, 0.0])

        ep = self.p - pd
        ev = self.v - vd
        self.integral = np.clip(self.integral + ep * dt, -2.0, 2.0)

        F_des = (
            -self.Kp @ ep
            - self.Kd @ ev
            - self.Ki @ self.integral
            + self.m * np.array([0.0, 0.0, self.g])
            + self.m * ad
        )

        b1d = np.array([1.0, 0.0, 0.0])
        b3d = F_des / (np.linalg.norm(F_des) + 1e-6)
        b2d = np.cross(b3d, b1d); b2d /= np.linalg.norm(b2d) + 1e-6
        b1d = np.cross(b2d, b3d)
        Rd  = np.column_stack([b1d, b2d, b3d])

        R_cur = self.R.as_matrix()
        eR = 0.5 * (Rd.T @ R_cur - R_cur.T @ Rd)
        eR_vec = np.array([eR[2, 1], eR[0, 2], eR[1, 0]])
        tau = -self.Kr @ eR_vec - self.Kw @ self.w

        thrust = max(0.0, float(np.dot(F_des, R_cur[:, 2])))

        ct, l, cq = 8.54858e-6, 0.17, 0.016
        A = np.array([
            [ ct,   ct,   ct,   ct],
            [ 0,  -ct*l,   0,  ct*l],
            [ct*l,   0, -ct*l,   0],
            [-cq,   cq,  -cq,   cq],
        ])
        try:
            w2 = np.linalg.solve(A, np.array([thrust, tau[0], tau[1], tau[2]]))
        except np.linalg.LinAlgError:
            w2 = np.zeros(4)

        self.input_ref = list(np.sqrt(np.clip(w2, 0.0, None)))

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

        # Wait for stage to load
        stage = omni.usd.get_context().get_stage()
        for _ in range(100):
            omni.kit.app.get_app().update()
            if stage.GetPrimAtPath("/World").IsValid():
                break
            time.sleep(0.05)

        add_dome_light(stage)
        spawn_falling_cubes(stage)

        # ------------------------------------------------------------------
        # Build OmniGraph: PX4 node + ZED + Ouster
        # The PX4 OmniGraph node defines the graph topology and publishes ROS 2
        # sensor data.  The Multirotor physics object is created below via the
        # Python API (spawn_prim=False) so we control it with the nonlinear
        # controller rather than a PX4 process.
        # ------------------------------------------------------------------
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim=DRONE_PRIM,
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=DRONE_PRIM,
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=DRONE_PRIM,
            robot_name="robot_1",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,
        )

        # ------------------------------------------------------------------
        # Spawn Multirotor via Python API on the same prim — no second body is
        # created; the OmniGraph node will find the prim already present.
        # ------------------------------------------------------------------
        controller = BackAndForthController()
        config = MultirotorConfig()
        config.backends = [controller]
        config.sensors = []          # OmniGraph handles sensor publishing

        self.vehicle = Multirotor(
            DRONE_PRIM,
            DRONE_USD,
            1,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0]).as_quat(),
            config=config,
        )

        self.timer = SimTimer("omnigraph-sensors-nonlinear", interval_s=5.0)

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
