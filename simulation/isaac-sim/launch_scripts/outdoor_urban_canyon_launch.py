"""Isaac Sim urban-canyon scene for GPS degradation testing.

Launch:
    ISAACSIM_PYTHON launch_scripts/outdoor_urban_canyon_launch.py [--scenario urban]

Buildings and ground are registered with UsdPhysics.CollisionAPI so PhysX raycasting
can detect them for LOS/NLOS classification.
"""
# SimulationApp MUST be created before any omni.* imports
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf
import rclpy

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_LAUNCH_DIR, "..")))

from gps_degradation_node import GPSDegradationNode
from gps_degradation_staging import GpsDegradationConfig


# (x, y, half-width, half-depth, half-height) in meters — approximate urban block
_BUILDINGS = [
    ( 30.0,   0.0, 10.0, 20.0, 30.0),
    (-30.0,   0.0, 10.0, 20.0, 25.0),
    (  0.0,  40.0, 20.0, 10.0, 35.0),
    (  0.0, -40.0, 20.0, 10.0, 22.5),
]


def _apply_collision(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() and not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)


class UrbanCanyonApp:
    def __init__(self, robot_id: int = 1, scenario: str = "urban"):
        self._robot_id = robot_id
        self._scenario = scenario
        self._world = None
        self._gps_node = None

    def _build_scene(self):
        from omni.isaac.core import World
        self._world = World(stage_units_in_meters=1.0)
        stage = omni.usd.get_context().get_stage()

        # Ground plane
        ground = stage.DefinePrim("/World/Ground", "Cube")
        UsdGeom.XformCommonAPI(ground).SetTranslate(Gf.Vec3d(0.0, 0.0, -0.5))
        UsdGeom.XformCommonAPI(ground).SetScale(Gf.Vec3f(200.0, 200.0, 1.0))
        _apply_collision(stage, "/World/Ground")

        # Buildings
        for i, (bx, by, hw, hd, hh) in enumerate(_BUILDINGS):
            path = f"/World/Building_{i}"
            prim = stage.DefinePrim(path, "Cube")
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(bx, by, hh))
            UsdGeom.XformCommonAPI(prim).SetScale(Gf.Vec3f(hw, hd, hh))
            _apply_collision(stage, path)

        # Physics scene required for raycasting
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        self._world.reset()

    def _setup_ros(self):
        if not rclpy.ok():
            rclpy.init()
        cfg = GpsDegradationConfig(scenario=self._scenario)
        self._gps_node = GPSDegradationNode(
            vehicle_id=self._robot_id,
            scenario=self._scenario,
            cfg=cfg,
        )

    def run(self):
        self._build_scene()
        self._setup_ros()

        from omni.timeline import get_timeline_interface
        get_timeline_interface().play()

        print(f"[UrbanCanyonApp] robot={self._robot_id}  scenario={self._scenario}")
        step = 0
        while simulation_app.is_running():
            self._world.step(render=False)
            t = float(self._world.current_time)

            # In a real drone scenario, read prim transform here
            self._gps_node.set_world_pos(0.0, 0.0, 10.0)
            self._gps_node.physics_step(t)
            rclpy.spin_once(self._gps_node, timeout_sec=0.0)

            step += 1
            if step % 500 == 0:
                o = self._gps_node._model.last_output
                print(
                    f"  t={t:7.1f}s  {o.state.name:<10}  "
                    f"n_los={o.n_los:2d}  hdop={o.hdop:.2f}  "
                    f"eph={o.eph_m:.2f}m  epv={o.epv_m:.2f}m  "
                    f"Δalt={o.delta_alt:+.2f}m"
                )

        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", type=int, default=1)
    ap.add_argument("--scenario", default="urban",
                    choices=["open_sky", "suburban", "urban", "dense_urban"])
    args = ap.parse_args()
    UrbanCanyonApp(robot_id=args.robot_id, scenario=args.scenario).run()
