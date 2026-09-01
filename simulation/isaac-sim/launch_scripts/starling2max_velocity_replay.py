#!/usr/bin/env python
"""Isaac / Pegasus scene for hardware velocity-command replay.

Loads starling2max.usd as the physics plant, inspects/fixes physics:mass,
spawns at the hardware start pose, and starts PX4 SITL. No camera/lidar.
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"
if _LIVESTREAM:
    _SIM_APP_CONFIG = {
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
    }
else:
    _headless = os.environ.get("ISAAC_SIM_HEADLESS", "").lower() == "true"
    _SIM_APP_CONFIG = {"headless": _headless}

simulation_app = SimulationApp(launch_config=_SIM_APP_CONFIG)

if _LIVESTREAM:
    from isaacsim.core.utils.extensions import enable_extension
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/enabled", True)
    LIVESTREAM_UDP_PORT = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
    simulation_app.set_setting("/app/livestream/fixedHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/minHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/maxHostPort", LIVESTREAM_UDP_PORT)
    enable_extension("omni.kit.livestream.webrtc")

import omni.kit.app
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from pxr import UsdPhysics

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import add_dome_light

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
DRONE_USD = os.environ.get(
    "STARLING_USD",
    "/isaac-sim/AirStack/simulation/isaac-sim/assets/robots/starling2max.usd",
)
TARGET_MASS_KG = 0.557
# Hardware mocap start is z=0.0012 (wheels/props on the floor). The USD body
# origin is near the COM; spawning at 1 mm puts the mesh in the ground plane
# and PX4 cannot lift off. Hold XY, lift Z so the lowest point is above the
# default Grid ground. The 8 cm bias is removed from overlay plots.
START_POS = [0.0242, 0.0353, 0.08]
START_YAW_QUAT = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w
DRONE_PRIM = "/World/base_link"


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


def _attr(prim, name):
    a = prim.GetAttribute(name)
    if a and (a.HasAuthoredValue() or a.HasValue()):
        try:
            return a.Get()
        except Exception:
            return None
    return None


def inspect_and_fix_plant(stage, drone_prim_path: str, target_mass: float) -> None:
    print(f"[Plant] inspecting under {drone_prim_path}")
    masses = []
    rotors = []
    drone = stage.GetPrimAtPath(drone_prim_path)
    if not drone.IsValid():
        print(f"[Plant] ERROR: {drone_prim_path} is not a valid prim")
        return

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(drone_prim_path):
            continue
        mass = _attr(prim, "physics:mass")
        if mass is not None:
            masses.append((path, float(mass)))
            com = _attr(prim, "physics:centerOfMass")
            iner = _attr(prim, "physics:diagonalInertia")
            print(f"[Plant] MASS {path} = {mass}  COM={com}  I={iner}")
        if "rotor" in prim.GetName().lower():
            rotors.append(path)
            print(f"[Plant] ROTOR {path} type={prim.GetTypeName()}")

    if not masses:
        print("[Plant] WARNING: no physics:mass authored under the drone prim")
        vehicle = stage.GetPrimAtPath(f"{drone_prim_path}/vehicle")
        if vehicle.IsValid():
            if not vehicle.HasAPI(UsdPhysics.MassAPI):
                UsdPhysics.MassAPI.Apply(vehicle)
            vehicle.GetAttribute("physics:mass").Set(target_mass)
            print(f"[Plant] applied MassAPI on {vehicle.GetPath()} mass={target_mass}")
    else:
        path, m = max(masses, key=lambda x: x[1])
        print(f"[Plant] primary mass prim {path} = {m} kg")
        if abs(m - target_mass) > 1e-4:
            stage.GetPrimAtPath(path).GetAttribute("physics:mass").Set(target_mass)
            print(f"[Plant] set physics:mass {m} -> {target_mass}")
        else:
            print(f"[Plant] physics:mass already {m} kg (target {target_mass})")

    for i in range(4):
        expected = f"{drone_prim_path}/rotor{i}"
        if stage.GetPrimAtPath(expected).IsValid():
            print(f"[Plant] {expected} present")
            continue
        matches = [r for r in rotors if r.rstrip("/").endswith(f"rotor{i}")]
        if not matches:
            print(f"[Plant] WARNING: no rotor{i} prim; Pegasus applies force at {expected}")
            continue
        src = matches[0]
        print(f"[Plant] remapping {src} -> {expected}")
        try:
            import omni.kit.commands
            omni.kit.commands.execute("MovePrim", path_from=src, path_to=expected)
        except Exception as exc:
            print(f"[Plant] MovePrim failed for rotor{i}: {exc}")


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
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
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)


class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.timeline.stop()

        usd = os.path.expanduser(DRONE_USD)
        if not os.path.isfile(usd):
            raise FileNotFoundError(f"starling2max USD not found: {usd}")
        print(f"[Plant] using USD {usd}")

        self.pg.load_environment(ENV_URL)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        add_dome_light(stage)

        spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim=DRONE_PRIM,
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=usd,
            init_pos=START_POS,
            init_orient=START_YAW_QUAT,
        )
        for _ in range(5):
            omni.kit.app.get_app().update()

        inspect_and_fix_plant(omni.usd.get_context().get_stage(), DRONE_PRIM, TARGET_MASS_KG)
        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()
        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
