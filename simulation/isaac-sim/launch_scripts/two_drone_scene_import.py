#!/usr/bin/env python3

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
simulation_app = SimulationApp({"headless": False})

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client
import asyncio
import time

from omni.isaac.core.world import World
from omni.isaac.core.objects import GroundPlane
from pxr import Gf, UsdGeom, UsdLux, Sdf, UsdPhysics

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd"
#f"omniverse://{NUCLEUS_SERVER}/Library/Assets/FireAcademyFaro/fire_academy_faro.usd"
#f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/RayFronts-Planner/FireAcademy.scene.usd" 
#f"omniverse://{NUCLEUS_SERVER}/Library/Assets/Fire_Academy_Digital_Twin/fire_academy.usd"
STAGE_SCALE = 0.01


DRONE_USD = "/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# Lighting
ADD_DOME_LIGHT = True
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -3.0

#Drone offset
SPAWN_HEIGHT_ABOVE_FLOOR_M = 0.15

DRONE1_XY_M = (-3.0, 3.5)
DRONE2_XY_M = (3.0, 3.0)
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.physx.forcefields",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "airlab.pegasus",                   # Airlab extension Pegasus core extension
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        # Try immediate enable if available (more robust across Kit versions), fall back otherwise
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            ext_manager.set_extension_enabled(ext, True)


def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


def add_dome_light(stage):
    if stage.GetPrimAtPath(DOME_LIGHT_PATH).IsValid():
        dome = UsdLux.DomeLight.Get(stage, DOME_LIGHT_PATH)
    else:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path(DOME_LIGHT_PATH))

    dome.CreateIntensityAttr(DOME_LIGHT_INTENSITY)
    dome.CreateExposureAttr(DOME_LIGHT_EXPOSURE)


def get_stage_scale(stage):
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if mpu is None or mpu <= 0:
        mpu = 1.0
    s = 1.0 / mpu
    return mpu, s


def add_collision_to_prim(prim):
    if prim.IsA(UsdGeom.Mesh):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            print(f"Added collision to: {prim.GetPath()}")
    
    # Recursively process children
    for child in prim.GetChildren():
        add_collision_to_prim(child)


class PegasusApp:

    def __init__(self):
        omni.client.initialize()
        nucleus_stat(f"omniverse://{NUCLEUS_SERVER}")
        nucleus_stat(ENV_URL)

        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load environment
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        # Wait for the stage to fully load
        for _ in range(100):  # Wait up to ~10 seconds
            omni.kit.app.get_app().update()
            world_prim = stage.GetPrimAtPath("/World")
            if world_prim.IsValid():
                children = list(world_prim.GetChildren())
                # Check if we have more than just PhysicsScene
                non_physics_children = [c for c in children if c.GetName() != "PhysicsScene"]
                if len(non_physics_children) > 0:
                    break
            time.sleep(0.1)

        world_prim = stage.GetPrimAtPath("/World")

        # Scale the /World/stage prim
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            xformable = UsdGeom.Xformable(stage_prim)
            xformable.ClearXformOpOrder()
            
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
            
            scale_op = xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
            scale_op.Set(Gf.Vec3d(STAGE_SCALE, STAGE_SCALE, STAGE_SCALE))
            
            add_collision_to_prim(stage_prim)
            print("Finished adding collisions.")
            
            # Let the app process the changes
            for _ in range(10):
                omni.kit.app.get_app().update()
            
            # Optionally save the stage
            # stage.GetRootLayer().Export("/path/to/save/scene.usd")
            
        else:
            print("Warning: /World/stage not found, environment not scaled")

        # Lighting
        if ADD_DOME_LIGHT:
            add_dome_light(stage)

        # Units
        mpu, s = get_stage_scale(stage)

        drone1_z_m = SPAWN_HEIGHT_ABOVE_FLOOR_M
        drone2_z_m = SPAWN_HEIGHT_ABOVE_FLOOR_M
        
        drone1_pos = [DRONE1_XY_M[0] * s, DRONE1_XY_M[1] * s, drone1_z_m * s]
        drone2_pos = [DRONE2_XY_M[0] * s, DRONE2_XY_M[1] * s, drone2_z_m * s]

        ####################################################################################################
        # Spawn vehicle 1
        ####################################################################################################
        graph_handle1 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_1",
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            vehicle_id=1,  # defines MAVLink port offset
            domain_id=1,  # defines ROS2 domain ID
            usd_file=DRONE_USD,
            init_pos=drone1_pos,
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera (with an associated subgraph) to the drone
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.1, 0.0, 0.0],  # X, Y, Z offset from drone base_link
            camera_rotation_offset=[0.0, 0.0, 0.0],  # Rotation in degrees (roll, pitch, yaw)
        )

        # Add an Ouster lidar (with an associated subgraph) to the drone
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],  # X, Y, Z offset from drone base_link
            lidar_rotation_offset=[0.0, 0.0, 0.0],  # Rotation in degrees (roll, pitch, yaw)
            lidar_min_range = 0.75, # Minimum detection range (m) to avoid propeller hits
        )

        ####################################################################################################
        # Spawn vehicle 2
        ####################################################################################################
        graph_handle2 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_2",
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            vehicle_id=2,  # defines MAVLink port offset. Define as 2 for second vehicle
            domain_id=2,  # defines ROS2 domain ID. Define as 2 for second vehicle
            usd_file=DRONE_USD,
            init_pos=drone2_pos,
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera (with an associated subgraph) to the drone
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            camera_name="ZEDCamera",
            camera_offset=[0.1, 0.0, 0.0],  # X, Y, Z offset from drone base_link
            camera_rotation_offset=[0.0, 0.0, 0.0],  # Rotation in degrees (roll, pitch, yaw)
        )

        # Add an Ouster lidar (with an associated subgraph) to the drone
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],  # X, Y, Z offset from drone base_link
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range = 0.75 
        )

        # Reset so physics/articulations are ready
        self.world.reset()

        self.stop_sim = False

    def run(self):
        # Start sim timeline
        self.timeline.play()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            try:
                self.world.step(render=True)
            except Exception as e:
                carb.log_error(f"Error during simulation step: {e}")
                break

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()