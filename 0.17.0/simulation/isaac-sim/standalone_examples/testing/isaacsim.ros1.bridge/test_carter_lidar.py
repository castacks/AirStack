# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import sys

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="ROS Clock Example")
parser.add_argument("--test", action="store_true")
args, unknown = parser.parse_known_args()

CARTER_STAGE_PATH = "/Carter"
CARTER_USD_PATH = "/Isaac/Robots/Carter/carter_v1_physx_lidar.usd"
BACKGROUND_STAGE_PATH = "/FlatGrid"
BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

CONFIG = {"renderer": "RaytracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)
import carb
import omni
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, prims, rotations, stage, viewports
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

extensions.enable_extension("isaacsim.ros1.bridge")

simulation_app.update()

if args.test:
    from isaacsim.ros1.bridge.scripts.roscore import Roscore
    from isaacsim.ros1.bridge.tests.common import wait_for_rosmaster

    roscore = Roscore()
    wait_for_rosmaster()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.20, 1.20, 0.80]), target=np.array([0, 0, 0.50]))

# Loading the flat grid environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Loading the carter robot USD
prims.create_prim(
    CARTER_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0.25]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=assets_root_path + CARTER_USD_PATH,
)

simulation_app.update()

# Add Lidar publisher
graph_path = "/ActionGraph"

try:
    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # Added nodes used for Lidar Publisher
                ("ReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("PublishLidar", "isaacsim.ros1.bridge.ROS1PublishLaserScan"),
            ],
            keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "ReadLidarBeams.inputs:execIn"),
                ("ReadLidarBeams.outputs:execOut", "PublishLidar.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishLidar.inputs:timeStamp"),
                ("ReadLidarBeams.outputs:azimuthRange", "PublishLidar.inputs:azimuthRange"),
                ("ReadLidarBeams.outputs:depthRange", "PublishLidar.inputs:depthRange"),
                ("ReadLidarBeams.outputs:horizontalFov", "PublishLidar.inputs:horizontalFov"),
                ("ReadLidarBeams.outputs:horizontalResolution", "PublishLidar.inputs:horizontalResolution"),
                ("ReadLidarBeams.outputs:intensitiesData", "PublishLidar.inputs:intensitiesData"),
                ("ReadLidarBeams.outputs:linearDepthData", "PublishLidar.inputs:linearDepthData"),
                ("ReadLidarBeams.outputs:numCols", "PublishLidar.inputs:numCols"),
                ("ReadLidarBeams.outputs:numRows", "PublishLidar.inputs:numRows"),
                ("ReadLidarBeams.outputs:rotationRate", "PublishLidar.inputs:rotationRate"),
            ],
            keys.SET_VALUES: [
                ("ReadLidarBeams.inputs:lidarPrim", [usdrt.Sdf.Path(CARTER_STAGE_PATH + "/chassis_link/carter_lidar")])
            ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
simulation_context.play()

frame = 0

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Publish Lidar each frame
    og.Controller.attribute(graph_path + "/OnImpulseEvent.state:enableImpulse").set(True)

    if frame > 120:
        break
    frame = frame + 1

simulation_context.stop()

if args.test:
    roscore.shutdown()
    roscore = None

simulation_app.close()
