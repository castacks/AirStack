# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# This file is meant as a tool for the isaac sim developers to test and debug.
# It is not meant for users, so use at your own risk.
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--geo-type", type=str, choices=["cubes", "sphere"], default="cubes", help="Shape to spawn in scene"
)
parser.add_argument("--config", type=str, default="Example_Rotary", help="Lidar config name")
args, _ = parser.parse_known_args()
geo_type = args.geo_type
lidar_config = args.config

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import carb
import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

enable_extension("isaacsim.ros2.bridge")

simulation_app.update()


def printinc(i):
    print(f"{i}")
    return i + 1


i = 0

i = printinc(i)  # 0


def add_cube(stage, path, scale, offset, physics=False):
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)
    cubeGeom.CreateSizeAttr(1.0)
    cubeGeom.AddTranslateOp().Set(offset)
    cubeGeom.AddScaleOp().Set(scale)
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    return cubePrim


i = printinc(i)  # 2
simulation_app.update()

if geo_type == "cubes":
    # add_cube(stage.get_current_stage(), "/World/cxube_x2", (1, 20, 1000), (-5, 0, 500), physics=True)
    add_cube(stage.get_current_stage(), "/World/cxube_x1", (1, 20, 5), (5, 0, 0), physics=False)
    add_cube(stage.get_current_stage(), "/World/cxube_x2", (1, 20, 1), (-5, 0, 0), physics=False)
    add_cube(stage.get_current_stage(), "/World/cxube_x3", (20, 1, 1), (0, 5, 0), physics=False)
    add_cube(stage.get_current_stage(), "/World/cxube_x4", (20, 1, 1), (0, -5, 0), physics=False)
    add_cube(stage.get_current_stage(), "/World/cxube_x5", (20, 1, 1), (-5, -5, 0), physics=False)
    add_cube(
        stage.get_current_stage(),
        "/World/cube_5",
        (0.1764972, 2.0025313, 1.5832705),
        (-3.0258131660928367, 0, 0),
        physics=False,
    )

elif geo_type == "sphere":
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="Sphere",
        attributes={"radius": 5, "extent": [(-5, -5, -5), (5, 5, 5)]},
    )

lidar_config = "RPLIDAR_S2E"
lidar_config = "SICK_microscan3_ABAZ90ZA1P01"
lidar_config = "Sick_MISC3"
lidar_config = "Example_Rotary"
lidar_config = "Example_Solid_State"

omni.kit.commands.execute(
    "CreatePrim", prim_type="DomeLight", attributes={"inputs:intensity": 1000, "inputs:texture:format": "latlong"}
)

# configNames = [
#     "SICK_tim781",  # ok, ros2 scan looks like curve
#     "SICK_tim781_legacy",  # ok
#     "SICK_picoScan150",  # ok
#     "SICK_multiScan136",  # flat scan strange (OK, not evenly spaced or single elevation in the line)
#     "SICK_multiScan165",  # flat scan strange (OK, not evenly spaced or single elevation in the line)
# ]
# lidar_config = "SICK_multiScan165"


i = printinc(i)  # 3
simulation_app.update()

# Create the lidar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about X so that its Z up

# Possible options are Example_Rotary and Example_Solid_State
# drive sim applies 0.5,-0.5,-0.5,w(-0.5), we have to apply the reverse

i = printinc(i)  # 4
_, sensor1 = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensor_solid_state",
    parent=None,
    config=lidar_config,
    translation=(0, 0, -0.04),
    orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
)

i = printinc(i)  # 4
_, sensor2 = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensor_solid_state_2",
    parent=None,
    config=lidar_config,
    translation=(0, 0, -0.04),
    orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
)

i = printinc(i)  # 5
hydra_texture_1 = rep.create.render_product(sensor1.GetPath(), [1, 1], name="Isaac").path
hydra_texture_2 = rep.create.render_product(sensor2.GetPath(), [1, 1], name="Isaac").path

# Create the debug draw pipeline in the post process graph
from omni.syntheticdata import sensors

i = printinc(i)
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)

i = printinc(i)
writerNames = [
    # "Writer" + "IsaacPrintRTXLidarInfo",
    # "Writer" + "IsaacReadRTXLidarData",
    # "RtxLidar" + "DebugDrawPointCloud",
    # "RtxLidar" + "DebugDrawPointCloud" + "Buffer",
    "RtxLidar"
    + "ROS2PublishLaserScan",
]

annoNames = [
    # "RtxSensorCpuIsaacReadRTXLidarData",
    # "RtxSensorCpuIsaacComputeRTXLidarPointCloud",
    # "RtxSensorCpuIsaacCreateRTXLidarScanBuffer",
    # "RtxSensorCpuIsaacComputeRTXLidarFlatScan",
]
writers = {}
for writ in writerNames:
    writers[writ] = rep.writers.get(writ)
    writers[writ].attach([hydra_texture_1])  # , render_product_path2])
    writers[writ].attach([hydra_texture_2])  # , render_product_path2])
# writer.initialize(testMode=True)
annotators = {}
for anno in annoNames:
    annotators[anno] = rep.AnnotatorRegistry.get_annotator(anno)
    # annotators[anno].initialize(keepOnlyPositiveDistance=True)
    annotators[anno].attach([hydra_texture_1])
    annotators[anno].attach([hydra_texture_2])

# disable_extension("omni.replicator.core")
i = printinc(i)
simulation_app.update()
simulation_app.update()


# omni.kit.commands.execute(
#    "ChangeProperty",
#    prop_path=Sdf.Path("/Render/PostProcess/SDGPipeline/DispatchSync.inputs:enabled"),
#    value=True,
#    prev=None,
# )

i = printinc(i)
simulation_context.play()

i = printinc(i)


while simulation_app.is_running():
    simulation_app.update()
    if simulation_context.is_playing():
        for anno in annotators:
            print(f"~~~{anno} Data~~")
            data = annotators[anno].get_data()
            for entry in data:
                print(f"{entry}: ", end="")
                if hasattr(data[entry], "__len__"):
                    print(f"len {len(data[entry])}::", end="")
                print(data[entry])

# cleanup and shutdown

i = printinc(i)
simulation_context.stop()

i = printinc(i)
simulation_app.close()
"""
# Snippet of similar code to use in script editor.
from isaacsim.core.utils import stage
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, Gf

#omni.kit.commands.execute('ToolbarPlayButtonClicked')

UsdGeom.Cube.Define(stage.get_current_stage(), "/World/cube_1").AddTranslateOp().Set((5, 5, 0))
import omni.kit.commands
_, sensorR = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensorR",
    parent=None,
    config="Example_Solid_State",
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)

hydra_textureR = rep.create.render_product(sensorR.GetPath(), [1, 1], name="Isaac")

import omni.replicator.core as rep
# Create the debug draw pipeline in the post process graph
writerR = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
writerR.attach([hydra_textureR])


~~~Create a Camera then use this to add a lidar to it~~

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/Camera")
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
IsaacSensorSchema.IsaacRtxLidarSensorAPI.Apply(prim)
from pxr import Sdf
camSensorTypeAttr = prim.CreateAttribute("cameraSensorType", Sdf.ValueTypeNames.Token, False)
camSensorTypeAttr.Set("lidar")
tokens = camSensorTypeAttr.GetMetadata("allowedTokens")
if not tokens:
    camSensorTypeAttr.SetMetadata("allowedTokens", ["camera", "radar", "lidar"])
prim.CreateAttribute("sensorModelPluginName", Sdf.ValueTypeNames.String, False).Set("omni.sensors.nv.lidar.lidar_core.plugin")
prim.CreateAttribute("sensorModelConfig", Sdf.ValueTypeNames.String, False).Set("Example_Rotary")

import omni.replicator.core as rep
hydra_texture = rep.create.render_product("/Camera", [1, 1], name="Isaac")
writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
writer.attach([hydra_texture])
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
import asyncio
import omni.replicator.core as rep
import carb.settings
import omni.usd

RP_RESOLUTION = (1280, 720)
NUM_RP = 6

async def create_new_stage_with_rp_async():
    omni.usd.get_context().new_stage()
    from isaacsim.core.utils import stage
    stage.add_reference_to_stage("omniverse://isaac-dev.ov.nvidia.com/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/background")
    # None (`0`), TAA (`1`), FXAA (`2`), DLSS (`3`) and DLAA (`4`)
    # carb.settings.get_settings().set("/rtx/post/aa/op", 4)
    for i in range(NUM_RP):
        rep.create.render_product("/OmniverseKit_Persp", RP_RESOLUTION, name=f"rp_{i}")

asyncio.ensure_future(create_new_stage_with_rp_async())
"""
