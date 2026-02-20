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
parser.add_argument("--config", type=str, default="Example", help="Radar config name")
args, _ = parser.parse_known_args()
geo_type = args.geo_type
config = args.config

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import carb
import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


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

omni.kit.commands.execute(
    "CreatePrim", prim_type="DomeLight", attributes={"inputs:intensity": 1000, "inputs:texture:format": "latlong"}
)

i = printinc(i)  # 3
simulation_app.update()

# Create the lidar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about X so that its Z up

# Possible options are Example_Rotary and Example_Solid_State
# drive sim applies 0.5,-0.5,-0.5,w(-0.5), we have to apply the reverse

i = printinc(i)  # 4
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config=config,
    translation=(0, 0, -0.04),
    orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
)

i = printinc(i)  # 5
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

# Create the debug draw pipeline in the post process graph
from omni.syntheticdata import sensors

i = printinc(i)
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)

i = printinc(i)
writerNames = [
    "RtxRadar" + "DebugDrawPointCloud",
    # "Writer" + "IsaacPrintRTXSensorInfo",
]


annoNames = []
writers = {}
for writ in writerNames:
    writers[writ] = rep.writers.get(writ)
    writers[writ].attach([hydra_texture])

annotators = {}
for anno in annoNames:
    annotators[anno] = rep.AnnotatorRegistry.get_annotator(anno)
    annotators[anno].attach([hydra_texture])


i = printinc(i)
simulation_app.update()

i = printinc(i)
simulation_context.play()

i = printinc(i)
while simulation_app.is_running():
    simulation_app.update()

# cleanup and shutdown

i = printinc(i)
simulation_context.stop()

i = printinc(i)
simulation_app.close()
"""
import omni.replicator.core as rep
from pxr import Gf
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config="Example",
    translation=(0, 0, -0.04),
    orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
    #translation=(-0.937, 1.745, 0.8940),
    #orientation=Gf.Quatd(0.70711, 0.70711, 0, 0),  # Gf.Quatd is w,i,j,k
)

hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
writer = rep.writers.get("RtxRadar" + "DebugDrawPointCloud")
writer.attach([hydra_texture])
"""

"""
# Callstack of crash

__pthread_kill_implementation (@pthread_kill@@GLIBC_2.34:81)
__pthread_kill_internal (@pthread_kill@@GLIBC_2.34:59)
__GI___pthread_kill (@pthread_kill@@GLIBC_2.34:59)
__GI_raise (@raise:10)
__GI_abort (@abort:46)
___lldb_unnamed_symbol7245 (@___lldb_unnamed_symbol7245:27)
___lldb_unnamed_symbol7659 (@___lldb_unnamed_symbol7659:8)
std::terminate() (@7892c0aae277..7892c0aae2f7:3)
__cxa_throw (@7892c0aae4d8..7892c0aae550:3)
thrust::cuda_cub::throw_on_error(cudaError, char const*) (@thrust::cuda_cub::throw_on_error(cudaError, char const*):31)
float* thrust::cuda_cub::gather<thrust::cuda_cub::execute_on_stream, int*, float*, float*>(thrust::cuda_cub::execution_policy<thrust::cuda_cub::execute_on_stream>&, int*, int*, float*, float*) (@float* thrust::cuda_cub::gather<thrust::cuda_cub::execute_on_stream, int*, float*, float*>(thrust::cuda_cub::execution_policy<thrust::cuda_cub::execute_on_stream>&, int*, int*, float*, float*):133)
float* thrust::gather<thrust::cuda_cub::execute_on_stream, int*, float*, float*>(thrust::detail::execution_policy_base<thrust::cuda_cub::execute_on_stream> const&, int*, int*, float*, float*) (@float* thrust::gather<thrust::cuda_cub::execute_on_stream, int*, float*, float*>(thrust::detail::execution_policy_base<thrust::cuda_cub::execute_on_stream> const&, int*, int*, float*, float*):24)
omni::sensors::nv::radar::sortPointsByDistance_CUDA(omni::sensors::nv::radar::ProcessingContext&, omni::sensors::nv::radar::SortBuffer*, CUstream_st*, int) (@omni::sensors::nv::radar::sortPointsByDistance_CUDA(omni::sensors::nv::radar::ProcessingContext&, omni::sensors::nv::radar::SortBuffer*, CUstream_st*, int):319)
omni::sensors::nv::radar::processResultsCuda(omni::sensors::wpm::Config*, omni::sensors::wpm::TraceResult*, omni::sensors::nv::radar::ProcessingContext&, CUstream_st*, float*, unsigned char, bool, omni::sensors::nv::radar::SortBuffer*) (@omni::sensors::nv::radar::processResultsCuda(omni::sensors::wpm::Config*, omni::sensors::wpm::TraceResult*, omni::sensors::nv::radar::ProcessingContext&, CUstream_st*, float*, unsigned char, bool, omni::sensors::nv::radar::SortBuffer*):203)
omni::sensors::nv::radar::WpmDmatApproxRadar::closeTrace(void*, unsigned long*, void*, unsigned long*) (@omni::sensors::nv::radar::WpmDmatApproxRadar::closeTrace(void*, unsigned long*, void*, unsigned long*):329)
___lldb_unnamed_symbol934 (@___lldb_unnamed_symbol934:15)
___lldb_unnamed_symbol9313 (@___lldb_unnamed_symbol9313:222)
___lldb_unnamed_symbol9293 (@___lldb_unnamed_symbol9293:102)
___lldb_unnamed_symbol9204 (@___lldb_unnamed_symbol9204:8)
___lldb_unnamed_symbol1176 (@___lldb_unnamed_symbol1176:194)
"""
