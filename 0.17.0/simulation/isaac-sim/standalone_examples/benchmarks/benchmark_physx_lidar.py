# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-sensors", type=int, default=1, help="Number of sensors")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_sensor = args.num_sensors
n_frames = args.num_frames

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.kit.test
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from pxr import Gf, UsdGeom

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark


# Create PhysX Lidar from params
def add_physx_lidar(prim_path, translation=Gf.Vec3f(0, 0, 0), orientation=Gf.Vec4f(0, 0, 0, 0)):
    _, lidar = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=prim_path,
        parent=None,
        min_range=0.4,
        max_range=100.0,
        draw_points=True,
        draw_lines=True,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=4.0,
        rotation_rate=0.0,
        high_lod=False,
        yaw_offset=0.0,
    )
    lidar_prim = lidar.GetPrim()

    if "xformOp:translate" not in lidar_prim.GetPropertyNames():
        UsdGeom.Xformable(lidar_prim).AddTranslateOp()
    if "xformOp:orient" not in lidar_prim.GetPropertyNames():
        UsdGeom.Xformable(lidar_prim).AddOrientOp()

    lidar_prim.GetAttribute("xformOp:translate").Set(translation)
    lidar_prim.GetAttribute("xformOp:orient").Set(orientation)


# ----------------------------------------------------------------------
# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_physx_lidar",
    workflow_metadata={"metadata": [{"name": "num_lidars", "data": n_sensor}]},
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
PhysicsContext(physics_dt=1.0 / 60.0)

for i in range(n_sensor):
    lidar_path = f"/World/PhysxLidar_{i}"
    sensor_translation = Gf.Vec3f([-8, 13, 2.0])  # Positions set for full_warehouse.usd
    q = euler_angles_to_quat([90, 0, 90 + i * 360 / n_sensor], degrees=True)
    sensor_orientation = Gf.Quatf(q[0], q[1], q[2], q[3])
    add_physx_lidar(prim_path=lidar_path, translation=sensor_translation, orientation=sensor_orientation)

    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.set_phase("benchmark")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(0, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()

simulation_app.close()
