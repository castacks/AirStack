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
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument(
    "--lidar-type", type=str, default="Rotary", choices=["Rotary", "Solid_State"], help="Type of lidar to create"
)

parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)


args, unknown = parser.parse_known_args()

n_sensor = args.num_sensors
n_gpu = args.num_gpus
n_frames = args.num_frames

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpu})

import carb
import omni
import omni.replicator.core as rep
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import delete_prim
from pxr import Gf

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_rtx_lidar",
    workflow_metadata={
        "metadata": [
            {"name": "num_3d_lidars", "data": n_sensor},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
timeline = omni.timeline.get_timeline_interface()
hydra_textures = []
writers = []
sensors = []
lidar_type = args.lidar_type
for i in range(n_sensor):
    lidar_path = "/World/Rtx" + lidar_type + "Lidar_" + str(i)
    sensor_translation = Gf.Vec3f([-8, 13 + i * 2.0, 2.0])  # these positions are used for full_warehouse.usd
    # make sure to test rotary and solid state together.
    lidar_config = "Example_" + lidar_type
    print("Lidar Config:", lidar_config)

    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_path,
        parent=None,
        config=lidar_config,
        translation=sensor_translation,
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
    )
    sensors.append(sensor)
    hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
    hydra_textures.append(hydra_texture)
    # Create the post process graph that publishes the render var
    writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    writer.initialize()
    writer.attach([hydra_texture])
    writers.append(writer)

    omni.kit.app.get_app().update()

benchmark.store_measurements()

benchmark.set_phase("benchmark")
timeline.play()

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()

for writer in writers:
    writer.detach()
omni.kit.app.get_app().update()

for sensor in sensors:
    delete_prim(sensor.GetPath())
omni.kit.app.get_app().update()

for texture in hydra_textures:
    omni.kit.app.get_app().update()
    texture.destroy()
    texture = None
omni.kit.app.get_app().update()

simulation_app.close()
