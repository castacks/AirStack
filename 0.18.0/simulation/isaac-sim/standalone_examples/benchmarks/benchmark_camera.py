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
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument(
    "--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution as [width, height] px"
)
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_camera = args.num_cameras
resolution = args.resolution
n_gpu = args.num_gpus
n_frames = args.num_frames

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpu})

TEST_NUM_APP_UPDATES = 60 * 10

import carb
import omni
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_camera",
    workflow_metadata={
        "metadata": [
            {"name": "num_cameras", "data": n_camera},
            {"name": "width", "data": resolution[0]},
            {"name": "height", "data": resolution[1]},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)


scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
cameras = []

for i in range(n_camera):
    render_product_path = None
    if i == 0:
        viewport_api = get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()
    cameras.append(
        Camera(
            prim_path="/Cameras/Camera_" + str(i),
            position=np.array([-8, 13, 2.0]),
            resolution=resolution,
            orientation=euler_angles_to_quat([90, 0, 90 + i * 360 / n_camera], degrees=True),
            render_product_path=render_product_path,
        )
    )

    omni.kit.app.get_app().update()
    cameras[i].initialize()

# make sure scene is loaded in all viewports
while is_stage_loading():
    print("asset still loading, waiting to finish")
    omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark")

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
