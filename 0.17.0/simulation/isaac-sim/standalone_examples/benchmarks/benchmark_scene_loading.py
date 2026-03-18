# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--duration", type=int, default=None, help="Optional - duration in minutes (wall-clock time), overrides frame count"
)
parser.add_argument("--env-url", default=None, required=True, help="Path to the environment url - required")
parser.add_argument(
    "--camera-position", type=float, nargs=3, default=None, help="Set perspective position <x> <y> <z> - optional"
)
parser.add_argument(
    "--camera-target", type=float, nargs=3, default=None, help="Set perspective target <x> <y> <z> - optional"
)
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_frames = args.num_frames
duration = args.duration
env_url = args.env_url
cam_pos = args.camera_position
cam_target = args.camera_target

# Both cam_pos and cam_target should be specified if used
if (cam_pos and not cam_target) or (cam_target and not cam_pos):
    parser.error("Both --camera-position and --camera-target must be specified together.")

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni
import omni.kit.test
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

enable_extension("isaacsim.ros2.bridge")
omni.kit.app.get_app().update()

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_scene_loading",
    workflow_metadata={"metadata": [{"name": "env_url", "data": env_url}, {"name": "duration", "data": duration}]},
    backend_type=args.backend_type,
)

# Track scene loading time
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)
benchmark.fully_load_stage(benchmark.assets_root_path + env_url)
benchmark.store_measurements()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

benchmark.set_phase("benchmark")

if cam_pos is not None:
    set_camera_view(eye=cam_pos, target=cam_target, camera_prim_path="/OmniverseKit_Persp")

if duration is not None:
    start_time = time.time()
    while time.time() - start_time < (duration * 60):
        omni.kit.app.get_app().update()

else:
    for _ in range(1, n_frames):
        omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
