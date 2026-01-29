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
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--max-in-line", type=int, default=10, help="Max number of robots in line")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_robots = args.num_robots
n_frames = args.num_frames
n_gpus = args.num_gpus
max_line = args.max_in_line

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpus})

import carb
import numpy as np
import omni.kit.test
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_o3dyn_robot",
    workflow_metadata={
        "metadata": [
            {"name": "num_robots", "data": n_robots},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

robot_path = "/Isaac/Robots/O3dyn/o3dyn.usd"
scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
PhysicsContext(physics_dt=1.0 / 60.0)
set_camera_view(eye=[-6, -15.5, 6.5], target=[-6, 10.5, -1], camera_prim_path="/OmniverseKit_Persp")

# Configure robots
robots = []
for i in range(n_robots):
    robot_prim_path = f"/Robots/Robot_{i}"
    robot_usd_path = benchmark.assets_root_path + robot_path

    # Arrange robot positions
    robot_pos = np.array([-3 * (i % max_line) + 3, -3 * np.floor(i / max_line), 0.1])
    current_robot = WheeledRobot(
        prim_path=robot_prim_path,
        wheel_dof_names=["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"],
        create_robot=True,
        usd_path=robot_usd_path,
        position=robot_pos,
    )

    omni.kit.app.get_app().update()
    robots.append(current_robot)

viewport = get_active_viewport()
viewport.set_texture_resolution([1280, 720])
omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
# NOTE: PhysX Simulation Context error if this update() is removed
omni.kit.app.get_app().update()

# Start robot movement - rotation in place
for robot in robots:
    robot.initialize()
    robot.apply_wheel_actions(
        ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=5 * np.array([1, -1, 1, -1]))
    )
omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.set_phase("benchmark")

for _ in range(0, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()

simulation_app.close()
