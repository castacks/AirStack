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
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--device", type=str, default="cpu", help="simulation device, cpu or cuda")
parser.add_argument("--visual", type=bool, default=False, help="Render for debugging purposes")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
n_gpu = args.num_gpus
n_frames = args.num_frames
device = args.device
visual = args.visual

import numpy as np
import torch
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not visual, "max_gpu_count": n_gpu})

import asyncio
from functools import partial

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.physx as _physx
import omni.timeline
from isaacsim.core.api import PhysicsContext, World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage_async, update_stage_async
from isaacsim.core.utils.types import ArticulationActions
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robots_ur10",
    workflow_metadata={
        "metadata": [
            {"name": "num_robots", "data": n_robot},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
            {"name": "device", "data": device},
        ]
    },
    backend_type=args.backend_type,
)

# Something about this being in an array makes it work as a global variable inside the physics sub
timestep = [0]

observed_positions, observed_velocities = [], []
commanded_positions, commanded_velocities = [], []

# v_max is the maximum velocity that each joint will hit in its range of motion
v_max = torch.tensor([2.09, 2.09, 3.14, 3.14, 3.14, 3.14])

# T is the period of each sinusoid
T = torch.tensor([9.43, 9.43, 6.28, 6.28, 6.28, 6.28])

joint_indices = torch.arange(6)

robot_path = "/ur10"


def get_clipped_joint_ranges(articulation_view):

    limits = articulation_view.get_dof_limits()
    lower_limit = limits[..., 0]
    upper_limit = limits[..., 1]

    l = lower_limit.clone()
    u = upper_limit.clone()
    d = upper_limit - lower_limit
    mask = d > 2 * torch.pi

    if torch.any(mask):
        l[mask] = (upper_limit[mask] - lower_limit[mask]) / 2 + lower_limit[mask] - torch.pi
        u[mask] = (upper_limit[mask] - lower_limit[mask]) / 2 + lower_limit[mask] + torch.pi

    return l, u


def get_joint_commands(articulation_view, v_max, T, joint_indices):
    lower_joint_limits, upper_joint_limits = get_clipped_joint_ranges(articulation_view)

    lower_joint_limits = lower_joint_limits[:, joint_indices]
    upper_joint_limits = upper_joint_limits[:, joint_indices]

    p_0 = lower_joint_limits + (upper_joint_limits - lower_joint_limits) / 2

    position = lambda t: p_0 - v_max * T / torch.pi * torch.cos(torch.pi * t / T)
    velocity = lambda t: v_max * torch.sin(torch.pi * t / T)

    return position, velocity


def on_physics_step(articulation_view, position_commands, velocity_commands, step):
    if position_commands is None:
        return
    timestep[0] += step
    if timestep[0] > 5:
        return

    observed_positions.append(articulation_view.get_joint_positions(joint_indices=joint_indices))
    observed_velocities.append(articulation_view.get_joint_velocities(joint_indices=joint_indices))

    position_command = position_commands(timestep[0])
    velocity_command = velocity_commands(timestep[0])

    commanded_positions.append(position_command)
    commanded_velocities.append(velocity_command)

    action = ArticulationActions(position_command, velocity_command, joint_indices=joint_indices)
    articulation_view.apply_action(action)


benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

get_active_viewport().updates_enabled = visual

robot_usd_path = "omniverse://ov-isaac-dev.nvidia.com/Isaac/Robots/UR10/ur10.usd"

my_world = World(backend="torch", device=device)
PhysicsContext(physics_dt=1.0 / 60.0)
MAX_IN_LINE = 10
positions = torch.zeros((n_robot, 3))
for i in range(n_robot):
    robot_prim_path = "/Robots/Robot_" + str(i)
    # position the robot
    robot_position = torch.tensor([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
    positions[i, :] = robot_position
    stage_utils.add_reference_to_stage(robot_usd_path, robot_prim_path)


omni.kit.app.get_app().update()
my_world.scene.add_default_ground_plane(z_position=-1)

robot_view = Articulation("/Robots/Robot_*", positions=positions)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

robot_view.initialize()
omni.kit.app.get_app().update()

position_commands, velocity_commands = get_joint_commands(robot_view, v_max, T, joint_indices)
_physxIFace = _physx.acquire_physx_interface()
physx_subscription = _physxIFace.subscribe_physics_step_events(
    partial(on_physics_step, robot_view, position_commands, velocity_commands)
)

position_command = position_commands(0)
velocity_command = velocity_commands(0)

robot_view.set_joint_positions(position_command, joint_indices=joint_indices)

commanded_positions.append(position_command)
commanded_velocities.append(velocity_command)

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark")

for _ in range(0, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

physics_subscription = None

timeline.stop()
simulation_app.close()
