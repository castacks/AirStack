# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
from pprint import pprint

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import cuboid
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)
from isaacsim.robot_motion.motion_generation.lula import RmpFlow
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument(
    "-v",
    "--verbose",
    action="store_true",
    default=True,
    help="Print useful runtime information such as the list of supported robots",
)
parser.add_argument(
    "--robot-name",
    type=str,
    default="Cobotta_Pro_900",
    help="Key to use to access RMPflow config files for a specific robot.",
)
parser.add_argument(
    "--usd-path",
    type=str,
    default="/Isaac/Robots/Denso/cobotta_pro_900.usd",
    help="Path to supported robot on Nucleus Server",
)
parser.add_argument("--add-orientation-target", action="store_true", default=False, help="Add orientation target")
args, unknown = parser.parse_known_args()

robot_name = args.robot_name
usd_path = get_assets_root_path() + args.usd_path
prim_path = "/my_robot"

add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

light_prim = create_prim("/DistantLight", "DistantLight")
light_prim.GetAttribute("inputs:intensity").Set(5000)

my_world = World(stage_units_in_meters=1.0)

robot = my_world.scene.add(Robot(prim_path=prim_path, name=robot_name))

if args.verbose:
    print("Names of supported robots with provided RMPflow config")
    print("\t", list(get_supported_robot_policy_pairs().keys()))
    print()

# The load_supported_motion_policy_config() function is currently the simplest way to load supported robots.
# In the future, Isaac Sim will provide a centralized registry of robots with Lula robot description files
# and RMP configuration files stored alongside the robot USD.
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

if args.verbose:
    print(
        f"Successfully referenced RMPflow config for {robot_name}.  Using the following parameters to initialize RmpFlow class:"
    )
    pprint(rmp_config)
    print()

# Initialize an RmpFlow object
rmpflow = RmpFlow(**rmp_config)

physics_dt = 1 / 60.0
articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, physics_dt)

articulation_controller = robot.get_articulation_controller()

# Make a target to follow
target_cube = cuboid.VisualCuboid(
    "/World/target", position=np.array([0.5, 0, 0.5]), color=np.array([1.0, 0, 0]), size=0.1
)

# Make an obstacle to avoid
obstacle = cuboid.VisualCuboid(
    "/World/obstacle", position=np.array([0.8, 0, 0.5]), color=np.array([0, 1.0, 0]), size=0.1
)
rmpflow.add_obstacle(obstacle)

my_world.reset()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        # Set rmpflow target to be the current position of the target cube.
        if args.add_orientation_target:
            target_orientation = target_cube.get_world_pose()[1]
        else:
            target_orientation = None

        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0], target_orientation=target_orientation
        )

        # Query the current obstacle position
        rmpflow.update_world()

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)

simulation_app.close()
