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

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.franka import Franka

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
my_franka = my_world.scene.add(Franka(prim_path="/World/Franka", name="my_franka"))
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        i += 1
        gripper_positions = my_franka.gripper.get_joint_positions()
        if i < 500:
            my_franka.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - (0.005), gripper_positions[1] - (0.005)])
            )
        if i > 500:
            my_franka.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + (0.005), gripper_positions[1] + (0.005)])
            )
        if i == 1000:
            i = 0
    if args.test is True:
        break


simulation_app.close()
