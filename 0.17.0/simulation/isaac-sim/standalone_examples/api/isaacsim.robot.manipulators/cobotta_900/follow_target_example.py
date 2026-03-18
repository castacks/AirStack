# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
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

import numpy as np
from controllers.rmpflow import RMPFlowController
from isaacsim.core.api import World
from tasks.follow_target import FollowTarget

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
# Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="denso_follow_target", target_position=np.array([0.5, 0, 0.5]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("denso_follow_target").get_params()
target_name = task_params["target_name"]["value"]
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)

# initialize the controller
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_denso)

# make RmpFlow aware of the ground plane
ground_plane = my_world.scene.get_object(name="default_ground_plane")
my_controller.add_obstacle(ground_plane)

articulation_controller = my_denso.get_articulation_controller()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
    if args.test is True:
        break
simulation_app.close()
