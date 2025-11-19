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

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.universal_robots.controllers import StackingController
from isaacsim.robot.manipulators.examples.universal_robots.tasks import Stacking

my_world = World(stage_units_in_meters=1.0)
my_task = Stacking()
my_world.add_task(my_task)
my_world.reset()
robot_name = my_task.get_params()["robot_name"]["value"]
my_ur10 = my_world.scene.get_object(robot_name)
my_controller = StackingController(
    name="stacking_controller",
    gripper=my_ur10.gripper,
    robot_articulation=my_ur10,
    picking_order_cube_names=my_task.get_cube_names(),
    robot_observation_name=robot_name,
)
articulation_controller = my_ur10.get_articulation_controller()

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(observations=observations, end_effector_offset=np.array([0.0, 0.0, 0.02]))
        articulation_controller.apply_action(actions)

simulation_app.close()
