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

import carb
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.universal_robots import KinematicsSolver
from isaacsim.robot.manipulators.examples.universal_robots.tasks import FollowTarget

my_world = World(stage_units_in_meters=1.0)
my_task = FollowTarget(name="follow_target_task", attach_gripper=True, target_position=[0.2, 0.4, 0.4])
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("follow_target_task").get_params()
ur10_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_ur10 = my_world.scene.get_object(ur10_name)
my_controller = KinematicsSolver(my_ur10, attach_gripper=True)
articulation_controller = my_ur10.get_articulation_controller()
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
        actions, succ = my_controller.compute_inverse_kinematics(
            target_position=observations[target_name]["position"],
            target_orientation=observations[target_name]["orientation"],
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

simulation_app.close()
