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

import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationActions
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.sensors.physics import IMUSensor
from isaacsim.storage.native import get_assets_root_path

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Carter")

my_carter = my_world.scene.add(Articulation("/World/Carter", name="my_carter", positions=np.array([[0, 0.0, 0.5]])))
wheel_dof_names = ["joint_wheel_left", "joint_wheel_right"]

my_controller = DifferentialController(name="simple_control", wheel_radius=0.04295, wheel_base=0.4132)


imu_sensor = my_world.scene.add(
    IMUSensor(
        prim_path="/World/Carter/caster_wheel_left/imu_sensor",
        name="imu",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
)
my_world.reset()
i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        wheel_dof_indices = [my_carter.get_dof_index(wheel_dof_names[i]) for i in range(len(wheel_dof_names))]
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        print(imu_sensor.get_current_frame())
        actions = ArticulationActions()
        if i >= 0 and i < 1000:
            # forward
            # convert from ArticulationAction to ArticulationActions
            actions.joint_velocities = np.expand_dims(my_controller.forward(command=[0.05, 0]).joint_velocities, axis=0)

        elif i >= 1000 and i < 1265:
            # rotate
            # convert from ArticulationAction to ArticulationActions
            actions.joint_velocities = np.expand_dims(
                my_controller.forward(command=[0.0, np.pi / 12]).joint_velocities, axis=0
            )
        elif i >= 1265 and i < 2000:
            # forward
            # convert from ArticulationAction to ArticulationActions
            actions.joint_velocities = np.expand_dims(my_controller.forward(command=[0.05, 0]).joint_velocities, axis=0)
        elif i == 2000:
            i = 0
        i += 1
        joint_actions = ArticulationActions()
        joint_actions.joint_velocities = np.zeros([1, my_carter.num_dof])
        if actions.joint_velocities is not None:
            for j in range(len(wheel_dof_indices)):
                joint_actions.joint_velocities[0, wheel_dof_indices[j]] = actions.joint_velocities[0, j]

        my_carter.apply_action(joint_actions)

simulation_app.close()
