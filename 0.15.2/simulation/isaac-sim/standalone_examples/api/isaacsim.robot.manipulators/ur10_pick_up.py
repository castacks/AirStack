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
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
    )
)
ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.3, 0.3, 0.3]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([0, 0, 1]),
    )
)
my_world.scene.add_default_ground_plane()
ur10.gripper.set_default_state(opened=True)
my_world.reset()

my_controller = PickPlaceController(name="pick_place_controller", gripper=ur10.gripper, robot_articulation=ur10)
articulation_controller = ur10.get_articulation_controller()

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
        actions = my_controller.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([0.7, 0.7, 0.0515 / 2.0]),
            current_joint_positions=ur10.get_joint_positions(),
            end_effector_offset=np.array([0, 0, 0.02]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    if args.test is True:
        break


simulation_app.close()
