# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})
import omni.physx as _physx
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka", name="my_franka_1"))


def step_callback_1(step_size):
    b = articulated_system_1.get_joint_velocities()


physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(step_callback_1)
my_world.reset()
for j in range(10):
    for i in range(5):
        my_world.step(render=False)
simulation_app.close()
