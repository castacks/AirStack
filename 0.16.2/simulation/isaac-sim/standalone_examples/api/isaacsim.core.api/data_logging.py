# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka_1", name="my_franka_1"))


my_world.reset()
data_logger = my_world.get_data_logger()


def frame_logging_func(tasks, scene):
    return {
        "joint_positions": scene.get_object("my_franka_1").get_joint_positions().tolist(),
        "applied_joint_positions": scene.get_object("my_franka_1").get_applied_action().joint_positions.tolist(),
    }


data_logger.add_data_frame_logging_func(frame_logging_func)
data_logger.start()
for j in range(100):
    my_world.step(render=True)

data_logger.save(log_path="./isaac_sim_data.json")
data_logger.reset()

data_logger.load(log_path="./isaac_sim_data.json")
print(data_logger.get_data_frame(data_frame_index=2))
simulation_app.close()
