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
import random
import sys

import carb
import numpy as np
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.omni_glass import OmniGlass
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0, backend="torch", device="cuda:0")
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
# define_prim(prim_path="/World/Frame_1")
# define_prim(prim_path="/World/Frame_2")
# define_prim(prim_path="/World/Frame_3")
# define_prim(prim_path="/World/Frame_1/Target")
# define_prim(prim_path="/World/Frame_2/Target")
# define_prim(prim_path="/World/Frame_3/Target")
new_positions = torch.tensor([[10.0, 10.0, 0], [100.0, 100.0, 0]], dtype=torch.float32) / 100.0
new_orientations = torch.tensor(
    euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]])), dtype=torch.float32
)
frankas_view = Articulation(prim_paths_expr="/World/Franka_[1-2]", name="frankas_view")
my_world.scene.add(frankas_view)
glass_1 = OmniGlass(
    prim_path=f"/World/franka_glass_material_1",
    ior=1.25,
    depth=0.001,
    thin_walled=False,
    color=np.array([random.random(), random.random(), random.random()]),
)

glass_2 = OmniGlass(
    prim_path=f"/World/franka_glass_material_2",
    ior=1.25,
    depth=0.001,
    thin_walled=False,
    color=np.array([random.random(), random.random(), random.random()]),
)

my_world.reset()
frankas_view.set_world_poses(positions=new_positions)

frankas_view.apply_visual_materials(visual_materials=[glass_1, glass_2], indices=[1, 0])
frankas_view.set_gains(
    kps=torch.tensor(
        [
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
        ]
    )
)
frankas_view.switch_control_mode(mode="velocity")
print("Gains here", frankas_view.get_gains())
frankas_view.set_effort_modes("force")
print(frankas_view.get_effort_modes())
print(frankas_view.get_max_efforts())
my_world.reset()
frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations)
frankas_view.set_joint_positions(
    torch.tensor([[1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5], [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]])
)
for i in range(10000):
    my_world.step(render=True)
    if i % 100 == 0:
        frankas_view.apply_action(ArticulationAction(joint_positions=torch.randn(2, 9)))
simulation_app.close()
