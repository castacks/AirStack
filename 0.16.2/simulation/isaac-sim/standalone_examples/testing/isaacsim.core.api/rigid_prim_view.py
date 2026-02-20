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
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.cloner import Cloner
from isaacsim.core.prims import GeometryPrim, RigidPrim
from isaacsim.core.utils.torch.rotations import euler_angles_to_quats
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_cloner = Cloner()
my_world = World(stage_units_in_meters=1.0, backend="torch")
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

cube = DynamicCuboid(prim_path="/World/cube_0")
prim_paths = my_cloner.generate_paths("/World/cube", 3)
my_cloner.clone(cube.prim_path, prim_paths)

rigid_prim_view = RigidPrim(prim_paths_expr="/World/cube_[0-2]")

physics_material_1 = PhysicsMaterial(
    prim_path="/Physics_material_1", dynamic_friction=0.2, static_friction=0.2, restitution=0.0
)
physics_material_2 = PhysicsMaterial(
    prim_path="/Physics_material_2", dynamic_friction=0.2, static_friction=0.2, restitution=0.0
)
physics_material_3 = PhysicsMaterial(
    prim_path="/Physics_material_3", dynamic_friction=0.2, static_friction=0.2, restitution=0.0
)
geometry_prim_view = GeometryPrim(
    prim_paths_expr="/World/cube_[0-2]", collisions=torch.tensor([True, True, True], dtype=torch.bool)
)
geometry_prim_view.apply_physics_materials(physics_materials=[physics_material_1, physics_material_3], indices=[2, 0])
geometry_prim_view.set_contact_offsets(offsets=torch.tensor([0.3, 0.3, 0.3], dtype=torch.float32))
my_world.scene.add(rigid_prim_view)
my_world.reset()
new_positions = torch.tensor([[10.0, 10.0, 100], [40, 40, 100]], dtype=torch.float32)
new_orientations = euler_angles_to_quats(torch.tensor([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]], dtype=torch.float32))
linear_velocities = torch.tensor([[0, 0, -10.0], [0, 0, -10.0], [0, 0, -10.0]], dtype=torch.float32)
rigid_prim_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=[0, 1])
rigid_prim_view.set_linear_velocities(velocities=linear_velocities)
rigid_prim_view.set_local_poses(translations=new_positions, orientations=new_orientations, indices=[0, 1])
print(rigid_prim_view.get_world_poses())
rigid_prim_view.set_masses(torch.tensor([10.0, 10.0, 10.0], dtype=torch.float32))
print(rigid_prim_view.get_local_poses(indices=[0, 1]))
print(rigid_prim_view.get_linear_velocities())
print(rigid_prim_view.get_masses())
for i in range(10000):
    my_world.step(render=True)
    print(rigid_prim_view.get_linear_velocities())
simulation_app.close()
