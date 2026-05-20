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

import torch
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import XFormPrim

my_world = World(stage_units_in_meters=1.0, device="cuda:0", backend="torch")
cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_1",
        position=torch.tensor([0, 0, 1.0]),
        scale=torch.tensor([0.6, 0.5, 0.2]),
        size=1.0,
        color=torch.tensor([255, 0, 0]),
    )
)
xfrom_cube = XFormPrim("/new_cube_2")
my_world.scene.add_default_ground_plane()
my_world.reset()
for i in range(500):
    my_world.step(render=False)
my_world.render()
if not (xfrom_cube.get_world_poses()[0][:, -1].item() < 10e-02):
    raise (ValueError(f"PhysX status is not updated in the rendering call"))

simulation_app.close()
