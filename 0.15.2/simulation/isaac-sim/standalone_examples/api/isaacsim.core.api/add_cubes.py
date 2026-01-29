# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid

my_world = World(stage_units_in_meters=1.0)

cube_1 = my_world.scene.add(
    VisualCuboid(
        prim_path="/new_cube_1",
        name="visual_cube",
        position=np.array([0, 0, 0.5]),
        size=0.3,
        color=np.array([255, 255, 255]),
    )
)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_1",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)

cube_3 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_3",
        name="cube_2",
        position=np.array([0, 0, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )
)

my_world.scene.add_default_ground_plane()

for i in range(5):
    my_world.reset()
    for i in range(500):
        my_world.step(render=True)
        print(cube_2.get_angular_velocity())
        print(cube_2.get_world_pose())

simulation_app.close()
