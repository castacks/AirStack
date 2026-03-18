# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

asset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/orientation_bug.usd")
my_world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=asset_path, prim_path="/World")
articulated = Articulation("/World/microwave")
my_world.scene.add(articulated)
my_world.reset()
for i in range(3):
    my_world.step(render=True)
if not (np.isclose(articulated.get_world_poses()[1], [-0.50, -0.49, 0.49, 0.50], atol=1e-02)).all():
    raise (
        ValueError(
            f"Articulation is not using the correct default state due to a mismatch in the ArticulationRoot representation"
        )
    )
simulation_app.close()
