import numpy as np

import carb

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from omni.isaac.core.world import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid


def customize_world(world: World, pg: PegasusInterface):
    carb.log_info("Adding default ground plane to the scene.")
    world.scene.add_default_ground_plane()

    # see isaac sim stand_alone_examples add_cubes.py
    cube_1 = world.scene.add(
        VisualCuboid(
            prim_path="/World/new_cube_1",
            name="visual_cube",
            position=np.array([2.0, 0.5, 0.5]),
            size=1.0,
            color=np.array([255, 255, 255]),
        )
    )
