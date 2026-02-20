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

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux

# Add Ground Plane
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

# Add Light Source
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# Add Visual Cubes
visual_cube = VisualCuboid(
    prim_path="/visual_cube",
    name="visual_cube",
    position=np.array([0, 0.5, 1.0]),
    size=0.3,
    color=np.array([255, 255, 0]),
)

visual_cube_static = VisualCuboid(
    prim_path="/visual_cube_static",
    name="visual_cube_static",
    position=np.array([0.5, 0, 0.5]),
    size=0.3,
    color=np.array([0, 255, 0]),
)

# Add Physics Cubes
dynamic_cube = DynamicCuboid(
    prim_path="/dynamic_cube",
    name="dynamic_cube",
    position=np.array([0, -0.5, 1.5]),
    size=0.3,
    color=np.array([0, 255, 255]),
)

# start a world to step simulator
my_world = World(stage_units_in_meters=1.0)

# start the simulator
for i in range(3):
    my_world.reset()
    print("simulator running", i)
    if i == 1:
        print("Adding Physics Properties to the Visual Cube")
        from isaacsim.core.prims import RigidPrim

        RigidPrim("/visual_cube")

    if i == 2:
        print("Adding Collision Properties to the Visual Cube")
        from isaacsim.core.prims import GeometryPrim

        prim = GeometryPrim("/visual_cube")
        prim.apply_collision_apis()

    for j in range(100):
        my_world.step(render=True)  # stepping through the simulation

# shutdown the simulator automatically
simulation_app.close()
