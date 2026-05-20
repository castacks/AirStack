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

import isaacsim.core.utils.numpy.rotations as rot_utils
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera

my_world = World(stage_units_in_meters=1.0)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_1",
        position=np.array([5.0, 3, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)

cube_3 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_3",
        name="cube_2",
        position=np.array([-5, 1, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )
)

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

my_world.scene.add_default_ground_plane()
my_world.reset()
camera.initialize()

i = 0
camera.add_motion_vectors_to_frame()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    print(camera.get_current_frame())
    if i == 100:
        points_2d = camera.get_image_coords_from_world_points(
            np.array([cube_3.get_world_pose()[0], cube_2.get_world_pose()[0]])
        )
        points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        print(points_2d)
        print(points_3d)
        imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        if matplotlib.get_backend() in ["TkAgg", "nbAgg"]:
            plt.draw()
            plt.pause(0.01)
        print(camera.get_current_frame()["motion_vectors"])
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
    i += 1


simulation_app.close()
