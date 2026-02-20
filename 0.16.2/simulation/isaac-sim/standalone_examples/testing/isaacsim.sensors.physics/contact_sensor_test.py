# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.physics import _sensor
from pxr import Gf

timeline = omni.timeline.get_timeline_interface()
cs = _sensor.acquire_contact_sensor_interface()

world = World(stage_units_in_meters=1.0)

# add a cube in the world
cube_path = "/World/cube"
cube_1 = world.scene.add(DynamicCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 1.5]), size=1.0))
# Add a plane for cube to collide with
world.scene.add_default_ground_plane()

# Setup contact sensor on cube
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateContactSensor",
    path="/Contact_Sensor",
    parent=cube_path,
    min_threshold=0,
    max_threshold=100000000,
    color=Gf.Vec4f(1, 1, 1, 1),
    radius=-1,
    sensor_period=1.0 / 60.0,
    translation=Gf.Vec3d(0, 0, 0),
)

# start simulation
# We must do one full step with rendering before the sensor will work correctly.
world.step(render=True)
timeline.play()
world.step(render=False)

for frame in range(100):
    world.step(render=False)

print("cube pose", cube_1.get_world_pose())

# Get processed contact data
reading = cs.get_sensor_reading(cube_path + "/Contact_Sensor")

if not reading.is_valid:
    raise ValueError("No contact sensor readings")

if reading.is_valid:
    print(str(reading))


# Cleanup
timeline.stop()
simulation_app.close()
