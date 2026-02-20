# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class Subscriber(Node):
    def __init__(self):
        super().__init__("tutorial_subscriber")

        # setting up the world with a cube
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()
        # add a cube in the world
        cube_path = "/cube"
        self.ros_world.scene.add(
            VisualCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 10]), size=0.2)
        )
        self._cube_position = np.array([0, 0, 0])

        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(Empty, "move_cube", self.move_cube_callback, 10)
        self.ros_world.reset()

    def move_cube_callback(self, data):
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        if self.ros_world.is_playing():
            self._cube_position = np.array([np.random.rand() * 0.40, np.random.rand() * 0.40, 0.10])

    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False
                # the actual setting the cube pose is done here
                self.ros_world.scene.get_object("cube_1").set_world_pose(self._cube_position)

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    subscriber = Subscriber()
    subscriber.run_simulation()
