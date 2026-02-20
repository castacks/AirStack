# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Carter Stereo Example")
parser.add_argument("--test", action="store_true")
args, unknown = parser.parse_known_args()

# Example ROS2 bridge sample showing manual control over messages
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
import carb
import omni
import omni.graph.core as og
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

# Locate assets root folder to load sample
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()

usd_path = assets_root_path + "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
omni.usd.get_context().open_stage(usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from isaacsim.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

ros_cameras_graph_path = "/World/Nova_Carter_ROS/front_hawk"

# Enabling rgb image publishers for left camera. Cameras will automatically publish images each frame
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/left_camera_render_product.inputs:enabled"), True)

# Enabling rgb image publishers for right camera. Cameras will automatically publish images each frame
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/right_camera_render_product.inputs:enabled"), True)

simulation_context.play()
simulation_context.step()


# Simulate for one second to warm up sim and let everything settle
for frame in range(60):
    simulation_context.step()


# Create a ROS publisher to publish message to spin robot in place

# If system level rclpy is sourced in bashrc or terminal, it is imported otherwise backup rclpy libraries shipped with Isaac sim is used
import rclpy

rclpy.init()

from geometry_msgs.msg import Twist

node = rclpy.create_node("carter_stereo")
publisher = node.create_publisher(Twist, "cmd_vel", 10)

frame = 0
while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)

    # Publish the ROS Twist message every 2 frames
    if frame % 2 == 0:
        message = Twist()
        message.angular.z = 0.5  # spin in place
        publisher.publish(message)

    if args.test and frame > 120:
        break
    frame = frame + 1
node.destroy_node()
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
