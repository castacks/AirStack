# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import time

from isaacsim import SimulationApp

# Example ROS2 bridge sample showing rclpy and rosclock interaction
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": True})
import carb
import omni
import omni.graph.core as og
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

simulation_app.update()
# Note that this is not the system level rclpy, but one compiled for omniverse
import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()
clock_topic = "sim_time"
manual_clock_topic = "manual_time"

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("PublishManualClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                # Connecting execution of OnPlaybackTick node to PublishClock to automatically publish each frame
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                # Connecting execution of OnImpulseEvent node to PublishManualClock so it will only publish when an impulse event is triggered
                ("OnImpulseEvent.outputs:execOut", "PublishManualClock.inputs:execIn"),
                # Connecting simulationTime data of ReadSimTime to the clock publisher nodes
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishManualClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Assigning topic names to clock publishers
                ("PublishClock.inputs:topicName", clock_topic),
                ("PublishManualClock.inputs:topicName", manual_clock_topic),
            ],
        },
    )
except Exception as e:
    print(e)


simulation_app.update()
simulation_app.update()


# Define ROS2 callbacks
def sim_clock_callback(data):
    print("sim time:", data.clock)


def manual_clock_callback(data):
    print("manual stepped sim time:", data.clock)


# Create rclpy ndoe
node = rclpy.create_node("isaac_sim_clock")

# create subscribers
sim_clock_sub = node.create_subscription(Clock, clock_topic, sim_clock_callback, 1)
manual_clock_sub = node.create_subscription(Clock, manual_clock_topic, manual_clock_callback, 1)

time.sleep(1.0)
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

# perform a fixed number of steps with fixed step size
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        simulation_context.render()  # This updates rendering/app loop which calls the sim clock

    simulation_context.step(render=False)  # runs with a non-realtime clock
    rclpy.spin_once(node, timeout_sec=0.0)  # Spin node once
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# perform a fixed number of steps with realtime clock
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

    simulation_app.update()  # runs with a realtime clock
    rclpy.spin_once(node, timeout_sec=0.0)  # Spin node once
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# shutdown
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
