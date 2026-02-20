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

parser = argparse.ArgumentParser(description="ROS Clock Example")
parser.add_argument("--test", action="store_true")
args, unknown = parser.parse_known_args()


# Example ROS bridge sample showing rospy and rosclock interaction
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": True})
import carb
import omni
import omni.graph.core as og
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension

if args.test:
    from isaacsim.ros1.bridge.scripts.roscore import Roscore
    from isaacsim.ros1.bridge.tests.common import wait_for_rosmaster

    roscore = Roscore()
    wait_for_rosmaster()

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()
import rospy

# Note that this is not the system level rospy, but one compiled for omniverse
from rosgraph_msgs.msg import Clock

clock_topic = "sim_time"
manual_clock_topic = "manual_time"

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("PublishManualClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                # Connecting execution of OnPlaybackTick node to PublishClock  to automatically publish each frame
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


# Define ROS callbacks
def sim_clock_callback(data):
    print("sim time:", data.clock.to_sec())


def manual_clock_callback(data):
    print("manual stepped sim time:", data.clock.to_sec())


# Create rospy ndoe
rospy.init_node("isaac_sim_clock", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
# create subscribers
sim_clock_sub = rospy.Subscriber(clock_topic, Clock, sim_clock_callback)
manual_clock_sub = rospy.Subscriber(manual_clock_topic, Clock, manual_clock_callback)
time.sleep(1.0)
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
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# perform a fixed number of steps with realtime clock
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

    simulation_app.update()  # runs with a realtime clock
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# cleanup and shutdown
sim_clock_sub.unregister()
manual_clock_sub.unregister()
simulation_context.stop()

if args.test:
    roscore = None

simulation_app.close()
