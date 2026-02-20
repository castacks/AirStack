# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import time

from isaacsim import SimulationApp

# Simple example showing how to fix frame rate to (roughly) a constant value (with respect to wall-clock)
# Note frame rate cannot be set artificially higher than what the sim will run at on the current hardware,
# but it can be kept artificially lower for (eg.) synchronization with an external service.

DESIRED_FRAME_RATE = 10.0  # frames per second
frame_period_s = 1.0 / DESIRED_FRAME_RATE

simulation_app = SimulationApp({"headless": True})

import carb
import omni

# Callback to measure app update time as precisely as possible
last_frametime_timestamp_ns = 0.0
app_update_time_s = 0.0


def update_event_callback(event: carb.events.IEvent):
    timestamp_ns = time.perf_counter_ns()
    app_update_time_s = round((timestamp_ns - last_frametime_timestamp_ns) / 1e9, 9)
    last_frametime_timestamp_ns = timestamp_ns


omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(update_event_callback)

while simulation_app.is_running():
    # Measure duration of single app update
    simulation_app.update()
    # Sleep for the duration of the fixed frame
    sleep_duration_s = frame_period_s - app_update_time_s
    if sleep_duration_s <= 0.0:
        carb.log_warn(f"simulation_app.update() took {app_update_time_s} s >= fixed period {frame_period_s} s.")
    else:
        time.sleep(sleep_duration_s)
    instantaneous_fps = 1.0 / max(frame_period_s, app_update_time_s)
    carb.log_warn(f"FPS is {instantaneous_fps}")

simulation_app.close()
