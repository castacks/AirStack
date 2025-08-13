# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

simulation_context = SimulationContext(stage_units_in_meters=1.0)
add_reference_to_stage(asset_path, "/Franka")
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()


def step_callback(step_size):
    print("simulate with step: ", step_size)
    return


def render_callback(event):
    print("update app with step: ", event.payload["dt"])


simulation_context.add_physics_callback("physics_callback", step_callback)
simulation_context.add_render_callback("render_callback", render_callback)
simulation_context.stop()
simulation_context.play()

print("step physics once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=False)

print("step physics & rendering once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=True)

print("step physics & rendering once with a step size of 1/60 second")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)

print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)

print("step physics once at 600Hz without rendering")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=False)

print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
for step in range(10):
    simulation_context.step(render=False)
simulation_context.render()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.render()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=0.0, rendering_dt=1.0 / 60)
simulation_context.step(render=True)

print("step physics once 1/60s per step and rendering 10 times at 1.0/600s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
for step in range(10):
    simulation_context.step(render=True)

print("step physics once 1/60s per step and rendering once at 1.0/600s by explicitly calling step and render")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
simulation_context.step(render=False)
simulation_context.render()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=False)
simulation_context.render()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=True)

print("render a new frame with simulation stopped, editor timeline does not move forward")
simulation_context.stop()
simulation_context.render()

print("cleanup and exit")
simulation_context.stop()
simulation_app.close()
