# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import unittest

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
if not math.isclose(simulation_context.get_physics_dt(), 1.0 / 60.0):
    raise ValueError()
if not math.isclose(simulation_context.get_rendering_dt(), 1.0 / 60.0):
    raise ValueError()
simulation_context.clear_instance()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
if not math.isclose(simulation_context.get_physics_dt(), 1.0 / 60.0):
    raise ValueError()
if not math.isclose(simulation_context.get_rendering_dt(), 1.0 / 60.0):
    raise ValueError()
add_reference_to_stage(asset_path, "/Franka")
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()


class TimeStepTester(unittest.TestCase):
    def __init__(self):
        self.physics_steps = 0
        self.render_steps = 0
        self.physics_dt = 1.0 / 60.0
        self.render_dt = 1.0 / 60.0

    def step_callback(self, step_size):
        print("simulate with step: ", step_size)
        self.physics_steps = self.physics_steps + 1
        self.physics_dt = step_size

    def render_callback(self, event):
        print("update app with step: ", event.payload["dt"])
        self.render_steps = self.render_steps + 1
        self.render_dt = event.payload["dt"]

    def check_steps(self, physics_steps, render_steps):
        if physics_steps != self.physics_steps:
            self.assertAlmostEqual(physics_steps, self.physics_steps)
        if render_steps != self.render_steps:
            self.assertAlmostEqual(render_steps, self.render_steps)

    def check_dt(self, physics_dt, render_dt):
        if physics_dt != self.physics_dt:
            self.assertAlmostEqual(physics_dt, self.physics_dt)
        if render_dt != self.render_dt:
            self.assertAlmostEqual(render_dt, self.render_dt)

    def reset_values(self):
        self.physics_steps = 0
        self.render_steps = 0
        self.physics_dt = 1.0 / 60.0
        self.render_dt = 1.0 / 60.0


tester = TimeStepTester()
simulation_context.add_physics_callback("physics_callback", tester.step_callback)
simulation_context.add_render_callback("render_callback", tester.render_callback)
simulation_context.stop()
simulation_context.play()
tester.reset_values()

print("step physics once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=False)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 0)
tester.reset_values()

print("step physics & rendering once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 1)
tester.reset_values()
print("step physics & rendering once with a step size of 1/60 second")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 1)
tester.reset_values()
print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(10, 1)
tester.reset_values()

print("step physics once at 600Hz without rendering")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=False)
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(1, 0)
tester.reset_values()

print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
for step in range(10):
    simulation_context.step(render=False)
simulation_context.render()
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(10, 1)
tester.reset_values()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.render()
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(0, 1)
tester.reset_values()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=0.0, rendering_dt=1.0 / 60)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(0, 1)
tester.reset_values()

print("step physics once 1/60s per step and rendering 10 times at 1.0/600s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
for step in range(10):
    simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 600.0)
tester.check_steps(1, 10)
tester.reset_values()

print("step physics once 1/60s per step and rendering once at 1.0/600s by explicitly calling step and render")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
simulation_context.step(render=False)
simulation_context.render()
tester.check_dt(1.0 / 60.0, 1.0 / 600.0)
tester.check_steps(1, 1)
tester.reset_values()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=False)
simulation_context.render()

tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(1, 1)
tester.reset_values()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(1, 1)
tester.reset_values()

print("render a new frame with simulation stopped, editor timeline does not move forward")
simulation_context.stop()  # stop calls render, so clear tester after this
tester.reset_values()

simulation_context.render()
tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(0, 1)
tester.reset_values()


print("cleanup and exit")
simulation_context.stop()
simulation_context.clear_instance()
simulation_app.close()
