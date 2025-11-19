# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys

import carb
import omni.kit.test
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# ----------------------------------------------------------------------
# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_physx_lidar",
)


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")

benchmark.set_phase("benchmark")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

physics_context = PhysicsContext(physics_dt=1.0 / 60.0)
time = 0
for _ in range(0, 1000):
    physics_context._step(time)
    time += physics_context.get_physics_dt()

benchmark.store_measurements()
min_physics_time = 0
for measurement in benchmark._test_phases[0].measurements:
    if "Min Physics Frametime" in measurement.name:
        min_physics_time = measurement.value

benchmark.stop()
timeline.stop()
assert min_physics_time > 0
simulation_app.close()
