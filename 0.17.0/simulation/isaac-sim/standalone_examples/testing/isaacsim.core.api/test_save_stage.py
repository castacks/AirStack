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

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage, save_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
simulation_context = SimulationContext()
add_reference_to_stage(asset_path, "/Franka")
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
simulation_context.play()

simulation_context.step(render=True)

assets_root = get_assets_root_path()
if simulation_context._sim_context_initialized == False:
    raise (ValueError(f"simulation context is not initialized"))
save_stage(assets_root + "/Users/test/save_stage.usd", save_and_reload_in_place=False)
if simulation_context._sim_context_initialized == False:
    raise (ValueError(f"simulation context is not initialized"))
simulation_context.step(render=True)
save_stage(assets_root + "/Users/test/save_stage.usd", save_and_reload_in_place=True)
# this should reload the stage and the context should not be initialized anymore
if simulation_context._sim_context_initialized == True:
    raise (ValueError(f"simulation context should not be initialized"))
simulation_context.stop()
simulation_app.close()
