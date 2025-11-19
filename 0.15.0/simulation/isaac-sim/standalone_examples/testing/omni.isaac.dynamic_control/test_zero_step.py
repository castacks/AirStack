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
import carb
import omni
from isaacsim.core.api import SimulationContext
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control

stage = simulation_app.context.get_stage()
sim_context = SimulationContext(stage_units_in_meters=1.0)

physx_interface = omni.physx.acquire_physx_interface()
physx_interface.start_simulation()
physx_interface.force_load_physics_from_usd()
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

prim = stage.DefinePrim("/panda", "Xform")
prim.GetReferences().AddReference(asset_path)
physx_interface.force_load_physics_from_usd()
sim_context._timeline.play()
omni.physx.acquire_physx_interface().update_simulation(elapsedStep=0, currentTime=0)
dc = _dynamic_control.acquire_dynamic_control_interface()
# Get the handle to force it to refresh, this should not crash.
art = dc.get_articulation("/panda")
sim_context._timeline.stop()
sim_context._timeline.play()
simulation_app.update()
simulation_app.close()
