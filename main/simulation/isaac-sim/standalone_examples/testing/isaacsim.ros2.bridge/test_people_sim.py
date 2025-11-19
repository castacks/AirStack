# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()

ADDITIONAL_EXTENSIONS_PEOPLE = [
    "omni.isaac.core",
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.graph.ui",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.anim.retarget.ui",
    "omni.kit.scripting",
]

import carb
import omni
from isaacsim.core.utils.extensions import enable_extension

for e in ADDITIONAL_EXTENSIONS_PEOPLE:
    enable_extension(e)
    kit.update()

enable_extension("isaacsim.ros2.bridge")
kit.update()

# Locate Isaac Sim assets folder to load sample
from isaacsim.storage.native import get_assets_root_path, is_file

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    kit.close()
    sys.exit()
usd_path = assets_root_path + "/Isaac/Samples/NvBlox/nvblox_sample_scene.usd"

omni.usd.get_context().open_stage(usd_path)

for i in range(100):
    kit.update()

omni.timeline.get_timeline_interface().play()

for i in range(100):
    kit.update()

kit.close()  # Cleanup application
