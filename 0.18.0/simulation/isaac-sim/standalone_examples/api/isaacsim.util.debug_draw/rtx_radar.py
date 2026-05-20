# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--config", type=str, default="Example", help="Name of radar config.")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Example for creating a RTX lidar sensor and publishing PCL data
simulation_app = SimulationApp({"headless": False})
import carb
import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

# enable ROS bridge extension
enable_extension("isaacsim.util.debug_draw")

simulation_app.update()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

simulation_app.update()
# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/background"
)
simulation_app.update()

radar_config = args.config

# Create the radar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about +Z so it faces warehouse shelves.
# Possible config options are Example.
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config=radar_config,
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(0.70711, 0.0, 0.0, 0.70711),
)
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()

# Create the debug draw pipeline in the post process graph
writer = rep.writers.get("RtxRadar" + "DebugDrawPointCloud")
writer.attach([hydra_texture])

simulation_app.update()

simulation_context.play()

while simulation_app.is_running():
    simulation_app.update()

# cleanup and shutdown
simulation_context.stop()
simulation_app.close()
