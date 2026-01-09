# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""Standalone application script demonstrating a use case for the isaacsim.util.clash_detection API.
Randomly places Carter assets in a warehouse scene and informs the user if a carter mesh is clashing 
with any other mesh in the scene. Supports exporting detailed information of any clashes to JSON 
files for further analysis.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import random

import numpy as np
from omni.isaac.core.utils.extensions import enable_extension

# enable isaac sim clash detection extension
enable_extension("isaacsim.util.clash_detection")
simulation_app.update()

from isaacsim.storage.native import get_assets_root_path
from isaacsim.util.clash_detection import ClashDetector
from omni.isaac.core.prims import XFormPrim, XFormPrimView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage, open_stage

parser = argparse.ArgumentParser()
parser.add_argument("--export_json", default=False, action="store_true", help="Export clash detection results to JSON")
parser.add_argument(
    "--export_folder", action="store", type=str, help="Path to folder storing JSON files if exporting clash results"
)
args, unknown = parser.parse_known_args()

ENV_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
CARTER_PATH = "/Isaac/Robots/Carter/carter_v1.usd"
EXPORT = args.export_json
EXPORT_PATH = args.export_folder

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Setup environment
open_stage(assets_root_path + ENV_PATH)
stage = get_current_stage()

simulation_app.update()

# Initialize clash detection engine
clash_detector = ClashDetector(stage, logging=False, clash_data_layer=False)

# Place Carter assets
carter_usd_path = assets_root_path + CARTER_PATH

for idx in range(5):
    carter_prim_path = f"/World/Carter_{idx}"
    add_reference_to_stage(usd_path=carter_usd_path, prim_path=carter_prim_path)

    XFormPrim(carter_prim_path).set_local_pose(
        translation=np.array([random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0, 0.4)])
    )

    simulation_app.update()

    query_name = f"carter_{idx}_query"
    if clash_detector.is_prim_clashing(get_prim_at_path(carter_prim_path), query_name=query_name):
        print(f"Clash detected for {carter_prim_path}")
        if EXPORT:
            query_id = clash_detector.get_current_query_id()
            clash_detector.export_to_json(EXPORT_PATH + f"/Carter_{idx}_clash_data.json")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
