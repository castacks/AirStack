# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.syntheticdata import sensors
from omni.syntheticdata.tests.utils import add_semantics

viewport_api = get_active_viewport()
simulation_app.update()
stage = get_current_stage()
simulation_app.update()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    exit()
robot_usd = assets_root_path + "/Isaac/Robots/Carter/carter_v1_physx_lidar.usd"

# setup high-level robot prim
prim = stage.DefinePrim("/robot", "Xform")
prim.GetReferences().AddReference(robot_usd)
add_semantics(prim, "robot")

simulation_app.update()

sensors.enable_sensors(
    viewport_api,
    [
        sd.SensorType.Rgb,
        sd.SensorType.DistanceToImagePlane,
        sd.SensorType.InstanceSegmentation,
        sd.SensorType.SemanticSegmentation,
        sd.SensorType.BoundingBox2DTight,
        sd.SensorType.BoundingBox2DLoose,
        sd.SensorType.BoundingBox3D,
        sd.SensorType.Occlusion,
    ],
)

for frame in range(100):
    simulation_app.update()

print(sensors.get_rgb(viewport_api))
print(sensors.get_distance_to_image_plane(viewport_api))
print(sensors.get_instance_segmentation(viewport_api, parsed=True, return_mapping=True))
print(sensors.get_semantic_segmentation(viewport_api))
print(sensors.get_bounding_box_2d_tight(viewport_api))
print(sensors.get_bounding_box_2d_loose(viewport_api))
print(sensors.get_bounding_box_3d(viewport_api, parsed=True, return_corners=True))
print(sensors.get_occlusion(viewport_api))


simulation_app.close()
