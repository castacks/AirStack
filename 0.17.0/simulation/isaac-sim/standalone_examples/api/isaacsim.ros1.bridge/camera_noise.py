# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys

from isaacsim import SimulationApp

CAMERA_STAGE_PATH = "/Camera"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RaytracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)
import carb
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import warp as wp
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, stage
from isaacsim.core.utils.render_product import set_camera_prim_path
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom

# enable ROS bridge extension
extensions.enable_extension("isaacsim.ros1.bridge")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Creating a Camera prim
camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim)
xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim.GetHorizontalApertureAttr().Set(21)
camera_prim.GetVerticalApertureAttr().Set(16)
camera_prim.GetProjectionAttr().Set("perspective")
camera_prim.GetFocalLengthAttr().Set(24)
camera_prim.GetFocusDistanceAttr().Set(400)

simulation_app.update()

# grab our render product and directly set the camera prim
render_product_path = get_active_viewport().get_render_product_path()
set_camera_prim_path(render_product_path, CAMERA_STAGE_PATH)

# GPU Noise Kernel for illustrative purposes, input is rgba, outputs rgb
@wp.kernel
def image_gaussian_noise_warp(
    data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8), seed: int, sigma: float = 0.5
):
    i, j = wp.tid()
    dim_i = data_out.shape[0]
    dim_j = data_out.shape[1]
    pixel_id = i * dim_i + j
    state_r = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 0))
    state_g = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 1))
    state_b = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 2))

    data_out[i, j, 0] = wp.uint8(float(data_in[i, j, 0]) + (255.0 * sigma * wp.randn(state_r)))
    data_out[i, j, 1] = wp.uint8(float(data_in[i, j, 1]) + (255.0 * sigma * wp.randn(state_g)))
    data_out[i, j, 2] = wp.uint8(float(data_in[i, j, 2]) + (255.0 * sigma * wp.randn(state_b)))


# register new augmented annotator that adds noise to rgba and then outputs to rgb to the ROS publisher can publish
rep.annotators.register(
    name="rgb_gaussian_noise",
    annotator=rep.annotators.augment_compose(
        source_annotator=rep.annotators.get("rgb", device="cuda"),
        augmentations=[
            rep.annotators.Augmentation.from_function(
                image_gaussian_noise_warp, sigma=0.1, seed=1234, data_out_shape=(-1, -1, 3)
            ),
        ],
    ),
)

# Create a new writer with the augmented image
rep.writers.register_node_writer(
    name=f"CustomROS1PublishImage",
    node_type_id="isaacsim.ros1.bridge.ROS1PublishImage",
    annotators=[
        "rgb_gaussian_noise",
        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
            "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
        ),
    ],
    category="custom",
)
# Register writer for Replicator telemetry tracking
rep.WriterRegistry._default_writers.append(
    "CustomROS1PublishImage"
) if "CustomROS1PublishImage" not in rep.WriterRegistry._default_writers else None

# Create the new writer and attach to our render product
writer = rep.writers.get(f"CustomROS1PublishImage")
writer.initialize(topicName="rgb_augmented", frameId="sim_camera")
writer.attach([render_product_path])

simulation_app.update()
# Need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
simulation_context.play()

frame = 0

while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)

    if simulation_context.is_playing():
        # Rotate camera by 0.5 degree every frame
        xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        frame = frame + 1

simulation_context.stop()
simulation_app.close()
