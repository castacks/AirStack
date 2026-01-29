# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.semantics import add_update_semantics
from omni.replicator.core import Writer
from pxr import Sdf, UsdGeom


# Create a custom writer to access the annotator data
class MyWriter(Writer):
    def __init__(self, camera_params: bool = True, bounding_box_3d: bool = True):
        # Organize data from render product perspective (legacy, annotator, renderProduct)
        self.data_structure = "renderProduct"
        if camera_params:
            self.annotators.append(rep.annotators.get("camera_params"))
        if bounding_box_3d:
            self.annotators.append(rep.annotators.get("bounding_box_3d"))
        self._frame_id = 0

    def write(self, data):
        print(f"[MyWriter][{self._frame_id}] data:{data}")
        self._frame_id += 1


# Register the writer for use
rep.writers.register_writer(MyWriter)


def run_example():
    # Create a new stage and disable capture on play
    omni.usd.get_context().new_stage()
    rep.orchestrator.set_capture_on_play(False)

    # Setup stage
    stage = omni.usd.get_context().get_stage()
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
    cube = stage.DefinePrim("/World/Cube", "Cube")
    add_update_semantics(cube, "MyCube")

    # Capture from two perspectives, a custom camera and the viewport perspective camera
    camera = stage.DefinePrim("/World/Camera", "Camera")
    UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 20))

    # Create the render products
    rp_cam = rep.create.render_product(camera.GetPath(), (400, 400), name="camera_view")
    rp_persp = rep.create.render_product("/OmniverseKit_Persp", (512, 512), name="perspective_view")

    # Use the annotators to access the data directly, each annotator is attached to a render product
    rgb_annotator_cam = rep.annotators.get("rgb")
    rgb_annotator_cam.attach(rp_cam)
    rgb_annotator_persp = rep.annotators.get("rgb")
    rgb_annotator_persp.attach(rp_persp)

    # Use the custom writer to access the annotator data
    custom_writer = rep.writers.get("MyWriter")
    custom_writer.initialize(camera_params=True, bounding_box_3d=True)
    custom_writer.attach([rp_cam, rp_persp])

    # Use the pose writer to write the data to disk
    pose_writer = rep.WriterRegistry.get("PoseWriter")
    out_dir = os.getcwd() + "/_out_pose_writer"
    print(f"Output directory: {out_dir}")
    pose_writer.initialize(output_dir=out_dir, write_debug_images=True)
    pose_writer.attach([rp_cam, rp_persp])

    # Trigger a data capture request (data will be written to disk by the writer)
    for i in range(3):
        print(f"Step {i}")
        rep.orchestrator.step()

        # Get the data from the annotators
        rgb_data_cam = rgb_annotator_cam.get_data()
        rgb_data_persp = rgb_annotator_persp.get_data()
        print(f"[Annotator][Cam][{i}] rgb_data_cam shape: {rgb_data_cam.shape}")
        print(f"[Annotator][Persp][{i}] rgb_data_persp shape: {rgb_data_persp.shape}")

    # Detach the render products from the annotators and writers and clear them to release resources
    pose_writer.detach()
    custom_writer.detach()
    rgb_annotator_cam.detach()
    rgb_annotator_persp.detach()
    rp_cam.destroy()
    rp_persp.destroy()

    # Wait for the data to be written to disk
    rep.orchestrator.wait_until_complete()


run_example()

simulation_app.close()
