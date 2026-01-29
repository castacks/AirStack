# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os

import omni.kit
import omni.replicator.core as rep
import omni.usd
from omni.replicator.core import AnnotatorRegistry, Writer
from PIL import Image
from pxr import Sdf, UsdGeom

NUM_FRAMES = 5

# Save rgb image to file
def save_rgb(rgb_data, file_name):
    rgb_img = Image.fromarray(rgb_data, "RGBA")
    rgb_img.save(file_name + ".png")


# Randomize cube color every frame using a replicator randomizer
def cube_color_randomizer():
    cube_prims = rep.get.prims(path_pattern="Cube")
    with cube_prims:
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    return cube_prims.node


# Access data through a custom replicator writer
class MyWriter(Writer):
    def __init__(self, rgb: bool = True):
        self._frame_id = 0
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))
        # Create writer output directory
        self.file_path = os.path.join(os.getcwd(), "_out_mc_writer", "")
        print(f"Writing writer data to {self.file_path}")
        dir = os.path.dirname(self.file_path)
        os.makedirs(dir, exist_ok=True)

    def write(self, data):
        for annotator in data.keys():
            annotator_split = annotator.split("-")
            if len(annotator_split) > 1:
                render_product_name = annotator_split[-1]
            if annotator.startswith("rgb"):
                save_rgb(data[annotator], f"{self.file_path}/{render_product_name}_frame_{self._frame_id}")
        self._frame_id += 1


rep.WriterRegistry.register(MyWriter)

# Create a new stage with a dome light
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(900.0)

# Create cube
cube_prim = stage.DefinePrim("/World/Cube", "Cube")
UsdGeom.Xformable(cube_prim).AddTranslateOp().Set((0.0, 5.0, 1.0))

# Register cube color randomizer to trigger on every frame
rep.randomizer.register(cube_color_randomizer)
with rep.trigger.on_frame():
    rep.randomizer.cube_color_randomizer()

# Create cameras
camera_prim1 = stage.DefinePrim("/World/Camera1", "Camera")
UsdGeom.Xformable(camera_prim1).AddTranslateOp().Set((0.0, 10.0, 20.0))
UsdGeom.Xformable(camera_prim1).AddRotateXYZOp().Set((-15.0, 0.0, 0.0))

camera_prim2 = stage.DefinePrim("/World/Camera2", "Camera")
UsdGeom.Xformable(camera_prim2).AddTranslateOp().Set((-10.0, 15.0, 15.0))
UsdGeom.Xformable(camera_prim2).AddRotateXYZOp().Set((-45.0, 0.0, 45.0))

# Create render products
rp1 = rep.create.render_product(str(camera_prim1.GetPrimPath()), resolution=(320, 320))
rp2 = rep.create.render_product(str(camera_prim2.GetPrimPath()), resolution=(640, 640))
rp3 = rep.create.render_product("/OmniverseKit_Persp", (1024, 1024))

# Acess the data through a custom writer
writer = rep.WriterRegistry.get("MyWriter")
writer.initialize(rgb=True)
writer.attach([rp1, rp2, rp3])

# Acess the data through annotators
rgb_annotators = []
for rp in [rp1, rp2, rp3]:
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(rp)
    rgb_annotators.append(rgb)

# Create annotator output directory
file_path = os.path.join(os.getcwd(), "_out_mc_annot", "")
print(f"Writing annotator data to {file_path}")
dir = os.path.dirname(file_path)
os.makedirs(dir, exist_ok=True)

# Data will be captured manually using step
rep.orchestrator.set_capture_on_play(False)

for i in range(NUM_FRAMES):
    # The step function provides new data to the annotators, triggers the randomizers and the writer
    rep.orchestrator.step(rt_subframes=4)
    for j, rgb_annot in enumerate(rgb_annotators):
        save_rgb(rgb_annot.get_data(), f"{dir}/rp{j}_step_{i}")

simulation_app.close()
