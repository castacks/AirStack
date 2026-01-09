# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"renderer": "RaytracedLighting", "headless": False})

import json
import os

import carb.settings
import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.semantics import add_update_semantics
from PIL import Image


# Util function to save rgb annotator data
def write_rgb_data(rgb_data, file_path):
    rgb_img = Image.fromarray(rgb_data, "RGBA")
    rgb_img.save(file_path + ".png")


# Util function to save semantic segmentation annotator data
def write_sem_data(sem_data, file_path):
    id_to_labels = sem_data["info"]["idToLabels"]
    with open(file_path + ".json", "w") as f:
        json.dump(id_to_labels, f)
    sem_image_data = np.frombuffer(sem_data["data"], dtype=np.uint8).reshape(*sem_data["data"].shape, -1)
    sem_img = Image.fromarray(sem_image_data, "RGBA")
    sem_img.save(file_path + ".png")


# Create a new stage with the default ground plane
omni.usd.get_context().new_stage()

# Setup the simulation world
world = World()
world.scene.add_default_ground_plane()
world.reset()

# Setting capture on play to False will prevent the replicator from capturing data each frame
carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

# Create a camera and render product to collect the data from
cam = rep.create.camera(position=(5, 5, 5), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (512, 512))

# Set the output directory for the data
out_dir = os.getcwd() + "/_out_sim_event"
os.makedirs(out_dir, exist_ok=True)
print(f"Outputting data to {out_dir}..")

# Example of using a writer to save the data
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir=f"{out_dir}/writer", rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True
)
writer.attach(rp)

# Run a preview to ensure the replicator graph is initialized
rep.orchestrator.preview()

# Example of accesing the data directly from annotators
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach(rp)
sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True})
sem_annot.attach(rp)

# Spawn and drop a few cubes, capture data when they stop moving
for i in range(5):
    cuboid = world.scene.add(DynamicCuboid(prim_path=f"/World/Cuboid_{i}", name=f"Cuboid_{i}", position=(0, 0, 10 + i)))
    add_update_semantics(cuboid.prim, "Cuboid")

    for s in range(500):
        world.step(render=False)
        vel = np.linalg.norm(cuboid.get_linear_velocity())
        if vel < 0.1:
            print(f"Cube_{i} stopped moving after {s} simulation steps, writing data..")
            # Tigger the writer and update the annotators with new data
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
            write_rgb_data(rgb_annot.get_data(), f"{out_dir}/Cube_{i}_step_{s}_rgb")
            write_sem_data(sem_annot.get_data(), f"{out_dir}/Cube_{i}_step_{s}_sem")
            break

rep.orchestrator.wait_until_complete()

simulation_app.close()
