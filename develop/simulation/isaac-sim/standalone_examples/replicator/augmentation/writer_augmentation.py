# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""Generate augmented synthetic from a writer
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import os
import time

import carb.settings
import numpy as np
import omni.replicator.core as rep
import warp as wp
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--num_frames", type=int, default=25, help="The number of frames to capture")
parser.add_argument("--use_warp", action="store_true", help="Use warp augmentations instead of numpy")
args, unknown = parser.parse_known_args()

NUM_FRAMES = args.num_frames
USE_WARP = args.use_warp
ENV_URL = "/Isaac/Environments/Grid/default_environment.usd"

# Enable scripts
carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)

# Gaussian noise augmentation on rgba data in numpy (CPU) and warp (GPU)
def gaussian_noise_rgb_np(data_in, sigma: float, seed: int):
    np.random.seed(seed)
    data_in[:, :, 0] = data_in[:, :, 0] + np.random.randn(*data_in.shape[:-1]) * sigma
    data_in[:, :, 1] = data_in[:, :, 1] + np.random.randn(*data_in.shape[:-1]) * sigma
    data_in[:, :, 2] = data_in[:, :, 2] + np.random.randn(*data_in.shape[:-1]) * sigma
    return data_in


@wp.kernel
def gaussian_noise_rgb_wp(
    data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8), sigma: float, seed: int
):
    i, j = wp.tid()
    state = wp.rand_init(seed, wp.tid())
    data_out[i, j, 0] = wp.uint8(wp.int32(data_in[i, j, 0]) + wp.int32(sigma * wp.randn(state)))
    data_out[i, j, 1] = wp.uint8(wp.int32(data_in[i, j, 1]) + wp.int32(sigma * wp.randn(state)))
    data_out[i, j, 2] = wp.uint8(wp.int32(data_in[i, j, 2]) + wp.int32(sigma * wp.randn(state)))
    data_out[i, j, 3] = data_in[i, j, 3]


# Gaussian noise augmentation on depth data in numpy (CPU) and warp (GPU)
def gaussian_noise_depth_np(data_in, sigma: float, seed: int):
    np.random.seed(seed)
    return data_in + np.random.randn(*data_in.shape) * sigma


rep.AnnotatorRegistry.register_augmentation(
    "gn_depth_np", rep.annotators.Augmentation.from_function(gaussian_noise_depth_np, sigma=0.1, seed=None)
)


@wp.kernel
def gaussian_noise_depth_wp(
    data_in: wp.array2d(dtype=wp.float32), data_out: wp.array2d(dtype=wp.float32), sigma: float, seed: int
):
    i, j = wp.tid()
    state = wp.rand_init(seed, wp.tid())
    data_out[i, j] = data_in[i, j] + sigma * wp.randn(state)


rep.AnnotatorRegistry.register_augmentation(
    "gn_depth_wp", rep.annotators.Augmentation.from_function(gaussian_noise_depth_wp, sigma=0.1, seed=None)
)

# Setup the environment
assets_root_path = get_assets_root_path()
open_stage(assets_root_path + ENV_URL)

# Disable capture on play and async rendering
carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)
carb.settings.get_settings().set("/omni/replicator/asyncRendering", False)
carb.settings.get_settings().set("/app/asyncRendering", False)

# Create a red cube and a render product from a camera looking at the cube from the top
red_mat = rep.create.material_omnipbr(diffuse=(1, 0, 0))
red_cube = rep.create.cube(position=(0, 0, 0.71), material=red_mat)
cam = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (512, 512))

# Update the app a couple of times to fully load texture/materials
for _ in range(5):
    simulation_app.update()

# Access default annotators from replicator
rgb_to_hsv_augm = rep.annotators.Augmentation.from_function(rep.augmentations_default.aug_rgb_to_hsv)
hsv_to_rgb_augm = rep.annotators.Augmentation.from_function(rep.augmentations_default.aug_hsv_to_rgb)

# Access the custom annotators as functions or from the registry
gn_rgb_augm = None
gn_depth_augm = None
if USE_WARP:
    gn_rgb_augm = rep.annotators.Augmentation.from_function(gaussian_noise_rgb_wp, sigma=6.0, seed=None)
    gn_depth_augm = rep.AnnotatorRegistry.get_augmentation("gn_depth_wp")
else:
    gn_rgb_augm = rep.annotators.Augmentation.from_function(gaussian_noise_rgb_np, sigma=6.0, seed=None)
    gn_depth_augm = rep.AnnotatorRegistry.get_augmentation("gn_depth_np")

# Create a writer and apply the augmentations to its corresponding annotators
out_dir = os.path.join(os.getcwd(), "_out_augm_writer")
print(f"Writing data to: {out_dir}")
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=out_dir, rgb=True, distance_to_camera=True)

augmented_rgb_annot = rep.annotators.get("rgb").augment_compose(
    [rgb_to_hsv_augm, gn_rgb_augm, hsv_to_rgb_augm], name="rgb"
)
writer.add_annotator(augmented_rgb_annot)
writer.augment_annotator("distance_to_camera", gn_depth_augm)

# Attach render product to writer
writer.attach([rp])

# Generate a replicator graph randomizing the cube's rotation every frame
with rep.trigger.on_frame():
    with red_cube:
        rep.randomizer.rotation()

# Evaluate the graph
rep.orchestrator.preview()

# Measure the duration of capturing the data
start_time = time.time()

# The `step()` function will trigger the randomization graph, feed annotators with new data, and trigger the writers
for i in range(NUM_FRAMES):
    rep.orchestrator.step()

print(
    f"The duration for capturing {NUM_FRAMES} frames using '{'warp' if USE_WARP else 'numpy'}' was: {time.time() - start_time:.4f} seconds, with an average of {(time.time() - start_time) / NUM_FRAMES:.4f} seconds per frame."
)

simulation_app.close()
