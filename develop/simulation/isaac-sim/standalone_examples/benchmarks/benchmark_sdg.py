# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse

VALID_ANNOTATORS = {
    "rgb",
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
    "distance_to_camera",
    "distance_to_image_plane",
    "bounding_box_3d",
    "occlusion",
    "normals",
    "motion_vectors",
    "camera_params",
    "pointcloud",
    "skeleton_data",
}

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to capture")
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution")
parser.add_argument(
    "--asset-count", type=int, default=10, help="Number of assets of each type (cube, cone, cylinder, sphere, torus)"
)
parser.add_argument(
    "--annotators",
    nargs="+",
    default=["rgb"],
    choices=list(VALID_ANNOTATORS) + ["all"],
    help="List of annotators to enable, separated by space. Use 'all' to select all available.",
)
parser.add_argument("--disable-viewport-rendering", action="store_true", help="Disable viewport rendering")
parser.add_argument("--delete-data-when-done", action="store_true", help="Delete local data after benchmarking")
parser.add_argument("--print-results", action="store_true", help="Print results in terminal")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

parser.add_argument("--skip-write", action="store_true", help="Skip writing annotator data to disk")
parser.add_argument("--env-url", default=None, help="Path to the environment url, default None")

args, unknown = parser.parse_known_args()

num_frames = args.num_frames
num_cameras = args.num_cameras
width, height = args.resolution[0], args.resolution[1]
asset_count = args.asset_count
annotators_str = ", ".join(args.annotators)
disable_viewport_rendering = args.disable_viewport_rendering
delete_data_when_done = args.delete_data_when_done
print_results = args.print_results
headless = args.headless
n_gpu = args.num_gpus
skip_write = args.skip_write
env_url = args.env_url

if "all" in args.annotators:
    annotators_kwargs = {annotator: True for annotator in VALID_ANNOTATORS}
else:
    annotators_kwargs = {annotator: True for annotator in args.annotators if annotator in VALID_ANNOTATORS}

print(f"[SDG Benchmark] Running SDG Benchmark with:")
print(f"\tnum_frames: {num_frames}")
print(f"\tnum_cameras: {num_cameras}")
print(f"\tresolution: {width}x{height}")
print(f"\tasset_count: {asset_count}")
print(f"\tannotators: {annotators_kwargs.keys()}")
print(f"\tdisable_viewport_rendering: {disable_viewport_rendering}")
print(f"\tdelete_data_when_done: {delete_data_when_done}")
print(f"\tprint_results: {print_results}")
print(f"\theadless: {headless}")
print(f"\tskip_write: {skip_write}")
print(f"\tenv_url: {env_url}")

import os
import shutil
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

REPLICATOR_GLOBAL_SEED = 11

import carb
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_sdg",
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": num_frames},
            {"name": "num_cameras", "data": num_cameras},
            {"name": "width", "data": width},
            {"name": "height", "data": height},
            {"name": "asset_count", "data": asset_count},
            {"name": "annotators", "data": annotators_str},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

if env_url is not None:
    env_path = env_url if env_url.startswith("omniverse://") else get_assets_root_path() + env_url
    print(f"[SDG Benchmark] Loading stage from path: {env_path}")
    omni.usd.get_context().open_stage(env_path)
else:
    print(f"[SDG Benchmark] Loading a new empty stage..")
    omni.usd.get_context().new_stage()

if disable_viewport_rendering:
    print(f"[SDG Benchmark] Disabling viewport rendering..")
    get_active_viewport().updates_enabled = False

rep.set_global_seed(REPLICATOR_GLOBAL_SEED)
rep.create.light(rotation=(315, 0, 0), intensity=2000, light_type="distant")
rep.create.light(intensity=400, light_type="dome")
cubes = rep.create.cube(count=asset_count, semantics=[("class", "cube")])
cones = rep.create.cone(count=asset_count, semantics=[("class", "cone")])
cylinders = rep.create.cylinder(count=asset_count, semantics=[("class", "cylinder")])
spheres = rep.create.sphere(count=asset_count, semantics=[("class", "sphere")])
tori = rep.create.torus(count=asset_count, semantics=[("class", "torus")])

cameras = []
for i in range(num_cameras):
    cameras.append(rep.create.camera(name=f"cam_{i}"))
render_products = []
for i, cam in enumerate(cameras):
    render_products.append(rep.create.render_product(cam, (width, height), name=f"rp_{i}"))
if skip_write:
    print("[SDG Benchmark] Skipping writing to disk, attaching annotators to render products..")
    for annot_type, enabled in annotators_kwargs.items():
        if enabled:
            annot = rep.AnnotatorRegistry.get_annotator(annot_type)
            for rp in render_products:
                annot.attach(rp)
else:
    writer = rep.writers.get("BasicWriter")
    output_directory = (
        os.getcwd()
        + f"/_out_sdg_benchmark_{num_frames}_frames_{num_cameras}_cameras_{asset_count}_asset_count_{len(annotators_kwargs)}_annotators"
    )
    print(f"[SDG Benchmark] Output directory: {output_directory}")
    writer.initialize(output_dir=output_directory, **annotators_kwargs)
    writer.attach(render_products)
assets = rep.create.group([cubes, cones, cylinders, spheres, tori])
cameras = rep.create.group(cameras)

with rep.trigger.on_frame():
    with assets:
        rep.modify.pose(
            position=rep.distribution.uniform((-3, -3, -3), (3, 3, 3)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
            scale=rep.distribution.uniform(0.1, 1),
        )
        rep.randomizer.color(rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    with cameras:
        rep.modify.pose(
            position=rep.distribution.uniform((5, 5, 5), (10, 10, 10)),
            look_at=(0, 0, 0),
        )

rep.orchestrator.preview()
# Run for a few frames to ensure everything is loaded
for _ in range(10):
    omni.kit.app.get_app().update()
benchmark.store_measurements()

print("[SDG Benchmark] Starting SDG..")
benchmark.set_phase("benchmark")
start_time = time.time()
rep.orchestrator.run_until_complete(num_frames=num_frames)
end_time = time.time()
benchmark.store_measurements()
omni.kit.app.get_app().update()

duration = end_time - start_time
avg_frametime = duration / num_frames
if delete_data_when_done and not skip_write:
    print(f"[SDG Benchmark] Deleting data: {output_directory}")
    shutil.rmtree(output_directory)
if print_results:
    print(f"[SDG Benchmark] duration: {duration} seconds")
    print(f"[SDG Benchmark] avg frametime: {avg_frametime:.4f} seconds")
    print(f"[SDG Benchmark] avg FPS: {1 / avg_frametime:.2f}")
    results_csv = f"{num_frames}, {num_cameras}, {width}, {height}, {asset_count}, {duration:.4f}, {avg_frametime:.4f}, {1 / avg_frametime:.2f}"
    print(f"num_frames, num_cameras, width, height, asset_count, duration, avg_frametime, avg_fps\n{results_csv}\n")

benchmark.stop()

simulation_app.close()
