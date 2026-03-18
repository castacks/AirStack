# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-sensors", type=int, default=1, help="Number of sensors")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_sensor = args.num_sensors
n_frames = args.num_frames
n_gpus = args.num_gpus

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpus})

import carb
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import delete_prim
from pxr import Gf

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark


# Create RTX Radar from params
def add_rtx_radar(prim_path, sensor_translation, sensor_orientation):
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxRadar",
        path=prim_path,
        parent=None,
        config="Example",
        translation=sensor_translation,
        orientation=sensor_orientation,
    )
    return sensor


# ----------------------------------------------------------------------
# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_rtx_radar",
    workflow_metadata={
        "metadata": [
            {"name": "num_radars", "data": n_sensor},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
timeline = omni.timeline.get_timeline_interface
sensors = []
writers = []
hydra_textures = []

for i in range(n_sensor):
    radar_path = f"/World/rtx_radar_{i}"
    sensor_translation = Gf.Vec3f([-0.937, -2.0 + i * 2.0, 0.8940])  # defined for full_warehouse.usd
    sensor_orientation = Gf.Quatd(0.70711, 0.70711, 0, 0)
    sensor = add_rtx_radar(radar_path, sensor_translation, sensor_orientation)
    sensors.append(sensor)

    hydra = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
    hydra_textures.append(hydra)

    # Post-process graph to publish the render var
    writer = rep.writers.get("Writer" + "IsaacPrintRTXSensorInfo")
    writer.initialize()
    writer.attach([hydra])
    writers.append(writer)

    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.set_phase("benchmark")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# NOTE: Need extra updates to process full num of frames
omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

for _ in range(0, n_frames):
    omni.kit.app.get_app().update()


benchmark.store_measurements()
benchmark.stop()

timeline.stop()

# Destroy sensor components
for writer in writers:
    writer.detach()
omni.kit.app.get_app().update()

for sensor in sensors:
    delete_prim(sensor.GetPath())
omni.kit.app.get_app().update()

for texture in hydra_textures:
    omni.kit.app.get_app().update()
    texture.destroy()
    texture = None
omni.kit.app.get_app().update()

simulation_app.close()
