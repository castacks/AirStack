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
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import asyncio

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path, recursive_list_folder

enable_extension("isaacsim.benchmark.services")

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.datarecorders import interface
from isaacsim.benchmark.services.metrics import measurements


class IsaacSimNucleusKPIRecorder(interface.MeasurementDataRecorder):
    def __init__(self):
        self.assets_root_path = get_assets_root_path()
        self._loop = asyncio.get_event_loop()

    def _get_num_usds_in_path(self, path: str):
        files = self._loop.run_until_complete(recursive_list_folder(path))
        return len([f for f in files if f.endswith(".usd")])

    def get_data(self):

        measurements_out = []

        # of objects & scenes for SDG in USD
        nucleus_env_paths = self.assets_root_path + "/Isaac/Environments/"
        num_environments = self._get_num_usds_in_path(nucleus_env_paths)

        nucleus_prop_paths = self.assets_root_path + "/Isaac/Props/"
        num_props = self._get_num_usds_in_path(nucleus_prop_paths)
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Number of Objects & Scenes for SDG",
                value=num_environments + num_props,
                unit="",
            )
        )

        # of robots from partners
        nucleus_robot_paths = self.assets_root_path + "/Isaac/Robots/"
        num_robots_from_partners = self._get_num_usds_in_path(nucleus_robot_paths)
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Number of Robots from Partners",
                value=num_robots_from_partners,
                unit="",
            )
        )

        # of sensors from partners
        nucleus_sensor_paths = self.assets_root_path + "/Isaac/Sensors/"
        num_sensors_from_partners = self._get_num_usds_in_path(nucleus_sensor_paths)
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Number of Sensors from Partners",
                value=num_sensors_from_partners,
                unit="",
            )
        )

        return interface.MeasurementData(measurements=measurements_out)


# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_nucleus_kpis",
    backend_type=args.backend_type,
)
benchmark.set_phase("benchmark", start_recording_frametime=False, start_recording_runtime=False)
benchmark.recorders.append(IsaacSimNucleusKPIRecorder())
benchmark.store_measurements()

benchmark.stop()

simulation_app.close()
