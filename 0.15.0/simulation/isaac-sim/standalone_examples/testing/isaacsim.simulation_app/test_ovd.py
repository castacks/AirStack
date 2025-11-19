# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
from pathlib import Path

from isaacsim import SimulationApp

kit = SimulationApp()

import carb

for _ in range(10):
    kit.update()

# get the current output path and check if the file exists
pvd_output_dir = carb.settings.get_settings().get_as_string("/persistent/physics/omniPvdOvdRecordingDirectory")

print("omniPvdOvdRecordingDirectory: ", pvd_output_dir)
my_file = Path(os.path.join(pvd_output_dir, "tmp.ovd"))
assert my_file.is_file()

kit.close()
