# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp({"extra_args": ["--/app/extra/arg=1", "--/app/some/other/arg=2"]})

import carb

kit.update()

server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")

if server_check != "omniverse://ov-test-this-is-working":
    raise ValueError(f"isaac nucleus default setting not omniverse://ov-test-this-is-working, instead: {server_check}")

arg_1 = carb.settings.get_settings().get_as_int("/app/extra/arg")
arg_2 = carb.settings.get_settings().get_as_int("/app/some/other/arg")

if arg_1 != 1:
    raise ValueError(f"/app/extra/arg was not 1 and was {arg_1} instead")

if arg_2 != 2:
    raise ValueError(f"/app/some/other/arg was not 2 and was {arg_2} instead")

kit.close()  # Cleanup application
