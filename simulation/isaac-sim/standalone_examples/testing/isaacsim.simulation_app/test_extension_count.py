# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp()

simulation_app.update()
simulation_app.update()

from typing import Tuple

import omni
import omni.ext
import omni.kit.app

app = omni.kit.app.get_app()
ext_manager = app.get_extension_manager()
ext_summaries = ext_manager.get_extensions()


def get_bundled_exts() -> dict[str, Tuple]:
    local_exts: dict[str, Tuple] = {}

    for ext_summary in ext_summaries:
        ext_name = ext_summary["name"]
        ext_enabled = bool(ext_summary["enabled"])
        if not ext_enabled:
            continue

        local_exts[ext_name] = (ext_name, ext_enabled)
    return local_exts


bundled_exts = get_bundled_exts()
print(f"Enabled extensions count: {len(bundled_exts)}")

# Cleanup application
simulation_app.close()
