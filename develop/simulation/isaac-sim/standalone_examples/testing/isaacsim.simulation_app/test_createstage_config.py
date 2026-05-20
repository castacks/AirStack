# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# Test app startup without creating new stage
kit = SimulationApp({"create_new_stage": False})

import omni

for i in range(100):
    kit.update()

omni.kit.app.get_app().print_and_log("Config: No empty stage was created")

kit.close()  # Cleanup application
