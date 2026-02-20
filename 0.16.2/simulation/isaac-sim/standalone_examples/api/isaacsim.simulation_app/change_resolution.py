# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import random

from isaacsim import SimulationApp

# Simple example showing how to change resolution
kit = SimulationApp({"headless": True})
kit.update()
for i in range(100):
    width = random.randint(128, 1980)
    height = random.randint(128, 1980)
    kit.set_setting("/app/renderer/resolution/width", width)
    kit.set_setting("/app/renderer/resolution/height", height)
    kit.update()
    print(f"resolution set to: {width}, {height}")

# cleanup
kit.close()
