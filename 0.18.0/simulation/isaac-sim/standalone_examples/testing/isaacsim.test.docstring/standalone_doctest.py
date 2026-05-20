# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

# enable the extension
import isaacsim.core.utils.extensions as extensions_utils

simulation_app.update()
extensions_utils.enable_extension("isaacsim.test.docstring")
simulation_app.update()

# run test
from isaacsim.test.docstring import StandaloneDocTestCase

tester = StandaloneDocTestCase()
tester.assertDocTests(StandaloneDocTestCase)

# quit
simulation_app.close()
