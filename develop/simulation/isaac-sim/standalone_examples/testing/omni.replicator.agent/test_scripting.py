# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import asyncio

from isaacsim import SimulationApp

CONFIG = {"renderer": "RaytracedLighting", "headless": True, "width": 1920, "height": 1080}

if __name__ == "__main__":
    app = SimulationApp(launch_config=CONFIG)

    from isaacsim.core.utils.extensions import enable_extension

    app.update()

    enable_extension("omni.kit.scripting")

    import omni.usd
    from omni.kit.scripting import ApplyScriptingAPICommand
    from pxr import OmniScriptingSchema, Sdf

    async def work():

        # Create new prim and attach python scripting api.
        await omni.usd.get_context().new_stage_async("tmp")
        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim("/test")
        ApplyScriptingAPICommand(paths=["/test"]).do()

        # Test
        prim = stage.GetPrimAtPath("/test")
        assert prim.HasAPI(OmniScriptingSchema.OmniScriptingAPI)

    asyncio.run(work())
