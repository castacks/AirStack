# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp()

import omni

simulation_app.update()
omni.usd.get_context().new_stage()
simulation_app.update()

import omni.graph.core as og

keys = og.Controller.Keys
(graph, (tick_node, test_node, str_node), _, _) = og.Controller.edit(
    {"graph_path": "/controller_graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("IsaacTest", "isaacsim.core.nodes.IsaacTestNode"),
            ("TestStr", "omni.graph.nodes.ConstantString"),
        ],
        keys.SET_VALUES: [("TestStr.inputs:value", "Hello"), ("OnTick.inputs:onlyPlayback", False)],  # always tick
        keys.CONNECT: [
            ("OnTick.outputs:tick", "IsaacTest.inputs:execIn"),
            ("TestStr.inputs:value", "IsaacTest.inputs:input"),
        ],
    },
)

input_attr = og.Controller.attribute("inputs:value", str_node)
output_attr = og.Controller.attribute("outputs:output", test_node)

simulation_app.update()
value = og.DataView.get(output_attr)
print(value)
if value != "Hello":
    raise ValueError("Output does not equal Hello")
simulation_app.update()
og.DataView.set(input_attr, "Goodbye")
simulation_app.update()
value = og.DataView.get(output_attr)
print(value)
if value != "Goodbye":
    raise ValueError("Output does not equal Goodbye")

simulation_app.update()
# Cleanup application
simulation_app.close()
