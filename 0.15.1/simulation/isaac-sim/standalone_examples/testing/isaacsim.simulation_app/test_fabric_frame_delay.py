# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": True}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.zero_delay.kit'
)

import sys

import carb
import isaacsim.core.utils.numpy.rotations as rot_utils
import isaacsim.core.utils.prims as prim_utils
import isaacsim.core.utils.stage as stage_utils
import matplotlib.pyplot as plt
import numpy as np
import torch
from isaacsim.core.api import SimulationContext
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.core.utils.prims import add_update_semantics, get_prim_attribute_value
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"


def main():
    for device, backend in [["cuda:0", "torch"], ["cpu", "numpy"]]:
        SimulationContext.clear_instance()
        stage_utils.create_new_stage()
        sim = SimulationContext(stage_units_in_meters=1.0, physics_dt=0.01, device=device, backend=backend)
        prim_utils.create_prim("/World/Origin1", "Xform", translation=[0.0, 0.0, 0.0])
        cube = DynamicCuboid(
            prim_path="/World/Origin1/cube",
            name="cube",
            position=np.array([-3.0, 0.0, 0.1]),
            scale=np.array([1.0, 2.0, 0.2]),
            size=1.0,
            color=np.array([255, 0, 0]),
        )
        stage_utils.add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        articulated_system = Articulation("/World/Franka")
        rigid_link = RigidPrim("/World/Franka/panda_link1")
        sim.reset()
        cube.initialize()
        rigid_link.initialize()
        articulated_system.initialize()
        articulated_system.set_world_poses(
            positions=torch.tensor([[-10, -10, 0]], device=device) if backend == "torch" else [[-10, -10, 0]]
        )
        position = cube.get_world_pose()[0]
        position[0] += 3
        cube.set_world_pose(position=position)
        if not (
            np.isclose(
                get_prim_attribute_value("/World/Origin1/cube", "_worldPosition", fabric=True),
                np.array([-3.0, 0.0, 0.1]),
                atol=0.01,
            ).all()
        ):
            raise (ValueError(f"PhysX is not synced with Fabric CPU"))
        sim.render()
        if not (
            np.isclose(
                get_prim_attribute_value("/World/Franka/panda_link1", "_worldPosition", fabric=True),
                np.array([-10.0, -10.0, 0.33]),
                atol=0.01,
            ).all()
        ):
            raise (ValueError(f"Kinematic Tree is not updated in fabric"))
        if not (
            np.isclose(
                get_prim_attribute_value("/World/Origin1/cube", "_worldPosition", fabric=True),
                np.array([0.0, 0.0, 0.1]),
                atol=0.01,
            ).all()
        ):
            raise (ValueError(f"PhysX is not synced with Fabric CPU"))


if __name__ == "__main__":
    main()
