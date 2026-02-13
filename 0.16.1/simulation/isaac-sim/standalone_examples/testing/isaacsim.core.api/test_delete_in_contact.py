# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

import sys

import carb
import numpy as np
from isaacsim.core.api import World as Simulator
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.prims import add_reference_to_stage, delete_prim
from isaacsim.sensors.physics import _sensor
from isaacsim.storage.native import get_assets_root_path
from pxr import PhysxSchema

#################################################
# Set this!
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    app.close()
    sys.exit()

#################################################


sim = Simulator(stage_units_in_meters=1.0)
sim.scene.add_ground_plane()
stage = sim.stage
sim.stop()

sim.get_physics_context().enable_gpu_dynamics(True)
sim.get_physics_context().set_broadphase_type("GPU")

block_0_prim = add_reference_to_stage(
    prim_path="/World/block_0", usd_path=assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
)
block_0 = RigidPrim("/World/block_0/Cube", name="block_0", positions=np.array([[0, 0, 0.5]]), scales=[np.ones(3) * 1.0])
PhysxSchema.PhysxContactReportAPI.Apply(block_0.prims[0])

block_1_prim = add_reference_to_stage(
    prim_path="/World/block_1", usd_path=assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
)
block_1 = RigidPrim(
    "/World/block_1/Cube", name="block_1", positions=np.array([[0, 0, 10.0]]), scales=[np.ones(3) * 1.0]
)
PhysxSchema.PhysxContactReportAPI.Apply(block_1.prims[0])

cs = _sensor.acquire_contact_sensor_interface()


def block_1_is_contacting_block_0():
    raw_data = cs.get_rigid_body_raw_data(block_1.prim_paths[0])
    in_contact = False
    for c in raw_data:
        c = [*c]
        print(c)
        if block_0.prim_paths[0] in {cs.decode_body_name(c[2]), cs.decode_body_name(c[3])}:
            in_contact = True
            break

    return in_contact


sim.play()
sim.step()

while not block_1_is_contacting_block_0():
    sim.step()

# delete prim once its in contact.
delete_prim(block_0_prim.GetPrimPath().pathString)

for i in range(1000):
    sim.step()

sim.stop()
app.close()
