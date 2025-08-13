# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import random

import isaacsim.cortex.behaviors.ur10.bin_stacking_behavior as behavior
import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.core.api.objects import VisualCapsule, VisualSphere
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.framework.cortex_rigid_prim import CortexRigidPrim
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path_or_die
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.robot import CortexUr10


class Ur10Assets:
    def __init__(self):
        self.assets_root_path = get_assets_root_path_or_die()

        self.ur10_table_usd = (
            self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_short_suction.usd"
        )
        self.small_klt_usd = self.assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        self.background_usd = self.assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        self.rubiks_cube_usd = self.assets_root_path + "/Isaac/Props/Rubiks_Cube/rubiks_cube.usd"


def print_diagnostics(diagnostic):
    print("=========== logical state ==========")
    if diagnostic.bin_name:
        print("active bin info:")
        print("- bin_obj.name: {}".format(diagnostic.bin_name))
        print("- bin_base: {}".format(diagnostic.bin_base))
        print("- grasp_T:\n{}".format(diagnostic.grasp))
        print("- is_grasp_reached: {}".format(diagnostic.grasp_reached))
        print("- is_attached:  {}".format(diagnostic.attached))
        print("- needs_flip:  {}".format(diagnostic.needs_flip))
    else:
        print("<no active bin>")

    print("------------------------------------")


def random_bin_spawn_transform():
    x = random.uniform(-0.15, 0.15)
    y = 1.5
    z = -0.15
    position = np.array([x, y, z])

    z = random.random() * 0.02 - 0.01
    w = random.random() * 0.02 - 0.01
    norm = np.sqrt(z**2 + w**2)
    quat = math_util.Quaternion([w / norm, 0, 0, z / norm])
    if random.random() > 0.5:
        print("<flip>")
        # flip the bin so it's upside down
        quat = quat * math_util.Quaternion([0, 0, 1, 0])
    else:
        print("<no flip>")

    return position, quat.vals


class BinStackingTask(BaseTask):
    def __init__(self, env_path, assets):
        super().__init__("bin_stacking")
        self.assets = assets

        self.env_path = "/World/Ur10Table"
        self.bins = []
        self.stashed_bins = []
        self.on_conveyor = None

    def _spawn_bin(self, rigid_bin):
        x, q = random_bin_spawn_transform()
        rigid_bin.set_world_pose(position=x, orientation=q)
        rigid_bin.set_linear_velocity(np.array([0, -0.30, 0]))
        rigid_bin.set_visibility(True)

    def post_reset(self) -> None:
        if len(self.bins) > 0:
            for rigid_bin in self.bins:
                self.scene.remove_object(rigid_bin.name)
            self.bins.clear()

        self.on_conveyor = None

    def pre_step(self, time_step_index, simulation_time) -> None:
        """Spawn a new randomly oriented bin if the previous bin has been placed."""
        spawn_new = False
        if self.on_conveyor is None:
            spawn_new = True
        else:
            (x, y, z), _ = self.on_conveyor.get_world_pose()
            is_on_conveyor = y > 0.0 and -0.4 < x and x < 0.4
            if not is_on_conveyor:
                spawn_new = True

        if spawn_new:
            name = "bin_{}".format(len(self.bins))
            prim_path = self.env_path + "/bins/{}".format(name)
            add_reference_to_stage(usd_path=self.assets.small_klt_usd, prim_path=prim_path)
            self.on_conveyor = self.scene.add(CortexRigidPrim(name=name, prim_path=prim_path))

            self._spawn_bin(self.on_conveyor)
            self.bins.append(self.on_conveyor)


def main():
    world = CortexWorld()

    env_path = "/World/Ur10Table"
    ur10_assets = Ur10Assets()
    add_reference_to_stage(usd_path=ur10_assets.ur10_table_usd, prim_path=env_path)
    add_reference_to_stage(usd_path=ur10_assets.background_usd, prim_path="/World/Background")
    background_prim = XFormPrim(
        "/World/Background",
        positions=np.array([[10.00, 2.00, -1.18180]]),
        orientations=np.array([[0.7071, 0, 0, 0.7071]]),
    )
    robot = world.add_robot(CortexUr10(name="robot", prim_path="{}/ur10".format(env_path)))

    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/FlipStationSphere",
            name="flip_station_sphere",
            position=np.array([0.73, 0.76, -0.13]),
            radius=0.2,
            visible=False,
        )
    )
    robot.register_obstacle(obs)
    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/NavigationDome",
            name="navigation_dome_obs",
            position=[-0.031, -0.018, -1.086],
            radius=1.1,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    az = np.array([1.0, 0.0, -0.3])
    ax = np.array([0.0, 1.0, 0.0])
    ay = np.cross(az, ax)
    R = math_util.pack_R(ax, ay, az)
    quat = math_util.matrix_to_quat(R)
    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationBarrier",
            name="navigation_barrier_obs",
            position=[0.471, 0.276, -0.463 - 0.1],
            orientation=quat,
            radius=0.5,
            height=0.9,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationFlipStation",
            name="navigation_flip_station_obs",
            position=np.array([0.766, 0.755, -0.5]),
            radius=0.5,
            height=0.5,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    world.add_task(BinStackingTask(env_path, ur10_assets))
    world.add_decider_network(behavior.make_decider_network(robot, print_diagnostics))

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
