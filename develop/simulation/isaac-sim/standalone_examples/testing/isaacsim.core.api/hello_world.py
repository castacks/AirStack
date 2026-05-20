# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import torch
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
from abc import abstractmethod

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.cloner import GridCloner
from isaacsim.core.prims import RigidPrim

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


class HelloWorld(BaseTask):
    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """[summary]"""
        BaseTask.__init__(self, name=name, offset=offset)

        self._num_envs = num_envs
        self._env_spacing = env_spacing

        self._cloner = GridCloner(self._env_spacing)

        return

    def set_up_scene(self, scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """

        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        task_object = self.set_object()
        prim_paths = self._cloner.generate_paths("/World/object", self._num_envs)
        self._cloner.clone(
            source_prim_path=task_object.prim_path,
            prim_paths=prim_paths,
            position_offsets=np.array([[0, 0, 1.0]] * self._num_envs),
        )
        self._object = RigidPrim(prim_paths_expr=f"/World/object_[0-{self._num_envs-1}]", name="object_view")
        scene.add(self._object)

        return

    @abstractmethod
    def set_object(self):
        raise NotImplementedError

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        object_positions, _ = self._object.get_world_poses()
        object_velocities = self._object.get_velocities()

        observations = {self._object.name: {"positions": object_positions, "velocities": object_velocities}}
        return observations

    def calculate_metrics(self) -> None:
        """[summary]"""
        return torch.zeros(self._num_envs, device=self._device)

    def is_done(self) -> None:
        """[summary]"""
        return torch.zeros(self._num_envs, device=self._device)


class HelloWorldSphere(HelloWorld):
    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """[summary]"""
        super().__init__(name=name, num_envs=num_envs, env_spacing=env_spacing, offset=offset)

    def set_object(self):
        radius = 0.1
        density = 1000.0

        return DynamicSphere(prim_path="/World/object_0", name="object_0", radius=radius, mass=None, density=density)


class HelloWorldCuboid(HelloWorld):
    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """[summary]"""
        super().__init__(name=name, num_envs=num_envs, env_spacing=env_spacing, offset=offset)

    def set_object(self):
        size = np.array([0.2, 0.2, 0.2])
        density = 1000.0

        return DynamicCuboid(
            prim_path="/World/object_0", name="object_0", size=1.0, scale=size, mass=None, density=density
        )


num_envs = 10
env_spacing = 1
physicsscene_path = "/physicsScene"

my_world = World(stage_units_in_meters=1.0, physics_prim_path=physicsscene_path, backend="torch", device="cuda:0")
my_task = HelloWorldSphere(name="hello_world", num_envs=num_envs, env_spacing=env_spacing)
my_world.add_task(my_task)
my_world.reset()

reset_needed = False
while simulation_app.is_running():
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        # deal with sim re-initialization after restarting sim
        if reset_needed:
            # initialize simulation views
            my_world.reset(soft=True)
            reset_needed = False
        observations = my_world.get_observations()

    my_world.step(render=True)
    if args.test is True:
        break

simulation_app.close()
