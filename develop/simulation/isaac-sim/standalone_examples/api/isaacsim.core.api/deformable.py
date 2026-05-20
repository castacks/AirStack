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
import argparse
import sys

import carb
import isaacsim.core.utils.deformable_mesh_utils as deformableMeshUtils
import numpy as np
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.deformable_material import DeformableMaterial
from isaacsim.core.prims import DeformablePrim, SingleDeformablePrim
from isaacsim.storage.native import get_assets_root_path
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, UsdGeom, UsdLux

# The example shows how to create and manipulate environments with deformable prim through the DeformablePrim
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


class DeformableExample:
    def __init__(self):
        self._array_container = torch.Tensor
        self.my_world = World(stage_units_in_meters=1.0, backend="torch", device="cuda")
        self.stage = simulation_app.context.get_stage()
        self.num_envs = 10
        self.dimx = 5
        self.dimy = 5
        self.my_world.scene.add_default_ground_plane()
        self.initial_positions = None
        self.makeEnvs()

    def makeEnvs(self):
        for i in range(self.num_envs):
            init_loc = Gf.Vec3f(i * 2 - self.num_envs, 0.0, 0.0)
            env_scope = UsdGeom.Scope.Define(self.stage, "/World/Envs")
            env_path = "/World/Envs/Env" + str(i)
            env = UsdGeom.Xform.Define(self.stage, env_path)
            physicsUtils.set_or_add_translate_op(UsdGeom.Xformable(env), init_loc)

            mesh_path = env.GetPrim().GetPath().AppendChild("deformable")
            skin_mesh = UsdGeom.Mesh.Define(self.stage, mesh_path)
            tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(8)
            skin_mesh.GetPointsAttr().Set(tri_points)
            skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            physicsUtils.setup_transform_as_scale_orient_translate(skin_mesh)
            physicsUtils.set_or_add_translate_op(skin_mesh, (0.0, 0.0, 2.0))
            physicsUtils.set_or_add_orient_op(skin_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15 * i).GetQuat())
            deformable_material_path = env.GetPrim().GetPath().AppendChild("deformableMaterial").pathString
            self.deformable_material = DeformableMaterial(
                prim_path=deformable_material_path,
                dynamic_friction=0.5,
                youngs_modulus=5e4,
                poissons_ratio=0.4,
                damping_scale=0.1,
                elasticity_damping=0.1,
            )

            self.deformable = SingleDeformablePrim(
                name="deformablePrim" + str(i),
                prim_path=str(mesh_path),
                deformable_material=self.deformable_material,
                vertex_velocity_damping=0.0,
                sleep_damping=1.0,
                sleep_threshold=0.05,
                settling_threshold=0.1,
                self_collision=True,
                self_collision_filter_distance=0.05,
                solver_position_iteration_count=20,
                kinematic_enabled=False,
                simulation_hexahedral_resolution=2,
                collision_simplification=True,
            )
            self.my_world.scene.add(self.deformable)

        # create a view to deal with all the deformables
        self.deformableView = DeformablePrim(prim_paths_expr="/World/Envs/Env*/deformable", name="deformableView1")
        self.my_world.scene.add(self.deformableView)
        self.my_world.reset(soft=False)
        # mesh data is available only after cooking
        # rest_points are represented with respect to the env positions, but simulation_mesh_nodal_positions can be either global or local positions
        # However, because we don't currently consider subspace root path with World/SimulationContext initialization, the environment xforms are not identified
        # below and the following call will be positions w.r.t to a global frame.
        self.initial_positions = self.deformableView.get_simulation_mesh_nodal_positions().cpu()
        self.initial_velocities = self.deformableView.get_simulation_mesh_nodal_velocities().cpu()
        # print(self.initial_positions)
        # self.initial_positions = self.deformableView.get_simulation_mesh_rest_points().cpu()
        # for i in range(self.num_envs):
        #     self.initial_positions[i] += torch.tensor([i * 2, 0.0, 2.0])
        #     print(self.initial_positions[i])

    def play(self):
        while simulation_app.is_running():
            if self.my_world.is_playing():
                # deal with sim re-initialization after restarting sim
                if self.my_world.current_time_step_index == 1:
                    # initialize simulation views
                    self.my_world.reset(soft=False)

            self.my_world.step(render=True)

            if self.my_world.current_time_step_index == 200:
                for i in range(self.num_envs):
                    print(
                        "deformable {} average height = {:.2f}".format(
                            i, self.deformableView.get_simulation_mesh_nodal_positions()[i, :, 2].mean()
                        )
                    )
                    print(
                        "deformable {} average vertical speed = {:.2f}".format(
                            i, self.deformableView.get_simulation_mesh_nodal_velocities()[i, :, 2].mean()
                        )
                    )

            # reset some random environments
            if self.my_world.current_time_step_index % 500 == 1:
                indices = torch.tensor(
                    np.random.choice(range(self.num_envs), self.num_envs // 2, replace=False), dtype=torch.long
                )
                new_positions = self.initial_positions[indices] + torch.tensor([0, 0, 5])
                new_velocities = self.initial_velocities[indices] + torch.tensor([0, 0, 3])
                self.deformableView.set_simulation_mesh_nodal_positions(new_positions, indices)
                self.deformableView.set_simulation_mesh_nodal_velocities(new_velocities, indices)
                updated_positions = self.deformableView.get_simulation_mesh_nodal_positions()
                updated_velocities = self.deformableView.get_simulation_mesh_nodal_velocities()
                for i in indices:
                    print("reset index {} average height = {:.2f}".format(i, updated_positions[i, :, 2].mean()))
                    print(
                        "reset index {} average vertical speed = {:.2f}".format(i, updated_velocities[i, :, 2].mean())
                    )

        simulation_app.close()


DeformableExample().play()
