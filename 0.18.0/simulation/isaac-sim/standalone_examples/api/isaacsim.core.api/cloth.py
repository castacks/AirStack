# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
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
import numpy as np
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.prims import ClothPrim, SingleClothPrim, SingleParticleSystem
from isaacsim.storage.native import get_assets_root_path
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, UsdGeom

# The example shows how to create and manipulate environments with particle cloth through the ClothPrim
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


class ParticleClothExample:
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
            env_path = "/World/Env" + str(i)
            env = UsdGeom.Xform.Define(self.stage, env_path)
            # set up the geometry
            cloth_path = env.GetPrim().GetPath().AppendChild("cloth")
            plane_mesh = UsdGeom.Mesh.Define(self.stage, cloth_path)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx=5, dimy=5, scale=1.0)
            if self.initial_positions is None:
                self.initial_positions = torch.zeros((self.num_envs, len(tri_points), 3))
            plane_mesh.GetPointsAttr().Set(tri_points)
            plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            init_loc = Gf.Vec3f(i * 2, 0.0, 2.0)
            physicsUtils.setup_transform_as_scale_orient_translate(plane_mesh)
            physicsUtils.set_or_add_translate_op(plane_mesh, init_loc)
            physicsUtils.set_or_add_orient_op(plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15 * i).GetQuat())
            self.initial_positions[i] = torch.tensor(init_loc) + torch.tensor(plane_mesh.GetPointsAttr().Get())
            particle_system_path = env.GetPrim().GetPath().AppendChild("particleSystem")
            particle_material_path = env.GetPrim().GetPath().AppendChild("particleMaterial")

            self.particle_material = ParticleMaterial(
                prim_path=str(particle_material_path), drag=0.1, lift=0.3, friction=0.6
            )
            radius = 0.5 * (0.6 / 5.0)
            restOffset = radius
            contactOffset = restOffset * 1.5
            self.particle_system = SingleParticleSystem(
                prim_path=str(particle_system_path),
                simulation_owner=self.my_world.get_physics_context().prim_path,
                rest_offset=restOffset,
                contact_offset=contactOffset,
                solid_rest_offset=restOffset,
                fluid_rest_offset=restOffset,
                particle_contact_offset=contactOffset,
            )
            # note that no particle material is applied to the particle system at this point.
            # this can be done manually via self.particle_system.apply_particle_material(self.particle_material)
            # or to pass the material to the clothPrim which binds it internally to the particle system
            self.cloth = SingleClothPrim(
                name="clothPrim" + str(i),
                prim_path=str(cloth_path),
                particle_system=self.particle_system,
                particle_material=self.particle_material,
            )
            self.my_world.scene.add(self.cloth)

        # create a view to deal with all the cloths
        self.clothView = ClothPrim(prim_paths_expr="/World/Env*/cloth", name="clothView1")
        self.my_world.scene.add(self.clothView)
        self.my_world.reset(soft=False)

    def play(self):
        reset_needed = False
        while simulation_app.is_running():
            if self.my_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.my_world.is_playing():
                # deal with sim re-initialization after restarting sim
                if reset_needed:
                    # initialize simulation views
                    self.my_world.reset(soft=False)
                    reset_needed = False

            self.my_world.step(render=True)

            if self.my_world.current_time_step_index % 50 == 49:
                for i in range(self.num_envs):
                    print(
                        "cloth {} average height = {:.2f}".format(
                            i, self.clothView.get_world_positions()[i, :, 2].mean()
                        )
                    )
                if args.test is True:
                    break

            # reset some random environments
            if self.my_world.current_time_step_index % 200 == 1:
                indices = torch.tensor(
                    np.random.choice(range(self.num_envs), self.num_envs // 2, replace=False), dtype=torch.long
                )
                new_positions = self.initial_positions[indices] + torch.tensor([0, 0, 5])
                self.clothView.set_world_positions(new_positions, indices)
                updated_positions = self.clothView.get_world_positions()
                for i in indices:
                    print("reset index {} average height = {:.2f}".format(i, updated_positions[i, :, 2].mean()))

        simulation_app.close()


ParticleClothExample().play()
