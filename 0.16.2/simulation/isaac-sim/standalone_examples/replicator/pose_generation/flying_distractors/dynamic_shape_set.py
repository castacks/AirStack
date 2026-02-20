# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import random
from typing import Optional

import numpy as np
from isaacsim.core.api.materials.omni_glass import OmniGlass
from isaacsim.core.api.objects import DynamicCapsule, DynamicCone, DynamicCuboid, DynamicCylinder, DynamicSphere

from .collision_box import CollisionBox
from .dynamic_asset_set import DynamicAssetSet


class DynamicShapeSet(DynamicAssetSet):
    """Container class to hold and manage dynamic shapes, providing an API to keep shapes in motion within a collision
       box, and to allow various properties of the shapes to be randomized.

    Args:
        set_prim_path (str): prim path of the parent Prim to create, which contains all the shapes in the shape set
                             as its children.
        set_name (str): name of the parent prim in the scene.
        asset_prim_path_base_prefix (str): prefix of what the shapes are called in the stage (prim path base name).
        asset_name_prefix (str): prefix of the shapes' names in the scene.
        num_assets (int): number of shapes in the shape set.
        collision_box (CollisionBox): collision box in which to place shapes, and allow shapes to move within.
        scale (Optional[np.ndarray], optional): local scale to be applied to each shape's dimensions. Shape is (3, ).
                                                Defaults to None, which means left unchanged.
        mass (Optional[float], optional): mass of each shape in kg. Defaults to None.
        fraction_glass (int, optional): fraction of shapes for which glass material should be applied.
    """

    def __init__(
        self,
        set_prim_path: str,
        set_name: str,
        asset_prim_path_base_prefix: str,
        asset_name_prefix: str,
        num_assets: int,
        collision_box: CollisionBox,
        scale: Optional[np.ndarray] = None,
        mass: Optional[float] = None,
        fraction_glass: float = 0.0,
    ):

        super().__init__(
            set_prim_path,
            set_name,
            asset_prim_path_base_prefix,
            asset_name_prefix,
            num_assets,
            collision_box,
            scale,
            mass,
            fraction_glass,
        )

        self._create_random_dynamic_asset_set()

    def _create_random_dynamic_asset(self, glass=False):
        """Creates a random dynamic shape (Cuboid, Sphere, Cylinder, Cone, or Capsule) and adds it to the scene.

        Args:
            glass (bool, optional): flag to specify whether the created shape should have a glass material applied.
                                    Defaults to False.
        """

        prim_type = [DynamicCapsule, DynamicCone, DynamicCuboid, DynamicCylinder, DynamicSphere]

        shape_name = f"{self.asset_name_prefix}_{self.asset_count}"

        if glass:
            shape_path = f"{self.set_prim_path}/{self.asset_prim_path_base_prefix}_{self.asset_count}"
        else:
            shape_path = f"{self.set_prim_path}/{self.asset_prim_path_base_prefix}_nonglass_{self.asset_count}"

        position = self.collision_box.get_random_position()

        shape_prim = random.choice(prim_type)(
            prim_path=shape_path,  # The prim path of the cube in the USD stage
            name=shape_name,  # The unique name used to retrieve the object from the scene later on
            position=position,  # Using the current stage units which is meters by default.
            scale=self.scale,
            mass=self.mass,
        )

        self.asset_names.append(shape_name)

        if glass:
            color = np.random.rand(3)
            material = OmniGlass(
                shape_path + "_glass", name=shape_name + "_glass", ior=1.25, depth=0.001, thin_walled=False, color=color
            )
            self.glass_mats.append(material)
            shape_prim.apply_visual_material(material)
            self.glass_asset_paths.append(shape_path)
            self.glass_assets.append(shape_prim)
        else:
            self.nonglass_asset_paths.append(shape_path)
            self.nonglass_assets.append(shape_prim)

        self.world.scene.add(shape_prim)
        self.asset_count += 1
