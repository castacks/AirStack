# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import random
from typing import List, Optional

import numpy as np
from isaacsim.core.api.materials.omni_glass import OmniGlass

from .collision_box import CollisionBox
from .dynamic_asset_set import DynamicAssetSet
from .dynamic_object import DynamicObject


class DynamicObjectSet(DynamicAssetSet):
    """Container class to hold and manage dynamic objects, providing an API to keep objects in motion within a collision
       box, and to allow various properties of the assets to be randomized. Please note that this class assumes that
       each referenced asset in usd_path_list has only a single mesh prim defining its geometry.

    Args:
        set_prim_path (str): prim path of the parent Prim to create, which contains all the objects in the object set
                             as its children.
        set_name (str): name of the parent prim in the scene.
        usd_path_list (List[str]): list of possible USD reference paths that the prims of each dynamic object in the
                                   dynamic object set refer to.
        mesh_list (List[str]): list of prim path base names for underlying mesh prims. Each base name in mesh_list
                               corresponds to the mesh prim of the referenced asset in usd_path_list.
        asset_prim_path_base_prefix (str): prefix of what the objects are called in the stage (prim path base name).
        asset_name_prefix (str): prefix of the objects' names in the scene.
        num_assets (int): number of objects in the object set.
        collision_box (CollisionBox): collision box in which to place objects, and allow objects to move within.
        scale (Optional[np.ndarray], optional): local scale to be applied to each object's dimensions. Shape is (3, ).
                                                Defaults to None, which means left unchanged.
        mass (Optional[float], optional): mass of each object in kg. Defaults to None.
        fraction_glass (int, optional): fraction of objects for which glass material should be applied.
    """

    def __init__(
        self,
        set_prim_path: str,
        set_name: str,
        usd_path_list: List[str],
        mesh_list: List[str],
        asset_prim_path_base_prefix: str,
        asset_name_prefix: str,
        num_assets: int,
        collision_box: CollisionBox,
        scale: Optional[np.ndarray] = None,
        mass: Optional[float] = None,
        fraction_glass: float = 0.0,
    ):

        self.usd_path_list = usd_path_list
        self.mesh_list = mesh_list
        self.glass_object_mesh_paths = []
        self.nonglass_object_mesh_paths = []

        if len(usd_path_list) != len(mesh_list):
            raise Exception("usd_path_list and mesh_list must contain the same number of elements")

        self.mesh_map = self._create_mesh_map(usd_path_list, mesh_list)

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

    def _create_mesh_map(self, usd_path_list, mesh_list):
        """Gets a mapping from USD reference paths to the base name of the corresponding mesh prim in the referenced USD
           file.

        Args:
            usd_path_list (List[str]): List of possible USD reference paths that the prims of each dynamic object in the
                                       dynamic object set refer to.
            mesh_list (List[str]): List of prim path base names for underlying mesh prims. Each base name in mesh_list
                                   corresponds to the mesh prim of the referenced asset in usd_path_list.

        Returns:
            Dict: Mapping from USD reference paths to the base name of the corresponding mesh prim in the referenced USD
                  file.
        """

        mesh_map = {}

        for usd_path, mesh_name in zip(usd_path_list, mesh_list):
            mesh_map[usd_path] = mesh_name

        return mesh_map

    def _create_random_dynamic_asset(self, glass=False):
        """Creates a random dynamic object and adds it to the scene. The reference path of the object is randomly chosen
           from self.usd_path_list.

        Args:
            glass (bool, optional): flag to specify whether the created object should have a glass material applied.
                                    Defaults to False.
        """

        object_name = f"{self.asset_name_prefix}_{self.asset_count}"

        if glass:
            object_path = f"{self.set_prim_path}/{self.asset_prim_path_base_prefix}_{self.asset_count}"
        else:
            object_path = f"{self.set_prim_path}/{self.asset_prim_path_base_prefix}_nonglass_{self.asset_count}"

        usd_path = random.choice(self.usd_path_list)
        mesh_path = f"{object_path}/{self.mesh_map[usd_path]}"

        position = self.collision_box.get_random_position()

        dynamic_prim = DynamicObject(
            usd_path=usd_path,
            prim_path=object_path,
            mesh_path=mesh_path,
            name=object_name,
            position=position,
            scale=self.scale,
            mass=self.mass,
        )

        self.asset_names.append(object_name)

        if glass:
            color = np.random.rand(3)
            material = OmniGlass(
                object_path + "_glass",
                name=object_name + "_glass",
                ior=1.25,
                depth=0.001,
                thin_walled=False,
                color=color,
            )
            self.glass_mats.append(material)
            dynamic_prim.apply_visual_materials([material])
            self.glass_asset_paths.append(object_path)
            self.glass_assets.append(dynamic_prim)
            self.glass_object_mesh_paths.append(mesh_path)
        else:
            self.nonglass_asset_paths.append(object_path)
            self.nonglass_assets.append(dynamic_prim)
            self.nonglass_object_mesh_paths.append(mesh_path)

        self.world.scene.add(dynamic_prim)
        self.asset_count += 1
