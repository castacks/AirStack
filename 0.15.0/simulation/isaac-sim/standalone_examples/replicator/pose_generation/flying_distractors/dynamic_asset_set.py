# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import itertools
import math
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

from .collision_box import CollisionBox


class DynamicAssetSet(ABC):
    """Container class to hold and manage dynamic assets, providing an API to keep assets in motion within a collision
       box, and to allow various properties of the assets to be randomized.

    Args:
        set_prim_path (str): prim path of the parent Prim to create, which contains all the assets in the asset set as
                             its children.
        set_name (str): name of the parent prim in the scene.
        asset_prim_path_base_prefix (str): prefix of what the assets are called in the stage (prim path base name).
        asset_name_prefix (str): prefix of the assets' names in the scene.
        num_assets (int): number of assets in the asset set.
        collision_box (CollisionBox): collision box in which to place assets, and allow assets to move within.
        scale (Optional[np.ndarray], optional): local scale to be applied to each asset's dimensions. Shape is (3, ).
                                                Defaults to None, which means left unchanged.
        mass (Optional[float], optional): mass of each asset in kg. Defaults to None.
        fraction_glass (int, optional): fraction of assets for which glass material should be applied.
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
        self.world = World.instance()

        self.set_prim_path = set_prim_path
        self.set_name = set_name
        self.asset_prim_path_base_prefix = asset_prim_path_base_prefix
        self.asset_name_prefix = asset_name_prefix
        self.num_assets = num_assets
        self.collision_box = collision_box
        self.scale = scale
        self.mass = mass
        self.fraction_glass = fraction_glass
        self.asset_count = 0
        self.asset_names = []
        self.glass_asset_paths = []
        self.nonglass_asset_paths = []
        self.glass_assets = []
        self.nonglass_assets = []
        self.glass_mats = []
        self._rigid_prims = None

    def _create_random_dynamic_asset_set(self):
        """Create self.num_assets assets and add them to the dynamic asset set."""

        self.world.stage.DefinePrim(self.set_prim_path, "Xform")

        num_glass = math.floor(self.num_assets * self.fraction_glass)

        for i in range(self.num_assets):

            if i < num_glass:
                self._create_random_dynamic_asset(glass=True)
            else:
                self._create_random_dynamic_asset()

    @abstractmethod
    def _create_random_dynamic_asset(self, glass=False):
        pass

    def apply_force_to_assets(self, force_limit):
        """Apply a force in a random direction to each asset in the dynamic asset set.

        Args:
            force_limit (float): maximum force component to apply.
        """
        if self._rigid_prims is None:
            self._rigid_prims = []
            for path in itertools.chain(self.glass_asset_paths, self.nonglass_asset_paths):
                rigid_prim = RigidPrim(path)
                rigid_prim.initialize()
                self._rigid_prims.append(rigid_prim)

        for rigid_prim in self._rigid_prims:
            random_force = np.random.uniform(-force_limit, force_limit, 3).tolist()
            rigid_prim.apply_forces_and_torques_at_pos(random_force, is_global=False)

    def randomize_glass_color(self):
        """Randomize the color of the assets in the dynamic asset set with a glass material applied."""

        for asset in itertools.chain(self.glass_assets):
            try:
                glass_mat = asset.get_applied_visual_materials()[0]
            except:
                glass_mat = asset.get_applied_visual_material()
            glass_mat.set_color(np.random.rand(3))

    def reset_position(self):
        """Reset the positions of assets in the dynamic asset set. The positions at which to place assets are randomly
        chosen such that they are within the collision box.
        """

        for asset in itertools.chain(self.glass_assets, self.nonglass_assets):
            position = self.collision_box.get_random_position()
            asset.set_world_pose(position)
