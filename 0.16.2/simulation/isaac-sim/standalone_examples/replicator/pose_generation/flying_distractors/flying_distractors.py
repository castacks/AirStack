# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import itertools

from isaacsim.core.api import World

from .dynamic_object_set import DynamicObjectSet
from .dynamic_shape_set import DynamicShapeSet


class FlyingDistractors:
    """Container class to hold and manage both dynamic shape sets and dynamic object sets simultaneously. This class
    provides an API to keep assets in each asset set in motion within their respective collision boxes, to show/hide
    the assets of all the asset sets managed by this class, and to allow various properties of the assets of all the
    asset sets managed by this class to be randomized.
    """

    def __init__(self):
        self.world = World.instance()
        self.shape_sets = []
        self.object_sets = []

    def add(self, asset_set):
        """Add an asset set to be managed by this FlyingDistractors object.

        Args:
            asset_set (Union[DynamicShapeSet, DynamicObjectSet]): the asset set to add.

        Raises:
            Exception: if asset_set is neither a DynamicShapeSet nor a DynamicObjectSet.
        """

        if isinstance(asset_set, DynamicShapeSet):
            self.shape_sets.append(asset_set)
        elif isinstance(asset_set, DynamicObjectSet):
            self.object_sets.append(asset_set)
        else:
            raise Exception("The asset set provided is not of type DynamicShapeSet or DynamicObjectSet")

    def set_visible(self, visible):
        """Sets the visibility of all assets contained in the managed asset sets.

        Args:
            visible (bool): flag to set the visibility of all assets contained in the managed asset sets.
        """

        for asset_set in itertools.chain(self.shape_sets, self.object_sets):
            for asset_name in asset_set.asset_names:
                object_xform = self.world.scene.get_object(asset_name)
                try:
                    object_xform.set_visibilities([visible])
                except:
                    object_xform.set_visibility(visible)

    def reset_asset_positions(self):
        """Reset the positions of all assets contained in the managed asset sets to be within its corresponding
        collision box.
        """

        for asset_set in itertools.chain(self.shape_sets, self.object_sets):
            asset_set.reset_position()

    def apply_force_to_assets(self, force_limit):
        """Apply random forces to all assets contained in the managed asset sets.

        Args:
            force_limit (float): maximum force component to apply.
        """

        for asset_set in itertools.chain(self.shape_sets, self.object_sets):
            asset_set.apply_force_to_assets(force_limit)

    def randomize_asset_glass_color(self):
        """Randomize color of assets in the managed asset sets with glass material applied."""

        for asset_set in itertools.chain(self.shape_sets, self.object_sets):
            asset_set.randomize_glass_color()
