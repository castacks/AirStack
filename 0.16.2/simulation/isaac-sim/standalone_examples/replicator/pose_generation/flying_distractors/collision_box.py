# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.prims import define_prim
from pxr import Usd, UsdGeom


class CollisionBox(XFormPrim):
    """Creates a fixed box with collisions enabled, and provides an API to determine world coordinates of a random
       location in the interior of the collision box.

    Args:
        prim_path (str): top-level prim path (of the collision box) of the Prim to encapsulate or create.
        name (str): shortname to be used as a key by Scene class. Note: needs to be unique if the object is added to the
                    Scene.
        position (Optional[np.ndarray], optional): position in the world frame of the collision box. Shape is (3, ).
                                                   Defaults to None, which means left unchanged.
        translation (Optional[np.ndarray], optional): translation in the local frame of the collision box (with respect
                                                      to its parent prim). Shape is (3, ). Defaults to None, which means
                                                      left unchanged.
        orientation (Optional[np.ndarray], optional): quaternion orientation in the world/local frame of the collision
                                                      box (depends if translation or position is specified). Quaternion
                                                      is scalar-first (w, x, y, z). Shape is (4, ). Defaults to None,
                                                      which means left unchanged.
        scale (Optional[np.ndarray], optional): local scale to be applied to the collision box's dimensions. Shape is
                                                (3, ). Defaults to None, which means left unchanged.
        width (float): width of the collision box interior in world units (if unrotated, corresponds to x direction).
                       Defaults to 1.0.
        height (float): height of the collision box interior in world units (if unrotated, corresponds to y direction).
                        Defaults to 1.0.
        depth (float): depth of the collision box interior in world units (if unrotated, corresponds to z direction).
                       Defaults to 1.0.
        thickness (float, optional): thickness of the collision box walls in world units. Defaults to 0.2.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to False.
    """

    def __init__(
        self,
        prim_path: str,
        name: str,
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        width: float = 1.0,
        height: float = 1.0,
        depth: float = 1.0,
        thickness: float = 0.2,
        visible: bool = False,
    ):
        self.world = World.instance()

        define_prim(prim_path=prim_path, prim_type="Xform")
        XFormPrim.__init__(
            self,
            prim_paths_expr=prim_path,
            name=name,
            positions=None if position is None else np.array([position]),
            translations=None if translation is None else np.array([translation]),
            orientations=None if orientation is None else np.array([orientation]),
            scales=None if scale is None else np.array([scale]),
            visibilities=None if visible is None else np.array([visible]),
        )

        self.width = width
        self.height = height
        self.depth = depth
        self.thickness = thickness
        self.visible = visible
        self._create_collision_box()

    def _create_face(self, suffix, translation, size):
        """Create a face/wall of the Collision Box, which has collisions enabled.

        Args:
            suffix (str): suffix used for the name of the face so it can be retrieved from the scene. The name of the
                          face has the form "{collision_box_name}_{suffix}"
            translation (np.ndarray): translation of the center of the face (wall) from the center of the Collision
                                      Box, in stage units. Shape is (3, ).
            size (np.ndarray): dimensions of the face (wall) in the X, Y, and Z directions. Dimensions are in stage
                               units. Shape is (3, ).
        """
        face_name = f"{self.name}_{suffix}"
        face_path = f"{self.prim_paths[0]}/{face_name}"
        face_cuboid = FixedCuboid(
            prim_path=face_path,  # The prim path of the cube in the USD stage
            name=face_name,  # The unique name used to retrieve the object from the scene later on
            translation=translation,  # Using the current stage units which is cms by default.
            scale=size,  # most arguments accept mainly numpy arrays.
            size=1.0,
            visible=self.visible,
        )
        self.world.scene.add(face_cuboid)

    def _create_collision_box(self):
        """Create a Collision Box. The Collision Box consists of 6 faces/walls forming a static box-like volume. Each
        wall of the Collision Box has collisions enabled.
        """

        dx = self.width / 2.0 + self.thickness / 2.0
        dy = self.height / 2.0 + self.thickness / 2.0
        dz = self.depth / 2.0 + self.thickness / 2.0

        floor_center = np.array([0, 0, -dz])
        floor_dimensions = np.array([self.width, self.height, self.thickness])
        self._create_face("floor", floor_center, floor_dimensions)

        ceiling_center = np.array([0, 0, +dz])
        ceiling_dimensions = np.array([self.width, self.height, self.thickness])
        self._create_face("ceiling", ceiling_center, ceiling_dimensions)

        left_wall_center = np.array([dx, 0, 0])
        left_wall_dimensions = np.array([self.thickness, self.height, self.depth])
        self._create_face("left_wall", left_wall_center, left_wall_dimensions)

        right_wall_center = np.array([-dx, 0, 0])
        right_wall_dimensions = np.array([self.thickness, self.height, self.depth])
        self._create_face("right_wall", right_wall_center, right_wall_dimensions)

        front_wall_center = np.array([0, dy, 0])
        front_wall_dimensions = np.array([self.width, self.thickness, self.depth])
        self._create_face("front_wall", front_wall_center, front_wall_dimensions)

        back_wall_center = np.array([0, -dy, 0])
        back_wall_dimensions = np.array([self.width, self.thickness, self.depth])
        self._create_face("back_wall", back_wall_center, back_wall_dimensions)

    def get_random_local_translation(self):
        """Get a random translation within the Collision Box in local coordinates. Translations are within the
           volumetric region contained by the inner walls of the Collision Box. The local coordinate frame is considered
           to be the frame of the prim at self.prim_path (center of the Collision Box).

        Returns:
            np.ndarray: random translation within the Collision Box in the local frame of the Collision Box. Shape is
                        (3, ).
        """

        dim_fractions = np.random.rand(3)

        tx = dim_fractions[0] * self.width - self.width / 2.0
        ty = dim_fractions[1] * self.height - self.height / 2.0
        tz = dim_fractions[2] * self.depth - self.depth / 2.0

        translation = np.array([tx, ty, tz])

        return translation

    def get_random_position(self):
        """Get a random position within the Collision Box in world coordinates. Positions are within the volumetric
           region contained by the inner walls of the Collision Box.

        Returns:
            np.ndarray: random position within the Collision Box in the world frame. Shape is (3, ).
        """

        box_prim = self.world.stage.GetPrimAtPath(self.prim_paths[0])

        box_transform_matrix = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        box_to_world = np.transpose(box_transform_matrix)

        random_local_translation = self.get_random_local_translation()

        random_local_translation_homogenous = np.pad(random_local_translation, ((0, 1)), constant_values=1.0)

        position_homogenous = box_to_world @ random_local_translation_homogenous

        position = position_homogenous[:-1]

        return position
