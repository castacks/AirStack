# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import random

import numpy as np
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


# Add transformation properties to the prim (if not already present)
def set_transform_attributes(prim, location=None, orientation=None, rotation=None, scale=None):
    if location is not None:
        if not prim.HasAttribute("xformOp:translate"):
            UsdGeom.Xformable(prim).AddTranslateOp()
        prim.GetAttribute("xformOp:translate").Set(location)
    if orientation is not None:
        if not prim.HasAttribute("xformOp:orient"):
            UsdGeom.Xformable(prim).AddOrientOp()
        prim.GetAttribute("xformOp:orient").Set(orientation)
    if rotation is not None:
        if not prim.HasAttribute("xformOp:rotateXYZ"):
            UsdGeom.Xformable(prim).AddRotateXYZOp()
        prim.GetAttribute("xformOp:rotateXYZ").Set(rotation)
    if scale is not None:
        if not prim.HasAttribute("xformOp:scale"):
            UsdGeom.Xformable(prim).AddScaleOp()
        prim.GetAttribute("xformOp:scale").Set(scale)


# Enables collisions with the asset (without rigid body dynamics the asset will be static)
def add_colliders(root_prim):
    # Iterate descendant prims (including root) and add colliders to mesh or primitive types
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            # Physics
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
            # PhysX
            if not desc_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(desc_prim)
            else:
                physx_collision_api = PhysxSchema.PhysxCollisionAPI(desc_prim)
            # Set PhysX specific properties
            physx_collision_api.CreateContactOffsetAttr(0.001)
            physx_collision_api.CreateRestOffsetAttr(0.0)

        # Add mesh specific collision properties only to mesh types
        if desc_prim.IsA(UsdGeom.Mesh):
            # Add mesh collision properties to the mesh (e.g. collider aproximation type)
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set("convexHull")


# Check if prim (or its descendants) has colliders
def has_colliders(root_prim):
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


# Enables rigid body dynamics (physics simulation) on the prim
def add_rigid_body_dynamics(prim, disable_gravity=False, angular_damping=None):
    if has_colliders(prim):
        # Physics
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        else:
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        rigid_body_api.CreateRigidBodyEnabledAttr(True)
        # PhysX
        if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        else:
            physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(prim)
        physx_rigid_body_api.GetDisableGravityAttr().Set(disable_gravity)
        if angular_damping is not None:
            physx_rigid_body_api.CreateAngularDampingAttr().Set(angular_damping)
    else:
        print(f"Prim '{prim.GetPath()}' has no colliders. Skipping rigid body dynamics properties.")


# Add dynamics properties to the prim (if mesh or primitive) (rigid body to root + colliders to the meshes)
# https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/rigid-bodies.html#rigid-body-simulation
def add_colliders_and_rigid_body_dynamics(prim, disable_gravity=False):
    # Add colliders to mesh or primitive types of the descendants of the prim (including root)
    add_colliders(prim)
    # Add rigid body dynamics properties (to the root only) only if it has colliders
    add_rigid_body_dynamics(prim, disable_gravity=disable_gravity)


# Createa  collision box area wrapping the given working area with origin in (0, 0, 0) with thickness towards outside
def create_collision_box_walls(stage, path, width, depth, height, thickness=0.5, visible=False):
    # Define the walls (name, location, size) with thickness towards outside of the working area
    walls = [
        ("floor", (0, 0, (height + thickness) / -2.0), (width, depth, thickness)),
        ("ceiling", (0, 0, (height + thickness) / 2.0), (width, depth, thickness)),
        ("left_wall", ((width + thickness) / -2.0, 0, 0), (thickness, depth, height)),
        ("right_wall", ((width + thickness) / 2.0, 0, 0), (thickness, depth, height)),
        ("front_wall", (0, (depth + thickness) / 2.0, 0), (width, thickness, height)),
        ("back_wall", (0, (depth + thickness) / -2.0, 0), (width, thickness, height)),
    ]
    for name, location, size in walls:
        prim = stage.DefinePrim(f"{path}/{name}", "Cube")
        scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        set_transform_attributes(prim, location=location, scale=scale)
        add_colliders(prim)
        if not visible:
            UsdGeom.Imageable(prim).MakeInvisible()


# Create a random transformation values for location, rotation, and scale
def get_random_transform_values(
    loc_min=(0, 0, 0), loc_max=(1, 1, 1), rot_min=(0, 0, 0), rot_max=(360, 360, 360), scale_min_max=(0.1, 1.0)
):
    location = (
        random.uniform(loc_min[0], loc_max[0]),
        random.uniform(loc_min[1], loc_max[1]),
        random.uniform(loc_min[2], loc_max[2]),
    )
    rotation = (
        random.uniform(rot_min[0], rot_max[0]),
        random.uniform(rot_min[1], rot_max[1]),
        random.uniform(rot_min[2], rot_max[2]),
    )
    scale = tuple([random.uniform(scale_min_max[0], scale_min_max[1])] * 3)
    return location, rotation, scale


# Generate a random pose on a sphere looking at the origin
# https://docs.omniverse.nvidia.com/isaacsim/latest/reference_conventions.html
def get_random_pose_on_sphere(origin, radius, camera_forward_axis=(0, 0, -1)):
    origin = Gf.Vec3f(origin)
    camera_forward_axis = Gf.Vec3f(camera_forward_axis)

    # Generate random angles for spherical coordinates
    theta = np.random.uniform(0, 2 * np.pi)
    phi = np.arcsin(np.random.uniform(-1, 1))

    # Spherical to Cartesian conversion
    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(phi)
    z = radius * np.sin(theta) * np.cos(phi)

    location = origin + Gf.Vec3f(x, y, z)

    # Calculate direction vector from camera to look_at point
    direction = origin - location
    direction_normalized = direction.GetNormalized()

    # Calculate rotation from forward direction (rotateFrom) to direction vector (rotateTo)
    rotation = Gf.Rotation(Gf.Vec3d(camera_forward_axis), Gf.Vec3d(direction_normalized))
    orientation = Gf.Quatf(rotation.GetQuat())

    return location, orientation


# Enable or disable the render products and viewport rendering
def set_render_products_updates(render_products, enabled, include_viewport=False):
    for rp in render_products:
        rp.hydra_texture.set_updates_enabled(enabled)
    if include_viewport:
        get_active_viewport().updates_enabled = enabled
