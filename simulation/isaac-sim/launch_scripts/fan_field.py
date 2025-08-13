import numpy as np

import carb

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from omni.isaac.core.world import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from pxr import UsdGeom, Gf

from omni.physx.scripts import deformableUtils, physicsUtils
from isaacsim.core.utils.rotations import euler_angles_to_quat
from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

from pxr import Sdf, UsdGeom, UsdPhysics, Usd, UsdLux


def customize_world(world: World, pg: PegasusInterface):
    carb.log_info("Adding default ground plane to the scene.")
    world.scene.add_default_ground_plane()

    # light
    distant_light = UsdLux.DistantLight.Define(world.scene.stage, "/World/DistantLight")
    distant_light.CreateIntensityAttr(1000)

    # create_fan_with_wind(0, translation=[0.0, 1.0, 0.57])

    # Create hardcoded field of 5 fans with random variations
    create_fan_with_wind(id=1, rpy_deg=[90.0, 0.0, -25.0], translation=[-7.2, 4.8, 2.5])

    create_fan_with_wind(id=2, rpy_deg=[90.0, 0.0, 15.0], translation=[3.1, -6.7, 3.2])

    create_fan_with_wind(id=3, rpy_deg=[90.0, 0.0, -40.0], translation=[8.9, 2.3, 1.8])

    create_fan_with_wind(id=4, rpy_deg=[90.0, 0.0, 30.0], translation=[-4.5, -8.1, 2.9])

    create_fan_with_wind(id=5, rpy_deg=[90.0, 0.0, -10.0], translation=[6.7, 7.4, 3.1])


def create_fan_with_wind(
    id, scale=[1.0, 1.0, 1.0], rpy_deg=[90.0, 0.0, 0.0], translation=[0.0, 0.0, 0.57]
):
    carb.log_info("Creating a fan with wind in the scene.")

    # load the fan model from USD
    box_fan = add_reference_to_stage(
        usd_path="omniverse://airlab-storage.andrew.cmu.edu:8443/Library/Assets/box_fan.usd",
        prim_path=f"/World/box_fan_{id}",
    )

    q = euler_angles_to_quat(rpy_deg, degrees=True)
    physicsUtils.set_or_add_scale_orient_translate(
        box_fan,
        scale=Gf.Vec3d(*scale),
        orient=Gf.Quatf(q[0], q[1], q[2], q[3]),
        translate=Gf.Vec3d(*translation),
    )

    add_colliders(box_fan)

    add_force_field_to_prim(box_fan)


def add_force_field_to_prim(prim):
    # Set the force field properties
    force_field_api = ForceFieldSchema.PhysxForceFieldPlanarAPI.Apply(
        prim, "force_field"
    )
    force_field_api.CreateNormalAttr(Gf.Vec3d(0.0, 0.0, 1.0))
    force_field_api.CreateConstantAttr(0.0)
    force_field_api.CreateLinearAttr(0.0)
    force_field_api.CreateInverseSquareAttr(100)
    # force_field_api.CreateRangeAttr(Gf.Vec2d(0.0, 10.0))
    force_field_base_api = ForceFieldSchema.PhysxForceFieldAPI(prim, "force_field")
    force_field_base_api.CreateEnabledAttr(True)
    force_field_base_api.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    force_field_base_api.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))
    # Add the bodies
    include_bodies_api = Usd.CollectionAPI.Apply(
        prim, ForceFieldSchema.Tokens.forceFieldBodies
    )
    include_bodies_api.CreateIncludesRel().AddTarget("/World")


def add_colliders(prim):
    # Add colliders
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    else:
        collision_api = UsdPhysics.CollisionAPI(prim)
    collision_api.CreateCollisionEnabledAttr(True)


def add_rigid_body(prim):
    # Add rigid body dynamics
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    else:
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
    rigid_body_api.CreateRigidBodyEnabledAttr(True)
