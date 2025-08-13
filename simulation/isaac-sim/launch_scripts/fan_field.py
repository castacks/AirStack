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

from pxr import Sdf, UsdGeom, UsdPhysics, Usd


def customize_world(world: World, pg: PegasusInterface):
    carb.log_info("Adding default ground plane to the scene.")
    world.scene.add_default_ground_plane()

    create_fan_with_wind(translation=[0.0, 1.0, 0.57])


def create_fan_with_wind(scale=[1.0, 1.0, 1.0], rpy_deg=[90.0, 0.0, 0.0], translation=[0.0, 0.0, 0.57]):
    carb.log_info("Creating a fan with wind in the scene.")

    # load the fan model from USD
    box_fan = add_reference_to_stage(
        usd_path="omniverse://airlab-storage.andrew.cmu.edu:8443/Library/Assets/box_fan.usd",
        prim_path="/World/box_fan",
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
    force_field_api = ForceFieldSchema.PhysxForceFieldPlanarAPI.Apply(prim, "force_field")
    force_field_api.CreateNormalAttr(Gf.Vec3d(0.0, 0.0, 1.0))
    force_field_api.CreateConstantAttr(0.0)
    force_field_api.CreateLinearAttr(0.0)
    force_field_api.CreateInverseSquareAttr(100)
    # Add the bodies
    include_bodies_api = Usd.CollectionAPI.Apply(prim, ForceFieldSchema.Tokens.forceFieldBodies)
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
