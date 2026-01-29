# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.semantics import add_update_semantics
from pxr import Sdf, UsdGeom, UsdPhysics


def add_colliders_and_rigid_body_dynamics(prim):
    # Add colliders
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    else:
        collision_api = UsdPhysics.CollisionAPI(prim)
    collision_api.CreateCollisionEnabledAttr(True)
    # Add rigid body dynamics
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    else:
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
    rigid_body_api.CreateRigidBodyEnabledAttr(True)


def run_example():
    # Create a new stage and disable capture on play
    omni.usd.get_context().new_stage()
    rep.orchestrator.set_capture_on_play(False)

    # Add a light
    stage = omni.usd.get_context().get_stage()
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

    # Create a cube with colliders and rigid body dynamics at a specific location
    cube = stage.DefinePrim("/World/Cube", "Cube")
    add_colliders_and_rigid_body_dynamics(cube)
    if not cube.GetAttribute("xformOp:translate"):
        UsdGeom.Xformable(cube).AddTranslateOp()
    cube.GetAttribute("xformOp:translate").Set((0, 0, 2))
    add_update_semantics(cube, "MyCube")

    # Createa a sphere with colliders and rigid body dynamics next to the cube
    sphere = stage.DefinePrim("/World/Sphere", "Sphere")
    add_colliders_and_rigid_body_dynamics(sphere)
    if not sphere.GetAttribute("xformOp:translate"):
        UsdGeom.Xformable(sphere).AddTranslateOp()
    sphere.GetAttribute("xformOp:translate").Set((-1, -1, 2))
    add_update_semantics(sphere, "MySphere")

    # Create a render product using the viewport perspective camera
    rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512))

    # Write data using the basic writer with the rgb and bounding box annotators
    writer = rep.writers.get("BasicWriter")
    out_dir = os.getcwd() + "/_out_basic_writer_sim"
    print(f"Output directory: {out_dir}")
    writer.initialize(output_dir=out_dir, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True)
    writer.attach(rp)

    # Start the timeline (will only advance with app update)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Update the app and implicitly advance the simulation
    drop_delta = 0.5
    last_capture_height = cube.GetAttribute("xformOp:translate").Get()[2]
    for i in range(100):
        # Get the current height of the cube and the distance it dropped since the last capture
        simulation_app.update()
        current_height = cube.GetAttribute("xformOp:translate").Get()[2]
        drop_since_last_capture = last_capture_height - current_height
        print(f"Step {i}; cube height: {current_height:.3f}; drop since last capture: {drop_since_last_capture:.3f}")

        # Stop the simulation if the cube falls below the ground
        if current_height < 0:
            print(f"\t Cube fell below the ground at height {current_height:.3f}, stopping simulation..")
            timeline.pause()
            break

        # Capture every time the cube drops by the threshold distance
        if drop_since_last_capture >= drop_delta:
            print(f"\t Capturing at height {current_height:.3f}")
            last_capture_height = current_height
            # Pause the timeline to capture multiple frames of the same simulation state
            timeline.pause()

            # Setting delta_time to 0.0 will make sure the step function will not advance the simulation during capture
            rep.orchestrator.step(delta_time=0.0)

            # Capture again with the cube hidden
            UsdGeom.Imageable(cube).MakeInvisible()
            rep.orchestrator.step(delta_time=0.0)
            UsdGeom.Imageable(cube).MakeVisible()

            # Resume the timeline to continue the simulation
            timeline.play()

    # Destroy the render product to release resources by detaching it from the writer first
    writer.detach()
    rp.destroy()

    # Wait for the data to be written to disk
    rep.orchestrator.wait_until_complete()


# Run the example
run_example()

simulation_app.close()
