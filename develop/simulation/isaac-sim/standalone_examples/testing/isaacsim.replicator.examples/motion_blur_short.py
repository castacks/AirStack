# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.storage.native import get_assets_root_path
from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics

# Paths to the animated and physics-ready assets
PHYSICS_ASSET_URL = "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd"
ANIM_ASSET_URL = "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"

# -z velocities and start locations of the animated (left side) and physics (right side) assets (stage units/s)
ASSET_VELOCITIES = [0, 5, 10]
ASSET_X_MIRRORED_LOCATIONS = [(0.5, 0, 0.3), (0.3, 0, 0.3), (0.1, 0, 0.3)]

# Used to calculate how many frames to animate the assets to maintain the same velocity as the physics assets
ANIMATION_DURATION = 10

# Create a new stage with animated and physics-enabled assets with synchronized motion
def setup_stage():
    # Create new stage
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(ANIMATION_DURATION)

    # Create lights
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(100.0)
    distant_light = stage.DefinePrim("/World/DistantLight", "DistantLight")
    if not distant_light.GetAttribute("xformOp:rotateXYZ"):
        UsdGeom.Xformable(distant_light).AddRotateXYZOp()
    distant_light.GetAttribute("xformOp:rotateXYZ").Set((-75, 0, 0))
    distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(2500)

    # Setup the physics assets with gravity disabled and the requested velocity
    assets_root_path = get_assets_root_path()
    physics_asset_url = assets_root_path + PHYSICS_ASSET_URL
    for loc, vel in zip(ASSET_X_MIRRORED_LOCATIONS, ASSET_VELOCITIES):
        prim = stage.DefinePrim(f"/World/physics_asset_{int(abs(vel))}", "Xform")
        prim.GetReferences().AddReference(physics_asset_url)
        if not prim.GetAttribute("xformOp:translate"):
            UsdGeom.Xformable(prim).AddTranslateOp()
        prim.GetAttribute("xformOp:translate").Set(loc)
        prim.GetAttribute("physxRigidBody:disableGravity").Set(True)
        prim.GetAttribute("physxRigidBody:angularDamping").Set(0.0)
        prim.GetAttribute("physxRigidBody:linearDamping").Set(0.0)
        prim.GetAttribute("physics:velocity").Set((0, 0, -vel))

    # Setup animated assets maintaining the same velocity as the physics asssets
    anim_asset_url = assets_root_path + ANIM_ASSET_URL
    for loc, vel in zip(ASSET_X_MIRRORED_LOCATIONS, ASSET_VELOCITIES):
        start_loc = (-loc[0], loc[1], loc[2])
        prim = stage.DefinePrim(f"/World/anim_asset_{int(abs(vel))}", "Xform")
        prim.GetReferences().AddReference(anim_asset_url)
        if not prim.GetAttribute("xformOp:translate"):
            UsdGeom.Xformable(prim).AddTranslateOp()
        anim_distance = vel * ANIMATION_DURATION
        end_loc = (start_loc[0], start_loc[1], start_loc[2] - anim_distance)
        end_keyframe = timeline.get_time_codes_per_seconds() * ANIMATION_DURATION
        # Timesampled keyframe (animated) translation
        prim.GetAttribute("xformOp:translate").Set(start_loc, time=0)
        prim.GetAttribute("xformOp:translate").Set(end_loc, time=end_keyframe)


# Capture motion blur frames with the given delta time step and render mode
def run_motion_blur_example(num_frames=3, custom_delta_time=None, use_path_tracing=True, pt_subsamples=8, pt_spp=64):
    # Create a new stage with the assets
    setup_stage()
    stage = omni.usd.get_context().get_stage()

    # Set replicator settings (capture only on request and enable motion blur)
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)
    carb.settings.get_settings().set("/omni/replicator/captureMotionBlur", True)

    # Set motion blur settings based on the render mode
    if use_path_tracing:
        print(f"[MotionBlur] Setting PathTracing render mode motion blur settings")
        carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
        # (int): Total number of samples for each rendered pixel, per frame.
        carb.settings.get_settings().set("/rtx/pathtracing/spp", pt_spp)
        # (int): Maximum number of samples to accumulate per pixel. When this count is reached the rendering stops until a scene or setting change is detected, restarting the rendering process. Set to 0 to remove this limit.
        carb.settings.get_settings().set("/rtx/pathtracing/totalSpp", pt_spp)
        carb.settings.get_settings().set("/rtx/pathtracing/optixDenoiser/enabled", 0)
        # Number of sub samples to render if in PathTracing render mode and motion blur is enabled.
        carb.settings.get_settings().set("/omni/replicator/pathTracedMotionBlurSubSamples", pt_subsamples)
    else:
        print(f"[MotionBlur] Setting RaytracedLighting render mode motion blur settings")
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")
        # 0: Disabled, 1: TAA, 2: FXAA, 3: DLSS, 4:RTXAA
        carb.settings.get_settings().set("/rtx/post/aa/op", 2)
        # (float): The fraction of the largest screen dimension to use as the maximum motion blur diameter.
        carb.settings.get_settings().set("/rtx/post/motionblur/maxBlurDiameterFraction", 0.02)
        # (float): Exposure time fraction in frames (1.0 = one frame duration) to sample.
        carb.settings.get_settings().set("/rtx/post/motionblur/exposureFraction", 1.0)
        # (int): Number of samples to use in the filter. A higher number improves quality at the cost of performance.
        carb.settings.get_settings().set("/rtx/post/motionblur/numSamples", 8)

    # Setup camera and writer
    camera = rep.create.camera(position=(0, 1.5, 0), look_at=(0, 0, 0), name="MotionBlurCam")
    render_product = rep.create.render_product(camera, (1280, 720))
    basic_writer = rep.WriterRegistry.get("BasicWriter")
    delta_time_str = "None" if custom_delta_time is None else f"{custom_delta_time:.4f}"
    render_mode_str = f"pt_subsamples_{pt_subsamples}_spp_{pt_spp}" if use_path_tracing else "rt"
    output_directory = os.getcwd() + f"/_out_motion_blur_dt_{delta_time_str}_{render_mode_str}"
    print(f"[MotionBlur] Output directory: {output_directory}")
    basic_writer.initialize(output_dir=output_directory, rgb=True)
    basic_writer.attach(render_product)

    # Run a few updates to make sure all materials are fully loaded for capture
    for _ in range(50):
        simulation_app.update()

    # Use the physics scene to modify the physics FPS (if needed) to guarantee motion samples at any custom delta time
    physx_scene = None
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
            break
    if physx_scene is None:
        print(f"[MotionBlur] Creating a new PhysicsScene")
        physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))

    # Check the target physics depending on the custom delta time and the render mode
    target_physics_fps = stage.GetTimeCodesPerSecond() if custom_delta_time is None else 1 / custom_delta_time
    if use_path_tracing:
        target_physics_fps *= pt_subsamples

    # Check if the physics FPS needs to be increased to match the custom delta time
    orig_physics_fps = physx_scene.GetTimeStepsPerSecondAttr().Get()
    if target_physics_fps > orig_physics_fps:
        print(f"[MotionBlur] Changing physics FPS from {orig_physics_fps} to {target_physics_fps}")
        physx_scene.GetTimeStepsPerSecondAttr().Set(target_physics_fps)

    # Start the timeline for physics updates in the step function
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Capture frames
    for i in range(num_frames):
        print(f"[MotionBlur] \tCapturing frame {i}")
        rep.orchestrator.step(delta_time=custom_delta_time)

    # Restore the original physics FPS
    if target_physics_fps > orig_physics_fps:
        print(f"[MotionBlur] Restoring physics FPS from {target_physics_fps} to {orig_physics_fps}")
        physx_scene.GetTimeStepsPerSecondAttr().Set(orig_physics_fps)

    # Switch back to the raytracing render mode
    if use_path_tracing:
        print(f"[MotionBlur] Restoring render mode to RaytracedLighting")
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")

    # Wait until the data is fully written
    rep.orchestrator.wait_until_complete()


def run_motion_blur_examples():
    motion_blur_step_duration = [None, 1 / 240]  # [None, 1 / 30, 1 / 60, 1 / 240]
    for custom_delta_time in motion_blur_step_duration:
        # RayTracing examples
        run_motion_blur_example(custom_delta_time=custom_delta_time, use_path_tracing=False)
        # PathTracing examples
        spps = [32]  # [32, 128]
        motion_blur_sub_samples = [4]  # [4, 16]
        for motion_blur_sub_sample in motion_blur_sub_samples:
            for spp in spps:
                run_motion_blur_example(
                    custom_delta_time=custom_delta_time,
                    use_path_tracing=True,
                    pt_subsamples=motion_blur_sub_sample,
                    pt_spp=spp,
                )


run_motion_blur_examples()

simulation_app.close()
