"""
scene_prep.py — Utilities for preparing Isaac Sim / USD stages before simulation.

Functions:
    get_stage_meters_per_unit   — Read stage unit scale
    scale_stage_prim            — Apply a uniform scale transform to a prim
    add_colliders               — Recursively apply CollisionAPI to all meshes
    settle_rigid_props          — Physics-drop toppled props to rest, then freeze
    add_dome_light              — Add or update a dome light on the stage
    add_sky                     — Skybox from HDRI (dome texture) or borrowed stage prims
    save_scene_as_contained_usd — Collect all assets into a self-contained directory
"""

import asyncio
import os
import re
import time as _time
import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux, Sdf


# ---------------------------------------------------------------------------
# Stage units
# ---------------------------------------------------------------------------

def get_stage_meters_per_unit(stage) -> tuple:
    """Return (meters_per_unit, scene_scale_factor).

    scene_scale_factor is 1/mpu — multiply metric coordinates by this to get
    stage-space coordinates.
    """
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if mpu is None or mpu <= 0:
        mpu = 1.0
    return mpu, 1.0 / mpu


# ---------------------------------------------------------------------------
# Scaling
# ---------------------------------------------------------------------------

def scale_stage_prim(stage, prim_path: str, scale_factor: float):
    """Apply a uniform XYZ scale to *prim_path*, clearing any prior xform ops.

    Args:
        stage:        Active USD stage.
        prim_path:    Stage path of the prim to scale (e.g. "/World/stage").
        scale_factor: Uniform scale to apply (e.g. 0.01 to convert cm → m).

    Returns:
        The scaled UsdPrim.

    Raises:
        ValueError: If the prim is not found at *prim_path*.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Prim not found at path: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()

    # Match the precision of any existing xform attrs to avoid type-mismatch exceptions.
    # ClearXformOpOrder removes attrs from the op order but leaves the attributes on the prim,
    # so AddXformOp will fail if the requested precision doesn't match the baked-in type.
    def _precision(attr_name, default=UsdGeom.XformOp.PrecisionDouble):
        attr = prim.GetAttribute(attr_name)
        if attr.IsValid() and str(attr.GetTypeName()) == "float3":
            return UsdGeom.XformOp.PrecisionFloat
        return default

    translate_prec = _precision("xformOp:translate")
    scale_prec = _precision("xformOp:scale")

    if translate_prec == UsdGeom.XformOp.PrecisionFloat:
        xformable.AddTranslateOp(translate_prec).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    else:
        xformable.AddTranslateOp(translate_prec).Set(Gf.Vec3d(0.0, 0.0, 0.0))

    if scale_prec == UsdGeom.XformOp.PrecisionFloat:
        xformable.AddScaleOp(scale_prec).Set(Gf.Vec3f(scale_factor, scale_factor, scale_factor))
    else:
        xformable.AddScaleOp(scale_prec).Set(Gf.Vec3d(scale_factor, scale_factor, scale_factor))

    print(f"[scene_prep] Scaled '{prim_path}' by {scale_factor}")
    return prim


# ---------------------------------------------------------------------------
# Physics scene dedupe
# ---------------------------------------------------------------------------

def dedupe_physics_scenes(stage) -> str | None:
    """Keep the first UsdPhysics.Scene found in the stage; deactivate the rest.

    Isaac's World autocreates a PhysicsScene on init, and Kit-saved USDs
    often bake one in too. PhysX can only step a single scene coherently,
    so duplicates trigger "Physics scenes stepping is not the same" and
    desynced sensors. Duplicates are deactivated via SetActive(False) — not
    removed — because prims defined in a referenced sublayer have no spec
    in the live root layer for RemovePrim to delete. Deactivation writes
    an `active = false` override on the root layer so USD ignores the prim
    entirely without touching the source asset. Returns the kept prim path
    (or None if no scene).
    """
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        print("[scene_prep] No PhysicsScene found in stage")
        return None
    keep, *extras = scenes
    for extra in extras:
        path = extra.GetPath()
        # Prims that come from a referenced sublayer (e.g. PhysicsScene baked
        # into the loaded USD) can't be deleted with RemovePrim from the live
        # root layer — there's no spec there to remove. SetActive(False)
        # writes an "active = false" override on the root layer, which makes
        # USD ignore the prim entirely without touching the source asset.
        if not extra.SetActive(False):
            print(f"[scene_prep] WARN: failed to deactivate PhysicsScene at {path}")
        else:
            print(f"[scene_prep] Deactivated duplicate PhysicsScene: {path}")
    print(f"[scene_prep] Kept PhysicsScene: {keep.GetPath()}")
    return str(keep.GetPath())


# ---------------------------------------------------------------------------
# Collision
# ---------------------------------------------------------------------------

def add_colliders(prim):
    """Recursively apply UsdPhysics.CollisionAPI to every gprim (Mesh, Cube,
    …) under *prim*.

    Skips prims that already have the API applied.
    """
    if prim.IsA(UsdGeom.Gprim):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            print(f"[scene_prep] Added collider: {prim.GetPath()}")

    for child in prim.GetChildren():
        add_colliders(child)


# ---------------------------------------------------------------------------
# Physics settling
# ---------------------------------------------------------------------------

def settle_rigid_props(stage, prim_paths, sim_seconds: float = 3.0,
                       ground_path: str = None, max_travel_m: float = 12.0,
                       report: int = 5) -> int:
    """Drop props to a natural resting pose under physics, then freeze them.

    The scene generator places toppled/strewn props (flipped cars, downed
    streetlights) at *approximated* orientations, which leaves some floating
    or interpenetrating. This gives each prim in *prim_paths* a dynamic rigid
    body with convex-hull colliders, plays the timeline for *sim_seconds* so
    they fall to rest, captures the settled transforms, stops the timeline
    (which resets the sim), authors the captured transforms back, and strips
    the rigid bodies — so the props stay parked exactly where physics left
    them. Colliders are kept, so the settled props remain solid to sensors
    and the drone.

    *ground_path*, if given, gets static colliders applied first (the props
    need something to land on — pass the generated ground subtree).

    PROPS THAT START INSIDE EACH OTHER
    ----------------------------------
    The generator scatters debris on a ring around each ruin without testing
    what is already there, so props routinely spawn overlapping — measured on
    a severity-0.8 suburb, **75% of settle-marked props start interpenetrating
    something**, median 0.14 m but p90 2.66 m and worst 6.91 m, with one piece
    inside 25 others. PhysX resolves penetration with a separating impulse
    proportional to depth, so metres of overlap on a metre-wide piece is a
    catapult: it leaves at high speed and is still travelling when the sim
    ends, which is how debris turns up hundreds of metres outside the scene.
    The Blender prototype hit the same wall and documents it — "the solver
    resolves that by launching them apart... debris ends up scattered over
    many building-widths".

    Two defences, because neither is sufficient alone:

    * **`maxDepenetrationVelocity`** caps how fast the solver may push
      overlapping bodies apart. This is PhysX's own knob for exactly this and
      it treats the cause: pieces ooze out of each other instead of being
      fired. `maxLinearVelocity` bounds the damage from anything that still
      picks up speed.
    * **`max_travel_m`** is the safety net: any prop that still ends up
      further than that from where it started is *reverted to its authored
      pose* rather than left where it flew. Reverting, not deleting — unlike
      the prototype's fragments these are placements the generator chose
      deliberately, and its spot is a better answer than a solver artifact.
      This mirrors the existing fall-through-the-ground case.

    *report* prints that many worst-travelled props, so a run says whether the
    clamps are holding rather than leaving it to be eyeballed in the viewport.

    Call BEFORE spawning robot/vehicle graphs: the timeline runs briefly.
    Returns the number of props settled.
    """
    import omni.timeline

    prims = [stage.GetPrimAtPath(p) for p in prim_paths]
    prims = [p for p in prims if p and p.IsValid()]
    if not prims:
        return 0

    if ground_path:
        ground = stage.GetPrimAtPath(ground_path)
        if ground.IsValid():
            add_colliders(ground)

    # Physics needs a scene; standalone preview stages may not have one.
    roots = list(stage.GetPseudoRoot().GetChildren())
    two_levels = roots + [c for r in roots for c in r.GetChildren()]
    if not any(p.IsA(UsdPhysics.Scene) for p in two_levels):
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        print("[scene_prep] settle: defined /World/PhysicsScene")

    # Optional: only inside Isaac's Kit python, not in plain usd-core. Without
    # it the clamps are skipped and only `max_travel_m` catches the launches.
    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None

    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0

    # Dynamic rigid bodies need convex colliders (triangle-mesh collision is
    # static-only in PhysX) — approximate every gprim under each prop.
    for prim in prims:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if PhysxSchema is not None:
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            # 1 m/s out of a penetration is plenty to separate a prop over a
            # 3 s sim and far too slow to throw one across the map. 20 m/s
            # then bounds anything that picks up speed some other way.
            rb.CreateMaxDepenetrationVelocityAttr().Set(1.0 / mpu)
            rb.CreateMaxLinearVelocityAttr().Set(20.0 / mpu)
        for gp in Usd.PrimRange(prim):
            if gp.IsA(UsdGeom.Gprim):
                UsdPhysics.CollisionAPI.Apply(gp)
            if gp.IsA(UsdGeom.Mesh):
                mc = UsdPhysics.MeshCollisionAPI.Apply(gp)
                mc.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)

    # Where everything started, so the freeze pass can tell a prop that settled
    # from one that was fired.
    start_cache = UsdGeom.XformCache()
    start = {p.GetPath(): start_cache.GetLocalToWorldTransform(p)
             .ExtractTranslation() for p in prims}

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    timeline.play()
    steps = max(1, int(sim_seconds * 60.0))
    print(f"[scene_prep] settle: simulating {len(prims)} props "
          f"for ~{sim_seconds:.1f}s ({steps} frames)…")
    for _ in range(steps):
        app.update()

    # Capture settled transforms before stopping (stop resets the sim pose).
    floor_z = -2.0 / mpu    # anything below fell through the ground
    limit = max_travel_m / mpu
    cache = UsdGeom.XformCache()
    settled = {}
    for prim in prims:
        world = cache.GetLocalToWorldTransform(prim)
        parent_world = cache.GetLocalToWorldTransform(prim.GetParent())
        pos = world.ExtractTranslation()
        settled[prim.GetPath()] = (world * parent_world.GetInverse(), pos)
    timeline.stop()
    app.update()

    n_ok = n_lost = n_flung = 0
    travel = []
    for prim in prims:
        local, pos = settled[prim.GetPath()]
        moved = (pos - start[prim.GetPath()]).GetLength()
        travel.append((moved * mpu, prim.GetPath()))
        if pos[2] < floor_z:
            n_lost += 1     # tunnelled through the ground — keep authored pose
        elif moved > limit:
            # Launched out of a penetration. Its authored spot is where the
            # generator meant it to be; a solver artifact hundreds of metres
            # away is not an improvement on that.
            n_flung += 1
        else:
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(local)
            n_ok += 1
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    travel.sort(reverse=True)
    med = travel[len(travel) // 2][0] if travel else 0.0
    print(f"[scene_prep] settle: froze {n_ok} props at rest"
          + (f", {n_lost} fell through the ground (kept authored pose)"
             if n_lost else "")
          + (f", {n_flung} flung past {max_travel_m:.0f} m (reverted)"
             if n_flung else ""))
    if travel:
        print(f"[scene_prep] settle: travel median {med:.2f} m, "
              f"max {travel[0][0]:.1f} m")
    if report and travel and travel[0][0] > max_travel_m:
        print(f"[scene_prep] settle: worst {min(report, len(travel))} — "
              "these started inside other geometry:")
        for d, path in travel[:report]:
            print(f"    {d:8.1f} m  {path}")
    return n_ok


# ---------------------------------------------------------------------------
# Lighting
# ---------------------------------------------------------------------------

def add_dome_light(stage, prim_path: str = "/World/DomeLight", intensity: float = 3500.0,
                    exposure: float = -3.0, texture_file: str = None):
    """Add a dome light to the stage, or update it if it already exists.

    Args:
        stage:        Active USD stage.
        prim_path:    Stage path for the dome light prim.
        intensity:    Light intensity value.
        exposure:     Light exposure value.
        texture_file: Optional path/URL to an HDRI/.exr texture. When set,
                      the dome renders that image as the skybox instead of a
                      flat color.
    """
    if stage.GetPrimAtPath(prim_path).IsValid():
        dome = UsdLux.DomeLight.Get(stage, prim_path)
    else:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path(prim_path))
    dome.CreateIntensityAttr(intensity)
    dome.CreateExposureAttr(exposure)
    if texture_file:
        dome.CreateTextureFileAttr(Sdf.AssetPath(texture_file))
        # Latitude-longitude mapping is the standard projection for full-sphere
        # panoramic HDRIs (as opposed to "MirroredBall" or "Automatic" cross formats).
        dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
    print(f"[scene_prep] Dome light set at '{prim_path}' "
          f"(intensity={intensity}, exposure={exposure}, texture={texture_file})")


def add_sky(stage, sky_path: str = "", prim_path: str = "/World/DomeLight",
            intensity: float = 3500.0, exposure: float = -3.0):
    """Set up sky + ambient lighting from a config-resolved *sky_path*.

    Dispatches on the path:
      - ``.usd``/``.usda``/``.usdc`` — borrow the sky: reference the stage's
        root prims that sit *outside* its defaultPrim (sky sphere, sun,
        environment lights) under /World. Only those subtrees are composed,
        not the stage's main geometry, so this stays cheap. Skips the plain
        dome light when it pulls something in (borrowed stages bring their
        own lighting).
      - anything else non-empty — treated as an equirect HDRI for the dome
        light's skybox texture.
      - empty — plain untextured dome light.
    """
    if sky_path and os.path.splitext(sky_path.split("?")[0])[1].lower() in (
            ".usd", ".usda", ".usdc"):
        pulled = reference_root_prims_under_world(stage, sky_path)
        if pulled:
            print(f"[scene_prep] Sky borrowed from {sky_path}: {pulled}")
            return
        print(f"[scene_prep] No borrowable root prims in {sky_path}; "
              f"falling back to plain dome light")
        sky_path = ""
    add_dome_light(stage, prim_path, intensity, exposure, texture_file=sky_path or None)


# ---------------------------------------------------------------------------
# Top-down "map" camera: orthographic, fixed over (0,0), used to publish a
# one-shot aerial of the static sim scene that the GCS visualizer will
# convert into a textured ground in Foxglove's 3D panel.
# ---------------------------------------------------------------------------

def add_orthographic_camera(stage,
                            prim_path: str = "/World/MapCamera",
                            altitude_m: float = 80.0,
                            coverage_m: float = 80.0,
                            scene_scale_factor: float = 1.0,
                            center_x_m: float = 0.0,
                            center_y_m: float = 0.0):
    """Create a static orthographic camera straight over a configurable XY
    point looking down.

    Args:
        stage:               Active USD stage.
        prim_path:           Where to place the camera prim.
        altitude_m:          Camera height above world origin (metric).
        coverage_m:          Side length of the world square in frame (metric).
        scene_scale_factor:  1 / meters_per_unit. Pass the value returned by
                             ``get_stage_meters_per_unit`` so metric inputs
                             land in the right stage-space units.
        center_x_m:          World-X of the camera center (metric). Default 0.
        center_y_m:          World-Y of the camera center (metric). Default 0.

    Returns:
        The string prim path (handy for callers that pass it to the OG helper).
    """
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(prim_path))

    # USD cameras look along -Z by default — already straight down, no rotation.
    xform = UsdGeom.Xformable(cam.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(center_x_m) * float(scene_scale_factor),
                 float(center_y_m) * float(scene_scale_factor),
                 float(altitude_m) * float(scene_scale_factor)))

    # Orthographic projection. USD aperture is in tenths of a stage unit, so
    # for a 1 m stage `coverage_m * 10` puts a coverage_m × coverage_m square
    # exactly in frame; scale_factor extends that to non-meter stage units.
    cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
    aperture = float(coverage_m) * 10.0 * float(scene_scale_factor)
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture)
    cam.CreateClippingRangeAttr(
        Gf.Vec2f(0.1, max(2.0, float(altitude_m) * 2.0) * float(scene_scale_factor)))

    print(f"[scene_prep] Orthographic map camera at '{prim_path}' "
          f"(alt={altitude_m} m, coverage={coverage_m} m, "
          f"center=({center_x_m}, {center_y_m}) m)")
    return prim_path


def add_overhead_camera_publisher(parent_graph_path: str,
                                  camera_prim_path: str,
                                  topic: str = "/sim/overhead/image",
                                  spec_topic: str = "/sim/overhead/spec",
                                  center_x_topic: str = "/sim/overhead/center_x",
                                  center_y_topic: str = "/sim/overhead/center_y",
                                  frame_id: str = "map",
                                  coverage_m: float = 80.0,
                                  center_x_m: float = 0.0,
                                  center_y_m: float = 0.0,
                                  pixels_per_meter: float = 4.0,
                                  max_resolution: int = 2048,
                                  domain_id: int = 0):
    """Wire an orthographic camera to a raw ``sensor_msgs/Image`` topic, plus
    three spec ``std_msgs/Float32`` topics carrying ``coverage_m``,
    ``center_x_m``, ``center_y_m`` so consumers can size **and place** the
    ground texture without manual configuration.

    The image resolution is auto-derived from ``coverage_m × pixels_per_meter``
    and capped at ``max_resolution`` so a typo can't blow up bandwidth. Sim
    scene is static, so we only need one valid frame — the GCS visualizer
    catches it then unsubscribes, making the brief startup burst the only
    network cost.

    Builds a standalone OmniGraph at ``parent_graph_path`` with its own
    ROS2Context targeting ``domain_id`` regardless of which domain the
    drones are using.
    """
    import omni.graph.core as og  # lazy so non-sim contexts can import scene_prep

    res = max(64, min(int(round(float(coverage_m) * float(pixels_per_meter))),
                      int(max_resolution)))

    controller = og.Controller()
    g = parent_graph_path

    if og.get_graph_by_path(g) is None:
        og.Controller.create_graph({"graph_path": g, "evaluator_name": "execution"})

    nodes = {
        "context":      f"{g}/MapCameraROS2Context",
        "playback":     f"{g}/MapCameraOnPlaybackTick",
        "create_rp":    f"{g}/MapCameraCreateRenderProduct",
        "rgb":          f"{g}/MapCameraRGBHelper",
        "frame":        f"{g}/MapCameraFrameId",
        "topic":        f"{g}/MapCameraTopic",
        # Spec branches: publish coverage_m / center_x_m / center_y_m once per
        # tick on separate Float32 topics so the GCS visualizer auto-discovers
        # both FOV and world placement.
        "spec_value":   f"{g}/MapCameraSpecValue",
        "spec_topic":   f"{g}/MapCameraSpecTopic",
        "spec_pub":     f"{g}/MapCameraSpecPublisher",
        "cx_value":     f"{g}/MapCameraCenterXValue",
        "cx_topic":     f"{g}/MapCameraCenterXTopic",
        "cx_pub":       f"{g}/MapCameraCenterXPublisher",
        "cy_value":     f"{g}/MapCameraCenterYValue",
        "cy_topic":     f"{g}/MapCameraCenterYTopic",
        "cy_pub":       f"{g}/MapCameraCenterYPublisher",
    }

    controller.edit(
        graph_id=g,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (nodes["context"],    "isaacsim.ros2.bridge.ROS2Context"),
                (nodes["playback"],   "omni.graph.action.OnPlaybackTick"),
                (nodes["create_rp"],  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (nodes["rgb"],        "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (nodes["frame"],      "omni.graph.nodes.ConstantString"),
                (nodes["topic"],      "omni.graph.nodes.ConstantString"),
                (nodes["spec_value"], "omni.graph.nodes.ConstantFloat"),
                (nodes["spec_topic"], "omni.graph.nodes.ConstantString"),
                (nodes["spec_pub"],   "isaacsim.ros2.bridge.ROS2Publisher"),
                (nodes["cx_value"],   "omni.graph.nodes.ConstantFloat"),
                (nodes["cx_topic"],   "omni.graph.nodes.ConstantString"),
                (nodes["cx_pub"],     "isaacsim.ros2.bridge.ROS2Publisher"),
                (nodes["cy_value"],   "omni.graph.nodes.ConstantFloat"),
                (nodes["cy_topic"],   "omni.graph.nodes.ConstantString"),
                (nodes["cy_pub"],     "isaacsim.ros2.bridge.ROS2Publisher"),
            ],
            og.Controller.Keys.CONNECT: [
                # Image branch
                (f"{nodes['playback']}.outputs:tick",     f"{nodes['create_rp']}.inputs:execIn"),
                (f"{nodes['create_rp']}.outputs:execOut", f"{nodes['rgb']}.inputs:execIn"),
                (f"{nodes['create_rp']}.outputs:renderProductPath",
                 f"{nodes['rgb']}.inputs:renderProductPath"),
                (f"{nodes['context']}.outputs:context",   f"{nodes['rgb']}.inputs:context"),
                (f"{nodes['frame']}.inputs:value",        f"{nodes['rgb']}.inputs:frameId"),
                (f"{nodes['topic']}.inputs:value",        f"{nodes['rgb']}.inputs:topicName"),
                # Spec (coverage) branch
                (f"{nodes['playback']}.outputs:tick",     f"{nodes['spec_pub']}.inputs:execIn"),
                (f"{nodes['context']}.outputs:context",   f"{nodes['spec_pub']}.inputs:context"),
                (f"{nodes['spec_topic']}.inputs:value",   f"{nodes['spec_pub']}.inputs:topicName"),
                # Center-X branch
                (f"{nodes['playback']}.outputs:tick",     f"{nodes['cx_pub']}.inputs:execIn"),
                (f"{nodes['context']}.outputs:context",   f"{nodes['cx_pub']}.inputs:context"),
                (f"{nodes['cx_topic']}.inputs:value",     f"{nodes['cx_pub']}.inputs:topicName"),
                # Center-Y branch
                (f"{nodes['playback']}.outputs:tick",     f"{nodes['cy_pub']}.inputs:execIn"),
                (f"{nodes['context']}.outputs:context",   f"{nodes['cy_pub']}.inputs:context"),
                (f"{nodes['cy_topic']}.inputs:value",     f"{nodes['cy_pub']}.inputs:topicName"),
            ],
            og.Controller.Keys.SET_VALUES: [
                (("inputs:domain_id",         nodes["context"]),  int(domain_id)),
                (("inputs:cameraPrim",        nodes["create_rp"]), camera_prim_path),
                (("inputs:width",             nodes["create_rp"]), res),
                (("inputs:height",            nodes["create_rp"]), res),
                (("inputs:type",              nodes["rgb"]), "rgb"),
                (("inputs:value",             nodes["frame"]), str(frame_id)),
                (("inputs:value",             nodes["topic"]), str(topic)),
                (("inputs:value",             nodes["spec_value"]), float(coverage_m)),
                (("inputs:value",             nodes["spec_topic"]), str(spec_topic)),
                (("inputs:messageName",       nodes["spec_pub"]), "Float32"),
                (("inputs:messagePackage",    nodes["spec_pub"]), "std_msgs"),
                (("inputs:messageSubfolder",  nodes["spec_pub"]), "msg"),
                (("inputs:value",             nodes["cx_value"]), float(center_x_m)),
                (("inputs:value",             nodes["cx_topic"]), str(center_x_topic)),
                (("inputs:messageName",       nodes["cx_pub"]), "Float32"),
                (("inputs:messagePackage",    nodes["cx_pub"]), "std_msgs"),
                (("inputs:messageSubfolder",  nodes["cx_pub"]), "msg"),
                (("inputs:value",             nodes["cy_value"]), float(center_y_m)),
                (("inputs:value",             nodes["cy_topic"]), str(center_y_topic)),
                (("inputs:messageName",       nodes["cy_pub"]), "Float32"),
                (("inputs:messagePackage",    nodes["cy_pub"]), "std_msgs"),
                (("inputs:messageSubfolder",  nodes["cy_pub"]), "msg"),
            ],
        },
    )

    # The ROS2Publisher's value input is dynamically typed — created on the
    # node after the message type is set. Connect ConstantFloat → publisher
    # for each of the three spec topics.
    controller.edit(
        graph_id=g,
        edit_commands={
            og.Controller.Keys.CONNECT: [
                (f"{nodes['spec_value']}.inputs:value",
                 f"{nodes['spec_pub']}.inputs:data"),
                (f"{nodes['cx_value']}.inputs:value",
                 f"{nodes['cx_pub']}.inputs:data"),
                (f"{nodes['cy_value']}.inputs:value",
                 f"{nodes['cy_pub']}.inputs:data"),
            ],
        },
    )

    print(f"[scene_prep] Overhead camera publisher wired: "
          f"{topic} ({res}x{res} raw Image), {spec_topic} ({coverage_m} m), "
          f"{center_x_topic}={center_x_m} m, {center_y_topic}={center_y_m} m, "
          f"domain_id={domain_id}")


# ---------------------------------------------------------------------------
# Consolidate root prims under /World
# ---------------------------------------------------------------------------

def reference_root_prims_under_world(stage, source_usd_url: str) -> list:
    """Reference sibling root prims from *source_usd_url* under /World/<name>.

    pg.load_environment references the source USD's defaultPrim into
    /World/stage — anything outside that defaultPrim (sky, sun, environment
    sitting at root level) gets dropped. This function pulls those siblings
    in as individual references.

    Skips the defaultPrim itself, since re-referencing it would create a
    second independent copy of the same geometry next to /World/stage.
    Also skips a literal /World prim if one exists in the source.

    Args:
        stage:          Active USD stage.
        source_usd_url: omniverse:// or local path of the source USD.

    Returns:
        List of prim names that were referenced.
    """
    source_layer = Sdf.Layer.FindOrOpen(source_usd_url)
    if source_layer is None:
        print(f"[scene_prep] reference_root_prims_under_world: could not open {source_usd_url}", flush=True)
        return []

    default_prim = source_layer.defaultPrim  # name only, e.g. "Stage"
    skip = {'World'}
    if default_prim:
        skip.add(default_prim)

    siblings = [spec.name for spec in source_layer.rootPrims if spec.name not in skip]
    if not siblings:
        print(f"[scene_prep] reference_root_prims_under_world: no sibling root prims to pull in "
              f"(defaultPrim={default_prim!r}, all roots={[s.name for s in source_layer.rootPrims]})",
              flush=True)
        return []

    for name in siblings:
        dest_path = f"/World/{name}"
        dest_prim = stage.DefinePrim(dest_path)
        dest_prim.GetReferences().AddReference(source_usd_url, f"/{name}")
        print(f"[scene_prep] Referenced /{name} at {dest_path}", flush=True)

    return siblings


def move_root_prims_to_world_live(stage) -> list:
    """Move any non-/World root prims (e.g. /Environment, /Sun, /Sky) under /World
    on the currently active live stage.

    Useful when loading a USD from Nucleus whose sky/sun/environment prims sit at
    the root rather than under /World, causing them to be invisible to the sim.

    Args:
        stage: Active USD stage (from omni.usd.get_context().get_stage()).

    Returns:
        List of prim names that were moved.
    """
    root_layer = stage.GetRootLayer()
    all_root = [spec.name for spec in root_layer.rootPrims]
    print(f"[scene_prep] move_root_prims_to_world_live: root prims = {all_root}", flush=True)

    to_move = [name for name in all_root if name != 'World']
    if not to_move:
        print("[scene_prep] move_root_prims_to_world_live: nothing to move", flush=True)
        return []

    edit = Sdf.BatchNamespaceEdit()
    for name in to_move:
        edit.Add(Sdf.Path(f"/{name}"), Sdf.Path(f"/World/{name}"))

    if not root_layer.Apply(edit):
        print(f"[scene_prep] move_root_prims_to_world_live: namespace edit failed for {to_move}", flush=True)
        return []

    print(f"[scene_prep] Moved root prims under /World: {to_move}", flush=True)
    return to_move


def move_root_prims_to_world(usd_path: str) -> list:
    """Move any non-/World root prims (e.g. /Environment) under /World.

    After export_as_stage_async, sibling root prims like /Environment are
    excluded when pg.load_environment references the file via defaultPrim=/World.
    This function opens the flat exported USD layer directly and relocates
    those prims under /World so they are included in the reference.

    Args:
        usd_path: Path to the flat exported USD file to patch in-place.

    Returns:
        List of prim names that were moved.
    """
    layer = Sdf.Layer.Find(usd_path) or Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        print(f"[scene_prep] move_root_prims_to_world: could not open {usd_path}", flush=True)
        return []

    all_root = [spec.name for spec in layer.rootPrims]
    print(f"[scene_prep] move_root_prims_to_world: root prims in exported USD: {all_root}", flush=True)
    print(f"[scene_prep] move_root_prims_to_world: sublayers: {layer.subLayerPaths}", flush=True)

    to_move = [name for name in all_root if name != 'World']
    if not to_move:
        print(f"[scene_prep] move_root_prims_to_world: nothing to move", flush=True)
        return []

    edit = Sdf.BatchNamespaceEdit()
    for name in to_move:
        edit.Add(Sdf.Path(f"/{name}"), Sdf.Path(f"/World/{name}"))

    if not layer.Apply(edit):
        print(f"[scene_prep] move_root_prims_to_world: namespace edit failed for {to_move}", flush=True)
        return []

    layer.Save()
    print(f"[scene_prep] Moved root prims under /World: {to_move}", flush=True)
    return to_move


# ---------------------------------------------------------------------------
# Save as self-contained USD collection
# ---------------------------------------------------------------------------

def save_scene_as_contained_usd(source_usd_url: str, output_dir: str) -> bool:
    """Collect a USD stage and all its dependencies into a self-contained directory.

    Uses omni.kit.usd.collect.Collector (Isaac Sim 5.1 / Kit 107) which correctly
    handles Nucleus omniverse:// URLs, MDL materials, and texture files.

    The collected root USD will be written to output_dir, with all referenced
    assets copied alongside it using relative paths.

    Args:
        source_usd_url: Path or omniverse:// URL of the source USD to collect.
        output_dir:     Local directory to write the collected package into.

    Returns:
        True on success, False on failure.
    """
    # Enable the collect extension if not already active
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    if not ext_manager.is_extension_enabled("omni.kit.usd.collect"):
        ext_manager.set_extension_enabled_immediate("omni.kit.usd.collect", True)

    from omni.kit.usd.collect import Collector

    collector = Collector(
        usd_path=source_usd_url,
        collect_dir=output_dir,
        usd_only=False,        # include textures, MDLs, etc.
        flat_collection=True, # preserve source folder hierarchy
        skip_existing=False,
    )

    done = [False]
    result = [False]

    def on_progress(current: int, total: int):
        print(f"[scene_prep] Collecting assets: {current}/{total}", flush=True)

    def on_finish():
        result[0] = True
        done[0] = True
        print("[scene_prep] Collection complete.", flush=True)

    # Schedule the collect coroutine on Kit's event loop so the app loop keeps ticking
    asyncio.ensure_future(
        collector.collect(progress_callback=on_progress, finish_callback=on_finish)
    )

    # Pump the Kit app loop until collection finishes
    app = omni.kit.app.get_app()
    while not done[0]:
        app.update()

    collector.destroy()
    return result[0]


# ---------------------------------------------------------------------------
# Fix missing MDL textures
# ---------------------------------------------------------------------------

def _resolve_nucleus_url(base_url: str, relative: str) -> str:
    """Resolve a relative path against a Nucleus base directory URL."""
    parts = base_url.rstrip('/').split('/')
    for segment in relative.replace('\\', '/').split('/'):
        if segment == '..':
            parts.pop()
        elif segment and segment != '.':
            parts.append(segment)
    return '/'.join(parts)


def fix_missing_mdl_textures(output_dir: str, nucleus_env_url: str) -> int:
    """Download textures referenced in MDL files that the Collector missed.

    The Collector rewrites texture paths inside MDL files to relative local
    paths but does not always copy the actual texture files. This function
    downloads the original Nucleus MDL to find the real texture URLs, then
    downloads any missing textures to the expected local paths.

    Args:
        output_dir:       Local directory written by the Collector.
        nucleus_env_url:  Original omniverse:// URL of the source scene.

    Returns:
        Number of textures downloaded.
    """
    import omni.client
    import tempfile

    nucleus_base = nucleus_env_url.rsplit('/', 1)[0]
    nucleus_materials_dir = f"{nucleus_base}/Materials"

    texture_pattern = re.compile(
        r'["\']([^"\']*\.(?:png|jpg|jpeg|exr|hdr|dds|tga|bmp))["\']',
        re.IGNORECASE,
    )
    downloaded = 0

    for root, dirs, files in os.walk(output_dir):
        for fname in files:
            if not fname.endswith('.mdl'):
                continue

            local_mdl = os.path.join(root, fname)

            # Download the original Nucleus MDL to a temp file
            nucleus_mdl_url = f"{nucleus_materials_dir}/{fname}"
            tmp_mdl = os.path.join(tempfile.gettempdir(), f"orig_{fname}")
            copy_result = omni.client.copy(
                nucleus_mdl_url, tmp_mdl, omni.client.CopyBehavior.OVERWRITE
            )
            if copy_result != omni.client.Result.OK:
                print(f"[scene_prep] Could not fetch original MDL from Nucleus: {nucleus_mdl_url}")
                continue

            with open(tmp_mdl, 'r', errors='replace') as f:
                orig_content = f.read()
            os.remove(tmp_mdl)

            with open(local_mdl, 'r', errors='replace') as f:
                local_content = f.read()

            orig_refs = [m.group(1) for m in texture_pattern.finditer(orig_content)]
            local_refs = [m.group(1) for m in texture_pattern.finditer(local_content)]

            for orig_ref, local_ref in zip(orig_refs, local_refs):
                # Resolve local expected path
                local_abs = os.path.normpath(os.path.join(os.path.dirname(local_mdl), local_ref))

                if os.path.exists(local_abs):
                    continue

                # Resolve absolute Nucleus URL for the texture
                if orig_ref.startswith('omniverse:'):
                    nucleus_tex_url = orig_ref
                else:
                    nucleus_tex_url = _resolve_nucleus_url(nucleus_materials_dir, orig_ref)

                os.makedirs(os.path.dirname(local_abs), exist_ok=True)
                result = omni.client.copy(
                    nucleus_tex_url, local_abs, omni.client.CopyBehavior.OVERWRITE
                )
                if result == omni.client.Result.OK:
                    downloaded += 1
                    print(f"[scene_prep] Downloaded: {os.path.basename(local_abs)}")
                else:
                    print(f"[scene_prep] Failed ({result}): {nucleus_tex_url}")

    print(f"[scene_prep] fix_missing_mdl_textures: {downloaded} texture(s) downloaded.")
    return downloaded
