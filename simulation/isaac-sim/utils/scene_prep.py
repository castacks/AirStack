"""
scene_prep.py — Utilities for preparing Isaac Sim / USD stages before simulation.

Functions:
    get_stage_meters_per_unit   — Read stage unit scale
    scale_stage_prim            — Apply a uniform scale transform to a prim
    add_colliders               — Recursively apply CollisionAPI to all meshes
    settle_selection            — Which placements to settle, per SCENE_SETTLE
    settle_rigid_props          — Physics-drop toppled props to rest, then freeze
    add_dome_light              — Add or update a dome light on the stage
    add_sky                     — Skybox from HDRI (dome texture) or borrowed stage prims
    save_scene_as_contained_usd — Collect all assets into a self-contained directory
"""

import asyncio
import math
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

def settle_selection(placements) -> list:
    """Prim paths to hand `settle_rigid_props`, filtered by `$SCENE_SETTLE`.

    Every launch script settles the same set — the placements the generator
    marked `settle` — so the choice of what to settle belongs here rather than
    being restated (and diverging) at five call sites.

    WHY THIS IS A KNOB
    ------------------
    Settling is meant to fix approximated poses: a toppled pole or flipped car
    is placed at a guessed orientation and physics finds the real one. But the
    same pass is also what fires interpenetrating debris across the map — a
    quarter of `debris` pieces spawn inside a standing building's footprint
    (`disaster_stage` emits on a ring around each damaged building without
    testing what is already there), and PhysX resolves that with a separating
    impulse. `settle_rigid_props` clamps that with `maxDepenetrationVelocity`
    and reverts anything travelling past `max_travel_m`, but "are the clamps
    holding?" is only answerable against a run with the pass disabled.

    So this is the A/B control for that experiment, not a tuning parameter.

        SCENE_SETTLE=1                     everything marked (default)
        SCENE_SETTLE=0                     nothing — no physics runs at all
        SCENE_SETTLE=-debris,debris_pile   everything EXCEPT those categories
        SCENE_SETTLE=debris_fragment       ONLY that category

    The `-` form is the useful one for the debris question: it leaves the
    toppled props settling as normal, so anything still out of place is not
    explained by "settling was off".

    Turning it off entirely is not free — props the generator placed at
    approximated poses stay floating or half-sunk. That is the trade the
    experiment is buying.
    """
    marked = [p for p in placements
              if p.get("settle") and p.get("prim_path")]
    raw = (os.environ.get("SCENE_SETTLE") or "1").strip()

    if raw.lower() in ("0", "false", "no", "none", "off"):
        print(f"[scene_prep] SCENE_SETTLE={raw} — settling disabled, "
              f"{len(marked)} marked prop(s) left at their authored pose")
        return []

    if raw.lower() not in ("1", "true", "yes", "all", "on"):
        exclude = raw.startswith("-")
        names = {c.strip() for c in raw.lstrip("-").split(",") if c.strip()}
        kept = [p for p in marked
                if (p.get("category") in names) != exclude]
        print(f"[scene_prep] SCENE_SETTLE={raw} — settling {len(kept)} of "
              f"{len(marked)} marked prop(s)")
        return [p["prim_path"] for p in kept]

    return [p["prim_path"] for p in marked]


def _local_centroid(prim):
    """Mean vertex of the meshes under *prim*, in *prim*'s own frame.

    Where a piece IS, as opposed to where its prim origin is. A fracture
    fragment is authored with no xformOp, so its origin is the BUILDING's
    origin — a piece 40 m up a tower that drops 40 m carries that origin 40 m
    below the ground, and one that merely rotates while falling sweeps it
    through tens of metres. Judged at the origin, half of a real collapse
    read as "fell through the ground" and most of the rest as "flung".
    """
    import numpy as np
    xf = UsdGeom.XformCache()
    acc, n = Gf.Vec3d(0.0, 0.0, 0.0), 0
    for gp in Usd.PrimRange(prim):
        if not gp.IsA(UsdGeom.Mesh):
            continue
        pts = gp.GetAttribute("points").Get()
        if not pts:
            continue
        m = xf.ComputeRelativeTransform(gp, prim)[0]
        c = np.asarray(pts, dtype=float).mean(axis=0)
        acc += m.Transform(Gf.Vec3d(*c)) * len(pts)
        n += len(pts)
    return acc / n if n else Gf.Vec3d(0.0, 0.0, 0.0)


#: Placement category of the fracture's loose pieces (`generate_scene`).
FRAGMENT = "debris_fragment"


def _centroids(prims, pivot) -> dict:
    """World centroid of every prim, from the transforms PhysX wrote back."""
    xf = UsdGeom.XformCache()
    return {p.GetPath(): xf.GetLocalToWorldTransform(p).Transform(pivot[p.GetPath()])
            for p in prims}


def settle_rigid_props(stage, prim_paths, sim_seconds: float = 3.0,
                       ground_path: str = None, max_travel_m: float = 12.0,
                       report: int = 5, gpu: bool = True,
                       placements=None) -> int:
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

    WHICH DEVICE THIS RUNS ON
    -------------------------
    Until this reported it, nobody knew. The function defines a bare
    `UsdPhysics.Scene` when the stage has none and otherwise takes whatever it
    finds, so the settle inherited PhysX's defaults (CPU dynamics, CPU
    broadphase, PGS) or the environment's settings, silently either way — and
    a settle that ran on the CPU is indistinguishable in the log from one that
    ran on the GPU with undersized buffers. Both are slow and both drop pieces
    through the floor.

    So the scene's actual configuration is now printed every run, along with a
    setup / cook / sim / freeze timing split. **Cooking is the part a GPU does
    not help**: a convex hull per mesh is CPU work that happens on the first
    frame after `play()`, which is why it is timed separately.

    *gpu* selects the device and defaults to True, which is worth 41x on a
    real collapse — 907 fragments of a fractured building settled in **2.4 s
    against 98.4 s** on the CPU scene the launcher used to inherit, at
    indistinguishable quality (335 vs 294 pieces lost through the ground, 18.5
    vs 18.8 m median travel). It applies `disaster.settle.configure_scene`,
    which is also where the GPU buffer capacities come from; raising them is
    not optional, because PhysX preallocates fixed buffers, overflow DROPS
    contacts rather than falling back, and the symptom is pieces sinking
    through the ground. The scene the launcher inherits carries the stock
    524288-contact buffer; this asks for 4M.

    False leaves it on the CPU, None leaves the scene exactly as found and only
    reports it. `SCENE_SETTLE_GPU=1` / `=0` sets it from the environment.

    WHAT IS ACTUALLY IN HERE
    ------------------------
    Two populations with nothing in common but a flag. The generator's
    approximated-pose props — a toppled streetlight, a flipped car, scattered
    debris — are a few hundred prims that physics is meant to *correct*. The
    fracture's loose fragments (`category: "debris_fragment"`, appended by
    `generate_scene`) are thousands of prims that physics is meant to *place*,
    and they arrive interpenetrating because a Voronoi cut leaves neighbouring
    cells sharing a face.

    They fail differently and they need different fixes, so pass *placements*
    and the outcome is reported per category. Without it the summary reads
    "691 fell through the ground" and cannot say whether the rubble is broken
    or the debris props are.

    Call BEFORE spawning robot/vehicle graphs: the timeline runs briefly.
    Returns the number of props settled.
    """
    import omni.timeline

    prims = [stage.GetPrimAtPath(p) for p in prim_paths]
    prims = [p for p in prims if p and p.IsValid()]
    if not prims:
        return 0

    t_setup = _time.time()
    cat_of = {str(p["prim_path"]): str(p.get("category") or "?")
              for p in (placements or ()) if p.get("prim_path")}

    if ground_path:
        ground = stage.GetPrimAtPath(ground_path)
        if ground.IsValid():
            add_colliders(ground)

    # A HALF-SPACE UNDER THE GROUND. The generated ground is a zero-thickness
    # quad, and a 0.3 m slab fragment arriving at the 20 m/s velocity cap
    # covers 0.33 m a step — so it can be above the quad on one step and
    # clear below it on the next, and PhysX never sees a contact. Measured on
    # a 72 m partial collapse: 142 of 872 fragments through the floor, most of
    # them released from 20-40 m up. An infinite plane collider is a solid
    # half-space, which nothing can pass; `disaster.settle.prepare` has used
    # one as a backstop since it first lost a piece to infinity. Just below
    # the ground so the ground's own collider still decides where things
    # rest, and deactivated again once the poses are frozen.
    floor_prim = UsdGeom.Xform.Define(stage, "/World/settle_floor").GetPrim()
    UsdGeom.Xformable(floor_prim).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, -0.1 / (UsdGeom.GetStageMetersPerUnit(stage) or 1.0)))
    floor_plane = UsdGeom.Plane.Define(stage, "/World/settle_floor/plane")
    floor_plane.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(floor_plane.GetPrim())
    floor_prim.SetActive(True)

    # Physics needs a scene; standalone preview stages may not have one.
    roots = list(stage.GetPseudoRoot().GetChildren())
    two_levels = roots + [c for r in roots for c in r.GetChildren()]
    if not any(p.IsA(UsdPhysics.Scene) for p in two_levels):
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        print("[scene_prep] settle: defined /World/PhysicsScene")

    # The device, and the buffers it was given. `disaster.settle` owns both
    # (one copy of `GPU_CAPACITIES`, not two — a drifted capacity fails
    # silently); it is imported lazily because this module is also used by
    # launch scripts that never put scene_gen on the path.
    if gpu is None:
        env = os.environ.get("SCENE_SETTLE_GPU", "").strip().lower()
        if env:
            gpu = env not in ("0", "false", "no", "off")
    try:
        from disaster import settle as _settle
    except ImportError:
        _settle = None
        if gpu is not None:
            print("[scene_prep] settle: SCENE_SETTLE_GPU set but scene_gen is "
                  "not importable — leaving the physics scene as found")
    if _settle is not None:
        if gpu is not None:
            _settle.configure_scene(stage, gpu=bool(gpu))
        cfg = _settle.describe_scene(stage)
        if not cfg:
            print("[scene_prep] settle: no physics scene found")
        elif not cfg.get("physx_api"):
            print(f"[scene_prep] settle: physics scene {cfg['scene']} has no "
                  "PhysxSceneAPI — PhysX defaults (CPU dynamics, PGS solver)")
        else:
            print(f"[scene_prep] settle: physics scene {cfg['scene']} — "
                  f"GPU dynamics {'ON' if cfg.get('gpu') else 'off'}, "
                  f"broadphase {cfg.get('broadphase')}, "
                  f"solver {cfg.get('solver')}, "
                  f"contact buffer {cfg.get('gpuMaxRigidContactCount')}")

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
    # from one that was fired. At each piece's own centroid — see
    # `_local_centroid` for why the prim origin is the wrong point.
    start_cache = UsdGeom.XformCache()
    pivot = {p.GetPath(): _local_centroid(p) for p in prims}
    start = {p.GetPath(): start_cache.GetLocalToWorldTransform(p)
             .Transform(pivot[p.GetPath()]) for p in prims}
    # The prim-ORIGIN ruler too, so one run reports both and the change of
    # ruler is checkable rather than asserted. Diagnostic only.
    start_origin = {p.GetPath(): start_cache.GetLocalToWorldTransform(p)
                    .ExtractTranslation() for p in prims}

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    # HOW LONG. `sim_seconds` was a fixed 3 s budget, which is less than a
    # piece needs just to FALL off a 66 m tower — so a tall collapse was
    # sampled mid-air, judged to have travelled, and reverted into the sky.
    # It is now the FLOOR. The ceiling is the time the highest piece needs to
    # reach the ground under the 20 m/s velocity cap plus that budget to come
    # to rest, and the loop below stops as soon as nothing is moving — so a
    # scene of parked cars still costs half a second of sim.
    g, vcap = 9.81, 20.0
    h = max((float(c[2]) * mpu for c in start.values()), default=0.0)
    fall = (h / vcap + vcap / (2.0 * g)) if h > vcap * vcap / (2.0 * g) \
        else math.sqrt(2.0 * max(h, 0.0) / g)
    ceiling_s = max(float(sim_seconds), fall + float(sim_seconds))
    steps = max(1, int(ceiling_s * 60.0))
    print(f"[scene_prep] settle: simulating {len(prims)} props "
          f"for up to {ceiling_s:.1f}s ({steps} frames; highest piece "
          f"{h:.0f} m)…", flush=True)
    setup_s = _time.time() - t_setup
    dt = 1.0 / 60.0

    # STEP THE PHYSICS, NOT THE APPLICATION. `app.update()` is a whole Kit
    # frame — renderer, Fabric sync, USD change notifications — and PhysX
    # writes every body's transform back into USD on every one of them. A
    # settle wants none of that: nothing reads the intermediate poses, only the
    # final one. So this steps the simulation directly and asks for the
    # write-back exactly once, at the end (`update_transformations`).
    #
    # Measured on 8,000 boxed bodies over 179 frames (`tools/settle_bench.py`),
    # same scene, same PhysX:
    #
    #     timeline.play() + app.update()              126.8 s
    #     stepped directly, write-back on              95.2 s
    #     stepped directly, write-back off              2.6 s
    #
    # THAT BENCHMARK IS TOO KIND, and the real scene says so: 907 fragments of
    # an actual fractured building cost 98.4 s here, 350x more per body-step
    # than the boxes. Clean convex boxes dropped into free space barely touch
    # the solver, so on them the USD write-back is everything; Voronoi shell
    # fragments spawn interpenetrating by metres and the CONTACT SOLVE is
    # everything. Both fixes are real, they just matter to different scenes —
    # which is why `gpu` defaults to True (98.4 s -> 2.4 s on that same real
    # collapse) and why this loop still avoids the per-step write-back.
    #
    # `SCENE_SETTLE_STEP=timeline` restores the old loop for comparison.
    stepper = os.environ.get("SCENE_SETTLE_STEP", "physx").strip().lower()
    physx = None
    if stepper != "timeline":
        try:
            from omni.physx import get_physx_interface
            physx = get_physx_interface()
        except ImportError:
            print("[scene_prep] settle: omni.physx unavailable — stepping the "
                  "timeline instead")

    t_cook = _time.time()
    if physx is not None:
        # COOKING IS ASYNC AND IT IS NOT OPTIONAL. A convex hull is cooked off
        # the main thread when the collider is authored; stepping before the
        # tasks finish simulates bodies that have no collision yet, which is
        # indistinguishable in the output from a body that fell through the
        # ground. There is no task-count query on the cooking interface in this
        # build, so this pumps a bounded number of frames and lets them drain.
        for _ in range(10):
            app.update()
        physx.start_simulation()
        cook_s = _time.time() - t_cook

        t_sim = _time.time()
        now = 0.0
        done = 0
        chunk = 30          # half a second of sim between rest checks
        prev = dict(start)
        while done < steps:
            n = min(chunk, steps - done)
            for _ in range(n):
                physx.update_simulation(dt, now)
                now += dt
            done += n
            # One write-back PER CHUNK, not per step (per step was the 95 s).
            # `updateToFastCache=False, updateToUsd=True`: the transforms have
            # to land in USD, because that is what the freeze pass below reads
            # and re-authors. Velocities come too, for the still-moving count.
            physx.update_transformations(False, True, True)
            # AT REST = nothing moved a millimetre in the last half second.
            # Judged on the written-back POSES, not on velocities: a body that
            # is asleep reports zero, and so would a velocity that never got
            # written — and the two are indistinguishable from here.
            # Below 0.1 m/s over the half second — the same creep the
            # still-moving count below tolerates. Tighter thresholds were
            # never met: a millimetre by a 900-body pile PhysX reported
            # asleep, 4 cm/s by 24 parked props (204 of 205 frames used).
            cur = _centroids(prims, pivot)
            if max((cur[k] - prev[k]).GetLength() for k in cur) < 0.05 / mpu:
                break
            prev = cur
        sim_s = _time.time() - t_sim
        steps = done
    else:
        timeline.stop()
        timeline.play()
        app.update()
        cook_s = _time.time() - t_cook
        t_sim = _time.time()
        for _ in range(steps - 1):
            app.update()
        sim_s = _time.time() - t_sim
    t_freeze = _time.time()

    # Capture settled transforms before stopping (stop resets the sim pose).
    floor_z = -2.0 / mpu    # anything below fell through the ground
    limit = max_travel_m / mpu
    cache = UsdGeom.XformCache()
    settled = {}
    for prim in prims:
        world = cache.GetLocalToWorldTransform(prim)
        parent_world = cache.GetLocalToWorldTransform(prim.GetParent())
        pos = world.Transform(pivot[prim.GetPath()])
        settled[prim.GetPath()] = (world * parent_world.GetInverse(), pos,
                                   world.ExtractTranslation())
    # AFTER the capture, in both paths: `stop()` and `reset_simulation()` both
    # restore the authored poses, so reading them afterwards reads the poses
    # the props started at rather than the ones they settled into.
    if physx is not None:
        physx.reset_simulation()
    else:
        timeline.stop()
    app.update()

    n_ok = n_lost = n_flung = n_moving = n_gone = 0
    old_lost = old_flung = 0
    z_lost, z_flung = [], []       # where the failures STARTED, in metres
    travel = []
    by_cat = {}
    for prim in prims:
        local, pos, origin = settled[prim.GetPath()]
        delta = pos - start[prim.GetPath()]
        if origin[2] < floor_z:
            old_lost += 1
        elif (origin - start_origin[prim.GetPath()]).GetLength() > limit:
            old_flung += 1
        is_frag = cat_of.get(str(prim.GetPath())) == FRAGMENT
        # A FRAGMENT IS MEANT TO FALL. Its vertical displacement is the
        # collapse working, so only the horizontal part counts as travel; and
        # its authored pose is where it was CUT — inside the intact shell,
        # tens of metres up — so there is nothing to revert it to. A fragment
        # the solver lost is deactivated instead of being left in the sky.
        moved = math.hypot(delta[0], delta[1]) if is_frag else delta.GetLength()
        travel.append((moved * mpu, prim.GetPath()))
        # A piece off the top of a tower legitimately lands further out than
        # one off a bungalow: a collapse's debris field runs to about half
        # the drop. `max_travel_m` stays the floor of the limit. Measured on
        # the 72 m partial collapse with the backstop in: at a third of the
        # drop 181 of 884 pieces were over the line, from a median start of
        # 35 m — rubble across the street, not a solver launch (those read
        # 130 m). Half the drop keeps the street rubble.
        z0 = float(start[prim.GetPath()][2])
        lim = max(limit, 0.5 * z0) if is_frag else limit
        cat = by_cat.setdefault(cat_of.get(str(prim.GetPath()), "?"),
                                {"n": 0, "ok": 0, "lost": 0, "flung": 0,
                                 "moving": 0})
        cat["n"] += 1
        # STILL MOVING IS NOT SETTLED, AND THE DIFFERENCE IS NOT COSMETIC.
        # The loop above stops when everything is at rest or when the ceiling
        # runs out; only the second case leaves bodies moving here, and then
        # the poses below are sampled mid-flight. Counting them is what
        # separates "the settle threw it" from "the settle was not finished".
        vel = prim.GetAttribute("physics:velocity").Get()
        speed = Gf.Vec3f(*vel).GetLength() if vel is not None else 0.0
        if speed > 0.1 / mpu:
            n_moving += 1
            cat["moving"] += 1
        if pos[2] < floor_z:
            n_lost += 1     # tunnelled through the ground — keep authored pose
            cat["lost"] += 1
            z_lost.append(float(start[prim.GetPath()][2]) * mpu)
            if is_frag:
                prim.SetActive(False)
                n_gone += 1
        elif moved > lim:
            # Launched out of a penetration. A prop's authored spot is where
            # the generator meant it to be; a solver artifact hundreds of
            # metres away is not an improvement on that. A fragment has no
            # such spot — see above.
            n_flung += 1
            cat["flung"] += 1
            z_flung.append(float(start[prim.GetPath()][2]) * mpu)
            if is_frag:
                prim.SetActive(False)
                n_gone += 1
        else:
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(local)
            n_ok += 1
            cat["ok"] += 1
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    floor_prim.SetActive(False)

    travel.sort(reverse=True)
    med = travel[len(travel) // 2][0] if travel else 0.0
    if n_moving:
        print(f"[scene_prep] settle: NOT CONVERGED — {n_moving} of {len(prims)}"
              f" still moving at {steps / 60.0:.1f}s. Poses below are sampled "
              f"mid-motion, and `max_travel_m` is reverting some of them for "
              f"having fallen.")
    print(f"[scene_prep] settle: {len(prims)} props | setup {setup_s:.1f}s  "
          f"cook {cook_s:.1f}s  sim {sim_s:.1f}s ({steps - 1} frames)  "
          f"freeze {_time.time() - t_freeze:.1f}s")
    print(f"[scene_prep] settle: froze {n_ok} props at rest"
          + (f", {n_lost} fell through the ground (kept authored pose)"
             if n_lost else "")
          + (f", {n_flung} flung past {max_travel_m:.0f} m (reverted)"
             if n_flung else "")
          + (f" — {n_gone} of those are fragments and were deactivated"
             if n_gone else ""))
    if travel:
        print(f"[scene_prep] settle: travel median {med:.2f} m, "
              f"max {travel[0][0]:.1f} m")
    for name, zs in (("through-floor", z_lost), ("flung", z_flung)):
        if zs:
            zs = sorted(zs)
            q = lambda f: zs[min(len(zs) - 1, int(f * len(zs)))]
            print(f"[scene_prep] settle: {name} pieces started at z = "
                  f"{q(0.0):.1f} / {q(0.25):.1f} / {q(0.5):.1f} / "
                  f"{q(0.75):.1f} / {zs[-1]:.1f} m (min/q1/median/q3/max)")
    if old_lost or old_flung:
        print(f"[scene_prep] settle: (the prim-origin ruler would have called "
              f"{old_lost} through-floor and {old_flung} flung — see "
              f"`_local_centroid`)")
    # Per category, worst first by what did NOT settle: the fragments and the
    # props fail for different reasons and only this says which is which.
    for name, c in sorted(by_cat.items(),
                          key=lambda kv: -(kv[1]["lost"] + kv[1]["flung"])):
        if len(by_cat) < 2:
            break
        print(f"[scene_prep] settle:   {name:<18} {c['n']:>5}  "
              f"rest {c['ok']:>5}  through-floor {c['lost']:>5}  "
              f"flung {c['flung']:>5}  still-moving {c['moving']:>5}")
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
