"""settle stage — drop the loose pieces, let PhysX arrange them, bake the result.

WHY BAKE
--------
The pieces of a collapsed house need to be simulated exactly once. Nothing in
the dataset moves at capture time, and leaving a few hundred rigid bodies live
across a suburb costs frame rate forever in exchange for geometry that is
already at rest. So: enable physics, step it until the pile stops moving, write
the resulting transforms back as plain static transforms, and switch the bodies
off. What ships is ordinary geometry that happens to be arranged the way
gravity arranged it.

WHY NOT A FORMULA
-----------------
Felling by formula — rotate 80 degrees, drop to z=0.2, push outward — gives
panels at plausible angles that intersect each other and float where they
should have come to rest on something. A roof panel landing ON a standing wall
is the specific thing a solver gets right and arithmetic cannot: the answer
depends on what else is in the way.

NO FRACTURE IS INVOLVED. This build has no Blast extension and meshes are never
cut. The pieces are whole kit modules, already separate prims, which is what
makes the whole approach possible — see `disaster.damage`.

COLLIDERS ARE CONVEX HULLS
--------------------------
Per module, not per scene. Kit panels are near-flat slabs, so a hull is a close
fit and cooks in a fraction of the time convex decomposition takes. Anything
with a deep opening — a garage wall, a bay window — is the case where this
shows, and the fix there is `approximation="convexDecomposition"` on that prim
rather than on all of them.
"""

import carb

#: Cap on how fast PhysX may push two overlapping bodies apart, m/s. See the
#: note where it is applied. This is the knob that decides whether a pile
#: settles or inflates, and it is exposed because the right value depends on
#: how much the colliders overlap to begin with — which for convex hulls of
#: Voronoi-cut SHELL fragments can be metres, not millimetres.
DEPENETRATION_MS = 0.6


def _apply_collider(prim, approximation="convexHull", skip_dynamic=False):
    """Collision on every mesh under *prim*. Returns how many were set.

    `skip_dynamic` passes over anything that is already a rigid body. That is
    what makes it safe to hand over a whole BUILDING as static geometry: the
    single-mesh AEC assets are a Mesh at their default prim, the fragments cut
    out of them are authored as its children, and walking the subtree would
    otherwise put a triangle-mesh collider on every dynamic body in the pile.
    """
    from pxr import UsdGeom, UsdPhysics

    n = 0
    for p in _iter(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        if skip_dynamic and p.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        pts = p.GetAttribute("points")
        # The kit ships material-only USDs whose meshes carry no points; PhysX
        # logs an error per empty mesh, which buries anything real.
        if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(p)
        mesh_api.CreateApproximationAttr().Set(approximation)
        n += 1
    return n


def _iter(prim):
    from pxr import Usd
    return Usd.PrimRange(prim)


def blast_velocities(stage, paths, speed, centre=None, up=0.35, falloff=1.0):
    """Outward radial launch velocity per body: `{path: (vx, vy, vz)}`.

    WHY A BLAST AND NOT A BIGGER `kick`
    -----------------------------------
    `kick` is isotropic noise — it breaks the symmetry of an interlocked stack
    but has no preferred direction, so raising it just jitters the pile harder
    in place. Voronoi fragments tile the original volume with ZERO gap, and a
    tiling of convex cells under gravity alone is a stable arch: measured on
    the 96 m tower, 30 of 30 bodies were still grinding against each other
    after 1500 steps having dropped 3 m. The pieces have to be moved APART
    before gravity has anything to work with.

    So this is a radial impulse about the structure's own vertical axis,
    strongest at the centre and decaying outward, with an upward component so
    the pile opens rather than merely sliding. That is what a collapse looks
    like from the inside: material is thrown off the axis, then falls.

    *centre* defaults to the plan centroid of *paths* at their lowest z — the
    base of the structure, so pieces are thrown out and up rather than down
    into the ground. *falloff* scales the radius over which the speed decays;
    *up* is the vertical share of the speed.

    Returns a dict rather than authoring anything, so the caller decides when
    the velocities land — they must be assigned AFTER the bodies are dynamic.
    """
    import numpy as np

    from pxr import Usd, UsdGeom

    if speed <= 0.0 or not paths:
        return {}

    # THE BOUNDING BOX, NOT THE TRANSFORM. A fragment's position may live in
    # either place depending on who cut it: `vtk_fracture` centres each piece
    # on its centroid and puts that in an xformOp, while `mesh_damage` authors
    # points in the root's local space and gives the fragment NO transform at
    # all. Reading the translate returns the parent's origin for every one of
    # the latter, so every radius came out zero, every outward direction was
    # undefined, and the whole blast collapsed into one uniform shove straight
    # up. The world bound is where the geometry actually is, either way.
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    cen = {}
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            continue
        box = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        cen[p] = np.array(box.GetMidpoint(), dtype=float)
    if not cen:
        return {}

    pts = np.array(list(cen.values()))
    if centre is None:
        centre = np.array([pts[:, 0].mean(), pts[:, 1].mean(), pts[:, 2].min()])
    centre = np.asarray(centre, dtype=float)

    horiz = pts[:, :2] - centre[:2]
    radius = max(1e-6, float(np.linalg.norm(horiz, axis=1).max()))

    out = {}
    for p, c in cen.items():
        d = c[:2] - centre[:2]
        r = float(np.linalg.norm(d))
        unit = d / r if r > 1e-6 else np.zeros(2)
        # Linear decay to zero at the outer edge: the core is thrown hardest,
        # the facade barely at all, which is what keeps the footprint from
        # ballooning outward into a starburst.
        mag = float(speed) * max(0.0, 1.0 - r / (radius * max(1e-6, falloff)))
        out[p] = (float(unit[0] * mag), float(unit[1] * mag),
                  float(mag * up))
    return out


#: GPU BUFFER CAPACITIES, and why they are all raised at once.
#:
#: PhysX preallocates fixed GPU buffers and does not grow them: overflow does
#: not fall back to CPU, it DROPS contacts, and the symptom is pieces sinking
#: through each other or through the ground with only a warning in the log.
#: The defaults are sized for a few hundred bodies. Everything here is well
#: past what a single collapse needs, because the failure is silent and the
#: memory is cheap next to a card already holding the scene.
#:
#: Attribute name -> value, so `describe_scene` can read back exactly what
#: `configure_scene` claims to have set.
GPU_CAPACITIES = {
    "gpuMaxRigidContactCount": 4 * 1024 * 1024,
    "gpuMaxRigidPatchCount": 1024 * 1024,
    "gpuFoundLostPairsCapacity": 2 * 1024 * 1024,
    "gpuFoundLostAggregatePairsCapacity": 64 * 1024,
    "gpuTotalAggregatePairsCapacity": 64 * 1024,
    "gpuHeapCapacity": 256 * 1024 * 1024,
    "gpuTempBufferCapacity": 64 * 1024 * 1024,
    "gpuMaxNumPartitions": 8,
}


def configure_scene(stage, scene_prim=None, gpu: bool = True) -> dict:
    """Put the settle's PhysX settings on a physics scene. Returns what it set.

    THE SETTLE IS THE ONE PHASE HERE THAT CAN USE THE CARD — mesh slicing and
    texture compositing are CPU libraries end to end — and it is the phase
    carrying thousands of bodies, which is where a GPU broadphase and solver
    earn their keep.

    Shared rather than inlined in `prepare` because the SCENE-WIDE settle
    (`simulation/isaac-sim/utils/scene_prep.py:settle_rigid_props`) needs the
    identical configuration and had none of it: it defined a bare
    `UsdPhysics.Scene` and inherited whatever PhysX defaults happened to be in
    force. Two copies of `GPU_CAPACITIES` would drift, and a drifted capacity
    fails silently — see the note there.
    """
    from pxr import PhysxSchema, UsdPhysics

    if scene_prim is None:
        scene_prim = next((p for p in stage.Traverse()
                           if p.IsA(UsdPhysics.Scene)), None)
    if scene_prim is None:
        return {}

    sx = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    sx.CreateEnableGPUDynamicsAttr(bool(gpu))
    sx.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    # TGS converges far better than PGS on deep stacks, which is exactly what
    # a collapsed building is.
    sx.CreateSolverTypeAttr("TGS")
    was = {"gpu": bool(gpu), "broadphase": "GPU" if gpu else "MBP",
           "solver": "TGS", "scene": str(scene_prim.GetPath())}
    if gpu:
        for name, value in GPU_CAPACITIES.items():
            attr = getattr(sx, "Create" + name[0].upper() + name[1:] + "Attr")
            attr().Set(value)
        was.update(GPU_CAPACITIES)
    return was


def describe_scene(stage) -> dict:
    """What the stage's physics scene is ACTUALLY set to, or {} if there is none.

    Reported rather than assumed: a settle that silently ran on the CPU and one
    that ran on the GPU with undersized buffers look the same from the outside
    — slow, and pieces through the floor — and neither announces itself.
    """
    from pxr import PhysxSchema, UsdPhysics

    prim = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
    if prim is None:
        return {}
    out = {"scene": str(prim.GetPath()),
           "physx_api": bool(prim.HasAPI(PhysxSchema.PhysxSceneAPI))}
    if not out["physx_api"]:
        # No PhysxSceneAPI at all: PhysX runs this scene on its own defaults,
        # which is CPU dynamics with a CPU broadphase and the PGS solver.
        return out
    sx = PhysxSchema.PhysxSceneAPI(prim)
    def _get(getter, default=None):
        attr = getattr(sx, getter, None)
        if attr is None:
            return default
        got = attr().Get()
        return default if got is None else got
    out["gpu"] = bool(_get("GetEnableGPUDynamicsAttr", False))
    out["broadphase"] = str(_get("GetBroadphaseTypeAttr", "?"))
    out["solver"] = str(_get("GetSolverTypeAttr", "?"))
    for name in GPU_CAPACITIES:
        out[name] = _get("Get" + name[0].upper() + name[1:] + "Attr")
    return out


def prepare(stage, loose_paths, static_paths, gravity=-9.81,
            scene_path="/World/physicsScene", kick=0.0, rng=None,
            dynamic_approximation="convexHull", approx_map=None, gpu=True,
            blast=0.0, blast_center=None, blast_up=0.35, blast_falloff=1.0,
            depenetration=DEPENETRATION_MS):
    """Physics scene, static colliders, and a rigid body per loose piece."""
    import random as _random

    from pxr import Gf, Sdf, UsdPhysics, UsdShade
    from pxr import PhysxSchema

    rng = rng or _random.Random(0)
    burst = blast_velocities(stage, loose_paths, blast, centre=blast_center,
                             up=blast_up, falloff=blast_falloff)
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(abs(float(gravity)))

    configure_scene(stage, scene.GetPrim(), gpu=gpu)

    # A true infinite plane as a backstop. Even with the fix above, one
    # uncooked collider is enough to lose a piece to infinity, and a piece
    # that fell 25 m is indistinguishable in the logs from one that settled.
    # One physics material for everything: high friction, no bounce. Rubble
    # does not skitter and it does not rebound.
    mat_path = scene_path + "/rubbleMaterial"
    mat = UsdShade.Material.Define(stage, mat_path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.95)
    pm.CreateDynamicFrictionAttr(0.85)
    pm.CreateRestitutionAttr(0.0)

    from pxr import UsdGeom
    floor = UsdGeom.Xform.Define(stage, scene_path + "/floor").GetPrim()
    plane = UsdPhysics.CollisionAPI.Apply(floor)
    plane.CreateCollisionEnabledAttr(True)
    floor.CreateAttribute("physics:approximation",
                          Sdf.ValueTypeNames.Token).Set("none")
    UsdGeom.Plane.Define(stage, scene_path + "/floor/plane").CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(
        stage.GetPrimAtPath(scene_path + "/floor/plane"))

    n_static = n_body = 0
    bodies = []
    for path in loose_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        # `convexDecomposition` EXISTS FOR TREES. A wall fragment is roughly
        # convex already, so its hull is the piece and `convexHull` is both
        # right and cheap. A fallen bole is not: it is a trunk with limbs
        # coming off it, and its hull is the 20 m blob that contains all of
        # them — so the trunk comes to rest balanced on the hull of its own
        # branch tips, a metre clear of the ground, and reads as floating.
        # Decomposition costs cooking time and fixes exactly that.
        #
        # PER-PATH, because that cost is real and only a handful of pieces
        # need it. A debris stick is convex to within its own bark; cooking a
        # decomposition for three thousand of them buys nothing and dominates
        # start-up. `approx_map` lets the caller spend it where it matters.
        approx = dynamic_approximation
        if approx_map:
            approx = approx_map.get(path, dynamic_approximation)
        if not _apply_collider(prim, approximation=approx):
            continue          # nothing to collide with; leave it where it is
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateRigidBodyEnabledAttr(True)
        # DAMPED AND STICKY, so this reads as a collapse and not a detonation.
        # Undamped rigid bodies with default restitution behave like billiard
        # balls: every contact converts a drop into lateral speed and the pile
        # sprays outward. Real rubble is lossy — it lands, grinds, and stops.
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateLinearDampingAttr(0.55)
        px.CreateAngularDampingAttr(2.2)
        # THE CAP HAS TO CLEAR BOTH THE BLAST AND FREE FALL, and it used to
        # clear neither. It was 4 m/s, described as "right for a piece that is
        # only falling" — but a fragment dropping off a 66 m building reaches
        # 36 m/s, so 4 m/s is not a safety cap on falling, it IS the fall
        # speed. Everything descended in slow motion, nothing landed inside
        # the step budget, and pieces nudged sideways kept travelling at the
        # cap for the whole simulation: measured on a 1,161-cell collapse,
        # 703 of 755 bodies were still moving at the ceiling and the spread
        # reached 116 m. That reads as a detonation and was a clamp.
        #
        # 30 m/s is free fall from about 45 m, which covers the library, and
        # the cap still catches a solver blow-up.
        px.CreateMaxLinearVelocityAttr(max(30.0, float(blast) * 1.5))
        px.CreateMaxAngularVelocityAttr(5.0)
        # THE EXPLOSION KNOB. When PhysX finds two bodies overlapping it pushes
        # them apart, and by default it may do so at any speed — which turns
        # fracture fragments that start in contact into shrapnel. Capping the
        # separation speed makes penetration resolve as a shove instead.
        px.CreateMaxDepenetrationVelocityAttr(float(depenetration))
        px.CreateSolverPositionIterationCountAttr(16)
        px.CreateSolverVelocityIterationCountAttr(2)
        px.CreateSleepThresholdAttr(0.02)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath(mat_path)),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        # A NUDGE, because Voronoi fragments start perfectly interlocked.
        # They tile the original volume with zero gap, so a fractured wall is
        # a stable stack and gravity alone leaves it standing — cut, but not
        # collapsed. A small random kick breaks the symmetry and lets it go.
        # Just enough to break the interlock. The previous values — 0.9 m/s
        # with +-25 rad/s of spin, i.e. four revolutions a second — were an
        # explosion, not a nudge.
        #
        # The BLAST is the directed half of the same job and composes with it:
        # `kick` decides that a piece moves, `blast` decides which way. See
        # `blast_velocities`.
        bv = burst.get(path)
        if kick > 0.0 or bv:
            base = bv or (0.0, 0.0, 0.0)
            body.CreateVelocityAttr(Gf.Vec3f(
                float(base[0] + rng.uniform(-kick, kick)),
                float(base[1] + rng.uniform(-kick, kick)),
                float(base[2])))
            body.CreateAngularVelocityAttr(Gf.Vec3f(
                float(rng.uniform(-kick, kick)),
                float(rng.uniform(-kick, kick)),
                float(rng.uniform(-kick, kick))))
        UsdPhysics.MassAPI.Apply(prim).CreateDensityAttr(420.0)   # timber
        bodies.append(prim)
        n_body += 1

    # STATIC GEOMETRY GETS A TRIANGLE MESH, NOT A HULL. The ground is a flat
    # quad and the convex hull of a planar polygon is degenerate — PhysX has
    # no volume to cook, so the collider silently does nothing and every
    # fragment falls straight through the world. Static colliders may be
    # concave in PhysX, so "none" (use the real triangles) is both valid and
    # more accurate here; only the dynamic bodies need hulls.
    #
    # AND IT RUNS LAST, skipping anything already dynamic. The static list may
    # name a whole building — the part of it the fracture left standing — and
    # on a single-mesh asset the fragments are that prim's own children. Doing
    # this pass first, or without the guard, cooks a triangle-mesh collider
    # onto every rigid body in the pile.
    for path in static_paths:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            n_static += _apply_collider(prim, approximation="none",
                                        skip_dynamic=True)

    return {"bodies": bodies, "static_meshes": n_static, "rigid": n_body}


def bake(stage, bodies):
    """Freeze each body where it came to rest, then switch physics off.

    The transform is read as local-to-world and re-authored as translate /
    orient / scale. Rebuilding the op order rather than editing the existing
    ops matters: PhysX may have authored an `orient` alongside the `rotateXYZ`
    that `apply_placements` wrote, and leaving both composes them twice.

    THE OPS ARE REMOVED, NOT JUST UNORDERED
    ---------------------------------------
    `ClearXformOpOrder` clears the ORDER and leaves the attributes behind, and
    `AddXformOp` refuses to re-type an attribute that already exists — it
    raises `has typeName 'float3' which does not match the requested precision
    'PrecisionDouble'`, which pxr surfaces to Python as an exception, so the
    whole bake dies partway through.

    Whether it fires depends on who authored the translate first, which is why
    it hid for so long. A fragment that comes out of `vtk_fracture._write_mesh`
    already has a `Gf.Vec3d` translate, so PhysX updates that same double
    attribute and the precision matches. A fragment from
    `mesh_damage.fracture_to_stage` has NO transform at all — its points are
    authored in the root's local space — so PhysX creates `xformOp:translate`
    itself at FLOAT precision, and every mesh-damaged building failed to bake.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    xf = UsdGeom.XformCache()
    n = 0
    for prim in bodies:
        if not prim or not prim.IsValid():
            continue
        # PARENT SPACE, NOT WORLD SPACE. An xformOp is read relative to the
        # prim's parent, so authoring the local-to-world matrix is only correct
        # when the parent is the identity. `vtk_fracture` puts its fragments
        # under a plain Scope and got away with it; `mesh_damage` parents its
        # fragments to the BUILDING, which carries the placement's translate,
        # rotate and scale — so a world matrix authored there applies all three
        # a second time. On the centimetre-authored AEC packs (`scale: 0.01`)
        # that shrinks every settled fragment 100x and throws it off, which
        # reads as the rubble having vanished. Same double-transform trap
        # `mesh_damage.fracture_to_stage` documents for authoring POINTS.
        m = xf.GetLocalToWorldTransform(prim)
        parent = prim.GetParent()
        if parent and not parent.IsPseudoRoot():
            m = m * xf.GetLocalToWorldTransform(parent).GetInverse()
        tr = Gf.Transform(m)
        t = tr.GetTranslation()
        q = tr.GetRotation().GetQuat()
        sc = tr.GetScale()

        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
        for name in list(prim.GetPropertyNames()):
            if name.startswith("xformOp:"):
                prim.RemoveProperty(name)
        x.AddTranslateOp().Set(Gf.Vec3d(t))
        x.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
        x.AddScaleOp().Set(Gf.Vec3f(sc))

        # Off, not removed: dropping the API would also drop the collider, and
        # a disabled body is what makes this static geometry again.
        UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        n += 1
    return n


def _positions(bodies):
    from pxr import UsdGeom
    xf = UsdGeom.XformCache()
    return {str(b.GetPath()): xf.GetLocalToWorldTransform(b).ExtractTranslation()
            for b in bodies if b and b.IsValid()}


def _step(steps, dt=1.0 / 60.0):
    """Step physics, explicitly.

    `timeline.play()` plus `app.update()` is NOT reliably enough in a
    standalone SimulationApp: nothing attaches the physics scene, so the app
    renders happily while the solver never runs and every body stays exactly
    where it was authored. `SimulationContext` does the attach and steps the
    solver on demand, which is what makes the collapse actually happen.
    """
    try:
        from isaacsim.core.api import SimulationContext
        sc = SimulationContext(stage_units_in_meters=1.0)
        sc.initialize_physics()
        sc.play()
        for _ in range(int(steps)):
            sc.step(render=False)
        sc.pause()
        return "SimulationContext"
    except Exception as exc:
        carb.log_warn("[settle] SimulationContext unavailable ({0}); "
                      "falling back to the timeline".format(exc))
        import omni.kit.app
        import omni.timeline
        tl = omni.timeline.get_timeline_interface()
        app = omni.kit.app.get_app()
        tl.play()
        for _ in range(int(steps)):
            app.update()
        tl.stop()
        return "timeline"


def run(stage, loose_paths, static_paths, steps=360, settle_note=True,
        gravity=-9.81, kick=0.0, rng=None, bake_result=True,
        dynamic_approximation="convexHull", approx_map=None, gpu=True,
        blast=0.0, blast_center=None, blast_up=0.35, blast_falloff=1.0,
        depenetration=DEPENETRATION_MS):
    """prepare -> step physics -> measure -> bake. Returns a short report.

    MEASURES WHAT MOVED. A settle that silently does nothing looks identical
    to one that ran and found everything already at rest, and the two need
    completely different fixes — so the mean and max displacement are reported
    rather than assumed.

    `bake_result=False` leaves the bodies live, which is what you want when
    hand-testing in the viewport: drag a piece into the air, press play, and
    it should fall. With baking on (the default for capture) every body is
    disabled at the end and pressing play correctly does nothing.
    """
    import numpy as np

    info = prepare(stage, loose_paths, static_paths, gravity=gravity,
                   kick=kick, rng=rng,
                   dynamic_approximation=dynamic_approximation,
                   approx_map=approx_map, gpu=gpu,
                   blast=blast, blast_center=blast_center,
                   blast_up=blast_up, blast_falloff=blast_falloff,
                   depenetration=depenetration)
    if not info["bodies"]:
        carb.log_warn("[settle] nothing to settle")
        return info

    import time as _time
    _t0 = _time.time()
    before = _positions(info["bodies"])

    # STEP UNTIL IT STOPS MOVING, not for a fixed count. A fixed budget is a
    # guess about the slowest pile in the scene, and baking freezes whatever
    # it finds — so guessing low leaves houses frozen mid-collapse, which is
    # what "a lot of the house hasn't fallen" is. Raising the number is only
    # ever a bigger guess; measuring is not.
    #
    # `steps` is now a CEILING. The loop advances in chunks and stops early
    # once the busiest body in a chunk moves less than a millimetre, and it
    # reports how much of the budget it actually needed — so "not enough
    # steps" becomes visible instead of being inferred from the render.
    chunk = max(30, int(steps) // 12)
    used = 0
    prev = before
    info["driver"] = None
    while used < int(steps):
        n = min(chunk, int(steps) - used)
        info["driver"] = _step(n)
        used += n
        now = _positions(info["bodies"])
        moved = max((float(np.linalg.norm(np.array(now[k]) - np.array(prev[k])))
                     for k in prev if k in now), default=0.0)
        prev = now
        if moved < 0.001:
            break
    info["steps_used"] = used
    after = _positions(info["bodies"])

    # And one last look: how many bodies were STILL MOVING when the budget ran
    # out. Zero means the pile is genuinely at rest and baking is safe; a
    # non-zero count is the scene telling you the ceiling is too low.
    _step(20)
    settled = _positions(info["bodies"])
    info["still_moving"] = sum(
        1 for k in after
        if k in settled
        and float(np.linalg.norm(np.array(settled[k]) - np.array(after[k])))
        > 0.004)
    after = settled

    # HORIZONTAL vs VERTICAL is the whole question. A collapse drops pieces:
    # large -Z, small XY. An explosion throws them: large XY. One number for
    # total displacement cannot tell the two apart, and that is exactly the
    # judgement being made here.
    dv = [float(np.array(after[k])[2] - np.array(before[k])[2])
          for k in before if k in after]
    dh = [float(np.linalg.norm((np.array(after[k]) - np.array(before[k]))[:2]))
          for k in before if k in after]
    d = [float(np.linalg.norm(np.array(after[k]) - np.array(before[k])))
         for k in before if k in after]
    info["moved_mean"] = float(np.mean(d)) if d else 0.0
    info["moved_max"] = float(np.max(d)) if d else 0.0
    info["drop_mean"] = float(np.mean(dv)) if dv else 0.0
    # AND THE MEDIAN, which is the one to tune against. A pile is thousands of
    # bodies and a handful of them get launched — one slab catapulted off the
    # top of the wreckage carried the mean from -3 m to +7.4 m on BG_Building_F
    # and read as an explosion that never happened. The median says where the
    # material went; the mean says whether anything got thrown.
    info["drop_median"] = float(np.median(dv)) if dv else 0.0
    info["spread_mean"] = float(np.mean(dh)) if dh else 0.0
    info["spread_max"] = float(np.max(dh)) if dh else 0.0

    info["solve_s"] = _time.time() - _t0
    info["baked"] = bake(stage, info["bodies"]) if bake_result else 0
    if settle_note:
        print("[settle] {0} rigid, {1} static, driver={2}, baked {3}".format(
            info["rigid"], info["static_meshes"], info["driver"],
            info["baked"]))
        print("[settle]   {0:.1f}s solving ({1})".format(
            info.get("solve_s", 0.0), "GPU" if gpu else "CPU"))
        print("[settle]   {0} of {1} steps used; {2} body(s) STILL MOVING at "
              "bake time{3}".format(
                  info.get("steps_used", steps), steps,
                  info.get("still_moving", 0),
                  "" if not info.get("still_moving")
                  else "  <-- RAISE THE STEP BUDGET"))
        print("[settle]   drop  median {0:+.2f} m / mean {1:+.2f} m   "
              "(down = collapsing)".format(info["drop_median"],
                                           info["drop_mean"]))
        print("[settle]   spread mean {0:.2f} m / max {1:.2f} m   "
              "(large = exploding)".format(info["spread_mean"],
                                           info["spread_max"]))
        if info["moved_max"] < 0.01:
            print("[settle] NOTHING MOVED — the solver did not run, or every "
                  "body was already resting and interlocked")
    return info
