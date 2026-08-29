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

NOTHING MAY BE MOVING WHEN THIS BAKES
-------------------------------------
`bake()` freezes whatever it finds, so a body still in flight is exported as a
fragment hanging in the air, and no amount of downstream geometry cleanup can
tell it from one that legitimately came to rest on something. Three mechanisms
put material where it does not belong, and all three are handled here rather
than after the fact:

  * **the step count runs out.** `steps` has always been a ceiling with an
    early exit; `converge=True` makes it a floor as well — the loop keeps
    going, up to `max_steps`, until nothing is moving or the count of moving
    bodies stops improving. A ceiling is a guess about the slowest pile in
    the scene; convergence is a measurement.
  * **the throw never ends.** A settle with a `bias` is ballistic: pieces are
    still travelling when a plain collapse would be long finished.
    `quiet_steps` runs a second phase after the throw with the damping raised
    and the speed cap lowered — no new impulse, so the downwind lean the bias
    bought is untouched, but airborne pieces come down and resting ones stop
    creeping.
  * **the floor is not a floor.** A four-vertex ground quad cooked as a
    triangle mesh is infinitely thin, and a fragment thrown at 9 m/s covers a
    third of a metre per 60 Hz step — more than its own thickness, so it can
    be wholly through the mesh by the end of the step and never generate a
    contact. `ground_plane_z` authors a real PhysX half-space (a
    `UsdGeomPlane` collider, the shape omni.physx's own `add_ground_plane`
    uses) which cannot be tunnelled at any speed, and `ccd=True` turns on
    continuous collision detection so the pieces cannot tunnel each other
    either. `floor_z` is the belt to that braces: anything that still
    finishes below grade is lifted back onto it.

`SettleNotConverged` and the `!!!!` banner exist because the previous version
of this printed a note. A bake that ships 200 airborne fragments must be
impossible to mistake for one that worked.
"""

import carb


class SettleNotConverged(RuntimeError):
    """The pile was still moving, or below grade, when it was asked to bake.

    Raised only when the caller asks for it (`strict=True`, or the
    `SETTLE_STRICT` env var). Everything it reports is also printed, loudly,
    whether or not it is raised — see `run`.
    """


def _apply_collider(prim, approximation="convexHull", decomp_limits=None):
    """Collision on every mesh under *prim*. Returns how many were set."""
    from pxr import UsdGeom, UsdPhysics

    n = 0
    for p in _iter(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = p.GetAttribute("points")
        # The kit ships material-only USDs whose meshes carry no points; PhysX
        # logs an error per empty mesh, which buries anything real.
        if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(p)
        mesh_api.CreateApproximationAttr().Set(approximation)
        if approximation == "convexDecomposition":
            _bound_decomposition(p, decomp_limits)
        n += 1
    return n


# What the cooker is allowed to spend on ONE decomposition. Uncapped, a
# branchy or long-thin piece hits `ConvexDecompositionTask: polygon limit
# reached`, and the hull set that comes out of that makes every solver step
# crawl — the process sits at 30% of one core, logging nothing, looking hung.
# That is a measured 20-minute settle where hulls take 61 seconds.
DECOMP_LIMITS = {"max_hulls": 8, "vertex_limit": 32,
                 "voxel_resolution": 100000, "min_thickness": 0.01}


def _bound_decomposition(prim, limits=None):
    """Cap the convex decomposition of one mesh. Silent if the API is absent."""
    lim = dict(DECOMP_LIMITS)
    lim.update(limits or {})
    try:
        from pxr import PhysxSchema
        api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
        api.CreateMaxConvexHullsAttr(int(lim["max_hulls"]))
        api.CreateHullVertexLimitAttr(int(lim["vertex_limit"]))
        api.CreateVoxelResolutionAttr(int(lim["voxel_resolution"]))
        api.CreateMinThicknessAttr(float(lim["min_thickness"]))
    except Exception as exc:                       # pragma: no cover
        carb.log_warn("[settle] convex decomposition limits unavailable "
                      "({0}); cooking with the defaults".format(exc))


def _iter(prim):
    from pxr import Usd
    return Usd.PrimRange(prim)


def prepare(stage, loose_paths, static_paths, gravity=-9.81,
            scene_path="/World/physicsScene", kick=0.0, rng=None,
            dynamic_approximation="convexHull", approx_map=None, gpu=True,
            bias=None, max_speed=None, damping=None, velocity_map=None,
            density=420.0, ccd=False, ground_plane_z=None,
            decompose_larger_than=None, decomp_limits=None):
    """Physics scene, static colliders, and a rigid body per loose piece.

    `velocity_map` is `{prim_path: (vx, vy, vz)}` in m/s, a PER-BODY initial
    velocity added on top of `kick`/`bias`. The earthquake path needs it: a
    masonry wall failing out of plane rotates about its base and its top
    travels OUTWARD, so fragments high on the wall start with a lateral
    velocity that grows with height and the pile lands as a fan on the
    street rather than as a heap at the foot of the wall. A single `bias`
    cannot express that (it is one vector for the whole scene) and `kick`
    is zero-mean by design. `density` is kg/m3 for every body (420 = timber,
    the historical default; concrete/masonry pass ~2000).

    `bias` IS THE WIND, and it is what separates a collapse from a tornado.
    `kick` is deliberately zero-mean — it exists only to break the perfect
    interlock of a Voronoi partition so gravity can take hold — so a scene
    settled with it alone drops every piece into its own footprint. That is
    correct for a fire and wrong for a wind event, where the whole read is
    that material ended up somewhere DOWNTRACK of where it started.

    `bias=(bx, by, bz)` in m/s is added to every body's initial velocity, so
    the pile is thrown as it falls rather than after. Two ceilings will
    silently eat it if they are left alone, and both are raised by passing
    them explicitly:

      * `max_speed` -> `maxLinearVelocity`, 4.0 m/s by default. A 12 m/s
        throw authored against that cap comes out as a 4 m/s one and the
        debris lands a third of the way out, which looks like the bias "not
        working" rather than like a clamp.
      * `damping` -> `linearDamping`, 0.55 by default, which bleeds better
        than half the speed away every second. Fine for a piece that is only
        meant to topple; ruinous for one meant to travel.

    Leave all three None and this behaves exactly as it did before.

    ANTI-TUNNELLING, all three off by default so existing callers are
    untouched:

      * `ground_plane_z=0.0` authors a real infinite PhysX plane at that
        height — `UsdGeomPlane` + `CollisionAPI`, purpose `guide`, bound to
        the rubble material — and disables the legacy backstop below so there
        is exactly one floor. A half-space has no thickness to pass through:
        a body that ends a step below it is in penetration, not through it,
        and is pushed back out. A triangle-mesh ground quad has no such
        property, which is how 21-42% of a thrown archetype's meshes ended up
        under the world.
      * `ccd=True` sets `physxScene:enableCCD` plus `physxRigidBody:enableCCD`
        and `enableSpeculativeCCD` on every body, so fast pieces sweep against
        each other instead of teleporting through. Speculative CCD is the
        cheap one and is always safe; both are set because the failure is
        silent. (CCD is disabled by PhysX if `suppressReadback` is on — this
        path never sets it, because the whole bake reads results back through
        USD.)
      * `decompose_larger_than=<metres>` sends any loose piece whose world
        bounding box diagonal reaches that size to `convexDecomposition`
        instead of a hull, with `DECOMP_LIMITS` capping the cook. The hull of
        an L-shaped wall section or a folded roof panel encloses its own
        concavity, and a piece resting on that invisible volume is a floater
        with no geometry under it — the exact defect, at the exact scale, of
        "the bigger house debris parts". Off by default because it costs
        cooking time; use it on the big pieces only."""
    import random as _random

    from pxr import Gf, Sdf, UsdPhysics, UsdShade
    from pxr import PhysxSchema

    rng = rng or _random.Random(0)
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(abs(float(gravity)))

    # GPU DYNAMICS. The settle is the one phase here that can actually use the
    # card — mesh slicing and texture compositing are CPU libraries end to end
    # — and it is carrying thousands of bodies, which is where a GPU broadphase
    # and solver earn their keep.
    #
    # THE CAPACITIES ARE THE WHOLE STORY. PhysX preallocates fixed GPU buffers
    # and does not grow them: overflow does not fall back to CPU, it DROPS
    # contacts, and the symptom is pieces sinking through each other or through
    # the ground with only a warning in the log. The defaults are sized for a
    # few hundred bodies. Everything below is raised well past what this scene
    # needs, because the failure is silent and the memory is cheap next to a
    # 16 GB card already holding the scene.
    sx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    sx.CreateEnableGPUDynamicsAttr(bool(gpu))
    sx.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    # TGS converges far better than PGS on deep stacks, which is exactly what
    # a collapsed house is.
    sx.CreateSolverTypeAttr("TGS")
    if gpu:
        sx.CreateGpuMaxRigidContactCountAttr(4 * 1024 * 1024)
        sx.CreateGpuMaxRigidPatchCountAttr(1024 * 1024)
        sx.CreateGpuFoundLostPairsCapacityAttr(2 * 1024 * 1024)
        sx.CreateGpuFoundLostAggregatePairsCapacityAttr(64 * 1024)
        sx.CreateGpuTotalAggregatePairsCapacityAttr(64 * 1024)
        sx.CreateGpuHeapCapacityAttr(256 * 1024 * 1024)
        sx.CreateGpuTempBufferCapacityAttr(64 * 1024 * 1024)
        sx.CreateGpuMaxNumPartitionsAttr(8)
    # CONTINUOUS COLLISION DETECTION, opt-in. Discrete collision only asks
    # "do these two overlap at the end of the step"; at 20 m/s a 0.1 m plank
    # travels three of its own thicknesses per step and the answer is no,
    # every step, all the way through the floor. CCD sweeps the motion.
    if ccd:
        try:
            sx.CreateEnableCCDAttr(True)
        except Exception as exc:                   # pragma: no cover
            carb.log_warn("[settle] scene CCD unavailable ({0})".format(exc))

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
    plane.CreateCollisionEnabledAttr(ground_plane_z is None)
    floor.CreateAttribute("physics:approximation",
                          Sdf.ValueTypeNames.Token).Set("none")
    legacy_plane = UsdGeom.Plane.Define(stage, scene_path + "/floor/plane")
    legacy_plane.CreateAxisAttr("Z")
    lp = UsdPhysics.CollisionAPI.Apply(
        stage.GetPrimAtPath(scene_path + "/floor/plane"))

    # THE BACKSTOP ABOVE IS AUTHORED NESTED, and that is the reason for this
    # second one. `CollisionAPI` sits on the `floor` Xform AND on the
    # `UsdGeomPlane` beneath it, which is a nested collider — omni.physx has
    # an `ancestorHasAPI` test for precisely that shape, an Xform is not
    # valid collision geometry on its own, and whether the descendant plane
    # survives the parse is not something this module can assert from the
    # host. The measured symptom is a bake in which 21-42% of every wrecked
    # archetype's meshes finished BELOW the world, down to -2.9 m, which is
    # what a scene with no working floor and a 900 m triangle-mesh quad looks
    # like at 20 m/s.
    #
    # So when a caller names a grade, author the floor the way omni.physx's
    # own `physicsUtils.add_ground_plane` does — a plain Xform, a
    # `UsdGeomPlane` child with purpose `guide` carrying the ONLY
    # `CollisionAPI` in its chain, outside the `UsdPhysicsScene` prim — bind
    # it to the rubble material so pieces land dead, and switch the legacy
    # pair off so there is exactly one floor and no duplicate contact.
    plane_path = ""
    if ground_plane_z is not None:
        lp.CreateCollisionEnabledAttr(False)
        try:
            root = Sdf.Path(scene_path).GetParentPath()
            gp = root.AppendChild("settleGroundPlane")
            gx = UsdGeom.Xform.Define(stage, gp)
            gx.ClearXformOpOrder()
            gx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(ground_plane_z)))
            cp = UsdGeom.Plane.Define(stage, gp.AppendChild("CollisionPlane"))
            cp.CreateAxisAttr().Set("Z")
            cp.CreatePurposeAttr().Set("guide")
            cprim = cp.GetPrim()
            UsdPhysics.CollisionAPI.Apply(cprim).CreateCollisionEnabledAttr(True)
            UsdShade.MaterialBindingAPI.Apply(cprim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
            plane_path = str(cprim.GetPath())
        except Exception as exc:                   # pragma: no cover
            lp.CreateCollisionEnabledAttr(True)
            plane.CreateCollisionEnabledAttr(True)
            carb.log_warn("[settle] could not author a ground plane at z={0} "
                          "({1}); falling back to the legacy backstop"
                          .format(ground_plane_z, exc))

    # STATIC GEOMETRY GETS A TRIANGLE MESH, NOT A HULL. The ground is a flat
    # quad and the convex hull of a planar polygon is degenerate — PhysX has
    # no volume to cook, so the collider silently does nothing and every
    # fragment falls straight through the world. Static colliders may be
    # concave in PhysX, so "none" (use the real triangles) is both valid and
    # more accurate here; only the dynamic bodies need hulls.
    n_static = n_body = 0
    for path in static_paths:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            n_static += _apply_collider(prim, approximation="none")

    # BIG PIECES ONLY, and only when asked. One BBoxCache pass over the loose
    # set, so the size test costs nothing per body.
    sizes = {}
    if decompose_larger_than:
        from pxr import Usd as _Usd
        _bc = UsdGeom.BBoxCache(_Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        for path in loose_paths:
            pr = stage.GetPrimAtPath(path)
            if not pr or not pr.IsValid():
                continue
            try:
                rng_ = _bc.ComputeWorldBound(pr).ComputeAlignedRange()
                sizes[path] = 0.0 if rng_.IsEmpty() else float(
                    rng_.GetSize().GetLength())
            except Exception:
                sizes[path] = 0.0

    bodies = []
    no_collider = []
    no_local_frame = []
    n_decomp = 0
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
        if decompose_larger_than and sizes.get(path, 0.0) >= float(
                decompose_larger_than):
            approx = "convexDecomposition"
        if approx_map and path in approx_map:
            approx = approx_map[path]
        if approx == "convexDecomposition":
            n_decomp += 1
        if not _apply_collider(prim, approximation=approx,
                               decomp_limits=decomp_limits):
            # NOT A HARMLESS SKIP. A loose prim that never becomes a body is
            # not "left where it is" in any useful sense — it is left where
            # the DAMAGE stage authored it, which for a wrecked house is
            # part-way up a wall that no longer exists. Every one of these is
            # a floater by construction, so they are counted and reported
            # rather than passed over in silence.
            no_collider.append(path)
            continue
        # WORLD-BAKED GEOMETRY PICKED UP FOR PHYSICS. `RigidBodyAPI` treats
        # the PRIM's OWN transform as the body's origin, and the mesh's
        # local points — unmodified — as the shape's offset from it. A loose
        # path that IS the mesh (not a wrapper Xform around a referenced
        # prop, which always carries its own translate/rotate/scale) and
        # has NO xform ops at all has that origin nailed to its parent's
        # frame, while a helper that bakes absolute WORLD coordinates
        # straight into `points` puts the shape wherever those points
        # happen to be — a moment arm that can run to a hundred-plus metres
        # on a scene where the content is nowhere near the stage origin.
        # That is not survivable: a capped LINEAR speed on the origin does
        # nothing to bound how far the SHAPE moves once the body picks up
        # any angular velocity (`omega * arm` per step), and it is the
        # proven mechanism behind a measured 205 m "worst mover" that a real
        # ground plane and CCD both failed to hold to grade — both reason
        # about the body's own POSE, and here the pose was nowhere near the
        # geometry it was supposed to represent (`quake_flow._cyl`'s vent
        # stacks, before 2026-08-29 — see its docstring for the full
        # mechanism). Flagged loudly here rather than silently simulated, so
        # the next instance of this mistake — in this pipeline or any other
        # `settle.run` caller — shows up in the report instead of costing
        # another multi-round investigation.
        if prim.IsA(UsdGeom.Mesh) and not UsdGeom.Xformable(prim).GetOrderedXformOps():
            _pv = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if _pv:
                _cx = sum(float(q[0]) for q in _pv) / len(_pv)
                _cy = sum(float(q[1]) for q in _pv) / len(_pv)
                _cz = sum(float(q[2]) for q in _pv) / len(_pv)
                if (_cx * _cx + _cy * _cy + _cz * _cz) ** 0.5 > 3.0:
                    no_local_frame.append(path)
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateRigidBodyEnabledAttr(True)
        # DAMPED AND STICKY, so this reads as a collapse and not a detonation.
        # Undamped rigid bodies with default restitution behave like billiard
        # balls: every contact converts a drop into lateral speed and the pile
        # sprays outward. Real rubble is lossy — it lands, grinds, and stops.
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateLinearDampingAttr(0.55 if damping is None else float(damping))
        px.CreateAngularDampingAttr(2.2)
        px.CreateMaxLinearVelocityAttr(
            4.0 if max_speed is None else float(max_speed))
        px.CreateMaxAngularVelocityAttr(5.0)
        # THE EXPLOSION KNOB. When PhysX finds two bodies overlapping it pushes
        # them apart, and by default it may do so at any speed — which turns
        # fracture fragments that start in contact into shrapnel. Capping the
        # separation speed makes penetration resolve as a shove instead.
        px.CreateMaxDepenetrationVelocityAttr(0.6)
        px.CreateSolverPositionIterationCountAttr(16)
        px.CreateSolverVelocityIterationCountAttr(2)
        px.CreateSleepThresholdAttr(0.02)
        if ccd:
            try:
                px.CreateEnableCCDAttr(True)
                # The cheap one, and the one that is always available: it
                # inflates the contact prediction by the distance the body
                # will travel this step instead of sweeping it. Both are set
                # because a silently-ignored flag is how this failed before.
                px.CreateEnableSpeculativeCCDAttr(True)
            except Exception as exc:               # pragma: no cover
                carb.log_warn("[settle] body CCD unavailable ({0})".format(exc))
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
        # `bias` rides on top of it: same random nudge, offset by a constant
        # wind vector, so pieces still separate from each other AND all go the
        # same way. Spin is left zero-mean whatever the bias — a board thrown
        # downwind tumbles, it does not roll about one axis.
        bx, by, bz = (0.0, 0.0, 0.0) if bias is None else (
            float(bias[0]), float(bias[1]), float(bias[2]))
        vm = (velocity_map or {}).get(path)
        if vm is not None:
            bx, by, bz = bx + float(vm[0]), by + float(vm[1]), bz + float(vm[2])
        if kick > 0.0 or bias is not None or vm is not None:
            body.CreateVelocityAttr(Gf.Vec3f(
                bx + float(rng.uniform(-kick, kick)),
                by + float(rng.uniform(-kick, kick)),
                bz))
            spin = max(kick, 1.4 if bias is not None else 0.0)
            body.CreateAngularVelocityAttr(Gf.Vec3f(
                float(rng.uniform(-spin, spin)),
                float(rng.uniform(-spin, spin)),
                float(rng.uniform(-spin, spin))))
        UsdPhysics.MassAPI.Apply(prim).CreateDensityAttr(float(density))
        bodies.append(prim)
        n_body += 1

    return {"bodies": bodies, "static_meshes": n_static, "rigid": n_body,
            "no_collider": no_collider, "no_local_frame": no_local_frame,
            "decomposed": n_decomp,
            "ground_plane": plane_path, "ccd": bool(ccd)}


def bake(stage, bodies):
    """Freeze each body where it came to rest, then switch physics off.

    The transform is read as local-to-world and re-authored as translate /
    orient / scale. Rebuilding the op order rather than editing the existing
    ops matters: PhysX may have authored an `orient` alongside the `rotateXYZ`
    that `apply_placements` wrote, and leaving both composes them twice.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    xf = UsdGeom.XformCache()
    n = 0
    for prim in bodies:
        if not prim or not prim.IsValid():
            continue
        m = xf.GetLocalToWorldTransform(prim)
        tr = Gf.Transform(m)
        t = tr.GetTranslation()
        q = tr.GetRotation().GetQuat()
        sc = tr.GetScale()

        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
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


# ---------------------------------------------------------------------------
# Post-settle ledge cull (earthquake path; off unless asked for)
# ---------------------------------------------------------------------------
# A fragment that comes to rest on a window sill, a cornice or a string
# course halfway up an otherwise standing façade reads as a flake stuck to
# the wall, not as debris — a 0.3 m chunk does not balance on a 0.1 m ledge
# in the reconnaissance photographs, it bounces off and lands in the street.
# Callers register the footprint of each STILL-STANDING building and the
# settle deletes anything that finished in the narrow band just outside the
# wall line, above head height. Inside the footprint is left alone (that is
# debris resting on a floor slab, which is right), and so is anything
# further out than the band (that is the rubble heap).
LEDGE_ZONES = []            # [(cx, cy, W, D, yaw_deg, z0)]
LEDGE_Z_MIN = 3.0           # m above the building's base: above any windrow
#                             (research: shed-wall windrows are 1-2 m deep)
LEDGE_BAND_M = 1.5          # how far outside the wall line still counts as
#                             "on a ledge" rather than "on the pile"


def register_ledge_zone(cx, cy, W, D, yaw_deg=0.0, z0=0.0):
    """Declare one still-standing building's footprint for the cull."""
    LEDGE_ZONES.append((float(cx), float(cy), float(W), float(D),
                        float(yaw_deg), float(z0)))


def clear_ledge_zones():
    del LEDGE_ZONES[:]


def _cull_ledges(stage, bodies, zones=None, z_min=None, band_m=None):
    """Deactivate every body resting on a façade ledge. Returns the count."""
    import math

    from pxr import UsdGeom

    zones = LEDGE_ZONES if zones is None else zones
    if not zones:
        return 0
    z_min = LEDGE_Z_MIN if z_min is None else float(z_min)
    band = LEDGE_BAND_M if band_m is None else float(band_m)
    xf = UsdGeom.XformCache()
    n = 0
    for prim in bodies:
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        p = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
        for cx, cy, W, D, yaw, z0 in zones:
            if p[2] - z0 < z_min:
                continue
            a = math.radians(-yaw)
            dx, dy = p[0] - cx, p[1] - cy
            lx = dx * math.cos(a) - dy * math.sin(a)
            ly = dx * math.sin(a) + dy * math.cos(a)
            ex = abs(lx) - W / 2.0
            ey = abs(ly) - D / 2.0
            if ex <= 0.0 and ey <= 0.0:
                break                      # inside: on a floor slab, keep
            out = math.hypot(max(ex, 0.0), max(ey, 0.0))
            if out <= band:
                prim.SetActive(False)
                n += 1
                break
    return n


# ---------------------------------------------------------------------------
# Convergence, the quiet phase, and grade
# ---------------------------------------------------------------------------


def _settle_phase(bodies, chunk, cap, tol=0.001, rest_tol=0.004,
                  stall_chunks=0):
    """Step in chunks until the pile stops moving or `cap` steps are gone.

    Returns `(steps_used, moving, driver, at_rest, reason)`, where `moving`
    is how many bodies travelled more than `rest_tol` during the LAST chunk,
    `at_rest` says the loop stopped because the pile stopped rather than
    because it ran out of budget, and `reason` is one of:

      * `"rest"`    — the busiest body moved less than `tol` in a chunk.
      * `"stalled"` — `stall_chunks` chunks passed with the moving-body count
        not falling. This is NOT "ran out of steps": the count plateaued
        before the cap, which means more steps would not have helped — some
        body is not slowing down (a tunnelled floor, a persistent overlap
        feeding it fresh velocity every step), and the fix is to find that
        body, not to raise `steps`/`max_steps`.
      * `"cap"`     — the loop exhausted `cap` with neither of the above.

    `stall_chunks=0` keeps the historical behaviour exactly: chunked
    stepping with an early exit as soon as the busiest body moves less than
    a millimetre, `reason` only ever `"rest"` or `"cap"`. A non-zero value
    adds the other half of a convergence test."""
    import numpy as np

    used, driver, moving = 0, None, 0
    at_rest = False
    reason = "cap"
    best, stalled = None, 0
    prev = _positions(bodies)
    while used < cap:
        n = min(chunk, cap - used)
        driver = _step(n)
        used += n
        now = _positions(bodies)
        keys = [k for k in prev if k in now]
        if keys:
            d = np.linalg.norm(
                np.array([list(now[k]) for k in keys])
                - np.array([list(prev[k]) for k in keys]), axis=1)
            moved, moving = float(d.max()), int((d > rest_tol).sum())
        else:
            moved, moving = 0.0, 0
        prev = now
        if moved < tol:
            at_rest = True
            reason = "rest"
            break
        if stall_chunks:
            if best is None or moving < best:
                best, stalled = moving, 0
            else:
                stalled += 1
                if stalled >= int(stall_chunks):
                    reason = "stalled"
                    break
    return used, moving, driver, at_rest, reason


def _quiet_bodies(bodies, damping=(1.6, 5.0), max_speed=6.0):
    """Raise the damping and drop the speed cap for the settling-out phase.

    NO NEW IMPULSE, which is the whole point: the throw has already happened,
    so the downwind lean the `bias` bought is in the transforms and stays
    there. What this removes is everything AFTER the throw — the piece still
    sailing when the budget expires, and the piece creeping a millimetre a
    second down a pile forever. Terminal velocity under 1.6/s damping is
    still ~6 m/s, so an airborne fragment lands in well under a second; it
    just does not travel while it does so.
    """
    from pxr import PhysxSchema

    lin, ang = float(damping[0]), float(damping[1])
    n = 0
    for prim in bodies:
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        try:
            px = PhysxSchema.PhysxRigidBodyAPI(prim)
            if not px:
                px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            px.CreateLinearDampingAttr(lin)
            px.CreateAngularDampingAttr(ang)
            if max_speed:
                px.CreateMaxLinearVelocityAttr(float(max_speed))
            n += 1
        except Exception:                          # pragma: no cover
            pass
    return n


def _bbox_cache():
    from pxr import Usd, UsdGeom
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def _z_min(bc, prim):
    """World-space min-z of a prim's geometry, or None if it has no extent."""
    try:
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:                              # pragma: no cover
        return None
    if r.IsEmpty():
        return None
    return float(r.GetMin()[2])


def _below_grade(stage, bodies, floor_z, tol=0.02):
    """(count, worst_z, [(path, z_min), ...]) for bodies that finished under
    the floor. Geometry, not origins: a fragment whose pivot is above grade
    with half its mesh under it is still a hole in the ground."""
    bc = _bbox_cache()
    worst, out = 0.0, []
    for prim in bodies:
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        z0 = _z_min(bc, prim)
        if z0 is None or z0 >= floor_z - tol:
            continue
        out.append((str(prim.GetPath()), z0))
        worst = min(worst, z0 - floor_z)
    out.sort(key=lambda q: q[1])
    return len(out), worst, out[:5]


def _lift_below_grade(stage, bodies, floor_z, tol=0.02, live=False):
    """Raise anything under the floor until its lowest point is ON the floor.

    THE TIE BREAKS TOWARD THE GROUND, the same call `bake._reseat_roots`
    records: a fragment lying on the ground is a pose the solver could
    plausibly have reached, and one buried three metres under the lawn — or
    hanging in the air — is not.

    `live=True` is the mid-solve rescue: it also zeroes the body's velocity
    so the quiet phase that follows re-settles it onto the pile properly
    instead of shooting it back down. `live=False` is the post-bake clamp,
    which is pure geometry on transforms `bake()` has already frozen and is
    therefore guaranteed to land, whatever PhysX did or did not do with the
    teleport. Returns (lifted, failed).
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    bc = _bbox_cache()
    xf = UsdGeom.XformCache()
    lifted = failed = 0
    for prim in bodies:
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        z0 = _z_min(bc, prim)
        if z0 is None or z0 >= floor_z - tol:
            continue
        dz = floor_z - z0
        x = UsdGeom.Xformable(prim)
        ops = [o for o in x.GetOrderedXformOps()
               if o.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        if not ops:
            failed += 1
            continue
        # PARENT SPACE, NOT WORLD. The translate op is authored in the
        # prim's parent frame, and a Scope is not always the identity.
        try:
            inv = xf.GetParentToWorldTransform(prim).GetInverse()
            d = inv.TransformDir(Gf.Vec3d(0.0, 0.0, dz))
        except Exception:                          # pragma: no cover
            d = Gf.Vec3d(0.0, 0.0, dz)
        op = ops[0]
        v = op.Get()
        if v is None:
            v = Gf.Vec3d(0.0, 0.0, 0.0)
        try:
            op.Set(type(v)(v[0] + d[0], v[1] + d[1], v[2] + d[2]))
        except Exception:                          # pragma: no cover
            failed += 1
            continue
        if live:
            try:
                body = UsdPhysics.RigidBodyAPI(prim)
                body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            except Exception:                      # pragma: no cover
                pass
        lifted += 1
    return lifted, failed


def run(stage, loose_paths, static_paths, steps=360, settle_note=True,
        gravity=-9.81, kick=0.0, rng=None, bake_result=True,
        dynamic_approximation="convexHull", approx_map=None, gpu=True,
        bias=None, max_speed=None, damping=None, velocity_map=None,
        density=420.0, cull_ledges=None, ccd=False, ground_plane_z=None,
        decompose_larger_than=None, decomp_limits=None, converge=False,
        max_steps=None, quiet_steps=0, quiet_damping=(1.6, 5.0),
        quiet_max_speed=6.0, rest_tol=0.004, stall_chunks=3, floor_z=None,
        floor_tol=0.02, strict=None):
    """prepare -> step physics -> measure -> bake. Returns a short report.

    MEASURES WHAT MOVED. A settle that silently does nothing looks identical
    to one that ran and found everything already at rest, and the two need
    completely different fixes — so the mean and max displacement are reported
    rather than assumed.

    `bake_result=False` leaves the bodies live, which is what you want when
    hand-testing in the viewport: drag a piece into the air, press play, and
    it should fall. With baking on (the default for capture) every body is
    disabled at the end and pressing play correctly does nothing.

    CONVERGENCE, off by default so existing callers step exactly as before:

      * `converge=True` makes `steps` a target rather than a ceiling. The
        throw phase keeps going up to `max_steps` (default 3x `steps`) until
        the busiest body moves less than a millimetre in a chunk, or until
        the number of bodies still moving stops falling for `stall_chunks`
        chunks — at which point more steps are demonstrably not the fix and
        the report says so.
      * `quiet_steps=N` runs a second phase after the throw with the damping
        raised and the speed cap lowered (`quiet_damping`, `quiet_max_speed`).
        The bias is an INITIAL velocity, so nothing about this touches the
        downwind lean already in the transforms; what it removes is the
        piece still airborne at the end of the budget and the piece creeping
        down a pile forever. This is the single cheapest way to drive
        `still_moving` to zero.
      * `floor_z=<grade>` audits the result against the ground and repairs
        it: bodies whose geometry finished under the floor are lifted back
        onto it before the quiet phase (velocity zeroed, so they re-settle
        properly), and anything still under it after the bake is clamped
        geometrically. That is the belt to `ground_plane_z`/`ccd`'s braces —
        with a real half-space and CCD nothing should get under the world in
        the first place.
      * `strict=True` (or `SETTLE_STRICT=1`) turns a bad settle into a raised
        `SettleNotConverged` instead of a warning, for callers that would
        rather lose the run than export debris frozen in mid-air. The report
        is printed either way, and loudly.
    """
    import numpy as np

    info = prepare(stage, loose_paths, static_paths, gravity=gravity,
                   kick=kick, rng=rng,
                   dynamic_approximation=dynamic_approximation,
                   approx_map=approx_map, gpu=gpu, bias=bias,
                   max_speed=max_speed, damping=damping,
                   velocity_map=velocity_map, density=density, ccd=ccd,
                   ground_plane_z=ground_plane_z,
                   decompose_larger_than=decompose_larger_than,
                   decomp_limits=decomp_limits)
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
    # The loop advances in chunks and stops early once the busiest body in a
    # chunk moves less than a millimetre. With `converge=True` it may also
    # run PAST `steps` — up to `max_steps` — because the measured failure was
    # the opposite of an early exit: 1200 of 1200 steps consumed with bodies
    # still in flight, baked as they were.
    chunk = max(30, int(steps) // 12)
    cap = int(steps)
    if converge:
        cap = max(cap, int(max_steps) if max_steps is not None
                  else 3 * int(steps))
    used, moving, driver, at_rest, reason = _settle_phase(
        info["bodies"], chunk, cap, tol=0.001, rest_tol=rest_tol,
        stall_chunks=stall_chunks if converge else 0)
    info["driver"] = driver
    info["steps_used"] = used
    info["steps_cap"] = cap
    info["converged"] = bool(at_rest)
    info["stop_reason"] = reason
    after = _positions(info["bodies"])

    # THE QUIET PHASE. Everything up to here was the throw; this is the pile
    # coming to rest. Rescue first — a piece that tunnelled the floor before
    # the plane existed, or that a bad collider let through, is lifted back
    # to grade with its velocity zeroed so the quiet steps settle it onto the
    # pile instead of into it.
    info["rescued"] = info["rescue_failed"] = 0
    if floor_z is not None and int(quiet_steps or 0) > 0:
        info["rescued"], info["rescue_failed"] = _lift_below_grade(
            stage, info["bodies"], float(floor_z), tol=floor_tol, live=True)
    info["quiet_used"] = 0
    if int(quiet_steps or 0) > 0:
        _quiet_bodies(info["bodies"], quiet_damping, quiet_max_speed)
        q_chunk = max(20, int(quiet_steps) // 8)
        info["quiet_used"], moving, qdriver, q_rest, q_reason = _settle_phase(
            info["bodies"], q_chunk, int(quiet_steps), tol=0.001,
            rest_tol=rest_tol, stall_chunks=stall_chunks)
        info["driver"] = qdriver or info["driver"]
        # The quiet phase is the one that has to reach rest — it is the
        # phase whose whole job is to stop things.
        info["converged"] = bool(q_rest)
        info["stop_reason"] = q_reason
        after = _positions(info["bodies"])

    # And one last look: how many bodies were STILL MOVING when the budget ran
    # out. Zero means the pile is genuinely at rest and baking is safe; a
    # non-zero count is the scene telling you it was baked mid-flight.
    # NAME THEM, not just count them — "1 body(s) STILL MOVING" with no path
    # sends the next debugging pass back to a live rerun to find out which
    # one; the paths are sitting right here in `settled`/`after`.
    _step(20)
    settled = _positions(info["bodies"])
    still_moving_keys = [
        k for k in after
        if k in settled
        and float(np.linalg.norm(np.array(settled[k]) - np.array(after[k])))
        > rest_tol]
    info["still_moving"] = len(still_moving_keys)
    info["still_moving_examples"] = still_moving_keys[:5]
    after = settled

    # HORIZONTAL vs VERTICAL is the whole question. A collapse drops pieces:
    # large -Z, small XY. An explosion throws them: large XY. One number for
    # total displacement cannot tell the two apart, and that is exactly the
    # judgement being made here.
    moved_keys = [k for k in before if k in after]
    dv = [float(np.array(after[k])[2] - np.array(before[k])[2])
          for k in moved_keys]
    dh = [float(np.linalg.norm((np.array(after[k]) - np.array(before[k]))[:2]))
          for k in moved_keys]
    d = [float(np.linalg.norm(np.array(after[k]) - np.array(before[k])))
         for k in moved_keys]
    info["moved_mean"] = float(np.mean(d)) if d else 0.0
    info["moved_max"] = float(np.max(d)) if d else 0.0
    info["drop_mean"] = float(np.mean(dv)) if dv else 0.0
    info["spread_mean"] = float(np.mean(dh)) if dh else 0.0
    info["spread_max"] = float(np.max(dh)) if dh else 0.0
    # WHICH ONE. A max without a path is a number to argue about; a path is
    # one prim to go look at — "ask for one prim path" is what actually
    # ended each round of the floating-debris investigation, not another
    # sweep. `spread_max_path` is the body driving `spread_max` (the
    # "large = exploding" reading); `moved_max_path` is the same for total
    # displacement, which can differ when the worst mover fell straight down
    # rather than sideways.
    info["spread_max_path"] = moved_keys[int(np.argmax(dh))] if dh else ""
    info["moved_max_path"] = moved_keys[int(np.argmax(d))] if d else ""

    # BELOW GRADE IS A SEPARATE FAILURE FROM STILL MOVING, and it is the one
    # a step budget cannot fix: material under the world was let through the
    # floor, not caught in flight.
    info["below_grade"] = 0
    info["below_grade_worst"] = 0.0
    info["below_grade_examples"] = []
    if floor_z is not None:
        (info["below_grade"], info["below_grade_worst"],
         info["below_grade_examples"]) = _below_grade(
            stage, info["bodies"], float(floor_z), tol=floor_tol)

    info["solve_s"] = _time.time() - _t0
    # THE LEDGE CULL, off by default. `cull_ledges=True` (or the env var, so
    # the bench and the bake can turn it on without a code change) deletes
    # the handful of fragments that finished balanced on a sill or a cornice
    # of a building that is still standing — see LEDGE_ZONES above. A list
    # of (cx, cy, W, D, yaw, z0) tuples may be passed instead of True to
    # override the registry.
    if cull_ledges is None:
        import os as _os
        cull_ledges = _os.environ.get("SETTLE_CULL_LEDGES", "").strip() \
            not in ("", "0", "false", "False")
    if cull_ledges:
        zones = None if cull_ledges is True else cull_ledges
        info["culled_ledges"] = _cull_ledges(stage, info["bodies"], zones)
        info["bodies"] = [b for b in info["bodies"] if b and b.IsValid()
                          and b.IsActive()]
    info["baked"] = bake(stage, info["bodies"]) if bake_result else 0

    # THE LAST CLAMP, and the only one that cannot fail quietly: the bodies
    # are frozen, the transforms are plain translate/orient/scale, and
    # raising one is arithmetic. Anything still under the floor here got
    # there despite a half-space and CCD, and shipping it is not an option —
    # a hole in the lawn reads worse than a plank lying flat in it.
    info["clamped"] = info["clamp_failed"] = 0
    if floor_z is not None and bake_result and info["below_grade"]:
        info["clamped"], info["clamp_failed"] = _lift_below_grade(
            stage, info["bodies"], float(floor_z), tol=floor_tol, live=False)

    if settle_note:
        print("[settle] {0} rigid, {1} static, driver={2}, baked {3}".format(
            info["rigid"], info["static_meshes"], info["driver"],
            info["baked"]))
        print("[settle]   {0:.1f}s solving ({1}){2}{3}".format(
            info.get("solve_s", 0.0), "GPU" if gpu else "CPU",
            ", CCD" if info.get("ccd") else "",
            ", ground plane " + info["ground_plane"]
            if info.get("ground_plane") else ""))
        if info.get("decomposed"):
            print("[settle]   {0} body(s) cooked as convex DECOMPOSITIONS "
                  "(the rest are hulls)".format(info["decomposed"]))
        if info.get("culled_ledges"):
            print("[settle]   {0} body(s) culled off façade ledges "
                  "({1} zone(s))".format(info["culled_ledges"], len(LEDGE_ZONES)))
        _stop_label = {"stalled": "  <-- STALLED (moving-body count stopped "
                                  "falling; more steps will not fix this)",
                      "cap": "  <-- CAP REACHED"}.get(
            info.get("stop_reason"), "")
        print("[settle]   {0} of {1} steps used{2}; {3} quiet step(s); "
              "{4} body(s) STILL MOVING at bake time".format(
                  info.get("steps_used", steps), info.get("steps_cap", steps),
                  "" if info.get("converged") else _stop_label,
                  info.get("quiet_used", 0), info.get("still_moving", 0)))
        if info.get("rescued") or info.get("clamped"):
            print("[settle]   {0} body(s) lifted back to grade before the "
                  "quiet phase, {1} clamped after the bake".format(
                      info.get("rescued", 0), info.get("clamped", 0)))
        print("[settle]   drop  mean {0:+.2f} m   (down = collapsing)".format(
            info["drop_mean"]))
        # THE READING INVERTS WHEN THERE IS A BIAS. Horizontal spread is the
        # explosion diagnostic for a collapse and the SUCCESS criterion for a
        # wind event — a tornado settle that reports 0.6 m of spread is one
        # that threw nothing. Label it by which run this was, or the next
        # person tunes the bias down until the "warning" goes away.
        print("[settle]   spread mean {0:.2f} m / max {1:.2f} m   ({2})".format(
            info["spread_mean"], info["spread_max"],
            "large = thrown downwind, which is the point" if bias is not None
            else "large = exploding"))
        if info.get("spread_max_path") and info["spread_max"] > 0.5:
            print("[settle]     worst mover: {0}  ({1:.2f} m horizontal)"
                  .format(info["spread_max_path"], info["spread_max"]))
        if info["moved_max"] < 0.01:
            print("[settle] NOTHING MOVED — the solver did not run, or every "
                  "body was already resting and interlocked")

    # ---- the verdict, and it is not a footnote ---------------------------
    faults = []
    if info.get("still_moving"):
        faults.append(
            "{0} body(s) STILL MOVING at bake time — they are frozen "
            "mid-flight in the export, e.g. {1}".format(
                info["still_moving"],
                (info.get("still_moving_examples") or ["?"])[0]))
    if info.get("no_collider"):
        faults.append(
            "{0} loose prim(s) NEVER SIMULATED (no cookable mesh under them); "
            "they are still wherever the damage stage authored them, e.g. {1}"
            .format(len(info["no_collider"]), info["no_collider"][0]))
    if info.get("no_local_frame"):
        faults.append(
            "{0} loose prim(s) simulated with NO LOCAL XFORM — the "
            "RigidBody's origin is nowhere near its own geometry (points "
            "baked in world space, no xform ops), which is how a small "
            "piece becomes a 200+ m 'worst mover' or slips a ground plane "
            "CCD is supposed to hold it to; e.g. {1}".format(
                len(info["no_local_frame"]), info["no_local_frame"][0]))
    if info.get("below_grade"):
        faults.append(
            "{0} body(s) finished BELOW GRADE (worst {1:.2f} m under the "
            "floor) — the ground collider is not holding{2}".format(
                info["below_grade"], info["below_grade_worst"],
                "" if not info.get("clamped")
                else "; {0} were clamped back onto it".format(
                    info["clamped"])))
    if info.get("rescue_failed") or info.get("clamp_failed"):
        faults.append(
            "{0} body(s) could NOT be lifted (no translate op to edit)".format(
                info.get("rescue_failed", 0) + info.get("clamp_failed", 0)))
    stalled = info.get("stop_reason") == "stalled"
    if not info.get("converged"):
        if stalled:
            faults.append(
                "the moving-body count STALLED (stopped falling for "
                "{0} chunks) before the step cap ({1}) — the pile is not "
                "slow, something in it is not settling, e.g. {2}".format(
                    stall_chunks, info.get("steps_cap", steps),
                    info.get("spread_max_path")
                    or (info.get("still_moving_examples") or ["?"])[0]))
        else:
            faults.append(
                "the step cap ({0}) was reached without the pile coming to "
                "rest".format(info.get("steps_cap", steps)))
    info["faults"] = faults
    if faults:
        head = ("SETTLE DID NOT CONVERGE" if not bake_result
                else "SETTLE BAKED A PILE THAT WAS NOT AT REST")
        msg = head + ": " + "; ".join(faults)
        print("[settle] " + "!" * 68)
        print("[settle] !! " + head)
        for f in faults:
            print("[settle] !!   - " + f)
        for path, z0 in (info.get("below_grade_examples") or [])[:3]:
            print("[settle] !!     {0}  z_min {1:+.2f}".format(path, z0))
        for path in (info.get("still_moving_examples") or [])[:3]:
            print("[settle] !!     {0}  STILL MOVING".format(path))
        for path in (info.get("no_local_frame") or [])[:3]:
            print("[settle] !!     {0}  NO LOCAL XFORM (world-baked points, "
                  "body origin far from its own geometry)".format(path))
        # STALLED IS NOT A BUDGET PROBLEM. The old, single "!! FIX: raise
        # steps/max_steps" line was printed for a stall exactly the same as
        # for a genuine cap exhaustion, and it is the wrong advice for a
        # stall: `converge=True` already ran the throw out to 2.5-3x `steps`
        # (measured: 1995 of a 4000-step cap on `uf_bench_ref`,
        # 2026-08-29 — the moving-body count plateaued long before the cap,
        # which the old message reported as "CAP REACHED") and a stalled
        # body was never going to stop on its own. A stall means something
        # is feeding it fresh velocity every step (a tunnelled floor, a
        # persistent deep overlap): check `ground_plane_z`/`ccd`/`floor_z`
        # first, and go look at the named prim before touching the budget.
        if stalled:
            print("[settle] !! FIX: this STALLED, it did not run out of "
                  "budget — raising `steps`/`max_steps` will not help. "
                  "Check `ground_plane_z`, `ccd` and `floor_z` are set (a "
                  "thin quad ground tunnels at speed) and go look at the "
                  "prim(s) named above.")
        else:
            print("[settle] !! FIX: raise `steps`/`max_steps`, add "
                  "`quiet_steps`, and check `ground_plane_z`/`ccd` are set "
                  "for a thrown settle.")
        print("[settle] " + "!" * 68)
        carb.log_error("[settle] " + msg)
        if strict is None:
            import os as _os
            strict = _os.environ.get("SETTLE_STRICT", "").strip() \
                not in ("", "0", "false", "False")
        if strict:
            raise SettleNotConverged(msg)
    elif settle_note:
        print("[settle]   AT REST: nothing moving, nothing below grade.")
    return info
