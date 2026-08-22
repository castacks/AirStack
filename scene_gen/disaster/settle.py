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


def _apply_collider(prim, approximation="convexHull"):
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
        n += 1
    return n


def _iter(prim):
    from pxr import Usd
    return Usd.PrimRange(prim)


def prepare(stage, loose_paths, static_paths, gravity=-9.81,
            scene_path="/World/physicsScene", kick=0.0, rng=None):
    """Physics scene, static colliders, and a rigid body per loose piece."""
    import random as _random

    from pxr import Gf, Sdf, UsdPhysics, UsdShade
    from pxr import PhysxSchema

    rng = rng or _random.Random(0)
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(abs(float(gravity)))

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

    bodies = []
    for path in loose_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if not _apply_collider(prim):
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
        px.CreateMaxLinearVelocityAttr(4.0)
        px.CreateMaxAngularVelocityAttr(5.0)
        # THE EXPLOSION KNOB. When PhysX finds two bodies overlapping it pushes
        # them apart, and by default it may do so at any speed — which turns
        # fracture fragments that start in contact into shrapnel. Capping the
        # separation speed makes penetration resolve as a shove instead.
        px.CreateMaxDepenetrationVelocityAttr(0.6)
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
        if kick > 0.0:
            body.CreateVelocityAttr(Gf.Vec3f(
                float(rng.uniform(-kick, kick)),
                float(rng.uniform(-kick, kick)),
                0.0))
            body.CreateAngularVelocityAttr(Gf.Vec3f(
                float(rng.uniform(-kick, kick)),
                float(rng.uniform(-kick, kick)),
                float(rng.uniform(-kick, kick))))
        UsdPhysics.MassAPI.Apply(prim).CreateDensityAttr(420.0)   # timber
        bodies.append(prim)
        n_body += 1

    return {"bodies": bodies, "static_meshes": n_static, "rigid": n_body}


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


def run(stage, loose_paths, static_paths, steps=360, settle_note=True,
        gravity=-9.81, kick=0.0, rng=None, bake_result=True):
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
                   kick=kick, rng=rng)
    if not info["bodies"]:
        carb.log_warn("[settle] nothing to settle")
        return info

    before = _positions(info["bodies"])
    info["driver"] = _step(steps)
    after = _positions(info["bodies"])

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
    info["spread_mean"] = float(np.mean(dh)) if dh else 0.0
    info["spread_max"] = float(np.max(dh)) if dh else 0.0

    info["baked"] = bake(stage, info["bodies"]) if bake_result else 0
    if settle_note:
        print("[settle] {0} rigid, {1} static, driver={2}, baked {3}".format(
            info["rigid"], info["static_meshes"], info["driver"],
            info["baked"]))
        print("[settle]   drop  mean {0:+.2f} m   (down = collapsing)".format(
            info["drop_mean"]))
        print("[settle]   spread mean {0:.2f} m / max {1:.2f} m   "
              "(large = exploding)".format(info["spread_mean"],
                                           info["spread_max"]))
        if info["moved_max"] < 0.01:
            print("[settle] NOTHING MOVED — the solver did not run, or every "
                  "body was already resting and interlocked")
    return info
