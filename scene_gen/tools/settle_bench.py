"""settle_bench — what the settle actually spends its time on.

    docker exec isaac-sim bash -c "cd /isaac-sim && ./python.sh \\
        AirStack/scene_gen/tools/settle_bench.py <n_bodies> <mode> [frames]"

MODES

    bare        a plain `UsdPhysics.Scene` with no `PhysxSceneAPI` — PhysX's
                own defaults — stepped with `app.update()`. What
                `scene_prep.settle_rigid_props` did before this was measured.
    cpu | gpu   the same, with `disaster.settle.configure_scene` applied
                (TGS, MBP/GPU broadphase, and on GPU the raised capacities).
    sc_cpu      physics stepped directly (`SimulationContext`, `render=False`),
    sc_gpu      no Kit update loop and no renderer.
    fabric_cpu  as `sc_*`, but with PhysX's per-step USD write-back turned off
    fabric_gpu  and one write-back at the end — which is all a settle needs.

WHAT IT FOUND  (RTX 5090, 8,000 boxed bodies, 179 frames, sim phase only)

    bare         126.8 s        sc_cpu    95.2 s        fabric_cpu   2.6 s
    cpu          134.5 s        sc_gpu    94.0 s        fabric_gpu   2.6 s

On THIS workload the settle is USD-bound, not solver-bound: PhysX writes every
body's transform into USD on every step, and that is ~97% of the cost. GPU
dynamics changes nothing, because the solve was never the expensive part.

AND THAT CONCLUSION DOES NOT SURVIVE CONTACT WITH A REAL COLLAPSE.
The same instrumentation on 907 fragments of a fractured `BG_Building_F`
(`scene_prep.settle_rigid_props`, the identical stepping path):

    CPU scene, per-step write-back removed        98.4 s
    GPU dynamics + raised capacities               2.4 s

350x more expensive per body-step than the boxes, and 41x faster on the card.
The difference is the contacts. Clean boxes dropped into free space barely
touch the solver; Voronoi shell fragments spawn interpenetrating by metres,
and resolving that is the entire bill. So this benchmark measures the FLOOR of
a settle — the part that is pure bookkeeping — and a real scene's cost lives
in geometry it does not reproduce. Use it to compare stepping strategies, not
to predict what a scene will cost.

Bodies are box MESHES rather than analytic cubes, so each one takes a cooked
convex hull the way a real fragment does, and they are dropped in a dense
cluster (~1 body per cubic metre, the density a collapsed building's fragments
come out at) so the broadphase and solver have real contacts to chew.

CAVEAT: a box hull is the CHEAPEST thing to cook. Real Voronoi shell fragments
carry far more vertices, so the cook column here is a floor, not an estimate.
Measure the real thing with `scene_prep.settle_rigid_props`'s own report line.
"""

import sys, time

from isaacsim import SimulationApp
app = SimulationApp(launch_config={"headless": True})

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

from disaster import settle as S

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
# bare      what `scene_prep.settle_rigid_props` does today: a plain
#           UsdPhysics.Scene, PhysX's own defaults, stepped with app.update()
# cpu/gpu   the same, with `settle.configure_scene` applied
# sc_cpu    step physics directly (SimulationContext, render=False), no Kit
#           update loop and no renderer
# sc_gpu    the same on the GPU
# fabric_*  as sc_*, but with PhysX's per-step USD write-back turned OFF:
#           the sim runs in Fabric and USD is written once, at the end, which
#           is all a settle needs (nothing reads the intermediate poses).
MODE = sys.argv[2].lower() if len(sys.argv) > 2 else "bare"
GPU = MODE.endswith("gpu")
DIRECT = MODE.startswith("sc") or MODE.startswith("fabric")
FABRIC = MODE.startswith("fabric")
FRAMES = int(sys.argv[3]) if len(sys.argv) > 3 else 180

# Rubble-like density: ~1 body per cubic metre, which is what a collapsed
# building's fragments come out at (872 pieces inside a ~30 m footprint).
side = max(10.0, (N / 1.0) ** (1.0 / 3.0) * 1.6)

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, Sdf.Path("/World"))

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
# "bare" is what `scene_prep.settle_rigid_props` does today: a plain
# UsdPhysics.Scene with no PhysxSceneAPI, i.e. PhysX's own defaults.
if MODE != "bare":
    S.configure_scene(stage, scene.GetPrim(), gpu=GPU)
cfg = S.describe_scene(stage)
print(f"[bench] N={N} mode={MODE} "
      f"broadphase={cfg.get('broadphase')} solver={cfg.get('solver')} "
      f"region={side:.0f}m", flush=True)

ground = UsdGeom.Mesh.Define(stage, "/World/ground")
g = side * 2
ground.CreatePointsAttr([Gf.Vec3f(-g, -g, 0), Gf.Vec3f(g, -g, 0),
                         Gf.Vec3f(g, g, 0), Gf.Vec3f(-g, g, 0)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

rng = np.random.default_rng(0)
pos = rng.uniform(-side / 2, side / 2, size=(N, 3))
pos[:, 2] = rng.uniform(1.0, side * 0.6, size=N)
half = 0.5
box = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) * half
faces = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]

t = time.time()
for i in range(N):
    m = UsdGeom.Mesh.Define(stage, f"/World/b{i}")
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in box])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr(faces)
    UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(*pos[i]))
    prim = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(
        UsdPhysics.Tokens.convexHull)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    px.CreateMaxDepenetrationVelocityAttr().Set(1.0)
    px.CreateMaxLinearVelocityAttr().Set(30.0)
author_s = time.time() - t

if DIRECT:
    # PHYSICS ONLY. `app.update()` is a whole Kit frame — renderer, Fabric
    # sync, USD change notifications — and the settle needs none of it. This
    # is the same physics on the same scene, stepped directly.
    from isaacsim.core.api import SimulationContext
    sc = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0,
                           stage_units_in_meters=1.0)
    import carb
    st = carb.settings.get_settings()
    if FABRIC:
        st.set_bool("/physics/updateToUsd", False)
        st.set_bool("/physics/updateVelocitiesToUsd", False)
    t = time.time()
    sc.initialize_physics()
    cook_s = time.time() - t
    t = time.time()
    for _ in range(FRAMES - 1):
        sc.step(render=False)
    sim_s = time.time() - t
    if FABRIC:
        # One write-back, at the end. This is the pose the settle bakes.
        t2 = time.time()
        st.set_bool("/physics/updateToUsd", True)
        sc.step(render=False)
        print(f"[bench] final USD write-back {time.time() - t2:.1f}s", flush=True)
else:
    app_i = omni.kit.app.get_app()
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    tl.play()
    t = time.time()
    app_i.update()
    cook_s = time.time() - t
    t = time.time()
    for _ in range(FRAMES - 1):
        app_i.update()
    sim_s = time.time() - t
    tl.stop()

z = np.array([UsdGeom.XformCache().GetLocalToWorldTransform(
    stage.GetPrimAtPath(f"/World/b{i}")).ExtractTranslation()[2]
    for i in range(min(N, 2000))])
print(f"[bench] RESULT N={N} {MODE} "
      f"author={author_s:.1f}s cook={cook_s:.1f}s sim={sim_s:.1f}s "
      f"total={author_s + cook_s + sim_s:.1f}s "
      f"z_med={np.median(z):.2f} z_min={z.min():.2f} "
      f"through_floor={int((z < -2).sum())}", flush=True)
app.close()
