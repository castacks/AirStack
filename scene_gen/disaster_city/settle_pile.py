"""Settle a rubble pile under gravity (PhysX) and bake the rest poses: nothing floats.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh settle_pile.py rubble_west/R01_assets.usd [--seconds 3]

rubble_pile.py stacks pieces on a running height estimate, so some hang in the air over the gaps
between what is under them. Here every piece's box collider (80% of its bounds, the one the scene
uses) becomes a dynamic rigid body -- concrete density, high friction, damped, with a slow depenetration (they start interpenetrating) -- with the mound, B01
and the site ground as static colliders; the simulation runs `seconds`, and each piece and its collider
take the box's final pose. Pieces that end up > 2 m below where they started (slid off the pile) keep
their start pose. The pile's layer is rewritten in place.
"""
import argparse, sys, tempfile
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Gf
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R

ap = argparse.ArgumentParser(); ap.add_argument("pile"); ap.add_argument("--seconds", type=float, default=3.0); a = ap.parse_args()
src = R / a.pile
tmp = Path(tempfile.mkdtemp()) / "settle.usda"
work = Usd.Stage.CreateNew(str(tmp)); UsdGeom.SetStageUpAxis(work, "Z"); UsdGeom.SetStageMetersPerUnit(work, 1.0)
work.SetDefaultPrim(work.DefinePrim("/World", "Xform"))
work.DefinePrim("/World/pile").GetReferences().AddReference(str(src))
for rel in ("b01/B01.usd", "site_ground.usd"):                     # what the pile rests on / leans against
    if (R / rel).exists(): work.DefinePrim(f"/World/{Path(rel).stem}").GetReferences().AddReference(str(R / rel))
sc = UsdPhysics.Scene.Define(work, "/World/physics"); sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); sc.CreateGravityMagnitudeAttr(9.81)
rough = UsdShade.Material.Define(work, "/World/rough"); pm = UsdPhysics.MaterialAPI.Apply(rough.GetPrim())
pm.CreateStaticFrictionAttr(1.2); pm.CreateDynamicFrictionAttr(1.0); pm.CreateRestitutionAttr(0.0)
cols = [p for p in work.GetPrimAtPath("/World/pile/colliders").GetChildren()]
start = {}
for c in cols:
    UsdPhysics.RigidBodyAPI.Apply(c); UsdPhysics.MassAPI.Apply(c).CreateDensityAttr(2000.0)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(c); px.CreateLinearDampingAttr(1.5); px.CreateAngularDampingAttr(4.0)
    px.CreateMaxDepenetrationVelocityAttr(0.15)                     # the pieces start interpenetrating: ease apart, don't explode
    UsdShade.MaterialBindingAPI.Apply(c.GetChild("box")).Bind(rough, UsdShade.Tokens.weakerThanDescendants, "physics")
    start[c.GetName()] = np.array(UsdGeom.Xformable(c).ComputeLocalToWorldTransform(0))
work.Save(); print(f"{len(cols)} pieces", flush=True)

ctx = omni.usd.get_context(); ctx.open_stage(str(tmp))
for _ in range(30): app.update()
tl = omni.timeline.get_timeline_interface(); tl.play()
for i in range(int(a.seconds * 60)):
    app.update()
    if i % 60 == 0: print(f"  t = {i / 60:.0f} s", flush=True)
tl.pause(); app.update()
st = ctx.get_stage()
end = {c.GetName(): np.array(UsdGeom.Xformable(st.GetPrimAtPath(c.GetPath())).ComputeLocalToWorldTransform(0)) for c in cols}
tl.stop()

out = Usd.Stage.Open(str(src)); root = out.GetDefaultPrim().GetPath()
moved, kept = [], 0
for name, e in end.items():
    s0 = start[name]
    if not np.isfinite(e).all() or s0[3, 2] - e[3, 2] > 2.0: kept += 1; continue   # slid off the pile: leave it
    for p in (out.GetPrimAtPath(root.AppendPath(f"colliders/{name}")), out.GetPrimAtPath(root.AppendPath(f"pieces/p{name[1:]}"))):
        if not p: continue
        xf = UsdGeom.Xformable(p); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(Gf.Matrix4d(*e.ravel().tolist()))
    moved.append(np.linalg.norm(e[3, :3] - s0[3, :3]))
out.GetRootLayer().Save()
d = np.array(moved) if moved else np.zeros(1)
print(f"settled {len(moved)} pieces ({kept} slid off, left in place): moved median {np.median(d):.2f} m, p90 {np.percentile(d, 90):.2f} m, max {d.max():.2f} m -> {src}")
app.close()
