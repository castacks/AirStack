#!/usr/bin/env python3
"""Lit, framed viewer for an already-exported experimental fire bake.

Review lights, ground and camera are authored into the anonymous session
layer.  The baked root layer is never saved or modified.
"""
import importlib.util
import os
import sys

USD = os.environ.get("OPEN_USD", "").strip()
SNAP_DIR = os.environ.get(
    "SNAP_DIR", "/isaac-sim/.nvidia-omniverse/logs/fast_sm30_f5").strip()
if not USD or not os.path.exists(USD):
    sys.exit("OPEN_USD must name an existing container-visible USD")

from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

ctx = omni.usd.get_context()
ctx.open_stage(USD)
for _ in range(60):
    app.update()
stage = ctx.get_stage()
if stage is None:
    app.close()
    sys.exit("stage failed to open")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Everything below is transient review state, never part of the bake.
stage.SetEditTarget(stage.GetSessionLayer())
world = stage.GetPrimAtPath("/World")
# Proof/fallback for bakes authored before the experimental writer began
# marking final triangle meshes as non-subdivision surfaces.
mesh_count = 0
for prim in Usd.PrimRange(world):
    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh_count += 1
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rng = bb.ComputeWorldBound(world).ComputeAlignedRange()
lo, hi = rng.GetMin(), rng.GetMax()
cx, cy = (float(lo[0] + hi[0]) * 0.5, float(lo[1] + hi[1]) * 0.5)
width = float(hi[0] - lo[0])
depth = float(hi[1] - lo[1])
height = float(hi[2] - lo[2])

dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/ReviewDome"))
dome.CreateIntensityAttr(700.0)
dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/ReviewKey"))
key.CreateIntensityAttr(3200.0)
key.CreateAngleAttr(0.9)
key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))

pad = max(width, depth) * 1.8
ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ReviewGround"))
z = float(lo[2]) - 0.02
ground.CreatePointsAttr([
    Gf.Vec3f(cx - pad, cy - pad, z), Gf.Vec3f(cx + pad, cy - pad, z),
    Gf.Vec3f(cx + pad, cy + pad, z), Gf.Vec3f(cx - pad, cy + pad, z)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
ground.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])

snap_path = "/isaac-sim/AirStack/simulation/isaac-sim/utils/snapshots.py"
spec = importlib.util.spec_from_file_location("snapshots", snap_path)
snaps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snaps)
os.makedirs(SNAP_DIR, exist_ok=True)

# View the E/S fire-facing corner and aim at the middle of the burning band.
span = max(width, depth, height)
eye = (cx - 0.88 * span, cy - 0.88 * span, float(lo[2]) + 0.72 * span)
target = (cx, cy, float(lo[2]) + 0.48 * height)
snaps.place_camera(stage, eye, target, focal_mm=22.0)
snaps.snapshot(os.path.join(SNAP_DIR, "SM30_F5_oblique.png"), frames=60)
top_h = float(hi[2]) + max(width, depth) / 1.164 * 1.45
snaps.place_camera(stage, (cx, cy, top_h), (cx, cy, z), focal_mm=18.0)
snaps.snapshot(os.path.join(SNAP_DIR, "SM30_F5_top.png"), frames=50)
# Leave the useful oblique active for the person at the GUI.
snaps.place_camera(stage, eye, target, focal_mm=22.0)
snaps.hide_decorations()
for _ in range(30):
    app.update()

print("[fast_view] lit review ready: %s" % USD)
print("[fast_view] bbox %.1f x %.1f x %.1f; %d meshes forced to subdivision=none; snapshots %s" %
      (width, depth, height, mesh_count, SNAP_DIR))
try:
    while app.is_running():
        app.update()
except KeyboardInterrupt:
    pass
finally:
    timeline.stop()
    app.close()
