#!/usr/bin/env python
"""open_in_isaac.py — open a built scene in the Isaac Sim GUI, on the host.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES DISPLAY=:1 \
    AirStack/.venv/bin/python scene_gen/tools/open_in_isaac.py /tmp/dc_final.usd

The container cannot do this: `isaac-sim` is bind-mounted from a DIFFERENT
checkout (`/home/pranavkumara/Documents/AirStack`) and authenticates to Nucleus
as `guest`, so every library reference comes back ACCESS_DENIED. The repo venv
carries a full Isaac Sim install and takes credentials from the environment.

CREDENTIALS ARE NEEDED EVEN THOUGH THE FILE IS LOCAL: the 64 MB stage is local
geometry plus absolute `omniverse://` references — the buildings, the HDR
skybox and most ground materials are pulled at open time.

The stage path is taken POSITIONALLY, and consumed off `sys.argv` before
`SimulationApp` boots. SimulationApp forwards whatever is left of argv straight
to Kit, so an unrecognised `--usd ...` reaches the kernel as a Kit argument.
"""
import sys

usd = sys.argv[1] if len(sys.argv) > 1 else ""
# Optional "WxH" for the WINDOW. `width`/`height` in the launch config are the
# render resolution; the window itself is `window_width`/`window_height` and
# defaults to 1440x900, which is a small pane on a 4K desktop and smaller still
# over a remote-desktop link.
win = sys.argv[2] if len(sys.argv) > 2 else ""
sys.argv = sys.argv[:1]
if not usd:
    sys.exit("usage: open_in_isaac.py <stage.usd>")

from isaacsim import SimulationApp
cfg = {"headless": False, "width": 1920, "height": 1080}
if win and "x" in win.lower():
    w, h = (int(v) for v in win.lower().split("x"))
    cfg.update(window_width=w, window_height=h, width=w, height=h)
app = SimulationApp(launch_config=cfg)
import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom

# open_stage's return shape varies by Kit version: a bare bool here, a
# (ok, error) pair elsewhere. Normalise rather than unpack blind.
res = omni.usd.get_context().open_stage(usd)
ok = res[0] if isinstance(res, (tuple, list)) else bool(res)
print(f"[open] {usd} -> ok={ok}", flush=True)

# Kit opens on whatever camera the stage last had, which for an exported
# stage is the origin looking down -Z — a close-up of one grass verge. Frame
# the whole site instead, from the SE at a shallow angle: an overhead shot
# flattens the buildings and this scene is judged on its skyline.
stage = omni.usd.get_context().get_stage()
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
mn, mx = r.GetMin(), r.GetMax()
cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
span = max(mx[0] - mn[0], mx[1] - mn[1])
cam = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
cam.CreateFocalLengthAttr(24.0)
# Clipping matters at this range: the default far plane is 1e6 stage units,
# but the near plane of 1.0 m would clip nothing while 0.1 lets you fly in.
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 10000.0))
eye = Gf.Vec3d(cx + span * 0.62, cy - span * 0.62, span * 0.42)
tgt = Gf.Vec3d(cx, cy, 0.0)
fwd = (tgt - eye).GetNormalized()
right = Gf.Cross(fwd, Gf.Vec3d(0, 0, 1)).GetNormalized()
up = Gf.Cross(right, fwd)
m = Gf.Matrix4d(right[0], right[1], right[2], 0,
                up[0], up[1], up[2], 0,
                -fwd[0], -fwd[1], -fwd[2], 0,
                eye[0], eye[1], eye[2], 1)
UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(m)
get_active_viewport().camera_path = cam.GetPath()
print(f"[open] framed {mx[0]-mn[0]:.0f} x {mx[1]-mn[1]:.0f} m on "
      f"/World/OverviewCamera", flush=True)

print("[open] window is up on this display; close it to exit.", flush=True)
while app.is_running():
    omni.kit.app.get_app().update()
app.close()
