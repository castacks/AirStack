#!/usr/bin/env python
"""facade_photos_launch_script.py — FOUR HEAD-ON facade captures of ONE bake.

    docker exec isaac-sim bash -c "ISAAC_SIM_HEADLESS=true \
      FP_BAKE=/isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_19_F3_s100.usd \
      SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/sm19_facades \
      PYTHONPATH=\"\\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh \
      /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/facade_photos_launch_script.py"

WHY THIS EXISTS (fire_dtc3 row-6 review, 2026-08-31): "Take a photo from all
4 facades head on, you'll see what i mean" — the fire-facing oblique shows
two elevations at a slant, and the pale-lattice class the user is gating on
needs an elevation-orthogonal look per side to attribute. Head-on = the
camera on the outward normal of each elevation at mid height, looking at the
facade centre. No Flow, no physics, no bench row — one referenced bake, the
bench ground/light, four frames (plus one oblique for orientation).
"""
import os
import sys


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


from isaacsim import SimulationApp                              # noqa: E402

_HEADLESS = _env("ISAAC_SIM_HEADLESS", "true").lower() in ("1", "true", "yes")
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

import omni.kit.app                                             # noqa: E402
import omni.usd                                                 # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom                           # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import snapshots as snaps                                       # noqa: E402
from disaster import fire_assembly_lib as fal                   # noqa: E402

BAKE = _env("FP_BAKE")
SNAP_DIR = _env("SNAP_DIR", "/isaac-sim/.nvidia-omniverse/logs/facades")
FRAMES = int(_env("FP_FRAMES", "60"))

if not BAKE or (not BAKE.startswith("omniverse://")
                and not os.path.isfile(BAKE)):
    # omniverse:// sources are allowed through — the resolver decides
    # (user request 2026-08-31: shoot the PRISTINE pack asset for a
    # scorched-vs-unscorched comparison)
    print("[fp] FP_BAKE missing or not a file: {0!r}".format(BAKE))
    simulation_app.close()
    sys.exit(1)
os.makedirs(SNAP_DIR, exist_ok=True)

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
holder = stage.DefinePrim(Sdf.Path("/World/b"))
holder.GetReferences().AddReference(BAKE)

for _ in range(10):
    omni.kit.app.get_app().update()

bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
rng = bc.ComputeWorldBound(holder).ComputeAlignedRange()
lo, hi = rng.GetMin(), rng.GetMax()
cx, cy = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])
W, D, H = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
hz = lo[2] + 0.48 * H
print("[fp] bake {0}: {1:.0f} x {2:.0f} x {3:.0f} m".format(
    os.path.basename(BAKE), W, D, H))

fal.build_ground_and_light(stage, max(W, D, H) * 6.0, prefix="fp")
# FP_LIGHT scales every authored light — a dark-brick asset underexposes
# under the bench light and "the original" reads scorched (user, 2026-08-31:
# "both look scortched"); brighten the pristine reference shots instead of
# arguing about tone from a dim frame.
_LSCALE = float(_env("FP_LIGHT", "1.0"))
if abs(_LSCALE - 1.0) > 1e-3:
    from pxr import UsdLux
    for p in stage.Traverse():
        if p.IsA(UsdLux.DomeLight) or p.IsA(UsdLux.DistantLight):
            a = UsdLux.LightAPI(p).GetIntensityAttr()
            if a and a.Get() is not None:
                a.Set(float(a.Get()) * _LSCALE)
for _ in range(20):
    omni.kit.app.get_app().update()

name = os.path.splitext(os.path.basename(BAKE))[0]
snaps.hide_decorations()

# head-on per elevation: eye on the outward normal, target the facade centre
VIEWS = {
    "S": ((cx, lo[1], hz), (0.0, -1.0)),
    "E": ((hi[0], cy, hz), (1.0, 0.0)),
    "N": ((cx, hi[1], hz), (0.0, 1.0)),
    "W": ((lo[0], cy, hz), (-1.0, 0.0)),
}
for side in ("S", "E", "N", "W"):
    tgt, (nx, ny) = VIEWS[side]
    face_w = W if side in ("S", "N") else D
    d = 0.95 * max(face_w, H)
    eye = (tgt[0] + nx * d, tgt[1] + ny * d, hz)
    snaps.place_camera(stage, eye, tgt, focal_mm=24.0)
    out = os.path.join(SNAP_DIR, "{0}_{1}.png".format(name, side))
    snaps.snapshot(out, frames=FRAMES)
    print("[fp] {0} facade -> {1}".format(side, out))

# one oblique for orientation
eye = (cx + 0.9 * max(W, H), cy - 0.9 * max(D, H), lo[2] + 0.8 * H)
snaps.place_camera(stage, eye, (cx, cy, lo[2] + 0.45 * H), focal_mm=24.0)
snaps.snapshot(os.path.join(SNAP_DIR, "{0}_obl.png".format(name)), frames=FRAMES)
print("[fp] DONE -> {0}".format(SNAP_DIR))
simulation_app.close()
