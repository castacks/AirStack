#!/usr/bin/env python
"""open_usd_launch_script.py — open ONE usd in Isaac and hold it, nothing else.

For looking at something. No scene generation, no disaster pass, no physics
step — it boots Kit, opens the stage named by `OPEN_USD`, frames it, and idles
so a human can fly around.

    docker exec isaac-sim bash -lc '
      cd /isaac-sim/AirStack &&
      OPEN_USD=/isaac-sim/AirStack/scene_gen/_plans/slice_compare/slice_compare.usd \\
      ./python.sh simulation/isaac-sim/launch_scripts/open_usd_launch_script.py'

`OPEN_USD` is required and must be a path the CONTAINER can see — the repo is
mounted at /isaac-sim/AirStack, a host `~/…` path is not visible.

HEADLESS defaults OFF here, unlike every other launcher in this directory:
the entire purpose is that someone looks at it. Set `ISAAC_SIM_HEADLESS=1`
to boot without a window (useful only to check the stage composes).
"""
import os
import sys

USD = os.environ.get("OPEN_USD", "").strip()
HEADLESS = str(os.environ.get("ISAAC_SIM_HEADLESS", "0")).strip() in (
    "1", "true", "True")

if not USD:
    sys.exit("OPEN_USD is not set — give it a container-visible .usd path")
if not os.path.exists(USD):
    sys.exit("OPEN_USD does not exist in the container: %s\n"
             "  (the repo is mounted at /isaac-sim/AirStack; a host ~/ path "
             "is not visible here)" % USD)

from isaacsim import SimulationApp  # noqa: E402

# `--no-window` is NOT passed even when headless: this script never freezes or
# exports, so it does not hit the carb.eventdispatcher/omni.appwindow segfault
# that forces `--no-window` on the freeze path.
app = SimulationApp({"headless": HEADLESS})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

print("[open_usd] opening %s" % USD)
omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()

stage = omni.usd.get_context().get_stage()
if stage is None:
    app.close()
    sys.exit("[open_usd] stage failed to open")

# Report what is actually in it, so the pane says whether the thing loaded
# rather than leaving it to the viewport.
scopes = {}
for prim in stage.GetPrimAtPath("/World").GetChildren() \
        if stage.GetPrimAtPath("/World") else []:
    n = 0
    for d in Usd.PrimRange(prim):
        if d.IsA(UsdGeom.Mesh):
            n += 1
    scopes[prim.GetName()] = (n, dict(prim.GetCustomData()))

print("[open_usd] /World holds %d scope(s):" % len(scopes))
for name, (n, cd) in scopes.items():
    extra = ""
    if cd:
        extra = "  " + "  ".join("%s=%s" % (k.split(":")[-1], v)
                                 for k, v in sorted(cd.items()))
    print("    %-18s %5d mesh(es)%s" % (name, n, extra))

bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
r = bb.ComputeWorldBound(stage.GetPrimAtPath("/World")).ComputeAlignedRange()
if not r.IsEmpty():
    print("[open_usd] world bbox  x[%.1f, %.1f]  y[%.1f, %.1f]  z[%.1f, %.1f]"
          % (r.GetMin()[0], r.GetMax()[0], r.GetMin()[1], r.GetMax()[1],
             r.GetMin()[2], r.GetMax()[2]))

print("[open_usd] ready — Ctrl-C this pane to close")
try:
    while app.is_running():
        app.update()
except KeyboardInterrupt:
    pass
finally:
    app.close()
