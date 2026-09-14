#!/usr/bin/env python
"""render_aerial.py — ONE high-resolution nadir frame of a built scene.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/render_aerial.py \
        --usd /tmp/dc.usd --out ~/coasei/portable/disaster_city_aerial.png

`render_gallery.py` makes a top-down too, but at gallery resolution and as one
frame among twenty-one. This is the plan view on its own and big enough to read
block by block, which is how the site gets compared against the photograph it
was traced from — the working loop for this scene is "look at the aerial, look
at the render, find what differs".

A LONG LENS, NOT A WIDE ONE. Kit has no orthographic capture here, and a 24 mm
nadir leans every building outward from the centre, which is exactly the error
that makes a render hard to compare with a real overhead image. Framing the
same ground from four times the height at 50 mm keeps the walls near vertical.
The render resolution is fixed when the app boots, before the stage is open, so
the aspect comes from `--width`/`--height`; the defaults suit this site.
"""

import argparse
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--usd", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=2400)
    ap.add_argument("--height", type=int, default=1840)
    ap.add_argument("--focal", type=float, default=50.0)
    ap.add_argument("--margin", type=float, default=1.06,
                    help="fraction of the stage extent to frame")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True,
                                       "width": args.width,
                                       "height": args.height})
    import importlib.util as ilu
    import omni.kit.app
    import omni.usd
    from pxr import Usd, UsdGeom

    here = os.path.dirname(os.path.abspath(__file__))
    sp = ilu.spec_from_file_location(
        "snapshots", os.path.normpath(os.path.join(
            here, "..", "..", "simulation", "isaac-sim", "utils",
            "snapshots.py")))
    snaps = ilu.module_from_spec(sp)
    sp.loader.exec_module(snaps)

    ctx = omni.usd.get_context()
    ctx.open_stage(args.usd)
    stage = ctx.get_stage()
    stage.Load()
    for _ in range(360):
        omni.kit.app.get_app().update()

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
    w_m, h_m = mx[0] - mn[0], mx[1] - mn[1]
    # Frame whichever axis the sensor is tighter on. The horizontal aperture is
    # 20.955 mm (`snapshots.place_camera`), so the vertical one follows the
    # pixel aspect; a site wider than the frame has to be fitted on width.
    ap_h = 20.955
    ap_v = ap_h * args.height / float(args.width)
    dist = max(w_m * args.margin * args.focal / ap_h,
               h_m * args.margin * args.focal / ap_v)
    out = os.path.expanduser(args.out)
    # THE MAPPING, printed, because the whole point of this image is to be
    # compared against the plan and the photograph — and a measurement off it
    # ("the shadow runs which way?", "is that patch where the manifest puts
    # it?") needs world metres, not pixels. +X is right and +Y is UP: the
    # plumb-view yaw is pinned so the frame reads as a map (`snapshots._look_at`).
    scale = args.width / (2.0 * (mx[2] + dist) * (ap_h / 2.0) / args.focal)
    print(f"[aerial] extent {w_m:.0f} x {h_m:.0f} m, camera {dist:.0f} m up "
          f"at {args.focal:.0f} mm, {args.width}x{args.height}", flush=True)
    print(f"[aerial] world ({cx:.1f}, {cy:.1f}) is image centre; "
          f"{scale:.3f} px/m, +x right, +y up", flush=True)
    snaps.place_camera(stage, (cx, cy, mx[2] + dist), (cx, cy, 0.0),
                       focal_mm=args.focal)
    snaps.snapshot(out, frames=90)
    # THE MAPPING AS DATA, beside the image. Printing it is enough to read a
    # measurement off by hand; `annotate_aerial.py` needs it exactly, and
    # re-deriving the camera maths in a second tool is how the two drift apart.
    import json
    side = os.path.splitext(out)[0] + ".json"
    with open(side, "w") as fh:
        json.dump({"image": os.path.basename(out),
                   "width_px": args.width, "height_px": args.height,
                   "centre_m": [cx, cy], "px_per_m": scale,
                   "extent_m": [w_m, h_m],
                   "note": "+x right, +y up; px = (w/2 + (x-cx)*s, "
                           "h/2 - (y-cy)*s)"}, fh, indent=1)
    print(f"[aerial] -> {out}", flush=True)
    print(f"[aerial] -> {side}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
