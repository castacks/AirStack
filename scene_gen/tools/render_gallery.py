#!/usr/bin/env python
"""render_gallery.py — overviews and close-ups of a built scene, plus a check.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/render_gallery.py \
        --usd /tmp/dc_final.usd --out ~/coasei/portable/images

Overviews are framed from the stage's own bounds, so the set stays correct as
the scene grows. Close-ups are named points on THIS site; edit `SHOTS` when the
plan changes.

IT ALSO AUDITS WHAT IT PHOTOGRAPHS, because both things it reports have been
wrong in ways no single frame showed: debris drawn from one catalogue repeats
the same dozen silhouettes, and a figure can be posed lying yet authored
upright. Distinct-asset count and a per-figure flat/upright verdict are cheap
here and the pictures alone do not give them.

THIS LIVES IN THE REPO ON PURPOSE. It was a /tmp script for a while and /tmp
cleanup ate it between sessions — along with the credentials file and a 572 MB
zip — leaving a sheet quietly built from stale frames.
"""

import argparse
import importlib.util as ilu
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))

SHOTS = [
    ("10_collapse_north_pair.png", (-10.0, 64.0), 34.0, 22.0, 30.0),
    ("11_collapse_house_02.png",   (-2.6, 64.8), 26.0, 18.0, 30.0),
    ("12_collapsed_hall.png",      (-48.7, 31.7), 32.0, 20.0, 30.0),
    ("13_rubble_field_east.png",   (-10.2, 30.3), 40.0, 28.0, 28.0),
    ("14_rubble_field_west.png",   (-68.8, 38.7), 40.0, 28.0, 28.0),
    ("15_victims_dig_crew.png",    (-36.0, 23.0), 13.0, 8.0, 35.0),
    ("16_victims_casualties.png",  (-45.5, 12.2), 12.0, 7.5, 35.0),
    ("17_victims_survivors.png",   (15.6, 35.8), 13.0, 8.0, 35.0),
    ("18_victims_buried.png",      (-2.7, 12.9), 11.0, 7.0, 35.0),
    ("19_burnt_cars.png",          (-10.0, 6.2), 16.0, 10.0, 32.0),
    ("20_sand_pit_wreck.png",      (44.6, 24.9), 30.0, 20.0, 28.0),
    ("21_training_pads.png",       (38.0, -30.0), 55.0, 34.0, 26.0),
    ("22_drill_tower.png",         (-9.2, -70.8), 26.0, 20.0, 30.0),
    ("23_warehouse_pad.png",       (95.9, 48.0), 95.0, 52.0, 28.0),
    ("24_warehouse_walks.png",     (76.0, 45.0), 34.0, 17.0, 30.0, 215.0),
    ("25_dead_trees_west.png",     (-95.0, -20.0), 44.0, 26.0, 28.0),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--usd", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=1600)
    args = ap.parse_args()
    out_dir = os.path.expanduser(args.out).rstrip("/") + "/"
    os.makedirs(out_dir, exist_ok=True)

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True, "width": args.width,
                                       "height": int(args.width * 0.625)})
    import omni.kit.app
    import omni.usd
    from pxr import Usd, UsdGeom

    sp = ilu.spec_from_file_location(
        "snapshots", os.path.normpath(os.path.join(
            _HERE, "..", "..", "simulation", "isaac-sim", "utils",
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
    span = max(mx[0] - mn[0], mx[1] - mn[1])
    tall = max(mx[2], 1.0)
    print(f"[gallery] extent {mx[0]-mn[0]:.0f} x {mx[1]-mn[1]:.0f} m, "
          f"tallest {tall:.1f}", flush=True)

    import collections
    assets = collections.Counter()
    for p in stage.Traverse():
        if p.GetName() != "geo":
            continue
        for spec in p.GetPrimStack():
            for rf in spec.referenceList.GetAddedOrExplicitItems():
                if rf.assetPath:
                    assets[rf.assetPath.rsplit("/", 1)[-1]] += 1
    print(f"[gallery] debris: {sum(assets.values())} pieces from "
          f"{len(assets)} distinct assets", flush=True)
    bad = 0
    for p in stage.Traverse():
        if not p.GetName().startswith("person_"):
            continue
        rr = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if rr.IsEmpty():
            print(f"[gallery] {p.GetName()}: EMPTY BOUND", flush=True)
            bad += 1
            continue
        a, b = rr.GetMin(), rr.GetMax()
        # A laid-down rig is genuinely flat, so this ONE thing the bbox can
        # answer even though it never evaluates skinning.
        if (b[2] - a[2]) > 1.0:
            print(f"[gallery] {p.GetName()} ({p.GetCustomDataByKey('pose')}) "
                  f"is {b[2]-a[2]:.2f} m tall — not laid down", flush=True)
            bad += 1
    print(f"[gallery] figures: {bad} not lying flat", flush=True)

    def shot(name, eye, look, focal=24.0):
        snaps.place_camera(stage, eye, look, focal_mm=focal)
        snaps.snapshot(out_dir + name)

    shot("01_overview_top.png", (cx, cy, tall + span / 1.05), (cx, cy, 0.0))
    for name, bear, elev, dist, focal, lz in (
            ("02_overview_ne.png", 45, 0.42, 0.62, 24.0, 0.0),
            ("03_overview_se.png", -45, 0.42, 0.62, 24.0, 0.0),
            ("04_overview_sw.png", 225, 0.42, 0.62, 24.0, 0.0),
            ("05_overview_nw.png", 135, 0.42, 0.62, 24.0, 0.0),
            ("06_overview_low_e.png", 5, 0.13, 0.78, 32.0, 6.0),
            ("07_overview_low_s.png", -95, 0.13, 0.78, 32.0, 6.0)):
        a = math.radians(bear)
        d = span * dist
        shot(name, (cx + d * math.cos(a), cy + d * math.sin(a), span * elev),
             (cx, cy, lz), focal)
    for row in SHOTS:
        name, (x, y), d, h, focal = row[:5]
        # BEARING, because a fixed one aims through the subject. Every close-up
        # looked from the south-east; the warehouse walks are on the WEST side
        # of a 41 m shed, so that camera sat on the roof and the frame was a
        # white rectangle. Degrees CCW from east, south-east by default.
        a = math.radians(row[5] if len(row) > 5 else -45.0)
        shot(name, (x + d * math.cos(a), y + d * math.sin(a), h),
             (x, y, 1.5), focal)
    print(f"[gallery] {7 + len(SHOTS)} images -> {out_dir}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
