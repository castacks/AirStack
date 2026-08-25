#!/usr/bin/env python3
"""Generate world-frame ground-truth boxes for the vehicles the scene-import
script drops into a stage, and merge them into the annotation JSONs.

``example_multi_drone_scene_import.py`` adds cars and trucks to
``downtown_edited_v3_818.usd`` by referencing them out of a donor stage
(``scene_props.add_vehicles``). Those props are real objects in the scene but
they are not in the hand-authored ``<Scene>.json`` annotation files, so nothing
downstream can score against them. This script closes that gap.

Geometry, matching ``scene_props.add_vehicles`` exactly:

* each vehicle is uniformly scaled so its **longest horizontal extent** equals
  the catalog's ``length_m`` (car 4.6 m, truck 7.5 m);
* the placement's ``z_m`` is where the vehicle's **underside** rests, so the box
  centre sits at ``z_m + height/2``;
* ``yaw_deg`` is a rotation about +Z, and the annotation format is an
  axis-aligned box, so a +/-90 deg yaw swaps the X and Y extents.

The donor asset extents are measured, not guessed — but they are measured *at
run time* by ``add_vehicles``, which prints them:

    [scene_props] /World/props/car_1 <- /Root/BP_MCar01  pos=(7.0, -26.4, 0.0)m
                  yaw=90 deg scale=1.085 (asset extent 4.24x2.07x1.44 stage units)

``ASSET_EXTENTS_M`` below holds those numbers for the two kinds in use, taken
from the ModernCityDowntown donor. If a donor stage or asset changes, re-read
the extents off ``logs/isaac-sim.log`` and update the table — everything else
follows.

Usage:
    python3 simulation/isaac-sim/utils/vehicle_annotations.py --write
    python3 simulation/isaac-sim/utils/vehicle_annotations.py            # dry run
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SCENE_IMPORT = os.path.join(
    REPO, "simulation", "isaac-sim", "launch_scripts",
    "example_multi_drone_scene_import.py")
ANNOTATION_DIRS = [
    os.path.join(REPO, "gcs", "ros_ws", "src", "gcs_visualizer", "annotations"),
    os.path.join(REPO, "robot", "ros_ws", "src", "global", "planners",
                 "raven_nav", "annotations"),
]

# Unscaled donor-asset AABBs in stage units (= metres for these donors), as
# printed by scene_props.add_vehicles. Keyed by VEHICLE_CATALOG kind.
ASSET_EXTENTS_M = {
    "car":   (4.24, 2.07, 1.44),   # /Root/BP_MCar01
    "truck": (8.24, 3.42, 3.54),   # /Root/PA_ConstructionTruck01FullyRigged_*
}
# scene_props.VEHICLE_CATALOG length targets, duplicated so this script can run
# outside Isaac Sim (scene_props imports pxr).
TARGET_LENGTH_M = {"car": 4.6, "truck": 7.5}

# The semantic label add_vehicles attaches -> the GT class written here. Must be
# what the mission queries for.
SEMANTIC_CLASS = {"car": "car", "truck": "truck"}


def load_placements(scene_import_path=SCENE_IMPORT):
    """Pull ``_DEFAULT_VEHICLE_PLACEMENTS`` out of the launch script without
    importing it (it pulls in carb/omni at module scope)."""
    src = open(scene_import_path).read()
    key = "_DEFAULT_VEHICLE_PLACEMENTS = ["
    i = src.index(key) + len(key) - 1
    depth, j = 0, i
    while True:
        if src[j] == "[":
            depth += 1
        elif src[j] == "]":
            depth -= 1
            if depth == 0:
                break
        j += 1
    return ast.literal_eval(src[i:j + 1])   # a Python literal (trailing commas)


def box_for(place):
    """-> (class, center_xyz_m, size_xyz_m) for one placement."""
    kind = place.get("kind", "car")
    if kind not in ASSET_EXTENTS_M:
        raise KeyError(f"no measured extent for kind {kind!r}; add it to "
                       f"ASSET_EXTENTS_M (read it off the isaac-sim log)")
    ex, ey, ez = ASSET_EXTENTS_M[kind]
    k = TARGET_LENGTH_M[kind] / max(ex, ey)
    sx, sy, sz = ex * k, ey * k, ez * k

    yaw = float(place.get("yaw_deg", 0.0)) % 180.0
    if abs(yaw - 90.0) < 45.0:          # 90 / -90 -> long axis along Y
        sx, sy = sy, sx

    x = float(place["x_m"])
    y = float(place["y_m"])
    z = float(place.get("z_m", 0.0)) + sz / 2.0   # z_m is the underside
    return (SEMANTIC_CLASS.get(kind, kind),
            [round(x, 3), round(y, 3), round(z, 3)],
            [round(sx, 3), round(sy, 3), round(sz, 3)])


def merge(path, boxes, classes_owned, write):
    """Replace every annotation whose class we own with the freshly generated
    set, leaving the hand-authored entries untouched. Idempotent."""
    with open(path) as f:
        data = json.load(f)
    kept = [it for it in data
            if str(it.get("class", "")).strip().lower() not in classes_owned]
    dropped = len(data) - len(kept)
    new = kept + [{"class": cls,
                   "bbox_world": {"center_xyz_m": c, "size_xyz_m": s}}
                  for cls, c, s in boxes]
    action = "would write" if not write else "wrote"
    print(f"  {os.path.relpath(path, REPO)}: {len(data)} -> {len(new)} "
          f"(dropped {dropped} owned, added {len(boxes)}) — {action}")
    if write:
        with open(path, "w") as f:
            json.dump(new, f, indent=2)
            f.write("\n")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="DowntownWest")
    ap.add_argument("--write", action="store_true",
                    help="actually modify the annotation files")
    args = ap.parse_args(argv)

    places = load_placements()
    boxes = [box_for(p) for p in places]
    print(f"{len(boxes)} vehicle boxes from "
          f"{os.path.relpath(SCENE_IMPORT, REPO)}:")
    for p, (cls, c, s) in zip(places, boxes):
        print(f"  {p.get('name', '?'):8s} {cls:6s} yaw={p.get('yaw_deg', 0):>6}  "
              f"center=({c[0]:>8.2f},{c[1]:>8.2f},{c[2]:>5.2f})  "
              f"size=({s[0]:.2f} x {s[1]:.2f} x {s[2]:.2f})")

    owned = {c.lower() for c, _, _ in boxes}
    print(f"\nclasses owned by this generator: {sorted(owned)}")
    for d in ANNOTATION_DIRS:
        p = os.path.join(d, f"{args.scene}.json")
        if not os.path.exists(p):
            print(f"  {p}: missing, skipped", file=sys.stderr)
            continue
        merge(p, boxes, owned, args.write)
    if not args.write:
        print("\n(dry run — pass --write to apply)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
