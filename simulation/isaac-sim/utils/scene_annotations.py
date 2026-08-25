#!/usr/bin/env python3
"""Ground-truth boxes for a GENERATED scene, written from the live stage.

The annotation JSONs under `gcs/ros_ws/src/gcs_visualizer/annotations/` are
hand-authored for the finished Nucleus stages. A procedurally generated scene
knows its own contents exactly — the layout generator placed every house, car,
tree and survivor — so its ground truth should be emitted BY THE GENERATOR
rather than measured back off a render or maintained by hand.

Two sources, because the two kinds of object know their extents differently:

* `boxes_from_placements` walks the placements the generator produced and reads
  each one's WORLD-ALIGNED bounding box off the composed stage with
  `UsdGeom.BBoxCache`. Nothing is assumed about asset extents, scale or pivot —
  which is the failure mode `vehicle_annotations.py` has to work around with a
  hand-maintained `ASSET_EXTENTS_M` table taken off a log.
* `boxes_from_people` builds boxes from the survivor records the people pass
  already writes (`humans.json`). Those carry x/y/z/yaw per person, and a human
  is a known size, so there is nothing to measure — and it means the GT for
  people is the SAME data the scenario was authored from.

Output schema matches the existing files exactly, so `annotation_viz_node`
reads it with no changes:

    [{"class": str,
      "bbox_world": {"center_xyz_m": [x, y, z], "size_xyz_m": [dx, dy, dz]}},
     ...]

Coordinates are the Isaac world frame, which is the GCS-side global ENU `map`
frame (see `annotation_viz_node`'s docstring, and the world->map note in
`robot.launch.xml`).
"""

import json
import os

# Placement `category` -> annotation `class`. Categories not listed are DROPPED:
# a ground-truth file full of hedges and paving slabs is noise, and every extra
# box is something a recall metric has to be told to ignore.
DEFAULT_CLASS_MAP = {
    "plot_car": "car",
    "car": "car",
    "evac_car": "car",
    "plot_tree": "tree",
    "tree": "tree",
    "plot_pool": "pool",
    "props": "prop",
    "person": "person",
}

# Every category starting with one of these becomes that class. Buildings are
# authored PIECE BY PIECE (wall, roof, garage, porch), so they only make sense
# grouped — see `group_key` below.
PREFIX_CLASS_MAP = (
    ("bld_", "house"),
    ("house", "house"),
)

# A person, in metres. Boxes for people come from records, not geometry: the
# rigs are posed (standing, sitting, prone) and their own AABB varies with the
# pose, but the GT question is "is there a person here", so one nominal box per
# person is both simpler and more consistent.
PERSON_SIZE_M = (0.7, 0.7, 1.8)


def class_for(category):
    """Annotation class for a placement category, or None to drop it."""
    if not category:
        return None
    if category in DEFAULT_CLASS_MAP:
        return DEFAULT_CLASS_MAP[category]
    for prefix, cls in PREFIX_CLASS_MAP:
        if category.startswith(prefix):
            return cls
    return None


def _union(a, b):
    if a is None:
        return b
    lo = [min(a[0][i], b[0][i]) for i in range(3)]
    hi = [max(a[1][i], b[1][i]) for i in range(3)]
    return (lo, hi)


def boxes_from_placements(stage, placements, class_map=None, min_size_m=0.2):
    """World-AABB boxes for every placement whose category maps to a class.

    Placements carrying the same `group` key are merged into ONE box. That is
    what makes a house a house: `build_building` emits a separate placement per
    wall, roof and garage piece, and one box per piece would score a single
    building as a dozen detections.
    """
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    groups = {}          # key -> [class, (lo, hi)]
    for idx, p in enumerate(placements):
        path = p.get("prim_path")
        if not path:
            continue
        cls = (class_map or {}).get(p.get("category")) if class_map else None
        cls = cls or class_for(p.get("category"))
        if cls is None:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        lo = [rng.GetMin()[i] for i in range(3)]
        hi = [rng.GetMax()[i] for i in range(3)]
        # No group key -> the placement is its own object.
        key = p.get("group") or f"{cls}#{idx}"
        if key in groups:
            groups[key][1] = _union(groups[key][1], (lo, hi))
        else:
            groups[key] = [cls, (lo, hi)]

    boxes = []
    for cls, (lo, hi) in groups.values():
        size = [hi[i] - lo[i] for i in range(3)]
        # A degenerate box is a placement that drew nothing, or a decal. Either
        # way it is not an object to be found.
        if max(size) < min_size_m:
            continue
        boxes.append({
            "class": cls,
            "bbox_world": {
                "center_xyz_m": [round((lo[i] + hi[i]) / 2.0, 3) for i in range(3)],
                "size_xyz_m": [round(s, 3) for s in size],
            },
        })
    return boxes


def boxes_from_people(records, size_m=PERSON_SIZE_M):
    """One box per survivor, from the people pass's own records.

    `z` in a record is the ground the person stands on, so the box centre is
    half a body above it.
    """
    boxes = []
    for r in records or []:
        try:
            x, y, z = float(r["x"]), float(r["y"]), float(r.get("z", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        boxes.append({
            "class": "person",
            "bbox_world": {
                "center_xyz_m": [round(x, 3), round(y, 3),
                                 round(z + size_m[2] / 2.0, 3)],
                "size_xyz_m": [size_m[0], size_m[1], size_m[2]],
            },
        })
    return boxes


def people_records(people_json):
    """Records out of a `humans.json`, tolerating a missing or partial file."""
    try:
        with open(people_json) as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        return []
    return doc.get("people", []) if isinstance(doc, dict) else list(doc)


def annotation_dirs(repo_root):
    """Both places the stack reads annotations from. Written to BOTH, because
    the GCS draws them and raven's scorer reads them, and a GT that differs
    between the picture and the number is worse than no GT."""
    return [
        os.path.join(repo_root, "gcs", "ros_ws", "src", "gcs_visualizer",
                     "annotations"),
        os.path.join(repo_root, "robot", "ros_ws", "src", "global", "planners",
                     "raven_nav", "annotations"),
    ]


def write_annotations(scene_name, boxes, dirs, quiet=False):
    """Write `<scene_name>.json` into every existing directory in `dirs`."""
    written = []
    for d in dirs:
        if not os.path.isdir(d):
            continue
        path = os.path.join(d, f"{scene_name}.json")
        with open(path, "w") as fh:
            json.dump(boxes, fh, indent=1)
        written.append(path)
    if not quiet:
        tally = {}
        for b in boxes:
            tally[b["class"]] = tally.get(b["class"], 0) + 1
        summary = ", ".join(f"{v} {k}" for k, v in sorted(tally.items()))
        print(f"[annotations] {len(boxes)} boxes ({summary})")
        for p in written:
            print(f"[annotations]   -> {p}")
        if not written:
            print(f"[annotations]   WARN nothing written; no dir of {dirs} exists")
    return written
