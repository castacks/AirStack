#!/usr/bin/env python3
"""Ground-truth boxes for a GENERATED scene, from the people pass's own records.

The annotation JSONs under `gcs/ros_ws/src/gcs_visualizer/annotations/` are
hand-authored for the finished Nucleus stages. A procedurally generated scene
knows its own contents exactly — Stage C placed every survivor and wrote down
where — so its ground truth should be emitted BY THE GENERATOR rather than
measured back off a render or maintained by hand.

`boxes_from_people` builds boxes from the survivor records `scene_gen/targets.py`
already writes (`humans.json`). Those carry x/y/z/yaw per person and a human is
a known size, so there is nothing to measure — and it means the GT for people is
the SAME data the scenario was authored from. The boxes and the scene cannot
disagree.

Output schema matches the existing hand-authored files exactly, so
`annotation_viz_node` reads it with no changes:

    [{"class": str,
      "bbox_world": {"center_xyz_m": [x, y, z], "size_xyz_m": [dx, dy, dz]}},
     ...]

Coordinates are the Isaac world frame, which is the GCS-side global ENU `map`
frame (see `annotation_viz_node`'s docstring, and the world->map note in
`robot.launch.xml`).

THE NAMES AND THE SCHEMA HERE ARE DELIBERATELY NOT OURS. They are
`krrishj/disaster-dataset`'s `simulation/isaac-sim/utils/scene_annotations.py`,
at the same path, so the wildfire side's `boxes_from_placements` /
`boxes_from_scopes` / `DEFAULT_CLASS_MAP` land on top of this as a superset
rather than as a conflict. Do not rename anything in it to match local taste.
"""

import json
import os

# ONE NOMINAL BOX PER PERSON. The rigs are posed (standing, seated, prone,
# buried) and their own AABB varies with the pose, but the GT question is "is
# there a person here", so a nominal box is both simpler and more consistent
# than a measured one — and for a victim under a metre of rubble there is no
# measured box that is not a lie in one direction or the other.
PERSON_SIZE_M = (0.7, 0.7, 1.8)


def people_records(people_json):
    """Records out of a `humans.json`, tolerating a missing or partial file."""
    try:
        with open(people_json) as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        return []
    return doc.get("people", []) if isinstance(doc, dict) else list(doc)


def boxes_from_people(records, size_m=PERSON_SIZE_M):
    """One box per survivor, from the people pass's own records.

    `z` in a record is the ground the person stands on, so the box centre is
    half a body above it. See `targets.to_records` for why that is the support
    surface rather than the authored prim height, and what `z_prim` is for.
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


def annotation_dirs(repo_root):
    """Both places the stack reads annotations from. Written to BOTH, because
    the GCS draws them and raven's scorer reads them, and a GT that differs
    between the picture and the number is worse than no GT. A directory that
    does not exist is skipped, not created — the planner-side one only exists
    on branches that carry the search baselines."""
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
    return written


def write_people_annotations(repo_root, scene_name, people_json, quiet=False):
    """`humans.json` -> `<scene_name>.json` in both annotation dirs.

    The whole people half of the pipeline in one call, for a launch script that
    has just run Stage C. Returns the paths written; an empty list means there
    was nothing to write, which is not an error — a preset with no cohort
    weights places nobody.
    """
    boxes = boxes_from_people(people_records(people_json))
    if not boxes:
        print(f"[annotations] no people in {people_json} — nothing written")
        return []
    return write_annotations(scene_name, boxes, annotation_dirs(repo_root),
                             quiet=quiet)


def gt_from_env(repo_root, people_json, quiet=False):
    """Write the people GT if the run asked for it. Returns the paths written.

    OFF UNLESS `GT_ANNOTATIONS` IS SET, and that is not timidity: the file it
    writes is named after `RESULTS_SCENE`, and a hand-authored annotation file
    for a Nucleus stage of the same name is not something to clobber because
    somebody ran a preview. A scored run turns it on; a look-at-the-scene run
    does not.
    """
    if os.environ.get("GT_ANNOTATIONS", "off").strip().lower() not in (
            "1", "true", "yes", "on"):
        return []
    scene = os.environ.get("RESULTS_SCENE", "").strip()
    if not scene:
        print("[annotations] GT_ANNOTATIONS is on but RESULTS_SCENE is unset "
              "— nothing to name the file after, skipping")
        return []
    return write_people_annotations(repo_root, scene, people_json, quiet=quiet)


def gt_for_config(config, quiet=False):
    """The whole people-GT step for a launch script, in one call.

    Resolves the `humans.json` Stage C just wrote (`targets.default_gt_path`,
    the same function `targets.place` used, so the two cannot drift) and the
    repo root (this file is at `<repo>/simulation/isaac-sim/utils/`), then
    defers to `gt_from_env` for whether to write anything at all.

    THE DEPENDENCY RUNS THIS WAY ROUND ON PURPOSE. `simulation/` may import the
    generator; `scene_gen/` may not import `simulation/` — it is sim-agnostic
    and only needs `pxr`. So the glue lives here, and a launch script is one
    line.
    """
    import targets

    repo = os.path.normpath(os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "..", "..", ".."))
    return gt_from_env(repo, targets.default_gt_path(config), quiet=quiet)
