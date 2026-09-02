"""people_json_to_annotations.py — a tornado_people PEOPLE_JSON file into a
raven_nav ground-truth annotations file.

    python3 tools/people_json_to_annotations.py \
        --people ~/raven_previews/suburb_tornado_250_people.json \
        --out ../robot/ros_ws/src/global/planners/raven_nav/annotations/RavenSuburbTornado250.json

WHY THIS EXISTS
---------------
`disaster.tornado_people.write_records` writes one JSON object per casualty
(`{"meta": {...}, "people": [...]}`) with the fields the PEOPLE PASS itself
needs (`pose`, `attitude`, `where`, `intensity`, `reach_m`, `covered_frac`,
`visible_parts`, ...) — see that module's own docstring. `raven_nav`'s scorer,
`compare_to_groundtruth.py`, reads a DIFFERENT, much smaller format: a flat
JSON list of `{"class", "bbox_world": {"center_xyz_m", "size_xyz_m"}}` in
world ENU (`_load_gt`, `robot/ros_ws/src/global/planners/raven_nav/
raven_nav/compare_to_groundtruth.py`) — the same shape every other bundled
scene under `raven_nav/annotations/*.json` already uses (e.g.
`ModularHousePreview.json`'s `person` entries: a fixed `[0.7, 0.7, 1.8]` box
per figure). This is the bridge between the two, for exactly one class:
`person`.

THE BOX. Every casualty gets the SAME box, `0.6 x 1.8 x 0.4` m — width
(across the body) x length (head to foot) x height (a lying figure's own
thickness) — not `reach_m`, which varies per pose and per body. This is a
deliberately coarse, uniform box: `compare_to_groundtruth`'s tolerant matcher
scores by IoU-or-within-`--center-dist` (10 m default) or within
`--surface-tol` of the box surface, so an approximate, TIGHT box beats a
model that tries to fit the exact reach of a raised-arm pose and gets it
subtly wrong for one class in fifty.

THE ORIENTATION. `body_axis_deg` (`tornado_people`'s own field: "unit vector
from the placement point toward the figure's head", degrees, 0 = +X) is a
YAW, not an axis-aligned box — a body lying across the wind is thinner along
X than along Y, and reporting `[0.6, 1.8, 0.4]` verbatim regardless of
heading either overstates a broadside figure's footprint or understates an
end-on one. So the 0.6 x 1.8 rectangle is rotated by `body_axis_deg` in the
XY plane and the AXIS-ALIGNED BOUNDING BOX of the rotated rectangle is what
gets written (`compare_to_groundtruth._load_gt` has no yaw field to read, so
an oriented box has to become an AABB somewhere, and this is that step — the
same choice `disaster.gt_hints` makes for houses/cars, except that module
keeps `yaw_deg` as a side-channel for a consumer that wants to rebuild the
OBB and this one has no such consumer to serve). The Z size stays 0.4 m
whatever the yaw, since the rotation is about Z only (a lying figure's
height off the ground does not change with which way its head points).

THE CENTRE. The record's own `(x, y, z)` is used AS THE BOX CENTRE, not
shifted along `body_axis_deg` by half `reach_m` to the body's true midpoint.
`tornado_people._Field.add` places `(x, y, z)` at the body's PLACEMENT POINT
(one END of the body — see `_body_axis`'s docstring), so the true centroid is
offset by roughly half the body's length along its own axis. This is a
KNOWN, DELIBERATE APPROXIMATION: `compare_to_groundtruth`'s defaults (10 m
centre-distance tolerance, IoU-or-touch matching) absorb an offset an order
smaller (well under 1 m) without changing whether a detection counts, and
correcting it needs `reach_m` folded into a per-record shift this tool would
then have to get right for every pose family — not worth it for a coarse
detection-scoring box. Flagged here rather than silently baked in.

PASSTHROUGH. `visibility`, `occlusion` and `covered_frac` ride along on each
entry, extra to what `compare_to_groundtruth._load_gt` reads (it only ever
looks at `class` and `bbox_world`) — so a reader that wants to filter by
"only the fully exposed casualties" or report recall split by occlusion class
can, without re-joining against the original PEOPLE_JSON.
"""
import argparse
import json
import math
import os

_BOX_W_M = 0.6   # across the body
_BOX_L_M = 1.8   # head to foot
_BOX_H_M = 0.4   # lying-figure thickness


def _rotated_aabb_xy(w_m: float, l_m: float, body_axis_deg: float):
    """Full (x, y) extent of a `w_m` (local +X) x `l_m` (local +Y) rectangle
    whose local +Y axis points at `body_axis_deg` in world XY.

    Local axes in world coordinates: `ey = (cos t, sin t)` (along the body,
    toward the head — this IS `body_axis_deg`'s own definition), `ex =
    (sin t, -cos t)` (perpendicular, in-plane). The rectangle's four corners
    are `center +- (w/2) ex +- (l/2) ey`; the AABB half-extent along world X
    is the sum of each half-edge's |X projection|, and likewise for Y.
    """
    t = math.radians(float(body_axis_deg))
    cx, sx_ = math.cos(t), math.sin(t)          # ey = (cx, sx_)
    ex_x, ex_y = sx_, -cx                       # ex = (sin t, -cos t)
    hw, hl = 0.5 * float(w_m), 0.5 * float(l_m)
    half_x = hw * abs(ex_x) + hl * abs(cx)
    half_y = hw * abs(ex_y) + hl * abs(sx_)
    return 2.0 * half_x, 2.0 * half_y


def person_to_annotation(rec: dict) -> dict:
    """One `tornado_people` casualty record -> one `compare_to_groundtruth`
    annotation entry. Pure function — no I/O, easy to unit test."""
    x = float(rec.get("x", 0.0))
    y = float(rec.get("y", 0.0))
    z = float(rec.get("z", 0.0))
    body_axis_deg = float(rec.get("body_axis_deg", 0.0) or 0.0)
    sx, sy = _rotated_aabb_xy(_BOX_W_M, _BOX_L_M, body_axis_deg)
    out = {
        "class": "person",
        "bbox_world": {
            "center_xyz_m": [round(x, 3), round(y, 3), round(z, 3)],
            "size_xyz_m": [round(sx, 3), round(sy, 3), _BOX_H_M],
        },
    }
    # Passthrough — ignored by `compare_to_groundtruth._load_gt` (it reads
    # only `class`/`bbox_world`), useful to anyone else reading this file.
    if "visibility" in rec:
        out["visibility"] = rec["visibility"]
    if "occlusion" in rec:
        out["occlusion"] = rec["occlusion"]
    if "covered_frac" in rec:
        out["covered_frac"] = rec["covered_frac"]
    return out


def convert(people: list) -> list:
    """The full `people` list (from `{"meta": ..., "people": [...]}`) ->
    the flat annotation list `compare_to_groundtruth` reads."""
    return [person_to_annotation(r) for r in people]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--people", required=True,
                    help="tornado_people PEOPLE_JSON file "
                         "({'meta':..., 'people':[...]})")
    ap.add_argument("--out", required=True,
                    help="output annotations file, e.g. "
                         "robot/ros_ws/src/global/planners/raven_nav/"
                         "annotations/<RESULTS_SCENE>.json")
    args = ap.parse_args(argv)

    with open(args.people) as f:
        data = json.load(f)
    people = data.get("people", []) if isinstance(data, dict) else (data or [])
    annotations = convert(people)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(annotations, f, indent=1)

    by_occ = {}
    for a in annotations:
        by_occ[a.get("occlusion", "?")] = by_occ.get(a.get("occlusion", "?"), 0) + 1
    print("[people_json_to_annotations] {0} -> {1}: {2} person "
          "annotation(s), by occlusion {3}".format(
              args.people, args.out, len(annotations), by_occ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
