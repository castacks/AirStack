#!/usr/bin/env python3
"""Print ready-to-paste `usds.buildings.<pool>` YAML entries from a faces plan.

    python3 scene_gen/tools/faces_to_yaml.py \\
        --faces scene_gen/_plans/gac_faces.json \\
        --names SM_Building_01,SM_Building_03 \\
        --root "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/" \\
        --scale 0.01 --ext .usd

HOST-SIDE ONLY — plain python3, no `pxr`, reads nothing but the JSON that
`gac_faces.py` already wrote inside the container. The measurement needs USD;
turning it into paste-able config does not.

Three things come out of `gac_faces.json`/`dtc_faces.json` that a hand-edited
asset-set entry needs and would otherwise have to be guessed or eyeballed:

`yaw-offset` — TURNS THE MEASURED front TO -X (WEST), because that is what
`_lay_terrace` assumes a building faces at `facing_deg` 0. Yaw is CCW about
+Z, so a +90 yaw walks the compass E -> N -> W -> S -> E. Solving
"rotate `front` onto W" for each of the four possible fronts:

    front W -> yaw-offset   0      (already facing W)
    front N -> yaw-offset  90      (+90 CCW: N -> W)
    front E -> yaw-offset 180      (+180: E -> W)
    front S -> yaw-offset 270      (+270, i.e. -90 CW: S -> W... check: +90 CCW
                                     from S goes S -> E, so S needs +270 to reach W)

`tags` — a FIXED CONTRACT another agent's code reads, not prose:
    place_mid / place_end / place_corner   for place mid/end/corner
    place_none                             for place none (all_blank assets)
    (nothing)                              for place any — the untagged default
    front:<S>                              the measured front, asset's own frame
    blank:<S1>,<S2>                        the measured blanks, own frame,
                                            OMITTED entirely when there are none

The trailing comment — `scene_gen/tools/plan_png.py` scrapes footprints out of
comments shaped `#\\s*([\\d.]+)\\s*x\\s*([\\d.]+)\\s*x\\s*([\\d.]+)\\s*m`, which
requires the `#` to sit IMMEDIATELY before the three numbers. Putting prose
first (`# 1-sided mid, ... 29x28x55 m`, the pattern already in urban_gac.yaml)
never matches, so every one of those assets falls back to a generic box in the
offline plan. Emitting the size FIRST — `# 29.1 x 28.4 x 55.4 m — 1-sided
mid, ...` — matches the regex regardless of whether that scraper is ever
loosened to also accept size-last.
"""

import argparse
import json
import os

# CCW yaw about +Z: front W needs none, and each 90 deg step walks the front
# one compass letter further from W the "wrong" way (W->S->E->N->W going
# CLOCKWISE is what +90 CCW undoes), so the offset to bring a given front onto
# W grows by 90 in the order W, N, E, S.
FRONT_TO_YAW = {"W": 0, "N": 90, "E": 180, "S": 270}

PLACE_TAG = {"mid": "place_mid", "end": "place_end", "corner": "place_corner",
             "none": "place_none"}          # "any" is untagged, on purpose


def _fmt_scale(s):
    return "%.1f" % s if s == int(s) else ("%g" % s)


def _tags(rec):
    tags = []
    pt = PLACE_TAG.get(rec["place"])
    if pt:
        tags.append(pt)
    tags.append("front:%s" % rec["front"])
    blanks = rec.get("blank_sides") or []
    if blanks:
        tags.append("blank:%s" % ",".join(blanks))
    return tags


def _prose(rec):
    blanks = rec.get("blank_sides") or []
    return "%d-sided %s, front %s, blank %s" % (
        rec["detailed_sides"], rec["place"], rec["front"],
        ",".join(blanks) if blanks else "-")


def emit(rec, root, scale, ext):
    usd = root + rec["name"] + ext
    yaw = FRONT_TO_YAW[rec["front"]]
    tags = ", ".join('"%s"' % t for t in _tags(rec))
    print('      - usd: "%s"' % usd)
    print("        scale: %s" % _fmt_scale(scale))
    print("        yaw-offset: %d" % yaw)
    print("        tags: [%s]" % tags)
    print("        # %.1f x %.1f x %.1f m — %s"
          % (rec["W"], rec["D"], rec["H"], _prose(rec)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--faces", required=True, help="gac_faces.json / dtc_faces.json")
    ap.add_argument("--names", required=True,
                     help='comma-separated building names, or "ALL"')
    ap.add_argument("--root", required=True, help="omniverse:// prefix, trailing /")
    ap.add_argument("--scale", type=float, required=True,
                     help="0.01 for GreatAmericanCity (cm), 1.0 for downtowncity (m)")
    ap.add_argument("--ext", default=".usd", help=".usd or .usdc")
    args = ap.parse_args()

    recs = json.load(open(os.path.normpath(args.faces)))
    by_name = {r["name"]: r for r in recs}

    if args.names.strip().upper() == "ALL":
        order = [r["name"] for r in recs]
    else:
        order = [n for n in (q.strip() for q in args.names.split(",")) if n]

    for nm in order:
        rec = by_name.get(nm)
        if rec is None:
            print("# %s NOT FOUND in %s" % (nm, args.faces))
            continue
        emit(rec, args.root, args.scale, args.ext)


if __name__ == "__main__":
    main()
