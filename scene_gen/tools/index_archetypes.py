#!/usr/bin/env python
"""index_archetypes.py — catalogue the damage archetypes, with measured extents.

    AirStack/.venv/bin/python scene_gen/tools/index_archetypes.py \
        --out scene_gen/_plans/archetypes.tsv

The earthquake archetype set is 438 local USDs named `<base>_<state>.usd` over
five damage states. This measures every one and records the state alongside the
base building's PRISTINE footprint, which is the number a placement query
actually wants.

WHY THE PRISTINE FOOTPRINT AND NOT THE DAMAGED ONE. A damaged archetype's
bounding box includes its debris, and debris travels: `Amar_Tower` is 42 x 49 m
standing and 177 x 177 m once it has partially collapsed. Matching a 20 m plot
against that box would reject every collapsed building in the set and keep only
the small ones. The footprint the building OCCUPIED is what has to fit the plot;
the debris is allowed to overhang, exactly as it does in a real collapse.

`.orig` files are the pre-bake originals and are skipped.
"""

import argparse
import glob
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

STATES = ("pristine", "cracked", "soft_storey", "partial_collapse", "pancaked",
          "collapsed", "leaning", "racked", "debris", "rubble", "roof_loss",
          "scoured", "windthrown", "shattered")
# Roughly increasing damage; a query for "destroyed" wants the tail of this.
SEVERITY = {s: i for i, s in enumerate(
    ("pristine", "cracked", "roof_loss", "racked", "leaning", "soft_storey",
     "scoured", "shattered", "partial_collapse", "windthrown", "pancaked",
     "collapsed", "debris", "rubble"))}


def split_state(stem):
    for s in sorted(STATES, key=len, reverse=True):
        if stem.endswith("_" + s):
            return stem[: -len(s) - 1], s
    return stem, ""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--roots", default="assets/archetypes_urban_v3,assets/archetypes_tornado,"
                                       "assets/archetypes_hurricane,assets/archetype,"
                                       "assets/archetype_r15,"
                                       "assets/standalone/buildings/destroyed")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    from pxr import Usd, UsdGeom

    base_dir = os.path.dirname(_HERE)
    rows = []
    for root in args.roots.split(","):
        root = root.strip()
        pat = os.path.join(base_dir, root, "**", "*.usd*")
        for f in sorted(glob.glob(pat, recursive=True)):
            nm = os.path.basename(f)
            # `_edit_session.usda` is a stray Kit edit-target left in the pack;
            # it references the archetypes by a CONTAINER path (/isaac-sim/...)
            # that does not exist here, and is not an asset in its own right.
            if ".orig." in f or nm.startswith("_") or f.endswith((".usda.orig", ".json")):
                continue
            stem = re.sub(r"\.usd[ac]?$", "", os.path.basename(f))
            base, state = split_state(stem)
            try:
                st = Usd.Stage.Open(f)
                st.Load()
                mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
                bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       [UsdGeom.Tokens.default_])
                r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                mn, mx = r.GetMin(), r.GetMax()
                ext = [(mx[i] - mn[i]) * mpu for i in range(3)]
            except Exception:
                continue
            rows.append({
                "path": os.path.relpath(f, os.path.dirname(base_dir)),
                "base": base, "state": state,
                "severity": SEVERITY.get(state, -1),
                "x": ext[0], "y": ext[1], "z": ext[2],
                "set": root.split("/")[-1],
            })

    # The pristine footprint of each base, so a damaged row can be matched on
    # the plot it actually stood on rather than on its debris field.
    pristine = {}
    for r in rows:
        if r["state"] == "pristine":
            pristine[r["base"]] = (r["x"], r["y"], r["z"])
    for r in rows:
        p = pristine.get(r["base"])
        r["px"], r["py"], r["pz"] = p if p else (r["x"], r["y"], r["z"])
        r["has_pristine"] = "yes" if p else "no"

    cols = ["path", "set", "base", "state", "severity", "x", "y", "z",
            "px", "py", "pz", "has_pristine"]
    out = args.out if os.path.isabs(args.out) else os.path.join(base_dir, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as fh:
        fh.write("\t".join(cols) + "\n")
        for r in sorted(rows, key=lambda r: (r["set"], r["base"], r["severity"])):
            fh.write("\t".join(
                f"{r[c]:.2f}" if isinstance(r[c], float) else str(r[c])
                for c in cols) + "\n")

    import collections
    by_state = collections.Counter(r["state"] for r in rows)
    print(f"[archetypes] {len(rows)} rows -> {out}")
    print(f"[archetypes] {len(pristine)} distinct buildings with a pristine twin")
    for s, n in sorted(by_state.items(), key=lambda kv: -kv[1]):
        print(f"    {n:4d}  {s or '(no state)'}")


if __name__ == "__main__":
    main()
