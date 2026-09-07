#!/usr/bin/env python3
"""Decide what order an unattended Stage A bake should work in.

    python3 scene_gen/tools/bake_order.py > scene_gen/_bakelab/groups.txt

Consumed by `tools/bake_overnight.sh`, one `<name>\\t<type,type,...>` line per
run. An overnight bake is always cut short by something, so this file is what
decides which assets have a COMPLETE set of rungs when it stops — and a
complete set is the unit that is worth anything, because the question a damage
ladder has to answer is how one asset's rungs compare with each other.

ONE RULE, PLUS A TIE-BREAK
--------------------------
1. CHEAPEST FIRST, GLOBALLY, by bounding-box VOLUME — not point count.
2. Ties (assets of similar size) go to the newest assets: `standalone` and
   `selected_citydemo` are what was just added to the pack, the Nucleus packs
   have been baked before.

   Grouping by source and sorting only WITHIN a group was the first attempt,
   and it is wrong for an unattended run: the newest group also holds the
   pack's most expensive towers, so putting that whole group first meant the
   six trees and the twenty-eight small legacy buildings behind it would never
   be reached at all. Sorting globally spends the night on the largest number
   of COMPLETE assets, which is what a partial bake is for.
   Measured 2026-08-27: `house_01` (734 m3, 55 k points) baked all four damaged
   rungs in 76 s for 158 MB, while `office_tower` (257 000 m3, 106 k points)
   spent 364 s on `cracked` ALONE and wrote 1.47 GB for it. The source's
   points barely moved between them; the volume moved 350x, because the
   fracture cuts a fixed-ish fragment size out of the whole envelope and it is
   the CELL COUNT that sets both the time and the file size.

   Fitted over those two points, cells ~ 0.65 * V^(2/3) and ~0.7 MB per cell,
   so a full rung set costs roughly `2 * V^(2/3)` MB: 80 MB for a house, 4.6 GB
   for a 110 000 m3 tower. On a disk with 20-odd GB spare that is the whole
   budget for three towers, or for thirty houses.
"""
from __future__ import annotations

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)

#: Tie-break when two assets are the same size: the newly-added packs first.
GROUP_RANK = {"new_buildings": 0, "vegetation": 1, "legacy_buildings": 2,
              "modular": 3}


def group_of(row: dict) -> str:
    src = str(row.get("source", ""))
    if row.get("build") == "modular":
        return "modular"
    if row.get("kind") == "vegetation":
        return "vegetation"
    if "/assets/standalone/" in src or src.startswith("selected_citydemo"):
        return "new_buildings"
    return "legacy_buildings"


def volume(row: dict) -> float:
    """Bounding-box volume in m3, or +inf when it could not be measured.

    UNMEASURED SORTS LAST, deliberately. The eight `FactoryDistrict` assets do
    not open in host `pxr` at all (a crate-version error; Kit reads them fine),
    so their cost is unknown — and an unknown cost is exactly what an
    unattended bake should attempt only once everything priced has been done.
    """
    if not row.get("opened"):
        return float("inf")
    return ((row.get("x_m") or 0.0) * (row.get("y_m") or 0.0)
            * (row.get("z_m") or 0.0))


def main() -> int:
    path = os.path.join(_SCENE_GEN, "_bakelab", "asset_properties.json")
    if not os.path.exists(path):
        print(f"missing {path}; run tools/asset_properties.py first",
              file=sys.stderr)
        return 1
    rows = json.load(open(path))
    # MODULAR LAST, always. A kit house is assembled rather than referenced, so
    # it has no source USD and no measured volume — it cannot be priced with
    # the others, and it is a SUBURBAN asset in an urban pack besides.
    kit = [r for r in rows if group_of(r) == "modular"]
    rest = sorted((r for r in rows if group_of(r) != "modular"),
                  key=lambda r: (volume(r), GROUP_RANK[group_of(r)], r["type"]))

    # ONE RUN FOR THE PRICED SET, not one per source. Each line costs a ~40 s
    # Kit boot, the disk floor is checked per cell inside the baker anyway, and
    # `--skip-existing` makes any run resumable — so splitting buys nothing.
    if rest:
        print(f"by_cost\t{','.join(r['type'] for r in rest)}")
    if kit:
        print(f"modular\t{','.join(r['type'] for r in kit)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
