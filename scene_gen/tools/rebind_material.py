#!/usr/bin/env python3
"""rebind_material — point prims at a different material, without re-baking.

    AirStack/.venv/bin/python scene_gen/tools/rebind_material.py \
        <archetype.usd> [more.usd ...] --from MAT [--from MAT ...] --to MAT

Some source assets carry Unreal "fake interior" cards -- a flat, EMISSIVE photo
of an office sitting behind the window glass. Intact they are invisible; the
moment the pipeline hollows and fractures the building they become the visible
surface, and the facade reads as a glowing photograph of a room. That is the
asset's art meeting our damage, not a fracture bug, so the repair is a material
REBIND on the file that already exists -- no re-bake, which is the whole point.

Rebinding covers GeomSubsets as well as Meshes: the standing shell binds per
`GeomSubset` section, while the loose fragments are whole prims, and fixing
only one of the two leaves half the wreck still glowing.

The edit is made on a COPY which then replaces the original, because a rung the
baker just wrote is owned by root inside the container and cannot be opened for
writing from the host -- but the library directory is ours, so an unlink and a
move succeed where an in-place save fails.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import sys


def rebind(path: str, old: set, new: str) -> list:
    from pxr import Usd, UsdShade, Sdf
    tmp = path + ".rebind.tmp.usd"
    shutil.copy2(path, tmp)
    stage = Usd.Stage.Open(tmp)
    looks = None
    for p in stage.Traverse():
        if p.IsA(UsdShade.Material) and p.GetName() == new:
            looks = p
            break
    if looks is None:
        os.unlink(tmp)
        raise SystemExit(f"[rebind] no material named {new!r} in {path}")
    target = UsdShade.Material(looks)
    hit = []
    for p in stage.Traverse():
        api = UsdShade.MaterialBindingAPI(p)
        mp = api.GetDirectBinding().GetMaterialPath()
        if mp and mp.name in old:
            api.Bind(target)
            hit.append((str(p.GetPath()), mp.name))
    stage.GetRootLayer().Save()
    del stage
    os.unlink(path)
    shutil.move(tmp, path)
    return hit


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("usds", nargs="+")
    ap.add_argument("--from", dest="old", action="append", required=True)
    ap.add_argument("--to", dest="new", required=True)
    ap.add_argument("--manifest", default="")
    a = ap.parse_args(argv)
    old = set(a.old)

    touched = {}
    for f in a.usds:
        if not os.path.isfile(f):
            print(f"[rebind] missing: {f}", file=sys.stderr)
            return 2
        hit = rebind(f, old, a.new)
        touched[os.path.basename(f)] = hit
        print(f"[rebind] {os.path.basename(f)}: {len(hit)} prim(s) -> {a.new}")
        for p, was in hit:
            print(f"           {was:<20} {p}")

    if a.manifest and os.path.isfile(a.manifest):
        doc = json.load(open(a.manifest))
        now = dt.datetime.now().astimezone().isoformat(timespec="seconds")
        n = 0
        for r in doc.get("archetypes", []):
            hit = touched.get(str(r.get("usd")))
            if not hit:
                continue
            # Record what it WAS, so the rebind stays reversible: the old
            # binding is nowhere else on disk once the file is rewritten.
            r["material_rebound"] = {was: a.new for _, was in hit}
            r["hand_edited_at"] = now
            n += 1
        json.dump(doc, open(a.manifest, "w"), indent=2, sort_keys=True)
        print(f"[rebind] manifest: {n} record(s) updated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
