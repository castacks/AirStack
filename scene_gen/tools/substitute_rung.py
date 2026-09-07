#!/usr/bin/env python3
"""substitute_rung — stand one baked rung in for another, on purpose.

    AirStack/.venv/bin/python scene_gen/tools/substitute_rung.py \
        <type> <dest_level> <src_level> [--disaster earthquake]

Some rungs come out of the bake wrong in a way no re-bake fixes -- a
soft-storey that reads as untouched, a partial collapse that never dropped.
The cheap repair is to point that rung at a neighbour's geometry. This does
that COPY plus the two pieces of bookkeeping that are easy to forget:

1. A `.orig.usd` backup, written ONCE. Re-substituting must not overwrite the
   backup with an already-substituted file, or the original bake is gone.

2. The manifest record. A record is mostly MEASUREMENTS -- `frag_cells`,
   `meshes`, `settle`, `usd_mb` -- and after a substitution those describe the
   SOURCE, not this rung. Carrying the destination's old numbers would leave a
   manifest that confidently reports the wrong geometry, which is worse than
   no record. So the measured fields come from the source; only identity
   (`type`, `level`, `usd`, `preview`, `used_by`) stays with the destination.

The preview is NOT re-shot here -- that needs Kit. Run `tools/repreview.py` on
the destination afterwards or the contact sheet keeps the old picture.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

#: Destination-owned: what this rung IS, not what it looks like.
IDENTITY = ("type", "level", "usd", "preview", "used_by")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("type")
    ap.add_argument("dest_level")
    ap.add_argument("src_level")
    ap.add_argument("--disaster", default="earthquake")
    ap.add_argument("--root", default="assets/archetypes")
    a = ap.parse_args(argv)

    d = os.path.join(a.root, a.disaster)
    dest = os.path.join(d, f"{a.type}_{a.dest_level}.usd")
    src = os.path.join(d, f"{a.type}_{a.src_level}.usd")
    man = os.path.join(d, "manifest.json")
    for p in (src, man):
        if not os.path.exists(p):
            print(f"[subst] missing: {p}", file=sys.stderr)
            return 2
    # THE DESTINATION MAY NOT EXIST, and that is the common case: a rung the
    # settle REJECTED never wrote a file or a manifest record, which is
    # precisely when standing a neighbour in for it is wanted. Requiring the
    # destination refused every substitution that mattered.
    fresh = not os.path.exists(dest)
    if fresh:
        print(f"[subst] {os.path.basename(dest)} does not exist — creating it")

    backup = os.path.join(d, f"{a.type}_{a.dest_level}.orig.usd")
    if fresh:
        pass
    elif not os.path.exists(backup):
        shutil.copy2(dest, backup)
        print(f"[subst] backup -> {os.path.basename(backup)}")
    else:
        print(f"[subst] backup exists, kept: {os.path.basename(backup)}")

    # The baker runs as root INSIDE the container, so a freshly baked rung is
    # root-owned and the host user cannot open it for writing -- even though
    # the library directory itself is ours. Unlinking first turns an
    # unwritable FILE into a writable DIRECTORY entry, which we do have.
    try:
        os.unlink(dest)
    except OSError:
        pass
    shutil.copy2(src, dest)
    mb = os.path.getsize(dest) / 1e6
    print(f"[subst] {os.path.basename(src)} -> {os.path.basename(dest)}"
          f"  ({mb:.1f} MB)")

    doc = json.load(open(man))
    recs = doc.get("archetypes", [])
    by = {(r.get("type"), r.get("level")): r for r in recs}
    dr = by.get((a.type, a.dest_level))
    sr = by.get((a.type, a.src_level))
    if sr is None:
        print("[subst] no manifest record for the SOURCE rung; file copied, "
              "record NOT updated", file=sys.stderr)
        return 1
    if dr is None:
        # A rejected rung has no record either. Build one from the source and
        # re-point the identity below, or the archetype sits on disk invisible
        # to Stage B and to every gallery.
        dr = {}
        recs.append(dr)

    now = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    keep = {k: dr[k] for k in IDENTITY if k in dr}
    keep.setdefault("type", a.type)
    keep.setdefault("level", a.dest_level)
    keep.setdefault("usd", os.path.basename(dest))
    keep.setdefault("preview", {
        "obl": f"previews/{a.type}_{a.dest_level}_obl.png",
        "top": f"previews/{a.type}_{a.dest_level}_top.png"})
    if "used_by" not in keep and sr.get("used_by"):
        keep["used_by"] = sr["used_by"]
    dr.clear()
    dr.update(sr)          # the measurements now describe the source
    dr.update(keep)        # ...but this is still the destination rung
    dr["usd_mb"] = round(mb, 1)
    dr["substituted_from"] = os.path.basename(src)
    dr["substituted_at"] = now
    dr["hand_edited_at"] = now

    json.dump(doc, open(man, "w"), indent=2, sort_keys=True)
    print(f"[subst] manifest updated ({len(recs)} records)")
    print(f"[subst] NOW RE-SHOOT: tools/repreview.py {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
