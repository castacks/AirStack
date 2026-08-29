#!/usr/bin/env python
"""Upload the damage-archetype library to Nucleus.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/upload_archetypes.py

Moved off the repo and renamed (user, 2026-08-29): "for the archetype
assets, don't mention quake anywhere, move them to the nucleus at
Projects/SEI-COA/scene_gen/assets/archetype/... and refer to that".

WHY IT IS A MOVE AND NOT A COPY-AND-KEEP. These are 631 MB across 146 files,
they were referenced `airstack://` (i.e. resolved against the repo root or
`AIRSTACK_ASSET_ROOT`), and only 2 of the 146 are tracked in git — so the
library was already a local artefact that a fresh clone would not have, and
every scene that referenced it silently fell back to a missing asset. Putting
it on Nucleus beside the building packs makes it resolvable the same way
everything else in this project is.

THE NAME. Nothing in the FILENAMES ever said quake (`bld_apartment_DG0.usd`,
`bld_brownstone_row_DG2.usd`, ...) — the directory was the only place the word
appeared. `archetype/` is what it becomes, and the library is not
disaster-specific: the same shells are used by the fire and wind passes.
"""
import os
import sys
import time

import omni.client

SRC = "/isaac-sim/AirStack/scene_gen/assets/archetype"
DST = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "scene_gen/assets/archetype")


def main():
    names = sorted(os.listdir(SRC))
    print("%d entries -> %s" % (len(names), DST), flush=True)
    omni.client.create_folder(DST)
    ok = fail = skip = 0
    t0 = time.time()
    for i, nm in enumerate(names, 1):
        src = os.path.join(SRC, nm)
        if not os.path.isfile(src):
            print("  SKIP (not a file) %s" % nm, flush=True)
            skip += 1
            continue
        dst = DST + "/" + nm
        r = omni.client.copy(src, dst, omni.client.CopyBehavior.OVERWRITE)
        if r == omni.client.Result.OK:
            ok += 1
        else:
            fail += 1
            print("  FAIL %-46s %s" % (nm, r), flush=True)
        if i % 25 == 0 or i == len(names):
            print("  %3d/%d  ok=%d fail=%d  %.0fs"
                  % (i, len(names), ok, fail, time.time() - t0), flush=True)
    print("\nuploaded %d, failed %d, skipped %d in %.0f s"
          % (ok, fail, skip, time.time() - t0), flush=True)
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
