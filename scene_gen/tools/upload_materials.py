#!/usr/bin/env python
"""Upload the megascans material library to Nucleus.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/upload_materials.py

WHY. `AIRSTACK_ASSET_ROOT` repoints every `airstack://` reference at
`omniverse://.../Projects/SEI-COA`, and every OSMO MISSION yaml already sets
it — so missions resolve their assets from Nucleus and never carry them. The
dev workflow did not, which meant ~4.4 GB was being rsynced onto every fresh
pod for assets that were already sitting on Nucleus. This closes that gap for
the materials; `upload_archetypes.py` did it for the archetype library.

It also solves a problem git cannot. `Crushed_Asphalt_Ground` at 8K has a
106.7 MB normal map — past GitHub's hard 100 MB limit — so it is NOT in the
repo and cannot be. On Nucleus it is unremarkable, and it resolves through the
same `airstack://` path as everything else.

IDEMPOTENT AND SIZE-CHECKED. An entry already present with the same byte count
is skipped, so re-running after adding one material costs one upload rather
than the whole library. `--force` overrides.
"""
import os
import sys
import time

import omni.client

SRC = "/isaac-sim/AirStack/scene_gen/assets/materials"
DST = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "scene_gen/assets/materials")
# Generated caches, not source. `surge` writes its ripple normal and alpha map
# into `flood/` on demand, keyed by a content hash — uploading them would put
# a rebuildable artefact on Nucleus and then leave it stale.
SKIP_DIRS = {"flood"}
FORCE = "--force" in sys.argv


def remote_sizes(url):
    """{name: size} for one Nucleus folder, empty if it does not exist."""
    r, ents = omni.client.list(url)
    if r != omni.client.Result.OK:
        return {}
    return {e.relative_path: int(getattr(e, "size", 0) or 0) for e in ents}


def push(local, url, have):
    """Copy one FILE if absent or a different size. Returns 'ok'|'skip'|'fail'."""
    name = os.path.basename(local)
    sz = os.path.getsize(local)
    if not FORCE and have.get(name) == sz:
        return "skip"
    r = omni.client.copy(local, url + "/" + name,
                         behavior=omni.client.CopyBehavior.OVERWRITE)
    return "ok" if r == omni.client.Result.OK else "fail"


def main():
    t0 = time.time()
    tally = {"ok": 0, "skip": 0, "fail": 0}
    print("walking %s -> %s" % (SRC, DST), flush=True)
    for root, dirs, files in os.walk(SRC):
        dirs[:] = [d for d in sorted(dirs) if d not in SKIP_DIRS]
        rel = os.path.relpath(root, SRC)
        url = DST if rel == "." else DST + "/" + rel.replace(os.sep, "/")
        omni.client.create_folder(url)
        have = remote_sizes(url)
        n = {"ok": 0, "skip": 0, "fail": 0}
        for f in sorted(files):
            fp = os.path.join(root, f)
            if os.path.isfile(fp):
                n[push(fp, url, have)] += 1
        for k in tally:
            tally[k] += n[k]
        if n["ok"] or n["fail"]:
            print("  %-44s %d up, %d current, %d FAILED"
                  % (rel + "/", n["ok"], n["skip"], n["fail"]), flush=True)
    print("\n%d uploaded, %d already current, %d FAILED in %.0fs"
          % (tally["ok"], tally["skip"], tally["fail"], time.time() - t0))
    return 1 if tally["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
