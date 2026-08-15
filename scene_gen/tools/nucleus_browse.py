#!/usr/bin/env python
"""
nucleus_browse.py — recursive listing of Nucleus directories.

Must run inside the isaac-sim container: `pxr`/`omni.client` only import after
SimulationApp has started, so this boots a headless app first (~80 s) and then
walks the trees named in ROOTS. Writes to OUT because container stdout is
swallowed by the Kit logger.

    docker cp nucleus_browse.py isaac-sim:/tmp/s.py
    docker exec isaac-sim bash -c 'cd /isaac-sim && \
        PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ./python.sh /tmp/s.py >/dev/null 2>&1; \
        cat /tmp/nucleus_browse.txt'

NOTE (superseded for browsing): `tools/nucleus.py` reads Nucleus from the HOST
in under a second — `omni.client` never needed Kit, only `libcarb.so` on the
loader path. Use this only when you need Kit's USD resolver (i.e. to OPEN a
stage); for list / stat / read, prefer `nucleus.py`.
"""

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import os

import carb
import omni.client

OUT = "/tmp/nucleus_browse.txt"
LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"

ROOTS = [
    LIB,
    LIB + "Dmytro/Assets/Game/Downtown_West/",
    LIB + "Muyang/DownTown/",
    LIB + "CityPark/",
    LIB + "RetroNeighborhood/",
]

MAX_DEPTH = int(os.environ.get("BROWSE_DEPTH", "3"))
# Texture/material trees are enormous and hold no placeable geometry.
SKIP = ("textures", "texture", "materials", "material", "maps", ".thumbs",
        "sourceimages", "images")


def walk(url, depth, out, seen):
    if depth > MAX_DEPTH or url in seen:
        return
    seen.add(url)
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        out.write(f"!! {url}  -> {res}\n")
        return
    dirs = []
    for e in entries:
        name = e.relative_path
        is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
        if is_dir:
            dirs.append(name)
            out.write(f"D {url}{name}/\n")
        else:
            out.write(f"F {url}{name}\t{getattr(e, 'size', 0)}\n")
    for name in dirs:
        if name.lower() in SKIP:
            continue
        walk(f"{url}{name}/", depth + 1, out, seen)


def main():
    root = os.environ.get("OMNI_SERVER", "").strip().strip('"').rstrip("/")
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", root)
    with open(OUT, "w") as out:
        for r in ROOTS:
            out.write(f"\n===== {r} (depth {MAX_DEPTH}) =====\n")
            walk(r, 0, out, set())


main()
app.close()
