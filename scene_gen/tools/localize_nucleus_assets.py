#!/usr/bin/env python
"""
localize_nucleus_assets.py — pull Nucleus buildings onto local disk as usdz.

WHY THIS EXISTS
---------------
`tools/damage_gallery.py` can only preview assets that resolve on the HOST:
usd-core has no `omniverse://` resolver, and neither does Blender, so the whole
`urban` / `urban_intact` building library — nine Nucleus assets — is invisible
to the gallery and `--list` reports `0/9`. That is the only reason mesh damage
has never been previewed on the city buildings it does most of its work on.

Only one process in this repo can read `omniverse://`: Isaac Sim's Kit python,
where `omni.client` is registered as a USD asset resolver. So this runs THERE
and copies the assets down.

WHY A MIRROR AND NOT A USDZ
---------------------------
`UsdUtils.CreateNewUsdzPackage` is the obvious tool and it does not work here.
Its dependency walk is fine — it finds every sublayer, reference and texture —
but `UsdZipFileWriter::AddFile` ingests each one by MEMORY-MAPPING it, and you
cannot mmap a URL. Every dependency fails with `Failed to map
'omniverse://...': No such file or directory` and it writes a 22-byte empty
package. Packaging has to happen after the bytes are local, not instead of.

So: `UsdUtils.ComputeAllDependencies` for the file list, `omni.client.read_file`
for the bytes, and each one written to the path it already has RELATIVE TO
`Library/Stages/`. Mirroring the layout rather than flattening into one folder
is what keeps the internal references valid — these are Omniverse "collected"
exports, whose layers point at `./SubUSDs/*.usd` and `./SubUSDs/textures/*.png`
relative to themselves, so a faithful subtree resolves locally with nothing
rewritten.

    docker exec isaac-sim bash -c 'cd /isaac-sim && \
        ./python.sh /isaac-sim/AirStack/scene_gen/tools/localize_nucleus_assets.py'
    docker exec isaac-sim cat /tmp/localize_nucleus.txt

That second line is not optional: Kit swallows stdout, so the report is written
to a file. (`tools/nucleus_browse.py` documents the same trap.)

Output lands in `scene_gen/assets/nucleus/<Nucleus-relative-path>`, gitignored
for the same reason `assets/objaverse/` is: large, and reproducible by
re-running this. Files come out owned by root — `docker exec` runs as root —
so the caller chowns them; `--chown UID:GID` does it here instead.

Reference them with the `airstack://` scheme, exactly as the AEC packs are:

    - {usd: "airstack://scene_gen/assets/nucleus/Muyang/DownTown/Assets/BG_Building_A.usd"}

NOTE (superseded for browsing): `tools/nucleus.py` reads Nucleus from the HOST
in under a second — `omni.client` never needed Kit, only `libcarb.so` on the
loader path. Use this only when you need Kit's USD resolver (i.e. to OPEN a
stage); for list / stat / read, prefer `nucleus.py`.
"""

import argparse
import os
import sys
import traceback

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import omni.client                                              # noqa: E402
from pxr import Usd, UsdUtils                                   # noqa: E402

LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"

OUT_ROOT = "/isaac-sim/AirStack/scene_gen/assets/nucleus"

REPORT = "/tmp/localize_nucleus.txt"

#: Exactly `urban.yaml`'s `usds.buildings.intact`, which `urban_intact`
#: inherits unchanged — it only empties the ruin pools. Kept as a literal list
#: rather than parsed out of the YAML because this file runs inside the
#: container, where importing `scene_generator` would drag the whole config
#: system in for nine strings.
ASSETS = [
    "Muyang/ModernCityEnvironment/Collected_Building01/SM_MERGED_BP_MBuilding01.usd",
    "Muyang/ModernCityEnvironment/Collected_Building02/SM_MERGED_BP_MBuilding02.usd",
    "Muyang/ModernCityEnvironment/Collected_Building05/SM_MERGED_BP_MBuilding05.usd",
    "Muyang/DownTown/Assets/BG_Building_A.usd",
    "Muyang/DownTown/Assets/BG_Building_B.usd",
    "Muyang/DownTown/Assets/BG_Building_C.usd",
    "Muyang/DownTown/Assets/BG_Building_D.usd",
    "Muyang/DownTown/Assets/BG_Building_E.usd",
    "Muyang/DownTown/Assets/BG_Building_F.usd",
]

#: Whole directories to mirror, for pools rather than single assets. A gallery
#: cell is a building plus the debris the disaster stage drops around it, and
#: without these the wrecked building stands on clean ground — which is the one
#: thing `GENERATION.md` singles out as the most obviously wrong thing in an
#: aerial view. Everything else `urban` names on Nucleus (people, vehicles,
#: street furniture, road decals) is placed by passes a gallery never runs, so
#: mirroring it would be 100 MB for nothing.
PACKS = [
    "Muyang/DebrisPack/Assets",
    "Muyang/DestroyedBuildings/Assets",
]


def _fetch(url: str, stats: dict) -> bool:
    """Copy one Nucleus file to its mirrored local path. True if it landed.

    Anything outside `Library/Stages/` is refused rather than guessed at: the
    mirror's whole correctness argument is that a file keeps its path relative
    to that root, and a dependency from somewhere else has no such path.
    """
    if not url.startswith(LIB):
        stats.setdefault("foreign", []).append(url)
        return False
    dst = os.path.join(OUT_ROOT, url[len(LIB):])
    if os.path.exists(dst) and os.path.getsize(dst) > 0:
        stats["cached"] = stats.get("cached", 0) + 1
        return True

    # `read_file` returns (result, version, content) on this build and has
    # returned (result, content) on others — take the payload off the end
    # rather than pinning a shape that varies with the Kit release.
    out = omni.client.read_file(url)
    res, content = out[0], out[-1]
    if res != omni.client.Result.OK:
        stats.setdefault("failed", []).append(f"{res} {url}")
        return False
    buf = memoryview(content)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "wb") as fh:
        fh.write(buf)
    stats["copied"] = stats.get("copied", 0) + 1
    stats["bytes"] = stats.get("bytes", 0) + len(buf)
    return True


def _walk(url: str, stats: dict, depth: int = 0) -> None:
    """Mirror a Nucleus directory recursively."""
    if depth > 8:
        return
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        stats.setdefault("failed", []).append(f"{res} (list) {url}")
        return
    for e in entries:
        child = url.rstrip("/") + "/" + e.relative_path
        is_dir = bool(int(e.flags) & int(omni.client.ItemFlags.CAN_HAVE_CHILDREN))
        if is_dir:
            _walk(child, stats, depth + 1)
        else:
            _fetch(child, stats)


def _dependencies(src: str, stats: dict) -> set:
    """Every layer and asset *src* composes, or an empty set if USD refuses.

    `ComputeAllDependencies` is the precise answer and it is not always
    available: two of the nine buildings are crate files this USD build cannot
    fully unpack (`Attempted to unpack unsupported type enum value 0`) and it
    raises rather than degrading. That is a reason to have a second route, not
    a reason to stop — the containing directory is mirrored regardless, and on
    a "Collected_*" export that already holds every dependency.
    """
    try:
        layers, assets, _unresolved = UsdUtils.ComputeAllDependencies(src)
    except Exception as exc:
        stats["dep_scan"] = f"unavailable ({type(exc).__name__})"
        return set()
    out = {str(lyr.identifier) for lyr in layers}
    out.update(str(a) for a in assets)
    stats["dep_scan"] = f"{len(out)} deps"
    return out


def localize(rel: str) -> str:
    """Mirror one building and everything it composes. Returns a report line."""
    src = LIB + rel
    name = os.path.splitext(os.path.basename(rel))[0]

    res, _ = omni.client.stat(src)
    if res != omni.client.Result.OK:
        return f"MISS   {name}  ({res}) {src}"

    stats: dict = {}
    # Two routes, unioned. The directory sweep is what actually carries these
    # assets — an Omniverse "collected" export is self-contained inside its own
    # folder, textures included — and the dependency scan catches anything
    # referenced from outside it.
    _walk(LIB + os.path.dirname(rel), stats)
    for url in sorted(_dependencies(src, stats) | {src}):
        _fetch(url, stats)

    dst = os.path.join(OUT_ROOT, rel)
    if not os.path.exists(dst):
        return f"FAIL   {name}  root layer never landed"

    # Open the MIRROR, not the source: the point of the exercise is a tree the
    # host can read, and only re-opening it locally proves the internal
    # references were relative. `omni.client` is still registered in this
    # process, so an absolute `omniverse://` reference would resolve here and
    # fail on the host — hence the explicit scan for surviving ones rather
    # than trusting that the stage opened.
    stage = Usd.Stage.Open(dst)
    n_mesh = sum(1 for p in stage.Traverse() if p.GetTypeName() == "Mesh")
    n_pts = 0
    for p in stage.Traverse():
        if p.GetTypeName() == "Mesh":
            pts = p.GetAttribute("points").Get()
            n_pts += len(pts) if pts else 0
    remote = sum(1 for lyr in stage.GetUsedLayers()
                 if "omniverse://" in str(lyr.identifier))

    bits = [f"{stats.get('copied', 0)} new", f"{stats.get('cached', 0)} cached",
            str(stats.get("dep_scan", "?"))]
    if stats.get("failed"):
        bits.append(f"{len(stats['failed'])} FAILED")
    if stats.get("foreign"):
        bits.append(f"{len(stats['foreign'])} off-root")
    if remote:
        bits.append(f"!! {remote} layers still remote")
    return (f"ok     {name:34s} {stats.get('bytes', 0) / 1e6:7.1f} MB  "
            f"{n_mesh:4d} meshes {n_pts / 1000:7.1f}k pts   "
            + ", ".join(bits))


def main(argv) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--chown", metavar="UID:GID",
                    help="chown the mirror after writing; docker exec runs as "
                         "root and the host user cannot otherwise delete it")
    args = ap.parse_args(argv)

    os.makedirs(OUT_ROOT, exist_ok=True)
    lines = []
    for rel in ASSETS:
        try:
            line = localize(rel)
        except Exception:
            line = f"ERROR  {os.path.basename(rel)}\n" + traceback.format_exc()
        lines.append(line)
        print(line, flush=True)

    for pack in PACKS:
        stats: dict = {}
        try:
            _walk(LIB + pack, stats)
            line = (f"pack   {pack:34s} {stats.get('bytes', 0) / 1e6:7.1f} MB  "
                    f"{stats.get('copied', 0)} new, "
                    f"{stats.get('cached', 0)} cached"
                    + (f", {len(stats['failed'])} FAILED"
                       if stats.get("failed") else ""))
        except Exception:
            line = f"ERROR  {pack}\n" + traceback.format_exc()
        lines.append(line)
        print(line, flush=True)

    if args.chown:
        uid, _, gid = args.chown.partition(":")
        for root, dirs, files in os.walk(OUT_ROOT):
            for n in dirs + files:
                os.chown(os.path.join(root, n), int(uid), int(gid or uid))
        os.chown(OUT_ROOT, int(uid), int(gid or uid))
        lines.append(f"chowned {OUT_ROOT} to {args.chown}")

    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return 0


try:
    rc = main(sys.argv[1:])
finally:
    app.close()
sys.exit(rc)
