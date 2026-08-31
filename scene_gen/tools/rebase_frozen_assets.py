#!/usr/bin/env python3
"""rebase_frozen_assets.py — make a frozen dataset cell PORTABLE.

THE DEFECT
----------
`disaster/freeze.export_scene` flattens with `collect=False` on purpose: every
POSITION is baked, but every LOOK is still a reference to wherever the asset sat
on the build machine. Measured on `fire_suburban_lvl1_1.usd` (2026-08-30): of
534 asset paths, **248 are absolute `/isaac-sim/AirStack/scene_gen/assets/...`**
— 124 `materials` (megascans road/asphalt/pavement, burn, 108 scorch decals),
92 `objaverse` prop textures, 32 `aec` (the GRASS, DIRT and tree-bark MDLs).

On the build box those resolve because the files are sitting there. Anywhere
else they cannot: `.gitignore` excludes `scene_gen/assets/aec/*` (line 112),
`objaverse/*` (107) and `materials/scorched/` (143), so a fresh clone has 1
tracked file under `aec`, 1 under `objaverse` and 380 of 13,164 under
`materials`. On an OSMO pod Hydra then logs, hundreds of times:

    [UsdToMdl] ... parameter 'diffuse_texture': References an asset that can
    not be found: '/isaac-sim/AirStack/scene_gen/assets/materials/scorched/...'

and the ground renders untextured — `ground/materials/{grass,grass_rough,
grass_park,dirt}` are the four AEC MDLs, and they carry the 14 `ground_base`
tiles that span the whole 1 km plate.

THE FIX THIS SCRIPT APPLIES
---------------------------
Rewrite those paths IN THE CELL to the Nucleus mirror at
`Projects/SEI-COA/scene_gen/assets/`, which already holds the same tree (all
248 verified present, after 20 stale scorch decals were uploaded 2026-08-30).
Doing it in the file rather than at load time is what actually makes a cell
portable — a load-time patch has to be repeated by every consumer, and it
lands after Hydra has already tried and failed to resolve the textures.

`UsdUtils.ModifyAssetPaths` is the right tool: it walks EVERY asset-valued
field of a layer — attribute defaults and time samples, references, payloads,
sublayers, value clips, and shader `info:mdl:sourceAsset` — so nothing is
missed the way a hand-rolled prim walk would miss it.

WHAT IS DELIBERATELY LEFT ALONE
-------------------------------
  * `/isaac-sim/kit/mdl/core/Base/OmniPBR.mdl` and bare `OmniPBR.mdl` — Kit
    core modules, present in every Isaac container, resolved from the MDL
    search path rather than from disk.
  * `/Game/...` — Unreal `info:unreal:sourceAsset` breadcrumbs. They were never
    loadable files and rewriting them would invent a path that does not exist.
  * `omniverse://.../Library/Stages/...` — Muyang / RetroNeighborhood textures
    that already resolve on Nucleus.

USAGE (inside a container that has pxr; omni.client only needed for --upload)
    python3 rebase_frozen_assets.py --root /isaac-sim/final_disaster_dataset --dry-run
    python3 rebase_frozen_assets.py --root /isaac-sim/final_disaster_dataset \\
        --cell Fire/Suburban/level_1/1/fire_suburban_lvl1_1.usd --write --upload

`--write` rewrites the LOCAL file in place (via a temp file + atomic replace).
`--upload` then pushes it to `--nucleus`. Without `--upload` nothing leaves the
machine, so a dry run and a local-only pass are both safe.
"""
import argparse
import os
import shutil
import sys
import time

from pxr import Sdf, UsdUtils

DEFAULT_LOCAL_PREFIX = "/isaac-sim/AirStack/scene_gen/assets/"
DEFAULT_MIRROR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                  "Projects/SEI-COA/scene_gen/assets/")
DEFAULT_NUCLEUS = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                   "Projects/SEI-COA/final_disaster_dataset")


def cells_under(root):
    out = []
    for dirpath, _, files in os.walk(root):
        for f in sorted(files):
            if f.endswith(".usd"):
                out.append(os.path.join(dirpath, f))
    return sorted(out)


def rebase_layer(path, local_prefix, mirror, write=False):
    """Open *path*, rewrite every asset path under *local_prefix*.

    Returns (n_changed, n_total_asset_paths, sample_changes). With
    ``write=False`` the layer is only inspected — `ModifyAssetPaths` mutates
    the in-memory layer, and without an Export/Save nothing reaches disk.
    """
    layer = Sdf.Layer.FindOrOpen(path)
    if layer is None:
        raise RuntimeError("could not open " + path)

    stats = {"total": 0, "changed": 0}
    samples = []

    def modifier(asset_path):
        stats["total"] += 1
        if not asset_path.startswith(local_prefix):
            return asset_path
        new = mirror + asset_path[len(local_prefix):]
        stats["changed"] += 1
        if len(samples) < 3:
            samples.append((asset_path, new))
        return new

    UsdUtils.ModifyAssetPaths(layer, modifier)

    if write and stats["changed"]:
        tmp = path + ".rebase.tmp"
        if not layer.Export(tmp):
            raise RuntimeError("Export failed for " + path)
        # Atomic within the filesystem, so a crash cannot leave a half-written
        # cell where a good one used to be.
        os.replace(tmp, path)
    # Drop the layer so the next cell does not accumulate 240 MB in the cache.
    del layer
    return stats["changed"], stats["total"], samples


def verify(path, local_prefix):
    """Reopen and count anything still pointing at the build machine."""
    layer = Sdf.Layer.FindOrOpen(path)
    left = {"n": 0}
    firsts = []

    def check(ap):
        if ap.startswith(local_prefix):
            left["n"] += 1
            if len(firsts) < 3:
                firsts.append(ap)
        return ap

    UsdUtils.ModifyAssetPaths(layer, check)
    del layer
    return left["n"], firsts


def upload(local_path, root, nucleus):
    import omni.client
    rel = os.path.relpath(local_path, root).replace(os.sep, "/")
    dst = nucleus.rstrip("/") + "/" + rel
    r = omni.client.copy(local_path, dst, omni.client.CopyBehavior.OVERWRITE)
    return str(r), dst


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="/isaac-sim/final_disaster_dataset")
    ap.add_argument("--cell", action="append", default=[],
                    help="relative cell path; repeatable. Default: every .usd")
    ap.add_argument("--local-prefix", default=DEFAULT_LOCAL_PREFIX)
    ap.add_argument("--mirror", default=DEFAULT_MIRROR)
    ap.add_argument("--nucleus", default=DEFAULT_NUCLEUS)
    ap.add_argument("--write", action="store_true", help="rewrite the local file")
    ap.add_argument("--upload", action="store_true", help="push to --nucleus (implies --write)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.upload:
        args.write = True
    if args.dry_run:
        args.write = args.upload = False

    paths = ([os.path.join(args.root, c) for c in args.cell] if args.cell
             else cells_under(args.root))
    if not paths:
        print("no cells found under " + args.root)
        return 1

    print(f"{len(paths)} cell(s); prefix {args.local_prefix!r} -> {args.mirror!r}")
    print(f"mode: {'DRY RUN' if not args.write else 'WRITE'}"
          f"{' + UPLOAD' if args.upload else ''}\n")

    tot_changed = 0
    for p in paths:
        t0 = time.time()
        rel = os.path.relpath(p, args.root)
        changed, total, samples = rebase_layer(
            p, args.local_prefix, args.mirror, write=args.write)
        tot_changed += changed
        msg = (f"{rel:<52} {changed:>4}/{total:<5} rewritten"
               f"  {time.time() - t0:5.1f}s")
        if args.write and changed:
            left, firsts = verify(p, args.local_prefix)
            msg += f"  | left={left}"
            if left:
                msg += "  !! " + str(firsts[:1])
        print(msg, flush=True)
        for old, new in samples[:1]:
            print(f"      e.g. {old}\n        -> {new}", flush=True)
        if args.upload and changed:
            res, dst = upload(p, args.root, args.nucleus)
            print(f"      upload: {res} -> {dst}", flush=True)

    print(f"\ntotal asset paths rewritten: {tot_changed}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
