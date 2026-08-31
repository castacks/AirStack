#!/usr/bin/env python3
"""stage_frozen_assets.py — put the assets a frozen cell references where the
cell expects them, by pulling them from the Nucleus mirror.

WHY THIS EXISTS
---------------
`freeze.export_scene` flattens with `collect=False`, so a cell's LOOKS are
still references to wherever the asset sat on the build machine — absolute
paths like `/isaac-sim/AirStack/scene_gen/assets/materials/scorched/*.png`.
Measured across all 18 cells (2026-08-30): **332 distinct build-machine
paths**, 272 png / 34 jpg / 26 mdl, in three trees:

    materials/scorched   166      objaverse/<id>/textures   ~100
    materials/megascans   15      aec/{brownstone,tower}     38

Every one of those trees is git-ignored (`.gitignore` 107, 112, 143), so a pod
whose AirStack is a fresh clone has none of them, and Hydra logs hundreds of

    [UsdToMdl] ... 'diffuse_texture': References an asset that can not be
    found: '/isaac-sim/AirStack/scene_gen/assets/materials/scorched/...'

plus, for the vegetation MDLs, the same failure under different tags —
`rtx.neuraylib.plugin  [MDLC:COMPILER] C120 could not find module '::...'`
and `omni.usd  USD_MDL: ... MdlModuleId ... is Invalid`. The visible result is
untextured ground: `ground/materials/{grass,grass_rough,grass_park,dirt}` are
AEC MDLs and they carry the 14 `ground_base` tiles spanning the whole km.

WHY NOT REWRITE THE CELL INSTEAD
--------------------------------
That was tried first and it CANNOT BE DONE with any USD build available here.
The cells were written by Kit and contain a crate value no other reader can
unpack — both `omni.usd.libs`' pxr (Isaac 5.1) and a clean `usd-core` 26.08
raise on `Sdf.Layer.Export`:

    Sdf_CrateFile::CrateFile::_UnpackValue :
    'Attempted to unpack unsupported type enum value 0'

READING is fine (83,215 prims, 14,307 asset attributes, 534 unique paths all
enumerate cleanly) — it is re-serialising that fails, so `UsdUtils.
ModifyAssetPaths` + `Export` would either throw or risk writing a lossy cell
over a good one. Hence: move the ASSETS, do not rewrite the CELL.

MDL CLOSURE. An `.mdl` resolves its textures RELATIVE to itself, so copying
the module alone is not enough — the whole directory containing it comes too
(that is what picks up `materials/textures/` for the vegetation modules and
the `<Name>/` sibling folders under `aec/brownstone/Materials/Base/`). With
that closure the set is 659 files / ~1.3 GB.

DURABILITY. This fixes the pod it runs on. It is lost when the pod is
recycled, so it is a stopgap: the durable fix is the launcher's
`_rebase_local_assets`, which repoints absent paths at the mirror at load
time and travels with the repo.

USAGE (inside a container with omni.client — i.e. run it via Kit's python)
    python3 stage_frozen_assets.py --list transfer_list.txt --dry-run
    python3 stage_frozen_assets.py --list transfer_list.txt
"""
import argparse
import os
import sys

DEFAULT_DEST = "/isaac-sim/AirStack/scene_gen/assets/"
DEFAULT_MIRROR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                  "Projects/SEI-COA/scene_gen/assets/")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", required=True,
                    help="file of paths RELATIVE to the assets root, one per line")
    ap.add_argument("--dest", default=DEFAULT_DEST)
    ap.add_argument("--mirror", default=DEFAULT_MIRROR)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    import omni.client

    rels = [l.strip() for l in open(args.list) if l.strip()]
    dest_root = args.dest.rstrip("/") + "/"
    mirror = args.mirror.rstrip("/") + "/"

    have = need = copied = failed = 0
    missing_on_mirror = []
    for rel in rels:
        dst = dest_root + rel
        if os.path.exists(dst) and os.path.getsize(dst) > 0:
            have += 1
            continue
        need += 1
        src = mirror + rel
        if args.dry_run:
            r, _ = omni.client.stat(src)
            if str(r) != "Result.OK":
                missing_on_mirror.append(rel)
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        r = omni.client.copy(src, dst, omni.client.CopyBehavior.OVERWRITE)
        if str(r) == "Result.OK":
            copied += 1
        else:
            failed += 1
            missing_on_mirror.append(f"{rel}  [{r}]")
        if (copied + failed) % 50 == 0:
            print(f"  ... {copied} copied, {failed} failed", flush=True)

    print(f"\n{len(rels)} in closure: {have} already present, {need} needed")
    if args.dry_run:
        print(f"absent from the mirror too: {len(missing_on_mirror)}")
    else:
        print(f"copied {copied}, failed {failed}")
    for m in missing_on_mirror[:20]:
        print("   MISSING:", m)
    return 1 if (missing_on_mirror and not args.dry_run) else 0


if __name__ == "__main__":
    sys.exit(main())
