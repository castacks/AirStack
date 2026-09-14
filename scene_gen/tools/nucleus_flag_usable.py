#!/usr/bin/env python3
"""nucleus_flag_usable.py — add a `usable` flag to a `nucleus_index.py` table.

    python3 scene_gen/tools/nucleus_flag_usable.py \
        --index scene_gen/_plans/sei_coa_index.tsv \
        --out   scene_gen/_plans/sei_coa_index_flagged.tsv

NOTHING IS REMOVED. Two columns are appended — `usable` (1/0) and `reason`, the
rule that decided it — so a judgement can be argued with, and re-run with a
different rule set, without re-walking 365k files over the network.

WHAT `usable` MEANS: *this is a thing you would name in an asset set or open in
a viewport when building a new scene.* It is not a copy manifest. Textures are
flagged 0 because you do not place a texture — but a USD that survives the
filter still NEEDS its textures, so mirroring the usable set alone would give
you untextured geometry. Take the dependency closure, not this column.

THE RULES, coarse on purpose:

  excluded trees   URBAN_V3_BAKE, final_disaster_dataset and
                   ...payloads — output of previous runs, not input to new
                   ones, and 90% of the store by size.
  caches           any path segment `cache`/`*_cache`, plus `.thumbs`
                   (Nucleus's own UI thumbnails, 21k files).
  scratch          `_edit_session*` — Kit's in-progress edit state, including
                   the single largest file in the project at 4.2 GB.
  dependencies     png jpg jpeg dds exr tga: maps a material references, not
                   assets in their own right.
  baked damage     `archetype*` and `<pack>_quake` — one USD per source asset
                   x damage grade x seed, regenerable from art that is itself
                   in this index. 540 files, 2.99 GB.
  wrong pipeline   uasset umap: Unreal source, unreadable by anything here.
  not geometry     zip bin parquet json jsonl md yaml csv py lock bak.

  KEPT             usd usda usdc usdz (geometry and stages), mdl (materials),
                   fbx gltf (convertible source), hdr (environment maps — a
                   sky is a scene input even though it is an image).
"""

import argparse
import os

ASSET_EXT = {"usd", "usda", "usdc", "usdz", "mdl", "fbx", "gltf", "hdr"}
TEXTURE_EXT = {"png", "jpg", "jpeg", "dds", "exr", "tga", "tif", "tiff"}
UNREAL_EXT = {"uasset", "umap"}
BLOB_EXT = {"zip", "bin", "parquet", "json", "jsonl", "md", "yaml", "yml",
            "csv", "py", "lock", "bak", "txt", "(none)"}
EXCLUDED_TREES = ("URBAN_V3_BAKE", "final_disaster_dataset",
                  "final_disaster_dataset_payloads")


def classify(path, name, ext):
    """``(usable, reason)`` for one row."""
    segs = path.split("/")
    if segs and segs[0] in EXCLUDED_TREES:
        return 0, "excluded-tree:" + segs[0]
    low = [s.lower() for s in segs[:-1]]
    if any(s == "cache" or s.endswith("_cache") or s == "caches" for s in low):
        return 0, "cache"
    if ".thumbs" in low:
        return 0, "thumbnail"
    if name.lower().startswith("_edit_session"):
        return 0, "kit-scratch"
    # BAKED DAMAGE STATES, not art. `archetype*` and `<pack>_quake` hold one
    # USD per (source asset x damage grade x seed) — `bld_block_residential_DG5`,
    # `gac_SM_Building_17_DG3_s596`, `tree_American_Beech_fallen`. They are
    # OUTPUT of the disaster stage over art that is itself in this index, they
    # are regenerable, and two of the six folders (`archetype`, `archetype_r15`)
    # are revisions of the same ladder. Building a NEW scene starts from the
    # source asset, not from somebody's earlier bake of it.
    if any(s.startswith("archetype") or s.endswith("_quake") for s in low):
        return 0, "baked-damage"
    if ext in TEXTURE_EXT:
        return 0, "texture-dependency"
    if ext in UNREAL_EXT:
        return 0, "unreal-source"
    if ext in BLOB_EXT:
        return 0, "not-geometry"
    if ext in ASSET_EXT:
        return 1, "asset"
    return 0, "unknown-ext:" + ext


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--index", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    n_in = n_usable = 0
    size_usable = size_total = 0
    with open(args.index) as fin, open(args.out, "w") as fout:
        header = fin.readline().rstrip("\n")
        fout.write(header + "\tusable\treason\n")
        for line in fin:
            path, name, ext, size = line.rstrip("\n").split("\t")
            u, why = classify(path, name, ext)
            fout.write(f"{path}\t{name}\t{ext}\t{size}\t{u}\t{why}\n")
            n_in += 1
            size_total += int(size)
            if u:
                n_usable += 1
                size_usable += int(size)
    print(f"[flag] {n_usable:,} usable of {n_in:,} files "
          f"({100.0 * n_usable / max(n_in, 1):.1f}%), "
          f"{size_usable / 1e9:.2f} of {size_total / 1e9:.2f} GB "
          f"-> {args.out}")


if __name__ == "__main__":
    main()
