#!/usr/bin/env python3
"""tag_assets.py — tag indexed assets from their names, against a fixed vocabulary.

    python3 scene_gen/tools/tag_assets.py \
        --index scene_gen/_plans/sei_coa_usable.tsv \
        --extents scene_gen/_plans/sei_coa_extents.tsv \
        --out scene_gen/_plans/sei_coa_tagged.tsv \
        --unsure scene_gen/_plans/sei_coa_unsure.tsv

Every tag comes from `config/asset_tags.yaml` — this file defines none of its
own, so the vocabulary can be argued with in one place and the run repeated.

NAME ONLY. Nothing here opens a model. That is a real limit and the reason for
`--unsure`: an asset whose name yields no `class` tag has told us nothing about
what it is, and no amount of pattern-writing fixes `SM_Plain_Longwall_1` or
`houses_02_0772`. Those are listed for a person (or a render pass) to look at,
rather than being given a guessed tag that would then be trusted.

EXTENTS ARE JOINED, NOT MEASURED, here — `nucleus_extents.py` does that, over
the network, and it is slow enough to want to be a separate run.
"""

import argparse
import os
import re
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
VOCAB = os.path.join(os.path.dirname(_HERE), "config", "asset_tags.yaml")


def load_vocab(path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    tags = {}
    for name, spec in (doc.get("tags") or {}).items():
        pats = [re.compile(p) for p in (spec.get("patterns") or [])]
        tags[name] = (spec.get("group", "class"), pats,
                      set(spec.get("suppresses") or []))
    return doc.get("groups", []), tags


def context_of(path, name):
    """The text a tag is matched against: the file name plus its INTERMEDIATE
    directories, with the top-level pack dropped.

    The pack name is the one part of the path that says nothing about an
    individual asset — every file under `Old_Shipyard/` is not a vessel. Every
    directory BELOW it is genuine curation by whoever built the pack:
    `.../Vegetation/Hawthorn/Hawthorn.usd` is a tree, and no list of species
    names would have caught `Forsythia`, `Kousa_Dogwood` and `Prairie_Dropseed`
    the way one `Vegetation` folder does. `Vessel_Pack/`, `Meshes/Debris/` and
    `MergedBuildings/` all earn their tags the same way.
    """
    segs = path.split("/")
    return " ".join(segs[1:-1] + [str(name)])


def normalise(name):
    """Lowercase, separators to spaces — so `\\b` in a pattern means what a
    reader thinks it means against `SM_Fire_Hydrant_01.prop.usd`.

    THE NAME, NOT THE PATH. Including the path tagged every one of the 692
    files under `Old_Shipyard/` as `vessel`, and all 167 under `Trainyard/` as
    `rail`, because the pack name sits in the path of every file in it. The
    pack is real information and is kept as its own column — a fact about where
    the asset lives, not a claim about what it is.
    """
    s = str(name).lower()
    s = re.sub(r"[_\-/.]+", " ", s)
    s = re.sub(r"\s+", " ", s)
    return s


def tag_one(text, tags):
    """Matched tags, with broader ones removed where a specific one won.

    See `suppresses` in the vocabulary: a staircase inside a building matches
    both `building_part` and `building`, and only the first is true of it.
    """
    hit = {name: (group, sup) for name, (group, pats, sup) in tags.items()
           if any(p.search(text) for p in pats)}
    beaten = set()
    for _name, (_g, sup) in hit.items():
        beaten |= sup
    return [(g, n) for n, (g, _s) in hit.items() if n not in beaten]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--index", required=True)
    ap.add_argument("--extents", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--unsure", required=True)
    ap.add_argument("--vocab", default=VOCAB)
    args = ap.parse_args()

    groups, tags = load_vocab(args.vocab)
    ext_by_path = {}
    if args.extents and os.path.isfile(args.extents):
        with open(args.extents) as f:
            next(f)
            for line in f:
                c = line.rstrip("\n").split("\t")
                if len(c) >= 5 and c[1]:
                    ext_by_path[c[0]] = (c[1], c[2], c[3], c[4])

    rows, unsure = [], []
    with open(args.index) as f:
        header = f.readline().rstrip("\n").split("\t")
        for line in f:
            c = line.rstrip("\n").split("\t")
            path, name, ext, size = c[0], c[1], c[2], c[3]
            found = tag_one(normalise(context_of(path, name)), tags)
            by_group = {g: [] for g in groups}
            for g, t in found:
                by_group.setdefault(g, []).append(t)
            for g in by_group:
                by_group[g].sort()
            all_tags = sorted(t for _g, t in found)
            sx, sy, sz, method = ext_by_path.get(path, ("", "", "", ""))
            pack = path.split("/")[0]
            rows.append((path, name, ext, size, pack, ";".join(all_tags),
                         ";".join(by_group.get("class", [])), sx, sy, sz, method))
            # UNSURE = nothing was learned about WHAT IT IS. A material or a
            # payload with no class is expected and not interesting; a mesh
            # with no class is the thing a person has to look at.
            kinds = by_group.get("kind", [])
            if not by_group.get("class") and "material" not in kinds:
                unsure.append((path, name, ext, size, ";".join(all_tags) or "-"))

    with open(args.out, "w") as f:
        f.write("path\tname\text\tsize_bytes\tpack\ttags\tclass_tags"
                "\tsize_x_m\tsize_y_m\tsize_z_m\textent_method\n")
        for r in rows:
            f.write("\t".join(r) + "\n")
    with open(args.unsure, "w") as f:
        f.write("path\tname\text\tsize_bytes\ttags_so_far\n")
        for r in sorted(unsure, key=lambda r: -int(r[3])):
            f.write("\t".join(r) + "\n")

    n_ext = sum(1 for r in rows if r[7])
    print(f"[tag] {len(rows):,} assets tagged, {len(rows) - len(unsure):,} with a "
          f"class ({100.0 * (len(rows) - len(unsure)) / max(len(rows), 1):.1f}%), "
          f"{len(unsure):,} unsure -> {args.unsure}")
    print(f"[tag] {n_ext:,} carry measured extents -> {args.out}")


if __name__ == "__main__":
    main()
