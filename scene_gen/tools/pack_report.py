#!/usr/bin/env python3
"""pack_report.py — reconcile an asset pack's gallery with the pack, then
print what the pack actually contains.

    python3 tools/pack_report.py --pack urban_v2
    python3 tools/pack_report.py --pack urban_v2 --no-render   # tables only

WHAT IT IS FOR
--------------
The pack is edited by hand — entries are added, commented out when they look
bad, re-enabled, re-tagged. Two things drift as that happens:

  * the GALLERY, which keeps a hero render per building. An entry that was
    commented out leaves its render behind (the stitcher globs the folder, so
    the dead asset stays in the grid), and a new entry has none.
  * one's SENSE OF THE PACK — how many towers there are, how many are ruins,
    what they are made of, how much is still on Nucleus.

So this does both in one pass: it diffs the pack against the gallery, renders
what is missing, deletes what is stale, re-stitches, and prints the tables.

THE TWO INTERPRETERS
--------------------
Reading the pack needs `pxr` (AirStack's 3.11 venv); rendering needs `bpy`
(scenegen's 3.13 venv, `$BPY_PYTHON`). They cannot be the same process — see
`ENVIRONMENTS.md` — so the render step is a subprocess, and `--no-render`
skips it entirely.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import scene_generator as sg                                        # noqa: E402

BPY = os.environ.get("BPY_PYTHON", os.path.expanduser("~/coasei/scenegen/.venv/bin/python"))
PACK_DIR = os.path.join(_SCENE_GEN, "config", "asset_packs")


# --------------------------------------------------------------------------- #
# reading
# --------------------------------------------------------------------------- #
def name_of(usd: str) -> str:
    return os.path.splitext(os.path.basename(str(usd)))[0]


def source_of(usd: str) -> str:
    """Where an asset physically lives — what a pack costs to load offline."""
    u = str(usd)
    if u.startswith("objaverse://"):
        return "objaverse"
    if u.startswith("airstack://"):
        return "repo-local"
    if "://" in u:
        return "nucleus"
    return "nucleus"          # a bare relative path is asset_root, i.e. Nucleus


def walk_usds(node, path=()):
    """(dotted category, entry) for every USD entry in a `usds:` tree."""
    if isinstance(node, dict) and "usd" in node:
        yield ".".join(path), node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from walk_usds(v, path + (str(k),))
    elif isinstance(node, (list, tuple)):
        for v in node:
            yield from walk_usds(v, path)
    elif isinstance(node, str):
        # A bare string is an asset only if it looks like a path: pools carry
        # plain USD paths, but `park_features` and `play_groups` also hold a
        # `name:` for the group, which is not an asset.
        if "/" in node or "://" in node or node.lower().endswith((".usd", ".usda", ".usdc", ".usdz")):
            yield ".".join(path), node


def disabled_entries(pack: str) -> Counter:
    """Commented-out entries per category, read from the pack FILE.

    Not visible to the parser by definition, and worth surfacing: an asset
    commented out because it looked bad is a decision someone made, and it
    should not read as an asset that was never there.
    """
    out = Counter()
    path = os.path.join(PACK_DIR, f"{pack}.yaml")
    if not os.path.isfile(path):
        return out
    cat = "?"
    for ln in open(path):
        m = re.match(r"^\s{4,6}(\w+):\s*$", ln)
        if m:
            cat = m.group(1)
        if re.match(r"^\s*#\s*-\s*[{\"']", ln):
            out[cat] += 1
    return out


# --------------------------------------------------------------------------- #
# gallery reconciliation
# --------------------------------------------------------------------------- #
def gallery_state(gallery: str):
    idx_path = os.path.join(gallery, "individual", "index.json")
    idx = json.load(open(idx_path)) if os.path.isfile(idx_path) else {}
    rendered = {}
    root = os.path.join(gallery, "individual")
    for dp, _, fs in os.walk(root):
        for f in fs:
            if f.endswith(".png") and not f.endswith("_views.png"):
                rendered[f[:-4]] = os.path.relpath(os.path.join(dp, f), root)
    return idx, rendered


def drop_render(gallery: str, name: str, rel: str, idx: dict):
    base = os.path.join(gallery, "individual")
    for suffix in ("", "_views"):
        p = os.path.join(base, rel).replace(f"{name}.png", f"{name}{suffix}.png")
        if os.path.exists(p):
            os.remove(p)
    for k in [k for k in idx if k.split("/")[-1] == name]:
        idx.pop(k)


def refresh_gallery(pack, gallery, want, res, samples, verbose=True):
    """Render what the pack has and the gallery does not, drop what it no
    longer has, re-stitch. Returns (rendered, dropped)."""
    idx, have = gallery_state(gallery)
    todo = sorted(set(want) - set(have))
    stale = sorted(set(have) - set(want))
    for n in stale:
        drop_render(gallery, n, have[n], idx)
    if stale:
        json.dump(idx, open(os.path.join(gallery, "individual", "index.json"), "w"),
                  indent=1, sort_keys=True)
        if verbose:
            print(f"[report] dropped {len(stale)} stale render(s): {', '.join(stale)}")
    if not os.path.exists(BPY):
        print(f"[report] no bpy interpreter at {BPY}; set BPY_PYTHON. Skipping render.")
        return [], stale
    gal = os.path.join(_HERE, "building_gallery.py")
    if todo:
        rx = "^(" + "|".join(re.escape(n) for n in todo) + ")$"
        if verbose:
            print(f"[report] rendering {len(todo)} new asset(s): {', '.join(todo)}")
        cmd = [BPY, gal, "--pack", pack, "--out", gallery, "--res", str(res),
               "--samples", str(samples), "--only", rx]
    else:
        if verbose:
            print("[report] nothing new to render; re-stitching")
        cmd = [BPY, gal, "--pack", pack, "--out", gallery, "--stitch"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    for ln in (r.stdout or "").splitlines():
        if ln.startswith("[gallery]") and "MDL-only" not in ln:
            print("  " + ln)
    if r.returncode:
        print("[report] RENDER FAILED:\n" + (r.stderr or "")[-2000:])
    return todo, stale


# --------------------------------------------------------------------------- #
# tables
# --------------------------------------------------------------------------- #
def rule(w): return "-" * w


def table(title, headers, rows, aligns=None):
    print(f"\n{title}")
    cols = len(headers)
    aligns = aligns or ["<"] + [">"] * (cols - 1)
    w = [max(len(str(headers[i])), max((len(str(r[i])) for r in rows), default=0))
         for i in range(cols)]
    line = "  ".join(f"{str(headers[i]):{aligns[i]}{w[i]}}" for i in range(cols))
    print(line)
    print(rule(len(line)))
    for r in rows:
        print("  ".join(f"{str(r[i]):{aligns[i]}{w[i]}}" for i in range(cols)))


def report(pack, cfg, gallery):
    usds = cfg.get("usds") or {}
    bld = sg._building_section(cfg)
    tagged = sg._is_tagged_layout(bld)
    idx, have = gallery_state(gallery)

    def size_of(n):
        v = idx.get(next((k for k in idx if k.split("/")[-1] == n), ""), None)
        if isinstance(v, dict):
            return v.get("size")
        return v

    print(f"\n{'=' * 78}\n  ASSET PACK: {pack}   (layout: "
          f"{'typology pools, condition as a tag' if tagged else 'condition-keyed, legacy'})\n{'=' * 78}")

    # ---- buildings: typology x condition ---------------------------------
    conds = list(sg.BUILDING_CONDITIONS)
    typs = [t for t in bld] if tagged else \
        sorted({t for c in conds for t in ([k for k in (bld.get(c) or {})] if isinstance(bld.get(c), dict) else ["(untyped)"])})
    rows = []
    for t in typs:
        counts = [len(sg.building_entries(cfg, condition=c, typology=t if tagged else None)) for c in conds] \
            if not tagged else [len(sg.building_entries(cfg, condition=c, typology=t)) for c in conds]
        heights = [size_of(name_of(e["usd"] if isinstance(e, dict) else e))
                   for e in sg.building_entries(cfg, typology=t)] if tagged else []
        heights = [h[2] for h in heights if h]
        span = f"{min(heights):.0f}-{max(heights):.0f} m" if heights else "-"
        rows.append([t] + counts + [sum(counts), span])
    tot = [sum(r[i] for r in rows) for i in range(1, 5)]
    rows.append(["TOTAL"] + tot + [""])
    table("BUILDINGS — typology x condition", ["typology"] + conds + ["total", "height span"], rows,
          ["<", ">", ">", ">", ">", ">"])

    # ---- buildings by material -------------------------------------------
    mats = defaultdict(Counter)
    for t in (typs if tagged else ["(all)"]):
        for e in sg.building_entries(cfg, typology=t if tagged else None):
            if isinstance(e, dict):
                mats[str(e.get("material") or "(unset)")][sg.condition_of(e)] += 1
    rows = [[m] + [mats[m][c] for c in conds] + [sum(mats[m].values())]
            for m in sorted(mats, key=lambda m: -sum(mats[m].values()))]
    table("BUILDINGS — construction material (drives the fracture core and which debris it sheds)",
          ["material"] + conds + ["total"], rows)

    # ---- every pool -------------------------------------------------------
    per_cat = Counter()
    per_src = Counter()
    tag_hist = Counter()
    for cat, e in walk_usds(usds):
        per_cat[cat] += 1
        usd = e["usd"] if isinstance(e, dict) else e
        per_src[source_of(usd)] += 1
        if isinstance(e, dict):
            for t in e.get("tags") or ():
                tag_hist[str(t)] += 1
    dis = disabled_entries(pack)
    rows = [[c, n, dis.get(c.split(".")[-1], 0) or ""] for c, n in sorted(per_cat.items())]
    table("EVERY POOL — entries per category (`disabled` = commented out in the pack file)",
          ["usds category", "entries", "disabled"], rows)

    table("TAGS — every tag in use, across all pools",
          ["tag", "entries"], [[t, n] for t, n in tag_hist.most_common()])

    table("WHERE THE ASSETS LIVE",
          ["source", "entries"], [[s, n] for s, n in per_src.most_common()])

    # ---- gallery coverage -------------------------------------------------
    want = {name_of(e["usd"] if isinstance(e, dict) else e)
            for e in sg.building_entries(cfg)}
    rows = [["buildings in pack", len(want)],
            ["hero renders on disk", len(have)],
            ["missing a render", len(want - set(have))],
            ["render with no entry", len(set(have) - want)],
            ["commented out in the pack file", sum(dis.values())]]
    table("GALLERY", ["", "count"], rows)
    miss = sorted(want - set(have))
    if miss:
        print("  missing:", ", ".join(miss))
    print()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pack", default="urban_v2")
    ap.add_argument("--gallery", default="")
    ap.add_argument("--no-render", action="store_true",
                    help="report only; do not touch the gallery")
    ap.add_argument("--res", type=int, default=2048)
    ap.add_argument("--samples", type=int, default=48)
    args = ap.parse_args()
    gallery = args.gallery or os.path.join(_SCENE_GEN, "galleries", args.pack)
    cfg = sg.resolve_asset_pack({"asset_pack": args.pack})
    if not args.no_render:
        want = {name_of(e["usd"] if isinstance(e, dict) else e)
                for e in sg.building_entries(cfg)}
        refresh_gallery(args.pack, gallery, want, args.res, args.samples)
    report(args.pack, cfg, gallery)
    return 0


if __name__ == "__main__":
    sys.exit(main())
