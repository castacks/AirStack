#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "pillow"]
# ///
"""Photograph a Stage A archetype library: one row per asset, one column per rung.

    ┌───────────┬──────────┬─────────┬─────────────┬──────────────────┬──────────┐
    │           │ pristine │ cracked │ soft_storey │ partial_collapse │ pancaked │
    ├───────────┼──────────┼─────────┼─────────────┼──────────────────┼──────────┤
    │ house_01  │          │         │             │                  │          │
    │ block_04  │          │         │             │                  │          │
    └───────────┴──────────┴─────────┴─────────────┴──────────────────┴──────────┘

The question a damage ladder has to answer is not "does severity 0.9 look bad"
but "do the rungs read as different KINDS of damage, in order" — which is only
visible with the rungs of one asset side by side, at one camera, in one row.
`tools/quake_preview.py` says the same thing about the pipeline; this says it
about the library that pipeline actually shipped.

    scenegen/.venv/bin/python scene_gen/tools/render_archetypes.py \\
        scene_gen/assets/archetypes_urban_v2/earthquake

ROLLING BY DEFAULT. Only tiles with no PNG are rendered, so this can be run
against a library a bake is still filling and re-run as more lands, at the cost
of one Cycles render (~2 s) per new archetype. `--force` re-renders everything.

ONE CAMERA PER ROW, FRAMED ON THE PRISTINE CELL
-----------------------------------------------
Taken from `render_damage_gallery.py`, whose engine, lighting, ground, camera
and sheet compositor this reuses wholesale rather than copying. Framing each
cell on its own bounds makes the sheet actively misleading: debris and thrown
fragments widen the scene, the camera pulls back to fit them, the building
shrinks, and a MORE damaged cell renders a SMALLER building — which the eye
reads as scale rather than as damage.

TEXTURES COME FROM THE SIDECAR
------------------------------
An archetype references its textures where Stage A found them, which for this
pack is `omniverse://`. Kit resolves that; Blender does not, and every tile
comes out untextured grey — the one state a look check cannot be run in. The
fix is a level down, in the USD: `tools/localize_archetype_textures.py` writes
`<name>.local.usda` beside each archetype, and this renders that when it is
there. Blender cannot be fixed from its own side, because it creates no image
datablock at all for a path it failed to open.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import bpy                                                      # noqa: E402
import render_damage_gallery as G                               # noqa: E402

#: Rungs, in ladder order, for the sheet's columns. Hardcoded because the bpy
#: env has no `pxr` and so cannot import `disaster.levels`, which is the
#: authority; a rung the ladder grows and this misses still renders, it just
#: sorts after the ones named here.
EARTHQUAKE_RUNGS = ("pristine", "cracked", "soft_storey", "partial_collapse",
                    "pancaked")

#: Rungs that only the VEGETATION ladder has. `pristine` is deliberately not
#: here — both ladders start there, so it cannot tell the two apart, and it is
#: the presence of one of these that identifies a row as a tree.
VEGETATION_RUNGS = ("fallen", "stump")


def render_cell(usd: Path, out: Path, res: int, frame, az: float, el: float):
    """`render_damage_gallery.render_cell` without the soot step.

    Textures are NOT fixed here. Blender creates no image datablock at all for
    a path it could not open, so there is nothing on the bpy side to repoint —
    which is why the fix has to happen one level down, in the USD, before the
    importer ever sees it. See `tools/localize_archetype_textures.py`.
    """
    G.import_usd(usd)
    measured = G.geometry_bounds()
    center, radius = frame or (measured[0], measured[1] * G.FRAME_PAD)
    G.add_ground(center, radius)
    G.add_lighting()
    G.place_camera(center, radius, az, el)
    G.render_to(out, res)
    return center, radius


# --------------------------------------------------------------------------- #
# what the library says about itself
# --------------------------------------------------------------------------- #
def load_library(root: Path) -> tuple:
    """``(rows, rungs)`` from a library's manifest, newest-baked first.

    The TRACE is preferred over the manifest for the per-cell numbers: it
    carries the timings and the cells the bake rejected, and the manifest only
    ever describes what exported.
    """
    manifest = json.loads((root / "manifest.json").read_text())
    recs = manifest.get("archetypes", manifest)
    trace = {}
    tpath = root / "bake_trace.jsonl"
    if tpath.exists():
        for line in tpath.read_text().splitlines():
            try:
                r = json.loads(line)
            except ValueError:
                continue
            trace[(r["type"], r["level"])] = r

    rungs = [lv for lv in EARTHQUAKE_RUNGS
             if any(r["level"] == lv for r in recs)]
    rungs += sorted({r["level"] for r in recs} - set(rungs))

    by_type: dict = {}
    for r in recs:
        by_type.setdefault(r["type"], {})[r["level"]] = r
    rows = []
    for name in sorted(by_type):
        cells = by_type[name]
        rows.append({
            "name": name,
            "cells": {lv: cells[lv]["usd"] for lv in cells},
            "stats": {lv: _note(cells[lv], trace.get((name, lv)))
                      for lv in cells},
        })
    return rows, rungs


def _note(rec: dict, tr: dict = None) -> dict:
    """The caption under a tile: what this rung cost and what it produced."""
    tr = tr or {}
    secs = rec.get("seconds", tr.get("seconds"))
    mb = rec.get("usd_mb")
    if rec.get("level") == "pristine":
        # `footprint_m`/`height_m`, not `note`: `G.compose` writes the pristine
        # column's caption itself out of those two keys and ignores a `note`
        # there, so this is the shape that reaches the sheet.
        return {"footprint_m": [tr.get("src_x_m", "?"), tr.get("src_y_m", "?")],
                "height_m": tr.get("src_z_m", "?")}
    note = (f"{tr.get('frag_loose', 0)}/{tr.get('frag_cells', 0)} frag · "
            f"{rec.get('meshes', 0)} mesh")
    if secs is not None:
        note += f" · {float(secs):.0f}s"
    if mb is not None:
        note += f" · {mb:.0f}MB"
    return {"note": note}


def compose_pages(rows, rungs, tiles, out_dir: Path, title: str,
                  per_page: int) -> list:
    """Sheets of *per_page* rows, ONE LADDER PER SHEET.

    A tree and a building do not fail the same way, so `disaster.levels` keys
    the ladder on KIND and their rung names do not overlap. Putting both on one
    sheet gives every building two permanently blank columns (`fallen`,
    `stump`) and every tree five — half the sheet spent proving that a stump
    cannot pancake. So rows are grouped by the ladder they actually have.

    Paged as well, because one 89-row sheet is 30 000 px of unopenable PNG.
    """
    # GROUPED ON THE LADDER, not on the rungs this asset happens to have
    # baked. Keying on what is present splits a half-finished asset onto its
    # own sheet — `pristine_cracked`, `pristine_soft_storey` — and the blanks
    # are the point: a column missing inside its own ladder says the bake did
    # not reach that rung, which is exactly what a reader wants to see.
    # `pristine` heads BOTH ladders — it is the undamaged reference every row
    # is framed on, and a tree sheet without it has nothing to compare against.
    veg = tuple(lv for lv in rungs
                if lv == "pristine" or lv in VEGETATION_RUNGS)
    struct = tuple(lv for lv in rungs if lv not in VEGETATION_RUNGS)
    kinds: dict = {}
    for i, row in enumerate(rows):
        got = {lv for lv in rungs if (i, lv) in tiles}
        if not got:
            continue
        key = veg if got & set(VEGETATION_RUNGS) else struct
        if key:
            kinds.setdefault(key, []).append((i, row))

    made = []
    for key in sorted(kinds, key=lambda k: -len(kinds[k])):
        group = kinds[key]
        for start in range(0, len(group), per_page):
            chunk = group[start:start + per_page]
            sub = {(r, col): tiles[(i, col)]
                   for r, (i, _row) in enumerate(chunk) for col in key
                   if (i, col) in tiles}
            if not sub:
                continue
            man = {"columns": list(key), "rows": [rw for _i, rw in chunk],
                   "title": title,
                   "subtitle": f"assets {start + 1}-{start + len(chunk)} of "
                               f"{len(group)} on the {key[0]}\u2192{key[-1]} "
                               f"ladder · caption: loose/cut fragments · "
                               f"meshes · bake seconds · archetype size"}
            stem = "_".join(key[:1] + key[-1:])
            dst = out_dir / f"sheet_{stem}_{start // per_page + 1:02d}.png"
            G.compose(man, sub, dst, title)
            made.append(dst)
    return made


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("library", help="a Stage A <root>/<disaster> directory")
    ap.add_argument("-o", "--out", default="",
                    help="gallery dir (default: <library>/_gallery)")
    ap.add_argument("--res", type=int, default=460)
    ap.add_argument("--samples", type=int, default=32)
    ap.add_argument("--az", type=float, default=38.0)
    ap.add_argument("--el", type=float, default=20.0)
    ap.add_argument("--per-page", type=int, default=8)
    ap.add_argument("--force", action="store_true",
                    help="re-render tiles that already have a PNG")
    ap.add_argument("--cpu", action="store_true")
    args = ap.parse_args()

    root = Path(args.library).resolve()
    out_dir = Path(args.out).resolve() if args.out else root / "_gallery"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows, rungs = load_library(root)
    if not rows:
        print("[render] library is empty", file=sys.stderr)
        return 1
    device = G.setup_engine(args.samples, not args.cpu)
    print(f"[render] {device}  {args.res}px  {args.samples} samples  "
          f"{len(rows)} asset(s) x {len(rungs)} rung(s)", flush=True)

    tiles, t0, done, skipped = {}, time.time(), 0, 0
    for r, row in enumerate(rows):
        frame = None
        for col in rungs:
            rel = row["cells"].get(col)
            if not rel:
                continue
            usd = root / rel
            # THE SIDECAR IF THERE IS ONE. `tools/localize_archetype_textures.py`
            # writes `<name>.local.usda` beside the archetype: the same scene
            # with its `omniverse://` maps redirected at the local mirror, which
            # is the only way Blender renders these textured rather than grey.
            sidecar = usd.parent / (usd.stem + ".local.usda")
            if sidecar.exists():
                usd = sidecar
            tile = out_dir / "tiles" / row["name"] / f"{col}.png"
            if not usd.exists():
                continue
            tiles[(r, col)] = tile
            # THE FRAME STILL HAS TO BE MEASURED even when the tile is cached,
            # or the first uncached rung in a resumed row frames itself and
            # comes out at a different scale from its neighbours. Measuring is
            # an import without a render, which is the cheap half.
            # STALE IF THE SOURCE IS NEWER. Without this, a tile rendered
            # before `localize_archetype_textures.py` had written that
            # archetype's sidecar stays cached untextured for good — which is
            # exactly what happens when the renderer and the localiser race,
            # and the gallery then quietly mixes textured and grey rows.
            fresh = (tile.exists()
                     and tile.stat().st_mtime >= usd.stat().st_mtime)
            if fresh and not args.force:
                skipped += 1
                if frame is None:
                    G.import_usd(usd)
                    c, rad = G.geometry_bounds()
                    frame = (c, rad * G.FRAME_PAD)
                continue
            frame = render_cell(usd, tile, args.res, frame, args.az, args.el)
            done += 1
            print(f"  [{r + 1}/{len(rows)}] {row['name'][:24]:24s} "
                  f"{col:17s} {'sidecar' if usd.suffix == '.usda' else 'raw':7s}"
                  f"  {time.time() - t0:6.1f}s", flush=True)

    sheets = compose_pages(rows, rungs, tiles, out_dir,
                           f"Stage A — {root.name} archetypes", args.per_page)
    print(f"[render] {done} rendered, {skipped} cached, "
          f"{time.time() - t0:.0f}s")
    for s in sheets:
        print(f"  {s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
