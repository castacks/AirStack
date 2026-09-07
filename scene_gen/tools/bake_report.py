#!/usr/bin/env python3
"""Join what each asset IS with what baking it COST. The bake's answer sheet.

    python3 scene_gen/tools/bake_report.py \\
        --library scene_gen/assets/archetypes_urban_v2/earthquake

Three inputs, all produced by other tools, none of which needs Isaac Sim:

    _bakelab/asset_properties.json   what the pack hands Stage A
                                     (`tools/asset_properties.py`)
    <library>/bake_trace.jsonl       one line per cell ATTEMPTED, with the
                                     timings and the failures
                                     (`archetypes/bake.py`)
    <library>/manifest.json          what actually exported

Three outputs:

    bake_rungs.csv     one row per (asset, rung) — the raw grain
    bake_assets.csv    one row per asset — properties beside totals
    BAKE_REPORT.md     the same, readable, with the cost model on top

THE TRACE, NOT THE MANIFEST, IS THE SPINE. A manifest only lists what
exported, so reading a bake from it makes every rejected settle and every
untextured export look like a cell that was never attempted — which is exactly
backwards when the question is what this pipeline cannot yet handle.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)

#: Ladder order, for column order. Mirrors `disaster.levels.LADDERS`.
RUNGS = ("pristine", "cracked", "soft_storey", "partial_collapse", "pancaked",
         "fallen", "stump")

DAMAGED = tuple(r for r in RUNGS if r != "pristine")


def read_trace(path: str) -> dict:
    """``{(type, level): row}`` — LAST attempt wins, so a re-bake supersedes."""
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path):
        try:
            row = json.loads(line)
        except ValueError:
            continue
        out[(row.get("type"), row.get("level"))] = row
    return out


def group_of(row: dict) -> str:
    src = str(row.get("source", ""))
    if row.get("build") == "modular":
        return "modular"
    if row.get("kind") == "vegetation":
        return "vegetation"
    if "/assets/standalone/" in src:
        return "standalone"
    if src.startswith("selected_citydemo"):
        return "citydemo"
    if "downtowncity" in src:
        return "downtowncity"
    return "nucleus"


RUNG_COLUMNS = ("type", "level", "group", "material", "hollow", "outcome",
                "seconds", "t_build_s", "t_settle_s", "t_export_s", "usd_mb",
                "frag_cells", "frag_loose", "frag_anchored", "thickened",
                "out_meshes", "settle_bodies", "settle_steps_used",
                "settle_still_moving", "settle_converged", "debris_r_m",
                "src_points", "src_meshes", "src_x_m", "src_y_m", "src_z_m",
                "finished_at", "note")


def rung_rows(props: list, trace: dict) -> list:
    by_type = {p["type"]: p for p in props}
    rows = []
    for (typ, level), tr in sorted(trace.items()):
        p = by_type.get(typ, {})
        st = tr.get("settle") or {}
        rows.append({
            "type": typ, "level": level, "group": group_of(p or tr),
            "material": tr.get("material") or p.get("material", ""),
            "hollow": p.get("hollow", tr.get("hollow", "")),
            "outcome": tr.get("outcome", ""),
            "seconds": tr.get("seconds"),
            "t_build_s": tr.get("t_build_s"), "t_settle_s": tr.get("t_settle_s"),
            "t_export_s": tr.get("t_export_s"), "usd_mb": tr.get("usd_mb"),
            "frag_cells": tr.get("frag_cells"), "frag_loose": tr.get("frag_loose"),
            "frag_anchored": tr.get("frag_anchored"),
            "thickened": tr.get("thickened"),
            "out_meshes": tr.get("meshes"),
            "settle_bodies": st.get("bodies"),
            "settle_steps_used": st.get("steps_used"),
            "settle_still_moving": st.get("still_moving"),
            "settle_converged": st.get("converged"),
            "debris_r_m": tr.get("debris_r_m"),
            "src_points": p.get("points", tr.get("src_points")),
            "src_meshes": p.get("meshes", tr.get("src_meshes")),
            "src_x_m": p.get("x_m", tr.get("src_x_m")),
            "src_y_m": p.get("y_m", tr.get("src_y_m")),
            "src_z_m": p.get("z_m", tr.get("src_z_m")),
            "finished_at": tr.get("finished_at", ""),
            "note": tr.get("note", ""),
        })
    return rows


ASSET_COLUMNS = (
    "type", "group", "material", "hollow", "x_m", "y_m", "z_m", "volume_m3",
    "src_points", "src_faces", "src_meshes", "src_materials", "src_textures",
    "src_file_mb", "rungs_attempted", "rungs_baked", "rungs_failed",
    "total_s", "total_mb", "cells_total", "loose_total",
) + tuple(f"{n}_s" for n in DAMAGED) + tuple(f"{n}_mb" for n in DAMAGED) + (
    "slowest_rung", "status", "source")


def asset_rows(props: list, trace: dict) -> list:
    by_type: dict = defaultdict(dict)
    for (typ, level), tr in trace.items():
        by_type[typ][level] = tr

    rows = []
    for p in props:
        typ = p["type"]
        cells = by_type.get(typ, {})
        baked = {lv: t for lv, t in cells.items() if t.get("outcome") == "baked"}
        failed = {lv: t for lv, t in cells.items() if t.get("outcome") != "baked"}
        vol = ((p.get("x_m") or 0) * (p.get("y_m") or 0) * (p.get("z_m") or 0))
        row = {
            "type": typ, "group": group_of(p), "material": p.get("material", ""),
            "hollow": p.get("hollow", ""),
            "x_m": p.get("x_m"), "y_m": p.get("y_m"), "z_m": p.get("z_m"),
            "volume_m3": round(vol, 1) if vol else None,
            "src_points": p.get("points"), "src_faces": p.get("faces"),
            "src_meshes": p.get("meshes"), "src_materials": p.get("materials"),
            "src_textures": p.get("textures"), "src_file_mb": p.get("file_mb"),
            "rungs_attempted": len(cells), "rungs_baked": len(baked),
            "rungs_failed": len(failed),
            "total_s": round(sum(float(t.get("seconds") or 0)
                                 for t in cells.values()), 1) or None,
            "total_mb": round(sum(float(t.get("usd_mb") or 0)
                                  for t in baked.values()), 1) or None,
            "cells_total": sum(int(t.get("frag_cells") or 0)
                               for t in baked.values()) or None,
            "loose_total": sum(int(t.get("frag_loose") or 0)
                               for t in baked.values()) or None,
            "source": p.get("source", ""),
        }
        for lv in DAMAGED:
            row[f"{lv}_s"] = (cells.get(lv) or {}).get("seconds")
            row[f"{lv}_mb"] = (cells.get(lv) or {}).get("usd_mb")
        slow = max(cells.items(), key=lambda kv: float(kv[1].get("seconds") or 0),
                   default=(None, {}))
        row["slowest_rung"] = slow[0] if slow[0] else ""
        if not cells:
            row["status"] = "not attempted"
        elif failed and baked:
            row["status"] = f"partial ({len(baked)} baked, {len(failed)} failed)"
        elif failed:
            row["status"] = "all rungs failed"
        elif len(baked) >= int(p.get("levels") or 5):
            row["status"] = "complete"
        else:
            row["status"] = f"in progress ({len(baked)} rungs)"
        rows.append(row)
    return rows


def _fmt(v, nd=0):
    if v is None or v == "":
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:,.{nd}f}"
    if isinstance(v, int):
        return f"{v:,}"
    return str(v)


def markdown(assets: list, rungs: list, library: str) -> str:
    done = [a for a in assets if a["rungs_baked"]]
    baked_rungs = [r for r in rungs if r["outcome"] == "baked"]
    fails = [r for r in rungs if r["outcome"] not in ("baked", "")]
    total_s = sum(float(r["seconds"] or 0) for r in rungs)
    total_mb = sum(float(r["usd_mb"] or 0) for r in baked_rungs)

    L = ["# Earthquake bake — `urban_v2`", "",
         f"Library: `{library}`", "",
         "## Where it got to", "",
         f"- **{len(done)} of {len(assets)} assets** have at least one rung baked; "
         f"**{sum(1 for a in assets if a['status'] == 'complete')}** are complete.",
         f"- **{len(baked_rungs)} archetypes** exported, {len(fails)} failed.",
         f"- **{total_s / 3600:.1f} h** of bake, **{total_mb / 1000:.1f} GB** written.",
         ""]

    if baked_rungs:
        per = defaultdict(list)
        for r in baked_rungs:
            if r["level"] != "pristine" and r["seconds"]:
                per[r["level"]].append(r)
        L += ["## Cost per rung", "",
              "| rung | n | median s | max s | median MB | max MB | median cells |",
              "|---|---:|---:|---:|---:|---:|---:|"]
        for lv in DAMAGED:
            got = per.get(lv) or []
            if not got:
                continue
            secs = sorted(float(r["seconds"]) for r in got)
            mbs = sorted(float(r["usd_mb"] or 0) for r in got)
            cells = sorted(int(r["frag_cells"] or 0) for r in got)
            L.append(f"| {lv} | {len(got)} | {secs[len(secs)//2]:,.0f} | "
                     f"{secs[-1]:,.0f} | {mbs[len(mbs)//2]:,.0f} | {mbs[-1]:,.0f} "
                     f"| {cells[len(cells)//2]:,} |")
        L.append("")

    L += ["## Per asset", "",
          "`vol` is the bounding box in m3 — the cost driver. `hollow` is the "
          "pack's `solid:` flag inverted: a hollow asset is thickened by "
          "`solidify` before it can be fractured.", "",
          "| asset | group | material | hollow | vol m3 | src pts | src mesh | "
          "rungs | bake s | out MB | cells | status |",
          "|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for a in sorted(assets, key=lambda a: (-(a["rungs_baked"] or 0),
                                           a["volume_m3"] or 0)):
        L.append(
            f"| `{a['type']}` | {a['group']} | {a['material']} | "
            f"{_fmt(a['hollow'])} | {_fmt(a['volume_m3'])} | "
            f"{_fmt(a['src_points'])} | {_fmt(a['src_meshes'])} | "
            f"{a['rungs_baked']}/{a['rungs_attempted'] or '-'} | "
            f"{_fmt(a['total_s'])} | {_fmt(a['total_mb'])} | "
            f"{_fmt(a['cells_total'])} | {a['status']} |")
    L.append("")

    if fails:
        L += ["## Cells that did not export", "",
              "| asset | rung | outcome | s | note |", "|---|---|---|---:|---|"]
        for r in fails:
            L.append(f"| `{r['type']}` | {r['level']} | {r['outcome']} | "
                     f"{_fmt(r['seconds'])} | {str(r['note'])[:90]} |")
        L.append("")
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--library", required=True,
                    help="a Stage A <root>/<disaster> directory")
    ap.add_argument("--properties", default="",
                    help="default: <scene_gen>/_bakelab/asset_properties.json")
    ap.add_argument("-o", "--out", default="",
                    help="default: <scene_gen>/_bakelab")
    args = ap.parse_args()

    lib = os.path.abspath(args.library)
    out = os.path.abspath(args.out or os.path.join(_SCENE_GEN, "_bakelab"))
    ppath = args.properties or os.path.join(_SCENE_GEN, "_bakelab",
                                            "asset_properties.json")
    if not os.path.exists(ppath):
        print(f"missing {ppath}; run tools/asset_properties.py first",
              file=sys.stderr)
        return 1
    props = json.load(open(ppath))
    trace = read_trace(os.path.join(lib, "bake_trace.jsonl"))

    rungs = rung_rows(props, trace)
    assets = asset_rows(props, trace)
    os.makedirs(out, exist_ok=True)

    for name, cols, rows in (("bake_rungs.csv", RUNG_COLUMNS, rungs),
                             ("bake_assets.csv", ASSET_COLUMNS, assets)):
        with open(os.path.join(out, name), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    md = os.path.join(out, "BAKE_REPORT.md")
    open(md, "w").write(markdown(assets, rungs, os.path.relpath(lib, _SCENE_GEN)))
    print(f"{len(rungs)} cell(s), {len(assets)} asset(s)")
    for n in ("bake_rungs.csv", "bake_assets.csv", "BAKE_REPORT.md"):
        print(f"  {os.path.join(out, n)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
