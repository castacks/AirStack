#!/usr/bin/env python3
"""fc_dump_crop.py — crop a `fire_city_placements_dump.v1` JSON (`FC_DUMP`,
written by `urban_fire_city_launch_script.dump_city_placements` on the pod)
to a non-centred window, so "generate 1.5 km, crop to 1 km" happens as a
POST-DUMP step rather than a smaller `region_m`. See `downtown_fire_1500.
yaml`'s header and `tools/crop_window.py`'s module docstring for the design
this is built on — this file is a thin schema adapter over that one.

    python3 scene_gen/tools/fc_dump_crop.py \\
        --in  scene_gen/_plans/city_placements_downtown_fire_1500_1.json \\
        --window -80 120 920 1120 \\
        --out scene_gen/_plans/city_placements_downtown_fire_1500_1_crop.json

`--window x0 y0 x1 y1` is in the DUMP's OWN (pre-crop) coordinate frame, the
same one `epicenter`/every placement's `x_m`/`y_m` already use.
`tools/baseline_layouts.py` is where the three baseline levels' actual window
draws are chosen and recorded, generally by centre + half-size:

    python3 scene_gen/tools/fc_dump_crop.py \\
        --in <dump.json> --centre 60 90 --size 1000 1000 --out <cropped.json>

OUTPUT SCHEMA IS UNCHANGED (`fire_city_placements_dump.v1`) — the whole
point is that `fire_city_dry_run.load_placements_dump` and
`disaster.fire_people.derive_layout` read a cropped dump with ZERO code
changes: `region_m` is rewritten to the window's own size, `placements`
carries only the SURVIVING houses (their `x_m`/`y_m` re-centred, their `i`/
`cell` UNTOUCHED — see `crop_window.py`'s "what this does not solve" for
why), and `typology.blocks` carries the CLIPPED block rects. A `"crop"` key
is added for provenance (window, shift, before/after counts); every existing
reader ignores unknown top-level keys.
"""
import argparse
import hashlib
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from crop_window import crop_layout  # noqa: E402

SCHEMA = "fire_city_placements_dump.v1"


def _doc_to_layout_placements(doc):
    """`fire_city_placements_dump.v1` -> the generic `(layout, placements)`
    shape `crop_window.crop_layout` wants. Every dump placement already
    carries `category` (`"house"` for every real record — see
    `dump_city_placements`'s own docstring, "only category == house
    placements are written") so the default `is_building` predicate applies
    unchanged; `W`/`D` are already measured and on the record, so
    `footprint_of` is a plain field read, no resolver needed."""
    placements = list(doc.get("placements") or [])
    blocks = []
    typ_of = {}
    for b in (doc.get("typology") or {}).get("blocks") or []:
        rect = tuple(float(v) for v in b["rect"])
        blocks.append(list(rect))
        typ_of[rect] = b.get("name")
    layout = {"blocks": blocks, "road_corridors": [], "_typology_of": typ_of}
    return layout, placements


def _footprint_of(p):
    if p.get("W") is None or p.get("D") is None:
        return None
    return float(p["W"]), float(p["D"])


def crop_fc_dump(doc, window, *, recenter=True):
    """`doc`: a loaded `fire_city_placements_dump.v1` dict. `window`:
    `(x0, y0, x1, y1)` in the dump's own coordinate frame. Returns
    `(new_doc, report)` — `new_doc` is schema-identical and JSON-serialisable
    (round-trips through `json.dumps`/`json.loads` unchanged), `report` is
    `crop_window.crop_layout`'s own report dict plus a couple of dump-level
    fields (`n_blocks_before`, `n_houses_before`)."""
    if doc.get("schema") != SCHEMA:
        raise ValueError("crop_fc_dump: expected schema {0!r}, got {1!r}"
                         .format(SCHEMA, doc.get("schema")))
    layout, placements = _doc_to_layout_placements(doc)
    new_layout, new_placements, rpt = crop_layout(
        layout, placements, window, footprint_of=_footprint_of,
        recenter=recenter)

    x0, y0, x1, y1 = new_layout["region"]
    new_blocks = [{"rect": [round(v, 3) for v in rect], "name": name}
                 for rect, name in new_layout["_typology_of"].items()]
    new_houses = []
    for p in new_placements:
        q = dict(p)
        # ROUND-TRIP-CLEAN for `load_placements_dump`'s required-field check
        # (`i, cell, usd, x_m, y_m, z_m, yaw_deg, W, D, H`) -- everything
        # already on the record survives `_shift`'s `dict(p)` copy; only
        # x_m/y_m are rewritten, and x_m_orig/y_m_orig are additive.
        q["x_m"] = round(float(q["x_m"]), 4)
        q["y_m"] = round(float(q["y_m"]), 4)
        new_houses.append(q)
    new_houses.sort(key=lambda p: p.get("i", 0))

    new_doc = dict(doc)
    new_doc["region_m"] = [round(x1 - x0, 3), round(y1 - y0, 3)]
    new_doc["placements"] = new_houses
    new_doc["typology"] = {"blocks": new_blocks}
    # n_placements_total is DELIBERATELY left unchanged -- it is the length
    # of the ORIGINAL full (every-category) placement list `i` indexes into,
    # not the cropped house count. See crop_window.py's docstring.
    new_doc["crop"] = {
        "window": rpt["window"], "shift": rpt["shift"],
        "source_region_m": doc.get("region_m"),
        "source_n_houses": len(doc.get("placements") or []),
        "houses_kept": rpt["buildings_kept"],
        "houses_dropped": rpt["buildings_dropped"],
        "blocks_kept": rpt["blocks_kept"], "blocks_dropped": rpt["blocks_dropped"],
    }
    report = dict(rpt)
    report["n_blocks_before"] = len(layout["blocks"])
    report["n_houses_before"] = len(placements)
    return new_doc, report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="in_path", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--window", nargs=4, type=float, default=None,
                    metavar=("X0", "Y0", "X1", "Y1"))
    ap.add_argument("--centre", nargs=2, type=float, default=None,
                    metavar=("CX", "CY"))
    ap.add_argument("--size", nargs=2, type=float, default=(1000.0, 1000.0),
                    metavar=("W", "H"), help="only used with --centre")
    ap.add_argument("--no-recenter", action="store_true",
                    help="keep the window's own (pre-crop) coordinates -- "
                         "for debugging the crop math itself, never for a "
                         "dump a downstream tool will actually solve on")
    a = ap.parse_args()

    if a.window is not None:
        window = tuple(a.window)
    elif a.centre is not None:
        cx, cy = a.centre
        w, h = a.size
        window = (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
    else:
        ap.error("give either --window x0 y0 x1 y1 or --centre cx cy [--size w h]")

    with open(a.in_path, "rb") as fh:
        raw = fh.read()
    src_sha256 = hashlib.sha256(raw).hexdigest()
    doc = json.loads(raw.decode("utf-8"))

    new_doc, report = crop_fc_dump(doc, window, recenter=not a.no_recenter)
    new_doc["crop"]["source_path"] = os.path.abspath(a.in_path)
    new_doc["crop"]["source_sha256"] = src_sha256

    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(new_doc, fh, indent=1)

    print("[fc_dump_crop] {0!r} -> {1!r}".format(a.in_path, a.out))
    print("[fc_dump_crop] window {0}  recenter={1}".format(
        window, not a.no_recenter))
    print("[fc_dump_crop] houses: {0} -> {1} kept, {2} dropped".format(
        report["n_houses_before"], report["buildings_kept"],
        report["buildings_dropped"]))
    print("[fc_dump_crop] blocks: {0} -> {1} kept, {2} dropped".format(
        report["n_blocks_before"], report["blocks_kept"],
        report["blocks_dropped"]))
    print("[fc_dump_crop] region_m: {0} -> {1}".format(
        doc.get("region_m"), new_doc["region_m"]))


if __name__ == "__main__":
    main()
