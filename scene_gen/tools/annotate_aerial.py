#!/usr/bin/env python
"""annotate_aerial.py — overlay a reference grid and block badges on the aerial.

    AirStack/.venv/bin/python scene_gen/tools/annotate_aerial.py \
        --aerial ~/coasei/portable/disaster_city_aerial.png \
        --config disaster_city \
        --out ~/coasei/portable/disaster_city_aerial_keyed.png

SO THAT A REVIEW CAN POINT. Every round of notes on this scene has had to say
things like "the north-east most block" or "the kink west of the rubble", and
each one costs a guess about which block or which kink. This draws the two
labels that remove the guess:

  * a GRID of lettered columns and numbered rows — "the junction in E7" — which
    works for anything, including road corners and ground that belongs to no
    block; and
  * a BADGE per block, B1..Bn, which is what the manifest's `blocks` list calls
    them, so a note about B12 lands on one entry in the YAML.

The axes are also labelled in WORLD METRES, because that is the coordinate the
site plan, the feature list and every measurement in this pipeline are written
in: "the east kerb at y = -50" needs no translation at either end.

It runs on the HOST with no Kit — it reads the PNG plus the sidecar the
renderer wrote, and the blocks straight from the site plan.
"""

import argparse
import json
import os
import string
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))


def _font(size):
    from PIL import ImageFont
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.isfile(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--aerial", required=True)
    ap.add_argument("--config", default="disaster_city",
                    help="site plan name, for the block badges")
    ap.add_argument("--out", default="")
    ap.add_argument("--cell-m", type=float, default=25.0,
                    help="grid cell size in metres")
    args = ap.parse_args()

    from PIL import Image, ImageDraw

    aerial = os.path.expanduser(args.aerial)
    side = os.path.splitext(aerial)[0] + ".json"
    if not os.path.isfile(side):
        raise SystemExit(f"no mapping sidecar beside the image: {side}\n"
                         f"re-run render_aerial.py, which writes it")
    m = json.load(open(side))
    cx, cy = m["centre_m"]
    s = float(m["px_per_m"])
    im = Image.open(aerial).convert("RGB")
    W, H = im.size

    def to_px(x, y):
        return (W / 2.0 + (x - cx) * s, H / 2.0 - (y - cy) * s)

    # Grid lines on ROUND world numbers, not on the image edge, so a cell
    # boundary is a number worth saying out loud.
    cell = float(args.cell_m)
    x0 = cell * ((cx - (W / 2.0) / s) // cell)
    y0 = cell * ((cy - (H / 2.0) / s) // cell)
    # Only cells whose CENTRE is on the image get a label — a half-cell hanging
    # off the edge is not somewhere anyone can point at — and the lettering
    # starts from the first of those, so the top-left visible cell is A1.
    cols, rows = [], []
    x = x0
    while to_px(x, 0)[0] < W:
        if 0 <= to_px(x + cell / 2.0, 0)[0] <= W:
            cols.append(x)
        x += cell
    y = y0
    while to_px(0, y)[1] > 0:
        if 0 <= to_px(0, y - cell / 2.0)[1] <= H:
            rows.append(y)
        y += cell
    rows.reverse()                      # top row first, so row 1 is the north
    if not cols or not rows:
        raise SystemExit("the grid does not fall on the image; check --cell-m")

    pad = 54
    out = Image.new("RGB", (W + pad, H + pad), (250, 250, 248))
    out.paste(im, (pad, pad))
    d = ImageDraw.Draw(out, "RGBA")
    f_grid = _font(19)
    f_axis = _font(15)
    f_badge = _font(27)

    gx = x0
    while to_px(gx, 0)[0] < W:
        px = to_px(gx, 0)[0] + pad
        if px >= pad:
            d.line([(px, pad), (px, H + pad)],
                   fill=(255, 255, 255, 90), width=1)
            d.text((px + 3, pad - 20), f"{gx:.0f}", font=f_axis,
                   fill=(90, 90, 95))
        gx += cell
    gy = y0
    while to_px(0, gy)[1] > 0:
        py = to_px(0, gy)[1] + pad
        if py >= pad:
            d.line([(pad, py), (W + pad, py)],
                   fill=(255, 255, 255, 90), width=1)
            d.text((4, py + 3), f"{gy:.0f}", font=f_axis, fill=(90, 90, 95))
        gy += cell

    # Cell labels: letters across, numbers down, centred in the cell.
    letters = list(string.ascii_uppercase)
    for i, x in enumerate(cols):
        if i >= len(letters):
            break
        px = to_px(x + cell / 2.0, 0)[0] + pad
        if not (pad < px < W + pad):
            continue
        d.text((px - 7, 6), letters[i], font=f_grid, fill=(40, 40, 45))
    for j, y in enumerate(rows):
        py = to_px(0, y - cell / 2.0)[1] + pad
        if not (pad < py < H + pad):
            continue
        d.text((pad - 26, py - 11), str(j + 1), font=f_grid, fill=(40, 40, 45))

    def cell_of(x, y):
        ci = int((x - cols[0]) // cell)
        rj = int((rows[0] - y) // cell)
        cl = letters[ci] if 0 <= ci < len(letters) else "?"
        return f"{cl}{rj + 1}"

    # Block badges, numbered as the manifest lists them.
    import random
    from layout import site_plan as spl
    spec = spl.load_spec(args.config)
    w_m, h_m = spl.region_of(spec)
    _net, blocks, _info = spl.generate(w_m, h_m, random.Random(0),
                                       {"site_plan": args.config})
    rows_out = []
    for i, b in enumerate(blocks):
        bx, by = b["centroid"]
        px, py = to_px(bx, by)
        px += pad
        py += pad
        lbl = f"B{i + 1}"
        r = 21
        d.ellipse([px - r, py - r, px + r, py + r],
                  fill=(255, 255, 255, 225), outline=(30, 30, 35), width=2)
        tw = d.textlength(lbl, font=f_badge)
        d.text((px - tw / 2.0, py - 15), lbl, font=f_badge, fill=(20, 20, 25))
        rows_out.append((lbl, str(b.get("role")), cell_of(bx, by),
                         f"{bx:.1f}", f"{by:.1f}", f"{abs(b['area']):.0f}"))

    dst = os.path.expanduser(args.out) or (
        os.path.splitext(aerial)[0] + "_keyed.png")
    out.save(dst)
    idx = os.path.splitext(dst)[0] + "_index.tsv"
    with open(idx, "w") as fh:
        fh.write("label\trole\tcell\tx_m\ty_m\tarea_m2\n")
        for r in rows_out:
            fh.write("\t".join(r) + "\n")
    print(f"[keyed] {len(blocks)} blocks, {len(cols)}x{len(rows)} cells "
          f"of {cell:.0f} m", flush=True)
    print(f"[keyed] -> {dst}", flush=True)
    print(f"[keyed] -> {idx}", flush=True)


if __name__ == "__main__":
    main()
