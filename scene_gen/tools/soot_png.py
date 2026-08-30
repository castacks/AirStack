#!/usr/bin/env python3
"""soot_png.py — the fire EVENTS and the soot skin they leave, without Isaac Sim.

    python3 scene_gen/tools/soot_png.py --style commercial_mid \\
        --levels F1,F2,F3,F4,F5,F6 --out ~/scorch_previews

WHY THIS EXISTS
---------------
"Is the soot where the fire is?" is a question about arithmetic, not about
rendering, and it should not cost an Isaac Sim launch to answer. This runs the
REAL code path — `urban_building.build_building` -> `quake_flow.describe` ->
`urban_fire.plan_fire` -> `soot_plume.plan_events` -> `soot_plume.skin` — the
same functions `urban_fire.burn_building` calls, and draws the result over a
synthetic elevation: mid-grey wall, every MEASURED opening of the kit as a dark
rectangle at its true position, storey lines, and the four corner seams.

On top of that it marks every fire EVENT at the openings it vents through:

    red     FLAME    — alight now; `r_flames` puts Flow fire sources here
    orange  SMOULDER — burnt out but still smoking; Flow smoke here
    white   OUT      — burnt out, nothing volumetric; the stain is all that is
                       left of it ("fire was", invisible in the sim)
    cyan    STAIN    — smoke damage only, never alight (F1)

So a preview shows the coupling directly: every plume should sit on a marked
run of windows, and every marked run should have a plume. The same style at
every level is stacked into one ladder image.

WHAT IT IS NOT. The base is synthetic — the kit's own cladding maps live on
Nucleus and are only read inside Kit — so tone against brick or render is not
judged here, only WHERE the soot is and HOW it fans, fades, overlaps and
saturates. `--elevations` additionally writes each side as its own image at
the skin's full resolution.
"""

import argparse
import math
import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from detail import urban_building as ub           # noqa: E402
from disaster import quake_flow as qf             # noqa: E402
from disaster import soot_plume as sp             # noqa: E402
from disaster import urban_fire as uf             # noqa: E402

STATE_RGB = {"flame": (235, 40, 30), "smoulder": (255, 150, 30),
             "out": (245, 245, 245), "stain": (60, 220, 240)}


def ladder_heavy(btype, level):
    """The `smoke_stain` `heavy` the ladder itself passes for this level, so
    a preview and the sim rasterise the same durations."""
    for name, kw in uf.LADDER.get(btype, {}).get(level, []):
        if name == "smoke_stain":
            return float((kw or {}).get("heavy", 1.0))
    return 1.0


def build_ctx(style, level, seed, origin_frac=None, sides=None, heavy=None):
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)
    placements = ub.build_building(style, 0.0, 0.0, 0.0, rng)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    ctx = {"info": info, "rng": rng, "nrng": nrng, "notes": []}
    mtag = max(info["masses"].items(),
               key=lambda kv: (len(kv[1]["levels"]), kv[0] == "main"))[0]
    n_st = len(info["masses"][mtag]["levels"])
    origin = None
    if origin_frac is not None:
        origin = max(0, min(n_st - 1, int(round(origin_frac * (n_st - 1)))))
    ctx["fire"] = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
    ctx["tag"] = "b{0}".format(seed)
    ctx["fire"]["events"] = sp.plan_events(ctx, uf._severity)
    ctx["heavy"] = ladder_heavy(info["type"], level) if heavy is None else heavy
    return ctx


def synthetic_base(ctx, sk):
    """Mid-grey wall, the measured openings as dark rectangles, storey lines,
    corner seams — all in the skin's own coordinates."""
    rgba = sk["rgba"]
    h, w = rgba.shape[0], rgba.shape[1]
    ppm, H, z0 = sk["ppm"], sk["H"], sk["z0"]
    m = ctx["info"]["masses"][sk["mass"]]
    base = np.full((h, w, 3), 0.66, dtype=np.float32)
    # faint courses so the streaks have something to read against
    yy = (np.arange(h) // max(2, int(round(0.30 * ppm)))) % 2
    base[yy == 1] *= 0.96
    for i, z in enumerate(m["levels"]):
        r = int(round((H - (z - z0)) * ppm))
        if 0 <= r < h:
            base[max(0, r - 1):r + 1] = 0.50
    off = sk["offsets"]
    for side in ("S", "E", "N", "W"):
        c = int(round(off[side] * ppm)) % w
        base[:, max(0, c - 1):c + 1] = 0.35
        for s in range(len(m["levels"])):
            for op in sp.openings(ctx, sk["mass"], side, s):
                if op.get("virtual"):
                    continue
                u0, u1, za, zb = op["span"]
                c0 = int(round((off[side] + u0) * ppm))
                c1 = int(round((off[side] + u1) * ppm))
                r0 = int(round((H - (zb - z0)) * ppm))
                r1 = int(round((H - (za - z0)) * ppm))
                cols = np.arange(c0, max(c0 + 1, c1)) % w
                r0, r1 = max(0, r0), max(r0 + 1, min(h, r1))
                base[r0:r1][:, cols] = 0.22
                base[max(0, r1 - 2):r1][:, cols] = 0.80   # the sill
    return base


def draw_events(img, ctx, sk):
    from PIL import ImageDraw

    d = ImageDraw.Draw(img)
    ppm, H, z0 = sk["ppm"], sk["H"], sk["z0"]
    w = img.width
    off = sk["offsets"]
    for ev in sk["events"]:
        col = STATE_RGB.get(ev["state"], (255, 0, 255))
        x0 = int(round((off[ev["side"]] + ev["u0"]) * ppm)) % w
        x1 = int(round((off[ev["side"]] + ev["u1"]) * ppm)) % w
        y0 = int(round((H - (ev["z_head"] - z0)) * ppm))
        y1 = int(round((H - (ev["z_sill"] - z0)) * ppm))
        if x1 < x0:
            x1 = w - 1
        for k in range(2):
            d.rectangle([x0 - k, y0 - k, x1 + k, y1 + k], outline=col)
        # the flame tip EC1 predicts, as a tick
        geo = ev.get("geo") or sp.flame_geometry(ev)
        yt = int(round((H - (geo["z_tip"] - z0)) * ppm))
        xc = (x0 + x1) // 2
        d.line([xc - 6, yt, xc + 6, yt], fill=col, width=2)
        d.text((x0 + 2, y0 - 12), "{0}{1}".format(ev["state"][0].upper(),
                                                   ev["id"]), fill=col)
    return img


def render(ctx, sk, out_path, label=""):
    from PIL import Image, ImageDraw

    base = synthetic_base(ctx, sk)
    out = sp.merge_rgb(base, sk["rgba"])
    img = Image.fromarray((np.clip(out, 0, 1) * 255.0 + 0.5).astype(np.uint8))
    img = draw_events(img, ctx, sk)
    if label:
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, 10 + 7 * len(label), 16], fill=(0, 0, 0))
        d.text((4, 2), label, fill=(255, 255, 0))
    img.save(out_path)
    return img


def profile(sk, ev, heights=(0.5, 1, 2, 3, 4, 6, 8, 10, 14, 20)):
    """Mean alpha in a column the width of the event's FIRST opening, above
    that opening's head (the run's centre can fall on the pier between two
    windows, which reads clean until the plumes merge)."""
    rgba, ppm, H, z0 = sk["rgba"], sk["ppm"], sk["H"], sk["z0"]
    h, w = rgba.shape[0], rgba.shape[1]
    off = sk["offsets"][ev["side"]]
    u0, u1 = ev["ops"][0]["span"][0], ev["ops"][0]["span"][1]
    c0 = int(round((off + u0) * ppm))
    c1 = max(c0 + 1, int(round((off + u1) * ppm)))
    cols = np.arange(c0, c1) % w
    out = []
    for dz in heights:
        z = ev["z_head"] + dz
        r = int(round((H - (z - z0)) * ppm))
        if 0 <= r < h:
            out.append((dz, float(rgba[r][cols, 3].mean())))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--style", default="commercial_mid")
    ap.add_argument("--levels", default="F1,F2,F3,F4,F5,F6")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--origin", type=float, default=0.25,
                    help="origin storey as a fraction of height; -1 = random")
    ap.add_argument("--sides", default="",
                    help="e.g. S or S,E; default: the bench rule "
                         "(S for F1/F2, S+E above)")
    ap.add_argument("--heavy", type=float, default=None,
                    help="override the ladder's own smoke_stain heavy")
    ap.add_argument("--out", default=os.path.expanduser("~/scorch_previews"))
    ap.add_argument("--elevations", action="store_true")
    ap.add_argument("--ladder-width", type=int, default=2200)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    levels = [v.strip() for v in a.levels.split(",") if v.strip()]
    origin_frac = None if a.origin < 0 else a.origin
    panels = []
    for i, lv in enumerate(levels):
        sides = tuple(v.strip() for v in a.sides.split(",") if v.strip()) or (
            ("S",) if lv in ("F1", "F2") else ("S", "E"))
        ctx = build_ctx(a.style, lv, a.seed + i, origin_frac, sides, a.heavy)
        f = ctx["fire"]
        finish = f.get("finish") or "char"
        glass = ctx["info"]["type"] == "rc_glass"
        sk = sp.skin(ctx, f["events"], np.random.default_rng(sp.event_seed(ctx) ^ 0x5EED),
                     finish=finish, glass=glass, duration_scale=ctx["heavy"])
        m = ctx["info"]["masses"][sk["mass"]]
        path = os.path.join(a.out, "soot_{0}_{1}.png".format(a.style, lv))
        label = "{0} {1} origin st{2} band {3}-{4} sides {5} {6}".format(
            a.style, lv, f["origin"], f["storeys"][0], f["top"],
            "/".join(f["sides"]), sp.summarise(f["events"]))
        img = render(ctx, sk, path, label)
        panels.append(img)
        al = sk["rgba"][..., 3]
        band_rows = []
        for s in f["storeys"]:
            z_lo = m["levels"][s]
            z_hi = m["levels"][s + 1] if s + 1 < len(m["levels"]) else m["top"]
            r0 = int(round((sk["H"] - (z_hi - sk["z0"])) * sk["ppm"]))
            r1 = int(round((sk["H"] - (z_lo - sk["z0"])) * sk["ppm"]))
            band_rows.append((max(0, r0), min(al.shape[0], r1)))
        band = np.concatenate([al[r0:r1] for r0, r1 in band_rows]) \
            if band_rows else al
        print("[soot_png] {0}: canvas {1}x{2} @ {3:.1f} px/m, {4}".format(
            lv, al.shape[1], al.shape[0], sk["ppm"], label))
        print("           alpha: mean {0:.3f}  band mean {1:.3f}  >0.5 share "
              "{2:.3f}  rgb std in band {3:.4f}".format(
                  float(al.mean()), float(band.mean()),
                  float((al > 0.5).mean()),
                  float(sk["rgba"][..., :3][np.concatenate(
                      [np.arange(r0, r1) for r0, r1 in band_rows])].std())
                  if band_rows else 0.0))
        for ev in f["events"][:3]:
            geo = ev["geo"]
            print("           ev{0} {1} {2} st{3} u {4:.1f}-{5:.1f} w_t {6:.2f} "
                  "h_eq {7:.2f} Q {8:.1f} MW L_L {9:.2f} {10} tau {11:.0f}s"
                  .format(ev["id"], ev["state"], ev["side"], ev["storey"],
                          ev["u0"], ev["u1"], ev["w_t"], ev["h_eq"], geo["Q"],
                          geo["L_L"], "attached" if geo["attached"] else
                          "PROJECTING", ev["tau"]))
            print("           alpha above head: " + "  ".join(
                "{0:g}m:{1:.2f}".format(dz, v) for dz, v in profile(sk, ev)))
        if a.elevations:
            off = sk["offsets"]
            for side in ("S", "E", "N", "W"):
                L = sp.side_length(m, side)
                c0 = int(round(off[side] * sk["ppm"]))
                c1 = int(round((off[side] + L) * sk["ppm"]))
                img.crop((c0, 0, c1, img.height)).save(os.path.join(
                    a.out, "soot_{0}_{1}_{2}.png".format(a.style, lv, side)))

    if len(panels) > 1:
        from PIL import Image

        W = a.ladder_width
        rows = []
        for im in panels:
            s = W / float(im.width)
            rows.append(im.resize((W, max(1, int(im.height * s)))))
        H = sum(im.height for im in rows) + 6 * (len(rows) - 1)
        ladder = Image.new("RGB", (W, H), (255, 0, 255))
        y = 0
        for im in rows:
            ladder.paste(im, (0, y))
            y += im.height + 6
        lp = os.path.join(a.out, "soot_{0}_ladder.png".format(a.style))
        ladder.save(lp)
        print("[soot_png] ladder -> {0}".format(lp))


if __name__ == "__main__":
    main()
