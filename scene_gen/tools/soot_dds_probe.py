#!/usr/bin/env python3
"""soot_dds_probe -- round-trip the REAL soot bake corpus through the BC1
encoder in `disaster/tex_compress.py` and report the honest quality/size
numbers, plus a before/after preview PNG for a human to eyeball.

Host-side, pure PIL/NumPy -- no `pxr`, no Kit, safe next to a live sim
(reads existing files on disk, writes only to `--out-dir`).

    python3 scene_gen/tools/soot_dds_probe.py \
        --glob '/home/USER/docker/isaac-sim/cache/main/fire_bakes/city_138/textures/sootbake_*.png' \
        --glob '/home/USER/docker/isaac-sim/cache/main/fire_bakes/city_138/textures/gacsoot_*.png' \
        --out-dir /home/USER/fire_previews --sample 6
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
from PIL import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import tex_compress as tc          # noqa: E402


def _psnr(a, b):
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    mse = float(np.mean((a - b) ** 2))
    return 99.0 if mse <= 1e-12 else 10.0 * np.log10((255.0 ** 2) / mse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", action="append", required=True,
                    help="one or more glob patterns over real sootbake_*/"
                         "gacsoot_*.png files; repeatable")
    ap.add_argument("--out-dir", default=None,
                    help="if set, writes a before/after montage PNG here "
                         "for --sample representative files")
    ap.add_argument("--sample", type=int, default=6)
    ap.add_argument("--limit", type=int, default=0,
                    help="cap total files measured (0 = all)")
    args = ap.parse_args()

    paths = []
    for g in args.glob:
        paths.extend(sorted(glob.glob(g)))
    if args.limit:
        paths = paths[:args.limit]
    if not paths:
        print("no files matched", args.glob)
        return 1

    rows = []
    dds_only = []          # real post-bake .dds -- no PNG original to diff
    t_enc = 0.0
    for p in paths:
        if p.lower().endswith(".dds"):
            # A REAL BAKE OUTPUT. There is no uncompressed original beside
            # it to diff against -- this is a "does it decode to something
            # sane" check, not a PSNR measurement. File size on disk IS the
            # true compressed size (no re-encoding here).
            dec, w, h = tc.read_dds_bc1(p)
            dds_only.append(dict(path=p, w=w, h=h, dec=dec,
                                 file_bytes=os.path.getsize(p)))
            continue
        im = Image.open(p).convert("RGB")
        arr = np.asarray(im, dtype=np.uint8)
        t0 = time.time()
        data, nbx, nby = tc.encode_bc1(arr)
        t_enc += time.time() - t0
        dec = tc.decode_bc1(data, nbx, nby)[:arr.shape[0], :arr.shape[1]]
        psnr = _psnr(arr, dec)
        png_bytes = os.path.getsize(p)
        rgba8_mips = arr.shape[0] * arr.shape[1] * 4 * 1.333
        bc1_mips_bytes = 0
        for lvl in tc._mip_chain(arr):
            d, nx, ny = tc.encode_bc1(lvl)
            bc1_mips_bytes += len(d)
        rows.append(dict(path=p, w=arr.shape[1], h=arr.shape[0],
                         psnr=psnr, png_bytes=png_bytes,
                         bc1_base_bytes=len(data),
                         bc1_mips_bytes=bc1_mips_bytes,
                         rgba8_mips_bytes=rgba8_mips))

    if dds_only:
        print("==== real .dds bake output(s) -- decode-only sanity check "
              "(no PNG original to diff) ====")
        for r in dds_only:
            rgba8_mips = r["w"] * r["h"] * 4 * 1.333
            print("%-70s %4dx%-4d  file=%8d B  implied vs RGBA8+mips: %.2fx"
                  % (os.path.basename(r["path"]), r["w"], r["h"],
                     r["file_bytes"], rgba8_mips / max(1, r["file_bytes"])))
        if args.out_dir:
            os.makedirs(args.out_dir, exist_ok=True)
            tiles = [np.asarray(Image.fromarray(r["dec"])) for r in dds_only]
            max_w = max(t.shape[1] for t in tiles)
            pad = 16
            ch = sum(t.shape[0] + pad for t in tiles)
            canvas = np.full((ch, max_w, 3), 30, dtype=np.uint8)
            y = 0
            for r, t in zip(dds_only, tiles):
                canvas[y:y + t.shape[0], :t.shape[1]] = t
                y += t.shape[0] + pad
            out_path = os.path.join(args.out_dir, "soot_dds_realbake_decode.png")
            Image.fromarray(canvas).save(out_path)
            print("wrote %s (%d real .dds decode(s))" % (out_path, len(dds_only)))
        print()
    if not rows:
        return 0

    print("%-70s %6s  %7s  %10s  %10s  %10s  %8s"
          % ("file", "dims", "psnr(dB)", "png(B)", "bc1+mip(B)",
             "rgba8+mip(B)", "vs rgba8"))
    for r in rows:
        vs = r["rgba8_mips_bytes"] / max(1, r["bc1_mips_bytes"])
        print("%-70s %4dx%-4d %7.2f  %10d  %10d  %10d  %6.2fx"
              % (os.path.basename(r["path"]), r["w"], r["h"], r["psnr"],
                 r["png_bytes"], r["bc1_mips_bytes"], int(r["rgba8_mips_bytes"]), vs))

    psnrs = [r["psnr"] for r in rows]
    tot_png = sum(r["png_bytes"] for r in rows)
    tot_bc1 = sum(r["bc1_mips_bytes"] for r in rows)
    tot_rgba8 = sum(r["rgba8_mips_bytes"] for r in rows)
    print("\n==== %d file(s) ====" % len(rows))
    print("PSNR: mean %.2f dB  min %.2f dB  max %.2f dB"
          % (float(np.mean(psnrs)), float(np.min(psnrs)), float(np.max(psnrs))))
    print("disk PNG total:        %.1f MB" % (tot_png / 1e6))
    print("VRAM RGBA8+mips total: %.1f MB  (the uncompressed baseline)"
          % (tot_rgba8 / 1e6))
    print("VRAM BC1+mips total:   %.1f MB  (%.2fx smaller than RGBA8+mips)"
          % (tot_bc1 / 1e6, tot_rgba8 / max(1, tot_bc1)))
    print("encode time: %.2f s for %d file(s) (%.1f ms/file, base level only)"
          % (t_enc, len(rows), 1000.0 * t_enc / max(1, len(rows))))

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        # pick a spread: worst PSNR, best PSNR, and evenly-spaced others
        order = sorted(range(len(rows)), key=lambda i: rows[i]["psnr"])
        n = min(args.sample, len(rows))
        pick_idx = sorted(set([order[0], order[-1]] +
                              [order[int(k * (len(order) - 1) / max(1, n - 1))]
                               for k in range(n)]))[:n]
        tiles = []
        for i in pick_idx:
            r = rows[i]
            im = np.asarray(Image.open(r["path"]).convert("RGB"), dtype=np.uint8)
            data, nbx, nby = tc.encode_bc1(im)
            dec = tc.decode_bc1(data, nbx, nby)[:im.shape[0], :im.shape[1]]
            diff = np.clip(np.abs(im.astype(int) - dec.astype(int)) * 4, 0, 255
                          ).astype(np.uint8)
            side = np.concatenate([im, dec, diff], axis=1)
            tiles.append((os.path.basename(r["path"]), r["psnr"], side))
        max_w = max(t[2].shape[1] for t in tiles)
        pad = 24
        canvas_h = sum(t[2].shape[0] + pad for t in tiles)
        canvas = np.full((canvas_h, max_w, 3), 30, dtype=np.uint8)
        y = 0
        for name, psnr, side in tiles:
            canvas[y:y + side.shape[0], :side.shape[1]] = side
            y += side.shape[0] + pad
        out_path = os.path.join(args.out_dir, "soot_bc1_before_after.png")
        Image.fromarray(canvas).save(out_path)
        with open(os.path.join(args.out_dir, "soot_bc1_before_after.txt"),
                  "w") as fh:
            fh.write("columns per row: ORIGINAL PNG | BC1 decoded | |diff|*4\n\n")
            for name, psnr, side in tiles:
                fh.write("%-60s psnr=%.2f dB\n" % (name, psnr))
        print("\nwrote %s (%d samples, %s)" % (out_path, len(tiles),
                                                "worst+best+spread by PSNR"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
