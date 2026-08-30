#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["numpy", "pillow"]
# ///
"""make_tileable.py — turn a debris-atlas base-colour photo into a seamless
ground tile for `quake_rubble_usd._rubble_look` (0.28-0.32 repeats/metre,
world-projected).

WHY THIS EXISTS: `assets/materials/quake/rubble_rc_B.jpg` / `rubble_urm_B.jpg`
(the mound/apron `diffuse_texture`) were a straight copy of a FAB debris
asset's own UV atlas — `concrete_debris_elements`'s / `brick_debris_pile`'s
baseColor map. That is correctly TEXTURED (real photographed rubble), but
it is NOT a ground tile: an atlas is laid out as many small islands packed
edge to edge for one specific mesh's unwrap, and repeating THAT at ground
scale reproduces the same packed-island grid over and over — the "plaid"
the round-4 v2 review called out (rc_dome_s3_close.png: a visible grid of
repeating rebar-strip islands from 40 m).

This script does three things numpy/PIL can do without a real texture-
synthesis library:

  1. crop the largest gap-free square out of the source (an atlas can carry
     small near-black un-mapped gaps between islands; landing a crop on one
     would bake a black hole into the tile);
  2. break the crop's own internal periodicity by blending FOUR copies of
     it (random torus-shifts, one each of the 0/90/180/270 degree
     rotations) through smooth low-frequency random masks that sum to 1
     pixel-wise — the same idea as texture "bombing": no single copy's
     island layout survives intact anywhere in the result;
  3. make THAT blend edge-to-edge seamless with the standard offset trick —
     roll the image by half its size (its own left/right and top/bottom
     seams meet in the middle instead of at the border) and feather a
     cross-shaped band around that new centre seam against a heavily
     softened copy of the same rolled image, 12-15% of the tile wide.

`--tone-only` skips all three steps — for a source that is ALREADY a
tileable PBR scan (a megascans surface material, not a debris atlas) and
only needs a plain colour grade (multiply + saturation + per-channel tint)
to work as an alternate rc/urm base, e.g. `Dirt_Rough` desaturated and
darkened per the round-4 debris-material brief.

ROUND-4 v4: the v3 mound texture was the atlas blend ALONE, and even after
the crop/blend/seamless repair a debris atlas's own big rebar-strip/panel
ISLAND SHAPES survive as recognisable (if scrambled) macro features — the
v3 review's "source atlas's rectangular chips repeating across the crown".
The fix is a fourth step, `--composite`: layer a HIGH-PASSED copy of the
atlas blend (itself minus a heavy Gaussian blur of itself — the atlas's OWN
big island shapes are exactly the LOW frequencies that blur keeps, so
subtracting it leaves only the fine aggregate/speckle detail) over a toned
`Dirt_Rough` base at partial strength. `Dirt_Rough` was checked first (a
plain 2x2 tile, no repair at all — see this agent's report) and found
ALREADY seamless — a real Megascans scan authored to tile — so its own
crop/blend/seamless steps are skipped entirely; only the atlas detail layer
still needs them.

Usage
-----
    # 1. the atlas blend (unchanged from v3) — this is the DETAIL layer now,
    #    not the final texture.
    uv run --python 3.13 --with numpy --with pillow python tools/make_tileable.py \\
        assets/concrete_rubble_debris/split/concrete_debris_elements/textures/Concrete_Debbris_Elements__baseColor.jpg \\
        ~/scorch_previews/rubble_r4/tiles/rc_atlas_blend.jpg \\
        --size 2048 --sheet ~/scorch_previews/rubble_r4/tiles/rc_atlas_blend_sheet.png

    # 2. the toned Dirt_Rough base — center-crop + colour grade only, no
    #    seam repair (it is already seamless).
    uv run --python 3.13 --with numpy --with pillow python tools/make_tileable.py \\
        assets/materials/megascans/Dirt_Rough/T_yd0lfcqcc_1k_B.png \\
        ~/scorch_previews/rubble_r4/tiles/rc_dirt_toned.jpg \\
        --size 2048 --tone-only --multiply 0.72 --saturation 0.6 \\
        --sheet ~/scorch_previews/rubble_r4/tiles/rc_dirt_toned_sheet.png

    # 3. composite: base + high-passed detail (`dst` is the ONLY positional
    #    in this mode — `src` is omitted, BASE/DETAIL replace it).
    uv run --python 3.13 --with numpy --with pillow python tools/make_tileable.py \\
        ~/scorch_previews/rubble_r4/tiles/v4_rc.jpg \\
        --composite ~/scorch_previews/rubble_r4/tiles/rc_dirt_toned.jpg \\
                    ~/scorch_previews/rubble_r4/tiles/rc_atlas_blend.jpg \\
        --blur-radius 75 --strength 0.6 \\
        --sheet ~/scorch_previews/rubble_r4/tiles/v4_rc_sheet.png
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
from PIL import Image


# ---------------------------------------------------------------------------
# io
# ---------------------------------------------------------------------------
def _load_rgb_float(path):
    """[0, 1] float32 HxWx3, sRGB-encoded (no gamma decode — every step here
    is a spatial/blend operation, not a radiometric one, so staying in the
    file's own encoding is fine and keeps the JPEG round-trip simple)."""
    im = Image.open(path).convert("RGB")
    return np.asarray(im, dtype=np.float32) / 255.0


def _save_rgb_float(arr, path, quality=90):
    arr8 = np.clip(arr * 255.0 + 0.5, 0, 255).astype(np.uint8)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(arr8, mode="RGB").save(path, quality=quality)


# ---------------------------------------------------------------------------
# step 1: largest gap-free square
# ---------------------------------------------------------------------------
def _integral(mask):
    """Summed-area table, one row/col of zero padding so `_box_sum` needs no
    bounds juggling."""
    ii = np.zeros((mask.shape[0] + 1, mask.shape[1] + 1), dtype=np.float64)
    ii[1:, 1:] = np.cumsum(np.cumsum(mask.astype(np.float64), axis=0), axis=1)
    return ii


def _box_sum(ii, y0, x0, size):
    y1, x1 = y0 + size, x0 + size
    return ii[y1, x1] - ii[y0, x1] - ii[y1, x0] + ii[y0, x0]


def largest_gap_free_square(arr, dark_thresh=10.0 / 255.0, tol=0.005, stride=32,
                            min_size=None):
    """The largest square crop of `arr` (float [0,1] HxWx3) whose fraction
    of near-black ("gap") pixels is at most `tol`. Searches candidate sizes
    from the full min(H, W) down, at `stride`-pixel steps for both size and
    position (an atlas gap is a patch many pixels wide, so a coarse search
    finds it without needing per-pixel precision). Falls back to a centred
    crop at the smallest size tried if nothing clears `tol` anywhere (very
    conservative sources only) — that is reported, not hidden.

    Returns (y0, x0, size, gap_fraction_at_that_crop).
    """
    h, w = arr.shape[:2]
    is_dark = (arr.max(axis=2) < dark_thresh)
    ii = _integral(is_dark)

    full = min(h, w)
    min_size = min_size or max(512, full // 4)
    best = None
    for size in range(full, min_size - 1, -stride):
        best_here = None
        for y0 in range(0, h - size + 1, stride):
            for x0 in range(0, w - size + 1, stride):
                frac = _box_sum(ii, y0, x0, size) / float(size * size)
                if best_here is None or frac < best_here[0]:
                    best_here = (frac, y0, x0)
        if best_here and best_here[0] <= tol:
            best = (best_here[1], best_here[2], size, best_here[0])
            break
    if best is None:
        # nothing cleared tolerance anywhere — use the least-bad spot at the
        # smallest size we tried, and say so loudly.
        size = min_size
        best_here = None
        for y0 in range(0, h - size + 1, stride):
            for x0 in range(0, w - size + 1, stride):
                frac = _box_sum(ii, y0, x0, size) / float(size * size)
                if best_here is None or frac < best_here[0]:
                    best_here = (frac, y0, x0)
        print("[make_tileable] WARNING: no {0}x{0}..{1}x{1} crop cleared "
              "tol={2}; using the least-bad spot found (gap frac={3:.4f})"
              .format(min_size, full, tol, best_here[0]))
        best = (best_here[1], best_here[2], size, best_here[0])
    return best


# ---------------------------------------------------------------------------
# step 2: break periodicity — blend 4 shifted/rotated copies through smooth
# low-frequency random masks that sum to 1
# ---------------------------------------------------------------------------
def _low_freq_field(h, w, rng, base=10):
    """A smooth random field: a tiny `base x base` grid of uniform noise,
    bicubic-upsampled to (h, w) — the upsample interpolation itself IS the
    low-pass filter, no separate blur pass needed."""
    small = (rng.random((base, base)) * 255.0).astype(np.uint8)
    im = Image.fromarray(small, mode="L").resize((w, h), Image.BICUBIC)
    return np.asarray(im, dtype=np.float32) / 255.0


def blend_four_copies(tile, rng):
    """`tile` is HxWx3 float, H == W (a square crop). Four torus-shifted
    copies, one each of the 0/90/180/270 degree rotations, combined through
    four smooth random masks normalised to sum to 1 at every pixel."""
    h, w = tile.shape[:2]
    assert h == w, "blend_four_copies expects a square crop"

    copies = []
    for k in range(4):
        dy = int(rng.integers(0, h))
        dx = int(rng.integers(0, w))
        shifted = np.roll(np.roll(tile, dy, axis=0), dx, axis=1)
        copies.append(np.rot90(shifted, k=k))

    fields = np.stack([_low_freq_field(h, w, rng) for _ in range(4)], axis=0)
    fields = fields + 1e-3  # keep every copy contributing something everywhere
    weights = fields / fields.sum(axis=0, keepdims=True)

    out = np.zeros_like(tile)
    for k in range(4):
        out += copies[k] * weights[k][:, :, None]
    return out


# ---------------------------------------------------------------------------
# step 3: seamless border cross-fade
# ---------------------------------------------------------------------------
def _low_freq_1d(n, rng, base=8):
    """A smooth random curve over `n` samples: `base` random control points,
    linearly interpolated — used to WOBBLE the mirror line below so it is
    not a perfectly straight axis of symmetry."""
    ctrl = rng.random(base + 1)
    xp = np.linspace(0, n - 1, base + 1)
    return np.interp(np.arange(n), xp, ctrl)


def make_seamless(arr, feather_frac=0.135, rng=None):
    """The offset trick: rolling by half the tile moves ITS OWN left/right
    and top/bottom seams from the border to the centre (a "+" through the
    middle), so a repair applied THERE is what makes the final tile repeat
    cleanly at its actual edges.

    The repair is a MIRROR cross-fade, not a blur: for a band `feather_frac`
    wide either side of the new centre seam, blend each pixel with its
    reflection across the seam line (weight 0.5 exactly at the seam, easing
    to 0 at the band's edge). This keeps the same noise/grain frequency as
    the rest of the tile — an early version of this function blended
    towards a heavily downsampled-and-back-up copy instead, which hid the
    hard cut but left an obvious soft "picture-frame" square around every
    tile once repeated (visible in this agent's first `*_atlas_blend_sheet`
    renders).

    A PLAIN mirror (reflection exactly across the seam line) still reads as
    a kaleidoscope — a dead-straight axis of symmetry down the middle of
    every tile (visible in `rc_atlas_blend.jpg`'s single-tile close-up: a
    faint but real vertical "hall of mirrors" through the centre). The
    mirror LINE itself is wobbled by a smooth per-row/per-column random
    offset (`_low_freq_1d`) so the reflected content does not line up
    exactly with itself — same seam repair, no dead-straight symmetry axis.
    """
    h, w = arr.shape[:2]
    rolled = np.roll(np.roll(arr, h // 2, axis=0), w // 2, axis=1)
    cy, cx = h // 2, w // 2
    rng = rng if rng is not None else np.random.default_rng(0)

    fx = max(1, int(feather_frac * w))
    fy = max(1, int(feather_frac * h))
    xs = np.arange(w)
    ys = np.arange(h)

    tx = np.clip(1.0 - np.abs(xs - cx) / fx, 0.0, 1.0)
    ty = np.clip(1.0 - np.abs(ys - cy) / fy, 0.0, 1.0)
    tx = 0.5 - 0.5 * np.cos(np.pi * tx)      # cosine ease, not a hard ramp
    ty = 0.5 - 0.5 * np.cos(np.pi * ty)

    # per-row wobble of the vertical mirror line, per-column wobble of the
    # horizontal one — up to ~60% of the feather width, smooth along the
    # seam so it reads as a wavy join rather than noise.
    jx = (_low_freq_1d(h, rng, base=8) - 0.5) * (1.2 * fx)
    jy = (_low_freq_1d(w, rng, base=8) - 0.5) * (1.2 * fy)
    mirror_x = np.clip(2 * cx - xs[None, :] + jx[:, None], 0, w - 1).astype(np.int64)
    mirror_y = np.clip(2 * cy - ys[:, None] + jy[None, :], 0, h - 1).astype(np.int64)

    out = rolled.astype(np.float32)
    wx = (0.5 * tx)[None, :, None]
    mirrored_cols = np.take_along_axis(rolled, np.repeat(mirror_x[:, :, None], 3, axis=2), axis=1)
    out = out * (1.0 - wx) + mirrored_cols * wx
    wy = (0.5 * ty)[:, None, None]
    mirrored_rows = np.take_along_axis(out, np.repeat(mirror_y[:, :, None], 3, axis=2), axis=0)
    out = out * (1.0 - wy) + mirrored_rows * wy
    return out


# ---------------------------------------------------------------------------
# tone-only path (an already-tileable PBR scan, e.g. Dirt_Rough)
# ---------------------------------------------------------------------------
def apply_tone(arr, multiply=1.0, saturation=1.0, tint=(1.0, 1.0, 1.0)):
    """`tint` is a per-channel LINEAR multiplier applied AFTER `multiply`/
    `saturation` — e.g. the urm base's warm (1.08, 0.95, 0.85) cast, which a
    single scalar `multiply` cannot express."""
    out = arr * float(multiply)
    if saturation != 1.0:
        luma = (0.2126 * out[..., 0] + 0.7152 * out[..., 1] + 0.0722 * out[..., 2])
        out = luma[:, :, None] + (out - luma[:, :, None]) * float(saturation)
    tint = tuple(float(c) for c in tint)
    if tint != (1.0, 1.0, 1.0):
        out = out * np.asarray(tint, dtype=np.float32).reshape(1, 1, 3)
    return np.clip(out, 0.0, 1.0)


# ---------------------------------------------------------------------------
# round-4 v4: composite a toned base (Dirt_Rough) with a HIGH-PASSED detail
# layer (the atlas blend, minus a heavy blur of itself) — see the module
# docstring's ROUND-4 v4 note for why: a debris atlas's own island shapes are
# a LOW-frequency artifact, so subtracting a blur of the same image leaves
# only the fine speckle detail behind, with no whole recognisable chip.
# ---------------------------------------------------------------------------
def gaussian_blur_wrap(arr, radius):
    """A Gaussian blur that WRAPS at the image border instead of clamping —
    correct for a texture meant to tile, where the content already matches
    across its own edges (the atlas blend went through `make_seamless`
    specifically so this holds; a plain edge-clamped blur would smear a
    slightly wrong colour into the last `radius`-ish pixels near each edge
    and reintroduce a faint border seam in the high-pass result)."""
    from PIL import ImageFilter

    h, w = arr.shape[:2]
    pad = max(1, int(round(radius)) * 3)
    padded = np.pad(arr, ((pad, pad), (pad, pad), (0, 0)), mode="wrap")
    im = Image.fromarray(np.clip(padded * 255.0 + 0.5, 0, 255).astype(np.uint8))
    im = im.filter(ImageFilter.GaussianBlur(radius))
    blurred = np.asarray(im, dtype=np.float32) / 255.0
    return blurred[pad:pad + h, pad:pad + w]


def high_pass(arr, radius):
    """`arr` minus a wrap-safe Gaussian blur of itself — a SIGNED detail
    layer centred on 0 (mid-grey when visualised), carrying only spatial
    frequencies finer than `radius` pixels."""
    return arr - gaussian_blur_wrap(arr, radius)


def composite_base_detail(base, detail, blur_radius=75.0, strength=0.6):
    """`base` (toned Dirt_Rough) + `strength` x `high_pass(detail, blur_radius)`
    — `detail` and `base` must be the same size (the caller resizes)."""
    return np.clip(base + high_pass(detail, blur_radius) * float(strength), 0.0, 1.0)


def center_square(arr, size):
    h, w = arr.shape[:2]
    s = min(h, w)
    y0, x0 = (h - s) // 2, (w - s) // 2
    crop = arr[y0:y0 + s, x0:x0 + s]
    if s != size:
        im = Image.fromarray(np.clip(crop * 255.0, 0, 255).astype(np.uint8))
        im = im.resize((size, size), Image.LANCZOS)
        crop = np.asarray(im, dtype=np.float32) / 255.0
    return crop


# ---------------------------------------------------------------------------
# contact sheet
# ---------------------------------------------------------------------------
def contact_sheet(tile, out_path, grid=4, tile_px=512):
    im = Image.fromarray(np.clip(tile * 255.0, 0, 255).astype(np.uint8))
    im = im.resize((tile_px, tile_px), Image.LANCZOS)
    sheet = Image.new("RGB", (tile_px * grid, tile_px * grid))
    for j in range(grid):
        for i in range(grid):
            sheet.paste(im, (i * tile_px, j * tile_px))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)


def _parse_triple(s):
    parts = [float(v) for v in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected 3 comma-separated floats, got: " + s)
    return tuple(parts)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", nargs="?", default=None,
                    help="source image (omit with --composite — dst is the only positional then)")
    ap.add_argument("dst")
    ap.add_argument("--size", type=int, default=2048)
    ap.add_argument("--sheet", default=None, help="4x4 tiled contact-sheet PNG path")
    ap.add_argument("--sheet-grid", type=int, default=4)
    ap.add_argument("--sheet-tile-px", type=int, default=512)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--feather", type=float, default=0.135,
                    help="border cross-fade width as a fraction of --size (0.12-0.15)")
    ap.add_argument("--dark-thresh", type=float, default=10.0 / 255.0)
    ap.add_argument("--gap-tol", type=float, default=0.005)
    ap.add_argument("--tone-only", action="store_true",
                    help="skip crop/blend/seamless — just centre-crop + colour-grade "
                         "(for a source that is already a tileable PBR scan)")
    ap.add_argument("--seamless-base", action="store_true",
                    help="apply ONLY the wobbled-mirror seam repair (make_seamless), "
                         "no crop/blend-four-copies — for a --tone-only base that turns "
                         "out NOT already seamless (check with a bare 2x2 tile first)")
    ap.add_argument("--multiply", type=float, default=1.0)
    ap.add_argument("--saturation", type=float, default=1.0)
    ap.add_argument("--tint", type=_parse_triple, default=(1.0, 1.0, 1.0),
                    help="per-channel linear multiplier 'R,G,B', applied after "
                         "--multiply/--saturation (e.g. a warm urm cast)")
    ap.add_argument("--quality", type=int, default=90)
    ap.add_argument("--composite", nargs=2, metavar=("BASE", "DETAIL"),
                    help="skip src entirely: BASE (a toned tile) + strength x "
                         "high_pass(DETAIL, --blur-radius) -> dst")
    ap.add_argument("--blur-radius", type=float, default=75.0,
                    help="--composite: Gaussian blur radius (px) subtracted from DETAIL "
                         "to make the high-pass layer (60-90 px)")
    ap.add_argument("--strength", type=float, default=0.6,
                    help="--composite: how strongly the high-pass DETAIL layer is added")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    dst = os.path.expanduser(args.dst)

    if args.composite:
        base_path, detail_path = (os.path.expanduser(p) for p in args.composite)
        base = _load_rgb_float(base_path)
        detail = _load_rgb_float(detail_path)
        print("[make_tileable] composite base={0} {1}x{2}  detail={3} {4}x{5}"
              .format(base_path, base.shape[1], base.shape[0],
                      detail_path, detail.shape[1], detail.shape[0]))
        if detail.shape[:2] != base.shape[:2]:
            h, w = base.shape[:2]
            im = Image.fromarray(np.clip(detail * 255.0, 0, 255).astype(np.uint8))
            detail = np.asarray(im.resize((w, h), Image.LANCZOS), dtype=np.float32) / 255.0
        tile = composite_base_detail(base, detail, blur_radius=args.blur_radius,
                                     strength=args.strength)
        print("[make_tileable] composited: blur_radius={0} strength={1}"
              .format(args.blur_radius, args.strength))
    else:
        if not args.src:
            ap.error("src is required unless --composite is given")
        src = os.path.expanduser(args.src)
        arr = _load_rgb_float(src)
        print("[make_tileable] loaded {0}  {1}x{2}".format(src, arr.shape[1], arr.shape[0]))

        if args.tone_only:
            tile = center_square(arr, args.size)
            if args.seamless_base:
                tile = make_seamless(tile, feather_frac=args.feather, rng=rng)
            tile = apply_tone(tile, args.multiply, args.saturation, args.tint)
        else:
            y0, x0, size, gap_frac = largest_gap_free_square(
                arr, dark_thresh=args.dark_thresh, tol=args.gap_tol)
            print("[make_tileable] crop: y0={0} x0={1} size={2} gap_frac={3:.4f}"
                  .format(y0, x0, size, gap_frac))
            crop = arr[y0:y0 + size, x0:x0 + size]
            if size != args.size:
                im = Image.fromarray(np.clip(crop * 255.0, 0, 255).astype(np.uint8))
                im = im.resize((args.size, args.size), Image.LANCZOS)
                crop = np.asarray(im, dtype=np.float32) / 255.0
            blended = blend_four_copies(crop, rng)
            tile = make_seamless(blended, feather_frac=args.feather, rng=rng)
            if args.multiply != 1.0 or args.saturation != 1.0 or tuple(args.tint) != (1.0, 1.0, 1.0):
                tile = apply_tone(tile, args.multiply, args.saturation, args.tint)

    _save_rgb_float(tile, dst, quality=args.quality)
    print("[make_tileable] wrote {0}".format(dst))

    if args.sheet:
        sheet_path = os.path.expanduser(args.sheet)
        contact_sheet(tile, sheet_path, grid=args.sheet_grid, tile_px=args.sheet_tile_px)
        print("[make_tileable] wrote {0} ({1}x{1} grid of {2}x{2} tiles, {3}px each)"
              .format(sheet_path, args.sheet_grid, args.sheet_grid, args.sheet_tile_px))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
