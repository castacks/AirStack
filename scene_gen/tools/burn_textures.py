"""burn_textures.py — the burnt-material albedo maps, in variants.

    python3 tools/burn_textures.py            # all variants
    python3 tools/burn_textures.py --list

WHY VARIANTS RATHER THAN ONE MAP
--------------------------------
Burnt material does not look like one thing, and which one is right is a
judgement call best made by looking at them side by side on a wall. So this
generates a SET, the bench shows them together, and the ones that survive get
kept. Nothing here is load-bearing until something picks it.

WHAT REAL BURNT WOOD LOOKS LIKE
-------------------------------
Two references, both from photographs:

  black char   an ALLIGATOR CHECKING pattern — as timber chars, the surface
               expands and splits into a field of plates with deep fissures
               between them, elongated ALONG the grain. Fire investigators use
               the pattern by name. Nearly black, with the fissures darker
               still and the plate faces catching a little light.

  ashy         a chaotic litter of pale angular FLAKES with black char showing
               between them and occasional warm tan. Hard value jumps from
               flake to flake, not smooth mottling — which is exactly what
               fractal noise cannot produce and a Voronoi field can.

The generator therefore has two engines: spectral noise for tonal variation,
and a jittered Voronoi field for anything with plates or flakes in it.

TILEABILITY
-----------
Spectral noise is periodic by construction. The Voronoi field wraps its cell
centres. Photographs are made seamless by offset-and-blend. All three tile.
"""

import argparse
import os

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

_HERE = os.path.dirname(os.path.abspath(__file__))
_OUT = os.path.join(os.path.dirname(_HERE), "assets", "materials", "burn")
_REFS = os.path.expanduser("~/Downloads")
_N = 1024


# ---------------------------------------------------------------------------
# engines
# ---------------------------------------------------------------------------

def _fbm(rng, n, beta=2.4, lo=None, hi=None):
    """Isotropic fractal noise by spectral synthesis. Tileable, no lattice."""
    f = np.fft.fft2(rng.normal(size=(n, n)))
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.fftfreq(n)[None, :]
    r = np.sqrt(fx * fx + fy * fy)
    r[0, 0] = 1.0e-6
    amp = r ** (-beta / 2.0)
    if lo is not None:
        amp = amp * (r >= lo)
    if hi is not None:
        amp = amp * (r <= hi)
    amp[0, 0] = 0.0
    return np.real(np.fft.ifft2(f * amp))


def _voronoi(rng, n, cells=26, aniso=1.0, jitter=0.46):
    """Jittered Voronoi. Returns (edge, cell_value).

    `edge` is the distance to the second-nearest centre minus the nearest —
    zero on a boundary, large mid-cell — which is what draws the fissures.
    `cell_value` is one random level per cell, which is what draws flat flakes.

    `aniso` > 1 stretches the cells along X, the way checking elongates along
    the grain. Centres are wrapped so the field tiles.
    """
    # Anisotropy belongs in the DISTANCE, not in the cell count. Using fewer
    # rows just makes bigger square cells; scaling one axis before the nearest-
    # neighbour lookup is what actually elongates them along the grain.
    cy = max(2, int(round(cells)))
    gx, gy = np.meshgrid(np.arange(cells), np.arange(cy))
    pts = np.stack([gx.ravel() / cells, gy.ravel() / cy], -1).astype(float)
    pts += rng.uniform(-jitter, jitter, pts.shape) * np.array(
        [1.0 / cells, 1.0 / cy])
    vals = rng.random(len(pts))

    # 3x3 wrap so lookups near an edge see the far side.
    offs = np.array([[dx, dy] for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
    tiled = (pts[None, :, :] + offs[:, None, :]).reshape(-1, 2)
    tiled_vals = np.tile(vals, len(offs))

    ys, xs = np.mgrid[0:n, 0:n]
    q = np.stack([xs.ravel() / n, ys.ravel() / n], -1)

    metric = np.array([1.0 / max(1e-6, aniso), 1.0])
    d, idx = cKDTree(tiled * metric).query(q * metric, k=2)
    edge = (d[:, 1] - d[:, 0]).reshape(n, n)
    cell = tiled_vals[idx[:, 0]].reshape(n, n)
    return _norm(edge), cell


def _norm(a):
    lo, hi = a.min(), a.max()
    return (a - lo) / (hi - lo + 1e-9)


def _tileable(img, _blend=None):
    """Make a photograph tile, by blending four half-shifted copies.

    The first cut mirrored the edges into each other, which removes the seam
    but writes a visible axis of symmetry through the tile — the cross you can
    see running through it. Blending four copies offset by half in each
    direction, with complementary ramps, is seamless AND has no symmetry; the
    cost is a little ghosting, which a chaotic ash surface hides completely.
    """
    a = np.asarray(img, dtype=np.float64) / 255.0
    n, m = a.shape[0], a.shape[1]

    rx = np.linspace(0.0, 1.0, m)[None, :, None]
    ry = np.linspace(0.0, 1.0, n)[:, None, None]
    rx = rx * rx * (3.0 - 2.0 * rx)                 # smoothstep
    ry = ry * ry * (3.0 - 2.0 * ry)

    a00 = a
    a10 = np.roll(a, m // 2, axis=1)
    a01 = np.roll(a, n // 2, axis=0)
    a11 = np.roll(a, (n // 2, m // 2), axis=(0, 1))
    return (a00 * (rx * ry) + a10 * ((1 - rx) * ry)
            + a01 * (rx * (1 - ry)) + a11 * ((1 - rx) * (1 - ry)))


def _load_ref(name, n=_N):
    for ext in (".webp", ".jpg", ".png", ".jpeg"):
        p = os.path.join(_REFS, name + ext)
        if os.path.exists(p):
            im = Image.open(p).convert("RGB").resize((n, n), Image.LANCZOS)
            return _tileable(im)
    return None


# ---------------------------------------------------------------------------
# variants
# ---------------------------------------------------------------------------

def v_char_alligator(rng, n=_N):
    """Procedural match for the black reference: checking along the grain."""
    edge, cellv = _voronoi(rng, n, cells=30, aniso=2.6, jitter=0.44)
    grain = _norm(_fbm(rng, n, 1.5, lo=0.02))
    broad = _norm(_fbm(rng, n, 2.4, lo=0.008, hi=0.20))

    # Narrow, deep fissures and brighter plate faces — the reference has real
    # contrast at the crack edges, which a soft falloff washes out.
    fissure = np.clip(1.0 - edge / 0.11, 0.0, 1.0) ** 2.2
    plate = 0.075 + 0.115 * cellv + 0.055 * broad + 0.035 * grain
    v = plate * (1.0 - 0.93 * fissure)
    return np.stack([v * 1.10, v * 0.99, v * 0.93], -1)


def v_char_ref(rng, n=_N):
    """The black reference photograph itself, made seamless."""
    a = _load_ref("burnt wood blak", n)
    return a if a is not None else v_char_alligator(rng, n)


def v_ash_flake(rng, n=_N):
    """Procedural match for the ashy reference: pale angular flakes on char."""
    # BIGGER FLAKES. At 64/150 cells the flakes were pixel-scale and the whole
    # thing read as asphalt; the reference has flakes you can pick out
    # individually, so they need to be a real fraction of the tile.
    edge, cellv = _voronoi(rng, n, cells=22, aniso=1.35, jitter=0.52)
    edge2, cellv2 = _voronoi(rng, n, cells=55, aniso=1.0, jitter=0.52)
    grain = _norm(_fbm(rng, n, 1.2, lo=0.03))

    # Flat per-flake levels, two scales, hard steps between them. The power
    # pushes most flakes bright and leaves a minority dark, as in the photo.
    flake = 0.6 * cellv + 0.4 * cellv2
    v = 0.20 + 0.78 * flake ** 0.62 + 0.05 * grain

    # Char in the gaps between flakes.
    gap = np.clip(1.0 - edge2 / 0.10, 0.0, 1.0) ** 1.1
    v = v * (1.0 - 0.90 * gap)

    rgb = np.stack([v, v * 0.985, v * 0.955], -1)
    # Occasional warm tan, as in the reference.
    warm = (cellv2 > 0.955)
    rgb[warm] = rgb[warm] * np.array([1.18, 1.02, 0.74])
    return np.clip(rgb, 0.0, 1.0)


def v_ash_ref(rng, n=_N):
    """The ashy reference photograph itself, made seamless."""
    a = _load_ref("ashy burnt wood", n)
    return a if a is not None else v_ash_flake(rng, n)


def v_ash_over_char(rng, n=_N):
    """Ash bloom lying ON checked char — the two references combined.

    This is what a burnt wall mostly is: black underneath, pale deposit on the
    plates, char showing through wherever the ash has been knocked off.
    """
    edge, cellv = _voronoi(rng, n, cells=30, aniso=2.6, jitter=0.44)
    fissure = np.clip(1.0 - edge / 0.22, 0.0, 1.0) ** 1.5
    grain = _norm(_fbm(rng, n, 1.5, lo=0.02))

    v = (0.05 + 0.075 * cellv + 0.03 * grain) * (1.0 - 0.80 * fissure)
    rgb = np.stack([v * 1.08, v * 0.98, v * 0.92], -1)

    bloom = _norm(_fbm(rng, n, 1.4, lo=0.012, hi=0.24))
    # Ash sits ON the plates and is scoured out of the fissures, so the mask
    # follows plate interior (high `edge`), not the cracks.
    cover = (np.clip((bloom - 0.28) / 0.72, 0.0, 1.0)
             * np.clip((edge - 0.16) / 0.34, 0.0, 1.0))[..., None] ** 0.7
    ash = np.stack([np.full((n, n), 0.66), np.full((n, n), 0.655),
                    np.full((n, n), 0.635)], -1) * (0.7 + 0.5 * grain[..., None])
    return np.clip(rgb * (1.0 - cover) + ash * cover, 0.0, 1.0)


def v_scorch(rng, n=_N):
    """Heat-darkened timber, not consumed. Kept — this one already reads."""
    base = _norm(_fbm(rng, n, 2.6))
    grain = _norm(_fbm(rng, n, 3.0))
    band = 0.5 + 0.5 * np.sin(np.linspace(0, 34 * np.pi, n))[None, :]
    v = 0.10 + 0.16 * base + 0.05 * band * grain
    return np.stack([v * 1.25, v * 0.95, v * 0.74], -1)


VARIANTS = {
    "Char_Alligator": (v_char_alligator, 11),
    "Char_Ref":       (v_char_ref, 12),
    "Ash_Flake":      (v_ash_flake, 21),
    "Ash_Ref":        (v_ash_ref, 22),
    "Ash_Over_Char":  (v_ash_over_char, 23),
    "Scorch":         (v_scorch, 31),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--size", type=int, default=_N)
    ap.add_argument("--sheet", default=None, help="write a contact sheet here")
    args = ap.parse_args()

    if args.list:
        for k in VARIANTS:
            print(" ", k)
        return

    os.makedirs(_OUT, exist_ok=True)
    imgs = []
    for name, (fn, seed) in VARIANTS.items():
        rgb = np.clip(fn(np.random.default_rng(seed), args.size), 0.0, 1.0)
        img = Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), "RGB")
        img.save(os.path.join(_OUT, "Burn_{0}.png".format(name)))
        imgs.append((name, img))
        print("  Burn_{0:<16s} mean {1:.3f}".format(name, rgb.mean()))

    if args.sheet:
        cell = 300
        cols = 3
        rows = (len(imgs) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * cell, rows * cell))
        for i, (_n, im) in enumerate(imgs):
            sheet.paste(im.resize((cell, cell)),
                        ((i % cols) * cell, (i // cols) * cell))
        sheet.save(args.sheet)
        print("  sheet ->", args.sheet)


if __name__ == "__main__":
    main()
