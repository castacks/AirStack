"""scorch stage — composite a soot layer ONTO a surface's own texture.

WHY NOT JUST DARKEN
-------------------
Turning `albedo_brightness` down darkens a wall uniformly, and uniform is the
one thing soot never is. The result reads as a dimmer switch: same wall, less
light. What makes a surface look burnt is the UNEVENNESS — soot rises, so it
streaks upward from every opening, pools under the eaves and along the head of
each window, and leaves sheltered areas almost clean. The contrast between the
fouled parts and the surviving original colour is the whole effect.

So this reads the surface's own base-colour map, composites a soot layer over
it with a directional mask, and writes a new texture. The wall keeps its
identity — cream siding is still visibly cream siding — while carrying real
scorch marks where a fire would actually have left them.

WHERE THE SOURCE TEXTURES LIVE
------------------------------
Mostly on Nucleus (`omniverse://...`), so they cannot be opened with `open()`.
`omni.client.read_file` handles both those and local paths, which is the same
route `tools/foliage_season.py` uses to sample leaf colour.

Composited maps are cached on disk under
`scene_gen/assets/materials/scorched/` and keyed by source texture and soot
level, so a street of houses composites each unique combination once.
"""

import hashlib
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(os.path.dirname(_HERE), "assets", "materials",
                       "scorched")

# Soot colour. Not black: real deposit is a very dark warm grey, and pure
# black kills the texture underneath it entirely.
SOOT_RGB = (0.038, 0.034, 0.031)


def read_texture(url, max_px=1024):
    """Load a texture from Nucleus or disk into an HxWx3 float array."""
    from PIL import Image
    import io

    data = None
    try:
        import omni.client
        result, _, content = omni.client.read_file(url)
        if str(result).endswith("OK"):
            data = bytes(memoryview(content))
    except Exception:
        data = None
    if data is None:
        path = url
        if path.startswith("file://"):
            path = path[7:]
        if not os.path.exists(path):
            return None
        with open(path, "rb") as fh:
            data = fh.read()

    try:
        im = Image.open(io.BytesIO(data)).convert("RGB")
    except Exception:
        return None
    if max(im.size) > max_px:
        s = max_px / float(max(im.size))
        im = im.resize((max(1, int(im.width * s)), max(1, int(im.height * s))))
    return np.asarray(im, dtype=np.float64) / 255.0


def _noise(rng, h, w, beta=2.2, stretch_v=1.0, lo=None, hi=None):
    """Spectral noise, normalised to 0..1.

    `stretch_v` squashes the vertical frequency axis, which turns blobs into
    upward streaks. `lo`/`hi` band-limit it, which is how a mosaic is made
    without the low frequencies turning it into one big blob.
    """
    f = np.fft.fft2(rng.normal(size=(h, w)))
    fy = np.fft.fftfreq(h)[:, None] * float(stretch_v)
    fx = np.fft.fftfreq(w)[None, :]
    r = np.sqrt(fx * fx + fy * fy)
    r[0, 0] = 1e-6
    amp = r ** (-beta / 2.0)
    if lo is not None:
        amp = amp * (r >= lo)
    if hi is not None:
        amp = amp * (r <= hi)
    amp[0, 0] = 0.0
    a = np.real(np.fft.ifft2(f * amp))
    return (a - a.min()) / (a.max() - a.min() + 1e-9)


def soot_mask(h, w, rng, coverage=0.5, from_below=True):
    """How much soot lands where. 0 clean, 1 fully fouled.

    DIRECTION DEPENDS ON WHERE THE FIRE WAS, and for a wildfire it is from
    BELOW. The flame front arrives at ground level and washes up the outside of
    a wall, so the scorch is heaviest at the base and tapers upward into licks
    — the opposite of a compartment fire, where flame vents out of the openings
    and the deposit is heaviest at the head of each window and under the eaves.
    The first cut here assumed the compartment case and looked upside down on a
    building that burned from the outside.

    `from_below=False` restores the top-down version for interior surfaces or
    for a structure that burned from within.

    Three things stacked:
      wash      the gradient. Most of the effect.
      streaks   noise stretched ~8x vertically, so it smears into licks
                rather than blobs.
      patches   ordinary mottling, so the streaks do not read as a curtain.
    """
    y = np.linspace(1.0, 0.0, h)[:, None]        # 1 at the top of the image
    wash = (1.0 - y) ** 1.5 if from_below else y ** 1.5

    streaks = _noise(rng, h, w, beta=2.0, stretch_v=8.0)
    patches = _noise(rng, h, w, beta=2.6)

    m = 0.52 * wash + 0.32 * streaks + 0.26 * patches
    m = (m - m.min()) / (m.max() - m.min() + 1e-9)
    # `coverage` moves the threshold rather than scaling the mask, so a light
    # scorch is a SMALLER fouled area rather than the same area at lower
    # opacity — the difference between scorched and merely dim.
    t = 1.0 - float(coverage)
    m = np.clip((m - t) / max(1e-6, 1.0 - t), 0.0, 1.0)
    # <1 pushes mid-range values UP, so more of the surface sits at heavy
    # coverage rather than tapering off gently.
    return m ** 0.62


def composite(base, mask, soot_rgb=SOOT_RGB, char_bite=0.72):
    """Lay soot over a base colour map."""
    m = mask[..., None]
    soot = np.asarray(soot_rgb, dtype=float)[None, None, :]
    # Where the mask is strongest the surface is not merely coated but charred,
    # so it also loses saturation.
    grey = base.mean(axis=2, keepdims=True)
    desat = base * (1.0 - char_bite * m) + grey * (char_bite * m)
    return np.clip(desat * (1.0 - m) + soot * m, 0.0, 1.0)


def scorched_texture(url, coverage, rng, out_dir=None, verbose=False,
                     from_below=True):
    """Composite and cache one scorched map. Returns a local path, or None."""
    from PIL import Image

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    # The recipe is part of the key. Without it, retuning SOOT_RGB or
    # char_bite silently reuses the maps baked under the OLD values and
    # nothing appears to change.
    key = hashlib.md5(
        "{0}|{1:.3f}|{2}|{3}".format(
            url, coverage, int(from_below),
            "{0}-{1:.2f}".format(SOOT_RGB, 0.72)).encode("utf-8")
    ).hexdigest()[:16]
    path = os.path.join(out_dir, "scorch_{0}.png".format(key))
    if os.path.exists(path):
        return path

    base = read_texture(url)
    if base is None:
        if verbose:
            print("[scorch] could not read {0}".format(url))
        return None
    h, w = base.shape[:2]
    out = composite(base, soot_mask(h, w, rng, coverage=coverage,
                                    from_below=from_below))
    Image.fromarray((out * 255.0 + 0.5).astype(np.uint8), "RGB").save(path)
    if verbose:
        print("[scorch] {0} @ {1:.2f} -> {2}".format(
            url.rsplit("/", 1)[-1], coverage, os.path.basename(path)))
    return path


# ---------------------------------------------------------------------------
# Ground — one baked map for the whole block
# ---------------------------------------------------------------------------
#
# WHY BAKE RATHER THAN TILE. A ground material tiled across 250 m repeats
# whatever its scale: at one tile per 2 m that is 125 repeats and reads as a
# grid of rectangles, and enlarging the tile only trades a visible grid for a
# blurry one. Baking a single texture for the entire plate removes the repeat
# outright, and — more importantly — lets the burn be PAINTED rather than
# uniform, which is what a real scar needs.
#
# WHAT A GRASS BURN ACTUALLY LOOKS LIKE, and it is nothing like the wash on a
# wall. Fire scars are a MOSAIC: a patchy, mottled interior inside an irregular
# outline, with unburned islands the fire skipped where ground was wetter, a
# road or stream broke the run, or microtopography sheltered it. There is no
# directionality and no streaking — grass burns to black ash over bare soil in
# patches, and the patches are the signal.

def ground_burn_map(region_m, coverage_fn, grass_url, burnt_url, rng,
                    size=2048, out_dir=None, verbose=False):
    """Bake grass-to-burnt across a whole region. Returns a local path.

    `coverage_fn(x_m, y_m) -> 0..1` is the fire's own field, so the scar
    follows the front rather than being painted by hand. Mosaic noise and
    unburned islands are layered on top, because a field alone gives a smooth
    gradient and real scars are blotchy.
    """
    from PIL import Image

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    w_m, h_m = float(region_m[0]), float(region_m[1])
    key = hashlib.md5("{0}|{1}|{2}|{3}|{4}".format(
        grass_url, burnt_url, w_m, h_m, size).encode("utf-8")).hexdigest()[:16]
    path = os.path.join(out_dir, "ground_{0}.png".format(key))
    if os.path.exists(path):
        return path

    grass = read_texture(grass_url, max_px=1024)
    burnt = read_texture(burnt_url, max_px=1024)
    if grass is None or burnt is None:
        if verbose:
            print("[scorch] ground: missing {0}".format(
                "grass" if grass is None else "burnt"))
        return None

    def tiled(src, reps):
        """Repeat `src` `reps` times across `size` pixels."""
        k = max(2, int(size / float(reps)))
        small = np.asarray(
            Image.fromarray((src * 255).astype(np.uint8)).resize((k, k)),
            dtype=np.float64) / 255.0
        # Tile count from the ROUNDED tile size, not from `reps`: int() on
        # size/reps loses up to a whole tile, and tiling `reps+1` times then
        # left the canvas short of `size` and broke the blend.
        nt = int(np.ceil(size / float(k))) + 1
        return np.tile(small, (nt, nt, 1))[:size, :size, :]

    # ~4 m per tile of source detail; the mosaic mask is what hides the repeat.
    reps = max(2, int(round(max(w_m, h_m) / 4.0)))
    g = tiled(grass, reps)
    b = tiled(burnt, reps)

    # The fire's own field, sampled across the plate.
    ys, xs = np.mgrid[0:size, 0:size]
    X = (xs / float(size) - 0.5) * w_m
    Y = (0.5 - ys / float(size)) * h_m
    field = np.vectorize(coverage_fn)(X, Y).astype(np.float64)

    # MOSAIC. Mid-frequency blotching pushes the smooth field into patches,
    # so the boundary fingers in and out the way a real fire edge does.
    blotch = _noise(rng, size, size, 2.3, lo=0.004, hi=0.05)
    m = np.clip(field * 1.15 + (blotch - 0.5) * 0.55, 0.0, 1.0)

    # UNBURNED ISLANDS. Rare, compact, fully green — the patches a fire
    # skipped. Without them a scar reads as a paint job.
    isl = _noise(rng, size, size, 2.0, lo=0.012, hi=0.09)
    m = m * (1.0 - np.clip((isl - 0.80) / 0.20, 0.0, 1.0))

    m = np.clip(m, 0.0, 1.0)[..., None] ** 0.85
    out = np.clip(g * (1.0 - m) + b * m, 0.0, 1.0)
    Image.fromarray((out * 255.0 + 0.5).astype(np.uint8), "RGB").save(path)
    if verbose:
        print("[scorch] ground map {0}x{0} -> {1} ({2:.0%} burnt)".format(
            size, os.path.basename(path), float(m.mean())))
    return path
