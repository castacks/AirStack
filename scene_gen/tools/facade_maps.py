#!/usr/bin/env python3
"""facade_maps — procedural PBR maps for a monolith wall: albedo, height,
normal, roughness.

WHY PROCEDURAL AND NOT AN AI GENERATOR
--------------------------------------
The suggested workflow is to screenshot a wall, feed it to Polycam / Meshy /
Substance Sampler and download a PBR set. That is a good way to get ONE nice
material and a bad way to get a hundred: it is manual, it is per-asset, and
nothing about the result is reproducible from the repo — which is the same
argument `tools/burn_textures.py` already makes for the char and ash maps,
and it generates those with spectral synthesis for exactly this reason.

A building façade is also the easy case. It is not organic: it is a GRID of
window reveals in a panelised wall, and the geometry of that grid is a handful
of numbers (bay pitch, storey height, reveal depth, mullion width). Generating
it means the height map lines up with the albedo by construction — which is
the thing an AI texture pack cannot promise, and misalignment between the two
is exactly what makes displaced windows look like smeared mud.

WHAT THE MAPS ARE FOR
---------------------
The point is Workflow 1: put the DEPTH in the material instead of in the mesh,
so a 266-point box gets window reveals, panel seams and a sill line without a
single extra polygon or any physics cost. `--kind` picks the façade type.

    python3 scene_gen/tools/facade_maps.py OUTDIR --kind office --px 1024
"""

import argparse
import math
import os


def _spectral(rng, n, beta=2.2):
    """Band-limited tileable noise (the `burn_textures` construction).

    White noise shaped by a radial 1/f**(beta/2) falloff in the frequency
    domain: no preferred axis, no lattice, and the DFT is periodic so it
    tiles exactly. Value noise on a grid shows its grid — that lesson is
    already in the wildfire skill and it applies to a wall as much as to ash.
    """
    import numpy as np
    w = rng.normal(size=(n, n))
    F = np.fft.fft2(w)
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.fftfreq(n)[None, :]
    r = np.sqrt(fx ** 2 + fy ** 2)
    r[0, 0] = 1e-6
    F *= r ** (-beta / 2.0)
    F[0, 0] = 0.0
    out = np.real(np.fft.ifft2(F))
    out -= out.min()
    return out / max(out.max(), 1e-9)


def build(kind, px, bays, storeys, seed):
    """(albedo, height, roughness) as float arrays in 0..1."""
    import numpy as np
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:px, 0:px] / float(px)

    # --- the window grid -------------------------------------------------
    # one bay across, one storey down; the window is a fraction of each cell
    u = (x * bays) % 1.0
    v = (y * storeys) % 1.0
    if kind == "office":
        wl, wr, wb, wt, depth = 0.12, 0.88, 0.18, 0.80, 1.0
    elif kind == "residential":
        wl, wr, wb, wt, depth = 0.26, 0.74, 0.22, 0.72, 0.85
    else:                                   # brutalist: slot windows
        wl, wr, wb, wt, depth = 0.10, 0.90, 0.34, 0.62, 1.0
    win = (u > wl) & (u < wr) & (v > wb) & (v < wt)

    # --- HEIGHT ----------------------------------------------------------
    # 1.0 = the wall plane, 0 = deepest. A window is a RECESS; the spandrel
    # under it and the mullions between are proud.
    h = np.ones((px, px), dtype=float)
    h[win] = 1.0 - 0.75 * depth
    # a chamfered reveal rather than a cliff — a one-texel step displaces to
    # a vertical wall that shows stair-stepping at any grazing angle
    for k, s in ((1, 0.82), (2, 0.90), (3, 0.96)):
        e = ((u > wl - k * 0.012) & (u < wr + k * 0.012) &
             (v > wb - k * 0.012) & (v < wt + k * 0.012)) & (~win)
        h[e] = np.minimum(h[e], s)
    # panel seams on the storey line and the bay line
    seam = (np.abs(v - 0.0) < 0.012) | (np.abs(v - 1.0) < 0.012) | \
           (np.abs(u - 0.0) < 0.010) | (np.abs(u - 1.0) < 0.010)
    h[seam] = np.minimum(h[seam], 0.88)
    # a sill, proud of the wall, under every opening
    sill = (u > wl - 0.03) & (u < wr + 0.03) & (v > wb - 0.05) & (v < wb)
    h[sill] = 1.06
    h += 0.02 * (_spectral(rng, px, 2.6) - 0.5)      # cast-surface tooth
    h = np.clip((h - h.min()) / max(float(h.max() - h.min()), 1e-9), 0, 1)

    # --- ALBEDO ----------------------------------------------------------
    conc = 0.34 + 0.10 * (_spectral(rng, px, 2.4) - 0.5)
    alb = np.dstack([conc, conc * 0.99, conc * 0.96])
    glass = np.dstack([np.full((px, px), 0.055),
                       np.full((px, px), 0.075),
                       np.full((px, px), 0.085)])
    alb = np.where(win[..., None], glass, alb)
    # weather streaks BELOW each sill — the single most recognisable thing
    # about a real concrete façade, and free here because the sill is known
    st = _spectral(rng, px, 3.4)
    below = (v > wt) & (u > wl) & (u < wr)
    alb = np.where((below & (st > 0.55))[..., None], alb * 0.72, alb)
    alb = np.clip(alb, 0, 1)

    rough = np.where(win, 0.18, 0.86 + 0.08 * (_spectral(rng, px, 2.0) - 0.5))
    return alb, h, np.clip(rough, 0, 1)


def normal_from_height(h, strength=6.0):
    import numpy as np
    gy, gx = np.gradient(h * strength)
    n = np.dstack([-gx, -gy, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return (n * 0.5 + 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--kind", default="office",
                    choices=("office", "residential", "brutalist"))
    ap.add_argument("--px", type=int, default=1024)
    ap.add_argument("--bays", type=int, default=6)
    ap.add_argument("--storeys", type=int, default=6)
    ap.add_argument("--seed", type=int, default=3)
    a = ap.parse_args()

    import numpy as np
    from PIL import Image

    os.makedirs(a.outdir, exist_ok=True)
    alb, h, rough = build(a.kind, a.px, a.bays, a.storeys, a.seed)
    nrm = normal_from_height(h)
    stem = os.path.join(a.outdir, a.kind)

    def _w(name, arr, mode):
        p = "{0}_{1}.png".format(stem, name)
        Image.fromarray((np.clip(arr, 0, 1) * 255).astype("uint8"),
                        mode=mode).save(p)
        return p

    out = [_w("albedo", alb, "RGB"),
           # HEIGHT AND ROUGHNESS ARE DATA, NOT COLOUR. They must be written
           # and read as linear/raw; an sRGB decode on a height map is a
           # non-linear remap of the depth and the reveals come out wrong
           # (the wildfire skill's `sourceColorSpace` finding).
           _w("height", np.dstack([h] * 3), "RGB"),
           _w("normal", nrm, "RGB"),
           _w("rough", np.dstack([rough] * 3), "RGB")]
    print("\n".join(out))
    print("height range {0:.3f}..{1:.3f}  window depth ~{2:.0f}% of scale"
          .format(float(h.min()), float(h.max()),
                  100.0 * (1.0 - float(np.percentile(h, 5)))))


if __name__ == "__main__":
    main()
