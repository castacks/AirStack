"""Prior families for the planner benchmark, all expressed as axis-aligned Gaussian bumps.

Every consumer in the stack (mtl_search_planner, tigris_search_planner, mtl_metrics_logger,
the Isaac scene, and score.py here) rebuilds the prior from ``airstack.belief``:

    raster = min(sum_k amplitude_k * exp(-0.5 ((n-n_k)/sn_k)^2 - 0.5 ((e-e_k)/se_k)^2), belief_cap)
    raster = max(raster, base_uncertainty); raster /= raster.sum()

A bump mixture plus the floor is therefore the one representation every planner and every
scorer already reads identically. The families below build very different priors out of
it: blobs, clusters, heavy-tailed peaks, decoys, roads/rivers (lines of small bumps),
rings, drift plumes, smooth random fields, diffuse backgrounds and multi-scale mixtures.

Each family is ``fn(rng, area, **params) -> (bumps, base_uncertainty)``; ``area`` is
``(n_min, n_max, e_min, e_max)`` in mission NED metres. ``FAMILIES`` maps the name to the
function and to the parameter ranges ``sample_family_params`` draws from.
"""

from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

Bump = dict[str, float]


def _bump(n, e, sn, se, a) -> Bump:
    return {"n": float(n), "e": float(e), "sigma_n": float(sn), "sigma_e": float(se), "amplitude": float(a)}


def _inside(area, margin):
    n0, n1, e0, e1 = area
    return n0 + margin, n1 - margin, e0 + margin, e1 - margin


def _uniform_xy(rng, area, margin, k=1):
    n0, n1, e0, e1 = _inside(area, margin)
    return rng.uniform(n0, n1, k), rng.uniform(e0, e1, k)


def _polyline(rng, area, margin, n_vertices, step_len, turn_sd):
    """A wandering polyline (a road, a river, a ridge) inside the area."""
    n0, n1, e0, e1 = _inside(area, margin)
    p = np.array([rng.uniform(n0, n1), rng.uniform(e0, e1)])
    h = rng.uniform(0, 2 * math.pi)
    pts = [p.copy()]
    for _ in range(n_vertices):
        h += rng.normal(0, turn_sd)
        q = p + step_len * np.array([math.cos(h), math.sin(h)])
        if not (n0 <= q[0] <= n1 and e0 <= q[1] <= e1):  # bounce off the margin
            h += math.pi
            q = p + step_len * np.array([math.cos(h), math.sin(h)])
            q = np.clip(q, [n0, e0], [n1, e1])
        p = q
        pts.append(p.copy())
    return np.array(pts)


def _along(pts, spacing):
    """Points every `spacing` metres along a polyline."""
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        L = float(np.hypot(*seg))
        k = max(1, int(L // spacing))
        for i in range(k):
            out.append(a + seg * (i / k))
    out.append(pts[-1])
    return np.array(out)


# ----------------------------------------------------------------------------- families
def gaussian_blobs(rng, area, n_peaks=10, sigma=(200, 400), amp=(0.4, 0.4), margin_frac=0.12):
    """Independent blobs, uniformly placed (the stack's default generator)."""
    size = area[1] - area[0]
    ns, es = _uniform_xy(rng, area, margin_frac * size, n_peaks)
    return [_bump(n, e, rng.uniform(*sigma), rng.uniform(*sigma), rng.uniform(*amp)) for n, e in zip(ns, es)], 0.0


def clustered(rng, area, n_clusters=3, per_cluster=5, cluster_radius=500, sigma=(100, 250), amp=(0.2, 0.5)):
    """Groups of small blobs: the route must choose groups, then cover them."""
    size = area[1] - area[0]
    cn, ce = _uniform_xy(rng, area, 0.15 * size + cluster_radius, n_clusters)
    bumps = []
    for n, e in zip(cn, ce):
        for _ in range(per_cluster):
            r, t = cluster_radius * math.sqrt(rng.uniform()), rng.uniform(0, 2 * math.pi)
            bumps.append(_bump(n + r * math.cos(t), e + r * math.sin(t), rng.uniform(*sigma), rng.uniform(*sigma),
                               rng.uniform(*amp)))
    return bumps, 0.0


def heavy_tailed(rng, area, n_peaks=20, sigma=(120, 350), alpha=1.2, amp_max=0.6):
    """Pareto amplitudes: a few dominant peaks among many weak ones."""
    size = area[1] - area[0]
    ns, es = _uniform_xy(rng, area, 0.1 * size, n_peaks)
    raw = rng.pareto(alpha, n_peaks) + 1.0
    amps = amp_max * raw / raw.max()
    return [_bump(n, e, rng.uniform(*sigma), rng.uniform(*sigma), a) for n, e, a in zip(ns, es, amps)], 0.0


def decoy(rng, area, home=(0.0, 0.0), n_decoys=10, decoy_amp=0.25, decoy_sigma=(100, 200), decoy_radius=900,
          far_peaks=3, far_amp=0.5, far_sigma=(200, 350), far_dist=1700):
    """Many weak blobs near home and one heavy group far away: punishes short-sighted routes."""
    bumps = []
    for _ in range(n_decoys):
        r, t = decoy_radius * math.sqrt(rng.uniform(0.05, 1)), rng.uniform(0, 2 * math.pi)
        bumps.append(_bump(home[0] + r * math.cos(t), home[1] + r * math.sin(t), rng.uniform(*decoy_sigma),
                           rng.uniform(*decoy_sigma), decoy_amp * rng.uniform(0.6, 1.0)))
    t0 = rng.uniform(0, 2 * math.pi)
    n0, n1, e0, e1 = _inside(area, 300)
    cn = float(np.clip(home[0] + far_dist * math.cos(t0), n0, n1))
    ce = float(np.clip(home[1] + far_dist * math.sin(t0), e0, e1))
    for _ in range(far_peaks):
        r, t = 350 * math.sqrt(rng.uniform()), rng.uniform(0, 2 * math.pi)
        bumps.append(_bump(cn + r * math.cos(t), ce + r * math.sin(t), rng.uniform(*far_sigma),
                           rng.uniform(*far_sigma), far_amp * rng.uniform(0.8, 1.0)))
    return bumps, 0.0


def lines(rng, area, n_lines=3, vertices=6, step_len=700, turn_sd=0.5, width=70, amp=(0.15, 0.3), spacing=None):
    """Roads / rivers / ridgelines: belief concentrated along wandering polylines."""
    spacing = spacing or 1.2 * width
    bumps = []
    for _ in range(n_lines):
        a = rng.uniform(*amp)
        for p in _along(_polyline(rng, area, 250, vertices, step_len, turn_sd), spacing):
            bumps.append(_bump(p[0], p[1], width, width, a))
    return bumps, 0.0


def ring(rng, area, n_rings=1, radius=(700, 1200), width=120, amp=0.25, spacing=None, gap_frac=0.0):
    """Annuli around a last-known point (range-only information)."""
    spacing = spacing or 1.2 * width
    size = area[1] - area[0]
    bumps = []
    for _ in range(n_rings):
        R = rng.uniform(*radius)
        cn, ce = _uniform_xy(rng, area, min(R + 150, 0.45 * size), 1)
        k = max(8, int(2 * math.pi * R / spacing))
        start = rng.uniform(0, 2 * math.pi)
        for i in range(k):
            t = start + 2 * math.pi * i / k
            if gap_frac > 0 and (i / k) < gap_frac:
                continue
            bumps.append(_bump(cn[0] + R * math.cos(t), ce[0] + R * math.sin(t), width, width, amp))
    return bumps, 0.0


def drift_plume(rng, area, n_plumes=1, length=2500, sigma0=80, growth=0.12, amp0=0.6, decay=0.6, spacing=150):
    """Search-and-rescue drift: a plume from a last-known point, widening and fading downstream."""
    bumps = []
    for _ in range(n_plumes):
        n0, n1, e0, e1 = _inside(area, 300)
        p = np.array([rng.uniform(n0, n1), rng.uniform(e0, e1)])
        centre = np.array([(n0 + n1) / 2, (e0 + e1) / 2])
        # drift roughly toward the area centre, so the plume stays on the map
        h = math.atan2(centre[1] - p[1], centre[0] - p[0]) + rng.uniform(-1.0, 1.0)
        d = 0.0
        while d < length:
            s = sigma0 + growth * d
            a = amp0 * math.exp(-decay * d / length)
            q = p + d * np.array([math.cos(h), math.sin(h)])
            if not (n0 <= q[0] <= n1 and e0 <= q[1] <= e1):
                break
            bumps.append(_bump(q[0], q[1], s, s, a))
            h += rng.normal(0, 0.05)
            d += spacing
    return bumps, 0.0


def random_field(rng, area, lattice=12, sigma_frac=0.7, lognorm_sd=1.0, amp=0.3, margin=150):
    """A smooth random field: a lattice of overlapping bumps with log-normal amplitudes."""
    n0, n1, e0, e1 = _inside(area, margin)
    ns, es = np.linspace(n0, n1, lattice), np.linspace(e0, e1, lattice)
    step = (n1 - n0) / (lattice - 1)
    w = rng.lognormal(0.0, lognorm_sd, (lattice, lattice))
    w = amp * w / w.max()
    return [_bump(n, e, sigma_frac * step, sigma_frac * step, w[i, j])
            for i, n in enumerate(ns) for j, e in enumerate(es)], 0.0


def diffuse_plus_peaks(rng, area, n_peaks=4, sigma=(150, 300), amp=(0.3, 0.5), floor_frac=0.15):
    """A uniform background (base_uncertainty) under a few peaks: much of the mass is everywhere."""
    bumps, _ = gaussian_blobs(rng, area, n_peaks=n_peaks, sigma=sigma, amp=amp)
    return bumps, floor_frac * max(b["amplitude"] for b in bumps)


def multi_scale(rng, area, n_wide=2, wide_sigma=(700, 1100), wide_amp=0.12, n_sharp=10, sharp_sigma=(50, 110),
                sharp_amp=(0.3, 0.6)):
    """Wide, weak regions with sharp strong peaks scattered through the map."""
    a, _ = gaussian_blobs(rng, area, n_peaks=n_wide, sigma=wide_sigma, amp=(wide_amp, wide_amp), margin_frac=0.2)
    b, _ = gaussian_blobs(rng, area, n_peaks=n_sharp, sigma=sharp_sigma, amp=sharp_amp)
    return a + b, 0.0


def _r(rng, lo, hi, integer=False):
    v = rng.uniform(lo, hi)
    return int(round(v)) if integer else float(v)


# name -> (function, parameter sampler)
FAMILIES: dict[str, tuple[Callable, Callable[[Any], dict]]] = {
    "gaussian_blobs": (gaussian_blobs, lambda g: {
        "n_peaks": int(g.choice([2, 4, 8, 12, 20, 32])),
        "sigma": [(60, 120), (150, 300), (200, 400), (400, 700)][int(g.choice(4, p=[.25, .25, .3, .2]))],
        "amp": [(0.4, 0.4), (0.1, 0.8)][int(g.integers(2))]}),
    "clustered": (clustered, lambda g: {
        "n_clusters": int(g.integers(2, 6)), "per_cluster": int(g.integers(3, 9)),
        "cluster_radius": _r(g, 250, 700), "sigma": (80, _r(g, 120, 300))}),
    "heavy_tailed": (heavy_tailed, lambda g: {
        "n_peaks": int(g.integers(10, 40)), "alpha": _r(g, 0.7, 2.0), "sigma": (100, _r(g, 200, 450))}),
    "decoy": (decoy, lambda g: {
        "n_decoys": int(g.integers(6, 16)), "decoy_amp": _r(g, 0.15, 0.4),
        "far_dist": _r(g, 1300, 2300), "far_peaks": int(g.integers(2, 5))}),
    "lines": (lines, lambda g: {
        "n_lines": int(g.integers(1, 5)), "width": _r(g, 40, 140), "turn_sd": _r(g, 0.1, 0.9),
        "vertices": int(g.integers(3, 9))}),
    "ring": (ring, lambda g: {
        "n_rings": int(g.integers(1, 3)), "radius": (600, _r(g, 800, 1500)), "width": _r(g, 60, 200),
        "gap_frac": float(g.choice([0.0, 0.0, 0.3, 0.5]))}),
    "drift_plume": (drift_plume, lambda g: {
        "n_plumes": int(g.integers(1, 3)), "length": _r(g, 1500, 3500), "growth": _r(g, 0.05, 0.25),
        "decay": _r(g, 0.2, 1.5)}),
    "random_field": (random_field, lambda g: {
        "lattice": int(g.integers(6, 16)), "lognorm_sd": _r(g, 0.5, 1.8), "sigma_frac": _r(g, 0.45, 0.9)}),
    "diffuse_plus_peaks": (diffuse_plus_peaks, lambda g: {
        "n_peaks": int(g.integers(2, 10)), "floor_frac": _r(g, 0.03, 0.3)}),
    "multi_scale": (multi_scale, lambda g: {
        "n_wide": int(g.integers(1, 4)), "n_sharp": int(g.integers(4, 20))}),
}


def make_prior(family: str, rng, area, home=(0.0, 0.0), **params):
    fn, _ = FAMILIES[family]
    if family == "decoy":
        params.setdefault("home", home)
    bumps, floor = fn(rng, area, **params)
    return bumps, float(floor), params


def sample_family_params(family: str, rng) -> dict:
    return FAMILIES[family][1](rng)


def rasterize(bumps, floor, area, res=10.0, cap=0.85):
    """The prior raster every consumer rebuilds (normalised to sum to 1). Rows = north, cols = east."""
    n0, n1, e0, e1 = area
    na = np.arange(n0, n1 + 1e-9, res)
    ea = np.arange(e0, e1 + 1e-9, res)
    V = np.zeros((na.size, ea.size))
    for b in bumps:
        gn = np.exp(-0.5 * ((na - b["n"]) / b["sigma_n"]) ** 2)
        ge = np.exp(-0.5 * ((ea - b["e"]) / b["sigma_e"]) ** 2)
        m = gn > 1e-12
        V[m] += b["amplitude"] * np.outer(gn[m], ge)
    V = np.minimum(V, cap)
    if floor > 0:
        V = np.maximum(V, floor)
    s = V.sum()
    if not s > 0:
        raise ValueError("prior has no mass")
    return na, ea, V / s
