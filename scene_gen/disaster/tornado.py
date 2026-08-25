"""tornado stage — the TRACK, and everything that hangs off it.

WHAT A TORNADO SCENE IS, AND HOW IT DIFFERS FROM THE WILDFIRE ONE
----------------------------------------------------------------
The wildfire pipeline is driven by ONE scalar — burn age, the time since an
elliptical front reached a point — and `damage.level_for_age` turns that into a
structural level, a finish and a fire state together. A tornado is driven by
one scalar too, but it is a different one and it is not a clock:

    intensity(x, y)   0..1, an EF proxy: how hard the wind hit this point

and it is a function of DISTANCE FROM A LINE rather than of distance from a
point. That single change is most of what makes the two scenes look nothing
alike from the air:

  * a burn scar is a lobe that grows outward from an ignition point, so its
    damage gradient is radial and its edge is a conic;
  * a tornado track is a CORRIDOR — total destruction on the centreline,
    near-nothing a hundred metres either side, and the transition happens
    over a few tens of metres. `tornado.jpeg` in the repo root is the
    reference: an intact green suburb with a swathe cut through it.

THE SECOND DIFFERENCE IS DIRECTION
----------------------------------
A burnt house collapses into its own footprint; gravity is the only vector in
the scene. A wrecked house is BLOWN, and everything that came off it lies
downtrack of where it started. So this module publishes a `throw` field
alongside the intensity one, and every debris pass takes it: fallen trees lie
pointing one way, plank debris trails away from each slab in one direction,
and the whole corridor reads as having been swept by something that was
travelling rather than by something that was burning.

`throw` is the heading rotated by `curl_deg` toward the LEFT of travel, not the
heading itself. Damage surveys put the deposition zone left of the track for a
cyclonic vortex — the rotational and translational components add on the right
flank and oppose on the left, so material lofted on the strong side is carried
across and dropped on the weak one. It is also the more useful default for
looking at a scene: debris that trails exactly along the centreline is hidden
by the centreline, and debris at a slight angle to it is legible.

NO FIRE, ANYWHERE
-----------------
Nothing here chars, scorches, soots or consumes. `disaster.damage`'s material
passes are all fire passes and none of them is called on this path — the
wreckage is bare structural timber and it takes a plain wood material
(`planks.wood_material`). The one place that needed saying out loud is
`vegetation.burn_tree`, whose char is gated on its `_PLAN` table rather than on
an argument; `tornado_plan()` below is the table that turns it off.

THE GROUND
----------
Scour: the turf is peeled off the centreline and the ground under a track is
bare wet soil. That is the same problem the burn scar solves and it takes the
same machinery — `disaster.ground.build_overlay` with a `coverage_at` from
`scour_coverage()` below and the `Soil_Mud` megascans surface instead of the
burnt floor. Darkest on the centreline, fading out to nothing at the path
edge, which is exactly what `build_overlay`'s opacity bands already do given a
coverage field that peaks in the middle.

READ `build-wildfire-scenes` FIRST. Every trap in that skill's ground section
applies here unchanged: the fractional-cutout Kit flag, re-asserting it after
a stage load, one tile rather than many, greedy-meshed bands so a translucent
overlay does not print its own lattice.
"""

import math
import os

import numpy as np

# The scour surface. Same coupling as `ground.BURNT_TEXTURE`: the resolution is
# in the filename, `tools/import_megascans.py` REPLACES a surface in place, and
# a stale path here draws untextured rather than failing. Re-point after an
# import; `pjuph20` is the surface id and only the `_1K_` changes.
MUD_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
               "Soil_Mud/T_pjuph20_1K_B.jpg")

# The low-level `disaster.tornado` config block, and what each knob is measured
# against. `compile_disaster.compile_tornado` writes this; a launcher merges it
# over these defaults so an older compiled config still runs.
DEFAULTS = {
    "enabled": True,
    "seed": 0,

    # A point ON the centreline, in the same metric frame as the placements
    # (region centred on the origin), and the direction of travel.
    "origin_m": [0.0, 0.0],
    "heading_deg": 35.0,

    # WIDTH OF THE DAMAGE PATH, tip to tip. US tornado paths run from tens of
    # metres to well over a kilometre; the median significant (EF2+) track is
    # a few hundred. On a 500 m plate a 150 m path is the value that leaves
    # intact suburb on BOTH sides — which is the whole read, since a corridor
    # you cannot see the edges of is just a damaged scene.
    "width_m": 150.0,
    # Share of the half-width that is at full intensity before the falloff
    # starts. 0 gives a triangular profile (a ridge), 1 a top-hat with a hard
    # edge. 0.30 is a core with shoulders, which is what a damage survey's
    # contours look like.
    "core_frac": 0.30,

    # THE TRACK MEANDERS. A tornado does not travel in a straight line and a
    # perfectly straight corridor is the single most artificial thing this
    # scene can show. Two harmonics rather than one, for the reason
    # `vegetation.scar_patch` uses three: one sine is a recognisable shape and
    # the eye locks onto it.
    "wobble_m": 22.0,
    "wobble_period_m": 340.0,

    # AND IT BREATHES. Width and intensity both vary along the track, out of
    # phase, so the corridor pinches and widens and there are weak stretches
    # where a house survives inside it. Real tracks are documented this way —
    # EF ratings along one path routinely swing two categories.
    "along_min": 0.55,
    "along_period_m": 460.0,
    "width_min": 0.70,

    # Peak intensity on the centreline at the strongest point, 0..1, as an EF
    # proxy: ~0.35 = EF1, ~0.55 = EF2, ~0.75 = EF3, ~0.90 = EF4, 1.0 = EF5.
    # `severity` in the high-level spec is what moves this.
    "peak": 0.92,

    # Edge noise, in metres of lateral wander added to the falloff. Perturbs
    # the LEVEL SET, never the value — see `intensity_field`.
    "edge_noise_m": 26.0,

    # WHERE THE DEBRIS GOES. `curl_deg` is measured from the heading toward
    # the left of travel (+ = left); `spread_deg` is the per-object scatter
    # about that. `throw_m` is how far a piece travels at intensity 1.
    "curl_deg": 20.0,
    "spread_deg": 34.0,
    "throw_m": 26.0,
}


# ---------------------------------------------------------------------------
# the track frame
# ---------------------------------------------------------------------------

def resolve_cfg(config):
    """`DEFAULTS` with the scene config's `disaster.tornado` block merged over.

    Same contract as `fire.DEFAULTS` + the launcher's `fcfg` dance, in one
    place so three launchers cannot drift apart on it.
    """
    cfg = dict(DEFAULTS)
    cfg.update((config.get("disaster") or {}).get("tornado") or {})
    return cfg


def frame(cfg):
    """`to_track(x, y) -> (along_m, cross_m)` and the unit vectors.

    `along` is metres downtrack of `origin_m`, `cross` is metres LEFT of the
    centreline — after the meander, so `cross` is distance from where the
    vortex actually was rather than from the straight line through the origin.
    Returns `(to_track, (ux, uy), (vx, vy))`.
    """
    ox, oy = (float(cfg["origin_m"][0]), float(cfg["origin_m"][1]))
    th = math.radians(float(cfg["heading_deg"]))
    ux, uy = math.cos(th), math.sin(th)
    vx, vy = -math.sin(th), math.cos(th)          # left of travel
    amp = float(cfg.get("wobble_m", 0.0))
    per = max(1e-6, float(cfg.get("wobble_period_m", 340.0)))

    def wobble(a):
        """Lateral offset of the centreline at `a` metres downtrack."""
        if amp <= 0.0:
            return 0.0
        return amp * (0.62 * math.sin(2.0 * math.pi * a / per)
                      + 0.38 * math.sin(2.0 * math.pi * a / (per * 0.37)
                                        + 1.71))

    def to_track(x, y):
        dx, dy = float(x) - ox, float(y) - oy
        a = dx * ux + dy * uy
        c = dx * vx + dy * vy
        return a, c - wobble(a)

    return to_track, (ux, uy), (vx, vy)


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def _edge_noise(region, rng, n=256, band_m=(30.0, 110.0)):
    """A seamless -1..1 sampler over `region`, band-limited to `band_m`.

    Same construction (and the same reasons) as `ground.edge_fields`: spectral,
    so it tiles and has no lattice, and band-limited so the corridor's edge
    fingers at house-lot scale rather than either wobbling as one shape or
    breaking into per-cell confetti.
    """
    from . import scorch

    x0, y0, x1, y1 = region
    w, h = float(x1 - x0), float(y1 - y0)
    px = max(w, h) / float(n)
    lo, hi = px / float(band_m[1]), px / float(band_m[0])
    f = scorch._noise(rng, n, n, beta=2.0, lo=lo, hi=hi) * 2.0 - 1.0

    def sample(x, y):
        i = min(n - 1, max(0, int((y - y0) / h * n)))
        j = min(n - 1, max(0, int((x - x0) / w * n)))
        return float(f[i, j])

    return sample


def intensity_field(cfg, region, rng):
    """`(x, y) -> 0..1`, the EF proxy every other pass in this scene reads.

    1 on the centreline at the track's strongest point, 0 outside the path.
    Three things shape it:

      * the CROSS-TRACK profile — flat across `core_frac` of the half-width,
        then a smoothstep to zero at the edge;
      * the ALONG-TRACK modulation — width and strength both breathe, out of
        phase, so the corridor pinches and there are weak stretches inside it;
      * band-limited noise on the EDGE.

    THE NOISE MOVES THE BOUNDARY, IT DOES NOT ADD TO THE VALUE. Writing
    `i * k + (noise - 0.5) * m` is the obvious way to perturb a field and it
    is wrong in exactly the way `ground.py` records: where `i` is zero the
    noise still contributes, so ground the tornado never touched comes out
    speckled with damage. Here the noise is added to the CROSS-TRACK DISTANCE
    before the profile is evaluated, so it can only move the edge in and out —
    outside the widened path the profile is identically zero.
    """
    to_track, _u, _v = frame(cfg)
    half = max(1e-6, 0.5 * float(cfg["width_m"]))
    core = min(0.98, max(0.0, float(cfg.get("core_frac", 0.3))))
    peak = float(cfg.get("peak", 0.92))
    a_min = float(cfg.get("along_min", 0.55))
    w_min = float(cfg.get("width_min", 0.70))
    a_per = max(1e-6, float(cfg.get("along_period_m", 460.0)))
    noise_m = float(cfg.get("edge_noise_m", 0.0))
    noise = (_edge_noise(region, rng) if noise_m > 0.0
             else (lambda x, y: 0.0))

    def _breathe(a, period, lo, phase):
        """`lo`..1 along the track, two harmonics, never periodic-looking."""
        s = (0.66 * math.sin(2.0 * math.pi * a / period + phase)
             + 0.34 * math.sin(2.0 * math.pi * a / (period * 0.43)
                               + phase * 1.9))
        return lo + (1.0 - lo) * (0.5 + 0.5 * max(-1.0, min(1.0, s)))

    def intensity(x, y):
        a, c = to_track(x, y)
        hw = half * _breathe(a, a_per * 1.31, w_min, 0.0)
        q = (abs(c) + noise_m * noise(x, y)) / hw
        if q >= 1.0:
            return 0.0
        if q <= core:
            prof = 1.0
        else:
            prof = 1.0 - _smoothstep((q - core) / (1.0 - core))
        return peak * prof * _breathe(a, a_per, a_min, 2.4)

    return intensity


def throw_field(cfg):
    """`(x, y, rng) -> (dx, dy, dist_m)`: which way debris here was blown.

    Direction is the heading rotated `curl_deg` toward the left of travel,
    jittered by `spread_deg` per object so a debris field fans rather than
    striping. Distance scales super-linearly with intensity, because the
    thing being modelled is a wind speed and the drag on a tumbling board
    goes as its square — a house at the edge of the path loses its shingles
    into the next garden, one on the centreline is found a field away.

    The caller passes the intensity it already computed rather than this
    recomputing it: every caller has it, and evaluating the field twice per
    object across a plat's worth of debris is not free.
    """
    th = math.radians(float(cfg["heading_deg"]) + float(cfg.get("curl_deg", 0.0)))
    spread = math.radians(float(cfg.get("spread_deg", 30.0)))
    reach = float(cfg.get("throw_m", 26.0))

    def throw(intensity, rng):
        a = th + rng.uniform(-spread, spread)
        i = max(0.0, min(1.0, float(intensity)))
        d = reach * (i ** 1.6) * (0.25 + 0.75 * rng.random())
        return math.cos(a), math.sin(a), d

    return throw


# ---------------------------------------------------------------------------
# damage levels
# ---------------------------------------------------------------------------

# THE HOUSE LADDER, as an EF proxy. Names are what the thing looks like, not
# what caused it, because the fire ladder's names (`scorched`, `burned_out`)
# describe a cause that is absent here and a bake keyed on them would be
# actively misleading to the next reader.
#
#   roof_stripped     covering and sheathing gone, structure and walls up
#                     (EF0-EF1: the most common damage in any track)
#   roof_collapsed    roof structure down, walls up
#   partial_collapse  exterior walls failed on the windward side
#   leveled           reduced to a debris pile standing on its own slab
#   swept             slab wiped clean, everything downtrack (EF4-EF5)
#
# The thresholds are deliberately NOT evenly spaced: the profile spends most of
# its cross-track distance in the shoulders, so evenly-spaced cuts put almost
# every house in the two lightest levels and the corridor loses its core.
HOUSE_LEVELS = ("pristine", "roof_stripped", "roof_collapsed",
                "partial_collapse", "leveled", "swept")

# THE LIGHT CLASSES GET THE WIDEST BANDS, and that is the shape of a real
# damage survey rather than an even split. EF0-EF1 damage — covering off,
# soffits out, a garage door in — extends well past the swathe anything else
# can be seen in, and by count it is most of what a tornado does. Splitting
# 0..1 evenly gave 62% of the damaged houses in one class, which reads as a
# corridor with a hard edge and nothing outside it.
_HOUSE_CUTS = ((0.08, "pristine"),
               (0.36, "roof_stripped"),
               (0.54, "roof_collapsed"),
               (0.70, "partial_collapse"),
               (0.87, "leveled"),
               (1.01, "swept"))

# THE TREE LADDER. Every level here keeps its leaves — that is the point, and
# it is the sharpest single difference from the burnt stand. A fire consumes
# fine fuel and leaves black poles; a tornado breaks WOOD and the foliage on
# what it broke off is still green when the photograph is taken.
#
#   limbed     standing, crown thinned, a few limbs on the ground
#   leaning    root-sprung, tipped 20-40 deg, entirely intact
#   fallen     uprooted, whole tree on the ground with its crown
#   snapped    bole broken partway up, standing stub, crown gone
#
# `snapped` is the one level that loses its foliage, and that is correct: a
# snapped bole's crown is somewhere else entirely. It is also the rarest,
# because it takes the highest wind — see `tree_level_for_intensity`.
TREE_LEVELS = ("pristine", "limbed", "leaning", "fallen", "snapped")

# Same shape, shifted down: a tree comes apart at a lower wind than a house
# does. The threshold that matters is `fallen` — a windthrown tree with its
# crown on the ground is the most legible single feature of a track from the
# air, and it wants to be the majority outcome inside the corridor.
# SPECIES WHOSE CROWN IS TOO WIDE TO LIE DOWN, so they SNAP instead of
# uprooting. Not a fudge — it is the real bias, and the numbers are why:
# `tip_tree` seats a windthrown tree by bisecting its lean until its lowest
# point is just into the turf, and measured across the library that works for
# every species except Black_Oak, whose 25.4 m crown is still 8.2 m below
# grade at the shallowest lean the level allows. A tree that large genuinely
# does tend to fail in the STEM rather than at the roots: its root plate is
# enormous and its trunk section is what runs out of capacity first.
#
# The effect is small in practice — the suburban asset set plants Black_Oak
# only in open ground (`suburb_tornado.yaml` keeps it out of the frontage
# pool, where a 25 m canopy sits on the roof), so this promotes a handful of
# park specimens and nothing on the streets.
NO_UPROOT = ("Black_Oak",)

_TREE_CUTS = ((0.07, "pristine"),
              (0.30, "limbed"),
              (0.46, "leaning"),
              (0.80, "fallen"),
              (1.01, "snapped"))


def _ladder(cuts, i, rng, jitter):
    """Pick a level from `cuts` for intensity `i`, with a per-object jitter.

    JITTER IS WHAT STOPS THE LEVELS FROM BEING CONTOUR LINES. Without it every
    boundary in the ladder is a clean curve parallel to the track — five
    stripes of identical houses, which reads as a gradient map rather than as
    damage. One draw per object, applied to the intensity before the lookup,
    is the same trick `damage` gets for free from the fire's arrival jitter.
    """
    v = float(i) + (rng.uniform(-jitter, jitter) if jitter > 0.0 else 0.0)
    for lim, name in cuts:
        if v < lim:
            return name
    return cuts[-1][1]


def house_level_for_intensity(i, rng, jitter=0.07):
    """Structural level for a house standing where intensity is `i`."""
    return _ladder(_HOUSE_CUTS, i, rng, jitter)


def tree_level_for_intensity(i, rng, jitter=0.09, species=None):
    """Damage level for a tree standing where intensity is `i`.

    A LARGER JITTER THAN THE HOUSES GET, on purpose. Whether a given tree goes
    over is decided by its rooting, its lean, its exposure and what its
    neighbours did, none of which this scene models — and a stand where every
    tree at a given distance did the same thing is the one thing that never
    happens. Same argument the wildfire skill makes for severity being a
    property of the STAND, run the other way.

    `species` promotes `fallen` to `snapped` for the wide-crowned species —
    see `NO_UPROOT`. Omit it and the ladder is returned unmodified, which is
    what a caller that has no species to hand (the host-side plan against a
    placement with no `usd`) should get.
    """
    lv = _ladder(_TREE_CUTS, i, rng, jitter)
    if lv == "fallen" and species and str(species) in NO_UPROOT:
        return "snapped"
    return lv


# `_PLAN`-shaped override for `vegetation.burn_tree`, which reads its level
# table rather than its arguments — see that module's `_PLAN`.
# `(keep_base, keep_top, bole_cov, browning, scorch_h, geometry)`; the two
# zeros in the middle are what turn the char and the leaf-browning passes off,
# and `keep_* = 1.0` keeps the crown.
_TORNADO_PLAN = {
    "pristine": (1.00, 1.00, 0.00, 0.00, 0.00, None),
    "limbed":   (0.78, 0.62, 0.00, 0.00, 0.00, None),
    "leaning":  (1.00, 1.00, 0.00, 0.00, 0.00, "lean"),
    "fallen":   (1.00, 1.00, 0.00, 0.00, 0.00, "uproot"),
    "snapped":  (1.00, 1.00, 0.00, 0.00, 0.00, "snap"),
}


def tornado_plan(level):
    """The `vegetation._PLAN` tuple for a tornado tree level."""
    return _TORNADO_PLAN.get(level, _TORNADO_PLAN["pristine"])


# ---------------------------------------------------------------------------
# the ground scour
# ---------------------------------------------------------------------------

def knobs_from_env(span_m):
    """Scour overlay knobs from MUD_* env vars, defaults derived from the plate.

    Deliberately parallel to `ground.knobs_from_env` so what was tuned on one
    means the same on the other, with two differences that matter:

      * `tile_m` is NOT the whole plate. `ground` projects one tile across the
        scar because the burnt floor is a 4K map and a burn covers everything;
        the mud pack is 1K, and one tile across 500 m is half a metre a pixel,
        which is mush at any altitude worth flying. A ~45 m tile is 4.4 cm a
        pixel and repeats about a dozen times along a track — acceptable
        because the corridor is a ragged band rather than a full plate, so
        there is no regular grid for the eye to lock onto.
      * more bands, because the whole read here IS the cross-track gradient,
        and a coarse quantisation of it shows as stripes parallel to the track.
    """
    def _f(name, default):
        return float(os.environ.get(name, default))

    return dict(
        cell_m=_f("MUD_CELL", "3.0"),
        bands=int(os.environ.get("MUD_BANDS", "14")),
        tile_m=_f("MUD_TILE_M", "45.0") or None,
        op_range=(_f("MUD_OPACITY_MIN", "0.10"),
                  _f("MUD_OPACITY_MAX", "0.94")),
        # Islands are turf the vortex skipped. Unlike the burn scar's, these
        # are worth having: scour is genuinely patchy at the edges of a path
        # and a few surviving green patches inside the corridor is what a real
        # one looks like from above.
        islands=_f("MUD_ISLANDS", "0.05"),
        gamma=_f("MUD_GAMMA", "0.85"),
    )


def scour_coverage(cfg, region, rng, intensity=None, gamma=0.85, islands=0.05):
    """`(x, y) -> 0..1` for `ground.build_overlay`: the mud along the track.

    Darkest on the centreline and fading to nothing at the path edge, which is
    what the request asks for and also what the overlay's opacity bands do
    given a field shaped like this one — `build_overlay` buckets coverage and
    maps the bucket onto `op_range`, so a coverage field that peaks in the
    middle IS an opacity that peaks in the middle.

    `gamma < 1` pushes coverage up, which WIDENS the visibly muddy band
    without widening the structural damage: scour reaches further out than
    collapse does, because peeling turf takes less wind than failing a wall.
    `islands` removes compact patches of surviving turf, as a quantile of a
    band-limited field so the share of area is by construction.
    """
    inten = intensity if intensity is not None else intensity_field(
        cfg, region, rng)
    # A SECOND, INDEPENDENT rng draw for the islands. Sharing the edge noise's
    # generator would correlate the islands with the corridor's own edge
    # wobble, and they would all appear on the same side.
    _f, isl = _island_field(region, rng, islands)
    g = max(1e-3, float(gamma))

    def coverage_at(x, y):
        i = inten(x, y)
        if i <= 0.0:
            return 0.0
        return min(1.0, (i ** g)) * (1.0 - isl(x, y))

    return coverage_at


def _island_field(region, rng, islands, n=256, band_m=(18.0, 55.0)):
    """`(finger, island)` samplers — the island half of `ground.edge_fields`.

    Kept here rather than reused from `ground` because the bands differ: turf
    the vortex skipped is a smaller feature than fuel a fire skipped (a fire
    skips a wet hollow, a tornado skips because its own suction spots missed),
    and `ground.edge_fields` bakes its 20-60 m island band in.
    """
    from . import scorch

    x0, y0, x1, y1 = region
    w, h = float(x1 - x0), float(y1 - y0)
    px = max(w, h) / float(n)
    lo, hi = px / float(band_m[1]), px / float(band_m[0])
    raw = scorch._noise(rng, n, n, beta=2.0, lo=lo, hi=hi)
    if islands > 0.0:
        thr = float(np.quantile(raw, 1.0 - min(0.9, float(islands))))
        e = np.clip((raw - thr) / 0.05, 0.0, 1.0)
        f = e * e * (3.0 - 2.0 * e)
    else:
        f = np.zeros_like(raw)

    def sample(x, y):
        i = min(n - 1, max(0, int((y - y0) / h * n)))
        j = min(n - 1, max(0, int((x - x0) / w * n)))
        return float(f[i, j])

    return None, sample


# ---------------------------------------------------------------------------
# host-side reporting
# ---------------------------------------------------------------------------

def summarise(cfg, region, rng, n=64):
    """Sample the field on an n x n grid and describe it. No Isaac Sim needed.

    The tornado analogue of `tools/fire_png.py`'s plan: the numbers that decide
    whether a scene is worth a container launch are the share of the plate
    inside the path and the spread of levels across it, and both are cheap.
    A track that misses the fabric, or one wide enough to level everything, is
    visible here in a second rather than in twenty minutes.
    """
    inten = intensity_field(cfg, region, rng)
    x0, y0, x1, y1 = region
    xs = [x0 + (k + 0.5) * (x1 - x0) / n for k in range(n)]
    ys = [y0 + (k + 0.5) * (y1 - y0) / n for k in range(n)]
    vals = [inten(x, y) for x in xs for y in ys]
    hit = [v for v in vals if v > 0.0]
    return {
        "cells": len(vals),
        "in_path_frac": round(len(hit) / float(len(vals)), 4),
        "mean_intensity": round(sum(hit) / len(hit), 4) if hit else 0.0,
        "max_intensity": round(max(vals), 4),
    }
