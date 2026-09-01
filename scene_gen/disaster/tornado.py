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

    # A GENTLE ARC ON TOP OF THE WOBBLE. Real tracks curve as well as
    # meander — the parent low pressure system steers the vortex along a
    # slowly turning heading over a km-scale window, which the two-harmonic
    # wobble above (a drift about a fixed heading) does not reach on its own.
    # Degrees of heading change per kilometre of `along`, + = the track turns
    # LEFT of travel as `along` increases. 0.0 is the default because every
    # existing scene was built and reviewed on a straight-plus-wobble track;
    # this is additive so nothing already shipped moves. Implemented inside
    # `_wobble` as another `along`-only lateral offset, so `frame`'s
    # `to_track` and `from_track` stay EXACT inverses of each other — see
    # `_wobble`'s docstring for why that matters.
    "curvature_deg_per_km": 0.0,

    # AND IT BREATHES. Width and intensity both vary along the track, out of
    # phase, so the corridor pinches and widens and there are weak stretches
    # where a house survives inside it. Real tracks are documented this way —
    # EF ratings along one path routinely swing two categories.
    "along_min": 0.55,
    "along_period_m": 460.0,
    "width_min": 0.70,

    # A FINITE TRACK. `None` (both) is the default and means what it always
    # meant: the track spans the whole plate, `along` never gates anything.
    # Set BOTH to put a touchdown and a liftoff on the plate — `along_m`
    # coordinates relative to `origin_m`, in the direction `heading_deg`
    # points. Between them the field ramps up from 0 over `ramp_m` after
    # `touchdown_m` and ropes out — width AND intensity taper together —
    # over the `ramp_m` before `liftoff_m`. Outside `[touchdown_m,
    # liftoff_m]` the field is EXACTLY 0, not asymptotically small: a track
    # that touched down mid-scene should leave houses behind it genuinely
    # untouched, not lightly grazed by a long tail.
    "touchdown_m": None,
    "liftoff_m": None,
    # How long the ramp-up / rope-out takes, in metres of `along`. 120 m is
    # about a third of a weak tornado's own width — long enough to read as a
    # spin-up rather than a light switch, short enough that most of a short
    # track is still at full strength.
    "ramp_m": 120.0,

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

    # `wind_at`'s NEAR-SURFACE WIND MODEL (plan `urban_tornado_plan.md`
    # 2.4) — the tangential/translation/inflow mix, Karstens et al. 2013 /
    # Beck & Dotzek 2010. New knobs, no existing caller reads `wind_at` yet,
    # so these are free to retune; a later research pass may move them.
    #
    # Share of the vortex's own translational motion in the total wind,
    # relative to `peak`. 1/Gmax, where Gmax is the ratio of peak tangential
    # speed to translation speed: NIST's own Rankine-vortex fit to the Joplin
    # tree-fall field (NCSTAR 3, 2014) gives Gmax 4.5-5.0 -> 0.20-0.22; a
    # five-tornado cross-check spans 0.15-0.67 (weak, fast movers at the top).
    # 0.22 is the violent-tornado end, which is the regime an urban EF3-5
    # core scene represents. See `.agents/skills/model-tornado-paths`.
    "translation_frac": 0.22,
    # Share of the tangential magnitude that becomes convergent inflow
    # toward the centreline. THE NEAR-SURFACE FLOW IS MOSTLY RADIAL: NIST's
    # Joplin fit has alpha = 15-25 deg (0 = radial, 90 = tangential) below
    # 20 m AGL, i.e. inflow 2-3.7x the tangential component, and Karstens et
    # al. 2013's independent tree-fall fit is "on the order of 2:1". But it
    # is height-dependent — Kosiba & Wurman's DOW boundary-layer work finds
    # the inflow confined to the lowest 10-14 m and gone by ~30 m AGL — and
    # `wind_at` is one 2-D value for a whole building, so 0.5 is a deliberate
    # compromise between the ground floor (~2.0) and a tower's upper storeys
    # (~0.3), not the empirical value at either. A z-aware `wind_at` is the
    # follow-up; the anchor numbers are here for it.
    "inflow_frac": 0.50,
    # |cross_frac| below which a point is UNDER the vortex core rather than
    # on one flank: every face was loaded in turn as the core passed over,
    # so no single bearing is "the" windward one. `None` (the default) ties
    # it to `core_frac` — the model's own definition of "inside the flat
    # core" — rather than carrying a second, unvalidated threshold beside a
    # validated one (no source gives this number directly).
    "over_frac": None,
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


def _axes(cfg):
    """`(origin, along-unit, left-unit)` of the track frame."""
    ox, oy = (float(cfg["origin_m"][0]), float(cfg["origin_m"][1]))
    th = math.radians(float(cfg["heading_deg"]))
    return ((ox, oy), (math.cos(th), math.sin(th)),
            (-math.sin(th), math.cos(th)))       # left of travel


# Above this radius (metres) `R - sqrt(R^2 - a^2)` starts subtracting two
# nearly-equal large numbers and losing precision, so `_curve_offset` switches
# to the mathematically-equivalent small-angle limit `a^2 / (2R)` instead —
# see that function. 1e6 m (1000 km) is reached only by a curvature under
# ~0.006 deg/km, an order below anything a real gentle arc would use.
_CURVE_HUGE_R_M = 1.0e6


def _curve_offset(a, curvature_deg_per_km):
    """Lateral offset (m, + = left) of a gentle circular arc at `a` metres
    downtrack, tangent to the heading axis at the origin.

    `curvature_deg_per_km` is degrees of heading turned per kilometre of
    `along`, so a circle of radius `R` metres turns `1000/R` radians (`=
    (1000/R) * 180/pi` degrees) over 1 km of arc length — invert that for
    `R`. The offset of a circular arc from its own tangent line at distance
    `a` along the tangent is the sagitta `R - sqrt(R^2 - a^2)`, exact for
    `|a| <= R`; `|a|` is clamped to `R` beyond that (a curvature tight enough
    for the plate to run past a quarter turn is not a "gentle arc" this
    module models, and clamping avoids a domain error rather than silently
    returning something wrong).
    """
    c = float(curvature_deg_per_km)
    R = 1000.0 / (abs(c) * math.pi / 180.0)
    sign = 1.0 if c > 0.0 else -1.0
    aa = min(abs(float(a)), R)
    if R >= _CURVE_HUGE_R_M:
        mag = aa * aa / (2.0 * R)
    else:
        mag = R - math.sqrt(max(0.0, R * R - aa * aa))
    return sign * mag


def _wobble(cfg):
    """`a -> lateral offset of the centreline` at `a` metres downtrack.

    Pulled out of `frame` so `from_track` can invert it EXACTLY. It was inline
    there, and the inverse map is the one place where a re-derived copy of a
    formula is worse than useless: a second wobble that agreed to three
    decimals would put the authored ground relief a metre off the corridor the
    intensity field actually cut, and nothing in a render says which of the two
    moved.

    `a` IS ALWAYS MEASURED ALONG THE ORIGINAL HEADING AXIS — `frame`'s
    `to_track` computes it as the dot product against the fixed unit vector
    `(ux, uy)` before this function ever runs, and `curvature_deg_per_km`
    below only ever nudges the CROSS coordinate as a function of that `a`.
    So the combined map (wobble + curvature) is a SHEAR of the plane, not a
    rotation: a straight line of constant `along` stays a straight line
    perpendicular to the original heading, it is only ever displaced
    sideways. That is what keeps `frame`/`from_track` exact inverses of each
    other with no iteration — see `from_track`'s own docstring for what
    "shear, not rigid motion" already means for the sinusoidal wobble; a
    circular arc folded into the same offset function inherits it for free.
    """
    amp = float(cfg.get("wobble_m", 0.0))
    per = max(1e-6, float(cfg.get("wobble_period_m", 340.0)))
    curvature = float(cfg.get("curvature_deg_per_km", 0.0))

    def wobble(a):
        out = 0.0
        if amp > 0.0:
            out += amp * (0.62 * math.sin(2.0 * math.pi * a / per)
                          + 0.38 * math.sin(2.0 * math.pi * a / (per * 0.37)
                                            + 1.71))
        if curvature != 0.0:
            out += _curve_offset(a, curvature)
        return out

    return wobble


def frame(cfg):
    """`to_track(x, y) -> (along_m, cross_m)` and the unit vectors.

    `along` is metres downtrack of `origin_m`, `cross` is metres LEFT of the
    centreline — after the meander, so `cross` is distance from where the
    vortex actually was rather than from the straight line through the origin.
    Returns `(to_track, (ux, uy), (vx, vy))`.
    """
    (ox, oy), (ux, uy), (vx, vy) = _axes(cfg)
    wobble = _wobble(cfg)

    def to_track(x, y):
        dx, dy = float(x) - ox, float(y) - oy
        a = dx * ux + dy * uy
        c = dx * vx + dy * vy
        return a, c - wobble(a)

    return to_track, (ux, uy), (vx, vy)


def from_track(cfg):
    """`(along_m, cross_m) -> (x, y)`: the exact inverse of `frame`'s `to_track`.

    Everything in this file so far reads the track field at a world point and
    asks how hard the wind hit it. `disaster.scour_relief` needs the other
    direction — it draws the marks a suction vortex leaves, and those are
    generated in the vortex's own frame (a trochoid in `(along, cross)`) and
    then placed. Without this they would have to be traced by searching the
    world for points with the right track coordinates, which is both slow and
    a second implementation of the meander.

    Note the map is a SHEAR, not a rigid motion: `wobble` is a function of
    `along`, so a straight line of constant `cross` comes out bent by the same
    meander the damage corridor has. That is the point — a mark authored at
    `cross = +12` lies 12 m left of the centreline wherever the centreline
    happens to be, which is what "12 m left of the track" means.
    """
    (ox, oy), (ux, uy), (vx, vy) = _axes(cfg)
    wobble = _wobble(cfg)

    def to_world(a, c):
        cc = float(c) + wobble(float(a))
        return (ox + ux * float(a) + vx * cc, oy + uy * float(a) + vy * cc)

    return to_world


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


def _breathe(a, period, lo, phase):
    """`lo`..1 along the track, two harmonics, never periodic-looking.

    Module-level (moved out of `intensity_field`'s closure so `wind_at` can
    read the same local half-width without a second copy of the formula —
    see `_wobble`'s docstring for why that copy would be worse than useless).
    It captures nothing from a caller's scope, so hoisting it changes
    nothing about what it computes.
    """
    s = (0.66 * math.sin(2.0 * math.pi * a / period + phase)
         + 0.34 * math.sin(2.0 * math.pi * a / (period * 0.43)
                           + phase * 1.9))
    return lo + (1.0 - lo) * (0.5 + 0.5 * max(-1.0, min(1.0, s)))


def _cross_profile(q, core):
    """The CROSS-TRACK profile: flat across `core` of the half-width, then a
    smoothstep to zero at the edge (`q` is `|cross| / half_width`, already
    past any edge noise). Shared by `intensity_field` and `wind_at` — the
    "vortex core in the middle, shoulders falling off" shape is one formula
    and both readers of the field have to agree on where its edge is.
    """
    if q >= 1.0:
        return 0.0
    if q <= core:
        return 1.0
    return 1.0 - _smoothstep((q - core) / (1.0 - core))


def _in_window(a, touchdown, liftoff):
    """True unless `a` (metres downtrack) is outside `[touchdown, liftoff]`.
    Either bound `None` (the default) does not gate at all."""
    if touchdown is not None and a < touchdown:
        return False
    if liftoff is not None and a > liftoff:
        return False
    return True


def _rope_out_factor(a, liftoff, ramp):
    """1.0 down to 0.35 over the `ramp` metres before `liftoff` — the WIDTH
    half of the rope-out. `liftoff` must not be `None`; callers gate that."""
    into_end = float(liftoff) - float(a)
    if into_end >= ramp:
        return 1.0
    t = max(0.0, into_end) / ramp
    return 0.35 + 0.65 * _smoothstep(t)


def _edge_ramp(a, touchdown, liftoff, ramp):
    """0..1 multiplier on INTENSITY from the touchdown ramp-up and the
    liftoff rope-out. 1.0 wherever neither knob's ramp reaches `a`."""
    m = 1.0
    if touchdown is not None:
        since = float(a) - float(touchdown)
        if since < ramp:
            m *= _smoothstep(max(0.0, since) / ramp)
    if liftoff is not None:
        into_end = float(liftoff) - float(a)
        if into_end < ramp:
            m *= _smoothstep(max(0.0, into_end) / ramp)
    return m


def intensity_field(cfg, region, rng):
    """`(x, y) -> 0..1`, the EF proxy every other pass in this scene reads.

    1 on the centreline at the track's strongest point, 0 outside the path.
    Four things shape it:

      * the CROSS-TRACK profile — flat across `core_frac` of the half-width,
        then a smoothstep to zero at the edge;
      * the ALONG-TRACK modulation — width and strength both breathe, out of
        phase, so the corridor pinches and there are weak stretches inside it;
      * band-limited noise on the EDGE;
      * OPTIONALLY, a finite track: `touchdown_m`/`liftoff_m` (`None` by
        default, meaning the track spans the whole plate exactly as before)
        gate `along` to a window and ramp the field up/down across `ramp_m`
        at each end, ROPING OUT the width as well as the intensity before
        `liftoff_m` — a weakening vortex's damage path narrows before it
        lifts, it does not stay full width until it vanishes.

    THE NOISE MOVES THE BOUNDARY, IT DOES NOT ADD TO THE VALUE. Writing
    `i * k + (noise - 0.5) * m` is the obvious way to perturb a field and it
    is wrong in exactly the way `ground.py` records: where `i` is zero the
    noise still contributes, so ground the tornado never touched comes out
    speckled with damage. Here the noise is added to the CROSS-TRACK DISTANCE
    before the profile is evaluated, so it can only move the edge in and out —
    outside the widened path the profile is identically zero.

    `touchdown_m`/`liftoff_m` BOTH `None` (the default) is guarded with
    plain `if`s rather than folded into the arithmetic unconditionally, so a
    cfg that never mentions them runs the exact same floating-point
    operations, in the exact same order, that this function always has.
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
    touchdown = cfg.get("touchdown_m")
    touchdown = None if touchdown is None else float(touchdown)
    liftoff = cfg.get("liftoff_m")
    liftoff = None if liftoff is None else float(liftoff)
    ramp = max(1e-6, float(cfg.get("ramp_m", 120.0)))

    def intensity(x, y):
        a, c = to_track(x, y)
        if not _in_window(a, touchdown, liftoff):
            return 0.0
        hw = half * _breathe(a, a_per * 1.31, w_min, 0.0)
        if liftoff is not None:
            hw *= _rope_out_factor(a, liftoff, ramp)
        q = (abs(c) + noise_m * noise(x, y)) / hw
        prof = _cross_profile(q, core)
        if prof == 0.0:
            return 0.0
        val = peak * prof * _breathe(a, a_per, a_min, 2.4)
        if touchdown is not None or liftoff is not None:
            val *= _edge_ramp(a, touchdown, liftoff, ramp)
        return val

    return intensity


def _local_half_width(cfg, a):
    """The raw (noiseless) cross-track half-width at `a` metres downtrack —
    breathing and, if `liftoff_m` is set, the rope-out taper, but NOT the
    spectral edge noise `intensity_field` adds (that needs a `region` and an
    `rng`; see `wind_at`'s docstring for why it does not need it either).
    """
    half = max(1e-6, 0.5 * float(cfg["width_m"]))
    w_min = float(cfg.get("width_min", 0.70))
    a_per = max(1e-6, float(cfg.get("along_period_m", 460.0)))
    hw = half * _breathe(a, a_per * 1.31, w_min, 0.0)
    liftoff = cfg.get("liftoff_m")
    if liftoff is not None:
        ramp = max(1e-6, float(cfg.get("ramp_m", 120.0)))
        hw *= _rope_out_factor(a, float(liftoff), ramp)
    return hw


def wind_at(cfg, x, y, region=None, rng=None):
    """The near-surface wind at one point: `{"bearing_deg", "speed_frac",
    "cross_frac", "over"}` — plan `urban_tornado_plan.md` 2.4.

    A tornado's near-surface wind is tangential (cyclonic) + translational +
    a convergent inflow (Karstens et al. 2013's 104k-tree-fall analysis;
    Beck & Dotzek 2010), not a single bearing pointing at the centreline. In
    the track frame, with `c` metres LEFT of the centreline and `hw` the
    local half-width:

        r      = c / hw                        -1..1 across the corridor
        vt     = the SAME cross-track profile `intensity_field` uses (1 in
                 the core, smoothstep to 0 at the edge) — `_cross_profile`,
                 shared rather than re-derived; see that function.
        t_dir  = -heading_dir if c > 0 (left flank): tangential wind blows
                 BACKWARD, down-track
                 +heading_dir if c < 0 (right flank): tangential wind blows
                 FORWARD, up-track — rotation and translation ADD here,
                 which is also why `throw_field`'s debris curls left: the
                 strong flank is the right one.
        V      = translation_frac * peak       the storm's own motion,
                                                along +heading_dir, constant
        inflow = inflow_frac * vt, directed TOWARD the centreline
                 (`-sign(c) * left_dir`)
        wind   = vt * t_dir + V * heading_dir + inflow * inward_dir

    Returns `bearing_deg` as the direction the wind BLOWS TOWARD, world
    frame, MATH CONVENTION (0 = +x, counter-clockwise positive) — the same
    convention `heading_deg` already uses, so `bearing_deg == heading_deg`
    is "blowing the way the storm is travelling". `speed_frac` is `|wind| /
    (1 + V)` clipped to 0..1 (the `1 + V` ceiling is the fastest the model
    can produce: `vt` and `inflow` maxed at the core, plus the full
    translation). `cross_frac` is `r`. `over` is `|r| < over_frac` (which
    defaults to `core_frac` — see DEFAULTS): the
    vortex core passed OVER this point, so every face of a building there
    was loaded in turn and no single bearing is "the" windward one.

    OUTSIDE THE CORRIDOR (`vt == 0`) this returns the pure translation
    direction (`bearing_deg == heading_deg`) with `speed_frac 0.0` and
    `over False` — there is a storm moving through the region, just not
    one hitting this point.

    NO EDGE NOISE, ON PURPOSE. `intensity_field`'s spectral edge noise needs
    a `region` and an `rng` because it perturbs where the corridor boundary
    falls at map-authoring scale; the urban damage ladder that calls this
    samples ONE point per building (the footprint centre) and already draws
    its own per-building intensity jitter before the level lookup (`_ladder`
    -- house/tree levels do the same). A second, independent wobble here
    would not change which buildings are near the edge, only add noise this
    function has no region to evaluate anyway. `region`/`rng` stay as
    parameters for signature symmetry with `intensity_field`/`scour_coverage`
    and are otherwise unused.

    Gated by `touchdown_m`/`liftoff_m` exactly like `intensity_field`: a
    point outside that along-track window is "outside the corridor" here
    too, so a building past a track's liftoff never gets handed a wind
    bearing for a storm that has already lifted off.
    """
    to_track, (ux, uy), (vx, vy) = frame(cfg)
    a, c = to_track(float(x), float(y))

    heading_deg = float(cfg["heading_deg"])
    trans_frac = float(cfg.get("translation_frac", 0.22))
    peak = float(cfg.get("peak", 0.92))
    V = trans_frac * peak

    touchdown = cfg.get("touchdown_m")
    touchdown = None if touchdown is None else float(touchdown)
    liftoff = cfg.get("liftoff_m")
    liftoff = None if liftoff is None else float(liftoff)

    hw = _local_half_width(cfg, a)
    r = c / hw
    core = min(0.98, max(0.0, float(cfg.get("core_frac", 0.3))))
    vt = (_cross_profile(abs(r), core)
          if _in_window(a, touchdown, liftoff) else 0.0)

    if vt <= 0.0:
        return {"bearing_deg": heading_deg % 360.0, "speed_frac": 0.0,
                "cross_frac": r, "over": False}

    inflow_frac = float(cfg.get("inflow_frac", 0.50))
    # `over_frac` None -> `core_frac`: see DEFAULTS. Falls back to the same
    # `core` this function already clipped for the profile above.
    over_frac = cfg.get("over_frac")
    over_frac = core if over_frac is None else float(over_frac)

    sign_c = 1.0 if c > 0.0 else (-1.0 if c < 0.0 else 0.0)
    tx, ty = -sign_c * ux, -sign_c * uy
    inflow = inflow_frac * vt
    ix, iy = -sign_c * vx, -sign_c * vy

    wx = vt * tx + V * ux + inflow * ix
    wy = vt * ty + V * uy + inflow * iy

    speed = max(0.0, min(1.0, math.hypot(wx, wy) / (1.0 + V)))
    bearing = math.degrees(math.atan2(wy, wx)) % 360.0
    over = abs(r) < over_frac

    return {"bearing_deg": bearing, "speed_frac": speed, "cross_frac": r,
            "over": over}


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
# the log debris surface
# ---------------------------------------------------------------------------
#
# WHAT A TORNADO LOG IS MADE OF, AND WHY IT IS NOT WHAT THE FIRE PATH USES.
#
# Reported off the first assembled plate: *"the logs that are thrown around
# ... their material looks to be the 'burnt forest floor' one. I can't have
# that."* Nothing bound the burnt floor. What was bound, measured out of the
# baked archetypes, was each tree's OWN bark diffuse through
# `damage._pbr` — diffuse map only, no normal, no ORM — at
# `texture_scale = (0.3, 0.3)`, i.e. ONE TILE PER 3.33 M:
#
#   species            log diffuse             mean luma
#   Douglas_Fir        alter49_tree10           0.195      near-charcoal
#   American_Beech     bark3                    0.324
#   Black_Oak          alter49_tree7            0.362
#   Shumard_Oak        alter49_tree7            0.362
#   Largetooth_Aspen   alter49_tree4            0.535 (sd 0.223)  white birch
#
# A 1.5 m stick under a 3.33 m tile shows a crop about 6% wide by 45% long of
# one photograph, with no normal to give the cylinder any relief — so what
# renders is a soft mottled patch whose value depends on which species the
# tree happened to be. Two of the six read as burnt ground at any scale, and
# the stand as a whole covers a 2.7x value range, which says nothing about
# species and everything about a bug.
#
# THE FIX IS ONE SURFACE PAIR FOR THE WHOLE STAND, not six lotteries. Species
# identity is worth nothing at these sizes — a 0.2 m limb photographed from 40
# m — and it costs a debris field that swings between charcoal and birch bark.
# `vegetation.split_wood_material` carries the derivation of which piece gets
# which: `wood_debris` cuts riven columns out of the bole rather than branches
# off the outside, so most pieces have no bark on them at all.
#
# THE VALUES ARE THE POINT. A tornado debris field photographs LIGHT against
# green grass and a burn scar photographs dark — the same argument
# `planks.WOOD_BASE` is chosen on. So the bark is tinted UP and warm (mean
# luma 0.44 -> ~0.49) where `scene_api._load_burnt_wood` tints the identical
# surface DOWN to ~0.13 for the wildfire path. Getting these two the same way
# round is the fastest way to build the wrong disaster.
LOG_BARK_TILE_M = 1.25
LOG_BARK_TINT = (1.22, 1.10, 0.94)
LOG_SPLIT_TILE_M = 0.75
LOG_SPLIT_TINT = (1.06, 1.03, 0.98)

# 1.25 m rather than `bark_material`'s 1.7 m default, and the difference is
# trunk bark against LIMB bark. `bark_oak_diff`'s author scale is 2.0 m a tile
# (`Wood_Bark.mdl` annotates `texture_scale` with `dimension(float2(1,1))` and
# halves it internally) and the sheet carries ~52 furrow ridges, so 3.8 cm a
# ridge as photographed — a mature bole. `_WIND_THICK` debris is 0.10-0.30 m
# through, and bark on a piece that size is finer; 1.25 m puts a ridge at
# 2.4 cm and about twenty of them round the girth of a 0.2 m stick, which is
# what makes it read as bark rather than as a brown tube.


def log_materials(stage, parent_path, suffix=""):
    """`(bark_path, split_path)` for tornado tree debris. Authors both.

    Built once by a bake or an assembly and handed to every `wind_tree` call,
    the same way the bake already shares one soil material for the root balls
    — a material per tree would be six identical surfaces on the stage and a
    different `Looks` name in every archetype.
    """
    from . import vegetation as veg

    bark = "{0}/WindLooks/log_bark{1}".format(parent_path, suffix)
    split = "{0}/WindLooks/log_split{1}".format(parent_path, suffix)
    veg.bark_material(stage, bark, tile_m=LOG_BARK_TILE_M,
                      tint=LOG_BARK_TINT)
    veg.split_wood_material(stage, split, tile_m=LOG_SPLIT_TILE_M,
                            tint=LOG_SPLIT_TINT)
    return bark, split


# ---------------------------------------------------------------------------
# how far a baked tree archetype reaches, and which way to point it
# ---------------------------------------------------------------------------
#
# THE SECOND HALF OF THE SAME REPORT: *"the logs that are thrown around look
# like they're floating"*. They are not floating and there is no z bug in the
# bake — every `log*` mesh in every archetype has a world minimum BELOW zero,
# so inside its own frame each piece is seated or slightly buried. What is
# missing is the GROUND. `suburb_scene.apply_ground` lays its sheet over
# exactly `region` and nothing beyond it, and a tree archetype throws its
# debris up to 27 m from the trunk, so any tree within a reach of the plate
# boundary puts part of its debris bed over the void. Same defect
# `scour_relief.clip_to_region` already fixes for spoil heaps, arriving from
# the other side: there the feature can simply be dropped, and here it cannot,
# because a tree archetype is referenced as an INSTANCE and USD forbids
# authoring inside one. The only levers are WHICH archetype and WHICH YAW.
#
# MEASURED, NOT ASSUMED — and the measurement is why this is a table of
# sixteen numbers rather than one radius. Sampling every mesh point of all 23
# baked archetypes gives a reach that is a tight LOBE about local +X (the
# throw direction every archetype is baked with): sectors spanning roughly
# -68 to +68 degrees carry 12-27 m and everything behind the tree is under
# 4 m. A single circular radius would therefore reject about six times the
# area it needs to.
#
# ONLY NEAR-GROUND GEOMETRY COUNTS. The profile is taken over points below
# 2 m, and dropping the crown is deliberate: a standing tree whose canopy
# overhangs the plate edge is what a tree on a boundary looks like, while a
# LOG lying past the edge is a log hanging over nothing. Including the crown
# would have put Black_Oak's 25 m canopy into every `limbed` entry and
# downgraded half the plate for no visual gain.
#
# THREE THINGS THE FIRST DIAGNOSIS OF THIS MISSED, all of which the numbers
# show and all of which change the fix:
#
#   * IT IS NOT ONLY `leaning` AND `fallen`. `snapped` reaches 20.9-24.8 m and
#     `limbed` 13.0-26.7 m, and NEITHER is in the launcher's
#     `_TREE_TRACK_YAWED`, so both are placed at the tree's own arbitrary
#     layout yaw. Those are the worse cases: at least a track-yawed tree
#     throws its debris in a direction somebody chose.
#   * THE REACH IS THE DEBRIS, NOT THE TREE. `_WIND_DEBRIS` scatters to 8-17 m
#     and then `settle` throws the big pieces further with its +X bias at
#     `THROW_MPS`, which is where 24 m off a 9 m/s bias comes from. Retuning
#     either moves this table.
#   * IT IS SEED-DEPENDENT. These are one bake's scatter (`ARCH_SEED=7`,
#     `THROW_MPS=9`). Re-bake with a different seed and the numbers move by a
#     metre or two, which is why every entry is rounded UP and why
#     `_REACH_ANY` — the per-level envelope over all species — is the fallback
#     for a combination that is not in the table.
#
# Re-measure after any re-bake, from a bare `python.sh` with no SimulationApp
# (safe beside a running sim): open each `tree_*.usd`, transform every mesh
# point to world, and take the max radius per sector over points with z < 2.

REACH_SECTORS = 16

# (species, level) -> 16 radii in metres, sector 0 spanning local azimuth
# -180..-157.5 and running counter-clockwise, so local +X straddles the
# boundary between sectors 7 and 8. Measured 2026-08-27 off the ARCH_SEED=7
# bake.
TREE_REACH = {
    ("American_Beech", "limbed"):    (1.8, 2.2, 2.1, 2.2, 1.8, 1.6, 3.4, 14.7,
                                      16.0, 13.1, 4.3, 1.3, 0.9, 1.0, 1.5, 2.1),
    ("American_Beech", "leaning"):   (1.0, 0.5, 1.2, 1.6, 2.2, 2.3, 2.4, 14.0,
                                      12.6, 2.9, 2.6, 0.8, 0.7, 0.5, 0.8, 0.9),
    ("American_Beech", "fallen"):    (0.4, 0.4, 0.5, 0.2, 1.5, 2.6, 16.7, 20.5,
                                      19.1, 17.7, 1.7, 0.5, 0.4, 0.3, 0.4, 0.4),
    ("American_Beech", "snapped"):   (1.8, 2.3, 2.2, 2.2, 2.1, 6.4, 19.8, 23.5,
                                      23.1, 21.1, 8.3, 2.0, 1.7, 2.0, 2.1, 2.2),
    ("Black_Oak", "limbed"):         (1.0, 0.8, 0.8, 0.8, 0.9, 0.9, 0.8, 23.6,
                                      26.7, 16.2, 0.9, 0.6, 0.9, 0.9, 1.0, 0.7),
    ("Black_Oak", "leaning"):        (0.9, 0.7, 0.8, 0.6, 0.9, 1.0, 17.1, 19.2,
                                      16.8, 15.8, 11.0, 0.6, 0.9, 0.8, 0.9, 0.6),
    ("Black_Oak", "snapped"):        (1.1, 0.9, 0.8, 0.8, 1.0, 0.9, 16.2, 24.5,
                                      23.9, 21.9, 0.9, 0.6, 1.0, 0.9, 1.0, 0.7),
    ("Common_Apple", "limbed"):      (3.1, 2.9, 3.0, 0.8, 1.3, 7.1, 6.6, 12.5,
                                      13.0, 5.9, 6.0, 2.0, 1.7, 1.3, 2.4, 3.0),
    ("Common_Apple", "leaning"):     (1.9, 2.5, 2.4, 2.6, 1.2, 2.2, 4.6, 14.3,
                                      13.4, 13.4, 6.3, 2.4, 0.7, 0.3, 2.1, 2.1),
    ("Common_Apple", "fallen"):      (0.7, 0.8, 0.9, 2.2, 2.3, 4.2, 13.4, 14.9,
                                      18.3, 17.6, 9.0, 0.6, 0.2, 0.6, 1.1, 1.0),
    ("Common_Apple", "snapped"):     (3.1, 2.9, 3.0, 2.3, 2.3, 10.7, 19.1, 22.9,
                                      22.8, 16.2, 8.9, 2.2, 2.1, 2.2, 2.4, 3.1),
    ("Douglas_Fir", "limbed"):       (0.3, 0.6, 0.5, 0.2, 0.5, 0.5, 0.5, 13.8,
                                      15.9, 2.4, 0.5, 0.8, 0.8, 0.5, 0.4, 0.6),
    ("Douglas_Fir", "leaning"):      (0.1, 0.1, 0.1, 0.2, 0.2, 0.5, 1.8, 13.1,
                                      13.1, 13.8, 0.7, 0.8, 0.8, 0.6, 0.4, 0.4),
    ("Douglas_Fir", "fallen"):       (0.4, 0.4, 0.5, 0.3, 0.2, 0.3, 16.0, 17.5,
                                      19.4, 16.2, 0.3, 0.8, 0.8, 0.6, 0.4, 0.4),
    ("Douglas_Fir", "snapped"):      (0.2, 0.1, 0.1, 0.2, 0.2, 0.2, 15.6, 21.4,
                                      24.8, 20.7, 0.5, 0.8, 0.8, 0.5, 0.3, 0.3),
    ("Largetooth_Aspen", "limbed"):  (0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 4.1, 13.6,
                                      13.3, 2.8, 2.7, 0.2, 0.2, 0.2, 0.2, 0.2),
    ("Largetooth_Aspen", "leaning"): (0.1, 0.2, 0.2, 0.2, 0.2, 3.6, 3.7, 12.9,
                                      12.3, 2.8, 2.9, 0.2, 0.2, 0.2, 0.0, 0.1),
    ("Largetooth_Aspen", "fallen"):  (0.4, 0.1, 0.4, 0.3, 0.2, 3.4, 16.7, 20.1,
                                      20.6, 19.0, 3.5, 0.3, 0.4, 0.5, 0.4, 0.4),
    ("Largetooth_Aspen", "snapped"): (2.6, 2.4, 1.7, 2.4, 2.6, 12.4, 16.7, 19.1,
                                      20.9, 16.1, 2.5, 2.7, 2.3, 2.4, 2.4, 2.5),
    ("Shumard_Oak", "limbed"):       (3.1, 2.6, 0.4, 0.4, 0.3, 0.3, 2.1, 13.5,
                                      12.3, 12.3, 0.4, 0.4, 2.2, 2.5, 3.1, 3.7),
    ("Shumard_Oak", "leaning"):      (2.4, 2.3, 0.5, 0.3, 0.3, 0.4, 3.3, 13.5,
                                      14.7, 4.6, 4.7, 0.3, 2.2, 2.4, 2.5, 2.8),
    ("Shumard_Oak", "fallen"):       (2.2, 2.3, 0.5, 0.3, 0.4, 9.5, 17.8, 18.6,
                                      21.5, 17.3, 3.3, 0.4, 2.2, 2.4, 2.4, 2.3),
    ("Shumard_Oak", "snapped"):      (3.1, 2.6, 0.4, 0.4, 0.3, 0.3, 20.6, 24.3,
                                      19.9, 22.7, 5.8, 0.4, 2.2, 2.5, 3.1, 3.7),
}

# Per-level envelope over every species, for a combination the table has not
# seen — a new species in the asset set, or a bake this file has not been
# re-measured against. Conservative by construction (it is the elementwise
# max), so an unknown tree is treated as the worst one measured rather than as
# a point object, which fails toward downgrading a tree that could have
# stayed rather than toward debris over the void.
_REACH_ANY = {
    "limbed":  (3.1, 2.9, 3.0, 2.2, 1.8, 7.1, 6.6, 23.6,
                26.7, 16.2, 6.0, 2.0, 2.2, 2.5, 3.1, 3.7),
    "leaning": (2.4, 2.5, 2.4, 2.6, 2.2, 3.6, 17.1, 19.2,
                16.8, 15.8, 11.0, 2.4, 2.2, 2.4, 2.5, 2.8),
    "fallen":  (2.2, 2.3, 0.9, 2.2, 2.3, 9.5, 17.8, 20.5,
                21.5, 19.0, 9.0, 0.8, 2.2, 2.4, 2.4, 2.3),
    "snapped": (3.1, 2.9, 3.0, 2.4, 2.6, 12.4, 20.6, 24.5,
                24.8, 22.7, 8.9, 2.7, 2.3, 2.5, 3.1, 3.7),
}


def tree_reach(species, level):
    """The 16-sector reach profile for one archetype, or None if unbounded.

    `pristine` returns None because it is not an archetype at all — the
    assembly references the green species USD, whose only geometry is a
    standing tree, and a canopy over the plate edge is not the defect this
    guards against.
    """
    if not level or str(level) == "pristine":
        return None
    r = TREE_REACH.get((str(species), str(level)))
    if r is not None:
        return r
    return _REACH_ANY.get(str(level))


def _arc_extremes(a0, a1):
    """Azimuths (deg) at which an arc a0..a1 can touch an axis-aligned box.

    An arc of constant radius touches a rectangle either at one of its own
    ENDS or where it crosses an axis direction — that is where x or y is
    stationary along the arc — so testing those points is EXACT rather than a
    sampling approximation, and it is at most six points per sector. Sampling
    the arc instead would miss a tangent by up to `r * (1 - cos(11.25 deg))`,
    half a metre at the reach of a snapped oak, which is exactly the size of
    the overhang being hunted.
    """
    out = [a0, a1]
    for base in (0.0, 90.0, 180.0, 270.0):
        a = a0 + ((base - a0) % 360.0)
        if a <= a1:
            out.append(a)
    return out


def reach_fits(reach, x, y, yaw_deg, region, margin_m=0.0):
    """True when an archetype at `(x, y)` yawed by `yaw_deg` stays on the plate.

    `region` is `(x0, y0, x1, y1)` — the same tuple `suburb_scene.apply_ground`
    lays its sheet over and `scour_relief.clip_to_region` tests against, so
    "inside" means "there is ground underneath".
    """
    if not reach:
        return True
    x0, y0, x1, y1 = (float(q) for q in region)
    m = float(margin_m)
    x0, y0, x1, y1 = x0 + m, y0 + m, x1 - m, y1 - m
    n = len(reach)
    step = 360.0 / n
    for k in range(n):
        r = float(reach[k])
        if r <= 0.0:
            continue
        a0 = -180.0 + step * k + float(yaw_deg)
        for a in _arc_extremes(a0, a0 + step):
            t = math.radians(a)
            px = float(x) + r * math.cos(t)
            py = float(y) + r * math.sin(t)
            if px < x0 or px > x1 or py < y0 or py > y1:
                return False
    return True


# How finely the inward sweep is searched, in degrees. 9 is a third of the
# ladder's own +-38 deg fall jitter, so a tree that only needs nudging keeps a
# bearing indistinguishable from the one it drew, and 40 candidates covers the
# full circle for the ones that need turning right round.
YAW_STEP_DEG = 9.0


def tree_level_and_yaw(intensity, rng, species, x, y, region, *,
                       track_yaw_deg, base_yaw_deg=0.0, jitter_deg=38.0,
                       track_yawed=("leaning", "fallen"), margin_m=0.0,
                       step_deg=YAW_STEP_DEG):
    """Level and yaw for one tree, with its debris kept ON the plate.

    Returns `(level, yaw_deg, info)` where `info` is
    `{"drawn", "turned_deg", "downgraded"}` for the assembly's banner.

    TURN IT BEFORE YOU DOWNGRADE IT. Both moves keep the debris on the ground,
    and they are not equally cheap: the fall bearing is already a ±38 degree
    draw, because "trees go over in the direction their own rooting and
    neighbours allow as much as in the direction the wind was going", so
    turning one further costs a little of the stand's grain and nothing else.
    Downgrading it changes what the scene SAYS — it puts an intact tree where
    the intensity field asked for a fallen one, and near a plate edge that
    would print a ring of undamaged vegetation round the whole scene, which is
    the exact artefact `clip_to_region` rejects an inset for. So: the drawn
    level is tried at every yaw first, and the ladder is only walked down when
    no yaw on the circle fits.

    The candidate yaws are searched OUTWARD FROM THE NATURAL ONE, so a tree
    that needs a nudge gets a nudge and only a tree in a corner is turned right
    round. `turned_deg` is how far it had to go; a scene where that is large
    for many trees is a scene whose archetypes out-reach its plate, and the
    answer to that is a smaller `_WIND_DEBRIS` radius, not a bigger sweep.

    THE JITTER IS DRAWN UNCONDITIONALLY, one `rng` value per tree whatever
    happens next. Drawing it lazily — only for the levels that use it, which is
    what the launcher did — makes the number of draws depend on the outcome, so
    one tree changing level re-rolls every tree after it and a re-tune of the
    plate size silently reshuffles the whole stand.
    """
    drawn = tree_level_for_intensity(intensity, rng, species=species)
    jit = rng.uniform(-float(jitter_deg), float(jitter_deg))

    order = list(TREE_LEVELS)
    try:
        start = order.index(drawn)
    except ValueError:
        start = 0
    n_yaw = max(1, int(round(360.0 / max(1e-6, float(step_deg)))))
    for idx in range(start, -1, -1):
        level = order[idx]
        if level == "pristine":
            break
        # The promotion `tree_level_for_intensity` applies has to be applied
        # again on the way DOWN, or a wide-crowned species walking back from
        # `snapped` lands on `fallen` — a combination the bake deliberately
        # never wrote, so the assembly would reference a missing archetype and
        # fall back to a green tree anyway.
        if level == "fallen" and species and str(species) in NO_UPROOT:
            continue
        reach = tree_reach(species, level)
        natural = (float(track_yaw_deg) + jit if level in track_yawed
                   else float(base_yaw_deg))
        if reach_fits(reach, x, y, natural, region, margin_m=margin_m):
            return level, natural, {"drawn": drawn, "turned_deg": 0.0,
                                    "downgraded": level != drawn}
        for i in range(1, n_yaw + 1):
            for sign in (1.0, -1.0):
                d = sign * float(step_deg) * i
                if abs(d) > 180.0:
                    continue
                if reach_fits(reach, x, y, natural + d, region,
                              margin_m=margin_m):
                    return level, natural + d, {"drawn": drawn,
                                                "turned_deg": d,
                                                "downgraded": level != drawn}
    return "pristine", float(base_yaw_deg), {"drawn": drawn,
                                             "turned_deg": 0.0,
                                             "downgraded": drawn != "pristine"}


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
# vehicles in the track
# ---------------------------------------------------------------------------
#
# WHY A VEHICLE GETS ITS OWN MODEL AND A TRASH CAN DOES NOT. A car is the most
# legible object in a debris field: a viewer knows its correct resting pose
# without being told, so a wrong one is read instantly and a right one carries
# the whole severity of the frame. It was also the explicit second element of
# Joplin's search doctrine — "house by house, CAR BY CAR, block by block" —
# which is the reason this dataset has any business modelling it carefully.
#
# EVERY RATE BELOW IS Paulikas, Schmidlin & Marshall 2016, 959 vehicles across
# 12 tornadoes, and the ladder is built from it rather than from taste:
#
#     EF0        10% shifted
#     EF1-EF2    36% displaced,   5% rolled or lofted
#     EF3-EF4    63% displaced,  15% rolled or lofted
#     EF5        69% moved,      31% tipped
#
# Two things fall straight out of that table and both are easy to get wrong.
# First, even on the centreline only about a third go over: TWO THIRDS OF THE
# MOVED CARS ARE MERELY SHOVED — parked askew, nosed into a kerb, pushed off a
# drive — and a corridor where every car is upside down is as wrong as one
# where none is. Second, those are UNCONDITIONAL shares of all vehicles in the
# damage zone: "EF5 69% moved, 31% tipped" means 31% of ALL cars, not 31% of
# the 69% that moved. Testing a tip probability only after a car has already
# passed a move probability multiplies the two and produces an effective 6%
# where the data says 15% — MEASURED on the first tornado run, 5 of 25 cars in
# the path moved and NONE tipped. A tipped car is displaced by definition, so
# the correct structure is ONE draw against nested thresholds, which is what
# `car_pose` does.
CAR_P_MOVE = (0.10, 0.62)      # p_move = a + b * intensity
CAR_P_TIP = (0.05, 0.30)       # p_tip  = a + b * intensity ** 1.5

# Below this the wind is not doing anything to a parked car worth authoring;
# above it and below `CAR_MIN_TIP_INTENSITY` a car can be shoved but not rolled.
CAR_MIN_INTENSITY = 0.12

# The saloon the rates are quoted against. `Paulikas`' population is
# overwhelmingly light passenger vehicles, and the residential pool here is
# 4.60-4.84 m, so the reference is the pool's own median rather than a round
# number.
CAR_REF_LEN_M = 4.60

# HOW FAR, and it has to be conditioned on WHICH OUTCOME. A car that only slid
# still has four tyres on the ground: friction dominates, and it stops inside a
# car length or two — that is what "pushed off the drive" means. A car that
# went over left the ground, and those are the ones a survey finds tens of
# metres away. Driving both from one distribution (which the first cut of this
# did, using the plank field's reach for everything) teleports a wheels-down
# car ten metres down the street with no mark on it, which reads as a
# continuity error rather than as damage.
CAR_SHOVE_M = (1.0, 3.4)       # d = (a + b * intensity) * U(0.55, 1.6)
CAR_SHOVE_JITTER = (0.55, 1.6)

# The cone the travel bearing is drawn in, about the throw heading. A shoved
# car is pushed by the near-surface flow it is standing in and goes broadly
# where that flow goes; a lofted one is at the mercy of the vortex and scatters
# far wider. A nosed-in car was arrested head-on, so its bearing is the bearing
# it was travelling.
CAR_SPREAD_DEG = {"shoved": 22.0, "side": 52.0, "roof": 52.0, "nose": 30.0}

# Shares WITHIN the tipped population, and the order is by how often a damage
# photograph shows them.
CAR_TIP_MIX = (0.55, 0.30, 0.15)          # side, roof, nose

# How close a jammed car comes to rest against the thing that stopped it, and
# how far a blocker search steps. 0.35 m is a bumper's worth of crush plus the
# fact that the blocker radii below are nominal.
CAR_JAM_GAP_M = 0.35
CAR_MARCH_M = 0.5

# A car shoved hard into something rides up it — the nose lifts and the car
# rests leaning on whatever stopped it. CAPPED BELOW 30 DEGREES ON PURPOSE:
# the launcher's `toppled` flag (and therefore `tornado_people`'s `in_vehicle`
# scenario, which is UPRIGHT CARS ONLY) is `|roll| > 30 or |pitch| > 30`, so a
# jammed-but-upright car has to stay under that line or the occupant planner
# loses it. 18 degrees is a visible lean and not a rolled car.
CAR_JAM_PITCH_DEG = (8.0, 18.0)

# What can actually stop a car, as a nominal radius in metres. Everything not
# in here is transparent to a sliding car, and the omissions are as deliberate
# as the entries: a FENCE does not stop a car (and by the time this runs the
# corridor's fences have been deactivated anyway), a mailbox and a sign shear
# off, and a hydrant is a 0.5 m casting that a car rides over. A tree bole and
# a house wall stop one dead, and a car stops against another car — that pile
# is one of the most characteristic things in a real track.
CAR_BLOCKER_R_M = {"tree": 0.7, "car": 2.4, "streetlight": 0.3}


def _wrap180(deg):
    """Fold an angle into (-180, 180]. Used so a yaw DELTA is the short way
    round: a car does not need to be told to spin 340 degrees to end up 20."""
    d = float(deg) % 360.0
    return d - 360.0 if d > 180.0 else d


def car_blockers(standing=(), trees=(), cars=(), extra=(), cell_m=12.0):
    """``(x, y) -> tag`` for the first thing a sliding car would jam against.

    THE POINT OF THIS IS THAT A CAR COMES TO REST AGAINST SOMETHING. Left to
    a bare throw vector a displaced car lands wherever the vector put it, which
    in an open plat is most often the middle of a lawn — and a car sitting
    politely in open grass twelve metres from its drive is the one vehicle pose
    that reads as "somebody moved this asset" rather than as wind. In a real
    track they pile against standing walls, wrap round tree boles, and end up
    nose to nose in the gutter. Arresting the travel is what buys that.

    *standing* is the launcher's own list of houses that still have a building
    on the lot, as `(x, y, footprint_m)` — a WRECKED house is still a heap of
    material that stops a car, so this is deliberately `standing` and not
    `intact`. *trees* and *cars* are `(x, y)`, *extra* is `(x, y, radius, tag)`
    for anything a caller wants to add. Same hash grid, and the same reason,
    as `suburb_scene._CarKeepout`.

    RADII ARE NOMINAL AND THAT IS FINE HERE. A house is entered as half its
    footprint, which is a disc round a rectangle and therefore generous at the
    corners; the consequence of being generous is a car that stops a metre
    early, and the consequence of being exact would be a car authored inside
    somebody's living room. The asymmetry is the whole argument for not
    measuring properly.

    A CAR DOES NOT BLOCK ITSELF, and the caller does not have to filter the
    list per vehicle. `car_pose` probes the car's NOSE, which starts one march
    step plus half a car length ahead of the origin — 2.7 m for the shortest
    vehicle in the suburban pool (4.40 m) against a 2.40 m `car` radius — so
    the first probe is already clear of the disc the car is standing on. That
    margin is thin, so it is written down: a vehicle shorter than about 3.8 m
    WOULD arrest itself at zero displacement, and the fix if one is ever added
    is to drop the moving car's own entry rather than to shrink the radius.
    """
    grid, cell = {}, float(cell_m)
    def _add(x, y, r, tag):
        x, y, r = float(x), float(y), float(r)
        for gx in range(int(math.floor((x - r) / cell)),
                        int(math.floor((x + r) / cell)) + 1):
            for gy in range(int(math.floor((y - r) / cell)),
                            int(math.floor((y + r) / cell)) + 1):
                grid.setdefault((gx, gy), []).append((x, y, r, tag))
    for h in (standing or ()):
        _add(h[0], h[1], float(h[2] if len(h) > 2 else 12.0) * 0.5, "house")
    for t in (trees or ()):
        _add(t[0], t[1], CAR_BLOCKER_R_M["tree"], "tree")
    for c in (cars or ()):
        _add(c[0], c[1], CAR_BLOCKER_R_M["car"], "car")
    for e in (extra or ()):
        _add(e[0], e[1], e[2], str(e[3]))

    def blocked(x, y):
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        for (bx, by, r, tag) in grid.get(key, ()):
            if math.hypot(x - bx, y - by) <= r:
                return tag
        return None

    return blocked


def car_pose(intensity, rng, throw_deg, throw_m, x=0.0, y=0.0,
             long_axis_deg=None, length_m=CAR_REF_LEN_M, blocked=None,
             force=None):
    """Where one parked car ends up, and in what attitude. Pure geometry.

    Returns a dict — `moved`, `pose`, `dx`, `dy`, `d_m`, `roll_deg`,
    `pitch_deg`, `yaw_delta_deg`, `arrested_by`, `toppled` — which the caller
    hands to `toss_prim`. Nothing here touches USD, so it is testable offline
    and it is the only place the vehicle ladder lives.

    *intensity* is the track field at the car's parked position, *throw_deg*
    the scene's debris heading (`heading_deg + curl_deg`), *throw_m* the
    scene's reach knob. *long_axis_deg* is the world bearing of the car's own
    long axis BEFORE the toss — the placement's `yaw_deg` with the asset's art
    `yaw-offset` taken back off — and passing it is what makes the resting
    heading physical rather than a jitter; omit it and the yaw degrades to the
    old undirected wobble. *blocked* is a `car_blockers` predicate.

    THE FOUR POSES, and what each one is:

      shoved  wheels down, slewed, moved a car length or two. TWO THIRDS OF
              EVERY MOVED CAR, per the rates above.
      side    rolled about its own long axis and stopped against its roof
              rail. The commonest tipped pose by far.
      roof    all the way over, resting on roof and pillars — a flatter,
              wider silhouette than `side` and it reads differently from the
              air, which is why it is worth separating.
      nose    thrown against something and left standing on its front end.
              Rare, and the single most arresting vehicle pose in a real
              damage photograph.

    THE RESTING HEADING IS SET BY THE OUTCOME, and this is the part the first
    version got wrong — it applied one `U(-80, 80)` yaw wobble to everything,
    which is a heading drawn from nothing.

      * A SHOVED car SLEWS, it does not spin. Four tyres on tarmac resist
        rotation far better than they resist sliding, so what a survey
        photographs is a car pushed askew across its own bay by a few tens of
        degrees, not one facing back down the street. Hence a delta about the
        car's EXISTING heading.
      * A ROLLED car rolls about its LONG AXIS, so when it stops that axis is
        ACROSS the direction it travelled. This is the same argument
        `planks.py` makes for board yaw (heading + 90, sigma 46) and it comes
        from the same place: a long thin object driven by a flow ends up lying
        across it. Hence an ABSOLUTE bearing of `travel + 90`, either hand.
      * A NOSED-IN car hit something head-on, so its long axis points at what
        it hit — the travel bearing, tightly.

    MASS MATTERS AND LENGTH IS THE PROXY. Paulikas' population is light
    passenger vehicles; the same wind does not roll a 9.5 m transit bus. For
    geometrically similar vehicles the wind force goes as frontal area and the
    resistance as mass, so force/mass scales as 1/L and both probabilities take
    `CAR_REF_LEN_M / length_m`, capped at 1. That puts the bus at 0.48 of a
    saloon's rates and leaves the residential pool (4.60-4.84 m) within 5% of
    the reference, which is the intended behaviour: this exists to stop a bus
    cartwheeling, not to re-rank the saloons. IT IS A POOR PROXY FOR A HIGH-
    SIDED VAN, which is light for its frontal area and in reality flips early;
    the honest fix is a mass/area field on the asset entry and there is not one.
    """
    it = max(0.0, min(1.0, float(intensity)))
    rest = {"moved": False, "pose": "parked", "dx": 0.0, "dy": 0.0, "d_m": 0.0,
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_delta_deg": 0.0,
            "arrested_by": None, "toppled": False}
    if it <= CAR_MIN_INTENSITY and force is None:
        return rest

    mass = min(1.0, CAR_REF_LEN_M / max(1.0, float(length_m)))
    p_move = (CAR_P_MOVE[0] + CAR_P_MOVE[1] * it) * mass
    p_tip = (CAR_P_TIP[0] + CAR_P_TIP[1] * it ** 1.5) * mass

    # ONE DRAW, NESTED THRESHOLDS — see the block comment above for why this
    # is not two independent coins.
    #
    # `force` SKIPS THE DRAW AND NOTHING ELSE. `"tip"` takes the tipped branch
    # and `"move"` the shoved one; the attitude mix, the throw distance, the
    # march against the blockers and the resting heading are all the ordinary
    # model. It exists for REVIEW SCENES — a 100 m plate holds about eight
    # in-track cars, mostly on the shoulder where p_tip is under a tenth, so
    # "no rolled car anywhere" is the ordinary outcome of a deterministic seed
    # and re-rolling the seed until one appears is a poor way to answer "show
    # me what a rolled car looks like". The caller that uses it says so in its
    # banner (`TOR_MIN_TIPPED`); nothing sets it by default.
    draw = rng.random()
    if force is None and draw > p_move:
        return rest

    if force == "tip" or (force is None and draw < p_tip):
        r = rng.random()
        if r < CAR_TIP_MIX[0]:
            pose = "side"
            roll = rng.choice((-1.0, 1.0)) * rng.uniform(74.0, 106.0)
            pitch = 0.0
        elif r < CAR_TIP_MIX[0] + CAR_TIP_MIX[1]:
            # THE FULL TURN, not 90 more degrees: a car that has gone all the
            # way over rests on its roof and pillars.
            pose = "roof"
            roll = rng.choice((-1.0, 1.0)) * rng.uniform(158.0, 202.0)
            pitch = 0.0
        else:
            pose = "nose"
            roll = rng.uniform(-22.0, 22.0)
            # 38-72 STOOD A VAN ON END. Reviewed on sight 2026-08-27 — "the
            # car seems to have been turned 90 into the ground, which looks
            # weird" — and the render is a 4.7 m box standing vertically in
            # the middle of a road like a monolith. At 72 degrees plus 30 of
            # roll the silhouette is indistinguishable from 90, and a car
            # balanced on its bumper is not a pose anything comes to rest in.
            # 30-52 is a car with its nose down and its tail in the air,
            # which is what the damage photographs actually show.
            pitch = rng.choice((-1.0, 1.0)) * rng.uniform(30.0, 52.0)
        # The plank field's reach, and the same exponent, so a vehicle and the
        # boards off the house beside it are thrown by one gradient.
        d = float(throw_m) * (it ** 1.6) * rng.uniform(0.2, 0.9)
    else:
        pose = "shoved"
        roll = rng.uniform(-7.0, 7.0)
        pitch = 0.0
        d = ((CAR_SHOVE_M[0] + CAR_SHOVE_M[1] * it)
             * rng.uniform(*CAR_SHOVE_JITTER))

    spread = CAR_SPREAD_DEG[pose]
    travel = float(throw_deg) + rng.uniform(-spread, spread)
    ta = math.radians(travel)

    # ---- march until something stops it -----------------------------------
    # Stepped rather than solved, because the blocker set is a soup of discs
    # and the first one HIT is what matters, not the nearest one. The probe is
    # the car's NOSE (its centre plus half its length), so a car jams when its
    # bumper reaches the wall rather than when its middle does.
    arrested = None
    if blocked is not None and d > 0.0:
        half = float(length_m) * 0.5
        s = 0.0
        while s < d:
            s = min(d, s + CAR_MARCH_M)
            tag = blocked(float(x) + math.cos(ta) * (s + half),
                          float(y) + math.sin(ta) * (s + half))
            if tag is not None:
                arrested = tag
                d = max(0.0, s - CAR_JAM_GAP_M)
                break

    # A car that slammed into something and stayed on its wheels does not stop
    # level — it rides up whatever stopped it. Under the 30 degree `toppled`
    # line by construction; see `CAR_JAM_PITCH_DEG`.
    if arrested is not None and pose == "shoved" and it > 0.5:
        pitch = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_JAM_PITCH_DEG)

    # A NOSED-IN CAR HAS TO HAVE HIT SOMETHING. This is the physical content
    # of the pose and it was missing: `nose` was drawn from the tip mix and
    # then thrown down an empty street, so the scene got a car standing on its
    # bumper in open carriageway with nothing in front of it. A car pitches
    # onto its nose because its front end stopped against a wall, a tree or
    # another car and its back end kept going — no blocker, no nose. Demoted
    # to the pose it would otherwise have taken, which is the commonest one.
    if pose == "nose" and arrested is None:
        pose = "side"
        roll = rng.choice((-1.0, 1.0)) * rng.uniform(74.0, 106.0)
        pitch = 0.0

    # ---- the resting heading ----------------------------------------------
    if long_axis_deg is None:
        # No art offset was supplied, so an absolute bearing cannot be
        # computed — fall back to an undirected wobble and say so here rather
        # than authoring a confidently wrong heading.
        yaw_delta = rng.uniform(-38.0, 38.0) if pose == "shoved" \
            else rng.uniform(-80.0, 80.0)
    elif pose in ("side", "roof"):
        target = travel + rng.choice((-90.0, 90.0)) + rng.gauss(0.0, 30.0)
        yaw_delta = _wrap180(target - float(long_axis_deg))
    elif pose == "nose":
        yaw_delta = _wrap180(travel + rng.gauss(0.0, 22.0)
                             - float(long_axis_deg))
    else:
        yaw_delta = rng.uniform(-38.0, 38.0)

    rest.update({"moved": True, "pose": pose,
                 "dx": math.cos(ta) * d, "dy": math.sin(ta) * d, "d_m": d,
                 "roll_deg": roll, "pitch_deg": pitch,
                 "yaw_delta_deg": yaw_delta, "arrested_by": arrested,
                 "toppled": abs(roll) > 30.0 or abs(pitch) > 30.0})
    return rest


# ---------------------------------------------------------------------------
# posing props that are already on the stage
# ---------------------------------------------------------------------------

def toss_prim(stage, prim_path, dx, dy, roll_deg, yaw_jitter_deg, pitch_deg=0.0,
          seat=True):
    """Move, roll and pitch an already-placed prop. Returns True if it moved.

    THE ONE THING A TORNADO DOES THAT A FIRE DOES NOT, applied to the things
    the assembly has already built. A burnt car is an ordinary car standing in
    a black landscape; a car in a track has been PUSHED, and very often rolled
    onto its side or its roof. That is cheap to author here and impossible to
    bake into an archetype, because these are per-placement props rather than
    per-type.

    IT MEASURES THE PROP TO SEAT IT, which the first version did not. That one
    took a `lift_m` argument and every caller passed the same guessed 0.7 m for
    anything rolled past 30 degrees. A car pivots about its own origin — which
    on these assets is the middle of the wheelbase, ON the ground — so rolling
    it 90 degrees swings half the BODY below grade, and how far below is half
    the car's WIDTH, which is not 0.7 m and is not the same for a hatchback and
    a pickup. Half of them sank into the road and the rest hovered.

    So: take the prop's own untransformed bounding box, apply exactly the
    rotation about to be authored, and lift by whatever that puts under the
    old ground contact. Exact for any prop and any angle, and it costs one
    bbox query.

    Rebuilds the op order rather than appending, for the same reason
    `vegetation.tip_tree` does: a prim placed by `apply_placements` carries
    translate + rotateZ + maybe scale, and appending a rotate to the end of
    that list applies it in the wrong frame — the prop orbits the world origin
    instead of rolling in place.
    """
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    sx, sy, sz = ((float(sc[0]), float(sc[1]), float(sc[2])) if sc is not None
                  else (1.0, 1.0, 1.0))

    # CLEAR THE OPS BEFORE MEASURING, and this is not tidiness — it is a bug
    # fix (2026-08-27).
    #
    # `ComputeUntransformedBound` does NOT exclude the prim's own `rotateZ`.
    # Measured on a 2.0 x 0.5 x 1.0 m box referenced under translate +
    # rotateZ + scale:
    #
    #     yaw   0     -> (-1.000, -0.250, 0) .. (1.000, 0.250, 1)   correct
    #     yaw  37 deg -> (-1.240, -1.211, 0) .. (1.240, 1.211, 1)   WRONG
    #
    # The box comes back rotated and then re-aligned, so the across-axis half
    # extent is inflated 0.25 -> 1.211 m, nearly five-fold. `lift` is computed
    # from exactly those x/y extents (`|y| sin roll`, `|x| sin pitch`), so a
    # 4.7 x 1.8 m car parked at yaw 45 reported a half-width of ~3.25 m
    # instead of 0.9 and, rolled onto its side, was lifted 3.25 m — 2.35 m of
    # air under it. Worst at 45 degrees, exact at multiples of 90, which is
    # why it hid for so long. `scale` IS correctly excluded, so the separate
    # multiply below is right and stays.
    #
    # Clearing the op order first makes the query measure the referenced
    # geometry and nothing else. The ops are rebuilt from `vals` below either
    # way, so nothing is lost by clearing early.
    #
    # (The obvious other suspect — that an INSTANCED prim returns an empty
    # bound and silently drops `lift` to 0 — was tested and is FALSE: the
    # bound comes back byte-identical to the non-instanced case.)
    old_yaw = float(vals.get("rotateZ") or 0.0)
    xf.SetXformOpOrder([])

    lift = 0.0
    if seat and (abs(roll_deg) > 0.01 or abs(pitch_deg) > 0.01):
        try:
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_])
            r = bc.ComputeUntransformedBound(prim).ComputeAlignedRange()
            if not r.IsEmpty():
                mn, mx = r.GetMin(), r.GetMax()
                cr, sr = (math.cos(math.radians(roll_deg)),
                          math.sin(math.radians(roll_deg)))
                cp, sp = (math.cos(math.radians(pitch_deg)),
                          math.sin(math.radians(pitch_deg)))
                z_before = float(mn[2]) * sz          # current ground contact
                z_after = None
                # Only Z matters, and rotateZ does not change it — so the
                # world z of a corner is the z of Ry(pitch) * Rx(roll) applied
                # to the scaled corner. Eight corners, take the lowest.
                for cx_ in (float(mn[0]), float(mx[0])):
                    for cy_ in (float(mn[1]), float(mx[1])):
                        for cz_ in (float(mn[2]), float(mx[2])):
                            x_, y_, z_ = cx_ * sx, cy_ * sy, cz_ * sz
                            # Rx(roll): y,z rotate
                            y2, z2 = y_ * cr - z_ * sr, y_ * sr + z_ * cr
                            # Ry(pitch): z,x rotate
                            z3 = -x_ * sp + z2 * cp
                            z_after = z3 if z_after is None else min(z_after,
                                                                     z3)
                if z_after is not None:
                    lift = z_before - z_after
        except Exception:
            lift = 0.0

    xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]) + dx, float(t[1]) + dy,
                                     float(t[2]) + lift))
    xf.AddRotateZOp().Set(old_yaw + float(yaw_jitter_deg))
    # ORDER IS Rz * Ry * Rx, matching the seating maths above. Pitch before
    # roll in the list means roll is applied first about the car's own long
    # axis, which is what "rolled onto its side" means; pitching a car that is
    # already on its side then stands it on its nose, which is the pose a car
    # thrown against a wall ends up in.
    if abs(pitch_deg) > 0.01:
        xf.AddRotateYOp().Set(float(pitch_deg))
    if abs(roll_deg) > 0.01:
        xf.AddRotateXOp().Set(float(roll_deg))
    if sc is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    return True


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
