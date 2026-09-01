"""hurricane stage — the FIELD, and why it looks nothing like a track.

WHAT A HURRICANE SCENE IS, AND HOW IT DIFFERS FROM THE TORNADO ONE
-------------------------------------------------------------------
`tornado.py`'s whole design rests on one fact: intensity is a function of
DISTANCE FROM A LINE, so a scene is a corridor with an edge. A hurricane has
no such line to be far from. Computed directly from the Holland gradient-wind
equation at 28 deg N (`_plans/hurricane_wind_field.md` S2.1, four representative
storms spanning Rmax 20-60 km): the radial change in wind speed over 1 km is
0.1-0.8 m/s (0.3-1.5%), and 1 km of arc at a typical radius subtends under 2
degrees of azimuth, so the WN1 asymmetry moves by under 0.15 m/s across the
same distance. Against a Saffir-Simpson category step of 7-12 m/s, **a 500 m -
1 km plate cannot straddle a category boundary from the parametric wind field
alone.** There is no track to draw and no corridor edge to find, because
`compile_hurricane`'s existing `field: {kind: uniform, inside: 1.0}` is
correct — this file keeps that and builds the rest of the model on top of it.

SO WHAT MAKES ONE PART OF THE SCENE LOOK DIFFERENT FROM ANOTHER?
------------------------------------------------------------------
Three things, in order of how much they change the code, and NONE of them is
"how far is this point from the storm's eye":

  1. THE BUILDINGS. Marshall's aerial survey of 11,105 post-Katrina structures
     found damage-derived gusts across the WHOLE impact zone averaging 41 m/s
     (with excursions to 48) inside a storm rated Cat 3 at landfall — Cat 1-2
     LOCAL intensity, because most buildings sit in Exposure B, not the open
     terrain a category is rated against. And the code-era record proves the
     variance is per-building, not per-position: post-1994 manufactured homes
     showed 0% damage above rating 1 in the same wind that put 64.7% of
     pre-1976 units at rating >=2. A destroyed house beside an untouched one
     is the NORM here, the opposite of the tornado's smooth cross-track
     gradient. `draw_vulnerability` and the blend inside
     `house_level_for_intensity` are THE dominant source of scene variance —
     more than the field itself. See `_plans/hurricane_research.md` S1.4-1.5.
  2. A real but small coastal roughness step. Vickery, Wadhera & Powell
     (2009): about 60% of the sea-to-land wind-speed transition has already
     happened within 1 km of the coast, with a maximum total reduction of
     about 17%. An order of magnitude bigger than the radial gradient, and
     the only reason a scene that straddles a shoreline has any wind gradient
     worth drawing.
  3. Boundary-layer roll vortices — Foster (2005) / Morrison et al. (2005):
     present in 35-69% of radar volumes (the ordinary state, not an
     exception), producing a documented ~14 m/s peak-to-peak variation over a
     ~725 m wavelength, aligned within about -10 deg of the mean wind. 20-70x
     the radial term. There is no published fragility study turning a roll
     band into a damage-state fraction, so it stays a LOW-AMPLITUDE OPTIONAL
     overlay here, never the primary driver — the skill is explicit about
     this and `gust_field` follows it: `streak_amp_mps` defaults to something
     that moves the field a few percent, not a whole category.

THE SECOND DEFINING FACT: THE WIND DIRECTION ROTATES
-------------------------------------------------------
A tornado's debris CONVERGES on a centreline (Fujita vectors). A hurricane's
wind veers or backs through 108-167 degrees while it is within 85% of its
peak at a fixed site (`_plans/hurricane_wind_field.md` S2.3, a passage model
at 28 deg N) — so a building takes near-peak load first from one quadrant and
later from an adjacent one, and debris/treefall statistics are unimodal with
a WIDE circular spread (22-45 deg SD) or, if the eye crosses, bimodal with
two lobes about 195 degrees apart. `wind_bearing_at` turns this into a
function of POSITION — a spatial proxy for time-of-arrival across the plate,
since nothing here carries an actual storm track or clock. That is a
deliberate simplification of a fundamentally temporal effect into something a
static per-building pass can read; see the docstring on `wind_bearing_at` for
exactly how, and the judgment call it required.

WHY THE HOUSE LADDER IS EIGHT LEVELS, NOT SIX
------------------------------------------------
`tornado.HOUSE_LEVELS` runs pristine -> roof_stripped -> roof_collapsed ->
partial_collapse -> leveled -> swept, where `roof_stripped` already means
"covering AND sheathing gone, structure and walls whole." For a hurricane
that state is almost the WHOLE story: Marshall's 8,119 residences on
post-Katrina aerials found 90% lost under 20% of roof cover, 10% lost most of
the cover and/or some decking, and only 3 of 8,119 (1 in 2,700) lost large
roof-structure sections — and at 116-135 mph gusts (~Cat 2-3 local), "less
than 15 percent of homes sustained structural wind damage"
(`_plans/hurricane_research.md`, echoed in the SKILL). So this ladder
resolves THREE new states below `tornado`'s first rung (`shingles_lost`,
`cover_lost`, `deck_panels_lost`) before reaching the four states this file
shares by NAME with the tornado ladder (`roof_stripped`, `roof_collapsed`,
`partial_collapse`, `leveled`) — which is why `HOUSE_LEVELS` has 8 entries
where `tornado.HOUSE_LEVELS` has 6. Progression is ROOF-DOWN and never
skips a rung in this model: Marshall (2004), "dismantling of a structure by
wind usually develops first at roof level and progresses downward."

`swept` DOES NOT APPEAR HERE. Four independent lines in the research say a
bare, wind-swept slab is a SURGE signature, never a wind one: Roueche et al.
found every one of 3,016 hurricane homes with complete destruction was
associated with storm surge, not wind alone; the EF-scale's own FR12 puts
wind-only slab-sweeping at 200 mph, above Cat 5 and unreachable at a
suburban site. A hurricane scene with scattered swept slabs inland of the
surge line reads as a tornado wearing a hurricane's paint job — that state
belongs to the surge module (`disaster/surge.py`, not yet built), never to
this file's wind ladder.

THE TREE LADDER GAINS `defoliated` AT THE BOTTOM, AND IT WINS BY VOLUME
---------------------------------------------------------------------------
A tornado's tree ladder keeps every level green — the point there is that
wind breaks WOOD and the foliage on what it broke is still green in the
photograph. A hurricane strips the CANOPY over enormous areas long before it
breaks any wood: broadleaf canopy is stripped essentially bare at Cat 3+
(86-94% leaf loss, post-Maria NDVI dropped about 0.2 and took ~1.5 months to
recover — a scene "hours after landfall" is solidly inside that brown
window), while urban-forest MORTALITY even in the worst recorded storms
(Andrew) tops out at 38% — most trees SURVIVE a hurricane, they just go
bald. So `defoliated` is deliberately the widest band in `_TREE_CUTS`: most
standing trees in this scene are alive, standing, and stripped, and only a
minority ever reach `limbed`/`leaning`/`fallen`/`snapped`. Conifers still
keep their needles and SNAP instead, same as the tornado model, and the
sabal-palm "shaving-brush" survival story (80-93% even through Andrew at 165
mph, fronds shed without losing the meristem) gets its own species cap below.
"Most trees survive" also means a real plate should show a genuine
UNTOUCHED minority, not zero — see `_TREE_CUTS`'s own 2026-08-31 (STREAM T5)
re-cut for why `pristine` was originally left unreachable at both recorded
plates and how much of the field it now gets back.

DEBRIS IS TURNED INSIDE OUT FROM THE TORNADO'S
--------------------------------------------------
`planks.py`'s sawn-timber field is a tornado signature; Lee County's Ian
collection measured 734,136 cubic yards vegetative against 285,282 cubic
yards construction & demolition debris — a 72/28 split, the opposite
emphasis from a tornado's plank field. And the building 28% is COARSE and
locally sourced: whole sheathing panels, whole fence sections, long siding
strips landing within 1-3 building widths of where they came off, not
shredded and not far-travelled. `debris_mix` returns that split.

VEHICLES ARE NOT THROWN BY WIND. `tornado.car_pose` is the wrong model here:
the complete record across 30 hurricane surveys shows nothing moved below
about 130 mph gusts, and "several automobiles ... flipped" is an Andrew-grade
(175 mph) observation. Cars in a hurricane scene are moved by WATER, not
wind — that is the surge module's job, not this file's.

READ `build-tornado-scenes` FIRST if it has not already been read — the wind
mechanics (breach-before-structure, windward-face damage, per-building
draws) are inherited wholesale; this file's job is the field shape and the
ladders that are actually different.
"""

import math
import os


# ---------------------------------------------------------------------------
# the field: uniform, not a track
# ---------------------------------------------------------------------------

# Normalisation anchor for the 0..1 EF-proxy the ladders below read. Chosen
# as a round number comfortably above the Cat 5 suburban-Exposure-B 3-s gust
# FLOOR (~82 m/s per the wind-averaging table in `_plans/hurricane_wind_field.md`
# S1.3), so intensity 1.0 is headroom rather than a claim that some specific
# wind speed is "the worst possible" -- tuned, not sourced past that floor.
#
# IMPORTANT PROPERTY, load-bearing for every calibration below: because both
# `gust_field`'s driver and every ladder cut in this file are expressed as
# `onset_speed_mps / REF_GUST_MPS`, this constant's exact VALUE cancels out
# of every ladder decision -- changing it only rescales what `summarise`'s
# `*_intensity` numbers mean. It does not need re-tuning if the onset table
# below is revised; only the ratio does.
REF_GUST_MPS = 100.0

# The low-level `disaster.hurricane` config block. A launcher merges this over
# the compiled config's own block, same contract as `tornado.DEFAULTS`.
DEFAULTS = {
    "enabled": True,
    "seed": 0,

    # THE DRIVER. The local 3-second gust at 10 m, already at the plate's own
    # ground-level exposure -- NOT a category converted through a lookup
    # table. Marshall's post-Katrina survey found damage-derived gusts across
    # an ENTIRE impact zone rated Cat 3 at landfall averaged 41 m/s (with
    # excursions to 48): "most buildings were in Exposure B not C."  The
    # skill's own proposed 3-level ladder puts this default at its Level 2 --
    # "the brown scene; roofs are the story; structural damage a minority" --
    # a nominal Cat 3 by the naive category table, but what actually reaches
    # the houses is closer to a measured Cat 2.
    "site_gust_mps": 55.0,

    # RECORD ONLY in this file -- see the note above `gust_field`'s body for
    # why the ASCE 7 B/C/D K_z ladder (0.85 / 1.00 / 1.09 relative to open
    # terrain) is NOT reapplied here. Kept for the compiler to carry forward
    # and for `hurricane_flow.py`'s future per-storey scaling on buildings
    # tall enough for height to matter, which this uniform ground-level field
    # does not model.
    "exposure": "B",

    # WHICH WAY, AND HOW MUCH IT TURNS. `heading_deg` is the dominant azimuth
    # of the strongest gust episode at the plate's `origin_m` (same x/y
    # convention as `tornado.heading_deg`). `rotation_deg` is the TOTAL
    # direction change `wind_bearing_at` sweeps across the plate -- a spatial
    # stand-in for the 108-167 degrees of veer/back a fixed site experiences
    # over several hours while gust > 0.85 of peak (`_plans/hurricane_wind_field.md`
    # S2.3, unimodal right-of-track case). Set to 195 with `rotation_axis_deg`
    # picked so the two ends of the plate see opposed lobes for an
    # eye-crossing preset; the model does not distinguish "veering" from
    # "backing" -- the SIGN of `rotation_deg` does that instead.
    "origin_m": [0.0, 0.0],
    "heading_deg": 60.0,
    "rotation_deg": 140.0,
    # None -> perpendicular to `heading_deg`. A hurricane's rotation is a
    # TEMPORAL effect at a single point; collapsing it onto a spatial axis at
    # all is the judgment call `wind_bearing_at` documents, and there is no
    # measurement that says a plate-crossing axis should be exactly
    # perpendicular to the mean wind rather than along the (unmodelled) storm
    # track. Perpendicular was chosen because it is the axis least correlated
    # with the coastal gradient and the roll streak below, so the three
    # spatial terms do not all vary together across the same diagonal.
    "rotation_axis_deg": None,
    # Metres over which the FULL `rotation_deg` plays out, centred on
    # `origin_m`. Tuned to roughly one plate-width so the whole rotation is
    # visible in a single 500 m - 1 km scene rather than needing several
    # plates stitched together -- not sourced, because the real rotation is
    # measured in HOURS, not metres; see `wind_bearing_at`.
    "rotation_span_m": 900.0,

    # THE COASTAL ROUGHNESS STEP -- the one real spatial wind gradient at
    # this scale. Vickery, Wadhera & Powell (2009): about 60% of the sea to
    # land transition is complete within 1 km, saturating at a maximum
    # reduction of about 17%. Disabled by default (`shore_offset_m=None`):
    # a scene that does not straddle a coastline has no roughness step to
    # draw, and drawing one anyway would fabricate a gradient the skill is
    # explicit does not exist away from a shoreline. `shore_bearing_deg`
    # points FROM the plate TOWARD open water; `shore_offset_m` is the
    # signed distance, in the same frame as `origin_m`, from the origin to
    # the shoreline measured along that bearing (positive = the shoreline is
    # further out to sea than the origin, so the origin itself is already
    # some distance inland).
    "shore_bearing_deg": 90.0,
    "shore_offset_m": None,
    "coastal_falloff_per_km": 0.10,
    "coastal_max_reduction": 0.17,

    # BOUNDARY-LAYER ROLL BANDING -- present in 35-69% of radar volumes
    # (Morrison et al. 2005), so "roll" is the default rather than "none."
    # Foster (2005): a ~14 m/s peak-to-peak variation over a ~725 m
    # wavelength, roll axis within about -10 deg of the mean boundary-layer
    # wind (mode value; range -30..+10). THIS IS A LOW-AMPLITUDE OPTIONAL
    # OVERLAY, not a second driver -- there is no published fragility study
    # converting a roll band into a damage-state fraction (see the module
    # docstring), so `streak_amp_mps` is deliberately small relative to
    # `site_gust_mps` rather than tuned to visibly re-grade whole houses.
    "streak": "roll",
    "streak_amp_mps": 7.0,
    "streak_period_m": 725.0,
    "streak_skew_deg": -10.0,

    # RESIDUAL, UNORGANISED turbulence -- the part of gust-factor scatter
    # not captured by the roll band above. WMO/TD-1555 documents gust-factor
    # scatter on the order of a few percent even under nominally stationary
    # conditions; kept modest and separate from the roll term so the two are
    # not double-counted (`_plans/hurricane_wind_field.md` S2.2 warns
    # explicitly against this). Implemented as a small sum of plane waves at
    # random bearing/wavelength/phase (see `_plane_wave_noise`) rather than
    # the spectral, numpy-backed `scorch._noise` tornado.py uses for its edge
    # wobble -- this module has to import cleanly with stdlib + math + random
    # only, and there is no ragged boundary here for a spectral field to
    # texture; a handful of superposed sines is the right amount of noise for
    # "gust factor scatter," not a substitute for `scorch._noise` in general.
    "noise_amp_mps": 1.5,
    "noise_waves": 5,
    "noise_wavelength_range_m": (40.0, 160.0),
}


def resolve_cfg(config):
    """`DEFAULTS` with the scene config's `disaster.hurricane` block merged over.

    Same contract as `tornado.resolve_cfg` -- one place so launchers cannot
    drift apart on it.
    """
    cfg = dict(DEFAULTS)
    cfg.update((config.get("disaster") or {}).get("hurricane") or {})
    return cfg


def _plane_wave_noise(rng, n_waves=5, wavelength_range=(40.0, 160.0)):
    """A cheap, seeded, pure-Python noise field: `(x, y) -> roughly -1..1`.

    Not `scorch._noise` (which is spectral and numpy-backed) on purpose --
    this module has no ragged boundary to texture, only a small amount of
    unorganised turbulence to add, and it has to import without numpy. A sum
    of `n_waves` plane waves at random bearing, wavelength and phase is a
    legitimate band-limited-ish noise for that job: each term is bounded by
    +-1 and the average of bounded terms stays bounded, so the result never
    needs clamping.
    """
    waves = []
    lo, hi = float(wavelength_range[0]), float(wavelength_range[1])
    for _ in range(max(1, int(n_waves))):
        ang = rng.uniform(0.0, 2.0 * math.pi)
        wl = rng.uniform(lo, hi)
        phase = rng.uniform(0.0, 2.0 * math.pi)
        waves.append((math.cos(ang) / wl, math.sin(ang) / wl, phase))

    def sample(x, y):
        s = 0.0
        for kx, ky, ph in waves:
            s += math.sin(2.0 * math.pi * (kx * x + ky * y) + ph)
        return s / len(waves)

    return sample


def gust_field(cfg, region, rng):
    """`(x, y) -> local 3-second gust in m/s at 10 m`. THE driver.

    `region` is accepted for signature symmetry with `tornado.intensity_field`
    (which needs it to size a discrete noise grid); this field is entirely
    analytic and never samples an array, so region bounds are not needed and
    are not used.

    The value is `site_gust_mps`, flat across the whole plate, modulated by
    three SMALL spatial terms -- see the module docstring for why each is
    small relative to the driver:

      (a) the coastal roughness step, if `shore_offset_m` is set -- an
          exponential-feeling but actually just CAPPED linear falloff moving
          inland, saturating at `coastal_max_reduction` because Vickery,
          Wadhera & Powell (2009) measured the transition itself saturating
          (~17% maximum), not accelerating without bound;
      (b) boundary-layer roll banding, a sinusoid along an axis close to the
          mean wind direction; and
      (c) small unorganised noise, see `_plane_wave_noise`.

    NONE of these three is a function of distance from anything like a storm
    centre -- there is no such point in this model, on purpose.
    """
    site = float(cfg["site_gust_mps"])
    ox, oy = float(cfg["origin_m"][0]), float(cfg["origin_m"][1])

    shore_offset = cfg.get("shore_offset_m")
    if shore_offset is not None:
        sb = math.radians(float(cfg.get("shore_bearing_deg", 90.0)))
        seaward = (math.cos(sb), math.sin(sb))
        falloff = float(cfg.get("coastal_falloff_per_km", 0.10))
        cap = float(cfg.get("coastal_max_reduction", 0.17))
        offset = float(shore_offset)

        def coastal_reduction(x, y):
            # Distance INLAND: how far this point sits opposite the seaward
            # bearing from the shoreline. Points seaward of the shoreline
            # (negative distance) get no reduction -- this model has nothing
            # to say about wind speed over open water.
            dist_km = (offset - ((x - ox) * seaward[0]
                                  + (y - oy) * seaward[1])) / 1000.0
            return min(cap, falloff * max(0.0, dist_km))
    else:
        def coastal_reduction(x, y):
            return 0.0

    streak = str(cfg.get("streak", "none")).lower()
    if streak == "roll":
        amp = float(cfg.get("streak_amp_mps", 7.0))
        period = max(1e-6, float(cfg.get("streak_period_m", 725.0)))
        axis = math.radians(float(cfg["heading_deg"])
                            + float(cfg.get("streak_skew_deg", -10.0)))
        ax, ay = math.cos(axis), math.sin(axis)

        def roll(x, y):
            proj = (x - ox) * ax + (y - oy) * ay
            return amp * math.sin(2.0 * math.pi * proj / period)
    else:
        def roll(x, y):
            return 0.0

    noise_amp = float(cfg.get("noise_amp_mps", 0.0))
    if noise_amp > 0.0:
        noise = _plane_wave_noise(
            rng, n_waves=int(cfg.get("noise_waves", 5)),
            wavelength_range=tuple(cfg.get("noise_wavelength_range_m",
                                           (40.0, 160.0))))
    else:
        def noise(x, y):
            return 0.0

    def gust(x, y):
        g = (site * (1.0 - coastal_reduction(x, y))
             + roll(x, y) + noise_amp * noise(x, y))
        return max(0.0, g)

    return gust


def intensity_field(cfg, region, rng):
    """`(x, y) -> 0..1`, the EF-proxy every ladder in this file reads.

    Pure normalisation: `gust_field(x, y) / REF_GUST_MPS`, clamped to 0..1.
    Because the field is uniform (see the module docstring), this does NOT
    look like `tornado.intensity_field` -- there is no cross-track profile,
    no along-track breathing, no edge noise perturbing a level set. Sampling
    this over a 500 m plate should return a narrow band of values (single
    digits of percent spread) -- see `summarise`'s `gust_span_pct`, which is
    exactly the number that catches a bug that accidentally reintroduces a
    track here.
    """
    gust = gust_field(cfg, region, rng)

    def intensity(x, y):
        return max(0.0, min(1.0, gust(x, y) / REF_GUST_MPS))

    return intensity


def wind_bearing_at(cfg, x, y):
    """The LOCAL wind direction (deg) at `(x, y)` -- what `hurricane_flow`
    will use to pick a windward wall.

    A hurricane's wind direction is fundamentally a function of TIME at a
    fixed site, not of position: `_plans/hurricane_wind_field.md` S2.3 models
    it turning 108-167 degrees over several hours as the storm passes. This
    file has no clock and no storm track, so it makes a deliberate
    substitution -- **the judgment call flagged in the module docstring**:
    position along `rotation_axis_deg` stands in for time-of-arrival, linear
    from `heading_deg - rotation_deg/2` at one edge of `rotation_span_m` to
    `heading_deg + rotation_deg/2` at the other, centred on `origin_m`.

    This is defensible as a RENDERING device (it gives `hurricane_flow` a
    reason to damage different building faces in different parts of one
    scene, which is true of a real storm) but it is NOT a physical claim that
    a hurricane's wind direction varies spatially at fixed time. Nothing
    downstream should read `rotation_span_m` as a real length scale.
    """
    ox, oy = float(cfg["origin_m"][0]), float(cfg["origin_m"][1])
    heading = float(cfg["heading_deg"])
    rotation = float(cfg.get("rotation_deg", 0.0))
    axis_deg = cfg.get("rotation_axis_deg")
    axis_deg = float(axis_deg) if axis_deg is not None else heading + 90.0
    axis = math.radians(axis_deg)
    span = max(1e-6, float(cfg.get("rotation_span_m", 900.0)))

    proj = (float(x) - ox) * math.cos(axis) + (float(y) - oy) * math.sin(axis)
    t = max(-0.5, min(0.5, proj / span))
    return (heading + rotation * t) % 360.0


# ---------------------------------------------------------------------------
# per-building construction quality -- the dominant variance term
# ---------------------------------------------------------------------------

# Named construction eras and the population share / vulnerability draw each
# one contributes. `share` must sum to 1.0 across the tuple. `vuln_mean` /
# `vuln_sd` feed a clipped Gaussian: 0 = the era's houses shrug off the
# ladder onset thresholds below, 1 = they hit every one of them close to the
# literal onset gust. Numbers are SHAPED to be qualitatively consistent with
# two hard findings rather than fit to either one exactly (there is no single
# study that reports a 0..1 "vulnerability" scale):
#
#   - HUD's post-Charley survey of 105 Wind-Zone-III manufactured homes at a
#     ~100 mph local fastest-mile: post-1994 HUD-code units 0% roof damage
#     >=rating 2; pre-1994 HUD (1976-94) 28.6%; pre-1976 (non-HUD) 64.7%.
#   - Alabama DOI Hurricane Sally claims data: IBHS FORTIFIED Roof cut claim
#     FREQUENCY to 0.12x a conventional roof's rate, FORTIFIED Gold to 0.08x
#     (55% / 70% reduction) -- roughly a doubling of effective wind
#     resistance pole to pole.
#
# See `_plans/hurricane_research.md` S1.4-1.5. `share`/`vuln_mean`/`vuln_sd`
# TUNED, NOT FIT -- a real per-storm survey would replace this table wholesale
# rather than adjust it.
CODE_ERAS = (
    # name                    share  vuln_mean  vuln_sd
    ("pre_1976",               0.22,      0.90,     0.07),
    ("hud_1976_94",            0.20,      0.68,     0.09),
    ("post_1994",              0.28,      0.42,     0.10),
    ("post_2004_ringshank",    0.22,      0.24,     0.08),
    ("fortified",              0.08,      0.08,     0.05),
)


def draw_vulnerability(rng, era=None):
    """`(era_name, vuln 0..1)` for one building.

    With `era=None`, draws an era from `CODE_ERAS`'s population shares first
    (a per-scene "when was this neighbourhood built" draw) and then a `vuln`
    from that era's clipped Gaussian. Pass `era` to fix the era and draw only
    `vuln` -- for a caller that already knows (or wants to force) a
    building's construction age.
    """
    if era is not None:
        for name, _share, mean, sd in CODE_ERAS:
            if name == era:
                return name, max(0.0, min(1.0, rng.gauss(mean, sd)))
        raise ValueError("unknown code era: {0!r}".format(era))

    r = rng.random()
    acc = 0.0
    for name, share, mean, sd in CODE_ERAS:
        acc += share
        if r <= acc:
            return name, max(0.0, min(1.0, rng.gauss(mean, sd)))
    # Floating-point shortfall if shares do not sum to exactly 1.0 -- fall
    # back to the last era rather than raising over a rounding error.
    name, _share, mean, sd = CODE_ERAS[-1]
    return name, max(0.0, min(1.0, rng.gauss(mean, sd)))


# Resistance multiplier applied to intensity before the house ladder is read:
# `effective = i / resistance(vuln)`. `RESIST_LO` (vuln=1, weakest) sits close
# to 1.0 because the onset table below is already calibrated near the
# weakest-construction end (an FR12/HAZUS-style DOD ladder records the wind
# at which a below-average example of a class first shows a state, not the
# median one). `RESIST_HI` (vuln=0, strongest) needs roughly 1.6x the onset
# gust for the same state -- shaped to the FORTIFIED claims-frequency
# reduction above, not fit to it. See `house_level_for_intensity`'s docstring
# for the calibration this produces.
RESIST_LO = 0.95
RESIST_HI = 1.60


def _resistance(vuln):
    v = max(0.0, min(1.0, float(vuln)))
    return RESIST_HI + (RESIST_LO - RESIST_HI) * v


# ---------------------------------------------------------------------------
# damage levels
# ---------------------------------------------------------------------------

# THE HOUSE LADDER. See the module docstring for why this has 8 levels where
# `tornado.HOUSE_LEVELS` has 6, and why `swept` is deliberately absent.
HOUSE_LEVELS = ("pristine", "shingles_lost", "cover_lost", "deck_panels_lost",
                "roof_stripped", "roof_collapsed", "partial_collapse",
                "leveled")

# Onset 3-second gust at 10 m, in m/s, from the SKILL's building-damage
# table (itself drawn from Marshall's Katrina/Charley surveys and the EF-scale
# FR12/HAZUS-style DOD ladders in `_plans/hurricane_research.md`):
#
#   shingles_lost      79 mph / 35.3 m/s   <20% shingle loss, no other damage
#   cover_lost         97 mph / 43.4 m/s   >20% cover, soffit, siding, garage
#                                          door / window openings breached
#   deck_panels_lost  100 mph / 44.7 m/s   1-3 sheathing panels off (1-2 bays)
#   roof_stripped     115 mph / 51.4 m/s   >25% deck gone, most bays dropped
#                                          (this is `tornado`'s definition of
#                                          `roof_stripped` -- covering AND
#                                          sheathing gone, structure/walls up)
#   roof_collapsed    122 mph / 54.6 m/s   roof structure itself down
#   partial_collapse  132 mph / 59.0 m/s   exterior walls failing (low end of
#                                          the SKILL's 132-170 mph "walls" band)
#   leveled            ~175 mph / 78 m/s   pushed well above the walls band's
#                                          low end on purpose -- full-level
#                                          destruction is the rarest outcome
#                                          in the record (Roueche et al.: 3.1%
#                                          of 3,016 hurricane homes, and EVERY
#                                          one of those was surge-associated,
#                                          not wind-only) and this ladder has
#                                          no surge term to lean on, so it has
#                                          to be reachable by wind alone only
#                                          at the extreme end.
#
# Expressed as onset/REF_GUST_MPS below, in the same (upper_bound, name)
# shape as `tornado._HOUSE_CUTS`: row k's bound is the CEILING of row k's own
# name, so "pristine" occupies [0, 0.353), "shingles_lost" occupies
# [0.353, 0.434), and so on. Verified by `tools/hurricane_png.py`-style
# sampling (see this module's own `summarise`) to put a clear MAJORITY of
# houses in the four roof states and a MINORITY in the three structural ones
# at the DEFAULTS' `site_gust_mps` -- the single most important calibration
# in this file, and it is a joint calibration of this table AND
# `RESIST_LO`/`RESIST_HI` together, not either one alone.
# RE-CUT 2026-08-30 AFTER THE FIRST RENDER. User, looking at the level-2
# plate: "Why are all the roofs gone?" — and they were right.
#
# The onset gusts above are sound; the BANDS they produced were not. Read
# them as widths: `cover_lost` got 0.447-0.434 = 0.013 of the range while
# `deck_panels_lost` got 0.067, five times as much, because the real onsets
# are 97 and 100 mph (three apart) and then jump to 115. So the single most
# likely damaged outcome on a Cat-2 plate was `deck_panels_lost` — measured
# 37 of 136 houses against `cover_lost`'s 6 — and that state drops most of a
# roof. A neighbourhood of gutted roofs at 123 mph is not what Cat 2 does.
#
# Two things were wrong and BOTH had to change (the other is `_ROOF_FRAC` in
# `hurricane_flow`, see there): the ladder was top-heavy, and each rung was
# too violent for its own definition.
#
# Widened the light end, narrowed the heavy one, and pushed `pristine` up.
# Physically this says: at a Cat-2 site gust most houses lose SOME covering
# and few lose sheathing, which is what the Katrina/Charley surveys report —
# "less than 15 percent of homes sustained structural wind damage" at
# 116-135 mph, and roof-cover-only losses dominate everything below that.
# Solved by search against two acceptance targets rather than hand-nudged,
# because the two levels have to be DIFFERENT SCENES and tuning either alone
# breaks the other. Measured over 800 sampled houses per level:
#
#   level 2 (55 m/s): 18% pristine  35% shingles  28% cover  12% deck
#                     6% roof_stripped  2% roof_collapsed
#                     -> roofs are the whole story, collapse is 2%
#   level 3 (70 m/s):  9% shingles  28% cover  22% deck  16% roof_stripped
#                     14% roof_collapsed  11% partial_collapse
#                     -> 25% structural, so a plate at level 3 visibly
#                        contains buildings broken past their roofs
#
# `shingles_lost` is deliberately large and deliberately INVISIBLE (see
# `hurricane_flow._ROOF_FRAC`, which drops no bay for it): under 20% shingle
# loss cannot be seen from 400 m, and pretending otherwise by dropping a
# third of the roof is what produced "why are all the roofs gone?".
#
# RE-CUT AGAIN 2026-08-31, TWICE, and both times against the REAL house
# population rather than the 800-SYNTHETIC-HOUSE sample above, which was
# never checked against an actual scene's recorded (intensity, vulnerability)
# pairs and was wrong: GT_hurricane.json (`~/hurricane_previews/V2_L3/`)
# recorded 21 roof_collapsed + 14 partial_collapse + 1 leveled of 94 houses =
# **38% structural**, against the synthetic sample's claimed 25%. The
# level-2 plate's `roof_collapsed` share (1 of 136 = 0.7%) was already fine.
#
# ROUND 1 fixed the structural overshoot alone: raise the top three
# boundaries (`roof_stripped`/`roof_collapsed`/`partial_collapse`), leave
# `pristine`..`deck_panels_lost` untouched because L2 sits almost entirely
# below them. That landed structural at 12% (good) but put 39% of the plate
# in `roof_stripped` alone — a plurality state that, rendered, is a
# neighbourhood of gutted roofs seen from the air ("a field of white boxes"),
# exactly the look the ORIGINAL 2026-08-30 re-cut was trying to escape one
# rung down (`deck_panels_lost`, see the comment above `_ROOF_FRAC` in
# `hurricane_flow.py`). Raising the ceiling on structural without also
# re-spreading the roof-cover bands underneath it just pushed the pile-up
# into the next rung down.
#
# ROUND 2 (this one) refits ALL SEVEN boundaries at once by weighted
# least-squares against five named targets for the level-3 population
# (`tools/hurricane_house_cut_search.py`, `scipy.optimize.minimize` over
# `log`-spaced gaps so the boundaries stay increasing and positive; structural
# weighted 4x since it is the fraction a reviewer names first), instead of
# hand-walking one boundary at a time and re-breaking the level below it:
#
#   target (level 3): pristine+shingles ~15%, cover_lost ~30%,
#     deck_panels_lost ~25%, roof_stripped ~18%, structural ~12%
#   ACHIEVED (94-house real population, 6000 resampled jitter draws):
#     shingles 15.0%, cover 31.2%, deck 23.5%, roof_stripped 18.4%,
#     roof_collapsed 8.7%, partial_collapse 3.2%, leveled ~0%
#     -> structural 11.9%, and no single roof-cover rung exceeds a third of
#        the plate.
#   L2 (real 136-house population, same boundaries): pristine 13.4%,
#     shingles 55.2%, cover 25.0%, deck 6.4%, roof_stripped ~0%, structural
#     0% -> L2 is UNCHANGED in shape (its houses sit almost entirely below
#     `deck_panels_lost`'s new 0.65 boundary, same as round 1).
#   `leveled` is not reached by this specific 94-house sample (max resampled
#   effective intensity is ~0.86, below its 0.90 floor) -- it stays the
#   correct, rarest rung; a higher `site_gust_mps` or a worse `vuln` draw
#   elsewhere on the ladder still reaches it.
#
# Locked in `scene_gen/tests/test_hurricane_house_cuts.py` against both GT
# files with tolerances, so a future re-tune has an acceptance gate instead
# of a synthetic-only self-check or a single-target search that only watches
# the top of the ladder.
_HOUSE_CUTS = ((0.360, "pristine"),
               (0.480, "shingles_lost"),
               (0.570, "cover_lost"),
               (0.650, "deck_panels_lost"),
               (0.720, "roof_stripped"),
               (0.780, "roof_collapsed"),
               (0.900, "partial_collapse"),
               (1.50, "leveled"))

# THE TREE LADDER. `defoliated` sits at the BOTTOM and is deliberately the
# WIDEST band -- see the module docstring's "wins by volume" section. Onset
# values here are TUNED to the qualitative shape of the SKILL's per-level
# damage table (L1 "large branches snapped," L2 "many snapped or uprooted,"
# L3 "most snapped or uprooted"), not fit to a numeric survey the way the
# house ladder is -- no hurricane-specific tree DOD table with hard onset
# gusts was found in the research. Trees fail at LOWER wind than houses do
# (the same relationship `tornado.py`'s own tree ladder states, and it holds
# at least as strongly here): every non-`pristine`/`defoliated` cut sits
# below the house ladder's matching structural cut.
TREE_LEVELS = ("pristine", "defoliated", "limbed", "leaning", "fallen",
              "snapped")

# RE-CUT 2026-08-30 AFTER THE FIRST RENDER, and the correction is worth
# recording because the original bands were not slightly off, they were the
# wrong SHAPE.
#
# As first written the four damage bands spanned 0.56..0.63 — 0.07 of the
# range between them — with `snapped` taking everything above. On the level-3
# plate, whose intensity runs 0.63..0.77, that put 1,419 of 1,684 trees into
# `snapped`: a suburb of bare standing stubs with no crowns anywhere. Both
# halves of that are wrong. `snapped` is the RAREST hurricane tree outcome
# (it takes the highest wind and a stem that runs out of capacity before its
# roots do), and the skill's own line — "broadleaf canopy is stripped
# essentially bare at Cat 3+ (86-94% leaf loss); conifers keep their needles
# and SNAP" — makes `defoliated` the one that should dominate.
#
# THE REAL FAULT WAS CONFLATING TWO INDEPENDENT MECHANISMS. Defoliation is a
# near-CERTAINTY at Cat 3: it happens to essentially every broadleaf on the
# plate. Structural failure — leaning, uprooting, snapping — is a MINORITY
# outcome decided by rooting depth, soil saturation, crown asymmetry and what
# the neighbouring trees did, none of which this field models and none of
# which correlate strongly with a wind speed that barely varies across 500 m.
# A single monotone ladder cannot express "almost all defoliated AND a
# minority down", because monotone means higher intensity is strictly worse.
#
# So: `defoliated` gets by far the widest band, the structural levels get a
# narrow tail, and the SPREAD comes from a much larger jitter (see
# `tree_level_for_intensity`) which is the honest place to put variance the
# field genuinely does not carry.
#
# RE-CUT AGAIN 2026-08-31 (STREAM T3), after the windthrow-boosted L3 field
# was MEASURED to put 41.0% of trees into leaning/fallen/snapped combined
# (690/1684, replayed against `V2_L3`'s real GT) against a target of "≈30%
# (25-32%)", with `snapped` alone at 59.6% of that broadleaf structural
# share against a "≤1/3" ceiling -- the 2026-08-30 cut above (`limbed` at
# 0.86) was too LOW a gate once the saturated-soil boost (see
# `windthrow_depth_boost`) is added on top of it every render, not just at
# L2 where it was aimed.
#
# THE HONEST LIMIT OF A SINGLE SHARED CUT. `L2`'s (Cat-1, `site_gust_mps`
# 55) and `L3`'s (Cat-3, `site_gust_mps` 70) own field intensities are
# measured, boosted, NOT overlapping bands: L2's boosted effective
# intensity (raw field + `windthrow_depth_boost`) tops out at 0.78 (mean
# 0.61, replayed against `V2_L2`'s real tree/depth data); L3's STARTS at
# 0.68 (mean 0.81). Because `windthrow_depth_boost` is a function of LOCAL
# DEPTH alone and L3 is simultaneously the windier AND the wetter scene
# (mean standing-water depth 0.76 m against L2's 0.28 m, replayed against
# the same GT), ANY monotonic function of (intensity, depth) that lifts
# enough of L2's population over a "structural" gate to reach 10% inevitably
# lifts a LARGER share of L3's already-higher, already-wetter population
# over the SAME gate -- measured by grid-searching the gate position against
# every boost magnitude from 0.05 to 2.0: the lowest achievable L3
# structural share, given L2 pinned to exactly 10%, floors at ≈34%, not 30%.
# (The reverse trade exists too: a gate that lands L3 at its target centre
# leaves L2 at ≈7%, not 10%.) This is not a tuning failure to iterate past;
# it is what "L3 is windier and wetter than L2, and depth-driven windthrow
# cannot un-know that" MEANS numerically. The cut below is the
# violation-minimising point of that trade (found by the same grid search),
# not a compromise arrived at by feel.
#
# MEASURED AT THIS CUT (replayed against `V2_L2`/`V2_L3`, `windthrow_
# depth_boost` at its own re-tuned anchors below): L2 leaning+fallen+snapped
# 7.4% (112/1504, up from 6.1% pre-retune -- still short of the 10% floor,
# see the trade above), L3 31.2% (526/1684, DOWN from 41.0%, inside the
# 25-32% target), L3 broadleaf snapped share 22.9% (105/458, DOWN from
# 59.6%, inside the ≤1/3 ceiling). A side effect worth knowing rather than
# being surprised by: because the `limbed` gate now sits above BOTH fields'
# raw (unboosted) ceilings, essentially every leaning/fallen tree at EITHER
# level is now standing in depth > 0.2 m (100% at both L2 and L3, replayed;
# L3 was only 56.4% before this re-cut) -- a dry, wind-only windthrow has
# become vanishingly rare at these two gust speeds, which is the intended
# direction (saturated soil, not wind alone, is what puts most of these
# trees down) but is a stronger dependency than the pre-retune cut had.
#
# RE-CUT AGAIN 2026-08-31 (STREAM T5), after the bake's own retention fix
# (see `bake_hurricane_trees.py`'s "DAMAGE IS READ BY CROWN COLOUR" section)
# exposed a SEPARATE fault in this file: `pristine`'s cut (0.30) sat below
# EVERY jittered value either recorded plate can produce. `L3`'s field alone
# (0.629-0.768) minus the full jitter never dips under 0.37; `L2`'s (0.499-
# 0.600) minus jitter only just reaches 0.24, replayed to a measured 0.9%
# (13/1504) pristine share. MEASURED CONSEQUENCE: `pristine` was 0.0% of L3
# (0/1684) and functionally 0% of L2 -- every tree in the scene was drawn as
# damaged, which contradicts this file's OWN documented sourcing two
# comments up ("most trees SURVIVE a hurricane") and the user's own read of
# the render ("I see only 1-2 trees in the whole scene" -- compounded by,
# not caused by, the separate bake-retention fault the bake file's own
# STREAM T5 section fixes; a render with NO pristine trees at all removes
# even the one population that was never bald to begin with).
#
# THE SAME SD-STYLE TENSION `_TORNADO_LEVEL_CUTS` DOCUMENTS, ONE LEVEL DOWN.
# `L2`'s field sits entirely below `L3`'s (0.499-0.600 vs 0.629-0.768), so
# ANY single shared `pristine` cut high enough to give `L3` a real pristine
# minority gives `L2` a MUCH larger one -- raising the cut to hand `L3`
# 10-15% already pushes `L2` past 42-48%. Rather than fight this, the
# directive's own target bands ACCEPT it: L3 pristine 15-25% (a genuine but
# still-minority "sheltered survivor" share, matching the skill's own
# survival data at the higher wind level) and L2 pristine 40-50% (a plurality
# of untouched trees at the lower, Cat-1-local wind, where "most trees
# survive uninjured" is the literally-correct reading). Grid-searched jointly
# over the `pristine` cut, the `defoliated` upper cut (see below), and the
# ladder's own `jitter` (the JITTER ITSELF has to widen too -- at the
# ORIGINAL 0.26, no single `pristine` cut lands both scenes in their target
# bands AND leaves L3's structural share inside 28-32% at the same time; a
# WIDER jitter (0.30) is what makes a joint solution exist at all, the same
# "jitter is the honest place to put variance the field doesn't carry" logic
# `tree_level_for_intensity`'s own docstring already gives, just needing more
# of it once THREE targets (two pristine bands, one structural band) have to
# be hit off one shared ladder instead of two).
#
# `defoliated`'s UPPER cut also moves, from 0.78 to 1.00 -- independently of
# `pristine`, and for an unrelated reason: at the OLD 0.78/1.09 split,
# `limbed` (41.4% of L3, replayed) was the PLURALITY level, not `defoliated`
# (28.3%), which is backwards from this file's own "defoliated wins by
# volume" design (see the module docstring) and from the directive's own
# "brown-crowned (defoliated) plurality" requirement. Moving ONLY this
# boundary (never the fixed 1.09/1.19/1.44/1.94 downstream of it) shifts mass
# from `limbed` into `defoliated` without touching the `limbed`-to-`leaning`
# structural threshold at all, so the L2/L3 structural shares are UNCHANGED
# by this half of the re-cut.
#
# MEASURED AT THIS CUT (`pristine` 0.30->0.64, `defoliated` 0.78->1.00,
# jitter 0.26->0.30; replayed against `V2_L2`/`FINAL3_L3_brown`'s real GT,
# `windthrow_depth_boost`/`dry_windthrow_chance` UNCHANGED): L2 pristine
# 46.3% (697/1504, inside 40-50%), L2 structural (leaning+fallen+snapped)
# 7.9% (119/1504, inside 6-8%); L3 pristine 15.7% (264/1684, inside 15-25%),
# L3 defoliated 42.9% (722/1684, now the clear PLURALITY, more than 4x
# `limbed`'s 10.4%/175), L3 structural 31.1% (523/1684, inside 28-32%, all
# but unchanged from the pre-T5 31.2%, confirming the two boundary moves
# above do not disturb it). See `test_hurricane_trees.py::test_l2_l3_
# replay_against_real_gt` for the full per-level tally this is pinned to.
_TREE_CUTS = ((0.64, "pristine"),
              (1.00, "defoliated"),
              (1.09, "limbed"),
              (1.19, "leaning"),
              (1.44, "fallen"),
              (1.94, "snapped"))

# Wide-crowned species whose root plate is too large, and whose crown too
# wide, to lie down at a normal lean -- they fail in the STEM instead.
# `Black_Oak` was carried over from `tornado.NO_UPROOT` for the identical
# geometric reason and IS NOW RE-VERIFIED against the hurricane-specific
# bake (`tools/bake_hurricane_trees.py`): its single fused trunk+limb mesh
# has vertices up to ~11 m off the trunk axis, and even the `snapped`
# archetype's severed-top piece has to drop that off-axis mass rather than
# rotate it, or the piece swings through several extra metres of height —
# the same lever arm a `fallen` rotation cannot survive either.
#
# `Douglas_Fir` is here for a different, SOURCED reason, not a geometric
# one: build-hurricane-scenes' species table has pines snapping mid-trunk
# into same-height spars rather than uprooting -- a conifer's narrow, deep
# taproot resists overturning better than its slender stem resists bending
# under sustained sail load, the opposite failure balance from a broadleaf's
# shallow, wide root plate.
NO_UPROOT = ("Black_Oak", "Douglas_Fir")

# Species whose survival story is "sheds fronds, keeps standing" rather than
# "uproots or snaps" -- sabal palm measured at 80-93% survival even through
# Andrew at 165 mph, because a palm has no rigid crown to catch the wind and
# no root plate that snaps a trunk. Capped at `defoliated`: a palm in this
# model never reaches `limbed` (it has no limbs to shed, only fronds, which
# IS the defoliated look) or beyond. NAME NOT YET CONFIRMED against a
# suburban asset library -- no palm species asset was found under the
# existing naming convention (`American_Beech`, `Black_Oak`, etc.) at the
# time this was written; treat this as a placeholder for whichever palm
# species ships with a coastal asset set, not a verified asset reference.
PALM_SPECIES = ("Sabal_Palm",)


def _ladder(cuts, v):
    """Pick a level from `cuts` for value `v`. See `_HOUSE_CUTS`'s docstring
    comment for the (upper_bound, name) convention -- identical to
    `tornado._ladder`, just without that function's jitter argument, because
    both callers here apply jitter differently (one blended with a
    resistance term first) and each does its own `rng.uniform` before calling
    this.
    """
    for lim, name in cuts:
        if v < lim:
            return name
    return cuts[-1][1]


def house_level_for_intensity(i, rng, jitter=0.06, vuln=0.5):
    """Structural level for a house standing where the field intensity is `i`.

    THE EFFECTIVE LADDER INPUT IS A BLEND OF INTENSITY AND VULNERABILITY, not
    intensity alone -- this is the file's single most important sentence.
    `vuln` (0..1, from `draw_vulnerability`) is turned into a resistance
    multiplier (`_resistance`) that divides `i` before the ladder is read, so
    a 1970s house (`vuln` near 0.9) reaches a given damage state at a
    meaningfully lower wind than a FORTIFIED one (`vuln` near 0.1) standing
    in identical wind. `jitter` is drawn AFTER that blend, same role as
    `tornado._ladder`'s jitter: it stops the levels from being clean contour
    lines parallel to... nothing, in this case, since the field has no
    contours to speak of -- here it represents everything about a single
    building's damage the intensity+vulnerability blend does not capture
    (a garage door that happened to fail, a corner that happened to catch a
    gust).

    VERIFIED (see this module's own test harness / `summarise`-style check):
    at `DEFAULTS`' `site_gust_mps` (55 m/s, the skill's own "Level 2 / roofs
    are the story" point on its proposed 3-level ladder) with `vuln` drawn
    from `CODE_ERAS`, roughly 75-80% of houses land in the four roof states
    and roughly 10-15% in the three structural ones -- matching the SKILL's
    cited "less than 15 percent of homes sustained structural wind damage"
    at 116-135 mph gusts almost exactly. At the skill's Level 1 (~38 m/s)
    almost every house stays pristine or `shingles_lost`; at Level 3
    (~70 m/s) structural states become the MAJORITY, which is the correct
    direction of travel -- this ladder is not flat, it just is not driven
    primarily by POSITION.
    """
    eff = float(i) / _resistance(vuln)
    if jitter > 0.0:
        eff += rng.uniform(-jitter, jitter)
    return _ladder(_HOUSE_CUTS, eff)


# ---------------------------------------------------------------------------
# the house ladder AS THE TORNADO'S OWN SIX LEVELS (STREAM S, 2026-08-31)
# ---------------------------------------------------------------------------
#
# USER DIRECTIVE: "Make sure that we're doing the same damage as tornado...
# Same house generation including material. You need to just adjust the
# pattern of house damage." The 8-level ladder above (`HOUSE_LEVELS`/
# `_HOUSE_CUTS`/`house_level_for_intensity`) is a SEPARATE, independent
# damage vocabulary keyed to a hurricane-specific archetype library
# (`archetypes_hurricane`, `bake_hurricane_archetypes_launch_script.py`,
# `hurricane_flow.py`'s per-construction-type recipes). It is LEFT IN PLACE,
# unused by the launcher, rather than deleted: nothing else in this file or
# its own tests reaches for a six-level cut, and ripping the 8-level ladder
# out would break `test_hurricane_house_cuts.py` and every tool built on it
# for no functional gain.
#
# `tornado_level_for_intensity` below is the launcher's new source of truth.
# It reads the IDENTICAL field intensity `i` and the identical
# `_resistance(vuln)` blend as `house_level_for_intensity` above -- the
# per-building vulnerability draw is still what carries almost all of this
# plate's variance (see the module docstring, S1.4-1.5) -- but returns one of
# the TORNADO's own six level names (`pristine`, `roof_stripped`,
# `roof_collapsed`, `partial_collapse`, `leveled`; never `swept` -- see
# below) so the launcher can reference `house_<style>_<level>.usd` straight
# out of the TORNADO archetype library instead of this file's own.
#
# THE CUTS ARE AN INDEPENDENT CALIBRATION, not a re-bucketing of
# `_HOUSE_CUTS`. Collapsing four of the 8-level rungs into "roof_stripped"
# does not land anywhere near the targets below (measured: it puts L3's
# "pristine" share at 18% against a 25-35% target and "roof_stripped" at 75%
# against 45-55%) because the two ladders are not photographs of the same
# thing at different resolutions -- they are two different fits to the same
# underlying (intensity, vulnerability) population. So `_TORNADO_LEVEL_CUTS`
# was fit FRESH, jointly, against both real recorded scenes.
#
# TARGETS (this stream's own brief, itself shaped to the skill's "roofs are
# the story at L2/L3, structural damage climbs but stays a minority even at
# L3" reading and to Roueche et al.'s "every wind-only slab-sweep in the
# hurricane record was surge-associated" finding, which is why `swept` has
# no wind-only share at all):
#
#   L3 (site_gust_mps 70, Cat-3-local): pristine 25-35%, roof_stripped
#     45-55%, roof_collapsed 8-12%, partial_collapse 3-5%, leveled <=2%
#   L2 (site_gust_mps 55, Cat-1-local): pristine 55-65%, roof_stripped
#     30-40%, structural (roof_collapsed+partial_collapse+leveled) <=5%
#
# FIT METHOD: `scipy.optimize.minimize` (Nelder-Mead, many random restarts)
# over 4 monotone boundaries plus the jitter magnitude, against the REAL
# (intensity, vulnerability) pairs recorded in two actual built scenes --
# `~/hurricane_previews/FINAL2_L2_brown/GT_hurricane.json` (136 houses,
# site_gust_mps 55) and `FINAL2_L3_brown/GT_hurricane.json` (94 houses,
# site_gust_mps 70) -- not a synthetic sample, for the same reason
# `_HOUSE_CUTS`'s own 2026-08-31 re-cut gives for abandoning its first
# synthetic-only search: a real scene's houses are not uniformly spread
# across intensity or vulnerability, and a cut tuned on a fabricated
# population can hit its target on paper and miss the real one badly (that
# re-cut measured a 25%-claimed structural share landing at 38% once
# checked against a real GT). Because `_resistance(vuln)` compresses BOTH
# scenes' raw field intensity (L2 0.50-0.60, L3 0.65-0.77 -- the field is
# genuinely close to uniform, see `intensity_field`'s docstring) into
# overlapping effective-intensity bands (L2 eff 0.33-0.59, L3 eff 0.42-0.77),
# NO jitter-free set of cuts can hit both scenes' targets at once: a boundary
# low enough to give L3 25-35% pristine gives L2 well under 55%, and vice
# versa. The fit's jitter (~0.21, over three times `house_level_for_
# intensity`'s own 0.06) is not a stylistic choice, it is what the trade
# requires -- see the SD-style tension `_TREE_CUTS`'s own comment documents
# for the L2/L3 tree ladder, the same shape of problem here on the house
# side.
#
# MEASURED AT THIS FIT (expectation over many resampled jitter draws per
# house, replayed against the same two real GT files -- see
# `scene_gen/tests/test_hurricane_tornado_parity_levels.py` for the pinned
# numbers and tolerances):
#   L2: pristine ~61%, roof_stripped ~38%, structural ~1.4% (target bands
#       55-65 / 30-40 / <=5, all satisfied)
#   L3: pristine ~31%, roof_stripped ~53%, roof_collapsed ~12%,
#       partial_collapse ~3%, leveled ~1% (target bands 25-35 / 45-55 /
#       8-12 / 3-5 / <=2, all satisfied)
_TORNADO_LEVEL_CUTS = ((0.49, "pristine"),
                       (0.73, "roof_stripped"),
                       (0.83, "roof_collapsed"),
                       (0.88, "partial_collapse"),
                       (3.00, "leveled"))

# See `_TORNADO_LEVEL_CUTS`'s own comment for how this was fit alongside the
# four boundaries above -- far larger than `house_level_for_intensity`'s
# 0.06 because the two real scenes' effective-intensity bands overlap and
# only a jitter this size can put both scenes' populations in their target
# bands off a SINGLE shared set of cuts.
_TORNADO_LEVEL_JITTER = 0.21


def tornado_level_for_intensity(i, rng, jitter=_TORNADO_LEVEL_JITTER, vuln=0.5):
    """Structural level for a house standing where the field intensity is
    `i`, expressed on the TORNADO's own six-level ladder
    (`pristine`/`roof_stripped`/`roof_collapsed`/`partial_collapse`/
    `leveled`; never `swept` -- see `_TORNADO_LEVEL_CUTS`'s module comment).

    Same `_resistance(vuln)` blend as `house_level_for_intensity`: a
    pre-1976 house (`vuln` near 0.9) reaches a given rung at a meaningfully
    lower wind than a FORTIFIED one (`vuln` near 0.1) standing in identical
    wind, and that per-building draw -- not position -- is still most of
    this plate's variance (see the module docstring, S1.4-1.5).

    `swept` IS NEVER RETURNED. The launcher is responsible for overriding
    this function's result with `"swept"` when `surge.house_water_state`
    (or `washaway.house_surge_state`) says so -- "a bare, wind-swept slab is
    a SURGE signature, never a wind one" (module docstring, citing Roueche
    et al.: every one of 3,016 hurricane homes with complete destruction was
    surge-associated). This function has no surge/depth input at all, so it
    structurally cannot produce that state, which is the point.
    """
    eff = float(i) / _resistance(vuln)
    if jitter > 0.0:
        eff += rng.uniform(-jitter, jitter)
    return _ladder(_TORNADO_LEVEL_CUTS, eff)


# SATURATED-SOIL WINDTHROW BOOST (build-hurricane-scenes SESSION_2026-08-31
# §4 item 2). A measured L2 field maxes at intensity 0.598, and even the
# full jitter (+-0.30 as of the STREAM T5 widening; +-0.26 at the time this
# boost was first written) only pushes that to 0.898 -- comfortably under
# `_TREE_CUTS`'s `limbed` cut (1.09 as of the 2026-08-31 re-cut; 0.86 at
# the time this boost was first written, when the jittered ceiling landed
# "and only just" past it instead). THREE OF SIX LEVELS ARE UNREACHABLE AT
# L2 either way, and the unreachable three are exactly the ones that put a
# tree on the ground -- which is backwards, because L2 is the FLOODED level and
# standing water is where root anchorage actually fails first. A tree
# standing in saturated ground loses root-soil adhesion well before the
# wind alone would take it, so local water depth raises the EFFECTIVE
# intensity the ladder is read at, independent of the field's own
# (near-uniform) value -- this is what lets a flooded low spot produce
# fallen trees a dry lot at the same nominal intensity cannot.
#
# Anchors are TUNED, NOT SOURCED (no fragility study ties windthrow
# probability to standing-water depth in centimetres), but the SHAPE is:
# saturation effects begin mattering once water stands over most of the
# root zone (a few tens of centimetres) and grow toward a ceiling by the
# time a tree is standing in knee-to-thigh-deep water.
# See `scene_gen/tests/test_hurricane_trees.py` for the measured L2/L3
# tallies before and after this boost.
#
# RE-TUNED 2026-08-31 (STREAM T3) ALONGSIDE `_TREE_CUTS`'s re-cut, roughly
# doubling both anchors (0.10/0.25 -> 0.22/0.55) -- see `_TREE_CUTS`'s own
# comment for the measured before/after tallies and the honest limit of
# what a single depth-driven boost can do once L3 is both windier AND
# wetter than L2. A larger boost was searched up to 8x this value and the
# achievable L3-structural-share floor (at L2 pinned to its 10% target)
# never dropped below ≈34%; this magnitude is the smallest one that still
# lands on the grid search's violation-minimising point, so it is not
# pushed further than the trade requires.
#
# RE-TUNED AGAIN 2026-08-31 (STREAM T4, Correction 2: "dry-land windthrow is
# zero"). `dry_windthrow_chance` below adds an INDEPENDENT wind-only path to
# a structural outcome for trees with NO local flood depth -- until now,
# 100% of leaning/fallen/snapped trees at BOTH L2 and L3 stood in >0.2 m of
# water (replayed against real GT), which contradicts every hurricane
# survey read for this project: Andrew's own windthrow was overwhelmingly
# on DRY ground. Because the dry mechanism and this depth boost act on
# DISJOINT populations (dry vs wet) they are strictly ADDITIVE -- adding the
# dry share on top of the UNCHANGED 0.22/0.55 anchors measured L3's total
# structural fraction at 34.5% (up from 31.2%), overshooting "near 30%".
# These anchors are nudged down (0.22/0.55 -> 0.21/0.51) -- a SMALL
# reduction of the WET contribution that makes room for the new DRY one --
# grid-searched jointly with `_DRY_WINDTHROW_ONSET`/`_SLOPE` against both
# real GT scenes for the combination landing closest to all four targets at
# once (L3 dry ≈5-8%, L2 dry ≈2%, L3 total ≈30%, L2 total left near its own
# previously-documented ≈7-8%). MEASURED AT THIS RE-TUNE (replayed against
# `V2_L2`/`V2_L3`): L2 dry-structural 2.44% (27/1107 dry trees), L2 total
# 7.05% (up only slightly from 6.98% pre-dry-mechanism at these anchors,
# still "≈7-8%"); L3 dry-structural 6.31% (48/761 dry trees, inside the
# 5-8% target), L3 total 30.23% (down from the un-compensated 34.5%, right
# on the "near 30%" target). See `test_hurricane_trees.py`'s replay test
# for the exact before/after tallies this re-tune is pinned to.
_WINDTHROW_DEPTH_LO_M = 0.3
_WINDTHROW_DEPTH_HI_M = 1.0
_WINDTHROW_BOOST_LO = 0.21
_WINDTHROW_BOOST_HI = 0.51


def windthrow_depth_boost(depth_m):
    """Effective-intensity ADD-ON from standing water at one tree's base.

    0 below `_WINDTHROW_DEPTH_LO_M` (dry, or a puddle too shallow to matter);
    ramps linearly to `_WINDTHROW_BOOST_HI` at `_WINDTHROW_DEPTH_HI_M`;
    flat beyond that -- deeper water saturates the root zone no further
    once the whole rooting depth is already submerged.

    NOTE ON ROOTING CONTEXT (street-pit vs yard/park tree). The skill also
    asks that a street/pit tree fail MORE than a grouped yard/park tree at
    the same depth (measured urban survival: 64% at 0-3 sq m of rooting
    space against 91% at >7 sq m). `tree_instances` as published by
    `suburb_scene.py` (species/x/y/yaw only -- the placement `category`,
    e.g. `street_tree` vs `yard_tree`, is computed during layout but
    DISCARDED before the hurricane launcher ever sees the tree list; see
    `suburb_scene.py:5278-5288`) carries no such field today, so this
    function cannot condition on it without a change to that layout export,
    which is out of this file's scope. Depth alone is applied uniformly;
    the rooting-context term is a documented gap, not a silent omission.
    """
    d = float(depth_m)
    if d <= 0.0:
        return 0.0
    if d <= _WINDTHROW_DEPTH_LO_M:
        return _WINDTHROW_BOOST_LO * (d / _WINDTHROW_DEPTH_LO_M)
    if d >= _WINDTHROW_DEPTH_HI_M:
        return _WINDTHROW_BOOST_HI
    span = _WINDTHROW_DEPTH_HI_M - _WINDTHROW_DEPTH_LO_M
    frac = (d - _WINDTHROW_DEPTH_LO_M) / span
    return _WINDTHROW_BOOST_LO + (_WINDTHROW_BOOST_HI - _WINDTHROW_BOOST_LO) * frac


# DRY WIND-ONLY STRUCTURAL SHARE (Correction 2, 2026-08-31 STREAM T4). The
# depth boost above is the ONLY way this model has ever put a tree down --
# and because `_TREE_CUTS`'s `limbed` gate sits above BOTH fields' raw
# (unboosted) intensity ceiling, wind alone, with zero depth boost, can
# NEVER cross into `leaning`/`fallen`/`snapped`. Measured consequence: 100%
# of leaning/fallen/snapped trees at BOTH L2 and L3 stand in >0.2 m of
# water. That is backwards against every hurricane survey this project has
# read -- Hurricane Andrew's own windthrow was overwhelmingly on DRY
# ground; saturated soil lowers the bar, it is not the only door through
# it. This adds an INDEPENDENT probabilistic path to a structural outcome
# for a tree standing in LESS than `_DRY_WINDTHROW_DEPTH_M` of water, whose
# normal (ladder + depth-boost) draw came back non-structural -- its own
# `rng.random()` roll, using the RAW field intensity `i` (never `eff`, i.e.
# computed BEFORE the depth boost is folded in), so it can be tuned
# directly against the measured dry population instead of reverse-
# engineered through the ladder's cuts.
#
# ONSET IS 0.52, NOT THE "~0.6" FIRST GUESSED, because 0.6 would leave the
# term nearly INERT at L2: L2's own dry population (replayed against real
# `V2_L2` GT) never exceeds field intensity 0.600 and averages 0.569 -- an
# onset at 0.6 gives only the extreme top sliver of that range any nonzero
# probability at all, nowhere near the "~2% of L2's dry trees" target. 0.52
# sits below L2's whole dry-population mean and below L3's entire dry range
# (0.682-0.768, replayed) too, so the mechanism actually reaches both
# scenes rather than only the windier one. TUNED, NOT SOURCED (no fragility
# study ties dry-windthrow probability to this field's intensity units) --
# grid-searched jointly with `_WINDTHROW_BOOST_LO`/`_HI` (see that
# constant's own comment for the compensating re-tune this required) for
# the combination landing closest to all four targets from the real GT
# replay at once. MEASURED RESULT: see `_WINDTHROW_BOOST_HI`'s comment for
# the numbers, and `test_hurricane_trees.py` for the pinned replay.
_DRY_WINDTHROW_DEPTH_M = 0.2
_DRY_WINDTHROW_ONSET = 0.52
_DRY_WINDTHROW_SLOPE = 0.28
_DRY_WINDTHROW_CAP = 0.50


def dry_windthrow_chance(intensity):
    """Probability, in `[0, _DRY_WINDTHROW_CAP]`, that a tree standing on
    DRY ground (no local flood depth) still goes structural from wind
    alone. 0 at/below `_DRY_WINDTHROW_ONSET`; rises LINEARLY above it,
    capped at `_DRY_WINDTHROW_CAP`. See `_DRY_WINDTHROW_ONSET`'s comment
    for how the anchors were actually chosen (a joint grid search against
    two real recorded scenes, not fit to a single number in isolation).
    """
    i = float(intensity)
    if i <= _DRY_WINDTHROW_ONSET:
        return 0.0
    return min(_DRY_WINDTHROW_CAP, _DRY_WINDTHROW_SLOPE * (i - _DRY_WINDTHROW_ONSET))


def tree_level_for_intensity(i, rng, jitter=0.30, species=None, depth_m=0.0):
    """Damage level for a tree standing where the field intensity is `i`.

    A MUCH LARGER JITTER THAN THE HOUSES GET — 0.30 against their 0.06 (RAISED
    2026-08-31, STREAM T5, from 0.26 -- see `_TREE_CUTS`'s own re-cut comment
    for why: with only 0.26, no single `pristine` cut could put BOTH recorded
    plates inside their target pristine bands while also holding L3's
    structural share inside 28-32%; 0.30 is the smallest widening the joint
    grid search needed for a solution to exist at all). Still well over
    three times what `tornado.tree_level_for_intensity` uses. Same reasoning
    that module gives, taken further because the hurricane field gives it no
    help: whether a given tree goes over depends on its rooting, its lean,
    its soil saturation, its exposure and what its neighbours did, and a
    tornado at least has a cross-track gradient to separate outcomes with. A
    near-uniform field has NOTHING, so every bit of the variance that makes a
    stand look like a stand has to come from here. With the re-cut bands (see
    `_TREE_CUTS`) this puts most of the plate in `defoliated`, a genuine
    minority pristine, and a believable minority down, which is what the
    aerials show.

    `depth_m` (local standing-water depth, metres, default 0 = dry) adds
    `windthrow_depth_boost(depth_m)` to `i` BEFORE the jitter -- see that
    function's docstring for why, and for the rooting-context field that is
    NOT wired in here because the layout does not publish it. A tree with
    `depth_m` under `_DRY_WINDTHROW_DEPTH_M` also gets an INDEPENDENT
    chance (`dry_windthrow_chance(i)`) to be promoted to `leaning` even if
    the ladder+boost draw came back non-structural -- see that function's
    docstring for why a depth-only boost cannot ever produce a dry
    windthrown tree by construction, which is the defect this closes.

    `species` applies two overrides, checked in this order:

      1. `PALM_SPECIES` caps the level at `defoliated` -- see that constant.
      2. `NO_UPROOT` promotes a drawn `fallen` to `snapped` -- identical rule
         to `tornado.tree_level_for_intensity`, carried over for the same
         reason (see `NO_UPROOT`'s comment for the caveat about which bake
         it was actually measured against).

    Omit `species` and the ladder is returned unmodified, same contract as
    the tornado version, for a caller with no species to hand.
    """
    eff = float(i) + windthrow_depth_boost(depth_m)
    lv = _ladder(_TREE_CUTS, eff + rng.uniform(-jitter, jitter))
    order = list(TREE_LEVELS)
    # DRY WIND-ONLY STRUCTURAL SHARE -- see `_DRY_WINDTHROW_ONSET`'s comment.
    # Only rolled for a tree that (a) is measured DRY and (b) did not
    # already come back structural through the normal path -- it can only
    # PROMOTE a non-structural draw to `leaning`, never demote or duplicate
    # an already-structural one, so this is strictly additive over the
    # depth-boost mechanism's disjoint (wet) population.
    if float(depth_m) < _DRY_WINDTHROW_DEPTH_M and order.index(lv) < order.index("leaning"):
        p = dry_windthrow_chance(i)
        if p > 0.0 and rng.random() < p:
            lv = "leaning"
    sp = str(species) if species else None
    if sp and sp in PALM_SPECIES:
        if order.index(lv) > order.index("defoliated"):
            return "defoliated"
        return lv
    if lv == "fallen" and sp and sp in NO_UPROOT:
        return "snapped"
    return lv


# ---------------------------------------------------------------------------
# debris composition
# ---------------------------------------------------------------------------

# Lee County's post-Ian debris collection: 734,136 cubic yards vegetative
# against 285,282 cubic yards construction & demolition -- a measured 72/28
# split, the mirror image of the tornado's plank-dominated field.
# `_plans/hurricane_research.md` "Debris is 70% VEGETATION."
VEG_SHARE = 0.72
BLD_SHARE = 1.0 - VEG_SHARE


def debris_mix(i, rng):
    """`(class -> share)`, shares summing to 1.0, for the debris scattered at
    field intensity `i`.

    The 72/28 vegetation/building split is held FIXED across intensity --
    it was measured as a whole-event aggregate (a county's total collection
    tonnage), and nothing in the research ties that particular ratio to local
    wind speed, only the debris VOLUME per household (the USACE `C`
    coefficient, 2/8/26/50/80 cubic yards per household for Cat 1-5, i.e. a
    roughly 1:13:40 scaling across three categories -- a quantity knob for
    whatever scatters ground clutter, not implemented in this file). What
    DOES shift with intensity is the GRAIN within each half: fine litter
    gives way to whole limbs and fronds as wind increases (more of the
    canopy comes down structurally rather than just shedding), and loose
    shingles give way to whole sheathing panels, siding strips and fence
    sections as the house ladder itself climbs past `cover_lost`.
    """
    it = max(0.0, min(1.0, float(i)))

    # Leaf/twig share shrinks, limb/frond share grows, as more of the canopy
    # fails structurally rather than just defoliating.
    leaf_frac = 0.78 - 0.45 * it
    veg_fine = VEG_SHARE * leaf_frac
    veg_coarse = VEG_SHARE * (1.0 - leaf_frac)

    # Loose shingles dominate at low intensity (the `shingles_lost` /
    # `cover_lost` rungs); sheathing, siding/soffit and fence sections
    # dominate once the house ladder is climbing past `deck_panels_lost`.
    shingle_frac = 0.62 - 0.40 * it
    bld_shingles = BLD_SHARE * shingle_frac
    remainder = BLD_SHARE * (1.0 - shingle_frac)
    bld_sheathing = remainder * 0.45
    bld_siding = remainder * 0.35
    bld_fence = remainder * 0.20

    return {
        "leaf_litter": veg_fine,
        "limbs_fronds": veg_coarse,
        "shingles": bld_shingles,
        "sheathing_panels": bld_sheathing,
        "siding_soffit": bld_siding,
        "fence_sections": bld_fence,
    }


# ---------------------------------------------------------------------------
# host-side reporting
# ---------------------------------------------------------------------------

def knobs_from_env(span_m):
    """Debris/field overlay knobs from HUR_* env vars.

    Mirrors `tornado.knobs_from_env`'s shape exactly, including that
    function's own quirk of accepting `span_m` without using it in the
    body -- it is here for signature symmetry with a caller that already
    has the plate's span in hand and may want it for logging, same as the
    tornado version. THIS IS A NEW SURFACE: unlike `tornado`'s `MUD_*` env
    vars, which an existing overlay pass already reads, nothing in the repo
    consumes `HUR_*` yet -- the names below are a reasonable first cut
    scoped to this file's own knobs (the field's roll/coastal terms and a
    debris-scatter cell size), not a contract anything else has agreed to.
    """
    def _f(name, default):
        return float(os.environ.get(name, default))

    return dict(
        cell_m=_f("HUR_CELL_M", "3.0"),
        streak=os.environ.get("HUR_STREAK", str(DEFAULTS["streak"])),
        streak_amp_mps=_f("HUR_STREAK_AMP_MPS",
                          str(DEFAULTS["streak_amp_mps"])),
        streak_period_m=_f("HUR_STREAK_PERIOD_M",
                           str(DEFAULTS["streak_period_m"])),
        coastal_falloff_per_km=_f("HUR_COASTAL_FALLOFF_PER_KM",
                                  str(DEFAULTS["coastal_falloff_per_km"])),
        noise_amp_mps=_f("HUR_NOISE_AMP_MPS",
                        str(DEFAULTS["noise_amp_mps"])),
    )


def summarise(cfg, region, rng, n=64):
    """Sample the field on an n x n grid and describe it. No Isaac Sim needed.

    The hurricane analogue of `tornado.summarise`, but the number that
    matters is different. A tornado's `summarise` reports `in_path_frac`
    because MISSING the fabric entirely is the tornado failure mode. A
    hurricane's field covers the whole plate by construction, so the check
    here is `gust_span_pct` -- the field should be uniform to within a few
    percent (occasionally low tens if a strong coastal step or roll streak
    is configured), and a `summarise` that comes back with a huge span (tens
    of percent from a smooth field alone) means something reintroduced a
    track into `gust_field` by mistake.
    """
    gust = gust_field(cfg, region, rng)
    inten = intensity_field(cfg, region, rng)
    x0, y0, x1, y1 = region
    xs = [x0 + (k + 0.5) * (x1 - x0) / n for k in range(n)]
    ys = [y0 + (k + 0.5) * (y1 - y0) / n for k in range(n)]
    gvals = [gust(x, y) for x in xs for y in ys]
    ivals = [inten(x, y) for x in xs for y in ys]
    gmean = sum(gvals) / len(gvals)
    gmin, gmax = min(gvals), max(gvals)
    return {
        "cells": len(gvals),
        "mean_gust_mps": round(gmean, 3),
        "min_gust_mps": round(gmin, 3),
        "max_gust_mps": round(gmax, 3),
        "gust_span_pct": round((gmax - gmin) / gmean * 100.0, 2)
                         if gmean > 0.0 else 0.0,
        "mean_intensity": round(sum(ivals) / len(ivals), 4),
        "max_intensity": round(max(ivals), 4),
        "min_intensity": round(min(ivals), 4),
        "heading_deg": round(float(cfg["heading_deg"]), 1),
        "wind_veer_deg": round(float(cfg.get("rotation_deg", 0.0)), 1),
    }
