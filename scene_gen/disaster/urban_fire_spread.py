"""urban_fire_spread — how a fire gets from ONE building to the NEXT.

WHY THIS IS A SEPARATE MODEL FROM THE WILDFIRE'S
------------------------------------------------
`disaster.fire` solves an elliptical front over a continuous fuel bed: every
point on the plate has a fuel state, the front sweeps it, and "when does this
tree ignite" is a closed form in the plate's coordinates. A city block is not
a fuel bed. It is a set of DISCRETE, non-combustible containers separated by
gaps, and fire crosses those gaps by three mechanisms that have nothing to do
with each other and different reaches by an order of magnitude:

  ATTACHED / PARTY WALL   gap < ~1 m. Terraces, back-to-back blocks, anything
                          sharing a wall or a roof void. Fire passes by
                          conduction through the wall, through service
                          penetrations, and over the top through a common
                          roof space. FAST — minutes — and near-certain. This
                          is the mechanism behind every historical
                          conflagration in a terraced street.
  RADIATION               gap ~1-12 m. A fully-involved façade is a radiant
                          panel; the neighbour's window glass fails and its
                          contents ignite when the received flux passes
                          ~12.5 kW/m2 (the piloted-ignition figure used in
                          separation-distance codes; ~6 m is where codes start
                          restricting unprotected openings). Flux falls off
                          roughly as 1/d^2, so the delay grows steeply with
                          the gap and the reach is short.
  BRANDS / SPOTTING       12-60 m, DOWNWIND ONLY. Burning debris lifted off
                          the fire building lands on a roof downwind. Long,
                          erratic, and the only mechanism that crosses a
                          street. This is what makes a conflagration jump a
                          road instead of stopping at it.

So the solve is a SHORTEST-PATH over a building graph, not a field. It is
also why the three mechanisms are kept separate rather than collapsed into
one distance falloff: they set the ENTRY POINT differently, and the entry
point is most of what makes a spreading fire read as one.

THE ENTRY POINT IS THE POINT
-----------------------------
A building that caught from its neighbour is alight ON THE SIDE FACING THAT
NEIGHBOUR, at about the storey the neighbour's fire was on — because that is
where the radiant panel was. A building that caught from a brand is alight at
its TOP, because that is where the brand landed. Scatter the origin storey and
the burning elevation randomly and you get a street of unrelated fires that
happen to be near each other; derive them from the source and the same street
reads as one event moving through it. `plan.entry_side` and `plan.origin` are
what `urban_fire.burn_building` is handed.

THE CLOCK
---------
Per building, measured from its own ignition:

    0                 ignition, one compartment
    ~8 min            flashover: that compartment is fully alight
    ~20 min           fully involved: several storeys, the façade is a
                      radiant panel and it can light its neighbours
    ~75 min           burning down; the fuel load is going
    ~140 min          burnt out; smoke only
    ~200 min          cold, and a weakened URM shell may have dropped its
                      top floor

A building only RADIATES once it is fully involved, which is why
`T_FULL` appears in the edge relaxation rather than the ignition time — a
neighbour does not catch from a fire that is still in one room. That single
term is what spaces the ignitions out into a visible wave instead of lighting
the whole block at once.

READING THE WAVE — WHICH ELAPSED TIME PRODUCES WHICH GRADIENT
------------------------------------------------------------
The whole point of solving ignition TIMES rather than picking levels is that
one `elapsed_s` then paints the whole plate at once and the result reads as
ONE event moving through a city:

    the origin            F5 / F5c / F6   burnt out, shell gutted or dropped
    the ring behind it    F4              burning down, roof gone
    the front             F2 / F3         fully involved, flames out of it
    the just-caught edge  F1              one room, smoke out of two openings
    everything beyond     F0              untouched

That gradient is NOT tuned — it falls out of `level_for_age` reading each
building's own age. But it only appears if `elapsed_s` is deep enough into
the burn, and the arithmetic is worth writing down because it is easy to ask
for an epoch that cannot show it:

  * a building leaves F4 only at `T_OUT` (140 min) and only reaches F5/F6 at
    `T_COLD` (200 min). So `elapsed_s = duration_s * start_offset_frac` must
    be >= 8400 s before ANY building can be F5c, and >= 12000 s before the
    origin can be F5 or F6. At 0.55 of a 3-hour duration (99 min) the origin
    is still F4 and the top of the ladder is simply not reachable — the wave
    is there, but it tops out one rung low.
  * along a CHAIN (a terrace, or any 1-D run) consecutive ignitions are at
    least `T_FULL + D_ATTACHED[0]` = 24 min apart, because a building has to
    be fully involved before it lights the next one. The F1 window is 8 min
    wide and the F2 window 12 min, so a single line of buildings shows at
    most one F1 and one F2 at any instant, and often neither. A 2-D block
    does show every band at once, because one fully-involved building lights
    several neighbours within the same few minutes.
  * `level_for_age` is stochastic past `T_OUT` (the F5c / F6 draws), so the
    LEVEL gradient is only strictly monotone when it is called with
    `rng=None`. The AGE gradient is always monotone. Group F5/F5c/F6 into
    one "burnt out" rank (`RANK`) before asserting monotonicity on a solve
    that was given an rng.

Nothing here imports `pxr` or touches a stage: the whole plan is solved and
asserted host-side, the same contract `disaster.people` keeps. `urban_fire`
is imported LAZILY (for `_side_neighbors` only) so this module stays free of
even a host-side dependency until something actually asks for an entry point.
"""

import math

# --- the one optional dependency ------------------------------------------
# `urban_fire` is host-side importable (no `pxr` at module scope), but it
# drags in `soot_plume`, `fire_collapse` and numpy, so it is fetched ONLY
# when something asks for a side ring or a levels check. `_UF` is a
# three-state cache: None = not tried, False = tried and unavailable.
_UF = None


def _urban_fire():
    """`disaster.urban_fire` if it imports host-side, else None."""
    global _UF
    if _UF is None:
        _UF = False
        try:                                  # inside the `disaster` package
            from . import urban_fire as m
            _UF = m
        except Exception:
            try:                              # ...or on a bare sys.path
                import urban_fire as m
                _UF = m
            except Exception:
                _UF = False
    return _UF or None


# The four elevations of a rectangular footprint, in ring order, so that
# consecutive entries share a CORNER. `urban_fire._SIDE_RING` is the same
# tuple and the same convention (`quake_flow._side_of`: the building's front
# is -Y in its own frame and is called "S"); it is replicated rather than
# imported so this module keeps working with `urban_fire` absent.
_SIDE_RING = ("S", "E", "N", "W")


def side_neighbors(side):
    """The two elevations that share a corner with `side`.

    Delegates to `urban_fire._side_neighbors` when that module is available,
    so the two can never disagree about the ring order, and falls back to the
    local `_SIDE_RING` when it is not.
    """
    uf = _urban_fire()
    fn = getattr(uf, "_side_neighbors", None) if uf is not None else None
    if fn is not None:
        return tuple(fn(side))
    i = _SIDE_RING.index(side)
    return (_SIDE_RING[i - 1], _SIDE_RING[(i + 1) % len(_SIDE_RING)])


# --- the clock, in seconds -------------------------------------------------
T_FLASHOVER = 8 * 60
T_FULL = 20 * 60          # fully involved: only now can it light a neighbour
T_DECAY = 75 * 60
T_OUT = 140 * 60
T_COLD = 200 * 60

# --- spread delays, in seconds --------------------------------------------
# Attached: the party wall buys some time and not much. Historical terrace
# fires move at roughly a house every few minutes once established.
D_ATTACHED = (4 * 60, 11 * 60)
ATTACHED_GAP_M = 1.2
# Radiation: the delay at 2 m and the delay at the reach limit. Between them
# it scales with d^2, because the flux does.
D_RAD_NEAR = 7 * 60
D_RAD_FAR = 52 * 60
RAD_REACH_M = 13.0
# Brands: long, and only downwind.
D_SPOT = (28 * 60, 95 * 60)
SPOT_MIN_M = 8.0
SPOT_REACH_M = 55.0
SPOT_P = 0.55             # share of downwind pairs a brand actually takes

# Wind. A downwind neighbour catches much sooner: the plume leans onto it, the
# flame tilts, and the radiant view factor goes up. Upwind is not immune —
# radiation does not care about wind — but it is slower.
WIND_DOWN = 0.55          # delay multiplier straight downwind
WIND_UP = 1.7             # ...and straight upwind
# A TALLER FIRE BUILDING RADIATES MORE. The panel is bigger, so the flux at a
# given distance is higher and the neighbour lights sooner. Referenced to a
# 20 m block.
H_REF_M = 20.0

# THE LADDER, AND IT IS `urban_fire.LEVELS`. This tuple is a COPY rather than
# an import so the module stays dependency-free at import time, and
# `check_levels_sync()` (and `tests/test_urban_fire_spread.py`) assert the two
# never drift. F5c and F6 were missing here while `urban_fire` already had
# them, which meant a city solve could never ask for the two states the user
# specifically asked for ("I want some partial collapse buildings for fire in
# all sets", 2026-08-30) — a spread plan can only produce a level it names.
LEVELS = ("F0", "F1", "F2", "F3", "F4", "F5", "F5c", "F6")

# How BAD each level is, for ordering only. F5c ranks WITH F5, not above it:
# it is the same fire with a different structural outcome (`urban_fire`'s own
# comment), so a gradient that runs ... F4, F5c, F5 ... has not gone
# backwards. F6 is the one rung above — the cold, fully burnt-out shell.
RANK = {"F0": 0, "F1": 1, "F2": 2, "F3": 3, "F4": 4, "F5": 5, "F5c": 5,
        "F6": 6}

# Past `T_OUT` the fire is out and the building's fate is drawn once.
COLLAPSE_P = 0.35         # ...part of the shell comes down          -> F5c
BURNT_OUT_P = 0.30        # ...a cold URM shell is a gutted ruin     -> F6
# Origin draw: the same low-biased exponent `urban_fire.plan_fire` uses for
# the origin storey, reused here for "which building starts it".
ORIGIN_BIAS = 1.7

# ORIGIN_FRAC_CAP: no ignition mechanism may push a building's fire-floor
# fraction above this, however it lit. `plan_fire`'s F4/F5/F5c/F6 band always
# runs from the origin to the TOP of the mass (`BAND[level][1] >= 99`), and
# F3's band is windowed by `n - origin` too — so an origin near the top of a
# 20-30 storey GAC tower leaves the entire lower building untouched and
# confines every flame/soot/smoke event to a handful of storeys under the
# roof. That is exactly the "fire only on the higher floors" signature a
# user flagged against the 39-record city manifest (2026-08-31): every
# top-heavy record traced back to `how == "spot"` (a brand landing on the
# roof, `frac = 0.88`) — the one mechanism deliberately biased toward the
# top so it "reads instantly as a different mechanism from the wall-to-wall
# spread happening down the street" (see `solve()`). The cap keeps that cue
# — a spot fire still starts higher than every other mechanism, since
# `origin`/`attached`/`radiation` (0.15/0.22/0.45) all sit under it — while
# guaranteeing at least half of every mass stays below the origin, the same
# threshold this fix's own audit used to flag a record as top-heavy.
ORIGIN_FRAC_CAP = 0.5


def level_for_age(age_s, btype="urm", rng=None, burnt_out_p=BURNT_OUT_P,
                  collapse_p=COLLAPSE_P):
    """Seconds since THIS building ignited -> an `urban_fire` level.

    The same shape as `damage.level_for_age` in the wildfire path and for the
    same reason: one number has to drive the structural level, the visual
    state and the fire state TOGETHER, or a scene reads as three unrelated
    effects that happen to be in the same place. Here that number is the age
    of this building's own fire, not the age of a front passing over it.

    PAST `T_OUT` THE FIRE IS OUT AND THE STRUCTURE DECIDES, NOT THE CLOCK.
    Three outcomes, and the construction type picks which are on the table:

      F5c  part of the shell has come down and the rest stands. Drawn at
           `collapse_p` for `urm` and `rc` from `T_OUT` on. THIS IS THE ONLY
           WAY F5c ENTERS A CITY SOLVE — `fire_collapse` registers the level
           in `soot_plume.DURATION_S` at import, and nothing else asks for it.
      F6   cold and completely gutted. `urm` only, and only past `T_COLD`,
           because it is a shell that has stood burnt-out long enough to lose
           its floors.
      F5   the default end state for a cold URM shell that neither dropped a
           wall nor went to F6: gutted, top storeys fallen in, four walls up.

    `rc_glass` NEVER RETURNS F5 OR F5c. A curtain-wall frame is a concrete
    cage with non-structural cladding: the glass goes, the spandrels spall,
    and the cage stands. Giving it a partial collapse would be the one
    construction type in the set where the mechanism is wrong.

    With `rng=None` no draw is made, so the mapping is deterministic and
    monotone in age — which is what the gradient assertions in
    `tests/test_urban_fire_spread.py` and `check()` rely on.
    """
    a = float(age_s)
    if a < 0:
        return "F0"
    if a < T_FLASHOVER:
        return "F1"                 # one room, smoke out of a few openings
    if a < T_FULL:
        return "F2"                 # that compartment fully alight
    if a < T_DECAY:
        return "F3"                 # fully involved, climbing
    if a < T_OUT:
        return "F4"                 # burning down / burnt out
    if btype == "rc_glass":
        return "F4"                 # the cage stands: never F5, never F5c
    if btype in ("urm", "rc") and rng is not None \
            and rng.random() < collapse_p:
        return "F5c"
    if a >= T_COLD and btype == "urm":
        if rng is not None and rng.random() < burnt_out_p:
            return "F6"
        return "F5"
    return "F4"


# ---------------------------------------------------------------------------
# HEIGHT CLASS: WHO IS ALLOWED TO COLLAPSE, AND WHOSE ROOF IS ALLOWED TO OPEN
# ---------------------------------------------------------------------------
# TWO SEPARATE POLICIES, LAYERED. `level_for_age` (and the age clock above
# it) decide WHEN a building would naturally reach a given level; neither
# knows or cares how tall the building is or what district it stands in.
# Two independent, real-world constraints sit on top of that, and both are
# expressed as a LEVEL DEGRADATION -- never as "drop the building" and never
# as "reset to F0" -- because a building that is structurally too tall (or
# too rarely timber-framed) to have collapsed the way the raw age/rng draw
# says is still ON FIRE; it just stopped short of the collapse.
#
#   1. THE RANK CAP (`cap_level_for_class`), by HEIGHT CLASS (user policy,
#      2026-08-31, superseding the earlier blanket "no fire in skyscraper
#      districts"):
#        low         rowhouse / lowrise            -- full collapse stands
#                    (F6 allowed, F5c allowed)
#        mid_high    midrise / brick_midrise /      -- PARTIAL collapse only
#                    tower / highrise                  (cap F5c, never F6)
#        skyscraper  -- see below: NOTHING is        -- fire only (cap F5,
#                    "skyscraper" by typology name;      never F5c/F6)
#                    see ROOF ELIGIBILITY below for
#                    what actually reaches this cap
#      `tower`/`highrise` were `NO_FIRE_TYPOLOGIES` in `urban_fire_city.py`
#      before this policy -- fire was banned outright in both. That ban is
#      LIFTED (both may now ignite), replaced by this cap: `midrise` and
#      `brick_midrise` (never banned, and previously UNCAPPED -- an `urm`
#      kit archetype dropped into one of those districts could reach F6
#      today) get the new "partial only" ceiling; `tower`/`highrise` (the
#      old ban list, still grouped together here because their MEASURED
#      height pools overlap 44.7-131 m tower vs. 103.7-312 m highrise --
#      splitting them by typology name is unambiguous, splitting them by
#      height is not) get "fire only, never any collapse".
#   2. ROOF ELIGIBILITY (`enforce_roof_eligibility`), by CONSTRUCTION, not
#      by rank (fact-checked policy, 2026-08-31): structural roof loss in a
#      real fire is a LIGHTWEIGHT-TIMBER phenomenon -- joist-and-truss
#      roofs, the rowhouse/lowrise stock. An RC deck essentially never
#      collapses in a compartment fire and a steel-deck failure is rare and
#      local, which is the ladder's own doctrine already (`LADDER["rc_glass"]`
#      carries no collapse recipe at any level -- "the shell stands"). F5C
#      AND F6 ARE THE TWO LEVELS WHOSE LADDER RECIPE ACTUALLY BREACHES THE
#      SHELL (`partial_collapse` / `fire_collapse` in `urban_fire.LADDER`);
#      F5 keeps `level_for_age`'s own doctrine for that state ("gutted, top
#      storeys fallen in, FOUR WALLS UP" -- the shell stands). So `F5c`/`F6`
#      (`ROOF_LEVELS`) are eligible ONLY for the `low` class, REGARDLESS of
#      what the rank cap above would otherwise allow -- this is what makes
#      `mid_high`'s own "cap at F5c" a ceiling that is rarely if ever
#      actually reached in the final manifest: `cap_level_for_class` still
#      lets a `mid_high` building's F6 degrade to F5c (that degradation is
#      independently correct and independently tested), and
#      `enforce_roof_eligibility` then degrades THAT F5c to F5, because a
#      partial collapse is still a roof-affecting outcome and `mid_high`
#      construction (predominantly `rc`) is not the timber class it is
#      reserved for. `skyscraper`'s own rank cap already forbids F5c/F6
#      outright, so this is a no-op there -- roof eligibility only ever
#      bites on `mid_high`.
#
# `urban_fire_city.damaged_manifest` additionally caps the SHARE of the
# whole manifest that may show a `ROOF_LEVELS` outcome at all (a knob,
# `roof_collapse_max`, default 2 per a ~26-building city manifest) --
# "eligible" is necessary but not sufficient; roof collapse must also stay
# RARE. That budget needs the whole city's record list, so it lives there,
# not here; everything on this page is a pure per-building function.
HEIGHT_CLASS_LOW = "low"
HEIGHT_CLASS_MIDHIGH = "mid_high"
HEIGHT_CLASS_SKYSCRAPER = "skyscraper"
HEIGHT_CLASSES = (HEIGHT_CLASS_LOW, HEIGHT_CLASS_MIDHIGH, HEIGHT_CLASS_SKYSCRAPER)

#: district typology name (`districts.py`'s six `downtown_gac.yaml`
#: typologies, `config/presets/downtown_gac.yaml:1264` `districts.
#: typologies`) -> height class. The SOURCE OF TRUTH: `height_class()` always
#: prefers this over either fallback below.
TYPOLOGY_HEIGHT_CLASS = {
    "rowhouse": HEIGHT_CLASS_LOW,          # brownstones, height_median_m 14.5
    "lowrise": HEIGHT_CLASS_LOW,           # height_median_m 13.5, pool to 16.7
    "midrise": HEIGHT_CLASS_MIDHIGH,       # height_median_m 36.0, pool 29-48
    "brick_midrise": HEIGHT_CLASS_MIDHIGH, # height_median_m 58.0, pool 38.6-71.8
    "tower": HEIGHT_CLASS_SKYSCRAPER,      # height_median_m 78.0, pool 44.7-131/140
    "highrise": HEIGHT_CLASS_SKYSCRAPER,   # height_median_m 165.0, pool 103.7-312/320
}

# Storey-count fallback -- for a caller that has an estimated storey count
# (`fire_city_dry_run._estimate_storeys`) but no typology (a synthetic bench,
# or a record whose district block was not recovered). Boundaries are ROUND
# numbers bracketing the MEASURED pools above at a generic ~3.0-3.5 m/storey
# (`fire_city_dry_run._estimate_storeys`'s own GAC/kit constants): rowhouse/
# lowrise's ~13.5-16.7 m tops out at 4-5 storeys; brick_midrise's ~71.8 m
# ceiling is 20-24 storeys; tower's own measured FLOOR (44.7 m, ~13 storeys)
# sits inside that same band -- typology name resolves that overlap when it
# is known, this is only the fallback for when it is not.
STOREY_LOW_MAX = 4          # <=4 storeys: low
STOREY_MIDHIGH_MAX = 25     # 5..25 storeys: mid_high; >25: skyscraper

# Height-in-metres fallback -- for a caller with only `H` (e.g. `solve()`'s
# own per-building default when it is not handed a `height_class_of`
# callback). Same measured pools, same acknowledged tower/highrise overlap.
LOW_H_MAX_M = 20.0          # above lowrise's measured 16.7 m ceiling
MIDHIGH_H_MAX_M = 80.0      # above brick_midrise's measured 71.8 m ceiling


def height_class(typology=None, n_storeys=None, H_m=None):
    """The collapse-eligibility class for a building: `"low"`, `"mid_high"`
    or `"skyscraper"`.

    Tries, in order: the district TYPOLOGY name (`TYPOLOGY_HEIGHT_CLASS`,
    the source of truth); a storey count (`STOREY_LOW_MAX`/
    `STOREY_MIDHIGH_MAX`); a height in metres (`LOW_H_MAX_M`/
    `MIDHIGH_H_MAX_M`). Falls back to `"mid_high"` -- the CONSERVATIVE
    middle -- when none of the three is given: a building nobody could
    identify the district or height of is assumed NOT to be the low-rise/
    timber stock (so it never gets an unrestricted full collapse) and NOT
    to be a skyscraper either (so a caller that only wants "never F6" is not
    surprised by "also never F5c").
    """
    if typology in TYPOLOGY_HEIGHT_CLASS:
        return TYPOLOGY_HEIGHT_CLASS[typology]
    if n_storeys is not None:
        n = int(n_storeys)
        if n <= STOREY_LOW_MAX:
            return HEIGHT_CLASS_LOW
        if n <= STOREY_MIDHIGH_MAX:
            return HEIGHT_CLASS_MIDHIGH
        return HEIGHT_CLASS_SKYSCRAPER
    if H_m is not None:
        h = float(H_m)
        if h <= LOW_H_MAX_M:
            return HEIGHT_CLASS_LOW
        if h <= MIDHIGH_H_MAX_M:
            return HEIGHT_CLASS_MIDHIGH
        return HEIGHT_CLASS_SKYSCRAPER
    return HEIGHT_CLASS_MIDHIGH


#: level -> the level it degrades to when its class refuses it. F6 always
#: steps down to F5c first (still gutted and gone, just not to the ground),
#: F5c steps down to F5 (burnt-out, standing, no collapse at all) -- never
#: straight to F0 and never outside `LEVELS`.
_LEVEL_DEGRADE = {"F6": "F5c", "F5c": "F5"}

#: class -> the levels that class's RANK CAP forbids outright.
_CLASS_BANNED_LEVELS = {
    HEIGHT_CLASS_LOW: frozenset(),
    HEIGHT_CLASS_MIDHIGH: frozenset({"F6"}),
    HEIGHT_CLASS_SKYSCRAPER: frozenset({"F5c", "F6"}),
}


def cap_level_for_class(level, cls):
    """`level`, degraded along `_LEVEL_DEGRADE` until `cls`'s rank cap no
    longer forbids it. A no-op for a `level`/`cls` combination the cap
    already allows (including every level below F5c for every class)."""
    banned = _CLASS_BANNED_LEVELS.get(cls, frozenset())
    lvl = level
    while lvl in banned and lvl in _LEVEL_DEGRADE:
        lvl = _LEVEL_DEGRADE[lvl]
    return lvl


#: the two levels whose `urban_fire.LADDER` recipe actually breaches the
#: shell (`partial_collapse`/`fire_collapse`) rather than just gutting it --
#: see the module-docstring section above for why F5 is deliberately not a
#: member.
ROOF_LEVELS = frozenset({"F5c", "F6"})

#: the one height class light-timber, joist-and-truss roof construction is
#: common in -- the only class a `ROOF_LEVELS` outcome may ever be assigned
#: to, regardless of what the rank cap alone would allow.
ROOF_ELIGIBLE_CLASS = HEIGHT_CLASS_LOW


def roof_eligible(cls):
    """Whether `cls` may ever carry a `ROOF_LEVELS` outcome at all."""
    return cls == ROOF_ELIGIBLE_CLASS


def enforce_roof_eligibility(level, cls):
    """`level`, forced to `"F5"` if it is a `ROOF_LEVELS` outcome `cls` is
    not eligible for; a no-op for every other level/class combination.

    Applying this AFTER `cap_level_for_class` is what makes `mid_high`'s own
    "cap at F5c" ceiling rarely visible in a real manifest: the rank cap
    still lets F6 degrade to F5c there (a real, independently-tested step),
    and this then degrades that F5c the rest of the way to F5, because a
    partial collapse is still roof-affecting and `mid_high` is not the
    timber class. `skyscraper` never reaches this with a `ROOF_LEVELS` level
    in hand at all -- its own rank cap already forbids both.
    """
    if level in ROOF_LEVELS and not roof_eligible(cls):
        return "F5"
    return level


def check_levels_sync():
    """`LEVELS` here == `urban_fire.LEVELS` there, or a list of complaints.

    Returns [] when `urban_fire` cannot be imported at all — this module is
    allowed to be used without it.
    """
    uf = _urban_fire()
    if uf is None:
        return []
    bad = []
    if tuple(uf.LEVELS) != tuple(LEVELS):
        bad.append("LEVELS drifted from urban_fire: {0} vs {1}".format(
            LEVELS, tuple(uf.LEVELS)))
    for lv in LEVELS:
        if lv not in RANK:
            bad.append("no RANK for level {0}".format(lv))
    return bad


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def _corners(b):
    """The four world corners of a building's footprint."""
    a = math.radians(b["yaw"])
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = b["W"] / 2.0, b["D"] / 2.0
    return [(b["x"] + ca * dx - sa * dy, b["y"] + sa * dx + ca * dy)
            for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def _seg_dist(p, q, r, s):
    """Distance between segments pq and rs (0 if they cross)."""
    def _pt_seg(pt, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        L2 = ax * ax + ay * ay
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(
            1.0, ((pt[0] - a[0]) * ax + (pt[1] - a[1]) * ay) / L2))
        return math.hypot(pt[0] - (a[0] + t * ax), pt[1] - (a[1] + t * ay))

    d1 = (q[0] - p[0]) * (s[1] - r[1]) - (q[1] - p[1]) * (s[0] - r[0])
    if abs(d1) > 1e-12:
        t = ((r[0] - p[0]) * (s[1] - r[1]) - (r[1] - p[1]) * (s[0] - r[0])) / d1
        u = ((r[0] - p[0]) * (q[1] - p[1]) - (r[1] - p[1]) * (q[0] - p[0])) / d1
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0
    return min(_pt_seg(p, r, s), _pt_seg(q, r, s),
               _pt_seg(r, p, q), _pt_seg(s, p, q))


def gap_m(a, b):
    """Clear distance between two building footprints, 0 if they touch.

    EDGE TO EDGE, NOT CENTRE TO CENTRE. Two 40 m blocks 2 m apart have their
    centres 42 m apart, which every distance-based rule would score as "far"
    — and they are close enough to share a fire. The whole model turns on this
    being the clear gap between the walls.
    """
    ca, cb = _corners(a), _corners(b)
    best = 1e9
    for i in range(4):
        for j in range(4):
            best = min(best, _seg_dist(ca[i], ca[(i + 1) % 4],
                                       cb[j], cb[(j + 1) % 4]))
    return best


def bearing(a, b):
    """Compass bearing from a's centre to b's centre, radians, +X = 0."""
    return math.atan2(b["y"] - a["y"], b["x"] - a["x"])


def side_facing(b, wx, wy):
    """Which of b's elevations (S/E/N/W, in its own frame) faces (wx, wy).

    `urban_building`'s front is -Y in the building's local frame, which
    `quake_flow._side_of` calls "S"; the same convention has to be used here
    or every fire vents out of the back of its building.
    """
    a = math.radians(-b["yaw"])
    dx, dy = wx - b["x"], wy - b["y"]
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    # normalise by the half-extents so a long thin building still picks its
    # long face when the source is off its end
    nx = lx / max(1e-3, b["W"] / 2.0)
    ny = ly / max(1e-3, b["D"] / 2.0)
    if abs(nx) >= abs(ny):
        return "E" if nx > 0 else "W"
    return "N" if ny > 0 else "S"


def _wind_factor(brg, wind_dir, strength):
    """Delay multiplier for a neighbour lying at `brg` from the fire.

    `wind_dir` is the direction the wind BLOWS TOWARD, in radians.
    """
    if strength <= 0:
        return 1.0
    c = math.cos(brg - wind_dir)          # +1 straight downwind
    f = 1.0 + (1.0 - c) * 0.5 * (WIND_UP - WIND_DOWN) + (WIND_DOWN - 1.0)
    return max(WIND_DOWN, min(WIND_UP, 1.0 + (f - 1.0) * min(1.0, strength / 8.0)))


# ---------------------------------------------------------------------------
# STREET-FACING SIDE PREFERENCE (user policy, 2026-08-31 review of the live
# 500 m city): "most of the fire seems to face a side/another building.
# Let's try to keep it street facing."
#
# `entry_for_plan_fire` below is where a building's VENTING side(s) get
# picked. Two of the three things that decide a side there are already
# settled by the time this section's code ever runs, and neither is
# renegotiable here:
#
#   * for a non-origin building the ENTRY side is the elevation FACING THE
#     NEIGHBOUR THAT LIT IT (`solve()`'s own `side_facing`) — that is
#     PHYSICAL REALISM, not a free choice, and re-picking it for a nicer
#     camera angle would break the "one event moving through a street" read
#     the whole spread model exists for (module docstring, "THE ENTRY POINT
#     IS THE POINT"). This code never touches it.
#   * how MANY sides vent at all is the side-COUNT policy just below
#     (`entry_for_plan_fire`'s F3+ branch) — orthogonal to this section.
#
# What IS still a free choice, in exactly two places, is WHICH elevation:
# the ORIGIN building's own entry side (nothing lit it, so nothing dictates
# a direction) and, for F3+, WHICH of the two `side_neighbors(side)`
# elevations gets added when only one is being added (see the count policy).
# `street_side_score` scores every one of S/E/N/W by how open it is, and
# `entry_for_plan_fire` uses that score to PREFER a street-facing elevation
# in exactly those two free-choice spots — never overriding the entry side
# itself.
#
# THE METRIC (plan's own "at minimum" suggestion): a facade is more
# street-like the FARTHER it is from the nearest building that faces it and
# the CLOSER it is to leaving the block it stands in (crossing into the
# `road_corridors` a real layout carries — `disaster.fire_people.
# derive_layout` builds exactly that as `rect_complement(region, blocks)`
# off the same FC-dump typology blocks `urban_fire_city.typology_at` already
# reads). Concretely: `score = neighbour_clearance - block_edge_distance`.
# A big POSITIVE score means the facade reaches the edge of its own block
# — the road — long before it would reach another building: genuinely
# street-facing. A small or negative score means a neighbour sits AS CLOSE
# OR CLOSER than the block boundary: that face looks at the neighbour,
# however far away the road eventually is. `block_rect=None` (no layout
# available — a synthetic bench, or a caller with no district data) drops
# the second term and scores on neighbour clearance alone, which is exactly
# the "at minimum" fallback: still bigger-is-more-open, just without the
# road-crossing refinement.
# ---------------------------------------------------------------------------
#: defensive clamp on both terms of the score -- not a physical sight
#: distance, just a ceiling so one very isolated building on a very large
#: plate does not produce an unbounded score that swamps every comparison.
STREET_SCORE_CAP_M = 200.0


def _facade_point_and_normal(b, side):
    """`(fx, fy, nx, ny)`: the world-space midpoint of `b`'s `side`
    elevation and its OUTWARD unit normal. Same front-is--Y convention as
    `_corners`/`side_facing` (`quake_flow._side_of`'s own "S")."""
    hw, hd = b["W"] / 2.0, b["D"] / 2.0
    lx, ly, lnx, lny = {"S": (0.0, -hd, 0.0, -1.0), "E": (hw, 0.0, 1.0, 0.0),
                        "N": (0.0, hd, 0.0, 1.0), "W": (-hw, 0.0, -1.0, 0.0)
                        }[side]
    a = math.radians(b["yaw"])
    ca, sa = math.cos(a), math.sin(a)
    fx = b["x"] + ca * lx - sa * ly
    fy = b["y"] + sa * lx + ca * ly
    nx, ny = ca * lnx - sa * lny, sa * lnx + ca * lny
    return fx, fy, nx, ny


def _ray_exit_distance(px, py, dx, dy, rect):
    """Distance from `(px, py)` — assumed inside axis-aligned `rect`
    `(x0, y0, x1, y1)` — to where a ray in direction `(dx, dy)` leaves it.
    `None` if the point is not actually inside `rect` or the direction is
    the zero vector (never happens for a `_facade_point_and_normal` output,
    but checked rather than assumed)."""
    x0, y0, x1, y1 = rect
    if not (x0 - 1e-6 <= px <= x1 + 1e-6 and y0 - 1e-6 <= py <= y1 + 1e-6):
        return None
    best = None
    if abs(dx) > 1e-9:
        t = ((x1 if dx > 0 else x0) - px) / dx
        if t > 0:
            best = t
    if abs(dy) > 1e-9:
        t = ((y1 if dy > 0 else y0) - py) / dy
        if t > 0 and (best is None or t < best):
            best = t
    return best


def street_side_score(b, side, buildings, block_rect=None,
                      cap_m=STREET_SCORE_CAP_M):
    """Higher = `side` more likely opens onto a STREET; lower = onto a
    neighbouring building. See the section docstring above for the metric.

    `buildings` is the plate's own building list (identity-compared against
    `b`, so `b` itself is skipped even if it is also IN `buildings`, which
    it usually is — callers do not need to filter it out first).
    `block_rect` is the `(x0, y0, x1, y1)` block `b` was zoned into
    (`urban_fire_city.typology_at`'s own key), or `None` to score on
    neighbour clearance alone.
    """
    neighbour = cap_m
    for c in buildings:
        if c is b:
            continue
        if side_facing(b, c["x"], c["y"]) != side:
            continue
        neighbour = min(neighbour, gap_m(b, c))
    neighbour = min(neighbour, cap_m)
    if block_rect is None:
        return neighbour
    fx, fy, nx, ny = _facade_point_and_normal(b, side)
    edge = _ray_exit_distance(fx, fy, nx, ny, block_rect)
    edge = cap_m if edge is None else min(edge, cap_m)
    return neighbour - edge


# ---------------------------------------------------------------------------
# The solve
# ---------------------------------------------------------------------------
def edges(buildings, wind_dir=0.0, wind_mps=5.0, rng=None,
          blocked=frozenset()):
    """Every ordered pair the fire could cross, with its delay and mechanism.

    Returns [(i, j, delay_s, mechanism)].

    A BLOCKED INDEX IS NOT IN THE GRAPH AT ALL — no edge in, no edge out.
    That is a firebreak, not a fireproof building: the fire has to route
    AROUND a blocked run, and if the only path to the far side went through
    one, the far side simply never catches. This is how a city solve keeps
    the tower/highrise districts out of a conflagration (the no-fire rule
    lives on the BLOCK, `layout["_typology_of"]`), and it is why the rule is
    expressed by deleting nodes rather than by discarding the result
    afterwards: a tower cannot be a stepping stone it was never on.
    """
    out = []
    n = len(buildings)
    blocked = frozenset(blocked or ())
    for i in range(n):
        if i in blocked:
            continue
        a = buildings[i]
        ha = max(6.0, float(a.get("H", H_REF_M)))
        for j in range(n):
            if i == j or j in blocked:
                continue
            b = buildings[j]
            g = gap_m(a, b)
            brg = bearing(a, b)
            wf = _wind_factor(brg, wind_dir, wind_mps)
            # a bigger radiant panel lights its neighbour sooner
            hf = (H_REF_M / ha) ** 0.45
            if g <= ATTACHED_GAP_M:
                lo, hi = D_ATTACHED
                d = (lo + (hi - lo) * (rng.random() if rng else 0.5))
                out.append((i, j, d * hf, "attached"))
                continue
            if g <= RAD_REACH_M:
                # flux ~ 1/d^2, so delay ~ d^2 between the two anchors
                t = (g - 2.0) / max(1e-3, RAD_REACH_M - 2.0)
                t = max(0.0, min(1.0, t))
                d = D_RAD_NEAR + (D_RAD_FAR - D_RAD_NEAR) * (t ** 2)
                jitter = (0.8 + 0.4 * rng.random()) if rng else 1.0
                out.append((i, j, d * wf * hf * jitter, "radiation"))
                continue
            if SPOT_MIN_M <= g <= SPOT_REACH_M:
                # BRANDS ONLY GO DOWNWIND. This is the only mechanism that
                # crosses a street, and gating it on the wind is what gives a
                # conflagration a DIRECTION — without it the fire grows as a
                # disc and reads as an explosion rather than a spread.
                if math.cos(brg - wind_dir) < 0.35 or wind_mps < 1.0:
                    continue
                if rng is not None and rng.random() > SPOT_P:
                    continue
                lo, hi = D_SPOT
                u = rng.random() if rng else 0.5
                d = lo + (hi - lo) * (g / SPOT_REACH_M) * (0.6 + 0.8 * u)
                out.append((i, j, d * wf, "spot"))
    return out


def solve(buildings, origin_idx, elapsed_s, wind_dir=0.0, wind_mps=5.0,
          rng=None, btype_of=None, collapse_p=COLLAPSE_P,
          blocked=frozenset(), max_burnt=None, burnt_out_p=BURNT_OUT_P,
          height_class_of=None):
    """Ignition time, level and entry point for every building.

    A Dijkstra relaxation from `origin_idx` over the edge set, with the
    building's own `T_FULL` added before it can light anyone — a fire in one
    room does not ignite the building opposite.

    `blocked` is a set of indices that are NOT IN THE GRAPH (see `edges`):
    they come back `t_ignite=None`, `level="F0"`, and can never appear as
    another building's `via`. `origin_idx` may not be blocked.

    `max_burnt`, if given, caps the burnt set to that many buildings by
    `cap_to_prefix` — see there for why a prefix of the Dijkstra order is
    still one connected fire.

    `height_class_of(building) -> one of HEIGHT_CLASSES`, if given, is
    consulted per building and the RAW `level_for_age` result is degraded
    with `cap_level_for_class` before it is returned — see the module
    docstring's "HEIGHT CLASS" section. Without it, every building falls
    back to `height_class(H_m=b.get("H"))` — the height-in-metres signal is
    always available here (every building carries `"H"`), so a cap is
    ALWAYS applied, just a less precise one than a caller with real typology
    data can supply. This is the RANK CAP ONLY; roof-outcome ELIGIBILITY
    (`enforce_roof_eligibility`) and the city-wide roof-outcome SHARE budget
    are manifest-generation concerns (`urban_fire_city.damaged_manifest`)
    and are not applied here — see the module docstring for why the two are
    kept in separate places.

    Returns a list of dicts, one per building, in the same order:
        t_ignite   seconds from t=0 (`None` = never caught)
        age        `elapsed_s - t_ignite`
        level      one of `LEVELS`, already capped by height class
        height_class the class `level` was capped against
        via        which building lit it, or None for the origin
        how        attached / radiation / spot / origin
        entry_side S/E/N/W in the building's own frame
        origin_frac the height the fire entered at, as a fraction
    """
    n = len(buildings)
    blocked = frozenset(blocked or ())
    if origin_idx in blocked:
        raise ValueError(
            "solve: origin {0} is blocked — pick_origin() draws only from "
            "the unblocked set for exactly this reason".format(origin_idx))
    E = edges(buildings, wind_dir, wind_mps, rng, blocked=blocked)
    adj = {}
    for i, j, d, how in E:
        adj.setdefault(i, []).append((j, d, how))
    INF = float("inf")
    t = [INF] * n
    via = [None] * n
    how = [None] * n
    t[origin_idx] = 0.0
    how[origin_idx] = "origin"
    # small n, so an O(n^2) scan is simpler than a heap and just as fast.
    # A blocked node starts DONE, so the scan can never settle it and it can
    # never be relaxed out of (belt and braces: `edges` already dropped it).
    done = [i in blocked for i in range(n)]
    for _ in range(n):
        u, best = -1, INF
        for k in range(n):
            if not done[k] and t[k] < best:
                u, best = k, t[k]
        if u < 0:
            break
        done[u] = True
        # IT MUST BE FULLY INVOLVED BEFORE IT CAN LIGHT ANYTHING.
        ready = t[u] + T_FULL
        for v, d, hw in adj.get(u, ()):
            if done[v]:
                continue
            if ready + d < t[v]:
                t[v] = ready + d
                via[v] = u
                how[v] = hw
    out = []
    for k in range(n):
        b = buildings[k]
        ti = None if t[k] == INF else t[k]
        age = None if ti is None else (float(elapsed_s) - ti)
        bt = (btype_of(b) if btype_of else "urm")
        lvl = ("F0" if age is None
               else level_for_age(age, bt, rng, burnt_out_p, collapse_p))
        cls = (height_class_of(b) if height_class_of is not None
               else height_class(H_m=b.get("H")))
        lvl = cap_level_for_class(lvl, cls)
        # --- where the fire got in ---------------------------------------
        side, frac = None, 0.25
        if via[k] is not None:
            src = buildings[via[k]]
            side = side_facing(b, src["x"], src["y"])
            if how[k] == "spot":
                # A BRAND LANDS ON THE ROOF. It is the one mechanism that
                # starts a fire at the TOP of a building, and that reads
                # instantly as a different mechanism from the wall-to-wall
                # spread happening down the street.
                frac = 0.88
            elif how[k] == "attached":
                # through the party wall, wherever the neighbour was worst —
                # low, because that is where its fire started
                frac = 0.22
            else:
                # radiation: at about the height of the radiant panel, which
                # is the middle of the neighbour's involved band
                frac = 0.45
        elif how[k] == "origin":
            side = None                    # drawn by the caller
            frac = 0.15                    # fires start low
        # LOW-BIASED CAP. See ORIGIN_FRAC_CAP: no mechanism (today, only
        # "spot" exceeds it) may start a fire above half the mass.
        frac = min(frac, ORIGIN_FRAC_CAP)
        out.append({"i": k, "t_ignite": ti, "age": age, "level": lvl,
                    "height_class": cls,
                    "via": via[k], "how": how[k], "entry_side": side,
                    "origin_frac": frac})
    if max_burnt is not None:
        out = cap_to_prefix(out, max_burnt)
    return out


# ---------------------------------------------------------------------------
# Choosing the origin, and capping the size of the fire
# ---------------------------------------------------------------------------
def pick_origin(buildings, blocked=frozenset(), rng=None, epicenter=None):
    """Which building starts it — biased toward the epicentre, never blocked.

    A city preset names an `epicenter` (`compile_disaster` compiles one for
    `fire` the same way it does for every other disaster type). The origin
    has to RESPECT it without being pinned to it: pinning puts the fire in
    whatever happens to stand at that coordinate — a tower, a car park, a
    building with no bake kind — and the whole plate is then decided by one
    lookup. So the unblocked candidates are RANKED by `1 / (1 + distance)`
    (nearest first) and one is drawn off that ranking with `u ** 1.7`, the
    same low-biased shape `urban_fire.plan_fire` uses to put a fire's origin
    storey near the ground: about 60 % of draws land in the nearest third,
    and the tail still reaches the far side of the plate.

    Deterministic for a given `rng` — exactly one `rng.random()` is consumed,
    so a caller can reproduce a plate from `(layout seed, FIRE_SEED)`.
    `epicenter` defaults to the plate centre, `(0, 0)`.
    """
    blocked = frozenset(blocked or ())
    cands = [i for i in range(len(buildings)) if i not in blocked]
    if not cands:
        raise ValueError("pick_origin: no unblocked building to ignite")
    ex, ey = ((0.0, 0.0) if epicenter is None
              else (float(epicenter[0]), float(epicenter[1])))

    def _key(i):
        b = buildings[i]
        d = math.hypot(float(b["x"]) - ex, float(b["y"]) - ey)
        # rank by the WEIGHT, descending; the index breaks ties so two
        # buildings the same distance out always rank the same way
        return (-(1.0 / (1.0 + d)), i)

    ranked = sorted(cands, key=_key)
    u = rng.random() if rng is not None else 0.0
    k = int(len(ranked) * (u ** ORIGIN_BIAS))
    return ranked[max(0, min(len(ranked) - 1, k))]


def cap_to_prefix(plan, n):
    """Keep the `n` earliest ignitions and send the rest back to F0.

    WHY A PREFIX IS ALWAYS ONE CONNECTED FIRE. `solve` is a Dijkstra, so
    every building's `via` is its parent in a shortest-path tree rooted at
    the origin, and the relaxation is `t[v] = t[via] + T_FULL + delay`. Both
    `T_FULL` (1200 s) and every `delay` are strictly positive, so

        t_ignite(via(v))  <  t_ignite(v)      for every non-origin v

    — a parent ALWAYS ignites strictly before its child. Sorting by
    `t_ignite` therefore lists every parent before its child, and a prefix of
    that order is closed under `via`: the kept set is a subtree containing
    the origin, which is to say one connected fire. No search, no repair
    pass, no "nearest N" heuristic that would leave islands of burnt
    buildings with untouched ones between them.

    That is asserted here rather than assumed, because it is the property the
    whole city plan rests on (`_plans/urban_fire_city_plan.md` §5 check 2).

    Returns a NEW plan (the dicts are copied); the input is untouched.
    """
    n = max(0, int(n))
    lit = sorted((p for p in plan if p["t_ignite"] is not None),
                 key=lambda p: (p["t_ignite"], p["i"]))
    keep = set(p["i"] for p in lit[:n])
    for p in lit[:n]:
        if p["via"] is not None and p["via"] not in keep:
            raise AssertionError(
                "cap_to_prefix: building {0} kept but its via {1} was not — "
                "the Dijkstra order is not parent-before-child".format(
                    p["i"], p["via"]))
    out = []
    for p in plan:
        q = dict(p)
        if q["t_ignite"] is not None and q["i"] not in keep:
            q.update({"t_ignite": None, "age": None, "level": "F0",
                      "via": None, "how": None, "entry_side": None,
                      "origin_frac": 0.25})
        out.append(q)
    return out


def entry_for_plan_fire(rec, n_storeys, rng=None, street_score=None):
    """One spread record + a building's storey count -> `plan_fire` arguments.

    Returns `(origin_storey, sides)` to be passed straight through as
    `urban_fire.plan_fire(info, level, rng, origin=..., sides=...)`.

    THIS IS THE JOIN BETWEEN THE TWO MODELS AND IT IS THE WHOLE POINT OF
    SOLVING SPREAD AT ALL. `plan_fire` left to itself DRAWS the origin storey
    and shuffles the elevations, which is right for a single building on a
    bench and wrong for a street: every fire then vents in a random
    direction and the row reads as unrelated fires that happen to be
    adjacent. Handing it the storey and the side the SOURCE implies is what
    makes a row read as one event travelling along it.

      * `origin_frac` is a fraction of the building's HEIGHT and `plan_fire`
        wants a storey INDEX, so it is scaled by `n_storeys - 1`. A brand
        (0.88) therefore lands in the top eighth of the block — the top
        storey of a 3-storey terrace, storey 21 of a 25-storey tower — and a
        party wall (0.22) low, at both heights. It is a fraction rather than
        "the top storey" because on anything tall those are different
        places, and a brand fire that starts four floors down and climbs is
        the correct picture.
      * F1/F2 vent through exactly ONE elevation — the compartment that is
        alight opens onto exactly one face.
      * F3+ vent through TWO OR THREE (2026-08-31 user policy: "live fire
        seems to mostly only stay on 1 side of the building, it has to look
        like more" — F3+ records were carrying 1-2 sides in practice, never
        3). From F3 the fire has gone through the floor plate, so it always
        takes at least one elevation round the CORNER from the entry side
        (`side_neighbors`, exactly 2 candidates); which COUNT it takes is a
        draw shifted up by one option from what this used to be a fixed
        count of 1 — `rng.randint(1, len(nb))` now covers `{1, 2}` extra
        elevations (2 or 3 total) with no `rng` still falling back to the
        OLD deterministic "exactly 1 extra" (2 total), so a caller that
        never passed an `rng` here (there was never a reason to, since the
        old code did not read it for the count) sees no change at all.
        F1/F2's single-elevation rule is untouched either way.

    The ORIGIN building has `entry_side=None` (nothing lit it, so nothing
    about contagion dictates a direction — see the module's "STREET-FACING
    SIDE PREFERENCE" section above `street_side_score`), so its side is the
    ONE place this function has a completely free choice of elevation, and
    `street_score` (when given) decides it by ranking S/E/N/W rather than by
    a uniform `rng` draw (which is still the fallback when `street_score` is
    `None`, and `"S"` when `rng` is `None` too). The SAME function is also
    consulted for F3+'s extra elevation, but ONLY when exactly one of the
    two `side_neighbors(side)` candidates is being added — the more
    street-facing of the two wins that tie (ties in `street_score` itself,
    or no `street_score` at all, fall back to the OLD `rng.randrange` pick).
    THE ENTRY SIDE ITSELF (the elevation facing the neighbour that actually
    lit this building) is NEVER re-ranked by `street_score` — precedence is
    entry-side REALISM first, street-facing VISIBILITY only where nothing
    about contagion has already decided the answer (the two spots above).
    """
    n = max(1, int(n_storeys))
    frac = rec.get("origin_frac")
    frac = 0.15 if frac is None else max(0.0, min(1.0, float(frac)))
    storey = max(0, min(n - 1, int(round(frac * (n - 1)))))
    side = rec.get("entry_side")
    if side not in _SIDE_RING:
        if street_score is not None:
            side = max(_SIDE_RING, key=street_score)
        else:
            side = _SIDE_RING[rng.randrange(len(_SIDE_RING))] if rng else "S"
    if RANK.get(rec.get("level", "F0"), 0) >= RANK["F3"]:
        nb = side_neighbors(side)
        extra_n = (rng.randint(1, len(nb)) if rng else 1)
        if extra_n >= len(nb):
            return storey, (side,) + tuple(nb)
        if street_score is not None:
            chosen = max(nb, key=street_score)
        else:
            chosen = nb[rng.randrange(len(nb))] if rng else nb[0]
        return storey, (side, chosen)
    return storey, (side,)


def summarise(buildings, plan, elapsed_s):
    """One line per building, oldest fire first — the banner."""
    rows = []
    for p in plan:
        b = buildings[p["i"]]
        rows.append((-1e9 if p["t_ignite"] is None else -p["t_ignite"],
                     p["i"], p, b))
    # SORT ON THE KEY AND THE INDEX ONLY. A plain `rows.sort()` compares the
    # tuples elementwise, so ANY TIE on the first element falls through to
    # comparing the `plan` dicts and raises `TypeError: '<' not supported
    # between instances of 'dict' and 'dict'`. Every building the fire never
    # reached ties at -1e9, so this fires the moment a plate has two of them
    # — which a 100 m block never had and a 500 m city has eighteen of.
    rows.sort(key=lambda r: (r[0], r[1]))
    lines = []
    for _k, _i, p, b in rows:
        if p["t_ignite"] is None:
            lines.append("  {0:<18} {1}  not reached".format(
                b.get("style", "?"), p["level"]))
            continue
        lines.append(
            "  {0:<18} {1}  lit at T+{2:>3.0f} min ({3:<9}) burning {4:>3.0f} "
            "min, in on the {5} face at {6:.0%} height{7}".format(
                b.get("style", "?"), p["level"], p["t_ignite"] / 60.0,
                p["how"], max(0.0, p["age"]) / 60.0, p["entry_side"] or "-",
                p["origin_frac"],
                "" if p["via"] is None
                else "  <- {0}".format(buildings[p["via"]].get("style", "?"))))
    return lines


def check(verbose=True):
    """Host-side sanity: the mechanisms fire, and the wave has a direction."""
    import random
    bad = []
    # a terrace: five 20 x 15 m buildings 0.5 m apart, and one across a 14 m
    # street from the middle of it
    bs = [{"x": -40.0 + i * 20.5, "y": 0.0, "W": 20.0, "D": 15.0,
           "yaw": 0.0, "H": 18.0, "style": "t{0}".format(i)} for i in range(5)]
    bs.append({"x": 0.0, "y": -29.0, "W": 20.0, "D": 15.0, "yaw": 180.0,
               "H": 18.0, "style": "across"})
    rng = random.Random(3)
    pl = solve(bs, 0, 120 * 60, wind_dir=0.0, wind_mps=6.0, rng=rng)
    ts = [p["t_ignite"] for p in pl[:5]]
    if any(x is None for x in ts):
        bad.append("terrace: not every attached neighbour caught")
    elif ts != sorted(ts):
        bad.append("terrace: ignition order is not down the row: {0}".format(ts))
    if pl[0]["level"] not in ("F4", "F5"):
        bad.append("origin at T+120 min should be burnt out, got {0}"
                   .format(pl[0]["level"]))
    if pl[4]["t_ignite"] is not None and pl[4]["level"] in ("F4", "F5"):
        bad.append("the far end of the terrace should still be burning")
    # the entry side of a building lit from its west neighbour must be W
    if pl[1]["entry_side"] != "W":
        bad.append("entry side should face the source, got {0}"
                   .format(pl[1]["entry_side"]))
    # gap: touching rects
    if gap_m(bs[0], bs[0]) > 1e-6:
        bad.append("gap to self should be 0")
    if abs(gap_m(bs[0], bs[1]) - 0.5) > 0.01:
        bad.append("terrace gap should be 0.5 m, got {0:.2f}".format(
            gap_m(bs[0], bs[1])))
    # --- the ladder is the one `urban_fire` publishes ---------------------
    bad.extend(check_levels_sync())
    # --- a blocked building is a firebreak, not a fireproof house ---------
    blk = solve(bs, 0, 120 * 60, wind_dir=0.0, wind_mps=6.0,
                rng=random.Random(3), blocked=frozenset([2]))
    if blk[2]["t_ignite"] is not None or blk[2]["level"] != "F0":
        bad.append("blocked building ignited anyway: {0}".format(blk[2]))
    if any(p["via"] == 2 for p in blk):
        bad.append("blocked building was used as a via")
    # --- the prefix cap keeps a connected fire ----------------------------
    cap = cap_to_prefix(pl, 3)          # raises if the prefix is not a subtree
    if sum(1 for p in cap if p["t_ignite"] is not None) != 3:
        bad.append("cap_to_prefix kept the wrong number of buildings")
    if cap[0]["t_ignite"] != 0.0:
        bad.append("cap_to_prefix dropped the origin")
    if cap_to_prefix(pl, 999) != pl:
        bad.append("cap_to_prefix over-capped a plan smaller than N")
    # --- the origin is drawn, low-biased, off the epicentre ---------------
    o1 = pick_origin(bs, frozenset([0, 1]), random.Random(7), (0.0, 0.0))
    o2 = pick_origin(bs, frozenset([0, 1]), random.Random(7), (0.0, 0.0))
    if o1 != o2:
        bad.append("pick_origin is not deterministic for a given rng")
    if o1 in (0, 1):
        bad.append("pick_origin returned a blocked building")
    # --- the entry point `plan_fire` is handed ----------------------------
    st, sides = entry_for_plan_fire(pl[1], 6)
    if not 0 <= st <= 5 or sides[0] != "W":
        bad.append("entry_for_plan_fire: got storey {0} sides {1}".format(
            st, sides))
    if len(entry_for_plan_fire(pl[2], 6)[1]) != 2:
        bad.append("F3+ should vent through two elevations")
    # --- (3) MORE SIDES BURNING: F3+ is now a draw over {2, 3}, F1/F2 always
    # stays at 1 -- see `entry_for_plan_fire`'s own docstring. `random.
    # Random(3).randint(1, 2) == 1` (the OLD, no-rng default: 2 total sides)
    # and `random.Random(0).randint(1, 2) == 2` (the NEW option: 3 total) --
    # both pinned here so a stdlib PRNG change would fail loudly rather than
    # silently stop covering the 3-side branch.
    if random.Random(3).randint(1, 2) != 1:
        bad.append("random.Random(3).randint(1, 2) drifted -- the F3+ "
                   "2-vs-3-sides check below assumes it is 1")
    if random.Random(0).randint(1, 2) != 2:
        bad.append("random.Random(0).randint(1, 2) drifted -- the F3+ "
                   "3-sides check below assumes it is 2")
    st3, sides3 = entry_for_plan_fire(pl[2], 6, random.Random(3))
    if len(sides3) != 2:
        bad.append("entry_for_plan_fire with an rng that draws 1 extra "
                   "side should still give 2 total, got {0}".format(sides3))
    st0, sides0 = entry_for_plan_fire(pl[2], 6, random.Random(0))
    if len(sides0) != 3 or set(sides0[1:]) != set(side_neighbors(sides0[0])):
        bad.append("entry_for_plan_fire with an rng that draws 2 extra "
                   "sides should give 3 total (entry + BOTH corner "
                   "neighbours), got {0}".format(sides0))
    rec_f2 = {"origin_frac": 0.3, "entry_side": "S", "level": "F2"}
    if len(entry_for_plan_fire(rec_f2, 5, random.Random(0))[1]) != 1:
        bad.append("F1/F2 should stay at exactly 1 side even with an rng "
                   "that would draw 2 extra sides for F3+")
    # --- (2) STREET-FACING SIDE PREFERENCE ---------------------------------
    # geometry: b0 has a close neighbour to its EAST and nothing at all to
    # its WEST -- W must score strictly more street-facing than E.
    _b0 = {"x": 0.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}
    _b1 = {"x": 20.0, "y": 0.0, "W": 10.0, "D": 10.0, "yaw": 0.0}
    _bl = [_b0, _b1]
    if not (street_side_score(_b0, "W", _bl) >
            street_side_score(_b0, "E", _bl)):
        bad.append("street_side_score: an open side should outscore one "
                   "with a close neighbour")
    if street_side_score(_b0, "E", _bl) != gap_m(_b0, _b1):
        bad.append("street_side_score without a block_rect should be "
                   "exactly the neighbour clearance")
    # with a block_rect: W reaches the block edge (45 m) long before it
    # would reach a neighbour (there is none) -> a big positive score; E's
    # neighbour (10 m clear) is much closer than that same 45 m edge -> a
    # NEGATIVE score, even though the edge itself is far away.
    _rect = (-50.0, -50.0, 50.0, 50.0)
    sw = street_side_score(_b0, "W", _bl, block_rect=_rect)
    se = street_side_score(_b0, "E", _bl, block_rect=_rect)
    if not (sw > 0):
        bad.append("street_side_score(W, block_rect): expected a positive "
                   "(street-facing) score, got {0}".format(sw))
    if not (se < 0):
        bad.append("street_side_score(E, block_rect): a neighbour closer "
                   "than the block edge should score negative, got "
                   "{0}".format(se))
    if abs(sw - 155.0) > 1e-6 or abs(se - (-35.0)) > 1e-6:
        bad.append("street_side_score(block_rect) arithmetic drifted: "
                   "W={0} (want 155.0) E={1} (want -35.0)".format(sw, se))
    # entry_for_plan_fire: the ORIGIN's free choice follows street_score...
    rec_origin = {"origin_frac": 0.15, "entry_side": None, "level": "F1"}
    _prefer_e = lambda s: 10.0 if s == "E" else 0.0
    _, sides_o = entry_for_plan_fire(rec_origin, 4, None, street_score=_prefer_e)
    if sides_o != ("E",):
        bad.append("entry_for_plan_fire should pick the origin's own entry "
                   "side by street_score when one is given, got {0}".format(
                       sides_o))
    # ...but NEVER for a non-origin building's entry side (contagion wins).
    rec_lit = {"origin_frac": 0.22, "entry_side": "W", "level": "F2"}
    _, sides_lit = entry_for_plan_fire(rec_lit, 4, None, street_score=_prefer_e)
    if sides_lit != ("W",):
        bad.append("entry_for_plan_fire must never override a REAL entry "
                   "side with street_score, got {0}".format(sides_lit))
    # ...and for F3+, street_score is the TIEBREAK for the single extra
    # corner side (only when exactly one is being added).
    rec_f3 = {"origin_frac": 0.45, "entry_side": "S", "level": "F3"}
    _, sides_f3 = entry_for_plan_fire(rec_f3, 6, random.Random(3),
                                      street_score=_prefer_e)
    if sides_f3 != ("S", "E"):
        bad.append("entry_for_plan_fire's F3+ single extra side should "
                   "follow street_score's preference, got {0}".format(
                       sides_f3))
    # --- height class: typology is the source of truth --------------------
    for typ, want in TYPOLOGY_HEIGHT_CLASS.items():
        if height_class(typology=typ) != want:
            bad.append("height_class({0!r}) should be {1!r}, got {2!r}".format(
                typ, want, height_class(typology=typ)))
    if height_class(typology="not_a_real_typology") != HEIGHT_CLASS_MIDHIGH:
        bad.append("an unrecognised typology should fall through to storeys/H")
    # --- height class: the storey fallback, at its own boundaries ---------
    if height_class(n_storeys=STOREY_LOW_MAX) != HEIGHT_CLASS_LOW:
        bad.append("STOREY_LOW_MAX itself should still be low")
    if height_class(n_storeys=STOREY_LOW_MAX + 1) != HEIGHT_CLASS_MIDHIGH:
        bad.append("just past STOREY_LOW_MAX should be mid_high")
    if height_class(n_storeys=STOREY_MIDHIGH_MAX) != HEIGHT_CLASS_MIDHIGH:
        bad.append("STOREY_MIDHIGH_MAX itself should still be mid_high")
    if height_class(n_storeys=STOREY_MIDHIGH_MAX + 1) != HEIGHT_CLASS_SKYSCRAPER:
        bad.append("just past STOREY_MIDHIGH_MAX should be skyscraper")
    # --- height class: the metres fallback, at its own boundaries ---------
    if height_class(H_m=LOW_H_MAX_M) != HEIGHT_CLASS_LOW:
        bad.append("LOW_H_MAX_M itself should still be low")
    if height_class(H_m=LOW_H_MAX_M + 0.1) != HEIGHT_CLASS_MIDHIGH:
        bad.append("just past LOW_H_MAX_M should be mid_high")
    if height_class(H_m=MIDHIGH_H_MAX_M) != HEIGHT_CLASS_MIDHIGH:
        bad.append("MIDHIGH_H_MAX_M itself should still be mid_high")
    if height_class(H_m=MIDHIGH_H_MAX_M + 0.1) != HEIGHT_CLASS_SKYSCRAPER:
        bad.append("just past MIDHIGH_H_MAX_M should be skyscraper")
    # --- height class: typology beats storeys beats H ----------------------
    if height_class(typology="rowhouse", n_storeys=90, H_m=300.0) != HEIGHT_CLASS_LOW:
        bad.append("typology should win over storeys/H when both are given")
    if height_class(n_storeys=2, H_m=300.0) != HEIGHT_CLASS_LOW:
        bad.append("storeys should win over H when both are given")
    if height_class() != HEIGHT_CLASS_MIDHIGH:
        bad.append("no signal at all should default to mid_high")
    # --- the rank cap degrades, never drops to F0 --------------------------
    if cap_level_for_class("F6", HEIGHT_CLASS_LOW) != "F6":
        bad.append("low should never cap F6 (brownstones/rowhouses fully collapse)")
    if cap_level_for_class("F5c", HEIGHT_CLASS_LOW) != "F5c":
        bad.append("low should never cap F5c")
    if cap_level_for_class("F6", HEIGHT_CLASS_MIDHIGH) != "F5c":
        bad.append("mid_high should cap an old-age F6 down to F5c (partial "
                   "collapse only), got {0}".format(
                       cap_level_for_class("F6", HEIGHT_CLASS_MIDHIGH)))
    if cap_level_for_class("F5c", HEIGHT_CLASS_MIDHIGH) != "F5c":
        bad.append("mid_high's own rank cap should still allow F5c")
    if cap_level_for_class("F6", HEIGHT_CLASS_SKYSCRAPER) != "F5":
        bad.append("skyscraper should cap an old-age F6 all the way to F5 "
                   "(fire only, never any collapse), got {0}".format(
                       cap_level_for_class("F6", HEIGHT_CLASS_SKYSCRAPER)))
    if cap_level_for_class("F5c", HEIGHT_CLASS_SKYSCRAPER) != "F5":
        bad.append("skyscraper should cap F5c down to F5 too")
    for lvl in ("F0", "F1", "F2", "F3", "F4", "F5"):
        for cls in HEIGHT_CLASSES:
            if cap_level_for_class(lvl, cls) != lvl:
                bad.append("cap_level_for_class should never touch {0} for "
                           "{1}".format(lvl, cls))
    # --- roof eligibility: low only, and it degrades straight to F5 --------
    if not roof_eligible(HEIGHT_CLASS_LOW):
        bad.append("low should be roof-eligible")
    if roof_eligible(HEIGHT_CLASS_MIDHIGH) or roof_eligible(HEIGHT_CLASS_SKYSCRAPER):
        bad.append("only low should be roof-eligible")
    if enforce_roof_eligibility("F6", HEIGHT_CLASS_LOW) != "F6":
        bad.append("low keeps F6 under the roof-eligibility gate")
    if enforce_roof_eligibility("F5c", HEIGHT_CLASS_LOW) != "F5c":
        bad.append("low keeps F5c under the roof-eligibility gate")
    if enforce_roof_eligibility("F5c", HEIGHT_CLASS_MIDHIGH) != "F5":
        bad.append("mid_high's own F5c should be forced to F5 by roof "
                   "eligibility -- never a roof-opening outcome outside low")
    if enforce_roof_eligibility("F6", HEIGHT_CLASS_SKYSCRAPER) != "F5":
        bad.append("skyscraper's F6 should be forced to F5 too")
    for lvl in ("F0", "F1", "F2", "F3", "F4", "F5"):
        for cls in HEIGHT_CLASSES:
            if enforce_roof_eligibility(lvl, cls) != lvl:
                bad.append("enforce_roof_eligibility should never touch a "
                           "non-ROOF_LEVELS level ({0}, {1})".format(lvl, cls))
    # --- the cap wired end to end through solve() ---------------------------
    # three buildings 500 m apart -- far beyond every spread mechanism's
    # reach (attached 1.2 m, radiation 13 m, spot 55 m) -- so `edges()` finds
    # nothing between them and consumes NO rng draws; each is soloed as its
    # own origin so only ITS OWN age decides its level. `collapse_p=0.0,
    # burnt_out_p=1.0` makes the T_COLD urm branch of `level_for_age`
    # deterministically F6 for ANY rng (the F5c coin can never hit 0.0, the
    # F6 coin can never miss 1.0) -- this is what "an old-age skyscraper
    # gets F5, an old-age midrise F5c, an old-age brownstone F6" actually
    # means end to end: the RAW level is F6 for all three, and only the
    # height-class cap tells them apart.
    cap_bs = [{"x": 0.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
              "style": "brownstone"},
             {"x": 500.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
              "style": "midrise"},
             {"x": 1000.0, "y": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0, "H": 14.0,
              "style": "skyscraper"}]
    cap_cls = {"brownstone": HEIGHT_CLASS_LOW, "midrise": HEIGHT_CLASS_MIDHIGH,
              "skyscraper": HEIGHT_CLASS_SKYSCRAPER}
    for i, want_level, want_cls in ((0, "F6", HEIGHT_CLASS_LOW),
                                    (1, "F5c", HEIGHT_CLASS_MIDHIGH),
                                    (2, "F5", HEIGHT_CLASS_SKYSCRAPER)):
        solo = solve(cap_bs, i, 220 * 60, wind_dir=0.0, wind_mps=0.0,
                    rng=random.Random(1), collapse_p=0.0, burnt_out_p=1.0,
                    height_class_of=lambda b: cap_cls[b["style"]])
        if solo[i]["level"] != want_level:
            bad.append("solve() height-class cap: building {0} ({1}) at "
                       "T+220 min should be {2}, got {3}".format(
                           i, want_cls, want_level, solo[i]["level"]))
        if solo[i]["height_class"] != want_cls:
            bad.append("solve() did not record height_class {0} for "
                       "building {1}, got {2}".format(
                           want_cls, i, solo[i]["height_class"]))
    if verbose:
        print("[urban_fire_spread] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
        for ln in summarise(bs, pl, 120 * 60):
            print(ln)
    return bad
