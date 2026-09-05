"""quake_collapse — the EARTHQUAKE collapse family, built on the urban-fire
partial-collapse primitives.

WHY THIS MODULE EXISTS
----------------------
The user, looking at the first 500 m M7.8 city on screen (2026-08-30):

    "How does urban fire do partial collapse damage to modern city
     environment. I want you to use that, it looks more realistic."
    "lots of material mismatches. damaged part and undamaged part look like
     completely diff materials, etc. I think urban fire fixes this (don't use
     the scorching from urban fire) only use the partial collapse mechanism
     and expand to total collapse, etc"
    "the debris you are forming has to be more realistic than what it is rn."

`disaster/fire_collapse.py` (level F5c) is the mechanism that reads right.
Nothing about it is a fire mechanism except its palette:

  1. `plan_partial_collapse` is a PURE PLANNER — no `pxr`, no stage — so
     every decision is arithmetic on the element table and every invariant is
     host-testable. `r_partial_collapse` is only the USD authoring of a plan
     that has already been checked.
  2. The kill list is quantised to the KIT MODULE GRID and shaped by a
     STAIRCASE that widens upward (`PROFILE_FOOT`), never a rectangle and
     never one long diagonal.
  3. Every SURVIVING module that touches a dead one is torn on the touching
     edge (`plan_edges` -> `_tear_perimeter`), so the outline of the hole is
     made of Voronoi cell boundaries and not of module joints. That is the
     whole answer to "sharp straight or rectangular cuts".
  4. Fragments come out of `quake_flow._break(mode="uniform")` AFTER
     `fracture.solidify`, so ~82 % of a fragment's faces keep the module's
     OWN cladding (`_clad_material`, triplanar) and only the `core` face
     subset — the faces the fracture invented — takes an interior material.
     That is the whole answer to "damaged part and undamaged part look like
     completely different materials".
  5. Per-fragment outward throw (`THROW_BASE + THROW_TOP * z/H`) instead of a
     scene-wide bias, so a wall that let go lands as a fan in the street.

This module is that mechanism with the earthquake's own failure modes on top
of it and NONE of the fire's semantics: no soot, no scorch, no `ctx["fire"]`,
no `soot_plume`, no `_refire`, no `bind_break` char palette, no `HEAP_MIX`,
no `_a_dustify`. Fragments keep the module cladding and fallen pieces get
`quake_flow._dust_loose` — a dusted COPY of each fragment's OWN material,
texture kept, tint applied. One material family per building, dusty where it
broke.

WHAT IT DOES **NOT** REPLACE
----------------------------
Every recipe in `quake_flow.RECIPES` is untouched, and `quake_flow.LADDER` is
untouched, so `EQ_LADDER=legacy` reproduces today byte-for-byte. The new modes
are registered under their own names (`qc_*`) and selected by a SECOND ladder
table, `LADDER_QC`, composed in `quake_flow` from `LADDER` plus this module's
`LADDER_OVERRIDES` (so the DG0-DG2 rows — glass, scars, rooftop plant — are
literally the same objects the legacy table holds and cannot drift).

THE MODES
---------
| mode | what comes away | who uses it |
|---|---|---|
| `elevation` | one WEAK elevation (the most-glazed face, or a caller's) from a failure line up, staircased; URM peels from the parapet down, RC drops its infill from a storey up | urm DG4, rc DG3 (`infill=True`) |
| `corner` | the corner two elevations share, a bay and a half of each, from the failure line up | urm DG3 (drawn) |
| `soft_storey` | a crush band at storey k on ALL FOUR sides, bitten into the storeys above and below so the band is not a clean slice; the block above drops and leans/racks | rc DG4, rc_glass DG4 |
| `mid_storey` | the same band at k >= 1, block above twisted in plan and offset | rc DG4 (drawn) |
| `pancake` | every storey's shell from s0 up, progressively wider; the slabs re-authored as a stack at a pitch | rc DG5 |
| `total` | every shell module including the roof and the parapet ring; a standing stub on the blind / party-wall sides | urm DG5 |
| `auto` | corner or elevation, drawn from the building's own private rng | urm DG3 |

THE RNG CONTRACT — THIS FAMILY CONSUMES ZERO SHARED DRAWS
---------------------------------------------------------
`wreck_building` runs one ladder in order off ONE `random.Random`, so a
recipe that draws a different NUMBER of values than it did yesterday moves
every LATER recipe's outcome. `fire_collapse` records the same rule for
`urban_fire.LADDER`. So, exactly as there:

* the generators are `random.Random(seed)` and `numpy.random.default_rng(seed)`
  with `seed` derived from the BUILDING (`zlib.crc32` of tag/style/type/pose,
  never `hash()` — `hash()` of a `str` is salted per process and would make a
  plan irreproducible between two runs of the same scene);
* `fire_collapse._own_rng` INSTALLS both on the ctx for the duration of the
  authoring, because `quake_flow._rubble`, `_a_slab_rim`, `_ragged_slabs`,
  `_disturb_interior`, `_dust_loose` and `r_droop` all read `ctx["rng"]` /
  `ctx["nrng"]` off the ctx and take no generator argument;
* the shared pair is restored in `__exit__`.

`plan_collapse` never touches the ctx at all: it takes its own generator and
returns. `tests/test_quake_collapse.py` advances the shared rng 50 times
between two plans of the same building and asserts they are identical.

TESTED WITHOUT A STAGE
----------------------
`plan_collapse` imports no `pxr` (and this module imports none at module
scope — every USD import is inside an authoring function), so the whole plan
is checkable host-side on REAL kit buildings:
`scene_gen/tests/test_quake_collapse.py`.

WHAT THE TESTS CANNOT SEE: whether the fan actually lands where the plan says,
whether the block above a soft storey reads as crushed rather than lifted,
whether `_dust_loose`'s tint is deep enough at 40 m. That needs a render.
"""

import os
import random
import zlib


# ---------------------------------------------------------------------------
# Knobs
# ---------------------------------------------------------------------------
# The private seed. XOR'd into the building's own crc32 so this family's draws
# are stable per building and independent of the shared ladder rng.
QC_SEED_XOR = 0x9CE1

# Roles a collapse may take away. Same set `fire_collapse.SHELL_ROLES` holds —
# `balcony` included, because a cantilever off a wall that is no longer there
# has nothing to hang from.
SHELL_ROLES = ("wall", "corner", "parapet", "parapet_corner", "balcony")
# ...and, unlike the fire's, a TOTAL collapse takes the roof with it. The fire
# ladder excludes `roof` deliberately (a burnt-out shell keeps its skyline);
# a URM building that has come down has no roof left standing anywhere.
ROOF_ROLES = ("roof",)
ALL_ROLES = SHELL_ROLES + ROOF_ROLES

# ROOF PROPS THAT LOSE THEIR ROOF. `quake_flow.dress_roof` seats tanks and AC
# units on the "main" mass's roof before any recipe runs, and hands every one
# of them to physics UNCONDITIONALLY at the very end of `wreck_building`
# (`_b_settle_roof_plant`) — right for a roof that survived, wrong for one
# this family just took away. `elevation`/`corner` reach the parapet by
# construction (`_spans_elevation` always runs to `top_i`), and `total` /
# `pancake` take the whole shell including the roof: those three (plus
# `corner`) are handled HERE, by `_sweep_roof_props`.
#
# ROUND 5 tried a velocity kick + the generic settle and it was NOT enough:
# "a tank with nothing under it has to fall a whole storey height on gravity
# alone inside one settle's step budget, and a body that does not make it
# down in time bakes frozen mid-air" — exactly what round 6 found baked,
# active and bolt upright, on `bld_brownstone_row_DG3.usd` (fell ~1 m of a
# required ~3 m and froze) and on `bld_office_wide_DG4.usd` (never moved by
# any recipe at all — see below — then only partly dropped by the same
# generic settle). The manifest's `still_moving: 0` said "converged" for
# both: that stat only measures whether a body moved >4 mm in the LAST
# settle chunk, not whether it landed on anything. `_sweep_roof_props` now
# resolves the landing GEOMETRICALLY — a points/triangle support probe under
# the prop's own footprint (`_deck_support_z`, the `tri_soup` idiom from
# `tools/fc_roof_deck_probe.py`) — and places it there directly, the same
# "pure geometry, no physics dependency" contract the `total`/`pancake` bury
# path already had.
#
# `soft_storey` / `mid_storey` are NOT here: `_author_band` carries the roof
# down WITH the block above when the band reaches that high, and — as of
# this round, for real — the roof plant with it (`ctx["roof_plant"]` /
# `ctx["roof_fixed"]` are now in `_author_band`'s own "above" list; they used
# to be in neither that list nor `_els`, which is what let a soft-storey
# crush move the roof out from under a tank that never moved at all —
# measured on `bld_office_wide_DG4.usd` above). A prop this family's OTHER
# recipes already carry is not this family's plan to re-derive twice.
ROOF_PROP_MODES = ("elevation", "corner", "total", "pancake")
# A resolved drop deeper than this share of the mass's own TOP-STOREY height
# does not get to land bolt upright: `quake_flow._a_bury_props`'s tip/roll
# (a random near-horizontal axis, 20-75 deg) is reused verbatim — the same
# dressing DG5 already trusts for "the contents of a collapsed building are
# under the rubble," not a new one invented for this path. Shallower than
# that, the small idle tip `quake_flow._b_settle_roof_plant` already gives an
# untouched prop (`quake_flow.B_ROOF_PLANT_TIP_DEG`) is enough — it barely
# fell.
ROOF_PROP_BIG_TIP_STOREY_FRAC = 0.5
# The support probe's own slop, in metres: a candidate deck triangle up to
# this far ABOVE a prop's current resting height still counts (authored
# geometry interpenetrates by a few cm all over this family; see
# `_deck_support_z`).
ROOF_PROP_SUPPORT_MARGIN_M = 0.5

# THE STAIRCASE, `fire_collapse`'s own: the share of the region's full width
# lost at the FAILURE LINE, ramping to 1.0 at the top of the mass. 0.55 gives
# roughly two module steps over a five-storey band. 1.0 is a rectangular hole,
# which is the artefact the profile exists to avoid.
PROFILE_FOOT = 0.55
# Share of the elevation lost at the TOP, drawn per building. Never all of it:
# a collapse that leaves no stub of wall standing at either end reads as a
# demolition, and the stub is what tells a viewer how tall the wall was.
SPAN_FRAC = (0.62, 0.86)
# Corner reach, per adjoining side, measured FROM the corner in MODULES — not
# as a share of the side. `fire_collapse` records the measurement: 0.45 of
# `block_residential`'s 88 m south elevation is FORTY-ONE METRES, which is
# half the building. `quake_flow.r_corner_fail` writes the same reach as
# `max(4, module) * 1.6` for the same reason.
CORNER_REACH_MODULES = (1.3, 2.3)
CORNER_REACH_MAX_FRAC = 0.40

# HOW TALL THE LOST STRIP MAY BE on the elevation/corner modes. A 60 m strip
# of elevation on the ground is a bombing. The band is capped from the TOP
# DOWN and the failure line rises to meet it.
MAX_FALL_STOREYS = 6
# A LAST-RESORT BUDGET on how many kit modules may be fractured. Each one is
# a `_break` into 8-13 pieces plus a solidify, so this is the recipe's whole
# cost. Over budget the failure line is raised a storey at a time (the notch
# gets shallower, not narrower, so the staircase profile survives) — and if
# it still does not fit the overrun is REPORTED, never silently dropped.
MAX_MODULES = 130
# ...and the same shape for the edge tears (each is a `_break_split`).
MAX_EDGE_MODULES = 200

# ---------------------------------------------------------------------------
# THE RING (round 7). "The break-aways should be like the urban fire's."
#
# MEASURED, not guessed (`tools/qc_edge_probe.py`, 2026-09-01, the archetype
# bake's own pose and seed): `fire_collapse.plan_edges` -> `_tear_perimeter`
# tears 100 % of what it is given — `apartment_long` DG4 15/15, `office_plain`
# DG4 55/55, `commercial_mid` DG4 10/10, zero failures, zero straight seams on
# the ORTHOGONAL perimeter. The machinery is not broken; the RING IS TOO
# SMALL, and it is too small for a reason that is specific to the quake ladder
# and does not apply to the fire's:
#
#   * a URM elevation PEELS FROM THE PARAPET DOWN (`URM_TOP_DOWN`), so the
#     notch is 2-3 storeys of one wall. `fire_collapse`'s F5c hole starts at
#     the fire's origin and is FIVE storeys of nearly the whole elevation
#     (measured on the same building: fire kill 37 + 23 torn = 60 modules,
#     34 % of the shell; quake kill 18 + 15 torn = 33, 19 %);
#   * `plan_edges` tears only what is ORTHOGONALLY adjacent to a dead module —
#     right, and enough when the hole is that big, because there is barely a
#     pristine module left near it. Round a 2-storey notch it leaves a wall of
#     factory rectangles ONE BAY OUT, a square re-entrant corner at every
#     staircase step, and untouched corner columns at the ends of the run;
#   * and a burnt building's boundary is dissolved by things a shaken one has
#     none of — soot, `_drop_face_art`, blown-out openings. A quake wall keeps
#     its clean kit face, so every seam the fire hides is visible here.
#
# So this family adds a SECOND RING of its own on top of `plan_edges`'s jobs
# (never instead of them, and never by editing `fire_collapse`, which is
# frozen). Same job schema, same judges, same `_tear_perimeter`; only the
# penetrations are smaller, because a second-ring module is cracked and
# spalled, not peeled.
#
# The share of a module's own width/height a second-bay bite takes. FEMA 306's
# crack at a wall return, not a peel: a tenth to a quarter.
RING_PEN2 = (0.10, 0.24)
# The RE-ENTRANT CORNER of a staircase step — the module diagonally across
# from the hole, which `plan_edges` cannot see (it overlaps the dead run in
# NEITHER t nor z) and which is the sharpest right angle in the whole notch.
# Bitten on both of its two exposed edges at once.
RING_DIAG_PEN = (0.18, 0.38)
# A CORNER COLUMN standing proud of a wall that has gone. `corner` /
# `parapet_corner` pieces are 1.2-1.8 m square and are assigned to whichever
# of their two walls `quake_flow._side_of` found nearest, so a corner at the
# end of a lost run is routinely on the side that did NOT fail and escapes
# every class `plan_edges` has. This is the "one wall extending further than
# the rest" of the 2026-08-31 review.
RING_CORNER_PEN = (0.20, 0.45)
RING_CORNER_REACH_MODULES = 1.25
# THE TOP COURSE OFF THE ROOFLINE, on the elevations the collapse did NOT
# take. `r_parapet_fall` breaks a parapet band with `partial=None` — the band
# goes entirely and the top-storey wall under it keeps a machined top edge, so
# a building that lost one elevation still reads as "a bunch of perfect
# rectangular wall pieces all intact" along the whole of the other three. Its
# own `partial=True` branch (the parapet-less styles) already authors exactly
# this look; this extends it to the styles that HAVE a band. Per module, not
# a uniform band — a uniform take is just a lower machined line.
RING_TOP_PEN = (0.10, 0.24)
RING_TOP_P = 0.55
# THE BLOCK ABOVE A CRUSHED STOREY IS NOT PRISTINE. `_author_band` moves it as
# ONE RIGID BODY — which is right, it is what a soft storey does — so on the
# rc DG4 rung every module above the band keeps its factory face and the whole
# tower reads undamaged above the gap. Measured on the baked
# `bld_office_plain_DG4`: 125 of its 128 meshes carry one identical 2.36 deg
# tilt and only 3 are real debris. The record does not say the block is
# undamaged: an infilled frame that has drifted enough to lose a storey shows
# corner crushing and diagonal cracking in the panels above it (FEMA 306's
# infill damage states; the Northridge and Kahramanmaraş soft-storey
# photographs). So a light share of that block spalls at ONE edge — a chip,
# never a peel, and never a horizontal cut (a panel with its top taken would
# drop the band above it).
RING_SPALL_P = 0.22
RING_SPALL_PEN = (0.08, 0.18)
# The ring's own cost ceiling. Each job is a `_break_split` (8-11 cells plus a
# solidify), so this is what the look costs in bake time; `QC_RING_MAX=0`
# turns the whole second ring off and gives back the round-6 look exactly.
RING_MAX = 90
# A band / total / pancake mode sweeps whole storeys and cannot raise a
# failure line to get under a budget: a total collapse that left half the
# building standing to save fracture time is not a total collapse. So for
# those modes this is a COST WARNING, not a limit — `plan["budget"]
# ["over_modules"]` reports the overrun and the note says so, and nothing is
# ever dropped. (`r_masonry_collapse` / `r_pancake` have no cap at all today,
# so an overrun here is a report of what the legacy recipes already spend.)
#
# MEASURED, host-side, at the 420 line: `block_stone` total 792 modules over
# 3 masses, `highrise_step` 350 over 2, `block_residential` 2,558 over 5 —
# and that last one is 1,194 on `wing3` alone, which is the number worth
# seeing in a bake log before wondering why one building took ten minutes.
MAX_MODULES_TOTAL = 420

# WHERE THE FAILURE LINE STARTS, per construction type. A URM elevation peels
# from the PARAPET DOWN — the top one or two storeys are the unrestrained
# ones (no diaphragm over them), which is why the reconnaissance photographs
# of out-of-plane failure are nearly all top-storey. An RC frame does not
# peel: it loses its INFILL, which starts at the first floor (the ground
# storey is the shopfront and is glass, not masonry).
URM_TOP_DOWN = (1, 2)
RC_FROM_STOREY = 1
# The corner mode reaches further down a frame than a masonry parapet: on RC
# the corner COLUMN zone is what failed, and it fails over several storeys.
CORNER_STOREYS = {"urm": (2, 3), "rc": (3, 4), "rc_glass": (2, 3)}

# THE BAND IS NOT A CLEAN SLICE. A crushed storey whose kill list is exactly
# `storey == k` leaves a dead-level line top and bottom, right on two slab
# lines — the "building lifted off with a pallet fork" artefact the first
# mid-storey bench showed and the one the round-4 review photographed again
# (`r4_commercial/4_commercial_soft_storey_ne.png`). So the band BITES into
# the storey above (the bottom of the block that came down with it) and, at a
# mid storey, into the one below.
BAND_BITE_UP = (0.14, 0.34)
BAND_BITE_DOWN = (0.08, 0.20)

# PANCAKE. The lowest storeys keep a run at each end (the frame did not go
# all at once); everything from `PANCAKE_RAMP_STOREYS` up goes whole.
# 0.62, MEASURED, not 0.74. A kit piece is pivoted at one END and the kill
# window carries half a module of tolerance either side, so on a 20 m wall a
# 0.74 run leaves 2.6 m at each end — less than one 4-5 m module, and NO
# module survives the bottom storey at all: the ramp is invisible and the
# whole pancake reads as a clean vertical slice again. 0.62 leaves 3.8 m,
# which is one module per end per side, which is what `plan_edges` then tears.
PANCAKE_FOOT = 0.62
PANCAKE_RAMP_STOREYS = 2
# `r_pancake`'s own stacking numbers, reused verbatim so the two look alike.
PANCAKE_PITCH = (0.55, 0.95)
PANCAKE_TILT = (-4.0, 4.0)
PANCAKE_JITTER = 0.6
PANCAKE_ROOF_TILT = (3.0, 9.0)

# THE STANDING STUB after a total collapse. Research (`eq_round4_rubble_
# research.md` sec5a, and the `STUB_KEEP_P` row of the skill's constants
# table): ~40-60 % of totally-collapsed buildings keep a stub, and where one
# survives it is the party wall / blind side — the wall that was braced by
# its neighbour. `r_masonry_collapse` keeps one on EVERY side, which is why
# its DG5 reads as a bathtub; here it is the blind sides only.
STUB_FRAC = (0.25, 0.62)
BLIND_GAP_M = 2.5
STUB_KEEP_P = 0.5
STUB_SIDES_FALLBACK = (0, 2)

# THROW. A wall rotating about its foot: the top leads (`r_out_of_plane`'s own
# 0.4 + 2.4 z/H profile, which the bench's `max_speed=6.0` never saturates).
THROW_BASE = 0.40
THROW_TOP = 2.40
# A TOTAL collapse drops nearly straight down — the outward fan belongs to a
# peel, not to a building falling into its own footprint (`r_masonry_collapse`
# makes the same correction for the same reason).
TOTAL_THROW_BASE = 0.20
TOTAL_THROW_TOP = 0.90
# A CRUSHED BAND IS NOT THROWN AT ALL. It is squeezed out sideways at walking
# pace under the mass above it; `r_soft_storey` uses 0.3-1.2 m/s.
BAND_THROW = (0.3, 1.2)
BAND_PUSH_M = (0.8, 2.2)

# FRACTURE. `mode="uniform"` and NOT `quake_flow._p_frac_kw` — that is the
# whole point of the round-5 rebuild. `_p_frac_kw` seeds the fracture on the
# masonry BOND (`mode="brick"`) or on a prism lattice (`mode="prism"`), and a
# lattice makes cuboids: "clean toy-brick prisms" is exactly what the user
# photographed in `r4_commercial/0_commercial_DG5_close.png`. The fire path
# takes uniform Voronoi cells and its rubble reads as rubble.
BREAK_PIECES = (8, 13)
BREAK_CONSUME = 0.26
BREAK_ROUGH = 0.014
# `_break` overrides this to 0.18 whenever the piece has a readable texture
# AND a core subset (see its own note) — which is the rule that keeps the
# cladding on ~82 % of the faces. It is passed here at the fire's own value so
# the fallback (a piece with no texture at all) matches the fire path too.
INNER_P = 0.4
# A total collapse sheds more, smaller, and thins harder: what lands on the
# CROWN of the pile is always on top, so the only lever that works is how many
# there are (`r_masonry_collapse`'s own note, and its own numbers).
TOTAL_PIECES = (18, 24)
TOTAL_CONSUME = 0.80
TOTAL_MAX_PIECE_M = 0.9
PANCAKE_PIECES = (15, 21)
PANCAKE_CONSUME = 0.55
PANCAKE_MAX_PIECE_M = 1.2
BAND_PIECES = (8, 12)
BAND_CONSUME = 0.45

# THE PILES. Every one of them goes through `quake_flow._rubble`, which is
# `EQ_RUBBLE`-gated (v2 = the round-4 mound/large-elements/instancer planner,
# v1 = the old `_heap` box crate). Nothing here ever calls `_heap` directly.
FAN_DEPTH_M = (1.0, 1.9)          # r_out_of_plane's own figures
FAN_SPREAD = (0.22, 0.40)
INFILL_DEPTH_M = (0.55, 1.00)     # dropped infill panels, not a fallen wall
CORNER_DEPTH_M = 0.8              # r_corner_fail's v2 value
CORNER_ALONG_PAD_M = 2.0
COLLAR_DEPTH_M = (0.5, 0.9)       # r_soft_storey's collar
DOME_CROWN_FRAC = 0.28            # FEMA's 0.33 air-space factor, as used today
DOME_SPREAD = (0.20, 0.34)
PANCAKE_CROWN_FRAC = 0.85
PANCAKE_SPREAD = (0.30, 0.45)
PANCAKE_LARGE_BUDGET = 4          # the stack carries what a pancake pile shows

# How deep into the plan the "lost region" reaches, as a share of the mass's
# other dimension. This is what restricts the position sweep on the
# elevation/corner modes: a static prim goes to the solver only when it is
# above the failure line AND inside this strip. Without it the sweep drops the
# far half of the roof deck and the plant on it.
REGION_DEPTH_FRAC = 0.26
REGION_DEPTH_MIN_M = 2.6

# THE TEETH AT THE BREAK. Bearing stubs still in the wall pocket where a floor
# framed into the lost wall. Steel on a frame, timber on masonry — the fire
# path authors these as `steel` posts unconditionally, which is right for a
# burnt shell and wrong for a collapsed brick block.
TEETH_N = (2, 5)
TEETH_LEN_M = (0.5, 1.8)
TEETH_SIZE_M = (0.16, 0.24)

# Which kit names read as an OPENING, for `weak_side`. A pattern match, not a
# measurement: `urban_building` carries no "has an opening" flag on a piece
# (that is a `quake_sliced` / GAC-slice concept), and `quake_flow.
# _pick_opening_panels` makes the same call the same way.
OPENING_PAT = ("facade", "window", "shop", "store", "door", "arch", "glass",
               "display", "entry")


def _f(name, default):
    """An override from the environment, for a bench run that wants to push
    one number without a code change."""
    v = (os.environ.get(name) or "").strip()
    try:
        return float(v) if v else float(default)
    except ValueError:
        return float(default)


MODES = ("elevation", "corner", "soft_storey", "mid_storey", "pancake",
         "total")


# ---------------------------------------------------------------------------
# The private generator
# ---------------------------------------------------------------------------
def plan_seed(ctx, mode, seed=None, mass=None):
    """The seed every draw in a plan comes from.

    `zlib.crc32` OF THE BUILDING, never `hash()`. `fire_collapse._private_rng`
    falls back to `abs(hash(key))` when `soot_plume` is missing, and `hash()`
    of a `str` is salted per PROCESS (`PYTHONHASHSEED`) — a plan drawn from it
    is not reproducible between two runs of the same scene, which is exactly
    what a baked archetype library needs it to be. crc32 is stable, cheap and
    has no import cost.

    The REQUESTED mode is part of the key, so `qc_elevation` and `qc_corner`
    on one building are different draws; the pose is part of it, so two
    buildings of one style in one city are different draws even if a caller
    hands them the same tag; and the MASS is part of it, so the four wings a
    total collapse brings down do not all draw the same stub sides and the
    same pile.
    """
    if seed is not None:
        return int(seed) & 0xFFFFFFFF
    info = ctx.get("info") or {}
    key = "|".join(str(q) for q in (
        ctx.get("tag", ""), info.get("style"), info.get("type"),
        info.get("x"), info.get("y"), info.get("yaw"), mode,
        mass or "main"))
    return (zlib.crc32(key.encode("utf-8")) ^ QC_SEED_XOR) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Pure geometry helpers — no pxr, no stage
#
# The frame arithmetic (`side_length`, `outward_of`, `along_of`,
# `wall_point`, `el_span`, `el_z_span`, `el_footprint`, `in_region`,
# `region_side`, `corner_sides`, `corner_at_high_end`, `corner_of_end`,
# `other_side`, `shared_corner`) is `fire_collapse`'s and is IMPORTED, not
# copied: it is the same mass frame, and a second copy of `outward_of` with a
# sign flipped is the class of bug that puts a windrow inside a building.
# ---------------------------------------------------------------------------
def _fc():
    from . import fire_collapse as fc
    return fc


def _qf():
    from . import quake_flow as qf
    return qf


def blind_sides(ctx, given=None, prng=None):
    """The sides a collapsed building may keep a standing stub on.

    A stub survives where the wall was BRACED — the party wall of a terrace,
    the blind flank against the next block. `quake_flow.d_set_neighbours` /
    `ctx["neighbours"]` is where the assembly records that, so a neighbour
    closer than `BLIND_GAP_M` makes its side blind.

    With no neighbour registry at all (a bench building standing alone, a
    monolith the assembly never paired) there is no ground truth, so the
    research's own `STUB_KEEP_P` is used as a DRAW instead: about half of
    total collapses keep a stub, and when one is kept it is on one or two
    sides. `prng` is required for that branch; without one the answer is
    simply "no blind side", which keeps the function pure for a caller that
    only wants the registry's answer.
    """
    if given is not None:
        return tuple(s for s in ("S", "E", "N", "W") if s in tuple(given))
    qf = _qf()
    nbs = qf._d_nbs(ctx) if ctx.get("info") is not None else []
    out = sorted(set(n["side"] for n in (nbs or ())
                     if float(n.get("gap", 1e9)) <= BLIND_GAP_M))
    if out:
        return tuple(s for s in ("S", "E", "N", "W") if s in out)
    if prng is None:
        return ()
    if prng.random() >= STUB_KEEP_P:
        return ()
    n = prng.randint(*STUB_SIDES_FALLBACK)
    pool = ["S", "E", "N", "W"]
    prng.shuffle(pool)
    return tuple(s for s in ("S", "E", "N", "W") if s in pool[:max(1, n)])


def weak_side(ctx, mtag, prng, storeys=None):
    """The elevation a quake takes first: the one with the most OPENINGS.

    A wall's out-of-plane capacity is what is left of it after the windows are
    cut out, and the storefront elevation — a shopfront under four storeys of
    masonry — is the classic URM street-wall failure. There is no per-piece
    opening flag on this kit, so this is a name-pattern share (the same call
    `quake_flow._pick_opening_panels` makes), with a small bonus for the side
    that simply has more wall on it and a front bias to break a tie: `S` is
    the street elevation in `urban_building`'s own frame, and the bench
    cameras aim at it.

    Deterministic: the same building and the same generator give the same
    answer, and the number of draws taken depends only on the building.
    """
    qf = _qf()
    score = {}
    counts = {}
    for sd in ("S", "E", "N", "W"):
        els = [e for e in qf._els(ctx, mass=mtag, role=("wall", "corner"))
               if e["side"] == sd
               and (storeys is None or int(e["storey"]) in storeys)]
        counts[sd] = len(els)
        if not els:
            score[sd] = -1.0
            continue
        n_open = sum(1 for e in els
                     if any(k in (e.get("name") or "").lower()
                            for k in OPENING_PAT))
        score[sd] = n_open / float(len(els)) + 0.002 * len(els)
    best = max(score.values())
    tied = [sd for sd in ("S", "E", "N", "W") if score[sd] >= best - 1e-9]
    if len(tied) == 1:
        return tied[0]
    if "S" in tied and prng.random() < 0.55:
        return "S"
    return tied[prng.randrange(len(tied))]


def collapse_masses(info, mode, mass=None):
    """Which masses ONE call brings down.

    A TOTAL collapse and a PANCAKE take the whole building — `r_masonry_
    collapse` iterates `info["masses"].items()` and `r_pancake` iterates
    `[t for t in masses if t == mass or t.startswith(mass) or mass ==
    "main"]`, both for the same reason: `urban_building` builds a block as a
    main mass plus wings, and on the measured stock that is most of the tall
    stock — `block_residential` is 5 masses and 2,377 elements with only 181
    of them on `main`, `block_stone` is a 1-level base under two 8-storey
    wings. A "total collapse" that swept `main` alone would leave 584 modules
    of eight-storey wing standing on a heap.

    A PARTIAL mode (elevation, corner) and a BAND mode are single-mass by
    definition: one elevation of one mass peels, one storey of one mass
    crushes, and `_author_band` already carries every OTHER mass down with the
    block above (`r_soft_storey`'s own `mass == "main" and e["mass"] !=
    "main"` rule). An explicit `mass=` from the caller always wins.
    """
    mtag = mass or "main"
    if mtag not in info["masses"]:
        mtag = "main"
    if mode not in ("total", "pancake") or mass is not None:
        return [mtag]
    return [t for t in info["masses"]
            if t == mtag or t.startswith(mtag) or mtag == "main"]


def _spans_elevation(m, sides, s0, top_i, n_lv, base, centres, corner=None,
                     reach_m=None, foot=None):
    """The staircase, in metres along each wall from its low end.

    `fire_collapse.plan_partial_collapse`'s own profile: the width grows from
    `foot` of the full run at the failure line to all of it at the top, so
    what is left standing is a NOTCH on the module grid with a ragged edge.
    Anchored at the corner in corner mode, centred (wandering a little off
    centre) otherwise.
    """
    fc = _fc()
    # RESOLVED AT CALL TIME, not as a default argument: `foot=PROFILE_FOOT` in
    # the signature binds the module constant once, at import, so a bench (or
    # a test that flattens the profile to prove the staircase is real) could
    # set `quake_collapse.PROFILE_FOOT` and change nothing at all.
    foot = PROFILE_FOOT if foot is None else float(foot)
    out = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        at_hi = fc.corner_at_high_end(sd, corner) if corner else None
        for s in range(s0, n_lv):
            k = (0.0 if top_i == s0
                 else (s - s0) / float(top_i - s0))
            ramp = foot + (1.0 - foot) * k
            if corner:
                w = min(reach_m * ramp, CORNER_REACH_MAX_FRAC * L)
                t0, t1 = ((L - w, L) if at_hi else (0.0, w))
            else:
                w = min(L, base * ramp * L)
                c = centres[sd]
                t0 = max(0.0, min(c - w / 2.0, L - w))
                t1 = min(L, t0 + w)
            out[(sd, s)] = (float(t0), float(t1))
    return out


def _kill_from_spans(ctx, m, mtag, spans, roles, pad, skip=()):
    """Every element of `mtag` whose pivot falls inside its own (side, storey)
    span. `skip` is a set of `id(element)` the caller has already claimed (the
    stub jobs), so one prim is never handed to two fracture calls —
    `_break_split` deactivates its source and the second call would fracture
    a dead prim.
    """
    qf = _qf()
    out = []
    for e in qf._els(ctx, mass=mtag, role=roles):
        if id(e) in skip:
            continue
        iv = spans.get((e["side"], int(e["storey"])))
        if not iv:
            continue
        lo, hi = iv
        t = qf._p_el_t(m, e["side"], e)
        if lo - pad <= t <= hi + pad:
            out.append(e)
    return out


def _heap_centres(m, side, along_m, out_m):
    """`{"local", "world", "outward_m"}` for one side of a pile, `out_m`
    metres OUTSIDE the wall line at the middle of its run.

    Every windrow/fan `quake_flow._rubble` authors is placed OUTWARD from the
    wall line by construction (`quake_rubble._side_axes` offsets by
    `+halfD + offset_m` and reaches further out still; v1's `_heap` draws its
    windrow distance positive). So a plan whose centre comes out NEGATIVE
    here has put the rubble inside the building, and the test asserts on
    exactly this number at four yaws.
    """
    fc, qf = _fc(), _qf()
    lo, hi = along_m
    lx, ly = fc.wall_point(m, side, 0.5 * (lo + hi), out_m=float(out_m))
    wx, wy = qf._to_world(m, lx, ly)
    return {"local": (lx, ly), "world": (wx, wy),
            "outward_m": fc.outward_of(m, side, lx, ly)}


# ---------------------------------------------------------------------------
# THE PLAN — every decision, host-side
# ---------------------------------------------------------------------------
def plan_collapse(ctx, mode="elevation", mass=None, side=None, corner=None,
                  storey=None, from_storey=None, span_frac=None, depth_m=None,
                  drop_slabs=1, max_storeys=None, max_modules=None,
                  max_edges=None, blind=None, infill=False, lean_side=None,
                  lean_deg=None, crush_m=None, twist_deg=None, offset_m=None,
                  pitch_m=None, stack_base_z=None, pile_base_z=None,
                  keep_stub=True, seed=None):
    """Decide what comes down, what tears, what falls and where it lands.
    NO USD, NO STAGE, NO SHARED DRAWS.

    Returns a dict. Every key is present in every mode; the ones that do not
    apply are empty (`{}` / `[]` / `None`), never absent, so a caller never
    has to branch on the mode to read the plan.

      mode          the RESOLVED mode: one of `MODES`. `"auto"` resolves to
                    `elevation` / `corner`; `soft_storey` with `storey=None`
                    resolves to `mid_storey` when the draw picks k >= 1.
      requested     what the caller asked for, before the resolution above.
      mass          mass tag the failure is in.
      btype         `urm` / `rc` / `rc_glass` — which recipe shape was used.
      sides         the elevation(s) that lose modules. One for `elevation`,
                    the corner's two for `corner`, all four for the band /
                    total / pancake modes.
      keep_sides    the elevations no module is taken off. Empty for the band
                    and total modes; they are still TORN at the band edges,
                    which is `tears`' business, not `kill`'s.
      corner        "SE" etc, or None.
      s0            the failure line: the lowest storey that comes down.
      storeys       every storey a span was drawn for, ascending.
      top_storey    the top of the mass (the staircase's own top).
      n_levels      len(mass["levels"]).
      band          [k] for `soft_storey` / `mid_storey`, else [].
      span          {(side, storey): (t0_m, t1_m)} — the staircase, in metres
                    along that wall from its LOW end.
      roles         the element roles this mode sweeps.
      kill          the element records that come away. NOT marked dead here;
                    `r_collapse` marks them as it breaks them.
      stub_jobs     [{el, side, storey, z_line, frac}] — the storey-0 wall
                    modules on a blind side that are TORN at `z_line` and left
                    standing as a stub instead of being killed.
      stub          {side: mean stub height, m} for the sides in `stub_jobs`.
      panels        element records kept WHOLE so the pile can lay them
                    half-buried on it (round-4 design table row 2: "1-3 wall
                    PANELS ... the building's own kit modules kept whole").
                    Never in `kill`, never in `tears`, and `EQ_RUBBLE=v2`
                    only — v1's `_heap` has no `panels` concept, and a module
                    kept whole for a pile that will never lay it is a wall
                    left standing in the middle of a total collapse.
      blind_sides   the sides `stub` was allowed to use (see `blind_sides`).
      tears         `fire_collapse.plan_edges` jobs: every SURVIVING module
                    that touches a dead one, with the class of its own edge
                    the hole is on (above/below/left/right/return) and the
                    numeric break line already drawn. This is what makes the
                    outline of the hole cell boundaries and not module joints.
      region        {side: (t0_m, t1_m, depth_m)} — the plan footprint of the
                    loss, used to restrict the position sweep.
      sweep         True when `r_collapse` should run that position sweep
                    (elevation / corner). The band / total / pancake modes
                    move what they move explicitly and must not sweep.
      heaps         [{kind, where, sides, along, along_m, depth_m, offset_m,
                     crown_m, spread_frac, elem_h_m, stub_h_m, base_z, budget,
                     tag, centres}] — one `quake_flow._rubble` call each.
                    `kind` is "dome" | "windrow" | "fan"; `centres` is
                    {side: {"local", "world", "outward_m"}} and is empty for a
                    dome (whose centre is the mass's own).
      drop          [(mass, storey)] — floor SLABS that go to the solver
                    WHOLE. `elevation` only: a slab that lost its bearing wall
                    on one side comes down as one plate. A pancake's slabs are
                    re-authored as `stack` and a total's are broken into
                    boards, so both leave this empty — a slab that is both
                    dropped and re-authored is one prim in two places.
      break_slabs   [(mass, storey)] — floor slabs broken into pieces instead
                    (`total`: a joisted timber floor comes apart into boards
                    and joists, and is then buried).
      fit_fall      [(mass, storey)] — storeys whose fit-out (partitions,
                    columns, and props unless `bury_props`) comes down
                    wherever it stands on that storey.
      fit_region_storeys
                    [storey] — storeys where the fit-out comes down only where
                    its FOOTPRINT reaches into `region` (the partial modes: a
                    partition behind the part of the wall that still stands is
                    in a room that still has its walls).
      bury_props    True when the building's contents end up UNDER the pile
                    (`quake_flow._a_bury_props`) rather than falling as loose
                    bodies — a total collapse and a pancake; an intact desk on
                    the crown of a DG5 heap is a round-2 review finding.
      teeth         True when bearing stubs are authored at the break. False
                    where the floor they would stand on is itself coming down.
      cut_z         world z above which a static prim inside `region` falls.
      crush         None, or the band mode's own geometry:
                    {storey, z_lo, z_hi, h_st, lean_side, geo, crush_m,
                     twist_deg, offset_m} where `geo` is EXACTLY what
                    `quake_flow._soft_storey_geometry` returned — the block
                    above is moved by `quake_flow._soft_storey_matrix(geo)`
                    and nothing here re-derives it.
      stack         None, or the pancake's slab stack:
                    {pitch_m, base_z, top_z, plates: [{storey, z, tilt_deg,
                     axis, jitter, sides, top}]}.
      roof          "kill" | "stack" | "ragged" — what happens to the roof.
      floor_ragged  True on `elevation`/`corner` (infill or not): the exposed
                    floor/ceiling slab along the failed side is broken by
                    `quake_flow._ragged_slabs` rather than left a machined
                    rectangle. False elsewhere (`crush` handles its own two
                    slabs with `_a_slab_rim`; `total`/`pancake` break every
                    slab into boards/a stack). `_author_one` reads this
                    field rather than re-deriving `mode`/`infill` itself, so
                    the plan is the one place this decision is made.
      throw         (base, top) m/s for the per-fragment outward velocity.
      budget        {"modules", "edges", "trimmed_storeys", "over_modules",
                     "over_edges"} — what was spent and what did not fit.
      masses        [mass tag] — the masses this PLAN covers, always one.
                    `r_collapse` overwrites it on the first plan with the full
                    sweep (`collapse_masses`) and puts the other masses' plans
                    in `extra`, because a total collapse and a pancake take
                    the wings too.
      extra         [] on a plan; the other masses' plans on what `r_collapse`
                    returns.
      seed          the private seed everything above was drawn from.
      pad_m         half a module of tolerance on a span end (a kit piece is
                    pivoted at one end, so a piece whose pivot sits just
                    outside the span can still have most of its body inside).
      reaches_end   {side: (at_low_end, at_high_end)} at the top storey.
      infill        True when this is the RC infill variant of `elevation`.
      roof_prop_fall
                    [] on a bare plan (no stage, so no roof prop is known
                    yet) — `r_collapse` fills it with the `ctx["roof_plant"]`
                    / `ctx["roof_fixed"]` paths whose footprint MAJORITY sits
                    over roof area this plan just killed (`roof_prop_
                    footprint_lost`), never absent so a caller can always
                    read it. Only `elevation` / `corner` / `total` /
                    `pancake` ever populate it (`ROOF_PROP_MODES`); those
                    paths are sent to `ctx["loose"]` with a downward/outward
                    kick (partial modes) or buried with the fit-out's own
                    props sweep (`total` / `pancake`), and removed from
                    `ctx["roof_plant"]` so `quake_flow._b_settle_roof_plant`
                    never re-tips or re-drops the same prim a second time.
    """
    fc = _fc()

    requested = mode
    info = ctx["info"]
    mtag = mass or "main"
    if mtag not in info["masses"]:
        mtag = "main"
    m = info["masses"][mtag]
    btype = info.get("type") or "urm"
    n_lv = max(1, len(m["levels"]))
    top_i = n_lv - 1
    seed = plan_seed(ctx, requested, seed, mass=mtag)
    prng = random.Random(seed)
    module_m = max(4.0, float(m.get("module") or 4.0))
    pad = 0.5 * module_m

    # ---- 0. resolve the mode --------------------------------------------
    if mode == "auto":
        # urm DG3: a corner or an elevation, drawn from the building's own
        # generator so the ladder's sequence is untouched either way.
        mode = "corner" if prng.random() < 0.55 else "elevation"
    if mode == "soft_storey" and storey is None:
        # rc DG4: `r_storey_collapse`'s own 2-in-3 split between a ground
        # soft storey (Northridge / Antakya) and a mid-storey crush (Kobe /
        # Mexico City), drawn privately.
        if n_lv >= 4 and prng.random() >= 0.66:
            mode = "mid_storey"
    if mode not in MODES:
        raise ValueError("quake_collapse: unknown mode " + repr(requested))

    plan = {"mode": mode, "requested": requested, "mass": mtag,
            "btype": btype, "seed": seed, "n_levels": n_lv,
            "top_storey": top_i, "pad_m": pad, "infill": bool(infill),
            "band": [], "stub_jobs": [], "stub": {}, "panels": [],
            "drop": [],
            "break_slabs": [], "fit_fall": [], "fit_region_storeys": [],
            "bury_props": False, "crush": None, "stack": None, "heaps": [],
            "region": {}, "sweep": False, "roof": "ragged", "teeth": False,
            "blind_sides": (), "extra": [], "roof_prop_fall": [],
            "floor_ragged": False,
            "throw": (THROW_BASE, THROW_TOP)}
    plan["masses"] = [mtag]

    # ---- 1. which elevations, which storeys, what dies -------------------
    if mode in ("elevation", "corner"):
        _plan_partial(ctx, plan, m, prng, mode, side, corner, from_storey,
                      span_frac, max_storeys, max_modules, drop_slabs,
                      infill, module_m)
    elif mode in ("soft_storey", "mid_storey"):
        _plan_band(ctx, plan, m, prng, mode, storey, lean_side, lean_deg,
                   crush_m, twist_deg, offset_m)
    elif mode == "pancake":
        _plan_pancake(ctx, plan, m, prng, from_storey, pitch_m, blind,
                      keep_stub, stack_base_z)
    else:
        _plan_total(ctx, plan, m, prng, blind, keep_stub, pile_base_z)

    # ---- 2. the plan footprint of the loss --------------------------------
    dep_default = max(REGION_DEPTH_MIN_M,
                      REGION_DEPTH_FRAC * min(float(m["W"]), float(m["D"])))
    dep = float(depth_m) if depth_m is not None else dep_default
    region = {}
    for sd in plan["sides"]:
        runs = [plan["span"][(sd, s)] for s in plan["storeys"]
                if (sd, s) in plan["span"]]
        if not runs:
            continue
        region[sd] = (float(min(a for a, _b in runs)),
                      float(max(b for _a, b in runs)), dep)
    plan["region"] = region

    # DOES THE LOSS REACH A CORNER? It decides which machinery rags which
    # edge; `plan_edges` asks the same question per storey, this is the top
    # storey's answer and is what a caller/test reads.
    reaches = {}
    for sd in plan["sides"]:
        L = fc.side_length(m, sd)
        iv = plan["span"].get((sd, plan["top_storey"]))
        if iv is None:
            runs = [plan["span"][(sd, s)] for s in plan["storeys"]
                    if (sd, s) in plan["span"]]
            iv = (min(a for a, _b in runs), max(b for _a, b in runs)) if runs \
                else (0.0, 0.0)
        reaches[sd] = (iv[0] <= pad, iv[1] >= L - pad)
    plan["reaches_end"] = reaches

    # ---- 3. THE WHOLE PERIMETER OF THE HOLE -------------------------------
    # A KIT SEAM IS A STRAIGHT LINE AND AN EARTHQUAKE DOES NOT CUT ONE.
    # `fire_collapse.plan_edges` asks, per SURVIVING module, "is a dead module
    # against me, and on which of my four edges" — so the hole's whole
    # perimeter is covered whatever shape the hole is. The kill list it is
    # given carries the STUB JOBS too: those modules are about to be broken by
    # a z-line split of their own and `_break_split` deactivates its source,
    # so a second job on the same prim would fracture a dead prim.
    #
    # ...and the ROOF is filtered out of it. A roof tile's `side` is whichever
    # wall its CENTRE happens to be nearest and its `storey` is the top one,
    # so leaving it in the dead table invents adjacencies on a wall it never
    # touched. `plan_edges`'s own `live()` filter already ignores roofs
    # (`fire_collapse.SHELL_ROLES` has no `roof`), so this only has to match
    # it on the dead side.
    edge_kill = [e for e in plan["kill"] if e["role"] in fc.SHELL_ROLES]
    edge_kill += [j["el"] for j in plan["stub_jobs"]]
    # ...AND THE PANELS. A module kept whole for the pile that also picks up a
    # tear job is `_break_split` into cells before `_lay_existing` ever sees
    # it, and the pile then lays a deactivated prim.
    edge_kill += list(plan["panels"])
    edge_plan = dict(plan)
    edge_plan["kill"] = edge_kill
    lim = int(MAX_EDGE_MODULES if max_edges is None else max_edges)
    tears = fc.plan_edges(ctx, edge_plan, m, prng, budget=lim)
    # ...AND THE SECOND RING, this family's own. `plan_edges` covers the
    # ORTHOGONAL perimeter and covers it completely (measured: 100 % torn on
    # every style probed); what it cannot see is the bay one module out, the
    # re-entrant corner of a staircase step, the corner column at the end of
    # the run, and the roofline of the elevations that did not fail. See the
    # RING_* block. Appended to the SAME list so `_tear_perimeter` authors
    # both in one pass and no module is ever split twice.
    ring = _ring_edges(ctx, plan, m, prng, tears, edge_kill)
    plan["tears"] = tears
    plan["ring"] = ring
    plan["ring_edges"] = int(ring["modules"])

    # THE BUDGET IS REPORTED, NEVER SILENTLY ENFORCED. `_plan_partial` raises
    # its failure line a storey at a time while it can (the notch gets
    # shallower, not narrower, so the staircase survives); a band / total /
    # pancake mode has no such lever — it sweeps whole storeys by definition —
    # so an overrun there has to be VISIBLE in the plan and in the note, and
    # an under-torn edge is a bug, not a saving.
    n_mod = len(plan["kill"])
    cap = int(plan.pop("_module_cap", MAX_MODULES))
    dropped_edges = len([j for j in tears if j.get("dropped")])
    plan["budget"] = {
        "modules": n_mod, "module_cap": cap,
        # `edges` is the FIRST ring only — the jobs `max_edges` / `edge_cap`
        # actually governs. The second ring has a cap of its own
        # (`QC_RING_MAX`) because it is a different cost with a different
        # lever, and folding the two together made `edges > edge_cap` read as
        # an overrun when nothing had overrun.
        "edges": len(tears) - int(ring["new"]), "edge_cap": lim,
        "ring": int(ring["modules"]), "ring_new": int(ring["new"]),
        "ring_cap": int(ring["cap"]),
        "trimmed_storeys": int(plan.pop("_trimmed", 0)),
        "over_modules": max(0, n_mod - cap),
        "over_edges": dropped_edges,
    }
    return plan


# ---------------------------------------------------------------------------
# THE SECOND RING
# ---------------------------------------------------------------------------
def _ring_edges(ctx, plan, m, prng, jobs, edge_kill):
    """Everything round the hole that `fire_collapse.plan_edges` cannot see.

    Appends to `jobs` IN PLACE and returns how many modules it added or
    extended. Pure geometry, host-side, no `pxr`: exactly `plan_edges`'s own
    contract, and every job it writes carries the same keys and the same
    `cuts` dicts, so `fire_collapse._tear_perimeter` authors them with the
    same three judges and nothing here has to know about USD.

    A module `plan_edges` already gave a job gets its extra cuts MERGED into
    that job rather than a second one, because `quake_flow._break_split`
    deactivates the source prim and a module can only be split once.

    Four things, in the order they matter to a viewer:

      1. RE-ENTRANT CORNERS. A staircase step's inner corner: the module that
         is diagonally across from the dead run — it overlaps it in neither t
         (so no left/right) nor z (so no above/below) — and is therefore the
         one right angle in the notch `plan_edges` leaves whole. Bitten on
         BOTH exposed edges at once, which is what makes it read as a broken
         corner rather than as a chamfer.
      2. THE SECOND BAY. The module one out from a first-ring tear, on the
         same storey, bitten on its near end at `RING_PEN2`. This is the
         "full intact rectangle with windows surviving" of the review: one
         bay past a torn one, a factory module with a factory seam.
      3. CORNER COLUMNS. `quake_flow._side_of` gives a corner piece whichever
         of its two walls is nearest, and at a tie the dict order (S, N, W, E)
         decides — so the corner at the end of a lost run is routinely
         BOOKED TO THE OTHER WALL and escapes every one of `plan_edges`'s
         classes, including `return` (which needs the dead run to reach
         within `pad_m` of the wall's end). It then stands proud of the wall
         that went: "one wall extending further than the rest".
      4. THE ROOFLINE. On `elevation` / `corner`, when the loss reached the
         top of the mass, the top course of the elevations that did NOT fail.
         `r_parapet_fall` takes a parapet BAND whole (`partial=None`) and
         leaves the wall under it a machined edge; on a style with no band it
         already does exactly this instead (`partial=True`, "a course off most
         of the side"). Drawn per module at `RING_TOP_P`, so the line that
         comes out is irregular and not a second machined one.

    `total` and `pancake` are skipped: nothing of the shell is left standing
    to read as a rectangle except the blind-side stub, and `_author_stubs`
    already tears that at its own height.
    """
    fc = _fc()
    ring_max = int(_f("QC_RING_MAX", RING_MAX))
    if ring_max <= 0 or plan["mode"] in ("total", "pancake"):
        return {"modules": 0, "new": 0, "cap": ring_max}
    mtag = plan["mass"]
    module_m = max(4.0, float(m.get("module") or 4.0))
    killed = set(id(e) for e in edge_kill)
    by_el = dict((id(j["el"]), j) for j in jobs)
    added = set()

    dead = {}
    for e in edge_kill:
        if e["role"] not in fc.SHELL_ROLES:
            continue
        dead.setdefault((e["side"], int(e["storey"])), []).append(
            fc.el_span(m, e))

    rows = {}
    for e in ctx["info"]["elements"]:
        if e["mass"] != mtag or e["role"] not in fc.SHELL_ROLES:
            continue
        if e.get("dead") or id(e) in killed:
            continue
        rows.setdefault((e["side"], int(e["storey"])), []).append(e)
    for k in rows:
        rows[k].sort(key=lambda q: fc.el_span(m, q)[0])

    new = set()

    def _job(e):
        j = by_el.get(id(e))
        if j is not None and j.get("dropped"):
            # `plan_edges` ran out of budget on this module and `_tear_
            # perimeter` will skip it. A second job on the same element would
            # be a second `_break_split` of a prim the first one deactivated.
            return None
        if j is None:
            t0, t1 = fc.el_span(m, e)
            za, zb = fc.el_z_span(m, e)
            j = {"el": e, "name": e.get("name"), "side": e["side"],
                 "storey": int(e["storey"]), "t0": t0, "t1": t1,
                 "za": za, "zb": zb, "w": max(0.3, t1 - t0),
                 "h": max(0.3, zb - za), "classes": [], "cuts": [],
                 "ring": True}
            by_el[id(e)] = j
            jobs.append(j)
            new.add(id(e))
        added.add(id(e))
        return j

    def _room():
        # The ring's own ceiling, counted in modules it has to SPLIT. Extra
        # cuts merged into a job `plan_edges` already wrote cost nothing —
        # `_tear_perimeter` unions the judges and breaks the prim once.
        return len(new) < ring_max

    def _cls(j, c):
        if c not in j["classes"]:
            j["classes"].append(c)
            j["classes"] = [q for q in fc.EDGE_CLASSES if q in j["classes"]]

    def _has(e, kind, flag):
        """Is that end of this module already cut? Two cracks at one end are
        one crack with a sliver between them."""
        j = by_el.get(id(e))
        if j is None:
            return False
        key = "loose_hi" if kind == "v" else "loose_above"
        return any(q["kind"] == kind and bool(q.get(key)) == flag
                   for q in j["cuts"])

    def _v(e, low_end, span):
        """A vertical crack `span` of the module's width in from one end.

        THE JOB IS CREATED LAST, after the cut is known to be going in. A job
        with an empty `cuts` list is a module `_tear_perimeter` skips (`if not
        judges: continue`) but `boundary_line` counts — a boundary counter
        that can never reach N/N.
        """
        if _has(e, "v", not low_end):             # never two cracks at one end
            return
        j = _job(e)
        if j is None:
            return
        pen = prng.uniform(*span) * j["w"]
        j["cuts"].append({"cls": "right" if low_end else "left", "kind": "v",
                          "pen": pen,
                          "line": (j["t0"] + pen) if low_end
                          else (j["t1"] - pen),
                          "loose_hi": not low_end,
                          "amp": min(fc.EDGE_AMP_MAX, fc.EDGE_AMP_FRAC * pen),
                          "ring": True})
        _cls(j, "right" if low_end else "left")

    def _carries_band(e):
        """Does a surviving parapet / cornice band rest on this module's top?

        A parapet has NOTHING else holding it up — it is a band sitting on the
        top course — so taking that course away leaves it in the air. A full
        storey module above is different: it bears on the floor slab, and a
        bite at the slab line is the same thing `plan_edges`'s own `below`
        class authors under a hole.
        """
        za_e, zb_e = fc.el_z_span(m, e)
        t0_e, t1_e = fc.el_span(m, e)
        w_e = max(0.3, t1_e - t0_e)
        for q in rows.get((e["side"], int(e["storey"])), ()) or ():
            if q is e or q["role"] not in ("parapet", "parapet_corner"):
                continue
            qa, _qb = fc.el_z_span(m, q)
            q0, q1 = fc.el_span(m, q)
            if abs(qa - zb_e) > 0.15:
                continue
            if min(q1, t1_e) - max(q0, t0_e) > 0.5 * min(w_e,
                                                         max(0.3, q1 - q0)):
                return True
        return False

    def _z(e, from_top, span):
        """A horizontal break `span` of the module's height in from one edge."""
        if _has(e, "z", from_top):
            return
        j = _job(e)
        if j is None:
            return
        pen = prng.uniform(*span) * j["h"]
        j["cuts"].append({"cls": "below" if from_top else "above", "kind": "z",
                          "pen": pen,
                          "line": (j["zb"] - pen) if from_top
                          else (j["za"] + pen),
                          "loose_above": from_top,
                          "amp": min(fc.EDGE_AMP_MAX, fc.EDGE_AMP_FRAC * pen),
                          "ring": True})
        _cls(j, "below" if from_top else "above")

    # ---- 1. the re-entrant corners of the staircase ----------------------
    for (sd, s), run in sorted(rows.items()):
        for e in run:
            if not _room():
                break
            t0, t1 = fc.el_span(m, e)
            w = max(0.3, t1 - t0)
            etol = fc.edge_gap_tol(e, w, 0.6)
            over = min(1.2, 0.3 * w)
            for key, from_top in ((s + 1, True), (s - 1, False)):
                iv = dead.get((sd, key)) or ()
                # `plan_edges` owns anything the dead run sits OVER or UNDER:
                # that is its `below` / `above`, and a module that already has
                # its whole top or foot taken must not also lose both ends.
                # A re-entrant corner is the one with no vertical overlap at
                # all — the module diagonally across the staircase step.
                if sum(max(0.0, min(b, t1) - max(a, t0)) for a, b in iv) > over:
                    continue
                hi = any(-0.25 * w <= (a - t1) <= etol for a, _b in iv)
                lo = any(-0.25 * w <= (t0 - b) <= etol for _a, b in iv)
                if not (hi or lo):
                    continue
                _v(e, low_end=lo, span=RING_DIAG_PEN)
                if not (from_top and _carries_band(e)):
                    _z(e, from_top=from_top, span=RING_DIAG_PEN)

    # ---- 2. the second bay along the wall --------------------------------
    first = set(id(j["el"]) for j in jobs
                if not j.get("ring") and not j.get("dropped")
                and ("left" in j["classes"] or "right" in j["classes"]))
    for (sd, s), run in sorted(rows.items()):
        for i, e in enumerate(run):
            if id(e) not in first or not _room():
                continue
            jf = by_el[id(e)]
            # away from the hole: a `left` job has the hole at higher t, so
            # the bay one out is the one BELOW it in t, and vice versa.
            for cls, nb in (("left", run[i - 1] if i else None),
                            ("right", run[i + 1] if i + 1 < len(run) else None)):
                if cls not in jf["classes"] or nb is None:
                    continue
                if id(nb) in killed or nb.get("dead") or not _room():
                    continue
                t0n, t1n = fc.el_span(m, nb)
                gap = (fc.el_span(m, e)[0] - t1n) if cls == "left" \
                    else (t0n - fc.el_span(m, e)[1])
                if gap > fc.edge_gap_tol(nb, max(0.3, t1n - t0n), 0.6):
                    continue
                _v(nb, low_end=(cls == "right"), span=RING_PEN2)

    # ---- 3. the corner columns at the ends of a lost run -----------------
    reach = RING_CORNER_REACH_MODULES * module_m
    by_storey = {}
    for e in edge_kill:
        if e["role"] in fc.SHELL_ROLES:
            by_storey.setdefault(int(e["storey"]), []).append(
                (float(e["lx"]), float(e["ly"])))
    for (sd, s), run in sorted(rows.items()):
        near = [q for k in (s - 1, s, s + 1) for q in by_storey.get(k, ())]
        if not near:
            continue
        for e in run:
            if e["role"] not in ("corner", "parapet_corner"):
                continue
            if not _room():
                break
            ex, ey = float(e["lx"]), float(e["ly"])
            d2 = min((ex - x) ** 2 + (ey - y) ** 2 for x, y in near)
            if d2 > reach * reach:
                continue
            # The near end is the one facing the wall that went, measured on
            # THIS piece's own side: the mean t of the dead run, against the
            # piece's own midpoint.
            L2 = (m["W"] if sd in ("S", "N") else m["D"]) / 2.0
            vals = [(x if sd in ("S", "N") else y) for x, y in near]
            t_dead = sum(vals) / float(len(vals)) + L2
            t0, t1 = fc.el_span(m, e)
            _v(e, low_end=(t_dead <= 0.5 * (t0 + t1)),
               span=RING_CORNER_PEN)

    # ---- 4. the roofline of the elevations that did not fail -------------
    # THE TOPMOST PIECE AT EACH BAY, NOT EVERY PIECE IN THE BAY. A top-storey
    # wall module usually has a parapet/cornice band sitting on it, and taking
    # a course off the WALL's top edge with the band still standing on it
    # leaves the band hanging in the air — the same class of bug as soot on a
    # wall a later recipe took away. Break the piece with the highest top in
    # each bay and leave whatever it is standing on alone.
    top = int(plan["top_storey"])
    if plan["mode"] in ("elevation", "corner") and top in plan["storeys"]:
        for sd in plan.get("keep_sides") or ():
            # BAYS BY OVERLAP IN t, not by a rounded midpoint: a corner's wall
            # piece and the parapet_corner standing on it are the same bay but
            # different lengths (1.25 m against 1.81 m on `apartment_long`), so
            # a midpoint bucket puts them in different bays and cuts the top
            # off the WALL with the parapet still sitting on it.
            # ...and MOSTLY overlapping, not merely touching: consecutive kit
            # pieces on one wall share about 0.12 m of t (a 5.07 m wall module
            # runs 0.96-6.03 and the next 5.96-11.03), so "any overlap" chains
            # a whole elevation into one bay. Half the shorter piece separates
            # 2 % neighbours from a 100 % wall-and-its-parapet pair.
            bays = []
            for e in sorted(rows.get((sd, top), ()),
                            key=lambda q: fc.el_span(m, q)[0]):
                t0, t1 = fc.el_span(m, e)
                w_e = max(0.3, t1 - t0)
                for b in bays:
                    ov = min(b["t1"], t1) - max(b["t0"], t0)
                    if ov > 0.5 * min(w_e, b["t1"] - b["t0"]):
                        if fc.el_z_span(m, e)[1] > fc.el_z_span(m, b["e"])[1]:
                            b["e"], b["t0"], b["t1"] = e, t0, t1
                        break
                else:
                    bays.append({"t0": t0, "t1": t1, "e": e})
            for e in [b["e"] for b in bays]:
                if not _room():
                    break
                if prng.random() >= RING_TOP_P:
                    continue
                _z(e, from_top=True, span=RING_TOP_PEN)

    # ---- 5. the block above a crushed storey -----------------------------
    # A `v` cut only. A `z` cut would take the top or the foot off a panel
    # that is carrying the storeys above it, and the block moves as one rigid
    # body — the fragment would ride down with a hole under it.
    band = plan["band"][0] if plan["band"] else None
    if band is not None:
        for (sd, s), run in sorted(rows.items()):
            if s <= int(band):
                continue
            for e in run:
                if not _room():
                    break
                if e["role"] not in ("wall", "corner"):
                    continue
                if id(e) in by_el:
                    continue          # already at the edge of the crush gap
                if prng.random() >= RING_SPALL_P:
                    continue
                _v(e, low_end=(prng.random() < 0.5), span=RING_SPALL_PEN)

    return {"modules": len(added), "new": len(new), "cap": ring_max}


# ---------------------------------------------------------------------------
# Per-mode planning
# ---------------------------------------------------------------------------
def _plan_partial(ctx, plan, m, prng, mode, side, corner, from_storey,
                  span_frac, max_storeys, max_modules, drop_slabs, infill,
                  module_m):
    """`elevation` and `corner`: a NOTCH in one or two elevations, staircased,
    with every other elevation untouched.

    THE FAILURE LINE IS A CONSTRUCTION-TYPE DECISION, not a fire origin. A URM
    elevation peels from the PARAPET DOWN: the top one or two storeys are the
    unrestrained ones and are what the reconnaissance photographs show going
    out of plane. An RC frame does not peel at all — it loses its INFILL, bay
    by bay, from the first floor up, and its ground storey is a shopfront made
    of glass rather than masonry.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    btype = plan["btype"]
    mtag = plan["mass"]
    top_i = plan["top_storey"]

    # ---- which elevation(s) ---------------------------------------------
    base_side = side or weak_side(ctx, mtag, prng)
    if mode == "corner":
        if not corner:
            nb = ("E", "W") if base_side in ("S", "N") else ("S", "N")
            corner = fc.shared_corner((base_side, nb[prng.randrange(2)]))
        sides = fc.corner_sides(corner)
    else:
        corner = None
        sides = (base_side,)
    plan["sides"] = tuple(sides)
    plan["corner"] = corner
    plan["keep_sides"] = tuple(s for s in ("S", "E", "N", "W")
                               if s not in sides)

    # ---- the failure line -------------------------------------------------
    if from_storey is not None:
        s0 = int(from_storey)
    elif mode == "corner":
        lo, hi = CORNER_STOREYS.get(btype, CORNER_STOREYS["urm"])
        s0 = n_lv - prng.randint(lo, hi)
    elif btype == "urm":
        s0 = n_lv - 1 - prng.randint(*URM_TOP_DOWN)
    else:
        s0 = RC_FROM_STOREY
    s0 = max(0, min(int(s0), n_lv - 1))
    # THE BAND IS CAPPED FROM THE TOP DOWN. A 60 m strip of elevation on the
    # ground is a bombing; the failure line rises to meet the cap.
    n_max = int(max_storeys) if max_storeys else int(
        3 if infill else MAX_FALL_STOREYS)
    s0 = max(s0, n_lv - max(1, n_max))
    s0 = max(0, min(s0, n_lv - 1))
    # ...but a one-storey loss on a tall block is a hole, not a collapse.
    if n_lv - s0 < 2 and s0 > 0:
        s0 -= 1

    base = (float(span_frac) if span_frac is not None
            else prng.uniform(*SPAN_FRAC))
    reach_m = prng.uniform(*CORNER_REACH_MODULES) * module_m
    centres = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        centres[sd] = (None if mode == "corner"
                       else 0.5 * L + prng.uniform(-0.14, 0.14) * L)

    # AN RC INFILL LOSS TAKES PANELS, NOT THE FRAME. `corner` / `parapet` /
    # `balcony` pieces are the frame's own expression on the kit; taking them
    # away turns "the infill fell out of three bays" into "the corner of the
    # building is missing", which is a DG5 signature at DG3.
    roles = ("wall",) if infill else SHELL_ROLES
    plan["roles"] = roles

    cap = int(max_modules) if max_modules else int(MAX_MODULES)
    trimmed = 0
    while True:
        spans = _spans_elevation(m, sides, s0, top_i, n_lv, base, centres,
                                 corner=corner, reach_m=reach_m)
        kill = _kill_from_spans(ctx, m, mtag, spans, roles, plan["pad_m"])
        if len(kill) <= cap or s0 >= top_i or s0 + 1 > top_i - 1:
            break
        s0 += 1
        trimmed += 1
    plan["s0"] = int(s0)
    plan["span"] = spans
    plan["kill"] = kill
    plan["storeys"] = list(range(s0, n_lv))
    plan["span_frac"] = base
    plan["corner_reach_m"] = reach_m
    plan["_module_cap"] = cap
    plan["_trimmed"] = trimmed
    plan["sweep"] = True
    plan["roof"] = "ragged"
    # THE EXPOSED FLOOR/CEILING SLAB EDGE IS RAGGED EITHER WAY. An RC infill
    # loss only takes the wall panel (`roles = ("wall",)` above, not the
    # frame), but the slab that panel used to hide behind is the SAME
    # dead-straight authored box `fit_interior` built either way, and it is
    # the "exposed floor plates with dead-straight rectangular edges"
    # artefact whether the opening came from an infill loss or a masonry
    # elevation/corner failure. This used to read `not infill` and skip the
    # break for the infill case — round-6 diagnosis (rect-cutouts review,
    # 2026-08-31) found no reason a slab exposed behind a missing infill
    # panel should stay a machined rectangle when the same slab exposed by a
    # peeled masonry wall does not.
    plan["floor_ragged"] = True
    plan["cut_z"] = float(m["levels"][s0]) - 0.4
    if infill:
        # dropped infill panels have no height to gather speed over
        plan["throw"] = (0.25, 0.9)

    # ---- the floors behind it ---------------------------------------------
    # The topmost slab in the band lost its bearing wall on this side and came
    # down onto the one below; the rest of the band keeps its slab and loses a
    # ragged strip along the open edge. Dropping every slab in the band is a
    # TOTAL collapse wearing this mode's name. And a corner that let go takes
    # the CORNER of each slab (`_ragged_slabs` breaks the open edge), never
    # the whole plate.
    n_drop = 0 if (mode == "corner" or infill) else max(0, int(drop_slabs))
    storeys = plan["storeys"]
    plan["drop"] = ([(mtag, s) for s in storeys[-n_drop:]]
                    if (n_drop and len(storeys) >= 2) else [])
    # EVERYTHING ON A STOREY WHOSE WHOLE PLATE WENT, region or no region: the
    # far half of that floor is standing on nothing the moment the plate
    # moves. Everything else only where its footprint reaches the loss.
    plan["fit_fall"] = list(plan["drop"])
    plan["fit_region_storeys"] = list(storeys)
    plan["teeth"] = True

    # ---- the fan in the street --------------------------------------------
    z_fail = (m["levels"][s0] if s0 < len(m["levels"]) else m["z0"])
    elem_h = max(1.0, float(m["top"]) - float(z_fail))
    for sd in sides:
        L = fc.side_length(m, sd)
        lo, hi = plan["span"][(sd, top_i)]
        if mode == "corner":
            # `r_corner_fail`'s own span: the last `reach + 2 m` of each of the
            # two sides that meet at the corner.
            w = min(0.5, (reach_m + CORNER_ALONG_PAD_M) / L)
            at_hi = fc.corner_at_high_end(sd, corner)
            along = (0.5 - w, 0.5) if at_hi else (-0.5, -0.5 + w)
            along_m = ((L - w * L, L) if at_hi else (0.0, w * L))
            depth = CORNER_DEPTH_M
            kind = "fan"
        elif infill:
            along = (lo / L - 0.5, hi / L - 0.5)
            along_m = (lo, hi)
            depth = prng.uniform(*INFILL_DEPTH_M)
            kind = "windrow"
        else:
            along = (lo / L - 0.5, hi / L - 0.5)
            along_m = (lo, hi)
            depth = prng.uniform(*FAN_DEPTH_M)
            kind = "fan"
        reach = max(1.5, 0.55 * elem_h)
        plan["heaps"].append({
            "kind": kind, "where": "outside", "sides": (sd,),
            "along": (float(along[0]), float(along[1])),
            "along_m": {sd: (float(along_m[0]), float(along_m[1]))},
            "depth_m": float(depth), "offset_m": 0.0, "crown_m": None,
            "spread_frac": prng.uniform(*FAN_SPREAD),
            "elem_h_m": float(elem_h), "stub_h_m": 0.0,
            "base_z": float(m["z0"]), "budget": None,
            "tag": "{0}_{1}".format(plan["mode"], sd),
            "centres": {sd: _heap_centres(m, sd, along_m, 0.45 * reach)},
        })


def _plan_band(ctx, plan, m, prng, mode, storey, lean_side, lean_deg, crush_m,
               twist_deg, offset_m):
    """`soft_storey` and `mid_storey`: ONE storey crushes on ALL FOUR sides
    and the block above drops onto it.

    THE BAND IS BITTEN, NOT SLICED. A kill list of exactly `storey == k`
    leaves a dead-level line top and bottom, both on a slab line — the
    "building lifted off with a pallet fork" artefact the round-4 review
    photographed (`r4_commercial/4_commercial_soft_storey_ne.png`: a clean
    rectangular gap under an otherwise pristine block). So the band takes a
    run of the storey ABOVE with it (the bottom of the block that came down)
    and, at a mid storey, a smaller run of the one BELOW; the ends of those
    runs are where `plan_edges` then puts left/right tears, and the seam under
    and over the band gets below/above tears everywhere else.

    THE MECHANISM IS `quake_flow._soft_storey_geometry` AND IS NOT
    RE-DERIVED HERE. It is two mechanisms drawn 60/40 (differential crush on a
    wedge, and a plumb sidesway rack), and the ORIGINAL code in this pipeline
    pivoted the wrong base edge and leaned buildings AWAY from the side it
    named — see "Soft-storey mechanics fix" in `build-earthquake-scenes`. The
    fix lives there; this plan carries its output verbatim in `crush["geo"]`
    and `r_collapse` hands that straight to `_soft_storey_matrix`.
    """
    fc, qf = _fc(), _qf()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    lv = list(m["levels"])

    # A MID-STOREY CRUSH NEEDS A STOREY IN THE MIDDLE. `r_mid_storey` makes
    # the same call (`if n < 3: return r_soft_storey(storey=0)`) — on a one-
    # or two-level mass there is nothing between the ground and the roof, and
    # forcing k = 1 crushes the top storey, which is a roof collapse.
    if n_lv < 3 and storey is None:
        mode = "soft_storey"
        plan["mode"] = mode
    if storey is not None:
        k = int(storey)
    elif mode == "mid_storey":
        k = prng.randrange(1, max(2, n_lv - 1))
    else:
        k = 0
    k = max(0, min(k, n_lv - 1))
    if mode == "mid_storey" and k < 1:
        k = min(1, max(0, n_lv - 1))
    z_lo = float(lv[k])
    z_hi = float(lv[k + 1]) if k + 1 < len(lv) else float(m["top"])
    h_st = max(0.6, z_hi - z_lo)

    sides = ("S", "E", "N", "W")
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["band"] = [k]
    plan["s0"] = k
    plan["roles"] = SHELL_ROLES
    plan["roof"] = "ragged"
    plan["cut_z"] = z_lo
    plan["throw"] = None            # crushed, not thrown: see BAND_THROW
    plan["sweep"] = False

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        spans[(sd, k)] = (0.0, L)
        # A ZERO-WIDTH BITE IS NOT A BITE. Recording `(t0, t0)` would put an
        # entry in `span` that kills nothing, reads as "the band is bitten"
        # to anything that walks the table, and hides the very artefact the
        # bite exists to prevent.
        if k + 1 < n_lv:
            w = prng.uniform(*BAND_BITE_UP) * L
            t0 = prng.uniform(0.0, max(0.0, L - w))
            if w > 1e-6:
                spans[(sd, k + 1)] = (float(t0), float(min(L, t0 + w)))
        if k >= 1:
            w = prng.uniform(*BAND_BITE_DOWN) * L
            t0 = prng.uniform(0.0, max(0.0, L - w))
            if w > 1e-6:
                spans[(sd, k - 1)] = (float(t0), float(min(L, t0 + w)))
    plan["span"] = spans
    plan["storeys"] = sorted(set(s for _sd, s in spans))
    cap = int(MAX_MODULES_TOTAL)
    plan["kill"] = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES,
                                    plan["pad_m"])
    plan["_module_cap"] = cap
    plan["_trimmed"] = 0

    side_lean = lean_side or ("S", "E", "N", "W")[prng.randrange(4)]
    crush_frac = (float(crush_m) / h_st) if crush_m is not None else None
    if mode == "mid_storey":
        # `r_mid_storey`'s own numbers: a gentler tilt with a plan TWIST and
        # an offset, which is the one thing that tells a mid-storey crush from
        # a building that is merely shorter.
        lean = lean_deg if lean_deg is not None else prng.uniform(1.5, 4.0)
        twist = (twist_deg if twist_deg is not None
                 else prng.uniform(2.0, 8.0) * (1 if prng.random() < 0.5 else -1))
        off = offset_m if offset_m is not None else prng.uniform(0.3, 2.0)
    else:
        lean = lean_deg
        twist = twist_deg or 0.0
        off = offset_m or 0.0
    geo = qf._soft_storey_geometry(m, side_lean, h_st, z_lo, prng,
                                   lean_deg=lean, crush_frac=crush_frac)
    plan["crush"] = {
        "storey": k, "z_lo": z_lo, "z_hi": z_hi, "h_st": h_st,
        "lean_side": side_lean, "geo": geo,
        "crush_m": 0.5 * (geo["r_lean"] + geo["r_far"]),
        "twist_deg": float(twist), "offset_m": float(off),
    }
    # ONLY THE CRUSHED STOREY. `plan["storeys"]` also holds the bites at
    # k +- 1, and a partition at k + 1 is standing on the slab at k + 1 —
    # which is intact and rides down with the block. Dropping it would leave a
    # sheet of plasterboard in the gap the block just closed.
    plan["fit_fall"] = [(mtag, k)]
    plan["fit_region_storeys"] = []
    plan["drop"] = []
    plan["teeth"] = True

    # THE COLLAR: the crushed storey's material squeezed out round the
    # perimeter, 1-3 m wide (Northridge Meadows, Antakya). ONE `_rubble` call
    # with all four sides — `quake_rubble` builds one strip cell per side from
    # it, which is cheaper than four piles and is what `r_soft_storey` already
    # does. The per-side centres are carried so a test can check every one of
    # them is outside its own wall line at any yaw.
    depth = prng.uniform(*COLLAR_DEPTH_M) + plan["crush"]["crush_m"] * 0.25
    plan["heaps"].append({
        "kind": "windrow", "where": "outside", "sides": sides,
        "along": None, "along_m": dict(
            (sd, (0.0, fc.side_length(m, sd))) for sd in sides),
        "depth_m": float(depth), "offset_m": 0.0, "crown_m": None,
        "spread_frac": None, "elem_h_m": float(h_st), "stub_h_m": 0.0,
        "base_z": float(z_lo), "budget": None,
        "tag": "collar_{0}".format(k),
        "centres": dict(
            (sd, _heap_centres(m, sd, (0.0, fc.side_length(m, sd)),
                               0.45 * max(1.0, h_st)))
            for sd in sides),
    })


def _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=0):
    """The storey-`storey` wall modules on a BLIND side, torn at a stub height
    instead of killed. Fills `stub_jobs` / `stub` / `blind_sides`.

    `r_masonry_collapse(keep_stub=True)` keeps a stub on EVERY side, and the
    result reads as a bathtub with rubble in it. The record is the opposite:
    where a stub survives a total collapse it is the PARTY WALL — the one the
    neighbour braced (research sec5a; the skill's `STUB_KEEP_P` row records
    that no code gated this before).
    """
    qf = _qf()
    bs = blind_sides(ctx, given=blind, prng=prng)
    plan["blind_sides"] = tuple(bs)
    if not keep_stub or not bs:
        return
    jobs, hs = [], {}
    # NO `prim_path` FILTER. Whether a placement has been written to the stage
    # yet is an AUTHORING fact, not a planning one — host-side (the tests,
    # `check()`, a dry run) no placement has a path at all, and filtering on it
    # here made the stub list silently empty in every offline check while
    # working in Kit. `_author_stubs` skips a pathless element, exactly as
    # `fire_collapse._tear_perimeter` does with its own jobs.
    for e in qf._els(ctx, mass=plan["mass"], role="wall", storey=int(storey)):
        if e["side"] not in bs:
            continue
        frac = prng.uniform(*STUB_FRAC)
        h = float(e.get("h") or 3.0)
        jobs.append({"el": e, "side": e["side"], "storey": int(storey),
                     "frac": float(frac), "h_m": float(h * frac),
                     "z_line": float(e["z"]) + h * frac})
        hs.setdefault(e["side"], []).append(h * frac)
    plan["stub_jobs"] = jobs
    plan["stub"] = dict((sd, sum(v) / len(v)) for sd, v in hs.items() if v)


def _plan_panels(ctx, plan, m, prng, storey=1, n=(1, 2)):
    """1-2 whole wall modules with openings, kept WHOLE for the pile to lay.

    `quake_flow._pick_opening_panels` does the same job for
    `r_masonry_collapse`, but it draws from `ctx["rng"]`, mutates the element
    table and needs a `prim_path` — three authoring concerns in a planning
    decision. This picks the ELEMENTS from the private generator and
    `_author_heaps` turns them into the `(prim_path, size)` pairs `_rubble`
    wants (`quake_rubble_usd._lay_existing` re-lays the prim itself onto the
    mound, so the module has to be kept out of BOTH the fracture loop and the
    tear list or it is destroyed before the pile gets it).

    There is no per-piece "has an opening" flag on this kit, so the pool is a
    name-pattern share with a fall-back to any wall module — the same call
    `_pick_opening_panels` makes the same way. Never on a blind side: that
    wall is where the stub is.
    """
    qf = _qf()
    if qf._RUBBLE_MODE != "v2":
        return []
    st = max(0, min(int(storey), plan["n_levels"] - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    pool = [e for e in qf._els(ctx, mass=plan["mass"], role="wall", storey=st)
            if id(e) not in skip
            and e["side"] not in plan.get("blind_sides", ())]
    named = [e for e in pool if any(k in (e.get("name") or "").lower()
                                    for k in OPENING_PAT)]
    pool = sorted(named or pool,
                  key=lambda q: (q["side"], round(q["lx"], 3),
                                 round(q["ly"], 3), q.get("name") or ""))
    if not pool:
        return []
    k = min(len(pool), 1 if len(pool) < 2 else prng.randint(*n))
    idx = list(range(len(pool)))
    prng.shuffle(idx)
    return [pool[i] for i in sorted(idx[:k])]


def _plan_total(ctx, plan, m, prng, blind, keep_stub, pile_base_z=None):
    """`total`: a URM building has come down into its own footprint.

    Everything the shell has goes — walls, corners, balconies, the parapet
    ring AND the roof, which is the one thing the fire's `SHELL_ROLES`
    deliberately excludes (a burnt-out shell keeps its skyline; a collapsed
    one has no skyline left). A stub survives on the blind sides only.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    sides = ("S", "E", "N", "W")
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["s0"] = 0
    plan["roles"] = ALL_ROLES
    plan["roof"] = "kill"
    plan["sweep"] = False
    plan["throw"] = (TOTAL_THROW_BASE, TOTAL_THROW_TOP)
    plan["cut_z"] = float(m["z0"]) - 0.4

    _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=0)
    plan["panels"] = _plan_panels(ctx, plan, m, prng,
                                  storey=min(1, n_lv - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    skip |= set(id(e) for e in plan["panels"])

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        for s in range(0, n_lv):
            spans[(sd, s)] = (0.0, L)
    plan["span"] = spans
    plan["storeys"] = list(range(0, n_lv))
    plan["_module_cap"] = int(MAX_MODULES_TOTAL)
    plan["_trimmed"] = 0
    kill = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES, plan["pad_m"],
                            skip=skip)
    # THE ROOF, UNCONDITIONALLY. A roof tile's `side` is whichever wall its
    # centre is nearest and its span test is meaningless, so it is swept by
    # ROLE and not by geometry.
    qf = _qf()
    kill += list(qf._els(ctx, mass=mtag, role=ROOF_ROLES))
    plan["kill"] = kill
    # A JOISTED FLOOR COMES APART INTO BOARDS, AND THEN IT IS BURIED. Dropping
    # a 22 x 18 m plate whole puts one sheet on the crown of the pile;
    # `r_masonry_collapse` learned that in round 2 ("the DG5 heaps read as a
    # LUMBER YARD") and breaks them instead, small and hard-thinned.
    plan["drop"] = []
    plan["break_slabs"] = [(mtag, s) for s in range(0, n_lv)]
    plan["fit_fall"] = [(mtag, s) for s in range(0, n_lv)]
    plan["fit_region_storeys"] = []
    plan["bury_props"] = True
    plan["teeth"] = False

    H = max(3.0, float(m["top"]) - float(m["z0"]))
    stub_h = (sum(plan["stub"].values()) / len(plan["stub"])
              if plan["stub"] else 0.0)
    fall = tuple(s for s in sides if s not in plan.get("blind_sides", ()))
    plan["heaps"].append({
        "kind": "dome", "where": "footprint",
        "sides": fall or ("S", "E"),
        "along": None, "along_m": {},
        "depth_m": None, "offset_m": 0.0,
        "crown_m": float(H * DOME_CROWN_FRAC),
        "spread_frac": prng.uniform(*DOME_SPREAD),
        "elem_h_m": None, "stub_h_m": float(stub_h),
        # ON THE GROUND, NOT AT THE WING'S OWN BASE. A wing's `m["z0"]` is
        # the MAIN mass's roof, so `r_masonry_collapse`'s per-mass
        # `_rubble(ctx, m, "dome", ...)` leaves a wing's pile floating four
        # metres up over the heap the main body just made (`block_stone`: a
        # 1-level base under two 8-storey wings). `_pile_mass` moves the base
        # and keeps `top - z0` — every derived height and reach — unchanged.
        "base_z": float(m["z0"] if pile_base_z is None else pile_base_z),
        "budget": None,
        "tag": "collapse_{0}".format(mtag), "centres": {},
        "panels": list(plan["panels"]),
    })


def _plan_pancake(ctx, plan, m, prng, from_storey, pitch_m, blind, keep_stub,
                  stack_base_z=None):
    """`pancake`: an RC frame comes down storey by storey and the SLABS stack.

    The shell is killed from `s0` up, progressively wider — the bottom two
    storeys keep a run at each end (the frame did not let go all at once), and
    everything above goes whole. The slabs are then RE-AUTHORED as a stack at
    a pitch rather than simulated: a stack of thin plates is exactly what
    PhysX does worst, and `r_pancake` learned that in round 1. The stacking
    numbers here ARE `r_pancake`'s.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    sides = ("S", "E", "N", "W")
    s0 = max(0, min(int(from_storey or 0), n_lv - 1))
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["s0"] = s0
    plan["roles"] = SHELL_ROLES
    plan["roof"] = "stack"
    plan["sweep"] = False
    plan["throw"] = (0.30, 1.60)          # r_pancake's own façade shed
    plan["cut_z"] = float(m["levels"][s0]) - 0.4

    _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=s0)
    plan["panels"] = _plan_panels(ctx, plan, m, prng,
                                  storey=min(s0 + 1, n_lv - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    skip |= set(id(e) for e in plan["panels"])

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        for s in range(s0, n_lv):
            k = min(1.0, (s - s0) / float(max(1, PANCAKE_RAMP_STOREYS)))
            ramp = PANCAKE_FOOT + (1.0 - PANCAKE_FOOT) * k
            w = min(L, ramp * L)
            t0 = max(0.0, min(0.5 * L - w / 2.0, L - w))
            spans[(sd, s)] = (float(t0), float(min(L, t0 + w)))
    plan["span"] = spans
    plan["storeys"] = list(range(s0, n_lv))
    plan["_module_cap"] = int(MAX_MODULES_TOTAL)
    plan["_trimmed"] = 0
    plan["kill"] = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES,
                                    plan["pad_m"], skip=skip)
    # THE SLABS ARE THE STACK. They are re-authored at a pitch below, so they
    # must not ALSO be handed to the solver: one prim in two places is the
    # double-transform class of bug the round-4 catalogue records twice.
    plan["drop"] = []
    plan["fit_fall"] = [(mtag, s) for s in range(s0, n_lv)]
    plan["fit_region_storeys"] = []
    plan["bury_props"] = True
    plan["teeth"] = False

    pitch = float(pitch_m) if pitch_m is not None else prng.uniform(*PANCAKE_PITCH)
    # A WING PANCAKES ONTO WHAT THE MAIN BODY LEFT, not onto the ground:
    # `r_pancake` puts a wing's stack base at the main stack's own top, which
    # is what a tower over a collapsed podium actually does. `r_collapse`
    # computes that height from the main plan and passes it in.
    base = float(m["z0"] if stack_base_z is None else stack_base_z)
    plates = []
    for i in range(1, n_lv + 1):
        top_plate = (i >= n_lv - 1)
        allside = ["S", "E", "N", "W"]
        prng.shuffle(allside)
        n_sides = ((3 + (1 if prng.random() < 0.5 else 0)) if top_plate
                   else (2 if prng.random() < 0.45 else 1))
        plates.append({
            "storey": i, "z": base + pitch * (i - 1 + 0.5),
            "tilt_deg": prng.uniform(*PANCAKE_TILT),
            "axis": ((1.0, 0.0, 0.0) if prng.random() < 0.5
                     else (0.0, 1.0, 0.0)),
            "jitter": (prng.uniform(-PANCAKE_JITTER, PANCAKE_JITTER),
                       prng.uniform(-PANCAKE_JITTER, PANCAKE_JITTER)),
            "sides": tuple(allside[:n_sides]), "top": bool(top_plate),
        })
    plan["stack"] = {"pitch_m": pitch, "base_z": base,
                     "top_z": base + pitch * n_lv + 0.15, "plates": plates,
                     "roof_tilt_deg": prng.uniform(*PANCAKE_ROOF_TILT)}

    fall = tuple(s for s in sides if s not in plan.get("blind_sides", ()))
    plan["heaps"].append({
        "kind": "dome", "where": "footprint",
        "sides": fall or ("S", "E"),
        "along": None, "along_m": {},
        "depth_m": None, "offset_m": 0.0,
        "crown_m": float(pitch * n_lv * PANCAKE_CROWN_FRAC),
        "spread_frac": prng.uniform(*PANCAKE_SPREAD),
        "elem_h_m": None, "stub_h_m": 0.0,
        "base_z": float(base), "budget": {"n_large": PANCAKE_LARGE_BUDGET},
        "tag": "pancake_{0}".format(mtag), "centres": {},
        "panels": list(plan["panels"]),
    })


# ---------------------------------------------------------------------------
# Reading a plan
# ---------------------------------------------------------------------------
def band_transform(plan):
    """The band modes' rigid displacement as pure numbers, for a host-side
    check: `(translate, pivot, axis, deg, twist_deg, offset_xy)`.

    `r_collapse` composes exactly this through `quake_flow.
    _soft_storey_matrix` + `_rot_about` + `_translate`; a test can apply it
    with numpy (the routing test's `_apply`) and assert both base edges of the
    block land on their own residual heights. Returns None outside the band
    modes.
    """
    c = plan.get("crush")
    if not c:
        return None
    geo = c["geo"]
    return {"translate": tuple(geo["translate"]), "pivot": geo.get("pivot"),
            "axis": tuple(geo["axis"]), "deg": float(geo.get("deg") or 0.0),
            "twist_deg": float(c.get("twist_deg") or 0.0),
            "offset_m": float(c.get("offset_m") or 0.0),
            "lean_side": c["lean_side"], "pivot_z": (c["z_lo"]
                                                     + c["crush_m"])}


def describe(plan):
    """One line, for `ctx["notes"]` and for a host-side dry run."""
    heaps = ", ".join("{0} {1} on {2}".format(
        h["kind"], h["where"],
        "/".join(h["sides"]) if h["sides"] else "the footprint")
        for h in plan["heaps"])
    extra = ""
    if plan["crush"]:
        c = plan["crush"]
        extra = (", storey {0} crushed to {1:.2f} m ({2}, lean {3} {4:.1f} "
                 "deg)".format(c["storey"], c["crush_m"], c["geo"]["mode"],
                               c["lean_side"], c["geo"]["lean_deg"]))
    elif plan["stack"]:
        extra = ", {0} slab(s) stacked at {1:.2f} m pitch".format(
            len(plan["stack"]["plates"]), plan["stack"]["pitch_m"])
    elif plan["stub"]:
        extra = ", stub kept on " + "/".join(
            "{0} {1:.2f} m".format(k, v) for k, v in sorted(plan["stub"].items()))
    return ("quake collapse ({0}{1}): {2} on {3} from storey {4} up, "
            "{5} module(s) taken, {6} surviving module(s) torn at the edge, "
            "{7} slab(s) dropped; {8}{9}".format(
                plan["mode"],
                " [infill]" if plan["infill"] else "",
                plan["btype"], "/".join(plan["sides"]), plan["s0"],
                len(plan["kill"]), len(plan["tears"]), len(plan["drop"]),
                heaps, extra))


def boundary_line(plan):
    """`[qc] boundary tears: N/M modules treated (...)` — the bake-log line.

    M is every SURVIVING module either ring put a judge on; N is how many of
    them `_tear_perimeter` actually split. Anything but N/M is a kit seam
    still standing at the edge of the hole, which is the whole complaint this
    ring exists to answer, so the number is printed whether or not anything
    went wrong. `ring` is the share of M this family's own second ring added
    on top of `fire_collapse.plan_edges`'s orthogonal perimeter.
    """
    tears = plan.get("tears") or ()
    live = [j for j in tears if not j.get("dropped")]
    torn = [j for j in live if j.get("torn")]
    ring = plan.get("ring") or {"modules": 0, "new": 0}
    return ("[qc] boundary tears: {0}/{1} modules treated ({2} first ring, "
            "{3} second ring of which {4} new module(s), {5} dropped over "
            "budget, {6} failed to split)".format(
                len(torn), len(live),
                len(live) - int(ring["new"]), int(ring["modules"]),
                int(ring["new"]), len(tears) - len(live),
                len([j for j in live if j.get("failed")])))


# ---------------------------------------------------------------------------
# AUTHORING — everything below touches USD. Every `pxr` import is local.
# ---------------------------------------------------------------------------
def _teeth(ctx, plan, m, prng, storeys):
    """Bearing stubs still in the wall pocket where a floor framed into a wall
    that has gone, and the joist/purlin ends standing out of the break.

    STEEL ON A FRAME, TIMBER ON MASONRY. `urban_fire._joist_stubs` authors the
    same posts but binds `mats["steel"]` unconditionally and reads
    `ctx["fire"]` for its deck height — right for a burnt shell, wrong for a
    brick block that came down, and unusable here without dragging the fire
    ctx in. Same geometry, same reasoning (a POST, not a rod: a thin `_cyl`
    reads as a wire and PhysX cooks nothing for it), two materials.
    """
    qf = _qf()
    mtag = plan["mass"]
    btype = plan["btype"]
    mats = ctx["mats"]
    mat = mats.get("steel" if btype != "urm" else "timber") or mats["plaster"]
    fit = ctx.get("fit") or {}
    slabs = fit.get("slabs") or {}
    W = float(m["W"]) - 2 * qf.WALL_INSET
    D = float(m["D"]) - 2 * qf.WALL_INSET
    n_made = 0
    for s in storeys:
        if s < 1 or s >= len(m["levels"]):
            continue
        # A POST STANDS ON A FLOOR. Without a slab under it the post hangs in
        # clear air, which is the exact bug the fire path recorded on
        # `office_wide` F5c.
        if not slabs.get((mtag, s - 1)):
            continue
        z_slab = float(m["levels"][s]) - 0.18
        z_floor = float(m["levels"][s - 1])
        if z_slab - z_floor < 0.6:
            continue
        for _ in range(prng.randint(*TEETH_N)):
            L = prng.uniform(TEETH_LEN_M[0], min(TEETH_LEN_M[1],
                                                 z_slab - z_floor))
            if prng.random() < 0.5:
                lx = prng.uniform(-W / 2.0, W / 2.0)
                ly = (D / 2.0) * (1.0 if prng.random() < 0.5 else -1.0)
            else:
                ly = prng.uniform(-D / 2.0, D / 2.0)
                lx = (W / 2.0) * (1.0 if prng.random() < 0.5 else -1.0)
            wx, wy = qf._to_world(m, lx, ly)
            sz = prng.uniform(*TEETH_SIZE_M)
            path = "{0}/qcjoist_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                qf._uid(ctx))
            qf._box(ctx["stage"], path, wx, wy, z_floor + L / 2.0, sz, sz, L,
                    prng.uniform(-15.0, 15.0), mat)
            ctx["authored"].append(path)
            ctx["static_extra"].append(path)
            n_made += 1
    return n_made


def _author_kill(ctx, plan, m, prng, pnrng):
    """Fracture every module in the kill list and throw its fragments.

    `mode="uniform"` and NOT `quake_flow._p_frac_kw`: a bond/prism lattice
    makes cuboids, and cuboids are the "toy bricks" the round-5 review
    photographed. `inner_p` is the fire's, which `_break` itself lowers to
    0.18 whenever the piece has a texture and a core subset — so ~82 % of a
    fragment's faces keep the module's own cladding and only the invented cut
    faces take an interior material. Loose fragments then get `_dust_loose`:
    a dusted COPY of whatever each one is actually bound to, so a fallen piece
    is the same material as the wall it came out of, darker.
    """
    qf = _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    mode = plan["mode"]
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))
    n_broke = n_frag = 0
    band_k = plan["band"][0] if plan["band"] else None

    if mode == "total":
        pieces, consume, cap = TOTAL_PIECES, TOTAL_CONSUME, TOTAL_MAX_PIECE_M
    elif mode == "pancake":
        pieces, consume, cap = (PANCAKE_PIECES, PANCAKE_CONSUME,
                                PANCAKE_MAX_PIECE_M)
    elif band_k is not None:
        pieces, consume, cap = BAND_PIECES, BAND_CONSUME, None
    else:
        pieces, consume, cap = BREAK_PIECES, BREAK_CONSUME, None
    consume = _f("QC_CONSUME", consume)

    for e in plan["kill"]:
        if e.get("dead"):
            continue
        if e["role"] in ROOF_ROLES:
            # A MASONRY ROOF IS A TIMBER DECK and comes apart into boards, not
            # into concrete plates. `_break_box_like` swaps the kit tile for a
            # solid slab first (a kit roof tile is a zero-thickness quad) and
            # is the quake pipeline's own helper for it.
            made = qf._break_box_like(ctx, e, 52,
                                      timber=(plan["btype"] == "urm"),
                                      consume=0.85, dusty=True,
                                      max_piece_m=1.0)
            qf._a_lay_flat(ctx, made)
            ctx["loose"] += made
            e["dead"] = True
            n_broke += 1
            n_frag += len(made)
            continue
        st, lo = qf._break(stage, ctx["parent"], e, ctx["tag"],
                           prng.randint(*pieces), prng, pnrng, ctx["mats"],
                           ctx["cache"], info["type"], inner_p=INNER_P,
                           mode="uniform", rough=BREAK_ROUGH,
                           consume=consume, max_piece_m=cap,
                           style=info.get("style"))
        ox, oy = qf._outward(m, e["side"])
        if band_k is not None:
            # THE BAND IS CRUSHED, NOT THROWN. It is squeezed out sideways at
            # walking pace under the mass above it, and the fragments are
            # squashed down to the storey's residual height so the collar sits
            # in the gap rather than floating in it.
            geo = plan["crush"]["geo"]
            z_lo = plan["crush"]["z_lo"]
            h_st = plan["crush"]["h_st"]
            residual = qf._soft_storey_residual(
                geo, m, plan["crush"]["lean_side"], e["side"], e["lx"], e["ly"])
            qf._squash(stage, lo, z_lo, residual / h_st)
            push = prng.uniform(*BAND_PUSH_M)
            qf._transform_prims(stage, lo, qf._translate(ox * push,
                                                         oy * push, 0.0))
            for pth in lo:
                ctx["velocity"][pth] = (ox * prng.uniform(*BAND_THROW),
                                        oy * prng.uniform(*BAND_THROW), 0.0)
        else:
            base_v, top_v = plan["throw"]
            for pth in lo:
                zf = min(1.0, max(0.0, (float(e["z"]) + float(e["h"]) * 0.5
                                        - float(m["z0"])) / Hm))
                v = base_v + top_v * zf + prng.uniform(-0.2, 0.3)
                if mode == "total":
                    # nearly straight down; the outward fan belongs to a peel
                    ctx["velocity"][pth] = (ox * v * prng.uniform(0.3, 1.0),
                                            oy * v * prng.uniform(0.3, 1.0),
                                            -0.4 * prng.random())
                else:
                    ctx["velocity"][pth] = (ox * v, oy * v, 0.15 * v)
        if mode in ("total", "pancake"):
            qf._a_lay_flat(ctx, lo)
        qf._dust_loose(ctx, lo)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n_broke += 1
        n_frag += len(st) + len(lo)
    return n_broke, n_frag


def _author_stubs(ctx, plan, m, prng):
    """Tear the storey-0 wall modules on the blind sides at their stub height.

    `_break_split` with `quake_flow._p_zline_judge`: the break line STAIRCASES
    along the courses (urm) or tears (rc), which is the one edge a total
    collapse's heap leans against and therefore the most-looked-at line in the
    whole recipe (`r_masonry_collapse`'s own note). The surviving stub keeps
    the module's own cladding (`static_mat`); only what comes away is dusted.
    """
    from . import damage
    qf = _qf()
    stage = ctx["stage"]
    btype = plan["btype"]
    n = 0
    for j in plan["stub_jobs"]:
        e = j["el"]
        if e.get("dead"):
            continue
        path = (e.get("p") or {}).get("prim_path")
        if not path:
            continue
        tex = damage.bound_texture(stage, path)
        st, lo = qf._break_split(
            ctx, path, 10 + prng.randrange(4),
            qf._p_zline_judge(m, e["side"], j["z_line"], prng, btype=btype,
                              amp=float(e.get("h") or 3.0) * 0.30,
                              loose_above=True, pitch=qf._p_pitch(ctx)),
            qf._mat_fn(ctx, tex, 0.35),
            static_mat=(qf._clad_material(stage, ctx["parent"], ctx["cache"],
                                          tex) if tex else None))
        ox, oy = qf._outward(m, e["side"])
        for pth in lo:
            v = prng.uniform(0.2, 0.8)
            ctx["velocity"][pth] = (ox * v, oy * v, -0.2 * v)
        qf._dust_loose(ctx, lo)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n += 1
    return n


def _author_heaps(ctx, plan, m, prng):
    """Every pile, through `quake_flow._rubble` and nothing else.

    `_rubble` is `EQ_RUBBLE`-gated (v2 = the round-4 mound / large-element /
    instancer planner; v1 = the old `_heap` box crate) and is the ONE call
    site a routed recipe may use. Nothing here calls `_heap` — the round-4
    routing test asserts exactly that of every recipe it owns, and this family
    is written to pass the same sweep.
    """
    qf = _qf()
    n = 0
    for h in plan["heaps"]:
        mm = qf._pile_mass(m, h["base_z"])
        # THE WHOLE MODULES THE PILE LAYS. `quake_rubble_usd._lay_existing`
        # re-lays the prim itself (it does not copy it), so an element with no
        # `prim_path` — a host-side dry run — simply contributes nothing, and
        # one that IS laid is marked dead so `wreck_building`'s closing sweep
        # does not also file it as a standing wall.
        pan = []
        for e in (h.get("panels") or ()):
            path = (e.get("p") or {}).get("prim_path")
            if not path:
                continue
            pan.append((path, qf._module_size(m, e)))
            e["dead"] = True
        ret = qf._rubble(ctx, mm, h["kind"], sides=h["sides"],
                         along=h["along"], depth_m=h["depth_m"],
                         offset_m=h["offset_m"], stub_h_m=h["stub_h_m"],
                         elem_h_m=h["elem_h_m"], crown_m=h["crown_m"],
                         spread_frac=h["spread_frac"], budget=h["budget"],
                         panels=pan, tag=h["tag"])
        made = ret.get("all") or []
        n += len(made)
        if plan["crush"] and plan["crush"]["storey"] > 0:
            # a collar authored in the air beside a MID-storey band must fall
            # rather than hover (`r_soft_storey`'s own correction) — and, same
            # as there, the mound/apron are a world-baked mesh with no xform
            # op, safe only as long as they stay STATIC; recentre them to a
            # local frame before they become a loose RigidBody, or settle's
            # kick throws them (round 7, s4g2/office_DG4 `collar_1_mound`).
            from . import quake_rubble_usd as qru
            qru.recentre_for_loose(ctx["stage"],
                                   (ret.get("mound"), ret.get("apron")))
            keep = set(made)
            ctx["static_extra"] = [q for q in ctx["static_extra"]
                                   if q not in keep]
            ctx["loose"] += made
    return n


def _fracture_fallen_partition(ctx, path):
    """Turn one unsupported plaster divider into settleable wall chunks.

    `fit_interior` authors a partition as one closed 0.12 m-thick box.  Sending
    that box straight to PhysX preserves a complete room wall: after settling
    it becomes the 6--10 m white sheet seen across the DG5 rubble pile.  The
    support decision belongs to `_fall_fitout`; once that decision is made,
    this helper fractures only that doomed divider in its own thin mid-plane.
    Supported partitions elsewhere in the building never reach this function.

    The fracture seed is derived from the prim path rather than either shared
    generator.  Enabling this appearance fix therefore cannot move the shell,
    slabs, piles, or any later recipe decision.  `mode="prism"` keeps one seed
    layer through the 0.12 m thickness: gypsum breaks into broad irregular
    pieces, not tetrahedral shards.  If VTK cannot fracture a particular box,
    the source is retained, chipped in place, and still sent to settle; no
    geometry silently disappears.

    Returns ``(paths_to_settle, fractured)``.
    """
    import numpy as np

    from . import fracture

    qf = _qf()
    stage = ctx["stage"]
    prim = stage.GetPrimAtPath(path) if path else None
    if not prim or not prim.IsValid() or not prim.IsActive():
        return [], False

    try:
        _cx, _cy, _cz, sx, sy, sz, _yaw = qf._box_dims(stage, path)
        ext = sorted((abs(float(sx)), abs(float(sy)), abs(float(sz))))
        # About one 1.4 x 1.4 m face patch per requested cell.  The caps keep
        # a short apartment divider recognisable and a long office divider
        # well below the scene's rigid-body budget.
        n_piece = max(6, min(18, int(round(ext[1] * ext[2] / 2.0))))
    except Exception:
        n_piece = 10

    seed = fracture.stable_seed("quake_fallen_partition", str(path))
    try:
        made = qf._break_box(
            stage, path, n_piece,
            random.Random(seed), np.random.default_rng(seed),
            qf._a_mat(ctx, "plaster_dusty"),
            qf._a_mat(ctx, "dust"), inner_p=0.18,
            mode="prism", consume=0.22, consume_pool=1.05,
            max_piece_m=1.8)
    except Exception as exc:
        made = []
        ctx.setdefault("notes", []).append(
            "quake partition fracture failed for {0}: {1}: {2}".format(
                path, type(exc).__name__, exc))
    if made:
        return made, True

    # Fail visible, not absent: a chipped whole panel is inferior to chunks,
    # but still better than deleting a room wall or aborting the entire bake.
    _chip_pieces(ctx, [path], _CHIP_PRISM, tessellate=True)
    qf._bind(stage, path, qf._a_mat(ctx, "plaster_dusty"))
    ctx.setdefault("notes", []).append(
        "quake partition fracture fallback (chipped whole panel): " + path)
    return [path], False


def _detached_rect_kind(ctx, path):
    """Provenance class for a simple authored member sent to physics.

    This is intentionally a positive list.  Sliced/GAC shell fragments can be
    open, UV-rich meshes and must never enter the authored-box fracture path
    merely because their bounding box happens to be rectangular.
    """
    # Prefer the authoring records over spelling.  Real kit members are named
    # things like ``bld_office_plain_roof_7_210``: checking only for a
    # ``roof_`` prefix misses the exact roof called out in review.  Fit-out
    # dictionaries are even stronger provenance because only qf authored
    # those slabs/columns/partitions.
    fit = ctx.get("fit") or {}
    if path in set((fit.get("slabs") or {}).values()):
        return "slab"
    if path in set(fit.get("partitions") or ()):
        return "partition"
    if any(path in (paths or ())
           for paths in (fit.get("columns") or {}).values()):
        return "column"
    try:
        e = _qf()._t_el(ctx, path)
    except Exception:
        e = None
    if e and e.get("role") == "roof":
        return "roof"

    p = str(path or "").lower()
    leaf = p.rsplit("/", 1)[-1]
    if "/part_" in p or leaf.startswith("part_"):
        return "partition"
    if "/slab_" in p or leaf.startswith(("slab_", "deck_")):
        return "slab"
    if "/roofslab_" in p or leaf.startswith("roofslab_"):
        return "roof"
    if "/col_" in p or leaf.startswith(("col_", "column_")):
        return "column"
    return None


def _simple_rect_info(ctx, path):
    """Shape facts for one active pristine quad/box, else ``None``.

    Eight points and six quad faces is ``quake_flow._box`` exactly; four
    points and one/two faces is a kit roof tile.  Once chipping/fracture has
    made the silhouette irregular it no longer matches, which is the gate's
    desired definition of "intact rectangle".
    """
    from pxr import UsdGeom

    stage = ctx["stage"]
    pr = stage.GetPrimAtPath(path) if path else None
    if not pr or not pr.IsValid() or not pr.IsActive() or not pr.IsA(UsdGeom.Mesh):
        return None
    if UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)):
        return None
    mesh = UsdGeom.Mesh(pr)
    pts = mesh.GetPointsAttr().Get() or ()
    cnt = [int(q) for q in (mesh.GetFaceVertexCountsAttr().Get() or ())]
    is_box = len(pts) == 8 and len(cnt) == 6 and all(q == 4 for q in cnt)
    is_quad = len(pts) == 4 and len(cnt) in (1, 2) and all(q in (3, 4) for q in cnt)
    if not (is_box or is_quad):
        return None
    qf = _qf()
    try:
        _cx, _cy, _cz, sx, sy, sz, _yaw = qf._box_dims(stage, path)
    except Exception:
        # `_box_dims` expects the authored-box xform layout.  A flat kit roof
        # is handled by `_break_box_like`, whose element record owns its size.
        from pxr import Usd
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        lo, hi = r.GetMin(), r.GetMax()
        sx, sy, sz = (float(hi[i] - lo[i]) for i in range(3))
    ext = sorted((abs(float(sx)), abs(float(sy)), abs(float(sz))))
    return {"size": (float(sx), float(sy), float(sz)), "ext": ext,
            "area": ext[1] * ext[2], "quad": is_quad}


def _oversized_detached_rect(ctx, path):
    kind = _detached_rect_kind(ctx, path)
    info = _simple_rect_info(ctx, path)
    if not kind or not info:
        return None
    e = info["ext"]
    bad = ((kind == "partition" and e[2] > 2.5)
           or (kind in ("slab", "roof")
               and (info["area"] > 16.0 or e[2] > 5.0))
           or (kind == "column" and e[2] > 4.0))
    return (kind, info) if bad else None


def _fracture_detached_rect(ctx, path, kind, info):
    """Replace one doomed pristine member with bounded irregular pieces."""
    import numpy as np

    from . import fracture

    qf = _qf()
    if kind == "partition":
        return _fracture_fallen_partition(ctx, path)[0]

    # A simple zero-thickness roof needs `_roof_box` before VTK/PhysX.  The
    # element record is the only safe way to distinguish it from a sliced
    # shell, and `_simple_rect_info` has already constrained it to four points.
    if kind == "roof" and info.get("quad"):
        e = qf._t_el(ctx, path)
        if e is None:
            return []
        n = max(8, min(32, int(round(info["area"] / 5.0))))
        seed = fracture.stable_seed("quake_detached_rect", kind, str(path))
        from . import fire_collapse as fc
        with fc._own_rng(ctx, random.Random(seed),
                         np.random.default_rng(seed)):
            return qf._break_box_like(ctx, e, n, timber=False, consume=0.12,
                                      consume_pool=1.05, max_piece_m=2.8)

    seed = fracture.stable_seed("quake_detached_rect", kind, str(path))
    area = float(info["area"])
    if kind in ("slab", "roof"):
        n = max(8, min(36, int(round(area / 6.0))))
        mat, inner, max_piece = (qf._a_mat(ctx, "concrete_dusty"),
                                 qf._a_mat(ctx, "dust"), 2.8)
    else:                         # a multi-storey / otherwise long column
        n = max(3, min(9, int(round(info["ext"][2] / 1.4))))
        mat, inner, max_piece = (qf._a_mat(ctx, "concrete_dusty"),
                                 qf._a_mat(ctx, "dust"), 1.8)
    made = qf._break_box(
        ctx["stage"], path, n, random.Random(seed),
        np.random.default_rng(seed), mat, inner, inner_p=0.22,
        mode="prism", consume=0.10, consume_pool=1.05,
        max_piece_m=max_piece)
    if made:
        _chip_pieces(ctx, made, _CHIP_SLAB if kind in ("slab", "roof")
                     else _CHIP_PRISM, tessellate=False,
                     beam=(kind in ("slab", "column")),
                     beam_keep=("a_dust",))
    return list(made)


def normalize_detached_rectangles(ctx, strict=None):
    """Enforce the no-pristine-rectangles invariant on ``ctx['loose']``.

    Runs once after every recipe (including roof-plant follow) and before the
    bake launcher sees the rigid-body list.  It changes only positive-listed,
    authored fit-out/roof members that are already doomed and oversized.
    Supported slabs and columns are never in ``loose`` and are untouched.

    Returns a report.  In strict mode (default, ``EQ_RECT_GATE=1``), any
    oversized pristine rectangle that survives a failed fracture aborts that
    archetype instead of silently publishing the defect.
    """
    if strict is None:
        strict = os.environ.get("EQ_RECT_GATE", "1").strip().lower() not in (
            "0", "false", "no", "off")
    old = list(ctx.get("loose") or ())
    replaced = []
    new_loose = []
    for path in old:
        bad = _oversized_detached_rect(ctx, path)
        if not bad:
            new_loose.append(path)
            continue
        kind, info = bad
        velocity = ctx.get("velocity", {}).pop(path, None)
        made = _fracture_detached_rect(ctx, path, kind, info)
        if not made:
            new_loose.append(path)
            continue
        replaced.append((path, kind, len(made)))
        for q in made:
            if q not in new_loose:
                new_loose.append(q)
            if velocity is not None:
                ctx.setdefault("velocity", {})[q] = velocity
        ctx["static_extra"] = [q for q in ctx.get("static_extra", ())
                               if q != path]
        fit = ctx.get("fit") or {}
        fit["all"] = [q for q in fit.get("all", ()) if q != path]
    ctx["loose"] = new_loose

    violations = []
    for path in ctx["loose"]:
        bad = _oversized_detached_rect(ctx, path)
        if bad:
            kind, info = bad
            violations.append({"path": path, "kind": kind,
                               "size": info["size"]})
    report = {"replaced": len(replaced),
              "pieces": sum(q[2] for q in replaced),
              "violations": violations, "details": replaced}
    ctx["detached_rectangles"] = report
    if replaced:
        line = ("[rect-gate] {0} detached pristine member(s) -> {1} "
                "irregular fragments; 0 survivors".format(
                    report["replaced"], report["pieces"]))
        print(line)
        ctx.setdefault("notes", []).append(line)
    if violations and strict:
        raise RuntimeError(
            "earthquake detached-rectangle gate: {0} oversized pristine "
            "member(s) survived: {1}".format(
                len(violations), ", ".join(q["path"] for q in violations[:6])))
    return report


def _fall_fitout(ctx, plan, m, prng):
    """Send down the fit-out that has lost its floor or its bearing wall.

    Three populations, and each one was a review finding once:
      * the SLABS in `plan["drop"]` (and their props, which `fit_interior`
        seats directly on them — a prop left standing on a plate that is about
        to move is `r_fire_collapse`'s own lesson);
      * everything on a storey in `plan["fit_fall"]`, region or no region: a
        storey whose whole plate went, or whose columns crushed, has nothing
        under its partitions;
      * on a storey in `plan["fit_region_storeys"]`, anything whose FOOTPRINT
        reaches into `plan["region"]` — "part_main_6_1 ... look like they're
        floating" (user, fire review). The footprint, not the centre: a
        partition is a 0.12 m sheet running the short way across the plan, so
        one framing into the lost wall has its END in the region and its
        centre several metres inside.

    Props are skipped entirely when `plan["bury_props"]` is set: a total
    collapse and a pancake put the contents of the building UNDER the pile
    (`quake_flow._a_bury_props`, called by the authoring step that owns the
    pile), and a prop that is both buried and loose is one prim in two places.
    """
    from pxr import Usd, UsdGeom
    qf = _qf()
    stage = ctx["stage"]
    fc = _fc()
    mtag = plan["mass"]
    fit = ctx.get("fit") or {}
    n = {"slab": 0, "part": 0, "part_frag": 0,
         "col": 0, "prop": 0}
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))

    for key in plan["drop"]:
        slab = (fit.get("slabs") or {}).get(key)
        if not slab:
            continue
        pr = stage.GetPrimAtPath(slab)
        if pr and pr.IsValid() and pr.IsActive() and slab not in ctx["loose"]:
            # Do not merely chip this WHOLE plate.  Chipping changes its rim
            # but leaves one building-width rigid body, which still lands as
            # the giant intact rectangle called out in review.  The common
            # end-of-recipe gate (`normalize_detached_rectangles`) sees this
            # pristine authored slab in `loose` and fractures it into bounded
            # concrete pieces before the launcher hands anything to PhysX.
            ctx["loose"].append(slab)
            n["slab"] += 1

    fall_storeys = set(int(s) for (mt, s) in plan["fit_fall"] if mt == mtag)
    region_storeys = set(int(s) for s in plan["fit_region_storeys"])
    bury = bool(plan.get("bury_props"))
    cands = []
    for p in (fit.get("partitions") or ()):
        # `part_<mass>_<storey>_<k>`; the mass tag may itself contain
        # underscores, so index from the RIGHT.
        bits = str(p).rsplit("/", 1)[-1].split("_")
        if len(bits) < 4:
            continue
        try:
            s_, mt_ = int(bits[-2]), "_".join(bits[1:-2])
        except ValueError:
            continue
        cands.append((p, mt_, s_, "part"))
    for (mt_, s_), cols in (fit.get("columns") or {}).items():
        for p in (cols or ()):
            cands.append((p, mt_, int(s_), "col"))
    if not bury:
        for (mt_, s_), props in (fit.get("props") or {}).items():
            for p in (props or ()):
                cands.append((p, mt_, int(s_), "prop"))

    xf = UsdGeom.XformCache()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for p, mt_, s_, kind in cands:
        if mt_ != mtag or not p or p in ctx["loose"]:
            continue
        if s_ not in fall_storeys and s_ not in region_storeys:
            continue
        pr = stage.GetPrimAtPath(p)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        try:
            t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        except Exception:
            continue
        sd = None
        if plan["region"] and s_ in region_storeys:
            pts = [(float(t[0]), float(t[1]))]
            try:
                bx = bc.ComputeWorldBound(pr).ComputeAlignedRange()
                if not bx.IsEmpty():
                    lo_b, hi_b = bx.GetMin(), bx.GetMax()
                    pts += [(float(lo_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(hi_b[1])),
                            (float(lo_b[0]), float(hi_b[1]))]
            except Exception:
                pass
            for wx_, wy_ in pts:
                lx, ly = qf._to_local(m, wx_, wy_)
                sd = fc.region_side(plan, m, lx, ly)
                if sd is not None:
                    break
        if sd is None and s_ not in fall_storeys:
            continue
        drop_paths = [p]
        if kind == "part":
            drop_paths, fragmented = _fracture_fallen_partition(ctx, p)
            if not drop_paths:
                continue
            if fragmented:
                n["part_frag"] += len(drop_paths)
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q != p]
        for drop_path in drop_paths:
            if drop_path not in ctx["loose"]:
                ctx["loose"].append(drop_path)
            if sd is not None and plan["throw"]:
                ox, oy = qf._outward(m, sd)
                zf = min(1.0, max(0.0,
                                  (float(t[2]) - float(m["z0"])) / Hm))
                v = plan["throw"][0] + plan["throw"][1] * zf
                ctx["velocity"][drop_path] = (ox * v, oy * v, -0.15 * v)
        n[kind] += 1
    return n


# ---------------------------------------------------------------------------
# round 5: chipping the AUTHORED cuboids on the way out
# ---------------------------------------------------------------------------
# User, on the first 500 m OSMO scene: "There's a lot of perfect rectangular
# debris. While they should look rectangular, they shouldn't look perfect ...
# use VTK to cause chips ... random from small to very large chips. Is it
# possible to make it look warped? bent?" — and then "the actual breaking
# should not be clean."
#
# Two populations here own that look, and both are boxes THIS code authored:
#   * the plank/prism cells `_author_floors` gets back from `_break_box`, and
#   * the whole floor plates `_fall_fitout` drops out of `plan["drop"]`, which
#     are `quake_flow._box`es with four ruler edges (`_a_slab_rim`'s own words).
#
# `fracture.chip_box` does the work; everything here is the USD round trip and
# — more importantly — the REFUSALS. A chip is only ever applied to a small,
# closed, UV-less mesh: that is what `_break_box`'s cells and `_box`'s plates
# are, and it is emphatically not what a SLICED building piece is. A clipped
# shell handed to this path is the `vtkStripper::GetPointCells` SIGSEGV in the
# round-4 catalogue and `quake_sliced`'s standing rule, so the guards below are
# load-bearing, not defensive padding.
#
# `EQ_RUBBLE=v2` gates it with the rest of the round-4/5 rubble work;
# `QC_CHIP=0` turns it off on its own, and with it off NOTHING here draws from
# `prng`, so a recipe's whole random stream is identical either way (the seeds
# are hashed from the prim path — `fracture.stable_seed`).
CHIP_MAX_FACES = 320       # bigger than this is not one of our boxes
#
# ROUND 6 adds `bites` (very large corner losses sized by the SECTION, not by
# the span along the cut normal) and `gouges` (scallops taken out at a station
# ALONG the piece). See the long note on `quake_rubble_usd._CHIP_KIND` for the
# measurement that forced them: a plane clip can only remove material at an
# extreme corner, so round 5 left the middle of every slender piece — and every
# straight edge run of every plate — exactly as cast.
_CHIP_GOUGE = {"gouge_depth": (0.07, 0.26), "gouge_big_p": 0.26,
               "gouge_big_frac": (0.30, 0.45), "gouge_vol_frac": 0.20,
               "gouge_budget": 760, "rough_lam_frac": 1.7,
               "bite_frac": (0.32, 0.78),
               "min_loss": 0.07, "max_loss": 0.36}
_CHIP_PLANK = dict(_CHIP_GOUGE, chips=(2, 5), depth_frac=(0.03, 0.18),
                   ends=0.50, rough_frac=0.065, bites=(1, 2), gouges=(2, 4),
                   warp_frac=0.012, twist_deg=5.0)
_CHIP_PRISM = dict(_CHIP_GOUGE, chips=(2, 5), depth_frac=(0.03, 0.16),
                   ends=0.35, rough_frac=0.055, bites=(1, 2), gouges=(2, 4))
# A SLAB IS THE STOP-SIGN CASE. Round 5 cut its four corners off and left every
# edge between them a ruler-straight line and both decks dead flat — which is
# the flat grey plate lying on the rubble mound in `eq500_v3/b0_apartment_DG5_
# obl.png`. It needs its damage on the EDGE RUNS, so it gets the most gouges of
# any kind and a bigger refinement budget to put them on.
_CHIP_SLAB = dict(_CHIP_GOUGE, chips=(3, 7), depth_frac=(0.025, 0.13),
                  ends=0.0, rough_frac=0.07, bites=(1, 3), gouges=(4, 6),
                  gouge_budget=900)
CHIP_WARP_ASPECT = 3.0     # only a genuinely long piece is bowed
CHIP_TINY_M = 0.32         # longest dimension below this: silhouette only
CHIP_SMALL_M = 0.85        # below this: a quarter of the gouge budget
#                            (see `spec_for_shape` for the population counts
#                            that make this a cost control rather than a knob)


def _chip_ok(counts, indices, npts):
    """Triangles of a mesh this module is allowed to chip, or None.

    Returns None — chip nothing — for anything that is not a small closed
    triangle/quad solid. See the section note for why the refusal matters.
    """
    import numpy as _np
    counts = [int(c) for c in (counts or ())]
    if not counts or len(counts) > CHIP_MAX_FACES:
        return None
    if any(c not in (3, 4) for c in counts):
        return None
    idx = [int(i) for i in (indices or ())]
    if sum(counts) != len(idx) or (idx and (min(idx) < 0 or max(idx) >= npts)):
        return None
    tris, k = [], 0
    for c in counts:
        tris.append((idx[k], idx[k + 1], idx[k + 2]))
        if c == 4:
            tris.append((idx[k], idx[k + 2], idx[k + 3]))
        k += c
    return _np.asarray(tris, dtype=_np.int64)


def _chip_prim(stage, path, spec, tessellate=False):
    """Chip one authored box / box fragment IN PLACE. True when it changed.

    `tessellate=True` rebuilds the mesh from its own bbox with segments before
    chipping — an 8-corner plate has nothing for a roughening pass to displace,
    so a dropped 22 x 18 m floor slab would otherwise come back with cut
    corners and four still-perfectly-straight edges.
    """
    import numpy as _np

    from pxr import Gf, Sdf, UsdGeom, Vt

    from . import fracture

    prim = stage.GetPrimAtPath(path) if path else None
    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        return False
    # A mesh with real UVs is kit/sliced art, not one of our boxes: chipping
    # would silently drop the primvar and the cladding with it.
    if UsdGeom.PrimvarsAPI(prim).HasPrimvar("st"):
        return False
    pts = mesh.GetPointsAttr().Get()
    if not pts or len(pts) < 4:
        return False
    v = _np.asarray([[float(p[0]), float(p[1]), float(p[2])] for p in pts],
                    dtype=float)
    f = _chip_ok(mesh.GetFaceVertexCountsAttr().Get(),
                 mesh.GetFaceVertexIndicesAttr().Get(), len(v))
    if f is None:
        return False

    kw = dict(spec)
    warp_frac = float(kw.pop("warp_frac", 0.0))
    twist = float(kw.pop("twist_deg", 0.0))
    seed = fracture.stable_seed(str(path))
    rng = random.Random(seed)
    ext = v.max(0) - v.min(0)
    if warp_frac > 0.0 and float(ext.max()) < CHIP_WARP_ASPECT * float(
            max(_np.sort(ext)[1], 1e-6)):
        warp_frac, twist = 0.0, 0.0        # too stubby to read as bowed
    warp_m = warp_frac * float(ext.max())

    if tessellate:
        c0 = 0.5 * (v.max(0) + v.min(0))
        nv, nf = fracture.chip_box(sizes=tuple(float(q) for q in ext), rng=rng,
                                   bottom=False, warp_m=warp_m,
                                   twist_deg=twist, **kw)
        nv = nv + (c0 - 0.5 * (nv.max(0) + nv.min(0)))   # keep the local origin
    else:
        if fracture.open_edge_count(f) > 0:
            return False
        nv, nf = fracture.chip_box(v, f, rng, warp_m=warp_m, twist_deg=twist,
                                   **kw)
        if nf is f or len(nf) == len(f) and _np.array_equal(nv, v):
            return False
    if nv is None or nf is None or len(nf) < 4:
        return False

    mesh.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in nv]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(nf)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [int(i) for tri in nf for i in tri]))
    nrm = _np.cross(nv[nf[:, 1]] - nv[nf[:, 0]], nv[nf[:, 2]] - nv[nf[:, 0]])
    ln = _np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = _np.repeat(nrm / _np.maximum(ln, 1e-12), 3, axis=0)
    mesh.CreateNormalsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in nrm]))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateExtentAttr([Gf.Vec3f(*[float(q) for q in nv.min(0)]),
                           Gf.Vec3f(*[float(q) for q in nv.max(0)])])
    # A planar `st`, for the same reason `quake_rubble_usd._chipped_box`
    # authors one: the MDL material is world-triplanar and does not need it,
    # but an offline Blender/Hydra preview has no UVs to sample the real map
    # through and renders the piece as a flat tint.
    from . import quake_rubble_usd as qru
    pv = UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(float(a), float(b))
                          for a, b in qru._planar_st(nv, nf, 0.9)]))
    return True


def spec_for_shape(size, timber=False):
    """The `_CHIP_*` table entry a piece of these dimensions should use.

    ROUND 6, and it exists because of what the census found. `fracture.
    chip_box` is wired into exactly four emitters — `quake_rubble_usd._box`
    (the rubble mound's large elements), the fit-out slabs a collapse drops,
    the cells `quake_flow._break_box` leaves, and `quake_sliced`'s slabs /
    columns / partitions. Every OTHER authored cuboid in the earthquake path
    is emitted by `quake_flow._box` and never sees a chip: `_p_lintels`
    (3-6 lintel / quoin / coping bars per masonry heap — the straight-edged
    bars in the DG5 shots), `_disturb_interior`, `_c_clods`, `_c_kerb`,
    `_c_fissures`, `_c_overturn_ground`, `_b_crumbs`, `_d_chunk`,
    `_d_face_band`, `_shaft`, `_buckled_pavement`, `fit_interior`'s standing
    columns and piers, `_roof_box` and `r_signage_fail`. `grep -c chip
    quake_flow.py` is 1, and that one hit is a comment.

    Wiring any of them is one line — `_chip_pieces(ctx, made,
    spec_for_shape(size))` after the emitter's own `ctx["loose"] += made` —
    because `_chip_prim`'s refusal ladder already rejects anything that is not
    one of our own small, closed, UV-less boxes. This picks the table entry so
    that call site does not have to know about the tables:

      plate-like (thinnest axis under a third of the next)  -> `_CHIP_SLAB`
      timber                                                -> `_CHIP_PLANK`
      everything else (bars, stubs, blocks)                 -> `_CHIP_PRISM`

    THE SIZE LADDER IS THE COST CONTROL, and without it this wiring is
    unaffordable. `quake_flow`'s unchipped populations are not a handful of
    lintels: `_disturb_interior` alone authors `W*D/100*9` litter boxes PER
    STOREY (about 27 on a 20 x 15 m mass, so 130-190 for a seven-storey
    building), `_b_crumbs` 5-12 per loss patch, `_c_kerb` one per 1.25 m of
    kerb line. At the full 760-triangle gouge budget a single building would
    pay several hundred thousand triangles and seconds of authoring time for
    pieces that are 6-55 cm across — sub-pixel at survey altitude and already
    reading as crumbs at contact range.

    So the budget follows the piece's LONGEST dimension:

      under `CHIP_TINY_M`   chips and end steps only, no gouge pass, no
                            refinement — a 30-60 triangle piece whose
                            SILHOUETTE is no longer a rectangle, which is all
                            that can read at that size;
      under `CHIP_SMALL_M`  a gouge pass on a quarter budget;
      above it              the full table entry, unchanged.

    Measured on the ladder: a 0.18 m crumb 42 triangles, a 0.45 m litter box
    186, a 1.8 m lintel 310-714.
    """
    ext = sorted(abs(float(q)) for q in size)
    if timber:
        base = _CHIP_PLANK
    elif ext[0] < 0.34 * max(ext[1], 1e-9):
        base = _CHIP_SLAB
    else:
        base = _CHIP_PRISM
    if ext[2] >= CHIP_SMALL_M:
        return base
    if ext[2] >= CHIP_TINY_M:
        return dict(base, gouges=(1, 2), gouge_budget=max(
            200, int(base.get("gouge_budget", 760)) // 4))
    return dict(base, gouges=(0, 0), bites=(1, 1), rough_frac=0.0)


def _chip_pieces(ctx, paths, spec, tessellate=False, beam=False, beam_keep=()):
    """Chip a list of authored prims. Returns how many actually changed.

    `beam` also re-binds each chipped piece to the Damaged_Concrete_Floor scan
    (`quake_rubble_usd._beam_look`) — the round-5 addendum's material for
    broken concrete. It is skipped for a piece whose current binding is named
    in `beam_keep`: `_break_box` deliberately puts 45 % of a broken deck's
    cells on the mortar-dust tint for variety, and flattening that to one
    surface would undo it. `_beam_look` returns None when the megascans pack
    is not on disk, in which case every piece keeps what it had and the chips
    still happen.
    """
    from . import fracture
    qf = _qf()
    if qf._RUBBLE_MODE != "v2" or not fracture.chips_enabled():
        return 0
    from pxr import UsdShade

    from . import quake_rubble_usd as qru
    stage = ctx["stage"]
    bmat = None
    if beam:
        try:
            bmat = qru._beam_look(stage, ctx["parent"], ctx.get("mats"))
        except Exception as exc:
            print("[quake_collapse] beam look unavailable: {0}".format(exc))
    n = 0
    for p in (paths or ()):
        try:
            if not _chip_prim(stage, p, spec, tessellate=tessellate):
                continue
            n += 1
            if bmat is None:
                continue
            pr = stage.GetPrimAtPath(p)
            cur = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
            name = cur.GetPath().name if (cur and cur.GetPrim().IsValid()) else ""
            if name not in beam_keep:
                UsdShade.MaterialBindingAPI.Apply(pr).Bind(bmat)
        except Exception as exc:                 # a chip is cosmetic, always
            print("[quake_collapse] chip skipped on {0}: {1}".format(p, exc))
    return n


def _author_floors(ctx, plan, m, prng):
    """Break the floor slabs a TOTAL collapse brings down, and bury what was
    standing on them.

    `r_masonry_collapse`'s own treatment, and its own numbers: a joisted
    timber floor comes apart into 54-70 plank cells with `consume` 0.88 at
    `consume_pool` 1.05 (so the thinning takes the BIGGEST pieces), bound to
    the dusty joist tint with 45 % of them in mortar dust instead. Round 2:
    "the DG5 heaps read as a LUMBER YARD" — 30-40 cells out of a 22 x 18 m
    deck are 3.5 x 1.2 m boards, five storeys of them cover the brick
    completely, and bound to the pale `timber` map they are the brightest
    thing in the shot.

    NOT `_dust_loose`. Every material this authors is already an `A_DEBRIS`
    flat colour, calibrated in LINEAR albedo; `diffuse_tint` MULTIPLIES, so
    tinting one a second time is the near-black bug the round-4 catalogue
    records for `_material_for_look` and round 2 records for
    `planks.wood_material`. `_dust_loose`'s own docstring makes the same
    exclusion for the same reason.
    """
    qf = _qf()
    stage = ctx["stage"]
    fit = ctx.get("fit") or {}
    mtag = plan["mass"]
    timber = (plan["btype"] == "urm")
    n = 0
    n_chip = 0
    for key in plan["break_slabs"]:
        pth = (fit.get("slabs") or {}).get(key)
        if not pth:
            continue
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        made = qf._break_box(
            stage, pth, 54 + prng.randrange(16), prng, ctx["nrng"],
            qf._a_mat(ctx, "timber_dusty" if timber else "concrete_dusty"),
            qf._a_mat(ctx, "brick_dusty"), 0.45,
            mode=("plank" if timber else "prism"),
            aspect=((1.8, 3.6) if timber else None), consume=0.88,
            consume_pool=1.05, max_piece_m=1.0)
        # ROUND 5. `mode="prism"` is a brick lattice and `mode="plank"` a board
        # lattice — both give near-cuboid Voronoi cells, which is exactly the
        # "perfect rectangular debris" the user called out. Chip the corners
        # and the break ends off each; a PLANK also bows (`warp_frac`), a
        # concrete prism does not. BEFORE `_a_lay_flat`, so a piece is laid on
        # its real chipped extent rather than on the cuboid's.
        # `beam` only for the CONCRETE deck: a broken RC floor is exactly the
        # Damaged_Concrete_Floor scan, a broken joisted timber one is not.
        # `beam_keep` preserves `_break_box`'s own 45 % mortar-dust variety.
        n_chip += _chip_pieces(ctx, made,
                               _CHIP_PLANK if timber else _CHIP_PRISM,
                               beam=not timber,
                               beam_keep=("a_brick_dusty",))
        qf._a_lay_flat(ctx, made, p=0.9)
        ctx["loose"] += made
        fit["all"] = [q for q in fit["all"] if q != pth]
        n += len(made)
    if plan.get("bury_props"):
        # UNDER the pile, not on it. A pancake's contents are under the slab
        # STACK, so the burial depth is the stack's own; a total collapse's
        # are under a heap of H/3 (FEMA's air-space factor).
        st = plan.get("stack")
        if st:
            base, depth, keep = (float(st["base_z"]),
                                 st["pitch_m"] * plan["n_levels"] * 0.6, 0.25)
        else:
            H = max(3.0, float(m["top"]) - float(m["z0"]))
            base, depth, keep = float(m["z0"]), H * DOME_CROWN_FRAC, 0.3
        for (pm, _i), props in (fit.get("props") or {}).items():
            if pm == mtag:
                qf._a_bury_props(ctx, props, base, depth, keep=keep)
    if n_chip:
        from . import fracture
        ctx.setdefault("notes", []).append(
            "[chip] floor cells: {0} chipped, {1} passed-through "
            "(vtk={2})".format(n_chip, n - n_chip, fracture.chips_enabled()))
    return n


def _author_band(ctx, plan, m, prng, snap_before_tears):
    """Move the block above a crushed storey.

    `quake_flow._soft_storey_matrix(plan["crush"]["geo"])` and NOTHING
    RE-DERIVED. The mid-storey signature (a plan twist and an offset) is
    composed on top exactly as `r_soft_storey` composes it.

    THE TORN STATICS OF THE STOREY ABOVE HAVE TO RIDE DOWN WITH IT. They are
    not `_els` any more (`_tear_perimeter` marks the module dead), so the only
    way to find them is a diff of what this recipe has authored: everything
    new in `static_extra` whose world z sits at or above the band's ceiling
    belongs to the block. Leave them behind and the seam grows a row of
    fragments hanging where the storey used to be.

    SO DOES THE ROOF PLANT (round-6 fix). `ctx["roof_plant"]` /
    `ctx["roof_fixed"]` — the tanks and AC units `quake_flow.dress_roof`
    seats on THIS mass's own roof — are in neither `_els` (structural kit
    elements only) nor any `fit[...]` collection above, so before this fix
    they rode NOTHING: the real roof (an `_els` element) came down in
    `above` while the tank standing on it stayed exactly where `dress_roof`
    put it, floating once the block below it dropped away. That is why
    `ROOF_PROP_MODES` could leave `soft_storey`/`mid_storey` out — the claim
    that this function already carries "whatever sits on the roof" used to
    be false; it is asserted here and enforced by the same z-threshold test
    every other category above uses, not merely claimed in a comment.
    """
    from pxr import UsdGeom
    fc, qf = _fc(), _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    fit = ctx.get("fit") or {}
    c = plan["crush"]
    mtag = plan["mass"]
    k, z_hi, z_lo = c["storey"], c["z_hi"], c["z_lo"]
    ox, oy = qf._outward(m, c["lean_side"])
    xf = UsdGeom.XformCache()

    above = []
    for e in qf._els(ctx, mass=None):
        em = info["masses"].get(e["mass"]) or m
        if e["mass"] == mtag or e["mass"].startswith(mtag + "_") or (
                mtag == "main" and e["mass"] != "main"):
            if float(e["z"]) >= z_hi - 0.05 or (
                    e["role"] in ("roof",) and float(em["top"]) >= z_hi):
                above.append((e.get("p") or {}).get("prim_path"))
    for (mt, i), pth in (fit.get("slabs") or {}).items():
        if i > k or (mt != mtag and mt.startswith(mtag)) or (
                mtag == "main" and mt != "main"):
            above.append(pth)
    for (mt, i), cols in (fit.get("columns") or {}).items():
        if i > k or (mtag == "main" and mt != "main"):
            above.extend(cols)
    for pth in (fit.get("partitions") or ()):
        try:
            i = int(str(pth).rsplit("_", 2)[-2])
        except ValueError:
            i = 0
        if i > k:
            above.append(pth)
    for (mt, i), props in (fit.get("props") or {}).items():
        if i > k or (mtag == "main" and mt != "main"):
            above.extend(props)

    # THE ROOF PLANT RIDES DOWN TOO. `dress_roof` seats tanks/AC units on
    # THIS mass's own roof (`ctx["roof_plant_mass"]`, always "main" in
    # practice) BEFORE any recipe runs, and until this fix nothing here ever
    # looked at them: a soft/mid-storey crush moved the real roof (it's
    # already in `above` above, via the `role in ("roof",)` branch) out from
    # under a tank that stayed exactly where `dress_roof` put it — measured
    # on `bld_office_wide_DG4.usd`, roof at 22 m, tank frozen at 17.3-19.9 m
    # (between the two lower slabs a later, unrelated physics pass dropped it
    # to). Same z-threshold test as every other category above: this mass,
    # at or above the crush ceiling. Anything already resolved (kicked loose
    # or buried by `_sweep_roof_props`) is already filtered by the `in
    # ctx["loose"]` check below, same as everything else in `above`.
    if mtag == ctx.get("roof_plant_mass", "main"):
        for pth in (list(ctx.get("roof_plant") or ())
                    + list(ctx.get("roof_fixed") or ())):
            if not pth:
                continue
            pr = stage.GetPrimAtPath(pth)
            if not pr or not pr.IsValid() or not pr.IsActive():
                continue
            try:
                t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
            except Exception:
                continue
            if float(t[2]) >= z_hi - 0.05:
                above.append(pth)

    # the new statics the tears left, split by height
    _new_lo, new_st = fc._new_since(ctx, snap_before_tears)
    for pth in new_st:
        pr = stage.GetPrimAtPath(pth) if pth else None
        if not pr or not pr.IsValid():
            continue
        try:
            t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        except Exception:
            continue
        if float(t[2]) >= z_hi - 0.05:
            above.append(pth)

    seen, out = set(), []
    for a in above:
        if not a or a in seen or a in ctx["loose"]:
            continue
        seen.add(a)
        out.append(a)
    M = qf._soft_storey_matrix(c["geo"])
    if c["twist_deg"] or c["offset_m"]:
        M = M * qf._rot_about((m["cx"], m["cy"], z_lo + c["crush_m"]),
                              (0.0, 0.0, 1.0), float(c["twist_deg"])) \
            * qf._translate(ox * float(c["offset_m"]),
                            oy * float(c["offset_m"]), 0.0)
    qf._transform_prims(stage, out, M)
    ctx["static_extra"] += out
    return len(out)


def _author_stack(ctx, plan, m, prng):
    """Re-author the slabs of a pancaked frame as a STACK.

    `r_pancake`'s own arithmetic, kept: each plate loses one to four sides
    along a wandering line (a pancaked slab does not keep its formwork edge —
    the first pass left a deck of cards seen from the air), then translates to
    its pitch height with a small tilt and jitter, then takes the dark roof
    map (the fit-out binds slabs with a pale pavement map, which reads from
    the air as a sheet of bathroom tile on the crown of the pile).
    """
    from pxr import UsdShade
    qf = _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    fit = ctx["fit"]
    mtag = plan["mass"]
    st_plan = plan["stack"]
    n_plates = 0
    stack = []
    for pl in st_plan["plates"]:
        key = (mtag, pl["storey"])
        pth = (fit.get("slabs") or {}).get(key)
        if pth is None:
            continue
        cur, sts = pth, []
        for sd in pl["sides"]:
            km = UsdShade.MaterialBindingAPI(
                stage.GetPrimAtPath(cur)).ComputeBoundMaterial()[0]
            d = prng.uniform(0.5, 1.6)
            rem, strip = qf._split_strip(ctx, cur, m, sd, d + 1.5,
                                         ctx["mats"]["concrete"])
            st, lo = qf._break_split(
                ctx, strip, 10 + prng.randrange(4),
                qf._p_edge_judge(m, sd, d, prng, btype=info["type"]),
                qf._mat_fn(ctx, None, 1.0),
                min_volume_frac=0.0008, static_mat=km if km else None,
                refine_max=6, max_loose_m=2.6, rough=qf.ROUGH_STRIP_M)
            cur, sts = rem, sts + st
            qf._dust_loose(ctx, lo)
            ctx["loose"] += lo
            ctx["authored"].append(rem)
        group = [cur] + sts
        fit["slabs"][key] = cur
        z = float(pl["z"])
        i = int(pl["storey"])
        n_lv = plan["n_levels"]
        self_z = (m["levels"][i] - qf.SLAB_T[info["type"]] / 2.0 if i < n_lv
                  else float(m["top"]) - qf.SLAB_T[info["type"]] / 2.0)
        jx, jy = pl["jitter"]
        M = qf._translate(jx, jy, z - self_z) * qf._rot_about(
            (m["cx"], m["cy"], z), pl["axis"], pl["tilt_deg"])
        qf._transform_prims(stage, group, M)
        for q in group:
            qf._bind(stage, q, qf._a_roof_mat(ctx))
        stack += group
        n_plates += 1
        for _ in range(2):
            qf._rebar_tuft(ctx, group[0], z + 0.1, n=2, length=(0.4, 1.0))
    ctx["static_extra"] += stack

    # the roof lands on top, tilted
    top_z = float(st_plan["top_z"])
    for e in list(qf._els(ctx, mass=mtag, role="roof")):
        pth = qf._roof_box(ctx, e)
        if not pth:
            continue
        M = qf._translate(prng.uniform(-0.8, 0.8), prng.uniform(-0.8, 0.8),
                          top_z - float(e["z"])) * qf._rot_about(
            (m["cx"], m["cy"], top_z),
            ((1.0, 0.0, 0.0) if prng.random() < 0.5 else (0.0, 1.0, 0.0)),
            float(st_plan["roof_tilt_deg"]))
        qf._transform_prims(stage, [pth], M)
        ctx["static_extra"].append(pth)
        e["dead"] = True
    # (the props were buried by `_author_floors`, which owns `bury_props`
    # for both total and pancake — burying them twice moves them twice)
    return n_plates


def _sweep_statics(ctx, plan, m, pre_static):
    """The position sweep, RESTRICTED IN PLAN (elevation / corner only).

    "Everything static above the failure line" is right when the whole top of
    a block is coming down and catastrophic here: it would drop the far half
    of the roof deck, its plant and the walls on three untouched elevations.
    The reason for a POSITION test at all still holds (the roof is `_roof_box`
    slabs, then `_split_strip` remainders, then whatever a recipe authored,
    each in a different list or in none), so the test gains a second
    predicate: above the failure line AND inside `plan["region"]` — and it
    only ever looks at statics that existed BEFORE this recipe ran, because
    everything the recipe itself adds is the part that STAYS UP.
    """
    from pxr import UsdGeom
    fc, qf = _fc(), _qf()
    stage = ctx["stage"]
    xf = UsdGeom.XformCache()
    keep, moved = [], []
    for pth in list(ctx["static_extra"]):
        if pth not in pre_static:
            keep.append(pth)
            continue
        pr = stage.GetPrimAtPath(pth) if pth else None
        if not pr or not pr.IsValid() or not pr.IsActive():
            keep.append(pth)
            continue
        try:
            t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        except Exception:
            keep.append(pth)
            continue
        lx, ly = qf._to_local(m, float(t[0]), float(t[1]))
        if float(t[2]) > plan["cut_z"] and fc.in_region(plan, m, lx, ly):
            moved.append(pth)
        else:
            keep.append(pth)
    ctx["static_extra"] = keep
    n = 0
    for pth in moved:
        if pth not in ctx["loose"]:
            ctx["loose"].append(pth)
            n += 1
    return n


def roof_prop_footprint_lost(plan, m, footprint):
    """Does the MAJORITY of `footprint` sit over roof area this plan killed?

    `footprint` is an iterable of world `(x, y)` points describing a roof
    prop's own extent — typically its four footprint corners plus its
    centre, the same shape `quake_flow._mostly_in_hole` already tests a
    rooftop item's hole membership with (`r_roof_hole`), and for the same
    reason the fire skill records as bug 9: THE CENTRE ALONE IS NOT ENOUGH.
    A housekeeping pad several metres across can have its centre resting on
    a live cell while most of its own footprint hangs over the part that
    came down, or the other way round.

    `total` / `pancake`: the whole roof of THIS mass is gone, so every point
    on it answers yes — there is no surviving part of this mass's roof to
    tell a prop's footprint from.
    `elevation` / `corner`: majority of points inside `plan["region"]`, the
    same near-wall strip `_sweep_statics` and `_fall_fitout` already test a
    static prim or a fit-out item's position against.
    Any other mode: this family never kills the roof there, so nothing on it
    is ever "lost" by this test.

    Pure geometry — no `pxr`, no stage — so a test can hand it a synthetic
    footprint built from `quake_flow._to_world(m, lx, ly)` and check the
    call it makes without ever touching a stage.
    """
    fc = _fc()
    qf = _qf()
    pts = list(footprint or ())
    if not pts:
        return False
    mode = plan.get("mode")
    if mode in ("total", "pancake"):
        return True
    if mode not in ("elevation", "corner"):
        return False
    if not plan.get("region"):
        return False
    n_in = 0
    for wx, wy in pts:
        lx, ly = qf._to_local(m, wx, wy)
        if fc.in_region(plan, m, lx, ly):
            n_in += 1
    return n_in * 2 > len(pts)


# Same threshold `tools/fc_roof_deck_probe.py` uses for "is this triangle
# part of a deck": area-weighted normal, keep the ones facing enough like
# straight up. Shared by name so the authoring-time probe below and the
# offline verifier (`tools/roof_plant_seat_probe.py`) can never quietly
# drift apart on what counts as "support."
ROOF_PROP_UP_THRESHOLD = 0.72


def _deck_support_z(stage, root, cx, cy, half_w, half_d, z_ceiling,
                    exclude=(), margin=ROOF_PROP_SUPPORT_MARGIN_M,
                    up_threshold=ROOF_PROP_UP_THRESHOLD, candidates=None):
    """The highest Z of any UPWARD-FACING triangle under a prop's own
    footprint — the `tri_soup` idiom `tools/fc_roof_deck_probe.py` already
    uses to answer "does the roof actually reach up there", run live against
    the STAGE mid-authoring instead of an exported file, so a fallen prop can
    be placed on whatever is REALLY there instead of handed a kick and a
    step budget and hoped for (round-5's mechanism, and round-5's bug: it
    already failed silently once — `bld_brownstone_row_DG3.usd` froze ~2 m
    short).

    `candidates` (round 8, `SETTLE_BODY_BUDGET`'s own perf fix): an OPTIONAL
    precomputed list from `_deck_support_candidates(stage, some_root)` —
    walks that already-built list instead of `stage.Traverse()`, skipping
    the per-mesh world-transform and fan-triangulation this function would
    otherwise redo on EVERY call (the expensive, QUERY-INDEPENDENT half of
    its own work — a caller running many footprint queries against the same
    static scope, `apply_settle_budget`'s one call per over-budget piece,
    pays for that once instead of once per query). Every QUERY-dependent
    test — `exclude`, the Z/XY AABB prune, `up_threshold`, `margin`, the
    5-point containment test — still runs exactly as it does on a live
    traversal, against the SAME precomputed per-triangle data a fresh
    traversal would compute fresh, so the answer is byte-identical to
    calling this function with `candidates=None` against the same stage
    state (`test_deck_support_z_cache_is_byte_identical_to_uncached` pins
    this). `None` (the default — every one of this function's other FOUR
    call sites, `_sweep_roof_props` here, `_settle_foundation_roof_plant` in
    `quake_flow`, the sliced-path roof sweep in `quake_sliced`, and `tools/
    roof_plant_seat_probe.py`, none of which pass this) runs the ORIGINAL
    `stage.Traverse()` loop below completely unchanged.

    Every Mesh under `root` (one building's own scope, never the whole
    stage) is a candidate; a cheap per-prim AABB prune runs first (most of a
    fractured building is nowhere near one prop's footprint) and only
    survivors pay for the full triangle pass. `exclude` keeps a prop from
    reporting ANOTHER unresolved prop as its own floor.

    ROUND 7: the prune's Z test is on the mesh's BOTTOM (`lo[2]`), never its
    top. A merged GAC/kit prim can legitimately span multiple heights — a
    coping run or a raised section elsewhere on the SAME prim sitting well
    above a genuine deck under THIS footprint (measured on a real bake:
    `roof_x_0_14_0271` in `gac_SM_Building_19_DG1_*.usd` spans
    z=[68.549, 70.196], real deck triangle at 69.276) — and pruning on the
    mesh's top threw the whole candidate out whenever ANY part of it cleared
    the ceiling, silently returning the wrong (or no) support under exactly
    the merged prims this idiom exists to handle correctly. Only a mesh
    whose entire z-range sits above `z_ceiling + margin` can be ruled out
    without reading a single triangle; the XY rectangle-overlap tests were
    always correct and are unchanged. `quake_sliced._reseat_roof_plant`
    faced the same shape of bug on the seat side (a "ceiling" derived from
    the ADVERTISED seat, not the query) and dropped its ceiling test
    entirely rather than tune the margin — not an option here, since this
    function sweeps the WHOLE building scope per call (`tools/roof_plant_
    seat_probe.py` sweeps every archetype file) and an unbounded triangle
    pass over every mesh regardless of height would cost real wall-clock.

    Returns the resolved Z, or `None` if nothing upward-facing is under the
    footprint at all (a genuine hole all the way down — the caller falls
    back to this mass's own ground, `m["z0"]`).
    """
    import numpy as np
    from pxr import Usd, UsdGeom

    excl = set(str(p) for p in exclude)

    if candidates is not None:
        # CACHED PATH — every candidate's world AABB / triangle arrays were
        # already computed once by `_deck_support_candidates`; only the
        # QUERY-dependent tests run here, on those same arrays, so this
        # branch answers exactly what the traversal below would for the
        # same (cx, cy, half_w, half_d, z_ceiling, margin, up_threshold).
        best = None
        for path, lo, hi, A, B, C, nz, tz in candidates:
            if path in excl:
                continue
            if (lo[2] > z_ceiling + margin or hi[0] < cx - half_w
                    or lo[0] > cx + half_w or hi[1] < cy - half_d
                    or lo[1] > cy + half_d):
                continue
            if len(A) == 0:
                continue
            cand = (nz > up_threshold) & (tz <= z_ceiling + margin)
            ci = np.nonzero(cand)[0]
            if not len(ci):
                continue
            qx = (cx - half_w, cx + half_w, cx + half_w, cx - half_w, cx)
            qy = (cy - half_d, cy - half_d, cy + half_d, cy + half_d, cy)
            Ax, Ay = A[ci, 0], A[ci, 1]
            Bx, By = B[ci, 0], B[ci, 1]
            Cx, Cy = C[ci, 0], C[ci, 1]
            hit = np.zeros(len(ci), dtype=bool)
            for px, py in zip(qx, qy):
                d1 = (px - Bx) * (Ay - By) - (Ax - Bx) * (py - By)
                d2 = (px - Cx) * (By - Cy) - (Bx - Cx) * (py - Cy)
                d3 = (px - Ax) * (Cy - Ay) - (Cx - Ax) * (py - Ay)
                neg = (d1 < 0) | (d2 < 0) | (d3 < 0)
                pos = (d1 > 0) | (d2 > 0) | (d3 > 0)
                hit |= ~(neg & pos)
            tz_hit = tz[ci][hit]
            if len(tz_hit):
                cbest = float(tz_hit.max())
                if best is None or cbest > best:
                    best = cbest
        return best

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    root = str(root)
    under_root = root in ("", "/")   # the pseudo-root: every prim qualifies,
                                     # never just the ones literally AT "/"
    best = None
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        p = str(prim.GetPath())
        if p in excl or not (under_root or p == root or p.startswith(root + "/")):
            continue
        rng_ = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_.IsEmpty():
            continue
        lo, hi = rng_.GetMin(), rng_.GetMax()
        # Z-PRUNE ON THE MESH'S BOTTOM, NEVER ITS TOP. A merged prim's own
        # AABB can span multiple heights when a coping run or raised section
        # ELSEWHERE on the same prim sits well above a genuine deck under
        # THIS footprint — measured on a real bake: GAC `roof_x_0_14_0271`
        # in `gac_SM_Building_19_DG1_*.usd` spans z=[68.549, 70.196], but its
        # real up-facing deck triangle under the query footprint is at
        # exactly 69.276. The OLD prune (`hi[2] > z_ceiling + margin`) threw
        # the whole mesh out whenever its TOP cleared the ceiling, even
        # though the triangle pass below would have found the lower deck —
        # a false "no support" (or a wrong, lower support) under exactly the
        # merged prims this idiom exists to handle. The only thing that
        # legitimately rules a mesh OUT is its z-range never reaching
        # `(-inf, z_ceiling + margin]` at all, i.e. its BOTTOM is already
        # above the ceiling: nothing inside it can then be at or below
        # `z_ceiling`. The XY tests are an ordinary rectangle-overlap check
        # (footprint vs. mesh AABB) and were never the bug.
        if (lo[2] > z_ceiling + margin or hi[0] < cx - half_w
                or lo[0] > cx + half_w or hi[1] < cy - half_d
                or lo[1] > cy + half_d):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        M = np.array(xc.GetLocalToWorldTransform(prim), dtype=np.float64)
        P = np.array([[q[0], q[1], q[2]] for q in pts], dtype=np.float64)
        P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
        counts = np.asarray(counts)
        idx = np.asarray(idx)
        off = np.concatenate([[0], np.cumsum(counts)[:-1]])
        # THE QUERY RECTANGLE, NOT THE TRIANGLE'S CENTROID, HAS TO BE THE
        # ONE TESTED FOR CONTAINMENT. A kit slab is authored as ONE big quad
        # (`_box`'s own shape, `_roof_box`, every floor/roof piece in this
        # file), and a fan triangulation of a wide quad puts each triangle's
        # centroid well off to one SIDE of the quad's own middle — a 30 m
        # slab's two triangles centre at roughly a quarter and three-
        # quarters along its length, never at 0. Testing "is the centroid
        # inside the prop's small footprint" against a face many times that
        # footprint's size is therefore never true, and every real deck
        # silently reads as no support at all (caught by
        # `tests/test_quake_collapse.py::test_deck_support_z_finds_a_wide_
        # slab_not_just_its_triangle_centroid` on exactly this shape, before
        # this fix). The other way round instead: the same 4-corners-plus-
        # centre 5 points `roof_prop_footprint_lost` already samples a prop's
        # OWN footprint with, tested for point-in-triangle against each
        # candidate face — right whichever one is bigger.
        qx = (cx - half_w, cx + half_w, cx + half_w, cx - half_w, cx)
        qy = (cy - half_d, cy - half_d, cy + half_d, cy + half_d, cy)
        # VECTORISED FAN TRIANGULATION (ROUND 7 PERF FIX). The Z-prune fix
        # above legitimately keeps far more meshes as candidates now — any
        # mesh whose BOTTOM could hold up the prop, not just ones whose
        # WHOLE AABB happened to fit under the ceiling — including, per
        # building, whatever single big merged/LOD proxy mesh used to be
        # pruned out wholesale by the old (wrong) test. Measured: with the
        # scalar per-triangle Python loop this used to be, that alone took
        # `tools/roof_plant_seat_probe.py` from ~2.3 s/file to ~5.3 s/file
        # on `bld_apartment_DG*` (one newly-surviving 2754-face mesh,
        # re-walked in pure Python once per roof prop). Grouping faces by
        # vertex count and doing the up-face + ceiling + containment test
        # as numpy array ops instead of a Python loop per triangle per
        # query point brings it back down — same fan order, same
        # `up_threshold` / `margin` / point-in-triangle test as the scalar
        # version this replaces, so WHAT counts as support is unchanged,
        # only how fast it is computed.
        if len(counts):
            tri_a, tri_b, tri_c = [], [], []
            for fn in np.unique(counts[counts >= 3]):
                fn = int(fn)
                face_off = off[counts == fn]
                verts = idx[face_off[:, None] + np.arange(fn)[None, :]]
                a_idx = verts[:, 0]
                for k in range(1, fn - 1):
                    tri_a.append(a_idx)
                    tri_b.append(verts[:, k])
                    tri_c.append(verts[:, k + 1])
            if tri_a:
                ta, tb, tc = (np.concatenate(g) for g in (tri_a, tri_b, tri_c))
                A, B, C = P[ta], P[tb], P[tc]
                nrm = np.cross(B - A, C - A)
                ar = 0.5 * np.linalg.norm(nrm, axis=1)
                ok = ar > 1e-9
                nz = np.zeros(len(ar))
                nz[ok] = nrm[ok, 2] / (2.0 * ar[ok])
                up = ok & (nz > up_threshold)
                tz = (A[:, 2] + B[:, 2] + C[:, 2]) / 3.0
                cand = up & (tz <= z_ceiling + margin)
                ci = np.nonzero(cand)[0]
                if len(ci):
                    Ax, Ay = A[ci, 0], A[ci, 1]
                    Bx, By = B[ci, 0], B[ci, 1]
                    Cx, Cy = C[ci, 0], C[ci, 1]
                    hit = np.zeros(len(ci), dtype=bool)
                    for px, py in zip(qx, qy):
                        d1 = (px - Bx) * (Ay - By) - (Ax - Bx) * (py - By)
                        d2 = (px - Cx) * (By - Cy) - (Bx - Cx) * (py - Cy)
                        d3 = (px - Ax) * (Cy - Ay) - (Cx - Ax) * (py - Ay)
                        neg = (d1 < 0) | (d2 < 0) | (d3 < 0)
                        pos = (d1 > 0) | (d2 > 0) | (d3 > 0)
                        hit |= ~(neg & pos)
                    tz_hit = tz[ci][hit]
                    if len(tz_hit):
                        cbest = float(tz_hit.max())
                        if best is None or cbest > best:
                            best = cbest
    return best


def _deck_support_candidates(stage, root):
    """Precompute `_deck_support_z`'s own per-mesh world-space triangle data
    for every Mesh under `root`, ONCE — feed the result to that function's
    `candidates=` parameter instead of letting it re-traverse the stage and
    re-triangulate every mesh on every call. `apply_settle_budget` (below)
    is the reason this exists: many footprint queries against the SAME
    static scope, one per over-budget piece, used to each pay for a full
    `stage.Traverse()` plus a fresh world-transform + fan-triangulation of
    every candidate mesh — the traversal and the triangulation are exactly
    as expensive per query as they are per stage, since neither depends on
    the query's own (cx, cy, half_w, half_d, z_ceiling); this factors that
    query-INDEPENDENT half out so it runs once per root instead of once per
    piece.

    Degenerate triangles (`_deck_support_z`'s own `ok = ar > 1e-9` gate) are
    DROPPED here rather than carried through and re-masked at query time —
    a degenerate triangle's normal is the zero vector, so `nz` (its Z/area
    ratio) is 0 and `nz > up_threshold` can only ever be `True` for a
    caller-supplied `up_threshold < 0`, which nothing in this codebase ever
    passes and `_deck_support_z`'s own `up = ok & (nz > up_threshold)` rules
    out unconditionally regardless of threshold — so dropping them here
    changes nothing any query could ever observe.

    Returns `[(path, lo, hi, A, B, C, nz, tz), ...]`:
      `lo`/`hi`   the mesh's world AABB corners (3-tuples of float) —
                  `_deck_support_z`'s own per-candidate prune reads these
                  exactly the way it always has, just precomputed;
      `A`/`B`/`C` the world-space triangle corner arrays (already
                  transformed, already fan-triangulated);
      `nz`        each triangle's normal-Z / (2 x area) ratio — compared
                  against a QUERY's own `up_threshold` inside
                  `_deck_support_z`, never baked in here;
      `tz`        each triangle's mean world Z.

    `root` is baked in at build time (a cache is only ever valid for the
    root it was built for — `apply_settle_budget` keys its own cache dict by
    root string for exactly this reason). `exclude` is deliberately NOT
    baked in: it is applied fresh, per query, inside `_deck_support_z`
    itself (a `path in excl` check against this list's own `path` field),
    since which paths are excluded is a per-QUERY decision, not a property
    of the candidate set.
    """
    import numpy as np
    from pxr import Usd, UsdGeom

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    root = str(root)
    under_root = root in ("", "/")
    out = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        p = str(prim.GetPath())
        if not (under_root or p == root or p.startswith(root + "/")):
            continue
        rng_ = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_.IsEmpty():
            continue
        lo, hi = rng_.GetMin(), rng_.GetMax()
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        counts_a = np.asarray(counts)
        if not len(counts_a):
            continue
        idx_a = np.asarray(idx)
        off = np.concatenate([[0], np.cumsum(counts_a)[:-1]])
        M = np.array(xc.GetLocalToWorldTransform(prim), dtype=np.float64)
        P = np.array([[q[0], q[1], q[2]] for q in pts], dtype=np.float64)
        P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
        tri_a, tri_b, tri_c = [], [], []
        for fn in np.unique(counts_a[counts_a >= 3]):
            fn = int(fn)
            face_off = off[counts_a == fn]
            verts = idx_a[face_off[:, None] + np.arange(fn)[None, :]]
            a_idx = verts[:, 0]
            for k in range(1, fn - 1):
                tri_a.append(a_idx)
                tri_b.append(verts[:, k])
                tri_c.append(verts[:, k + 1])
        if not tri_a:
            continue
        ta, tb, tc = (np.concatenate(g) for g in (tri_a, tri_b, tri_c))
        A, B, C = P[ta], P[tb], P[tc]
        nrm = np.cross(B - A, C - A)
        ar = 0.5 * np.linalg.norm(nrm, axis=1)
        ok = ar > 1e-9
        if not np.any(ok):
            continue
        A, B, C = A[ok], B[ok], C[ok]
        nz = nrm[ok, 2] / (2.0 * ar[ok])
        tz = (A[:, 2] + B[:, 2] + C[:, 2]) / 3.0
        out.append((p, (float(lo[0]), float(lo[1]), float(lo[2])),
                    (float(hi[0]), float(hi[1]), float(hi[2])),
                    A, B, C, nz, tz))
    return out


# ---------------------------------------------------------------------------
# SETTLE BODY BUDGET (round 8)
# ---------------------------------------------------------------------------
# User, 2026-08-31: "settle/bake the buildings lazily. 15000 bodies seems
# like too much, you wanna place some by hand or something." Measured:
# `bake_quake_archetypes_launch_script.py` settles one whole STYLE ROW (every
# grade's `ctx["loose"]`, summed) in a single PhysX scene. Most rows are
# 1-3k bodies (`apartment_tall` DG3-5: 3,144, ~322 s); `block_residential`
# hit 18,771 and ran over 1.5 h before being killed. The reason is not a bug,
# it is the shape: `urban_building` builds a "block" style as a MAIN mass
# plus WINGS (`collapse_masses`'s own docstring — `block_residential` is 5
# masses, 2,377 elements, only 181 of them on `main`; a total/pancake grade
# sweeps every one of those masses, per `r_masonry_collapse`'s own `for mt, m
# in info["masses"].items()`), and every per-ELEMENT wall break
# (`quake_flow._break`/`_break_split`, 10-24 loose cells each — 10-13 for a
# ground-storey stub's `_break_split`, 18-24 for an ordinary wall element's
# `_break`) and every
# per-STOREY slab break (`_break_box_like`, 30-52 plank cells per slab per
# storey) multiplies by BOTH the mass count (5x) AND each wing's own storey
# count (16-20 storeys vs. `apartment_tall`'s single 9-storey mass) — not by
# a bug, by simple element-count arithmetic on a building that is really five
# buildings' worth of wall.
#
# Running every one of those cells through PhysX is not what makes a
# collapse pile READ as one: the CROWN — the bigger pieces that fell the
# furthest and are what a review camera's eye actually lands on — is what
# needs a believable rest angle and real non-interpenetration. A crumb
# buried three layers deep in a heap, or a plank under forty others, needs
# to not float and not clip through the ground; it does not need a rigid
# body to discover that. `SETTLE_BODY_BUDGET` (env, default 3000) caps how
# many of a row's loose pieces get one at all.
#
# `apply_settle_budget` is called ONCE per row, on the SAME accumulated
# `loose` list `settle.run` is about to receive, right before that call. It
# ranks pieces by `rank_loose_for_settle_budget` (volume x current world Z —
# a simple "z-and-size" proxy for visual importance, per this round's own
# brief: a piece already sitting high is one about to fall the furthest and
# land the most visibly). The top `budget` keep physics, unchanged. Every
# piece past it is placed GEOMETRICALLY, right here, with the SAME idioms
# this module and `quake_flow` already trust for exactly this rather than a
# new one invented for the occasion:
#   - `quake_flow._a_lay_flat` (via the `_qf()` lazy import `_sweep_roof_
#     props`/`_a_bury_props` above already use to cross this exact module
#     boundary) for the thin-axis-up orientation + yaw jitter — the SAME
#     "measure the piece's own LOCAL mesh points, turn the thinnest axis up"
#     idiom the DG5 masonry heap's plates already rest on, just called with
#     `p=1.0` so every piece in the budget's overflow gets it, not a share;
#   - `_deck_support_z` (this file, unchanged) for the landing height under
#     the piece's own footprint, with a caller-supplied `ground_z` fallback
#     exactly like `_sweep_roof_props`'s own OV/TILT path a few lines below;
#   - a small downward sink (2-6 cm) so a laid piece does not hover a
#     hairline above its support, the same "a few cm of interpenetration"
#     tolerance `quake_sliced._reseat_roof_plant`'s round-6c fix already
#     accepts.
# A piece placed this way is returned in `geometric`, never in `kept` (the
# trimmed loose list) — the caller appends it to the row's STATIC list
# instead, so it costs the settle nothing: no rigid body, no step budget, no
# convergence risk, and nothing for `deactivate_airborne` to have to sweep
# afterward either.
#
# BACKWARD COMPAT: `budget is None`, or `budget >= len(loose)`, is a NO-OP —
# `apply_settle_budget` returns the SAME loose list, an empty geometric list,
# and touches NOTHING on the stage. A bake at a budget above the actual body
# count (or with the env unset and the default raised past every row's real
# count) reproduces byte-for-byte. `tests/test_settle_budget.py` pins this.
SETTLE_BODY_BUDGET_ENV = "SETTLE_BODY_BUDGET"
SETTLE_BODY_BUDGET_DEFAULT = 3000
# A laid piece sinks this far below its resolved support, so it reads as
# seated rather than balanced on a knife-edge contact.
SETTLE_BUDGET_SINK_M = (0.02, 0.06)


def settle_body_budget(default=SETTLE_BODY_BUDGET_DEFAULT):
    """`SETTLE_BODY_BUDGET` from the environment: unset/empty -> `default`;
    a non-negative integer -> that many bodies; a NEGATIVE integer (`-1`) ->
    `None`, i.e. UNLIMITED — `apply_settle_budget` is then always a no-op,
    today's behaviour. An unparseable value falls back to `default` rather
    than raising: a typo in a launcher's environment should not crash a bake
    that would otherwise run fine at the default."""
    raw = os.environ.get(SETTLE_BODY_BUDGET_ENV, "").strip()
    if raw == "":
        return default
    try:
        n = int(raw)
    except ValueError:
        return default
    return None if n < 0 else n


def _settle_budget_world_aabb(stage, path, bc):
    """(lo_x, lo_y, lo_z, hi_x, hi_y, hi_z) world AABB of `path`, or `None`
    if the prim is missing, inactive, or has no resolvable extent."""
    from pxr import UsdGeom
    pr = stage.GetPrimAtPath(path)
    if not pr or not pr.IsValid() or not pr.IsActive():
        return None
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return (float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2]))


def rank_loose_for_settle_budget(stage, paths):
    """`paths` ranked DESCENDING by volume x current-world-Z-midpoint — the
    "z-and-size" proxy for visual importance this round's brief calls out
    (measuring real per-triangle surface exposure is hard; a piece already
    sitting high in the authored (pre-settle) stack is one about to fall the
    furthest and land the most visibly, on the crown of the pile, so height
    at THIS point, before any physics has run, is a fair stand-in). Ties
    broken by path string, so the order is fully determined by stage content
    — there is no rng draw in this function at all, this is a measurement,
    not a choice.

    Returns `[(path, score), ...]`, most important first. A piece with no
    resolvable AABB (already gone, or not a boundable prim) scores `-1.0`
    and sorts LAST — treated as the least urgent for physics rather than
    raising, since there is nothing to measure."""
    from pxr import Usd, UsdGeom
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    scored = []
    for p in paths:
        box = _settle_budget_world_aabb(stage, p, bc)
        if box is None:
            scored.append((p, -1.0))
            continue
        lx, ly, lz, hx, hy, hz = box
        vol = max(hx - lx, 1e-4) * max(hy - ly, 1e-4) * max(hz - lz, 1e-4)
        z_mid = 0.5 * (lz + hz)
        scored.append((p, vol * max(z_mid, 0.0)))
    scored.sort(key=lambda pr: (-pr[1], pr[0]))
    return scored


def _settle_budget_place_geometric(stage, path, ground_z, root, rng, exclude,
                                   candidates=None):
    """Seat one over-budget piece geometrically: lay it flat if it is a
    plate (`quake_flow._a_lay_flat`'s own thin-axis test decides that, not
    this function — a chunky, non-plate piece is left in its authored
    orientation), find its landing Z under its own footprint
    (`_deck_support_z`, `ground_z` fallback), sink it a few cm, done — no
    rigid body registered anywhere. Returns a small report dict (used by the
    test suite to assert seating within tolerance), or `None` if the piece
    could not be resolved at all — the caller leaves an unresolved piece
    loose rather than lose its geometry.

    `candidates`: passed straight through to `_deck_support_z` — see that
    function's own `candidates` parameter and `_deck_support_candidates`.
    `None` (the default) is the original, uncached behaviour."""
    from pxr import Usd, UsdGeom
    qf = _qf()
    fake_ctx = {"stage": stage, "rng": rng}
    qf._a_lay_flat(fake_ctx, [path], p=1.0)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = _settle_budget_world_aabb(stage, path, bc)
    if box is None:
        return None
    lx, ly, lz, hx, hy, hz = box
    cx, cy = 0.5 * (lx + hx), 0.5 * (ly + hy)
    half_w = max(0.15, (hx - lx) / 2.0)
    half_d = max(0.15, (hy - ly) / 2.0)
    support = _deck_support_z(stage, root, cx, cy, half_w, half_d, lz,
                              exclude=exclude, candidates=candidates)
    if support is None:
        support = float(ground_z)
    sink = rng.uniform(*SETTLE_BUDGET_SINK_M)
    dz = (support - sink) - lz
    qf._transform_prims(stage, [path], qf._translate(0.0, 0.0, dz))
    return {"path": path, "support_z": support, "bottom_before": lz,
            "bottom_after": lz + dz}


def apply_settle_budget(stage, loose_paths, budget, root="/",
                        ground_z=0.0, rng=None, exclude=()):
    """Split a row's accumulated `loose_paths` into `(kept, geometric,
    report)`.

    `kept` is the `budget` most visually-important pieces (by
    `rank_loose_for_settle_budget`) — hand this to `settle.run` exactly as
    before. `geometric` is everything past the budget, already placed at
    rest on the stage (support + lay-flat + sink) with NO rigid body — the
    caller appends these to its own STATIC collider list instead (so
    whatever stays loose still rests against them correctly) and to its
    export list (unchanged: a geometric piece is still the same prim path,
    just transformed in place, exactly like a piece `settle.bake` would have
    written back). `report` is one dict per geometric piece, for tests.

    A NO-OP — `(list(loose_paths), [], [])`, nothing touched on the stage —
    whenever `budget` is `None` or already covers every piece, so a bake at
    a high-enough budget reproduces today's export byte-for-byte.

    `root` scopes each `_deck_support_z` query: either a single path
    (applied to every piece) or a callable `path -> root_path` for a caller
    that knows which building authored which piece (tighter scoping cuts how
    many candidate meshes the containment test itself has to walk, though
    `_deck_support_z`'s own stage traversal cost is unchanged either way).
    `ground_z` is the fallback landing height when nothing is found under a
    piece's own footprint at all (e.g. this row's shared ground plane).
    `exclude` is extra paths never to treat as a candidate support (on top
    of every OTHER over-budget piece in this same call, which are excluded
    from each other unconditionally — none of them are in a final position
    yet, the same reasoning `_sweep_roof_props` already uses for its own
    `exclude_paths = set(fall)`).

    Deterministic: `rank_loose_for_settle_budget` draws no rng at all, and
    every jitter this function itself draws comes from the caller-supplied
    `rng` (a `random.Random`) in the fixed ranked order — the same seed
    against the same stage content always yields the same split and the
    same placements.

    PERFORMANCE (round 8): every `_deck_support_z` query below is run
    against a `_deck_support_candidates` cache, keyed by `root` string and
    built the first time each distinct root is seen in this call —
    `stage.Traverse()` and the per-mesh world-transform/triangulation
    happen ONCE per root, not once per over-budget piece. This is safe
    because nothing outside `over` moves during this function (`kept`
    pieces are untouched here — they only move later, in `settle.run`) and
    every path IN `over` is already excluded from being its own or another
    piece's support via `exclude_all` regardless of whether its cached
    geometry predates a later move — see `_deck_support_candidates`'s own
    docstring and `test_deck_support_z_cache_is_byte_identical_to_uncached`.
    """
    paths = list(loose_paths)
    if budget is None or budget >= len(paths):
        return paths, [], []
    rng = rng if rng is not None else random.Random(0)
    root_fn = root if callable(root) else (lambda _p: root)
    ranked = rank_loose_for_settle_budget(stage, paths)
    over = [p for p, _score in ranked[budget:]]
    exclude_all = set(exclude) | set(over)
    report, geometric = [], []
    cand_cache = {}
    for p in over:
        r = root_fn(p)
        if r not in cand_cache:
            cand_cache[r] = _deck_support_candidates(stage, r)
        rec = _settle_budget_place_geometric(
            stage, p, ground_z, r, rng, exclude_all,
            candidates=cand_cache[r])
        if rec is None:
            continue
        report.append(rec)
        geometric.append(p)
    geo_set = set(geometric)
    kept = [p for p in paths if p not in geo_set]
    return kept, geometric, report


# ---------------------------------------------------------------------------
# Z-OUTLIER SWEEP — one settled body a whole storey off its own siblings
# ---------------------------------------------------------------------------
# `bld_brownstone_row_DG4.usd`'s `/Baked/LOD0_108` (2762 pts / 2754 faces),
# measured: z~=17, sitting on the roof deck, while its FIVE topology-
# identical siblings (`LOD0_105`, `106`, `107`, `109`, `110` — same
# signature, one wall-break event's own cells) landed at z~=11, at the
# actual break line. A settle artefact — one body climbed, or got stuck,
# on its way down — and nothing upstream catches it: the gap from `LOD0_
# 108` to the roof deck it actually stopped on is a real -0.07 m, so
# `fire_bake.deactivate_airborne`'s own points-based "is this seated"
# test PASSES it. That test only ever asks "is this body resting on
# something", never "does this body agree with its own family" — this
# sweep is the second question.
Z_OUTLIER_ENV = "EQ_Z_OUTLIER"
Z_OUTLIER_MIN_GROUP = 3
Z_OUTLIER_STOREYS_DEFAULT = 1.5
Z_OUTLIER_RESEAT_STOREYS_DEFAULT = 1.0
# `quake_flow.d_box_mass`'s own default storey height — a general sweep run
# against an already-exported archetype file (this family's own offline
# verification path, `tools/*_probe.py`-style) has no per-building spec to
# read a real one from; the launcher's own per-row `H`/style spec is not
# threaded through here for exactly that reason — one constant, honoured
# the same way whether this runs mid-authoring or against a baked file.
Z_OUTLIER_STOREY_H_DEFAULT = 3.4


def z_outlier_enabled():
    """`EQ_Z_OUTLIER` from the environment: unset/empty -> enabled (every
    archetype bake gets the sweep unless told otherwise); `0`/`false`/
    `False` -> disabled. Boolean, not numeric, because this sweep has no
    budget to size, only an on/off — `settle_body_budget`'s own env-reader
    convention, adapted."""
    raw = os.environ.get(Z_OUTLIER_ENV, "").strip()
    return raw not in ("0", "false", "False")


def _mesh_topology_signature(prim):
    """`(point_count, face_count)` of one Mesh prim, or `None` if it has no
    resolvable points / `faceVertexCounts` — an empty or malformed mesh,
    never a candidate for anything."""
    from pxr import UsdGeom
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    if not pts or not counts:
        return None
    return (len(pts), len(counts))


def _topology_groups(stage, root, allow=None):
    """Every ACTIVE Mesh prim under `root` (a single path; the pseudo-root
    `"/"` / `""` matches everything — `_deck_support_z`'s own `under_root`
    convention), grouped by its exact `_mesh_topology_signature`.

    `allow`, if given, is a set/sequence of candidate paths (as `str`) —
    only a Mesh whose path is IN it is even considered. See
    `z_outlier_sweep`'s own docstring for why this matters: without it, a
    legitimate prop repeated once per storey (same topology, different
    floor BY DESIGN, never a settle artefact) can collide with a real
    outlier's detection. `None` (the default) keeps today's behaviour —
    every Mesh under `root` is a candidate.

    A mesh path containing a `Prototypes` scope (a `PointInstancer`'s
    TEMPLATE geometry, never an individually placed piece) is never a
    candidate — none of this family's debris packs are Mesh-typed
    prototypes today (measured against the real archetype library: every
    reference under a `Prototypes` scope in `bld_brownstone_row_DG4.usd`
    fails to resolve rather than authoring a stray Mesh there), but the
    exclusion costs nothing and a future pack could differ.

    Returns `{(points, faces): [path, ...]}`, each list in stage-traversal
    order — `Usd.Stage.Traverse()` is a stable depth-first walk, so this is
    fully deterministic against the same stage content."""
    from pxr import UsdGeom
    root = str(root)
    under_root = root in ("", "/")
    allow_set = None if allow is None else set(str(p) for p in allow)
    groups = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        p = str(prim.GetPath())
        if not (under_root or p == root or p.startswith(root + "/")):
            continue
        if allow_set is not None and p not in allow_set:
            continue
        if "/Prototypes/" in p or p.endswith("/Prototypes"):
            continue
        sig = _mesh_topology_signature(prim)
        if sig is None:
            continue
        groups.setdefault(sig, []).append(p)
    return groups


def z_outlier_sweep(stage, root, paths=None, storey_h=Z_OUTLIER_STOREY_H_DEFAULT,
                    threshold_storeys=Z_OUTLIER_STOREYS_DEFAULT,
                    reseat_storeys=Z_OUTLIER_RESEAT_STOREYS_DEFAULT,
                    min_group=Z_OUTLIER_MIN_GROUP, rng=None, verbose=True):
    """Find, and fix, one body from a topology-identical family that
    settled a whole storey off where every one of its siblings did — the
    module docstring above has the measured defect this exists for.

    `paths`, if given, restricts which meshes under `root` are even
    ELIGIBLE to be grouped — a building's own `res["loose"]` (the SAME
    pool `settle.run`/`apply_settle_budget` already run on, from
    `quake_flow.wreck_building`'s return), never the static shell/props.
    `None` (the default) scans every Mesh under `root` instead — the ONLY
    option against an already-exported archetype file, which carries no
    `ctx` to read a loose list back from (this sweep's own offline
    verification path, run against `bld_brownstone_row_DG4.usd`).

    WHY THIS MATTERS, MEASURED: an unrestricted scan of that same file
    also flagged `prop_main_4_7` — a floor-4 fixture — as an "outlier",
    because the SAME prop recurs once per storey (`prop_main_1_3`/`1_5`/
    `1_6` on floor 1, `prop_main_2_0`/`2_3` on floor 2, all sharing one
    (180, 312) topology) and floor 4's own copy sits several storeys above
    the floor-1/2 median BY DESIGN, not by any settle accident. A
    legitimate prop repeated once per storey is never a "loose fracture-
    cell mesh" and must never enter this sweep's candidate pool at all —
    passing the building's own `paths` (its production call site, in
    `bake_quake_archetypes_launch_script.py`) keeps every static/prop mesh
    out from the start, so the false positive above cannot occur there;
    only the unrestricted fallback (necessarily, over an exported file
    with no loose-list metadata left) can still see it.

    METHOD. Every eligible Mesh under `root` is grouped by its exact
    (point count, face count) signature (`_topology_groups`) — the same
    cheap, no-name-parsing test the real defect was found with: one
    wall-break event's fracture cells are several meshes of identical chip
    topology (an accident of how `fracture.solidify` tessellates one break
    spec, not something any recipe tags on purpose), so grouping by name
    would need inventing one. A group under `min_group` (default 3) is
    left alone — two chips can legitimately land far apart with no
    majority vote to be an outlier FROM.

    Within a group of >= `min_group`, each member's Z is its world AABB
    BOTTOM (`_settle_budget_world_aabb`) — "where does this piece actually
    sit," the same measure `_sweep_roof_props`'s `base_z` and
    `apply_settle_budget`'s `lz` already use, never a rotated fragment's
    raw pivot translate, which can sit well above or below its own
    footprint. The MEDIAN (never the mean — one outlier must not drag its
    own threshold toward itself) is the group's resting band; any member
    more than `threshold_storeys * storey_h` (default 1.5 x 3.4 m = 5.1 m)
    above that median is an outlier — measured on the real defect: `LOD0_
    108` sits 5.68 m above its siblings' median, past the 5.1 m line;
    every sibling sits within 0.48 m of it.

    RESEATING. An outlier is dropped onto the highest real support under
    its OWN footprint (`_deck_support_z`, fed `_deck_support_candidates`'
    cache — one traversal for the whole call, not one per outlier), but
    the search CEILING is `median + reseat_storeys * storey_h` (default
    ONE storey above the band — deliberately tighter than the detection
    threshold), never the outlier's own current height. Searching up to
    the outlier's own height would just find the very roof deck it wrongly
    stopped on again; one storey above where its siblings actually rest is
    what finds the real structure instead. Every OTHER outlier resolved in
    this same call is excluded from every search (none of them are in a
    final position yet — `apply_settle_budget`'s own `exclude_all`
    reasoning), but a non-outlier sibling is not: it is real, resolved
    geometry and a legitimate landing spot for another building's stray
    piece.

    A resolved drop gets the SAME small dressing every other
    geometrically-placed piece in this family gets: sunk `SETTLE_BUDGET_
    SINK_M` below the support so it reads as seated, then the idle tip
    `quake_flow.B_ROOF_PLANT_TIP_DEG` gives an untouched prop
    (`_sweep_roof_props`'s own "small drop" branch, reused verbatim) — no
    velocity kick, no rigid body, no settle dependency: geometry in,
    static geometry out. An outlier with NO real support anywhere under
    its footprint (a genuine hole all the way down) is deactivated
    instead — `fire_bake.deactivate_airborne`'s own answer to "nothing is
    really there."

    DETERMINISTIC. `rng` (default a fresh `random.Random(0)`) is the ONLY
    source of randomness this function draws from — the tip axis/angle
    for each resolved outlier, in sorted-path order — the median, the
    outlier test and the support search are pure measurements with no
    draw at all, so the same stage content and the same `rng` seed always
    yield the same outliers and the same placements.

    `EQ_Z_OUTLIER=0` (`z_outlier_enabled()`) disables this sweep entirely:
    a no-op returning `{"reseated": 0, "deactivated": 0, "groups": 0,
    "outliers": []}` without even traversing the stage.

    Returns `{"reseated": N, "deactivated": M, "groups": <groups found>,
    "outliers": [path, ...]}` (`path` order matches `outliers`' own
    resolution order — sorted, so it too is deterministic).
    """
    if not z_outlier_enabled():
        return {"reseated": 0, "deactivated": 0, "groups": 0, "outliers": []}

    from pxr import Sdf, Usd, UsdGeom
    qf = _qf()
    rng = rng if rng is not None else random.Random(0)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    groups = _topology_groups(stage, root, allow=paths)

    boxes = {}

    def _box_of(path):
        if path not in boxes:
            boxes[path] = _settle_budget_world_aabb(stage, path, bc)
        return boxes[path]

    threshold_m = float(threshold_storeys) * float(storey_h)
    ceiling_pad_m = float(reseat_storeys) * float(storey_h)

    outliers = []   # [(path, median_z), ...]
    for _sig, group_paths in groups.items():
        boxed = [(p, _box_of(p)) for p in group_paths]
        boxed = [(p, b) for p, b in boxed if b is not None]
        if len(boxed) < min_group:
            continue
        zs = sorted(b[2] for _p, b in boxed)
        n = len(zs)
        median_z = (zs[n // 2] if n % 2 else
                   0.5 * (zs[n // 2 - 1] + zs[n // 2]))
        for p, b in boxed:
            if b[2] - median_z > threshold_m:
                outliers.append((p, median_z))

    if not outliers:
        return {"reseated": 0, "deactivated": 0, "groups": len(groups),
                "outliers": []}

    outliers.sort(key=lambda t: t[0])
    exclude_all = set(p for p, _mz in outliers)
    cand_cache = _deck_support_candidates(stage, root)

    n_reseat = n_deact = 0
    for p, median_z in outliers:
        box = _box_of(p)
        lx, ly, lz, hx, hy, hz = box
        cx, cy = 0.5 * (lx + hx), 0.5 * (ly + hy)
        half_w = max(0.3, (hx - lx) / 2.0 * 1.15)
        half_d = max(0.3, (hy - ly) / 2.0 * 1.15)
        ceiling = median_z + ceiling_pad_m
        support = _deck_support_z(stage, root, cx, cy, half_w, half_d,
                                  ceiling, exclude=exclude_all,
                                  candidates=cand_cache)
        if support is None:
            stage.GetPrimAtPath(Sdf.Path(p)).SetActive(False)
            n_deact += 1
            continue
        sink = rng.uniform(*SETTLE_BUDGET_SINK_M)
        dz = (support - sink) - lz
        qf._transform_prims(stage, [p], qf._translate(0.0, 0.0, dz))
        c = qf._pivot_of({"stage": stage}, p)
        M = qf._rot_about(c, (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                          rng.uniform(-qf.B_ROOF_PLANT_TIP_DEG,
                                       qf.B_ROOF_PLANT_TIP_DEG))
        qf._transform_prims(stage, [p], M)
        n_reseat += 1

    if verbose and (n_reseat or n_deact):
        print("[qarch] z-outlier sweep: {0} re-seated, {1} deactivated"
              .format(n_reseat, n_deact))
    return {"reseated": n_reseat, "deactivated": n_deact,
           "groups": len(groups), "outliers": [p for p, _mz in outliers]}


def _sweep_roof_props(ctx, plan, m, prng):
    """Resolve every rooftop prop whose support this plan just took away —
    the fix for "a row of water tanks hovering in the sky" (round-5 review,
    `ne_obl.png`): a `qc_*` recipe removed the roof/top storeys under them
    and nothing upstream of the generic end-of-build pass
    (`quake_flow._b_settle_roof_plant`) ever asked whether their OWN roof
    survived.

    ROUND 6: round 5's own fix — a small velocity kick plus
    `ctx["loose"]`, left for the generic settle to carry the rest of the way
    down — bakes frozen mid-air exactly as its own docstring warned it
    might, MEASURED on `bld_brownstone_row_DG3.usd` (fell ~1 m of a required
    ~3 m). This resolves the landing GEOMETRICALLY instead: a support probe
    under the prop's own footprint (`_deck_support_z`), placed there as pure
    static geometry — no rigid body, no step budget, no settle dependency —
    the same contract the `total`/`pancake` bury path below already keeps.
    A drop deeper than `ROOF_PROP_BIG_TIP_STOREY_FRAC` of this mass's own
    top-storey height does NOT land bolt upright: it gets
    `quake_flow._a_bury_props`'s own tip/roll, reused verbatim. Shallower
    than that, it gets the small idle tip `quake_flow._b_settle_roof_plant`
    already gives an untouched prop. Everything here draws from `prng` — the
    per-building private generator this whole family is seeded from
    (module docstring, "THE RNG CONTRACT") — so a rebake of the same
    building at the same seed lands every prop in exactly the same place.

    `ctx["roof_plant_mass"]` is the one mass `dress_roof` ever seats plant
    on (always "main" in practice), so a wing's own total collapse — a
    SEPARATE `_author_one` call in the same `r_collapse` — never re-tests
    the main roof's tanks a second time; only the call for that mass runs
    this at all.

    Resolved paths are REMOVED from `ctx["roof_plant"]` / `ctx["roof_
    fixed"]`, which `_b_settle_roof_plant` reads FRESH after every recipe
    has run (`wreck_building`'s own order): a path this function has already
    placed and made static must never go through that generic idle-tip-
    then-loose pass a second time — a rigid body registered on a prim this
    call already resolved.

    Returns `(n_fall, n_buried)`.
    """
    if plan["mode"] not in ROOF_PROP_MODES:
        plan["roof_prop_fall"] = []
        return 0, 0
    if plan["mass"] != ctx.get("roof_plant_mass", "main"):
        plan["roof_prop_fall"] = []
        return 0, 0
    plant = list(dict.fromkeys(
        list(ctx.get("roof_plant") or ()) + list(ctx.get("roof_fixed") or ())))
    if not plant:
        plan["roof_prop_fall"] = []
        return 0, 0

    from pxr import Usd, UsdGeom
    fc, qf = _fc(), _qf()
    stage = ctx["stage"]
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    loose_now = set(ctx["loose"])
    fall, bbox_by_path = [], {}
    for pth in plant:
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid() or not pr.IsActive() or pth in loose_now:
            continue
        r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        lo, hi = r.GetMin(), r.GetMax()
        footprint = ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]),
                    (lo[0], hi[1]),
                    ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0))
        if roof_prop_footprint_lost(plan, m, footprint):
            fall.append(pth)
            bbox_by_path[pth] = (lo, hi)
    plan["roof_prop_fall"] = list(fall)
    if not fall:
        return 0, 0

    gone = set(fall)
    if ctx.get("roof_plant"):
        ctx["roof_plant"] = [p for p in ctx["roof_plant"] if p not in gone]
    if ctx.get("roof_fixed"):
        ctx["roof_fixed"] = [p for p in ctx["roof_fixed"] if p not in gone]

    if plan["mode"] in ("total", "pancake"):
        # THE SAME PROPS SWEEP `_author_floors` ALREADY OWNS for `bury_
        # props` — under the pile, not on it, exactly like the building's
        # own interior contents.
        H = max(3.0, float(m["top"]) - float(m["z0"]))
        base, depth, keep = float(m["z0"]), H * DOME_CROWN_FRAC, 0.3
        st = plan.get("stack")
        if st:
            base = float(st["base_z"])
            depth = st["pitch_m"] * plan["n_levels"] * 0.6
            keep = 0.25
        qf._a_bury_props(ctx, fall, base, depth, keep=keep)
        return 0, len(fall)

    # ELEVATION / CORNER: place each fallen prop on whatever is really there.
    storey_h = max(2.5, float(m["top"]) - float(m["levels"][-1]))
    big_drop_m = storey_h * ROOF_PROP_BIG_TIP_STOREY_FRAC
    exclude_paths = set(fall)
    n_placed = 0
    for pth in fall:
        lo, hi = bbox_by_path[pth]
        cx = (lo[0] + hi[0]) / 2.0
        cy = (lo[1] + hi[1]) / 2.0
        half_w = max(0.3, (hi[0] - lo[0]) / 2.0 * 1.15)
        half_d = max(0.3, (hi[1] - lo[1]) / 2.0 * 1.15)
        base_z = float(lo[2])
        support = _deck_support_z(stage, ctx["parent"], cx, cy, half_w,
                                  half_d, base_z, exclude=exclude_paths)
        if support is None:
            support = float(m["z0"])
        drop = base_z - support
        if drop > big_drop_m:
            # A REAL FALL. DG5's own bury dressing, reused verbatim, not
            # invented: it reads the prop's CURRENT (still unmoved) pivot
            # and moves it in ONE transform straight to `support` (plus its
            # own small upward jitter and up to 1.2 m of xy scatter — right
            # for landing wrecked on a real floor/ground slab several
            # metres across, same as a DG5 pile) with the 20-75 deg
            # near-horizontal tip/roll that says "this did not stay
            # upright." `keep=1.0`: every one of these is already resolved
            # and placed, so none may be silently deactivated instead.
            qf._a_bury_props(ctx, [pth], support, max(0.4, drop * 0.15), keep=1.0)
        elif drop > 0.02:
            # A SMALL DROP: correct the height, then the idle tip
            # `quake_flow._b_settle_roof_plant` already gives a prop nobody
            # kicked over — it barely fell, so it barely leans.
            qf._transform_prims(stage, [pth], qf._translate(0.0, 0.0, support - base_z))
            c = qf._pivot_of(ctx, pth)
            M = qf._rot_about(c, (prng.uniform(-1, 1), prng.uniform(-1, 1), 0.0),
                              prng.uniform(-qf.B_ROOF_PLANT_TIP_DEG,
                                           qf.B_ROOF_PLANT_TIP_DEG))
            qf._transform_prims(stage, [pth], M)
            ctx["static_extra"].append(pth)
        else:
            # STILL SUPPORTED where it stands (this plan's region test can
            # over-select a prop whose footprint majority is lost but whose
            # ACTUAL rectangle still has a live cell under it) — the same
            # idle tip, no translate needed.
            c = qf._pivot_of(ctx, pth)
            M = qf._rot_about(c, (prng.uniform(-1, 1), prng.uniform(-1, 1), 0.0),
                              prng.uniform(-qf.B_ROOF_PLANT_TIP_DEG,
                                           qf.B_ROOF_PLANT_TIP_DEG))
            qf._transform_prims(stage, [pth], M)
            ctx["static_extra"].append(pth)
        n_placed += 1
    return n_placed, 0


def r_collapse(ctx, mode="elevation", **kw):
    """THE RECIPE. Author `plan_collapse(ctx, mode, **kw)`.

    Order matters and every step is here for a reason already paid for once:

      1. the SHELL comes away first — `quake_flow._els` skips elements a
         recipe has marked dead, so a wall taken away after art has been
         authored on it leaves the art standing in the sky;
      2. the STUBS (total / pancake) are torn at their own height;
      3. the PERIMETER of the hole is torn, every surviving module that
         touches a dead one, on the edge the hole is on;
      4. the floors: the slab that lost its bearing wall drops whole, the rest
         lose a ragged strip and droop into the opening;
      5. the FIT-OUT that has lost its floor or its wall goes down;
      6. the band's block above moves / the pancake's slabs stack;
      7. the position sweep (elevation / corner only);
      8. the PILES, through `_rubble`;
      9. the teeth at the break.

    Everything draws from the PRIVATE generators, installed on the ctx for the
    duration (`fire_collapse._own_rng`) because `_rubble`, `_a_slab_rim`,
    `_ragged_slabs`, `_disturb_interior`, `r_droop` and `_dust_loose` all read
    `ctx["rng"]` off the ctx. Zero shared draws.

    Returns the FIRST mass's plan, with `plan["masses"]` naming every mass
    this call brought down and `plan["extra"]` holding the others' plans;
    `ctx["quake_collapse"]` is the whole list.
    """
    # EVERY MASS, for a total collapse and a pancake. See `collapse_masses`:
    # `urban_building` builds a block as a main mass plus wings, and on the
    # tall stock the wings ARE the building (`block_residential`: 181 of its
    # 2,377 elements are on `main`). A partial or band mode is single-mass by
    # definition and this loop runs once.
    tags = collapse_masses(ctx["info"], mode, kw.get("mass"))
    plans = []
    stack_top = None
    first = None
    for i, mt in enumerate(tags):
        kw2 = dict(kw)
        kw2["mass"] = mt
        if i and stack_top is not None:
            # a wing pancakes ONTO what the main body left (`r_pancake`)
            kw2.setdefault("stack_base_z", stack_top)
        if i and first is not None and first["mode"] == "total":
            # ...and every mass's heap sits on the GROUND, not on the wing's
            # own base, which is the main mass's roof
            kw2.setdefault("pile_base_z", first["heaps"][0]["base_z"])
        p_i = _author_one(ctx, mode, kw2)
        plans.append(p_i)
        if first is None:
            first = p_i
        if p_i["stack"] and stack_top is None:
            stack_top = (float(p_i["stack"]["base_z"])
                         + p_i["stack"]["pitch_m"] * p_i["n_levels"] + 0.4)
    plan = plans[0]
    plan["masses"] = list(tags)
    plan["extra"] = plans[1:]
    ctx["quake_collapse"] = plans
    return plan


def _author_one(ctx, mode, kw):
    """`r_collapse` for ONE mass — the whole authoring order, in one place so
    the multi-mass sweep above is a loop and not a second copy of it."""
    import numpy as np
    fc, qf = _fc(), _qf()

    plan = plan_collapse(ctx, mode=mode, **kw)
    mtag = plan["mass"]
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    prng = random.Random(plan["seed"])
    pnrng = np.random.default_rng(plan["seed"] & 0xFFFFFFFF)
    pre_static = set(q for q in (ctx.get("static_extra") or ()) if q)

    with fc._own_rng(ctx, prng, pnrng):
        # ---- 1/2. the shell and the stubs -------------------------------
        n_broke, n_frag = _author_kill(ctx, plan, m, prng, pnrng)
        n_stub = _author_stubs(ctx, plan, m, prng)

        # ---- 3. the whole perimeter of the hole -------------------------
        snap = fc._snapshot(ctx)
        n_edge = fc._tear_perimeter(ctx, plan, m, prng, plan["tears"])

        # ---- 4. the floors ----------------------------------------------
        n_slabrim = 0
        if plan["crush"]:
            # BOTH slabs the gap exposes: the one under the crushed storey
            # (it stays put, and takes rebar tufts) and the one the block
            # above sits on (it is about to be transformed, so no tufts).
            k = plan["crush"]["storey"]
            n_slabrim += len(qf._a_slab_rim(ctx, mtag, k, n_sides=2))
            n_slabrim += len(qf._a_slab_rim(ctx, mtag, k + 1, n_sides=2,
                                            bars=False))
        elif plan["floor_ragged"]:
            for sd in plan["sides"]:
                qf._ragged_slabs(ctx, mtag, sd, set(plan["storeys"]))
                lo_t, hi_t = plan["span"][(sd, plan["top_storey"])]
                # THE FLOORS AT THE OPENING HINGE DOWN — the USAR lean-to —
                # BUT ONLY WHEN A REAL LENGTH OF WALL HAS GONE. `_droop_strip`
                # splits a strip 25-50 % of the mass DEEP along the WHOLE of
                # that side; behind a wall that is still standing that is a
                # floor tilting into a room for no visible reason.
                if (hi_t - lo_t) >= 0.60 * fc.side_length(m, sd):
                    qf.r_droop(ctx, mass=mtag, side=sd,
                               storeys=set(plan["storeys"]), p=0.35)
            qf._disturb_interior(ctx, mtag, set(plan["storeys"]),
                                 side=plan["sides"][0])

        # ---- 5. the fit-out ---------------------------------------------
        n_fit = _fall_fitout(ctx, plan, m, prng)
        n_board = _author_floors(ctx, plan, m, prng) \
            if (plan["break_slabs"] or plan["bury_props"]) else 0

        # ---- 6. the block above / the slab stack ------------------------
        n_above = n_plates = 0
        if plan["crush"]:
            n_above = _author_band(ctx, plan, m, prng, snap)
        elif plan["stack"]:
            n_plates = _author_stack(ctx, plan, m, prng)

        # ---- 7. the position sweep --------------------------------------
        n_swept = _sweep_statics(ctx, plan, m, pre_static) if plan["sweep"] \
            else 0

        # ---- 7b. roof props whose roof/top storey just died --------------
        n_roof_fall, n_roof_buried = _sweep_roof_props(ctx, plan, m, prng)

        # ---- 8. the piles ------------------------------------------------
        n_heap = _author_heaps(ctx, plan, m, prng)

        # ---- 9. the teeth at the break -----------------------------------
        # ...and NOT where the floor they would stand on is itself coming
        # down: a total collapse and a pancake leave nothing for a bearing
        # stub to stand in, and a static post over a slab that has just gone
        # to the solver is a stick hanging in the sky (uf_r2j, 2026-08-28).
        n_teeth = _teeth(ctx, plan, m, prng, plan["storeys"]) \
            if plan["teeth"] else 0

        # THE DUST IS APPLIED AT EACH FRACTURE SITE, NOT IN ONE SWEEP HERE,
        # and that is deliberate. `_dust_loose` rebinds a dusted COPY of
        # whatever a prim is CURRENTLY bound to, so calling it twice on one
        # fragment tints an already-tinted copy — `diffuse_tint` multiplies,
        # and a second pass is the near-black bug the round-4 catalogue
        # records for `_material_for_look` and round 2 records for
        # `planks.wood_material`. It is also why the pieces `_ragged_slabs`,
        # `_a_slab_rim`, `r_droop`, `_disturb_interior` and `_author_floors`
        # author are left alone: every one of those is already an `A_DEBRIS`
        # flat colour calibrated in linear albedo. What IS dusted is exactly
        # what `_chunk_material` / `_mat_fn` can hand a PRISTINE palette entry
        # or a cladding photo — the shell fragments and the stub shards — and
        # that is the answer to "damaged part and undamaged part look like
        # completely diff materials". Nothing here calls `_a_dustify` (a
        # palette SWAP with no entry for `brick` or for a cladding photo at
        # all) or `HEAP_MIX`.

    cen = fc.edge_census(plan["tears"])
    plan["edge_census"] = cen
    ctx["notes"].append(describe(plan))
    ctx["notes"].append(
        "quake collapse: {0} module(s) broken into {1} fragment(s), {2} stub "
        "module(s) torn at their own height, {3} surviving module(s) torn at "
        "the edge of the hole, {4} slab rim piece(s), {5} static prim(s) "
        "swept into the loss, {6} prim(s) carried with the block above, {7} "
        "slab(s) stacked, {8} floor board(s), {9} rubble prim(s), {10} joist "
        "stub(s)".format(
            n_broke, n_frag, n_stub, n_edge, n_slabrim, n_swept, n_above,
            n_plates, n_board, n_heap, n_teeth))
    ctx["notes"].append(
        "quake collapse fit-out: {0} slab(s), {1} partition(s), {2} "
        "column(s), {3} prop(s) sent down; {4} partition fragment(s) "
        "settled instead of whole sheets".format(
            n_fit["slab"], n_fit["part"], n_fit["col"], n_fit["prop"],
            n_fit["part_frag"]))
    if n_roof_fall or n_roof_buried:
        # `quake_flow`'s own "[quake] roof_plant: N dropped to physics, M
        # buried" banner line, one level up: this is what THIS plan already
        # resolved before the generic end-of-build pass ever saw the rest.
        ctx["notes"].append(
            "quake collapse roof props: {0} fallen (roof/top storey lost), "
            "{1} buried".format(n_roof_fall, n_roof_buried))
    ctx["notes"].append(
        "quake collapse edges: " + ", ".join(
            "{0} {1}/{2}".format(c, cen[c][1], cen[c][0])
            for c in fc.EDGE_CLASSES) +
        "; budget {0}/{1} module(s), {2}/{3} edge(s){4}".format(
            plan["budget"]["modules"], plan["budget"]["module_cap"],
            plan["budget"]["edges"], plan["budget"]["edge_cap"],
            ", OVER BUDGET" if (plan["budget"]["over_modules"]
                                or plan["budget"]["over_edges"]) else ""))
    # THE ONE LINE A BAKE LOG IS READ FOR. Every module the two rings put a
    # judge on and how many of them `_tear_perimeter` actually split: anything
    # but N/N is a factory seam still standing at the edge of the hole.
    ctx["notes"].append(boundary_line(plan))
    return plan


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------
def _recipe(mode):
    """One `RECIPES` entry per mode. `mode` may still be overridden by a
    ladder entry or an `EQ_RECIPES` bench line, which is how `qc_auto` is
    expressible at all."""
    def _r(ctx, **kw):
        return r_collapse(ctx, mode=kw.pop("mode", mode), **kw)
    _r.__name__ = "r_qc_" + mode
    _r.__doc__ = "quake_collapse.r_collapse(mode={0!r}).".format(mode)
    return _r


RECIPES = {
    "qc_elevation": _recipe("elevation"),
    "qc_corner": _recipe("corner"),
    "qc_soft_storey": _recipe("soft_storey"),
    "qc_mid_storey": _recipe("mid_storey"),
    "qc_pancake": _recipe("pancake"),
    "qc_total": _recipe("total"),
    # THE SEVENTH NAME, and why it exists: a ladder table is STATIC, so
    # "urm DG3 -> a corner or an elevation, drawn" cannot be written as a
    # table entry — the draw has to live in a recipe. `quake_flow.
    # r_storey_collapse` is the same shape for the same reason (a soft storey
    # two times in three, a mid storey otherwise). The draw is made from the
    # BUILDING'S OWN private generator, so it takes no shared draw either.
    "qc_auto": _recipe("auto"),
}


# THE LADDER, as OVERRIDES rather than a second full table.
#
# `quake_flow` composes `LADDER_QC` by copying `LADDER` and applying these,
# so every row this table does not name — DG0-DG2, the glass slots, the
# rooftop plant, the foundation family — is literally the legacy row and
# cannot drift from it. What changes is only the grades where a collapse
# happens.
#
#   urm  DG3  corner or elevation, drawn      (was `corner_fail`)
#        DG4  an elevation from a storey up   (was `out_of_plane`)
#        DG5  total                           (was `masonry_collapse`)
#   rc   DG3  the infill of one elevation     (was `infill_fail` alone)
#        DG4  a soft storey, or a mid storey  (was `storey_collapse`)
#        DG5  pancake                         (was `pancake`)
#   rc_glass DG4  the podium's soft storey    (was `soft_storey`)
#        DG5  unchanged — `tilt_sink`. No curtain-wall tower has come down in
#             the reviewed record, and that is the one thing this family must
#             not change about the tower ladder.
LADDER_OVERRIDES = {
    "urm": {
        "DG3": [("parapet_fall", {"sides": 2, "frac": 0.8}),
                ("qc_auto", {}),
                ("roof_hole", {"frac": 0.25}),
                ("facade_scars", {"frac": 0.22}),
                ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("storefront_glass", {"grade": 3})],
        "DG4": [("qc_elevation", {}),
                ("parapet_fall", {"sides": 3, "frac": 0.9}),
                ("roof_hole", {"frac": 0.35}),
                ("rooftop_fail", {"frac": 0.8}),
                ("facade_scars", {"frac": 0.18}),
                ("storefront_glass", {"grade": 4})],
        "DG5": [("qc_total", {})],
    },
    "rc": {
        "DG3": [("qc_elevation", {"infill": True}),
                ("balcony_fail", {"frac": 0.6}),
                ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("storefront_glass", {"grade": 3})],
        "DG4": [("balcony_fail", {"frac": 0.7}),
                ("storefront_glass", {"grade": 4}),
                ("qc_soft_storey", {}),
                ("rooftop_fail", {"frac": 0.8}),
                ("glass_follow", {})],
        "DG5": [("qc_pancake", {})],
    },
    "rc_glass": {
        "DG4": [("curtain_wall", {"grade": 4}),
                ("storefront_glass", {"grade": 4}),
                ("qc_soft_storey", {"storey": 0, "lean_deg": 2.5}),
                ("glass_follow", {})],
    },
}


# ---------------------------------------------------------------------------
# Host-side self-check
# ---------------------------------------------------------------------------
def check(verbose=True, styles=("commercial_mid", "commercial", "apartment",
                                "block_residential", "highrise_step",
                                "dw_terrace", "brownstone_row")):
    """No stage, no pxr: plan every mode on real kit buildings and assert the
    invariants the family is built around. Kept here so a launch script can
    gate on it the way it gates on `quake_flow.check`."""
    import numpy as np
    from detail import urban_building as ub
    from . import quake_flow as qf

    bad = []
    for name, recs in LADDER_OVERRIDES.items():
        for grade, lst in recs.items():
            for rname, _kw in lst:
                if rname not in RECIPES and rname not in qf.RECIPES:
                    bad.append("LADDER_OVERRIDES {0}/{1}: unknown recipe "
                               "{2}".format(name, grade, rname))
    for style in styles:
        for mode in MODES + ("auto",):
            rng = random.Random(11)
            pls = ub.build_building(style, 0.0, 0.0, 0.0, rng)
            info = qf.describe(style, pls, 0.0, 0.0, 0.0)
            ctx = {"info": info, "rng": rng,
                   "nrng": np.random.default_rng(11), "notes": [],
                   "tag": "chk_" + style}
            try:
                plan = plan_collapse(ctx, mode=mode)
            except Exception as exc:                       # pragma: no cover
                bad.append("{0}/{1}: plan raised {2!r}".format(style, mode,
                                                               exc))
                continue
            m = info["masses"][plan["mass"]]
            if not plan["kill"]:
                bad.append("{0}/{1}: nothing taken away".format(style, mode))
            for e in plan["kill"]:
                if e["side"] not in plan["sides"] and e["role"] not in ROOF_ROLES:
                    bad.append("{0}/{1}: killed {2} on {3}, not in {4}".format(
                        style, mode, e["name"], e["side"], plan["sides"]))
                    break
            if plan["mode"] in ("elevation", "corner") and not plan["keep_sides"]:
                bad.append("{0}/{1}: no untouched elevation left".format(
                    style, mode))
            for h in plan["heaps"]:
                for sd, c in (h["centres"] or {}).items():
                    if c["outward_m"] <= 0.0:
                        bad.append("{0}/{1}: {2} heap on {3} at {4:+.2f} m of "
                                   "the wall line, wrong side".format(
                                       style, mode, h["kind"], sd,
                                       c["outward_m"]))
            for sd in plan["stub"]:
                if sd not in plan.get("blind_sides", ()):
                    bad.append("{0}/{1}: stub on {2}, not a blind side".format(
                        style, mode, sd))
            _ = m
    if verbose:
        print("[quake_collapse] check {0}".format("ok" if not bad else
                                                  "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":                                    # pragma: no cover
    import sys
    sys.path.insert(0, os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
    raise SystemExit(1 if check() else 0)
