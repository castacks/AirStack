"""urban_fire — a STRUCTURE fire in an urban block.

WHY THIS IS NOT `disaster.fire` WITH BUILDINGS INSTEAD OF TREES
---------------------------------------------------------------
A wildfire is a FRONT that sweeps a plate at ground level. Everything in
`disaster/fire.py` and the burn-age model follows from that: an elliptical
arrival time per point, damage keyed to how long the front has been past, a
soot wash that runs UP the outside of every wall from its base, a ground scar,
and a debris field of consumed timber.

None of that describes a fire in a city block, and reusing it is the fastest
way to build the wrong disaster:

* **A structure fire climbs, it does not sweep.** It starts in ONE compartment
  on ONE floor and goes UP — out of a window, up the façade, in at the window
  above (the leapfrog / auto-exposure path), or up a shaft. The storeys BELOW
  the fire floor are essentially untouched. So the signature is a VERTICAL
  stripe of blackened storeys on one elevation with clean masonry under it,
  not a plate-wide gradient.
* **The openings are the fire's exits, and the soot goes ABOVE them.** The
  wildfire skill records this exactly backwards for its own case and says so:
  "A WILDFIRE arrives at ground level and washes UP the outside of a wall...
  A compartment fire vents from the openings and is heaviest under the eaves.
  The first cut assumed the compartment case and looked upside down." This
  file is the compartment case. Every plume tongue is rooted at a window HEAD.
* **These buildings do not burn away.** A timber house is fuel; a masonry or
  concrete block is a non-combustible container for a fuel load that is
  entirely INSIDE it. Fire-induced collapse of a masonry or RC building is
  rare and specific (long-span unprotected steel, or a prolonged burn in an
  already weak URM shell). Consuming the structure the way `damage_flow` does
  produces a rubble pile, and a rubble pile is an EARTHQUAKE. What a burnt-out
  block looks like is a STANDING BLACK SHELL with its floors gone.
* **The debris is inside, and it is dark.** A tornado scatters pale sawn
  timber across the lot (`build-tornado-scenes`: "the scene is PALE, not
  dark"); a fire leaves a black interior seen through empty openings, glass on
  the sidewalk, and a modest apron of spalled render at the wall foot.

WHAT IS REUSED, AND WHY IT IS SAFE
-----------------------------------
Everything about the BUILDING comes from `disaster.quake_flow`: `describe`,
the element records, `fit_interior` (an urban kit shell is hollow and a fire
is only visible through its openings, so the interior has to exist before any
of this reads), `_break` / `_break_box_like`, `_heap`, `_roof_box`,
`_face_patch`, the measured window tables `_G2_WIN_FACES` / `_G_SHOP_FACES`
and the on-face authoring primitives. Those are construction geometry, not
earthquake physics, and they were built for exactly this kit.

Everything about the FIRE comes from `disaster.damage` (the char / scorch / ash
maps, `scorched_material`'s texture compositing) and `disaster.fire` (NVIDIA
Flow). The one wildfire habit deliberately NOT carried over is
`damage.damage_placements` / `damage_flow.BREAK_PLAN`, which fracture a
building to a structural level on the assumption that the structure itself is
burning.

WHERE THE FIRE IS, AND WHERE IT WAS, IS ONE LIST. `disaster.soot_plume.
plan_events` draws the building's discrete fire EVENTS — each a run of
openings on one storey of one elevation, in a state of "flame", "smoulder",
"out" (burnt out: nothing volumetric, only its stain) or "stain" (smoke
damage only) — right after `plan_fire`, and BOTH `r_flames` (the Flow
emitters) and `r_smoke_stain` (the soot skin merged into every module's own
texture) read that one list. So a window with flame coming out of it has a
plume of soot over it by construction, a burnt-out compartment three floors
under the flame front shows its stain and nothing else, and a burnt-out
block is black because every one of its compartments vented for an hour,
not because a level said "paint it black". The model behind the stain — EN
1991-1-2 Annex B's external flame, Heskestad's plume, Riahi & Beyler's
thermophoretic deposition, Beer-Lambert darkness — is documented in that
module, with its citations.

THE SEVERITY LADDER
-------------------
Not EMS-98 grades — those are shaking. A fire officer's ladder, and the
distinction that matters most is between a fire that is BURNING and one that
has BURNT OUT, because those photograph completely differently:

    F0  untouched
    F1  smoke-damaged     staining above a few openings, glass cracked or
                          out on one or two floors. The fire was next door,
                          or it was knocked down early.
    F2  compartment fire  ACTIVE. One or two storeys well alight: flame out
                          of the windows, heavy soot plumes above each,
                          contents burning, glass gone in that band.
    F3  fully involved    ACTIVE and climbing. Four-plus storeys, a vertical
                          stripe of black façade, render peeling / concrete
                          spalling, roof plant tipped, the top of the block
                          venting.
    F4  burnt out         The fire has PASSED. No flame, heavy smoulder
                          smoke; every opening in the involved block empty
                          and black, floors gone or hanging, roof burnt
                          through, the shell standing.
    F5  burnt collapse    The rare one. A burnt-out URM shell that has lost a
                          wall or dropped its floors into itself, or a
                          long-span deck that came down. Charred rubble, no
                          dust — this is what separates it from a quake heap.
    F6  gutted, near gone A building that has burnt for hours and is barely
                          standing: every floor down, the roof gone, more
                          than one elevation lost, and what is left black
                          top to bottom rather than stained in a stripe.
                          Added 2026-08-29 on review — F5 stops short of the
                          state a long-burning block actually ends in, and a
                          near-collapsed building needs a texture to match.

`LADDER[btype][level]` is the recipe list, keyed by the same construction
types `quake_flow.FAMILY_TYPE` assigns, because how a building burns depends
on what it is made of every bit as much as how it shakes:

  urm       masonry shell, TIMBER floors and roof. The floors are the fuel.
            This is the type that gets gutted and the only one that collapses.
  rc        concrete frame with masonry infill. Spalls, does not fall.
  rc_glass  curtain-wall tower. The glass goes in a vertical stripe and the
            structure does nothing at all.
"""

import itertools
import math
import os
import random

# ---------------------------------------------------------------------------
# The ladder
# ---------------------------------------------------------------------------
# F5c ("c" for collapse) sits BESIDE F5, not above it: same fire, a
# different structural outcome. F5 is `fire_collapse` — the burnt-out TOP
# storeys drop into the floors below and all four walls are still there from
# the street. F5c is `partial_collapse` (`disaster/fire_collapse.py`) — PART
# of the shell has come down: one burnt elevation lying in the road with the
# floors behind it sagged, or a corner gone from the fire floor up, and the
# rest of the building standing. Both are real and they photograph
# differently, so a set that wants one asks for it by name (user,
# 2026-08-30: "I want some partial collapse buildings for fire in all sets").
LEVELS = ("F0", "F1", "F2", "F3", "F4", "F5", "F5c", "F6")

# Is there still fire in it? Drives the Flow state and the material finish:
# an active fire is FRESH char (wet-black, still glossy) with flame; a
# burnt-out one is cooled char going to grey ash, and smoke only.
ACTIVE = {"F0": None, "F1": None, "F2": "flame", "F3": "flame",
          "F4": "smoulder", "F5": "residual", "F5c": "residual",
          "F6": "residual"}
FINISH = {"F0": None, "F1": "scorch", "F2": "char", "F3": "char",
          "F4": "ash", "F5": "ash", "F5c": "ash", "F6": "ash"}

# How many storeys the fire is in, as a fraction of the block's height, and
# where it started. A fire that starts on the top floor and a fire that
# starts on the third are the same event and photograph differently, so the
# origin is drawn rather than fixed — but it is drawn LOW-BIASED, because a
# fire is far more likely to start in an occupied lower storey and because a
# fire that started high leaves nothing below it to show the contrast.
BAND = {
    "F0": (1, 1, 0.0),      # untouched: a plan with no events (soot_plume)
    "F1": (1, 2, 0.0),      # (min storeys, max storeys, share above origin)
    "F2": (1, 2, 0.35),
    "F3": (3, 6, 0.75),
    "F4": (4, 99, 1.0),     # everything from the origin up
    "F5": (4, 99, 1.0),
    # F5c is F5's fire with a different structural outcome, so it takes F5's
    # band exactly: everything from the origin up, roof included. The
    # collapse's own failure line is drawn from this band in
    # `fire_collapse.plan_partial_collapse` and can never sit below `origin`.
    "F5c": (4, 99, 1.0),
    # F6 was added to LEVELS/LADDER without a BAND entry, so `plan_fire`
    # raised KeyError on it (soot_png.py, 2026-08-30). Same band as F5.
    "F6": (4, 99, 1.0),
}

LADDER = {
    "urm": {
        "F0": [],
        "F1": [("smoke_stain", {"heavy": 0.45}),
               ("window_burnout", {"frac": 0.35, "empty": False})],
        "F2": [("window_burnout", {"frac": 0.9}),
               ("smoke_stain", {"heavy": 1.0}),
               ("char_facade", {}),
               ("gut_interior", {"frac": 0.6}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F3": [("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.25}),
               ("char_facade", {}),
               ("render_peel", {"rate": 0.30}),
               ("gut_interior", {"frac": 0.85}),
               ("roof_burnthrough", {"frac": 0.20}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F4": [("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.4}),
               ("char_facade", {}),
               ("render_peel", {"rate": 0.42}),
               ("gut_interior", {"frac": 1.0}),
               ("floor_burnthrough", {}),
               ("roof_burnthrough", {"frac": 0.34}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # THE COLLAPSE RUNS FIRST, AND THE ORDER IS LOAD-BEARING. `_els`
        # skips elements a recipe has marked `dead`, so anything that takes
        # away a wall must run BEFORE the passes that author art ON walls.
        # With `smoke_stain` first, the top storey's soot tongues were drawn
        # on modules `fire_collapse` then broke away, and the tongues stayed
        # behind — a row of grey flags standing in the sky over an open shell
        # (uf_bench2 dw_terrace, 2026-08-28). Same argument for the voids and
        # the window frames.
        "F5": [("floor_burnthrough", {}),
               ("roof_burnthrough", {"frac": 0.42}),
               ("fire_collapse", {}),
               ("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.4}),
               ("char_facade", {}),
               ("gut_interior", {"frac": 1.0}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # F5c — PART OF THE SHELL IS GONE, THE REST STANDS. Same fire as
        # F4 (this list is F4's, in F4's order) with `partial_collapse`
        # inserted at the front: one burnt elevation lying in the street with
        # the floors behind it sagged and dropped, the interior on show
        # through the hole, and three untouched elevations still holding the
        # building up. Masonry gets `mode="elevation"` because that IS the
        # masonry mechanism — a URM wall fails OUT OF PLANE, rocking about
        # its foot as one plane, which is why the rubble ends up in the road
        # rather than in a pile against the plinth.
        #
        # COLLAPSE FIRST, for the same reason F5 says so: `_els` skips
        # elements a recipe has marked `dead`, so anything that takes a wall
        # away must run BEFORE the passes that author art ON walls, or the
        # art is left standing in the air after its wall is gone.
        # `roof_burnthrough` still runs ahead of it so the deck exists (and
        # so `partial_collapse` does not have to author one).
        "F5c": [("floor_burnthrough", {}),
                ("roof_burnthrough", {"frac": 0.30}),
                ("partial_collapse", {"mode": "elevation"}),
                ("window_burnout", {"frac": 1.0}),
                ("smoke_stain", {"heavy": 1.4}),
                ("char_facade", {}),
                ("render_peel", {"rate": 0.42}),
                ("gut_interior", {"frac": 1.0}),
                ("expose_interior", {}),
                ("roof_scorch", {}),
                ("street_debris", {"density": 1.3}),
                ("flames", {})],
        "F6": [("floor_burnthrough", {"p": 1.0, "keep_below": 0, "budget": 9}),
               ("roof_burnthrough", {"frac": 0.85}),
               ("fire_collapse", {}),
               ("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 2.0}),
               ("char_facade", {"coverage": 0.92}),
               ("gut_interior", {"frac": 1.0}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {"density": 1.6}),
               ("flames", {})],
    },
    "rc": {
        "F0": [],
        "F1": [("smoke_stain", {"heavy": 0.45}),
               ("window_burnout", {"frac": 0.3, "empty": False})],
        "F2": [("window_burnout", {"frac": 0.9}),
               ("smoke_stain", {"heavy": 1.0}),
               ("char_facade", {}),
               ("gut_interior", {"frac": 0.6}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F3": [("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.25}),
               ("char_facade", {}),
               ("spall", {"rate": 0.26}),
               ("gut_interior", {"frac": 0.9}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F4": [("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.4}),
               ("char_facade", {}),
               ("spall", {"rate": 0.40}),
               ("gut_interior", {"frac": 1.0}),
               ("floor_burnthrough", {"p": 0.35}),
               ("roof_burnthrough", {"frac": 0.22}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # A concrete frame that HAS come down after a fire is the Plasco /
        # Windsor case: a partial collapse of the upper storeys onto the
        # floors below with the lower frame still standing, not a pancake to
        # grade. `fire_collapse` takes the top of the block only.
        # collapse first — see the urm F5 note
        "F5": [("floor_burnthrough", {"p": 0.8}),
               ("roof_burnthrough", {"frac": 0.40}),
               ("fire_collapse", {}),
               ("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 1.4}),
               ("char_facade", {}),
               ("spall", {"rate": 0.5}),
               ("gut_interior", {"frac": 1.0}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # F5c on a FRAME loses a CORNER, not a street wall. A concrete
        # frame is continuous: its infill panels can burn out and fall bay by
        # bay, but the whole elevation does not peel off as one plane the way
        # unreinforced masonry does — that is a URM mechanism and putting it
        # on an office block reads as a bomb. What a frame does after a long
        # fire is drop the corner bay and the slab corners with it (the
        # Windsor tower's own perimeter loss), which is `mode="corner"`.
        # Collapse first — see the urm F5/F5c note.
        "F5c": [("floor_burnthrough", {"p": 0.5}),
                ("roof_burnthrough", {"frac": 0.24}),
                ("partial_collapse", {"mode": "corner"}),
                ("window_burnout", {"frac": 1.0}),
                ("smoke_stain", {"heavy": 1.4}),
                ("char_facade", {}),
                ("spall", {"rate": 0.5}),
                ("gut_interior", {"frac": 1.0}),
                ("expose_interior", {}),
                ("roof_scorch", {}),
                ("street_debris", {"density": 1.3}),
                ("flames", {})],
        "F6": [("floor_burnthrough", {"p": 1.0, "keep_below": 0, "budget": 9}),
               ("roof_burnthrough", {"frac": 0.8}),
               ("fire_collapse", {}),
               ("window_burnout", {"frac": 1.0}),
               ("smoke_stain", {"heavy": 2.0}),
               ("char_facade", {"coverage": 0.9}),
               ("spall", {"rate": 0.8}),
               ("gut_interior", {"frac": 1.0}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {"density": 1.6}),
               ("flames", {})],
    },
    "rc_glass": {
        "F0": [],
        "F1": [("curtain_burn", {"grade": 1}),
               ("smoke_stain", {"heavy": 0.4})],
        "F2": [("curtain_burn", {"grade": 2}),
               ("smoke_stain", {"heavy": 0.9}),
               ("char_facade", {}),
               ("gut_interior", {"frac": 0.5}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F3": [("curtain_burn", {"grade": 3}),
               ("smoke_stain", {"heavy": 1.2}),
               ("char_facade", {}),
               ("gut_interior", {"frac": 0.8}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        "F4": [("curtain_burn", {"grade": 4}),
               ("smoke_stain", {"heavy": 1.4}),
               ("char_facade", {}),
               ("spall", {"rate": 0.22}),
               ("gut_interior", {"frac": 1.0}),
               ("floor_burnthrough", {"p": 0.25}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # NO COLLAPSE ENTRY BY DESIGN. A curtain-wall tower on an RC or steel
        # core has never come down in a fire in the reviewed record (the two
        # that did — WTC 7, Plasco — were neither), and a toppled one reads as
        # a different disaster entirely. F5 on a tower is F4 over more of it.
        "F5": [("curtain_burn", {"grade": 5}),
               ("smoke_stain", {"heavy": 1.5}),
               ("char_facade", {}),
               ("spall", {"rate": 0.35}),
               ("gut_interior", {"frac": 1.0}),
               ("floor_burnthrough", {"p": 0.45}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {}),
               ("flames", {})],
        # NO PARTIAL COLLAPSE ON A TOWER EITHER, for the same reason F5
        # has no `fire_collapse`: a curtain-wall tower on an RC or steel core
        # has never lost part of its shell in a fire in the reviewed record —
        # the cladding burns off (Grenfell, the Address, the Torch) and the
        # structure does nothing at all. F5c on a tower is F5, so that
        # `UF_SET`s and `MF_LEVELS` can name one level across a mixed row
        # without a tower silently falling over. `urban_fire.check` requires
        # every type to carry every level, which is what this entry is for.
        "F5c": [("curtain_burn", {"grade": 5}),
                ("smoke_stain", {"heavy": 1.5}),
                ("char_facade", {}),
                ("spall", {"rate": 0.35}),
                ("gut_interior", {"frac": 1.0}),
                ("floor_burnthrough", {"p": 0.45}),
                ("expose_interior", {}),
                ("roof_scorch", {}),
                ("street_debris", {}),
                ("flames", {})],
        "F6": [("curtain_burn", {"grade": 5}),
               ("smoke_stain", {"heavy": 2.0}),
               ("char_facade", {"coverage": 0.9}),
               ("spall", {"rate": 0.6}),
               ("gut_interior", {"frac": 1.0}),
               ("floor_burnthrough", {"p": 0.9, "keep_below": 0, "budget": 9}),
               ("fire_collapse", {}),
               ("expose_interior", {}),
               ("roof_scorch", {}),
               ("street_debris", {"density": 1.5}),
               ("flames", {})],
    },
}


# ---------------------------------------------------------------------------
# The no-interior gate — COMPUTED from LADDER, never a hardcoded level list.
# ---------------------------------------------------------------------------
# A recipe in this set is the only thing that ever puts the interior ON
# SHOW: `roof_burnthrough`/`floor_burnthrough` cut a hole a camera (or a
# window) can see clean through to whatever is behind it, and
# `fire_collapse`/`partial_collapse` take a wall or the roof away outright.
# Every other recipe in `LADDER` — `window_burnout`, `smoke_stain`,
# `char_facade`, `render_peel`, `spall`, `curtain_burn`, `roof_scorch`,
# `street_debris`, `flames` — works on the SHELL, and `gut_interior`/
# `expose_interior` (present on some of these levels too) already degrade
# to a no-op / their own small, footprint-aware fallback
# (`r_expose_interior`'s catch floor, authored through `_plate`, not the
# BOUNDING-BOX grid `fit_interior` lays down) when `ctx["fit"]` is empty —
# see that function's own docstring. So none of them NEEDS `quake_flow.
# fit_interior`'s full per-storey slab/column/partition/furniture grid to
# exist at all: a building whose ladder run never reaches one of these four
# recipes has a shell that stays closed, its openings going to darkened/
# crazed glass rather than a real hole, and authoring a full interior
# behind it is pure waste — worse, on an L-shaped or multi-tier whole-asset
# building, `fit_interior`'s grid is a `W x D` BOUNDING BOX, so it comes out
# rectangular under a footprint that is not (user, 2026-08-31, reviewing
# `gac_SM_Building_28_F4_o22_SEW_s219`: "this building is L shaped however,
# it's interior is rectangular and so it looks weird ... For building's
# who's insides are not gonna be shown (intact but burnt on the outside)
# don't have any interior").
INTERIOR_EXPOSING_RECIPES = frozenset(
    {"fire_collapse", "partial_collapse", "roof_burnthrough", "floor_burnthrough"})


def shows_interior(btype, level):
    """True if `LADDER[btype][level]` ever puts the interior on show (one of
    `INTERIOR_EXPOSING_RECIPES` is in its recipe list). False means the
    shell stays closed for that (construction type, severity) combination —
    the caller has no reason to pay for a fit-out nobody will ever see
    behind.

    `gac_fire.burn_gac` is the one caller that acts on this (its own
    `fit_storeys` top-up, `INTERIOR_EXPOSING_RECIPES` re-used directly there
    so this stays a single source of truth). The kit-building path
    (`burn_building`'s own `fit_storeys is None` default) does NOT check
    this — that path is frozen (see the `build-urban-fire-scenes` skill's
    bug catalogue and `tools/kit_burn_probe.py`), and every kit style is a
    plain rectangular massing anyway, so `fit_interior`'s box was never
    wrong there to begin with.
    """
    names = set(n for n, _kw in LADDER.get(btype, {}).get(level, ()))
    return bool(names & INTERIOR_EXPOSING_RECIPES)


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
# LINEAR ALBEDO, NOT SCREEN GREY — the single most expensive mistake in the
# earthquake round-1 palette (`quake_flow.A_DEBRIS`): `damage._pbr` writes
# these into OmniPBR's `diffuse_color_constant` and the renderer encodes to
# sRGB, so screen ~= linear ** 0.42. A 0.30 "dark grey" renders at 0.60.
# Fire wants the DARK end of that scale and nothing else: 0.012 linear is
# 0.15 on screen, which is what fresh char measures against a lit wall.
_FLAT = {
    # Fresh char on the outside of a building actively burning: near black,
    # and slightly glossy because it is wet from the hose stream and still
    # off-gassing. Roughness under 1.0 is deliberate; a matte black patch
    # reads as a hole in the geometry rather than as a burnt surface.
    "soot":        ((0.0125, 0.0115, 0.0105), 0.68),   # -> ~0.15 screen
    # The three plume bands. They have to STEP, not blend, and each step has
    # to be visible against the sooted wall under it (~0.20-0.30 on screen at
    # this key) — so the top band is barely darker than the wall and the root
    # is nearly black. A single tone for the whole tongue is what made the
    # first pass read as black panels bolted to the façade.
    "soot_mid":    ((0.0250, 0.0230, 0.0210), 0.74),   # -> ~0.20
    "soot_light":  ((0.0520, 0.0480, 0.0440), 0.82),   # -> ~0.29, the taper
    # Cooled char going to ash: a burnt-out shell days later is GREY, not
    # black, and the difference between F3 and F4 is largely this.
    "ash":         ((0.105, 0.100, 0.094), 0.95),      # -> ~0.38
    # The inside of a burnt-out room seen through an empty window. Not pure
    # black — a black quad in a wall reads as a modelling hole — but close,
    # with the faintest warm bounce a charred room actually has.
    "void":        ((0.0080, 0.0070, 0.0062), 0.90),   # -> ~0.13
    # THERE IS NO TIMBER IN THIS SET, AND THAT IS THE POINT. The first cut
    # carried a warm brown `char_timber` for joists, roof decks and debris,
    # straight from the wildfire palette where a house IS timber. On a brick
    # and concrete block it reads as scorched lumber lying in a masonry
    # street — "the burnt wood texture looks out of place" (user review,
    # 2026-08-28). Everything structural here is charred CONCRETE or
    # calcined masonry, both of which are neutral-to-cool greys.
    "char_concrete": ((0.0175, 0.0170, 0.0165), 0.97),  # -> ~0.17, neutral
    # Masonry that has been through a fire and cooled: calcined brick goes
    # pale and chalky where the soot has spalled off it. The one light tone
    # in the set, and it is what stops a burnt-out shell reading as a
    # silhouette with no material in it.
    "calcined":    ((0.0680, 0.0620, 0.0560), 1.0),     # -> ~0.32
    # Concrete exposed by spalling: PALER than the sooted face around it,
    # which is the whole point — a spall on a burnt wall is a light scar,
    # the inverse of the dark scar a quake spall makes on a clean one.
    # BUT ONLY JUST PALER. At 0.145 (0.44 on screen) against a sooted wall at
    # 0.15-0.25 the scars rendered as bright grey amoebas spattered over the
    # façade — the "cow spots" the earthquake round-1 palette produced in the
    # other direction (uf_bench office_wide, 2026-08-28). Calcined concrete
    # under fire soot is a dirty buff about 1.5x the wall, not 3x.
    "spall_face":  ((0.052, 0.049, 0.045), 1.0),       # -> ~0.29
    # Steel that has been in a fire: blued/oxidised, matte, no shine left.
    "burnt_metal": ((0.030, 0.026, 0.022), 0.85),      # -> ~0.23
    # FIRE-EXPOSED STRUCTURAL STEEL — a joist/rafter stub still upright in a
    # wall pocket, or an internal column/beam, not decorative hardware.
    # `burnt_metal` is WARM at this luminance (R > G > B) and warm-dark with
    # roughness 0.85 is what charred TIMBER looks like, not steel — "rods,
    # probably structural. They looked wooden though" (user review,
    # 2026-08-30). Cool structural steel is the opposite: B > G > R, a shade
    # darker still, with some sheen left — heated steel is not matte black.
    "steel":       ((0.040, 0.043, 0.048), 0.55),      # -> ~0.26, cool grey
    # GLASS IN A FIRE IS A LADDER, NOT ONE GREY. Round 1 bound a single flat
    # dark tone to every pane and that read as a wall of identical matte grey
    # rectangles. Round 2 (still here, below) replaced the single tone with
    # THREE hand-picked flat colours in stacked bands — and on a real pane
    # that read as three PANELS in three different HUES, green over beige
    # over brown, not as dirty glass with fire behind it (prim `pane_b2_193`,
    # user review, 2026-08-29: "there's just 3 rectangles, green, beige and
    # brown. Why can't you have the original one"). The cause: three
    # constants tuned independently, each nearly black in LINEAR terms, and
    # this renderer's sRGB encode stretches a dark colour's HUE far more than
    # its value (`_FLAT`'s own note: 0.30 linear renders at 0.60 screen) — so
    # three colours that all look like "near-black" side by side on a colour
    # picker come out visibly different once encoded. `_GLASS_DEP`, below
    # `fire_glass`, replaces the bands with a genuine gradient between ONE
    # pair of endpoints, so hue cannot drift between steps — only value can.
    "fire_glass":   ((0.022, 0.026, 0.024), 0.86),     # -> ~0.20, the shards
    # The crack line itself: brighter than the pane, because a fracture in
    # glass scatters light rather than absorbing it. This is why a cracked
    # pane reads at all — the cracks are the LIGHT part of it.
    "glass_crack":  ((0.30, 0.33, 0.33), 0.16),        # -> ~0.60
}

# THE SMOKE-DEPOSIT GRADIENT ON A LIT PANE. Two endpoints, LERPED into
# `_GLASS_DEP_STEPS` bands rather than three independently hand-tuned
# colours — see the note above `fire_glass`. `_LO` is a near-nothing tar
# film (glossy still, barely tinted) and `_HI` is opaque soot at the head; a
# pane's own kit glass is left showing below wherever the deposit does not
# reach, rather than painting a "clear" quad over glass that was already
# fine (`_glass_pane`'s `lo` cutoff does the leaving-alone).
_GLASS_DEP_STEPS = 6
_GLASS_DEP_LO = (0.048, 0.041, 0.034)
_GLASS_DEP_HI = (0.013, 0.011, 0.009)
_GLASS_DEP_ROUGH = (0.26, 0.62)   # glossy where the film is thin -> matte at full soot
# TARGET opacity_constant, thin -> heavy. NOT the same thing as the colour
# gradient being dark: this is authored so the material is CORRECT the day
# translucency actually renders (see the note in `materials()`), and in the
# meantime costs nothing extra to set.
_GLASS_DEP_OPACITY_RANGE = (0.30, 0.90)


def _lerp3(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


_GLASS_DEP_OPACITY = {}
for _gi in range(_GLASS_DEP_STEPS):
    _gt = _gi / float(max(1, _GLASS_DEP_STEPS - 1))
    _FLAT["glass_dep{0}".format(_gi)] = (
        _lerp3(_GLASS_DEP_LO, _GLASS_DEP_HI, _gt),
        _GLASS_DEP_ROUGH[0] + (_GLASS_DEP_ROUGH[1] - _GLASS_DEP_ROUGH[0]) * _gt)
    _GLASS_DEP_OPACITY["glass_dep{0}".format(_gi)] = (
        _GLASS_DEP_OPACITY_RANGE[0]
        + (_GLASS_DEP_OPACITY_RANGE[1] - _GLASS_DEP_OPACITY_RANGE[0]) * _gt)
del _gi, _gt


# The masonry behind a lost render. `quake_flow.materials`' `brick` is the
# megascans wrapper bound BY REFERENCE, which samples UV space — on an
# authored, UV-less patch mesh that is one flat crop of the map at whatever
# scale the crop happens to be, i.e. a pasted photograph. `damage._pbr` with a
# texture turns on world triplanar, so the courses come out at their real size
# and line up with the courses on the wall either side of the hole.
_BRICK_TEX = ("airstack://scene_gen/assets/materials/megascans/"
              "Brick_Wall_Worn/T_sexkaitb_1K_B.jpg")
# The floor slabs, beams and piers are authored `_box`es with NO UVs, and
# `quake_flow.materials()["concrete"]` is the megascans `Worn_Pavement.usda`
# bound BY REFERENCE — a UV-space material. On a UV-less box that renders as
# a pale green-and-white lattice, which is what was visible down the roof
# hole where a charred floor should have been (uf_r2f, 2026-08-28). The
# earthquake skill records the same trap one round earlier: "Megascans packs
# bound BY REFERENCE sample UV space on an authored mesh — go through
# `_pbr(texture=...)`", which turns on `project_uvw` and makes the scale
# metric.
# NOT Worn_Pavement — "don't use Worn Pavement material anywhere" (user,
# 2026-08-31), which overrides the MCE freeze for this look: the kit
# buildings' fit-out slabs change with the sliced ones, deliberately.
# `quake_flow._C_TEX` already records the precedent and the reason ("its map
# carries green moss in the joints ... Damaged_Asphalt is a plain grey
# cracked surface; brightened it is concrete, darkened it is the road").
# NO TINT NUDGE IS NEEDED HERE, measured rather than assumed: over the two
# 2K albedo maps, Damaged_Asphalt is BRIGHTER and more NEUTRAL than the map
# it replaces — linear mean 0.1535 vs 0.1167 (x1.32), sRGB per channel
# 0.447/0.421/0.402 against Worn_Pavement's warm 0.386/0.369/0.310. The
# slab therefore moves toward pale neutral concrete, not toward dark
# roadway, at the `_triplanar` call's existing untinted default (with a
# texture bound `damage._pbr` puts `tint` in `diffuse_color_constant` and
# the `rgb` argument is only the fallback if the map will not resolve, so
# there is no tint on this material to soften).
_CONCRETE_TEX = ("airstack://scene_gen/assets/materials/megascans/"
                 "Damaged_Asphalt/T_vizcebf_2K_B.png")


def _triplanar(stage, path, url, rgb, rough, scale):
    """An OmniPBR around a megascans map, world-projected. None if missing."""
    import os as _o
    import scene_generator as sg
    from . import damage
    try:
        res = sg._join_asset_root(url, "")
        if res.startswith("omniverse://") or _o.path.exists(res):
            return damage._pbr(stage, path, rgb, rough, texture=res,
                               scale_uv=(scale, scale))
    except Exception:
        pass
    return None


def _brick_bare(stage, parent):
    """Triplanar brick for an exposed patch, or None if the map is missing."""
    return _triplanar(stage, parent + "/FireLooks/brick_bare", _BRICK_TEX,
                      (0.42, 0.30, 0.25), 1.0, 1.6)


def materials(stage, parent):
    """The urban-fire material set. Superset of `quake_flow.materials`.

    Everything structural (brick, concrete, the debris tints, the plank
    texture) comes from the quake set, because a burnt building is still made
    of the same things; this adds the fire-specific finishes and the three
    char/scorch/ash TEXTURE variants from `damage.char_materials`, which are
    the maps that stop char reading as flat paint.
    """
    from pxr import Sdf, UsdShade

    from . import damage, quake_flow as qf

    out = dict(qf.materials(stage, parent))
    scope = parent + "/FireLooks"
    for key, (rgb, rough) in _FLAT.items():
        path = scope + "/" + key
        m = UsdShade.Material.Get(stage, path)
        if not m:
            m = damage._pbr(stage, path, rgb, rough)
        out[key] = m
    # THE GLASS DEPOSIT WANTS TO BE TRANSLUCENT, AND ON THIS RENDERER IT
    # STILL IS NOT. `enable_opacity` / `opacity_constant` / `opacity_
    # threshold=0` (a blend, not a stochastic cutout) / `opacity_mode=0` is
    # the exact recipe `disaster.ground.overlay_material` uses for the burn
    # scar overlay, which DOES render translucent — but only because
    # `ground.KIT_ARGS` passes `--/rtx/raytracing/fractionalCutoutOpacity` and
    # its pathtracing twin on the Kit command line; without that flag OmniPBR
    # forces fractional opacity to opaque (`detail/vehicles.py`'s whole
    # docstring: "car glass renders OPAQUE today"). Checked 2026-08-29:
    # none of `urban_fire_bench_launch_script.py`, `downtown_fire_launch_
    # script.py`, `urban_fire_city_launch_script.py` or `urban_fire_city250_
    # launch_script.py` pass it, so these bands still render fully opaque on
    # every launch script this dataset's fire benches actually use. Authored
    # anyway — it is free, it is correct, and it makes the fix real the day
    # that flag is added to one of those scripts (a one-line follow-up
    # outside this file, described in the round where this was found). What
    # is load-bearing on TODAY's render is the gradient itself and dropping
    # the old flat "clear" quad — see `_glass_pane`.
    for key, op in _GLASS_DEP_OPACITY.items():
        m = out.get(key)
        sh = UsdShade.Shader.Get(stage, m.GetPath().AppendChild("Shader")) \
            if m else None
        if not sh:
            continue
        sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("opacity_constant",
                       Sdf.ValueTypeNames.Float).Set(float(op))
        sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
    bb = _brick_bare(stage, parent)
    out["brick_bare"] = bb if bb is not None else out["spall_face"]
    cc = _triplanar(stage, parent + "/FireLooks/slab_concrete", _CONCRETE_TEX,
                    (0.30, 0.29, 0.28), 1.0, 0.55)
    out["slab_concrete"] = cc if cc is not None else out["dark_concrete"]
    # The textured char / scorch / ash maps, three UV variants each. A whole
    # elevation bound to ONE flat black is the "dimmer switch" failure the
    # wildfire skill records; these carry alligator checking and ash flecks.
    out["_burn"] = _burn_set(stage, parent)
    return out


# `Burn_Ash_Over_Char.png` is a PALE map (its tint constant is 0.52 linear,
# 0.75 on screen) and `damage._pbr`'s `rgb` cannot pull it down: with a
# texture bound, OmniPBR's `diffuse_color_constant` is what the map REPLACES,
# so the tint is a no-op (the earthquake round-2 finding, `quake_flow`:
# "A tint does nothing once a texture is bound"). `albedo_brightness` is the
# float that does work, and it has to be used — a burnt-out terrace came back
# with a dozen CREAM slabs of debris in the street, the brightest objects in
# the frame on the darkest building in the row (uf_bench2, 2026-08-28).
#
_BURN_BRIGHTNESS = {"char": 1.0, "scorch": 1.0, "ash": 0.55}

# WHICH BURN MAPS AN URBAN BUILDING MAY USE — measured, not guessed
# (`scene_gen/assets/materials/burn/`, mean linear RGB over each map):
#
#   Burn_Char_Ref.png        0.12 / 0.12 / 0.12   neutral dark      USE
#   Burn_Ash_Over_Char.png   0.16 / 0.15 / 0.15   neutral dark      USE
#   Burn_Scorch.png          0.25 / 0.19 / 0.15   BROWN             no
#   Burn_Char_Alligator.png  0.17 / 0.16 / 0.15   wood checking     no
#   Burn_Ash_Flake.png       0.63 / 0.62 / 0.59   white ash         no
#   Burn_Ash_Ref.png         0.52 / 0.53 / 0.53   white ash         no
#
# The wildfire set draws char / scorch / ash together and it is right to: a
# timber house genuinely offers all three surfaces. `Burn_Scorch` is 60 %
# redder than it is blue and `Burn_Char_Alligator` is a photograph of
# cracked WOOD — on a brick façade those two are the whole of the "burnt
# wood texture" complaint. `damage.char_materials(maps=...)` lets the map
# behind each role be swapped, so the urban set keeps the role NAMES the
# rest of the code uses and points the brown one at the neutral map.
_URBAN_MAPS = {"char": "Burn_Char_Ref.png",
               "scorch": "Burn_Ash_Over_Char.png",
               "ash": "Burn_Ash_Over_Char.png"}


def _burn_set(stage, parent, suffix="", grade_mult=1.0):
    """The burn maps an URBAN building may wear, at one brightness GRADE.

    `damage.char_materials` with the two wood-patterned maps swapped out
    (see `_URBAN_MAPS`), and `albedo_brightness` used to trim what is left —
    `damage._pbr`'s `rgb` cannot, because with a texture bound OmniPBR's
    `diffuse_color_constant` is what the map REPLACES (the earthquake
    round-2 finding). `grade_mult` stacks on top of `_BURN_BRIGHTNESS` for
    the DISTANCE-FROM-ORIGIN gradient `_burn_mat` drives (see there); with
    the defaults (`suffix=""`, `grade_mult=1.0`) this is bit-for-bit what
    `materials()` has always authored, which is why that call site did not
    need to change.
    """
    from pxr import Sdf, UsdShade
    from . import damage

    mats = damage.char_materials(stage, parent, maps=_URBAN_MAPS,
                                 suffix="_urban" + suffix)
    for key, b in _BURN_BRIGHTNESS.items():
        bb = b * float(grade_mult)
        if abs(bb - 1.0) < 1e-3:
            continue
        for m in mats.get(key, ()):
            sh = UsdShade.Shader.Get(stage, m.GetPath().AppendChild("Shader"))
            if sh:
                sh.CreateInput("albedo_brightness",
                               Sdf.ValueTypeNames.Float).Set(float(bb))
    return mats


# SCORCH SPREADS AND GETS LIGHTER. `_burn_set`'s three maps used to be bound
# at ONE brightness per finish for the whole building — right for a single
# burning compartment, wrong for an elevation that runs from "this is where
# it started" to "this only caught the plume": stamped at one intensity,
# both ends of that run read as the same flat black rectangle ("have the
# scorched look better instead of just plain black/grey rectangles... scorch
# marks spread and get lighter", user review, 2026-08-29). `albedo_brightness`
# is already the float that does the darkening (see `_burn_set`'s docstring);
# this just drives it from a SEVERITY instead of a constant, at a handful of
# cached steps rather than one material per element.
_GRAD_STEPS = 4
# multiplies `_BURN_BRIGHTNESS[finish]`; 1.0 at the seat of the fire,
# climbing toward a calcined grey at the edge of the burnt reach. Capped
# under 2x — the maps are dark enough already that much more washes out the
# alligator-check detail that is the reason a textured map was used at all.
_GRAD_MULT = (1.00, 1.28, 1.58, 1.90)


def _grad_bucket(sev):
    """0 (at the seat of the fire) .. `_GRAD_STEPS`-1 (the edge of the burnt
    reach), for a 0..1 severity. Monotonic BY CONSTRUCTION — non-increasing
    as `sev` rises — which is the one property `check_scorch()` holds this
    to; it is what turns "distance from the origin" into "how light the
    material is" without an if/elif ladder to keep in sync by hand.
    """
    s = max(0.0, min(1.0, float(sev)))
    return min(_GRAD_STEPS - 1, int((1.0 - s) * _GRAD_STEPS))


def _burn_mat(ctx, finish="char", sev=1.0):
    """One textured burn material, by finish mix, GRADED by how close to the
    seat of the fire this element is. `finish` is char / scorch / ash — the
    same vocabulary `damage._pick` uses. `sev` is 0..1 on `_severity`'s own
    scale (1 at the seat, falling toward 0 at the edge of the burnt reach);
    every call site that has an element's own severity in hand passes it —
    the ones that do not (small debris and litter, not the visible
    ELEVATION) get `sev=1.0`, which reproduces the un-graded set exactly and
    costs no extra material.
    """
    from . import damage
    g = _grad_bucket(sev)
    key = "_burn" if g == 0 else "_burn_g{0}".format(g)
    mats = ctx["mats"].get(key)
    if mats is None:
        # CACHED IN `ctx["mats"]`, KEYED BY GRADE, NOT AUTHORED PER CALL.
        # `damage.char_materials` makes 9 new Shader networks (3 maps x 3 UV
        # variants) every time it runs; without this cache a per-element
        # `sev` would author one set PER ELEMENT rather than per grade step
        # — the exact "hundreds of duplicate materials" this was written to
        # avoid.
        mats = _burn_set(ctx["stage"], ctx["parent"],
                         suffix="_g{0}".format(g), grade_mult=_GRAD_MULT[g])
        ctx["mats"][key] = mats
    return damage._pick(ctx["rng"], finish, mats)


def _debris_mat(ctx):
    """A burn material for something lying on the ground or in a heap.

    ALWAYS THE DARK END. The façade can carry ash — calcined masonry really
    does go grey, and that is most of what separates a cooled ruin from a
    fire still burning — but a 2 m fragment in the street bound to the ash
    map is a pale slab, and a pale slab in a fire scene reads as demolition
    spoil. Debris draws char and scorch only.
    """
    rng = ctx["rng"]
    mats = ctx["mats"]["_burn"]
    key = "char" if rng.random() < 0.72 else "scorch"
    return mats[key][rng.randrange(len(mats[key]))]


def _bind_subsets(stage, placement, mat_fn, p, rng):
    """Rebind a share `p` of one placed module's GeomSubsets.

    EVERY FIRE PASS THAT TOUCHES A WALL GOES THROUGH HERE, and the reason is
    binding STRENGTH, not convenience. `quake_flow._b_bind_over` binds
    `strongerThanDescendants` on the module root, which is correct for a
    Nucleus prop whose materials are bound on its leaves — and fatal for a
    façade, because the next pass's ordinary per-subset bind is then silently
    ignored and the second effect simply does not appear. The smoke wash and
    the char stamp have to COMPOSE (a sooted wall with charred patches on it),
    so both work at subset granularity with ordinary bindings and the later
    pass overwrites the share it draws.

    Subset granularity is also what makes char read as char: a whole-module
    rebind paints one flat tone over a 5 m panel, and real soot deposition is
    mottled. This is the same walk `modular_house.apply_palette` and
    `damage.char_placements` use, because it is the granularity this kit
    actually binds materials at.
    """
    from pxr import Usd, UsdGeom, UsdShade

    path = placement.get("prim_path")
    if not path:
        return 0
    root = stage.GetPrimAtPath(path)
    if not root or not root.IsValid() or not root.IsActive():
        return 0
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        for t in ([s.GetPrim() for s in subsets] or [prim]):
            if p < 1.0 and rng.random() > p:
                continue
            m = mat_fn()
            if m is None:
                continue
            UsdShade.MaterialBindingAPI(t).Bind(m)
            n += 1
    return n


# ---------------------------------------------------------------------------
# The fire plan
# ---------------------------------------------------------------------------
def plan_fire(info, level, rng, origin=None, sides=None):
    """Which storeys are involved, from where up, on which elevations.

    THE ORIGIN IS DRAWN LOW AND THE FIRE GOES UP FROM IT. Nothing below the
    origin is damaged at all — that clean band under a black stripe is the
    single strongest cue that this is a building fire and not a bombing, a
    quake or a wildfire, and a scene that loses it looks like generic ruin.

    Returns a dict: `origin` (storey index), `storeys` (sorted list, involved,
    lowest first), `top` (highest involved storey), `sides` (elevations the
    fire is venting through), `roof` (does it reach the roof).
    """
    # THE FIRE IS IN ONE MASS, AND IT IS THE TALL ONE. A `tower` or
    # `block_office` style is a podium carrying separate slabs, each with its
    # own storey numbering starting at the podium roof — so a band planned on
    # the main mass and applied to every element sets the podium AND the same
    # storey numbers of every tower alight at once, which is four unrelated
    # fires in one building. Pick the mass with the most storeys (a podium is
    # 2-4 and a tower is 10-30, so this is never ambiguous) and leave the rest
    # untouched.
    mtag = max(info["masses"].items(),
               key=lambda kv: (len(kv[1]["levels"]), kv[0] == "main"))[0]
    m = info["masses"][mtag]
    n = max(1, len(m["levels"]))
    lo, hi, _share = BAND[level]
    # LOW-BIASED ORIGIN. u**1.7 puts ~60 % of draws in the bottom third of the
    # block. On a 2-3 storey building it is almost always the ground or first
    # floor, which is where a shop or a kitchen is.
    if origin is None:
        origin = int(min(max(0, n - 2), (n * (rng.random() ** 1.7))))
    origin = max(0, min(n - 1, int(origin)))
    # ...and low enough that the band's MINIMUM fits above it. A city manifest
    # estimates storeys host-side (H / period); the sliced grid can come back
    # shorter, and an origin pinned 2 storeys under a 16-storey roof with an
    # F3 band (min 3) drew `randint(3, 1)` — "empty range for randrange()
    # (3, 2, -1)" (city_4 bakes 10/11, 2026-08-30). Only activates where the
    # call previously raised, so no existing plan changes.
    lo_min = int(BAND[level][0])
    if BAND[level][1] < 99:
        origin = min(origin, max(0, n - lo_min))
    if hi >= 99:
        # F4/F5 mean "the fire went through everything above where it
        # started", so the band runs to the TOP OF THE MASS and `roof` is
        # true by construction. Drawing k here instead left a `commercial`
        # F4 with its top storey and its whole roof pristine — from directly
        # above, the one view this dataset is actually shot from, nothing at
        # all had happened to it.
        k = n - origin
    else:
        k = rng.randint(lo, min(hi, n - origin))
    storeys = list(range(origin, min(n, origin + k)))
    if sides is None:
        # The elevation the fire vents through is the one the compartment
        # opens onto, so it is ONE side plus, when the fire is big enough to
        # have gone right through the floor plate, the side round the corner
        # from it. Four-sided involvement is a fully-developed building fire
        # and belongs to F4/F5 only.
        allsides = ["S", "E", "N", "W"]
        rng.shuffle(allsides)
        n_side = 1 if level in ("F1", "F2") else (2 if level == "F3" else
                                                  rng.randint(2, 4))
        sides = tuple(allsides[:n_side])
    return {"origin": origin, "storeys": storeys, "top": storeys[-1],
            "sides": tuple(sides), "n_storeys": n, "mass": mtag,
            "roof": storeys[-1] >= n - 1, "level": level,
            "state": ACTIVE.get(level), "finish": FINISH.get(level) or "char"}


def _el_jitter(e):
    """A stable -0.5..0.5 for one element, from its own coordinates.

    Rolled by hand rather than with `hash()`, which Python randomises per
    process, so a building is the same building on every run and in the bake.
    """
    h = 0
    for q in (e.get("x", 0.0), e.get("y", 0.0), e.get("z", 0.0)):
        h = (h * 131 + int(round(float(q) * 100.0))) & 0xFFFFFFFF
    return (h % 1000) / 1000.0 - 0.5


def _severity(ctx, storey, mass=None, e=None):
    """0..1 — how hard THIS storey burned. Zero outside the burning mass.

    `e` (an element record) adds a stable per-module wobble to the band
    EDGES. Without it every module of a storey takes the same value and the
    top and bottom of the burnt block are razor-straight lines running the
    full width of the elevation — two hard horizontal rules across
    `apartment_tall` (uf_bench2, 2026-08-28). A real fire through a floor
    plate is ragged: some bays over the fire floor are black to the cill
    above and their neighbours are barely marked.

    Peaks one storey ABOVE the origin and tapers up, because the compartment
    of origin is starved (it burns out first and its own ceiling shields it)
    while the floor above it takes the full leapfrog plume and the one above
    that takes what is left. Below the origin it is exactly zero: the cut has
    to be hard, not a fade, or the clean band stops reading.
    """
    f = ctx["fire"]
    if mass is not None and mass != f["mass"]:
        return 0.0
    if storey < f["origin"]:
        # ONE STOREY OF SPILL DOWNWARD, and no more. The hard cut below the
        # origin is the signature and it stays — but at exactly zero it put a
        # BRIGHT WHITE seven-metre arcade under a burnt-out shell (uf_bench
        # commercial, 2026-08-28), which reads as two buildings stacked. Smoke
        # banks down a floor, the hose runs down the face, and the storey
        # immediately under a fire is grubby. Light: a stain, never a burn.
        if storey != f["origin"] - 1:
            return 0.0
        return max(0.0, 0.16 + (0.16 * _el_jitter(e) if e is not None else 0.0))
    d = storey - f["origin"]
    n = max(1, len(f["storeys"]))
    if d >= n:
        # Above the involved band: the plume still stains, but only just, and
        # only for a couple of storeys.
        return max(0.0, 0.30 - 0.12 * (d - n))
    peak = 1.0 if n == 1 else 0.72 + 0.28 * math.sin(math.pi * min(1.0, (d + 0.6) / n))
    if e is not None:
        # only the EDGES wobble: full strength in the middle of the band,
        # +-0.35 at the first and last involved storey
        edge = (1.0 - min(d, n - 1 - d) / max(1.0, (n - 1) / 2.0)) if n > 1 else 1.0
        peak += 0.7 * _el_jitter(e) * max(0.0, edge)
    return max(0.0 if e is not None else 0.25, min(1.0, peak))


# ---------------------------------------------------------------------------
# Reachability: a burnt element must be connected to the origin through
# other burnt elements
# ---------------------------------------------------------------------------
# "some parts of the building look burnt on the other side of the building
# that isn't even on fire... make sure the burnt parts are at least
# connected" (user review, 2026-08-29). MEASURED CAUSE: `r_char_facade` had
# two independent escapes from the side check `_severity` alone cannot
# provide (it only knows STOREY distance, never side):
#
#   1. its "round the corner" bleed (`sev *= 0.28`) fired for ANY side not
#      in `f["sides"]`, including the side directly OPPOSITE the fire —
#      there is no such thing as "the corner" between two elevations that do
#      not share one, so a fire on the south face was blackening the north
#      face too, at reduced but very visible strength;
#   2. its roof/parapet branch skipped the side check ENTIRELY —
#      `sev = _severity(ctx, f["n_storeys"] - 1, e["mass"])`, no `* 0.28`,
#      no side test at all — so a parapet on every one of the four
#      elevations charred at full strength as soon as the fire got close to
#      the top storey, whether or not that elevation was ever alight
#      ("...there is at least one place where a parapet is charred
#      regardless of side, with a comment about the roofline" — this was it).
#
# `r_smoke_stain`'s wall wash had the SAME escape for parapet/parapet_corner
# (skipped the `e["side"] not in f["sides"]` test outright for those two
# roles), and `r_roof_scorch`'s parapet loop had a THIRD, weaker version of
# it (a flat 55% chance regardless of adjacency, gated only by how close to
# the top the fire got, never by which side).
#
# `_side_reach` is the one place this is decided now — a real adjacency
# constraint over the SIDE, which is a field every element in the table
# already carries (`quake_flow._side_of` assigns S/E/N/W by nearest wall
# line, and does it for roof and parapet pieces exactly the same way it does
# for walls). A rectangular footprint's four elevations form a 4-cycle
# (`_SIDE_RING`); a side is reachable from the fire's own `f["sides"]` if it
# IS one of them, if it shares a CORNER with one (`_side_neighbors`, one
# hop), or — for the roof and its parapet only — if the fire's vertical
# reach already includes the top storey (`f["roof"]`), because the deck is
# one continuous horizontal plane and a hot side's wall reaching the coping
# is itself an unbroken chain of burnt elements into the roof envelope. Any
# side that is none of those gets NOTHING: not a smaller number, zero.
_SIDE_RING = ("S", "E", "N", "W")

# What a corner-adjacent (but not actually alight) elevation keeps of full
# severity. Unchanged from the number `r_char_facade` always used for this —
# only WHERE it applies changed, from "everywhere" to "one hop only".
SIDE_BLEED = 0.28


def _side_neighbors(side):
    """The two elevations that share a CORNER with `side` on a rectangular
    footprint — the only two an unlit elevation can plausibly pick up scorch
    from without a fire of its own ("a fire venting on the south face still
    blackens a metre or two round the corner", the original `r_char_facade`
    comment — kept, but now it is the ONLY discount a cold side can get)."""
    i = _SIDE_RING.index(side)
    return (_SIDE_RING[i - 1], _SIDE_RING[(i + 1) % len(_SIDE_RING)])


def _side_reach(ctx, side, roof_like=False):
    """0 / `SIDE_BLEED` / 1.0 — how much of an element's severity on `side`
    survives contact with which elevations are actually burning.

    `roof_like` is for the roof deck and its parapet: once the fire's own
    reach already includes the top storey (`f["roof"]`), every side of the
    parapet is reachable THROUGH the continuous roof plane even though its
    own wall never vented — the one place a chain of burnt elements
    legitimately crosses to a non-adjacent elevation, and it is gated on the
    vertical chain (a hot side's wall reaching the coping) being unbroken,
    which is exactly what `f["roof"]` already records. Below that, a
    roof/parapet piece is reachable exactly like a wall on the same side.
    """
    f = ctx["fire"]
    if roof_like and f.get("roof"):
        return 1.0
    if side in f["sides"]:
        return 1.0
    for s in f["sides"]:
        if side in _side_neighbors(s):
            return SIDE_BLEED
    return 0.0


# ---------------------------------------------------------------------------
# 1. THE SIGNATURE: the wall wash, and the face-authoring primitives every
#    other recipe in this file draws on (void quads, panes, cracks, scars)
# ---------------------------------------------------------------------------
# THIS USED TO ALSO DRAW A PAINTED SMOKE TONGUE OVER EACH OPENING — flat face
# geometry, three stacked bands widening and fading up the wall from a window
# head. It read as a decal stuck to the façade rather than as smoke ("if this
# is meant to look like smoke then it's doing a bad job... place some more
# fires that are smoke only", user review, 2026-08-29, prim `plume_b2_858`)
# and it is gone. `r_flames` now puts real NVIDIA Flow smoke at the same
# openings (`_flame_sources` called with `state="smoke"`) plus sources on the
# gutted floors themselves (`_interior_smoke`) — volumetric, so it actually
# rises, instead of a mesh painted on the masonry. What is left here is the
# wall wash (soot composited into the module's own cladding texture, see
# `r_smoke_stain`) and the flat-mesh authoring primitives every stain, void,
# pane and scar in this file goes through.
#
# `PLUME_PROUD` keeps its name — it is still `_face_polygon`'s default offset
# and half the file calls with that default.
PLUME_PROUD = 0.006
# A SPALL HAS TO SIT IN FRONT OF A SOOT STAIN. Both are flat meshes on the
# same wall plane, so a scar authored at the quake path's 4-7 mm can end up
# behind whatever is stamped on the wall over it. It is also physically
# right: the spall took the soot off with it.
# `quake_flow._face_patch` cannot be used for this: its shadow ring is
# hard-coded at PATCH_RING_PROUD (7 mm), so raising only the fill puts the
# ring behind the fill. `_scar` below draws the pair.
SCAR_HALO_PROUD = 0.030
SCAR_PROUD = 0.036


def _stamp_pt(ctx, fr, u, v, out):
    """`quake_flow._b_face_pt`, then — on the SLICED path only — the point is
    moved onto the MEASURED façade plane of that elevation.

    A kit module's frame depth is its wall face, so a stamp at `out` sits on
    the wall. A sliced GAC piece's frame depth is its bbox extent, which a
    cornice, an awning or an interior floor face can push a metre or two
    either way: spall patches and halos hung 0.6-1.8 m in front of
    SM_Building_23 and up to 15 m out on SM_Building_09 (fire_row3f,
    2026-08-30) — the "floating debris" beside every burnt GAC wall.
    `gac_fire.prepare` measures the plane per elevation from the window
    faces (`fire["planes"]`); the kit path has no planes and is untouched.
    """
    from . import quake_flow as qf
    x, y, z = qf._b_face_pt(fr, u, v, out)
    planes = (ctx.get("fire") or {}).get("planes") or {}
    if not planes:
        return (x, y, z)
    yaw = fr[2]
    nx, ny = math.sin(yaw), -math.cos(yaw)          # outward normal
    side = "E" if abs(nx) >= abs(ny) and nx > 0 else "W" if abs(nx) >= abs(ny) \
        else "N" if ny > 0 else "S"
    plane = planes.get(side)
    if plane is None:
        return (x, y, z)
    sign = 1.0 if side in ("E", "N") else -1.0
    if side in ("E", "W"):
        return (float(plane) + sign * float(out), y, z)
    return (x, float(plane) + sign * float(out), z)


def _face_polygon(ctx, fr, u0, v0, outline, mat, out=PLUME_PROUD, kind="plume",
                  owner=None):
    """Author a closed (du, dv) outline as one flat mesh on a wall face.

    `owner` is the element the art belongs to; the path is recorded against
    it in `ctx["face_art"]` so a recipe that later takes that module away can
    take its stains with it (`_drop_face_art`)."""
    from . import quake_flow as qf
    pts = [_stamp_pt(ctx, fr, u0 + du, v0 + dv, out) for du, dv in outline]
    n = len(pts)
    if n < 3:
        return None
    # fan from the centroid — the outline is star-shaped about it by
    # construction (radius is a positive wobble on a monotone base)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    cz = sum(p[2] for p in pts) / n
    P = [(cx, cy, cz)] + pts
    counts, idx = [], []
    for i in range(n):
        counts.append(3)
        idx += [0, 1 + i, 1 + ((i + 1) % n)]
    path = qf._b_face_mesh(ctx, (P, counts, idx), mat, kind)
    if owner is not None and path:
        ctx.setdefault("face_art", {}).setdefault(id(owner), []).append(path)
    return path


def _drop_face_art(ctx, el):
    """Deactivate every stain, void and frame authored on this module.

    Belt and braces behind the ladder ordering: a recipe that removes a wall
    calls this so its surface art cannot be left hanging in the air."""
    from . import quake_flow as qf
    n = 0
    for pth in ctx.get("face_art", {}).pop(id(el), ()):
        n += 1 if qf._deactivate(ctx["stage"], pth) else 0
    return n


# ---------------------------------------------------------------------------
# THE WALL WASH IS THE SOOT SKIN — one image per building, from its fire
# events, merged into every module's own texture
# ---------------------------------------------------------------------------
# Three earlier designs are gone from here, and the history matters because
# each one fixed the previous one's symptom and kept its cause:
#   1. a per-element `scorched_material` rebind — the burnt region's edge was
#      a module rectangle BY CONSTRUCTION ("the scorching still looks too
#      rectangular", 2026-08-29);
#   2. `wall_overlay`, one level-set mask per side revealed as opacity on a
#      proud quad — ragged edges, but the mask was a per-ROW severity curve
#      broadcast across the wall, so it was still a band; and it needed an
#      RTX flag to blend at all;
#   3. `facade_bake.building_skin`, the perimeter-unwrapped skin with X-shaped
#      plumes cropped per prim — right delivery, wrong SOURCE: it scattered
#      its own random plume seats across the storey band while `r_flames`
#      picked its own openings, so soot and flame never agreed ("the pattern
#      looks completely wrong", 2026-08-30).
# `disaster.soot_plume` keeps design 3's delivery (one unwrapped skin, one
# crop per prim, merged into the prim's own base map) and replaces its
# source: the skin is rasterised FROM `ctx["fire"]["events"]`, the same list
# `r_flames` places its emitters from, with a cited physical model for what
# each event does to the wall. See that module. What is left here is the
# geometry every pass shares (`_wall_run_frame`, still used by the placement
# test) and the binding (`_bind_soot`).


def _wall_run_frame(run, side):
    """(fr, L) spanning a whole contiguous run of same-side elements.

    `fr` borrows its depth/dw from the LEFTMOST element's own measured
    piece — the kit's wall pieces on one side of one style share a
    thickness, so this is exact for the ordinary case and at worst a few cm
    off for a mixed one.
    `L` runs from the leftmost piece's own origin to the RIGHTMOST piece's
    far edge (not `len(run) * module`, which assumes uniform module widths
    a corner piece beside ordinary bays does not have).
    """
    from . import quake_flow as qf

    # SORT ALONG THE FRAME'S OWN AXIS, NOT ALONG RAW `lx`/`ly`.
    #
    # A piece runs along its own local +X, and `_piece_frame` pivots it at
    # that end — but which WORLD direction that is depends on the elevation:
    # S faces +X, N faces -X, E faces +Y, W faces -Y. Sorting by ascending
    # `lx` therefore picks the correct end of an S or E run and THE WRONG END
    # of an N or W one, so the frame was anchored at the far end of half the
    # walls in the building and the quad ran off the far side of them.
    #
    # MEASURED (`tests/test_wall_overlay_placement.py`, 2026-08-29, before
    # this fix): 224 of 360 wall runs misplaced, always on N and W, always by
    # exactly the run's own length, and identically at every world position —
    # `apartment` N had the overlay at 0.00..20.07 while its wall was at
    # -15.00..5.07. `L` was right all along; only the anchor was wrong. This
    # is the "the actual transparent part isn't at the correct locations for
    # these buildings" report (user, 2026-08-29), and it is why the overlay
    # looked right on some elevations of the same building and wrong on
    # others.
    #
    # Projecting onto the axis makes the ordering frame-agnostic, so it stays
    # correct if a style ever yaws its masses.
    probe = None
    for e in run:
        probe = qf._piece_frame(e)
        if probe is not None:
            break
    if probe is None:
        return None, 0.0
    ax, ay = math.cos(probe[2]), math.sin(probe[2])

    def _t(e):
        f = qf._piece_frame(e)
        return 0.0 if f is None else (f[0] * ax + f[1] * ay)

    run = sorted(run, key=_t)
    left = run[0]
    fr = qf._piece_frame(left)
    if fr is None:
        return None, 0.0

    # LENGTH FROM THE RUN'S MEASURED EXTENT, not from the last piece's own
    # width. `(_t(right) - _t(left)) + fr_r[3]` assumes the rightmost piece
    # BY ORIGIN also reaches furthest along the run axis and that its width
    # is measured along that axis — neither holds when a piece is yawed
    # differently from the run (a returned end, a wing junction), and the
    # quad then ran PAST the corner. MEASURED
    # (`tests/test_wall_overlay_placement.py`): 92 runs overshot by 0.96 to
    # 4.00 m, `block_residential`'s wings by exactly one 4 m module on a 24 m
    # wall. Projecting both ends of every piece and taking the span is what
    # the test does independently, so the two now agree by construction.
    lo = hi = None
    for e in run:
        f = qf._piece_frame(e)
        if f is None:
            continue
        ex, ey = math.cos(f[2]), math.sin(f[2])
        for px, py in ((f[0], f[1]), (f[0] + ex * f[3], f[1] + ey * f[3])):
            t = px * ax + py * ay
            lo = t if lo is None else min(lo, t)
            hi = t if hi is None else max(hi, t)
    if lo is None:
        return None, 0.0
    return fr, max(0.5, hi - lo)


def _soot_skin(ctx, heavy):
    """The building's soot skin, rasterised ONCE from its fire events
    (`soot_plume.skin`) and kept on the ctx so every role's pass — walls,
    corners, parapets, balconies — crops the same image. `heavy` is the
    ladder's own `smoke_stain` knob and multiplies the events' venting
    durations (`soot_plume.DURATION_S`).

    `SOOT_SKIN_DIR` in the environment writes each building's skin as a PNG
    there (over a flat grey), the cheapest possible look at what the bake
    is about to merge into the modules.
    """
    import numpy as np
    from . import soot_plume as spl

    sk = ctx.get("soot_skin")
    if sk is not None:
        return sk
    f = ctx["fire"]
    events = f.get("events") or []
    if not events:
        return None
    # the skin's noise from the events' own seed — NOT a draw on the shared
    # rng, which would shift every recipe after `smoke_stain`
    nrng = np.random.default_rng(spl.event_seed(ctx) ^ 0x5EED)
    # `burn_zone` is set by `fire_collapse.r_partial_collapse` (F5c only) and
    # is `None` everywhere else, which is a strict no-op in `spl.skin` — the
    # rest of the ladder's soot is byte-identical. It says where a wall has
    # gone and flame was therefore directly on what is left round the hole.
    sk = spl.skin(ctx, events, nrng, finish=f.get("finish") or "char",
                  glass=(ctx["info"]["type"] == "rc_glass"),
                  duration_scale=float(heavy),
                  burn_zone=f.get("burn_zone"))
    ctx["soot_skin"] = sk
    snap = os.environ.get("SOOT_SKIN_DIR")
    if snap:
        try:
            os.makedirs(snap, exist_ok=True)
            spl.save_skin_png(sk, os.path.join(
                snap, "skin_{0}.png".format(ctx["tag"])))
        except Exception:
            pass
    return sk


SOOT_BAKE_PX = int((os.environ.get("SOOT_BAKE_PX") or "768").strip() or 768)
# 256 for a SLICED piece (2026-08-30): the per-piece bakes were the single
# largest VRAM item of the first assembled row — 864 MB of content-hashed
# maps nothing can share across buildings (`tools/bake_vram_census.py`) —
# and a 3-4 m pier at 256 px is still ~70 px/m for a soft gradient. The kit
# path's 768 (`SOOT_BAKE_PX`) is FROZEN with the rest of the MCE look.
SOOT_BAKE_PX_SLICE = int((os.environ.get("SOOT_BAKE_PX_SLICE") or "256").strip() or 256)
# PER-PIECE RESOLUTION POLICY (2026-09-01, VRAM pass). `SOOT_BAKE_PX_SLICE`
# is a flat 256 for every sliced piece regardless of its own size — measured
# (`tools/soot_piece_size_probe.py` against the real `city_138` GAC corpus)
# 42% of sooted GAC pieces have a horizontal bbox diagonal under
# `SOOT_PIECE_SMALL_M`: piers, bay strips, small balconies that a 256 px
# canvas is massive overkill for. Dropping just THOSE to
# `SOOT_BAKE_PX_SLICE_SMALL` measured a 31% cut in the probed buildings'
# soot VRAM (458.6 -> 315.1 MB across 16 real bakes) for zero visible loss
# — a 3-4 m pier does not carry more soot detail than a 128 px canvas can
# hold. `SOOT_BAKE_PX_SLICE` (256) is kept for anything at or above the
# threshold, so a whole facade run or a large block face is untouched. Only
# ever applies to the SLICED path (`one_off` in `_bind_soot`) — the kit
# path's 768 stays exactly as frozen as the comment above says.
SOOT_BAKE_PX_SLICE_SMALL = int(
    (os.environ.get("SOOT_BAKE_PX_SLICE_SMALL") or "128").strip() or 128)
SOOT_PIECE_SMALL_M = float(
    (os.environ.get("SOOT_PIECE_SMALL_M") or "6.0").strip() or 6.0)
# a piece whose own crop-mean soot coverage never reached this floor is
# barely touched regardless of size — the same "OR", not "AND", applies as
# for size: a big wall the plume only grazes does not need a big canvas
# either. Well under `SOOT_TONE_MIN` (0.35, the point flat-tone takes over
# entirely) on purpose — this only downgrades resolution, it never skips
# the bake.
SOOT_PIECE_LOW_COV = 0.15
# a subset whose faces do not face OUT of the building (a ceiling, a floor,
# a painted office interior behind the glass) cannot show its soot from
# outside and is not baked at all
SOOT_FACING_MIN = 0.15
# ...EXCEPT THAT ON A SLICED PIECE THE FAKE INTERIOR *IS* THE OUTSIDE.
# A GAC/downtowncity façade models the room behind each window as a shallow
# box — a floor plane, a ceiling plane and an inward-facing pane, on the
# TILED `M_Building_Floor` / `M_Buildings_Ceiling_03` / `M_Glass_In_*`
# atlases — and the camera reads it straight through the opening. The
# normal test above can never pass any of it: a ceiling/floor plane is
# HORIZONTAL (`n . out == 0` whichever way it faces) and the inner pane
# points back INTO the building. Measured on the two pieces the fire_dtc3
# review named (`tools/piece_face_probe.py`, 2026-08-30):
#
#   SM_19 pier_N_2_05_0027 (1.31 m deep)  mat_1 Glass_In   18 faces, 0 out,
#                                         all n.out<0, 0.63-0.74 m back
#                                         mat_2 Floor       9 faces, 6 horiz,
#                                         0.95-1.28 m back
#                                         mat_8 Ceiling_03  6 faces, 6 horiz,
#                                         0.95-1.20 m back
#   SM_26 corner_NE_0_10_0070 (1.79 m)    mat_2 Ceiling_03  4 faces, 4 horiz,
#                                         1.09-1.78 m back
#
# — every one of them thrown away as "inward" while the brick and concrete
# subsets of the SAME piece carried the plume ("everything around it is
# burnt except this ... repeating rectangular pattern", user review of the
# fire_dtc3 bench, 2026-08-30). So a face also counts as visible when it
# lies in the piece's own OUTER SHELL: within `SOOT_SHELL_D` of that piece's
# outward extreme, measured along the elevation's outward direction.
#
# 2.5 m clears the deepest fake interior measured (1.78 m) and the deepest
# ring corner piece (2.08 m over SM_26's 56 corners) with margin, while
# staying well under one GAC storey (3.84 m) — so it still excludes the
# inner two thirds of a wall piece (mean depth 4.73 m) and 7.5 m of a core
# piece (9.96 m), which is the geometry a camera genuinely cannot see
# through an intact shell.
SOOT_SHELL_D = 2.5
# ...AND WHAT THE SHELL RULE ADMITS CANNOT BE BAKED THROUGH ITS UVs.
# Binding those subsets was only half the job (user review of the LIVE row-5
# bench, 2026-08-31: still "unburnt with a repeating rectangular pattern").
# The soot skin is a PER-ELEVATION PROJECTION addressed by (distance along
# the wall, z) — `soot_bake.sample_skin` ignores depth entirely — so a
# fake-interior CEILING or FLOOR plane, being horizontal, has all of its
# texels on ONE z and therefore samples ONE ROW of the skin. Whether that
# row lands inside a plume tongue or in the gap between two is a lottery on
# the plane's height, and the bake either soaks the subset or deposits
# nothing at all. Measured on the two pieces the review named
# (`tools/soot_skip_probe.py`'s bake instrumentation — the alpha the
# composite was actually given):
#
#   SM_19 pier_N_2_05_0027    mat_8 Ceiling_03   alpha mean 0.024 max 0.091
#                             mat_2 Floor        alpha mean 0.092 max 0.380
#                             mat_7 the WALL     alpha mean 0.263 max 0.929
#   SM_19 pier_N_2_06_0044    mat_8 Ceiling_03   alpha mean 0.029 max 0.120
#   SM_26 corner_NE_0_10_0070 mat_2 Ceiling_03   alpha mean 0.932 (it won)
#
# — the wall on the SAME piece integrates a whole storey of rows and comes
# out at 0.338 mean luminance while its ceiling stays at 0.733, i.e. the
# clean tile. Across a whole building 4-6% of all bakes deposit a mean alpha
# under 0.05: a correctly-bound `sootbake_*.png` copy that renders CLEAN.
#
# So a subset the shell rule admits but that carries NO genuinely outward
# face — fake interior by GEOMETRY, not by material name — takes the flat
# soot tone graded by its module's own crop coverage instead. That number is
# integrated over the module's whole z span (`_r_soot_overlay` sets
# `_soot_cover`), so it cannot fall down the gap between two tongues, and it
# is the same machinery the untextured path below already uses. The
# threshold is that path's own 0.35: under it a module is merely fringed,
# and the file's existing warning applies — a lightly-fringed module going
# flat dark reads as rectangles, not as a film — so those fall through to
# the positional bake exactly as before.
SOOT_TONE_MIN = 0.35
# ...AND THE TONE IS A FLOOR, NOT ONLY A FAKE-INTERIOR RULE.
# "The main thing to fix was the weird pattern of unscorched prims" (user,
# on `~/fire_previews/fire_dtc3/3_SM_Building_19_F3_obl.png`): a periodic
# checkerboard of BRIGHT PALE pier caps and spandrel end-bands surviving
# across a soot-black facade, beside and above the flaming windows. Those
# are GENUINELY OUTWARD narrow bands, so neither the shell rule nor the
# fake-interior rule reaches them; they are baked positionally and the bake
# comes back near-clean, for the same one-row reason (a 0.3 m band spans a
# handful of skin rows, and if none of them is inside a tongue the composite
# is the base map).
#
# The honest test is therefore not the sampled alpha but THE RESULT: after
# `soot_bake.bake_module` has composited, is the map this subset actually
# samples still bright? Measured over the covered texels on the two review
# buildings, the two populations separate cleanly --
#
#   sooted and reading burnt   0.166 0.202 0.216 0.239 0.271 0.338 0.351
#   still reading pale         0.502 0.564 0.622 0.625 0.657 0.722
#
# -- so 0.45 sits in the gap with room either side. A subset over it falls
# back to the graded tone instead of shipping the near-clean copy.
SOOT_PALE_MAX = 0.45

# THE FLOOR HAS TO TELL "STILL PALE" FROM "PALE ON AVERAGE, PATTERNED FOR
# REAL". The checkerboard case above (`SOOT_PALE_MAX`'s own history) is a
# NARROW band that spans a handful of skin rows and none of them lands in a
# tongue — the composite comes back near-uniform AND pale, because it is
# just the base map, unchanged. That population has LOW spread. But on a
# genuine ring piece well inside the band (`wall_W_0_10_0034`, `SM_Building_
# 23` F5, storey 10 of a 7-27 burn, S+W venting) the SAME floor was firing on
# a subset whose composite legitimately carries a real gradient — a bright
# base colour the plume only partially reaches still averages above 0.45 even
# though two of that subset's neighbours on the identical piece read a clean
# 0.503/0.268-mean gradient with real structure — and the floor threw the
# gradient away for one FLAT, UNNUMBERED tone, "haven't gotten the scorch
# pattern well" (user review, 2026-08-31, the piece named directly). The
# checkerboard population and this one are told apart by SPREAD, not mean: a
# composite that is really just the untouched base is near-flat by
# construction (nothing but resampling noise varies across it); a composite
# the plume genuinely reached unevenly is not. `SOOT_PALE_SPREAD_MIN` /
# `SOOT_PALE_STD_MIN` ARE AN OR, not an AND: either signal of real structure
# is enough to keep the bake — both have to read near-zero (a genuinely flat
# result) before the floor is still allowed to fire. Thresholds picked
# against the two populations `SOOT_PALE_MAX` itself was calibrated on: a
# 0.3 m checkerboard band spanning a handful of skin rows samples close to
# ONE row of the skin, so its own resampling noise floor is the bar — a
# handful of `px`-canvas resample artefacts, not a real gradient. 0.05 of
# luminance (p90-p10) and 0.02 std sit comfortably above that noise floor
# and comfortably below what a real partial-coverage gradient shows —
# verified end to end (the acceptance piece, `wall_W_0_10_0034`) with
# `tools/repatch_gac_x_soot.py`'s extended ring-piece pass, not asserted.
SOOT_PALE_SPREAD_MIN = 0.05     # covered-texel luminance p90 - p10
SOOT_PALE_STD_MIN = 0.02        # covered-texel luminance std


def _mesh_arrays(prim):
    """points / counts / indices / uv (+ interpolation, indices) of one
    `UsdGeom.Mesh` as numpy, or None when it has no texture coordinates."""
    import numpy as np
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    cnt = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        return None
    pv = None
    for q in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if (q.GetTypeName().role == "TextureCoordinate"
                or q.GetBaseName() in ("st", "uv", "UVMap", "st0")):
            pv = q
            break
    if pv is None:
        return None
    vals = pv.Get()
    if not vals:
        return None
    ind = pv.GetIndices() if pv.IsIndexed() else None

    def arr(v, dt):
        try:
            return np.asarray(v, dtype=dt)
        except Exception:
            return np.array([tuple(x) if hasattr(x, "__len__") else x
                             for x in v], dtype=dt)

    return {"points": arr(pts, np.float32), "counts": arr(cnt, np.int64),
            "indices": arr(idx, np.int64), "uv": arr(vals, np.float32),
            "interp": str(pv.GetInterpolation()),
            "uv_indices": (arr(ind, np.int64) if ind is not None and len(ind)
                           else None)}


def _flat_diffuse(mat_prim):
    """The constant diffuse colour (r, g, b) of a material with no base map,
    or None. Looks at every shader's `diffuseColor` / `diffuse_color_constant`
    / `base_color` / `baseColor` input and takes the first colour value."""
    from pxr import Gf, Usd, UsdShade

    if mat_prim is None or not mat_prim.IsValid():
        return None
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        for name in ("diffuseColor", "diffuse_color_constant", "base_color",
                     "baseColor", "diffuse_tint"):
            inp = sh.GetInput(name)
            if not inp:
                continue
            try:
                v = inp.Get()
            except Exception:
                continue
            if isinstance(v, (Gf.Vec3f, Gf.Vec3d)) or (
                    hasattr(v, "__len__") and len(v) == 3):
                return (float(v[0]), float(v[1]), float(v[2]))
    return (0.6, 0.6, 0.6)


_RING_ORDER = ("S", "E", "N", "W")


def _face_side_ix(m, pts):
    """Index into `_RING_ORDER` of the elevation each point is NEAREST, in the
    mass's own yaw-honoured frame.

    The IDENTICAL rule `_skin_sample` (just below) uses to decide which
    elevation's skin a texel of a merged region-cut block is sampled from —
    shared deliberately, so that on those pieces face SELECTION and skin
    SAMPLING agree face by face instead of one being radial and the other
    per-elevation.
    """
    import numpy as np

    ang = math.radians(-float(m["yaw"]))
    ca, sa = math.cos(ang), math.sin(ang)
    dx = np.asarray(pts)[:, 0] - float(m["cx"])
    dy = np.asarray(pts)[:, 1] - float(m["cy"])
    lx = dx * ca - dy * sa
    ly = dx * sa + dy * ca
    W, D = float(m["W"]), float(m["D"])
    return np.argmin(np.stack([np.abs(ly + D / 2.0), np.abs(lx - W / 2.0),
                               np.abs(ly - D / 2.0), np.abs(lx + W / 2.0)],
                              axis=1), axis=1)


def _skin_sample(sk, side, m, world):
    """`soot_bake.sample_skin` for a piece that faces EVERY elevation (side
    "x", a merged region-cut block): each point is sampled on the elevation
    it is nearest, in the mass's own frame (yaw honoured)."""
    import numpy as np
    from . import soot_bake as sb

    wp = np.asarray(world, dtype=np.float64).reshape(-1, 3)
    ang = math.radians(-float(m["yaw"]))
    ca, sa = math.cos(ang), math.sin(ang)
    dx, dy = wp[:, 0] - float(m["cx"]), wp[:, 1] - float(m["cy"])
    lx = dx * ca - dy * sa
    ly = dx * sa + dy * ca
    W, D = float(m["W"]), float(m["D"])
    dist = np.stack([np.abs(ly + D / 2.0), np.abs(lx - W / 2.0),
                     np.abs(ly - D / 2.0), np.abs(lx + W / 2.0)], axis=1)
    ix = np.argmin(dist, axis=1)
    out = np.zeros((len(wp), 4), dtype=np.float32)
    for k, s in enumerate(("S", "E", "N", "W")):
        sel = ix == k
        if sel.any():
            out[sel] = sb.sample_skin(sk, s, m, wp[sel])
    return out


def _bind_soot(ctx, e, sk):
    """Bake the soot skin into the base-colour map of EVERY material this
    module's meshes and GeomSubsets bind, THROUGH THE MODULE'S OWN UVs, and
    rebind each with a copy of its own material carrying the baked map.

    WHY THROUGH THE UVs AND NOT A CROP. The first sim render stretched a
    crop of the skin over each module's whole base map. A bare-USD probe of
    the kit (`tools/soot_uv_probe.py`, 2026-08-30) showed why that came
    back as "not translating to the actual materials": the base maps are UV
    ATLASES — `SM_MBuilding01_Facade_A`'s wall face uses v 0.03..0.73 of its
    map with the module's bottom vertices at v~0.44 and its top at v~0.55;
    `SM_MBuilding04_Facade_B`'s outer face is the 0.56 x 0.42 corner of its
    map — so a corner-to-corner crop lands soot on arbitrary faces. The
    only mapping that is right for an arbitrary atlas is the mesh's own:
    `soot_bake.uv_position_map` rasterises the subset's triangles into
    texel space and recovers each texel's local position; that map depends
    only on the KIT PIECE (cached per piece name / mesh / subset across
    every placement and every building in `ctx["cache"]`), and
    `soot_bake.bake_module` then transforms those positions to world,
    samples the skin there and composites over the map.

    PER SUBSET, NOT PER MODULE: a module's wall, reveal and frame subsets
    bind different maps. A subset with no base map at all — glass, painted
    trim — binds the flat soot tone graded by coverage instead (the "no
    silent skip on a missing texture" fix, F5 `dw_terrace` 2026-08-29). A
    subset whose covered texels the skin never reaches is left completely
    alone, materials and all. Returns the number of bindings made.
    """
    import hashlib as _hl

    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade
    from . import quake_flow as qf, soot_bake as sb, soot_plume as spl
    from . import tex_compress as tc

    stage = ctx["stage"]
    path = e["p"].get("prim_path")
    root = stage.GetPrimAtPath(path) if path else None
    if not root or not root.IsValid() or not root.IsActive():
        return 0
    mats = ctx.setdefault("soot_mats", {})
    stats = ctx.setdefault("soot_stats", {"unreadable": 0, "flat_material": 0,
                                          "skipped_notex": 0, "flat_tone": 0,
                                          "no_uv": 0, "clean": 0,
                                          "pale_but_patterned": 0})
    cache = ctx.setdefault("cache", {})
    posmaps = cache.setdefault("soot_posmap", {})
    bases = cache.setdefault("soot_base", {})
    m = ctx["info"]["masses"][ctx["fire"]["mass"]]
    side = e["side"]
    xfc = UsdGeom.XformCache()
    # A SLICED PIECE IS ONE OF A KIND. The position-map cache below pays for
    # itself on a kit (a dozen module types, hundreds of placements) and is
    # pathological on a sliced whole-asset building, where every piece name
    # is unique: 3,158 pieces x 7 MB of cached map = most of a 25 GB
    # process on the first GAC bench (2026-08-30). One-off pieces get no
    # cache entry and a smaller bake — a 4-9 m piece does not need 768 px.
    one_off = str(e["p"].get("usd", "")).startswith("slice://")
    if one_off:
        # Sized off the PIECE'S OWN world bbox rather than any field on
        # `e["p"]` (varies by slicer) so this works uniformly for every
        # `slice://` piece; a piece this cannot measure at all (an
        # unexpectedly empty bound) is left at the full canvas rather than
        # risk under-resolving something large. See `SOOT_BAKE_PX_SLICE_SMALL`
        # above for the measured saving this buys.
        try:
            _bbc = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                useExtentsHint=True)
            _rng = _bbc.ComputeWorldBound(root).ComputeAlignedRange()
            if _rng.IsEmpty():
                _diag_m = float("inf")
            else:
                _sz = _rng.GetSize()
                _diag_m = math.hypot(float(_sz[0]), float(_sz[1]))
        except Exception:
            _diag_m = float("inf")
        _cov0 = float(e.get("_soot_cover", 0.0))
        px = (SOOT_BAKE_PX_SLICE_SMALL
              if _diag_m < SOOT_PIECE_SMALL_M
              or 0.0 < _cov0 < SOOT_PIECE_LOW_COV
              else SOOT_BAKE_PX_SLICE)
    else:
        px = SOOT_BAKE_PX
    # side "x" (a merged region-cut piece) faces every elevation: no single
    # outward direction for the facing test, and the skin is sampled by the
    # nearest elevation per texel (`_skin_sample`)
    # (`describe` gives every element a ring side from its centroid, so the
    # merged piece's own label lives on its PLACEMENT, `_side == "x"`)
    any_side = (side not in _SIDE_RING
                or str((e.get("p") or {}).get("_side", "")) == "x")
    # WHERE THE TONE FLOOR IS ALLOWED TO FIRE. Either the plume has soaked
    # this module (`_soot_cover`, the crop integrated over the module's whole
    # z span — so its neighbours are black and a pale band beside them is the
    # artefact), or the module is IN the fire: on a venting elevation and on
    # a storey the fire's own falloff calls touched, which is the same
    # `_severity >= 0.25` test `r_gut_interior` uses for "the fire reached
    # this floor". Outside both, nothing is toned — a fire has to keep
    # reading directional, and a module the plume never came near stays
    # exactly as pale as its asset shipped it.
    cov = float(e.get("_soot_cover", 0.0))
    try:
        in_fire = (side in (ctx["fire"].get("sides") or ())
                   and _severity(ctx, e.get("storey"), e.get("mass")) >= 0.25)
    except Exception:
        in_fire = False
    tone_ok = one_off and (cov >= SOOT_TONE_MIN or in_fire)
    ox_, oy_ = (None, None) if any_side else qf._outward(m, side)
    sampler = _skin_sample if any_side else None
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        arrays = None
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        targets = ([(sub.GetPrim(), sub) for sub in subsets]
                   or [(prim, None)])
        rel = str(prim.GetPath()).replace(str(root.GetPath()), "", 1)
        M = None
        face_true = None       # the strict outward mask, sliced ring only
        for t, sub in targets:
            bound = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            bprim = bound.GetPrim() if bound else None
            pre = ctx.get("soot_prebaked")
            if isinstance(pre, (set, frozenset)) and bprim is not None \
                    and str(bprim.GetPath()) in pre:
                continue          # already carries the skin (pre-slice bake)
            sh_path, inp, tex = spl.find_basecolor(bprim)
            flat_rgb = None
            if not tex and one_off:
                # A SLICED PIECE'S UNTEXTURED SUBSET IS BAKED LIKE ANY OTHER,
                # over its material's constant colour, through its UVs. The
                # flat-tone bind below (kept for the kit path, whose look is
                # frozen) painted the whole subset one tone or left it clean
                # — a uniform rectangle either way, next to textured
                # neighbours that carried the plume (user review, 2026-08-30).
                flat_rgb = _flat_diffuse(bprim)
            if not tex and flat_rgb is None:
                # a plain-coloured subset (a module's own window glass, painted
                # trim): the crop already said soot reaches this module; bind
                # the flat tone only when the module is WELL covered — a
                # lightly-fringed module's panes going flat dark read as pane-
                # shaped rectangles, not as a film
                a_mean = float(e.get("_soot_cover", 0.0))
                if a_mean < 0.35:
                    stats["skipped_notex"] += 1
                    continue
                mat = ctx["mats"]["soot" if a_mean > 0.70 else
                                  "soot_mid" if a_mean > 0.35 else "soot_light"]
                UsdShade.MaterialBindingAPI(t).Bind(mat)
                stats["flat_tone"] += 1
                n += 1
                continue
            if arrays is None:
                arrays = _mesh_arrays(prim)
                if arrays is None:
                    stats["no_uv"] += 1
                    break
                Mg = xfc.GetLocalToWorldTransform(prim)
                M = np.array([[float(Mg[r][c]) for c in range(4)]
                              for r in range(4)], dtype=np.float64)
                # per-face outward-facing test, once per mesh
                pts_w = arrays["points"].astype(np.float64) @ M[:3, :3]
                cnt = arrays["counts"]
                idx = arrays["indices"]
                starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
                i0 = idx[starts]
                i1 = idx[np.minimum(starts + 1, len(idx) - 1)]
                i2 = idx[np.minimum(starts + 2, len(idx) - 1)]
                nrm = np.cross(pts_w[i1] - pts_w[i0], pts_w[i2] - pts_w[i0])
                if any_side:
                    # outward = a mostly-vertical face whose normal points
                    # away from the mass centre (floors and ceilings are not
                    # façade, and the merged piece carries them too)
                    cen = (pts_w[i0] + pts_w[i1] + pts_w[i2]) / 3.0 + M[3, :3]
                    rx = cen[:, 0] - float(m["cx"])
                    ry = cen[:, 1] - float(m["cy"])
                    n_xy = np.hypot(nrm[:, 0], nrm[:, 1])
                    n_all = np.linalg.norm(nrm, axis=1) + 1e-12
                    face_out = ((nrm[:, 0] * rx + nrm[:, 1] * ry) > 1e-9) & (
                        n_xy > 0.5 * n_all)
                    if one_off:
                        # THE MERGED BLOCK GETS THE SAME PER-FACE TREATMENT
                        # THE RING PIECES DO — on its VENTING elevations only.
                        # The radial test above is the whole reason the block
                        # above the band kept a checkerboard of white caps
                        # while the ring below it went black: measured on the
                        # shipped `gac_SM_Building_19_F3_s100` bake
                        # (head-on facade captures + `tools/pale_census.py`,
                        # user gating on this building, 2026-08-31), it
                        # admits only 45.0% of `wall_x_0_08_0069/mat_7`'s
                        # 8,048 m2 — and 0.0% of `mat_5` and `mat_8` — so the
                        # rest was never composited and kept its base map.
                        # Classified per face by its own nearest elevation
                        # (`_face_side_ix`), the strict outward test and the
                        # `SOOT_SHELL_D` shell rule reach 100% of that
                        # subset, and 59.9% once restricted to E/W.
                        #
                        # RESTRICTED TO THE VENTING ELEVATIONS, AND ONLY EVER
                        # ADDITIVE. "Burnt on the other side of the building
                        # that isn't even on fire" is wrong (user, 2026-08-29)
                        # and the fire has to keep reading directional, so the
                        # new faces are `hot_f`-gated; the radial test's own
                        # selection is kept underneath by the union, so no
                        # cold-side face that is sooted today loses its soot.
                        # How high the stain climbs above the band is still
                        # the plume's answer, not this rule's — these faces
                        # are composited through `_skin_sample` exactly as
                        # before and its above-band decay does the rest.
                        # Interior geometry stays protected by the same
                        # `SOOT_SHELL_D` depth cut the ring pieces use, now
                        # measured per elevation: on the merged block it
                        # keeps all of a thin facade subset and only 36.2%
                        # of the deep `mat_2`, and it is what leaves SM_26's
                        # 245 m2 core ceiling alone.
                        six = _face_side_ix(m, cen)
                        hot_s = set(ctx["fire"].get("sides") or ())
                        strict = np.zeros(len(nrm), dtype=bool)
                        shell = np.zeros(len(nrm), dtype=bool)
                        hot_f = np.zeros(len(nrm), dtype=bool)
                        for _k, _s in enumerate(_RING_ORDER):
                            sel = six == _k
                            if not sel.any():
                                continue
                            ox2, oy2 = qf._outward(m, _s)
                            d2 = cen[:, 0] * ox2 + cen[:, 1] * oy2
                            strict[sel] = (nrm[sel, 0] * ox2
                                           + nrm[sel, 1] * oy2) > 1e-9
                            shell[sel] = d2[sel] >= d2[sel].max() - SOOT_SHELL_D
                            hot_f[sel] = _s in hot_s
                        # FAKE INTERIOR, WITH THE COLD SIDES AS A GUARD.
                        # On a ring piece "no strict outward face" means the
                        # subset is the room behind the opening, and the tone
                        # is bound to the WHOLE subset. A merged block's
                        # subset can wrap all four elevations at once
                        # (`mat_5` here is 33% E / 33% W / 17% N / 17% S), and
                        # a per-subset tone cannot be side-limited — it would
                        # flatten the fake interiors of the COLD elevations
                        # too, behind glass that never broke. So a cold face
                        # counts as "outward" for this test only: a subset
                        # that reaches a cold side is never toned wholesale,
                        # it just takes the positional bake with its hot
                        # shell faces now included.
                        face_true = strict | ~hot_f
                        face_out = face_out | (hot_f & (strict | shell))
                else:
                    face_out = (nrm[:, 0] * ox_ + nrm[:, 1] * oy_) > 1e-9
                    if one_off:
                        # ...PLUS THE PIECE'S OUTER SHELL (`SOOT_SHELL_D`):
                        # the fake interior behind the openings faces every
                        # way but out, and the camera reads all of it
                        # (fire_dtc3 review, 2026-08-30). Depths are face
                        # centroids projected on the elevation's outward
                        # direction, relative to this mesh's own outward
                        # extreme; `M[3, :3]` is left off both sides because
                        # a constant translation cancels in the difference.
                        # PER MESH IS PER PIECE HERE — every sliced piece is
                        # exactly one Mesh (156/156, 70/70 and 113/113 of the
                        # SM_26 / SM_19 / SM_02 bakes), and a piece that ever
                        # had two would simply be judged shell-by-shell,
                        # still bounded by `SOOT_SHELL_D`.
                        cen = (pts_w[i0] + pts_w[i1] + pts_w[i2]) / 3.0
                        d_o = cen[:, 0] * ox_ + cen[:, 1] * oy_
                        # the STRICT mask is KEPT: a subset with no face in
                        # it is fake interior and takes the tone rather than
                        # a bake (`SOOT_TONE_MIN`)
                        face_true = face_out
                        face_out = face_out | (
                            d_o >= float(d_o.max()) - SOOT_SHELL_D)
            sub_name = str(sub.GetPrim().GetName()) if sub is not None else ""
            face_ids = None
            if sub is not None:
                face_ids = sub.GetIndicesAttr().Get()
                face_ids = [int(k) for k in face_ids] if face_ids else []
            fsel = face_out[face_ids] if face_ids is not None else face_out
            if one_off:
                # A SLICED PIECE CARRIES THE BUILDING'S INTERIOR — floors,
                # ceilings and inner walls share a subset with the façade
                # faces of the same material, so the whole-subset facing
                # test below threw 173 of SM_Building_04's subsets away
                # ("inward") and their façade faces stayed clean: the
                # rectangles (user review, 2026-08-30). Bake the OUTWARD
                # faces only; the rest keep the base map.
                if not fsel.size or not bool(fsel.any()):
                    stats["inward"] = stats.get("inward", 0) + 1
                    continue
                # A FAKE INTERIOR TAKES THE TONE, NOT A BAKE — see
                # `SOOT_TONE_MIN`. No strict outward face in the subset means
                # the shell rule is the only reason it is here at all, i.e.
                # it is the room behind the opening; its horizontal planes
                # sample a single skin row and the copy comes back clean.
                # Counted as a flat-tone bind, which is exactly what it is,
                # so the note's own fields (and with them the kit path's
                # frozen wording) do not move.
                tsel = ((face_true if face_ids is None
                         else face_true[face_ids])
                        if face_true is not None else None)
                if tsel is not None and not bool(tsel.any()) \
                        and cov >= SOOT_TONE_MIN:
                    UsdShade.MaterialBindingAPI(t).Bind(
                        ctx["mats"]["soot" if cov > 0.70 else
                                    "soot_mid" if cov > SOOT_TONE_MIN else
                                    "soot_light"])
                    stats["flat_tone"] += 1
                    n += 1
                    continue
                all_ids = (face_ids if face_ids is not None
                           else list(range(len(face_out))))
                face_ids = [fid for fid, ok in zip(all_ids, fsel) if ok]
            elif fsel.size and float(fsel.mean()) < SOOT_FACING_MIN:
                stats["inward"] = stats.get("inward", 0) + 1
                continue
            key = (e["name"], rel, sub_name, px)
            pm = posmaps.get(key) if not one_off else None
            if pm is None:
                pm = sb.uv_position_map(
                    arrays["points"], arrays["counts"], arrays["indices"],
                    arrays["uv"], arrays["interp"], arrays["uv_indices"],
                    face_ids=face_ids, px=px)
                if not one_off:
                    posmaps[key] = pm
            pos, mask = pm
            if not bool(mask.any()):
                continue
            # does the skin actually reach this subset's face? sample first
            world = pos[mask] @ M[:3, :3] + M[3, :3]
            a = (sampler or sb.sample_skin)(sk, side, m, world)[..., 3]
            a_max = float(a.max())
            # THE EARLY EXIT STAYS, EXCEPT WHERE THE FLOOR MIGHT OWE THIS
            # SUBSET A TONE. "The skin never reached it" is the right answer
            # for a module out in the cold, and the wrong one for a pale cap
            # beside a flaming window — that one has to be composited first
            # so the result can be judged. The kit path has `one_off` False,
            # so `tone_ok` is False and this is the identical early exit it
            # has always taken.
            if a_max < 0.05 and not tone_ok:
                stats["clean"] += 1
                continue
            if flat_rgb is not None:
                base = np.full((8, 8, 3), flat_rgb, dtype=np.float32)
            else:
                base = bases.get(tex)
                if base is None:
                    b8 = spl._read_rgb(tex, max_px=1024)
                    # held as BYTES, not float32: fifty 2048 px atlases at
                    # 50 MB each were another 2.5 GB of the same run
                    bases[tex] = ((np.clip(b8, 0, 1) * 255.0 + 0.5).astype(np.uint8)
                                  if b8 is not None else False)
                    base = bases[tex]
                if base is not None and base is not False:
                    base = base.astype(np.float32) / 255.0
            if base is None or base is False:
                stats["unreadable"] += 1
                a_mean = float(a.mean())
                if a_mean < 0.12:
                    continue
                mat = ctx["mats"]["soot" if a_mean > 0.70 else
                                  "soot_mid" if a_mean > 0.35 else "soot_light"]
                UsdShade.MaterialBindingAPI(t).Bind(mat)
                stats["flat_tone"] += 1
                n += 1
                continue
            out = sb.bake_module(sk, side, m, M, pos, mask, base, px=px,
                                 sampler=sampler)
            # THE TONE FLOOR — see `SOOT_PALE_MAX`. Judged on what the subset
            # will actually sample (the COVERED texels, never the whole
            # atlas: a correctly sooted subset can occupy 7% of a pale tile
            # and the atlas mean says nothing about what renders). A healthy
            # positional bake is far under the threshold and is kept — this
            # is a floor, not a replacement for the wall's own bake.
            #
            # ONLY OVER A GENUINELY FLAT RESULT. `tone_ok` (this whole branch
            # is `one_off` already — see its own definition) is a real
            # coverage/severity read, so a subset that lands here IS inside
            # the fire's reach; the question is whether its OWN composite is
            # still worth keeping. Mean alone cannot answer that — a subset
            # the plume only partly reaches can average pale while carrying a
            # real gradient (`wall_W_0_10_0034`, `SM_Building_23` F5, "haven't
            # gotten the scorch pattern well", user review 2026-08-31: the
            # floor discarded a genuinely baked composite for a flat,
            # unnumbered tone on exactly this piece). SPREAD does: the
            # checkerboard population `SOOT_PALE_MAX` exists for is a narrow
            # band sampling a handful of skin rows that never cross a tongue,
            # so it comes back near-uniform AND pale — nothing but resample
            # noise varies across it. A subset with real luminance spread
            # over its own covered texels is not that population, whatever
            # its mean, and the bake below is kept.
            if tone_ok and float(out[mask].mean()) > SOOT_PALE_MAX:
                cov_lum = out[mask].mean(axis=-1) if out[mask].ndim > 1 \
                    else out[mask]
                spread = (float(np.percentile(cov_lum, 90)
                                - np.percentile(cov_lum, 10))
                          if cov_lum.size >= 8 else 0.0)
                lum_std = float(cov_lum.std()) if cov_lum.size >= 2 else 0.0
                if spread < SOOT_PALE_SPREAD_MIN and lum_std < SOOT_PALE_STD_MIN:
                    UsdShade.MaterialBindingAPI(t).Bind(
                        ctx["mats"]["soot" if cov > 0.70 else
                                    "soot_mid" if cov > SOOT_TONE_MIN else
                                    "soot_light"])
                    stats["flat_tone"] += 1
                    n += 1
                    continue
                stats["pale_but_patterned"] += 1
                # falls through: the bake below is bound as-is, pattern kept
            if a_max < 0.05:
                # let past the early exit only for the floor, which did not
                # fire: the composite is already dark, so leave it alone
                # exactly as before rather than shipping a pointless copy
                stats["clean"] += 1
                continue
            digest = _hl.md5(np.round(out * 255.0).astype(np.uint8).tobytes()
                             ).hexdigest()[:16]
            os.makedirs(spl.OUT_DIR, exist_ok=True)
            # `png` keeps its name for every downstream use below (dedup
            # key, `piece_material_like`/`piece_material`'s `tex_path`) —
            # it is a `.dds` by default now (`SOOT_TEX_COMPRESS`, see
            # `tex_compress.py`), `.png` only with the gate off.
            png = tc.save_soot_texture(
                out, os.path.join(spl.OUT_DIR, "sootbake_{0}".format(digest)))
            mkey = (str(bprim.GetPath()) if bprim is not None else "", png)
            mat = mats.get(mkey)
            if mat is None:
                mp = "{0}/FireLooks/soot_{1}".format(ctx["parent"], len(mats))
                mat = None
                if flat_rgb is None:
                    mat = spl.piece_material_like(stage, mp, bprim, sh_path,
                                                  inp, png)
                if mat is None:
                    # a constant-colour material has no map input to swap:
                    # a fresh surface carrying the baked map
                    mat = spl.piece_material(stage, mp, png)
                    stats["flat_material"] += 1
                mats[mkey] = mat
            UsdShade.MaterialBindingAPI(t).Bind(mat)
            n += 1
    return n


def _r_soot_overlay(ctx, heavy, role):
    """Every module of `role` on the burning mass takes its own crop of the
    soot skin and has it merged into its own textures. Returns the number of
    SIDES on which at least one module was sooted.

    A module's crop is addressed by its measured span along its side
    (`soot_plume.piece_span`, both ends projected so the piece's own local
    +X direction does not matter) and its z range; a module whose crop has
    no soot in it is left completely alone — its own materials, its own
    normal/roughness maps, untouched — which is also what keeps a curtain
    wall's un-sooted panes looking like the kit shipped them.
    """
    from . import quake_flow as qf, soot_plume as spl

    if ctx.get("soot_prebaked") is True:
        return 0          # the asset's atlases already carry the skin
    sk = _soot_skin(ctx, heavy)
    if sk is None:
        return 0
    mtag = ctx["fire"]["mass"]
    m = ctx["info"]["masses"][mtag]
    n_side = 0
    for side in _SIDE_RING:
        n_piece = 0
        for e in qf._els(ctx, mass=mtag, role=role, side=side):
            if str((e.get("p") or {}).get("_side", "")) == "x":
                # A MERGED REGION-CUT PIECE (below the fire origin, or the
                # block above the band) faces every elevation at once;
                # `describe` filed it under one ring side from its centroid.
                # No single-side crop can judge it — `_bind_soot` samples
                # the skin per texel by the nearest elevation and its own
                # per-subset test decides. The block above the band is
                # exactly where the plume climbs to.
                if not e["p"].get("prim_path"):
                    continue
                # NOT 0.5. `_soot_cover` is not just this loop's own
                # "did the crop reach it" flag — `_bind_soot` reads the SAME
                # field as `cov` to gate `tone_ok`, the floor that force-
                # binds a flat dark tone over a bake that came back too pale
                # (`SOOT_PALE_MAX`). A fixed 0.5 is >= `SOOT_TONE_MIN` (0.35)
                # on every merged piece regardless of storey, so the floor
                # fired unconditionally on the below-origin block and the
                # block above the band — exactly the storeys `_severity`'s
                # own docstring says must stay a hard, clean cut ("the clean
                # band stops reading"). Measured on `SM_Building_23` F5:
                # `wall_x_0_00_0000` (storeys 0-6, origin=7) came back 82% of
                # its area sooted, most of it at near-zero texture variance
                # — a flat dark wash on a floor the fire never reached (user
                # review, 2026-08-31: "haven't gotten the scorch pattern
                # well", the SW_s1254 bake). Leaving this at 0.0 means
                # `tone_ok` falls back to `in_fire` (a real per-storey
                # `_severity` read, still granted its one storey of spill
                # downward) and the per-texel `a_max` this same call samples
                # a few lines into `_bind_soot` decides everything else —
                # the compositor's own real measurement, not a placeholder.
                if _bind_soot(ctx, e, sk):
                    n_piece += 1
                continue
            fe = qf._piece_frame(e)
            if fe is None or not e["p"].get("prim_path"):
                continue
            u0, u1 = spl.piece_span(e, fe, m, side)
            za = float(e.get("z", m["z0"]))
            zb = za + max(0.3, float(fe[4]))
            # the crop is a PREFILTER only (does any soot reach this
            # module's rectangle?) — the bake itself goes through the UVs
            crop = spl.piece_crop(sk, side, u0, u1, za, zb)
            a = crop[..., 3]
            if float(a.mean()) < 0.01 and float(a.max()) < 0.15:
                continue
            e["_soot_cover"] = float(a.mean())
            if _bind_soot(ctx, e, sk):
                n_piece += 1
        if n_piece:
            n_side += 1
    return n_side


def r_smoke_stain(ctx, heavy=1.0, above=2):
    """The wall wash: the building's fire events, rasterised as one soot skin
    and merged into every module's own texture — walls, corners, parapets
    and balconies alike, curtain walls included (their skin is hardened
    into a film, `soot_plume.skin(glass=True)`, instead of the flat tone
    that used to be bound to a share of their panes).

    `heavy` multiplies the events' venting durations. `above` is kept for
    the ladder's signature and is not needed any more: how far the stain
    climbs above the band is the plume's own answer, not a storey count.
    """
    from . import quake_flow as qf, soot_plume as spl

    f = ctx["fire"]
    events = f.get("events") or []
    counts = {}
    # EVERY ROLE THE KIT HAS, not a fixed list. A sliced GreatAmericanCity
    # building is mostly `pier` pieces (82 of SM_Building_02's 173) plus
    # `core`; the fixed list below never visited them, so every pier subset
    # on a per-piece (tiled) atlas kept its clean map between sooted
    # neighbours — "perfect rectangular parts of the building that randomly
    # didn't get scorched" (user review, 2026-08-30). Roofs are
    # `r_roof_scorch`'s.
    roles = ["wall", "corner", "parapet", "parapet_corner", "balcony"]
    present = []
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # the sliced (GAC) path only — the kit path's look is FROZEN
        present = sorted(set(str(e.get("role"))
                             for e in qf._els(ctx, mass=f["mass"]))
                         - set(roles) - {"roof", "None", ""})
    for role in roles + present:
        k = _r_soot_overlay(ctx, heavy, role)
        if k:
            counts[role] = k
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # SLICED PATH ONLY — the kit look is frozen. A sooted copy that
        # keeps its source's Metalness/Roughness MAPS mirrors the sky
        # straight through the soot (dtc Amar_Tower F5c, "still very
        # reflective", fire_dtc3 review 2026-08-31). ONE SWEEP, HERE,
        # AFTER every binding loop has finished — hardening inside
        # `_bind_soot`'s own loop recomposes the reference-built copies
        # and expires the loop's held subset/material handles
        # (UsdExpiredPrimAccessError swallowed mid-role: a building that
        # "bakes fine" with half its skin missing). The sweep finds the
        # copies by their own sootbake_<digest>.png names and mattes the
        # ones `soot_bake.bake_module` logged as significantly sooted.
        from . import soot_bake as _sbh
        n_hard = _sbh.harden_baked_materials(
            ctx["stage"], root=ctx["stage"].GetPrimAtPath(ctx["parent"]))
        if n_hard:
            ctx["notes"].append(
                "soot gloss: {0} shader input(s) matted on the sooted "
                "copies".format(n_hard))
    st = ctx.get("soot_stats") or {}
    ctx["notes"].append(
        "smoke: soot skin from {0}; sooted sides per role: {1}; storeys "
        "{2}-{3} on {4}; {5} merged material(s), {6} unreadable base map(s), "
        "{7} flat-material fallback(s), {8} flat-tone bind(s), {9} untextured "
        "subset(s) too light to bind, {10} without UVs, {11} subset(s) the "
        "skin never reached, {12} inward-facing subset(s) skipped".format(
            spl.summarise(events),
            ", ".join("{0} {1}".format(r, k) for r, k in counts.items())
            or "none", f["origin"], f["top"], "/".join(f["sides"]),
            len(ctx.get("soot_mats") or {}), st.get("unreadable", 0),
            st.get("flat_material", 0), st.get("flat_tone", 0),
            st.get("skipped_notex", 0), st.get("no_uv", 0),
            st.get("clean", 0), st.get("inward", 0)))


# ---------------------------------------------------------------------------
# 2. The openings: glass out, black inside
# ---------------------------------------------------------------------------
def r_window_burnout(ctx, frac=1.0, empty=True):
    """Glass out of the openings in the band; the room behind them black.

    GLASS IS THE FIRST THING TO GO AND IT GOES EARLY. Ordinary annealed float
    glass cracks at a ~60 K differential and falls out well before flashover,
    which is why a building with fire on one floor already has EMPTY holes
    there while the floor below is still glazed. So this runs at every level
    from F2 up at essentially frac 1.0 in the band, and the interesting knob
    is not how many break but how many are then EMPTY (`empty`) versus merely
    cracked — F1 is the smoke-damaged case where the glass is crazed and
    sooted but still in the frame.

    The void quad is the piece that does the work. An opening with the kit's
    painted-on glass still showing behind a black plume reads as a decal on an
    intact building; a genuinely dark reveal reads as a room that has burned.
    """
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # GAC PATH: THE SLICER CANNOT ADDRESS A WINDOW. A GAC piece carries
        # no measured `_G2_WIN_FACES` entry (that table is populated by the
        # KIT's own module registration, `urban_building.PIECES`), so
        # `qf._g2_openings`/`_g_shop_openings` below find nothing on it —
        # this recipe was always a silent no-op here, but it still appended
        # its own "0 burnt out, 0 crazed" note, which read as though nothing
        # had been treated at all (user review, 2026-08-30: "windows: 0
        # burnt out, 0 crazed"). It HAD been — by `gac_fire.damage_windows`,
        # on the merged asset's own measured window ISLANDS, called
        # separately from `gac_fire.burn_gac` right after this ladder
        # returns; see that function's own note instead of this one.
        return
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    n_void = n_lit = 0
    ops = list(qf._g2_openings(ctx, sides=f["sides"])) + \
        list(qf._g_shop_openings(ctx, sides=f["sides"]))
    for op in ops:
        sev = _severity(ctx, op["storey"], op["e"]["mass"], op["e"])
        if sev < 0.15 or rng.random() > frac:
            continue
        hu0, hu1 = op.get("hua", op["ua"]), op.get("hub", op["ub"])
        hv0, hv1 = op.get("hva", op["va"]), op.get("hvb", op["vb"])
        w, h = hu1 - hu0, hv1 - hv0
        if w <= 0.15 or h <= 0.15:
            continue
        # NOT EVERY PANE IN THE BAND IS OUT, and the mix is what makes the
        # band read. `empty` was a per-recipe boolean, so an F2 building had
        # every opening in the fire floor as an identical black hole and NO
        # cracked glass anywhere — which is the state the user asked to see
        # (2026-08-28). Thermal fracture runs ahead of fallout, so at the hot
        # centre of the band the glass is gone and at its edges it is cracked
        # and tarred but still in the frame.
        out_p = 0.0 if not empty else max(0.0, min(1.0, (sev - 0.28) / 0.45))
        if rng.random() < out_p:
            # BEHIND THE WALL PLANE, ALWAYS. `op["out"]` is the measured
            # reveal depth and it is negative on the kits that HAVE a reveal
            # — but the Downtown_West storefronts report ~0, so
            # `op["out"] - 0.06` left the quad essentially in the wall plane
            # and the whole terrace came out plastered with black rectangles
            # standing on its face. Clamp it: a void is never proud.
            # NO VOID QUAD. This used to author a near-black panel inset into
            # the opening to stand for the dark interior behind it. Three
            # rounds of review went into stopping it reading as a flat black
            # rectangle — clamping it behind the wall plane, then insetting it
            # a few per cent so a rim of reveal survived — and it still did
            # not work: "the void looks outset rather than inset rn, they
            # aren't needed imo" (user, 2026-08-29, `void_b5_1715`).
            #
            # It is not needed because the things that make an emptied
            # opening read as an opening are all still here and are all real
            # geometry rather than a painted stand-in: the charred frame
            # below, the reveal the kit already models, `fit_interior`'s
            # floor slabs and contents visible through the hole, and the
            # storey-deep darkness behind them. A painted panel in front of
            # that only ever hid it.
            n_void += 1
            # A charred frame round the opening: four thin bars. With the
            # void panel gone these carry the edge on their own, which is
            # what they were doing the work of anyway.
            _burnt_frame(ctx, op, hu0, hu1, hv0, hv1)
            # The sill litter — a handful of glass and char crumbs on the
            # cill and on the pavement under a ground-floor opening.
            _sill_litter(ctx, op, hu0, hu1, hv0)
        else:
            # still in place, with fire on the other side of it
            _glass_pane(ctx, op, hu0, hu1, hv0, hv1,
                        _severity(ctx, op["storey"], op["e"]["mass"],
                                  op["e"]),
                        out=max(0.012, float(op["out"]) + 0.012))
            n_lit += 1
    ctx["notes"].append("windows: {0} burnt out (void + frame), {1} crazed"
                        .format(n_void, n_lit))


def _glass_pane(ctx, op, u0, u1, v0, v1, sev, out=0.012):
    """A pane that is still IN, with fire on the other side of it.

    Three things, and all three are needed — drop any one and it goes back to
    reading as a grey rectangle:

    1. THE DEPOSIT IS GRADED, HEAVIEST AT THE TOP, AND IT IS A GRADIENT, NOT
       PANELS. A compartment fire fills from the ceiling down (the smoke
       layer sits above the neutral plane), so the pane is opaque at the
       head, brown through the middle and comparatively clear at the cill —
       and below where the deposit reaches at all, the pane is left showing
       its OWN kit glass rather than painted over with a "clear" quad that
       just sat on top of perfectly good glazing. `_GLASS_DEP_STEPS` thin
       bands, lerped between two endpoints (`_GLASS_DEP_LO/HI`) so the hue
       cannot drift band to band the way three independently hand-tuned
       flat colours did ("there's just 3 rectangles, green, beige and
       brown", user review, 2026-08-29, prim `pane_b2_193` — see the note
       above `fire_glass` in `_FLAT`). They are ALSO authored with real
       opacity (`materials()`'s post-loop) so this becomes true translucent
       tinted glass the day this bench's launch script passes
       `/rtx/raytracing/fractionalCutoutOpacity` — today it still renders
       opaque, same as car glass (see that same note); what is real on
       today's render is the many-step gradient and the clean glass left
       showing below `lo`.
    2. CRACKS, AND THEY ARE THE LIGHT PART. Thermal fracture starts at the
       EDGE, where the frame shades the glass and holds it cool while the
       centre heats — so the cracks root at the frame and run inward, and
       they scatter light, so they render BRIGHTER than the pane, not darker
       (the opposite of a masonry crack, which is a shadow).
    3. A MISSING CORNER once it has really gone. A thermally-cracked pane
       loses a triangle at one edge long before the whole thing falls out.
    """
    from . import quake_flow as qf

    rng = ctx["rng"]
    fr = op["fr"]
    w, h = u1 - u0, v1 - v0
    if w <= 0.15 or h <= 0.15:
        return 0
    um = 0.5 * (u0 + u1)
    made = 0
    # 1) the graded deposit — a smooth ramp from the head down to `lo`;
    # NOTHING is authored below `lo`, which is what leaves the kit's own
    # clear glass showing at the cill instead of painting over it. Heavier
    # fires push the smoke layer further down the pane, so `lo` falls with
    # `sev`: 0.72 (the top 28%) at sev=0, 0.10 (the top 90%, a sliver of
    # clear glass always left at the very cill) at sev=1.
    lo = 1.0 - min(0.90, 0.28 + 0.62 * max(0.0, min(1.0, sev)))
    if lo < 0.99:
        span = 1.0 - lo
        for i in range(_GLASS_DEP_STEPS):
            t0 = lo + span * (i / float(_GLASS_DEP_STEPS))
            t1 = lo + span * ((i + 1) / float(_GLASS_DEP_STEPS))
            _face_polygon(ctx, fr, um, v0 + t0 * h,
                          [(-w / 2, 0.0), (w / 2, 0.0),
                           (w / 2, (t1 - t0) * h), (-w / 2, (t1 - t0) * h)],
                          ctx["mats"]["glass_dep{0}".format(i)],
                          out=out + 0.0006 * i, kind="pane", owner=op["e"])
            made += 1
    # 2) the cracks — rooted at an edge, running in
    n_cr = 2 + int(round(3 * sev))
    for _ in range(n_cr):
        edge = rng.random()
        if edge < 0.4:                      # from the head
            a = (rng.uniform(u0 + 0.05 * w, u1 - 0.05 * w), v1 - 0.01 * h)
        elif edge < 0.7:                    # from a jamb
            a = (u0 + 0.01 * w if rng.random() < 0.5 else u1 - 0.01 * w,
                 rng.uniform(v0 + 0.1 * h, v1 - 0.1 * h))
        else:                               # from the cill
            a = (rng.uniform(u0 + 0.05 * w, u1 - 0.05 * w), v0 + 0.01 * h)
        b = (rng.uniform(u0 + 0.15 * w, u1 - 0.15 * w),
             rng.uniform(v0 + 0.15 * h, v1 - 0.15 * h))
        qf._b_crack(ctx, fr, a[0], a[1], b[0], b[1],
                    ctx["mats"]["glass_crack"],
                    width=rng.uniform(0.008, 0.018), proud=out + 0.006,
                    n_seg=rng.randrange(3, 6), jag=0.06)
        made += 1
    # 3) a lost corner on a badly cracked pane
    if sev > 0.55 and rng.random() < 0.45:
        cw, ch = w * rng.uniform(0.18, 0.38), h * rng.uniform(0.18, 0.38)
        cu = u0 if rng.random() < 0.5 else u1 - cw
        cv = v1 - ch if rng.random() < 0.65 else v0
        _face_polygon(ctx, fr, cu + cw / 2.0, cv,
                      [(-cw / 2, 0.0), (cw / 2, 0.0), (0.0, ch)],
                      ctx["mats"]["void"],
                      out=min(float(op["out"]), -0.02) - 0.04, kind="panegap",
                      owner=op["e"])
        made += 1
    return made


def _burnt_frame(ctx, op, u0, u1, v0, v1, t=0.055):
    """Four charred bars round an emptied opening."""
    from . import quake_flow as qf
    fr, out = op["fr"], min(float(op["out"]), -0.02) + 0.014
    mat = ctx["mats"]["burnt_metal"]
    rng = ctx["rng"]
    for (a, b, va, vb) in ((u0, u0 + t, v0, v1), (u1 - t, u1, v0, v1),
                           (u0, u1, v0, v0 + t), (u0, u1, v1 - t, v1)):
        # the head bar sags a little on a burnt-out opening; the jambs do not
        sag = rng.uniform(0.0, 0.05) if (vb - va) <= t * 1.5 and va > v0 else 0.0
        _face_polygon(ctx, fr, 0.5 * (a + b), va - sag,
                      [(-(b - a) / 2, 0.0), ((b - a) / 2, 0.0),
                       ((b - a) / 2, vb - va), (-(b - a) / 2, vb - va)],
                      mat, out=out, kind="wframe", owner=op["e"])


def _sill_litter(ctx, op, u0, u1, v0, n=None):
    """Glass and char crumbs on the cill and the ground under an opening."""
    from . import quake_flow as qf
    rng = ctx["rng"]
    fr = op["fr"]
    m = op["m"]
    n = n if n is not None else rng.randint(2, 5)
    ground = m["z0"]
    for _ in range(n):
        u = rng.uniform(u0, u1)
        # a third stay on the cill, the rest are on the pavement below
        if rng.random() < 0.34 and v0 - ground > 0.6:
            z = v0 + 0.03
            out = op["out"] + rng.uniform(0.05, 0.18)
        else:
            z = ground + 0.02
            out = op["out"] + rng.uniform(0.2, 1.9)
        x, y, _ = qf._b_face_pt(fr, u, z, out)
        s = rng.uniform(0.06, 0.22)
        path = "{0}/glit_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        qf._a_lump(ctx["stage"], path, x, y, z + s * 0.15, s, rng,
                   ctx["mats"]["fire_glass"] if rng.random() < 0.6
                   else ctx["mats"]["char_concrete"], jitter=0.5)
        ctx["authored"].append(path)


# ---------------------------------------------------------------------------
# 3. Charring the elevation
# ---------------------------------------------------------------------------
def r_char_facade(ctx, coverage=0.34):
    """Stamp the textured char maps over a share of the sooted elevation.

    `r_smoke_stain` has already composited each module's OWN cladding texture
    with soot, which keeps the brick or the stone legible under the staining.
    This puts the char/scorch/ash MAPS on top of a share of the same subsets —
    alligator checking, ash flecks, a black that varies panel to panel. Two
    passes rather than one because either alone fails a specific way: the
    composite alone is a dimmer switch (uniform, and soot never is), and the
    maps alone erase the building's identity, so a burnt brownstone and a
    burnt office tower come out as the same black box.

    `coverage` is the share AT FULL SEVERITY; it scales down with severity, so
    a storey that was only licked keeps most of its own surface. IT IS A
    THIRD, NOT MOST: at 0.85 a burnt elevation came out as one black slab
    with no courses, no trim and no building in it, and at 0.5 it was a
    patchwork of roofing felt. A burnt brick wall is dark grey-brown and you
    can still read the brickwork — the COMPOSITE (the wall's own map with
    soot composited into it) is what should carry the surface, and the char
    maps are the minority of it that burnt through the finish.

    `damage.char_placements` does the same walk but takes its fraction from
    `_CHAR`, a table keyed by the WILDFIRE's structural level names whose
    lowest non-zero entry is 0.55 — there is no way to ask it for a light
    stamp. `_bind_subsets` takes the fraction directly.

    REACHABILITY, not a raw side test — and the SEVERITY THAT SURVIVES IT is
    what grades the material. `_side_reach` replaced two independent bugs
    here (see its own docstring): the "round the corner" bleed used to fire
    for ANY off-plan side, opposite included, and the roof/parapet branch
    skipped the side test altogether. What is new on top of that fix is that
    the resulting `sev` — already lower at the edge of the reach than at the
    seat of the fire, by construction — now also picks WHICH grade of char
    material gets bound (`_burn_mat`'s `sev` argument), so the stamp itself
    gets lighter outward instead of stamping one intensity everywhere it is
    stamped at all ("scorch marks spread and get lighter", user review,
    2026-08-29).
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    finish = f["finish"]
    n = 0
    if ctx["info"]["type"] == "rc_glass":
        # NOT ON A CURTAIN WALL. The char maps are photographed burnt TIMBER
        # and calcined masonry; on a glazed bay one of them reads as a sheet
        # of plywood nailed over the window (uf5 skyscraper_a, 2026-08-28).
        # `r_smoke_stain`'s flat soot film is the whole surface treatment a
        # burnt curtain wall gets, and `r_curtain_burn` supplies the rest.
        ctx["notes"].append("char: skipped, curtain wall takes soot film only")
        return
    roles = ("wall", "corner", "parapet", "parapet_corner", "balcony", "roof")
    if ctx.get("soot_skin") is not None or ctx.get("soot_prebaked"):
        # THE SKIN OWNS THE FAÇADE. With the soot merged into every module's
        # own map, a flat char map bound over a third of the subsets on top
        # of it is a field of hard-edged black and gravel rectangles cut
        # into a continuous stain — the "hard cutoff in rectangular shapes"
        # of the first uf_soot render (2026-08-30). The skin already
        # saturates to char where the fire was; only the roof deck, which
        # the skin does not cover, still takes the maps.
        roles = ("roof",)
    for e in qf._els(ctx, role=roles):
        roof_like = e["role"] in ("roof", "parapet", "parapet_corner")
        if roof_like:
            # the roof and its parapet see the fire only if it got that high
            sev = _severity(ctx, f["n_storeys"] - 1, e["mass"])
        else:
            sev = _severity(ctx, e["storey"], e["mass"], e)
        sev *= _side_reach(ctx, e["side"], roof_like=roof_like)
        # The spill storey below the origin takes the smoke WASH (which
        # `r_smoke_stain` has already applied) and no char maps: it was never
        # alight, and stamping char on it puts the fire a floor lower than it
        # was.
        if sev < 0.20 or e["storey"] < f["origin"]:
            continue
        # ONE MATERIAL PER MODULE, not one per subset. Nine burn materials
        # (three maps x three UV variants) drawn independently per subset
        # tiled a burnt elevation into a CHECKERBOARD of six visibly
        # different tones on 4 x 3 m rectangles — it reads as a texture bug,
        # not as fire (uf_bench2 commercial, 2026-08-28). Drawing once per
        # module and applying it to a share of that module's subsets keeps
        # the mottling (the share is what mottles) and loses the patchwork.
        mm = _burn_mat(ctx, finish, sev=sev)
        n += _bind_subsets(ctx["stage"], e["p"], lambda: mm,
                           min(1.0, coverage * sev), rng)
    ctx["notes"].append("char: {0} subset(s) blackened, finish={1}".format(n, finish))


# ---------------------------------------------------------------------------
# 4. What the heat does to the material: spalling and render peel
# ---------------------------------------------------------------------------
def _scar(ctx, fr, u, v, ra, rv, face_mat, kind="spall", owner=None):
    """A PALE scar on a sooted wall: a dark halo with the exposed face on it.

    Inverted from the earthquake's `_face_patch`, and the inversion is the
    whole point. A quake spall is a dark RECESS in a clean wall, so that one
    draws a dark fill with a darker shadow ring. A fire spall is the opposite
    event: the sooted cover blew off and what is underneath is PALER than
    everything round it, with a scorched lip where the heat ran out. So the
    halo is drawn first in soot and the exposed face sits on top of it,
    smaller — one extra mesh, and without it a pale blob on a black wall has
    a hard edge and reads as a sticker.
    """
    from . import quake_flow as qf
    rng = ctx["rng"]
    halo = qf._b_blob_outline(rng, ra * 1.22, rv * 1.22)
    face = qf._b_blob_outline(rng, ra, rv)
    _face_polygon(ctx, fr, u, v, halo, ctx["mats"]["soot"],
                  out=SCAR_HALO_PROUD, kind=kind + "halo", owner=owner)
    return _face_polygon(ctx, fr, u, v, face, face_mat, out=SCAR_PROUD,
                         kind=kind, owner=owner)


def r_spall(ctx, rate=0.3, bars=True):
    """Concrete spalling on the fire storeys — PALE scars on a black wall.

    Explosive spalling is the fire signature on a concrete frame: trapped
    moisture flashes to steam and blows the cover off in dish-shaped flakes,
    exposing pale aggregate and, where the cover is gone entirely, the
    reinforcement. It is the exact inverse of the earthquake's spall, and
    that inversion is the whole reason this cannot call `r_facade_scars`
    unchanged: a quake scar is a DARK recess in a clean wall, and a fire
    spall is a LIGHT patch on a sooted one. Binding the quake's `scar`
    material here gives black-on-black and nothing is visible at all.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    n_patch = n_bar = 0
    for e in qf._els(ctx, role=("wall", "corner")):
        if e["side"] not in f["sides"]:
            continue
        sev = _severity(ctx, e["storey"], e["mass"], e)
        if sev < 0.3 or rng.random() > rate * sev:
            continue
        fr = qf._piece_frame(e)
        if fr is None:
            continue
        width = fr[3]
        # SMALL AND FEW. Explosive spalling takes the 25-40 mm cover off in
        # dishes a hand to a dinner-plate across; the first pass drew them up
        # to 1.1 m and a wall carried a dozen, which is a camouflage pattern,
        # not a burnt surface.
        for _ in range(1 if rng.random() < 0.72 else 2):
            u = rng.uniform(0.15 * width, 0.85 * width)
            v = e["z"] + rng.uniform(0.3, max(0.5, e["h"] - 0.4))
            ra = rng.uniform(0.09, 0.30) * (0.7 + sev)
            rv = ra * rng.uniform(0.6, 1.4)
            _scar(ctx, fr, u, v, ra, rv, ctx["mats"]["spall_face"], "spall",
                  owner=e)
            n_patch += 1
            if bars and rng.random() < 0.16 * sev:
                # exposed reinforcement in the deepest spalls: short bars
                # lying IN the plane of the wall, not sticking out of it
                x, y, _ = _stamp_pt(ctx, fr, u, v, 0.012)
                for _b in range(rng.randint(2, 3)):
                    dz = rng.uniform(-rv * 0.8, rv * 0.8)
                    path = "{0}/sbar_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                     qf._uid(ctx))
                    qf._cyl(ctx["stage"], path,
                            (x, y, v + dz - ra * 0.9), (x, y, v + dz + ra * 0.9),
                            0.010, ctx["mats"]["burnt_metal"])
                    ctx["authored"].append(path)
                    n_bar += 1
    ctx["notes"].append("spall: {0} patch(es), {1} exposed bar(s)"
                        .format(n_patch, n_bar))


# WHICH FAMILIES HAVE A RENDER TO LOSE. Kit family 04 is BRICK COMMERCIAL and
# 01 is dressed STONE — neither is rendered, so "the stucco came off and
# exposed the brick" is a story about a building that does not exist, and on
# the bench it read exactly like what it was: pink brick stickers stuck on a
# black wall (uf_smoke, 2026-08-28). Only the plastered stock peels; brick and
# stone calcine and spall instead, which is `r_spall`.
PEEL_FAMILIES = ("03", "dw_b", "civ", "church")

# WHERE A PEEL SITS, AND WHAT IT IS MADE OF. Both halves of one user review of
# the live 500 m fire city (2026-08-31, `kit_brownstone_row_F4_o4_EN_s438`,
# prim `bake/k14/peel_k14_166`): "this doesn't match the material of the
# building it's on. it's also not on the building, it's floating".
#
# 1. THE PLANE. `quake_flow._piece_frame` returns the module's bbox FRONT
#    along its outward axis (`ymin - 0.02`), and `_stamp_pt`'s docstring
#    states the assumption that comes with it: "a kit module's frame depth is
#    its wall face". That holds for the flat stock the SPALL families are made
#    of (measured, `urban_building.PIECES`: `SM_MBuilding04_Facade_B` ymin
#    0.00, `..01_Facade_A` -0.03, `..02_Facade_A` -0.10) and it is FALSE for
#    exactly the stock that PEELS: a brownstone's upper storey is a projecting
#    BAY (`SM_MBuilding03_Facade_B_Upper` ymin -0.38) over a stoop
#    (`..03_FirstFloor_A` -1.50), and the Downtown_West frame is measured from
#    the wrong side altogether (`-abs(xmin) - 0.02`, so
#    `..lvl2_singlewindow` — 0.46 m of wall BEHIND its line and 0.02 m in
#    front — is read as 0.48 m in front of it). On the shipped bake every one
#    of the 10 peels and their 10 halos stands 0.430-0.436 m outside its own
#    wall line (peels at y = 7.436 / x = 19.436 against lines at 7.000 and
#    19.000 — mass 38 x 14, `tools/peel_backing_probe.py`): the bay's 0.38 +
#    the frame's 0.02 pad + `SCAR_PROUD`, on a 0.2-0.5 m patch.
#
#    AND A PEEL IS NOT A CURL HANGING OFF THE WALL. `_scar` draws the
#    substrate a sheet of render has EXPOSED — the sheets themselves are the
#    `peelrow` windrow at the wall foot — so it is a flat patch OF the wall
#    and belongs on the surface, never in the air in front of it.
#
#    So the plane is MEASURED, not assumed: the same answer `gac_fire.prepare`
#    already gives the sliced path through `fire["planes"]`, which is the
#    branch of `_stamp_pt` that exists because the sliced frames were wrong in
#    the same way. `_wall_relief` reads the module's own outward-facing
#    triangles once per kit piece and `_stamp_frame` puts the stamp on the
#    frontmost of them AT THAT (u, v) — a peel drawn over the bay sits on the
#    bay, one drawn beside it sits on the wall, and neither hangs in the gap
#    between them.
#
#    SCOPED TO THE PEEL, DELIBERATELY. `r_spall` stamps through the same
#    `_scar` on families 01/02/04/05, whose wall stock is authored with the
#    cladding AT the bbox front (the measurements above), so the frame already
#    IS their wall face and that path's output is frozen
#    (`tools/kit_burn_probe.py`). The one piece in that stock with real relief
#    is `..01_FirstFloor_A`, a 0.55 m entrance canopy a spall reaches only when
#    the fire starts on the ground floor: same class, same helper, the day it
#    is reviewed.
PEEL_WALL_PAD = 0.02          # `_piece_frame`'s own pad, off the MEASURED face
PEEL_RECESS_MAX = 0.25        # never stamp further behind the line than this

# 2. WHAT THE LOST RENDER EXPOSES. The face was `mats["brick_bare"]`: ONE
#    world-triplanar megascan brick sheet (`Brick_Wall_Worn/T_sexkaitb_1K_B.
#    jpg`, 1.6 m repeats), built once per STAGE and shared by every building
#    on it. So every peel in the city exposed the same red brick at full
#    brightness whatever the building was made of and whatever colour the fire
#    had left it — on a brownstone whose modules wear their own sooted atlases
#    (46 `FireLooks/soot_*` on that bake) that is a red sticker on a black
#    wall. Measured over the whole `city_4` set before this change: 28 of 28
#    peel/halo stamps bound to the shared `/World/bake/FireLooks/brick_bare`,
#    none to anything sampled from the building it is on.
#
#    THE FIX IS THE TEAR'S OWN FALLBACK. `fire_collapse.tone_material` samples
#    a flat tone from the parent's own map over the parent's own UV box, and
#    is already the accepted answer to "a fragment that cannot carry its
#    parent's UVs must still look like the wall it came off". A peel is that
#    problem exactly: a 0.2-0.5 m blob has no UVs worth carrying, but it has
#    to read as THIS building's masonry. It samples the material the module is
#    wearing NOW — after `r_smoke_stain` that is the SOOTED copy `_bind_soot`
#    rebound, so the tone already carries the fire — and `PEEL_FACE_GAIN` then
#    opens it up to the calcined substrate a lost render exposes, which is
#    `_FLAT["spall_face"]`'s own rule ("a dirty buff about 1.5x the wall, not
#    3x") applied per building instead of as one constant for all of them.
#    `fire_collapse.is_fake_interior` gates the pick, exactly as it gates the
#    tear's: an office card behind the glazing is never the façade, so it can
#    never become the thing under the render either.
#
#    THE SAMPLE CARRIES THE HUE, THE PALETTE KEEPS THE VALUE. `PEEL_FACE_GAIN`
#    and `PEEL_FACE_CLAMP` act on the tone's MEAN and rescale the triple, so a
#    brownstone's warm substrate stays warm — the module atlases measure
#    linear (0.091, 0.087, 0.075) and a per-channel clamp at any fire-palette
#    ceiling returns the same grey for every one of them. The window is the
#    palette's own light end: `_FLAT["soot_light"]` (0.048 mean) to a shade
#    over `_FLAT["calcined"]` (0.062), which is what calcined masonry under
#    soot measures and what stops a peel becoming a cow spot.
PEEL_FACE_GAIN = 1.55
PEEL_FACE_CLAMP = (0.030, 0.075)      # linear albedo, `_FLAT`'s own scale
PEEL_FACE_PREFIX = "/FireLooks/peelface_"


def _wall_relief(ctx, e, fr):
    """This module's OUTWARD-facing triangles in its OWN frame, as
    `(u0, u1, v0, v1, depth)` arrays — `u` along the wall the way
    `quake_flow._b_face_pt` measures it, `v` from the module's own base z, and
    `depth` metres in FRONT of the placement line.

    Cached per piece name (plus its stretch, the only thing that changes a
    placed module's shape), because every placement of a kit module is the
    same geometry in its own frame: a 40-module terrace measures once. `None`
    when the module is not on the stage or has no outward face to measure.
    """
    import numpy as np
    from pxr import Usd, UsdGeom
    from . import fire_collapse as fc

    p = e.get("p") or {}
    cache = ctx.setdefault("cache", {}).setdefault("wall_relief", {})
    key = (e.get("name"), tuple(p.get("stretch") or ()))
    hit = cache.get(key)
    if hit is not None:
        return hit or None
    path = p.get("prim_path")
    root = ctx["stage"].GetPrimAtPath(path) if path else None
    if not root or not root.IsValid() or not root.IsActive():
        cache[key] = False
        return None
    ox, oy, yaw, width, _h, _d, dw = fr
    ca, sa = math.cos(yaw), math.sin(yaw)
    along = np.array((ca, sa), dtype=float)      # `_b_face_pt`'s u direction
    outw = np.array((sa, -ca), dtype=float)      # ...and its outward normal
    org = np.array((float(ox), float(oy)), dtype=float)
    z0 = float(e.get("z") or 0.0)
    half = 0.5 * float(width) if dw else 0.0     # `_b_face_pt`'s own dw shift
    xfc = UsdGeom.XformCache()
    rows = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        cnt = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not cnt or not idx:
            continue
        M = np.array(xfc.GetLocalToWorldTransform(prim), dtype=float)
        P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        P = P @ M[:3, :3] + M[3, :3]
        ix, tri, o = list(idx), [], 0
        for c in cnt:
            c = int(c)
            for k in range(1, c - 1):
                tri.append((ix[o], ix[o + k], ix[o + k + 1]))
            o += c
        if not tri:
            continue
        T = P[np.asarray(tri, dtype=np.int64)]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        ok = ln > 1e-12
        if not ok.any():
            continue
        # OUTWARD-FACING, on the tear's own threshold: a sill's top face and a
        # window reveal's return are not the surface a stamp goes on.
        face = np.zeros(len(T), dtype=bool)
        face[ok] = (n[ok, :2] @ outw) / ln[ok] > fc.TEAR_OUT_COS
        if not face.any():
            continue
        Q = T[face]
        rel = Q[:, :, :2] - org
        u = rel @ along + half
        d = rel @ outw
        v = Q[:, :, 2] - z0
        rows.append(np.stack([u.min(1), u.max(1), v.min(1), v.max(1),
                              d.mean(1)], axis=1))
    if not rows:
        cache[key] = False
        return None
    A = np.vstack(rows)
    out = (A[:, 0], A[:, 1], A[:, 2], A[:, 3], A[:, 4])
    cache[key] = out
    return out


def _stamp_frame(ctx, e, fr, u, v):
    """`fr` with its depth replaced by the module's MEASURED outward surface
    at `(u, v)` — the wall the stamp is on, not the front of the bay hung off
    it. The unchanged frame when nothing can be measured, so a module that is
    not on the stage behaves exactly as it did.
    """
    import numpy as np

    m = _wall_relief(ctx, e, fr)
    if m is None:
        return fr
    u0, u1, v0, v1, d = m
    if not len(d):
        return fr
    vr = float(v) - float(e.get("z") or 0.0)
    sel = (u0 <= u) & (u <= u1) & (v0 <= vr) & (vr <= v1)
    # THE FRONTMOST SURFACE AT THAT POINT, because that is the one you can
    # see; away from any outward face (over an opening, say) the module's
    # median depth is the wall itself.
    depth = float(d[sel].max()) if bool(sel.any()) else float(np.median(d))
    depth = max(-PEEL_RECESS_MAX, min(depth, -float(fr[5])))
    return tuple(fr[:5]) + (-(depth + PEEL_WALL_PAD),) + tuple(fr[6:])


def _module_skin(ctx, e):
    """`{"mat", "tex", "uv"}` for this module's OWN outward cladding as it
    stands NOW — `fire_collapse.tone_material` takes exactly this shape
    (`facade_skin`'s return, without the geometry a flat tone does not need).

    After `r_smoke_stain` the material bound here is the sooted copy
    (`_bind_soot`'s `FireLooks/soot_N`), which is the point: the tone has to
    be sampled from the wall as the fire left it, not as the kit shipped it.
    A fake-interior or glass material is never a candidate — the same two
    rules `facade_skin` applies, for the same reason.
    """
    from pxr import Usd, UsdGeom, UsdShade
    from . import fire_collapse as fc, soot_plume as spl

    p = e.get("p") or {}
    path = p.get("prim_path")
    root = ctx["stage"].GetPrimAtPath(path) if path else None
    if not root or not root.IsValid():
        return None
    best = None
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        arr = _mesh_arrays(prim)
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        for t, sub in ([(q.GetPrim(), q) for q in subs] or [(prim, None)]):
            try:
                mat = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            except Exception:
                continue
            if not mat or not mat.GetPrim().IsValid():
                continue
            mp = str(mat.GetPrim().GetPath())
            _sh, _in, tex = spl.find_basecolor(mat.GetPrim())
            if not tex:
                continue
            if fc.is_fake_interior(mp, tex):
                continue                       # never the façade (row 5)
            low = (mp + "|" + str(tex)).lower()
            if any(k in low for k in fc.TEAR_GLASS_HINTS):
                continue                       # glass is not the substrate
            n_face = (len(sub.GetIndicesAttr().Get() or ()) if sub is not None
                      else (len(arr["counts"]) if arr else 0))
            # the building's OWN materials first (the per-cell sooted copies
            # live under `ctx["parent"]`), then whichever covers most faces
            own = 1 if mp.startswith(str(ctx["parent"]) + "/") else 0
            rank = (own, int(n_face))
            if best is None or rank > best[0]:
                best = (rank, mp, tex, arr)
    if best is None:
        return None
    _rank, mp, tex, arr = best
    # The UV box is the whole mesh's, not the chosen subset's: these maps are
    # per-module atlases (`_bind_soot`: "the base maps are UV ATLASES"), so
    # the mesh's own box already crops to this module's corner of the sheet,
    # and `tone_material` only takes a MEAN over it.
    return {"mat": mp, "tex": tex, "glass": False,
            "uv": (arr["uv"] if arr is not None else None)}


def _peel_face_mat(ctx, e):
    """(material, "tone"|"flat") — what the lost render exposes on THIS
    module: its own sooted tone opened up to the substrate, or `spall_face`
    when the module's map cannot be read at all."""
    from . import fire_collapse as fc

    sk = _module_skin(ctx, e)
    mat = None
    if sk is not None:
        mat = fc.tone_material(ctx, sk, gain=PEEL_FACE_GAIN,
                               clamp=PEEL_FACE_CLAMP,
                               prefix=PEEL_FACE_PREFIX, cache_key="peel_face")
    if mat is None:
        return ctx["mats"]["spall_face"], "flat"
    return mat, "tone"


def r_render_peel(ctx, rate=0.35):
    """Rendered masonry: stucco comes off in SHEETS and lands at the wall foot.

    Different failure from concrete spalling and it has to look different:
    render debonds over a continuous area and drops as slabs, so the scars are
    fewer and flatter with a windrow of pieces under them, where a spall is
    many small dishes with nothing under them worth authoring.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    n_patch, sides_hit = 0, set()
    n_tone = n_flat = 0
    if ctx["info"].get("family") not in PEEL_FAMILIES:
        # not rendered — spall it instead, so the ladder entry is never a
        # silent no-op on a brick or stone building
        r_spall(ctx, rate=rate * 0.8)
        return
    for e in qf._els(ctx, role=("wall", "corner")):
        if e["side"] not in f["sides"]:
            continue
        sev = _severity(ctx, e["storey"], e["mass"], e)
        if sev < 0.35 or rng.random() > rate * sev:
            continue
        fr = qf._piece_frame(e)
        if fr is None:
            continue
        width = fr[3]
        face_mat, how = _peel_face_mat(ctx, e)
        n_tone += 1 if how == "tone" else 0
        n_flat += 1 if how == "flat" else 0
        for _ in range(rng.randint(1, 2)):
            u = rng.uniform(0.2 * width, 0.8 * width)
            v = e["z"] + rng.uniform(0.4, max(0.6, e["h"] - 0.4))
            # 0.25-0.7 m, NOT 0.7-1.5. A 3 m blob on a 4 m module is most of
            # the module, and a patch that big cannot read as anything but a
            # decal however it is shaded.
            ra = rng.uniform(0.18, 0.48)
            _scar(ctx, _stamp_frame(ctx, e, fr, u, v), u, v, ra,
                  ra * rng.uniform(0.8, 1.7), face_mat, "peel", owner=e)
            n_patch += 1
        sides_hit.add(e["side"])
    # the windrow of fallen render at the foot of each affected wall
    m = ctx["info"]["masses"][f["mass"]]
    for side in sorted(sides_hit):
        qf._heap(ctx, m, m["z0"], 0.0, 0.06, fill=False, sides=(side,),
                 depth_m=rng.uniform(0.16, 0.34), along=(-0.42, 0.42),
                 tag="peelrow",
                 mat_fn=lambda: (qf._a_mat(ctx, "dust") if rng.random() < 0.35
                                 else ctx["mats"]["soot_light"]))
    # THE FACE TALLY IS THE TRIPWIRE, in the sidecar of every bake: a module
    # whose own map could not be read falls back to the one flat `spall_face`
    # tone, and a regression to "every peel in the city wears the same
    # material" is loud here instead of waiting for a review.
    ctx["notes"].append(
        "render peel: {0} sheet(s) off on {1} module(s) ({2} on the module's "
        "own tone, {3} on the flat fallback), windrow on {4}".format(
            n_patch, n_tone + n_flat, n_tone, n_flat,
            "/".join(sorted(sides_hit)) or "-"))


# ---------------------------------------------------------------------------
# 5. The inside: the part that actually burns
# ---------------------------------------------------------------------------
def r_gut_interior(ctx, frac=1.0):
    """Burn the contents and the partitions in the involved storeys.

    THE FUEL LOAD IS THE CONTENTS, NOT THE BUILDING. `quake_flow.fit_interior`
    has already put slabs, columns, partitions and furniture in; a fire
    removes the furniture (it is gone — that is what burnt means), topples or
    consumes the partitions, and chars everything left. Without this pass an
    empty window opening on a burnt building shows a tidy office with its desk
    still in it, which is worse than showing nothing.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    fit = ctx.get("fit") or {}
    n_gone = n_char = 0
    for (mtag, storey), props in (fit.get("props") or {}).items():
        sev = _severity(ctx, storey, mtag)
        if sev < 0.2:
            continue
        for p in props:
            if rng.random() < frac * sev:
                if qf._deactivate(ctx["stage"], p):
                    n_gone += 1
            else:
                qf._b_bind_over(ctx["stage"], p, _burn_mat(ctx, "char"))
                n_char += 1
    # Partitions: a plaster-on-stud partition in a burnt-out room is either
    # gone or standing as a charred skeleton. Half go, half char.
    n_part = 0
    for p in (fit.get("partitions") or []):
        s, pm = _storey_of_path(ctx, p)
        sev = _severity(ctx, s, pm) if s is not None else 0.0
        if sev < 0.25:
            continue
        if rng.random() < 0.5 * frac * sev:
            if qf._deactivate(ctx["stage"], p):
                n_part += 1
        else:
            qf._b_bind_over(ctx["stage"], p, _burn_mat(ctx, "char"))
    # Slabs and columns keep their geometry (concrete does not burn) but the
    # top face of a slab in a burnt room is black.
    n_slab = 0
    # THE CATCH FLOOR IS BLACK, WHATEVER `_severity` SAYS — GAC ONLY. A slab
    # the roof fell onto, or one in the top three storeys of a building whose
    # roof burned through, is what the camera sees looking straight down
    # through the opening — `_severity`'s falloff from the fire's origin has
    # nothing to do with whether this floor is on show, and a pale slab
    # under a black shell was the single most visible wrong thing in the
    # frame ("the inside floor of the collapsed roof looks unburnt", user
    # review, 2026-08-30). `collapse_storeys` is the exact set
    # `r_fire_collapse` drops its wreckage onto, when this level's ladder
    # also carries that recipe; the top-three fallback covers a roof
    # burn-through with no collapse in the ladder. GATED to the sliced GAC
    # path (`ctx["soot_prebaked"]` is a set/frozenset only when
    # `gac_fire.burn_gac` set it): the kit-building fire look is frozen
    # (2026-08-30) and must draw nothing new here.
    is_gac = isinstance(ctx.get("soot_prebaked"), (set, frozenset))
    roof_hit = is_gac and bool(f.get("roof"))
    collapse_storeys = ctx.get("collapse_storeys") if is_gac else None
    for (mtag, storey), slab in (fit.get("slabs") or {}).items():
        # `slab` can be None: `r_roof_hole` clears the entry for a floor it
        # broke, and every `pxr` call takes an ArgumentError on None rather
        # than missing quietly.
        if not slab:
            continue
        is_catch = False
        if is_gac:
            m_ = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
            n_st = len(m_["levels"])
            is_catch = (collapse_storeys is not None and storey in collapse_storeys) \
                or (roof_hit and storey >= n_st - 3)
        if not is_catch and _severity(ctx, storey, mtag) < 0.3:
            continue
        # THE FLOOR OF A BURNT ROOM IS THE DARKEST SURFACE IN IT — everything
        # that was in the room is on it. Bound to the shared `concrete`
        # (megascans Worn_Pavement, a PALE map) it came out as a clean pink
        # plate at the bottom of a black shell, seen straight down through
        # the roof hole (uf_r2e, 2026-08-28).
        qf._b_bind_over(ctx["stage"], slab,
                        ctx["mats"]["char_concrete"] if rng.random() < 0.5
                        else _burn_mat(ctx, f["finish"]))
        n_slab += 1
    # COLUMNS ARE THE PILLARS A BURNT ROOM IS SEEN THROUGH. `fit_interior`
    # binds a frame's columns to the shared `concrete` (megascans
    # Worn_Pavement, a PALE map) and no fire pass touched them, so an `rc`
    # partial collapse showed clean pale posts inside a black shell — the
    # one structure in the room that did not look burnt (user review,
    # 2026-08-30). Same tone as the beams `r_expose_interior` gives an `rc`
    # frame: `steel`, cool and dark, not the warm `burnt_metal` that reads
    # as timber.
    #
    # ON A SLICED BUILDING THE COLUMNS ARE CHARRED, NOT MERELY STEEL.
    # `steel` is a flat cool grey at ~0.26 screen with 0.55 roughness and no
    # map on it: measured on the fire_dtc3 bench, all 147 of
    # `gac_SM_Building_02_F5c_s193`'s fit columns were ALREADY on it — there
    # was no coverage gap — and they still read as clean posts in a burnt
    # room ("since this is a fire they have to look scorched", user review,
    # 2026-08-30). An RC column that has been through a compartment fire is
    # blackened, spalled concrete, so the sliced path takes exactly the
    # treatment the slabs above take: the flat `char_concrete` or one of the
    # textured burn maps, which is what stops char reading as paint. The kit
    # path keeps `steel` untouched, and with it `kit_burn_probe`'s
    # pale-columns FLAG (nothing there is on the quake `concrete` either
    # way).
    n_col = 0
    for (mtag, storey), cols in (fit.get("columns") or {}).items():
        # ...and it reaches the storeys a collapse or a roof hole put ON SHOW
        # as well as the ones the fire's own falloff calls hot — the same
        # `is_catch` the slab loop above needed, for the same reason.
        is_catch = False
        if is_gac:
            m_ = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
            is_catch = (collapse_storeys is not None
                        and storey in collapse_storeys) \
                or (roof_hit and storey >= len(m_["levels"]) - 3)
        if not is_catch and _severity(ctx, storey, mtag) < 0.25:
            continue
        for c in cols:
            qf._b_bind_over(ctx["stage"], c, ctx["mats"]["steel"] if not is_gac
                            else (ctx["mats"]["char_concrete"]
                                  if rng.random() < 0.5
                                  else _burn_mat(ctx, f["finish"])))
            n_col += 1
    ctx["notes"].append(
        # the last word is the only thing the sliced path changes here, and
        # the kit path still says "steel" — `kit_burn_probe`'s note is
        # byte-identical, which is what the MCE freeze asks for
        "gutted: {0} prop(s) consumed, {1} charred, {2} partition(s) down, "
        "{3} slab(s) charred, {4} column(s) {5}".format(
            n_gone, n_char, n_part, n_slab, n_col,
            "charred" if is_gac else "steel"))


def _storey_of_path(ctx, path):
    """`fit_<tag>/part_<mass>_<storey>_<k>` -> (storey, mass).

    `fit_interior` encodes both in the prim name and nowhere else, and the
    mass matters: a partition on storey 3 of the podium and one on storey 3
    of the tower are 40 m apart. The mass tag itself contains underscores
    (`wing0`, `wing0_wing1`), so it is everything between `part_` and the
    last two numeric fields.
    """
    name = str(path).rsplit("/", 1)[-1]
    bits = name.split("_")
    if len(bits) < 4 or not bits[-1].isdigit() or not bits[-2].isdigit():
        return None, None
    return int(bits[-2]), "_".join(bits[1:-2])


def _slab_concrete(ctx):
    """Rebind every fit-out slab to concrete, whatever the construction type.

    `quake_flow.fit_interior` gives a `urm` building a TIMBER deck, which is
    correct for a nineteenth-century masonry block and correct for a quake —
    the joists are what fail. It is wrong here for the same reason the timber
    debris was: the deck is the surface you see through every burnt-out
    window and down an opened roof, and a plank texture in a concrete street
    is the single most visible piece of the wrong disaster (user review,
    2026-08-28).
    """
    from . import quake_flow as qf
    n = 0
    for path in (ctx.get("fit") or {}).get("slabs", {}).values():
        if not path:
            continue
        qf._b_bind_over(ctx["stage"], path, ctx["mats"]["slab_concrete"])
        n += 1
    return n


def r_floor_burnthrough(ctx, p=1.0, keep_below=1, budget=1):
    """The top floors in the burnt band come down — and NO MORE THAN THAT.

    A fire-weakened floor plate fails locally: a bay lets go, the storey above
    drops onto the storey below, and the drop is arrested there. It does not
    unzip the building. `budget` caps how many slabs may fail and `keep_below`
    protects that many at the bottom of the mass whatever `p` says — together
    they are the whole reason this stops looking like a demolition. With every
    slab in the band taken, an opened roof looked straight down an EMPTY SHELL
    to the ground floor (user review, 2026-08-28), which is a bombed building,
    not a burnt one.

    Failures start at the TOP, because that is where the heat collects and
    where the roof lands when it comes through.

    The slab is fractured and dropped rather than deleted: an empty shell with
    no floor debris in the bottom reads as a building under construction.
    """
    from . import fracture, quake_flow as qf

    f, rng, nrng = ctx["fire"], ctx["rng"], ctx["nrng"]
    fit = ctx.get("fit") or {}
    # NO PLANK MODE, NO TIMBER. `mode="plank"` cuts a lattice of BOARDS,
    # which is the tornado path's debris and exactly what a concrete slab
    # does not do — it breaks into blocky prisms with the reinforcement
    # trailing out of them.
    timber = False
    n_down = n_kept = n_prop = 0
    _slab_concrete(ctx)
    # highest first: the heat and the falling roof are both up there
    cand = sorted((fit.get("slabs") or {}).items(), key=lambda kv: -kv[0][1])
    lowest = {}
    for (mtag, storey), _sl in cand:
        lowest[mtag] = min(lowest.get(mtag, 99), storey)
    left = int(budget)
    for (mtag, storey), slab in cand:
        if not slab or _severity(ctx, storey, mtag) < 0.45:
            continue
        if left <= 0 or storey <= lowest.get(mtag, 0) + keep_below - 1 \
                or rng.random() > p:
            n_kept += 1
            continue
        left -= 1
        pr = ctx["stage"].GetPrimAtPath(slab)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        out = slab + "_brn"
        # SEEDS SCALE WITH AREA, AND THE PIECES ARE CAPPED. 9-14 seeds over a
        # 20 x 16 m floor plate is 25 m2 a cell, so a burnt-out top storey
        # came down as four-metre RAFTS standing on edge inside the shell —
        # visible straight down the open roof and unmistakably wrong (uf_bench
        # commercial, 2026-08-28). A burnt timber deck arrives as boards and
        # joist lengths: one seed per ~3.5 m2, and nothing over 2.6 m.
        area = max(4.0, m_area(ctx, mtag))
        n_seed = int(min(90, max(12, area / 3.5)))
        made = fracture.fracture_prim(
            ctx["stage"], slab, out, n_pieces=n_seed, rng=nrng,
            mode=("plank" if timber else "uniform"),
            aspect=((2.6, 6.0) if timber else None),
            rough=0.012, verbose=False, consume=(0.42 if timber else 0.18),
            consume_pool=1.05, min_volume_frac=0.0004,
            max_piece_m=(2.6 if timber else 3.2))
        for pth in made:
            qf._bind(ctx["stage"], pth,
                     ctx["mats"]["char_concrete"] if rng.random() < 0.4
                     else _debris_mat(ctx))
        ctx["loose"] += made
        n_down += 1
        # THE FURNITURE ON THIS FLOOR HAS NOTHING LEFT UNDER IT NOW.
        # `quake_flow.fit_interior` seats `props[(mtag, storey)]` at
        # `z + 0.01`, directly on `slabs[(mtag, storey)]` (top face at `z`).
        # This loop just fractured that slab and handed the pieces to
        # `loose`, but never looked at the props sitting on it: whichever of
        # them `r_gut_interior` charred in place rather than deactivating is
        # still a live prim at the old floor height with no slab under it —
        # and outside `loose` it never gets a rigid body, so it hangs there
        # for the rest of the run. This is what measured out on the GAC
        # bench as a cluster of furniture-coloured boxes hanging 10-25 m
        # over the roof of the one building whose top-storey slab failed
        # (row1_gac.png; debris_float_probe.py confirms the population and
        # that it never reaches `loose`).
        for pp in (fit.get("props") or {}).get((mtag, storey), []):
            pr2 = ctx["stage"].GetPrimAtPath(pp)
            if pr2 and pr2.IsValid() and pr2.IsActive() and pp not in ctx["loose"]:
                ctx["loose"].append(pp)
                n_prop += 1
        # what did NOT fall: the bearing stubs still in the wall pocket,
        # which is what a USAR photograph of a gutted shell has
        _joist_stubs(ctx, mtag, storey)
    ctx["notes"].append(
        "floors: {0} slab(s) failed, {1} kept so the shell is not hollow, "
        "{2} prop(s) sent down with them".format(n_down, n_kept, n_prop))


def m_area(ctx, mtag):
    """Floor area of one mass, in m2 — how many pieces a slab should make."""
    from . import quake_flow as qf
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    return max(1.0, (m["W"] - 2 * qf.WALL_INSET) * (m["D"] - 2 * qf.WALL_INSET))


def _joist_stubs(ctx, mtag, storey, n=None):
    """Bearing stubs of a failed slab, still in the wall pocket.

    SQUARE-SECTION POSTS, NOT RODS. The old thin `_cyl` (0.06-0.08 m radius,
    `burnt_metal`) read as a wire, and `burnt_metal`'s warm brown-black at
    this luminance read as scorched timber ("rods, probably structural. They
    looked wooden though", user review, 2026-08-30). An upright box at the
    same wall-pocket base — a small random yaw in place of the old 3-D tilt —
    reads as a burnt structural column, a pillar, which is what was asked
    for; bound to the cooler `steel` tone.
    """
    from . import quake_flow as qf
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    rng = ctx["rng"]
    # A POST STANDS ON A FLOOR. The failed slab is `levels[storey]`; the stub
    # is the column that carried it, so it rises from the floor BELOW it and
    # stops short of where the slab was — hung from the slab line instead it
    # floated between two floors.
    if storey < 1 or storey >= len(m["levels"]):
        return
    # ...and only when that floor EXISTS as geometry: the kit shell has no
    # floors of its own, `fit_interior` puts slabs in fitted storeys only,
    # and a post over an unfitted storey stood 5 m up in clear air
    # (fire_row1 office_wide F5c, 2026-08-30). `fit["slabs"][(mass, i)]` is
    # the floor OF storey i.
    fit = ctx.get("fit") or {}
    if not (fit.get("slabs") or {}).get((mtag, storey - 1)):
        return
    z_slab = m["levels"][storey] - 0.18
    z_floor = m["levels"][storey - 1]
    if z_slab - z_floor < 0.6:
        return
    W, D = m["W"] - 2 * qf.WALL_INSET, m["D"] - 2 * qf.WALL_INSET
    n = n if n is not None else rng.randint(3, 7)
    for _ in range(n):
        side = rng.random() < 0.5
        L = rng.uniform(0.5, min(1.8, z_slab - z_floor))
        if side:
            lx = rng.uniform(-W / 2, W / 2)
            ly = (D / 2) * rng.choice((1.0, -1.0))
        else:
            ly = rng.uniform(-D / 2, D / 2)
            lx = (W / 2) * rng.choice((1.0, -1.0))
        wx, wy = qf._to_world(m, lx, ly)
        s = rng.uniform(0.18, 0.24)
        path = "{0}/joist_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        qf._box(ctx["stage"], path, wx, wy, z_floor + L / 2.0, s, s, L,
                rng.uniform(-15.0, 15.0), ctx["mats"]["steel"])
        ctx["authored"].append(path)


# ---------------------------------------------------------------------------
# 5a. ROOFTOP PLANT — laid out, not scattered
# ---------------------------------------------------------------------------
# A ROOF IS A DESIGNED SURFACE AND ITS PLANT IS INSTALLED, NOT DROPPED.
# `quake_flow.dress_roof` places 0-2 tanks and 2-6 AC units at uniformly
# random points inset from the parapet, which is fine when the point is that
# a quake tipped them over — nobody looks at where they started. From
# straight down, which is where this dataset is shot from, it is immediately
# wrong: "the AC units seem random instead of structured like it would on an
# actual building" (user review, 2026-08-28).
#
# What a real flat roof has, in the order it is built:
#   * a STAIR/LIFT BULKHEAD — one box, against an edge, squared to the
#     building. It is the tallest thing up there and it anchors everything.
#   * a HOUSEKEEPING PAD carrying the condensers in a ROW (or two rows back
#     to back), all on the same axis, evenly pitched, service clearance in
#     front. Plant is craned in and bolted down on a grid.
#   * a TANK on a stand, next to the bulkhead because that is where the riser
#     is.
#   * VENT STACKS through the deck, small and in a line.
# Every one of those is authored in the mass's own frame, so the whole
# installation is square to the building at any yaw.
AC_W, AC_D, AC_H = 1.05, 0.62, 0.78     # a packaged condenser
AC_PITCH = 1.7                          # bolt spacing along a row
PAD_MARGIN = 0.45                       # housekeeping pad past the units
BULKHEAD = (4.2, 3.0, 2.6)              # stair head: w, d, h


def _roof_tiles(ctx, mass):
    """`[(x0, y0, x1, y1, z), ...]` — one world-XY footprint + a
    representative deck height PER `role="roof"` element of this mass,
    cached in `ctx["cache"]`.

    PER-TILE, not one bbox over the whole mass: `UsdGeom.BBoxCache` on the
    mass would union every tier into the tallest one everywhere inside it —
    exactly the bug this exists to fix (a multi-tier or L-shaped massing's
    lower wing has its OWN roof piece(s), at its OWN height). Each tile's
    footprint is the min/max of its OWN mesh points (never `BBoxCache`),
    and its height is the mean of those points' Z — a flat deck tile's
    points are all within a few mm of the same plane, so mean and max agree
    to noise; mean is used because a stray point (a rim vertex, a lip)
    should not set the whole tile's seat.
    """
    cache = ctx.setdefault("cache", {})
    key = ("roof_tiles", mass)
    if key in cache:
        return cache[key]
    from pxr import Usd, UsdGeom

    from . import quake_flow as qf
    stage = ctx["stage"]
    tiles = []
    for e in qf._els(ctx, mass=mass, role="roof"):
        path = e["p"].get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        xlo = ylo = float("inf")
        xhi = yhi = float("-inf")
        zsum, zn = 0.0, 0
        for sub in Usd.PrimRange(prim):
            if not sub.IsA(UsdGeom.Mesh):
                continue
            raw = UsdGeom.Mesh(sub).GetPointsAttr().Get()
            if not raw:
                continue
            xf = UsdGeom.Xformable(sub).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            for pt in raw:
                w = xf.Transform(pt)
                x, y, zz = float(w[0]), float(w[1]), float(w[2])
                xlo, xhi = min(xlo, x), max(xhi, x)
                ylo, yhi = min(ylo, y), max(yhi, y)
                zsum += zz
                zn += 1
        if zn:
            tiles.append((xlo, ylo, xhi, yhi, zsum / zn))
    cache[key] = tiles
    return tiles


def _local_roof_z(ctx, mass, wx, wy, pad0=0.3, pad_max=4.0):
    """The deck height of whichever roof tile's own XY footprint covers
    `(wx, wy)`. `pad0`/`pad_max` forgive a query that lands just past a
    tile's own measured edge (a mesh seam, a prop drawn near a parapet);
    when several tiles legally overlap the padded box (a step between
    tiers) the HIGHEST deck under the point wins — the one actually
    reachable from directly above, where this dataset is shot from.

    `None` means nothing found by `pad_max`: this position is off the
    mass's real roof entirely — e.g. over the missing wing of an L — and
    the caller drops the item rather than seat it on a made-up height
    ("the roof props look floating since they are placed at roof height
    but parts of the building that aren't roof height", user, 2026-08-31).
    """
    tiles = _roof_tiles(ctx, mass)
    if not tiles:
        return None
    pad = float(pad0)
    while pad <= pad_max:
        hits = [z for (x0, y0, x1, y1, z) in tiles
                if x0 - pad <= wx <= x1 + pad and y0 - pad <= wy <= y1 + pad]
        if hits:
            return max(hits)
        pad *= 2.0
    return None


def dress_roof_urban(ctx, mass=None):
    """Structured rooftop plant. Replaces `quake_flow.dress_roof`.

    Returns the prim paths and registers them the same way, so
    `quake_flow._b_settle_roof_plant` still hands them to the solver at the
    end of the run and a burnt roof still drops its plant through the hole.
    """
    from . import quake_flow as qf

    rng = ctx["rng"]
    mass = mass or "main"
    m = ctx["info"]["masses"][mass]
    if len(m["levels"]) < 2:
        ctx["roof_plant"] = []
        return []
    # THE PLANT SITS ON THE REAL DECK, NOT ON THE PARAPET COPING. `m["top"]`
    # is the bbox top -- the parapet, on a GAC building measured well over a
    # metre proud of the actual roof surface (`gac_fire.mass_from_grid`'s
    # `deck_z`) -- so a bulkhead/pad/tank/vent authored at `top` hung in the
    # air above the true roof (row-2 review, 2026-08-30: "floating roof
    # props"). READ OFF `ctx["fire"]`, NOT `m`: `burn_building` rebuilds its
    # own mass box from the sliced pieces (`quake_flow.describe`) and that
    # rebuilt `m` never carries `deck_z` -- `fire` is the one dict
    # `gac_fire.prepare` hands `burn_gac`/`burn_building` UNCHANGED (`fire=`
    # all the way through), so that is where `prepare` stashes it. The kit
    # path's `plan_fire` never sets this key, so `.get` falls straight back
    # to `top` there -- byte-identical.
    z = ctx["fire"].get("deck_z", m["top"]) + 0.02
    W, D = m["W"], m["D"]
    made, kind = [], ctx.setdefault("roof_plant_kind", {})
    mat_metal = ctx["mats"]["plant_metal"]
    mat_deck = ctx["mats"]["dark_concrete"]
    # LOCAL SEATING, GAC PATH ONLY. `z` above is ONE height for the whole
    # mass; on a multi-tier or L-shaped whole-asset building a prop drawn
    # over a lower wing floats at the tall tier's height instead ("the roof
    # props look floating since they are placed at roof height but parts of
    # the building that aren't roof height", user, 2026-08-31). Gated on
    # `soot_prebaked` being a real `set`/`frozenset` (only `gac_fire.
    # burn_gac` sets it that way) so a kit building's `_seat_z` always
    # returns the same flat `z` it always did — byte-identical,
    # `kit_burn_probe.py` — because every kit style is a plain box with one
    # true roof height everywhere on it anyway.
    is_gac = isinstance(ctx.get("soot_prebaked"), (set, frozenset))
    n_dropped = [0]

    def _seat_z(wx, wy):
        if not is_gac:
            return z
        zl = _local_roof_z(ctx, mass, wx, wy)
        if zl is None:
            n_dropped[0] += 1
            return None
        return zl + 0.02

    def _box(lx, ly, off, sx, sy, sz, mat, tag):
        wx, wy = qf._to_world(m, lx, ly)
        zb = _seat_z(wx, wy)
        if zb is None:
            return None
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"],
                                        qf._uid(ctx))
        qf._box(ctx["stage"], path, wx, wy, zb + off, sx, sy, sz, m["yaw"],
                mat)
        ctx["authored"].append(path)
        return path

    # --- the bulkhead, hard against one edge and squared to it -------------
    bw, bd, bh = BULKHEAD
    side = rng.choice(("S", "N", "E", "W"))
    inset = 1.2
    if side in ("S", "N"):
        blx = rng.uniform(-W / 2 + bw / 2 + 1.0, W / 2 - bw / 2 - 1.0)
        bly = (-D / 2 + bd / 2 + inset) if side == "S" else (D / 2 - bd / 2 - inset)
    else:
        bly = rng.uniform(-D / 2 + bd / 2 + 1.0, D / 2 - bd / 2 - 1.0)
        blx = (-W / 2 + bw / 2 + inset) if side == "W" else (W / 2 - bw / 2 - inset)
    if W > bw + 4 and D > bd + 4:
        pth = _box(blx, bly, bh / 2.0, bw, bd, bh, mat_deck, "bulkhead")
        if pth:
            made.append(pth)
            kind[pth] = "bulkhead"
            # its own parapet-height upstand roof, so it is not a bare box
            pth = _box(blx, bly, bh + 0.10, bw + 0.30, bd + 0.30, 0.20,
                       mat_deck, "bulkcap")
            if pth:
                made.append(pth)
                kind[pth] = "bulkhead"

    # --- the condenser rows -----------------------------------------------
    # laid ALONG the building's long axis, on the opposite half from the
    # bulkhead so the two do not fight for the same corner
    long_x = W >= D
    run = (W if long_x else D) - 6.0
    n_ac = max(2, min(8, int(run / AC_PITCH)))
    n_ac = min(n_ac, rng.randint(3, 7))
    rows = 2 if (min(W, D) > 14.0 and n_ac >= 4 and rng.random() < 0.55) else 1
    per_row = max(1, n_ac // rows)
    # centre the run, and put it on the far side from the bulkhead
    off_sign = -1.0 if ((bly if long_x else blx) > 0) else 1.0
    cross = (D if long_x else W) * 0.22 * off_sign
    span = (per_row - 1) * AC_PITCH
    for r in range(rows):
        cr = cross + (r - (rows - 1) / 2.0) * (AC_D + 1.1)
        for k in range(per_row):
            a = -span / 2.0 + k * AC_PITCH
            lx, ly = (a, cr) if long_x else (cr, a)
            wx, wy = qf._to_world(m, lx, ly)
            # the housekeeping pad under the row, once per row — its OWN
            # position, not the first unit's: the pad runs the row's full
            # span and can cover ground the row's own units don't
            if k == 0:
                plx, ply = ((0.0, cr) if long_x else (cr, 0.0))
                pw = (span + AC_W + 2 * PAD_MARGIN) if long_x else (AC_D + 2 * PAD_MARGIN)
                pd = (AC_D + 2 * PAD_MARGIN) if long_x else (span + AC_W + 2 * PAD_MARGIN)
                padp = _box(plx, ply, 0.06, pw, pd, 0.12, mat_deck, "acpad")
                if padp:
                    made.append(padp)
                    kind[padp] = "pad"
            zb = _seat_z(wx, wy)
            if zb is None:
                n_dropped[0] += 1
                continue
            path = "{0}/ac_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                           qf._uid(ctx))
            if qf._prop(ctx["stage"], path, qf._AC_UNIT, wx, wy, zb + 0.12,
                        m["yaw"] + (0.0 if long_x else 90.0), 1.0, rng):
                qf._b_bind_over(ctx["stage"], path, mat_metal)
                made.append(path)
                kind[path] = "ac"
                ctx["authored"].append(path)

    # --- the tank, beside the bulkhead on a stand -------------------------
    if rng.random() < 0.6 and W > 10 and D > 10:
        tr = rng.uniform(1.0, 1.35)
        tlx = blx + (0.0 if side in ("S", "N") else (3.4 if side == "W" else -3.4))
        tly = bly + (3.4 if side == "S" else -3.4 if side == "N" else 0.0)
        tlx = max(-W / 2 + tr + 1.2, min(W / 2 - tr - 1.2, tlx))
        tly = max(-D / 2 + tr + 1.2, min(D / 2 - tr - 1.2, tly))
        wx, wy = qf._to_world(m, tlx, tly)
        zb = _seat_z(wx, wy)
        if zb is None:
            n_dropped[0] += 1
        else:
            for pth in qf._tank(ctx, wx, wy, zb, r=tr, h=rng.uniform(2.2, 2.8),
                                yaw=m["yaw"]):
                qf._b_bind_over(ctx["stage"], pth, mat_metal)   # steel, not cedar
                made.append(pth)
                kind[pth] = "tank"

    # --- vent stacks, in a line off the bulkhead --------------------------
    for k in range(rng.randint(2, 4)):
        vlx = blx + (k - 1) * 1.5 * (0.0 if side in ("E", "W") else 1.0)
        vly = bly + (k - 1) * 1.5 * (1.0 if side in ("E", "W") else 0.0)
        vlx = max(-W / 2 + 1.0, min(W / 2 - 1.0, vlx + (2.0 if side in ("S", "N") else 0.0)))
        vly = max(-D / 2 + 1.0, min(D / 2 - 1.0, vly + (2.0 if side in ("E", "W") else 0.0)))
        wx, wy = qf._to_world(m, vlx, vly)
        zb = _seat_z(wx, wy)
        if zb is None:
            n_dropped[0] += 1
            continue
        path = "{0}/vent_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                         qf._uid(ctx))
        h = rng.uniform(0.7, 1.4)
        qf._cyl(ctx["stage"], path, (wx, wy, zb), (wx, wy, zb + h), 0.16,
                mat_metal)
        ctx["authored"].append(path)
        made.append(path)
        kind[path] = "vent"

    # THE BULKHEAD AND THE PAD ARE BUILDING, NOT PLANT.
    # `quake_flow._b_settle_roof_plant` hands everything in `roof_plant` to
    # the solver at the end of the run, which is right for a tank or a
    # condenser and wrong for a 4 x 3 x 2.6 m stair head cast into the roof:
    # as a rigid body it is a large mass that has to be caught, and it took
    # the settle past its 2200-step cap with five bodies still moving
    # (uf_r2, 2026-08-28). They stay static.
    fixed = [q for q in made if kind.get(q) in ("bulkhead", "pad")]
    plant = [q for q in made if q not in set(fixed)]
    ctx["static_extra"] += fixed
    ctx["roof_fixed"] = fixed
    ctx["roof_plant"] = plant
    ctx["roof_plant_mass"] = mass
    ctx["notes"].append(
        "roof plant: bulkhead on {0}, {1} row(s) of condensers on a pad, "
        "{2} fixed + {3} loose item(s){4}".format(
            side, rows, len(fixed), len(plant),
            ", {0} dropped (no local roof support under them)".format(
                n_dropped[0]) if n_dropped[0] else ""))
    return plant


# ---------------------------------------------------------------------------
# 5b. THE ROOF FROM DIRECTLY ABOVE — the view this dataset is shot from
# ---------------------------------------------------------------------------
def r_roof_scorch(ctx, mass=None):
    """Soot, debris and a black parapet on the roof of a burning building.

    THE FIRST BENCH RUN LOOKED PERFECT FROM THE STREET AND COMPLETELY
    UNTOUCHED FROM DIRECTLY ABOVE — a clean deck inside a bright white
    parapet coping, with a burnt-out shell underneath it (uf_smoke,
    2026-08-28). For a drone dataset that is the failure that matters, and it
    is not fixed by the generic per-subset char: a roof is ONE mesh with one
    or two subsets, so a 25 % coverage roll on it comes out as all or
    nothing, and the parapet is a separate band whose severity the storey
    model reads as "above the fire".

    Three things, all cheap and all authored rather than rolled:

      * the coping and the parapet band on the burning elevations go black —
        it is the closest masonry to a plume leaving the top of the building
        and in every photograph it is the first thing that is;
      * the deck takes the char maps outright wherever the fire reached the
        top storey, not a coverage roll;
      * a scatter of charred debris on the deck: what a roof accumulates in a
        fire is fallen plant, blown ash and pieces of its own covering, and
        from 60 m that scatter is most of what says the roof is not fine.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    mass = mass or f["mass"]
    sev_top = _severity(ctx, f["n_storeys"] - 1, mass)
    if sev_top < 0.15:
        ctx["notes"].append("roof scorch: fire stayed low, roof clean")
        return
    _deck_slab(ctx, mass)
    n_par = n_deck = 0
    # THE PARAPET BELONGS TO THE SKIN. Binding the flat burn maps to a share
    # of the parapet subsets on every reachable side put hard black
    # rectangles along the coping of elevations that never burned ("random
    # parts of the other side are changing their material to a burnt
    # texture", user 2026-08-30); `r_smoke_stain` already bakes the soot
    # skin into the parapet modules' own maps, so the maps stay for the
    # deck, the plant and the debris only.
    parapets = [] if (ctx.get("soot_skin") is not None
                      or ctx.get("soot_prebaked")) else list(
        qf._els(ctx, mass=mass, role=("parapet", "parapet_corner")))
    for e in parapets:
        # REACHABILITY, not a flat 55% chance regardless of which elevation.
        # `_side_reach(..., roof_like=True)` is 1.0 on every side once the
        # fire has reached the top storey (`f["roof"]`) — the roof deck is
        # one continuous plane, so a hot side's wall reaching the coping is
        # itself an unbroken chain into the whole parapet ring — and falls
        # back to the ordinary wall-like side/bleed rule below that, instead
        # of a coin flip that did not care whether this elevation was ever
        # near the fire ("burnt on the other side of the building that
        # isn't even on fire", user review, 2026-08-29).
        reach = _side_reach(ctx, e["side"], roof_like=True)
        if reach <= 0.0:
            continue
        sev = sev_top * reach
        n_par += _bind_subsets(ctx["stage"], e["p"],
                               lambda: _burn_mat(ctx, f["finish"], sev=sev),
                               min(1.0, 0.9 * sev), rng)
    for e in qf._els(ctx, mass=mass, role="roof"):
        n_deck += _bind_subsets(ctx["stage"], e["p"],
                                lambda: _burn_mat(ctx, f["finish"], sev=sev_top),
                                1.0, rng)
    # ONCE A HOLE HAS BEEN CUT, THE DECK IS NOT A KIT ELEMENT ANY MORE.
    # `_roof_box` swaps every kit roof tile for an authored slab and marks the
    # tile dead, so `_els(role="roof")` returns nothing and this pass reported
    # "0 deck subset(s)" on exactly the buildings whose roofs had burned —
    # then skipped the deck debris as well, because `has_deck` was read off
    # the same empty list (uf_r2e, 2026-08-28). The surviving slabs are in
    # `ctx["roof_slabs"]`.
    for pth in ctx.get("roof_slabs", []):
        pr = ctx["stage"].GetPrimAtPath(pth)
        if pr and pr.IsValid() and pr.IsActive():
            qf._b_bind_over(ctx["stage"], pth,
                            _burn_mat(ctx, f["finish"], sev=sev_top))
            n_deck += 1
    # THE TANKS AND AC UNITS BURN WITH IT. `dress_roof` references them from
    # Nucleus with their own leaf bindings — a pale cedar tank and a near-white
    # condenser housing — so on a roof the fire came through they are the two
    # brightest objects in the frame. Bound OVER (`_b_bind_over`), which is
    # what beats a referenced asset's internal bindings.
    # ...AND THE FIXED PLANT, which is the half that was being missed. The
    # bulkhead, its cap and the condenser pad are held OUT of `roof_plant`
    # (they are building, not plant — see `dress_roof_urban`), so the char
    # pass walked straight past them and a burnt roof came back with a bright
    # white stair head and a white pad on it, the two brightest objects in a
    # nadir frame of a black building (uf_r2d, 2026-08-28).
    n_plant = 0
    for pth in list(ctx.get("roof_plant", [])) + list(ctx.get("roof_fixed", [])):
        qf._b_bind_over(ctx["stage"], pth,
                        ctx["mats"]["char_concrete"] if rng.random() < 0.55
                        else ctx["mats"]["burnt_metal"])
        n_plant += 1
    # IF THE DECK WENT, SO DOES WHAT WAS BOLTED TO IT. The plant is laid out
    # before the recipes run (it has to be — a hole cannot be cut round
    # something that is not there yet), so a housekeeping pad can end up
    # spanning the opening the roof recipe then cuts: a row of condensers
    # standing on nothing over a hole (uf_r2d). Once the deck is breached the
    # fixed items go to the solver like everything else and fall through it.
    if ctx.get("roof_breached") and ctx.get("roof_fixed"):
        drop = set(ctx["roof_fixed"])
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q not in drop]
        ctx["roof_plant"] = list(ctx.get("roof_plant", [])) + list(drop)
        ctx["roof_fixed"] = []
    # the debris field on the deck — ONLY IF THERE IS STILL A DECK. `_els`
    # skips what `r_roof_burnthrough` marked dead, so a count of zero live
    # roof pieces means the deck went through; scattering lumps at `m["top"]`
    # anyway left a cloud of charred blocks hanging over an open shell
    # (uf5 dw_terrace, 2026-08-28).
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    has_deck = n_deck > 0
    W, D = m["W"] - 1.6, m["D"] - 1.6
    n_deb = int(40 * sev_top * rng.uniform(0.7, 1.4)) if has_deck else 0
    # ON THE DECK, NOT ON THE PARAPET. `m["top"]` is the parapet coping on a
    # GAC building (`gac_fire.mass_from_grid`'s `deck_z` is the measured
    # roof surface). Read off `ctx["fire"]`, not `m` -- see `dress_roof_
    # urban`'s note: `burn_building`'s own `m` is rebuilt from the sliced
    # pieces and never carries `deck_z`. The kit path's `fire` has no
    # `deck_z` either, so `.get` falls back to `top` unchanged.
    deck_z = ctx["fire"].get("deck_z", m["top"])
    for _ in range(n_deb):
        lx, ly = rng.uniform(-W / 2, W / 2), rng.uniform(-D / 2, D / 2)
        wx, wy = qf._to_world(m, lx, ly)
        sz = 0.12 + 0.55 * rng.random() ** 2.0
        r = rng.random()
        mat = (ctx["mats"]["char_concrete"] if r < 0.45 else
               ctx["mats"]["soot"] if r < 0.75 else ctx["mats"]["ash"])
        path = "{0}/rdeb_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                         qf._uid(ctx))
        qf._a_lump(ctx["stage"], path, wx, wy, deck_z + sz * 0.3, sz, rng,
                   mat, jitter=0.5)
        ctx["authored"].append(path)
    ctx["notes"].append(
        "roof scorch: {0} parapet subset(s), {1} deck subset(s), {2} plant "
        "item(s) charred{3}, {4} piece(s) of debris on the deck".format(
            n_par, n_deck, n_plant,
            " (deck breached: plant dropped to physics)"
            if ctx.get("roof_breached") else "", n_deb))


def _plate(ctx, path, z, thickness, mat, storey):
    """A floor plate clipped to the building's own PLAN at `storey`, in
    place of a `W x D` box spanning the mass's full bounding box.

    THE PLAN IS NOT A CUBOID. `r_expose_interior`'s catch plate and
    `r_fire_collapse`'s heap floor both used to author `qf._box(..., W, D,
    ...)` at a storey level — fine on a rectangular kit module, wrong on a
    merged GAC/downtowncity asset with a setback plan: a full-width plate at
    a storey the building has stepped in from pokes out past the façade
    there (user review, fire_dtc2, 2026-08-30, dtc Building_11 F1: "the
    catch ... looks like it's coming outside the side walls ... it's
    irregular-shape façades + roof; you can't treat it like a cuboid").
    `gac_fire.prepare` measures a convex-hull, inset footprint polygon per
    storey off the merged mesh's own vertices (`_storey_footprints`) and
    hands it through `ctx["fire"]["footprints"]`; this authors an extruded
    polygon from it — fan-triangulated top/bottom caps, a quad side band —
    with the plate's TOP face at `z` and total height `thickness`, exactly
    matching `qf._box`'s own `(cz=z-thickness/2, sz=thickness)` convention.

    THE KIT PATH NEVER SEES A FOOTPRINT. `ctx["fire"]["footprints"]` is a
    key only `gac_fire.prepare` ever sets; a plain kit `burn_building`
    call's `fire` dict (`plan_fire`'s own return) never carries it, so
    `.get("footprints")` always misses there and this falls back to exactly
    the `W x D` box the two call sites authored before — byte-identical
    geometry (same `m["cx"]`/`m["cy"]`/`m["yaw"]`, same `W`/`D` off
    `qf.WALL_INSET`, same `cz`), same call.

    `mass` is read off `ctx["fire"]["mass"]`, not passed in: every LADDER
    entry that reaches either caller passes `{}` (mass is never overridden
    in practice), so this always resolves to the same mass the caller's own
    `mass = mass or f["mass"]` line already did.
    """
    from . import quake_flow as qf

    stage = ctx["stage"]
    f = ctx["fire"]
    mass = f["mass"]
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    cz = z - thickness / 2.0
    poly = (f.get("footprints") or {}).get(storey)
    if not poly or len(poly) < 3:
        W = m["W"] - 2 * qf.WALL_INSET
        D = m["D"] - 2 * qf.WALL_INSET
        return qf._box(stage, path, m["cx"], m["cy"], cz, W, D, thickness,
                      m["yaw"], mat)
    from pxr import Gf, Sdf, UsdGeom, Vt

    n = len(poly)
    hz = thickness / 2.0
    cx, cy = m["cx"], m["cy"]
    # POINTS ABOUT THE PLATE'S OWN CENTRE, exactly like `qf._box` — a rigid
    # body (were this ever handed to physics) rotates about its own origin,
    # not about the mass centre.
    loc = [(px - cx, py - cy) for px, py in poly]
    P = Gf.Vec3f
    top = [P(lx, ly, hz) for lx, ly in loc]
    bot = [P(lx, ly, -hz) for lx, ly in loc]
    pts = top + bot                      # top: 0..n-1, bottom: n..2n-1
    counts, idx, nrm = [], [], []
    # TOP CAP: fan from vertex 0, CCW as authored (viewed from +z) -> +z normal
    for i in range(1, n - 1):
        counts.append(3)
        idx += [0, i, i + 1]
        nrm += [(0.0, 0.0, 1.0)] * 3
    # BOTTOM CAP: same fan with the last two indices swapped -> -z normal
    for i in range(1, n - 1):
        counts.append(3)
        idx += [n, n + i + 1, n + i]
        nrm += [(0.0, 0.0, -1.0)] * 3
    # SIDE BAND: one outward-facing quad per polygon edge
    for i in range(n):
        j = (i + 1) % n
        dx, dy = loc[j][0] - loc[i][0], loc[j][1] - loc[i][1]
        el = math.hypot(dx, dy) or 1.0
        ox, oy = dy / el, -dx / el          # outward normal, CCW polygon
        counts.append(4)
        idx += [n + i, n + j, j, i]
        nrm += [(ox, oy, 0.0)] * 4
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray(pts))
    me.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    me.CreateNormalsAttr(Vt.Vec3fArray([P(*v) for v in nrm]))
    me.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    xs = [p[0] for p in loc]
    ys = [p[1] for p in loc]
    me.CreateExtentAttr([P(min(xs), min(ys), -hz), P(max(xs), max(ys), hz)])
    xf = UsdGeom.Xformable(me)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddRotateZOp().Set(float(m["yaw"]))
    if mat is not None:
        qf._bind(stage, path, mat)
    return path


def r_expose_interior(ctx, mass=None, beams=True, rubble=True):
    """Put STRUCTURE behind every hole, so an opened building is not a box.

    A KIT BUILDING IS A HOLLOW SHELL and this is the single biggest thing
    standing between a burnt-out façade and a burnt-out BUILDING. The
    earthquake pipeline hit it first and says so — `quake_flow.fit_interior`
    exists because "an earthquake EXPOSES interiors: a peeled masonry wall
    shows the floors behind it (the dollhouse)". A fire exposes them harder:
    every window in the band is an empty hole and the roof is open, so the
    inside is on show from the street AND from directly above, which is where
    this dataset is shot from. With only slabs behind it, an opened roof looks
    down onto a bare plate in an empty box (user review, 2026-08-28).

    Three things, in the order they read:

      * BEAMS under every exposed slab. `fit_interior` gives columns to the
        frame types only, so a masonry building had nothing spanning at all.
        A grid of downstand beams is what a floor looks like from underneath
        and it is the first thing visible through a roof hole.
      * PIERS on the masonry types, which `fit_interior` skips by
        construction (`columns and btype != "urm"`); a nineteenth-century
        commercial block has internal cast-iron columns on the same grid.
      * DEBRIS ON THE FLOOR under whatever failed. A slab that has had the
        roof and a storey land on it is not swept clean.

    Everything here is authored INSIDE the walls, so it costs nothing on a
    building whose façade is intact.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    mass = mass or f["mass"]
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    fit = ctx.get("fit") or {}
    stage = ctx["stage"]
    W = m["W"] - 2 * qf.WALL_INSET
    D = m["D"] - 2 * qf.WALL_INSET
    n_beam = n_pier = n_rub = 0
    live = [(k, v) for k, v in (fit.get("slabs") or {}).items()
            if v and k[0] == mass and stage.GetPrimAtPath(v).IsValid()
            and stage.GetPrimAtPath(v).IsActive()]
    urm = ctx["info"]["type"] == "urm"
    pitch = max(4.0, float(m["module"]))
    # BEAMS AND PIERS READ AS STRUCTURE, NOT AS CHARRED MASONRY. An `rc`
    # building's frame is what a burnt façade leaves standing behind it —
    # steel-black, not the same charred concrete as a masonry wall's own
    # mass — and a nineteenth-century masonry block's internal columns are
    # cast iron, steel-black too (user review, 2026-08-30: "rods, probably
    # structural. They looked wooden though — we would want them to look
    # like metal, or even better pillars instead of rods"). Piers are always
    # `steel`; beams are `steel` only on an `rc` frame, `char_concrete`
    # (unchanged) on masonry.
    beam_mat = (ctx["mats"]["steel"] if ctx["info"]["type"] == "rc"
                else ctx["mats"]["char_concrete"])
    for (mtag, storey), slab in live:
        if _severity(ctx, storey, mtag) < 0.25:
            continue
        z = m["levels"][storey]
        if beams:
            # downstands in the SHORT direction (the span a floor is framed
            # across), on the module pitch, plus one spine beam the long way
            bh, bw = 0.45, 0.28
            n_bay = max(2, int(round(W / pitch)))
            for k in range(1, n_bay):
                lx = -W / 2.0 + W * k / float(n_bay)
                wx, wy = qf._to_world(m, lx, 0.0)
                path = "{0}/beam_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                 qf._uid(ctx))
                qf._box(stage, path, wx, wy, z - qf.SLAB_T["rc"] - bh / 2.0,
                        bw, D, bh, m["yaw"], beam_mat)
                ctx["authored"].append(path)
                ctx["static_extra"].append(path)
                n_beam += 1
            path = "{0}/beam_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                             qf._uid(ctx))
            qf._box(stage, path, m["cx"], m["cy"],
                    z - qf.SLAB_T["rc"] - bh / 2.0, W, bw, bh, m["yaw"],
                    beam_mat)
            ctx["authored"].append(path)
            ctx["static_extra"].append(path)
            n_beam += 1
        if urm:
            # `fit_interior` gives columns to the FRAME types only
            # (`columns and btype != "urm"`), so the masonry stock had an
            # open floor plate with nothing holding the slab up.
            h_st = (m["levels"][storey + 1] if storey + 1 < len(m["levels"])
                    else m["top"]) - z
            for a in range(2):
                for b in range(2):
                    lx = -W / 4.0 + a * (W / 2.0)
                    ly = -D / 4.0 + b * (D / 2.0)
                    wx, wy = qf._to_world(m, lx, ly)
                    path = "{0}/pier_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                     qf._uid(ctx))
                    qf._box(stage, path, wx, wy, z + h_st / 2.0, 0.38, 0.38,
                            h_st, m["yaw"], ctx["mats"]["steel"])
                    ctx["authored"].append(path)
                    ctx["static_extra"].append(path)
                    n_pier += 1
        if rubble and _severity(ctx, storey, mtag) > 0.5:
            # what landed on this floor
            for _ in range(int(28 * rng.uniform(0.6, 1.4))):
                lx = rng.uniform(-W / 2 + 0.6, W / 2 - 0.6)
                ly = rng.uniform(-D / 2 + 0.6, D / 2 - 0.6)
                wx, wy = qf._to_world(m, lx, ly)
                sz = 0.14 + 0.55 * rng.random() ** 1.9
                path = "{0}/frub_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                 qf._uid(ctx))
                qf._a_lump(stage, path, wx, wy, z + sz * 0.35, sz, rng,
                           _debris_mat(ctx) if rng.random() < 0.5
                           else ctx["mats"]["char_concrete"], jitter=0.45)
                ctx["authored"].append(path)
                ctx["static_extra"].append(path)
                n_rub += 1
    # A FLOOR UNDER THE HOLE, ALWAYS. Beams and piers are the structure you
    # see THROUGH an opening; they do not stop you seeing all the way to the
    # bottom of the shell when the slab that should be catching the eye has
    # failed. A burnt-out storey has a floor in it covered in what came down —
    # that plate is what makes the building read as a building rather than a
    # box (user review, 2026-08-28: "when we collapse a roof of a hollow
    # building it looks weird to be an empty shell"). If nothing survives one
    # storey under the top of the burnt band, one is authored.
    n_catch = 0
    top_s = min(f["top"] + 1, len(m["levels"]) - 1)
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)) \
            and ctx.get("collapse_s0") is not None:
        # GAC ONLY (kit path frozen): after a fire collapse nothing is a
        # floor at or above the storey that failed — this plate was being
        # authored at the failed storey's level, 3 m over the heap
        # (SM_Building_09 F6, fire_row1, 2026-08-30).
        top_s = min(top_s, int(ctx["collapse_s0"]) - 1)
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # HARD INVARIANT: on a level the building has NOT collapsed, a catch
        # floor may never land at or above the real roof deck. It very
        # nearly always did on a GAC building whose fire band already
        # reaches the building's real top storey (an `origin` request past
        # `n_storeys` clamps there — this building asked for storey 18 on a
        # 16-storey block). `gac_slice.register_style` counts storeys off
        # the RAW `grid_for` grid, one entry more than `gac_fire.
        # mass_from_grid`'s own `levels` (which drops any mark within 0.5 m
        # of the bbox top as the parapet coping, not a floor) — so the
        # rebuilt runtime mass this function reads (`ctx["info"]["masses"]`,
        # built post-slice by `quake_flow._mass_specs`) carries one PHANTOM
        # storey beyond every real floor, sitting at the parapet coping. The
        # `len(m["levels"]) - 1` clamp above is an index bound, not a floor
        # bound, so it happily legalises `top_s` landing on that phantom
        # entry instead of catching the walk-off — one storey pitch above
        # the last real floor and (here) 2.4 m above the measured `deck_z`,
        # with no `fire["footprints"]` entry for it either, so `_plate`
        # fell back to the full W x D box: a slab of "roof debris" sitting
        # on an intact deck (SM_Building_11 F4, o18-of-16, user review
        # 2026-08-31 — "roof seems to have a bunch of debris ... building
        # can't just have debris on its roof for no reason"). Walk back
        # down through real storeys until the candidate is physically below
        # the deck; there is no floor above the real top one to catch on.
        deck_ceiling = ctx["fire"].get("deck_z", m["top"])
        while top_s > 0 and m["levels"][top_s] >= deck_ceiling - 0.05:
            top_s -= 1
    have = {k[1] for k, v in (fit.get("slabs") or {}).items()
            if k[0] == mass and v and stage.GetPrimAtPath(v).IsValid()
            and stage.GetPrimAtPath(v).IsActive()}
    for st_ in (top_s, top_s - 1):
        if st_ <= 0 or st_ in have:
            continue
        z = m["levels"][st_]
        path = "{0}/catch_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                          qf._uid(ctx))
        _plate(ctx, path, z, 0.26, ctx["mats"]["char_concrete"], st_)
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)
        fit.setdefault("slabs", {})[(mass, st_)] = path
        n_catch += 1
        for _ in range(int(34 * rng.uniform(0.7, 1.3))):
            lx = rng.uniform(-W / 2 + 0.6, W / 2 - 0.6)
            ly = rng.uniform(-D / 2 + 0.6, D / 2 - 0.6)
            wx, wy = qf._to_world(m, lx, ly)
            sz = 0.16 + 0.6 * rng.random() ** 1.8
            rp = "{0}/frub_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                           qf._uid(ctx))
            qf._a_lump(stage, rp, wx, wy, z + sz * 0.35, sz, rng,
                       _debris_mat(ctx) if rng.random() < 0.55
                       else ctx["mats"]["char_concrete"], jitter=0.45)
            ctx["authored"].append(rp)
            ctx["static_extra"].append(rp)
            n_rub += 1
        break
    ctx["notes"].append(
        "interior: {0} beam(s), {1} pier(s), {2} catch floor(s), {3} piece(s) "
        "of floor rubble behind the openings".format(n_beam, n_pier, n_catch,
                                                     n_rub))


# ---------------------------------------------------------------------------
# 6. The roof
# ---------------------------------------------------------------------------
def _deck_slab(ctx, mass):
    """Replace this mass's kit roof tiles with ONE slab clipped to its
    footprint, and register it where `quake_flow.r_roof_hole` will find it.

    THE KIT ROOF TILES ARE BIGGER THAN SOME OF THE BUILDINGS. `SM_Roof` is
    23.08 x 23.08 m (`urban_building.PIECES`), and `dw_terrace` is 25 x 15 m —
    so its roof is two of those at x = -0.96 and x = +11.54, overhanging the
    east wall by ELEVEN METRES and the two long walls by four. On an intact
    building nobody notices; char it, cut a hole in it and stand the plant on
    it, and it is a black plate hanging in the sky beside the building with a
    stair bulkhead on the end of it (uf_r2g/uf_r2h, 2026-08-28) — reported
    twice as floating geometry before it was measured and turned out to be
    the layout, not the physics.

    `quake_flow._roof_box` faithfully reproduces the tile's WORLD BBOX, which
    is the right behaviour for its own callers and is what propagates the
    overhang. Authoring the deck from the MASS instead costs one box and
    fixes it for every recipe downstream, because `_a_roofify` skips tiles
    that are already `dead` and reports whatever is in `ctx["roof_slabs"]`.
    """
    from . import quake_flow as qf
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    tiles = [e for e in qf._els(ctx, mass=mass, role="roof")]
    if not tiles:
        return None
    T = 0.16 if ctx["info"]["type"] == "urm" else 0.22
    over = 0.0
    for e in tiles:
        # how far past the wall line does this tile reach, in the mass frame?
        meas = None
        try:
            from detail import urban_building as ub
            meas = ub.PIECES.get(e["name"])
        except Exception:
            pass
        if meas:
            lx, ly = qf._to_local(m, e["x"], e["y"])
            over = max(over, abs(lx) + meas[0] / 2.0 - m["W"] / 2.0,
                       abs(ly) + meas[1] / 2.0 - m["D"] / 2.0)
    # THE SLAB'S TOP SURFACE IS THE REAL DECK, NOT THE PARAPET COPING.
    # `m["top"]` (the bbox top) is the parapet on a GAC building -- often
    # over a metre above the actual roof plateau `gac_fire.mass_from_grid`
    # measures as `deck_z` -- so authoring this box at `top` floated it clear
    # of the walls it should rest on, and every fragment `quake_flow.
    # r_roof_hole` later breaks IT into inherited the same float (35/121
    # unsupported `frag` on SM_Building_23 F4, row-2 review, 2026-08-30).
    # Read off `ctx["fire"]`, not `m` -- `burn_building` rebuilds `m` fresh
    # from the sliced pieces (`quake_flow.describe`) and it never carries
    # `deck_z`; `fire` is the dict `gac_fire.prepare` actually stashes it on
    # (see `dress_roof_urban`'s note). The kit path's `fire` has no
    # `deck_z`, so `.get` is unchanged there.
    path = "{0}/deck_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
    qf._box(ctx["stage"], path, m["cx"], m["cy"],
            ctx["fire"].get("deck_z", m["top"]) - T / 2.0,
            m["W"], m["D"], T, m["yaw"], qf._a_roof_mat(ctx))
    for e in tiles:
        qf._deactivate(ctx["stage"], e["p"].get("prim_path"))
        e["dead"] = True
    ctx["authored"].append(path)
    ctx["static_extra"].append(path)
    ctx.setdefault("roof_slabs", []).append(path)
    ctx["notes"].append(
        "deck: {0} kit roof tile(s) replaced by one {1:.0f} x {2:.0f} m slab "
        "(they overhung the walls by {3:.1f} m)".format(
            len(tiles), m["W"], m["D"], over))
    return path


def r_roof_burnthrough(ctx, frac=0.4, mass=None):
    """A HOLE burnt through the roof — not the roof removed.

    THE WHOLE ROOF IS THE WRONG ANSWER AND IT LOOKED IT. The first version
    fractured a share of the roof TILES, which on `commercial` is one 22 x 18
    m tile: `frac=0.62` therefore took 100 % of the roof and left an open box
    with a bare floor at the bottom of it, seen straight down from the nadir
    camera this dataset is shot from (user review, 2026-08-28 — "when we
    collapse a roof of a hollow building it looks weird to be an empty
    shell"). A burn-through is a HOLE with a deck round it. The deck is what
    makes the hole read.

    `quake_flow.r_roof_hole` already does exactly that, and does it well: a
    wobbly closed outline rather than a rounded box, a rim that SAGS into the
    opening on its own hinge, radiating cracks in the surviving slab, and a
    share of the pieces landing on the floor below rather than on the roof.
    The only thing wrong with it here is the palette — a quake hole is dusty
    grey concrete and a fire hole is black — so this wraps it and rebinds
    everything it authored.

    The mechanism is still different from the quake's and the difference is
    kept: a fire eats through from UNDERNEATH, so the burn-through sits over
    the compartment that was alight rather than wherever the diaphragm was
    weakest, and the bar and purlin ends stand up round it.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    mass = mass or f["mass"]
    if not f.get("roof"):
        ctx["notes"].append("roof: fire never reached it, roof intact")
        return
    _deck_slab(ctx, mass)
    before = set(ctx["loose"]) | set(ctx["authored"])
    # 0.18-0.45 of the plate: over that, the deck stops reading as a deck.
    qf.r_roof_hole(ctx, mass=mass, frac=max(0.12, min(0.45, float(frac))))
    made = [q for q in (list(ctx["loose"]) + list(ctx["authored"]))
            if q not in before]
    n_bind = 0
    for pth in made:
        name = pth.rsplit("/", 1)[-1]
        if name.startswith(("rebar", "sbar")):
            # fire-blackened steel, not the quake palette's rust (which
            # read as timber — user review, 2026-08-30)
            qf._b_bind_over(ctx["stage"], pth, ctx["mats"]["steel"])
            continue
        qf._b_bind_over(ctx["stage"], pth,
                        ctx["mats"]["char_concrete"] if rng.random() < 0.45
                        else _debris_mat(ctx))
        n_bind += 1
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    _rafter_teeth(ctx, m, n=rng.randint(5, 11))
    ctx["roof_breached"] = True
    ctx["notes"].append(
        "roof: burnt through over ~{0:.0f} % of the deck, {1} piece(s) "
        "rebound to char".format(100 * max(0.12, min(0.45, float(frac))),
                                 n_bind))


def _rafter_teeth(ctx, m, n=8):
    """Bar and purlin ends standing up out of a burnt-through roof deck.

    SQUARE-SECTION POSTS, NOT RODS. The old thin, gently-tilted `_cyl`
    (0.06-0.085 m radius, `burnt_metal`) read as a wire, and `burnt_metal`'s
    warm brown-black at this luminance read as scorched timber ("rods,
    probably structural. They looked wooden though", user review,
    2026-08-30). An upright box at the same base — a small random yaw in
    place of the old 3-D tilt — reads as a burnt structural stub, a pillar,
    which is what was asked for; bound to the cooler `steel` tone.
    """
    from . import quake_flow as qf
    rng = ctx["rng"]
    # THE TEETH STAND OUT OF THE REAL DECK, NOT THE PARAPET COPING (see
    # `_deck_slab`'s note: read off `ctx["fire"]`, not `m` -- `burn_building`
    # rebuilds `m` fresh and it never carries `deck_z`). `.get` is unchanged
    # on the kit path, whose `fire` has no `deck_z`.
    z = ctx["fire"].get("deck_z", m["top"])
    W, D = m["W"] - 1.2, m["D"] - 1.2
    for _ in range(n):
        lx = rng.uniform(-W / 2, W / 2)
        ly = rng.uniform(-D / 2, D / 2)
        wx, wy = qf._to_world(m, lx, ly)
        L = rng.uniform(0.5, 1.6)
        s = rng.uniform(0.14, 0.18)
        path = "{0}/rafter_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        # 0.14-0.18 m SQUARE — well clear of the cookable-size floor a thin
        # tube used to sit on. `_cyl`'s 0.06 m radius floor existed because
        # PhysX cooks nothing for a sub-centimetre tube and `settle` reports
        # it as "loose prim(s) NEVER SIMULATED (no cookable mesh under
        # them)" — the piece then stays exactly where authored, which on a
        # roof that has since fallen is a stick hanging in the sky (uf_r2j,
        # 2026-08-28). A square post this size is nowhere near that edge, but
        # it is still STATIC_EXTRA, not LOOSE, for the same reason as before.
        qf._box(ctx["stage"], path, wx, wy, z - 0.4 + L / 2.0, s, s, L,
                rng.uniform(-15.0, 15.0), ctx["mats"]["steel"])
        ctx["authored"].append(path)
        # STATIC_EXTRA, NOT LOOSE. A rafter tooth handed straight to physics
        # cannot stay standing up, and standing up is the whole point of the
        # detail — it is the burnt stub still upright in the wall pocket.
        # `static_extra` keeps it standing when nothing else collapses, and
        # is what `r_fire_collapse`'s position sweep (below) actually reads
        # to hand a piece to the solver when something DOES come down over
        # it — `authored` alone is invisible to that sweep, which is why
        # `rafter_b5_58` rendered floating over a collapsed F5 shell whose
        # roof had already burnt through under it (reported 2026-08-29).
        ctx["static_extra"].append(path)


# ---------------------------------------------------------------------------
# 7. The glass tower
# ---------------------------------------------------------------------------
def r_curtain_burn(ctx, grade=3, width_frac=None):
    """A curtain-wall tower on fire: a VERTICAL STRIPE of failed bays.

    The Grenfell / Address / Torch pattern, and it is completely unlike the
    earthquake's curtain-wall failure. A quake racks a STOREY, so glass is
    lost in a HORIZONTAL BAND across one or two floors and the tower above
    and below is untouched. A fire climbs the spandrel gap, so glass is lost
    in a NARROW VERTICAL COLUMN two or three bays wide running UP from the
    fire floor — and the aluminium mullions melt at 660 C, which no
    earthquake does, so the cage goes with the glass in the hottest part of
    the stripe instead of framing every hole.

    `quake_flow.r_curtain_wall` is deliberately NOT called. It owns the
    measured pane tables and it is the right code for a quake, but its band
    is drawn from a DRIFT profile — on the bench it put the glass loss at
    storeys 26-27 of a tower whose fire was at 17-20 (uf_bench2,
    2026-08-28), i.e. a horizontal band of missing panes floating five
    storeys above the fire. Two glass failures on one tower from two
    unrelated causes read worse than one, so the stripe is authored here.

    THE STRIPE RUNS ON ABOVE THE BAND. Fire in a curtain wall travels up the
    perimeter void faster than it travels through the floor plates, so the
    glass over the top of the involved floors is gone or crazed well before
    those floors are alight. Below the fire floor there is nothing.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    m = ctx["info"]["masses"][f["mass"]]
    side = f["sides"][0]
    span = m["W"] if side in ("S", "N") else m["D"]
    wf = width_frac if width_frac is not None else rng.uniform(0.16, 0.34)
    u0 = rng.uniform(0.10, max(0.11, 0.90 - wf)) * span
    w = wf * span
    lick = 2 + int(round(grade * 0.8))       # storeys the stripe licks above
    n_out = n_stain = 0
    # Candidate bays as (e, fr, width, hh, along, half): kit modules, or —
    # when there are none — measured bays from the openings table, both
    # through ONE authoring body below. Kit rng order is preserved exactly:
    # the same elements reach the same rolls in the same order (frames are
    # pure geometry, and an element whose frame is None still consumes its
    # soft-edge roll first, as it always did).
    cand = []
    for e in qf._els(ctx, mass=f["mass"], role=("wall", "corner")):
        if e["side"] != side or e["storey"] < f["origin"]:
            continue
        if e["storey"] - f["top"] > lick:
            continue
        along = (e["lx"] + m["W"] / 2.0) if side in ("S", "N") else \
                (e["ly"] + m["D"] / 2.0)
        cand.append((e, qf._piece_frame(e), None, None, along, 2.5))
    cb_src = "kit els"

    def _table_cand():
        # PER-BAY CANDIDATES FROM THE MEASURED OPENINGS TABLE — the sliced
        # path's substitute for kit wall modules: a per-bay frame anchored
        # on the measured façade plane lets the shared body author the
        # same voids, melted mullions and crazed panes. Only reachable
        # when `ctx["soot_openings"]` exists, which the kit path never
        # sets.
        from . import soot_plume as spl
        out = []
        for s in range(int(f["origin"]), int(f["top"]) + lick + 1):
            for op in spl.openings(ctx, f["mass"], side, s) or []:
                ua, ub, va, vb = op["span"]
                bw, bh = ub - ua, vb - va
                if bw < 0.6 or bh < 0.8:
                    continue
                f0 = op["fr"]
                bay = (f0[0] + math.cos(f0[2]) * ua,
                       f0[1] + math.sin(f0[2]) * ua,
                       f0[2], bw, bh, f0[5], f0[6])
                e = dict(op["e"])
                e["z"] = float(va)
                out.append((e, bay, bw, bh, 0.5 * (ua + ub), 0.5 * bw))
        return out

    # RESULT-BASED FALLBACK, NOT A SHAPE-BASED GATE. Two prior gates here
    # each missed a sliced variant: `not cand` (sliced styles DO have
    # elements) and `all frames None` (a CACHED kit — FB_BAKED_KITS=1 —
    # resolves SOME frames, and those few die in the stripe filters:
    # "curtain burn: 0 bay(s) out" three bakes running, fire_dtc3 review
    # 2026-08-31, while the same building fresh-sliced tore 20 bays). The
    # only invariant that holds everywhere: if the kit-els pass tears and
    # crazes NOTHING and a measured openings table exists, run the same
    # body again on the table's bays. Kit buildings never set
    # `soot_openings`, so their rng consumption is untouched.
    if not cand and ctx.get("soot_openings") is not None:
        cand, cb_src = _table_cand(), "table"
    def _bay(e, fr, width, hh, along, half):
        nonlocal n_out, n_stain
        # a soft edge to the stripe: a bay half in it is in it half the time
        over = min(along + half, u0 + w) - max(along - half, u0)
        if over <= 0.0:
            return
        if over < 4.0 and rng.random() > over / 4.0:
            return
        if fr is None:
            return
        if width is None:
            width, hh = fr[3], fr[4]
        above = e["storey"] - f["top"]
        gone = above <= 0 or rng.random() < max(0.15, 1.0 - above / float(lick))
        if gone:
            # the bay is OUT: a dark opening inset from the painted mullion
            # grid, so the kit's own cage still frames it
            _face_polygon(ctx, fr, width * 0.5, e["z"] + 0.30,
                          [(-width * 0.42, 0.0), (width * 0.42, 0.0),
                           (width * 0.42, hh - 0.62), (-width * 0.42, hh - 0.62)],
                          ctx["mats"]["void"], out=min(fr[5], -0.02) - 0.06,
                          kind="cwvoid")
            # the melted mullion: a charred bar down each side of a lost bay
            for du in (-width * 0.44, width * 0.44):
                _face_polygon(ctx, fr, width * 0.5 + du, e["z"] + 0.25,
                              [(-0.05, 0.0), (0.05, 0.0),
                               (0.05, hh - 0.5), (-0.05, hh - 0.5)],
                              ctx["mats"]["burnt_metal"], out=0.010,
                              kind="cwmul")
            n_out += 1
        else:
            # STILL GLAZED, AND IT HAS TO SHOW THAT. A curtain-wall bay beside
            # a burning one is the clearest place in the whole scene to read
            # thermally-cracked glass, and a flat grey rectangle there threw
            # it away. Same graded deposit and edge-rooted cracks as a
            # punched window, on a synthetic opening record for this bay.
            _glass_pane(ctx, {"fr": fr, "out": -0.03, "e": e},
                        width * 0.08, width * 0.92,
                        e["z"] + 0.30, e["z"] + hh - 0.32,
                        _severity(ctx, e["storey"], e["mass"], e),
                        out=0.014)
            n_stain += 1

    for c in cand:
        _bay(*c)
    if (n_out + n_stain) == 0 and cb_src == "kit els" \
            and ctx.get("soot_openings") is not None:
        # the result-based fallback — see the note above `_table_cand`
        cand, cb_src = _table_cand(), "table fallback"
        for c in cand:
            _bay(*c)
    # THE GLASS IS ON THE STREET. A tower that has lost eighty bays has put
    # every one of them on the pavement below, and that fall zone is the only
    # thing at ground level that says the fire is forty metres up.
    nx, ny = qf._outward(m, side)
    half = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
    n_dice = int(min(400, 22 * max(1, n_out)))
    for _ in range(n_dice):
        t = u0 - span / 2.0 + rng.uniform(-1.5, w + 1.5)
        # glass from height scatters WIDE — a pane off storey 20 lands 10-25 m
        # out, not at the wall foot
        d = half + abs(rng.gauss(0.0, 7.0)) + 1.0
        if side in ("S", "N"):
            lx, ly = t, math.copysign(d, ny or 1.0)
        else:
            lx, ly = math.copysign(d, nx or 1.0), t
        wx, wy = qf._to_world(m, lx, ly)
        sz = rng.uniform(0.05, 0.26)
        path = "{0}/cwglass_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                            qf._uid(ctx))
        qf._a_lump(ctx["stage"], path, wx, wy, m["z0"] + sz * 0.15, sz, rng,
                   ctx["mats"]["fire_glass"] if rng.random() < 0.8
                   else ctx["mats"]["burnt_metal"], jitter=0.6)
        ctx["authored"].append(path)
    ctx["notes"].append(
        "curtain burn: {0} bay(s) out, {1} crazed, in a {2:.0f} m stripe on "
        "{3} from storey {4} up; {5} piece(s) in the fall zone; {6} "
        "candidate bay(s) from {7}".format(
            n_out, n_stain, w, side, f["origin"], n_dice, len(cand), cb_src))


# ---------------------------------------------------------------------------
# 8. Collapse — the rare one
# ---------------------------------------------------------------------------
def r_fire_collapse(ctx, mass=None):
    """The burnt-out shell loses its top storeys / one elevation.

    RARE AND PARTIAL, ALWAYS. A fire collapse is not a quake collapse: the
    building has been standing empty and black for hours before anything
    comes down, so what fails is the burnt-out TOP of it, dropping into the
    storeys below, while the lower frame — never heated — stands. A heap at
    grade with the whole block gone is an earthquake and reads as one.

    The heap is CHARRED, not dusty. `quake_flow._heap`'s default mix is
    mortar dust over brick, which is exactly right for a quake and exactly
    wrong here: fire rubble has no dust plume, it is black and wet.
    """
    from . import quake_flow as qf

    f, rng, nrng = ctx["fire"], ctx["rng"], ctx["nrng"]
    mass = mass or f["mass"]
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    n_lv = len(m["levels"])
    # ONE STOREY, TWO AT THE OUTSIDE. The first cut took the top 35 % of the
    # block, which on a ten-storey building is three or four floors and reads
    # as a demolition. A fire collapse is a local failure of the burnt-out
    # top: Windsor, Plasco and the Wilton Paes de Almeida all lost the upper
    # storeys onto the floors below with the lower frame standing (user
    # review, 2026-08-28: "it prob won't collapse more than a floor or 2").
    n_fall = 1 if rng.random() < 0.62 else 2
    s0 = max(f["origin"] + 1, n_lv - n_fall)
    if s0 >= n_lv:
        s0 = max(f["origin"], n_lv - 1)
    # GAC ONLY, PER THE KIT-FIRE FREEZE (2026-08-30: "nothing may alter
    # kit-building output"). `ctx["soot_prebaked"]` is a set/frozenset only
    # when `gac_fire.burn_gac` set it (a plain kit `burn_building` call never
    # passes `soot_prebaked=`, so it is `False` there) — everything below
    # that records where the wreckage lands or rebinds a fallen roof piece to
    # the fire palette is gated on this, so the frozen kit look draws nothing
    # new and takes no new code path.
    is_gac = isinstance(ctx.get("soot_prebaked"), (set, frozenset))
    _roof_fall_mat = None
    if is_gac:
        # THE STOREY THE WRECKAGE ACTUALLY LANDS ON. `r_gut_interior` chars a
        # fit-out slab by `_severity`, which falls off with distance from the
        # fire's origin — wrong for the one floor everything above `s0` is
        # about to be dropped onto, which is black with soot and debris
        # however far that floor is from the seat of the fire. Recorded here
        # so `r_gut_interior` (which runs later in every ladder that also
        # carries this recipe) can force it, rather than guess from
        # `fire["roof"]` alone.
        ctx.setdefault("collapse_storeys", set()).add(max(0, s0 - 1))
        # A LOCAL GENERATOR FOR EVERYTHING THIS RECIPE DROPS FROM THE ROOF.
        # `_debris_mat` draws on the SHARED `ctx["rng"]`, and how many times
        # it would be called here depends on how much roof geometry this
        # particular building happens to carry (a GAC slice's roof-role kit
        # pieces, a `_roof_box` slab, a `_split_strip` remainder, the deck, a
        # shattered lid) — drawing material picks from the shared stream
        # would move every later recipe's outcome by however many fragments
        # a piece of geometry happened to produce. Seeded off the building
        # itself instead, exactly like the lid-shatter draws further down
        # this function already are.
        from . import soot_plume as _spl
        _lid_seed = _spl.event_seed(ctx) ^ 0xB0F
        lrng = random.Random(_lid_seed)

        def _roof_fall_mat():
            """char_concrete ~30% of the time, char/scorch debris the rest —
            `_debris_mat`'s own split, off `lrng` rather than the shared
            `rng` so nothing this recipe's own geometry does can perturb a
            later recipe's draws."""
            if lrng.random() < 0.30:
                return ctx["mats"]["char_concrete"]
            burn = ctx["mats"].get("_burn")
            if not burn:
                return ctx["mats"]["soot"]
            key = "char" if lrng.random() < 0.72 else "scorch"
            return burn[key][lrng.randrange(len(burn[key]))]

    n_broke = n_art = 0
    for e in list(qf._els(ctx, mass=mass, role=("wall", "corner", "parapet",
                                                "parapet_corner"))):
        if e["storey"] < s0:
            continue
        st, lo = qf._break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                           rng.randint(6, 11), rng, nrng, ctx["mats"],
                           ctx["cache"], ctx["info"]["type"],
                           inner_p=0.4, rough=0.014, consume=0.30,
                           style=ctx["info"]["style"])
        # EVERY fragment, not three quarters of them. `_break` binds the
        # module's own cladding to whatever it does not repaint, so a quarter
        # of a burnt-out terrace's rubble came out as clean tan stonework
        # lying in the street — the brightest thing in the frame and the one
        # that says "demolition site" rather than "fire".
        # `_b_bind_over`, NOT `_bind`. `quake_flow._break` binds the module's
        # own cladding on the prim AND a core material on the broken-face
        # subsets (`_t_core_bind`); an ordinary prim-level bind is
        # weakerThanDescendants, so those subsets win and a third of a burnt
        # terrace's rubble stayed clean tan stonework however dark the
        # material handed in was (uf4, 2026-08-28). A fire chars the broken
        # face too — there is nothing here that should still show masonry.
        # ...WITH ONE EXCEPTION, GAC ONLY (the kit path is frozen): a piece
        # that stays STATIC has not moved, so its outward faces are still the
        # façade the module next door shows, and charring them whole is the
        # hard dark rectangle of the second-row review ("the material of the
        # broken/debris part is a much darker colour than the intact façade
        # next to it", 2026-08-30). `fire_collapse.bind_break(cut_only=True)`
        # puts the char on the `core` subset — the faces the fracture
        # invented — and leaves the cladding/sooted atlas on the rest.
        # `_break` is called with `partial=None` here, so it returns no
        # statics today and this is the rule written where it belongs rather
        # than a live branch; the loose fragments are in the heap and take
        # the dark end whole, which is where it belongs.
        for pth in lo:
            qf._b_bind_over(ctx["stage"], pth, _debris_mat(ctx))
        for pth in st:
            if is_gac:
                _fire_collapse.bind_break(ctx, pth, _debris_mat(ctx),
                                          cut_only=True)
            else:
                qf._b_bind_over(ctx["stage"], pth, _debris_mat(ctx))
        ctx["loose"] += lo
        ctx["static_extra"] += st
        n_art += _drop_face_art(ctx, e)
        e["dead"] = True
        n_broke += 1
    # EVERYTHING OVER THE FAILURE COMES DOWN WITH IT, and the two that are
    # easy to forget are the ones that float most visibly. The ROOF is a
    # separate element role, so the wall sweep above misses it and a grey
    # plate is left 20 m over an open shell (`r_overturn` learned this the
    # same way). The FIT-OUT SLABS are authored boxes, not kit elements, so
    # `_els` cannot see them at all — leave them and a burnt-out top storey
    # keeps a clean concrete floor hanging in mid-air.
    n_roof = 0
    for e in list(qf._els(ctx, mass=mass, role="roof")):
        made = qf._break_box_like(ctx, e, rng.randint(8, 13), timber=False,
                                  consume=0.3, consume_pool=1.05)
        if is_gac:
            # `_b_bind_over`, NOT `_bind` — same reason as the wall break
            # above: a role="roof" piece can be a sliced GAC kit piece with
            # its own per-material GeomSubsets, and `_roof_box`/
            # `fracture_prim` fragments inherit whatever binding the piece
            # already carried where this loop does not repaint them itself.
            # Fire palette, not the roof's own (pale) texture ("the fallen
            # roof pieces... look unburnt", user review, 2026-08-30).
            for pth in made:
                qf._b_bind_over(ctx["stage"], pth, _roof_fall_mat())
        else:
            # KIT PATH — UNCHANGED (frozen 2026-08-30).
            for pth in made:
                qf._bind(ctx["stage"], pth, _debris_mat(ctx))
        ctx["loose"] += made
        e["dead"] = True
        n_roof += len(made)
    # ...AND EVERYTHING ELSE ABOVE THE FAILURE, SWEPT BY GEOMETRY.
    # Bookkeeping did not work here and it took three rounds to accept that.
    # The roof is not kit elements once a recipe has touched it — it is
    # `_roof_box` slabs, then `_split_strip` REMAINDERS cut out of those, then
    # the deck slab this module authors, each registered in a different list
    # or in none — so a sweep that walks any one of them misses the others and
    # a black plate with a stair bulkhead on it is left hanging six metres
    # above a collapsed storey (uf_r2g, uf_r2h, uf_r2i dw_terrace).
    # A POSITION TEST CANNOT MISS A LIST. Everything static this building
    # authored, whose centre sits above the failure line, is handed to the
    # solver.
    from pxr import Usd, UsdGeom
    cut = m["levels"][s0] - 0.4
    xf = UsdGeom.XformCache()
    keep, moved = [], []
    for pth in list(ctx["static_extra"]):
        pr = ctx["stage"].GetPrimAtPath(pth) if pth else None
        if not pr or not pr.IsValid() or not pr.IsActive():
            keep.append(pth)
            continue
        try:
            z = xf.GetLocalToWorldTransform(pr).ExtractTranslation()[2]
        except Exception:
            keep.append(pth)
            continue
        if z > cut:
            moved.append(pth)
        else:
            keep.append(pth)
    ctx["static_extra"] = keep
    for pth in moved:
        if pth not in ctx["loose"]:
            ctx["loose"].append(pth)
            n_roof += 1
        if is_gac:
            # BOUND HERE, NOT LEFT FOR THE LID CHECK BELOW — GAC ONLY. Most
            # of what this sweep catches never qualifies as a "lid" there —
            # a GAC `roof_*` piece keeps its own GeomSubsets and is
            # deliberately never handed to `fracture_prim` (see the comment
            # on the positive list below), and a small `_roof_box`/
            # `_split_strip` remainder is under `LID_AREA_M2` — and was
            # falling to the storey below still wearing whatever it was
            # bound to before the collapse (the roof's own pale texture, on
            # a GAC piece): "the inside floor of the collapsed roof looks
            # unburnt" (user review, 2026-08-30). Whatever DOES get
            # shattered as a lid just below is rebound again over its own
            # fragments; rebinding the whole piece here first costs nothing.
            # KIT PATH — no bind here at all (frozen 2026-08-30).
            qf._b_bind_over(ctx["stage"], pth, _roof_fall_mat())
    # A LID, NOT A FALL. What the sweep above just moved includes near-flat
    # slabs close to the MASS'S OWN FOOTPRINT — `_deck_slab` authors the deck
    # at exactly (W, D), and `r_roof_hole`'s `_break_split` can leave most
    # of it as ONE static rim remainder that nothing renames into
    # `roof_slabs` (only this sweep ever sees it). Handed to the solver as
    # one rigid body, a slab that size falls the ~3 m to storey s0-1 and
    # finds a stable rest on THAT storey's own walls — never touched by the
    # loop above (it is below `s0`) and already reduced by `window_burnout`
    # to piers between empty openings: "the roof is floating ... some beams
    # holding it up" (dw_terrace F5, user 2026-08-30; `fit_interior` authors
    # no columns for urm, so those "beams" ARE the piers). Anything that
    # wide is shattered into pieces small enough to drop between the piers
    # instead of bridging them — the treatment `_break_box_like` already
    # gives an un-holed kit roof tile. NO draw on the shared `rng`: every
    # piece count, jitter and material pick comes from a generator seeded
    # off the building itself, so no later recipe's outcome moves.
    from . import damage as _damage, fracture as _fracture, soot_plume as _spl
    LID_AREA_M2 = 20.0
    LID_MAX_FACES = 600
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    fixed_now = set(ctx.get("roof_fixed") or ())
    lids = []
    for pth in moved:
        if pth in fixed_now:
            continue
        pr = ctx["stage"].GetPrimAtPath(pth)
        if not pr or not pr.IsValid():
            continue
        # AUTHORED BOXES ONLY. The lid this exists for is `_deck_slab`'s deck
        # or `r_roof_hole`'s rim remainder — closed, low-poly boxes this file
        # authored. A SLICED whole-asset piece that the sweep also moves (a
        # GAC `roof_*` piece: a clipped open shell with per-material
        # GeomSubsets and thousands of triangles) is not a lid and is not
        # safe to hand to `fracture_prim`: it segfaulted inside
        # vtkClipPolyData on SM_Building_09 F6 (gac_fire bench, 2026-08-30,
        # `_vtk_slice` <- `slice_plane` <- `fracture_mesh`). Such a piece
        # drops whole, as every piece did before the lid fix.
        # POSITIVE LIST: the deck (`_deck_slab`), a `_roof_box` slab, or a
        # `_break_split` remainder — and nothing else, however big its
        # footprint. A referenced roof-plant asset (an Xform, not a Mesh, so
        # the mesh tests below never ran on it) with a 20 m2 condenser pad
        # went to `fracture_prim` on SM_Building_09 F6 and segfaulted VTK
        # again after the mesh guard alone was in (2026-08-30).
        if not pr.GetName().startswith(("deck_", "roofslab_")):
            continue          # `frag_*` remainders are clipped shells: VTK
                              # segfaulted on one (SM_Building_09 F6)
        if not pr.IsA(UsdGeom.Mesh):
            continue
        if UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)):
            continue
        cnt = UsdGeom.Mesh(pr).GetFaceVertexCountsAttr().Get()
        if cnt is None or len(cnt) == 0 or len(cnt) > LID_MAX_FACES:
            continue
        box = bc.ComputeWorldBound(pr).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        lo_pt, hi_pt = box.GetMin(), box.GetMax()
        area = (hi_pt[0] - lo_pt[0]) * (hi_pt[1] - lo_pt[1])
        if area > LID_AREA_M2:
            lids.append((pth, float(area)))
    n_lid = 0
    if lids:
        import numpy as _np
        if is_gac:
            # `fracture_prim` wants a numpy Generator (`rng.permutation`);
            # the SAME seed as `lrng` above (seeded off the building itself,
            # not the shared `rng` — see the comment where `lrng` is made)
            # so the two stay a matched pair.
            lnrng = _np.random.default_rng(_lid_seed)
        else:
            # KIT PATH — UNCHANGED (frozen 2026-08-30). `fracture_prim`
            # wants a numpy Generator (`rng.permutation`), the material
            # picks a `random.Random`; both seeded off the building.
            seed = _spl.event_seed(ctx) ^ 0xB0F
            lrng = random.Random(seed)
            lnrng = _np.random.default_rng(seed)
            burn = ctx["mats"].get("_burn")
        for pth, area in lids:
            n_pc = max(8, min(40, int(area / 6.0)))
            print("[urban_fire] fire collapse: shattering lid {0} ({1:.0f} m2) "
                  "into {2}".format(pth.rsplit("/", 1)[-1], area, n_pc))
            made = _fracture.fracture_prim(
                ctx["stage"], pth,
                "{0}/lidbrk_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                            qf._uid(ctx)),
                n_pieces=n_pc, rng=lnrng, mode="plank", aspect=(1.4, 2.8),
                rough=0.01, consume=0.3, consume_pool=1.05,
                min_volume_frac=0.0008, verbose=False)
            if not made:
                continue
            if is_gac:
                for p2 in made:
                    qf._b_bind_over(ctx["stage"], p2, _roof_fall_mat())
            else:
                for p2 in made:
                    mat = (_damage._pick(lrng, "char" if lrng.random() < 0.72
                                         else "scorch", burn)
                           if burn else ctx["mats"]["soot"])
                    qf._bind(ctx["stage"], p2, mat)
            ctx["loose"] = [q for q in ctx["loose"] if q != pth] + list(made)
            n_roof += len(made) - 1
            n_lid += 1
    if ctx.get("roof_fixed"):
        drop = set(ctx["roof_fixed"])
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q not in drop]
        ctx["roof_plant"] = list(ctx.get("roof_plant", [])) + list(drop)
        ctx["roof_fixed"] = []
    ctx["roof_breached"] = True
    for (mtag, storey), slab in list((ctx.get("fit") or {}).get("slabs", {}).items()):
        # `slab` CAN BE None. `quake_flow.r_roof_hole` breaks the top floor
        # under the opening and clears that entry, so the map holds a live
        # key with no path — and `GetPrimAtPath(None)` is a
        # `Boost.Python.ArgumentError`, not a quiet miss.
        if not slab or mtag != mass or storey < s0:
            continue
        pr = ctx["stage"].GetPrimAtPath(slab)
        if pr and pr.IsValid() and pr.IsActive():
            ctx["loose"].append(slab)
    # SAME ARGUMENT, FOR THE FURNITURE ON THOSE FLOORS. The slab loop above
    # sends the floor itself down; `quake_flow.fit_interior` seats
    # `props[(mtag, storey)]` right on top of `slabs[(mtag, storey)]`, so a
    # prop at or above `s0` that `r_gut_interior` charred rather than
    # deactivating is left standing on a plate that has just been added to
    # `loose` and will fall out from under it — unless the prop goes down
    # too. `ctx["fit"]["all"]` has not been folded into `static_extra` yet at
    # this point in `burn_building` (that happens once, after every recipe
    # has run), so nothing else in this file is going to catch these.
    n_fitprop = 0
    for (mtag, storey), props in list((ctx.get("fit") or {}).get("props", {}).items()):
        if mtag != mass or storey < s0:
            continue
        for pp in props:
            pr = ctx["stage"].GetPrimAtPath(pp)
            if pr and pr.IsValid() and pr.IsActive() and pp not in ctx["loose"]:
                ctx["loose"].append(pp)
                n_fitprop += 1
    # the heap the top landed in: sitting on the floor BELOW the failure, not
    # on the ground, because that is where a fire collapse stops
    base = m["levels"][max(0, s0 - 1)] if s0 > 0 else m["z0"]
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # A FLOOR UNDER THE HEAP, ALWAYS — GAC ONLY (the kit path is frozen).
        # A sliced GAC building is fitted out on its top storeys only, and
        # the slabs of the storeys that fail go down with them, so the storey
        # the heap "lands on" can have no floor at all: SM_Building_09 F6
        # baked 6,186 heap chips at z 44-49 m over nothing, with the
        # `r_expose_interior` catch plate 3 m ABOVE the failure (fire_row1,
        # 2026-08-30 — "lots of floating debris"). The debris stays; it gets
        # its floor. And anything registered as a floor at or above the
        # failure is no floor any more: it is sent down, not kept static.
        fit = ctx.setdefault("fit", {})
        slabs = fit.setdefault("slabs", {})
        bs = max(0, s0 - 1)
        ctx["collapse_s0"] = int(s0)      # `r_expose_interior` reads it
        # THE FRAME FALLS WITH THE FLOORS. `fit_interior`'s rc column grid on
        # the failed storeys stayed static after the walls and slabs went,
        # so a 4 m-pitch forest of three-storey columns stood over the shell
        # with the catch floor and the heap on it — a floating platform
        # (dtc Carved_13 F5, fire_dtc1, 2026-08-30). Columns and beams of
        # storeys at or above the failure are handed to physics.
        # ...AND MOST OF IT IS CONSUMED. Handing every column to physics
        # kept the forest: a 3 m box dropped one metre lands base-down and
        # stays standing, and the lid plates then rest ON the posts — the
        # same floating platform, one metre lower (dtc Carved_13 F5,
        # fire_dtc2, 2026-08-30). A collapse that takes the walls and slabs
        # grinds most of the frame into the heap with them: ~70 % of the
        # failed storeys' columns are simply gone, the rest go down with an
        # outward shove so they topple instead of landing on their feet.
        from . import soot_plume as _splc
        _crng = random.Random(_splc.event_seed({"info": ctx["info"],
                                                "fire": ctx["fire"],
                                                "tag": ctx["tag"]}) ^ 0xC01F)
        n_colfall = n_colgone = 0
        vel = ctx.setdefault("velocity", {})
        for (mt_, st_), cols in list((fit.get("columns") or {}).items()):
            if mt_ != mass or st_ < s0:
                continue
            for cpath in cols:
                pr_ = ctx["stage"].GetPrimAtPath(cpath)
                if not (pr_ and pr_.IsValid() and pr_.IsActive()) \
                        or cpath in ctx["loose"]:
                    continue
                ctx["static_extra"] = [q for q in ctx["static_extra"] if q != cpath]
                if _crng.random() < 0.70:
                    pr_.SetActive(False)
                    n_colgone += 1
                    continue
                a_ = _crng.uniform(0.0, 2.0 * math.pi)
                vel[cpath] = (2.2 * math.cos(a_), 2.2 * math.sin(a_), 0.0)
                ctx["loose"].append(cpath)
                n_colfall += 1
        if n_colfall or n_colgone:
            ctx["notes"].append("fire collapse: frame columns on the failed "
                                "storeys: {0} consumed, {1} toppled"
                                .format(n_colgone, n_colfall))
        # AND THE PARTITIONS FALL WITH THE FRAME — SAME TREATMENT, SAME
        # REASON, RIGHT ABOVE. `fit_interior`'s partitions are conspicuously
        # absent from every "storey at or above `s0` goes down/away" loop in
        # this function (slabs, columns, props, all above) — and unlike a
        # SURVIVING partition, `r_gut_interior` never relocates one either
        # (that function only chars a partition in place or deactivates it
        # outright; see its own docstring) — so a partition `fit_interior`
        # stood on a storey THIS collapse then took away is left exactly
        # where it was, now poking out past the shell's own new, shorter
        # roofline. MEASURED on a bare-USD (no Kit, no physics) re-
        # authoring of `aec:Reference_Brownstone10Row` F5 (default origin,
        # seed 7): 3 of 6 checked fit-out prims — every one a partition —
        # sat up to 5.38 m above the collapsed shell's own top
        # (`tools/aec_overshoot_detail_probe.py`, `tools/
        # aec_final_census.py`) — the "floor and pillars extend out of the
        # walls and roof" defect (user, reviewing the AEC brownstone bakes)
        # reproducing at AUTHORING TIME, no settle involved, on any sliced
        # (GAC/AEC) building whose fire ladder reaches `fire_collapse` with
        # a fitted-out storey at or above the failure line — this whole
        # block is gated on `soot_prebaked` being a set/frozenset, "GAC
        # ONLY (the kit path is frozen)" per the comment above it, so the
        # kit path is untouched by this addition, same as everything else
        # here. Same 70/30
        # consumed/toppled split, same `_crng` stream as the columns loop
        # just above: a partition on a failed storey is the same kind of
        # debris to a falling floor a column is.
        n_partfall = n_partgone = 0
        for pth in list(fit.get("partitions") or []):
            st_, mt_ = _storey_of_path(ctx, pth)
            if mt_ != mass or st_ is None or st_ < s0 or pth in ctx["loose"]:
                continue
            pr_ = ctx["stage"].GetPrimAtPath(pth)
            if not (pr_ and pr_.IsValid() and pr_.IsActive()):
                continue
            if _crng.random() < 0.70:
                pr_.SetActive(False)
                n_partgone += 1
                continue
            a_ = _crng.uniform(0.0, 2.0 * math.pi)
            vel[pth] = (2.2 * math.cos(a_), 2.2 * math.sin(a_), 0.0)
            ctx["loose"].append(pth)
            n_partfall += 1
        if n_partfall or n_partgone:
            ctx["notes"].append("fire collapse: partitions on the failed "
                                "storeys: {0} consumed, {1} toppled"
                                .format(n_partgone, n_partfall))
        for (mt_, st_), pth in list(slabs.items()):
            if mt_ != mass or st_ < s0 or not pth:
                continue
            if pth in ctx["static_extra"]:
                ctx["static_extra"] = [q for q in ctx["static_extra"] if q != pth]
                if pth not in ctx["loose"]:
                    ctx["loose"].append(pth)
        have = slabs.get((mass, bs))
        pr_ = ctx["stage"].GetPrimAtPath(have) if have else None
        if bs > 0 and not (pr_ and pr_.IsValid() and pr_.IsActive()
                           and have not in ctx["loose"]):
            path = "{0}/catch_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                              qf._uid(ctx))
            _plate(ctx, path, base, 0.26, ctx["mats"]["char_concrete"], bs)
            ctx["authored"].append(path)
            ctx["static_extra"].append(path)
            slabs[(mass, bs)] = path
            ctx["notes"].append("fire collapse: floor authored under the heap "
                                "at storey {0} (z={1:.1f})".format(bs, base))
    H = max(3.0, m["top"] - m["z0"])
    # NO DUST IN THE MIX. `_heap`'s own `HEAP_MIX` is mortar dust over pale
    # brick, which is exactly right for a quake — the dust plume is the
    # signature — and exactly wrong here: fire rubble is black, wet from the
    # hose, and has no fines standing on it. `brick_dusty` (0.38 on screen)
    # was the pale tan litter in the terrace shot.
    _before_heap = set(str(p.GetPath()) for p in
                       ctx["stage"].GetPrimAtPath(ctx["parent"]).GetChildren())
    qf._heap(ctx, m, base, (m["top"] - base) * 0.30, 0.10, fill=True,
             tag="fireheap",
             mat_fn=lambda: (ctx["mats"]["char_concrete"] if rng.random() < 0.45
                             else ctx["mats"]["soot"] if rng.random() < 0.80
                             else ctx["mats"]["calcined"]))
    if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
        # A HEAP CHIP OUTSIDE THE WALL LINE AT STOREY HEIGHT HANGS IN THE
        # SKY — GAC ONLY. `_heap` spreads its chips a little past the plan
        # and the fire heap sits on an upper floor, so the overhang had
        # nothing under it: the black flecks beside SM_Building_09 F6 at
        # 49.6 m (fire_row3, 2026-08-30 — 58 of them in the export). Chips
        # whose centre falls outside the floor plate are dropped here;
        # inside the plate they rest on the floor `r_fire_collapse` authors.
        from pxr import UsdGeom as _UG
        W_, D_ = m["W"] - 2 * qf.WALL_INSET, m["D"] - 2 * qf.WALL_INSET
        _xf = _UG.XformCache()
        n_clip = 0
        for p in ctx["stage"].GetPrimAtPath(ctx["parent"]).GetChildren():
            nm = p.GetName()
            if str(p.GetPath()) in _before_heap or not nm.startswith("fireheap"):
                continue
            t = _xf.GetLocalToWorldTransform(p).ExtractTranslation()
            lx, ly = qf._to_local(m, float(t[0]), float(t[1]))
            if abs(lx) > W_ / 2.0 - 0.15 or abs(ly) > D_ / 2.0 - 0.15:
                p.SetActive(False)
                n_clip += 1
        if n_clip:
            ctx["notes"].append("fire collapse: {0} heap chip(s) outside the "
                                "floor plate dropped".format(n_clip))
    ctx["notes"].append(
        "fire collapse: top {0} storey(s) down from storey {1}, {2} module(s) "
        "broken, {3} roof piece(s), {4} stain(s) removed with them, {5} "
        "fit-out prop(s) sent down too, {7} footprint-sized lid(s) shattered "
        "before the drop, heap on the storey below at z={6:.1f}".format(
            n_lv - s0, s0, n_broke, n_roof, n_art, n_fitprop, base, n_lid))


# ---------------------------------------------------------------------------
# 9. The street
# ---------------------------------------------------------------------------
def r_street_debris(ctx, density=1.0):
    """Glass, charred cladding and fallen render on the sidewalk under the fire.

    Small, and only under the elevations that are actually alight — a ring of
    debris all the way round a building is a bomb, not a fire. It is what
    stops the pavement reading as untouched under a burning façade.
    """
    from . import quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    # THE DEBRIS IS UNDER THE FIRE, NOT UNDER THE TALL BIT. If the fire is in
    # a tower standing on a podium, what falls lands on the PODIUM roof, not
    # in the street — so the apron is drawn round the mass whose foot is at
    # ground level, at that mass's own base height.
    m = ctx["info"]["masses"]["main"]
    # MORE OF IT, AND SPREAD WIDER, UNDER A SLICED BUILDING (fire_dtc3
    # review, 2026-08-30: the user wants noticeably more fallen debris round
    # the burning ones). A GAC/downtowncity elevation is 15-30 m wide and
    # 70 m tall against a kit module's ~4 m, so 24 lumps in a 4.5 m apron
    # read as a sprinkle at review distance rather than as a fall zone.
    # The kit path is BYTE-IDENTICAL — it keeps the original (24, 4.5, 0.5),
    # so every draw below happens in the same order with the same arguments
    # and `_a_lump` consumes the same stream after them; the
    # `rng.uniform(0.7, 1.3)` is drawn either way — because the MCE look is
    # frozen.
    n_deb, apron, run = (
        (50, 6.5, 0.58)
        if isinstance(ctx.get("soot_prebaked"), (set, frozenset))
        else (24, 4.5, 0.5))
    n = 0
    for side in f["sides"]:
        nx, ny = qf._outward(m, side)
        span = m["W"] if side in ("S", "N") else m["D"]
        half = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
        count = int(n_deb * density * rng.uniform(0.7, 1.3))
        for _ in range(count):
            t = rng.uniform(-run, run) * span
            d = half + rng.uniform(0.4, apron)
            if side in ("S", "N"):
                lx, ly = t, math.copysign(d, ny or 1.0)
            else:
                lx, ly = math.copysign(d, nx or 1.0), t
            wx, wy = qf._to_world(m, lx, ly)
            s = rng.uniform(0.07, 0.34)
            r = rng.random()
            mat = (ctx["mats"]["fire_glass"] if r < 0.42 else
                   ctx["mats"]["char_concrete"] if r < 0.7 else
                   ctx["mats"]["soot_light"])
            path = "{0}/sdeb_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                             qf._uid(ctx))
            qf._a_lump(ctx["stage"], path, wx, wy, m["z0"] + s * 0.18, s, rng,
                       mat, jitter=0.45)
            ctx["authored"].append(path)
            n += 1
    ctx["notes"].append("street: {0} piece(s) of debris on {1}".format(
        n, "/".join(f["sides"])))


# ---------------------------------------------------------------------------
# 10. The fire itself — NVIDIA Flow
# ---------------------------------------------------------------------------
# EMITTERS GO AT THE OPENINGS, NOT IN THE MIDDLE OF THE BUILDING.
# `fire.add_structure_fire` scatters emitters over a house's FLOOR PLATE,
# which is right for a timber house burning from the inside out and wrong
# here: a concrete or masonry block confines the fire completely, and every
# bit of flame and smoke a camera can see is coming out of a hole. So the
# emitter positions come from the same measured opening tables the plumes do,
# pushed out through the reveal, with an outward-and-up velocity so the plume
# licks the façade instead of rising through the floor above.
def _wall_vents(ctx, n):
    """Opening-shaped records made from the WALL modules of the burning band.

    For a family with no measured glazing table. Same record shape
    `_g2_openings` returns, so `r_flames` cannot tell the difference."""
    from . import quake_flow as qf
    f, rng = ctx["fire"], ctx["rng"]
    out = []
    for e in qf._els(ctx, mass=f["mass"], role=("wall",)):
        if e["side"] not in f["sides"]:
            continue
        if _severity(ctx, e["storey"], e["mass"]) <= 0.45:
            continue
        fr = qf._piece_frame(e)
        if fr is None:
            continue
        w, hh = fr[3], fr[4]
        m = ctx["info"]["masses"].get(e["mass"]) or ctx["info"]["masses"]["main"]
        out.append({"fr": fr, "ua": w * 0.25, "ub": w * 0.75,
                    "va": e["z"] + 0.4, "vb": e["z"] + hh - 0.4,
                    "out": -0.05, "e": e, "m": m, "side": e["side"],
                    "storey": e["storey"]})
    # SPREAD THEM ALONG THE ELEVATION, not up it. Shuffled, the draw put
    # several vents on the same bay at consecutive storeys and Flow merged
    # them into two vertical columns of fireballs climbing the façade like a
    # rocket exhaust (uf5 skyscraper_a, 2026-08-28). Bucket by position along
    # the wall and take at most one per bucket.
    out.sort(key=lambda o: (o["e"]["lx"], o["e"]["ly"], o["storey"]))
    step = max(1, len(out) // max(1, n))
    return out[::step][:n]


# WHY THE FIRST VERSION LOOKED LIKE FIREBALLS, AND WHAT FIXED IT.
# One `FlowEmitterSphere` per window at radius 0.55-1.05 m with the full
# emission scale injects a compact BALL of fuel that then burns as a ball:
# "the fire kinda looks like individual fire balls" (user review,
# 2026-08-28). Four things were wrong together and all four had to move:
#
#  1. THE SOURCE SHAPE. A window vents a SHEET of flame the width of the
#     opening, not a point. `FlowEmitterBox` with `halfSize` fitted to the
#     opening is the right primitive, and it is what NVIDIA's own incident
#     extension uses. `fire.add_fire_emitter`'s docstring argues against a
#     box — correctly, for a TREE: "a box fitted to a whole tree looks like a
#     burning cuboid", because a tree is not box-shaped. A window is.
#  2. ONE SOURCE PER WINDOW. A single emitter gives one blob; three small
#     ones across the width at jittered heights give a ragged front.
#  3. FUEL PER EMITTER. Each blob was over-fuelled, so it reached the top of
#     the colormap on its own and rendered as a saturated yellow sphere.
#     Spread the same total over more, smaller sources.
#  4. GRID RESOLUTION. At `densityCellSize` 0.14 m a 1 m flame is seven
#     voxels across and quantises to a lump. The bench now runs 0.09.
FLAME_OUT = 0.42       # m proud of the wall face
FLAME_UP = 2.6         # m/s of the emitter's own velocity, upward
FLAME_PUSH = 0.9       # m/s outward, so the plume clears the reveal
FLAME_PER_OPENING = 3  # sources across one window, when it is genuinely alight
FLAME_SCALE = 0.44     # per-source emission, since there are now several

# WHICH OPENINGS BURN IS NOT DECIDED HERE ANY MORE. `soot_plume.plan_events`
# grows contiguous compartment runs (`soot_plume.RUN_LEN`) along each
# burning storey — the same "a fire spreads to the compartment NEXT DOOR,
# not to every third window" rule the old `_flame_runs` enforced — and
# `r_flames` lights the runs whose state is "flame". The soot skin is
# rasterised from the same runs, which is the point.

# ITEMS 5/6 — SMOKE-ONLY, SHARING THE SAME BUDGET. Small and fixed rather
# than proportional to building size: removing the painted `_plume` geometry
# freed nothing on the GPU (it was mesh, not Flow), so every source added
# here is NEW load on the same `rtx/flow/maxBlocks` pool the flame sources
# already draw from. See `r_flames`'s docstring for the accounting.
SMOKE_EXTRA_MAX = 3     # extra smoke-only vents, ACTIVE fire only (F2/F3):
                        # windows that are staining but never got a flame
                        # source — what a painted plume tongue used to stand
                        # in for.
SMOKE_MIN_SEV = 0.15    # matches the old `_plume` threshold: "barely marked"
INTERIOR_SMOKE_MAX = 3  # sources seated on the gutted floor slabs, burnt-out
                        # states only (F4/F5) — "fires inside the burned-down
                        # buildings" (user review, 2026-08-29)


def _flame_sources(ctx, root, op, state, scale, tag, per_opening):
    """`per_opening` `FlowEmitterBox` sources across one opening's head.

    Factored out of `r_flames` so a window that is only SMOKING can be given
    the identical sheet-across-the-opening placement a flaming one gets, just
    with a different `state` (`fire.STATE_EMISSION`) and fewer sources —
    `state` and the building's own ACTIVE state (`ctx["fire"]["state"]`) are
    now two different things on purpose. Returns the number of Flow prims
    authored.
    """
    from pxr import Gf, Sdf
    from . import fire as fx, quake_flow as qf

    rng = ctx["rng"]
    hu0, hu1 = op.get("hua", op["ua"]), op.get("hub", op["ub"])
    hv0, hv1 = op.get("hva", op["va"]), op.get("hvb", op["vb"])
    w = max(0.4, hu1 - hu0)
    sev = _severity(ctx, op["storey"], op["e"]["mass"], op["e"])
    ox, oy = qf._outward(op["m"], op["side"])
    made = 0
    for k in range(per_opening):
        u = hu0 + ((k + 0.5) / per_opening) * (hu1 - hu0)
        # AT THE HEAD, NOT AT THE CENTRE. Hot gases leave through the top
        # two-thirds of an opening and cool air is drawn in at the bottom
        # (the neutral plane) — a source at mid-height puts the flame's
        # root below the cill and it reads as a fire in the street.
        # Jittered per source, so the front is ragged and not a level bar.
        v = hv0 + (0.66 + 0.22 * rng.random()) * (hv1 - hv0)
        x, y, _z = qf._b_face_pt(op["fr"], u, v, op["out"] + FLAME_OUT)
        path = "{0}/emitters/{1}_{2}_{3:02d}".format(root, ctx["tag"], tag, k)
        prim = fx._flow_create(ctx["stage"], path, "FlowEmitterBox")
        if not prim or not prim.IsValid():
            continue
        fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
        fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(float(x), float(y), float(v)))
        # wide across the opening, shallow through the wall, short in z:
        # a slot, which is the shape of the gap the gases leave through
        hw = 0.5 * w / per_opening * 1.35
        fx._set(prim, "halfSize", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(float(hw), float(hw), float(0.16 + 0.22 * sev)))
        fx._set(prim, "halfSizeIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx._set(prim, "coupleRateFuel", Sdf.ValueTypeNames.Float, 2.0)
        fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
        fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(float(ox * FLAME_PUSH * rng.uniform(0.7, 1.3)),
                         float(oy * FLAME_PUSH * rng.uniform(0.7, 1.3)),
                         float(FLAME_UP * rng.uniform(0.85, 1.2))))
        fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx.set_emission(prim, state,
                        scale=float(scale) * FLAME_SCALE
                        * (0.6 + 0.7 * sev) * rng.uniform(0.75, 1.25))
        made += 1
    return made


def _interior_smoke(ctx, root, state, scale, budget):
    """Smoke-only sources seated ON the gutted floor slabs, not just at the
    windows they happen to vent through.

    A burnt-out shell smoulders from its OWN debris bed — the severity
    ladder already says so ("heavy SMOULDER SMOKE") and the render did not
    carry it: every source `r_flames` authored sat at an opening, so a floor
    with two burnt-out windows got two little jets and the space between
    them, where the actual fuel bed is, got nothing ("place some more fires
    that are smoke only... we can replicate that", user review, 2026-08-29).
    One low source per involved storey that still has a slab to sit on
    (`r_floor_burnthrough` can have taken it already — `fit["slabs"]` holds
    `None` for those, the same guard every other pass in this file checks),
    hottest storeys first within `budget`, so it reads as rising OUT through
    the openings and the roof hole rather than floating in the void.
    """
    from pxr import Gf, Sdf
    from . import fire as fx, quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    fit = ctx.get("fit") or {}
    cand = []
    for (mtag, storey), slab in (fit.get("slabs") or {}).items():
        if not slab or storey not in f["storeys"]:
            continue
        sev = _severity(ctx, storey, mtag)
        if sev < 0.25:
            continue
        cand.append((sev, mtag, storey))
    cand.sort(key=lambda t: -t[0])
    made = 0
    for sev, mtag, storey in cand[:budget]:
        m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
        lx = rng.uniform(-0.25, 0.25) * m["W"]
        ly = rng.uniform(-0.25, 0.25) * m["D"]
        wx, wy = qf._to_world(m, lx, ly)
        z = m["levels"][min(storey, len(m["levels"]) - 1)] + 0.4
        path = "{0}/emitters/{1}_int_{2}_{3}".format(root, ctx["tag"], mtag,
                                                      storey)
        prim = fx._flow_create(ctx["stage"], path, "FlowEmitterSphere")
        if not prim or not prim.IsValid():
            continue
        fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
        fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(float(wx), float(wy), float(z)))
        fx._set(prim, "radius", Sdf.ValueTypeNames.Float, 1.1)
        fx._set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
        fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(0.0, 0.0, 1.6))
        fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx.set_emission(prim, state,
                        scale=1.1 * float(scale) * (0.6 + 0.6 * sev))
        made += 1
    return made


def _roof_plume(ctx, root, state, scale):
    """One or two big smoke sources over the roof: a burnt-through roof vents
    hard, and from the air — this dataset's own camera — it is the whole
    story. Smoke only, even while the building is still actively flaming
    below: by the time the roof is through the flame front is inside.
    Unchanged from the original inline version, just factored out.
    """
    from pxr import Gf, Sdf
    from . import fire as fx, quake_flow as qf

    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    made = 0
    for k in range(2):
        path = "{0}/emitters/{1}_roof{2}".format(root, ctx["tag"], k)
        prim = fx._flow_create(ctx["stage"], path, "FlowEmitterSphere")
        if not prim or not prim.IsValid():
            continue
        lx = rng.uniform(-0.3, 0.3) * m["W"]
        ly = rng.uniform(-0.3, 0.3) * m["D"]
        wx, wy = qf._to_world(m, lx, ly)
        fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
        fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(float(wx), float(wy), float(m["top"] - 0.6)))
        fx._set(prim, "radius", Sdf.ValueTypeNames.Float, 1.8)
        fx._set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
        fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3,
                Gf.Vec3f(0.6, 0.2, 3.2))
        fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        fx.set_emission(prim, "smoulder" if state == "flame" else state,
                        scale=1.25 * float(scale))
        made += 1
    return made


def r_flames(ctx, max_emitters=9, scale=1.0):
    """Flow emitters, placed by the building's FIRE EVENTS
    (`ctx["fire"]["events"]`, `soot_plume.plan_events`) — the same list the
    soot skin is rasterised from, so a window with flame coming out of it is
    a window with a plume of soot over it, by construction.

    Does nothing unless the launcher has authored the Flow stack
    (`fire.setup_flow_stack`) — the emitters need its layer, and its
    `rtx/flow/maxBlocks` pool is what decides whether emitter number twelve
    gets any voxels at all. `ctx["flow_root"]` is where they are parented.

    FOUR PARTS, ONE SHARED BUDGET:

      1. FLAME events — `FLAME_PER_OPENING` sheet sources across each of
         their openings, at most `max_emitters` openings in all. Planning
         already caps flame events at `soot_plume.FLAME_BUDGET_OPENINGS`
         (the same 9) and re-labels the overflow as burnt OUT, so the cap
         here is belt and braces. If nothing could be planned at all (a
         band with no openings of any kind) the old module-vent fallback
         (`_wall_vents`) runs.
      2. SMOKE from the compartments that have already burnt OUT while the
         building is still alight — F3's lower storeys, the fire having
         moved up — up to `SMOKE_EXTRA_MAX`, nearest the front first, one
         source each, `state="smoke"`.
      3. On a burnt-out building, its SMOULDER events (the topmost few, see
         `plan_events`) get one source each with the building's own state,
         plus `_interior_smoke` on the gutted slabs.
      4. The roof plume, unchanged, gated on `f["roof"]`.

    An event whose openings belong to a module a collapse has since taken
    (`dead`) is skipped: a flame on a wall that is no longer there floats.

    THE BUDGET is unchanged: worst case ~27 flame + 3 smoke + 3 interior +
    2 roof = ~35 Flow prims per building, fixed caps that do not scale with
    building size. If that is too many at city scale, the number to raise is
    `fire.DEFAULTS["max_blocks"]` (owned by `fire.py`).
    """
    from pxr import Sdf, UsdGeom
    from . import soot_plume as spl

    f, rng = ctx["fire"], ctx["rng"]
    state = f.get("state")
    if not state:
        return
    root = ctx.get("flow_root")
    if not root:
        ctx["notes"].append("flames: no flow stack on this stage, skipped")
        return
    is_flame = state == "flame"

    def live(ev):
        return all(not o["e"].get("dead") for o in ev["ops"])

    events = [ev for ev in (f.get("events") or []) if live(ev)]
    UsdGeom.Xform.Define(ctx["stage"], Sdf.Path(root + "/emitters"))
    n = n_open = 0

    # -- 1) the flame events ----------------------------------------------
    for ev in [ev for ev in events if ev["state"] == "flame"]:
        for op in ev["ops"]:
            if n_open >= max_emitters:
                break
            n += _flame_sources(ctx, root, op, "flame", scale,
                                "e{0}_{1}".format(ev["id"], n_open),
                                FLAME_PER_OPENING)
            n_open += 1
    if is_flame and n_open == 0:
        for i, op in enumerate(_wall_vents(ctx, max_emitters)):
            n += _flame_sources(ctx, root, op, state, scale, i,
                                FLAME_PER_OPENING)
            n_open += 1

    # -- 2) / 3) smoke ------------------------------------------------------
    n_smoke = 0
    if is_flame:
        out = sorted([ev for ev in events if ev["state"] == "out"],
                     key=lambda ev: (-ev["storey"], ev["id"]))
        for ev in out[:SMOKE_EXTRA_MAX]:
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += _flame_sources(ctx, root, op, "smoke", scale,
                                      "sm{0}".format(ev["id"]), 1)
    else:
        sm = [ev for ev in events if ev["state"] == "smoulder"]
        for ev in sm[:spl.SMOULDER_EVENTS_MAX]:
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += _flame_sources(ctx, root, op, state, scale,
                                      "sm{0}".format(ev["id"]), 1)
    n += n_smoke
    n_interior = 0
    if not is_flame:
        n_interior = _interior_smoke(ctx, root, state, scale,
                                     INTERIOR_SMOKE_MAX)
        n += n_interior

    # -- 4) the roof plume ---------------------------------------------------
    n_roof = 0
    if f.get("roof"):
        n_roof = _roof_plume(ctx, root, state, scale)
        n += n_roof

    ctx["notes"].append(
        "flames: {0} flame source(s) over {1} opening(s), {2} smoke source(s) "
        "at events, {3} interior, {4} roof, state={5}, {6} Flow prim(s) "
        "total".format(n - n_smoke - n_interior - n_roof, n_open, n_smoke,
                       n_interior, n_roof, state, n))


# `fire_collapse` owns the PARTIAL collapse (part of the shell down, the rest
# standing) the way this file owns `r_fire_collapse` (the top storeys down,
# all four walls still up). Imported at module scope on purpose and safely:
# `fire_collapse` reads this file's fire palette (`_debris_mat`, `_burn_mat`,
# `_joist_stubs`, `_deck_slab`, `_drop_face_art`) only from INSIDE its
# functions, so there is no import cycle — and the import has to happen when
# the ladder loads, because `fire_collapse` registers the `F5c` level with
# `soot_plume.DURATION_S` at import time and `soot_plume.plan_events` runs
# before the first recipe does (an unregistered level returns NO fire events
# at all: no soot, no flames, silently).
from . import fire_collapse as _fire_collapse                  # noqa: E402

RECIPES = {
    "smoke_stain": r_smoke_stain,
    "roof_scorch": r_roof_scorch,
    "expose_interior": r_expose_interior,
    "window_burnout": r_window_burnout,
    "char_facade": r_char_facade,
    "spall": r_spall,
    "render_peel": r_render_peel,
    "gut_interior": r_gut_interior,
    "floor_burnthrough": r_floor_burnthrough,
    "roof_burnthrough": r_roof_burnthrough,
    "curtain_burn": r_curtain_burn,
    "fire_collapse": r_fire_collapse,
    "partial_collapse": _fire_collapse.r_partial_collapse,
    "street_debris": r_street_debris,
    "flames": r_flames,
}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def burn_building(stage, parent, style, placements, x, y, yaw, level,
                  rng, nrng, mats, tag, flow_root=None, origin=None,
                  sides=None, mat_cache=None, fit_storeys=None,
                  events=None, openings_fn=None, soot_prebaked=False,
                  fire=None, skin=None):
    """Set one placed kit building on fire to `level` (F0..F5, or a recipe
    list). Returns the same ctx shape `quake_flow.wreck_building` does, so a
    caller can hand `loose` / `static_extra` / `velocity` straight to
    `settle.run`.

    The building must already be on the stage (`apply_placements` has set each
    placement's `prim_path`).

    `events` / `openings_fn` / `soot_prebaked` are the hooks a WHOLE-ASSET
    building sliced into a kit uses (`disaster.gac_fire`): its fire events
    are planned from the merged asset's own measured windows BEFORE it is
    sliced, its soot is baked into the asset's atlases once through the
    merged mesh's UVs (hundreds of pieces then share a dozen sooted maps
    instead of each baking its own), and `r_smoke_stain` therefore has
    nothing left to do on the pieces. `openings_fn(ctx, mass, side, storey)`
    replaces `soot_plume.openings` for `plan_events`/`r_flames` where the
    kit has no measured window table.
    """
    from . import quake_flow as qf

    info = qf.describe(style, placements, x, y, yaw)
    btype = info["type"]
    if isinstance(level, str):
        recipes = LADDER[btype][level]
        lvl = level
    else:
        recipes, lvl = level, "F3"
    ctx = {"stage": stage, "parent": parent, "info": info, "rng": rng,
           "nrng": nrng, "mats": mats,
           "cache": mat_cache if mat_cache is not None else {},
           "tag": tag, "loose": [], "static_extra": [], "velocity": {},
           "authored": [], "notes": [], "flow_root": flow_root,
           "fit": {"slabs": {}, "columns": {}, "partitions": [], "props": {},
                   "all": []}}
    # `fire` lets a caller that already planned the fire (to bake soot into
    # a merged asset before slicing it) hand the SAME plan in, rather than
    # re-drawing the band from `origin`/`sides` and getting a different one.
    ctx["fire"] = (dict(fire) if fire is not None
                   else plan_fire(info, lvl, rng, origin=origin, sides=sides))
    # THE EVENTS ARE DRAWN HERE, BEFORE ANY RECIPE RUNS, so that every pass
    # that needs to know where the fire is (the skin, the flames) reads one
    # list — and before `fire_collapse` can take a wall away, so an event on
    # a module that later falls still leaves its stain on the neighbours
    # (physically right) while `r_flames` skips it (a flame on a wall that
    # is no longer there floats).
    # ...and from their OWN rng (`soot_plume.event_rng`), so the recipes'
    # shared `rng` sees exactly the sequence it did before events existed.
    from . import soot_plume as spl
    if openings_fn is not None:
        ctx["soot_openings"] = openings_fn
    # `soot_prebaked` is True (nothing left to bake) or a SET of material
    # prim paths that already carry the skin (`_bind_soot` skips those and
    # bakes the rest per piece); `skin` is that same skin, so the per-piece
    # bake and the pre-bake agree pixel for pixel
    ctx["soot_prebaked"] = soot_prebaked if isinstance(soot_prebaked, (set, frozenset)) \
        else bool(soot_prebaked)
    if skin is not None:
        ctx["soot_skin"] = skin
    ctx["fire"]["events"] = (list(events) if events is not None
                             else spl.plan_events(ctx, _severity))
    ctx["notes"].append("fire events: " + spl.summarise(ctx["fire"]["events"]))
    if not recipes:
        dress_roof_urban(ctx)
        ctx["static_extra"] += list(ctx.get("roof_plant", []))
        return ctx
    # THE INTERIOR HAS TO EXIST BEFORE ANYTHING ELSE RUNS. A kit shell is
    # hollow, and a fire is seen almost entirely through its openings — with
    # no floors and no contents behind them, every burnt-out window looks
    # straight through the building and out the other side.
    if fit_storeys is None:
        # ONLY THE STOREYS THE FIRE OPENED — plus, always, the TOP THREE.
        # `fit_interior` on all of a 33-storey tower is 32 slabs, ~1,900
        # columns, ~100 partitions and ~900 referenced furniture props, none
        # of which is visible through an intact façade, and it is most of the
        # build time. But a roof that burns through is a hole looking straight
        # down into whatever is under it, and if the band stopped below the
        # top the answer was NOTHING — an empty shell with a floor missing
        # (user review, 2026-08-28). The top of the building is the one place
        # that is always visible from a drone, so it is always fitted out.
        f_ = ctx["fire"]
        n_ = len(info["masses"][f_["mass"]]["levels"])
        # FROM THE ORIGIN, not the storey under it: that storey only ever
        # gets cracked panes (`_severity` 0.16, under `r_window_burnout`'s
        # threshold), so nothing behind its façade is ever seen (user,
        # 2026-08-30: "I don't want props that nobody can see").
        fit_storeys = set(range(f_["origin"], f_["top"] + 2)) \
            | set(range(max(0, n_ - 3), n_))
    # THE COLUMN GRID FOLLOWS THE MEASURED PLAN, NOT THE BOUNDING BOX.
    # `fit_interior` lays its grid on the mass's `W x D`, which is the plan
    # only for a cuboid; on a sliced whole-asset building with a setback the
    # outer columns stand proud of the façade and poke through it (fire_dtc3
    # review, 2026-08-30, `fit_g6/col_main_10_2_1` on SM_Building_02 F5c).
    # `ctx["fire"]["footprints"]` is a key only `gac_fire.prepare` ever sets,
    # so this is None on the kit path and nothing there moves.
    # THE 2 M ROOF SHORTEN IS GATED TO THE SLICED PATH ONLY, same rule as
    # `dress_roof_urban`'s own local-seating fix just below: `soot_prebaked`
    # is a real `set`/`frozenset` only when `gac_fire.burn_gac` set it, so
    # this reads `col_roof_shorten=0.0` (the old height, unconditionally)
    # for every kit-building call — byte-identical, `kit_burn_probe.py`.
    is_gac = isinstance(ctx.get("soot_prebaked"), (set, frozenset))
    ctx["fit"] = qf.fit_interior(stage, parent, info, mats, rng,
                                 storeys=fit_storeys, tag=tag,
                                 footprint=ctx["fire"].get("footprints"),
                                 col_roof_shorten=(qf.COL_ROOF_SHORTEN_M
                                                   if is_gac else 0.0))
    dress_roof_urban(ctx)
    ctx["fit"]["all"] += list(ctx.get("roof_plant", []))
    for name, kw in recipes:
        RECIPES[name](ctx, **(kw or {}))
    qf._b_settle_roof_plant(ctx, recipes)
    ctx["static_extra"] += [e["p"].get("prim_path") for e in qf._els(ctx)
                            if e["p"].get("prim_path")]
    ctx["static_extra"] += [p for p in ctx["fit"]["all"] if p not in ctx["loose"]]
    # A DEACTIVATED PRIM MUST NOT BE IN `loose`. `r_gut_interior` deletes a
    # share of the partitions and the props; if any pass has since put one of
    # those paths in the loose list, `settle` cannot cook a mesh for it and
    # reports "loose prim(s) NEVER SIMULATED ... still wherever the damage
    # stage authored them" — which is a floating object with a confusing
    # diagnostic attached (uf_r2j/uf_r2k, `part_main_4_1`).
    ctx["loose"] = [q for q in ctx["loose"]
                    if stage.GetPrimAtPath(q).IsValid()
                    and stage.GetPrimAtPath(q).IsActive()]
    loose = set(ctx["loose"])
    seen, st = set(), []
    for p in ctx["static_extra"]:
        if p in loose or p in seen:
            continue
        seen.add(p)
        st.append(p)
    ctx["static_extra"] = st
    return ctx


def check(verbose=True):
    """Host-side: every ladder recipe exists, every family has a type."""
    from detail import urban_building as ub
    from . import quake_flow as qf

    bad = []
    for t, ladder in LADDER.items():
        if t not in ("urm", "rc", "rc_glass"):
            bad.append("unknown construction type {0}".format(t))
        for lv, recs in ladder.items():
            if lv not in LEVELS:
                bad.append("{0}: unknown level {1}".format(t, lv))
            for name, _kw in recs:
                if name not in RECIPES:
                    bad.append("{0}/{1}: unknown recipe {2}".format(t, lv, name))
    for lv in LEVELS:
        for t in ("urm", "rc", "rc_glass"):
            if lv not in LADDER[t]:
                bad.append("{0} has no {1}".format(t, lv))
    for s, spec in ub.STYLES.items():
        if qf.FAMILY_TYPE.get(spec.get("family")) not in LADDER:
            bad.append("style {0}: family {1} has no fire ladder".format(
                s, spec.get("family")))
    if verbose:
        print("[urban_fire] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


# ---------------------------------------------------------------------------
# Offline verification of the connectivity / gradient fix
# ---------------------------------------------------------------------------
# `scene_gen/tools/urban_fire_dryrun.py` was read before writing this: it
# solves the CITY-SCALE ignition schedule (`urban_fire_spread` /
# `urban_fire_city` — which buildings catch and when), a real and useful
# check but not this one — it never calls `plan_fire`, `_severity`,
# `_side_reach` or any per-element material pass, so it cannot see either
# bug this fixes. Both of those bugs live entirely in THIS file's pure
# functions (no `pxr`, no stage, no placements), so `check_scorch` tests them
# directly and needs nothing that tool already sets up.
def check_scorch(verbose=True):
    """Host-side, no stage: the two invariants the user's connectivity /
    gradient report reduces to, machine-checked rather than eyeballed.

      A. REACHABILITY. `_side_reach` must return exactly 0 for a side that
         is neither in the fire's own `sides`, nor a corner-neighbour of one,
         nor (roof-like) covered by a fire that reached the top storey — and
         must return exactly 1.0 / `SIDE_BLEED` in the cases that ARE
         reachable. Checked EXHAUSTIVELY: `_SIDE_RING` has 4 elements, so
         every non-empty subset of `sides` (15) x every `side` (4) x
         `roof_like` (2) x `f["roof"]` (2) is only 240 cases.
      B. GRADIENT. `_grad_bucket` must be non-increasing as `sev` rises
         (closer to the seat of the fire -> darker or equal, never lighter),
         and — end to end, through the REAL `_severity` — the "above the
         involved band" tail (`_severity`'s own `d >= n` branch, which is
         provably monotonic by its formula) must produce a `_grad_bucket`
         sequence that is non-decreasing with storey distance `d` from the
         top of the band, for several band lengths.
    """
    bad = []

    # -- A: reachability, exhaustive over the tiny side/roof state space ----
    def _all_nonempty_subsets(items):
        for k in range(1, len(items) + 1):
            for combo in itertools.combinations(items, k):
                yield combo

    for sides in _all_nonempty_subsets(_SIDE_RING):
        ctx = {"fire": {"sides": sides, "roof": False}}
        ctx_roof = {"fire": {"sides": sides, "roof": True}}
        neighbours_of_hot = set()
        for s in sides:
            neighbours_of_hot.update(_side_neighbors(s))
        for side in _SIDE_RING:
            for roof_like in (False, True):
                for c, roof_flag in ((ctx, False), (ctx_roof, True)):
                    got = _side_reach(c, side, roof_like=roof_like)
                    if roof_like and roof_flag:
                        want = 1.0
                    elif side in sides:
                        want = 1.0
                    elif side in neighbours_of_hot:
                        want = SIDE_BLEED
                    else:
                        want = 0.0
                    if abs(got - want) > 1e-9:
                        bad.append(
                            "_side_reach(sides={0}, side={1}, roof_like={2}, "
                            "f.roof={3}) = {4}, want {5} -- an element here "
                            "would be scorched with no burnt path to the "
                            "origin".format(sides, side, roof_like, roof_flag,
                                            got, want))
                    # THE CORE CLAIM, restated as a single assertion: nothing
                    # unreachable ever gets a positive multiplier.
                    is_reachable = (side in sides) or (side in neighbours_of_hot) \
                        or (roof_like and roof_flag)
                    if not is_reachable and got > 0.0:
                        bad.append(
                            "UNREACHABLE side scorched: sides={0} side={1} "
                            "roof_like={2} f.roof={3} -> reach={4} (want 0)"
                            .format(sides, side, roof_like, roof_flag, got))

    # -- B1: `_grad_bucket` is monotonic in `sev`, AND ACTUALLY VARIES. A flat
    # `lambda sev: 0` is "monotonic" too — that IS the original "one
    # intensity everywhere" bug — so monotonicity alone does not catch it;
    # this also demands at least two distinct grades across the range.
    samples = [i / 200.0 for i in range(201)]   # 0.00 .. 1.00 ascending
    buckets = [_grad_bucket(s) for s in samples]
    for i in range(1, len(buckets)):
        if buckets[i] > buckets[i - 1]:
            bad.append(
                "_grad_bucket not monotonic: sev {0:.3f}->{1:.3f} gave "
                "bucket {2}->{3} (bucket must not RISE as severity rises)"
                .format(samples[i - 1], samples[i], buckets[i - 1], buckets[i]))
            break
    if len(set(buckets)) < 2:
        bad.append(
            "_grad_bucket is FLAT across sev 0..1 (all bucket {0}) -- no "
            "gradient at all, the exact 'plain black/grey rectangles' "
            "defect".format(buckets[0]))

    # -- B2: end to end through the REAL `_severity`, above-the-band tail ---
    # This is the one region `_severity`'s own formula
    # (`max(0.0, 0.30 - 0.12 * (d - n))`) is provably monotonic decreasing in
    # `d`, so it is the region where "scorch gets lighter with distance" can
    # be checked against the ACTUAL function rather than reimplemented.
    seen_buckets = set()
    for n in (1, 2, 4, 6, 10):
        ctx = {"fire": {"mass": "main", "sides": ("S",), "origin": 0,
                        "storeys": list(range(n)), "top": n - 1,
                        "n_storeys": n + 6, "roof": False}}
        prev_sev, prev_bucket = None, None
        for d in range(n, n + 6):
            storey = d  # origin is 0, so storey == d here
            sev = _severity(ctx, storey, "main")
            bucket = _grad_bucket(sev)
            seen_buckets.add(bucket)
            if prev_sev is not None:
                if sev > prev_sev + 1e-9:
                    bad.append(
                        "_severity rose with distance above the band: "
                        "n={0} d={1}->{2} sev {3:.3f}->{4:.3f}".format(
                            n, d - 1, d, prev_sev, sev))
                if bucket < prev_bucket:
                    bad.append(
                        "grade got DARKER further from the origin: n={0} "
                        "d={1}->{2} bucket {3}->{4}".format(
                            n, d - 1, d, prev_bucket, bucket))
            prev_sev, prev_bucket = sev, bucket
    if len(seen_buckets) < 2:
        bad.append(
            "grade never changed across the whole above-the-band sweep "
            "(bucket(s) seen: {0}) -- distance from the origin is not "
            "affecting how light the material is".format(seen_buckets))

    if verbose:
        print("[urban_fire] check_scorch {0}".format(
            "ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


# ---------------------------------------------------------------------------
# Offline verification that the soot is WHERE THE FIRE IS
# ---------------------------------------------------------------------------
def check_soot_events(verbose=True, styles=("commercial_mid", "apartment",
                                             "office_slab", "highrise_step"),
                      levels=("F1", "F2", "F3", "F5")):
    """Host-side, no stage: the coupling `soot_plume` exists for, machine-
    checked on REAL kit buildings (`detail.urban_building` +
    `quake_flow.describe`, pure placement math — the same trick
    `check_scorch` uses) through the actual `plan_fire` -> `plan_events` ->
    `skin` path a scene takes:

      A. every event sits on the fire's own mass, band and sides;
      B. FLAME events only while the ladder says the building is alight —
         never on F1, never on F4+ — and never more than
         `soot_plume.FLAME_BUDGET_OPENINGS` openings' worth;
      C. the skin is CLEAN two storeys under the origin (alpha < 0.03
         everywhere there) and only STAINED, never burnt, in the storey
         directly under it (mean alpha < 0.12) — the "clean band under the
         black stripe" signature;
      D. every event's own head is DARK (mean alpha > 0.45 across its first
         opening, just above the head): the stain is rooted where the fire
         is, not merely somewhere on the same storey;
      E. an F5 skin's burning sides are near-saturated across the band (mean
         alpha > 0.75) and still NOT one flat colour (rgb std > 0.002).
    """
    import random as _random

    import numpy as _np

    from detail import urban_building as ub
    from . import quake_flow as qf, soot_plume as spl

    bad = []
    tried = 0
    for style in styles:
        for li, level in enumerate(levels):
            rng = _random.Random(20260830 + li)
            nrng = _np.random.default_rng(20260830 + li)
            placements = ub.build_building(style, 0.0, 0.0, 0.0, rng)
            info = qf.describe(style, placements, 0.0, 0.0, 0.0)
            ctx = {"info": info, "rng": rng, "nrng": nrng, "notes": []}
            mtag = max(info["masses"].items(),
                       key=lambda kv: (len(kv[1]["levels"]),
                                       kv[0] == "main"))[0]
            n_st = len(info["masses"][mtag]["levels"])
            origin = max(1, min(n_st - 2, int(round(0.3 * (n_st - 1)))))
            if n_st < 3:
                continue
            sides = ("S",) if level in ("F1", "F2") else ("S", "E")
            ctx["fire"] = plan_fire(info, level, rng, origin=origin,
                                    sides=sides)
            f = ctx["fire"]
            events = spl.plan_events(ctx, _severity, rng)
            f["events"] = events
            tag = "{0}/{1}".format(style, level)
            if not events:
                bad.append(tag + ": no fire events planned")
                continue
            tried += 1
            # -- A ------------------------------------------------------
            for ev in events:
                if ev["mass"] != mtag or ev["storey"] not in f["storeys"] \
                        or ev["side"] not in f["sides"]:
                    bad.append("{0}: event {1} off the plan (mass {2}, storey "
                               "{3}, side {4}; band {5} sides {6})".format(
                                   tag, ev["id"], ev["mass"], ev["storey"],
                                   ev["side"], f["storeys"], f["sides"]))
            # -- B ------------------------------------------------------
            n_flame_open = sum(len(ev["ops"]) for ev in events
                               if ev["state"] == "flame")
            if level in ("F1", "F4", "F5", "F6") and n_flame_open:
                bad.append("{0}: {1} flame opening(s) on a level with no "
                           "flame".format(tag, n_flame_open))
            if level in ("F2", "F3") and not n_flame_open:
                bad.append("{0}: an ACTIVE fire with no flame event".format(tag))
            if n_flame_open > spl.FLAME_BUDGET_OPENINGS:
                bad.append("{0}: {1} flame openings over the budget of {2}"
                           .format(tag, n_flame_open,
                                   spl.FLAME_BUDGET_OPENINGS))
            # -- the skin ---------------------------------------------------
            sk = spl.skin(ctx, events, nrng, finish=f.get("finish") or "char",
                          glass=(info["type"] == "rc_glass"))
            al = sk["rgba"][..., 3]
            h, w = al.shape
            m = info["masses"][mtag]
            ppm, H, z0 = sk["ppm"], sk["H"], sk["z0"]

            def row_of(z):
                return int(round((H - (z - z0)) * ppm))

            # -- C ------------------------------------------------------
            if origin >= 2:
                r = row_of(m["levels"][origin - 1])
                if r < h and float(al[r:].max()) > 0.03:
                    bad.append("{0}: soot two storeys under the origin (max "
                               "alpha {1:.3f} below z={2:.1f})".format(
                                   tag, float(al[r:].max()),
                                   m["levels"][origin - 1]))
            r_top = row_of(m["levels"][origin])
            r_bot = row_of(m["levels"][origin - 1])
            if r_top < r_bot <= h:
                under = float(al[r_top:r_bot].mean())
                if under > 0.12:
                    bad.append("{0}: the storey under the origin is burnt, "
                               "not stained (mean alpha {1:.3f})".format(
                                   tag, under))
            # -- D ------------------------------------------------------
            off = sk["offsets"]
            for ev in events:
                u0, u1, _zs, zh = ev["ops"][0]["span"]
                c0 = int(round((off[ev["side"]] + u0) * ppm))
                c1 = max(c0 + 1, int(round((off[ev["side"]] + u1) * ppm)))
                cols = _np.arange(c0, c1) % w
                r0 = max(0, row_of(zh + 0.6))
                r1 = max(r0 + 1, row_of(zh + 0.15))
                head = float(al[r0:r1][:, cols].mean())
                if head < 0.45:
                    bad.append("{0}: event {1} ({2}) is not dark at its own "
                               "head (alpha {3:.2f})".format(
                                   tag, ev["id"], ev["state"], head))
            # -- E ------------------------------------------------------
            if level == "F5":
                r0 = row_of(m["top"])
                r1 = row_of(m["levels"][f["storeys"][0]])
                band_cols = []
                for side in f["sides"]:
                    L = spl.side_length(m, side)
                    band_cols.append(_np.arange(
                        int(round(off[side] * ppm)),
                        int(round((off[side] + L) * ppm))) % w)
                cols = _np.concatenate(band_cols)
                band = al[max(0, r0):r1][:, cols]
                rgb = sk["rgba"][max(0, r0):r1][:, cols][..., :3]
                if float(band.mean()) < 0.75:
                    bad.append("{0}: burnt-out band not near-saturated (mean "
                               "alpha {1:.3f})".format(tag, float(band.mean())))
                if float(rgb.std()) < 0.002:
                    bad.append("{0}: burnt-out band is one flat colour (rgb "
                               "std {1:.4f})".format(tag, float(rgb.std())))
    if tried == 0:
        bad.append("check_soot_events: nothing was checked")
    if verbose:
        print("[urban_fire] check_soot_events {0} ({1} building(s))".format(
            "ok" if not bad else "FAILED", tried))
        for b in bad:
            print("  " + b)
    return bad
