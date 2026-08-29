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

`LADDER[btype][level]` is the recipe list, keyed by the same construction
types `quake_flow.FAMILY_TYPE` assigns, because how a building burns depends
on what it is made of every bit as much as how it shakes:

  urm       masonry shell, TIMBER floors and roof. The floors are the fuel.
            This is the type that gets gutted and the only one that collapses.
  rc        concrete frame with masonry infill. Spalls, does not fall.
  rc_glass  curtain-wall tower. The glass goes in a vertical stripe and the
            structure does nothing at all.
"""

import math
import os
import random

# ---------------------------------------------------------------------------
# The ladder
# ---------------------------------------------------------------------------
LEVELS = ("F0", "F1", "F2", "F3", "F4", "F5")

# Is there still fire in it? Drives the Flow state and the material finish:
# an active fire is FRESH char (wet-black, still glossy) with flame; a
# burnt-out one is cooled char going to grey ash, and smoke only.
ACTIVE = {"F0": None, "F1": None, "F2": "flame", "F3": "flame",
          "F4": "smoulder", "F5": "residual"}
FINISH = {"F0": None, "F1": "scorch", "F2": "char", "F3": "char",
          "F4": "ash", "F5": "ash"}

# How many storeys the fire is in, as a fraction of the block's height, and
# where it started. A fire that starts on the top floor and a fire that
# starts on the third are the same event and photograph differently, so the
# origin is drawn rather than fixed — but it is drawn LOW-BIASED, because a
# fire is far more likely to start in an occupied lower storey and because a
# fire that started high leaves nothing below it to show the contrast.
BAND = {
    "F1": (1, 2, 0.0),      # (min storeys, max storeys, share above origin)
    "F2": (1, 2, 0.35),
    "F3": (3, 6, 0.75),
    "F4": (4, 99, 1.0),     # everything from the origin up
    "F5": (4, 99, 1.0),
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
    },
}


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
    # GLASS IN A FIRE IS A LADDER, NOT ONE GREY. The first cut bound a single
    # flat dark tone to every pane it did not remove, and a wall of identical
    # matte grey rectangles is the one thing glass never looks like — "have
    # glass crack and scorch, not just blacken, looks weird when it's just
    # plain gray" (user review, 2026-08-28). Four states, and the ORDER is
    # the order a pane goes through them:
    #   clear      -> the pane is still glazed and still reflective
    #   smoked     -> a brown-grey film condensed on the inside face; it is
    #                 WARM, because what deposits on glass is tar, not soot
    #   sooted     -> opaque deposit, the state next to a vented opening
    #   shard      -> a piece on the pavement, soot-filmed on one face
    # Roughness climbs with the deposit: a clean pane mirrors the sky and a
    # tarred one has no specular left at all.
    "glass_clear":  ((0.030, 0.042, 0.040), 0.09),     # -> ~0.23, glossy
    "glass_smoked": ((0.055, 0.044, 0.033), 0.34),     # -> ~0.29, warm
    "glass_sooted": ((0.020, 0.018, 0.016), 0.55),     # -> ~0.19
    "fire_glass":   ((0.022, 0.026, 0.024), 0.86),     # -> ~0.20, the shards
    # The crack line itself: brighter than the pane, because a fracture in
    # glass scatters light rather than absorbing it. This is why a cracked
    # pane reads at all — the cracks are the LIGHT part of it.
    "glass_crack":  ((0.30, 0.33, 0.33), 0.16),        # -> ~0.60
}


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
_CONCRETE_TEX = ("airstack://scene_gen/assets/materials/megascans/"
                 "Worn_Pavement/T_uddhdb1fw_2K_B.png")


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
    from pxr import UsdShade

    from . import damage, quake_flow as qf

    out = dict(qf.materials(stage, parent))
    scope = parent + "/FireLooks"
    for key, (rgb, rough) in _FLAT.items():
        path = scope + "/" + key
        m = UsdShade.Material.Get(stage, path)
        if not m:
            m = damage._pbr(stage, path, rgb, rough)
        out[key] = m
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


def _burn_set(stage, parent):
    """The burn maps an URBAN building may wear.

    `damage.char_materials` with the two wood-patterned maps swapped out
    (see `_URBAN_MAPS`), and `albedo_brightness` used to trim what is left —
    `damage._pbr`'s `rgb` cannot, because with a texture bound OmniPBR's
    `diffuse_color_constant` is what the map REPLACES (the earthquake
    round-2 finding).
    """
    from pxr import Sdf, UsdShade
    from . import damage

    mats = damage.char_materials(stage, parent, maps=_URBAN_MAPS,
                                 suffix="_urban")
    for key, b in _BURN_BRIGHTNESS.items():
        if abs(b - 1.0) < 1e-3:
            continue
        for m in mats.get(key, ()):
            sh = UsdShade.Shader.Get(stage, m.GetPath().AppendChild("Shader"))
            if sh:
                sh.CreateInput("albedo_brightness",
                               Sdf.ValueTypeNames.Float).Set(float(b))
    return mats


def _burn_mat(ctx, finish="char"):
    """One textured burn material, by finish mix. `finish` is char / scorch /
    ash — the same vocabulary `damage._pick` uses."""
    from . import damage
    return damage._pick(ctx["rng"], finish, ctx["mats"]["_burn"])


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


def _in_band(ctx, e):
    """Is this element on a burning elevation, in the involved band?"""
    f = ctx["fire"]
    return e["storey"] in f["storeys"] and (e["side"] in f["sides"]
                                            or e["role"] in ("roof", "parapet",
                                                             "parapet_corner"))


# ---------------------------------------------------------------------------
# 1. THE SIGNATURE: soot plumes above the openings
# ---------------------------------------------------------------------------
# A COMPARTMENT FIRE'S PORTRAIT IS DRAWN ON THE SPANDRELS. What every
# photograph of a burnt block has and nothing else in this repo produces is
# the row of black tongues licking up the wall from each window head, widening
# and fading as they go, merging into one stripe where two floors were alight
# at once. It is authored geometry rather than a texture composite because it
# has to be ROOTED at a measured opening: a texture wash on a module puts the
# gradient wherever the module is, and the whole read is that the black starts
# exactly at the top of the hole.
PLUME_H = 2.6          # how far one tongue reaches above the head, m at
                       # severity 1.0 (a storey is 3 m, so it laps the sill
                       # above — which is how the fire gets in there)
PLUME_FLARE = 1.30     # width at the top as a multiple of the opening width
# NOT EVERY WINDOW GETS A TONGUE, and this is the difference between a burnt
# building and a black rectangle. Kit windows sit 1.2-1.5 m apart; at the
# first flare (1.55) every tongue touched its neighbours and 148 of them
# merged into one continuous sheet over two elevations, which is exactly the
# "reads as flat panels" failure. Real deposition is patchy — the window the
# fire actually vented through is heavily marked and the one beside it barely
# — so the tongue is ROLLED per opening and the flare is under the pitch.
PLUME_P = 0.62
# THREE STACKED BANDS, PROGRESSIVELY LIGHTER. One flat polygon per tongue has
# a hard top edge and reads as a panel stuck on the wall whatever shape it is;
# soot thins with height and the fade is most of what says "deposit" rather
# than "paint". Bands are cut from the same outline so they share its flanks.
PLUME_BANDS = (("soot", 0.00, 0.42), ("soot_mid", 0.36, 0.76),
               ("soot_light", 0.68, 1.00))
# HUG THE WALL. At 22 mm the tongues rendered as flat grey panels hovering in
# front of the façade — visibly detached, with their own silhouettes against
# the sky (uf_bench dw_terrace, 2026-08-28). A stain is ON the masonry; 6 mm
# is enough to beat z-fighting on a flat kit panel and too little to read as
# an object. The bands step by 1.5 mm above this.
PLUME_PROUD = 0.006
# A SPALL HAS TO SIT IN FRONT OF THE PLUME. Both are flat meshes on the same
# wall plane and the plume is 22 mm proud, so a scar authored at the quake
# path's 4-7 mm is BEHIND the soot tongue wherever the two overlap — which is
# exactly where a fire spalls, since the tongue marks the hottest strip of
# wall. It is also physically right: the spall took the soot off with it.
# `quake_flow._face_patch` cannot be used for this: its shadow ring is
# hard-coded at PATCH_RING_PROUD (7 mm), so raising only the fill puts the
# ring behind both the fill and the plume. `_scar` below draws the pair.
SCAR_HALO_PROUD = 0.030
SCAR_PROUD = 0.036


def _plume_profile(rng, w, h, flare=PLUME_FLARE, n=26):
    """One tongue as a pair of FLANKS, sampled `n` times up its height.

    Returns (ts, right, left) where `ts` are heights 0..h and the two lists
    are the signed half-widths at each. Keeping the flanks rather than a
    closed polygon is what lets `_plume` cut the same tongue into stacked
    bands that share their edges exactly — cut two independently drawn
    outlines and the seam between the bands is a visible notch.

    NOT A TRIANGLE AND NOT A GRADIENT RECTANGLE. A plume off a window is a
    turbulent stack of licks with a ragged edge and a notch or two, so any
    shape with two straight sides reads as a decal. Three harmonics on each
    flank, out of phase, and the two flanks are drawn INDEPENDENTLY — a
    symmetric flame shape is the giveaway. The first version sampled 9 times
    and the harmonics came out as facets: at that rate a 5.7-cycle term is
    aliased and the tongue is a black hexagon.
    """
    ph = [rng.uniform(0, 6.283) for _ in range(6)]
    ts, right, left = [], [], []
    for i in range(n + 1):
        t = i / float(n)
        # Narrow AT THE HEAD (the opening width is the source), widening
        # through the middle, pinched back at the top so it TAPERS rather
        # than ending in a bar.
        taper = 1.0 - 0.62 * max(0.0, t - 0.55) / 0.45
        base = 0.5 * w * (1.0 + (flare - 1.0) * t) * max(0.18, taper)
        wr = 1.0 + 0.22 * math.sin(3.1 * t * 6.283 + ph[0]) \
            + 0.14 * math.sin(5.7 * t * 6.283 + ph[1]) \
            + 0.09 * math.sin(9.3 * t * 6.283 + ph[2])
        wl = 1.0 + 0.22 * math.sin(2.7 * t * 6.283 + ph[3]) \
            + 0.14 * math.sin(6.3 * t * 6.283 + ph[4]) \
            + 0.09 * math.sin(8.1 * t * 6.283 + ph[5])
        ts.append(h * t)
        right.append(base * max(0.15, wr))
        left.append(-base * max(0.15, wl))
    return ts, right, left


def _plume(ctx, fr, u_mid, head, w, h, out=None, owner=None):
    """A soot tongue above one opening: three stacked bands, light upward.

    Returns the number of meshes authored."""
    rng = ctx["rng"]
    ts, right, left = _plume_profile(rng, w, h)
    n_pts = len(ts)
    made = 0
    for k, (key, t0, t1) in enumerate(PLUME_BANDS):
        i0 = max(0, int(round(t0 * (n_pts - 1))))
        i1 = min(n_pts - 1, int(round(t1 * (n_pts - 1))))
        if i1 - i0 < 2:
            continue
        outline = [(right[i], ts[i]) for i in range(i0, i1 + 1)] + \
                  [(left[i], ts[i]) for i in range(i1, i0 - 1, -1)]
        # each band a hair further off the wall than the one under it, so a
        # grazing view cannot z-fight the overlap
        o = (PLUME_PROUD if out is None else out) + 0.0015 * k
        if _face_polygon(ctx, fr, u_mid, head, outline, ctx["mats"][key],
                         out=o, kind="plume", owner=owner):
            made += 1
    return made


def _face_polygon(ctx, fr, u0, v0, outline, mat, out=PLUME_PROUD, kind="plume",
                  owner=None):
    """Author a closed (du, dv) outline as one flat mesh on a wall face.

    `owner` is the element the art belongs to; the path is recorded against
    it in `ctx["face_art"]` so a recipe that later takes that module away can
    take its stains with it (`_drop_face_art`)."""
    from . import quake_flow as qf
    pts = [qf._b_face_pt(fr, u0 + du, v0 + dv, out) for du, dv in outline]
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


def _wall_tops(ctx):
    """{(mass, side): highest z of a LIVE wall} — where the masonry ends.

    `mass["top"]` is the AUTHORED top of the building and it is the wrong
    ceiling for a stain in two common cases: a style whose top band is a
    parapet (`_mass_specs` skips parapet bands, so `top` is below the
    coping), and any building a recipe has already taken the top storey off.
    On the burnt terrace the second one put a row of soot tongues rooted at
    the top surviving window heads and reaching two metres into open SKY —
    grey flags along the roofline, the most-reported defect on this bench
    (uf_bench2/uf4 dw_terrace, 2026-08-28). Measure the wall that is actually
    standing instead.
    """
    from . import quake_flow as qf
    tops = {}
    for e in qf._els(ctx, role=("wall", "corner", "parapet", "parapet_corner")):
        k = (e["mass"], e["side"])
        tops[k] = max(tops.get(k, -1e9), e["z"] + e["h"])
    return tops


def r_smoke_stain(ctx, heavy=1.0, above=2):
    """Soot plumes above every opening in the band, plus the wall wash.

    Two passes, and both are needed:

    1. GEOMETRY — one tongue per opening, rooted at the head. This is what
       reads from 40 m and from the air, and it is the thing that says
       "the fire came out of THAT window".
    2. TEXTURE — the module's own cladding map composited with soot
       (`damage.scorched_material`), so the wall between the tongues is
       stained rather than clean. Direction matters: the module CONTAINING
       the fire compartment is heaviest at its TOP (the plume leaves under
       the head / the eaves), and every module ABOVE it takes an up-wash from
       its base. Getting that pair the wrong way round gives a wall that is
       black at the sill and clean at the head, which is the wildfire
       signature and looks upside down here.
    """
    from pxr import UsdShade

    from . import damage, quake_flow as qf

    f, rng, stage = ctx["fire"], ctx["rng"], ctx["stage"]
    glassy = ctx["info"]["type"] == "rc_glass"
    tops = _wall_tops(ctx)
    n_plume = n_wash = n_clip = 0
    ops = list(qf._g2_openings(ctx, sides=f["sides"])) + \
        list(qf._g_shop_openings(ctx, sides=f["sides"]))
    for op in ops:
        sev = _severity(ctx, op["storey"], op["e"]["mass"], op["e"]) * float(heavy)
        if sev < 0.12:
            continue
        if rng.random() > PLUME_P * min(1.0, 0.45 + sev):
            continue
        w = max(0.35, op.get("hub", op["ub"]) - op.get("hua", op["ua"]))
        head = op.get("hvb", op["vb"])
        u_mid = 0.5 * (op.get("hua", op["ua"]) + op.get("hub", op["ub"]))
        # HEIGHT VARIES HARD, and that is deliberate: a uniform 2.4 m over
        # every window is a stencil. 0.55-1.5 x spans "barely marked" to
        # "the tongue reached the sill above".
        h = PLUME_H * min(1.35, sev) * rng.uniform(0.55, 1.5)
        # AND IT IS CLIPPED TO THE TOP OF THE WALL. A tongue over a top-storey
        # window ran straight off the parapet and stood in the SKY as a grey
        # flag — a row of them read as bunting over the roofline. There is no
        # masonry up there to stain, so there is no tongue.
        wall_top = tops.get((op["e"]["mass"], op["e"]["side"]))
        if wall_top is None:
            continue
        headroom = wall_top - head - 0.15
        if headroom < 0.35:
            n_clip += 1
            continue
        h = min(h, headroom)
        n_plume += _plume(ctx, op["fr"], u_mid, head, w * 0.95, h,
                          owner=op["e"])
    # -- the wall wash -----------------------------------------------------
    top = f["top"]
    for e in qf._els(ctx, role=("wall", "corner", "parapet", "parapet_corner",
                               "balcony")):
        if e["side"] not in f["sides"] and e["role"] not in ("parapet",
                                                             "parapet_corner"):
            continue
        sev = _severity(ctx, e["storey"], e["mass"], e) * float(heavy)
        if sev < 0.10 or e["storey"] > top + above:
            continue
        path = e["p"].get("prim_path")
        if glassy:
            # A CURTAIN WALL IS NOT COMPOSITED. `scorched_material` paints a
            # soot wash INTO the surface's own base-colour map, which is
            # exactly right on brick or stone — the courses survive under the
            # staining — and wrong on the tower families, whose base colour
            # is a pale glazing atlas: the result is white panes with black
            # ink-runs down them (uf5 skyscraper_a, 2026-08-28). Soot on
            # glass is an opaque film, so bind the flat tone.
            # A SHARE, NOT ALL OF IT. At full coverage every pane in the
            # band went opaque black and the burning storeys came out as one
            # flat dark slab with no glazing left in them (uf6
            # skyscraper_a, 2026-08-28). A curtain wall on fire keeps some
            # bays clear, some filmed and some gone — `r_curtain_burn` owns
            # the "gone", this owns the "filmed".
            mat = ctx["mats"]["soot" if sev > 0.72 else "soot_mid"]
            if _bind_subsets(stage, e["p"], lambda: mat,
                             min(0.9, 0.35 + 0.55 * sev), rng):
                n_wash += 1
            continue
        tex = damage.bound_texture(stage, path) if path else None
        if not tex:
            continue
        # In the band: vented at the top of the storey. Above it: washed up
        # from the base. `from_above` is what selects between the two.
        vent = e["storey"] in f["storeys"]
        lvl = damage.bucket(min(1.0, 0.30 + 0.70 * sev))
        mat = damage.scorched_material(
            stage, ctx["parent"], None, lvl, from_above=vent,
            texture=tex, cache=ctx["cache"], wash_weight=0.62,
            brightness=(0.55 if sev > 0.8 else 0.8))
        if _bind_subsets(stage, e["p"], lambda: mat, 1.0, rng):
            n_wash += 1
    ctx["notes"].append(
        "smoke: {0} plume mesh(es) over openings ({1} skipped, no wall above), "
        "{2} module(s) washed, storeys {3}-{4} on {5}".format(
            n_plume, n_clip, n_wash, f["origin"], top, "/".join(f["sides"])))


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
            vo = min(float(op["out"]), -0.02) - 0.05
            # INSET BY A FEW PER CENT so a rim of the reveal survives round
            # it. Flush to the measured hole, a terrace of shopfronts came out
            # as a wall of identical black rectangles with no edge and the
            # openings stopped reading as openings (uf_bench2 dw_terrace).
            iw, ih = w * 0.94, h * 0.96
            _face_polygon(
                ctx, op["fr"], 0.5 * (hu0 + hu1), hv0 + 0.02 * h,
                [(-iw / 2, 0.0), (iw / 2, 0.0), (iw / 2, ih), (-iw / 2, ih)],
                (ctx["mats"]["void"] if rng.random() < 0.72
                 else ctx["mats"]["soot"]), out=vo, kind="void",
                owner=op["e"])
            n_void += 1
            # A charred frame round it: four thin bars. Without them the void
            # is a rectangle of nothing and the opening loses its edge.
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

    1. THE DEPOSIT IS GRADED, HEAVIEST AT THE TOP. A compartment fire fills
       from the ceiling down (the smoke layer sits above the neutral plane),
       so the pane is opaque at the head, brown through the middle and
       comparatively clear at the cill. That vertical gradient is the single
       most recognisable thing about a window with fire behind it, and one
       flat tone throws it away. Three stacked quads.
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
    # 1) the graded deposit — top band heaviest
    bands = (("glass_sooted", 0.62, 1.00),
             ("glass_smoked", 0.26, 0.66),
             ("glass_clear", 0.00, 0.30))
    for i, (key, t0, t1) in enumerate(bands):
        # heavier fires push the smoke layer further down the pane
        lo = max(0.0, t0 - 0.22 * sev)
        hi = min(1.0, t1 - 0.18 * sev) if i else 1.0
        if hi - lo < 0.06:
            continue
        _face_polygon(ctx, fr, um, v0 + lo * h,
                      [(-w / 2, 0.0), (w / 2, 0.0),
                       (w / 2, (hi - lo) * h), (-w / 2, (hi - lo) * h)],
                      ctx["mats"][key], out=out + 0.0012 * i, kind="pane",
                      owner=op["e"])
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
    for e in qf._els(ctx, role=("wall", "corner", "parapet", "parapet_corner",
                               "balcony", "roof")):
        sev = _severity(ctx, e["storey"], e["mass"], e)
        if e["role"] in ("roof", "parapet", "parapet_corner"):
            # the roof and its parapet see the fire only if it got that high
            sev = _severity(ctx, f["n_storeys"] - 1, e["mass"])
        elif e["side"] not in f["sides"]:
            # THE RETURN. A fire venting on the south face still blackens a
            # metre or two round the corner, and a stripe that stops dead at
            # the building's corner is the tell of a decal. Quarter strength.
            sev *= 0.28
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
        mm = _burn_mat(ctx, finish)
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
                x, y, _ = qf._b_face_pt(fr, u, v, 0.012)
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
        for _ in range(rng.randint(1, 2)):
            u = rng.uniform(0.2 * width, 0.8 * width)
            v = e["z"] + rng.uniform(0.4, max(0.6, e["h"] - 0.4))
            # 0.25-0.7 m, NOT 0.7-1.5. A 3 m blob on a 4 m module is most of
            # the module, and a patch that big cannot read as anything but a
            # decal however it is shaded.
            ra = rng.uniform(0.18, 0.48)
            _scar(ctx, fr, u, v, ra, ra * rng.uniform(0.8, 1.7),
                  ctx["mats"]["brick_bare"], "peel", owner=e)
            n_patch += 1
        sides_hit.add(e["side"])
    # the windrow of fallen render at the foot of each affected wall
    m = ctx["info"]["masses"][f["mass"]]
    for side in sides_hit:
        qf._heap(ctx, m, m["z0"], 0.0, 0.06, fill=False, sides=(side,),
                 depth_m=rng.uniform(0.16, 0.34), along=(-0.42, 0.42),
                 tag="peelrow",
                 mat_fn=lambda: (qf._a_mat(ctx, "dust") if rng.random() < 0.35
                                 else ctx["mats"]["soot_light"]))
    ctx["notes"].append("render peel: {0} sheet(s) off, windrow on {1}"
                        .format(n_patch, "/".join(sorted(sides_hit)) or "-"))


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
    for (mtag, storey), slab in (fit.get("slabs") or {}).items():
        # `slab` can be None: `r_roof_hole` clears the entry for a floor it
        # broke, and every `pxr` call takes an ArgumentError on None rather
        # than missing quietly.
        if not slab or _severity(ctx, storey, mtag) < 0.3:
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
    ctx["notes"].append(
        "gutted: {0} prop(s) consumed, {1} charred, {2} partition(s) down, "
        "{3} slab(s) charred".format(n_gone, n_char, n_part, n_slab))


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
    """Bearing stubs of a failed slab, still in the wall pocket."""
    from . import quake_flow as qf
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    rng = ctx["rng"]
    if storey >= len(m["levels"]):
        return
    z = m["levels"][storey] - 0.18
    W, D = m["W"] - 2 * qf.WALL_INSET, m["D"] - 2 * qf.WALL_INSET
    n = n if n is not None else rng.randint(3, 7)
    for _ in range(n):
        side = rng.random() < 0.5
        L = rng.uniform(0.5, 1.8)
        if side:
            lx = rng.uniform(-W / 2, W / 2)
            ly = (D / 2) * rng.choice((1.0, -1.0))
            p0 = qf._to_world(m, lx, ly)
            p1 = qf._to_world(m, lx, ly - math.copysign(L, ly))
        else:
            ly = rng.uniform(-D / 2, D / 2)
            lx = (W / 2) * rng.choice((1.0, -1.0))
            p0 = qf._to_world(m, lx, ly)
            p1 = qf._to_world(m, lx - math.copysign(L, lx), ly)
        path = "{0}/joist_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        qf._cyl(ctx["stage"], path, (p0[0], p0[1], z),
                (p1[0], p1[1], z - rng.uniform(0.0, 0.35)),
                rng.uniform(0.060, 0.080), ctx["mats"]["burnt_metal"])
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
    z = m["top"] + 0.02
    W, D = m["W"], m["D"]
    made, kind = [], ctx.setdefault("roof_plant_kind", {})
    mat_metal = ctx["mats"]["plant_metal"]
    mat_deck = ctx["mats"]["dark_concrete"]

    def _box(lx, ly, lz, sx, sy, sz, mat, tag):
        wx, wy = qf._to_world(m, lx, ly)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"],
                                        qf._uid(ctx))
        qf._box(ctx["stage"], path, wx, wy, lz, sx, sy, sz, m["yaw"], mat)
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
        pth = _box(blx, bly, z + bh / 2.0, bw, bd, bh, mat_deck, "bulkhead")
        made.append(pth)
        kind[pth] = "bulkhead"
        # its own parapet-height upstand roof, so it is not a bare box
        pth = _box(blx, bly, z + bh + 0.10, bw + 0.30, bd + 0.30, 0.20,
                   mat_deck, "bulkcap")
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
            path = "{0}/ac_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                           qf._uid(ctx))
            wx, wy = qf._to_world(m, lx, ly)
            # the housekeeping pad under the row, once per row
            if k == 0:
                plx, ply = ((0.0, cr) if long_x else (cr, 0.0))
                pw = (span + AC_W + 2 * PAD_MARGIN) if long_x else (AC_D + 2 * PAD_MARGIN)
                pd = (AC_D + 2 * PAD_MARGIN) if long_x else (span + AC_W + 2 * PAD_MARGIN)
                made.append(_box(plx, ply, z + 0.06, pw, pd, 0.12,
                                 mat_deck, "acpad"))
                kind[made[-1]] = "pad"
            if qf._prop(ctx["stage"], path, qf._AC_UNIT, wx, wy, z + 0.12,
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
        for pth in qf._tank(ctx, wx, wy, z, r=tr, h=rng.uniform(2.2, 2.8),
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
        path = "{0}/vent_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                         qf._uid(ctx))
        h = rng.uniform(0.7, 1.4)
        qf._cyl(ctx["stage"], path, (wx, wy, z), (wx, wy, z + h), 0.16,
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
        "{2} fixed + {3} loose item(s)".format(side, rows, len(fixed),
                                               len(plant)))
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
    for e in qf._els(ctx, mass=mass, role=("parapet", "parapet_corner")):
        # a corner piece has no meaningful side, so take it whenever either
        # of its two elevations is burning
        if e["role"] == "parapet" and e["side"] not in f["sides"] \
                and rng.random() > 0.45:
            continue
        n_par += _bind_subsets(ctx["stage"], e["p"],
                               lambda: _burn_mat(ctx, f["finish"]),
                               min(1.0, 0.9 * sev_top), rng)
    for e in qf._els(ctx, mass=mass, role="roof"):
        n_deck += _bind_subsets(ctx["stage"], e["p"],
                                lambda: _burn_mat(ctx, f["finish"]), 1.0, rng)
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
            qf._b_bind_over(ctx["stage"], pth, _burn_mat(ctx, f["finish"]))
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
    for _ in range(n_deb):
        lx, ly = rng.uniform(-W / 2, W / 2), rng.uniform(-D / 2, D / 2)
        wx, wy = qf._to_world(m, lx, ly)
        sz = 0.12 + 0.55 * rng.random() ** 2.0
        r = rng.random()
        mat = (ctx["mats"]["char_concrete"] if r < 0.45 else
               ctx["mats"]["soot"] if r < 0.75 else ctx["mats"]["ash"])
        path = "{0}/rdeb_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                         qf._uid(ctx))
        qf._a_lump(ctx["stage"], path, wx, wy, m["top"] + sz * 0.3, sz, rng,
                   mat, jitter=0.5)
        ctx["authored"].append(path)
    ctx["notes"].append(
        "roof scorch: {0} parapet subset(s), {1} deck subset(s), {2} plant "
        "item(s) charred{3}, {4} piece(s) of debris on the deck".format(
            n_par, n_deck, n_plant,
            " (deck breached: plant dropped to physics)"
            if ctx.get("roof_breached") else "", n_deb))


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
                        bw, D, bh, m["yaw"], ctx["mats"]["char_concrete"])
                ctx["authored"].append(path)
                ctx["static_extra"].append(path)
                n_beam += 1
            path = "{0}/beam_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                             qf._uid(ctx))
            qf._box(stage, path, m["cx"], m["cy"],
                    z - qf.SLAB_T["rc"] - bh / 2.0, W, bw, bh, m["yaw"],
                    ctx["mats"]["char_concrete"])
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
                            h_st, m["yaw"], ctx["mats"]["char_concrete"])
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
    have = {k[1] for k, v in (fit.get("slabs") or {}).items()
            if k[0] == mass and v and stage.GetPrimAtPath(v).IsValid()
            and stage.GetPrimAtPath(v).IsActive()}
    for st_ in (top_s, top_s - 1):
        if st_ <= 0 or st_ in have:
            continue
        z = m["levels"][st_]
        path = "{0}/catch_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                          qf._uid(ctx))
        qf._box(stage, path, m["cx"], m["cy"], z - 0.13, W, D, 0.26,
                m["yaw"], ctx["mats"]["char_concrete"])
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
    path = "{0}/deck_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
    qf._box(ctx["stage"], path, m["cx"], m["cy"], m["top"] - T / 2.0,
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
    """Bar and purlin ends standing up out of a burnt-through roof deck."""
    from . import quake_flow as qf
    rng = ctx["rng"]
    z = m["top"]
    W, D = m["W"] - 1.2, m["D"] - 1.2
    for _ in range(n):
        lx = rng.uniform(-W / 2, W / 2)
        ly = rng.uniform(-D / 2, D / 2)
        wx, wy = qf._to_world(m, lx, ly)
        L = rng.uniform(0.5, 1.6)
        a = rng.uniform(0, 6.283)
        tilt = rng.uniform(0.15, 0.55)
        path = "{0}/rafter_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        qf._cyl(ctx["stage"], path, (wx, wy, z - 0.4),
                (wx + math.cos(a) * L * tilt, wy + math.sin(a) * L * tilt,
                 z - 0.4 + L * math.sqrt(max(0.0, 1 - tilt * tilt))),
                # 0.06 m FLOOR ON THE RADIUS. PhysX cooks nothing for a
                # sub-centimetre tube and `settle` reports it as "loose
                # prim(s) NEVER SIMULATED (no cookable mesh under them)" —
                # the piece then stays exactly where it was authored, which
                # on a roof that has since fallen is a stick hanging in the
                # sky (uf_r2j, 2026-08-28).
                rng.uniform(0.060, 0.085), ctx["mats"]["burnt_metal"])
        ctx["authored"].append(path)


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
    for e in qf._els(ctx, mass=f["mass"], role=("wall", "corner")):
        if e["side"] != side or e["storey"] < f["origin"]:
            continue
        above = e["storey"] - f["top"]
        if above > lick:
            continue
        along = (e["lx"] + m["W"] / 2.0) if side in ("S", "N") else \
                (e["ly"] + m["D"] / 2.0)
        # a soft edge to the stripe: a bay half in it is in it half the time
        over = min(along + 2.5, u0 + w) - max(along - 2.5, u0)
        if over <= 0.0:
            continue
        if over < 4.0 and rng.random() > over / 4.0:
            continue
        fr = qf._piece_frame(e)
        if fr is None:
            continue
        width, hh = fr[3], fr[4]
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
        "{3} from storey {4} up; {5} piece(s) in the fall zone".format(
            n_out, n_stain, w, side, f["origin"], n_dice))


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
        for pth in st + lo:
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
    H = max(3.0, m["top"] - m["z0"])
    # NO DUST IN THE MIX. `_heap`'s own `HEAP_MIX` is mortar dust over pale
    # brick, which is exactly right for a quake — the dust plume is the
    # signature — and exactly wrong here: fire rubble is black, wet from the
    # hose, and has no fines standing on it. `brick_dusty` (0.38 on screen)
    # was the pale tan litter in the terrace shot.
    qf._heap(ctx, m, base, (m["top"] - base) * 0.30, 0.10, fill=True,
             tag="fireheap",
             mat_fn=lambda: (ctx["mats"]["char_concrete"] if rng.random() < 0.45
                             else ctx["mats"]["soot"] if rng.random() < 0.80
                             else ctx["mats"]["calcined"]))
    ctx["notes"].append(
        "fire collapse: top {0} storey(s) down from storey {1}, {2} module(s) "
        "broken, {3} roof piece(s), {4} stain(s) removed with them, {5} "
        "fit-out prop(s) sent down too, heap on the storey below at "
        "z={6:.1f}".format(
            n_lv - s0, s0, n_broke, n_roof, n_art, n_fitprop, base))


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
    n = 0
    for side in f["sides"]:
        nx, ny = qf._outward(m, side)
        span = m["W"] if side in ("S", "N") else m["D"]
        half = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
        count = int(24 * density * rng.uniform(0.7, 1.3))
        for _ in range(count):
            t = rng.uniform(-0.5, 0.5) * span
            d = half + rng.uniform(0.4, 4.5)
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
FLAME_PER_OPENING = 3  # sources across one window
FLAME_SCALE = 0.44     # per-source emission, since there are now several


def r_flames(ctx, max_emitters=9, scale=1.0):
    """Flow emitters at the burning openings and, if it is through, the roof.

    Does nothing unless the launcher has authored the Flow stack
    (`fire.setup_flow_stack`) — the emitters need its layer, and its
    `rtx/flow/maxBlocks` pool is what decides whether emitter number twelve
    gets any voxels at all. `ctx["flow_root"]` is where they are parented.
    """
    from pxr import Gf, Sdf, UsdGeom

    from . import fire as fx, quake_flow as qf

    f, rng = ctx["fire"], ctx["rng"]
    state = f.get("state")
    if not state:
        return
    root = ctx.get("flow_root")
    if not root:
        ctx["notes"].append("flames: no flow stack on this stage, skipped")
        return
    ops = list(qf._g2_openings(ctx, sides=f["sides"])) + \
        list(qf._g_shop_openings(ctx, sides=f["sides"]))
    # HOTTEST FIRST. With a block pool that can run out, the emitters that
    # matter are the ones on the storey that is actually alight; a random
    # subset puts half the budget on a storey that is only staining.
    ops.sort(key=lambda o: -_severity(ctx, o["storey"], o["e"]["mass"]))
    ops = [o for o in ops
           if _severity(ctx, o["storey"], o["e"]["mass"]) > 0.45]
    # SPACE THEM OUT, or the licks merge into one horizontal BAR of flame
    # across the whole elevation — which is what 14 emitters at 0.14 m cells
    # gave on `office_wide` (uf_bench, 2026-08-28). Real flame comes out of
    # individual windows. Keep every third opening along the wall, then cap.
    ops = ops[::3] if len(ops) > 3 * max_emitters else ops
    ops = ops[:max_emitters]
    if not ops:
        # NO MEASURED OPENINGS ON THIS FAMILY. The glass tower (family 05)
        # paints its mullion grid into the façade map, so `_G2_WIN_FACES` and
        # `_G_SHOP_FACES` are both empty for it and the burning storeys got no
        # flame at all while every masonry building in the row had it. Fall
        # back to the wall ELEMENTS of the band: the emitter goes at the top
        # of the module, proud of its face, which is where a curtain-wall
        # spandrel vents anyway.
        ops = _wall_vents(ctx, max_emitters)
    UsdGeom.Xform.Define(ctx["stage"], Sdf.Path(root + "/emitters"))
    n = 0
    for i, op in enumerate(ops):
        hu0, hu1 = op.get("hua", op["ua"]), op.get("hub", op["ub"])
        hv0, hv1 = op.get("hva", op["va"]), op.get("hvb", op["vb"])
        w = max(0.4, hu1 - hu0)
        sev = _severity(ctx, op["storey"], op["e"]["mass"], op["e"])
        ox, oy = qf._outward(op["m"], op["side"])
        # A SHEET ACROSS THE OPENING, NOT A BALL IN THE MIDDLE OF IT.
        for k in range(FLAME_PER_OPENING):
            u = hu0 + ((k + 0.5) / FLAME_PER_OPENING) * (hu1 - hu0)
            # AT THE HEAD, NOT AT THE CENTRE. Hot gases leave through the top
            # two-thirds of an opening and cool air is drawn in at the bottom
            # (the neutral plane) — a source at mid-height puts the flame's
            # root below the cill and it reads as a fire in the street.
            # Jittered per source, so the front is ragged and not a level bar.
            v = hv0 + (0.66 + 0.22 * rng.random()) * (hv1 - hv0)
            x, y, _z = qf._b_face_pt(op["fr"], u, v, op["out"] + FLAME_OUT)
            path = "{0}/emitters/{1}_{2:02d}_{3}".format(root, ctx["tag"],
                                                         i, k)
            prim = fx._flow_create(ctx["stage"], path, "FlowEmitterBox")
            if not prim or not prim.IsValid():
                continue
            fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
            fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
                    Gf.Vec3f(float(x), float(y), float(v)))
            # wide across the opening, shallow through the wall, short in z:
            # a slot, which is the shape of the gap the gases leave through
            hw = 0.5 * w / FLAME_PER_OPENING * 1.35
            fx._set(prim, "halfSize", Sdf.ValueTypeNames.Float3,
                    Gf.Vec3f(float(hw), float(hw),
                             float(0.16 + 0.22 * sev)))
            fx._set(prim, "halfSizeIsWorldSpace", Sdf.ValueTypeNames.Bool,
                    True)
            fx._set(prim, "coupleRateFuel", Sdf.ValueTypeNames.Float, 2.0)
            fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
            fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3,
                    Gf.Vec3f(float(ox * FLAME_PUSH * rng.uniform(0.7, 1.3)),
                             float(oy * FLAME_PUSH * rng.uniform(0.7, 1.3)),
                             float(FLAME_UP * rng.uniform(0.85, 1.2))))
            fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool,
                    True)
            fx.set_emission(prim, state,
                            scale=float(scale) * FLAME_SCALE
                            * (0.6 + 0.7 * sev) * rng.uniform(0.75, 1.25))
            n += 1
    # The roof plume: a burnt-through roof vents hard, and from the air it is
    # the whole story. One big smoke source, no flame — by the time the roof
    # is through, the flame front is inside.
    if f.get("roof") and n < max_emitters + 4:
        m = ctx["info"]["masses"]["main"]
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
            n += 1
    ctx["notes"].append("flames: {0} emitter(s), state={1}".format(n, state))


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
    "street_debris": r_street_debris,
    "flames": r_flames,
}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def burn_building(stage, parent, style, placements, x, y, yaw, level,
                  rng, nrng, mats, tag, flow_root=None, origin=None,
                  sides=None, mat_cache=None, fit_storeys=None):
    """Set one placed kit building on fire to `level` (F0..F5, or a recipe
    list). Returns the same ctx shape `quake_flow.wreck_building` does, so a
    caller can hand `loose` / `static_extra` / `velocity` straight to
    `settle.run`.

    The building must already be on the stage (`apply_placements` has set each
    placement's `prim_path`).
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
    ctx["fire"] = plan_fire(info, lvl, rng, origin=origin, sides=sides)
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
        fit_storeys = set(range(max(0, f_["origin"] - 1), f_["top"] + 2)) \
            | set(range(max(0, n_ - 3), n_))
    ctx["fit"] = qf.fit_interior(stage, parent, info, mats, rng,
                                 storeys=fit_storeys, tag=tag)
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
