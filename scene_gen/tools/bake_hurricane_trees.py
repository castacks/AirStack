#!/usr/bin/env python3
"""Bake the HURRICANE tree archetypes — offline, no Isaac Sim, no physics.

WHY THIS EXISTS. `suburb_hurricane_launch_script.py` looks up
`tree_<species>_<level>` in `archetypes_hurricane/` and, finding nothing,
falls back to the green species USD and rewrites the level to `pristine`.
On the 500 m L2 plate that was 1,454 of 1,504 trees: a Cat-3 suburb with two
metres of water through it and a full, healthy summer canopy over the top.
The build-hurricane-scenes skill already names this "the most visible
remaining defect".

WHY IT IS NOT THE TORNADO BAKE. `vegetation.WIND_LEVELS` is
`pristine/limbed/leaning/fallen/snapped` — there is no `defoliated` in it,
and `defoliated` is the hurricane's PLURALITY outcome (1,361 of those 1,504
trees). A tornado breaks wood over a corridor; a hurricane strips leaves
over a county. So the level that matters most here is one the tornado
pipeline has never needed and cannot supply.

It is also why this bake needs no solver. `vegetation.wind_tree` earns its
physics settle by generating LOOSE limb and log debris that has to come to
rest — and `bake-tree-and-debris/SKILL.md` is explicit that such a settle
does not reproduce ("re-run it with the same seed... throws away the good
pile and ships a different one"). Defoliating, thinning, leaning and felling
a tree are all POSE AND VISIBILITY changes to one rigid asset, plus (for
`snapped`) one deterministic geometric clip of a single mesh. All of it is
exact and reproducible byte-for-byte, and it runs on a laptop in seconds.
The loose debris a hurricane needs is already handled elsewhere and in the
right medium: `washaway._RAFT_DIMS["log"]` floats boles in the water.

FOLIAGE IS IDENTIFIED BY BOUND MATERIAL, NOT BY PRIM NAME. In the AEC tree
assets a leaf card is called `<Species>_branch4` and so is a woody limb
(`<Species>_branch2`); the only thing that separates them is that one binds
`Shumard_Oak_leaf_v3_Mat2` and the other binds `TreeBark_7`. Keying off the
name would have hidden the woody skeleton along with the leaves and left a
bare trunk pole — see `_is_foliage`.

THE COTTON-BALL BUG (found reviewing `V2_L3`). A large, dense, pale-white
sphere of full foliage showed up in four review frames near the shoreline.
No baked archetype can produce that: every non-conifer level here strips ALL
broadleaf foliage (verified offline — see `scene_gen/tests/test_hurricane_
trees.py::test_no_foliage_leak_in_damaged_levels`), and a conifer's retained
needles read as a thin green cone, not a dense white ball. The only pose in
this pipeline with a FULL, undamaged, round canopy is `pristine` — and before
this file added a baked `pristine` archetype, `pristine` was never baked at
all: the launcher's fallback path referenced the raw, undamaged, CENTIMETRE-
SCALE species USD directly (`TREE_SPECIES` in the launcher), bypassing this
module and its material handling entirely. `GT_hurricane.json` for `V2_L3`
lists 39 such fallback trees. Every leaf/bark material on every one of the
six species' SOURCE assets resolves cleanly, case-exact, on this workstation
(also verified offline — see `scene_gen/tools/hurricane_tree_audit.py`), so
the fault is host-specific: most likely an incomplete asset sync to whatever
machine rendered `V2_L3` (the exact class of bug the session record already
caught once for a different texture — "References an asset that can not be
found" — and there is independent, pre-existing evidence that THIS species'
own texture folder has hit a path/case problem before: it already carries 49
Title-Case symlinks aliasing its lowercase real files, added before this
session). This file cannot fix a missing file on a machine it cannot reach.
What it CAN do, and now does, is two things that make the defect impossible
regardless of what is missing on the far end:

  1. Bake `pristine` too, so EVERY tree in the scene goes through this
     module's material handling — the raw cm-scale fallback is no longer on
     the hot path (see the launcher's `arch.get(pristine)` lookup, now tried
     before the raw species USD).
  2. `_material_safety_net` verifies, for every prim that will stay VISIBLE
     in the baked file, that its bound material's MDL file and every texture
     it references resolve case-exact on THIS machine. Anything that does
     not resolve gets rebound to a small constant-colour material authored
     directly in the archetype (no texture, cannot fail to resolve anywhere)
     instead of silently keeping a material that might come back blank/white
     on a different host.

DAMAGE IS READ BY CROWN COLOUR, NOT CROWN REMOVAL (RE-TUNED 2026-08-31,
STREAM T5, USER DIRECTIVE: "Forget defoliated. I see only 1-2 trees in the
whole scene."). The first version of this file followed the textbook
number literally — real Cat-3 broadleaf leaf LOSS is 86-94%, so `_FOL_KEEP`
kept only 10-12% of cards on `defoliated`/`limbed`/`leaning`. That is
correct BOTANY and wrong ART DIRECTION: measured directly on the shipped
archetypes, `tree_American_Beech_defoliated.usd` has 122,862 visible mesh
FACES but is a bare skeleton topping out at 5.9 m, and `tree_Black_Oak_
defoliated.usd`'s 83k woody faces plus 1,547 surviving leaf cards read, from
a 400 m overview camera, as a few branch-width pixels — indistinguishable
from empty ground. A pixel-diff between the two most recent renders (
`FINAL2`/`FINAL3`) came back IDENTICAL at every tree: there was no
regression, the archetypes were never wrong geometrically, they are simply
INVISIBLE AT ALTITUDE by design, and a bare-skeleton majority (`defoliated`
+ `limbed` + `leaning` was >80% of every plate) reads as "1-2 trees in the
whole scene" — only the rare, still-green `pristine` tree has enough crown
mass to register a pixel. Legibility from the air is the entire point of a
synthetic SAR-training scene, and a textbook-accurate bare skeleton that
does not render is strictly worse than a slightly-too-leafy tree that does.

So the retention table below is now built the other way around: EVERY
non-`snapped` level keeps a real, silhouette-forming majority of its crown,
and the ladder's escalating damage is carried by (a) a strong brown/tan
`diffuse_tint` override — a healthy tree reads green, a damaged one reads
dead-leaf brown, and that colour shift is now the PRIMARY class-separator
the skill's own "Trees are the strongest class separator" section asks
for — and (b) a SMALL, escalating amount of card/instance culling, just
enough to break the perfectly-round, unbroken silhouette a pristine crown
has (a real Cat-3 canopy is thinned and ragged, not merely recoloured).
`defoliated` keeps the MOST (70-85%: culling only breaks the silhouette,
10-25% of cards removed) because a re-coloured, still-round crown reads as
"the most common outcome, unmistakably damaged" at the widest band in
`hurricane._TREE_CUTS`; each rung up the ladder culls a bit more
(`limbed` 55-70%, `leaning`/`fallen` 45-60%) so the SHAPE also degrades
alongside the colour, and only `snapped`'s severed top — the one level with
an actual geometric break, not just a pose/colour change — drops
noticeably further (35%, up from the textbook-driven 20%, still visibly
thinner than a standing tree's residue: "went over only just, at the very
end", per the module's own snag section below). This is a DELIBERATE,
DOCUMENTED departure from the leaf-loss survey number, not an oversight —
see `_FOL_KEEP`'s own comment for the exact bands and the reasoning.

`snapped` IS A GEOMETRIC SNAG, NOT A THINNED CROWN. The trunk mesh is
clipped by a horizontal plane at the break height (0.30-0.50 of tree
height) with a pure-numpy triangle clipper (`_clip_mesh_flat`) that carries
position, normals and UVs per face-corner (these assets are already fully
face-varying and triangulated — see the module's own probe). The STUMP
(below the cut) stays upright and is the visible `/Root/src` content; the
SEVERED TOP (above the cut, plus whatever crown material was above break
height) is a second, independent reference to the same source asset under
`/Root/top`, rotated 80-85 degrees about the break point so it lies on the
ground near the stump, with a reduced (`_SNAP_TOP_FOL_KEEP`), tinted
residue of foliage — "went over" but only just, at the very end.

THE TINT WAS A DEAD KNOB (FIXED 2026-08-31, STREAM T6). The STREAM T5 fix
above (`_copy_tinted_material`) copied the SOURCE species' own custom-MDL
material and authored a `diffuse_tint` override on the copy -- correct USD
(a copy, never the shared original), but STREAM T6's render evidence
(`FINAL5_L3_brown`, a checksum-verified render set) measured every
defoliated-or-worse crown coming back GREEN/dark-olive at 17/80 sampled GT
positions and BROWN at only 2/60 -- the tint never reached the screen.
Every one of this pool's leaf/needle materials IS, at the MDL source
level, a thin re-export of OmniPBR (`export material Foo(*) = OmniPBR(
diffuse_tint: color(1,1,1), ...)` -- verified for all six species by
reading every `Trees/materials/*_leaf*.mdl`/`Pine_needles.mdl` in this
asset family), which SHOULD forward `diffuse_tint` as a real, overridable
parameter. Whether Kit's MDL JIT actually honours that forwarding through
a `(*)`-re-exported custom subIdentifier on the render host is a question
this offline, no-Kit, no-Hydra bake cannot answer -- so rather than keep
chasing an unreproducible render-only symptom, this fix stops trusting the
custom subIdentifier entirely: every kept foliage prim at a DAMAGED level
(not `pristine`, which keeps the untouched original look) is now rebound
to a FRESH, hand-authored `OmniPBR.mdl`/`OmniPBR` shader -- the exact idiom
`planks.wood_material`, `surge._make` and `wall_overlay.overlay_material`
already use successfully elsewhere in this dataset -- with the SAME
diffuse texture the custom MDL used (so a species/variant's visual
identity is unchanged) and a `diffuse_tint` computed, not guessed, to land
on a genuinely BROWN colour regardless of how green or dark that texture's
own pixels are. See `_omnipbr_leaf_material`'s and `_hue_flip_tint`'s
docstrings for the mechanism, and this file's own `--stats-json` bake
report for the measured per-species numbers.

NO OPACITY IS WIRED ON THE REPLACEMENT, and that is a finding, not an
oversight: NONE of the six species' leaf/needle `.mdl` files sets
`enable_opacity` or binds an opacity/mask texture, EVERY basecolor PNG
this pool's foliage materials bind measures alpha == 255 uniformly
(checked with PIL -- alpha-weighted mean is therefore just the plain
mean), and every one of these meshes' `doubleSided` attribute is
unauthored (defaults to single-sided). All three facts point the same
way: this asset family's "leaf cards" are real, leaf-shaped GEOMETRY, not
alpha-cutout billboard quads -- there is no cutout to wire, and no
two-sided flag needed on the material (sidedness is the MESH's own
attribute, untouched by this replacement). Adding `enable_opacity=True`
here would risk a NEW, unrequested translucency defect for zero benefit.

STREAM T7 (2026-09-01): VISIBLE BREAKAGE ON THE STRUCTURAL LEVELS. Review of
`FINAL8_L3_brown` (full rust-brown canopy, correct) found every crown reading
INTACT from the air: `leaning`/`fallen`/`limbed` kept 45-60% of their cards
under the STREAM T5 art-direction above, which is enough to silhouette as a
healthy tree from a review altitude, not enough to silhouette as a BROKEN
one. Directive, verbatim: "the trees all look perfect, need some of them to
be broken down. Look at fire and tornado for how we did it there." Four
per-level changes, all still exact/reproducible/no-Isaac (nothing here
simulates anything):

  `limbed` drops to a 30-40% AVERAGE (was 55-70%) but is no longer a single
  global fraction: a seeded azimuth SECTOR (90-120 deg wide) strips 70-85%
  of the cards inside it while the rest of the crown is left fuller than
  before, solved so the two regions still average to the directive target
  (`_select_kept_cards_sector`). This is the tornado skill's own precedent
  read sideways -- `wind_tree`'s crown thinning is a height GRADIENT because
  a sustained gust strips the exposed top hardest; a single gust direction
  strips one SIDE hardest, which a whole-crown fraction cannot represent at
  any keep value. The stripped sector also grows 2-4 bare broken-limb stubs
  jutting from it and 1-2 of their fallen counterparts on the ground
  (`_author_limb_break`) -- the tornado skill's "the wind BREAKS, and
  breaking exposes the inside of everything it breaks" made visible as
  actual bark-material geometry rather than only as missing leaf cards.

  `leaning` keeps ~45% (was 45-60%, effectively unchanged) but gains a
  torn ROOT-PLATE DISC at the base (`_author_root_plate_disc`) so the tilt
  reads as windthrow rather than as a rendering slant -- the tornado
  skill's own "the disc is what tells a fallen tree from a felled one at
  any distance" cue, reused here as a stand-alone quantity (`vegetation.
  root_plate` is READ, not imported -- this bake stays self-contained, see
  above).

  `fallen` drops to ~35% (was 45-60%) and also gains the root-plate disc --
  a downed crown reads by its root ball as much as by its foliage loss.

  `snapped`'s stump gains a JAGGED COLLAR of 3-5 small bark spikes around
  the break plane (`_snap_collar`), because `_clip_mesh_flat`'s plane cut
  otherwise leaves a perfectly flat, almost sawn-looking disc -- the fire
  skill's "a snag reads by its splintered spar, not a clean cut" language.
  The severed top's own residual foliage drops to ~25% (`SNAP_TOP_FOL_KEEP`,
  was 30-40%), thinner than any standing level's.

  THIS DELIBERATELY BREAKS THE OLD MONOTONIC-LADDER TEST INVARIANT
  (`limbed >= max(leaning, fallen)`): `leaning` (0.45) now keeps MORE than
  `limbed`/`fallen` (0.35 each), because `leaning` carries its damage read
  through the root-plate disc and a real (if modest) lean rather than
  through defoliation, while `limbed` and `fallen` now carry theirs through
  asymmetry/added debris and outright removal respectively. Retention
  percentage stopped being the only channel damage travels through, so it
  stopped needing to be monotonic in it. `test_retention_table_matches_
  stream_t5_directive` is updated accordingly, not merely relaxed --  see
  its own docstring for the replacement invariants.

  STREAM T7's ITS OWN NUMBERS ARE NOW SUPERSEDED BY STREAM T8 BELOW --
  the 30-40%/45%/35%/25% figures in the four paragraphs above are STREAM
  T7 history, kept for the record of how the ladder got here; the live
  values are in `_FOL_KEEP`/`_LIMB_SECTOR_KEEP_IN`/`SNAP_TOP_FOL_KEEP`.

STREAM T8 (2026-09-01): RETENTION CUT TOWARD THE REAL LEAF-LOSS FIGURE, NOW
THAT THE TINT IS GONE. User, on the live 500 m plate, after STREAM T7:
"Too many of the trees still have leaves. Look at how we removed trees in
the fire baking of trees (except we don't want burnt texture so you're
gonna have to rebake). Use those and increase the use of them as intensity
increases." Two things changed since the STREAM T5 retention ceiling
(70-85% for `defoliated`) was set, and both point the same direction:

  1. `defoliated` is the PLURALITY level (720 of 1684 trees on the
     reference plate, the widest band in `hurricane._TREE_CUTS`) and was
     sitting at 80% kept -- four fifths of the single most common tree on
     the plate reading essentially undamaged is exactly "too many trees
     still have leaves," measured, not merely felt.
  2. THE BROWN TINT THAT WAS SUPPOSED TO CARRY THE DAMAGE SIGNAL AT HIGH
     RETENTION IS GONE (see "LEAVES STAY GREEN BY DEFAULT" below,
     `TREE_LEAF_TINT` defaults OFF) -- retention was allowed to stay high
     in STREAM T5 specifically BECAUSE colour was going to do most of the
     damage-reading work and culling only had to "break the silhouette."
     With colour off the table, crown removal is the ONLY channel left for
     `defoliated`, `limbed`, `fallen` and the standing part of `snapped`
     (`leaning` still gets a second channel, the root-plate disc). Keeping
     the old high-retention numbers with the tint removed is not a neutral
     choice that merely reverts to STREAM T5's compromise -- it is a
     strictly WORSE point in the design space than either STREAM T5
     (bare + tinted) or the textbook attempt this file's own history
     records (bare + green, before the tint existed at all): full green
     crowns with no colour cue read as no damage at all.

REFERENCE: `vegetation._PLAN`'s two-band shape, `(keep_base, keep_top, ...)`
for fire, borrowed for its REASONING, not its fields (no browning/scorch
here -- leaves stay green, see below). Two ideas carry over:

  (a) A crown is not a single dial. Fire's `keep_base` (near the ground,
      where flame enters the canopy) and `keep_top` (the exposed crown
      apex) are independently tunable because a uniform thin reads as "an
      unhealthy tree," not a burnt one (`vegetation.defoliate`'s own
      docstring). A hurricane crown is no different: an evenly-thinned 14%
      keep looks like noise scattered through the whole silhouette, while
      the SAME 14% concentrated where an aerial camera's line of sight
      actually resolves canopy -- the crown's OUTER, UPPER envelope, not
      its shaded, occluded interior -- reads as an intact, thinned crown
      shape rather than as scattered noise or as nothing at all.
  (b) NEAR-ZERO IS NOT ZERO. `torched`'s `keep_top` is 0.06, not 0.00,
      "because a crown that goes to exactly zero looks deleted rather than
      burnt" (`vegetation._PLAN`'s own comment). Every band below re-cut by
      this stream keeps that same floor logic: `_LIMB_SECTOR_KEEP_IN` (the
      inside of `limbed`'s stripped sector) moves to 0.02-0.06, matching
      `torched`'s own 0.00-0.06 almost exactly, and no other band is
      allowed to hit literal zero either.

WHERE FIRE'S SHAPE IS *NOT* COPIED. Fire's `keep_base` is the DAMAGED band
(flame enters low) and `keep_top` the SURVIVING one. A hurricane crown does
not have a botanically privileged "top always survives" story the way a
rising flame front does -- what actually matters for THIS scene is which
part of the crown an overview camera can still SEE, and that is the outer/
upper envelope regardless of which part a real storm would strip first.
So rather than add a literal `(keep_base, keep_top)` PAIR to `_FOL_KEEP`
(which would also silently change its type from float to tuple and break
every direct-value comparison and `_select_kept_cards(inv, bht._FOL_KEEP[
...], "low_inner", rng)` call in `test_hurricane_trees.py` -- a file this
stream does not own or touch), `_FOL_KEEP` STAYS A SINGLE FLOAT PER LEVEL
(the whole-crown target fraction, same contract `_select_kept_cards` has
always had) and the two-band IDEA is expressed instead through a NEW
selection mode, `"top"` (`_select_kept_cards`'s `mode` argument), which
biases survival toward high crown-normalised z exactly the way fire's
`keep_top` band does -- the number stays a scalar target, the SHAPE of
which cards survive it is what borrows fire's asymmetry.

THE NUMBERS. `defoliated` -> 0.14 (86% loss, the LOW end of the cited
86-94% Cat-3 broadleaf survey band -- the least severe, most numerous
level gets the most generous point in the sourced range, not the most
severe). `limbed` -> 0.09, `leaning` -> 0.07, `fallen` -> 0.05, snapped's
`SNAP_TOP_FOL_KEEP` -> 0.03: each rung is BARER than the one below it,
extending past the survey's 94%-loss ceiling for the levels that are more
than plain leaf-loss (limbed has also lost limbs, leaning and fallen are
structural failures, snapped is an outright break) -- "increase [culling]
as intensity increases" done the only way an archetype baked per (species,
level) CAN do it: since a higher-intensity draw already walks further up
`hurricane.TREE_LEVELS` (`tree_level_for_intensity`), making each level's
OWN retention properly reflect its OWN severity is what makes a
higher-intensity plate automatically read barer -- there is no per-instance
knob to turn, and none is needed. THIS ALSO RESTORES THE MONOTONIC LADDER
STREAM T7 DELIBERATELY BROKE (`leaning` no longer keeps more than `limbed`/
`fallen`): with colour gone, "each rung barer than the last" is a directive
in its own right, not merely a test invariant, and it now takes priority
over T7's "leaning carries its signal through the disc instead" argument --
the disc stays (it is untouched, separate geometry, `_author_root_plate_
disc`), it is just no longer used as licence to keep MORE crown than a less
severe level.

THE VISIBILITY RISK THIS STREAM KNOWINGLY RE-ACCEPTS, WITH NUMBERS. The
file's own history measured a green, untinted, ~10-12%-kept `defoliated`
crown as invisible at a 400 m overview altitude before any tint existed at
all: `tree_American_Beech_defoliated.usd` had 122,862 visible mesh faces
and still topped out at 5.9 m, and `tree_Black_Oak_defoliated.usd`'s 83k
woody faces plus only 1,547 surviving leaf cards read as "a few branch-
width pixels -- indistinguishable from empty ground." 0.14 sits just above
that measured failure point and the selection mode changed (`"top"`
instead of `"low_inner"` -- the earlier failure buried its survivors in the
shaded, low/inner crown, which is arguably PART of why they vanished; this
stream deliberately puts the surviving cards where the camera looks
instead), but neither of those is a proof the new numbers read as trees
rather than as bare poles from altitude -- that is a render-time question
this offline bake cannot answer, and it is the single most important thing
for the next reviewer to check on the live plate. `limbed`/`leaning`/
`fallen`/`snapped`, each barer still, carry MORE of that same risk, not
less -- see this file's own report on the bake for the exact per-level
card counts.

`test_hurricane_trees.py` IS NOT UPDATED BY THIS STREAM (out of scope --
this file owns exactly `tools/bake_hurricane_trees.py`). It still asserts
the STREAM T5/T7 bands (`test_foliage_retention_matches_directive_bands`,
`test_defoliated_keeps_70_to_85_percent_of_cards`,
`test_defoliated_bias_favours_lower_inner_crown`,
`test_retention_table_matches_stream_t5_directive`) and WILL fail against
the numbers below until whoever owns that file re-cuts it to match this
directive -- the same way STREAM T7's docstring above records that ITS
retune required a matching test update, one was simply not in scope here.

Usage:
    python3 scene_gen/tools/bake_hurricane_trees.py [--out DIR] [--dry-run]
"""

import argparse
import json
import math
import os
import random
import re
import sys

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

import numpy as np  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Mirrors `suburb_hurricane_launch_script.TREE_SPECIES`. Kept as repo-relative
# paths because the ARCHETYPE REFERENCES THEM RELATIVELY (see `_relpath`): the
# file is authored on a laptop and consumed inside a container at a completely
# different absolute root, and a relative asset path is the only form that
# resolves in both without a second copy of the asset.
TREE_SPECIES = {
    "Black_Oak": "scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# CONIFERS KEEP THEIR NEEDLES. The sourced line in build-hurricane-scenes is
# that broadleaf canopy is stripped essentially bare at Cat 3+ (86-94% leaf
# loss) while conifers hold most of theirs — a needle presents far less sail
# area per unit of stem than a broadleaf blade. Stripping a fir as hard as an
# oak is the single most obvious way to make a mixed stand look wrong.
CONIFERS = ("Douglas_Fir",)

# The source assets are authored in CENTIMETRES; every baked archetype in
# `archetypes_hurricane/` is referenced by the launcher at `scale = 1.0` and
# must therefore already be in METRES. This is the same trap the launcher
# documents at its own `scale = 0.01` fallback — mixing the two puts a tree a
# hundred times too big in the scene and the overview camera ends up inside
# the bark.
CM_TO_M = 0.01

# All levels the launcher (`hurricane.TREE_LEVELS`) can ask for. `pristine`
# is now baked too — see the module docstring's cotton-ball section.
LEVELS = ("pristine", "defoliated", "limbed", "leaning", "fallen", "snapped")

# (keep_wood, lean_deg, lift_m) for the non-`snapped` levels. Foliage
# retention is handled separately (`_FOL_KEEP` / `_FOL_MODE`) because it now
# varies in KIND (weighted vs random) as well as amount.
#
# TUNED, NOT SOURCED, except the foliage-loss fractions (see `CONIFERS` and
# `_FOL_KEEP`).
_WOOD_PLAN = {
    "pristine": (1.00, 0.0, 0.0),
    "defoliated": (1.00, 0.0, 0.0),
    # Leaves gone AND a third of the woody crown gone with them — the next
    # rung up, where the wind starts taking limbs rather than just foliage.
    "limbed": (0.66, 4.0, 0.0),
    # Root plate partly failed. Still attached, still standing, visibly out
    # of plumb. 22 deg reads clearly from altitude without looking felled.
    "leaning": (0.88, 22.0, 0.0),
    # Windthrown. Not 90: a fallen tree rests on its own crown and root
    # plate, so it lies a few degrees off the ground rather than flat on it.
    #
    # `lift` STAYS 0 FOR EVERY LEVEL, and that is deliberate. Rotating a tree
    # about its base is what keeps the trunk PLANTED at z=0, which is the
    # thing a viewer reads; but a rigid rotation also swings half of a 25 m
    # crown below grade (measured: the 84 deg bake bottoms out at -10.7 m).
    # Lifting the asset until that bbox minimum cleared the ground would put
    # the root plate ten metres in the air — trading a hidden problem for the
    # single defect `bake-tree-and-debris` spends a whole catalogue on. The
    # buried half of the crown is inside the terrain and cannot be seen; the
    # planted base can. So: keep the base, let the ground hide the rest.
    "fallen": (0.92, 80.0, 0.0),
}

# Foliage retained on the STANDING (non-`snapped`) levels, and how it is
# chosen. RE-CUT 2026-09-01 (STREAM T8) — see the module docstring's own
# STREAM T8 section for the full reasoning. Superseded here: STREAM T5's
# "art-directed for air-visibility, not fit to a leaf-loss survey" ceiling
# and STREAM T7's non-monotonic `leaning > limbed/fallen` ladder. Both were
# built around a brown `diffuse_tint` that was meant to carry most of the
# damage signal at high retention; that tint is gone (leaves stay green —
# see `TREE_LEAF_TINT` below), so RETENTION IS NOW THE ONLY DAMAGE CHANNEL
# for every band except `leaning` (root-plate disc). Each band below is
# picked toward the cited 86-94% Cat-3 broadleaf leaf-loss survey figure,
# barer at each rung, restoring the monotonic ladder:
#
#   defoliated  0.14 kept (86% loss) -- the LOW-SEVERITY end of the survey
#               band, deliberately: the WIDEST band in `hurricane.
#               _TREE_CUTS` and the level most trees land in (720/1684 on
#               the reference plate), so it is both the level most
#               responsible for "too many trees still have leaves" and the
#               one that most needs to still read as a TREE, not a pole,
#               once cut. Selection mode `"top"` (see below).
#   limbed      0.09 kept (91% loss) -- barer than `defoliated` (lost limbs
#               as well as leaves) but this is a TARGET, not a value
#               `_select_kept_cards` draws against directly: `limbed` still
#               goes through `_select_kept_cards_sector` (STREAM T7,
#               unchanged), which splits it into a seeded azimuth SECTOR
#               stripped to `_LIMB_SECTOR_KEEP_IN` (now 0.02-0.06, moved
#               down to match) and the rest of the crown solved so the
#               whole-crown average lands on 0.09, plus the stub/debris
#               geometry `_author_limb_break` already added.
#   leaning     0.07 kept (93% loss) -- barer than `limbed` again, restoring
#               the monotonic ladder STREAM T7 broke (see the module
#               docstring's STREAM T8 section for why that inversion no
#               longer wins now that colour cannot pick up the slack). The
#               root-plate disc (`_author_root_plate_disc`) is untouched and
#               still carries a second, non-foliage damage cue here.
#   fallen      0.05 kept (95% loss) -- barer still: fully down, no story
#               left to tell except "almost nothing survived the fall."
#               Also gets the root-plate disc.
#
#   "top":       STREAM T8. Bias the kept subset toward HIGH crown-
#                normalised z — borrowed from `vegetation._PLAN`'s
#                `keep_top` idea: an overview camera's line of sight
#                resolves the crown's outer/upper envelope, not its
#                shaded interior, so concentrating the thin surviving
#                remnant there is what keeps a heavily-cut crown reading as
#                a THINNED TREE rather than as scattered noise or nothing.
#                Replaces the STREAM T5 `"low_inner"` bias (still available
#                in `_select_kept_cards` but no longer used by any level
#                here) for exactly the levels `"low_inner"` used to cover.
#   "random":    plain seeded random subset (used for `fallen`, where the
#                story is "went over before it could be stripped", not
#                "stripped selectively" — a toppled crown has no clean
#                top/bottom envelope left for `"top"` to bias toward).
_FOL_KEEP = {
    "pristine": 1.00,
    "defoliated": 0.14,
    "limbed": 0.09,
    "leaning": 0.07,
    "fallen": 0.05,
}
_FOL_MODE = {
    "pristine": None,
    "defoliated": "top",
    "limbed": "top",  # unused in practice — `limbed` goes through
                       # `_select_kept_cards_sector` instead (see `bake_one`),
                       # kept here only so the dict stays total over LEVELS
                       # and documents what the (dead) fallback would be.
    "leaning": "top",
    "fallen": "random",
}

# Conifers keep most of their needles at every damaged level — sourced: a
# needle's sail area per unit of stem is much smaller than a broadleaf
# blade's, so the SAME wind strips a fir far more slowly. Applied as a
# max() over `_FOL_KEEP`, exactly as the original single-band bake did.
# LEFT UNCHANGED BY THE STREAM T8 BROADLEAF RE-CUT ABOVE ("as before" per
# that stream's own directive, same as STREAM T5's before it) — but its
# practical effect has flipped since the broadleaf numbers dropped so far
# below it: EVERY damaged broadleaf base (0.05-0.14) now sits WELL below
# this floor, so `max(base, conifer)` overrides the broadleaf number
# completely at every level, not just `leaning`/`fallen` as under STREAM T7.
# A conifer therefore keeps 50-62% of its needles at every damaged level
# regardless of how bare the broadleaf ladder gets — which is the sourced,
# intended behaviour (a fir should not read as stripped the way an oak
# does), but is flagged here because it means Douglas_Fir is NOT part of
# this stream's "too leafy" fix at all; if firs still look too intact on
# the live plate after this bake, this floor — not `_FOL_KEEP` — is where
# to look.
_CONIFER_FOL_KEEP = {"limbed": 0.50}
_CONIFER_FOL_KEEP_DEFAULT = 0.62

# TARGET linear-space colours (STREAM T6), NOT multiply tints -- see the
# module docstring's "THE TINT WAS A DEAD KNOB" section. These are what the
# REPLACEMENT OmniPBR's `diffuse_tint` is now computed TO REACH: `texel_mean
# * diffuse_tint ~= TARGET`, per `_hue_flip_tint`. Two families, matching
# the STREAM T5 design this replaces: broadleaf residue reads as dead/dying
# leaf litter; a conifer's browning is a paler, less saturated version of
# green rather than a colour change, since the needles are thinned/stressed
# but mostly still needles.
# RENDER-CALIBRATED 2026-09-01: the (0.30,0.17,0.06) targets rendered as
# SALMON-PINK sRGB (156,127,104) under the storm dome (FINAL7 measured, 91
# crown samples) -- scene lighting lifts the albedo well above the swatch
# prediction. Correction factor (target/rendered in linear) = (0.726,
# 0.601, 0.351), landing dead-leaf brown sRGB ~(135,100,62) in the render.
# LEAVES STAY GREEN BY DEFAULT (2026-09-01, user, after seeing the tinted
# bake on the live 500 m plate): "why are the tree leaves brown then? Keep
# them as green. Damaged trees don't turn brown just cause of the
# hurricane."
#
# That is correct, and the tint was never asked for. A tree stripped,
# limbed, leaned or snapped by a few hours of hurricane wind has LOST
# foliage — the damage is mechanical, and the leaves still on it are the
# same living green they were that morning. Browning is a response over
# days to weeks (salt burn, desiccation, root damage), not something a storm
# does while it is passing. The scene's own epoch is hours after landfall.
#
# Set HUR_TREE_TINT=1 to restore the dead-foliage colouring for a scene that
# deliberately depicts a later epoch; everything below it is kept intact for
# that case. Note the tint is baked INTO the archetypes, so changing this
# requires re-running this script — see the module docstring.
TREE_LEAF_TINT = os.environ.get(
    "HUR_TREE_TINT", "0").strip() not in ("", "0", "false", "False")

# THE "LEAVE IT GREEN" TARGET, and why it is a sentinel not `None`.
#
# `tint=None` is the obvious way to switch the colouring off and it is
# WRONG: `_author_foliage_cull` guards BOTH the replacement-material
# creation AND the `Bind()` behind `if tint is not None`, so None skips
# the BINDING too and leaves foliage-card meshes with NO MATERIAL BOUND.
# Measured with `hurricane_tree_audit.py`: 0 binding faults with the tint
# on, 27 with `tint=None`. An unbound mesh renders flat default grey —
# worse than the colour it was meant to fix, and exactly the class the
# `freeze-portable-scenes` skill catalogues.
#
# So the whole replacement path still runs — same OmniPBR shader, same
# diffuse texture, same rebinding — and only the hue SOLVE is bypassed:
# `diffuse_tint` is authored neutral and the leaf keeps its own green.
NEUTRAL_TINT = "__neutral__"
# Linear-space mean of this pack's own leaf textures, used ONLY for the
# no-texture fallback branch (a material with no diffuse map still needs
# a constant, and under NEUTRAL_TINT that constant must be a leaf green
# rather than the dead-foliage target).
NEUTRAL_FALLBACK_RGB = (0.102, 0.170, 0.012)

TARGET_BROADLEAF = (0.218, 0.102, 0.021)
# the severed crown: a shade more weathered than the standing damaged levels
TARGET_BROADLEAF_TOP = tuple(round(c * 0.90, 6) for c in TARGET_BROADLEAF)
TARGET_NEEDLE = (0.145, 0.084, 0.021)
TARGET_NEEDLE_TOP = tuple(round(c * 0.90, 6) for c in TARGET_NEEDLE)
# Per-channel `diffuse_tint` is capped at this multiplier -- see
# `_hue_flip_tint`'s docstring for why a per-channel-independent cap alone
# is not always sufficient to guarantee a brown result, and what corrects it
# when it is not.
HUE_FLIP_CAP = 3.0

# --------------------------------------------------------------------------
# `snapped` — a real geometric snag.
# --------------------------------------------------------------------------

# Break height as a fraction of total tree height. A seeded per-species draw
# inside this band, not one fixed number, so six snags on a plate are not
# clones of each other at different lean.
SNAP_BREAK_FRAC = (0.30, 0.50)
# The severed top rotates this many degrees about the break point. Not 90:
# same reasoning as `fallen` above (a felled crown does not lie flat) --
# scaled down some, since a snapped top is a much shorter, stubbier mass
# than a whole fallen tree and does not need as much lean-band bisection to
# clear the ground; measured (see `test_hurricane_trees.py`) that 80-85 deg
# is enough for every species' lowest point to land in the turf, not under
# it, without needing `tip_tree`'s full bisection machinery.
SNAP_TOP_TILT_DEG = (80.0, 85.0)
# Foliage kept on the severed top -- some residue, tinted, per the module
# docstring; RAISED 2026-08-31 (STREAM T5) from 0.20 to 0.35, LOWERED
# 2026-09-01 (STREAM T7) to 0.25, LOWERED AGAIN 2026-09-01 (STREAM T8) to
# 0.03 alongside the rest of the ladder's re-cut (see the module docstring's
# STREAM T8 section) -- `fallen` (its nearest standing counterpart) dropped
# to 0.05 in the same re-cut, and `snapped` is the one level with an actual
# GEOMETRIC break, not just a pose/colour change, so its residue must still
# read as the thinnest of every damaged level's foliage ("went over only
# just, at the very end") now that `fallen` is barer too. 0.03, not 0.00,
# for the same reason every other band keeps a non-zero floor: fire's
# `torched.keep_top` (0.06) never goes to exactly zero either — "a crown
# that goes to exactly zero looks deleted rather than burnt."
SNAP_TOP_FOL_KEEP = 0.03
# Woody prims (secondary branch instancers, not the clipped trunk itself)
# above the break are mustered onto the top piece at this fraction; a snag's
# canopy is not the tidy, near-complete crown of a `leaning` tree.
SNAP_TOP_WOOD_KEEP = 0.55
# A secondary woody/foliage item this far (cm) from the trunk axis is
# dropped from the snag entirely rather than carried onto the rotated top --
# see `bake_snapped`'s "THE SAME LEVER-ARM PROBLEM" comment.
_SNAP_RADIAL_CAP_CM = 250.0

# --------------------------------------------------------------------------
# STREAM T7 (2026-09-01) — constructed break-debris geometry: bare limb
# stubs + their fallen counterparts (`limbed`), a torn root-plate disc
# (`leaning`/`fallen`), a jagged break collar (`snapped`). See the module
# docstring's own STREAM T7 section for why. All authored directly with
# `pxr`/numpy, exact and reproducible -- no physics, matching the rest of
# this file's "no solver needed" design (see the module docstring's own
# "WHY IT IS NOT THE TORNADO BAKE" section).
# --------------------------------------------------------------------------

# `limbed`'s stripped sector: seeded centre azimuth (deg, full circle) and
# half-width -- 90-120 deg total, "one side of the crown", not a quadrant
# and not a hemisphere.
_LIMB_SECTOR_HALF_DEG = (45.0, 60.0)
# Card retention INSIDE the stripped sector -- RE-CUT 2026-09-01 (STREAM T8)
# from (0.15, 0.30) to (0.02, 0.06), matching `vegetation._PLAN["torched"]`'s
# own `keep_top` band (0.00-0.06) almost exactly: the inside of the strip is
# THIS level's "near-zero but not exactly zero" band (see the module
# docstring's STREAM T8 section), a few scraps rather than the 15-30% real
# chunk it used to be. The OUTSIDE-sector retention is not a constant: it is
# SOLVED per tree so the sector-weighted average lands on `_FOL_KEEP
# ["limbed"]` (now 0.09 -- see `bake_one`'s call site), which is what makes
# "keep the rest fuller" true regardless of how large the sector happens to
# land relative to the whole crown. Checked against the new, much lower
# target: at the sector's widest (120 deg, ~1/3 of the crown) and
# `_LIMB_SECTOR_KEEP_IN`'s highest draw (0.06), the solve still clears
# `keep_in` (0.09 - 0.33*0.06)/(1-0.33) ~= 0.105 > 0.06) at every combination
# sampled -- the outside band never degenerates to equal the inside one via
# the `max(keep_in, keep_out)` clamp below.
_LIMB_SECTOR_KEEP_IN = (0.02, 0.06)
# Broken limb stubs still attached to the tree, jutting from the stripped
# sector -- children of `/Root/src`, authored in the SOURCE's own cm frame
# (see `_author_limb_break`) so the tree's existing cm->m + lean transform
# places them for free. "Short": well under `_STREAM_T7_FALLEN_LIMB_LEN_M`,
# a stub is what is LEFT on the tree, not what came off it.
_STUB_LEN_CM = (60.0, 140.0)
_STUB_THICK_CM = (7.0, 15.0)
# Their fallen counterparts, on the ground -- children of `/Root` directly,
# authored in METRES with no inherited transform, so "seated at z=0 in
# archetype space" is exact regardless of `limbed`'s own small (4 deg)
# residual lean (see `_WOOD_PLAN["limbed"]`) rather than something that has
# to be solved back out of it.
_FALLEN_LIMB_LEN_M = (2.0, 4.0)
_FALLEN_LIMB_THICK_M = (0.10, 0.22)

# `leaning`/`fallen`'s root-plate disc radius and tilt (deg off horizontal —
# see `_author_root_plate_disc`'s `tilt_deg`). `leaning` is a PARTIALLY
# sprung plate (still mostly root-anchored, so smaller and less vertical);
# `fallen` is fully torn out (bigger, closer to standing on edge). Tuned
# against the tornado skill's own `vegetation.root_plate` precedent
# (default `tilt_deg=68`), not re-derived from first principles.
_ROOTPLATE_R_M = {"leaning": (1.5, 2.5), "fallen": (1.8, 3.0)}
_ROOTPLATE_TILT_DEG = {"leaning": (55.0, 68.0), "fallen": (68.0, 80.0)}

# `snapped`'s break-plane collar: a handful of small bark spikes ringing the
# cut, radiating outward and a little upward -- "a snag reads by its
# splintered spar, not a clean cut" (the fire skill's spar language, see the
# module docstring). Short and thin: these dress the RIM of the break, they
# are not a second population of fallen debris.
_COLLAR_N = (3, 5)
_COLLAR_LEN_CM = (18.0, 34.0)
_COLLAR_THICK_CM = (4.0, 9.0)


def _relpath(target_abs, from_file_abs):
    """Asset path for `target_abs` as seen from the layer at `from_file_abs`."""
    return os.path.relpath(target_abs, os.path.dirname(from_file_abs))


# Substrings of a bound MATERIAL name that mean foliage. `needle` is not
# optional: Douglas_Fir binds `Pine_needles`, and a `leaf`-only test reported
# that tree as 0% foliage / 100% wood — defoliating it would have been a
# silent no-op and `limbed` would have culled its needled boughs at random as
# though they were bare limbs. Every other species in the pool happens to say
# `leaf`, which is exactly why a one-word test looked correct.
_FOLIAGE_WORDS = ("leaf", "leaves", "needle", "frond", "foliage")


def _is_foliage(prim):
    """True if `prim` is a leaf/needle card rather than wood.

    Decided by the BOUND MATERIAL's name, because the prim names do not
    separate the two — see this module's docstring. A mesh with no bound
    material, or one whose material matches nothing here, is treated as
    WOOD: in the assets surveyed that is a small untextured branch stub
    (`Default_Material`, 99 points), and the failure mode of guessing wrong
    in this direction (a stub survives defoliation) is invisible, where the
    other direction deletes structure.
    """
    if prim.GetTypeName() != "Mesh":
        return False
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[:2]
    if not mat:
        return False
    name = mat.GetPath().name.lower()
    return any(w in name for w in _FOLIAGE_WORDS)


def _bound_material(prim):
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[:2]
    return mat


def _classify(src_stage):
    """`(foliage, woody)` — lists of prim paths, relative to the default prim.

    A `PointInstancer` is reported by the path of the INSTANCER, not of its
    prototype: hiding a prototype mesh that is scattered by an instancer is
    not reliably honoured, and the instancer is the thing that actually puts
    geometry in the scene. An instancer counts as foliage when any prototype
    under it is a leaf card.
    """
    default = src_stage.GetDefaultPrim()
    if not default:
        raise SystemExit("source has no default prim")
    root_path = default.GetPath()
    foliage, woody, seen_instancers = [], [], set()
    for prim in Usd.PrimRange(default):
        path = prim.GetPath()
        if path == root_path:
            continue
        # attribute a mesh to its enclosing instancer, if it has one
        inst = None
        for anc in path.GetAncestorsRange():
            if anc == root_path:
                break
            if src_stage.GetPrimAtPath(anc).GetTypeName() == "PointInstancer":
                inst = anc
        if prim.GetTypeName() == "Mesh":
            rel = (inst or path).MakeRelativePath(root_path)
            leaf = _is_foliage(prim)
            if inst is not None:
                if inst in seen_instancers:
                    continue
                if leaf:
                    seen_instancers.add(inst)
                    foliage.append(rel)
                    continue
                # a woody instancer: remember it so we don't add it twice
                seen_instancers.add(inst)
                woody.append(rel)
                continue
            (foliage if leaf else woody).append(rel)
    # WOODY IS RETURNED LARGEST-FIRST so `_hidden_set` (and the `snapped`
    # clip) can identify the trunk. In these assets the trunk and every
    # woody limb are frequently ONE mesh (Black_Oak's `_base` is 41k points;
    # Aspen's `trunk` is 90k), so the biggest woody prim IS the tree's
    # structure.
    sizes = {}
    for rel in woody:
        prim = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
        n = 0
        for sub in Usd.PrimRange(prim) if prim else []:
            pts = sub.GetAttribute("points")
            if pts and pts.Get():
                n += len(pts.Get())
        sizes[rel] = n
    woody.sort(key=lambda r: -sizes.get(r, 0))
    return foliage, woody


def _cull(items, keep, rng, protect=0):
    """Hide a seeded, UNWEIGHTED subset of `items`, always keeping the first
    `protect` of them. See `bake_one`'s call sites for `protect`'s reason.
    """
    if keep >= 1.0:
        return []
    pool = list(items[protect:])
    if keep <= 0.0:
        return pool
    pool = sorted(pool)
    rng.shuffle(pool)
    n_hide = int(round(len(pool) * (1.0 - keep)))
    return pool[:n_hide]


def _cull_weighted(items, keep, rng, weight_of):
    """Hide a seeded subset of `items`, biased by `weight_of(item) -> float`
    (higher weight = more likely to be KEPT).

    Used for the `low_inner` foliage mode: `weight_of` returns a score built
    from -(z) and -(radius), so the surviving cards cluster toward the
    lower, inner crown rather than being spread uniformly (which is what a
    plain `_cull` would give it, and what a wind that strips outer canopy
    first would not).
    """
    if keep >= 1.0:
        return []
    if keep <= 0.0:
        return list(items)
    n_keep = int(round(len(items) * keep))
    scored = sorted(
        items,
        key=lambda it: (-(weight_of(it) + rng.uniform(-0.05, 0.05)), it),
    )
    keep_set = set(scored[:n_keep])
    return [it for it in items if it not in keep_set]


# --------------------------------------------------------------------------
# Per-CARD foliage culling.
#
# `_cull`/`_cull_weighted` above operate on whole foliage PRIMS -- fine for a
# species whose crown is several separate instancer groups (Black_Oak: 8),
# but three species in this pool (`Largetooth_Aspen`, `American_Beech`,
# `Common_Apple`) ship their ENTIRE crown as ONE plain mesh, and even the
# multi-instancer species often have individual leaf-instancer groups with
# only a handful of them (Shumard_Oak: 3). At whole-prim granularity,
# "keep 10%" rounds to 0 of 1 (fully bare — the tornado skill's documented
# failure mode for this exact asset shape) or 1 of 3 (33%, three times the
# target). Real leaf loss is a per-LEAF fraction, not a per-branch-group
# fraction, so this operates one level finer: a PointInstancer's individual
# INSTANCES (hidden via the standard `invisibleIds`), and a plain mesh's
# individual TRIANGLES (a leaf-card proxy — the exact card boundary is not
# recoverable without a UV-island pass, and two triangles per quad card is
# the common case for this asset family, but a "70-85% of cards" target (or
# any other `_FOL_KEEP` band) is a percentage and a percentage of triangles
# lands in the same place to well within the rounding this bake already
# tolerates elsewhere).
# --------------------------------------------------------------------------

def _foliage_inventory(src_stage, root_path, foliage_rel, xcache):
    """`{rel: {"kind": "mesh"|"instancer", "n": int, "z": arr, "r": arr}}`
    for every foliage prim, at per-card granularity.

    `xcache` (a `UsdGeom.XformCache` over `src_stage`) is REQUIRED so mesh
    centroids land in the same ROOT-RELATIVE frame the instancers' own
    `positions` are already in — several of these foliage meshes carry their
    own `xformOp:transform` (verified: Black_Oak's `branch3..6` all do), so
    comparing raw local centroids against rooted instancer positions would
    bias the global low/inner-crown ranking with whatever that mesh's own
    placement transform happens to be. See `_mesh_face_attrs`'s docstring
    for the concrete bug this caused before it was rooted.
    """
    inv = {}
    for rel in foliage_rel:
        prim = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
        tn = prim.GetTypeName()
        if tn == "PointInstancer":
            pi = UsdGeom.PointInstancer(prim)
            pos = pi.GetPositionsAttr().Get() or []
            pos = np.array(pos, dtype=np.float64).reshape(-1, 3) if len(pos) else np.zeros((0, 3))
            z = pos[:, 2] if len(pos) else np.zeros(0)
            r = np.hypot(pos[:, 0], pos[:, 1]) if len(pos) else np.zeros(0)
            # STREAM T7: azimuth about the trunk axis, for `limbed`'s
            # stripped-sector selection (`_select_kept_cards_sector`). Every
            # other mode ignores this key, so adding it here costs nothing
            # for `low_inner`/`random`.
            theta = np.arctan2(pos[:, 1], pos[:, 0]) if len(pos) else np.zeros(0)
            inv[rel] = {"kind": "instancer", "n": len(pos), "z": z, "r": r, "theta": theta}
        else:
            # a Mesh, possibly nested one level under an instancer path that
            # turned out to hold a single leaf mesh directly (some species'
            # `_classify` groups resolve straight to a Mesh) -- find it.
            mesh_prim = prim
            if tn != "Mesh":
                for sub in Usd.PrimRange(prim):
                    if sub.GetTypeName() == "Mesh":
                        mesh_prim = sub
                        break
            points, fvi, fvc, _ = _mesh_face_attrs(mesh_prim, xcache=xcache)
            ntri = len(fvc)
            if ntri and np.all(fvc == 3) and len(points):
                tri = points[fvi.reshape(ntri, 3)]
                centroid = tri.mean(axis=1)
                z = centroid[:, 2]
                r = np.hypot(centroid[:, 0], centroid[:, 1])
                theta = np.arctan2(centroid[:, 1], centroid[:, 0])
            else:
                z = np.zeros(max(ntri, 0))
                r = np.zeros(max(ntri, 0))
                theta = np.zeros(max(ntri, 0))
            inv[rel] = {"kind": "mesh", "n": ntri, "mesh_path": mesh_prim.GetPath(),
                        "z": z, "r": r, "theta": theta}
    return inv


def _select_kept_cards(inv, keep_fol, mode, rng):
    """Global (whole-crown) selection of which cards survive.

    Returns `{rel: kept_indices_array}` -- indices into that rel's own
    instance/triangle ordering. Weighting (for `mode in ("low_inner",
    "top")`) is normalised ACROSS THE WHOLE CROWN, not per-rel, so "lower/
    inner"/"top" means lower/inner or top in the tree as a whole rather
    than merely within whichever branch group a card happens to belong to.

    `mode == "top"` (STREAM T8, see `_FOL_KEEP`'s own comment): bias toward
    HIGH crown-normalised z only -- no radius term, unlike `"low_inner"` --
    mirroring `vegetation._PLAN`'s `keep_top` idea that an overview
    camera's line of sight resolves a crown's outer/upper envelope, not its
    interior, so a thin surviving remnant reads best concentrated there.
    `"low_inner"` is kept, unused by any level as of STREAM T8, as a general
    capability -- see `_FOL_MODE`'s own comment for why it was retired.
    """
    total = sum(v["n"] for v in inv.values())
    if total == 0:
        return {rel: np.zeros(0, dtype=int) for rel in inv}
    if keep_fol >= 1.0:
        return {rel: np.arange(v["n"]) for rel, v in inv.items()}
    if keep_fol <= 0.0:
        return {rel: np.zeros(0, dtype=int) for rel in inv}

    all_z = np.concatenate([v["z"] for v in inv.values()]) if total else np.zeros(0)
    all_r = np.concatenate([v["r"] for v in inv.values()]) if total else np.zeros(0)
    z_lo, z_hi = (float(all_z.min()), float(all_z.max())) if len(all_z) else (0.0, 1.0)
    r_lo, r_hi = (float(all_r.min()), float(all_r.max())) if len(all_r) else (0.0, 1.0)
    z_span = (z_hi - z_lo) or 1.0
    r_span = (r_hi - r_lo) or 1.0

    n_keep = int(round(total * keep_fol))
    keys = []      # (rel, local_index)
    scores = []
    for rel, v in inv.items():
        for i in range(v["n"]):
            keys.append((rel, i))
            if mode == "low_inner":
                zn = (v["z"][i] - z_lo) / z_span
                rn = (v["r"][i] - r_lo) / r_span
                scores.append(-(zn + rn) + rng.uniform(-0.05, 0.05))
            elif mode == "top":
                zn = (v["z"][i] - z_lo) / z_span
                scores.append(zn + rng.uniform(-0.05, 0.05))
            else:
                scores.append(rng.random())
    order = sorted(range(len(keys)), key=lambda i: -scores[i])
    kept_pairs = {keys[i] for i in order[:n_keep]}
    out = {rel: [] for rel in inv}
    for rel, i in kept_pairs:
        out[rel].append(i)
    return {rel: np.array(sorted(idx), dtype=int) for rel, idx in out.items()}


def _sector_fraction(inv, sector_center_deg, sector_half_deg):
    """Fraction of the WHOLE crown's cards (by count, pooled across every
    rel) whose azimuth falls within `sector_half_deg` of `sector_center_
    deg`. Used by `bake_one` to solve the outside-sector retention that
    makes a `limbed` tree's sector-weighted average land on `_FOL_KEEP
    ["limbed"]` regardless of how large the sector happens to be relative
    to the crown -- see `_select_kept_cards_sector`.
    """
    total = sum(v["n"] for v in inv.values())
    if total == 0:
        return 0.0
    c = math.radians(float(sector_center_deg))
    h = math.radians(float(sector_half_deg))
    n_in = 0
    for v in inv.values():
        if v["n"] == 0:
            continue
        d = np.abs(np.mod(v["theta"] - c + math.pi, 2.0 * math.pi) - math.pi)
        n_in += int(np.sum(d <= h))
    return n_in / total


def _select_kept_cards_sector(inv, keep_in, keep_out, sector_center_deg,
                              sector_half_deg, rng):
    """STREAM T7. Per-card Bernoulli keep with TWO independent target
    retention rates instead of `_select_kept_cards`' single global fraction
    ranked by score: `keep_in` inside the stripped azimuth sector
    (`|angular distance to sector_center_deg| <= sector_half_deg`),
    `keep_out` outside it. This is what makes "one side of the crown
    stripped" an actual asymmetry rather than a uniformly-thinned crown
    that merely happens to average to the same number -- `_select_kept_
    cards`' rank-and-take-top-N construction has no notion of WHERE a card
    is in azimuth, only in z/r, so it cannot produce this shape at any
    `mode`.

    Independent per-card draws (not a ranked top-N) because the two target
    rates are ranges, not an exact global count to hit -- see the module
    docstring's own STREAM T7 section for how `keep_out` is solved so the
    realised whole-crown average still lands on the directive target.
    """
    center = math.radians(float(sector_center_deg))
    half = math.radians(float(sector_half_deg))
    out = {}
    for rel, v in inv.items():
        n = v["n"]
        if n == 0:
            out[rel] = np.zeros(0, dtype=int)
            continue
        d = np.abs(np.mod(v["theta"] - center + math.pi, 2.0 * math.pi) - math.pi)
        in_sector = d <= half
        p = np.where(in_sector, float(keep_in), float(keep_out))
        draws = np.array([rng.random() for _ in range(n)])
        out[rel] = np.where(draws < p)[0]
    return out


def _author_foliage_cull(stage, src_stage, src_abs, out_path, root_path, inv,
                          kept, tint, xcache, prefix="/Root/src", species="",
                          report_tag="standing"):
    """Author the per-card cull decided by `_select_kept_cards` onto the
    OUTPUT stage: `invisibleIds` for an instancer, a filtered replacement
    mesh for a plain mesh. Returns `(n_kept, n_total, tinted_rels)`.

    The replacement mesh is authored with ROOT-RELATIVE points (via
    `_mesh_face_attrs(..., xcache=xcache)`) and placed as a plain sibling
    with NO xformOp of its own -- it must NOT reuse the original mesh's own
    local coordinates, because `prefix` (e.g. `/Root/src`) carries no
    counter-transform to put a locally-authored mesh back where the
    original's own `xformOp:transform` would have. See `_mesh_face_attrs`.

    `tint` (despite the name, kept for caller continuity) is now a TARGET
    linear-space colour (`TARGET_BROADLEAF`/`TARGET_NEEDLE`/their `_TOP`
    variants), not a multiply factor -- `None` for `pristine` (no
    replacement at all, the untouched original stays bound), else handed to
    `_omnipbr_leaf_material` (STREAM T6), which computes the actual
    per-material `diffuse_tint` FROM this target and that material's own
    texture. See the module docstring's "THE TINT WAS A DEAD KNOB" section.
    """
    n_kept_total = 0
    n_total = 0
    tinted_rels = []
    for rel, v in inv.items():
        n_total += v["n"]
        keep_idx = set(int(i) for i in kept.get(rel, []))
        n_kept_total += len(keep_idx)
        if v["n"] == 0:
            continue
        # `fully_kept`: this REL's own cards are 100% kept (nothing of ITS
        # OWN to hide), which can happen even when the GLOBAL keep fraction
        # across the whole crown is far below 1.0 -- `_select_kept_cards`
        # selects per-CARD across every rel pooled together, so a small rel
        # can land entirely inside the kept set purely by chance while the
        # crown as a whole is mostly culled. THE OLD CODE `continue`D HERE
        # UNCONDITIONALLY, which skipped tinting too, not just the hide step
        # -- any rel that happened to be wholly kept stayed bound to the
        # RAW, UNTOUCHED, GREEN original material at every damaged level,
        # identical in kind to the discarded-tint bug this function's own
        # instancer branch already had a comment about (see below). Not
        # reproduced by the CURRENT seeded bakes (verified: no rel in any
        # of the 34 shipped archetypes is currently wholly-kept-but-partial-
        # crown), which is exactly why it is a LATENT gap rather than a
        # currently-visible defect -- closed here so a future re-seed,
        # re-tune of `_FOL_KEEP`/`SNAP_TOP_FOL_KEEP`, or a new species
        # cannot silently reintroduce the cotton-ball class of bug through
        # this specific door.
        fully_kept = len(keep_idx) == v["n"]
        if v["kind"] == "instancer":
            over = None
            if not fully_kept:
                hide_ids = [i for i in range(v["n"]) if i not in keep_idx]
                over = stage.OverridePrim(Sdf.Path(prefix).AppendPath(rel))
                pi_over = UsdGeom.PointInstancer(over)
                if hide_ids:
                    pi_over.CreateInvisibleIdsAttr().Set(Vt.Int64Array(hide_ids))
            if keep_idx:
                tinted_rels.append(rel)
                # THE KEPT INSTANCES MUST GET THE TINTED MATERIAL TOO, NOT
                # JUST invisibleIds -- an instancer's whole population shares
                # ONE prototype, so rebinding the PROTOTYPE mesh is what
                # tints every surviving instance (the hidden ones don't
                # render regardless of what they're bound to). Before this
                # fix, `tinted_rels` was computed and returned but never
                # acted on: every species whose crown is a PointInstancer
                # (Black_Oak, Douglas_Fir, Shumard_Oak -- most of the pool)
                # kept its residual 8-30% of foliage cards fully GREEN and
                # UNTINTED at every damaged level, because nothing ever
                # rebound the prototype. Verified missing by direct
                # inspection: `tree_Black_Oak_defoliated.usd`'s kept
                # instances all still bound `/Root/src/Looks/
                # Shumard_Oak_leaf_Mat2` (the untouched, untinted original)
                # with no `tint_mats` material anywhere in the file.
                if tint is not None:
                    src_inst = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
                    for proto_path in UsdGeom.PointInstancer(
                            src_inst).GetPrototypesRel().GetTargets():
                        proto_prim = src_stage.GetPrimAtPath(proto_path)
                        if not proto_prim or not proto_prim.IsValid():
                            continue
                        pmat = _bound_material(proto_prim)
                        final_mat = _omnipbr_leaf_material(
                            stage, src_stage, src_abs, out_path, pmat, tint,
                            salt=rel.name, species=species, report_tag=report_tag)
                        proto_rel = proto_path.MakeRelativePath(root_path)
                        proto_over = stage.OverridePrim(
                            Sdf.Path(prefix).AppendPath(proto_rel))
                        UsdShade.MaterialBindingAPI.Apply(proto_over).Bind(final_mat)
            else:
                UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        elif fully_kept:
            # Nothing of this rel's own cards needs to be hidden -- but it
            # may still need TINTING (see `fully_kept`'s comment above: a
            # small rel can be wholly kept while the crown overall is
            # mostly culled). Rebind the ORIGINAL mesh's material in place,
            # directly at its own path under `prefix` -- no clipped "_cards"
            # duplicate is needed since every one of its triangles survives.
            mesh_prim = src_stage.GetPrimAtPath(v["mesh_path"])
            mesh_rel = mesh_prim.GetPath().MakeRelativePath(root_path)
            tinted_rels.append(rel)
            if tint is not None:
                mat = _bound_material(mesh_prim)
                final_mat = _omnipbr_leaf_material(
                    stage, src_stage, src_abs, out_path, mat, tint,
                    salt=rel.name, species=species, report_tag=report_tag)
                mesh_over = stage.OverridePrim(Sdf.Path(prefix).AppendPath(mesh_rel))
                UsdShade.MaterialBindingAPI.Apply(mesh_over).Bind(final_mat)
        else:
            mesh_prim = src_stage.GetPrimAtPath(v["mesh_path"])
            points, fvi, fvc, face_attrs = _mesh_face_attrs(mesh_prim, xcache=xcache)
            ntri = len(fvc)
            mask = np.zeros(ntri, dtype=bool)
            if keep_idx:
                mask[list(keep_idx)] = True
            tri_idx = fvi.reshape(ntri, 3)
            corner_pos = points[tri_idx][mask]
            out_attrs = {n: np.asarray(a, dtype=np.float64).reshape(ntri, 3, -1)[mask]
                         for n, a in face_attrs.items()}
            # hide the original, full-crown mesh everywhere under this rel
            over = stage.OverridePrim(Sdf.Path(prefix).AppendPath(rel))
            UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
            if keep_idx:
                mat = _bound_material(mesh_prim)
                if tint is not None:
                    tinted_rels.append(rel)
                    final_mat = _omnipbr_leaf_material(
                        stage, src_stage, src_abs, out_path, mat, tint,
                        salt=rel.name, species=species, report_tag=report_tag)
                else:
                    final_mat = (mat if (mat and _material_ok(src_abs, mat))
                                 else _ensure_safe_material(stage, "leaf"))
                new_path = Sdf.Path(prefix).AppendPath(rel).GetParentPath().AppendChild(
                    rel.name + "_cards")
                _author_clipped_mesh(stage, new_path, corner_pos, out_attrs,
                                      material=final_mat)
    return n_kept_total, n_total, tinted_rels


# --------------------------------------------------------------------------
# Material safety net — "every visible prim binds a resolvable material".
# --------------------------------------------------------------------------

_TEX_RE = re.compile(r'texture_2d\(\s*"([^"]*)"')
_mdl_text_cache = {}
_material_ok_cache = {}


def _case_exact_exists(path):
    """True only if `path` exists AND every path component matches the
    on-disk case exactly.

    A plain `os.path.exists` would pass a case MISMATCH on a case-preserving-
    but-insensitive filesystem and only fail on a strictly case-sensitive
    one (which is what the render host actually is) -- checking the exact
    case here catches the fault class on any host, not only the one it
    happens to bite on.
    """
    if not os.path.isabs(path):
        return False
    cur = os.sep
    for part in [p for p in path.split(os.sep) if p]:
        try:
            entries = os.listdir(cur)
        except (FileNotFoundError, NotADirectoryError):
            return False
        if part in entries:
            cur = os.path.join(cur, part)
            continue
        return False
    return True


def _mdl_text(mdl_abs):
    if mdl_abs not in _mdl_text_cache:
        try:
            with open(mdl_abs, "r", errors="replace") as f:
                _mdl_text_cache[mdl_abs] = f.read()
        except OSError:
            _mdl_text_cache[mdl_abs] = ""
    return _mdl_text_cache[mdl_abs]


def _is_relative_asset_path(p):
    """True if the string inside an `Sdf.AssetPath` is a plain filesystem
    relative path -- i.e. something `os.path.join(some_dir, p)` can anchor.
    False for an absolute path AND for a URL-schemed one (`omniverse://...`,
    `http://...`): those are untouched by copying a spec between layers, so
    re-anchoring them would be both unnecessary and wrong.

    ALSO FALSE FOR A BARE FILENAME WITH NO DIRECTORY COMPONENT AT ALL
    (STREAM T6). One of Kit's own BUILT-IN MDL modules (`OmniPBR.mdl`, the
    only one this file ever authors -- see `_omnipbr_leaf_material`) is
    resolved through Kit's MDL search path, not anchored next to whatever
    asset happens to reference it; `os.path.join(src_dir, "OmniPBR.mdl")`
    does not exist on ANY host, even a perfectly correct one (measured:
    `Usd.Stage` itself reports an empty `resolvedPath` for it on a bare
    `pxr` with no Kit MDL resolver plugin loaded). Every GENUINE per-species
    custom `.mdl`/texture reference this asset family has ever shipped is
    written with an explicit `./...` directory prefix -- checked exhaustively
    (every leaf/needle/bark `.mdl`'s own `texture_2d(...)` calls and every
    Material-level texture override found anywhere in the six species this
    bake reads) -- so a bare, no-separator name is an unambiguous signal
    that it is a Kit built-in, not a guess that could misclassify a real
    asset reference.
    """
    if not p:
        return False
    if "://" in p:
        return False
    if os.path.isabs(p):
        return False
    if not os.path.dirname(p):
        return False
    return True


def _prim_asset_attrs(prim):
    """`[(attr, path_str)]` for every attribute directly authored on `prim`
    (not descendants) whose value is a relative `Sdf.AssetPath`.
    """
    out = []
    for a in prim.GetAttributes():
        if not a.HasAuthoredValue():
            continue
        v = a.Get()
        if isinstance(v, Sdf.AssetPath) and _is_relative_asset_path(v.path):
            out.append((a, v.path))
    return out


# A stock material with a saturated-red diffuse constant and NO texture is
# exactly the shape of defect this module's own safety net cannot see by
# checking "does the texture resolve" alone -- there is no texture to fail
# to resolve, the colour itself is simply wrong for bark or foliage (see
# build-hurricane-scenes' red-twig finding). Flagged here so `_material_ok`
# routes it through the same rebind path as an unresolvable texture.
def _is_red_flag_constant(rgb):
    r, g, b = float(rgb[0]), float(rgb[1]), float(rgb[2])
    return r > 0.35 and r > 1.6 * max(g, b, 0.001)


def _material_ok(src_abs, mat_prim):
    """True if every texture `mat_prim` (an MDL-backed material) resolves
    case-exact on THIS machine, AND no visible shader/material interface
    input is a red-flag constant with no texture. See the module
    docstring's cotton-ball section for why the texture check exists and
    what it cannot see (a different machine's copy of the same asset tree).

    CHECKS THE MATERIAL PRIM ITSELF, NOT ONLY ITS SHADER CHILDREN. A
    `Material` prim can carry its own INTERFACE-level Asset input that
    overrides its Shader's compiled-in MDL default -- measured on
    `Common_Apple`'s `Apple_leaf_Mat`: the MATERIAL prim (not the Shader)
    authors `inputs:diffuse_texture = ./materials/textures/
    hollyprivet_basecolor.png`, a value the Shader's own `Apple_leaf_Mat.mdl`
    never mentions at all (it points `diffuse_texture` at
    `./textures/apple_leaf_basecolor.png` instead). The ORIGINAL loop here
    only ever looked at `Shader`-typed prims (`if shader_prim.GetTypeName()
    != "Shader": continue`), so this exact override was invisible to it --
    on the untouched source asset it happens to resolve fine (relative to
    `Common_Apple/`), which is precisely why this bake's own `_material_ok`
    check waved it through as safe to copy, and the launcher's Hydra log
    then reported `References an asset that can not be found: './materials/
    textures/hollyprivet_basecolor.png'` 4 times once the copy re-anchored
    it to the archetype's own directory instead (see `_copy_tinted_
    material`'s `_reanchor_relative_assets` for the other half of this fix).
    """
    key = (src_abs, str(mat_prim.GetPath()))
    if key in _material_ok_cache:
        return _material_ok_cache[key]
    src_dir = os.path.dirname(src_abs)
    ok = True
    saw_shader = False
    for prim in Usd.PrimRange(mat_prim.GetPrim()):
        # Any prim's own directly-authored relative Asset inputs -- Material
        # interface overrides included, not just a Shader's own attributes.
        for attr, rel_path in _prim_asset_attrs(prim):
            tex_abs = os.path.normpath(os.path.join(src_dir, rel_path))
            if not _case_exact_exists(tex_abs):
                ok = False
        shader_prim = prim
        if shader_prim.GetTypeName() != "Shader":
            continue
        saw_shader = True
        shd = UsdShade.Shader(shader_prim)
        dc = shd.GetInput("diffuse_color_constant")
        dc_val = dc.Get() if dc else None
        tex = shd.GetInput("diffuse_texture")
        tex_val = tex.Get() if tex else None
        if dc_val is not None and not tex_val and _is_red_flag_constant(dc_val):
            ok = False
        a = shader_prim.GetAttribute("info:mdl:sourceAsset")
        if not (a and a.Get()):
            # A plain (non-MDL-subgraph) shader: its own asset inputs were
            # already covered by the generic `_prim_asset_attrs` pass above.
            continue
        if not _is_relative_asset_path(a.Get().path):
            continue  # absolute/URL sourceAsset -- can't check offline
        mdl_abs = os.path.normpath(os.path.join(src_dir, a.Get().path))
        if not _case_exact_exists(mdl_abs):
            ok = False
            continue
        mdl_dir = os.path.dirname(mdl_abs)
        for tex_rel in _TEX_RE.findall(_mdl_text(mdl_abs)):
            if not tex_rel:
                continue
            tex_abs = os.path.normpath(os.path.join(mdl_dir, tex_rel))
            if not _case_exact_exists(tex_abs):
                ok = False
    ok = ok and saw_shader
    _material_ok_cache[key] = ok
    return ok


_SAFE_LEAF_RGB = (0.24, 0.34, 0.12)
_SAFE_WOOD_RGB = (0.24, 0.19, 0.13)
# STREAM T7: dark, wet torn earth for the root-plate disc. Deliberately a
# constant colour rather than an imported soil texture, for the SAME reason
# every other fallback material here is a constant -- it cannot fail to
# resolve on any host, which is this whole file's material philosophy (see
# the module docstring's "THE TINT WAS A DEAD KNOB" / `_material_safety_net`
# sections). This is also one of the two materials the STREAM T7 directive
# names ("wet-mud or bark material"); a constant dark-soil colour is chosen
# over reusing the tree's own bark so the disc reads as EARTH, not as more
# wood, next to the bark-textured stubs/limbs it stands beside.
_SAFE_SOIL_RGB = (0.10, 0.07, 0.05)
_SAFE_ROUGHNESS = {"leaf": 0.75, "wood": 0.75, "soil": 0.92}


def _ensure_safe_material(stage, kind):
    """A tiny, texture-free OmniPBR defined directly in the OUTPUT stage --
    cannot fail to resolve on any host because it references no external
    asset at all. `kind` is "leaf", "wood" or "soil".
    """
    path = Sdf.Path("/Root/safe_mats/{0}_safe".format(kind))
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        return UsdShade.Material(prim)
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    # THE MDL SOURCE, NOT `info:id`. `CreateIdAttr("OmniPBR")` alone is an
    # UNKNOWN shader to Kit's Hydra delegate and renders as the INVALID-
    # SHADER RED (this was the "bright red twigs" of ROUND1 and the red
    # root-plate discs of FINAL9 -- every safety-net/safe material was
    # affected). Mirror `_omnipbr_leaf_material`'s proven authoring.
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateIdAttr("OmniPBR")
    rgb = {"leaf": _SAFE_LEAF_RGB, "soil": _SAFE_SOIL_RGB}.get(kind, _SAFE_WOOD_RGB)
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*rgb))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(
        _SAFE_ROUGHNESS.get(kind, 0.75))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _remap_material(stage, mat, root_path, prefix):
    """`mat` was resolved on the SOURCE stage (`_bound_material` called on a
    `src_stage` prim); its `GetPath()` is therefore in the SOURCE's own
    namespace (e.g. `/Root/Looks/bark3`), not the OUTPUT stage's, where the
    same material lives one level deeper because of the `<prefix>` reference
    wrapper (e.g. `/Root/src/Looks/bark3`). Binding the SOURCE-stage
    material object directly authors a relationship target at the
    UNPREFIXED path, which resolves to nothing on the composed output stage
    -- a material binding to a path that does not exist, i.e. no material at
    all, caught by `test_hurricane_trees.py::
    test_every_visible_mesh_has_a_resolvable_material` reporting a blank
    bound-material path on exactly the `snapped` archetypes' clipped trunk
    pieces. Returns `None` if `mat` is falsy or does not resolve at the
    remapped path (should not happen for anything reached through
    `root_path`, but never bind to an invalid prim either way).
    """
    if not mat:
        return None
    rel = mat.GetPath().MakeRelativePath(root_path)
    new_path = Sdf.Path(prefix).AppendPath(rel)
    prim = stage.GetPrimAtPath(new_path)
    if not prim or not prim.IsValid():
        return None
    return UsdShade.Material(prim)


def _composed_material_resolves(mat):
    """True if every relative `Sdf.AssetPath` authored anywhere under `mat`
    (a `Material` already placed on the COMPOSED OUTPUT stage) resolves on
    THIS machine, per the LIVE stage's own `resolvedPath` -- not a hand-
    reconstructed `os.path.join` guess the way `_material_ok` checks the
    SOURCE side before anything has been referenced or copied anywhere.

    WHY THIS EXISTS ON TOP OF `_material_ok`. `bake_snapped`'s severed top
    binds the bark material via `_remap_material` at a SECOND reference
    site (`/Root/top/src/...`, not `/Root/src/...`), and a reviewer flagged
    that a second reference *could* re-anchor a relative asset path
    differently from the first (it should not, in plain USD reference
    composition -- a relative Asset value is anchored to the LAYER that
    authored the opinion, which for an untouched reference is still the
    original species file regardless of where it is referenced from -- but
    "should not" is an assumption, not a measurement). This is the
    measurement: it asks the actual composed stage what it resolved,
    exactly what `hurricane_tree_audit.py`'s whole-stage sweep also does,
    so a host where the assumption happens to be wrong is caught HERE,
    before the file is saved, rather than only by a downstream audit run.
    """
    if not mat:
        return False
    for prim in Usd.PrimRange(mat.GetPrim()):
        for attr in prim.GetAttributes():
            if not attr.HasAuthoredValue():
                continue
            v = attr.Get()
            vals = v if isinstance(v, (list, tuple)) else [v]
            for item in vals:
                if not isinstance(item, Sdf.AssetPath) or not item.path:
                    continue
                if not _is_relative_asset_path(item.path):
                    continue  # absolute/URL/built-in-MDL -- not ours to verify offline
                if not item.resolvedPath:
                    return False
    return True


def _material_safety_net(stage, src_abs, visible_rel_paths, root_path, src_stage,
                          prefix="/Root/src"):
    """For every prim under `prefix` named in `visible_rel_paths` that binds
    a material whose textures do not resolve on THIS machine, OR THAT BINDS
    NO MATERIAL AT ALL, rebind it to the texture-free safety material.
    Returns the count rebound.

    THE "NO MATERIAL AT ALL" CASE IS NOT HYPOTHETICAL -- it is measured,
    on the stock asset, not something this bake introduced:
    `Black_Oak_branch1_instancer`'s bark binding (`TreeBark_7`) is authored
    on the INSTANCER, but its prototype mesh `Black_Oak_branch1` carries no
    binding of its own, and `ComputeBoundMaterial` does not walk up to the
    instancer for it -- every "MaterialBindingAPI is not applied on the
    prim" warning this module's own bake run prints is a binding authored
    without the schema, which is exactly the shape of authoring defect the
    inheritance walk declines to trust. Whether Hydra's own resolution is
    more forgiving than the bare `UsdShade` API is precisely the kind of
    thing this project has already been burned by once: this repo's own
    bug catalogue records "every floor mesh in every baked archetype had NO
    MATERIAL BOUND, and rendered as a saturated blue slab" from an
    unrelated pass that never checked for a missing binding either. A mesh
    with no material at all is not "nothing to verify" -- it is the
    worst-resolving case there is, and `_bound_material` returning a
    technically-non-`None` but `bool(mat) is False` invalid `Material` for
    it is exactly the value an earlier, narrower `if not mat: continue`
    here quietly walked past.
    """
    n = 0
    for rel in visible_rel_paths:
        src_prim = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
        if not src_prim or not src_prim.IsValid():
            continue
        # the direct material carrier may be the prim itself (a mesh) or,
        # for a foliage/woody INSTANCER, its prototype child -- walk and
        # patch every mesh underneath.
        for sub in Usd.PrimRange(src_prim):
            if sub.GetTypeName() != "Mesh":
                continue
            mat = _bound_material(sub)
            if mat and _material_ok(src_abs, mat):
                continue
            kind = "leaf" if _is_foliage(sub) else "wood"
            safe = _ensure_safe_material(stage, kind)
            sub_rel = sub.GetPath().MakeRelativePath(root_path)
            over = stage.OverridePrim(Sdf.Path(prefix).AppendPath(sub_rel))
            UsdShade.MaterialBindingAPI.Apply(over).Bind(safe)
            n += 1
    return n


# --------------------------------------------------------------------------
# STREAM T6 -- OmniPBR foliage replacement (defoliated / limbed / leaning /
# fallen / snag top). See the module docstring's "THE TINT WAS A DEAD KNOB"
# section for why this replaces (rather than copies-and-tints) the source
# leaf/needle material.
# --------------------------------------------------------------------------

# `param: texture_2d("path", ...)` -- a NAMED constructor argument in one of
# this asset family's MDL files (every one is `mdl 1.4`, `export material
# X(*) = OmniPBR(param: value, ...)`). A bare `texture_2d()` with no string
# argument -- this family's "no texture" sentinel for a slot it does not
# bind (every leaf/needle mdl's `ORM_texture`, most of them for `normalmap_
# texture`/`reflectionroughness_texture` too) -- does not match, correctly:
# that parameter has no texture.
_MDL_PARAM_TEX_RE = re.compile(r'(\w+)\s*:\s*texture_2d\(\s*"([^"]*)"')

_texel_mean_cache = {}
# `(species, material_name, report_tag) -> stats dict` -- accumulated across
# a whole `main()` run (or a whole `_bake_dir()` test fixture run) for the
# per-species/per-material report `main()` prints and writes into
# `--stats-json`. Keyed by (species, material_name, report_tag), not by
# archetype file or salt, so re-baking a level that reuses the same source
# material does not duplicate a report row -- `report_tag` ("standing" for
# `bake_one`'s defoliated/limbed/leaning/fallen, "top" for `bake_snapped`'s
# severed crown) is its own key component because the TARGET differs
# between the two (`TARGET_BROADLEAF`/`_NEEDLE` vs their dimmer `_TOP`
# variants), so a single (species, material) key would have the "top" bake
# -- which always runs last per species, `snapped` being the final LEVELS
# entry -- silently overwrite the standing-crown numbers in the report.
_species_material_report = {}


def _mdl_param_textures(mdl_abs):
    """`{param_name: abs_texture_path}` for every `param: texture_2d("...")`
    call-site argument in the MDL text at `mdl_abs`.
    """
    mdl_dir = os.path.dirname(mdl_abs)
    out = {}
    for param, rel_path in _MDL_PARAM_TEX_RE.findall(_mdl_text(mdl_abs)):
        if not rel_path:
            continue
        out.setdefault(param, os.path.normpath(os.path.join(mdl_dir, rel_path)))
    return out


def _leaf_texture_paths(src_abs, mat_prim):
    """`{"diffuse": abs_or_None, "normal": abs_or_None}` for the textures a
    foliage/needle material actually shows, resolved case-exact on THIS
    machine.

    A prim-level interface OVERRIDE (a relative Asset input authored
    directly on the MATERIAL prim, anchored to `src_abs`'s own directory)
    wins over the compiled-in MDL default (anchored to the `.mdl` file's
    own directory) -- see `_material_ok`'s docstring on `Common_Apple`'s
    `Apple_leaf_Mat`, whose MATERIAL prim overrides `diffuse_texture` to
    `hollyprivet_basecolor.png`, a file its own `.mdl` never mentions (it
    names `apple_leaf_basecolor.png` instead). This is what the render
    actually shows, so it is what this replacement's own diffuse texture
    must be too, or the species' visual identity would silently change.

    An absolute/URL-schemed override is SKIPPED, never trusted -- measured:
    `Apple_leaf_Mat`'s own `normalmap_texture` override is an
    `omniverse://airlab-nucleus...` path into the AEC Nucleus tree the
    `aec-nucleus-path-moved` note already records as restructured, almost
    certainly broken on the current server. This function only ever
    returns a path it has verified exists, case-exact, on local disk.
    """
    result = {"diffuse": None, "normal": None}
    if not mat_prim:
        return result
    src_dir = os.path.dirname(src_abs)
    overrides = {}
    for prim in Usd.PrimRange(mat_prim.GetPrim()):
        for attr, rel_path in _prim_asset_attrs(prim):
            base = attr.GetName().rsplit(":", 1)[-1]
            overrides.setdefault(base, os.path.normpath(os.path.join(src_dir, rel_path)))
    mdl_defaults = {}
    for prim in Usd.PrimRange(mat_prim.GetPrim()):
        if prim.GetTypeName() != "Shader":
            continue
        a = prim.GetAttribute("info:mdl:sourceAsset")
        if not (a and a.Get()) or not _is_relative_asset_path(a.Get().path):
            continue
        mdl_abs = os.path.normpath(os.path.join(src_dir, a.Get().path))
        if not _case_exact_exists(mdl_abs):
            continue
        for param, tex_abs in _mdl_param_textures(mdl_abs).items():
            mdl_defaults.setdefault(param, tex_abs)
    for key, param in (("diffuse", "diffuse_texture"), ("normal", "normalmap_texture")):
        cand = overrides.get(param) or mdl_defaults.get(param)
        if cand and _case_exact_exists(cand):
            result[key] = cand
    return result


def _srgb_to_linear(c):
    c = np.asarray(c, dtype=np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _linear_to_srgb(c):
    c = np.clip(np.asarray(c, dtype=np.float64), 0.0, 1.0)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * (c ** (1.0 / 2.4)) - 0.055)


def _alpha_weighted_linear_mean(tex_abs):
    """`(mean_lin_rgb, alpha_min, alpha_max)` for the basecolor PNG at
    `tex_abs`, or `None` if it cannot be read.

    `alpha_min`/`alpha_max` are reported, not just used to weight the mean:
    every basecolor PNG this pool's foliage materials bind measures fully
    opaque (255 everywhere) -- these leaf/needle prims are real leaf-shaped
    GEOMETRY, not alpha-cutout cards, which is also why no opacity wiring is
    authored on the replacement OmniPBR (see `_omnipbr_leaf_material`'s and
    the module docstring's own note).
    """
    if tex_abs in _texel_mean_cache:
        return _texel_mean_cache[tex_abs]
    try:
        from PIL import Image
        im = Image.open(tex_abs).convert("RGBA")
        arr = np.asarray(im, dtype=np.float64)
    except Exception:  # noqa: BLE001 -- an unreadable texture is "no data"
        _texel_mean_cache[tex_abs] = None
        return None
    rgb = arr[..., :3] / 255.0
    a = arr[..., 3] / 255.0
    lin = _srgb_to_linear(rgb)
    wsum = float(a.sum())
    if wsum > 0:
        mean = (lin * a[..., None]).reshape(-1, 3).sum(axis=0) / wsum
    else:
        mean = lin.reshape(-1, 3).mean(axis=0)
    result = (tuple(float(x) for x in mean), float(a.min()), float(a.max()))
    _texel_mean_cache[tex_abs] = result
    return result


def _hue_flip_tint(texel_mean, target_rgb, cap=HUE_FLIP_CAP):
    """`(tint, predicted, fixed)` such that `texel_mean * tint` reads as a
    genuinely BROWN colour (R > G > B), for ANY texel mean, never a
    green/olive residue -- that guarantee is the entire point of this fix
    (see the module docstring).

    Starts from the literal per-channel recipe -- `min(cap, target_c /
    texel_c)`, each channel independently -- which is correct for most of
    this pool's textures. It is NOT sufficient for every one of them:
    `American_Beech` and `Common_Apple` (its actual bound texture,
    `hollyprivet_basecolor.png`) are dark AND green enough that their own R
    channel needs a 5-6x boost to reach the broadleaf target. Capped at 3x,
    R falls short of target while G's much smaller (<3x) ratio reaches ITS
    full target untouched -- so the 'obvious' independent per-channel clamp
    ships a result with R < G: still green (measured, not guessed: see this
    file's own bake report).

    THE FIX IS NOT TO SCALE ALL THREE CHANNELS DOWN UNIFORMLY BY THE WORST
    RATIO. `Largetooth_Aspen`'s texel blue is essentially zero (~0.001), so
    its OWN ratio to reach target B (0.06) is ~60x -- sizing a single
    shared scale factor to respect a 3x cap on THAT channel would crush R
    and G (whose own ratios are both comfortably under the cap) to a
    sliver of their fine, correct values for no reason (a real Aspen's fall
    colour is a vivid gold; a near-zero blue barely matters to whether a
    pixel reads "brown"). So the correction here is TARGETED: only when the
    independently-capped result still has G at or above R (or B at or above
    G) does it pull that ONE channel down -- to 90% of the channel above it
    -- leaving every other channel at its own independently-capped value.
    """
    texel = [max(float(c), 1e-6) for c in texel_mean]
    tint = [min(cap, float(target_rgb[c]) / texel[c]) for c in range(3)]
    pred = [texel[c] * tint[c] for c in range(3)]
    fixed = [False, False]
    if pred[0] <= pred[1]:
        tint[1] = 0.9 * pred[0] / texel[1]
        pred[1] = texel[1] * tint[1]
        fixed[0] = True
    if pred[1] <= pred[2]:
        tint[2] = 0.9 * pred[1] / texel[2]
        pred[2] = texel[2] * tint[2]
        fixed[1] = True
    return tuple(tint), tuple(pred), tuple(fixed)


def _omnipbr_leaf_material(stage, src_stage, src_abs, out_path, mat_prim,
                            target_rgb, salt, species="", report_tag="standing"):
    """Author a fresh OmniPBR at `/Root/tint_mats/<name>_<salt>` that
    REPLACES `mat_prim` (the source's custom-MDL leaf/needle material --
    may be `None`/invalid) for a damaged level's kept foliage. NEVER
    returns `None` -- see the module docstring's "THE TINT WAS A DEAD KNOB"
    section for why a replacement, not a tinted copy of the original.

    TEXTURES ARE ARCHETYPE-RELATIVE (the same idiom `_copy_tinted_material`
    -- which this replaces -- used for its own re-anchored paths, and the
    one the `hollyprivet_basecolor.png` fix proved out): `os.path.relpath`
    from THIS output file's own directory, so the archetype resolves from
    a laptop or a container root without a second copy of the asset tree.

    NEVER RETURNS `None`: if `mat_prim` is falsy/invalid, or its diffuse
    texture cannot be found/read, this falls back to a FLAT
    `diffuse_color_constant` at exactly `target_rgb` -- no texture to fail
    to resolve on any host, and already the intended brown/olive colour by
    construction (this is the fallback the task brief asks for: "if a
    species' mdl hides its textures... fall back to a flat brown
    diffuse_color_constant OmniPBR ... and SAY SO" -- the `mode ==
    "flat_fallback"` entry in `_species_material_report` IS that
    disclosure). Not reachable for any of the six species this file bakes
    today (all resolve cleanly, see the bake report) -- kept as a genuine
    safety net for a future species this cannot read cleanly.
    """
    dst_name = mat_prim.GetPath().name if mat_prim else "flat"
    dst_path = Sdf.Path("/Root/tint_mats/{0}_{1}".format(dst_name, salt))
    existing = stage.GetPrimAtPath(dst_path)
    if existing and existing.IsValid():
        return UsdShade.Material(existing)

    tex = (_leaf_texture_paths(src_abs, mat_prim)
           if mat_prim else {"diffuse": None, "normal": None})
    out_dir = os.path.dirname(out_path)
    mat_name = mat_prim.GetPath().name if mat_prim else "none"
    texel_info = _alpha_weighted_linear_mean(tex["diffuse"]) if tex["diffuse"] else None

    # RESOLVE THE "LEAVE IT GREEN" SENTINEL HERE, once, before ANY consumer
    # touches `target_rgb`. Setting the target to this material's OWN measured
    # texel mean makes `_hue_flip_tint` solve `mean -> mean`, i.e. a (1, 1, 1)
    # multiply, so the leaf keeps exactly the colour its texture already has
    # while every other thing this function does — the OmniPBR replacement,
    # the diffuse map, the rebinding `_author_foliage_cull` depends on — runs
    # completely unchanged. Resolving it at ONE point rather than branching at
    # each use is deliberate: the first attempt threaded the sentinel through
    # five separate `float(c) for c in target_rgb` sites, missed one, and the
    # bake died with "could not convert string to float".
    # NOTE THE SOLVE MUST ALSO BE SKIPPED, not merely re-aimed.
    # `_hue_flip_tint` does not compute "whatever multiply reaches this
    # target" — it FORCES a brown, R > G > B, "for ANY texel mean" (its own
    # docstring). Handing it `target == texel_mean` therefore does NOT return
    # (1, 1, 1): measured on Shumard_Oak_leaf_v4 it returned
    # (1.000, 0.683, 1.000), pulling green down and browning the leaf anyway.
    # So `_neutral` is carried past this point and short-circuits the solve.
    _neutral = (target_rgb == NEUTRAL_TINT)
    if _neutral:
        target_rgb = (tuple(texel_info[0]) if texel_info is not None
                      else NEUTRAL_FALLBACK_RGB)

    mat = UsdShade.Material.Define(stage, dst_path)
    sh = UsdShade.Shader.Define(stage, dst_path.AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    # A flat 0.6 for every replacement, per this stream's own directive --
    # roughER than a fresh, glossy leaf (fitting for dry/dying/dead residue)
    # and simpler than also carrying a roughness map's own resolution risk
    # for a quantity a 400 m overview camera cannot resolve either way.
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.6)

    if texel_info is not None:
        texel_mean, a_min, a_max = texel_info
        if _neutral:
            tint, pred, fixed = (1.0, 1.0, 1.0), tuple(texel_mean), False
        else:
            tint, pred, fixed = _hue_flip_tint(texel_mean, target_rgb)
        rel_diffuse = os.path.relpath(tex["diffuse"], out_dir)
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(rel_diffuse))
        # The fallback OmniPBR uses if the texture ever fails to resolve on
        # some OTHER host -- see `planks.wood_material`'s own note on why
        # this is set even though a bound texture normally replaces it.
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(c) for c in target_rgb]))
        sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(c) for c in tint]))
        if tex["normal"]:
            sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(os.path.relpath(tex["normal"], out_dir)))
        mode = "textured"
    else:
        # No resolvable diffuse texture (unbound/unresolvable original, or
        # PIL could not read it) -- flat brown constant, per this stream's
        # own fallback directive. `diffuse_tint` is still authored at
        # (1,1,1) -- a harmless no-op multiply -- purely so EVERY material
        # this function returns is uniformly "OmniPBR with an authored
        # diffuse_tint input", which is what `hurricane_tree_audit.py`'s
        # extended check (STREAM T6) looks for; the CONSTANT already IS
        # the target colour.
        tint, pred = (1.0, 1.0, 1.0), tuple(float(c) for c in target_rgb)
        a_min = a_max = None
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(c) for c in target_rgb]))
        sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
        mode = "flat_fallback"

    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")

    srgb255 = tuple(int(round(x * 255.0))
                     for x in _linear_to_srgb(np.array(pred)))
    _species_material_report[(species, mat_name, report_tag)] = {
        "species": species,
        "material": mat_name,
        "report_tag": report_tag,
        "mode": mode,
        "diffuse_texture": (os.path.relpath(tex["diffuse"], REPO) if tex["diffuse"] else None),
        "normal_texture": (os.path.relpath(tex["normal"], REPO) if tex["normal"] else None),
        "texel_mean_linear": (list(texel_info[0]) if texel_info else None),
        "alpha_min_max": ([a_min, a_max] if texel_info else None),
        "target_linear": [float(c) for c in target_rgb],
        "tint": [float(c) for c in tint],
        "predicted_linear": [float(c) for c in pred],
        "predicted_srgb255": list(srgb255),
    }
    return mat


# --------------------------------------------------------------------------
# The `snapped` geometric clip.
# --------------------------------------------------------------------------

def _read_face_varying(prim, name):
    """A face-corner-aligned array for `name` ("normals" or a primvar), or
    None if not authored. Length must equal `sum(faceVertexCounts)`.
    """
    if name == "normals":
        attr = UsdGeom.Mesh(prim).GetNormalsAttr()
        if not attr or not attr.HasAuthoredValue():
            return None
        return np.array(attr.Get(), dtype=np.float64)
    pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar(name)
    if not pv or not pv.HasAuthoredValue():
        return None
    vals = pv.ComputeFlattened() if pv.IsIndexed() else pv.Get()
    if vals is None:
        return None
    return np.array(vals, dtype=np.float64)


def _clip_mesh_flat(points, fvi, fvc, face_attrs, plane_z, keep_side):
    """Clip a TRIANGULATED, face-varying mesh by the plane `z == plane_z`.

    `points`: source points (Nverts, 3). `fvi`/`fvc`: standard USD mesh
    topology (every `fvc` entry must be 3 -- these assets already are, see
    the module docstring's probe). `face_attrs`: `{name: array}`, each
    array face-corner-aligned (length `3 * nfaces`, any per-corner width).
    `keep_side`: "below" or "above".

    Returns `(out_pos, out_attrs)`: `out_pos` is `(n_out_tri, 3, 3)`,
    `out_attrs[name]` is `(n_out_tri, 3, k)` -- a fully UNWELDED triangle
    soup (every triangle owns its own three corners), which is fine here:
    these are hard-edged broken-wood surfaces, not a smooth subdivision
    target, and unwelded is what the rest of this repo's authored debris
    (`planks.py`) already uses for exactly the same "faceVarying must carry
    a real per-corner normal or the crisp/broken read is lost" reason.
    """
    pts = np.asarray(points, dtype=np.float64)
    fvi = np.asarray(fvi, dtype=np.int64)
    fvc = np.asarray(fvc)
    if not np.all(fvc == 3):
        raise ValueError("clip requires a triangulated mesh")
    ntri = len(fvc)
    tri_idx = fvi.reshape(ntri, 3)
    corner_pos = pts[tri_idx]                                  # (ntri,3,3)
    names = list(face_attrs.keys())
    corner_attrs = {}
    for n in names:
        arr = np.asarray(face_attrs[n], dtype=np.float64)
        corner_attrs[n] = arr.reshape(ntri, 3, -1)

    z = corner_pos[:, :, 2]
    below = z <= plane_z
    all_below = below.all(axis=1)
    all_above = (~below).all(axis=1)
    mixed = ~(all_below | all_above)
    keep_whole = all_below if keep_side == "below" else all_above

    pos_blocks = [corner_pos[keep_whole]]
    attr_blocks = {n: [corner_attrs[n][keep_whole]] for n in names}

    mixed_idx = np.where(mixed)[0]
    extra_pos, extra_attrs = [], {n: [] for n in names}
    for ti in mixed_idx:
        p = corner_pos[ti]
        b = below[ti] if keep_side == "below" else ~below[ti]
        a_here = {n: corner_attrs[n][ti] for n in names}
        poly_p, poly_a = [], {n: [] for n in names}
        for i in range(3):
            j = (i + 1) % 3
            if b[i]:
                poly_p.append(p[i])
                for n in names:
                    poly_a[n].append(a_here[n][i])
            if b[i] != b[j]:
                denom = p[j][2] - p[i][2]
                t = 0.5 if abs(denom) < 1e-9 else (plane_z - p[i][2]) / denom
                t = min(1.0, max(0.0, t))
                poly_p.append(p[i] + t * (p[j] - p[i]))
                for n in names:
                    poly_a[n].append(a_here[n][i] + t * (a_here[n][j] - a_here[n][i]))
        m = len(poly_p)
        for k in range(1, m - 1):
            extra_pos.append(np.array([poly_p[0], poly_p[k], poly_p[k + 1]]))
            for n in names:
                extra_attrs[n].append(
                    np.array([poly_a[n][0], poly_a[n][k], poly_a[n][k + 1]]))
    if extra_pos:
        pos_blocks.append(np.stack(extra_pos, axis=0))
        for n in names:
            attr_blocks[n].append(np.stack(extra_attrs[n], axis=0))

    out_pos = np.concatenate(pos_blocks, axis=0) if pos_blocks else np.zeros((0, 3, 3))
    out_attrs = {}
    for n in names:
        k = corner_attrs[n].shape[-1]
        out_attrs[n] = (np.concatenate(attr_blocks[n], axis=0)
                         if attr_blocks[n] else np.zeros((0, 3, k)))
    return out_pos, out_attrs


def _drop_far_triangles(pos, attrs, radial_cap):
    """Drop any triangle (from `_clip_mesh_flat`'s output) whose centroid is
    more than `radial_cap` from the vertical (Z) axis at (x=0, y=0).

    Used only for the `snapped` top piece: a fused limb inside the same
    mesh as the trunk (measured: Black_Oak) can reach 10+ m off-axis, and
    rotating that 80-85 degrees about the break point sends it to nearly
    full tree height -- see `bake_snapped`'s call site.
    """
    if pos.shape[0] == 0:
        return pos, attrs, 0
    centroid = pos.mean(axis=1)
    r = np.hypot(centroid[:, 0], centroid[:, 1])
    keep = r <= radial_cap
    n_dropped = int((~keep).sum())
    return pos[keep], {n: a[keep] for n, a in attrs.items()}, n_dropped


def _author_clipped_mesh(stage, path, out_pos, out_attrs, material=None):
    """Author `out_pos`/`out_attrs` (from `_clip_mesh_flat`) as a new,
    fully-unwelded Mesh prim at `path`.
    """
    ntri = out_pos.shape[0]
    mesh = UsdGeom.Mesh.Define(stage, path)
    if ntri == 0:
        # An empty clip is a legitimate outcome (e.g. break height above
        # every triangle in a short trunk) -- author a degenerate, empty
        # mesh rather than skip it, so callers can rely on the prim existing.
        mesh.CreatePointsAttr(Vt.Vec3fArray())
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray())
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray())
        return mesh
    flat_pos = out_pos.reshape(-1, 3).astype(np.float32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(flat_pos))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
        np.full(ntri, 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
        np.arange(ntri * 3, dtype=np.int32)))
    if "normals" in out_attrs:
        flat_n = out_attrs["normals"].reshape(-1, 3).astype(np.float32)
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(flat_n))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    pvapi = UsdGeom.PrimvarsAPI(mesh)
    for name in ("st", "st_1"):
        if name not in out_attrs:
            continue
        arr = out_attrs[name].reshape(-1, out_attrs[name].shape[-1])
        if arr.shape[-1] < 2:
            continue
        pv = pvapi.CreatePrimvar(
            name, Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray.FromNumpy(arr[:, :2].astype(np.float32)))
    if "displayColor" in out_attrs:
        arr = out_attrs["displayColor"].reshape(-1, out_attrs["displayColor"].shape[-1])
        if arr.shape[-1] >= 3:
            pv = pvapi.CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec3fArray.FromNumpy(arr[:, :3].astype(np.float32)))
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


def _mesh_face_attrs(prim, xcache=None):
    """`(points, fvi, fvc, face_attrs)` for a Mesh prim, ready for
    `_clip_mesh_flat`. `face_attrs` includes whichever of
    normals/st/st_1/displayColor are authored.

    POINTS ARE RETURNED IN ROOT-RELATIVE SPACE, not the mesh's own local
    space, whenever `xcache` is given. THIS MATTERS: several of these
    assets author each major limb as its OWN mesh with its OWN
    `xformOp:transform` placing it in the crown (e.g. Black_Oak's
    `branch2` sits at local z~106 cm but carries a transform that puts it
    at root-relative z~1,210-1,380 cm — a 12 m difference). Reading raw
    local points for a break-height comparison, or for a newly-authored
    sibling mesh with no transform of its own, silently uses the WRONG
    frame: the first measured symptom was a `snapped` Black_Oak whose
    severed-top piece topped out at 17 m — nearly full tree height — because
    the clip plane (computed in root space) was being compared against
    `branch2`'s LOCAL z, and the freshly authored replacement mesh then
    carried local (near-origin) coordinates into a prim with no transform
    to correct them. `xcache.GetLocalToWorldTransform(prim)` on the SOURCE
    stage IS root-relative here because these assets' `/Root` itself
    carries no xformOp (verified for every species this module bakes) —
    there is no need to separately track "relative to Root" vs "world".
    Normals are rotated by the same matrix's linear part; every transform
    seen in this asset family is a pure rotation (checked: each row of the
    3x3 has unit norm), so no inverse-transpose/rescale is needed, but the
    result is renormalised defensively in case a future asset is not rigid.
    """
    m = UsdGeom.Mesh(prim)
    points = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    fvi = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    fvc = np.array(m.GetFaceVertexCountsAttr().Get())
    face_attrs = {}
    for name in ("normals", "st", "st_1", "displayColor"):
        arr = _read_face_varying(prim, name)
        if arr is not None and len(arr) == 3 * len(fvc):
            face_attrs[name] = arr
    if xcache is not None and len(points):
        mat = np.array(xcache.GetLocalToWorldTransform(prim))  # (4,4), p' = p @ M
        rot, trans = mat[:3, :3], mat[3, :3]
        points = points @ rot + trans
        if "normals" in face_attrs:
            n = face_attrs["normals"] @ rot
            norm = np.linalg.norm(n, axis=-1, keepdims=True)
            norm[norm == 0] = 1.0
            face_attrs["normals"] = n / norm
    return points, fvi, fvc, face_attrs


# --------------------------------------------------------------------------
# STREAM T7 — constructed break-debris geometry. See the module docstring
# and the `_LIMB_SECTOR_*`/`_ROOTPLATE_*`/`_COLLAR_*` constants' own
# comments for the directive and the tuning. Every mesh below is authored
# directly (points + faceVarying normals), the same idiom `_author_clipped_
# mesh` already uses for the snag clip, so nothing here adds a new
# authoring mechanism to the file.
# --------------------------------------------------------------------------

def _oriented_box(center, size, yaw_deg=0.0, pitch_deg=0.0):
    """An axis-aligned box of `size=(length, width, height)` (metres OR cm,
    whatever frame `center` is in — this function does no unit conversion),
    long axis initially +X, PITCHED about local Y (tips the long axis
    toward +/-Z) THEN YAWED about global Z (aims it at a compass
    direction), centred at `center`. Matches how a stub/limb/spike is
    aimed here: tip it up or down, then swing it to face the sector's or
    the break ring's outward azimuth.

    Returns `(out_pos, out_attrs)` in exactly the shape `_author_clipped_
    mesh` expects: `out_pos.shape == (12, 3, 3)` (12 triangles, 2 per box
    face), `out_attrs["normals"]` the same shape with one flat per-face
    normal repeated at all 3 corners, so the box reads with crisp edges
    (see `_author_clipped_mesh`'s own "faceVarying" note) rather than the
    renderer's shared-vertex smoothing average.
    """
    sx, sy, sz = (0.5 * float(v) for v in size)
    signs = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
             (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
    corners = np.array([[sx * a, sy * b, sz * c] for a, b, c in signs],
                        dtype=np.float64)
    faces = [
        ((0, 1, 2, 3), (0.0, 0.0, -1.0)),   # bottom
        ((4, 7, 6, 5), (0.0, 0.0, 1.0)),    # top
        ((0, 4, 5, 1), (0.0, -1.0, 0.0)),   # -Y
        ((2, 6, 7, 3), (0.0, 1.0, 0.0)),    # +Y
        ((0, 3, 7, 4), (-1.0, 0.0, 0.0)),   # -X
        ((1, 5, 6, 2), (1.0, 0.0, 0.0)),    # +X
    ]
    yaw = math.radians(float(yaw_deg))
    pitch = math.radians(float(pitch_deg))
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    cp_, sp_ = math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy_, -sy_, 0.0], [sy_, cy_, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp_, 0.0, sp_], [0.0, 1.0, 0.0], [-sp_, 0.0, cp_]])
    rm = rz @ ry
    c = np.asarray(center, dtype=np.float64)

    tri_pos, tri_norm = [], []
    for (a, b, cc, d), n in faces:
        n_world = rm @ np.asarray(n, dtype=np.float64)
        for tri in ((a, b, cc), (a, cc, d)):
            pts = corners[list(tri)] @ rm.T + c
            tri_pos.append(pts)
            tri_norm.append(np.tile(n_world, (3, 1)))
    out_pos = np.stack(tri_pos, axis=0)
    out_attrs = {"normals": np.stack(tri_norm, axis=0)}
    return out_pos, out_attrs


def _box_min_z(size, yaw_deg, pitch_deg):
    """The minimum Z, among `_oriented_box(size, yaw_deg, pitch_deg)`'s 8
    corners, were that box centred at the origin — i.e. how far the box's
    TRUE lowest point sits below its own centre once yaw+pitch are applied.

    NEEDED BECAUSE A ROTATED BOX'S Z-EXTENT IS NOT HALF ITS UNROTATED
    THICKNESS. `_author_limb_break`'s first version seated a fallen limb at
    `z_c = 0.5 * thick_m - 0.01`, which is only correct at zero pitch — even
    a few degrees of pitch on a box whose LENGTH (2-4 m) is much larger
    than its thickness swings the true lowest corner by roughly
    `0.5 * length * sin(pitch)`, measured up to 12 cm low on this bake's
    own placements before this fix (an American_Beech fallen limb at
    z=-0.1018 m). Exactly the "seat on vertices, never a derived box"
    lesson this repo already has elsewhere (`bake.world_point_bounds`,
    `vegetation.log_points`) applies just as much to geometry this file
    authors itself as to geometry a solver produced.
    """
    sx, sy, sz = (0.5 * float(v) for v in size)
    signs = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
             (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
    corners = np.array([[sx * a, sy * b, sz * c] for a, b, c in signs],
                        dtype=np.float64)
    yaw = math.radians(float(yaw_deg))
    pitch = math.radians(float(pitch_deg))
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    cp_, sp_ = math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy_, -sy_, 0.0], [sy_, cy_, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp_, 0.0, sp_], [0.0, 1.0, 0.0], [-sp_, 0.0, cp_]])
    rm = rz @ ry
    world = corners @ rm.T
    return float(world[:, 2].min())


def _author_box(stage, path, center, size, yaw_deg, pitch_deg, material):
    """Author one `_oriented_box` as a Mesh at `path`, bound to `material`.

    `doubleSided` IS SET HERE, unlike every other mesh this file authors
    (see the module docstring's "NO OPACITY IS WIRED" section for why the
    LEAF replacement deliberately does not touch it). That reasoning does
    not transfer: this is hand-rolled box geometry with no real asset to
    inherit correct winding from, getting a box's 12-triangle winding
    right by hand is easy to get backward face-by-face, and a backward
    face on a single-sided mesh renders as a HOLE, not as a shading error
    — cheaper to make every face visible from both sides than to prove the
    winding by construction for something this small.
    """
    pos, attrs = _oriented_box(center, size, yaw_deg, pitch_deg)
    mesh = _author_clipped_mesh(stage, path, pos, attrs, material=material)
    mesh.CreateDoubleSidedAttr(True)
    return mesh


def _stump_bark_material(stage, src_stage, src_abs, root_path, woody,
                          prefix="/Root/src"):
    """The species' own bark material, remapped onto the OUTPUT stage at
    `prefix` — the same `_remap_material`-if-resolvable-else-None pattern
    `bake_snapped` already uses for `below_mat`, pulled out so `bake_one`'s
    new STREAM T7 debris can reuse it without duplicating the resolvability
    check. Returns `None` (never a broken/unresolvable material) if there
    is no woody prim, no bound material, or it does not resolve — callers
    fall back to `_ensure_safe_material(stage, "wood")`.
    """
    if not woody:
        return None
    trunk_prim = src_stage.GetPrimAtPath(root_path.AppendPath(woody[0]))
    trunk_mat = _bound_material(trunk_prim)
    if not trunk_mat or not _material_ok(src_abs, trunk_mat):
        return None
    return _remap_material(stage, trunk_mat, root_path, prefix)


def _author_limb_break(stage, src_stage, src_abs, default, root_path, woody,
                       sector_center_deg, sector_half_deg, rng):
    """STREAM T7, `limbed` only. 2-4 bare broken-limb stubs jutting from the
    stripped azimuth sector, plus 1-2 of those limbs' fallen counterparts
    lying on the ground. Returns the number of prims authored.

    STUBS are children of `/Root/src`, authored directly in the SOURCE
    asset's own CENTIMETRE frame (matching `_mesh_face_attrs`' own
    convention for this file) so `/Root/src`'s existing cm->m + `limbed`'s
    small (4 deg) lean transform places them exactly like the rest of the
    tree, with no separate coordinate math needed — they are meant to
    still read as ATTACHED.

    FALLEN LIMBS are children of `/Root` directly, authored in METRES with
    no transform of their own, so "seated at z=0 in archetype space" is
    exact regardless of that same 4 deg residual lean rather than something
    that has to be solved back out of it — see the 2026-09-01 rule in this
    module's own docstring: the cm->m scale (and any transform at all)
    lives on CHILD prims, never on `/Root`, and these are new children of
    `/Root`, deliberately left op-free, not a regression of that rule.
    """
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    whole_rng = bbox_cache.ComputeWorldBound(default).ComputeAlignedRange()
    if whole_rng.IsEmpty():
        return 0
    zlo, zhi = float(whole_rng.GetMin()[2]), float(whole_rng.GetMax()[2])
    xlo, xhi = float(whole_rng.GetMin()[0]), float(whole_rng.GetMax()[0])
    ylo, yhi = float(whole_rng.GetMin()[1]), float(whole_rng.GetMax()[1])
    height_cm = zhi - zlo
    crown_r_cm = 0.5 * max(xhi - xlo, yhi - ylo)
    if height_cm <= 0.0 or crown_r_cm <= 0.0:
        return 0

    mat = _stump_bark_material(stage, src_stage, src_abs, root_path, woody)
    if mat is None:
        mat = _ensure_safe_material(stage, "wood")

    n_made = 0
    n_stub = rng.randint(2, 4)
    for i in range(n_stub):
        az = sector_center_deg + rng.uniform(-sector_half_deg * 0.6,
                                             sector_half_deg * 0.6)
        a = math.radians(az)
        z0 = zlo + height_cm * rng.uniform(0.35, 0.72)
        r0 = crown_r_cm * rng.uniform(0.12, 0.30)
        length = rng.uniform(*_STUB_LEN_CM)
        thick = rng.uniform(*_STUB_THICK_CM)
        cx = math.cos(a) * (r0 + 0.5 * length)
        cy = math.sin(a) * (r0 + 0.5 * length)
        # broken limbs droop more often than they point up
        pitch = rng.uniform(-25.0, 15.0)
        path = Sdf.Path("/Root/src/limb_stub_{0:02d}".format(i))
        _author_box(stage, path, (cx, cy, z0), (length, thick, thick),
                   yaw_deg=math.degrees(a), pitch_deg=pitch, material=mat)
        n_made += 1

    n_fallen = rng.randint(1, 2)
    for i in range(n_fallen):
        az = sector_center_deg + rng.uniform(-sector_half_deg, sector_half_deg)
        a = math.radians(az)
        length_m = rng.uniform(*_FALLEN_LIMB_LEN_M)
        thick_m = rng.uniform(*_FALLEN_LIMB_THICK_M)
        reach_m = (crown_r_cm * CM_TO_M) * rng.uniform(0.35, 0.85)
        cx_m = math.cos(a) * reach_m
        cy_m = math.sin(a) * reach_m
        yaw = math.degrees(a) + rng.uniform(-20.0, 20.0)
        pitch = rng.uniform(-4.0, 4.0)
        # SEAT ON THE ROTATED CORNER, NOT ON HALF THE UNROTATED THICKNESS --
        # see `_box_min_z`'s own docstring for the up-to-12-cm float this
        # replaces. Bed in a hair beyond that, exactly like `vegetation.
        # wood_debris`'s own seated pieces -- a box resting exactly ON z=0
        # with no bed-in reads as floating under a low sun (the tornado
        # skill's shadow-displacement argument).
        z_c = -_box_min_z((length_m, thick_m, thick_m), yaw, pitch) - 0.01
        path = Sdf.Path("/Root/fallen_limb_{0:02d}".format(i))
        _author_box(stage, path, (cx_m, cy_m, z_c), (length_m, thick_m, thick_m),
                   yaw_deg=yaw, pitch_deg=pitch, material=mat)
        n_made += 1
    return n_made


def _author_root_plate_disc(stage, path, center_m, radius_m, azimuth_deg,
                            tilt_deg, rng, material, n=14):
    """STREAM T7, `leaning`/`fallen`. A tilted disc of torn earth at the
    base of a windthrown tree — a local reimplementation of the SAME
    construction `vegetation.root_plate` uses for the tornado track
    (read-only precedent; this bake does not import `vegetation` — see the
    module docstring's "WHY IT IS NOT THE TORNADO BAKE" section on why it
    stays self-contained). A triangle FAN about a real centre vertex, not
    one concave n-gon (see `root_plate`'s own note on this being
    unambiguous under any triangulator), tilted back against the fall so
    it reads as earth standing on edge rather than a coin lying flat, with
    an irregular jittered rim for the same "a clean circle reads as
    authored" reason `scar_patch`/`root_plate` wobble theirs.

    `center_m` is the disc's OWN base point in the OUTPUT stage's metre
    space — for this bake that is always the tree's local origin (0,0,0),
    since `lift` is 0 for every level (see `_WOOD_PLAN`'s own comment on
    why). The disc's centre vertex sits `radius_m * 0.45` above `center_m`
    and its rim spans below and above that, same as `vegetation.root_
    plate`'s own construction — "a hole in the ground with the plug beside
    it, not a coin balanced on the lawn", so PART of it is expected to sit
    below `center_m`'s own z, not merely at or above it.
    """
    a = math.radians(float(azimuth_deg))
    ux, uy = math.cos(a), math.sin(a)
    t = math.radians(float(tilt_deg))
    e1 = (-uy, ux, 0.0)
    e2 = (-ux * math.cos(t), -uy * math.cos(t), math.sin(t))
    cx, cy, cz0 = (float(v) for v in center_m)
    cz = cz0 + float(radius_m) * 0.45
    pts = [(cx, cy, cz)]
    for i in range(int(n)):
        th = 2.0 * math.pi * i / float(n)
        r = float(radius_m) * (1.0 + 0.20 * math.sin(3.0 * th + 0.7)
                               + 0.11 * math.sin(7.0 * th + 2.1)
                               + rng.uniform(-0.07, 0.07))
        c, s = math.cos(th) * r, math.sin(th) * r
        px = cx + e1[0] * c + e2[0] * s
        py = cy + e1[1] * c + e2[1] * s
        pz = cz0 + e1[2] * c + e2[2] * s + float(radius_m) * 0.45
        pts.append((px, py, pz))
    tri_pos = []
    for i in range(int(n)):
        j = (i + 1) % int(n)
        tri_pos.append(np.array([pts[0], pts[1 + i], pts[1 + j]]))
    out_pos = np.stack(tri_pos, axis=0)
    nrm = np.array([ux * math.sin(t), uy * math.sin(t), math.cos(t)])
    out_attrs = {"normals": np.tile(nrm, (out_pos.shape[0], 3, 1))}
    mesh = _author_clipped_mesh(stage, path, out_pos, out_attrs, material=material)
    mesh.CreateDoubleSidedAttr(True)  # a hand-wound fan, same reasoning as `_author_box`
    return mesh


def _snap_collar(stage, below_pos, break_z_cm, trunk_prim, bbox_cache,
                 material, rng):
    """STREAM T7, `snapped` only. 3-5 small bark spikes ringing the stump's
    break plane, radiating outward and a little upward — "a snag reads by
    its splintered spar, not a clean cut" (the fire skill's spar language,
    see the module docstring). `_clip_mesh_flat`'s plane cut otherwise
    leaves a perfectly flat, almost sawn-looking disc at the break, which
    is the one part of the whole ladder that most needs a jagged read.

    Children of `/Root/src`, in the SAME cm frame as `trunk_cut_below`
    (`below_pos`, already in that frame — see `bake_snapped`'s own call
    site), so the existing `/Root/src` transform places them with the rest
    of the stump; no separate coordinate math needed.

    The ring's centre and radius are MEASURED from `below_pos`'s own
    points that landed exactly on the cut plane (`_clip_mesh_flat` clips
    triangle EDGES against `plane_z` and interpolates the crossing point
    onto it exactly, so these are real, not approximate). Falls back to
    the trunk's own bbox half-extent at the cut only if the clip happened
    to leave no such points (e.g. every triangle already had a vertex
    exactly on the plane) — never raises, never leaves a zero-radius
    pincushion of spikes at the exact centre.
    """
    plane_pts = below_pos.reshape(-1, 3)
    near = np.abs(plane_pts[:, 2] - break_z_cm) < 1e-3
    ring_pts = plane_pts[near]
    if len(ring_pts) >= 3:
        ccx, ccy = float(ring_pts[:, 0].mean()), float(ring_pts[:, 1].mean())
        r_cut = float(np.median(np.hypot(ring_pts[:, 0] - ccx, ring_pts[:, 1] - ccy)))
    else:
        box = bbox_cache.ComputeWorldBound(trunk_prim).ComputeAlignedRange()
        ccx = 0.5 * (float(box.GetMin()[0]) + float(box.GetMax()[0]))
        ccy = 0.5 * (float(box.GetMin()[1]) + float(box.GetMax()[1]))
        r_cut = 0.25 * min(float(box.GetMax()[0]) - float(box.GetMin()[0]),
                           float(box.GetMax()[1]) - float(box.GetMin()[1]))
    r_cut = max(r_cut, 3.0)  # never degenerate to a zero-radius pincushion

    n_spikes = rng.randint(*_COLLAR_N)
    made = 0
    for i in range(n_spikes):
        th = rng.uniform(0.0, 2.0 * math.pi)
        length = rng.uniform(*_COLLAR_LEN_CM)
        thick = rng.uniform(*_COLLAR_THICK_CM)
        cx = ccx + math.cos(th) * (r_cut + 0.5 * length)
        cy = ccy + math.sin(th) * (r_cut + 0.5 * length)
        cz = break_z_cm + rng.uniform(-0.3 * length, 0.15 * length)
        path = Sdf.Path("/Root/src/snag_collar_{0:02d}".format(i))
        _author_box(stage, path, (cx, cy, cz), (length, thick, thick),
                   yaw_deg=math.degrees(th), pitch_deg=rng.uniform(5.0, 35.0),
                   material=material)
        made += 1
    return made


def bake_snapped(species, src_abs, src_stage, foliage, woody, out_path, rng):
    """Build the `snapped` archetype: a clipped stump + a separate, tipped
    severed-top piece. See the module docstring.

    Returns `(stage, stats)`. Falls back to the old thinned-crown
    approximation (documented as such in `stats["clip"]`) if the geometric
    clip cannot be built for this species -- never raises and never leaves
    an empty file.
    """
    default = src_stage.GetDefaultPrim()
    root_path = default.GetPath()
    # NOTE: the purpose token's STRING VALUE is "default", not "default_" --
    # `default_` is only the Python attribute name (avoiding the `default`
    # keyword). Passing the literal string "default_" silently matches
    # nothing and every `ComputeUntransformedBound` below comes back EMPTY
    # (min > max FLT_MAX sentinels) with no error. Use the actual tokens.
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    # `/Root` itself carries no xformOp in any of these six source assets
    # (verified), so its WORLD bound already IS the whole-tree extent, and
    # `XformCache.GetLocalToWorldTransform` on any descendant already IS
    # root-relative — see `_mesh_face_attrs`'s docstring for why this
    # distinction matters for the per-mesh queries below.
    xcache = UsdGeom.XformCache()
    whole_box = bbox_cache.ComputeWorldBound(default)
    whole_rng = whole_box.ComputeAlignedRange()
    height_cm = float(whole_rng.GetMax()[2] - whole_rng.GetMin()[2]) if not whole_rng.IsEmpty() else 0.0

    trunk_rel = woody[0] if woody else None
    stats = {"clip": None, "break_frac": None, "break_z_cm": None,
             "height_cm": height_cm}

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, Sdf.Path("/Root"))
    stage.SetDefaultPrim(root.GetPrim())
    m_root = Gf.Matrix4d().SetScale(Gf.Vec3d(CM_TO_M, CM_TO_M, CM_TO_M))
    # NOT on /Root -- see the site-2 comment (root ops wrap the launcher's
    # placement). The scale goes on /Root/src below, and /Root/top absorbs
    # it into its own matrix (row-vector convention: M_top_new = M_top *
    # M_root).

    if trunk_rel is None or height_cm <= 0.0:
        # No woody prim to clip at all -- degenerate source. Emit an empty
        # `/Root/src` reference with nothing hidden so at least the caller
        # gets SOMETHING rather than a crash; stats records the failure.
        stats["clip"] = "SKIPPED: no woody prim / zero height"
        src = stage.DefinePrim(Sdf.Path("/Root/src"), "Xform")
        UsdGeom.Xformable(src).AddTransformOp().Set(m_root)
        src.GetReferences().AddReference(_relpath(src_abs, out_path))
        return stage, stats

    frac = rng.uniform(*SNAP_BREAK_FRAC)
    break_z_cm = frac * height_cm
    stats["break_frac"] = frac
    stats["break_z_cm"] = break_z_cm

    trunk_prim = src_stage.GetPrimAtPath(root_path.AppendPath(trunk_rel))
    try:
        points, fvi, fvc, face_attrs = _mesh_face_attrs(trunk_prim, xcache=xcache)
        below_pos, below_attrs = _clip_mesh_flat(
            points, fvi, fvc, face_attrs, break_z_cm, "below")
        above_pos, above_attrs = _clip_mesh_flat(
            points, fvi, fvc, face_attrs, break_z_cm, "above")
        if below_pos.shape[0] == 0:
            raise ValueError("empty stump after clip")
        # THE LEVER-ARM PROBLEM ALSO HITS THE TRUNK MESH'S OWN GEOMETRY, not
        # only secondary branch prims (see `above_woody`'s filter below for
        # the mechanism). Several species (measured: Black_Oak) fuse a
        # major limb into the SAME mesh as the trunk/bole, so the "above
        # break" clip can itself contain vertices 10+ m off-axis; rotating
        # those 80-85 deg about the break sends them to nearly full tree
        # height. Drop any triangle whose centroid is that far from the
        # axis from the rotated piece — its mass reads as broken away
        # separately, which is the same call made for secondary items.
        above_pos, above_attrs, n_dropped_far = _drop_far_triangles(
            above_pos, above_attrs, _SNAP_RADIAL_CAP_CM)
    except Exception as exc:  # noqa: BLE001 -- defensive, see docstring
        stats["clip"] = "FAILED ({0}); used thinned-crown approximation".format(exc)
        # Old approximation: strip foliage, keep a quarter of the woody crown.
        src = stage.DefinePrim(Sdf.Path("/Root/src"), "Xform")
        UsdGeom.Xformable(src).AddTransformOp().Set(m_root)
        src.GetReferences().AddReference(_relpath(src_abs, out_path))
        hide = list(foliage)
        hide.extend(_cull(woody, 0.25, rng, protect=1))
        for rel in hide:
            over = stage.OverridePrim(Sdf.Path("/Root/src").AppendPath(rel))
            UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        return stage, stats

    stats["clip"] = "ok: {0} below-tri, {1} above-tri ({2} far-off-axis tri dropped)".format(
        below_pos.shape[0], above_pos.shape[0], n_dropped_far)
    stats["n_dropped_far_tri"] = n_dropped_far

    trunk_mat = _bound_material(trunk_prim)

    # -- other items: assign to stump/top by ROOT-RELATIVE bbox z-centre ---
    # `ComputeWorldBound`, not `ComputeUntransformedBound` — a prim's own
    # `xformOp:transform` (several branch meshes have one) must be included
    # or this compares apples (a raw local z) to oranges (`break_z_cm`,
    # which is in root space). See `_mesh_face_attrs`'s docstring.
    def _z_center(rel):
        prim = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
        box = bbox_cache.ComputeWorldBound(prim)
        r = box.ComputeAlignedRange()
        if r.IsEmpty():
            return 0.0
        return 0.5 * (r.GetMin()[2] + r.GetMax()[2])

    def _radial(rel):
        prim = src_stage.GetPrimAtPath(root_path.AppendPath(rel))
        box = bbox_cache.ComputeWorldBound(prim)
        r = box.ComputeAlignedRange()
        if r.IsEmpty():
            return 0.0
        cx = 0.5 * (r.GetMin()[0] + r.GetMax()[0])
        cy = 0.5 * (r.GetMin()[1] + r.GetMax()[1])
        return (cx * cx + cy * cy) ** 0.5

    other_woody = woody[1:]
    below_woody = [r for r in other_woody if _z_center(r) < break_z_cm]
    above_woody_all = [r for r in other_woody if _z_center(r) >= break_z_cm]
    below_fol = [r for r in foliage if _z_center(r) < break_z_cm]
    above_fol_all = [r for r in foliage if _z_center(r) >= break_z_cm]

    # THE SAME LEVER-ARM PROBLEM `tornado.NO_UPROOT` EXISTS FOR, ONE LEVEL
    # DOWN. Rotating the ENTIRE above-break subtree 80-85 deg about a
    # horizontal axis at the break height sends anything with a large
    # horizontal offset from the trunk axis (a big side branch, not the
    # stem itself) swinging through a huge vertical arc — measured on
    # Black_Oak: a branch authored 498 cm off-axis put the "severed top"'s
    # highest point at 17 m, effectively full tree height, because
    # `new_z = pivot + y*sin(tilt) + (z-pivot)*cos(tilt)` is dominated by
    # the 498 cm `y` term at tilt ~83 deg. A rotation that avoids this for
    # every branch on every species does not exist (the tornado skill's own
    # resolution for the equivalent whole-tree case was to stop rotating
    # Black_Oak at all, not to find a better angle) — so any secondary
    # woody/foliage item whose horizontal reach from the trunk axis is more
    # than `_SNAP_RADIAL_CAP_CM` is dropped entirely (neither stump nor
    # top): a real snag's far-flung side limbs typically break away
    # separately in the same event rather than riding cleanly down with the
    # main stem, so losing them from the silhouette is the honest outcome,
    # not an approximation of convenience.
    above_woody = [r for r in above_woody_all if _radial(r) <= _SNAP_RADIAL_CAP_CM]
    above_fol = [r for r in above_fol_all if _radial(r) <= _SNAP_RADIAL_CAP_CM]
    dropped_eccentric = ([r for r in above_woody_all if r not in above_woody]
                         + [r for r in above_fol_all if r not in above_fol])

    is_conifer = species in CONIFERS
    tint = ((TARGET_NEEDLE_TOP if is_conifer else TARGET_BROADLEAF_TOP)
            if TREE_LEAF_TINT else NEUTRAL_TINT)

    # ---- STUMP: /Root/src -------------------------------------------------
    src = stage.DefinePrim(Sdf.Path("/Root/src"), "Xform")
    UsdGeom.Xformable(src).AddTransformOp().Set(m_root)
    src.GetReferences().AddReference(_relpath(src_abs, out_path))
    hide_stump = [trunk_rel] + list(foliage) + above_woody + dropped_eccentric
    for rel in hide_stump:
        over = stage.OverridePrim(Sdf.Path("/Root/src").AppendPath(rel))
        UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    below_mat = (_remap_material(stage, trunk_mat, root_path, "/Root/src")
                 if (trunk_mat and _material_ok(src_abs, trunk_mat)) else None)
    if below_mat is None:
        below_mat = _ensure_safe_material(stage, "wood")
    _author_clipped_mesh(stage, Sdf.Path("/Root/src/trunk_cut_below"),
                          below_pos, below_attrs, material=below_mat)
    # STREAM T7: a jagged collar of small bark spikes around the break
    # plane -- see `_snap_collar`'s own docstring for why the plane cut
    # otherwise leaves a clean, almost sawn-looking disc.
    stats["n_collar_spikes"] = _snap_collar(
        stage, below_pos, break_z_cm, trunk_prim, bbox_cache, below_mat, rng)

    # ---- SEVERED TOP: /Root/top, a second independent reference ----------
    #
    # THE PIVOT MUST BE AT GROUND LEVEL, NOT AT THE BREAK HEIGHT. The first
    # version of this pivoted `Rx(tip)` about `(0, 0, break_z_cm)` — i.e.
    # hinged the piece from the elevated break point, the way a door hinges
    # from its frame. That cannot ever bring the far end down to the
    # ground: rotating a rigid `L`-long piece about a pivot at height `H`
    # keeps every point at least `H - L` from the ground, and Black_Oak's
    # `H` (932 cm) exceeds its own `L` (782 cm), so the piece's lowest
    # reachable point at ANY angle is 150 cm in the air — measured, it came
    # out floating at 679-1192 cm, nowhere near the lawn. The severed top
    # does not hinge in place; it FALLS, so the model is the same one
    # `tip_tree` uses for a whole windthrown tree: translate the piece so
    # its broken end sits at z=0 FIRST, then rotate about that (now
    # ground-level) origin — `m_top = T(0,0,-break) * Rx(tip)`, no
    # trailing translate back up.
    top = stage.DefinePrim(Sdf.Path("/Root/top"), "Xform")
    if above_pos.shape[0]:
        ay = above_pos[:, :, 1].reshape(-1)
        az = above_pos[:, :, 2].reshape(-1) - break_z_cm
    else:
        ay = az = np.zeros(0)

    def _extent(tip_deg_):
        th = math.radians(tip_deg_)
        if len(az) == 0:
            return 0.0, 0.0
        z = ay * math.sin(th) + az * math.cos(th)
        return float(z.min()), float(z.max())

    # `tip_deg` is chosen to bring the LOWEST point close to the ground
    # (ground contact is the directive; ``bake-tree-and-debris`` measures
    # exactly this for the same reason) while not sending the peak
    # needlessly high — measured and searched, not picked once and trusted,
    # because `new_z` is not monotone in `tip` once the crown has off-axis
    # (Y) spread (see the module's own record of the first, wrong version).
    theta_candidates = np.linspace(SNAP_TOP_TILT_DEG[0], 90.0, 21)
    best_t, best_score = theta_candidates[0], float("inf")
    scored = []
    for t in theta_candidates:
        lo, hi = _extent(t)
        # ground contact wants `lo` near 0 from either side (a few cm under
        # is fine -- terrain hides it, the same call `fallen` makes); a
        # tall peak is a much smaller sin than a floating trunk.
        score = abs(lo) * 3.0 + max(0.0, hi - 250.0) * 0.5
        scored.append((score, t, lo, hi))
        if score < best_score:
            best_score, best_t = score, t
    # keep every candidate within a small tolerance of the best, then let
    # `rng` pick among them so six snags of one species are not identical.
    near_best = [t for s, t, _, _ in scored if s <= best_score + 15.0]
    tip_deg = float(rng.choice(near_best))
    lo, hi = _extent(tip_deg)
    stats["top_lowest_cm"] = lo
    stats["top_peak_cm"] = hi
    m_top = (Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, -break_z_cm))
             * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(tip_deg)))
             * m_root)   # /Root is op-free now; the child absorbs the cm->m scale
    UsdGeom.Xformable(top).AddTransformOp().Set(m_top)
    top_src = stage.DefinePrim(Sdf.Path("/Root/top/src"), "Xform")
    top_src.GetReferences().AddReference(_relpath(src_abs, out_path))
    hide_top = [trunk_rel] + below_woody + list(dropped_eccentric)
    kept_top_wood = _cull(above_woody, SNAP_TOP_WOOD_KEEP, rng)
    hide_top.extend(kept_top_wood)  # `_cull` returns the HIDDEN subset
    hide_top.extend(below_fol)
    for rel in hide_top:
        over = stage.OverridePrim(Sdf.Path("/Root/top/src").AppendPath(rel))
        UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    above_mat = (_remap_material(stage, trunk_mat, root_path, "/Root/top/src")
                 if (trunk_mat and _material_ok(src_abs, trunk_mat)) else None)
    # BELT AND SUSPENDERS ON THE SECOND REFERENCE -- see
    # `_composed_material_resolves`'s docstring. `_material_ok` already
    # verified the SOURCE material resolves; this re-verifies the same
    # claim against the ACTUAL COMPOSED prim now sitting under
    # `/Root/top/src`, catching a host where a second reference resolves
    # differently from a first even though plain USD composition says it
    # should not.
    if above_mat is not None and not _composed_material_resolves(above_mat):
        above_mat = None
    if above_mat is None:
        # Prefer the STUMP's own bark material -- already proven to
        # resolve (`below_mat`, computed above) and visually identical
        # (same species, same texture) -- over the flat, textureless safety
        # colour, which exists for when NOTHING resolvable is available at
        # all, not as a first-choice substitute for a real bark look the
        # stump already has right next to it.
        above_mat = below_mat if below_mat is not None else _ensure_safe_material(stage, "wood")
    _author_clipped_mesh(stage, Sdf.Path("/Root/top/trunk_cut_above"),
                          above_pos, above_attrs, material=above_mat)

    # THE SEVERED TOP'S RESIDUAL FOLIAGE, AT PER-CARD GRANULARITY -- NOT A
    # WHOLE-PRIM `_cull`. `above_fol` is a species' ENTIRE crown foliage as
    # ONE OR A HANDFUL of prims (the three single-mesh-crown species --
    # `Largetooth_Aspen`, `American_Beech`, `Common_Apple` -- give it
    # exactly one), so `_cull(above_fol, SNAP_TOP_FOL_KEEP, rng)` (0.20 at
    # the time this was found, since raised -- see `SNAP_TOP_FOL_KEEP`'s own
    # comment) on a 1-item pool always rounds to either 0% or 100% kept,
    # never the intended fraction -- verified: every one of those species'
    # `snapped` archetype baked to a measured 0% top foliage before this fix,
    # not the intended residue.
    # `_foliage_inventory`/`_select_kept_cards`/`_author_foliage_cull` are
    # the SAME per-instance/per-triangle machinery `bake_one` already uses
    # for the standing levels -- reused here at `prefix="/Root/top/src"`
    # rather than re-deriving a second cull scheme for one archetype level.
    inv_top = _foliage_inventory(src_stage, root_path, above_fol, xcache)
    kept_top = _select_kept_cards(inv_top, SNAP_TOP_FOL_KEEP, "random", rng)
    n_top_fol_kept, n_top_fol_total, top_rebound = _author_foliage_cull(
        stage, src_stage, src_abs, out_path, root_path, inv_top, kept_top,
        tint, xcache, prefix="/Root/top/src", species=species,
        report_tag="top")

    # material safety net over everything left visible in both copies.
    # FOLIAGE IS DELIBERATELY EXCLUDED HERE, matching `bake_one`'s own
    # split: `_author_foliage_cull` already safety-checks (and OmniPBR-
    # replaces, STREAM T6) the material on every KEPT card/instance it
    # authors, so re-running the blunt "rebind by SOURCE binding" safety
    # net over the same rels here would check the WRONG (untouched,
    # custom-MDL) source binding and could clobber the OmniPBR replacement
    # this pass just made. UNLIKE `bake_one`, there is no "wholly-kept
    # foliage still needs the safety net" carve-out to make here: `tint` is
    # NEVER `None` for `snapped` (a snag always residue-tints its top, see
    # `SNAP_TOP_FOL_KEEP`), so EVERY foliage rel under `/Root/top/src` --
    # wholly-kept or not -- already went through `_omnipbr_leaf_material`
    # above, which does its own resolution check and never leaves an
    # unresolvable/untouched material bound.
    visible_stump = [r for r in ([trunk_rel] + woody[1:]) if r not in hide_stump]
    visible_top = [r for r in woody[1:] if r not in hide_top]
    n_safe = _material_safety_net(stage, src_abs, visible_stump, root_path,
                                   src_stage, prefix="/Root/src")
    n_safe += _material_safety_net(stage, src_abs, visible_top, root_path,
                                    src_stage, prefix="/Root/top/src")
    stats["n_safety_rebinds"] = n_safe
    stats["top_foliage_rebound"] = len(top_rebound)
    stats["top_tilt_deg"] = tip_deg
    stats["top_foliage_kept"] = n_top_fol_kept
    stats["top_foliage_total"] = n_top_fol_total
    stats["n_dropped_eccentric"] = len(dropped_eccentric)
    return stage, stats


def bake_one(species, src_rel, level, out_dir, dry_run=False):
    src_abs = os.path.join(REPO, src_rel)
    if not os.path.exists(src_abs):
        return None, "source missing: {0}".format(src_rel)

    src_stage = Usd.Stage.Open(src_abs)
    foliage, woody = _classify(src_stage)
    if not foliage and not woody:
        return None, "no meshes found"

    out_path = os.path.join(out_dir, "tree_{0}_{1}.usd".format(species, level))
    if dry_run:
        return out_path, "would write ({0} foliage / {1} woody prim(s))".format(
            len(foliage), len(woody))

    rng = random.Random("{0}/{1}".format(species, level))

    if os.path.exists(out_path):
        os.remove(out_path)

    is_conifer = species in CONIFERS

    if level == "snapped":
        stage, extra_stats = bake_snapped(
            species, src_abs, src_stage, foliage, woody, out_path, rng)
        stage.GetRootLayer().Save()
        return out_path, "snag: {0}; {1} collar spike(s)".format(
            extra_stats["clip"], extra_stats.get("n_collar_spikes", 0))

    keep_wood, lean, lift = _WOOD_PLAN[level]
    keep_fol = _FOL_KEEP[level]
    fol_mode = _FOL_MODE[level]
    if is_conifer and level != "pristine":
        keep_fol = max(keep_fol, _CONIFER_FOL_KEEP.get(level, _CONIFER_FOL_KEEP_DEFAULT))

    default = src_stage.GetDefaultPrim()
    root_path = default.GetPath()

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, Sdf.Path("/Root"))
    stage.SetDefaultPrim(root.GetPrim())
    # ONE explicit matrix rather than a stack of ops. USD composes xformOps in
    # list order and it is easy to get scale-then-rotate backwards; writing
    # the product directly makes the intent unambiguous. Row-vector
    # convention (p' = p * M): scale to metres, tip about +X, then lift.
    m = (Gf.Matrix4d().SetScale(Gf.Vec3d(CM_TO_M, CM_TO_M, CM_TO_M))
         * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(lean)))
         * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, float(lift))))
    # ON THE CHILD, NEVER ON /Root. A root-level xformOp COMPOSES AROUND the
    # launcher's own placement ops (Add*Op appends AFTER the referenced
    # order, and the first-listed op is outermost), so a 0.01 root scale
    # multiplied every tree's world TRANSLATE by 0.01 -- all 1,684 trees of
    # every render to date stacked full-size within +-2.5 m of the plate
    # origin (the "cotton ball"/"park tree"). Root stays op-free exactly
    # like the tornado house archetypes. Found 2026-09-01 via a stage dump.
    src = stage.DefinePrim(Sdf.Path("/Root/src"), "Xform")
    UsdGeom.Xformable(src).AddTransformOp().Set(m)
    src.GetReferences().AddReference(_relpath(src_abs, out_path))

    hide = []
    # The trunk may never be culled — see `_cull`'s "protect" docstring on
    # the bug this guards.
    hide.extend(_cull(woody, keep_wood, rng, protect=1))
    for rel in hide:
        over = stage.OverridePrim(Sdf.Path("/Root/src").AppendPath(rel))
        UsdGeom.Imageable(over).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    # Foliage is culled at PER-CARD granularity (per instance / per
    # triangle), not per-prim — see the "Per-CARD foliage culling" section.
    # A whole-prim cull cannot hit any of `_FOL_KEEP`'s targets on a species
    # whose crown is one mesh or a handful of instancer groups.
    xcache = UsdGeom.XformCache()  # see `_mesh_face_attrs` — rooted, not local
    inv = _foliage_inventory(src_stage, root_path, foliage, xcache)
    # STREAM T7: `limbed` strips one azimuth SECTOR hard and leaves the rest
    # of the crown fuller, instead of a single global fraction -- see the
    # module docstring's own STREAM T7 section and `_select_kept_cards_
    # sector`'s docstring. Every other level is unchanged.
    sector_stats = None
    if level == "limbed":
        sector_center_deg = rng.uniform(0.0, 360.0)
        sector_half_deg = rng.uniform(*_LIMB_SECTOR_HALF_DEG)
        keep_in = rng.uniform(*_LIMB_SECTOR_KEEP_IN)
        sector_frac = _sector_fraction(inv, sector_center_deg, sector_half_deg)
        if sector_frac >= 0.999:
            keep_out = keep_fol
        else:
            keep_out = (keep_fol - sector_frac * keep_in) / (1.0 - sector_frac)
        # "keep the rest fuller": never let the solve hand back an outside
        # retention BELOW the inside one (a pathological sector_frac could,
        # in principle, e.g. a crown whose whole population happens to fall
        # in a small sector).
        keep_out = min(1.0, max(keep_in, keep_out))
        kept = _select_kept_cards_sector(inv, keep_in, keep_out,
                                         sector_center_deg, sector_half_deg, rng)
        sector_stats = {"center_deg": sector_center_deg,
                        "half_deg": sector_half_deg, "keep_in": keep_in,
                        "keep_out": keep_out, "sector_frac": sector_frac}
    else:
        kept = _select_kept_cards(inv, keep_fol, fol_mode or "random", rng)
    tint = None
    if level != "pristine":
        tint = ((TARGET_NEEDLE if is_conifer else TARGET_BROADLEAF)
                if TREE_LEAF_TINT else NEUTRAL_TINT)
    n_fol_kept, n_fol_total, rebound_rels = _author_foliage_cull(
        stage, src_stage, src_abs, out_path, root_path, inv, kept, tint,
        xcache, species=species)

    # STREAM T7: constructed break debris (`limbed`) / root-plate disc
    # (`leaning`, `fallen`) -- see the module docstring's own section.
    n_extra = 0
    if level == "limbed":
        n_extra = _author_limb_break(
            stage, src_stage, src_abs, default, root_path, woody,
            sector_stats["center_deg"], sector_stats["half_deg"], rng)
    elif level in ("leaning", "fallen"):
        # MEASURE the fall azimuth from the actual rotation just built into
        # `m`, rather than deriving it analytically from `axis=(1,0,0)`'s
        # sign convention -- consistent with this file's own "measure it,
        # don't guess it" style (see `bake_snapped`'s tilt-angle search).
        rot_only = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(lean)))
        fall_dir = rot_only.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        fall_az_deg = math.degrees(math.atan2(fall_dir[1], fall_dir[0]))
        r_lo, r_hi = _ROOTPLATE_R_M[level]
        t_lo, t_hi = _ROOTPLATE_TILT_DEG[level]
        soil = _ensure_safe_material(stage, "soil")
        # `lift` is 0 for every level (see `_WOOD_PLAN`'s own comment), so
        # the tree's local origin -- (0, 0, 0) in the OUTPUT stage's metre
        # space -- is always the planted base, whatever `lean` is.
        _author_root_plate_disc(
            stage, Sdf.Path("/Root/rootplate"), (0.0, 0.0, 0.0),
            rng.uniform(r_lo, r_hi), fall_az_deg, rng.uniform(t_lo, t_hi),
            rng, soil)
        n_extra = 1

    visible = [r for r in woody if r not in set(hide)]
    n_safe = _material_safety_net(stage, src_abs, visible, root_path, src_stage)
    # A wholly-kept (100% of its own cards survive) foliage rel still needs
    # the same texture-resolution check -- but ONLY at `pristine` (`tint is
    # None`). At every DAMAGED level `_author_foliage_cull` has ALREADY run
    # every such rel through `_omnipbr_leaf_material` (STREAM T6), which
    # does its own resolution check and never leaves an unresolvable/
    # untouched material bound; re-running the blunt "rebind by SOURCE
    # binding" safety net over the same rels here would check the WRONG
    # (untouched, custom-MDL) source binding and could clobber the OmniPBR
    # replacement with the safety net's own GREEN fallback colour --
    # reintroducing the exact cotton-ball class of defect this whole file
    # exists to rule out, through a door not even STREAM T6 opened.
    if tint is None:
        whole_kept_fol = [rel for rel, v in inv.items()
                          if v["n"] and len(kept.get(rel, [])) == v["n"]]
        n_safe += _material_safety_net(stage, src_abs, whole_kept_fol, root_path, src_stage)

    stage.GetRootLayer().Save()
    extra_note = ""
    if sector_stats is not None:
        extra_note = (", sector {0:.0f}+-{1:.0f}deg in={2:.0%}/out={3:.0%}, "
                     "{4} break-debris prim(s)").format(
            sector_stats["center_deg"], sector_stats["half_deg"],
            sector_stats["keep_in"], sector_stats["keep_out"], n_extra)
    elif n_extra:
        extra_note = ", {0} root-plate disc".format(n_extra)
    return out_path, ("{0} woody hidden of {1}, {2}/{3} foliage cards kept "
                       "({4:.0%}), {5} foliage material(s) rebound, "
                       "{6} safety rebind(s){7}").format(
        len(hide), len(woody), n_fol_kept, n_fol_total,
        (n_fol_kept / n_fol_total) if n_fol_total else 0.0,
        len(rebound_rels), n_safe, extra_note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        REPO, "scene_gen", "assets", "archetypes_hurricane"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--levels", default=",".join(LEVELS))
    ap.add_argument("--stats-json", default=None,
                     help="write per-archetype stats (deliverable #6) here")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    levels = [l.strip() for l in args.levels.split(",") if l.strip()]
    # `NO_UPROOT` SPECIES NEVER REACH THE "fallen" ARCHETYPE AT RUNTIME --
    # `hurricane.tree_level_for_intensity` promotes a drawn `fallen` to
    # `snapped` for them before the launcher ever looks the file up. Baking
    # it anyway is not merely wasted work: Black_Oak's crown is wide enough
    # that a rigid `fallen` rotation buries part of it (measured: -7.8 m,
    # nearly 19 m tall) -- exactly the defect `NO_UPROOT` exists to route
    # around. Shipping that file invites a FUTURE caller to reference it by
    # accident (the tornado bake's own precedent for skipping this
    # combination, cited in `hurricane.NO_UPROOT`'s docstring).
    try:
        _scene_gen_dir = os.path.join(REPO, "scene_gen")
        if _scene_gen_dir not in sys.path:
            sys.path.insert(0, _scene_gen_dir)
        from disaster import hurricane as _hu
        _no_uproot = set(_hu.NO_UPROOT)
    except Exception as _imp_exc:  # noqa: BLE001 -- never let this block a bake
        print("  ?? could not import hurricane.NO_UPROOT ({0}); baking "
              "`fallen` for every species".format(_imp_exc))
        _no_uproot = set()
    n_ok = n_skip = 0
    stats_all = {}
    for species, rel in sorted(TREE_SPECIES.items()):
        for level in levels:
            if level not in LEVELS:
                print("  ?? unknown level {0}".format(level))
                continue
            if level == "fallen" and species in _no_uproot:
                print("  -- {0:18s} {1:11s} SKIP  in NO_UPROOT -- promoted "
                      "to `snapped` at runtime, never referenced".format(
                          species, level))
                n_skip += 1
                stats_all["{0}_{1}".format(species, level)] = {
                    "ok": False, "note": "skipped: NO_UPROOT promotes to snapped"}
                continue
            out, note = bake_one(species, rel, level, args.out, args.dry_run)
            key = "{0}_{1}".format(species, level)
            if out is None:
                print("  -- {0:18s} {1:11s} SKIP  {2}".format(species, level, note))
                n_skip += 1
                stats_all[key] = {"ok": False, "note": note}
            else:
                print("  ok {0:18s} {1:11s} {2}".format(species, level, note))
                n_ok += 1
                stats_all[key] = {"ok": True, "note": note, "path": out}
    print("baked {0}, skipped {1} -> {2}".format(n_ok, n_skip, args.out))

    # STREAM T6's own deliverable: per-species/per-material OmniPBR report
    # (texel mean, computed tint, predicted colour) -- see `_species_material_
    # report`'s docstring at its definition. Empty on a `--dry-run` (nothing
    # was actually baked, so nothing was authored to report on).
    if _species_material_report:
        print()
        print("OmniPBR foliage replacement report (STREAM T6):")
        for (sp, mat_name, tag), row in sorted(_species_material_report.items()):
            texel = row["texel_mean_linear"]
            texel_s = ("({0:.3f},{1:.3f},{2:.3f})".format(*texel) if texel else "n/a")
            tint = row["tint"]
            pred = row["predicted_linear"]
            print("  {0:18s} {1:32s} [{11}] mode={2:13s} texel={3:20s} "
                  "tint=({4:.3f},{5:.3f},{6:.3f}) pred_lin=({7:.3f},{8:.3f},{9:.3f}) "
                  "pred_sRGB255={10}".format(
                      sp, mat_name, row["mode"], texel_s,
                      tint[0], tint[1], tint[2], pred[0], pred[1], pred[2],
                      tuple(row["predicted_srgb255"]), tag))
            # ONLY MEANINGFUL WHEN WE ARE ACTUALLY BROWNING. With
            # HUR_TREE_TINT off (the default) the target IS the leaf's own
            # green, so "not strictly brown" is the correct outcome and
            # printing it on every material is noise that reads like a fault.
            if TREE_LEAF_TINT and not (pred[0] > pred[1] > pred[2]):
                print("      !! predicted colour is NOT strictly brown "
                      "(R>G>B) -- {0}".format(pred))

    if args.stats_json and not args.dry_run:
        stats_all["_omnipbr_materials"] = [
            row for _, row in sorted(_species_material_report.items())]
        with open(args.stats_json, "w") as f:
            json.dump(stats_all, f, indent=2)
        print("stats -> {0}".format(args.stats_json))
    return 0 if n_ok else 1


if __name__ == "__main__":
    sys.exit(main())
