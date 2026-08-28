---
name: place-people-in-tornado-scenes
description: Where the CASUALTIES of a tornado go in a post-tornado scene and how much of each one a drone can see — the gates that keep a body inside the corridor and out from under everything that would hide it — the two axes the benchmark is defined on (attitude: face-up / face-down / on a side / pinned sitting; occlusion: thirteen named patterns over named body parts), the `disaster/tornado_people.py` planner API, the measured debris deck that stops a body lying through the boards, the A-pose trap that stands limbs upright, and the bench that is the only way to check any of it. Read before touching tornado_people.py, the lying poses in scene_generator._HUMAN_POSES, the people pass in suburb_tornado_launch_script, or tornado_people_preview_launch_script. The wildfire people model does NOT transfer and neither does this module's own first draft.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Place Casualties in a Post-Tornado Scene

## The one-paragraph version

Every figure this module places is a **casualty lying in or on the debris**.
There are no diggers, no walkers, no bystanders and no occupants standing beside
cars — they were all there once, and they were all cut. The module controls two
axes explicitly because those two axes ARE the benchmark: **attitude** (face-up,
face-down, on the left side, on the right side, drawn up, pinned sitting) and
**occlusion** (nothing at all, or one of thirteen patterns that cover a *named*
stretch of the body and leave the rest proud). Nothing is ever fully buried —
`max_covered_frac` is a hard ceiling enforced while the covering pieces are
generated. The ground truth records the attitude, the pattern, the covered
fraction and the list of body parts a camera can still see.

---

## Read this first: the two renders that produced this design

### Render 1 — "a pavement full of commuters"

67 figures on a 100 x 100 m plate, of which **55 were standing or walking**:
neighbours digging, survivors picking through their own house, walking wounded
on the carriageway, occupants beside displaced cars. Twelve were casualties.

That was defensible sociology and a useless dataset, for two reasons:

1. **Most of the figures had not been hit by anything.** A digger at T+45 min
   is real — Greensburg had 68% of households doing search and rescue, Joplin
   fielded seventeen professional rubble-rescue personnel against 1,371 injured
   — but this dataset scores *finding the people the tornado hit*.
2. **A standing figure on a debris pile is the easy problem anyway.** It is a
   vertical object on a horizontal one: high contrast from any altitude and any
   azimuth. What is hard, and what nobody has a labelled set of, is a
   HORIZONTAL body in a horizontal field of pale boards with an unknown part of
   it under a sheet of plywood.

### Render 2 — the bench, and the two geometry faults it caught

Nothing here was findable by reading the code, and both are pinned by
`scene_gen/tests/test_tornado_people_poses.py` now:

- **LIMBS STANDING STRAIGHT UP off every side-lying figure.** The rig's REST
  pose is an A-pose: the arm already hangs 45 degrees out to the side, in the
  character's X. Spin the body about its own long axis to put it on its side
  and that 45 degrees becomes **vertical**. Swinging the arm forward about X
  without first bringing it to plumb leaves it standing in the air. Every
  lateral pose now starts each arm with the `+-40` plumb correction — the same
  correction `idle` measures — and only then swings it.
- **COVERING BOARDS PASSING THROUGH RAISED KNEES AND HEELS.** A piece was
  seated at `z_m + lift`, the top of the body's *centre line*, which is right
  for a flat chest and wrong for everything else on a body. `_BODY_RISE` now
  gives the body's height at each station and `_crest` takes the maximum over
  the piece's own footprint — a rigid board rests on the highest thing under
  it.

A third fault was cosmetic and cost a whole review pass: the bench's ground was
`planks.wood_material`, so the floor and the debris were the same sawn-timber
map and a pale figure had no contrast against either. **The bench floor is mud
now.** Contrast against the corridor floor is half of what the bench is for.

### Render 3 — this file described a gate that was never written

**2026-08-28.** The section below used to state, in the present tense, that
`min_intensity` refuses a body wherever `ctx["intensity_at"]` is too low,
"tested at all three stations". **None of it existed.** `intensity_at` appeared
exactly once in `tornado_people.py` — in `plan_people`'s ctx docstring, marked
OPTIONAL — and was never read by anything. A comment inside `_candidate` said
"`_Field.in_track` refuses those now" and there was no `in_track`. The launcher
had been passing the field the whole time. The same was true of `canopies`.

It was found by a reader checking the claim before repeating it, not by a test
and not by a render — and by then the claim had already been repeated to the
user on this file's authority. **A skill that documents unbuilt behaviour is
worse than one with a gap**, because a gap gets checked and a confident
paragraph gets quoted. Both gates are now implemented, both are pinned by host
tests that fail if the gate is unwired (`test_21`, `test_22` — each asserts the
gate-OFF run *does* produce violations, so a disconnected gate cannot pass by
placing nothing), and every record carries the `intensity` it was gated on.

The rule that follows from it: **when this file says the code does something,
grep for the identifier before you believe it.** Every gate named here now
names the function that implements it and the refusal key it increments.

---

# The research

Every number in the module traces to something here. Change a share, change
this section too.

## Where the bodies are

- **Almost everybody survives.** Joplin's catastrophic zone — total structural
  destruction — held 4,716 people and killed 122: **97.4% survived** (NIST
  NCSTAR 3 geolocated all 161 fatalities; Paul & Stimers 2014 supplied the
  population denominators). Whole path: 13,547 people, 161 dead (1.2%), ~1,371
  injured. Of the injured **89% are minor** (ISS < 10) and 86% are discharged
  home (Niederkrotenthaler et al. 2013, 1,398 patients across 39 hospitals).
  **So the casualties in this scene are deliberately over-represented** — a
  true rate over ~39 damaged houses puts one or two in a 500 m corridor and
  there is nothing to score. That is a dataset decision and it is recorded as
  one in the module.
- **They are where the houses were.** Ten to fifteen minutes of warning means
  nobody evacuates: people shelter on the ground floor, interior — bathroom
  39%, closet 37%, hallway 10%, other 14% (Hammer & Schmidlin 2002, 190
  occupants of 65 F4/F5-damaged homes, none with a basement). So the head count
  is driven **per wrecked house** (`per_wreck`, weighted by damage level), not
  by a global number spread over the plate.
- **A third of the dead are recovered off their own footprint.** CDC MMWR
  61(28) recorded injury location and recovery location separately for all 338
  April-2011 deaths: **90.5% injured indoors, 3.3% outdoors, 37.0% recovered
  outdoors.** The MMWR records no distance, and most of that is "went out with
  the wall that failed and landed in the yard" — which is why `where` puts most
  of the population in the debris skirt and the yard, a little on the road in
  front of its own house, and only one or two far downtrack.
- **They cluster.** Chiu et al. 2013 (247 Alabama decedents) recorded who each
  victim was found with: with other deceased and survivors 26.3%, with other
  deceased 24.7%, with other survivors 21.1%, **ALONE 7.7%**. `cluster_chance`
  puts a second body within a few metres of the first — and two bodies together
  is the strongest aerial cue there is.
- **Correct shelter does not guarantee survival.** In Joplin, **20 of the 62**
  single-family-home decedents had taken correct interior refuge. NIST NCSTAR 3
  p.203: best-available refuge areas *"are not expected to offer life-safety
  protection against tornado hazards."*
- **Basements are rare and they worked.** Joplin: **17%** of path homes (1,237
  of 7,411); 80% had crawl spaces. Moore OK: "10% or less". West South Central
  new construction is 96-98% slab. NIST found **no evidence any fatality
  occurred below grade** in Joplin. None of it is placeable — see *What is
  deliberately absent*.

## Long-range throw is record-book territory, not a category

The number most likely to be "corrected" upward by someone who has seen a
documentary, so the derivation is written out.

Applied to ~39 damaged houses (~100 residents present, Joplin path rates of
0.77% dead and 4.8% injured):

    0.8 dead          x 37% recovered outdoors  = 0.3 people
    1.25 hospitalised x 43% thrown              = 0.5 people
    ------------------------------------------------------------
    UNDER ONE PERSON in the whole 500 x 500 m scene

(Brown et al. 2002, OKC 1999: "picked up / blown by tornado" was the mechanism
for **43% of hospitalised** injuries and only **6% of treated-and-released** —
being thrown is a severity marker, not a common experience.) The documented
survivals are **398 m** (Matt Suter, an F2, GPS-measured by an NWS official)
and **76 m** (a mother and two children on a mattress, Dawson Springs 2021).
There is **no published distribution of human throw distances**. `trail` is
therefore short and capped.

### Do NOT drive deposition with a throw field

- The **78%-left** deposition statistic (Snow et al. 1995, 163 debris reports)
  is for **lightweight** debris lofted into the parent storm and deposited tens
  of kilometres downstream. Cheques and photographs, not people.
- Bodies are heavy debris and stay in the swath.
- Near-surface flow is **CONVERGENT toward the centreline** — Karstens et al.
  2013 digitised **10,300 tree falls at Joplin and 94,500 at
  Tuscaloosa-Birmingham** and found inward-pointing falls across most of the
  path.
- There is **no published azimuthal distribution for victim deposition**.

A modest downtrack displacement with a wide spread (`spread_deg` 55, Gaussian)
is defensible. A tight fan is an invented claim.

## The epoch is T+30-60 min, and it is not a lighting choice

| window | what is in frame |
|---|---|
| T+0-15 min | self-extrication; roads impassable; **nobody in uniform** |
| **T+15-60 min** | **PEAK.** Joplin: 100 patients arrived at Freeman in 16 minutes, starting T+39 |
| T+1-6 h | mutual-aid apparatus, field triage on hard standing |
| T+6-24 h | organised grid search, task forces, heavy equipment, orange search markings — **and nobody left alive to find** |

**No responders, no heavy equipment, no search markings, no triage tents.**
A scene with those is depicting T+24 h and a different problem. `epoch_min`
records the choice.

---

# The two axes

## Attitude — front, back, sideways

| pose | attitude | what it is | reach |
|---|---|---|---|
| `lying_supine` | `face_up` | one knee drawn up, forearm across the chest, head lolled | 1.00 H |
| `lying_supine_open` | `face_up` | arms out, legs straight — the **widest** silhouette in the set | 1.00 H |
| `lying_prone` | `face_down` | one arm sprawled in the ground plane, one heel up, cheek down | 1.00 H |
| `lying_prone_reach` | `face_down` | both arms overhead — **2.1 m of ground length** | 1.14 H |
| `lying_side_l` | `side` | recovery position, on the left side | 0.88 H |
| `lying_side_r` | `side` | mirrored, on the right side | 0.88 H |
| `lying_curled_l` | `side` | drawn up — the **SHORT** silhouette, ~1.1 m | 0.62 H |

**EVERY ATTITUDE IS HORIZONTAL.** `trapped_sit` — pinned sitting, legs under
the pile, head and shoulders proud — was the one upright casualty and it was
**cut on the second review**: a figure sitting bolt upright in a levelled block
reads as somebody who sat down, not as somebody a building fell on. It was the
same objection that removed the standing figures and it is settled the same way.
The pose survives in `scene_generator._HUMAN_POSES` (`disaster.people` still
names it in `GROUND_POSES`); nothing in `tornado_people` can draw it, and
`_visible_parts` has no seated branch any more.

Roughly a third face-up, a third face-down, a third on a side. **There is no
literature on the attitude distribution of tornado casualties** — there is
barely any on their location — so the mix is a DATASET decision to cover the
space evenly, and it says so in the module.

### How a figure gets onto its side, and the trap that comes with it

`apply_placements` authors `rotateXYZ`, so the ops compose X (roll), then Y
(pitch), then Z (yaw).

    roll +90, pitch   0    face-down            (`lying_prone`)
    roll -90, pitch   0    face-up              (`lying_supine`)
    roll +90, pitch +90    on the LEFT side     (`lying_side_l`)
    roll +90, pitch -90    on the RIGHT side    (`lying_side_r`)

The roll lays the body down along the pre-yaw `-+Y`; the **pitch axis is then
world Y, which IS the body's own long axis**, so `people.LYING_SPIN` spins the
laid-out figure about itself without moving its head, its feet or its placement
point. `_body_axis` therefore still reads the **roll alone** — a rotation about
an axis cannot move a vector lying along it.

**THE AXIS TRAP, and it is the mirror image of the face-up one.** For a face-up
or face-down figure the ground plane is span(X, Z), so a delta about **Y** keeps
a limb on the ground — which is why every arm delta in `lying_supine` and
`lying_prone` is `((0, 1, 0), a)`. Spin the body 90 degrees about its long axis
and the plane becomes span(Y, Z): now it is a delta about **X** that keeps a
limb down and a Y delta that lifts it. Every swing in a lateral pose is about X;
the Y deltas that remain are the plumb correction and the deliberate lift of the
top limb clear of the bottom one.

**AND THE REST POSE IS AN A-POSE.** The arm hangs 45 degrees out in the
character's X, which after the spin is vertical. Bring it to plumb (`+-40` about
Y, which is what `idle` measures: the hand finishes 7 mm off plumb under the
shoulder) BEFORE swinging it. The forearm needs no correction of its own — it
inherits the upper arm's posed frame.

**THE LIFT CHANGES WITH THE ATTITUDE.** On its back or its face a body is half
its measured A-pose DEPTH deep (`sy`, 0.33-0.40 m across this pack, so ~0.17 m).
On its side the vertical dimension is its BREADTH — and `sx` cannot supply it,
because the A-pose bbox is 1.20-1.34 m wide with the arms out.
`people._lying_lift` models it: biacromial breadth is 0.245 H (Drillis &
Contini 1966) so the spine sits 0.22 m up at 1.80 m; hips are narrower (0.191 H)
and shoulder flesh compresses, so **0.115 H** is the figure used. VERIFY ON
SIGHT — a shoulder in the debris with a floating hip means it is too small.

**NOTHING MAY STAND UP.** A limb may be flexed — a drawn-up knee is what makes a
casualty read as a body rather than a mannequin — but the bound is **45 degrees
off the ground**, and `test_03_no_limb_of_a_lying_figure_stands_up` carries every
limb of every lying pose through its own deltas and its lay-down rotation
analytically to check it. `lying_prone`'s heel was at 57 and is now at 36;
`lying_supine`'s knee solve was softened with it.

## Occlusion — which parts of the body are visible

Each pattern names the stretch (or stretches) of the body that goes UNDER, in
fractions of the figure's own ground reach, soles at 0 and crown at 1.
`_visible_parts` turns the covered spans back into the list the record carries.

| pattern | covers | what stays visible |
|---|---|---|
| `none` | — | everything. **The fully-observable class, and the largest single entry (0.30)** |
| `feet_shins` | 0.00-0.30 | everything above the knee |
| `legs` | 0.00-0.52 | head, torso, arms |
| `legs_hips` | 0.00-0.62 | head, torso, shoulders |
| `midriff` | 0.42-0.70 | head, arms, legs |
| `torso` | 0.46-0.84 | head, legs, feet |
| `torso_head` | 0.52-1.04 | legs only |
| `head_only` | 0.80-1.04 | a body with no head |
| `upper_body` | 0.40-1.04 | legs and feet |
| `all_but_head` | 0.00-0.80 | a head and shoulders |
| `all_but_feet` | 0.20-1.04 | feet and shins |
| `banded` | 0.06-0.30 and 0.56-0.86 | head, hips, thighs — two slots |
| `flank` | one SIDE, along 0.55-0.78 of the length | one arm, one leg, half of everything |

A part counts as visible while **less than half** of its own extent is under
something — because the ground truth is read by a scorer, not by a renderer, so
the useful answer is "would a human labeller draw this part", not "is a single
pixel of it lit".

**Any single pattern is something a debris photograph shows. The DISTRIBUTION is
a dataset decision**, because the point is that a detector meets every partial
silhouette rather than the one that happens to be commonest.

### The four widest patterns are OFF, and the cap is 0.55

**2026-08-28, on the 1 km review: "some are completely obscured and a lot of
them are very slightly visible. I need more visibility."** The built scene's
ground truth had `covered_frac` p90 **0.76**, six bodies over 60% covered and
one showing nothing but a head. `max_covered_frac` was doing exactly what it
promised and what it promised was not enough.

- **`max_covered_frac` 0.80 -> 0.55.** A fifth of a body is two limbs and a
  hip. 0.55 leaves at minimum a head, a shoulder and an arm on every figure.
- **`sink_frac` 0.32 -> 0.18.** Sink is the occluder nothing records:
  `covered_frac` counts boards, so a body a third of its depth into the deck is
  filed as clear and eats its silhouette from below — the half a top-down camera
  loses first.
- **`legs_hips` (0.62), `upper_body` (0.64), `all_but_head` (0.80) and
  `all_but_feet` (0.84) are weighted 0.** Not deleted: the vocabulary is the
  benchmark's occlusion axis and the bench still photographs all thirteen. The
  weight went to `none` (0.30 -> **0.44**) and the light end.

**A ZERO IS NOT A DELETION AND IT IS NOT OPTIONAL EITHER.** `_trim_spans` would
happily shorten `all_but_head` from 0.80 to the cap — which keeps the FIGURE
visible and makes the LABEL a lie: a body with its head, arms and chest in clear
view, filed under the name of the pattern that hides them. That is the wrong
label this module's whole occlusion argument is against, so **every pattern with
a non-zero weight must fit under the cap untrimmed**, and `test_20` fails if a
re-tune ever breaks that pairing. Change one, check the other.

### NOTHING IS FULLY BURIED, and it is enforced, not checked

`max_covered_frac` (**0.55** since the 1 km review; 0.80 before it) is a hard
ceiling. `_trim_spans` SHORTENS any pattern that would exceed it *before* the
pieces are generated, so the module cannot author a body it would then have to
describe as invisible. This is `disaster.people`'s `exposed_interior` lesson and
it is the most important line in the file: **a target that cannot be seen cannot
be labelled, and an annotation for an invisible one is worse than no annotation
at all.**

**AND IT ONLY EVER COUNTED THIS MODULE'S OWN BOARDS.** The 1 km review found
bodies that were invisible with `covered_frac` recorded as clear, because the
wreck pile, the scour relief and an intact crown are not boards and nothing
reconciled them with a figure already lying there. The ceiling is honoured by
the pieces; everything else is honoured by the **keepouts** below. Both halves
are needed and neither is sufficient.

### The pieces are mostly SHEET GOODS, deliberately

`planks.STOCK` draws a field that is 34% studs by count, because that is what a
stick-built house is made of. A 2x4 laid across a casualty covers 0.12 m of them
and is a stick at any altitude a drone flies at. What actually buries somebody,
and what actually reads from 40 m, is a broken half sheet of OSB or a slab of
roof deck with its shingles on. `_COVER_STOCK` is therefore **not** the field
mix: 52% sheathing, 22% deck, 26% lumber, and the lumber is there to break up
the edges rather than to hide anything.

A quarter of the pieces are **propped** — one end on the debris surface, the
piece rising over the body and overhanging past it. That is the one that gives
an aerial frame a shadow and a depth cue. `planks._box` puts the LENGTH along
local +X and lands that end at `-sin(pitch) * L/2`, so **the high end needs a
NEGATIVE pitch**; a propped piece with a launcher-randomised pitch is a piece
that has fallen off.

---

# Where the bodies go, and the surface they lie on

| class | share | where |
|---|---|---|
| `pile` | 0.36 | on the board mat around the wrecked house, **0.62-1.05 footprints out** — outside the wall line, because the baked pile standing on the slab is a keepout and not a surface |
| `skirt` | 0.28 | the outer mat, 0.95-1.60 footprints out — thinner, so the body has contrast under it |
| `yard` | 0.26 | open ground beside the wreck. The highest-contrast targets in the scene and the control case |
| `street` | 0.10 | the carriageway **in front of this house** — nearest six road points, refused past 45 m |
| `trail` | 1-3 absolute | thrown clear, 8-26 m downtrack of a flattened house. **Only ever placed with budget left over** — `max_total` is spent by the per-wreck loop first, so on a plate with more than ~20 wrecks `trail` places nothing at all |

## The gates every body passes, and why they exist

Every one of these runs at **all three stations** — feet, waist, head — through
the one `_Field.stations` helper, because a lying figure lies entirely on one
side of its placement point and testing that point tests its feet and nothing
else. The refusal tally `plan_people` prints names each of them.

**`min_intensity` — ONLY WHERE THE TORNADO WENT.** `yard` draws 1.15-2.00
footprints off a wreck and `trail` throws a body 8-26 m downtrack, so a wreck on
the shoulder of a 155 m track has much of both annuli outside the corridor. The
first 100 m render laid a casualty on an untouched green lawn two lots away: a
person the tornado did not hit, which is exactly what the whole module was
rebuilt to stop showing. `ctx["intensity_at"]` is `tornado.intensity_field` —
the same field the damage ladder, the scour and the plank scatter are drawn
from, so the planner refuses against the scene's own definition of "in the path"
rather than a second one. Refusals count as **`off_track`**.

The default is **0.12, and it is not this module's own number**: it is
`planks.scatter_over_region`'s `min_intensity`, the floor below which the
corridor lays no boards at all. The rule is therefore "a casualty lies where the
scene put debris", which cannot drift away from the damage the way a second
threshold would. (`tornado._HOUSE_CUTS` cuts `pristine` at 0.08, with a ±0.07
per-house jitter.) Every record now also carries **`intensity`** — the weakest
of its own three stations — so "is anybody outside the corridor" is answerable
from the ground truth alone, without rebuilding the field.

**IT DEGRADES OPENLY.** With no `ctx["intensity_at"]` the gate cannot run, and
`plan_people` says so on stdout rather than placing an unfiltered scene that
looks exactly like a filtered one. The bench and the host tests land there
deliberately.

**The keepouts — AND NOT INSIDE ANYTHING THE DECK CANNOT MEASURE.** `_Deck`
measures the plank field and the archetype mesh tops and *nothing else*, and
`covered_frac` counts only the boards this module lays. Four things can stand
over a body and be recorded as clear; all four are already in the context, and
`_blocker_list` turns them into one indexed circle list (`_Keepout`):

| source | radius | refusal | why |
|---|---|---|---|
| `wrecks` | `0.5 * fp + wreck_clear_m` (0.6) | `in_wreck` | the baked pile IS the tallest thing on a levelled lot, and `roof_collapsed` / `partial_collapse` still have walls — a body inside those is indoors |
| `intact` | `house_clear_m` (7.0 m flat; the ctx carries no footprint) | `in_house` | a standing house |
| `canopies` | the crown radius the launcher passes — `pristine` 4.2 m, `leaning` 3.0 m, `limbed` 2.6 m | `under_canopy` | the 100 m render's `yard` figure was photographed as leaves. Corridor trees are snapped, limbed or down and hide nothing, so only these three levels are passed |
| `blockers` | per feature | `in_relief` | the **scour relief**: 4,631 features on the built 1 km plate, mounds to 0.48 m, authored in step 7a *before* the people pass and previously invisible to it. `tornado_people.relief_blockers` reduces them to the ones tall enough (≥ 0.20 m — a lying body's own top) to swallow a figure, one keepout per polyline STATION rather than one per 11 m windrow |

Measured on `suburb_tornado_1000` (seed 10, TOR_SEED 11): **15 of 40 placed
bodies had a station inside one of these** — 9 in a wreck, 6 in the relief —
before the keepouts existed. After, 0.

**WHAT THE WRECK KEEPOUT COSTS.** No body can be seated on top of the archetype
mound any more, which `deck_points` was added to make possible. That is a
deliberate trade against a blind spot: the archetype deck is sampled one point
per mesh *plan centre*, so a cell beside a standing wall fragment reads as bare
ground and a body is planned at grade beside a wall. If the review wants bodies
back on top of the pile, the fix is a denser archetype sample — not a smaller
keepout.

## `_Deck` — the debris surface is MEASURED now

Every figure in the first render was seated on `DEBRIS_Z_M[level]`, one constant
per damage class, in a field of 755 boards whose real top surface swings from 0
to about 1.4 m over a couple of metres. Some floated, some were buried to the
shins, several stood on the tilted face of a half sheet of plywood.

The launcher now hands the planner **the plank specs it just authored**
(`ctx["plank_specs"]`) and `_Deck` stamps the top of every board's eight corners
into a 0.8 m grid. Then, per body:

- **seated on `max(profile)`, not the mean.** A body laid at the mean of its
  three stations is INSIDE whatever board stands above that mean. Laid on the
  maximum it rests on the highest board and merely bridges the lower ones,
  which is what a body on a plank mat actually does;
- **refused if the three stations disagree by more than `max_deck_tilt_m`**
  (0.26 m over ~1.8 m, about 8 degrees);
- **laid along the FLAT DIRECTION.** Eight bearings are tried per candidate
  point and the flattest is kept. A plank mat is not flat but it is not
  isotropic either — `planks._lay` aligns boards weakly across the flow — so at
  most points there IS a bearing along which a metre and a half of surface
  agrees. Measured on the bench's own field this took `pile`/`skirt` acceptance
  from about 1 candidate in 12 to better than half, and it is also what a body
  does: something that comes to rest on rubble settles along the surface rather
  than across it.

`DEBRIS_Z_M` survives as the **fallback** for the bench and the host-side tests,
which have no board field; with it the deck is flat and the tilt test cannot
fail. Say so if you are quoting a result that used it.

---

# The API

## The planner — `scene_gen/disaster/tornado_people.py`

A **pure planner**: no stage, no Isaac imports, no USD. The whole plan runs and
asserts on the host, which is how the geometry is actually tested.

    cfg = tornado_people.resolve_cfg(scene_config)   # merges a `people:` block
    humans, debris, records = tornado_people.plan_people(cfg, ctx, rng)

`ctx` keys, and who owns each:

| key | shape | owner |
|---|---|---|
| `wrecks` | `[{x, y, fp, intensity, level}]` — sites the bodies AND is a keepout | the assembly launcher |
| `road_pts` | `[(x, y, tangent_deg)]` | the assembly launcher, off `binfo["net"]` |
| **`plank_specs`** | the list handed to `planks.build` | the assembly launcher |
| `deck_points` | `[(x, y, top_z)]` off the archetype meshes, **optional** | the assembly launcher |
| **`intensity_at`** | `f(x, y) -> 0..1`, **optional** — the `min_intensity` gate | `disaster.tornado` |
| `intact` | `[(x, y)]` houses still standing, **optional** — keepout | the assembly launcher |
| `canopies` | `[(x, y, r)]` crowns that survived, **optional** — keepout | the assembly launcher |
| `blockers` | `[(x, y, r[, why])]`, **optional** — keepout; the scour relief via `relief_blockers` | the assembly launcher |
| `throw_deg` | float | `disaster.tornado` |
| `region` | `(x0, y0, x1, y1)` metres, **optional** | the assembly launcher |
| `humans` | `[usd]` RIGGED RenderPeople | `suburb_scene.AssetPools` |
| `resolver`, `asset_pools` | | `scene_generator`, `suburb_scene` |

**PASS `region` OR THE PLANNER WILL LAY SOMEBODY HALF OFF THE EDGE OF THE
WORLD.** A lying figure extends its whole reach past its placement point, so
feet a metre inside the boundary still put a head outside it. Every body is
tested at three stations. **Dropped, not clamped** — same argument as
`planks.clip_to_region`: clamping piles figures along the boundary in a line,
which is a worse artefact than the one it fixes. `plan_people` prints its
refusal tally so a scene that comes back short says why:

    off_plate  too_close  deck_tilt  wrong_surface     geometry and spacing
    off_track                                          outside the corridor
    in_wreck   in_house   under_canopy   in_relief     inside an occluder

Measured on `suburb_tornado_1000` (seed 10, TOR_SEED 11, host-side, no
`deck_points`): `in_relief` 13, `in_wreck` 8, `off_track` 1, `too_close` 22,
`wrong_surface` 58 — 40 casualties placed either way, because `max_total` (40)
binds long before the 56 wrecks run out. **On a plate with fewer wrecks these
refusals come straight off the head count.**

Returns:

- **`humans`** — `category: "human"` placements carrying `pose`, `roll_deg` and
  `pitch_deg`, ready for `sg.apply_placements(..., instance_categories=set())`
- **`debris`** — the plank specs that do the occluding, to be authored with
  `disaster.planks`
- **`records`** — ground truth, one dict per casualty:

      x, y, z, pose, attitude, where, intensity, yaw, body_axis_deg,
      reach_m, alive, visibility, occlusion, covered_frac, sunk_frac,
      visible_parts, boards, note

`summarise(records)` gives counts by attitude, occlusion, visibility, location
and pose, plus `max_covered_frac`.

`plan_catalogue(cfg, ctx, rng, poses, patterns, origin, step)` lays one casualty
per (pose, pattern) cell of a grid through **the same code path**, and returns
`(humans, debris, records, cells)`. That is what the bench photographs.

## Wired into the scene

`suburb_tornado_launch_script.py`, step **7b**, **after** the scour — a casualty
is not debris, and every pass before it moves, deletes or re-materialises
something. Env knobs:

| var | default | what |
|---|---|---|
| `TOR_PEOPLE` | 1 | 0 disables the whole pass |
| `PEOPLE_JSON` | `$ARCH_DIR/humans_<seed>.json` | ground truth |
| `PEOPLE_SNAPS` | 0 | photograph this many casualties CLOSE IN (11 m top-down, 9 m oblique), named after the record. **Sorted worst-first since 2026-08-28**: `p00` is the most-covered body in the scene, so the first frame is the counter-example if there is one |

**`PEOPLE_SNAPS` is how you review this pass.** A 100 m plate at
`views_around`'s 60 m default puts a body lying flat at about twenty pixels,
which is exactly the range at which a scene full of standing commuters shipped
and nobody could tell.

## The bench — `tornado_people_preview_launch_script.py`

Two units, `UNITS=GRID,HOUSE`:

- **GRID** — the catalogue. 8 attitudes x 13 occlusion patterns on a
  3.4 x 4.6 m lattice over **mud**, every body laid along +X so a row reads left
  to right and a column compares like with like. Photographed one row at a time
  in two halves at 13 m, close enough that a hand is a hand.
- **HOUSE** — one wrecked archetype, a real `planks` field scattered off it, and
  the real `plan_people` running over that field **with `plank_specs` in the
  context** — so what is photographed is the code the assembly runs, including
  the measured deck and the tilt refusal. Every casualty gets its own close
  top-down and oblique, plus a wide pair of the whole plate.

## The host tests — `scene_gen/tests/test_tornado_people_poses.py`

**21 checks**, no Isaac, no `pxr` (the pose table is read as SOURCE and
`literal_eval`'d). The load-bearing ones:

- `test_03` — **no limb of a lying figure stands up**, analytically, through the
  real deltas and the real lay-down rotation.
- `test_04` — the lateral poses swing about X and start each arm with the plumb
  correction; the flat ones swing about Y.
- `test_10` — nothing is ever fully buried, over five seeds.
- `test_11` — the pattern means what it says: `legs` covers the legs and leaves
  the head, `all_but_head` leaves a head.
- `test_13` — a covering piece clears the body's crest and does not float.
- `test_08` — every figure is a casualty AND every attitude is horizontal, so
  neither the standing population nor the seated one can come back by accident.
- `test_17` — no body is inside the board it lies on.
- `test_18` — turning the tilt cap down places FEWER bodies (with `max_total`
  off the critical path, or both runs simply hit the cap and the comparison
  says nothing).
- `test_20` — every occlusion pattern the scene can DRAW fits under
  `max_covered_frac` untrimmed, so no record can carry a name that the trim has
  made untrue.
- `test_21` — **nobody is placed outside the track**, at all three stations,
  against a synthetic corridor. Asserts the gate-OFF run *does* put stations
  outside it (71 of them), so an unwired gate cannot pass.
- `test_22` — **nobody lies inside a wreck, a house, a crown or a relief
  feature**. Same gate-off control (9 of 40 bodies).

`test_11` runs at an explicit `max_covered_frac` of 0.90 and that is deliberate:
it checks what each pattern NAME means, and four of them are wider than the
scene's own 0.55 cap. The cap is `test_10`'s job and the pairing is `test_20`'s.

---

# Traps

**ONLY RIGGED HUMANS CAN TAKE A POSE.** `posed_standing` assets are static
meshes frozen in one attitude; binding `lying_side_l` to one does nothing at all
and the figure ships upright, silently. `_rigged_humans()` in the launcher makes
the same selection `disaster.people.build_ctx` does.

**A LYING POSE PLACED UPRIGHT IS REFUSED.** `people._human_placement` raises
rather than composing: authored upright, `lying_prone` is a figure with an arm
stretched over its head.

**`_measure`'s `sy` IS THE A-POSE BBOX DEPTH** and it is used as the lying
body's thickness. For a posed lying figure with a raised knee the true
silhouette is deeper — which is what `_BODY_RISE` exists for, and if boards over
the thighs read as sunk into them, that table is the number, not this one.

**THE RECORD'S x/y ARE 3 DP, NOT 2, AND THAT IS DELIBERATE.** `_Deck`'s cells
are 0.8 m with hard edges, so a centimetre of rounding can move a body's
recorded position into the next cell and make the ground truth disagree with the
surface it was actually seated on.

**`DEBRIS_Z_M` IS STILL AN ESTIMATE.** It is only reached when there are no
plank specs. Do not quote a bench result about float or sink as though it
measured the assembly.

**BODIES STILL CANNOT SEE THE ARCHETYPE PILE — THEY ARE KEPT OUT OF IT
INSTEAD.** `_Deck` reads the plank field and `deck_points` (one sample per
archetype mesh, at its plan centre), which is sparse enough that a cell beside a
standing wall fragment reads as bare ground. Since 2026-08-28 the wreck's own
footprint is a **keepout** (`in_wreck`) and `pile` starts at 0.62 footprints
rather than 0.42, so bodies lie on the mat AROUND the mound and never in it.
That is also the likelier truth — a body in the middle of the deepest material
is invisible from any angle — but it is a refusal, not a model of the pile, and
it means nothing can be seated on top of one either.

---

# What is deliberately absent

**Every upright figure who was not hit** — `neighbour_dig`, `on_the_rubble`,
`street` walkers, `assisted` trios, `in_vehicle` occupants. Removed 2026-08-27.
Nothing about the research behind them was found to be wrong: Greensburg's 68%
of households doing SAR, Bartolucci et al. 2020's 60-100% civilian extrication
share, Paulikas, Schmidlin & Marshall 2016's 959-vehicle displacement rates
(EF5: 69% moved, 31% tipped — **unconditional**, not 31% of the 69%), Daley et
al. 2005's severe-injury **OR 0.2** for fleeing by car. They are gone because
the benchmark scores the people the tornado HIT, and because they are the easy
problem. The code is in git history if a distractor population is ever wanted.

**`parking_refuge`** — the largest WILDFIRE scenario at 0.30, and it has no
tornado counterpart. Moore, Joplin and Mayfield all had **no public tornado
shelters**. Madison County AL — Huntsville, among the most tornado-conscious
counties in the US — lists 14 safe rooms totalling ~2,700 capacity, all in the
rural fringe, and states the cities of Huntsville and Madison have none.
Observed public-shelter use is **~4% where one exists and 0% where none does**.

**`gridlock`** — across six metros over 2011-2018 only Oklahoma City ever
produced a mass traffic reversal (Hatzis & Klockow-McClain 2022), it was
broadcast-directed, and it is a metro-arterial phenomenon. The authors state
plainly they *"have no way to quantify the number of people who actually
evacuated"*.

**`pools` / `cul_de_sac`** — wildfire refuge geography. A pool is shelter from
radiant heat and nothing at all in a wind event.

**Cellars, safe rooms and house interiors** — real, and where people actually
shelter, but a drone benchmark cannot score a target it cannot see.

**Fully buried figures** — same argument, now enforced by `_trim_spans`.

**Figures under an intact tree canopy, and figures outside the track** — same
argument again, and since 2026-08-28 actually enforced, by `_Field.clear_of`
and `_Field.in_track`. A body on a mown lawn beside an untouched house is a
person the tornado did not hit; a body under a crown is one no camera can see.
(On `suburb_tornado_1000` no body was ever drawn under a crown — the corridor's
trees are down — so the canopy keepout costs nothing there and is still the
cheapest insurance in the file.)

**`trapped_sit`, the one upright casualty** — cut on the second review. See
*Attitude* above.

**Bystanders and onlookers at the track edge** — real (40% of people go outside
to look during a warning, Sherman-Morris 2010, n=2,921) but out of scope by
request.

**Responders, heavy equipment, search markings, triage tents** — T+12-24 h
artefacts. See the epoch.

---

# Known gaps and open decisions

- **`covered_frac` STILL ONLY COUNTS THIS MODULE'S OWN BOARDS.** The keepouts
  mean nothing else should be over a body, but "should not be there" and
  "measured" are different claims. Anything the launcher does not pass —
  fences, cars, street furniture, the archetype's own overhanging fragments —
  is unmodelled, and the only way to check any of it is `PEOPLE_SNAPS`.
- **`trail` is effectively dead on a large plate.** `_trail` runs on
  `budget - made`, and the per-wreck loop spends `max_total` first: 56 wrecks
  on the 1 km plate means zero thrown bodies. If the class is wanted, it needs
  a reserved share of the budget rather than the remainder.
- **The wildfire module's version of the same gate is spelled differently.**
  `disaster/people.py` uses `min_burn_age_s` against `ctx["age"]` and drops the
  head count outright; this one refuses a candidate and retries, so the head
  count holds while the class mix moves. Same request behind both.

- **`_BODY_RISE` is estimated from the pose arithmetic, not measured off a
  render.** A board that floats over a body is that table too high; one that
  cuts through a knee is it too low.
- **`_lying_lift`'s 0.115 H lateral half-breadth is MODELLED.** Same status.
- **Demographics are not modelled.** 60+ are **39.7%** of tornado fatalities and
  65+ are 28% against 12% of exposed — a 5x per-capita rate. If the
  RenderPeople pool has older figures, casualties should prefer them. Nothing
  does this today.
- **`casualty_share` is a coin flip, not a model.** A quarter of the records are
  `alive: false`. 86.6% of tornado deaths are on-scene (Chiu et al. 2013) and
  94% in May et al. 2000, so a corridor at T+45 min genuinely holds both — but
  nothing correlates death with occlusion, attitude or location, and it
  probably should.
- **Mobile homes are structurally absent.** 38-47% of US tornado deaths against
  ~5.4% of housing stock, and a **~10x exposure-adjusted** per-capita risk
  (Fricker & Friesenhahn 2022). Our suburb is entirely site-built modular kit.
  **A small manufactured-home park is the single highest-value asset addition
  available.**
- **Time of day is not a parameter.** Peak tornado hour is 16:00-17:00 CST and
  the scene is bright daylight, which is self-consistent — but a nocturnal
  scene would invert the mix entirely.
- **No drone has ever been documented finding a live tornado victim.** Lee
  County AL 2019 flew thermal drones; the sheriff's stated value was *negative
  assurance* — confidence nothing was missed — not detection. CRASAR flew
  Mayfield for damage assessment, not search. There is no operational ground
  truth to calibrate against, which is arguably the case for building this
  dataset.

---

# Sources

- **NIST NCSTAR 3** — Technical Investigation of the May 22, 2011 Joplin
  Tornado. 494 pp. All 161 fatalities geolocated; refuge and basement findings.
  https://nvlpubs.nist.gov/nistpubs/NCSTAR/NIST.NCSTAR.3.pdf
- **CDC MMWR 61(28):529-533** — Tornado-Related Fatalities, Five States,
  Southeastern US, April 25-28 2011. The injury-location vs recovery-location
  table (90.5% / 3.3% / 37.0%).
- **Chiu et al. 2013**, *Am J Public Health* 103(8) — Alabama 27 Apr 2011, 247
  decedents. Room-level locations; who victims were found with; 86.6% on-scene.
- **Niederkrotenthaler et al. 2013**, *PLoS ONE* 8:e83038 — 1,398 patients, 39
  hospitals. Injury severity distribution.
- **Hammer & Schmidlin 2002**, *Wea. Forecasting* 17:577 — 190 occupants of 65
  F4/F5-damaged homes, OKC 1999. Shelter-room shares; 47% fled; 0 of 90 injured.
- **Brown et al. 2002**, *Wea. Forecasting* 17:343 — OKC 1999; "picked up /
  blown" mechanism shares.
- **Daley et al. 2005**, *Am J Epidemiol* 161:1144 — odds ratios including
  fleeing by vehicle.
- **Paul & Stimers 2014**, *Wea. Climate Soc.* 6(2) — Joplin mortality by damage
  zone, with population denominators.
- **Paulikas, Schmidlin & Marshall 2016**, *Wea. Climate Soc.* 8:85 — 959
  vehicles, displacement and rollover rates by EF.
- **Karstens et al. 2013**, *J. Appl. Meteor. Climatol.* 52 — 10,300 + 94,500
  tree falls; convergent near-surface flow.
- **Snow, Wyatt, McCarthy & Bishop 1995**, *BAMS* 76:1777 — long-range debris
  fallout; the 78%-left statistic and what it actually applies to.
- **Bartolucci et al. 2020**, *BMJ Global Health* — civilian vs professional
  extrication across earthquake events.
- **Fricker & Friesenhahn 2022**, *Wea. Climate Soc.* 14:75 — fatality location
  shares 1995-2018; exposure-adjusted mobile-home risk.
- **Sherman-Morris 2010**, *Natural Hazards* 52:623 — n=2,921; 40% went outside.
- **Hatzis & Klockow-McClain 2022**, *Wea. Climate Soc.* 14 — the 31 May 2013
  OKC traffic reversal, and its limits.
- **Drillis & Contini 1966** — the anthropometric segment fractions (`_LANDMARK`,
  the biacromial breadth behind `_lying_lift`), as reproduced in Winter,
  *Biomechanics and Motor Control of Human Movement*.
- **FEMA/DHS 2011** — Joplin Lessons Learned Study; response timeline.
- **Paul, Che, Stimers & Dutt** (Natural Hazards Center QR) — Greensburg; 68% of
  households participated in SAR, with timing.
