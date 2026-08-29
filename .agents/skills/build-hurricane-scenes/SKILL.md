---
name: build-hurricane-scenes
description: Build HURRICANE-damaged scenes in scene_gen — the near-uniform wind field with a rotating direction (not a track), the roof-first failure ladder that tops out where the tornado ladder starts, the surge/inundation water rendered as static geometry with a muddy MDL, and the water-damage signatures (mud line, windrow, scour) that separate a hurricane from a tornado from the air. Read before creating disaster/hurricane.py, a hurricane surge/water module, or any hurricane launcher. Prerequisites - build-tornado-scenes (the wind mechanics) and simulate-water-in-isaac-sim (the water tiers).
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Hurricane Scenes

**Status: DESIGN, nothing built yet.** This file is the living record. It is
written before the code so that the code has something to be wrong about.
Update it as things are built and as bugs are found — that is the point of it.

## Prerequisites, in order

1. [build-tornado-scenes](../build-tornado-scenes/SKILL.md) — the wind damage
   machinery (`wind_flow`, `planks`, `settle` bias, windthrown trees) is 80% of
   the hurricane's structural half. Read it and its prerequisite,
   [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md), first.
2. [simulate-water-in-isaac-sim](../simulate-water-in-isaac-sim/SKILL.md) — the
   three water tiers. Tier 1 (static plane + water MDL) is the answer here.
3. [freeze-disaster-dataset](../freeze-disaster-dataset/SKILL.md) — hurricane is
   one of the four disasters in the 4 x 2 x 3 x 5 matrix and it is the ONLY one
   with no pipeline. Both its cells (Urban, Suburban) are gaps.

---

# What exists today (verified 2026-08-28)

Almost nothing. The word "hurricane" appears in exactly seven places and every
one of them is the LEGACY generic-damage path, not the archetype pipeline that
built the fire, tornado and earthquake scenes:

| where | what it is |
|---|---|
| `scene_gen/config/presets/hurricane.yaml` | 9 lines: `locale: downtown`, `disaster-type: hurricane`, `severity: 0.6` |
| `scene_gen/config/low_level/compiled/hurricane.yaml` | the compiled output of that |
| `scene_gen/compile_disaster.py:545` `compile_hurricane` | prop-toppling fractions + `field: {kind: uniform, inside: 1.0}` |
| `scene_gen/GENERATION.md:462` | "Tornado-like mechanisms spread evenly over the whole region at lower intensity. No untouched zone, no track." |
| `scene_gen/disaster/gt_hints.py:72,104` | `EXTRA_CLASSES["hurricane"] = ("Pool", "Parking Lot")`; tree classes copied from tornado |
| `scene_gen/tests/test_scene_modularity.py` | compiles the preset |
| `.agents/skills/freeze-disaster-dataset/SKILL.md:199` | "Hurricane — NOT DESIGNED" |

**The legacy `compile_hurricane` is not wrong, it is thin.** Its one real
insight — that the field is UNIFORM, with no untouched zone and no track — is
correct and survives into the design below (see "The model"). What it lacks is
everything else: no archetype ladder, no roof-cover damage state, no water, no
directional model, no tree model, no urban story.

---

# The plan documents

The research is long and it is not repeated here. This file carries the
DECISIONS; those carry the evidence.

| file | what |
|---|---|
| `scene_gen/_plans/hurricane_research.md` | the hazard and building-damage synthesis — the EF/HAZUS failure ladders, code eras, the tree model, the debris composition, the per-category scene recipe, the aerial signatures, 44 sources |
| `scene_gen/_plans/hurricane_water.md` | the WATER half — what post-surge water looks like, the six-layer static design, the material numbers, the GT contract, 12 risks with offline checks |
| `scene_gen/_plans/hurricane_wind_field.md` | the parametric wind field and the quantified 1 km gradient |
| `scene_gen/_plans/hurricane_survey.md` | the code survey, 2,836 lines — the three-tier spine and the `pxr` boundary, per-disaster call sequences launcher-to-stage, verified signatures + the one trap each for every shared module, the 544 MB material inventory, eight offline test patterns, the house record, `modular_house` strip-ability, and the ranked build plan |

---

# THE MODEL — what a hurricane IS, spatially

A tornado is a LINE and the damage is a function of distance from it. A hurricane
is not a line, and the single most common wrong turn is to reach for the tornado
field with a wider corridor.

## The SMOOTH field is uniform — and that is now measured, not asserted

`compile_hurricane` returns `field: {kind: uniform, inside: 1.0}` and
`GENERATION.md:462` says "no untouched zone, no track." **That is correct for the
parametric field**, and the numbers are in `_plans/hurricane_wind_field.md` S2.1,
computed from the Holland (1980) gradient-wind equation at 28 deg N for four
representative storms (Rmax 20/35/50/60 km, dp 25-90 hPa, B 1.1-1.6):

    radial gradient-wind change over 1 km ......... 0.1 - 0.8 m/s   (0.3 - 1.5 %/km)
    ... exactly at Rmax ........................... < 0.05 m/s/km
    azimuthal WN1 change over 1 km ................ < 0.15 m/s

1 km of arc at r = 30 km subtends 1.9 deg, which is why the forward-motion
asymmetry contributes almost nothing across a plate. **The smooth field is uniform
to better than 1 m/s (~1%) against a Saffir-Simpson category step of 7-12 m/s —
the plate cannot straddle a category boundary.**

## BUT the field is not the only spatial term, and I was wrong to imply it was

Two terms are an order of magnitude larger than the radial gradient, and both are
documented. Neither is a reason to reintroduce a track.

**(a) The coastal roughness step — a REAL wind gradient, and our scenes have one.**
Vickery, Wadhera & Powell (2009): *"at a distance of about 1 km approximately 60%
of the transition (or wind speed reduction) has already occurred"*, with a maximum
mean-wind reduction of ~17%. **The first 1 km inland of a coastline loses ~10% of
the mean wind** — independently reproducing NHC's own statement that peak winds
"diminish by one category within a short distance, perhaps a kilometre of the
coastline."

> **This matters here specifically because the surge design puts the shore on or
> near the plate.** A hurricane scene that straddles a shoreline or a forest/city
> edge has a genuine ~10% wind gradient across it — an order of magnitude larger
> than the radial term. A scene sitting entirely inside one roughness class does
> not. Tie the damage ladder to `exposure` and distance-from-shore, not to
> distance-from-storm-centre.

**(b) Boundary-layer roll vortices — a defensible optional band.**
Wurman & Winslow (1998) found intense sub-kilometre rolls in Fran; Morrison et al.
(2005) found them in **35-69% of radar volumes** — the normal state, not an
exception; Foster (2005) quantifies **~14 m/s periodic horizontal variation over
~725 m**.

    BOUNDARY-LAYER ROLL MODULATION ... ~14 m/s peak-to-peak over ~725 m
    -- 20 to 70x the radial term, comparable to a whole category step

If a second spatial term is wanted, this is the physically justified one:
orientation = mean wind rotated **-10 deg** (toward the storm centre); wavelength
**700-1500 m** (sub-km inside Rmax, 1-2 km outside); amplitude up to **+-7 m/s**,
i.e. bands roughly one damage step apart.

> **Caveat, and it is why this stays OPTIONAL.** The chain "rolls modulate surface
> wind" -> "modulated wind produces visible damage streaks" is asserted by the
> primary authors and documented in specific cases (Andrew's streaks at Naranja
> Lakes and Cutler Ridge; the CSWR Gustav streak/damage correlation), but **there
> is no published quantitative fragility study mapping roll bands onto damage-state
> fractions.** Include it as a low-amplitude visual overlay, never as the primary
> driver. It does not conflict with "construction dominates" below — it is a
> modest systematic band on top of a much larger construction-driven random field.

## So what creates spatial variation? THE BUILDINGS, not the field.

This is the whole design. Three findings from the damage research, in order of
how much they change the code:

**1. A "Cat 4 landfall" is NOT 130 mph at the houses you are rendering.**
Marshall's aerial survey of **11,105 structures** after Katrina found the
damage-derived gusts across the whole impact zone averaged **41 m/s (92 mph)**,
with excursions to 48 m/s — Cat 1-2 *local* intensity inside a storm rated Cat 3
at landfall, because "most buildings were in Exposure B not C, and were well
below 10 m." Suburban Exposure B cuts the 10 m gust 13-15% against open terrain.

> **Drive the compiler from a LOCAL SITE GUST. The category is a label the preset
> carries for the record, not the driver.**

**2. THE VARIANCE IS THE SIGNAL.** Marshall says it in three separate storms:
"flaws in the construction of the [structure] or poor attachment/anchoring can
better explain such sharp gradations in the damage rather than a rapid change in
wind speed." The code-era data proves it from the other side — post-1994
manufactured homes showed **0%** damage above rating 1 in the same wind that put
**64.7%** of pre-1976 units at rating >=2.

> **A scene where damage varies smoothly with a field will look WRONG. A
> destroyed house next to an almost-untouched one is the NORM.** The per-building
> draw must be dominated by a per-building CONSTRUCTION-QUALITY term. This is the
> exact inverse of the tornado, whose whole signature is a smooth cross-track
> gradient — and it means the hurricane needs a per-house `code_era` /
> `build_quality` attribute that no other disaster in this repo has.

**3. The wind DIRECTION ROTATES.** Bunting & Smith: "the direction of failure of
a building component, tree, or other object usually indicates direction of the
wind at failure." Marshall dated individual failures at Keesler AFB during Katrina
by the rotation **060 -> 070 -> 080 -> 100 -> 110 deg** over seven hours.

> Debris is neither aligned nor random: **one dominant azimuth, +-30 to 60 deg of
> spread, and a second ~180 deg population if the eye passed over.** Contrast the
> tornado, whose treefall CONVERGES on the centreline.

## The compiler's parameter set

Not a track. The minimal set is:

    site_gust_mps       the local 3-s gust at 10 m -- THE driver
    heading_deg         dominant azimuth of the strongest gust episode
    rotation_deg        spread about it (+-30..60); 180 for an eye passage
    exposure            B (suburban) | C (open) | D (coastal)  -- scales the gust
    surge_m             independent of category; see the water plan
    shore_deg/_offset_m where the water comes from -- ALSO drives the ~10%/km
                        roughness gradient, which is the only real wind gradient
    code_era_mix        the per-building quality draw -- the variance term
    streak              none | roll | mesovortex | tornado -- OPTIONAL banded
                        overlay, one damage step worse. roll: 700-1500 m
                        wavelength, aligned -10 deg off the mean wind

---

# BUILDING DAMAGE — the ladder, and where it differs from the tornado

## The hurricane ladder is the tornado ladder EXTENDED DOWNWARD

`wind_flow.BREAK_PLAN` runs `pristine -> roof_stripped -> roof_collapsed ->
partial_collapse -> leveled -> swept`. For a hurricane, **the top of that ladder
is nearly unreachable and the resolution has to go at the bottom.**

Two hard numbers: Marshall's 8,119 residences on post-Katrina aerials — **90%
lost <20% of roof cover; 10% lost most of the cover and/or some decking; 3 of
8,119 (1 in 2,700) lost large sections of roof structure.** And for 116-135 mph
gusts (~Cat 2-3 local): "**less than 15 percent of homes sustained structural
wind damage**."

So the states a hurricane actually needs, and none of them exist today:

| new state | onset (3-s gust) | geometry |
|---|---|---|
| `cover_partial` (<20% shingles) | 79 mph / 35 m/s | material only — no fracture |
| `cover_major` (>20%, soffit, siding) | 97 mph / 43 m/s | material + a deactivated soffit/trim |
| `openings` (windows, garage door) | 96-97 mph / 43 m/s | asset swap + `SetActive(False)` |
| `deck_partial` (1-3 sheathing panels) | ~100 mph / 45 m/s | **drop 1-2 roof BAYS** |
| `deck_major` (>25%) | 115-140 mph / 51-63 m/s | drop most bays |
| `roof_structure` | 122 mph / 55 m/s | ~= existing `roof_collapsed` |
| `walls` | 132-170 mph / 59-76 m/s | ~= `partial_collapse` / `leveled` |

**The first five are all BELOW the existing ladder's first rung.** `roof_stripped`
is already "covering AND sheathing peeled off; structure and walls whole" — a
hurricane needs three or four distinct states inside that one.

## THE PROGRESSION IS ROOF-DOWN. This is a hard rule.

Marshall (2004): "dismantling of a structure by wind usually develops first at
roof level and progresses downward with stronger wind velocities... because wind
velocity increases with height."

> **Never author wall damage on a house whose roof is intact.**

## `wind_flow.py` already contains the hurricane statement

`wind_flow.py:305-320` deactivates doors and garage doors **before** anything
structural:

> *a pressure differential takes the openings before it takes the structure,
> which is why a house at EF1 has its garage door in the back garden and its
> roof still on.*

That is the internal-pressurisation cascade, already written. Complete it:
**ASCE 7 Table 26.13-1 — GCpi jumps +-0.18 -> +-0.55 on breach, ~3x the internal
pressure**, which then adds to the external suction on the roof. The consequence
for the renderer is that **the failure is ASYMMETRIC AND ORIENTED TO THE BREACH**:
a house with a blown-in garage door shows the *garage-side* roof plane stripped,
not uniform damage. `wind_flow.py:216-224` records that it falls back to a random
draw for the windward face *because it does not have the inward normal* — and
`suburb_parcel.py:2554-2558` publishes exactly that as `n`. Wire it in.

## SLAB SWEPT CLEAN IS SURGE, NEVER WIND

Four independent lines say so. Roueche et al. on 3,016 hurricane homes: complete
destruction in 94 (3.1%), and "**all the homes that suffered complete destruction
in hurricanes were associated with storm surge, not high winds only**." EF FR12
DOD10 puts wind-only slab-sweeping at **200 mph** — above Cat 5 and unreachable
at a suburban site. In tornadoes complete destruction was 4.2% and those WERE
wind-only.

> **`swept` is a TORNADO level. For a hurricane, bare slabs appear only inside
> the inundation footprint, in a coherent shore-parallel band — never scattered
> inland.** This is a free, strong class separator for the dataset, and getting it
> wrong makes a hurricane scene read as a tornado.

## Damage states that are nearly free on the modular house

`detail/modular_house.py` publishes per-house prims under `{parent}/inst/h_{i}`
(the scoping `suburb_tornado_launch_script.py:361,370` already uses):

| want | cost |
|---|---|
| **partial roof loss** (the Cat 1-2 signature) | the roof is **per-BAY meshes** — one `Roof_01` per fully-covered 10x10 block (`:721-723`), `Roof_Half_01` per leftover 5 m column (`:726-727`). Dropping a bay is one `SetActive(False)`. `validate_roofs:1527` proves coverage offline |
| **windows blown out** | windows are whole WALL pieces, not prims. Swap the placement `usd` from a `_Win_` piece to `WALL_5["plain"]` — `damage._fragment_assets:271-277` already does this |
| **garage door gone** | `house_door_2_9` = `Garage_Door_02offset`, `SetActive(False)`; `wind_flow` already does it |
| **porch torn off** | deactivate `*_bay_roof_*`, swap that wall `Outer_Wall_Quart_Door_03` -> `Outer_Wall_Quart_01` |
| **individual gable plane** | NOT free. `Roof_01` is one mesh, the two slopes are not separate subsets. Use `fracture.fracture_prim(..., mode="plank", aspect=(1.3,2.6))` as `wind_flow.py:275-292` does |

Absent from the kit house: **no chimney, no railing, no gutter, no deck, no
foundation.** Chimney and gable-end failure — both textbook hurricane modes —
have **no geometry to fail.** Same finding the earthquake pipeline hit.

Prim counts: cottage 13, villa 18, ranch 23, terrace 38, l_family 50.
`_place` sets `raw_pivot: True` (`:417`) — do not let `apply_placements` recentre
a house or it comes apart.

## Trees are the strongest class separator in the dataset

**Broadleaf canopy is stripped essentially bare at Cat 3+ (86-94% leaf loss);
conifers keep their needles and SNAP.** After Maria, NDVI dropped ~0.2 and
recovered in ~1.5 months — **a scene hours after landfall is firmly inside the
brown window.**

| | HURRICANE | TORNADO | EARTHQUAKE |
|---|---|---|---|
| trees | **standing but BARE and brown**; pines snapped mid-trunk into same-height spars; street trees uprooted | **debarked**, stubs, convergent fall | **untouched, green** |

Species survival is measured and maps onto the repo's existing species list:
sand live oak 99%, live oak 91%, **sabal palm 80%** (and flat at 92-93% right
through Andrew at 165 mph — palms shed fronds without losing the meristem, so
render the "shaving-brush" look), slash pine 72%, loblolly 66%, longleaf 59%.
**Sand pine 4%.**

**Rooting space is the strongest URBAN predictor**: survival **64%** at 0-3 m2
(street tree pits, parking-lot islands), **73%** at 4-7 m2 (yards), **91%** at
>7 m2. Grouped trees beat isolated ones 80/70.

> **Street trees in pits are the ones DOWN; grouped back-yard and park trees are
> the SURVIVORS.** That maps directly onto the existing suburb yard/park split.

Urban forest MORTALITY by storm: Erin 11%, Georges 13%, Jeanne 16%, Charley 18%,
Rita 21%, Katrina 23%, **Andrew 38%**.

## Debris is 70% VEGETATION — the opposite of the tornado plank field

Lee County's Ian collection: **734,136 yd3 vegetative vs 285,282 yd3 C&D, a 72/28
split.** And the building 30% is **COARSE** — whole 4x8 sheathing panels, intact
fence sections, whole shingles, long siding strips — **locally sourced and
short-travelled**, landing within 1-3 building widths in the lee of its origin
and against the first obstruction downwind.

> The tornado skill's "the scene is PALE, not dark" still holds, but the GRAIN is
> different: a tornado shreds, a hurricane peels. `planks.py` gives sawn timber;
> a hurricane needs **whole panels and sheets**, which is a stock-list change, not
> a new module.

## Vehicles: NOT thrown by wind

The complete record from 30 hurricane surveys: nothing below ~130 mph gusts;
Charley (130-145) "a few automobiles were flipped"; Andrew (175) "several
automobiles had shifted, rolled, or flipped." **Aircraft go long before cars.**

> **0 wind-moved cars below ~120 mph. Cars are moved by WATER** — 30 cm floats a
> car, 45-60 cm carries most vehicles away — and they end up **against
> obstructions, in groups**, not scattered. `tornado.car_pose` is the wrong model.

## BLUE TARPS: NO

USACE Operation Blue Roof: Ida activated D+3, **first install D+10**, 30,000th on
day 42. Ian program began **D+5**. The iconic tarp density is **D+14 to D+60**.

> **An "immediately after" scene renders ZERO tarps.** A tarp variant is a
> separate TEMPORAL class at D+14, with tarps on ~30-60% of roofs that have cover
> damage but intact structure — never on a roof with the structure gone.

---

# WATER — static geometry, no simulation

Full design in `_plans/hurricane_water.md`. The decisions:

## The water skill's recommended material is WRONG, and this is verified

`simulate-water-in-isaac-sim/SKILL.md` says "flood water is brown, not Caribbean"
(line 64) and then points at a material that **cannot be made brown**. Reading
`Water_Blue_Ocean_Perlinwaves.mdl:193`: `water_tint` lerps **hue 0.47 -> 0.53** in
HSV (cyan to blue), saturation pinned 0.5; the scattering colour is the
hard-coded literal `(0.306, 0.448, 0.656)`; and `material_surface` is a pure
`specular_bsdf` with **no diffuse lobe and no opacity control.** Also, the
`Water_Ocean_` variant that skill's table lists at line 52 **is commented out**
(lines 363-386).

> **Do not use the vendored vMaterials water for floodwater.** Use
> `OmniPBR_ClearCoat` — a Fresnel coat at `clearcoat_ior 1.333` over a diffuse
> sediment body — which is core Kit MDL already in the container and needs no new
> vendored asset. Numbers in the water plan S3.7.

Floodwater is **opaque**: USGS gauges measured **100-1,300 FNU** after Harvey and
Florence against <5 NTU for clear water. Past 30-50 cm the colour is
depth-independent. Body colour, linear: **(0.155, 0.115, 0.070)**.

## The nadir correction that decides the whole render budget

At nadir a water film's Fresnel reflectance is **~2%** — the *minimum* of the
curve — and the sun's specular lobe is centred on the mirror direction, so **a
nadir camera never sees sun glint off a flat wet road.**

> **Spend the budget on ALBEDO and PUDDLE GEOMETRY, not on making the road a
> mirror. A wet road rendered as a mirror at nadir looks wrong.** Standing water
> IS a genuine mirror and does return sky even at nadir — that is what carries
> the "flooded" read, not the wet pavement.

## Vegetation does not darken

Dew on bluegrass measures **+4% in the visible** (brighter), -9% NIR. Concrete
goes 0.59 -> 0.36 (ratio 0.61, LBNL-48334). Asphalt sits at the darkening
crossover so its visible change is **gloss, not albedo**.

> **One wetness multiplier across soil, concrete and foliage produces an
> obviously wrong image.** Use the per-class table in the water plan S3.2.

## The six layers, and what they cost

    L0 wet pass          0 prims   -- at BAKE time on ~88 archetypes, not a scene walk
    L1 inundation        1 prim    -- ONE quad + a stretched alpha map
    L2 ponding           2 prims   -- ~1,200 merged puddles
    L3 waterline band    2 prims   -- proud ribbons, NOT a texture rebind
    L4 deposits         22 prims   -- silt overlay + wrack windrows + washover fans
    L5 floating matter   5 prims   -- rafts, foam, sheen
    L6 rain              0 prims   -- NOT AUTHORED; RTX fog instead
    ------------------------------------------------------------------
    total              ~32 prims, ~43k quads, 3 textures (~4.7 MB)

**32 prims against the measured 447,450 of a shipped 1 km cell — 0.007%.** The
water half is nearly free, which is the whole reason to do it statically. **Zero
physics bodies.**

## Two design rules inside that

**L3 is GEOMETRY, not a texture rebind.** Repainting a per-building albedo with a
mud band would opt that building out of instancing (`scene_api.py:190-193`), and
88 prototypes backing ~5,800 objects is why a 1 km plate is 265 MB. A proud
ribbon never touches a building material. Author **two** bands: the wet mud line
at the current surface, and a lighter seed/debris line at the highest reached
level — USGS calls the seed line the more reliable high-water mark.

**L1's shoreline comes from an alpha map, not from bands.** OmniPBR carries one
`texture_scale` for every map it samples, which is what forced `build_overlay`
into bands — but **water needs no tiled diffuse**, so one stretched opacity map
works where it could not for the burn scar, and the ripple normal rides on
`clearcoat_texture_scale`, which is independent.

## Rain is not authored, and that is deliberate

The scene is a frozen instant; rain is motion. A static USD with rain streaks is
a still frame of a video effect, **identical in every render of that scene** — a
detector would learn the streak pattern as scene identity. Everything rain would
contribute is already delivered by the wet pass, the standing water and an
overcast sky. Use **RTX fog** (and `rtx/fog/fogZup/enabled`, because these scenes
are Z-up).

---

# WHAT MUST BE BUILT

Nothing hurricane-specific exists. In dependency order:

| # | thing | notes |
|---|---|---|
| 1 | `tools/water_maps.py` | alpha map + ripple normal, host-side, PNG dump. **No Isaac at all** |
| 2 | `disaster/surge.py` field half | `surge_field`, `alpha_at`, `pond_specs`, `wrack_specs` — pure Python, `pxr` only inside `build`/`materials`, exactly as `scour_relief` splits |
| 3 | `tests/test_surge.py` + `tools/surge_png.py` | the offline gate, in the `test_scour_relief.py` style, with the `strict` decorator |
| 4 | **`compile_hurricane` rewrite** | site gust, exposure, rotation, surge, code-era mix. **Do this FIRST** — until it reads `spec`/`region` every steering knob is a no-op, and emitting a `disaster.hurricane` sub-block also fixes the `_disaster_kind` mislabel |
| 5 | `tools/hurricane_png.py` | the host-side gate, after `tools/tornado_png.py`. **Non-negotiable: nothing reaches a container without it** |
| 6 | **`disaster/hurricane.py`** + **`disaster/hurricane_flow.py`** | TWO modules, not one — the repo has made this split three times (`damage_flow`, `wind_flow`, `quake_flow`) and `wind_flow.py:5-8` states the rule: a separate module rather than a flag, because *"an `if tornado:` running through the middle of `damage_building` would be the kind of change that quietly breaks the wildfire path six months from now."* `hurricane.py` = field + ladders (mirror `tornado.py`); `hurricane_flow.py` = per-building (mirror `wind_flow.py`, but picking the **windward** wall from `h["n"]` against the wind bearing — the fix `wind_flow.py:216-224` asks for) |
| 7 | extend the ladder | the five new sub-`roof_stripped` states |
| 8 | `flood_water_bench_launch_script.py` | flat ground, a road, three houses, one water plane — where the material numbers get tuned |
| 9 | L0 wet pass in `bake.py` | ~88 archetype files, once |
| 10 | bake launcher + `assets/archetypes_hurricane/` | **its OWN directory**, after `bake_tornado_archetypes_launch_script.py` |
| 11 | `gt_hints` extras | `Flooded Road`, `Standing Water`, `Debris Raft`, `Washover Fan` + `water_state` attributes |
| 12 | presets | `downtown_hurricane.yaml` + `suburb_hurricane_1000_l{1,2,3}.yaml`, copied from `downtown_earthquake.yaml` and the tornado `_l1..l3` set |

**One open decision, flagged not answered: which people module.** `tornado_people`
needs `ctx["intensity_at"]`, `ctx["wrecks"]`, `ctx["plank_specs"]`; `people` needs
a share table and a replacement for the `min_burn_age_s` gate at `:1259`. **The
two write different `GT_people.json` envelopes** — reconcile them while choosing.

**And one thing is free today.** `compile_hurricane` already emits non-zero
`damaged_fraction` (0.08 -> 0.4) and `destroyed_fraction` (0.03 -> 0.3), and the
`urban`/`urban_v2` chain ships `buildings.damaged` and `.destroyed` pools — so the
legacy in-`build_city` path places genuine ruins with **no asset work at all**.
(Contrast `urban_quake.yaml:30-31`, which wipes those pools on purpose because the
quake supplies its own archetypes.) **Whether that is good enough for a first
urban hurricane, or whether it needs a full archetype pass, is the open call.**

## The intensity ladder, when it comes

Measured contract from the shipped presets: a level is **a different seed (a
different PLACE) + a different severity + a moved and re-aimed event + one
per-disaster geometry override.** The tornado ladder differs in only **7 of 78**
config leaves; the wildfire size ladder in **3 of 107**.

Proposed, following that shape — **not yet built, and the user sets these when
the cell comes up for export:**

| level | `site_gust_mps` | reads as | `surge_m` |
|---|---|---|---|
| 1 | ~38 (85 mph) | cladding only; **green** scene with litter; screen cages down | 0.9 |
| 2 | ~55 (123 mph) | the **brown** scene; roofs are the story; structural damage a minority | 2.0 |
| 3 | ~70 (157 mph) | you can see INTO buildings; canopy is bare sticks; poles down in runs | 2.8 |

**Level 3 surge is deliberately 2.8 m, not 3.6.** At 3.6 the whole plate is under
water and the surface is featureless — correct to the reference imagery, and
also **the level where a search benchmark has the least to find.** 2.8 puts ~35%
of the plate above water: the inland edge of a Cat 4 surge rather than its centre.

---

# The machinery that already exists, and what it can be asked for

Verified by survey 2026-08-28. Full detail in `scene_gen/_plans/hurricane_survey.md`.

## The single biggest cost is NOT the wind. It is that there is no ground.

**There is no elevation, no terrain z, no drainage, no slope, no wind field and
no exposure/fetch anywhere in the suburban layout.** Everything is 2-D world
metres on a flat plate; `suburb_scene.apply_ground:1160` writes coplanar sheets
on a z-ladder (`_Z_GRASS 0.02` .. `_Z_DASH 0.24`, `:39-58`).

So a flat water plane at height z over this plate floods either EVERYTHING or
NOTHING, and its shoreline is a perfectly straight line at the plate edge —
which is exactly what a flood must not look like. **An inundation pass has to
impose its own synthetic ground surface**: either a low-relief height field the
water plane is cut against, or a per-house depth derived analytically from a
synthetic slope + distance-to-shore. This is the real engineering cost of the
hurricane, and it is the thing that has no precedent in the repo.

### But the flood SHORELINE has a precedent, and it is the burn scar

`disaster/ground.py` solves the same shape of problem the water edge has: a
field that steps from 0 to something at a level set ends on a conic with a hard
cut against green grass. `feathered_coverage` ramps over `edge_m` metres
**perpendicular to the front** (distance is `(elapsed - t) / |grad t|` — measuring
along the ray from the origin was ~10 m off on a flank because the ray meets the
front obliquely), wobbles the line by +-`finger_m` of band-limited seamless
noise, and removes compact islands. **The noise only ever MOVES the boundary or
REMOVES coverage**, so ground the fire never reached stays clean.

Transposed to a flood: the inundation extent is a coverage field over a
synthetic slope, feathered and fingered the same way, with the water mesh cut
against it. That gives a ragged shoreline on a dead-flat plate without any
terrain, and it is code that already works.

Two traps carried with it:

- **`opacity_constant` is a FRACTIONAL CUTOUT opacity** (`OmniPBRBase.mdl`), and
  RTX Real-Time discards fractional cutout unless
  `/rtx/raytracing/fractionalCutoutOpacity` is on. It **must** be a command-line
  flag (`KIT_ARGS` via `SimulationApp(extra_args=...)`); a
  `carb.settings.set_bool` after startup is never copied onto the USD render
  property the renderer reads, and the overlay silently vanishes. Expect
  translucent water to land in the same trap.
- **A texture tiled at ~8 m reads as a grid of squares from the air.** One tile
  is projected across the whole overlay instead. Same rule applies to a water
  surface: any repeating normal/wave map will read as a checkerboard from 80 m.

## `wind_flow.py` already contains the hurricane statement

`wind_flow.py:305-320` deactivates doors and garage doors **before** anything
structural, with this comment:

> *a pressure differential takes the openings before it takes the structure,
> which is why a house at EF1 has its garage door in the back garden and its
> roof still on.*

That is the hurricane failure hierarchy, already written, in the tornado
module. The hurricane ladder is that sentence extended downward.

## Damage states that are nearly free on the modular house

`detail/modular_house.py` publishes per-house prims under `{parent}/inst/h_{i}`
(the per-house scoping `suburb_tornado_launch_script.py:361,370` already uses):

| want | cost |
|---|---|
| **partial roof loss** (the Cat 1-2 signature) | the roof is **per-BAY meshes** — one `Roof_01` per fully-covered 10x10 block (`:721-723`), `Roof_Half_01` per leftover 5 m column (`:726-727`). Dropping a bay is one `SetActive(False)`. `validate_roofs:1527` proves coverage offline. |
| **windows blown out** | windows are whole WALL pieces, not prims. Swap the placement `usd` from a `_Win_` piece to `WALL_5["plain"]` — `damage._fragment_assets:271-277` already does this. |
| **garage door gone** | `house_door_2_9` = `Garage_Door_02offset`, `SetActive(False)`; `wind_flow` already does it. |
| **porch torn off** | deactivate `*_bay_roof_*` and swap that wall from `Outer_Wall_Quart_Door_03` to `Outer_Wall_Quart_01`. |
| **individual gable plane** | NOT free. `Roof_01` is one mesh, the two slopes are not separate subsets. Use `fracture.fracture_prim(..., mode="plank", aspect=(1.3,2.6))` as `wind_flow.py:275-292` does. |

Absent from the kit house: no chimney, no railing, no gutter, no deck, no
foundation. Chimney and gable-end failure — both textbook hurricane modes —
have **no geometry to fail**. Same finding the earthquake pipeline hit.

Prim counts per house: cottage 13, villa 18, ranch 23, terrace 38, l_family 50.
`_place` sets `raw_pivot: True` (`:417`) — do not let `apply_placements`
recentre a house or it comes apart.

## The house record gives exposure for free

`detail/suburb_parcel.py:2501-2560` publishes per house, in `info_out["parcels"]`:

    c (cx,cy)  w  d  u          n (nx,ny)  <- INWARD NORMAL (:2554-2558)
    yaw_deg  corners[4]  frontage  lot_corners  lot_depth  lot_width
    archetype  size_index  has_garage  has_fence  has_pool
    wedge_lot  kerb_arc  front_gaps  fence_segs  garage{...}  density

`n` is exactly what a **windward-face** pick needs, and `wind_flow.py:216-224`
records that it falls back to a random draw *because it does not have it*. For
a hurricane, whose wind direction is a known field, wiring `n` in is what makes
damage land on the storm-facing elevation instead of a random one.

`disaster/people.py:884-936` `house_table(...)` already joins parcel geometry
to instance identity **on position, not index** (`:894-898`). Do not re-derive.

`detail/suburb_yardplan.py:1030-1041` tags placements with
`prop_kind in {None, "mailbox", "patio", "bin"}` — added so a wind pass can
tell street furniture from shrubbery. Use it.

## Materials are addressable per-surface

`modular_house.apply_palette:1268-1290` identifies subsets **by the texture
filename of the current binding**, because every kit material prim is literally
named `UnrealMaterial`. Wall subsets: `Cladding_01` (wall face), `Stucco_01_Inst`
(reveals), `Win_Roof_Style_01` (frames) (`:48-56`). Roof: `Section0` field /
`Section1` gable brick / `Section2` verge / `Section3` soffit (`:270-278`).

**So a water-stain material can be bound to `Cladding_01` on one chosen wall.**
Trap (`:360-373`): a **baked** archetype has already been palette-applied, so
its surfaces are `Wood_01_White` etc. — the match set must be the union of kit
surfaces and every surface a palette can produce.

Binding form: use **`MaterialBindingAPI.Apply(prim).Bind(...)`**, not
`MaterialBindingAPI(prim).Bind(...)`. Reason written out at
`scour_relief.py:1254-1257` — the bare form leaves core USD warning "Found
material bindings on prim ... but MaterialBindingAPI is not applied".

## The waterline — three routes, and the cheapest is already written

| route | how | verdict |
|---|---|---|
| **1. extend the soot compositor** | `scorch.soot_mask:143-144` already paints a vertical gradient (`y = linspace(1,0,h)[:,None]; wash = y**1.5`). Replace `wash` with a BAND function of `y`, take the ragged tide edge from the existing band-limited `_noise`, deliver through `damage.scorched_material` as a bound OmniPBR composited onto the surface's own texture. `scorched_texture` already has a `salt` param (`:182,196`) for cache-key separation. | **recommended** — least new code |
| 2. real geometry ring | `detail/greeble.py:102-134` `storey_outline(mesh, z, min_edge=1.6)` slices a trimesh at any Z and returns `[(p0,p1,outward,length)]` with `outward` disambiguated against the loop centroid — already handles L-plans, setbacks, multi-lobed towers. **It is a waterline ring generator.** | use where a mud line needs real thickness |
| 3. `materialBind` GeomSubset cut by height | copy `fracture.face_subset:2217-2270` but select by per-face centroid Z. Reading by height exists (`urban_fire.mono_parts:3040-3113`, `mono_wall_at(parts,z):3115-3123`); **binding by height does not.** Nothing joins the two. | most work |

**Pinning the band to a world Z is already solved twice.** `scorched_material
(triplanar=True)` projects from world coords with `texture_scale` in
repeats-per-metre, so a naive band would tile. The fix is "one tile fitted to
the object + a half-tile offset": `ground.py:96-125`
(`k = 1/tile_m`, `texture_translate = (0.5 - center*k, ...)`, mapping documented
at `:97-99`) and `scene_api._burnt_ground_mat:852-866`. Transposed to a facade:
`texture_scale = (s_horiz, 1/H)` with `texture_translate` from the building base
Z, and one non-repeating map spans the whole elevation with the waterline at a
fixed height. `texture_translate` is authored in only two places today
(`damage.py:434-435`, `ground.py:124-125`).

**Naming collision:** `soot_mask`'s `band=(lo,hi)` (`:97,146-149`) is a
FREQUENCY band-limit, not a spatial one. Do not overload it.

**Depth must be geometry, not a height map.** Measured by
`facade_displace_bench_launch_script.py` and recorded at `greeble.py:5-19` and
`tools/texture_depth.py:6-15`: OmniPBR has no displacement input in this build,
and RTX Real-Time does not tessellate OmniSurface displacement.

**UV caveat:** `primvars:st` is authored by this repo only on flat generated
quads. Building assets bring their own — the monoliths use `primvars:uv0` on 85
of 112 and `primvars:st` on 23 (`urban_fire.py:2953`). Route 1's world
projection sidesteps this, which is why every damage material in the repo takes
it.

## Water: the negative result

**Zero hits repo-wide** (outside `simulate-water-in-isaac-sim/SKILL.md` itself)
for `Water_Ocean`, `Perlinwaves`, `waves_strength`, `water_level_m`,
`flood_height`, `surge_m`, `rtx/fog`, `fogHeightDensity`, `omni.warp`,
`WarpSampleOceanDeform`, particle fluid. File census across `assets/materials`:
**760 png, 19 jpg, 16 usda, ZERO `.mdl`**.

The vMaterials water MDL is real and exports seven materials, but it is
vendored **incidentally**, inside the AEC brownstone pack's `vMaterials_2` tree.
Nothing in the repo pulled it in or has ever loaded it.

**The swimming pool is not a precedent.** `modular_house.py:169-173`
`POOL_FLOOR = "Swimming_Pool_Floor_01_0"`, `POOL_WATER_Z_M = -0.35` is a Nucleus
kit prop plane in a hole cut in the ground sheet — no water material, no shader,
no transparency work. In the people bench it is display-colour only, RGB
`(0.36, 0.52, 0.55)` (`people_showcase_launch_script.py:233,474`).

A material must **be** a `UsdShade.Material` to be bindable — the four
top-level `.usda` in `assets/materials` are wrappers around AEC MDLs
(`Grass_Cut.usda:5-13`); generator at `tools/import_megascans.py:62-88`. A
water material will need the same wrapper treatment.

## The intensity-ladder contract, measured

Diffed leaf by leaf across the shipped presets:

- **wildfire size ladder** (`suburb_wildfire` / `_500` / `_1000`): only **3 of
  107** leaves differ — `epicenter`, `region_m`, `overrides.layout.region_m`.
- **tornado intensity ladder** (`_1000` / `_l1` / `_l2` / `_l3`): only **7 of
  78** leaves differ:

| key | l1 | l2 | l3 |
|---|---|---|---|
| `severity` | 0.55 | 0.82 | 0.95 |
| `seed` | 13 | 10 | 19 |
| `epicenter` | -350,380 | -180,-300 | 380,-350 |
| `heading_deg` | 320 | 38 | 130 |
| `disaster.tornado.width_m` | 95.0 | 155.0 | 240.0 |
| `disaster.tornado.core_frac` | 0.18 | 0.22 | 0.32 |
| `people.max_total` | - | - | 70 |

So a level is: **a different seed (a different PLACE) + a different severity +
a moved and re-aimed event + a per-disaster geometry override.** Nothing else.

Asset-set inheritance is `shared -> suburban_nucleus -> suburban`,
`shared -> park`, and `suburban_park: extends [park, suburban]`. **That parent
order is load-bearing and deliberately not alphabetical**
(`suburban_park.yaml:10-18`): parents merge left-to-right, so `suburban` must be
last or `park -> shared` reinstates the 14 rejected trees. Every live suburban
disaster preset uses `suburban_park`.

## Two suburban paths exist; one is dead

| path | layout | detail | driver | status |
|---|---|---|---|---|
| **GRAPH** | `layout/suburb_net.py` | `suburb_parcel`, `suburb_yardplan`, `suburb_park`, `modular_house` | `suburb_scene.generate_suburb_on_stage:5044` | **everything current** |
| RECT (legacy) | `layout/suburb_layout.py` | `suburb_lots`, `suburb_yards` | `scene_generator.build_city` | **no callers** |

Ignore the rect path — axis-aligned AABBs with 4-way yaw. Related:
`config/low_level/compiled/suburb_earthquake.yaml` is the legacy rect path only
and is **not** the earthquake pipeline that shipped.

## Test idiom — and the gate most tests are missing

System `python3` 3.12 + system `pytest` 7.4.4. **`pxr` False, `trimesh` False,
`shapely` False**; `numpy/PIL/scipy/yaml` True. The repo-root `.venv` has
`trimesh` but no pytest — it is not the test interpreter. So a hurricane module
must keep its scatter/field/ladder maths in pure-Python functions that import
`pxr` only inside `build`/`materials`, exactly as `scour_relief` does.

`check()` **records** into `FAILS`; it does not raise. 44 existing tests are
ungated (`test_scour_relief.py` has 62 `check()` calls and 0 asserts). Copy the
decorator from `tests/test_car_toss.py:91-109`, whose own docstring says it is
"wrapped in a decorator so it cannot be left off a test somebody adds later":

```python
def strict(fn):
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__, run.__doc__ = fn.__name__, fn.__doc__
    return run
```

# Bug catalogue

Every bug found while building this goes here, in the item -> cause -> fix form
the earthquake skill uses. Nothing hurricane-specific yet — the pipeline does
not exist. What follows is inherited breakage found during the survey that a
hurricane pass WILL hit.

| item | cause | fix |
|---|---|---|
| yard props land on cul-de-sac turnarounds; `suburb_yardplan` `keepout` stat always reads 0 | `suburb_scene.py:5155` rebinds `pcfg` from the parcel config to the **park** config, so `pcfg.get("keepout_discs")` at `:5207` is always `None` whenever `park_content` is on | not fixed. Rebind to a separate name before the park block. Hits any pass that trusts `keepout_discs` — including debris and standing-water placement |
| `GENERATION.md:57` says `disaster/` is "Empty here; the damage passes live on the `muyang` branch" | stale | it has 24 modules and ~36k lines. Do not trust that line |
| a damage material bound with `MaterialBindingAPI(prim).Bind(...)` warns on every stage open | the API schema was never applied | use `MaterialBindingAPI.Apply(prim).Bind(...)` — `scour_relief.py:1254-1257` |
| a height-map "mud line" renders flat | OmniPBR has no displacement input in this build; RTX Real-Time does not tessellate OmniSurface displacement | depth must be geometry. `greeble.py:5-19`, `tools/texture_depth.py:6-15` |
| a waterline band tiles up the wall instead of sitting at one height | `scorched_material(triplanar=True)` projects from world coords, `texture_scale` is repeats-per-metre | fit one tile to the object and offset by half: `ground.py:96-125`, `scene_api._burnt_ground_mat:852-866` |
| a baked archetype ignores a surface-name match list | it has already been palette-applied, so surfaces are `Wood_01_White`, not `Cladding_01` | match the union of kit surfaces and every palette output — `modular_house.py:360-373` |
| **the flood water is BLUE** | the water skill recommends a vMaterials MDL whose `water_tint` is hue-clamped to 0.47-0.53 and whose scattering colour is a hard-coded blue literal | it cannot be fixed by a parameter. Use `OmniPBR_ClearCoat` — water plan S3.7 |
| **binding `subIdentifier="Water_Ocean_"` resolves to nothing** | that variant is inside a `/* PRESET TEMPLATE */` block at `Water_Blue_Ocean_Perlinwaves.mdl:363-386`; the water skill's table lists it as usable | use one of the five real exports, or (better) do not use this MDL at all |
| **a wet pass authors a NEGATIVE roughness** | `urban_fire.py:2917,2941` clamp `min(1.0, rb + rough_add)` with **no lower bound**, and wetness is a negative `rough_add` | `max(0.0, min(1.0, ...))`. Certain to bite, and only if unfixed |
| **the wet pass changes nothing and the API looks broken** | `reflection_roughness_texture_influence` is 1.0 at `vegetation.py:2509,3204` and every megascans `.usda` sets `enable_ORM_texture=1`; roughness then comes from the map, not the constant | lower influence to 0.2-0.3 wherever roughness is lowered. **[REPORTED, NOT YET CONFIRMED]** — check with `docker exec isaac-sim grep -rn reflection_roughness_texture_influence /isaac-sim/kit/mdl/core/` |
| a shared material is darkened k^3 | these assets bind ONE material to several subsets; a per-call `seen` set darkens it once per subset | thread ONE `seen` set through the whole pass — `urban_fire.py:2861-2866` |
| **water plane z-fights the ground and sidewalks poke through** | the suburb ground z-ladder runs grass 0.02 -> walk 0.17 -> dash 0.24 m; a 5-15 cm level-1 sheet lands INSIDE it | enforce `water_level_m >= 0.30`; assert against `_Z_DASH * ground_z_scale()` in a pure-Python test |
| a translucent water surface prints a grid | coplanar translucent quads sharing an edge composite the opacity TWICE in a ray tracer | greedy-mesh into maximal rectangles; never overlap cells — `ground.py:374-423` |
| **the water occludes ground-truth raycasts** | `_make_physx_ground_snap` raycasts down onto colliders and a water collider becomes "the ground" | author NO `PhysicsCollisionAPI` / `RigidBodyAPI` under `/World/stage/generated/surge`; assert it with bare `pxr` |
| submersion labels are wrong on exactly the objects that matter | the water skill's GT recipe is bbox-min-z, and `BBoxCache` returns the AABB of an AABB — measured 6.8x span error, 3,570 offenders a bbox audit called clean | use `bake.world_point_bounds` — `bake.py:723` |
| **`heading_deg` and `epicenter` in a hurricane preset do NOTHING** | `compile_hurricane(sev, spec, region)` takes `spec` and `region` and **uses neither** — it reads only `sev`. `compile_tornado` unpacks `w, h = region` on its first line | verified 2026-08-29. Rewrite `compile_hurricane` to read `spec`/`region`, using `compile_tornado:318` as the template. **Until then every steering knob in the parameter set above is a silent no-op** |
| **the freeze launcher silently labels a hurricane `"wildfire"`** | `freeze_dataset_launch_script._disaster_kind:157-172` looks for a `hurricane` or `flood` sub-key in the compiled `disaster` block; `compile_hurricane` emits neither, so it falls through to `return "wildfire"` at `:172` | pass `FREEZE_DISASTER=hurricane`, or (better) emit a `disaster.hurricane` sub-block — which the rewrite above does anyway |
| **`presets/hurricane.yaml` builds a v1 city, not the reviewed downtown** | it has **no `overrides:`**, so `compile_downtown` (`compile_locale.py:41`) gives it `max_block_m: 70` and no `layout.anisotropic`, no `districts`, no `city_detail`, no road markings. It compiles and runs — it just does not look like the downtown that was reviewed | copy the `overrides:` corpus from `downtown_earthquake.yaml` |


---

# Known gaps, recorded so they are not rediscovered

1. **There is no ground.** No elevation, no slope, no drainage anywhere in either
   layout. The inundation pass invents its own via a virtual relief field, and
   the consequence is that **apparent depth (what is drawn) and label depth (what
   is emitted) are different quantities** — see the water plan S3.8, which keeps
   the label honest and declares the shading gradient as apparent-only. The real
   fix is a ground height field, which means rewriting `apply_ground`,
   `apply_ground_planes` and every ribbon they lay. **A separate project.**
2. **No chimney, no gable end, no gutter, no deck** on the kit house — two
   textbook hurricane failure modes have no geometry to fail.
3. **No sand pack** in `assets/materials/megascans`. Washover-fan sand is a
   tinted `Dirt_Rough` until one is imported, and the fan is the highest-contrast
   feature in a surge scene — **the single highest-value asset acquisition here.**
4. **Pane-rect measurement exists only for the URBAN kit**
   (`tools/_g_glass_rects.py` -> `quake_flow._G_GLAZING`). Blowing out windows
   individually on a modular house needs it re-pointed at ModularNeighborhood.
5. **No per-house code-era attribute exists.** The variance term this disaster
   depends on has nowhere to live yet.
6. **Oil-sheen iridescence will be static** where the real thing is view-angle
   dependent. Acceptable at 30-120 m nadir; recorded as an approximation.
