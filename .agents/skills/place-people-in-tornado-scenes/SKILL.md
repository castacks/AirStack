---
name: place-people-in-tornado-scenes
description: Where survivors, diggers, trapped and thrown people go in a post-tornado scene, and why — the epidemiology behind every share, the `disaster/tornado_people.py` planner API, the T+30-60 min epoch, and the bench that is the only way to verify a partially-buried figure. Read before touching tornado_people.py, the people pass in suburb_tornado_launch_script, or tornado_people_preview_launch_script. The wildfire people model does NOT transfer and this file says exactly which parts and why.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Place People in a Post-Tornado Scene

## Read this first: the visual verification is OUTSTANDING

As of this writing the planner is **host-tested but never rendered**. What has
been checked:

- head counts, share normalisation and absolute caps
- the no-interpenetration invariant across a full 90-figure plan
- every bench unit produces figures, with the expected poses
- both launchers compile on 3.10 and 3.11; an AST pass finds no undefined
  names in the wired `main()`

What has **not** been checked, because it cannot be checked any other way:

- whether a figure stands ON debris rather than through or above it
  (`DEBRIS_Z_M` is an ESTIMATE — see below)
- whether a `trapped_partial` figure reads as partially buried from capture
  altitude, which is the entire premise of that scenario
- whether a toppled car is seated
- whether a prone figure is legible against mud

**Run `tornado_people_preview_launch_script.py` and look at it before trusting
any of this.** The bench exists precisely so those four questions get answered
in one launch instead of inside a twenty-minute full-scene build.

---

## Why the wildfire model does not transfer

`disaster/people.py` is a good model of the wrong thing. The difference is not
cosmetic and it is not about assets:

| | wildfire | tornado |
|---|---|---|
| warning | **hours** | **10-15 minutes** |
| what people do | **MOVE** — evacuate by car | **STAY** — shelter where they are |
| what the model is | a geography of EGRESS: refuge car parks, roadways, gridlocked queues, the cul-de-sac trap | a geography of WHERE THEY LIVED and what the wind did to it |
| location is a function of | the road network and refuge features | the housing distribution x the intensity field x did they emerge |

Four of the six wildfire scenarios have **no tornado counterpart at all** (see
*What is deliberately absent*). Do not port them. `tornado_people.py` shares
exactly one thing with `people.py` — `_human_placement`, the measured pose
geometry — and that is reused rather than reimplemented on purpose.

---

# The research

Every share in the module traces to something here. If you change a share,
change this section too, or the next reader will trust a number that no longer
has anything behind it.

## The fact the whole module turns on: almost everybody survives, on their feet

Joplin is the only event with a real denominator — NIST geolocated all 161
fatalities (NCSTAR 3) and Paul & Stimers 2014 mapped population by damage zone.

| zone | population | deaths | survived |
|---|---|---|---|
| **catastrophic** (total structural destruction) | 4,716 | 122 | **97.4%** |
| whole path | 13,547 | 161 (1.2%) | **~89% alive and mobile** |

And of the injured, **89% are minor** (ISS < 10), 6% moderate, 5% severe;
**86% are discharged home** (Niederkrotenthaler et al. 2013, chart abstraction
of 1,398 patients across 39 Alabama hospitals). Deaths are near-instantaneous:
**86.6% died on scene** (Chiu et al. 2013), 94% in May et al. 2000. There is no
large population of "dying but savable" victims the way an earthquake collapse
produces.

**Consequence, and it is counterintuitive enough to state twice: a levelled
block is not a morgue.** A scene built on that intuition would be EMPTY of the
targets the benchmark exists to find. The corridor is full of standing, walking,
digging people — and the visible population does **not** thin toward the
centreline, because with the houses gone there is nothing left to occlude
anyone. (An earlier draft of this design had it backwards and the Joplin
denominator is what corrected it.)

## People cluster — over 90% of them

Chiu et al. 2013 recorded who each victim was found with:

| | share |
|---|---|
| with other deceased **and** survivors | 26.3% |
| with other deceased | 24.7% |
| with other survivors | 21.1% |
| **ALONE** | **7.7%** |

Group placement is the empirical default, not a rendering convenience. And a
CLUSTER is itself the strongest aerial indicator that somebody is trapped
underneath it — which is why `neighbour_dig` is the anchor scenario and why its
figures all face the dig point.

## The epoch is T+30-60 min, and it is not a lighting choice

| window | what is in frame |
|---|---|
| T+0-15 min | self-extrication; roads impassable; **nobody in uniform** |
| **T+15-60 min** | **PEAK visible density.** Neighbours digging, injured moved in private pickups. Joplin: 100 patients arrived at Freeman in 16 minutes, starting T+39 |
| T+1-6 h | mutual-aid apparatus, field triage on hard standing |
| T+6-24 h | organised grid search, task forces, heavy equipment, orange search markings — **and nobody left alive to find** |

Joplin's professional rubble-rescue count was **seventeen**, against 1,371
injured. The earthquake literature puts civilian extrication at **60-100%**
(Bartolucci et al. 2020: Armenia 1988 95% by local inhabitants, 0.9% by
international teams; Turkey 2002 48% by neighbours; Philippines 1990 84% of
survivors rescued < 1 hour). Greensburg: **68% of households did SAR**, 28%
starting immediately, 28% within the hour.

**So: no responders, no heavy equipment, no search markings, no triage tents.**
A scene with those is depicting T+24 h and a different problem. `epoch_min`
records the choice.

## `thrown` is capped at one or two, and here is the arithmetic

This is the number most likely to be "corrected" upward by someone who has seen
a documentary, so the derivation is written out.

CDC ran mortality surveillance on all 338 deaths in the April 2011 outbreak,
recording location of **injury** and location of **recovery** separately
(MMWR 61(28)):

| | injured there | recovered there |
|---|---|---|
| indoors | **90.5%** | — |
| outdoors | **3.3%** | **37.0%** |

An ~11x displacement ratio — about a third of the **dead** end up off their own
footprint. But the MMWR records **no distance**, so "outdoors" does not
distinguish the front yard from two blocks away, and most of that 37% is
"went out with the wall that failed and landed in the yard".

Brown et al. 2002 (OKC 1999) gives the severity link: "picked up / blown by
tornado" was the mechanism for **43% of hospitalised** injuries and only **6% of
treated-and-released**. Being thrown is a severity marker, not a common
experience.

Applied to ~39 damaged houses (~100 residents present, Joplin path rates of
0.77% dead and 4.8% injured):

    0.8 dead      x 37% recovered outdoors  = 0.3 people
    1.25 hospitalised x 43% thrown          = 0.5 people
    ------------------------------------------------------
    UNDER ONE PERSON in the whole 500 x 500 m scene

Long-range lofting is record-book territory, not a category: the documented
survivals are **398 m** (Matt Suter, an **F2**, GPS-measured by an NWS official,
Guinness record) and **76 m** (a mother and two children on a mattress, Dawson
Springs 2021). There is **no published distribution of human throw distances**.
`range_m` is therefore deliberately short.

And a thrown figure is **PRONE**. Anyone walking in the open emerged and walked
there, which `street` and `on_the_rubble` already cover.

### Do NOT drive deposition with the `throw` field

The tempting move is to reuse `tornado.throw_field` — the same one that fans the
plank debris — for bodies. It would be wrong:

- The **78%-left** deposition statistic everyone quotes (Snow et al. 1995, 163
  debris reports) is for **lightweight** debris lofted into the parent storm and
  deposited tens of kilometres downstream. Cheques and photographs, not people.
- Bodies are **heavy** debris and stay in the swath.
- Near-surface flow is **CONVERGENT toward the centreline** — Karstens et al.
  2013 digitised **10,300 tree falls at Joplin and 94,500 at
  Tuscaloosa-Birmingham** and found inward-pointing falls across most of the
  path.
- There is **no published azimuthal distribution for victim deposition**.
  Nobody has done for bodies what Snow did for cancelled cheques.

A modest downtrack displacement with a wide spread (`gauss(0, 55 deg)`) is
defensible. A tight fan is an invented claim.

## Where people shelter, and why almost none of it is placeable

Useful background, and the reason so much of the real population is out of
scope:

- **Basements are rare.** Joplin: **17%** of path homes (1,237 of 7,411); 80%
  had crawl spaces, which NIST notes are typically accessed from OUTSIDE and as
  little as a foot high. Moore OK: **"10% or less"** have below-ground shelters.
  West South Central new construction is **96-98% slab**. Note OK is *not* in
  the basement-rich Plains group — that is KS/NE/IA/MO.
- **In-house sheltering is ground-floor interior**: bathroom 39%, closet 37%,
  hallway 10%, other 14% (Hammer & Schmidlin 2002, 190 occupants of 65 homes
  with F4/F5 damage, none with a basement).
- **Correct shelter does not guarantee survival.** In Joplin, **20 of the 62**
  single-family-home decedents had taken correct interior refuge. NIST NCSTAR 3
  p.203: best-available refuge areas *"are not expected to offer life-safety
  protection against tornado hazards."*
- **Basements did work where they existed**: NIST found **no evidence any
  fatality occurred below grade** in Joplin.

None of this is placeable in a drone benchmark. See *What is deliberately
absent*.

## Vehicles: use the measured displacement rates

Paulikas, Schmidlin & Marshall 2016 — **959 vehicles across 12 tornadoes**:

| | displaced | rolled / tipped |
|---|---|---|
| EF0 | 10% | — |
| EF1-EF2 | 36% | 5% |
| EF3-EF4 | 63% | 15% |
| EF5 | 69% | **31%** |

Even on the centreline only about a third go over; two thirds are merely
**shoved**. A corridor where every car is upside down is as wrong as one where
none is.

Two more findings that shape the scenario:

- **Fleeing by car was protective in every study that measured it.** Daley et
  al. 2005: severe-injury **OR 0.2 (0.1-0.6)** for those who fled vs stayed.
  Hammer & Schmidlin: **0 of 90 who fled were injured**, against 30% injured
  among those who stayed. The "cars are deathtraps" guidance traces to a single
  1979 event (Wichita Falls, 25 of 44 deaths in cars). **Moore 2013 had ZERO
  vehicle deaths.**
- So `in_vehicle` places people **beside** displaced cars, not as casualties
  inside them — and **upright cars only** (see the traps).

---

# The scenarios

`scene_gen/disaster/tornado_people.py`, `DEFAULTS["scenarios"]`. Shares are of
`total` and are normalised, so editing one does not silently change the head
count.

| scenario | share | what it is | grounding |
|---|---|---|---|
| `on_the_rubble` | 0.28 | survivors standing / walking on the wreckage of their own house (nobody waves — the pose was cut, see below) | 97.4% survive even in the catastrophic zone; nothing left to occlude them |
| `neighbour_dig` | 0.24 | **the anchor.** 3-6 at ONE collapsed house, all facing the dig point | 92% of victims found with others; civilians did 60-100% of extrications |
| `trapped_partial` | 0.12 | prone, sunk into the pile, boards across the lower body | over-represented on purpose — see below |
| `street` | 0.16 | walkers on the carriageway | the only navigable ground; Joplin searched "house by house, car by car, block by block" |
| `assisted` | 0.08 | trios, two supporting one | 89% of injuries are minor and walking; private vehicles were 38% of transport vs 31% by ambulance |
| `in_vehicle` | 0.08 | standing beside **upright** displaced cars | Paulikas rates; fleeing was protective |
| `thrown` | 0.04, **capped at 2** | prone, downtrack, wide spread | the arithmetic above |

`total` defaults to **90** over a 500 m corridor — an order of magnitude denser
per hectare than the wildfire plat's 60 over 1600 x 1200 m, because an evacuated
suburb is genuinely empty and a struck one is not.

## `trapped_partial` is the hard one, and it is a dataset decision

Fully buried is an **unlabellable target**; fully exposed is **not a trapped
person**. The whole scenario lives or dies on a fraction.

Mechanically: the figure goes at the **pile EDGE** (the middle of a collapsed
house is where the material is deepest and a body there is invisible from any
angle), is sunk by `sink_frac` of the local debris depth, and the module emits
its **own** boards laid **across** the body axis — a plank parallel to a prone
figure hides nothing and reads as coincidence. It emits its own rather than
trusting whatever the wreck archetype happens to have left at that spot.

**It is deliberately over-represented.** Chiu records "trapped in rubble" as the
mechanism for **1.7%** of fatalities and "crushed" for 18.7%; Brown puts
structural collapse behind 11-13% of injuries. A true rate would put well under
one of these in the scene. A dozen is a **benchmark decision**, recorded as such
in the module so nobody later reads the share as an estimate of the world.

---

# The API

## The planner — `scene_gen/disaster/tornado_people.py`

A **pure planner**: no stage, no Isaac imports, no USD. The whole plan runs and
asserts on the host, which is how the spacing and count invariants are actually
tested.

    cfg = tornado_people.resolve_cfg(scene_config)   # merges a `people:` block
    humans, debris, records = tornado_people.plan_people(cfg, ctx, rng)

`ctx` keys, and who owns each:

| key | shape | owner |
|---|---|---|
| `wrecks` | `[{x, y, fp, intensity, level}]` | the assembly launcher |
| `intact` | `[(x, y)]` standing houses | the assembly launcher |
| `road_pts` | `[(x, y, tangent_deg)]` | the assembly launcher, off `binfo["net"]` |
| `cars` | `[{x, y, toppled}]` | the assembly launcher |
| `throw_deg` | float | `disaster.tornado` |
| `humans` | `[usd]` RIGGED RenderPeople | `suburb_scene.AssetPools` |
| `resolver`, `asset_pools` | | `scene_generator`, `suburb_scene` |

Returns:

- **`humans`** — `category: "human"` placements carrying a `pose`, ready for
  `sg.apply_placements(..., instance_categories=set())`
- **`debris`** — plank specs for `trapped_partial`, to be authored with
  `disaster.planks`. Returned rather than authored because this module never
  touches a stage.
- **`records`** — ground truth, one dict per person: scenario, pose, alive,
  **`visibility`** (`full` / `partial`), note. `write_records()` dumps it.

`summarise(records)` gives counts by scenario and by visibility.

## Wired into the scene

`suburb_tornado_launch_script.py`, step **7b**, **after** the scour — a survivor
is not debris, and every pass before it moves, deletes or re-materialises
something. Env knobs:

| var | default | what |
|---|---|---|
| `TOR_PEOPLE` | 1 | 0 disables the whole pass |
| `PEOPLE_JSON` | `$ARCH_DIR/humans_<seed>.json` | ground truth |

## The bench — `tornado_people_preview_launch_script.py`

Six set-pieces on a 60 m grid over mud, each with its own top-down and oblique
camera at **26 m / 20 m** (not `views_around`'s 60 m default — a 1.8 m figure
at 60 m is a handful of pixels, which is exactly the condition this bench
exists to avoid judging from).

    A  on_the_rubble     B  trapped_partial   C  neighbour_dig
    D  vehicles          E  thrown+assisted   F  street

`UNITS=B,D` narrows it. **It calls the real `plan_people`** with all shares
zeroed but one (`_only()`), so what is photographed is the code the assembly
runs — a bench that reimplements the thing it is checking proves nothing.

Unit B deliberately places **three** trapped figures across the full
`sink_frac` range so one frame shows the shallow, middle and deep cases
together.

---

# Traps

**`DEBRIS_Z_M` IS AN ESTIMATE, and it is the one number here with no literature
behind it.** How deep the pile is by damage level, used to stand a figure ON the
wreckage rather than through it. A levelled house leaves the deepest pile
(everything is still on the lot); a swept slab the shallowest (the material is
downwind). If figures float or sink in the bench, **this table is the thing to
correct** — not the pose code.

**Only RIGGED humans can take a pose.** `posed_standing` assets are static
meshes frozen in one attitude; binding `walk` or `crouch` to one does nothing at
all and the figure ships in whatever attitude it was authored in — silently.
`_rigged_humans()` makes the same selection `disaster.people.build_ctx` does,
with the same fallback.

**`in_vehicle` uses UPRIGHT CARS ONLY.** The pose rig seats a figure against a
horizontal seat plane, so a `seated_car` pose in a rolled car comes out sideways
in mid-air. Toppled cars in this scene are empty — which is also the likelier
truth, since being in a vehicle that rolls is how you stop being seated in it.

**`toss_prim` MEASURES the prop to seat it.** It lives in `disaster.tornado` so
the bench and the scene share one copy. A car pivots about its own origin on the
ground, so rolling it 90 degrees swings half the BODY below grade by half the
car's **width** — not the 0.7 m an earlier version guessed for everything, and
not the same for a hatchback and a pickup. Half of them sank into the road and
the rest hovered.

**Vehicle displacement probabilities are UNCONDITIONAL.** Paulikas' "EF5 69%
moved, 31% tipped" means 31% of ALL cars, not 31% of the 69%. Testing a tip
probability only on cars that had already passed a move probability multiplies
the two: measured, that produced an effective 6% where the data says 15% —
**5 of 25 cars in the path moved and NONE tipped.** One draw against nested
thresholds.

**`assisted` trios are tighter than `min_separation_m` on purpose** — they are
touching. They bypass the spacing check and register afterwards.

**Scenario order is load-bearing.** `plan_people` runs the scenarios that need a
SPECIFIC spot before the ones that can go anywhere, so a trapped figure is never
crowded out of the pile edge by a bystander who could have stood two metres
away.

---

# What is deliberately absent

**`parking_refuge`** — the LARGEST wildfire scenario at 0.30, and it has no
tornado counterpart. Moore, Joplin and Mayfield all had **no public tornado
shelters**. Moore's emergency management says so explicitly and explains why:
10-15 minutes of warning is not enough to receive it, decide, lock up, drive,
park and get inside. Madison County AL — Huntsville, among the most
tornado-conscious counties in the US — lists 14 safe rooms totalling ~2,700
capacity, all in the rural fringe, and states the cities of Huntsville and
Madison have none. Observed public-shelter use is **~4% where one exists and 0%
where none does**.

**`gridlock`** — a neighbourhood-scale traffic jam is not supported. Across six
metros over 2011-2018 only Oklahoma City ever produced a mass traffic reversal
(Hatzis & Klockow-McClain 2022), it was **broadcast-directed**, and it is a
metro-arterial phenomenon. Note also that the authors state plainly they *"have
no way to quantify the number of people who actually evacuated"* — the
"hundreds of thousands fled" figure that circulates is not a measured finding.

**`pools` / `cul_de_sac`** — wildfire refuge geography. A pool is shelter from
radiant heat and nothing at all in a wind event.

**Cellars, safe rooms and house interiors** — real, and where people actually
shelter, but a drone benchmark cannot score a target it cannot see. This is
`disaster.people`'s `exposed_interior` lesson: *a target that cannot be seen
cannot be labelled, and an annotation for an invisible one is worse than no
annotation at all.* Not relitigated here.

**Bystanders and onlookers at the track edge** — real (40% of people go outside
to look during a warning, Sherman-Morris 2010, n=2,921) but out of scope by
request: this scene is about the people the tornado HIT.

**Responders, heavy equipment, search markings, triage tents** — T+12-24 h
artefacts. See the epoch.

---

# Known gaps and open decisions

- **The bench has never been run.** See the top of this file.
- **Mobile homes are structurally absent.** 38-47% of US tornado deaths against
  ~5.4% of housing stock, and a **~10x exposure-adjusted** per-capita risk
  (Fricker & Friesenhahn 2022; the ~20x figure elsewhere is unconditional and
  also picks up where mobile homes are sited). Our suburb is entirely
  site-built modular kit, so the scene cannot represent the highest-risk
  housing type at all. **A small manufactured-home park is the single
  highest-value asset addition available.**
- **Casualties are on for `thrown` only.** Those figures are recorded
  `alive: false`; everyone else is alive. There is no global `casualty_share`
  knob yet — `disaster.people` has one and this module does not.
- **Demographics are not modelled.** 60+ are **39.7%** of tornado fatalities and
  65+ are 28% against 12% of exposed — a 5x per-capita rate. If the
  RenderPeople pool has older figures, casualty and assisted placements should
  prefer them. Nothing does this today.
- **Time of day is not a parameter.** Peak tornado hour is 16:00-17:00 CST and
  the scene is bright daylight, which is self-consistent — but a nocturnal
  scene would invert the mix entirely (nearly everyone in a bedroom, essentially
  nobody outdoors or in vehicles, mobile-home share of deaths roughly doubling).
- **"En route" is real and unmodelled** — people caught in hallways moving
  toward a refuge when it hit (NIST Joplin Findings 43/44). Not placeable
  anyway, being indoors.
- **No drone has ever been documented finding a live tornado victim.** Lee
  County AL 2019 flew thermal drones; the sheriff's stated value was *negative
  assurance* — confidence nothing was missed — not detection. CRASAR flew
  Mayfield for damage assessment, not search. There is no operational ground
  truth to calibrate against, which is arguably the case for building this
  dataset.

---

# Sources

The load-bearing ones, so a future reader can check rather than trust.

- **NIST NCSTAR 3** — Technical Investigation of the May 22, 2011 Joplin
  Tornado. 494 pp. All 161 fatalities geolocated; interviews; basement and
  refuge findings. https://nvlpubs.nist.gov/nistpubs/NCSTAR/NIST.NCSTAR.3.pdf
- **CDC MMWR 61(28):529-533** — Tornado-Related Fatalities, Five States,
  Southeastern US, April 25-28 2011. The injury-location vs recovery-location
  table (90.5% / 3.3% / 37.0%).
- **Chiu et al. 2013**, *Am J Public Health* 103(8) — Alabama 27 Apr 2011, 247
  decedents. Room-level locations; who victims were found with; 86.6% on-scene.
- **Niederkrotenthaler et al. 2013**, *PLoS ONE* 8:e83038 — 1,398 patients, 39
  hospitals. Injury severity distribution; within-house odds ratios.
- **Hammer & Schmidlin 2002**, *Wea. Forecasting* 17:577 — 190 occupants of 65
  F4/F5-damaged homes, OKC 1999. Shelter-room shares; 47% fled; 0 of 90 injured.
- **Brown et al. 2002**, *Wea. Forecasting* 17:343 — OKC 1999 deaths and
  injuries; "picked up / blown" mechanism shares.
- **Daley et al. 2005**, *Am J Epidemiol* 161:1144 — odds ratios including
  fleeing by vehicle.
- **Paul & Stimers 2014**, *Wea. Climate Soc.* 6(2) — Joplin mortality by
  damage zone, with population denominators.
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
- **Sherman-Morris 2010**, *Natural Hazards* 52:623 — n=2,921; 40% went outside
  to look.
- **Hatzis & Klockow-McClain 2022**, *Wea. Climate Soc.* 14 — the 31 May 2013
  OKC traffic reversal, and its limits.
- **FEMA/DHS 2011** — Joplin Lessons Learned Study; response timeline.
- **Paul, Che, Stimers & Dutt** (Natural Hazards Center QR) — Greensburg;
  68% of households participated in SAR, with timing.
