# Where hurricane casualties actually are, and why the tornado's throw does not transfer

Research note, 2026-09-01. Written because the hurricane people pass inherits
`tornado_people` verbatim for the dry-land population, and the reviewer flagged
one inherited behaviour as wrong for this hazard:

> "so casualties thrown 10–40 m out — This has to be a tornado only thing. We
> don't want that in hurricane, that doesn't really happen in hurricane.
> Validate that with actual reports like we did for tornado"

**Finding: the reviewer is right, and the literature is one-sided about it.**
There is no documented case of a person being thrown any meaningful distance by
hurricane *wind* on land. Every hurricane wind fatality with a recorded location
was found in, under, or immediately beside the structure that failed on them.
The tornado's `_trail` pass is therefore switched OFF for the hurricane, and
`where` is re-weighted onto the structure. Both changes are cited below.

---

## 1. Wind is a minority killer in a hurricane to begin with

Rappaport (2014), *Fatalities in the United States from Atlantic Tropical
Cyclones: New Data and Interpretation*, BAMS 95(3) — ~2,500 direct deaths in
the continental US, 1963–2012. The cause split (taken from Rappaport's own WMO
RA-IV slide deck, which reproduces the paper's pie chart):

| Cause | Share of direct deaths |
|---|---|
| Storm surge | 49% |
| Rain / freshwater flood | 27% |
| Surf | 6% |
| Offshore (marine) | 6% |
| **Wind (non-tornadic)** | **8%** |
| Tornado (embedded) | 3% |
| Other | 1% |

> "Around 90% due to water, most by drowning."

Rappaport's own definition of the wind category is the mechanism, and it is
already the answer to this question:

> "incurred from wind-borne debris or structural failure induced by wind"

Structural failure and debris impact. Not transport of the body.

*(A more recent reanalysis shifts surge down to ~33% and rain up to ~36%; the
wind share is not what moves between the two studies.)*

---

## 2. The Hurricane Andrew case list — every direct death, with its location

The strongest single source, because it is a **medical examiner's case-by-case
table** rather than an aggregate. Dade County ME study of the 44 Florida deaths
attributed to Andrew (Cat 4, 145+ mph sustained, 24 Aug 1992). The direct
traumatic deaths, verbatim from the tables:

**Blunt trauma (8)**

| Decedent | Circumstance |
|---|---|
| 25 y/o | Collapsing roof |
| 74 y/o | Ejected from turning trailer |
| 49 y/o | Ejected from turning trailer |
| 46 y/o | Collapsing home |
| 49 y/o | Collapsing barn (debris) |
| 68 y/o | Collapsing townhouse |
| 54 y/o | Collapsing roof |

**Mechanical asphyxia (4)**

| Decedent | Circumstance |
|---|---|
| 62 y/o | **Found in destroyed mobile home** |
| 67 y/o | Collapsed ceiling of retirement home |
| 80 y/o | **Destroyed trailer** |
| 37 y/o | Struck by flying debris |

And from the running text:

> "eight died from injuries incurred as homes collapsed, and roofs and walls
> caved in, and four died of mechanical asphyxia from being crushed by falling
> debris."

> "One of the asphyxiated victims was in the bed of a truck covered by a
> camper-top when a tree fell and pinned the man inside."

> "Mobile homes and trailers were particularly susceptible to wind damage. Two
> of the four asphyxial deaths occurred in mobile homes."

One further case, a sheltering death:

> "Her decomposing body was found in her dining room four days after the storm.
> Scene findings indicated that she was hiding in the bathtub until the window
> was blown in."

She moved one room. That is the largest wind-driven displacement in the series.

**Tally of the 12 direct traumatic deaths by where the body was:**

| Where | n | Cases |
|---|---|---|
| In / under the structure | 8 | 5 collapses, retirement-home ceiling, destroyed mobile home, destroyed trailer |
| Immediately beside it | 2 | both "ejected from turning trailer" — the unit rolls, the occupant comes out with it |
| Outdoors / vehicle | 2 | flying-debris strike; tree onto a camper-top |

**Zero thrown.** The only body in the whole Andrew series recovered far from
where it was injured is a *marine* death — a crewman "virtually decapitated by
flying debris and thrown overboard; his body was found in the water six days
later." Water moved him, not wind, and he was never on land.

---

## 3. Corroboration from later storms

**Hurricane Ian (2022, Cat 4/5, 109 Florida deaths).** Florida Medical
Examiners Commission case reports, as summarised in press coverage of the
released records:

* A Manatee County woman "went outside to smoke when a gust of wind blew her
  off the porch" and "struck her head on an adjacent concrete step" — displaced
  by the length of a fall, onto an adjacent step.
* A 62-year-old woman died when "a tree fell on her manufactured home" — in the
  home.
* An 87-year-old man "found submerged in a car".

**Hurricane Irma (2017).** CDC MMWR 67(30): of 129 deaths, only 11 were direct
accidents — 7 drownings and 4 tree-related. 115 of 129 (89.1%) were *indirect*
(existing medical conditions, CO poisoning from generators). Nothing resembling
wind transport.

**Wind-related tree failures, all US causes** (Schmidlin 2009, *Nat Hazards*
50(1), 1995–2007): tropical cyclones are 14% of wind-related tree deaths, and
the fatality locations are vehicles 44%, outdoors 38%, mobile homes 9%, frame
houses 9%. Again: the person is where the tree found them.

---

## 4. Why the physics differs, in one paragraph

A tornado lofts because it has a concentrated, near-surface **vertical**
velocity component — the corner-flow updraft — coincident with an extreme
horizontal gradient over a few tens of metres. A landfalling hurricane's
boundary layer is quasi-steady and essentially horizontal at human scale;
its rotation is organised over hundreds of kilometres and produces no local
updraft a body can ride. The documented lofting events *inside* hurricanes
(e.g. a truck lofted after Harvey's landfall) are attributed to embedded
**tornado-scale vortices**, not to the hurricane wind field — which is exactly
why Rappaport counts embedded tornadoes as their own 3% category, separate from
the 8% wind. A hurricane scene that throws bodies is depicting the tornado
category, mislabelled.

---

## 5. The contrast with the tornado, stated precisely

The tornado module's throw is not arbitrary; it rests on a real signal that the
hurricane literature simply does not produce:

| | Tornado | Hurricane |
|---|---|---|
| Injured indoors | 90.5% | overwhelmingly indoors |
| Injured outdoors | 3.3% | — |
| **Recovered outdoors** | **37.0%** (≈40% "recovered outdoors near the impact area") | no such displacement reported |
| Source | CDC MMWR 61(28), all 338 April-2011 deaths, injury and recovery location coded separately | Andrew ME case list; Ian ME reports; Irma MMWR |

That ~34-point gap between *injured* indoors and *recovered* outdoors is the
whole empirical basis for the tornado's `yard` and `trail` shares. **No
hurricane study codes recovery location separately from injury location,
because in a hurricane they are the same place** — which is itself the finding.

Note also what the tornado module already concedes about its own throw: there
is *no* published distribution of human throw distances even for tornadoes, the
survivals quoted (398 m Matt Suter, 76 m Dawson Springs) are record-book cases,
and `_trail` was deliberately sized to put "under one person in a 500 m
corridor" in the air. Carrying that already-marginal pass into a hazard with
*zero* supporting cases is not a small extrapolation, it is an unsupported one.

---

## 6. What changes in the code

In `disaster/hurricane_people.resolve_cfg`, on the `dry` sub-config only
(`tornado_people` itself is untouched — the tornado keeps its own model):

1. **`trail` → off.** `count: [0, 0]`. `tornado_people._trail` returns 0 and no
   body is placed away from a wreck. Nothing else in that function is reached.

2. **`where` re-weighted onto the structure**, anchored on the Andrew tally
   (8 in / 2 beside / 2 out of 12):

   | | tornado | hurricane | why |
   |---|---|---|---|
   | `pile` | 0.36 | **0.55** | "collapsing roof / collapsing home / found in destroyed mobile home" — the modal hurricane death is under the structure |
   | `skirt` | 0.28 | **0.30** | "ejected from turning trailer", "blown off the porch onto an adjacent step" — displaced by metres, still on the lot |
   | `yard` | 0.26 | **0.15** | the flying-debris strike and the tree-onto-vehicle case; kept non-zero, kept small |
   | `street` | 0.10 | **0.00** | the tornado's share rests on Joplin's "house by house, car by car, block by block" search doctrine, which has no hurricane analogue. `_plan_dry` passes no `road_pts` anyway, so this share was already silently folding into `skirt` |

**Honesty about n.** The Andrew direct-death series is 12 cases. The split above
is *anchored* on it, not derived from it — 8/2/2 would be 0.67/0.17/0.17, and
the shipped numbers hedge toward the middle because a 12-case series cannot
support three significant figures. What the evidence *does* support without
hedging is the qualitative claim, and that is the one being encoded: casualties
belong on and immediately around the structure, and none of them are thrown.

## 7. What this does NOT change

* The **water** pass. Surge is 49% of hurricane deaths and a body in moving
  water genuinely does travel — that is flotation and washout, a different
  mechanism with its own literature, and it is already modelled separately.
* The **roof** pass. Refuge on a roof is a surge behaviour, unaffected.
* `tornado_people` itself, and therefore every tornado scene.

---

## Sources

* [Rappaport, E. (2014). *Fatalities in the United States from Atlantic Tropical Cyclones: New Data and Interpretation*. BAMS 95(3)](https://journals.ametsoc.org/view/journals/bams/95/3/bams-d-12-00074.1.xml)
* [Rappaport, E. *The Causes, Numbers and Locations of Atlantic Tropical Cyclone Deaths* (WMO RA-IV workshop slides, with the cause-split chart)](https://severeweather.wmo.int/TCFW/RAIV_Workshop2019/31_Loss-of-life_EdRappaport.pdf)
* [*Mortality from Hurricane Andrew* — Dade County Medical Examiner case series (blunt trauma and asphyxia tables)](https://www.mjusticia.gob.es/eu/Ciudadano/Victimas/Documents/1292428314772-Mortality_from_Hurricane_Andrew.PDF)
* [CDC. *Deaths Related to Hurricane Irma — Florida, Georgia, and North Carolina, 2017*. MMWR 67(30)](https://www.cdc.gov/mmwr/volumes/67/wr/mm6730a5.htm)
* [CDC. *Tornado-Related Fatalities — Five States, Southeastern United States, April 25–28, 2011*. MMWR 61(28)](https://www.cdc.gov/mmwr/preview/mmwrhtml/mm6128a3.htm)
* [Chiu et al. (2013). *Mortality From a Tornado Outbreak, Alabama, April 27, 2011*. AJPH](https://ajph.aphapublications.org/doi/full/10.2105/AJPH.2013.301291)
* [Schmidlin, T. (2009). *Human fatalities from wind-related tree failures in the United States, 1995–2007*. Natural Hazards 50(1)](https://link.springer.com/article/10.1007/s11069-008-9314-7)
* [Florida Medical Examiners Commission — Hurricane Ian cause-of-death reports (press summaries)](https://www.fox13news.com/news/florida-medical-examiners-commission-releases-causes-of-reported-hurricane-ian-deaths)
* [Tornado-scale vortices in the tropical cyclone boundary layer (ACP 19, 2477–2493, 2019) — the embedded-vortex lofting mechanism](https://acp.copernicus.org/articles/19/2477/2019/)
