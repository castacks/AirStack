# OSM measurements behind the generator's constants

Produced by `osm_street_stats.py`. **Aggregates only** — no per-object data is
recorded here, which is what keeps this clear of ODbL's share-alike (that
attaches to redistributing a Derivative Database; summary statistics are facts).

© OpenStreetMap contributors — ODbL. Measured with OSMnx; cite Boeing, G.
(2025), *Geographical Analysis* 57(4), 567–577.

Reproduce with:

```bash
uv run --no-project --with osmnx --with shapely --with scipy \
    scene_gen/tools/osm_street_stats.py --place manhattan --dist 450 --no-furniture
```

## Midtown Manhattan — 450 m radius, 2026-08-10

| Measure | Value | Sets |
|---|---|---|
| block short side | **79.7 m** (IQR 79.1–80.2) | `layout.anisotropic.block_short_m` |
| block long side | **273.9 m** (IQR 154–310) | `layout.anisotropic.block_long_m` |
| block long:short | **2.3×** (IQR 1.7–3.5) | validates the anisotropic split |
| block area | 21,684 m² (IQR 11.7k–24.6k) | — |
| blocks / km² | 35 | sanity check on subdivision depth |
| intersections / km² | 53 | — |
| street segment length | 85.5 m (IQR 79.6–178.8) | cross-check on block short side |
| landuse area share | commercial 96%, construction 2%, recreation 1% | `districts` mix |

Road classes, by share of street segments:

| Class | Share | Median lanes |
|---|---|---|
| secondary | 53.9% | 4 |
| residential | 34.2% | 1 |
| primary | 10.5% | 5 |
| living_street | 1.3% | 1 |

→ **64% of segments carry 4+ lanes.** This is what motivates the wide/narrow
split in `city_layout.py`: the built-in subdivision gives every road the same
width, whereas a real grid is mostly wide arterials with narrow residential
streets between them.

### Caveats, so these aren't over-trusted

- **The signalised-intersection figure is unusable as measured** — it came out
  at 132% (45 signals against 34 nodes of degree > 2). Signal nodes in OSM sit
  on approach ways rather than only at junction centres, so the denominator is
  wrong. `traffic_lights.intersection_chance` is therefore *not* OSM-derived;
  it keeps the generator's existing 0.9.
- **`building:levels` was not captured** — the buildings query hit a
  ConnectionError after Overpass rate-limited an earlier run. So the
  `districts` height bands are **not** yet OSM-calibrated; they currently split
  the library by measured USD height instead. This is the main open item.
- One city, one 450 m sample. Manhattan is an outlier: it is unusually
  elongated (2.3× vs Portland's ~1.0×) and 96% commercial. Numbers copied from
  it describe *a dense gridded downtown*, not cities in general.

## Street furniture — 5 cities, 700 m half-width box, 2026-08-10

```bash
uv run --no-project --with osmnx --with shapely --with scipy \
    python scene_gen/tools/osm_street_stats.py \
    --place manhattan --place chicago --place sf --place barcelona \
    --place portland --dist 700 --timeout 90 --direct --sleep 20
```

Each sample is a 1.4 × 1.4 km box = **1.96 km²** centred on the downtown.

**`frontage` is the column that maps onto a config value.**
`city_detail.categories.*.spacing_m` is metres of block-frontage ring per prop
and the ring runs along *one* kerb, so `frontage = 2 × street centreline / n`
is the directly comparable quantity, and the spacing a pedestrian perceives is
half of it. Nearest-neighbour distance is reported too but is **not** a spacing:
benches cluster in pairs, so Manhattan's 4.7 m NN median describes clumping, not
provision.

| City | street centreline | km/km² |
|---|---|---|
| Manhattan | 34.47 km | 17.6 |
| Chicago | 41.69 km | 21.3 |
| SF | 34.55 km | 17.6 |
| Barcelona | 36.56 km | 18.7 |
| Portland | 45.84 km | 23.4 |

Counts (n), coverage (per km²) and frontage-equivalent spacing (m):

| Category | Manhattan | Chicago | SF | Barcelona | Portland |
|---|---|---|---|---|---|
| bench | 67 / 34 / 1029 | 257 / 131 / 324 | 463 / 236 / 149 | 179 / 91 / 409 | 230 / 117 / 399 |
| trash_can | 63 / 32 / 1094 | 40 / 20 / 2085 | 155 / 79 / 446 | 98 / 50 / 746 | 293 / 149 / 313 |
| fire_hydrant | 34 / 17 / 2028 | 31 / 16 / 2690 | 94 / 48 / 735 | 41 / 21 / 1784 | 300 / 153 / 306 |
| bike_rack | 138 / 70 / 500 | 312 / 159 / 267 | 160 / 82 / 432 | 227 / 116 / 322 | 829 / 423 / 111 |
| mailbox | 33 / 17 / 2089 | 17 / 9 / 4905 | 63 / 32 / 1097 | 22 / 11 / 3324 | 28 / 14 / 3274 |
| streetlight | 111 / 57 / 621 | 158 / 81 / 528 | 106 / 54 / 652 | 56 / 29 / 1306 | 21 / 11 / 4365 |
| street_tree | 730 / 372 / 94 | 2395 / 1222 / 35 | 350 / 179 / 197 | 2229 / 1137 / 33 | 483 / 246 / 190 |

### How much weight these deserve — read this before copying a number

**These are lower bounds, and for most categories weak ones.** OSM records
street furniture only where a mapper walked the street and entered it, so a low
count means "unmapped" at least as often as "not there". The spread *between*
comparable downtowns is the tell: it is larger than any plausible real
difference, so it is measuring mapping effort.

- **Quantified under-count.** Midtown Manhattan returns **34 hydrants** in
  1.96 km² = 17/km². NYCDEP's own published hydrant dataset is **109,725
  hydrants** over 790 km² = **139/km²**. OSM is capturing roughly **1 in 8**.
- **Streetlights: unusable.** 11–81/km² across the five, and Portland — the
  best-mapped city for everything else — is the *worst* at 21 lamps. Nobody
  maps street lamps. Use lighting standards instead.
- **Benches, bins: weak.** 7× and 6.7× spread respectively. Chicago reports
  *fewer* bins than Manhattan reports hydrants.
- **Bike racks: the most trustworthy.** Four of five cities fall inside a 2×
  band (267–500 m); cyclists map bike parking diligently. Portland's 111 m is
  an outlier in the *dense* direction and is probably close to complete.
- **Fire hydrants: trust Portland only.** Its n=300 / 153 per km² is the one
  extract consistent with NYCDEP's independent 139/km²; the other four are
  8–10× short.
- **Street trees: bimodal.** Chicago and Barcelona (~1200/km²) have had
  inventory imports; Manhattan, SF and Portland (179–372/km²) have not. The
  imported pair is the real figure.

**Per km² is the sounder axis than frontage spacing** when comparing this
generator to a real city, because the generated grid carries only ~21.8 km of
frontage per km² against 35–47 km in all five samples. Matching frontage
spacing therefore under-provides per unit area, and matching per-km² density
over-provides per street. Where the two disagree, prefer whichever axis the
governing standard is written on — fire code is written along the street,
provision datasets are per area.

### Tooling notes, because this took several attempts

- **Overpass was never rate-limiting us.** `--endpoint` takes the interpreter
  URL, but osmnx wants the base and appends the path, so the status probe went
  to `…/interpreter/status`, which answers with an error page. osmnx cannot
  parse that and falls back to sleeping 60 s before *every* request. It looks
  exactly like throttling. Fixed in `main`.
- osmnx's own pause is broken against these servers anyway: it reads line 4 of
  `/status`, but both overpass-api.de and kumi emit an extra `Announced
  endpoint:` line that shifts the format, so it lands on `Currently running
  queries…`, sleeps 5 s and *recurses* — which never terminates against a busy
  mirror. `--rate-limit-pause off` is the fix.
- `--direct` replaces osmnx's eight per-city queries (node+way+relation with
  recursion, per tag) with two hand-written ones. Both lighter on a shared
  public API and the only path that finished reliably.
- Mirrors fail independently and transiently — kumi 504s on heavy geometry
  queries, overpass-api.de refused connections outright for a stretch. Retry
  and switch rather than concluding the data is unavailable.

## Residential lot size — ordinary suburb vs large-lot estate, 2026-08-20

Produced by `osm_lot_stats.py`. Aggregates only, same ODbL position as above.

```bash
uv run --no-project --with shapely scene_gen/tools/osm_lot_stats.py --dist 1200
```

This exists because `suburb_parcel.DENSITY` scales lot width, front setback and
lot depth per district — tight 0.78 / normal 1.00 / loose 1.45 / **estate
2.20** — and the estate row was guessed. It is measured as a *ratio* between
two samples, so it transfers even though the generator's `normal` lot is
already wider in metres than a real one (the modular house kit is oversized;
`suburb_net.yaml` documents that).

Two samples of five US places each, every box 2.4 × 2.4 km = **5.76 km²**, so
28.8 km² per sample:

| Sample | Places |
|---|---|
| **A** ordinary post-war suburb | Levittown NY, Naperville IL, Plano TX, Mesa AZ, Cary NC — the `osm_building_stats.SUBURBS` five, unchanged |
| **B** large-lot / estate | Old Westbury NY (2 ac), Potomac MD (RE-2, 2 ac), Bloomfield Hills MI (1 ac), Weston MA (1–2 ac), Greenwich CT backcountry (RA-1, 1 ac) |

### The measurements

| Measure | A suburb | B estate | B/A | n (A / B) |
|---|---|---|---|---|
| land per dwelling | 1,194 m² = **0.295 ac** (IQR 932–1,608) | 5,640 m² = **1.394 ac** (IQR 2,777–7,311) | **4.72×** | 98 / 38 polygons |
| land per dwelling, pooled | 1,238 m² = 0.306 ac | 5,470 m² = 1.352 ac | 4.42× | 11,164 / 1,227 dwellings |
| **dwellings / ha** in residential landuse | **8.08** | **1.83** | 0.23× | as above |
| dwellings / km² | 808 | 183 | 0.23× | as above |
| dwellings / km² gross over the whole box | 484 | 108 | 0.22× | 13,928 / 3,113 |
| **nearest-house spacing** | **19.5 m** (IQR 17.6–24.7) | **49.4 m** (IQR 33.2–68.1) | **2.53×** | 13,928 / 3,113 |
| building footprint area | 165 m² (IQR 125–218) | 333 m² (IQR 228–456) | 2.02× | 13,928 / 3,113 |
| footprint long side | 15.4 m | 26.4 m | 1.72× | 13,928 / 3,113 |
| coverage ratio, house ÷ land | 13.8% | 5.9% | 0.43× | median ÷ median |
| setback to road centreline | 17.4 m (IQR 15.1–19.8) | 32.4 m (IQR 22.9–50.8) | 1.86× | 13,928 / 3,110 |
| parcel frontage, where a cadastre exists | 44.4 m | 100.2 m | 2.25× | **14 / 4** |
| parcel depth | 66.1 m | 150.9 m | 2.28× | **14 / 4** |

**Sanity anchor.** Levittown alone returns 0.213 ac per dwelling and **18.1 m**
nearest-house spacing against a platted 60 × 100 ft lot — 18.29 m of frontage.
The spacing measure reproduces the recorded plat to within 1%, which is why it
carries the width multiplier below rather than the landuse-derived area does.

The setback is measured to the *centreline*; the generator's runs from the lot
front line, half a right-of-way further in. Subtracting a constant from both
sides moves a ratio, so the sweep matters more than any single value:

| Half-ROW assumed | A | B | B/A |
|---|---|---|---|
| 40 ft ROW | 11.3 m | 26.3 m | 2.32× |
| **50 ft ROW** (the common post-war residential ROW) | **9.8 m** | **24.8 m** | **2.53×** |
| 60 ft ROW | 8.3 m | 23.3 m | 2.80× |
| 66 ft ROW | 7.3 m | 22.3 m | 3.05× |

### What an `estate` class should carry

Width comes from spacing. Depth is then the residual: land = width × depth, so
once width is pinned, the rest of the 4.72× land ratio has to be depth. Doing
it the other way round would need per-parcel geometry, which US OSM mostly does
not have — note the n of 14 and 4 on the two parcel rows above.

| Multiplier vs `normal` | Current guess | Measured | Leave-one-out range | Verdict |
|---|---|---|---|---|
| `lot` (frontage width) | 2.20 | **2.53** | 2.30–2.79 | about right, a shade low → **2.4** |
| `setback` | 1.60 | **2.53** | 2.31–2.71 | **too low by ~60%** → **2.4** |
| `depth` | 1.35 | **1.86** | 1.46–2.36 | **too low** — below the whole range → **1.8** |
| `size` (house footprint) | 1.75 | **2.02** on area | — | slightly low → 1.9–2.0 |
| share of a **low-density district** that is estate | 0.06 | **0.63** | — | see below |

The leave-one-out column drops each of the five B places in turn and re-derives.
Width and setback are stable; **depth is the shaky one**, because it inherits
all the noise in the land figure.

**The weight is two different questions.** Thresholding at twice the ordinary
median spacing (≥ 39 m):

- **4.3%** of *ordinary-suburb* dwellings already sit that far apart. So
  `DENSITY["estate"]["w"] = 0.06`, read as the estate share of a *typical mixed*
  tract, is very close to right and should stay.
- **63%** of *estate-town* dwellings do. So a district that is meant to *be*
  low-density needs roughly **0.6**, an order of magnitude above the default.
  `suburb_net.yaml` currently overrides to `estate: 0.15` for the whole plat,
  which is the right shape for a mixed suburb and far too low for a preset
  intended to read as large-lot.

### Caveats, so these aren't over-trusted

- **Land per dwelling is not lot area.** It is `landuse=residential` polygon
  area ÷ dwellings inside, so it includes the streets, verges and stormwater
  inside the polygon. The bias applies to both samples, but *not equally*:
  road area per dwelling scales with frontage (2.5×) while lot area scales with
  the full land ratio (4.7×), so the street share is smaller in the estate
  sample. Netting an estimated street area off both sides moves the land ratio
  from 4.72× up to about **5.0×**, and the depth multiplier from 1.86 to ~2.0.
  **The measured depth figure is therefore a lower bound.**
- **The estate land figure rests on 39% of its dwellings.** OSM's landuse
  coverage in B is thin: the clipped union covers 23% of the box and holds
  1,227 of 3,113 dwellings, against 48% and 80% in A. Greenwich contributes
  **zero** landuse polygons and Potomac contributes 72% of the estate union
  area on its own. Nearest-neighbour spacing has no such dependency and is the
  figure to lean on.
- **Per-parcel geometry is essentially unobtainable in the US.** Across 57.6
  km² of the two samples, exactly **18** `landuse=residential` polygons held a
  single dwelling and could be measured as a real parcel. The frontage and
  depth rows are recorded because they *agree* with the derivation (2.25× and
  2.28×, against 2.53× and 1.86×), not because n = 4 supports anything.
- **Setback is to the centreline, and it excludes `highway=service`** on
  purpose: an estate driveway runs to the garage door and would report a 2 m
  setback for a 40 m one. The cost is that houses on private lanes tagged
  `service` are measured against a public road that is not their frontage,
  which inflates the estate tail — the B IQR runs 22.9–50.8 m.
- **Two of the original five estate picks were measuring OSM, not zoning.**
  Saddle River NJ returned 55 buildings in 5.76 km² (1.1 dwellings/km of road)
  and Paradise Valley AZ 4.4/km, against 11–22/km for the places that survived.
  Left in, they would have reported 6.5-acre lots with no warning. The tool now
  prints dwellings-per-km-of-road as a screen and flags anything under 8.
- **Atherton CA failed the opposite way**: 37.8 dwellings/km, 0.29 ac per
  dwelling, 23.0 m spacing — indistinguishable from sample A. The town is 13
  km², so a 5.76 km² box centred on it is mostly its denser neighbours. A
  one-acre town needs a box that fits inside it.
- **The surviving five are Northeast-heavy** (NY, CT, MA, MD) plus one Midwest.
  That is what passed the screens, not a choice: both Western/Sun Belt
  candidates failed. Large-lot platting in the West is unmeasured here.
- **53% of sample-A dwellings and only 23% of sample-B dwellings carry a
  single-family `building=*` tag**; the rest are `building=yes` admitted by a
  55–1,500 m² footprint filter. In the estate sample that filter will admit
  some pool houses and guest cottages as dwellings, which biases spacing
  *down* — so 2.53× is, again, a lower bound on the width ratio.

### Tooling notes

- **`overpass.openstreetmap.fr` 403s any User-Agent containing a space or a
  bracket.** `airstack-scene-gen/1.0 (research)` is refused; the same string
  without the bracketed part is served, and so is `curl/8.5.0`. The 403 arrives
  in 0.3 s, so the mirror answers a hand-typed `curl` and fails only under the
  tool.
- **overpass-api.de resolves to both A and AAAA records.** On a v4-only host the
  AAAA is unroutable and its v4 address was refusing connections outright for
  the whole session; the pair surfaces as `[Errno 101] Network is unreachable`,
  which reads exactly like the host being offline. It was not.
  `overpass.kumi.systems` and `overpass.private.coffee` are the same machine
  behind two names, so they fail together and count as one mirror.
- **Check any mirror you add against a known box.** `overpass.osm.ch` is a
  Switzerland-only extract that answers a Long Island query with `ways: 0` and
  HTTP 200 — it would have shown up here as "this suburb has no buildings".
- **Overpass returns whole ways that merely touch the bbox.** An unclipped
  `landuse=residential` blob made Levittown's 1.96 km² box report 8.8 km² of
  landuse, and land-per-dwelling came out 4.5× too high, because the polygon
  was unclipped while the dwellings inside it were not. The tool intersects
  every polygon with the box, and pools over the *union* rather than the sum —
  OSM nests residential polygons, which double-counted 2,522 dwellings out of
  1,816 real ones on the first run.

## Still to measure

- `building:levels` / `height` distribution → district height bands
- A second, non-gridded city for contrast
- A Western / Sun Belt large-lot sample. Both candidates tried failed the
  coverage screens above; this needs either a smaller box fitted inside the
  town or a boundary-polygon query instead of a bbox.
