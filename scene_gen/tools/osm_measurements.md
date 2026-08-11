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

## Still to measure

- `building:levels` / `height` distribution → district height bands
- A second, non-gridded city for contrast
