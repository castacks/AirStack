# Overview of the scene generation pipeline

The scene generation configuration system has two levels: High-level and Low-level.
Users specify High-level configs, where they select along three axes — **locale**
(how the place is laid out), **disaster type**, and **severity**. These
high-level configs are compiled into low-level configs that reflect the
high-level specifications. Each locale and each disaster has its own function
in compilation which creates the low-level settings for it.

Python scripts:
[`scene_generator.py`](scene_generator.py) creates a scene given a low-level config.
[`compile_disaster.py`](compile_disaster.py) creates a low-level config given a high-level config.
[`compile_locale.py`](compile_locale.py) supplies the locale axis it applies.

```
  presets/tornado.yaml            HIGH LEVEL — where, what happened, how bad
          │                       (locale + disaster-type + severity)
          │   compile_disaster.py
          ▼
  low_level/compiled/tornado.yaml LOW LEVEL — every generator knob
          │                       (generated, do not edit)
          │                            │
          │   scene_generator.py       │ asset_set: urban
          │                            ▼
          │                  asset_sets/urban.yaml   ASSETS — what to build with
          ▼                                          (paths, scales, art conventions)
        the scene
```

Generator settings and asset sources are separate concerns: a low-level config
says *how* to lay out a city, an asset set says *what* to build it from. Point
a config at a different set to re-skin a scene without touching a single knob.

## Layout

Everything lives under `scene_gen/` at the repo root. The generator is
sim-agnostic (only `pxr`), so it does not sit inside `simulation/isaac-sim/`;
the Isaac Sim launch scripts import it from here.

| Path | What it is |
|------|-----------|
| `scene_generator.py` | The generator: layout, packing, disaster field, USD writing. |
| `generate_scene.py` | **The detailed-city entry point.** Composes the generator with the `layout/` and `detail/` passes; `generate_scene_on_stage()` writes it to a live stage. |
| `suburb_scene.py` | **The graph-suburb entry point.** `generate_suburb_on_stage()` — the `layout/suburb_net.py` fabric written to a stage, ground included, since the built-in ground pass consumes axis-aligned rects and this layout has none. |
| **`layout/`** | **Stage 1 — where blocks, roads and buildings go.** `__init__.py` holds the two config readers both subdividers need. |
| `layout/city_layout.py` | Anisotropic block subdivision — real grids are directional (Manhattan ~80 m x ~280 m), the built-in BSP tends to 1:1. |
| `layout/suburb_layout.py` | Hierarchical suburb streets with cul-de-sacs, composed as rects. |
| `layout/suburb_net.py` | The suburb as a planar **graph** — polyline centrelines, blocks recovered as faces. A junction is the primitive, so streets curve and meet cleanly. |
| `layout/site_plan.py` | **A real place, traced off an aerial** instead of generated. Same `(net, blocks, info)` as `suburb_net.generate`, read from `config/site_plans/*.yaml`; blocks carry a `role` (rubble, pad, wooded, parking…) and the `features` standing on them. See *Tracing a real site* below. |
| **`detail/`** | **Stage 2 — what is placed on that plan.** |
| `detail/city_detail.py` | Street furniture against NACTO sidewalk zones, instead of everything on one kerb line. |
| `detail/districts.py` | Zoning: which building typology goes where, and the park superblocks. |
| `detail/parks.py` | Composes each park superblock as one designed place. |
| `detail/road_markings.py` | Crosswalks, stop bars, parking bays, hatching (MUTCD). |
| `detail/suburb_lots.py` / `detail/suburb_yards.py` | Suburban lotting and yard planting, on the **rect** layout. |
| `detail/suburb_parcel.py` / `detail/suburb_yardplan.py` | The same two jobs on the **graph** layout: lots hung off frontage arclength, planting composed in each lot's own frame rather than its AABB. |
| `detail/suburb_park.py` | A large suburban park: facilities at real dimensions, a path network that reaches each, generated court and pitch surfaces. |
| **`disaster/`** | **Stage 3 — what the event does to the finished scene.** Empty here; the damage passes live on the `muyang` branch. |
| `compile_disaster.py` | High-level spec → low-level config, and `load_scene_config()`. |
| `compile_locale.py` | The locale axis: one function per locale (`downtown`/`suburban`/`rural`). |
| `preset_report.py` | Dry-run every preset and compare the results. |
| `reload_scene.py` | Regenerate on a live Isaac Sim stage without restarting. |
| `prepare_assets.py` | **Run this before `airstack up`** — caches the Objaverse assets a scene needs (see below). |
| `objaverse_assets.py` / `convert_to_usd.py` / `render_usd.py` | The Objaverse → USD asset pipeline that backs it: search, download, Blender conversion, preview. |
| `inspect_usd_asset.py` | Print a USD's prims, bbox and up-axis. |
| `config/site_plans/*.yaml` | **Traced site plans.** Road centrelines in metres plus a block role per interior point — the committed output of `tools/trace_site_plan.py`, and the source of truth from then on. |
| `config/presets/*.yaml` | **High-level specs.** Hand-written, one per scenario. Name a locale, a disaster type and a severity. |
| `config/asset_sets/shared.yaml` | **Shared assets.** Everything every locale builds with — street furniture, greenery, tiles, vehicles, people — plus `asset_root`, `asset_scale`, `sky`, `orientation`, `fallback_sizes`. No `buildings` or `debris`: those read as a specific material (concrete rubble, timber wreckage) and belong entirely to the set whose damage they are. Not named directly by a config. |
| `config/asset_sets/<locale>.yaml` | **Specialized sets.** `extends: shared`, then only what makes that locale itself: its buildings and the debris they leave. |
| `config/low_level/default.yaml` | **Base low-level config.** Hand-written: layout, packing, prop densities, roads — the generator settings, with the city undamaged. Names an asset set rather than listing assets. Also the schema reference; it has the full comments. |
| `config/low_level/compiled/*.yaml` | **Generated.** `default.yaml` + the disaster's settings, carrying `asset_set:` by reference (so it stays short and the set stays single-source). Don't edit — recompile. |
| `assets/objaverse/<uid>/` | Cache of converted Objaverse USDs, keyed by dataset uid (git-ignored, rebuilt by `prepare_assets.py`) + the committed `manifest.yaml`. |
| `notebooks/` | Asset exploration (see *Finding assets worth importing*). |

What stays in `simulation/isaac-sim/`: `utils/scene_prep.py` (Isaac Sim stage
tooling — colliders, sky, settling — shared with the plain Pegasus launch
scripts) and the launch scripts themselves, which are the sim's entry points.

### Asset sets

A set is resolved at load time by `scene_generator.resolve_asset_set()`, which
merges it *under* the config — so a config can override anything a set defines,
but normally just names one:

```yaml
asset_set: urban          # low-level config
```
```yaml
asset-set: urban          # high-level spec (compiles into the above)
```

`tags` on an asset entry select **behavior**, not locale — the generator pools
on them: `park` (park-trail furniture only), `sidewalk` (street benches),
`tree` (planters big enough for a street tree), `stump` (never potted).
Locale (`urban`/`suburban`/`rural`) is expressed by *which file* an asset
lives in, not by a tag.

#### Shared base + specializations

A bench is a bench whether the scene is a downtown or a suburb, so the props
every locale shares live once in `shared.yaml` and each locale set extends it:

```yaml
extends: shared          # asset set inherits the shared library

usds:
  buildings:             # every set defines its own — never shared
    intact: [...]
  debris:                # ditto: debris reads as a specific material, so it
    pieces: [...]        # belongs entirely to one set, not shared and appended to
```

`buildings` and `debris` are the two categories every locale set defines from
scratch rather than inheriting: concrete rubble reads as a wrecked mid-rise,
lumber and scraped earth read as a wrecked house, and putting either in
`shared.yaml` would leak one locale's wreckage into another's. Everything
*else* — street furniture, greenery, tiles, vehicles, people — genuinely is
identical across locales and lives in `shared.yaml` once.

A category can still be *extended* rather than replaced with a key written
`<name>+`, which **appends** to whatever the same key held in the parent
instead of replacing it — useful within a family of closely related sets, just
not between `shared` and a locale for `buildings`/`debris`. `extends` chains
(base-first) and detects cycles; the resolved lineage is printed at load:

```
[scene_gen] asset set: suburban (suburban.yaml <- shared.yaml)
```

To add a locale: create `config/asset_sets/<name>.yaml` with `extends: shared`,
define its `buildings`, append its `debris`, and point a spec at it with
`asset-set: <name>` (or let `locale:` pick it — see `compile_locale.py`).
`suburban.yaml` is exactly this: detached timber-frame houses in place of
downtown blocks, with lumber and scraped earth appended to the debris instead
of the cinder block a mid-rise leaves behind.

### Where assets can live

`asset_root` prefixes relative paths (normally the Nucleus library). Two
pseudo-schemes address repo-local assets instead, so a config never has to
name a host path — the repo is mounted at `/isaac-sim/AirStack` in the sim
container, and these resolve correctly on both sides:

| Prefix | Resolves to |
|--------|-------------|
| `airstack://<path>` | `<repo root>/<path>` |
| `objaverse://<uid>` | `scene_gen/assets/objaverse/<uid>/<uid>.usdc` (see below) |

Anything else containing `://` is a real URL (`omniverse://`, `file://`) and
passes through untouched, as do absolute paths. They're defined in
`scene_generator.LOCAL_ASSET_ROOTS`.

## Objaverse assets

Nucleus is finite, and building out a new asset set is bottlenecked on finding
art. [Objaverse 1.0](https://objaverse.allenai.org/) adds ~799k Sketchfab
objects, but ships **glb**, not USD. `objaverse_assets.py` closes the gap, and
`convert_to_usd.py` does the conversion through headless Blender so materials
and textures survive.

**An Objaverse asset is identified by its dataset uid — nothing else.** An
asset set names one directly:

```yaml
- usd: "objaverse://6644de89c2f0449db3de934744162b63"   # Bungalow The Chase
  target-size-m: 12
```

The cache slot is derived from the uid (`assets/objaverse/<uid>/<uid>.usdc`),
so there is no local name to keep in sync, two sets referencing the same object
share one copy, and the entry stays meaningful with an empty cache.

### Preparing the cache before `airstack up`

The cache is **derived state, not a checked-in asset**, so it has to be filled
before the sim reads it. `prepare_assets.py` is the pre-flight step — run it on
the host, then launch:

```bash
.venv/bin/python scene_gen/prepare_assets.py suburban   # what this scene needs
airstack up isaac-sim
```

```
[prepare_assets] suburban -> config/presets/suburban.yaml, config/asset_sets/suburban.yaml
[prepare_assets] 5 of 5 assets need fetching (downloading + converting to USD; …)
[objaverse] 6644de89…: not cached — downloading and converting…
[objaverse] 6644de89…: 12.0 x 10.575 x 6.404 m  26,943 faces  Bungalow The Chase
[prepare_assets] 5 prepared, 0 failed, 0 already cached.
[prepare_assets] Ready — `airstack up` will use the cache.
```

| Invocation | Scope |
|---|---|
| `prepare_assets.py` | every asset set; caches whatever is missing |
| `prepare_assets.py <config>` | just that scene — a preset, a compiled config, a bare name, or a path |
| `prepare_assets.py --asset-set urban` | one asset set directly |
| `--list` | show cached / stale / missing and exit; downloads nothing |
| `--force` | re-convert even if cached (after changing `target-size-m`) |

Re-running is a no-op in well under a second, so it's safe to put in front of
every launch. Caching is keyed on uid **and** requested size, so a changed
`target-size-m` shows up as `stale` and gets re-converted, while everything
else is skipped.

It runs on the host rather than in the container by necessity: the container
has no `uv`, `objaverse` or Blender. (Isaac Sim's own `omni.kit.asset_converter`
does work in there, but costs ~80 s of app startup per run and emits Y-up
centimetre USDs that would need renormalizing — not worth it for a step that
only has to happen once per asset.) The repo is bind-mounted, so the container
sees whatever this writes.

**If you skip it**, nothing breaks loudly: uncached assets render as
placeholder prisms, and the generator lists the missing uids at startup with
the command to fix them.

Related commands, for finding assets rather than caching them:

```bash
cd scene_gen
python3 objaverse_assets.py search house --limit 20       # metadata only
python3 objaverse_assets.py ensure <uid> --target-size 12 # cache one by hand
python3 objaverse_assets.py list --yaml                   # cached + paste-ready entries
```

`manifest.yaml` is committed and records uid, title, author, license, baked
scale and measured size — provenance and attribution, not the source of truth.
The asset sets are; `prepare_assets.py` rebuilds the cache from them.

### Sizing

**glb carries no canonical unit**, so an Objaverse asset arrives at whatever
size its author used. `target-size-m` pins a real-world dimension and the
factor is **baked into the cached USD at conversion**, which is why the entry
above has no `scale:` — there is nothing to drift from the geometry, and the
generator just measures a metric asset.

`fit` picks which dimension is pinned, and defaults to `footprint`:

| `fit` | Pins | Use for |
|---|---|---|
| `footprint` | longest of x/y | buildings and anything sized by its plan |
| `height` | z | poles, posts |
| `max` | longest of all three | long thin objects at an arbitrary orientation |

`max` matters more than it looks. A scanned branch standing on end has a tiny
plan and a 7 m length, so `footprint` scales it into a spike; `fit: max` pins
the length instead. (Debris is dropped and physics-settled, so it lands flat
either way — but at the right size.)

Change `target-size-m` or `fit` and the next run re-converts, since the cached
file is the wrong shape.

Two things that are easy to get wrong:

* **Don't infer the converted size from the source mesh.** Which source axis
  ends up "up" in the USD depends on the glTF's root transform, so a fixed
  Y-up→Z-up swap is wrong for some assets. The converter measures Blender's
  world coordinates — the ones it writes — and reports them (`--report`).
* **Untextured assets are not usable** — they read as grey blobs. Search
  filters on `textureCount` by default, but that only counts images in the
  archive; whether the materials sample them needs a look at the asset.

Converted USDs come out **Z-up**, the generator's default — no `axis-up: "Y"`.
Preview them with `uv run --script render_usd.py assets/objaverse/`, which is
the only check that textures survived conversion.

### Finding assets worth importing

`notebooks/objaverse_asset_explorer.ipynb` is the front-end for the part that
can't be automated: metadata narrows 799k objects to a few hundred, but whether
something *looks* like a suburban house — and isn't a diorama on a modelled
lawn, a photogrammetry sky-sphere, or a fantasy cottage — has to be seen. It
renders candidates offscreen on the GPU as contact sheets, and its findings
section records the failure modes and traps hit along the way.

## Usage

```bash
cd scene_gen

python3 compile_disaster.py            # compile every preset
python3 compile_disaster.py tornado    # just one
python3 compile_disaster.py --list     # what disaster types exist
python3 compile_locale.py              # what locales exist

python3 preset_report.py               # dry-run + compare the results
```

### Pointing a launch script at a config

`SCENE_CONFIG` in the launch scripts (and `reload_scene.py`) takes **either
level**, via `compile_disaster.load_scene_config()`:

```python
SCENE_CONFIG = ".../presets/tornado.yaml"             # high level — compiled in memory
SCENE_CONFIG = ".../low_level/compiled/tornado.yaml"  # low level  — used as is
SCENE_CONFIG = "tornado"                              # bare name  — resolved for you
```

A bare name is looked up in `presets/`, then `low_level/compiled/`, then
`low_level/`; an unknown one lists what is available instead of failing
obscurely.

The **`SCENE_CONFIG` environment variable overrides** the value in the launch
script, so `airstack up` can target a scene without editing source. It's
plumbed through the isaac-sim compose file, so it can come from the shell or
`.env`:

```bash
SCENE_CONFIG=suburban airstack up isaac-sim   # or set it in .env
```

Which to use: point at a **high-level spec** while iterating — it recompiles
on every run, so edits to severity, `default.yaml`, or the compiler take
effect immediately with no stale artifact in between. Point at a **compiled
low-level config** for runs you want pinned to an exact on-disk record.

## High-level spec

```yaml
locale: downtown          # downtown | suburban | rural — how the place is laid out
disaster-type: tornado    # none | earthquake | tornado | explosion | flood | hurricane
severity: 0.7             # 0..1 — 0 is pristine, 1 is as bad as that disaster gets

asset-set: urban          # optional — the locale already implies one
seed: 42                  # optional — city layout + disaster RNG
region_m: [400, 400]      # optional — city extent

epicenter: [40, -30]      # optional — earthquake / explosion / flood
heading_deg: 35           # optional — tornado track direction

overrides:                # optional escape hatch: any low-level setting,
  packing:                # deep-merged last, wins over everything
    min_parks: 4
```

The three axes are independent: any locale × any disaster × any severity.
Severity 0 compiles to a pristine city whatever the type says, and the locale
still applies (that is what `suburban.yaml` is).


# Locales: downtown vs suburb

**An asset set is not a locale.** Swapping `asset-set: urban` for
`asset-set: suburban` changes *what* gets placed but keeps every generator
setting — so the result is a downtown built out of houses: blocks paved
wall-to-wall, houses shoulder-to-shoulder at the sidewalk, a streetlight every
18 m, traffic lights at 90% of intersections, and grass only inside the two or
three designated parks. Real suburbs differ in *layout*, not just in art.

The short version: **downtown's default ground surface is pavement and its
default state is full; a suburb's default ground surface is grass and its
default state is mostly empty.** Almost every difference below follows from
that one sentence.

## Tracing a real site

The locale axis invents a plausible place. Sometimes the scene has to be **one
particular place** instead — a real training ground, a neighbourhood a mission
was flown over, a site a partner asked for by name. That path replaces the
layout stage and nothing else:

```
photos/<site>.png          the aerial (NOT in this repo — see below)
   │  tools/trace_site_plan.py        run ONCE
   ▼
config/site_plans/<site>.yaml         committed; the source of truth
   │  layout/site_plan.py             generate() -> (net, blocks, info)
   ▼
suburb_scene.generate_suburb_on_stage        unchanged from here down
```

`layout/site_plan.generate()` has the same signature and return as
`layout/suburb_net.generate()`, so parcelling, ground, markings and the disaster
stage all run as they are. A preset switches the two with one line:

```yaml
overrides:
  site_plan: disaster_city      # names config/site_plans/disaster_city.yaml
```

**The YAML is the source of truth, not the image.** Segmentation is a guess
that is right about most of a site and wrong about some of it; if the scene were
a function of that guess, re-tuning one threshold would silently move twelve
streets. So the tracer runs once, the result is committed, and a wrong street is
fixed by editing four numbers. Hand edits are listed in the spec's header so a
re-trace knows what to re-apply.

### What the tracer does, and what it cannot do

| Stage | Notes |
|---|---|
| pavement mask | Asphalt is the one surface that is both desaturated *and* blue-shifted (LAB `b*` < 0). Concrete pads, bare earth and dry grass all sit warm of neutral, where brightness alone cannot separate them. |
| lots removed, rims kept | Pavement wider than `--blob-r` is a car park, not a street — but its **perimeter drive** is a road and is the same asphalt, so the lot's rim goes back into the mask. Without that the lot bounds nothing and comes back as no block at all. |
| skeleton → graph | 8-connected thinning; junction pixels clustered so a fat four-way is one node. Every path's ends are snapped to their junction's centroid — without that the streets never join and `faces()` returns one polygon the size of the whole crop, which looks exactly like a working build. |
| pruned | Short spurs, then leaves down to the 2-connected core, **except** at the crop edge where a leaf is a road leaving the photo. |
| `--min-width-m` | The filter spur length cannot be: tracks worn across a rubble field read as asphalt *and close loops*, so they survive pruning however obviously they are not streets. They are 2–3 m across and no road on a site is. |
| widths | 2× the median distance transform along each centreline — measured, not assumed. `road_class` is only the nearest of `suburb_net.CLASSES`, and the measured width is what is kept. |
| roles | Blocks are classified by what covers them (`grass`, `wooded`, `pad`, `parking`, `staging`). **`rubble` and `collapsed` cannot be found this way** — broken concrete photographs as grey aggregate, exactly like an intact apron — so they are set by hand. |

**Scale is the one thing an aerial cannot tell you.** There is no
georeferencing in a screenshot, so pick a feature of known size and measure it:
`disaster_city` is calibrated off the parking lot, whose stall-row head lines
repeat every 119.3 px against a standard 18.3 m double-loaded module. Two
independent measurements off the same lot agree with it. Record the derivation
in `source.calibration`; a scale that is 10% wrong makes every building the
wrong size and nothing downstream will notice.

**The aerial is not committed.** Third-party imagery is not ours to
redistribute, and `coasei/CLAUDE.md` forbids a path escaping the repo root, so
the spec records only a file name. `tools/site_plan_png.py` looks for it in the
working directory, beside the spec, and then `$SITE_PLAN_AERIALS`; without it
the plan still draws, on a blank sheet.

### The loop

```bash
# once, to author the spec
AirStack/.venv/bin/python scene_gen/tools/trace_site_plan.py \
    --image photos/disaster-city.png --mpp 0.1534 --name disaster_city \
    --out scene_gen/config/site_plans/disaster_city.yaml

# after every hand edit — this reads the YAML and nothing else
SITE_PLAN_AERIALS=photos AirStack/.venv/bin/python \
    scene_gen/tools/site_plan_png.py --plan disaster_city --out /tmp/plan.png
```

`tests/test_site_plan.py` is the gate: it checks the graph closes (the biggest
block is not the whole region), that measured widths survive the build, that
every block still carries a role from the spec, and that every feature lands on
a block.

### Features: what stands on a block

A role says the block is a concrete pad. A **feature** says there is a 17 × 6 m
container on it, here, at this bearing. They live in the same spec:

```yaml
features:
- kind: building          # building rubble wreck pit vehicle container mast debris
  at: [33.7, 71.2]        # centre, metres, region-centred — the roads' frame
  size_m: [21.0, 17.2]    # [long, short] footprint
  yaw_deg: 0              # bearing of the LONG axis, CCW from east
  note: gabled steel shed
- kind: rubble
  at: [-63.5, 38.8]
  poly: [[...], ...]      # an outline, where a box will not do
```

`layout/site_plan._attach_features` hangs each on the block whose polygon
contains `at` — falling back to the block containing any vertex of the outline,
because a block is inset half a carriageway from the street centreline and a
33 m trailer parked along the edge of a pad has its midpoint in that margin.
One that touches no block (a trailer stopped on the kerb) keeps `block: None`.
They arrive as `block["features"]` and as a flat `info["features"]`.

**They are hand-authored, and that is the design, not a shortcut.** Detection
was tried and half works: local texture separates broken concrete from every
smooth surface (grass 6, concrete 3, roof 2.5, rubble 29), but canopy scores 20
and only saturation splits those two (concrete is neutral, foliage is not, even
in shadow). That pair of rules finds the real rubble fields *and* finds bare
ground between trunks along the wood line just as confidently — and below that
scale, a shipping container and a garden shed are the same bright rectangle. So
footprints are read off the aerial at 2–3× zoom against a 25 px grid and
converted once. The `rubble` entries keep their traced outlines, because a
debris field is not a rectangle.

Check them the same way they were checked here — score each footprint against
the pixels it claims, rather than eyeballing the overlay. Every entry in
`disaster_city` scores ≥ 0.60 "built-like or rough" inside its own outline, and
the ones at the bottom of that range are boxes that include an apron.

**A shadow is not a footprint.** This is the failure mode a single nadir image
cannot protect you from: the sun on `disaster_city` is SSE, so everything tall
lays a streak to the NNW, and a five-storey drill tower standing on bright
concrete reads from directly above as one long dark object. It was first
authored as a 33 × 6 m "trailer"; its plan is a 7 m square. Suspect any footprint
elongated toward the NNW, and settle it against a **second, oblique view** —
`source.oblique` in the spec names one. A tall structure is unmistakable there
and its base is visible. Nothing reads the oblique; it is for the person doing
the authoring.

### Building the scene

`suburb_scene.generate_suburb_on_stage` writes the features as **rectangular
prisms** (`detail/site_features.py`) after the ground pass — one Xform per
feature carrying the measured translate and Z rotation, with an axis-aligned box
beneath it. The prism is a stand-in that is *correct in footprint, position and
bearing* and honest about the rest; swapping in real assets later changes one
function, because everything deciding WHERE things go already happened in the
layout.

`height_m` is the one dimension a nadir image cannot give, so it is read off the
oblique by counting storeys. `site_features.DEFAULT_H` covers a feature that has
none and is a guess by construction — `test_site_features` asserts the committed
plan relies on none of them.

Each prim carries `featureKind`, `featureNote`, `featureSizeM` and
`standIn: true` in `customData`, so selecting one in a viewport answers what it
is without regenerating anything.

Preview it host-side, no Isaac needed:

```bash
uv run --script scene_gen/tools/site_scene_png.py scene.usda -o shot.png \
    --azimuth 0 --elev 30        # camera due east looking west, as the oblique was shot
```

That tool colours by prim NAME, which is why `site_features` names the box after
its feature (`building_07_box`) rather than `box`: a USD importer that collapses
a single-child Xform keeps the mesh's name and drops the parent's, and a scene
of stand-ins all called `box` cannot be checked against anything. Trees and
signs are Nucleus references — deactivate them before rendering outside the
container, or they resolve to nothing.

### Zone ground, and the triangulation it needs

A block's ground is drawn from its role: `ZONE_GROUND` in `suburb_scene` maps
`parking` to asphalt, `pad`/`staging`/`rubble` to dirt, `wooded` to rough grass
and so on, colour and material together. A block with **no** role — every block
in a generated suburb — is untouched and draws the grass it always did.

Getting there needed a real triangulator, and the reason is worth keeping:

* `_make_polygon` used to fan from the block's CENTROID, on the stated
  assumption that blocks are "convex or mildly concave". A traced block need
  not be. `disaster_city`'s largest block wraps the west and north of the site
  and **its centroid falls outside it**, so the fan swept triangles clear
  across the region and drew that one block over all fifteen others.
* `offset_polygon` insets each vertex along its own bisector, which self-crosses
  wherever the inset is deeper than the local half-width. **Nine of sixteen**
  traced blocks come out self-crossing, and the folded-back lobe is a
  negative-area loop sitting on top of the real surface — a second sheet of
  ground at exactly the same height as the first.

Both produce coplanar overlapping geometry, which is what z-fighting is. The
fix is `suburb_net.simple_rings` (cut the ring at its own crossings, keep the
positively-wound pieces) followed by `suburb_net.earclip` (triangulate a simple
concave polygon). `test_site_plan` asserts every committed block triangulates
to its own area within 3%, and separately that the largest block still defeats
a centroid fan — so nobody puts the fan back once the symptom is gone.

`site_features` uses the same pair to EXTRUDE a feature that carries a `poly`
instead of boxing it: a rubble field is concave, and its bounding box is half
again its area standing as a plateau over ground that is not debris.

### Roads: runs, fillets and junctions

A traced centreline is Douglas-Peucker output over a pixel skeleton, and the
naive way to draw it is one mitred ribbon per graph edge. That produces exactly
what it sounds like — "segments with overlap and disconnection". Four separate
things had to change, each measured:

| Symptom | Cause | Fix |
|---|---|---|
| a seam every few metres | `_connect_route` splits a street at every crossing, so one road arrives as a dozen edges joined at **degree-2 nodes** — 52 of them, where nothing actually meets | `_road_runs` merges through them: 123 ribbons became **70 runs** |
| roads overlapping in a lens at every crossing | every ribbon butt-capped AT the node, all at the same z | `_trim_run` pulls each back by the junction radius; **43 junction patches** fill the middle, so surfaces MEET instead of interpenetrating |
| a visibly scalloped kerb | `_mitre_offsets` widens by 1/cos(θ/2), and a traced corner is sharp: the carriageway bulged to **1.58x** its width | corners get a **fillet**, not an interpolating spline — see below — and the mitre limit is 1.4 rather than 2.86 |
| the fillet not helping | all 10 remaining bulges sat on a node where two traced roads met end to end, and a fillet pins each road's ends | `merge_chains` joins them first: 130 spec roads → **93**, degree-2 nodes 52 → **16**, bulge p99 1.31 → **1.08** |

**Why a fillet and not a spline.** Catmull-Rom passes *through* every control
point, so a right-angle corner stays a right-angle corner however finely it is
sampled — measured, it still turned 38° at one vertex and the bulge was
unchanged. `site_plan.spline` cuts a corner instead: it leaves the straight
`radius_m` before the vertex, arcs through, and rejoins after. Straights stay
exactly on the traced line and the corner never moves further from it than the
radius, so the road still lies where the aerial put it.

Widths are averaged **length-weighted** when two runs merge, because the tracer
measures width per skeleton edge and two halves of one street come back 5.4 m
and 5.8 m. That is measurement noise, not a change of carriageway — demanding
exact agreement rejected 51 of the 52 real joins.

Smoothing happens in `site_plan.generate`, BEFORE the routes are cut into the
graph, so the blocks are inset from the smoothed line too: the kerb the ribbon
sweeps is the kerb the block stops at.

### Zone materials and planting

`ZONE_GROUND` gives each role a `(colour, material key, tile_m)`. `tile_m` is
the world size one texture repeat covers and it is not cosmetic: every ground
material here is OmniPBR with `project_uvw`/`world_or_object` set, so the map is
projected in **world space** and `inputs:texture_scale` is the only thing
setting the grain — the mesh's UVs are ignored entirely. `_load_mat(key,
tile_m)` authors that scale and returns a separate prim per grain, because the
scale lives on the shader rather than on the binding. A car park tiled at a
lawn's 6 m reads as corduroy; rubble tiled at 12 m reads as a photograph of
rubble rather than rubble.

| role | material | tile |
|---|---|---|
| `parking` | `Road_Asphalt` | 9 m |
| `pad` | `Worn_Pavement` | 7 m |
| `staging` | `Dirt_Rough` | 6 m |
| `rubble` | `Crushed_Asphalt_Ground` | 4 m |
| `collapsed` | `Damaged_Asphalt` | 5 m |
| `wooded` / `grass` | `Grass_Countryside` | 8 m / 6 m |

**All local, on purpose.** The shared asset set points `asphalt`, `concrete`,
`sidewalk` and `asphalt_road_tile` at `Library/Stages/...` on Nucleus, and the
container here authenticates as `guest` — so every one of them returns
ACCESS_DENIED, binds nothing, and the surface renders as flat displayColor with
no warning anywhere. The zone materials are the repo's own 8K Megascans
wrappers under `assets/materials/`, and `disaster_city.yaml` repoints the road
materials at them too. `test_site_plan` asserts every zone material is declared,
is `airstack://`, and resolves to a file that exists.

`ZONE_PLANTING` is a multiplier on `open_planting.open_rate`, and zero means do
not plant. `undeveloped` used to mean both "no houses" and "open ground, plant
it"; on a site plan it only means the first, and planting at the open rate put a
wood in the middle of the car park. The numbers were set against the aerial: at
`grass` 0.45 the 27,798 m² open block took 138 of 171 trees and read as
woodland, where the photograph shows mown field with the tree line outside the
crop. At 0.10 / 3.2 it is 79 trees, 37 of them in the three genuinely wooded
lots.

**These materials cannot be checked in the offline preview.** They are MDL, and
Blender renders an OmniPBR ground as solid black — `site_scene_png.py
--keep-materials` will show you that. Kit is the only renderer here that
evaluates them, so `suburb_preview_launch_script.py` now honours `SNAP_DIR` the
way `scene_launch_script.py` does.

## Characteristics

| | Downtown | Suburb |
|---|---|---|
| **Ground inside a block** | Paved wall-to-wall; greenery only in designated parks and planters | Grass by default; every house sits on a lawn, so most of the block is green |
| **Blocks** | Small and frequent — 30–70 m, dense intersection grid | Large and long — 60–150 m, few intersections, cul-de-sacs and loops |
| **Coverage** | 70–90% of block area built; buildings touch the sidewalk, zero setback | 15–30% built; 8–12 m front setback, 5–10 m side yards, deep back yards |
| **Building form** | Mid-rise, large footprint (~48 m), flat-packed, shared party walls | Detached 1–2 storey, small footprint (~12 m), free-standing with air on all sides |
| **Streets** | Arterials common (4 lanes), lane markings throughout | Residential 2-lane, narrower lanes, arterials rare and only at the edges; many streets have no centerline at all |
| **Sidewalk** | Wide, continuous, curb-to-building | Narrow, often separated from the curb by a **grass verge**, sometimes absent |
| **Trees** | Only where potted — street trees live in planters | Trees stand **directly in grass**: front lawns, back yards, and the verge. No planters. Dominant visual element |
| **Other vegetation** | Plant boxes on the sidewalk | Shrubs against house foundations, hedges on lot lines, garden beds |
| **Street furniture** | Dense: lights ~18 m, benches, trash cans ~25 m, bus shelters, planters | Sparse to absent: lights 40–60 m, **no** benches / public bins / planters / bus stops on residential streets; hydrants remain |
| **Traffic control** | Signals at ~90% of intersections | Signals essentially never (stop signs instead) — a signal reads as downtown |
| **Vehicles** | Dense on-street parking both sides, plus moving traffic | Sparse on-street; cars belong in **driveways beside houses** |
| **People** | Continuous sidewalk pedestrians | Very few; occasional figure on a lawn or driveway |
| **Parks** | Rare, small, and the only green — high contrast with surroundings | Larger and more common, but low contrast: they read as a *break in the houses*, not a break in the concrete |
| **Skyline** | Buildings dominate the horizon, sky occluded | Tree canopy is the tallest continuous element; open sky |

## The locale axis

`locale:` in a high-level spec picks one of these, compiled by
[`compile_locale.py`](compile_locale.py) — same shape as the disaster axis: one
function per locale, registered in `LOCALES`. It is applied *under* the
disaster settings and under `overrides:`, so the axes compose freely: any
locale × any disaster × any severity.

A locale also supplies a **default asset set** (`downtown` → `urban`,
`suburban`/`rural` → `suburban`), so naming a locale is usually enough;
`asset-set:` still overrides it when you want to mix.

| Setting | downtown | suburban | rural |
|---|---|---|---|
| `layout.min/max_block_m` | 30 / 70 | 60 / 150 | 150 / 320 |
| `packing.building_gap_m` | 2.5 | 9 | 30 |
| `packing.pave_blocks` | **true** | **false** | **false** |
| `packing.setback_m` | 0 | 9 | 20 |
| `frontage.verge_m` | 0 | 1.5 | 2.5 |
| `roads.main_road_chance` / `_lanes` | 0.25 / 4 | 0.05 / 2 | 0 / 2 |
| `trees.lawn_density_per_100m2` | 0 | 0.9 | 1.6 |
| `plants.lawn_density_per_100m2` | 0 | 1.4 | 2.5 |
| `planters.*_spacing_m` | 30 / 40 | off | off |
| `streetlights.spacing_m` | 18 | 50 | off |
| `benches` / `trash_cans` / `bus_stops` | 30 / 25 / 130 | off | off |
| `traffic_lights.intersection_chance` | 0.9 | 0.05 | 0 |
| `driveways.chance` | 0 | 0.75 | 0.9 |
| `cars.density` | 0.15 | 0.04 | 0.01 |
| `humans.sidewalk_spacing_m` | 45 | 150 | off |

A `spacing_m` of **0 means "this category doesn't belong here"** and switches
the pass off — that's how a suburb has no public bins and rural has no street
furniture at all.

### The five generator changes it needed

Numbers alone couldn't do it; these are behaviours the generator gained:

1. **`packing.pave_blocks`** — a packed block used to get concrete
   wall-to-wall unconditionally. With it false the block stays lawn and only
   buildings and driveways are hard surface. *The single biggest change:*
   downtown lays 4305 concrete tiles, the same-size suburb lays 226 (all
   driveway).
2. **`packing.setback_m`** — insets the packing rect from the sidewalk, so
   houses hold back behind a front yard instead of landing on the property
   line. Skipped automatically if it would leave no room to build.
3. **`trees.lawn_density_per_100m2` / `plants.lawn_density_per_100m2`** — the
   park scatter path, enabled on ordinary residential blocks. Trees stand in
   grass rather than in planters; shrubs sit against foundations
   (`lawn_house_margin_m` is smaller than the park margin for exactly this).
4. **`driveways.*`** — a paved strip from each house to the nearest block
   edge, with a car on `car_chance` of them. The drive is offset along the
   house frontage so it runs beside the building rather than through the front
   door, and joins `house_rects` so scatter stays off it.
5. **`frontage.verge_m`** — pushes the sidewalk in off the kerb; the exposed
   strip shows the block's grass plane, giving the planting strip that reads
   as residential.

### What it looks like

Pristine scene, same 400 × 400 m region and seed, offline footprints:

```
locale     blocks houses concrete  trees plants lights signal bench  bin  cars drives  ppl
downtown       56    240     4305    260    157    400     77   198  189   116      0  207
suburban       16     99      226    856   1214     98      1    24   10    62     70   54
rural           4      9       63   1759    742      0      0     0    0     6      7   31
```

Concrete collapses by 95%, trees triple, and the street kit thins out — the
suburb is now green, sparse and set back rather than a downtown of small
buildings.


# Disasters

What separates the types is not just bigger numbers — each has a distinct
**damage field** (`disaster.field` in the low-level config) that scales every
disaster knob by position. All the `disaster.*` fractions and counts are
*maxima*, reached only where the field reads 1.0.

| Type | Field | Signature |
|------|-------|-----------|
| `none` | uniform 0 | Pristine. |
| `earthquake` | wide radial from the epicenter, never reaching zero | Structures fail in place: buildings pancake, lean and sink. Rubble drops at the facades (small `pieces_scatter_m`); nothing is blown anywhere. |
| `tornado` | narrow **path** across the region, zero outside | Total destruction in a corridor, untouched beyond it. Everything light is thrown far — cans fly, cars flipped and strewn. Trees go down inside the track and stand just outside it. A continuous band of dirt and splintered debris is dragged along the corridor across lawns, streets and open ground alike (`debris.path_*`). Low tilt/sink: torn apart, not settled. |
| `explosion` | tight radial, fast falloff to zero | The sharpest gradient of any type. Ground zero obliterated, the rest barely touched. Debris thrown outward hard. |
| `flood` | broad radial, high everywhere | Little structural loss; anything that floats is carried off and dumped. Poles stand — there was no wind. |
| `hurricane` | uniform | Tornado-like mechanisms spread evenly over the whole region at lower intensity. No untouched zone, no track. |

`preset_report.py` shows the spatial signature as `hit%` (share of the region
at intensity ≥ 0.5) and `core:edge` (ruin rate inside vs outside that zone;
`-` means nothing was built in that band):

```
config        buildings    damaged  destroyed     debris       hit%  core:edge
earthquake          138         23         38        603         98      44%:-
explosion           138         14         33        861         33    78%:13%
tornado             138         18         25        515         38     77%:3%
hurricane           138         33         27        483        100    uniform
none                138          0          0          0          0    uniform
```

## Earthquake

The generation that was hand-tuned before this two-level system reflects an
earthquake, and the `earthquake` curve is calibrated so **severity ≈ 0.5–0.6
reproduces it** (`damaged_fraction` 0.3, `destroyed_fraction` 0.4, piles
`[2, 4]`, pieces `[10, 20]`, `tilt_chance` 0.35 …). Severity below that reads
as a lesser quake, above it as a catastrophic one.

## Fate fractions at severity 1

`damaged_fraction` / `destroyed_fraction` are **maxima**, reached only where
`disaster.field` reads 1.0 (dead center of a tornado's track, ground zero of
an explosion). For any type whose own description promises total destruction
there — tornado ("total destruction in a corridor"), explosion ("nothing is
left standing" at the center) — the fraction that matters, `destroyed_fraction`,
must curve all the way to **1.0** at `severity: 1`. A ceiling below that (this
was a real bug: tornado topped out at 0.65, explosion at 0.75) leaves a
residual chance of an untouched building sitting in the middle of a
"completely destroyed" corridor even at max severity — exactly the kind of
thing that's invisible in a dry `preset_report.py` run but immediately obvious
the moment you fly over the loaded scene.

That ceiling is necessary but not sufficient. Two more places have to honor it
for oversized buildings specifically — a downtown's skyscraper-sized intact
models are much bigger than any damaged/destroyed model in the library (91 m
vs. ~52 m footprint, measured), and both of the following silently defeat a
`destroyed_fraction: 1.0` if left unguarded:

* **The anchor pass.** Any building too big to be guaranteed a random packing
  slot (an intact skyscraper is much likelier to be one of these than a
  smaller ruin) gets a *forced* placement — first come, tightest-fitting free
  block — so it appears at least once in the scene. That placement happens
  **before** any fate is ever rolled for that block, so an oversized intact
  building could get force-placed dead-center in the tornado's track with no
  regard for the disaster at all. `_anchor_ok()` gates each candidate block by
  whether the local damage intensity would plausibly produce that building's
  fate — an intact model needs a block where destruction isn't the likely
  outcome, a destroyed one needs a block where it plausibly is — so an
  oversized building can go anchored-elsewhere or unplaced-this-run, but never
  anchored somewhere that contradicts the disaster.
* **The per-slot fallback.** When packing rolls "destroyed" for a lot but no
  destroyed-pool model happens to fit that lot's exact size, falling back to
  *any* fitting model — including intact ones — is how "destroyed" quietly
  becomes "looks untouched." `_pick_building()` instead degrades to the next
  closest fate (destroyed → damaged → intact) before ever reaching for
  something the roll didn't ask for.

Verified with real Nucleus-measured footprints (a synthetic uniform fallback
size hides this entirely, since it removes the size disparity that causes
it): at `severity: 1.0`, the innermost half of a tornado's track measures 100%
destroyed, the outer half 95%, zero intact anywhere within it.

## Adding a disaster type

In `compile_disaster.py`:

1. Write `compile_<name>(sev, spec, region) -> dict` returning a `disaster`
   block — fates, `debris`, aftermath fractions, and a `field`. Interpolate
   every value from severity with `lerp` / `lerp_pair` so `severity: 0.3` and
   `severity: 0.9` both read correctly.
2. Register it in `DISASTERS`.
3. Add a `presets/<name>.yaml`.

Use the shared vocabulary so types stay comparable: `damaged_fraction` /
`destroyed_fraction` for structural loss, `debris.*` for rubble volume and
spread, `*_toppled_fraction` for street furniture, `*_scatter_m` / `*_strewn`
for how far light things moved, `humans_*` for casualties, and `field` for
where any of it applies.

Adding a *knob* (rather than a type) means teaching the generator to read it:
see the `disaster.*` handling in `build_city`, where `_hit()` and
`_hit_count()` apply the damage field.

### Ground scour along the track

Building debris hangs off each ruin, so it inherits the *layout* — it clusters
on lots and stops at the property line. A tornado track doesn't: it lays a
continuous band across lawns, streets and open fields, and that unbroken line
is the most recognisable thing about the aftermath from the air. The **path
scour** pass fills it in, sampling the whole region and keeping points in
proportion to the local damage field:

```yaml
disaster:
  debris:
    path_pieces_per_100m2: 3.2    # loose debris along the corridor
    path_piles_per_100m2:  0.65   # dirt / rubble mounds along the same
    path_min_intensity:    0.2    # below this the ground stays clean
    path_density_shape:    1.6    # >1 concentrates debris near the centerline
```

Densities are per 100 m² of **affected ground**, not of the region — the pass
integrates the field to get the affected area — so the same number means the
same thing for a narrow tornado corridor and a hurricane's region-wide
blanket. A `field` that never reaches `path_min_intensity` produces no scour at
all, which is why `none` and `flood` stay clean.

**Extent vs. density are two separate things here, deliberately.** `damage_at`
(the field everything else reads — building fate, whether a streetlight
topples) is a *plateau*: flat across the whole corridor width, only easing
right at the edge, because a tornado hits every house in its path about
equally hard. Ground scour needs a different shape: real aftermath photos show
debris visibly **densest under the vortex track and thinning well before the
edge of the corridor**, not uniform across the whole width. `make_scour_density`
computes that gradient separately from `damage_at` — same track, same width,
same falloff, but peaked instead of flat — so `path_min_intensity` still gates
where debris can appear at all (the extent, unchanged), while
`path_density_shape` shapes how it thins out within that extent. Works the
same way for `explosion`'s radial field (denser toward ground zero); a
`uniform` field (hurricane) has no core to concentrate around, so density
stays flat there.

`trees_toppled_fraction` works the same way: trees inside the track are laid
flat (and physics-settled) while trees a few metres outside stay standing —
the treeline is the sharpest edge in real tornado imagery. Stumps are excluded
from the pool that can topple, being already down.
