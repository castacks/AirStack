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
          │   scene_generator.py       │ asset_pack: urban
          │                            ▼
          │                  asset_packs/urban.yaml   ASSETS — what to build with
          ▼                                          (paths, scales, art conventions)
        the scene
```

Generator settings and asset sources are separate concerns: a low-level config
says *how* to lay out a city, an asset pack says *what* to build it from. Point
a config at a different set to re-skin a scene without touching a single knob.

## The three stages

Generation runs in three stages, and **a low-level config is grouped by which
stage consumes it**:

```yaml
seed: 42              # scene-wide metaparameters: seed, asset_pack,
asset_pack: urban      # exclusions, measure_usds, usds, locale
layout:               # WHERE things go — the city plan
  region_m, min_block_m, max_block_m, split_jitter, anisotropic,
  packing, frontage, roads, districts
detail:               # WHAT is placed on that plan
  trees, plants, planters, streetlights, benches, trash_cans, bus_stops,
  fire_hydrants, traffic_lights, humans, cars, driveways, parks, city_detail
disaster:             # what the EVENT does to the finished scene
  field, damaged_fraction, destroyed_fraction, debris,
  *_toppled_fraction, *_scatter_m, *_strewn
```

The ordering is the point: **a locale and a seed fully specify the layout, and
severity only decides what happens to it.** Sweeping severity gives you the
same city at different damage levels, which is what makes comparing a search
algorithm across severities mean anything. This is enforced by tests — see
[`tests/README.md`](tests/README.md).

Reading a stage: `scene_generator._stage(config, "layout")`. Writing one: a
locale compiler or a preset's `overrides:` may be authored **flat** (one line
per knob, far more readable than three levels of nesting) and is normalised by
`scene_generator.restage()`, so configs written before the grouping existed
still load unchanged.

> **Gotcha, and it cost an afternoon.** The generator is sensitive to config
> key *order* — `city_detail` walks its `categories` dict in order, placing
> furniture into a shared occupancy grid, so whichever category comes first
> wins the contested kerb. `compile_spec` therefore deep-copies with
> `copy.deepcopy`, **not** `yaml.safe_load(yaml.safe_dump(base))`: `safe_dump`
> sorts keys, which silently alphabetised the base config and made a setting
> behave differently depending on which tier it was authored in.

> **Second gotcha.** `deep_merge` recurses into nested dicts, so an empty
> override (`categories: {}`) merges to a no-op and the inherited entries
> survive — you cannot *narrow* an inherited dict. Switching a pass off needs
> an explicit flag (`city_detail.enabled`, `districts.enabled`), and a bulky
> table that one locale wants and another does not has to live with the locale
> (see `config/low_level/locales/`), not in `default.yaml`.

## Two frontage implementations, and which locale owns which

There are two street-furniture passes, and this is deliberate rather than
leftover:

| locale | `detail/city_detail.py` | the built-in frontage passes in `build_city` |
|---|---|---|
| urban | **on** — NACTO sidewalk zones | zeroed (`spacing_m: 0.0`) |
| suburban | off | **live** — 94 streetlights, 64 hydrants, 17 benches |
| rural | off | live, but mostly zeroed: no kerb to put anything on |

Exactly one owns the sidewalk in any given scene. Running both puts two
benches on it, which is why turning `city_detail` on means zeroing the
built-in spacings — and why a locale that keeps the built-in passes sets
`city_detail.enabled: false` rather than clearing `categories` (`deep_merge`
cannot narrow an inherited dict).

So a `spacing_m: 0.0` in `compile_urban` is **not** dead config. The same
key carries a real value for suburban, where the built-in pass is what places
the furniture. Deleting the built-in passes would leave suburban streets bare;
migrating suburban to `city_detail` first needs a suburban category table in
`config/low_level/locales/`.

## The pipeline, in three stages

`SPEC.md` is the authority; this is the map from it to the code.

**Stage A — bake the archetype library** (`archetypes/`). Once, exhaustively,
independent of any layout: every displaceable asset TYPE crossed with every
damage LEVEL, exported as a self-contained USD under
`assets/archetypes/<disaster>/<type>_<level>.usd`.

    python3 scene_gen/archetypes/plan.py --config urban --disaster earthquake
    scene_gen/archetypes/bake_cli.py --config urban --disaster earthquake

Runs offline and exits with a status code. Measured ~40 s per library
archetype plus one global PhysX settle, so a whole pack is hours — price it
with `plan.py` first, and use `--used-only` while iterating.

**Stage B — assemble a scene** (`bake_scene.py`, or a launch script). Layout,
detail and disaster over the placement list; each damaged building references
the archetype its level calls for instead of being fractured live. Then the
disaster's own `bake_stage_b` hook authors whatever it adds that is *not* a
placement — for fire, the Flow rig and the burn (see below); for everything
else, nothing.

    python3 scene_gen/bake_scene.py --config urban_quake_tiny --severity 0,0.5,0.9
    python3 scene_gen/bake_scene.py --config fire

**Stage C — `airstack up`** (`targets.py`, and the launch scripts). Load the
stage; place the targets, recording where they are; apply anything the USD
could not carry (`attach_runtime` — fire's carb settings; nothing for the
others).

    ISAAC_SIM_SCRIPT_NAME=baked_scene_launch_script.py \
    SCENE_CONFIG=urban_quake_showcase airstack up isaac-sim

    TARGET_SEED=7 SCENE_CONFIG=urban_quake_showcase ... # same city, new people

`scene_launch_script.py` and `generated_scene_launch_script.py` run the same
two Stage C steps after generating in process;
`baked_scene_launch_script.py` is the entrypoint-2 form that opens a scene the
cache already holds.

### The targets are not part of the scene

Victims are placed at LOAD time and deliberately never baked, because they are
not scenery — they are what is being searched for. Baking them would mean one
population per baked city, so a second search trial on the same city would be
scored against the same answer key. `targets.seed` (or `TARGET_SEED=`) re-rolls
them against an unchanged USD.

Where people are is a model, not a scatter: five cohorts — trapped inside a
collapsed footprint (edge-biased, where the survivable voids are), struck just
outside a damaged facade while escaping, caught in the street, gathered in the
parks clear of every facade, and standing at the edge of a pile — mixed by an
`occupancy` (night / day / commute) knob. The weights come from post-earthquake
casualty and USAR literature, cited in `targets.py`'s docstring. Every type but
earthquake compiles to zero weights and places nobody.

Each target carries a **visibility class** (`open` / `partial` / `occluded`),
so a run can be scored on what was findable rather than on what existed. Ground
truth lands in three places: `targets.json` beside the scene, `customData
["airstack:victim"]` on the prim, and a Replicator `class=person` label.

**Earthquake scenes have no scenery humans.** `compile_disaster` zeroes
`detail.humans.*` and the `humans_prone_fraction` / `humans_strewn` aftermath
knobs whenever `targets.owns_humans` is set, because an unlabelled pedestrian
in a scene whose humans are the ground truth is a false positive by
construction. That is what moved the earthquake snapshots.

### One class per disaster: `disaster/kinds.py`

Everything a *type* decides used to be a branch in whichever stage needed it —
`if self.disaster == "earthquake"` in the baker, `fire=(self.disaster ==
"fire")` beside it, and a launch script that was the only thing that knew how
to light a fire. A `Disaster` is that decision set in one place, with a
subclass per type, and the stages call hooks instead of naming types:

| hook | stage | what it decides |
|------|-------|-----------------|
| `field(dis, region)` | B | which `DamageField` — the config's `kind` wins, `default_field_kind` is the fallback |
| `ladder(kind)` | A, B | the rungs an asset can land on (`levels.LADDERS`) |
| `damage_archetype(...)` | A | how one clean instance is wrecked, by rung where the type has a damage script, by intensity where it does not |
| `chars_vegetation` | A | whether a felled tree comes out charred |
| `bake_stage_b(...)` | B | what the type authors onto the finished scene that is not a placement |
| `attach_runtime(stage)` | C | what a loaded scene has to be told that the USD could not carry |
| `place_targets(...)` | C | who is in the scene to be found, and where (`targets.py`; config-driven, so no subclass overrides it) |

Two things it deliberately does **not** own. `compile_disaster.py` still owns
what a severity means for each type — that is config compilation, and it runs
on the host with no stage. And which damage script wrecks a building stays in
`mesh_damage.DAMAGE_SCRIPTS`, because the live path dispatches off it too;
`Disaster.damage_script` reads that registry rather than shadowing it, so a
library can never be baked with damage the live path would not have produced.

### Fire is baked, not scripted

The fire used to be Stage C by accident: `apply_wildfire` built the emitters at
load time and a `WildfireDriver` poked their attributes off a timeline
callback, so a fire scene could not be written to a USD at all while an
earthquake scene could. Nothing about it actually needed a running sim — the
emitters are one sphere fitted per fuel prim, and the schedule is solved in
closed form from the same elliptical front the damage field uses.

So `fire.bake_emitters` authors the whole rig plus the burn as **timeSamples**,
and `bake_scene.py` produces a fire scene that plays with no Python at all.
`apply_wildfire` is unchanged and still drives the live path; both go through
one `build_emitters`, and one `burn_windows` defines the clock they share.

What still cannot bake, and why it is a sidecar rather than a bug:

| | |
|---|---|
| the voxel solve | Flow simulates at render time; there is no volume cache to write (the FlowUSD volume writer is an Isaac 6.0 feature) |
| the carb settings | `rtx/flow/*` is renderer state, not scene description — recorded as `customData` on `/World/flow` and applied by `attach_runtime` |
| segmentation of the flame | Replicator's annotators do not see the Flow volume in 5.1, so only the emitter region carries a label |

Per-emitter ground truth (`t_ignite_s`, `intensity`, `will_flame`) is stamped
on each emitter as `customData`, so the burn state at any frame is recoverable
from the file — `WildfireDriver.burn_report` can only answer that while a
driver is alive.

### severity, field, level

Three things that are easy to conflate and are not the same:

| term | what it is | scope |
|------|-----------|-------|
| **severity** | how bad the event was, continuous 0-1, from the high-level config | whole scene |
| **field** | `f(x, y) -> 0..1`, a pure spatial SHAPE (`disaster/field.py`) | whole scene |
| **level** | which rung of the disaster's ladder one asset lands on (`disaster/levels.py`) | one asset |

`local damage = field(x, y) x severity`, and the level is the rung that lands
on. **Severity shapes the field, never the ladder** — that is what lets one
archetype library serve every severity, and it is enforced by
`tests/test_severity_shapes_only_the_field.py`. Severity must not be folded
into the field either: `disaster_stage` draws against the raw field while the
fractions are already severity-lerped, so a field carrying severity would
apply it twice.

Ladders are per (disaster, asset kind) — a wildfire leaves a house
`burned_out` and the oak beside it a `snag`. Vegetation rung names are
constrained to what `disaster.vegetation.plan_for` can render.

### The caches

| cache | what it holds | keyed on |
|-------|---------------|----------|
| `assets/objaverse/` | converted Objaverse USDs | uid + `target-size-m` + fit |
| `assets/.measurements.json` | asset footprints (`measure_cache.py`) | path + axis-up, invalidated by mtime/size |
| `assets/archetypes/` | Stage A library | disaster / type / level |
| `assets/scenes/` | finished scenes (`scene_cache.py`) | tier 1: seed + pack + locale + layout; tier 2: the whole `disaster:` block |

All gitignored. The scene cache's two tiers mirror the load-bearing invariant:
a locale and a seed fix the layout, severity only decides what happens to it,
so a severity sweep is one pristine entry with several children.

## Layout

Everything lives under `scene_gen/` at the repo root. The generator is
sim-agnostic (only `pxr`), so it does not sit inside `simulation/isaac-sim/`;
the Isaac Sim launch scripts import it from here.

| Path | What it is |
|------|-----------|
| `scene_generator.py` | The generator core: layout, packing, disaster field, USD writing. Also `_stage()` / `restage()` / `STAGE_OF`. |
| `generate_scene.py` | **The urban entry point.** `build_scene()` runs the three stages in pure Python (the offline consumers use it); `generate_scene_on_stage()` adds the USD writing. |
| `suburb_scene.py` | **The suburban entry point.** `generate_suburb_on_stage()` builds a `layout/suburb_net.py` graph layout, with its own polygon/polyline ground writer. |
| `asset_pack.py` | Reads a `config/asset_packs/*.yaml` pack into plain asset entries. |
| **`layout/`** | **Stage 1 — where blocks, roads and buildings go.** |
| `layout/city_layout.py` | Anisotropic block subdivision — real grids are directional (Manhattan ~80 m x ~280 m), the built-in BSP tends to 1:1. |
| **`detail/`** | **Stage 2 — what is placed on that plan.** |
| `detail/city_detail.py` | Street furniture against NACTO sidewalk zones, instead of everything on one kerb line. |
| `detail/districts.py` | Zoning: which building typology goes where, and the park superblocks. |
| `detail/parks.py` | Composes each park superblock as one designed place. |
| `detail/road_markings.py` | Crosswalks, stop bars, parking bays, hatching (MUTCD). |
| `detail/suburb_lots.py` / `detail/suburb_yards.py` | Suburban lotting and yard planting. **Not wired yet.** |
| **`disaster/`** | **Stage 3 — what the event does to the finished scene.** |
| `disaster/disaster_stage.py` | Building fate, debris, and prop effects by response class rather than per-kind knobs. |
| `disaster/mesh_damage.py` | Deforms a building's actual geometry — the USD port of `scenegen/damage.py`'s operators, plus the profiles the disaster types compose. |
| `compile_disaster.py` | High-level spec → low-level config, and `load_scene_config()`. |
| `compile_locale.py` | The locale axis: one function per locale (`urban`/`suburban`/`rural`). |
| `config_merge.py` | The one `deep_merge`, in a module neither compiler owns (they form an import cycle). |
| **`archetypes/`** | **Stage A.** `plan.py` enumerates the grid (free, no sim); `bake.py` builds it; `bake_cli.py` is the offline entry point; `library.py` addresses the result. |
| `disaster/field.py` | The damage field: one `DamageField` subclass per shape — `uniform` / `radial` / `path` / `ellipse`. Intensity and scour density come off the same object, so the corridor debris lies in cannot drift from the corridor damage is in. The wildfire front is a field kind, not special-cased. |
| `disaster/kinds.py` | One `Disaster` per type, and the stage hooks each fills in. |
| `disaster/fire.py` | The wildfire: spread model, Flow rig, and the burn — live (`apply_wildfire`) or baked into the USD (`bake_emitters`). |
| `disaster/levels.py` | The ladders, `local_damage` and the one quantiser. |
| `measure_cache.py` | Persistent asset footprints — SPEC's "measure assets / use cached". |
| `scene_cache.py` + `bake_scene.py` | **Entrypoint 1.** Generate a scene offline into the two-tier cache. |
| `tools/split_usd_pack.py` | Splits a multi-object USD into its separable sub-prims (unrelated to asset packs). |
| `preset_report.py` | Dry-run every preset and compare the results. |
| `tools/settle_overlap.py` | Screens a scene for props that spawn inside each other — the PhysX depenetration impulse is what launches debris out of the region. Host-side, seconds. |
| `tools/plan_png.py` | **Top-down plan of a scene, in about a second.** Two maps: pristine layout, and damage over the field. Calls the same `load_scene_config` → `build_scene` the launch script does, so it previews the real pipeline. |
| `tools/damage_gallery.py` + `tools/render_damage_gallery.py` | Preview what each disaster does to a building, as a rows-are-buildings / columns-are-disasters contact sheet. Builds through the real pipeline (host `python3`), renders with Cycles (`uv run --script`). |
| `tests/` | Host-side layout guardrails — no Isaac, no Nucleus, seconds to run. See [`tests/README.md`](tests/README.md). |
| `reload_scene.py` | Regenerate on a live Isaac Sim stage without restarting. |
| `prepare_assets.py` | **Run this before `airstack up`** — caches the Objaverse assets a scene needs (see below). |
| `objaverse_assets.py` / `convert_to_usd.py` / `render_usd.py` | The Objaverse → USD asset pipeline that backs it: search, download, Blender conversion, preview. |
| `inspect_usd_asset.py` | Print a USD's prims, bbox and up-axis. |
| `config/presets/*.yaml` | **High-level specs.** Hand-written, one per scenario. Name a locale, a disaster type and a severity. |
| `config/asset_packs/shared.yaml` | **Shared assets.** Everything every locale builds with — street furniture, greenery, tiles, vehicles, people — plus `asset_root`, `asset_scale`, `sky`, `orientation`, `fallback_sizes`. No `buildings` or `debris`: those read as a specific material (concrete rubble, timber wreckage) and belong entirely to the set whose damage they are. Not named directly by a config. |
| `config/asset_packs/<locale>.yaml` | **Specialized packs.** `extends: shared`, then only what makes that locale itself: its buildings and the debris they leave. |
| `config/low_level/default.yaml` | **The schema tier.** Hand-written, grouped by stage (`layout:` / `detail:` / `disaster:`): what every knob means and a safe default for it, with the full comments and citations. Names an asset pack rather than listing assets. |
| `config/low_level/locales/<name>.yaml` | Bulky locale-owned tables that cannot live in `default.yaml` because `deep_merge` cannot narrow them — currently urban's 18 street-furniture categories. Loaded by that locale's compiler. |
| `config/low_level/compiled/*.yaml` | **Generated.** `default.yaml` + the disaster's settings, carrying `asset_pack:` by reference (so it stays short and the set stays single-source). Don't edit — recompile. |
| `assets/objaverse/<uid>/` | Cache of converted Objaverse USDs, keyed by dataset uid (git-ignored, rebuilt by `prepare_assets.py`) + the committed `manifest.yaml`. |
| `notebooks/` | Asset exploration (see *Finding assets worth importing*). |

What stays in `simulation/isaac-sim/`: `utils/scene_prep.py` (Isaac Sim stage
tooling — colliders, sky, settling — shared with the plain Pegasus launch
scripts) and the launch scripts themselves, which are the sim's entry points.

### Asset packs

A set is resolved at load time by `scene_generator.resolve_asset_pack()`, which
merges it *under* the config — so a config can override anything a set defines,
but normally just names one:

```yaml
asset_pack: urban          # low-level config
```
```yaml
asset-pack: urban          # high-level spec (compiles into the above)
```

`tags` on an asset entry select **behavior**, not locale — the generator pools
on them: `park` (park-trail furniture only), `sidewalk` (street benches),
`tree` (planters big enough for a street tree), `stump` (never potted).
Locale (`urban`/`suburban`/`rural`) is expressed by *which file* an asset
lives in, not by a tag.

#### Shared base + specializations

A bench is a bench whether the scene is an urban scene or a suburb, so the props
every locale shares live once in `shared.yaml` and each locale set extends it:

```yaml
extends: shared          # asset pack inherits the shared library

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
[scene_gen] asset pack: suburban (suburban.yaml <- shared.yaml)
```

To add a locale: create `config/asset_packs/<name>.yaml` with `extends: shared`,
define its `buildings`, append its `debris`, and point a spec at it with
`asset-pack: <name>` (or let `locale:` pick it — see `compile_locale.py`).
`suburban.yaml` is exactly this: detached timber-frame houses in place of
urban blocks, with lumber and scraped earth appended to the debris instead
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

Nucleus is finite, and building out a new asset pack is bottlenecked on finding
art. [Objaverse 1.0](https://objaverse.allenai.org/) adds ~799k Sketchfab
objects, but ships **glb**, not USD. `objaverse_assets.py` closes the gap, and
`convert_to_usd.py` does the conversion through headless Blender so materials
and textures survive.

**An Objaverse asset is identified by its dataset uid — nothing else.** An
asset pack names one directly:

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
[prepare_assets] suburban -> config/presets/suburban.yaml, config/asset_packs/suburban.yaml
[prepare_assets] 5 of 5 assets need fetching (downloading + converting to USD; …)
[objaverse] 6644de89…: not cached — downloading and converting…
[objaverse] 6644de89…: 12.0 x 10.575 x 6.404 m  26,943 faces  Bungalow The Chase
[prepare_assets] 5 prepared, 0 failed, 0 already cached.
[prepare_assets] Ready — `airstack up` will use the cache.
```

| Invocation | Scope |
|---|---|
| `prepare_assets.py` | every asset pack; caches whatever is missing |
| `prepare_assets.py <config>` | just that scene — a preset, a compiled config, a bare name, or a path |
| `prepare_assets.py --asset-pack urban` | one asset pack directly |
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
The asset packs are; `prepare_assets.py` rebuilds the cache from them.

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
locale: urban          # urban | suburban | rural — how the place is laid out
disaster-type: tornado    # none | earthquake | tornado | hurricane | fire | explosion | flood
severity: 0.7             # 0..1 — 0 is pristine, 1 is as bad as that disaster gets

asset-pack: urban          # optional — the locale already implies one
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


# Locales: urban vs suburb

**An asset pack is not a locale.** Swapping `asset-pack: urban` for
`asset-pack: suburban` changes *what* gets placed but keeps every generator
setting — so the result is an urban scene built out of houses: blocks paved
wall-to-wall, houses shoulder-to-shoulder at the sidewalk, a streetlight every
18 m, traffic lights at 90% of intersections, and grass only inside the two or
three designated parks. Real suburbs differ in *layout*, not just in art.

The short version: **urban's default ground surface is pavement and its
default state is full; a suburb's default ground surface is grass and its
default state is mostly empty.** Almost every difference below follows from
that one sentence.

## Characteristics

| | Urban | Suburb |
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
| **Traffic control** | Signals at ~90% of intersections | Signals essentially never (stop signs instead) — a signal reads as urban |
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

A locale also supplies a **default asset pack** (`urban` → `urban`,
`suburban`/`rural` → `suburban`), so naming a locale is usually enough;
`asset-pack:` still overrides it when you want to mix.

| Setting | urban | suburban | rural |
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
   urban lays 4305 concrete tiles, the same-size suburb lays 226 (all
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
urban       56    240     4305    260    157    400     77   198  189   116      0  207
suburban       16     99      226    856   1214     98      1    24   10    62     70   54
rural           4      9       63   1759    742      0      0     0    0     6      7   31
```

Concrete collapses by 95%, trees triple, and the street kit thins out — the
suburb is now green, sparse and set back rather than an urban scene of small
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
| `fire` | wide radial core, **short** falloff, zero outside | The sharpest *perimeter* of any type, as against the explosion's sharpest gradient. Inside the burn scar every building is a gutted shell — roof consumed and dropped straight in, walls standing, everything charred; a street outside it the houses are untouched. Fire moves no mass, so scatter distances are the lowest of any type. Trees carry it and go almost completely; steel poles stand. |
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
for oversized buildings specifically — an urban scene's skyscraper-sized intact
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
4. Add a `LADDERS` entry in `disaster/levels.py` — the rungs Stage A bakes.
5. Add a `Disaster` subclass in `disaster/kinds.py` if the type needs anything
   the base does not give it: a different default field shape, charred
   vegetation, or work in Stage B or C. Most types need only `name` and
   `default_field_kind`.

No stage needs editing for any of that. If you find yourself adding a branch on
the type name inside `archetypes/bake.py` or `generate_scene.py`, it belongs on
the class instead — that is the whole point of the hooks.

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

### How a building actually fails

Two mechanisms ruin a building, and the fate drawn for it decides which:

* **Asset swap** — a purpose-built ruin from `usds.buildings.destroyed` /
  `.damaged` is dropped onto the same footprint. Preferred for *destroyed*: a
  modelled collapse beats any deformation of an intact model. Footprint-checked,
  so the layout never moves.
* **Mesh damage** ([`disaster/mesh_damage.py`](disaster/mesh_damage.py)) — the
  building's own geometry is wrecked in place. Preferred for *damaged* (there
  is little authored art for "damaged but standing"), and the fallback whenever
  no ruin fits the footprint.

Mesh damage is **one pipeline in four stages**, and a disaster type gets to
choose exactly one thing about it: where the building fails.

1. **Thicken** (`solidify`). Give the shell wall volume — see below. Nothing
   downstream can look like broken masonry until a wall has a thickness to
   break through.
2. **Fail** (`failure_field`). Evaluate a scalar **failure field** over world
   space: `damage(p) ∈ [0, 1]`, how much of the material at *p* has lost its
   integrity, plus an `ejecta(p)` saying where the event threw it. **This is
   the whole of the per-disaster specialization.**
3. **Propagate** (`fracture_to_stage`). Cut the mesh along a Voronoi crack
   network whose seeds are drawn *from the field* — dense where damage is high,
   absent where it is zero — with every cut capped, so the fragments are closed
   solids rather than shells. Failure also propagates structurally: a fragment
   whose supporting column has failed comes free even if its own material did
   not.
4. **Settle**. The fragments that came free are handed to PhysX by the launch
   script; the rest stay exactly where they were cut.

There is no separate vocabulary of effects. An earlier version had one — `lean`,
`pancake`, `crumble`, `shockwave`, `punch_hole`, composed into five hand-tuned
profiles and six enumerated earthquake "failure modes" — and each was a guess at
the *appearance* of a failure rather than a statement about it. The field says
the same things once and composes: a soft storey is a horizontal band of high
damage (which falls out of noise with a storey-height grain, not out of a mode
table), a blown-out corner is a ball of damage around the charge, a torn-off
roof is damage that grows with height, and "the building leaned" is what PhysX
does to a stack of released fragments.

A fourth thing is not damage to the geometry at all but to the **materials**:
`scorch` darkens what burned. It has to author the soot as `inputs:scale` on
the `UsdUVTexture` feeding albedo rather than as a value on the shader, because
building assets drive albedo from a texture — which in USD means `diffuseColor`
is *connected*, and a connection beats a value, so setting it does nothing at
all. Only RGB is scaled (the fourth component is alpha, and a texture whose
`.a` drives opacity would char the building to transparent), and roughness is
raised only where it is *not* connected, since these assets pack roughness and
metallic into two channels of one texture.

Each type is one function, and each is a statement of mechanism rather than of
appearance:

| Type | Where it fails, and why | Where the pieces go |
|------|-------------------------|---------------------|
| `earthquake` | at the **base**: lateral demand is the accumulated inertia of everything above, so it peaks at the ground. Modulated by noise with a *storey-shaped* grain, so the building fails at a level — soft-storey, mid-storey and total collapse all emerge from that | nowhere; shaking imparts no direction |
| `tornado` | at the **top**, windward-biased: wind speed rises with height and the load goes as its square. Deep peel | thrown hard along the track, lofted |
| `hurricane` | the same mechanism, a shallower peel | carried a shorter way along the storm bearing |
| `explosion` | a **ball** around the charge with a hard edge — overpressure falls very steeply, which is what makes a blast read as a bite rather than as subsidence | radially outward from the charge |
| `fire` | at the top and in the **interior**: timber spans burn, masonry in compression does not, so the envelope survives | nowhere; it drops what it consumes where it stood |
| `flood` | a thin band at the **waterline**, and the weakest of the six: it never reaches the release threshold, so a flooded building is cut by nothing | — |

`fire` and `tornado` leave the same silhouette from above and are told apart by
exactly this: the tornado's material is a hundred metres downwind and the
fire's is inside the building.

**A field is not the whole of a disaster type.** The field says where a
building fails; what a whole *rung of the ladder* means — which mechanisms are
in play, how big the debris is, how much of the mass leaves the lot, how hard
it is thrown — is a per-disaster SCRIPT on the `mesh_damage` API, listed in
`mesh_damage.DAMAGE_SCRIPTS` and called by rung rather than by intensity:
[`disaster/quake.py`](disaster/quake.py) for `earthquake` and
[`disaster/tornado.py`](disaster/tornado.py) for `tornado`. The types without a
script still go through `damage_building` at a continuous intensity, which is
where all of them started. Severity appears in none of the scripts: its only
job is choosing which rung a building lands on, which is what lets Stage A bake
one archetype library and reuse it at every severity.

**Only the fragments that came free are settled.** `fracture_to_stage` splits
its output into *loose* and *anchored*, and only the loose ones get a placement
marked `settle`. Handing every fragment to PhysX makes gravity level the
building whatever the severity was, so a 0.3 earthquake and a 0.9 one end as
the same flat pile. Two thresholds decide it, because two different things free
a piece of a building:

* `release` — **its own material failed**. This is what tears a roof off in a
  windstorm, where everything underneath is untouched.
* `collapse` — **what held it up failed**: the column of material between it
  and the ground contains a level at or past this. A ground floor that goes
  takes every storey above it with it.

If nothing anywhere clears `release`, nothing is authored at all — an anchored
fragment is cut where the geometry was and is indistinguishable from the source
it would replace, so a building that merely cracks is left alone.

Mesh damage is a **budget**, not something every marked building gets:
thickening doubles a mesh's point count and each fragment is a prim PhysX has
to rest, so doing it to all 167 buildings a severity-0.6 urban marks would
author thousands onto a scene that has OOM-killed before. The budget goes to
the worst-hit buildings — the ones at the epicentre or on the track — and the
rest keep the tilt-and-sink the disaster stage already gave them. Buildings the
field will never break are dismissed *before* they are thickened.

```yaml
disaster:
  mesh_damage:
    material: timber        # what the buildings are made of — see below
    fracture:
      enabled: true
      max_buildings: 60     # the budget; everything else has a default
      # optional overrides — the field supplies the rest, keyed off type:
      # fragment_m (world size of a chunk), min_cells, max_cells,
      # support, release, collapse, neighbors, min_faces, cap
```

### What the buildings are made of comes from the locale, not the disaster

`material` names a row of `mesh_damage.MATERIALS` and supplies three things
the disaster has no opinion about: how thick a wall is (`thickness.wall_m`),
what SHAPE a fragment comes out (masonry a lump, timber a plank), and how big
one is by default. A stud wall breaks into the same planks whether it was
shaken or blown down, so `compile_disaster.LOCALE_MATERIAL` sets it off the
locale — `urban` is masonry at a 0.5 m wall, `suburban` and `rural` are timber
at 0.15 m — and every disaster reads it from that one place: the live path
(`mesh_damage.apply_to_stage`), the Stage A bake (`kinds.damage_archetype`)
and the per-type scripts (`quake.at_level`, `tornado.at_level`), which rescale
their masonry-quoted rubble size and blast speed by it.

Getting it wrong is the single most visible way an urban-tuned ladder fails on
a house: a 6.4 m bungalow thickened to a half-metre wall and diced into
isotropic 2 m lumps is a pile of concrete where a collapsed wooden house
belongs.

### Walls need thickness before they can break

Building assets are **hollow shells** — one surface, no depth, because nothing
needed the inside of a wall until something broke it. Break that and it shows:
a cut has a knife edge and you see through it to the unlit backfaces
of the far wall, and a Voronoi cell cut from a zero-thickness surface is a
zero-thickness surface, so the rubble is a drift of curved sheets rather than
chunks.

So `mesh_damage.solidify` extrudes every shell **inward** — outward would grow
the building past the setbacks the layout stage packed it to — and caps the
open edges, before anything breaks it. It runs after the profile (which is what
punches the holes, so their edges get rimmed) and before the fracture (which
then cuts a solid).

**Every mesh is thickened unless the asset pack says otherwise.** The pipeline
does not try to work out whether a model is already solid — two attempts at
that are in the history and both were wrong in practice. The first compared the
*enclosed volume* against a slab of the surface, which for a CLOSED building is
the volume of the air in its rooms:

| asset | enclosed volume | verdict it gave | real median thickness |
|---|---|---|---|
| `BG_Building_D` | 50,617 m³ | "solid" | **2.00 m** |
| `BG_Building_A` | 143,792 m³ | "solid" | **3.11 m** |
| objaverse house | 62 m³ | shell | 0.014 m |

so every Nucleus building in an urban scene was left as a paper balloon. The second
was a ray probe, which is right, but is still a guess about art remade on every
run. The author of an asset already knows the answer, so it is **declared**:

```yaml
usds:
  buildings:
    damaged:
      - usd: "omniverse://.../SM_SolidTower.usd"
        solid: true       # has material in its walls — do not thicken
      - usd: "omniverse://.../SM_Facade.usd"
                          # unmarked = shell = thickened
```

Read by `scene_generator.solid_assets`, which walks the whole `usds:` tree, so
the flag works in any pool. **Omitted means false.** That is the right default
because it is what nearly all of this library is, and because the two failure
modes are not symmetric: thickening a solid model costs points, while failing
to thicken a shell leaves zero-thickness walls at every cut — the artifact the
operator exists to remove. Buildings skipped this way are reported as
`already_solid` in the `[mesh_damage]` tally, so a run says how many.

Same budget shape as `fracture`, spent on the same worst-hit buildings, because
doubling the point count of every damaged building in an urban scene is not
affordable.

```yaml
disaster:
  mesh_damage:
    thickness:
      enabled: true
      wall_m: 0.5           # WORLD METRES — see below
      max_buildings: 60
```

`wall_m` is **world metres after the placement scale**, not asset units.
Verified by composing the same asset at two placement scales and measuring the
offset `solidify` actually authored:

```
placement scale 1.0    world offset: p50 = 0.2500 m
placement scale 0.01   world offset: p50 = 0.2500 m
```

so a wall is the same thickness on a centimetre-authored Nucleus tower as on a
metre-authored objaverse shed. The per-mesh cap is `max_span_frac` (0.35) of
the mesh's second-smallest bounding-box dimension, which is what stops a 5 cm
window mullion being extruded into a block; it is why a fragment's measured
thickness lands a little under `wall_m` (0.38 m at the 0.5 m default).

### Fragments are capped

The Voronoi clip cuts the slab open, so without a cap every fragment is an open
shell and looking into a fracture surface shows the hollow between the two
sheets `solidify` just built — thickness everywhere except where the viewer is
looking. Each cut leaves exactly one segment per triangle in the cutting plane,
so the boundary of the cross-section is already in hand: it is chained into
loops and filled with a centroid fan.

Measured on `BG_Building_E`, 40 fragments:

| | open boundary edges | time | scene triangles |
|---|---|---|---|
| uncapped | 24,458 (13.6%) | 7.7 s | 131,917 |
| **capped** | **4,320 (1.8%)** | 12.2 s | 181,814 |

**Only loops that actually close are filled.** Building assets are
non-manifold, so a cut across a T-junction or an unwelded seam leaves a chain
that never returns to its start; those are dropped rather than guessed at,
because a wrong cap is a sheet of geometry hanging in space while an absent one
is the edge we had before. Turn it off with `fracture: {cap: false}`.

> Capping also surfaced a long-standing bug in `_clip_by_plane`: the crossing
> predicate fired only on outside→inside, so a straddling triangle recorded one
> crossing where it has two. A triangle with one vertex inside clipped to two
> points and was **dropped**; one with two inside came out a triangle instead of
> a quad. Every cut face in every Voronoi cell was affected. Fixed — a plane
> through a unit box now yields exactly half its volume, where it used to yield
> a sixth of its faces.

### Damage cannot be finer than the model's triangles

Every hole is cut by `delete_faces`, i.e. **whole faces only**, so the coarsest
triangle in an asset is the smallest hole it can have — and the shape of that
hole is the shape of its triangles. These assets are modelled for rendering,
not for being broken: `BG_Building_D` carries triangles up to **255 m² with a
22 m edge**. Deleting one is not a hole, it is a missing wall, and no hole-radius
tuning helps, because the geometry cannot express anything smaller.

So `mesh_damage.subdivide` refines the mesh **before** the profile runs, splitting
any face whose longest edge exceeds `max_edge_m`. On `BG_Building_D` that takes
the largest triangle in the *damaged* output from 71.3 m² to 3.8 m².

```yaml
disaster:
  mesh_damage:
    subdivide:
      enabled: true
      max_edge_m: 4.0       # world metres, so rubble is the same size on a
                            # tower as on a shed
      max_points: 400000    # per mesh; a dense asset is already fine enough
```

**It is not free, and the default is the knee rather than the best picture.**
Measured on `BG_Building_E`, one earthquake, whole mesh-damage pass:

| config | secs | output tris | largest triangle |
|---|---|---|---|
| before both fixes | 0.8 | 24,603 | 125.05 m² |
| thickening fix only | 1.7 | 37,107 | 147.02 m² |
| `max_edge_m: 4.0` | **7.2** | 106,823 | **8.02 m²** |
| `max_edge_m: 3.0` | 18.8 | 320,020 | 2.32 m² |
| `max_edge_m: 2.0` | 24.8 | 333,029 | 2.72 m² |

Subdivision itself is cheap (~1 s); the cost is downstream, because the
fracture then clips 7× more geometry. 4.0 m buys a 15× granularity gain for 4×
the time and 3.0 m buys almost nothing for another 2.6×, which is why the
default sits where it does. `tools/damage_lab.py` works one building at a time,
so drop it to 2.0 freely there; a 60-building scene is a different budget.
Turn it off for a fast iteration loop, not for a final scene. Only faces that need splitting are split, which leaves a
**T-junction** against unsplit neighbours — bounded by the neighbour's own size,
which is by definition already under `max_edge_m`, and invisible on rubble.
Making it conforming needs red-green refinement; uniform subdivision would avoid
it and quadruple every asset, which `max_points` exists to prevent.

`tools/damage_gallery.py --wall-thickness 0` rebuilds any sheet with this
switched off, which is the only way to see the two side by side.

### Debris belongs to every damaged building

`debris.*` counts are per building and scaled by the local field, as everything
else is. They apply to **every** building the disaster touched, not only the
ones that got a ruin asset swapped in — a shattered, half-collapsed building
standing on a spotless lot is the most obviously wrong thing in an aerial view,
and debris is occupancy, so its absence changes what the scene *means* to a
search algorithm and not just how it looks.

```yaml
disaster:
  debris:
    pieces_per_building:  [12, 22]  # a DESTROYED building's share
    damaged_debris_scale: 0.51      # what one still standing sheds, relative
```

A damaged building's rubble also stays closer in: it drops at its own facade
rather than being thrown across the street, so the pile and piece offsets
shrink with the same factor.

### Severity has to reach the individual building

`_mesh_damage` — the intensity mesh damage runs at — is **local field ×
severity**, not the field alone. The field is *spatial*: it says this spot is at
the epicentre or inside the corridor, and `compile_earthquake` and
`compile_hurricane` both give it a core that reads exactly 1.0 at every
severity. Taking it unscaled meant every building the disaster touched was
wrecked at full strength however mild the event was, and only their *number*
changed — so a severity sweep compared two scenes full of identical ruins.

### Previewing it

[`tools/damage_gallery.py`](tools/damage_gallery.py) builds a one-building scene
per (building, disaster) through this exact pipeline and renders a sheet: rows
are buildings, columns are disaster types with pristine on the left, or
`--sweep <type>` to make the columns severities instead. The severity sweep is
the one that matters — it is the picture of the invariant above, and it is how
all three of the defects noted in this section were found.

```bash
python3 tools/damage_gallery.py --list                  # what can be previewed
python3 tools/damage_gallery.py --rows 5 --severity 0.8
python3 tools/damage_gallery.py --sweep earthquake --severities 0.2,0.6,1.0
```

Assets must be **local** — Nucleus resolves only inside Isaac Sim — which today
means the `suburban` set. [`notebooks/damage_gallery.ipynb`](notebooks/damage_gallery.ipynb)
drives the same tool interactively.
