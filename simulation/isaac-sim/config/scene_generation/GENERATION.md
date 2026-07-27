# Overview of the scene generation pipeline

The scene generation configuration system has two levels: High-level and Low-level.
Users specify High-level configs, where they can select from different disaster types.
These high-level configs are compiled into low-level configs that reflect the high-level specifications.
Each disaster has its own function in compilation which creates the low-level settings for this disaster.

Two python scripts:
[`scene_generator.py`](../../utils/scene_generator.py) creates a scene given a low-level config.
[`compile_disaster.py`](../../utils/compile_disaster.py) creates a low-level config given a high-level config.

```
  presets/tornado.yaml            HIGH LEVEL — what happened, how bad
          │                       (disaster-type + severity)
          │   compile_disaster.py
          ▼
  low_level/compiled/tornado.yaml LOW LEVEL — every knob the generator reads
          │                       (self-contained, generated, do not edit)
          │   scene_generator.py
          ▼
        the scene
```

## Layout

| Path | What it is |
|------|-----------|
| `presets/*.yaml` | **High-level specs.** Hand-written, one per scenario. |
| `low_level/default.yaml` | **Base low-level config.** Hand-written: city layout, asset library, prop densities — everything, with the city undamaged. Also the schema reference; it is the file with the full comments. |
| `low_level/compiled/*.yaml` | **Generated.** `default.yaml` + the disaster's settings, written whole so a scene stays reproducible from one file even if `default.yaml` later changes. Don't edit — recompile. |

## Usage

```bash
cd simulation/isaac-sim

python3 utils/compile_disaster.py            # compile every preset
python3 utils/compile_disaster.py tornado    # just one
python3 utils/compile_disaster.py --list     # what disaster types exist

python3 utils/preset_report.py               # dry-run + compare the results
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

Which to use: point at a **high-level spec** while iterating — it recompiles
on every run, so edits to severity, `default.yaml`, or the compiler take
effect immediately with no stale artifact in between. Point at a **compiled
low-level config** for runs you want pinned to an exact on-disk record.

## High-level spec

```yaml
disaster-type: tornado    # none | earthquake | tornado | explosion | flood | hurricane
severity: 0.7             # 0..1 — 0 is pristine, 1 is as bad as that disaster gets

seed: 42                  # optional — city layout + disaster RNG
region_m: [400, 400]      # optional — city extent

epicenter: [40, -30]      # optional — earthquake / explosion / flood
heading_deg: 35           # optional — tornado track direction

overrides:                # optional escape hatch: any low-level setting,
  packing:                # deep-merged last, wins over everything
    min_parks: 4
```

Severity 0 compiles to a pristine city whatever the type says.


# Disasters

What separates the types is not just bigger numbers — each has a distinct
**damage field** (`disaster.field` in the low-level config) that scales every
disaster knob by position. All the `disaster.*` fractions and counts are
*maxima*, reached only where the field reads 1.0.

| Type | Field | Signature |
|------|-------|-----------|
| `none` | uniform 0 | Pristine. |
| `earthquake` | wide radial from the epicenter, never reaching zero | Structures fail in place: buildings pancake, lean and sink. Rubble drops at the facades (small `pieces_scatter_m`); nothing is blown anywhere. |
| `tornado` | narrow **path** across the region, zero outside | Total destruction in a corridor, untouched beyond it. Everything light is thrown far — cans fly, cars flipped and strewn. Low tilt/sink: torn apart, not settled. |
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
