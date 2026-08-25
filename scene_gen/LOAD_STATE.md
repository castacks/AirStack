# load pipeline — state

Mission 2 (`.agents/MISSIONS.md`): `airstack up` to an interactive earthquake
scene in reasonable wall-clock. This file is the durable record; the board
(`.agents/board.md`) carries the timing table.

## How to measure

    python3 scene_gen/tools/load_bench.py --config urban_quake_tiny          # warm
    python3 scene_gen/tools/load_bench.py --config urban_quake_tiny --cold   # from `down`

No instrumentation is involved. Kit stamps every log line with both an absolute
UTC time and a millisecond offset from its own start, and mirrors the
launcher's stdout into the same file, so one pass over that log dates every
phase boundary the launcher already prints. The host clock supplies only `t0`
and the readiness banner.

**cold** = `airstack down isaac-sim` + `up`. **warm** = the launcher re-sent to
the tmux pane. They differ by 5x and `airstack up` on an existing container does
nothing at all, so a timing without one of those two words is meaningless.

## Where the time goes (urban_quake_tiny, seed 42, PRE-archetype-fix)

| phase | cold s | warm s |
|---|---|---|
| kit app ready | 14.8 | 6.8 |
| kit startup (extensions + RTX pipelines) | **72.9** | 2.5 |
| config compile | 3.4 | 0.9 |
| generate + place | 5.2 | 4.3 |
| live mesh damage | — | — |
| scene colliders | 1.0 | 0.9 |
| renderer warmup (MDL/hydra) | **31.9** | 4.3 |
| settle (physics) | 3.6 | 2.4 |
| sky + Stage C targets | 0.6 | 0.7 |
| **total** | **136.8** | **24.6** |

**Two thirds of a cold load is the renderer compiling things**, and the scene
pipeline is under 10 s of it. Anyone optimising `scene_gen` for load time on a
small map is optimising the wrong 7%.

On `urban_quake_live` (archetypes off, live fracture) the picture inverts:
96 s warm, of which **64 s is `[mesh_damage]` fracturing ONE building**.

### Two corrections worth not repeating

1. The settle looked like 30 s. It is 3.6 s. A phase that ends at a print
   absorbs everything before that print, and what precedes the settle is an
   `app.update()` pump in which the renderer compiles MDL materials.
   `settle_rigid_props` reports its own setup/cook/sim/freeze and always did.
2. Live mesh damage had no phase boundary, because it runs *after* the
   placements are applied — it operates on the stage. Its 64 s was being
   charged to "scene colliders".

Both are fixed in `tools/load_bench.py`. The lesson generalises: a phase
boundary taken from a print measures "time until someone said something", which
is only the phase you meant if nothing else is happening in between.

## Landed

- **Kit's shader caches now persist across container recreates**
  (`docker-compose.yaml`). Kit keeps them under its install root,
  `/isaac-sim/kit/cache`, which the existing `.cache` mount does not cover, so
  700 MB (`nv_shadercache` 184 MB, `DerivedDataCache` 497 MB) lived in the
  container's writable layer and `airstack down` destroyed it. Evidence: after a
  cold run `nv_shadercache` holds exactly the two files that run built. The image
  ships the directory empty, checked against the image rather than a running
  container, so the bind mount masks nothing.
  **Not yet A/B measured** — expected to remove most of the ~32 s `RtPso async
  group` wait on the second and later cold starts. The first cold start after a
  `down` still pays full price while it populates the cache.

## The bake cache: already built, and currently unsound

`scene_gen/scene_cache.py` + `bake_scene.py` + `baked_scene_launch_script.py`
already implement the whole thing, two-tier and keyed the way the generator's
invariant demands (tier 1 = seed/locale/pack/layout, tier 2 = the compiled
`disaster:` block). `tests/test_caches.py` is green, 18 tests. It is not a
design task. It is a validation task, and validation currently fails:

**A host bake and a Kit generate produce DIFFERENT scenes from the same config
and seed.** `urban_quake_tiny`, seed 42:

| | host `bake_scene.py` | in-Kit `scene_launch_script.py` |
|---|---|---|
| placements | 638 | 784 |
| buildings | 6 | 4 |
| concrete | 441 | 576 |
| sidewalk | 88 | 100 |

So the layout itself differs, not just the dressing. The cause is the trap both
`urban_quake_tiny`'s header and `measure_cache.py`'s docstring already warn
about: **a plain `python3` cannot open a Nucleus asset**, so building footprints
fall back to guesses, and footprints drive block sizing. `bake_scene.py`'s
promise of "needs only usd-core, no Kit" holds only for a pack whose assets are
all local. `urban_intact` is Nucleus-backed, so it does not hold there.

Serving one of these as a cache entry for the other would be the worst failure
this cache could have — a scene that is not the scene the config names, with
nothing in the output to say so.

### The fix, and why it should work

`measure_cache.py` persists footprints keyed on `(path, axis_up)`, and it
already handles the dangerous half correctly: a remote asset that FAILED to
measure is deliberately not cached, so a host run cannot poison the file with
`None`s (the docstring records that exact incident — 156 of 214 entries). A
remote asset that measured SUCCESSFULLY *is* stored, with an empty stat token
that matches on read.

So the path is: let Kit measure once, and the host bake inherits it. The repo is
bind-mounted into the container, so both processes share
`scene_gen/assets/.measurements.json`.

**Open question, and it is the next thing to test.** That file currently holds
30 entries, all LOCAL assets, all written by a host run — no Nucleus entries at
all, although Kit demonstrably measured nine Nucleus buildings minutes earlier
(`measured house: BG_Building_A.usd -> ...` in the Kit log). `MeasureCache.save`
is registered with `atexit`, and a launcher never exits normally: it loops until
the app closes and the documented iteration loop kills it with `C-c`. The
hypothesis is that **Kit runs never persist their measurements at all**, in which
case every launch re-measures every Nucleus asset over the network, and the host
bake can never inherit anything.

Cheap to test with the container: launch, reach the banner, then check whether
`.measurements.json` has gained Nucleus entries — before killing the process and
after. If it has not, the fix is to save the cache at a point the launcher
actually reaches rather than at exit.

Until that is settled, **do not use `bake_scene.py` for a Nucleus-backed pack**.

## Open, in priority order

1. Test the measurement-cache hypothesis above. It gates the bake cache, and if
   it is right it is also a per-launch cost nobody has counted.
2. A/B the shader-cache mount: two cold starts in a row.
3. Re-baseline `urban_quake_tiny` post-archetype-fix (`bfb42c7e`) — the PRE-FIX
   rows measured a scene whose two wrecked buildings composed empty.
4. Scaling curve: `urban_quake_showcase`, then a large map, to see how
   generate / damage / renderer-warmup scale before committing to the cache.
5. `urban_quake_large` does not exist yet and has to be agreed with Mission 1,
   so that the scene benchmarked is the scene reviewed.
