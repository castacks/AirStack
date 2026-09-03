# Fast urban-fire pipeline on an OSMO pod

This is an experimental path. It does not modify or invoke
`urban_fire_cell.sh`, and it writes frozen cells under `Urban_fast` by
default. It retains the existing layout, rest, Flow, people, portability,
lighting, texture and frozen-output gates.

## What is accelerated

1. The city manifest is deduplicated by final bake stem before Isaac starts.
   The last duplicate is retained, including its original `FB_BUILD_SEED`, so
   the final stochastic bake matches the artifact the sequential pipeline
   would have left after repeatedly overwriting that stem.
2. VTK uses the experimental complementary sweep slicer and bulk NumPy-to-Vt
   authoring.
3. Per-piece soot UV rasterisation uses Numba while retaining the original
   triangle order, texel centres, barycentric interpolation and overwrite
   rules. Small equivalence tests produced identical masks and positions.
4. Region-specific sliced geometry is cached.
5. Canonical whole-building GAC slices are generated without fire/damage data
   (`signature=None`) for earthquake and tornado reuse.
6. Both forms of sliced kit are pulled from and pushed to a versioned Nucleus
   cache at `Projects/SEI-COA/scene_gen/cache/sliced_kits/v1`.
7. The standalone assembly launch is removed. The freeze launcher already
   performs the same city build, damage composition, prop placement and human
   placement before export, so the old path built the complete city twice.
8. GPU PhysX with Fabric remains enabled. Convex-decomposition threshold 0.8 m
   remains enabled because the measured hull-only experiment was slower and
   failed the rest gate.

## Files to put on the pod

If these changes are not committed and pushed, sync this explicit list after
`osmo_provision.sh` has opened the tunnel:

```bash
scene_gen/tools/osmo_sync.sh \
  scene_gen/detail/gac_storey_slice_fast.py \
  scene_gen/disaster/soot_bake_fast.py \
  scene_gen/tools/fire_city_bake_fast.py \
  scene_gen/tools/sync_sliced_kits.py \
  scene_gen/tools/experimental_warm_canonical_kits.py \
  scene_gen/tools/urban_fire_cell_fast.sh \
  simulation/isaac-sim/launch_scripts/experimental_fast_fire_bake_launch_script.py \
  scene_gen/FAST_URBAN_FIRE_OSMO.md
```

Do not copy the multi-gigabyte asset tree. The pod workflow should set:

```bash
AIRSTACK_ASSET_ROOT=omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA
```

## Preflight and dry run

The host planning artifacts must already exist:

- `scene_gen/_plans/fc_dump_1km_l<N>.json`
- `scene_gen/_plans/fire_corr_l<N>.json`
- `scene_gen/_plans/bake_l<N>.json`

Resolve the container name instead of assuming `isaac-sim`:

```bash
export CONTAINER=$(docker ps --format '{{.Names}}' | grep -m1 isaac-sim)
export REPO=/isaac-sim/AirStack
```

Inspect deduplication without writing anything:

```bash
FB_OUT=/isaac-sim/.cache/fire_bakes/urban_1km \
CONTAINER="$CONTAINER" REPO="$REPO" \
python3 scene_gen/tools/fire_city_bake_fast.py \
  scene_gen/_plans/fire_corr_l1.json --dry-run
```

Expected current-plan counts:

| level | manifest rows | unique final bakes | eliminated launches |
|---|---:|---:|---:|
| L1 | 42 | 38 | 4 |
| L2 | 74 | 44 | 30 |
| L3 | 86 | 44 | 42 |

## Run one level

Run detached on an OSMO pod. Do not send commands into a busy tmux pane.

```bash
mkdir -p /root/docker/isaac-sim/logs/urban_fire_fast
nohup env \
  CONTAINER="$CONTAINER" \
  REPO=/isaac-sim/AirStack \
  FB_NUCLEUS_KITS=1 \
  FB_CANONICAL_KITS=1 \
  FREEZE_SNAPS=1 \
  bash scene_gen/tools/urban_fire_cell_fast.sh 1 \
  > /root/docker/isaac-sim/logs/urban_fire_fast/L1_driver.log 2>&1 &
echo $!
```

Follow it with:

```bash
tail -F /root/docker/isaac-sim/logs/urban_fire_fast/L1_driver.log
```

Run the other levels by replacing the final `1` with `2` or `3`. To run all
three sequentially:

```bash
LEVELS="1 2 3" CONTAINER="$CONTAINER" REPO=/isaac-sim/AirStack \
  FB_NUCLEUS_KITS=1 FB_CANONICAL_KITS=1 \
  bash scene_gen/tools/urban_fire_cell_fast.sh
```

Outputs default to:

- frozen cells: `/isaac-sim/final_disaster_dataset/Fire/Urban_fast/`
- logs: `/isaac-sim/logs/urban_fire_fast/`
- timings: `/isaac-sim/logs/urban_fire_fast/l<N>_timings.json`
- damaged bakes: `/isaac-sim/.cache/fire_bakes/urban_1km/city_<seed>/`
- local sliced kits: `scene_gen/assets/kits/`
- remote sliced kits: `omniverse://.../scene_gen/cache/sliced_kits/v1/`

## Cache semantics

There are three different reuse layers:

| layer | key | what it skips |
|---|---|---|
| final damaged bake | kind, asset/style, level, origin, sides, seed | entire build, texture, physics and export |
| region slice | asset plus exact cut signature | slicing only |
| canonical slice | asset, `signature=None` | disaster-independent whole-building slicing |

Final-bake dedup supersedes the slice cache for exact duplicates: if an entire
damaged bake is reused, there is no slicing to accelerate. The slice cache
still helps distinct final bakes sharing physical cuts and later cells. On
SM_Building_30 F5, a region-slice cache hit reduced build time from 102.2 s to
78.5 s and total time from 159.4 s to 133.1 s. Creating that cache added about
19.4 s, so it pays back on the second matching use.

Canonical cache creation is one-time infrastructure work and is included in
the first fast run by default. Set `FB_CANONICAL_KITS=0` only when measuring a
pure cell-build time after the canonical library has already been warmed.
Set `FB_NUCLEUS_KITS=0` for an offline test with no Nucleus connection.
Publishing makes the geometry available; a future disaster implementation
still has to select `kit_bake.have_kit(name)` and call
`kit_bake.load_kit(..., signature=None)` rather than its own live slicer.

Manual cache operations:

```bash
docker exec "$CONTAINER" bash -lc 'cd /isaac-sim/AirStack && \
  bash scene_gen/tools/usd_python.sh scene_gen/tools/sync_sliced_kits.py --pull'

docker exec "$CONTAINER" bash -lc 'cd /isaac-sim/AirStack && \
  bash scene_gen/tools/usd_python.sh scene_gen/tools/sync_sliced_kits.py --push'
```

## Measured building result

SM_Building_30 F5, 333 loose bodies, 4,557 static objects, 91 fire events:

| configuration | build | settle | export | pipeline total |
|---|---:|---:|---:|---:|
| artifact-safe fast slicer | 144.1 s | 36.5 s | 2.8 s | 201.9 s |
| Numba soot rasteriser | 102.2 s | 35.8 s | 3.1 s | 159.4 s |
| Numba + region-slice cache hit | 78.5 s | 34.3 s | 2.3 s | 133.1 s |

The last result is 13.1 seconds above the under-two-minute target, so it is
not yet presented as complete. The output passed cold verification: converged,
zero moving bodies, zero below-grade bodies, zero missing textures, no physics
schemas and no Flow inside the bake.

The rejected hull-only test took 197.0 s, triggered CPU collider fallbacks,
stalled after 6,800 steps, changed the building bounds and deleted one moving
body. Do not set `SETTLE_DECOMP_M=0` for this asset.

## Current cold-cell estimates

These estimates assume canonical geometry is warm but final damaged bakes are
cold. Replace them with `l<N>_timings.json` from the pod:

- L1: 45–65 minutes
- L2: 60–85 minutes
- L3: 75–105 minutes
- combined assembly, people and freeze: provisionally 15–25 minutes per level

With a warm final damaged-bake cache, bake time becomes filesystem validation
and the combined assembly/freeze stage dominates.

## Remaining measured optimization targets

The next targets are not enabled until they pass output-equivalence gates:

1. Cache the per-asset tiled-atlas classification. It is geometry/material
   topology, not fire-state data, and currently repeats for every damage bake.
2. Reduce the seven-pass airborne fixed-point cost without changing its final
   active set, by retaining/rebuilding its acceleration structure incrementally.
3. Author export-owned materials initially to avoid the final material rehome
   and hundreds of rebinding operations.
4. Keep one Isaac process alive across a batch of independent building bakes.
   This removes roughly 10–15 seconds of Kit startup per unique output but
   requires a strong stage-reset and memory-growth gate.
5. Benchmark snapshot cost separately. `FREEZE_SNAPS=0` is valid for timing but
   must not be used for a dataset delivery that has not already been reviewed.
