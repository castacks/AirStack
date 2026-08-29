---
name: urban-layout
description: Placing BUILDINGS correctly in the generated city — how to tell which elevations of an asset are modelled, the corner/end/mid placement classes that follow from it, per-asset yaw-offset so a mixed-pack pool can be oriented at all, terrace vs pack morphology and the block sizing that decides which one fires, the building libraries available, and the greenery blacklist. Use with generate-urban-city, which covers the generator itself.
---

# Urban layout: getting buildings to face the right way

`generate-urban-city` covers running the generator. This is the layer above it:
**which** building goes **where**, and **which way round**. Every number here was
measured, and every rule exists because its absence was visible in a render.

## The problem

A kit building made for a game city is only modelled where the player was
expected to see it. The rest are flat untextured slabs. Measured over
GreatAmericanCity (`tools/gac_faces.py`, triangle density per elevation):

```
SM_Building_04    E 77.81   N 2.17   W 0.14   S 2.12   tri/m2
SM_Building_20    E  3.44   N 1.93   W 0.04   S 1.92
```

`SM_Building_04` carries **550x** more geometry on E than on W. Placed with W to
the street it reads as a missing wall. Placed with W against a neighbour it is
correct. Nothing about the asset says which — it has to be measured and then
respected by the layout.

## Measuring it

```bash
bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_faces.py
# any other library:
FACES_ROOT='omniverse://.../downtowncity/' FACES_EXT='.usdc' \
FACES_OUT='dtc_faces.json' FACES_NAMES='Amar_Tower,Carved_01,...' \
  bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_faces.py
```

Bins every triangle by area-weighted outward normal into N/E/S/W, keeping only
triangles within 2.5 m of that elevation's skin (interior partitions face
outward too). Writes `front`, `blank_sides`, `detailed_sides` and `place`.

**BLANKNESS IS RELATIVE, NOT ABSOLUTE.** A flat slab on a plain building and a
flat slab on an ornate one have very different densities. A fixed cut-off caught
`SM_Building_01` (0.14 against a 42.7 front) and missed `SM_Building_09` (2.80
against a 5.93 front) — both blank to the eye. Judge each side against **its own
building's best elevation** (`BLANK_RATIO = 0.55`).

## The placement classes

`place` follows from how many elevations are modelled:

| modelled sides | `place` | tag | where it may stand |
|---|---|---|---|
| 1 | `mid` | `place_mid` | interior of a run only — both flanks covered |
| 2 | `end` | `place_end` | an end of a run — one flank covered |
| 3 | `corner` | `place_corner` | a corner; only its back is blank |
| 4 | `any` | *(untagged)* | anywhere |

Declared as `tags:` beside the asset, so the class travels with the art the way
`scale` and `yaw-offset` do. `districts._pool_entries` carries it into the pool
entry; `districts._order_run` enforces it.

`_order_run` puts `corner`/`end`/`any` at the two ends of each terrace run and
`mid` in the interior. **A run it cannot lay legally is dropped, not laid
wrong** — a hole in a terrace is a vacant lot, which is a real thing; a blank
wall facing a street is not. A run of ONE only accepts an `any`, because a
lone building shows both flanks.

## Per-asset `yaw-offset` — the thing that makes any of it work

**These packs do not agree on which way is forward.** Among fifteen
GreatAmericanCity buildings: 01 fronts E, 03 fronts S, 22 fronts W, 24 fronts N.
A layout rule can only orient a pool that agrees on a front, so each asset gets
a `yaw-offset` that turns its best elevation to a common direction (-X, which is
what `_lay_terrace` assumes at `facing_deg` 0).

Without it **one rule orients some correctly and spins the rest to face the
block interior** — which is exactly the "same orientation regardless of where it
is" defect. `_normalize_usd_list` has always returned the offset;
`districts._pool_entries` used to throw it away.

## Terrace vs pack, and the block sizing that decides

| morphology | rows | orients to street? |
|---|---|---|
| `terrace` | exactly 2, back to back | **yes**, by construction |
| `pack` | unbounded (3+ seen) | no — guillotine fit only |

Both complaints — *"3 rows in a block"* and *"facing the wrong way"* — are the
same root cause: **terrace was being refused and everything fell back to pack.**

`_terrace_strips` accepts a block only when its short side is in
`[2*depth + alley_m, 2*depth + alley_max_m]`, where `depth` is the pool's
**deepest** member. So:

```
rowhouse pool deepest sx = 31.5 m,  alley_m 2.0, alley_max_m 26.0
   -> terrace band [65, 89] m
   -> districts.typologies.rowhouse.block_short_m MUST sit INSIDE that
```

**TUNE `districts.typologies.<name>.block_short_m`, NOT
`layout.anisotropic.block_short_m`.** The latter is only a fallback while
zoning is on — `city_layout`'s targets delegate to
`districts.zone_field(...).targets_at(x, y)`. Changing the layout value alone
does nothing, which cost a full rebuild here.

**DEPTH MEANS `sx`, THE X EXTENT — not `min(W, D)`.** `_lay_terrace` takes
`max(e[3]["sx"])` over the whole pool, so ONE deep member disqualifies every
block: two 44 m downtowncity fillers pushed the band to [91, 115] against
blocks of [67, 82] and refused 5 of 12 terraces. Keep the terrace pool tight.

At the inherited `[70, 105]` only a sliver overlapped and 8 of 8 rowhouse blocks
were refused (`rowhouse_refused=8`, *"terrace refused on N block(s) outside the
alley band — rebuilt as the next typology up"*). **Read that line every run —
it is the generator telling you the morphology never fired.**

Keep `pack` pools to `any`/`corner` stock, since a packed building is free-standing.

## Building libraries

| library | count | state |
|---|---|---|
| `GreatAmericanCity/.../Meshes/SM_Building_NN` | 31 | whole buildings, `scale: 0.01`, ships LOD0–LOD4 variants |
| `scene_gen/assets/downtowncity/` | 15 | 12 modelled all round — the fillers |
| `Muyang/DownTown/BG_Building_A..F` | 6 | `urban.yaml` tower/midrise pools |
| `Muyang/ModernCityEnvironment` | 4 | `urban.yaml` |
| AEC brownstones (`airstack://`) | 8 | `urban.yaml` rowhouse, 21.1 m deep |
| `selected_citydemo` | 112 | **excluded by review** — low-poly monoliths |
| `CitySample` | 20 | needs assembly, see its own notes; Nanite, no LOD |

`urban_v2.yaml` adds `selected_citydemo` via `tower+`; do not extend it unless
you want those back.

### The `urban_v2.json` baseline

`scene_gen/_plans/urban_v2.json` is generated output and is gitignored, so
`git log --follow` cannot identify a creating commit. Its filesystem timestamp
(2026-08-13 14:17), 800 m bounds, schema from `tools/plan_png.py`, and measured
block values tie it to `756b0eb6` six minutes later: **urban blocks sized from
measured assets**. The important layout changes from that commit were carried
forward through the v1/v2 retirement and file moves; restoring this baseline
means keeping the current equivalents of those rules and using `urban` as the
base asset library, not checking out the old top-level module paths wholesale.

### Mixed-depth terrace pools

Do not size an appended terrace library from its single deepest member. The
original AEC brownstones are 21.1 m deep; appending 31.5 m GAC stock moved every
block's admissible band to `[65, 89]` m and refused all eight old rowhouse
blocks. `_lay_terrace` now filters the pool per block to entries for which
`2*depth + alley_m <= block_short`, then derives the band from the deepest
fitting entry. This preserves brownstones on the original blocks while admitting
deeper new stock on blocks that can actually hold it.

## Greenery blacklist

CityPark props were reviewed and rejected. `shared.yaml` feeds them into every
urban scene through four pools — replacing only `trees` leaves the rest:

```
trees       5 CityPark trees + 3 stumps
rocks       4 CityPark stones
trash_cans  1
benches     2 SM_Log_shop
```

Current verdict (2026-08-29): **only `SM_Tree_04` survives, and no plants at
all** — `plants: []`. Use bare keys so the pool is REPLACED; `trees+` appends.

## Traps

- **A "pool" is a KEY under `usds.buildings`, not a tag.** `urban.yaml` defines
  `rowhouse`/`midrise`/`tower`; `urban_nucleus` does not. Extending the nucleus
  alone leaves all three empty and zoning replaces every building with nothing —
  `POOL EMPTY (asset paths unresolved?)`, 325 replaced by 0.
- **Retune `height_median_m` to the library.** Inherited rowhouse is 12.5 m (a
  Philadelphia row house); the shallowest GAC building is 38.6 m, so every draw
  lands outside `pick_sigma` and matching degenerates to "nearest available".
- **`repeat_radius_m: 0` disables the local repeat penalty** — one model took
  56.3% of 119 buildings. Set it to about a block (70 m).
- **Audit props/buildings by identity, not proximity.** Matching a prop to its
  building by radius fails on a real street, where the neighbour is closer than
  the far side of the same building. Carry an `of` tag.
- **A silent `str.replace` that does not match leaves the file unchanged.**
  Cost a full re-measure here. Assert the anchor.

## Files

| file | role |
|---|---|
| `scene_gen/tools/gac_faces.py` | per-elevation density → `front`/`blank_sides`/`place` |
| `scene_gen/_plans/gac_faces.json`, `dtc_faces.json` | the measurements |
| `scene_gen/detail/districts.py` | `_pool_entries`, `_order_run`, `_lay_terrace` |
| `scene_gen/detail/gac_props.py` | roof plant and fire escapes, see below |
| `scene_gen/config/asset_sets/urban_gac.yaml` | worked example of all of it |

## Building props

`detail/gac_props.py` dresses each GAC building from the pack's own 122 props
(`tools/gac_props_measure.py` → `_plans/gac_props.json`).

**There is no fire-escape asset in the pack.** The measurement identifies it:
`SM_Building_Stair` is 1.61 x 3.33 x 4.18 m — 4.18 m is one storey, and thin-on-X
is a thing that hangs off a wall — so a fire escape is that **stacked**.
`SM_Stair` is excluded despite the name: 0.06 x 0.57 x 1.92 m is a single step.

Fire escapes go **only on a blank elevation** — both how it is actually done, and
where this stock most needs something to break up a flat slab. Roof props inset
behind a 2.2 m parapet with overlap rejection; everything seats on its measured
`z0`, because several pivots are not on the base (`SM_Steel_Pipe_Plastic` at
-6.16).
