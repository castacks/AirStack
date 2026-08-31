---
name: slice-buildings-into-kits
description: Cut a MERGED whole-building asset (GreatAmericanCity, downtowncity, AEC brownstone) into kit-bash pieces — roof, per-storey corner and wall runs — so the urban-fire/earthquake ladders can damage it. Covers measuring the storey/bay grid from the asset's own windows, cutting BETWEEN window rows rather than through them, plane clipping with VTK while carrying UVs and materials inside the polydata, the exact 3x3 ring partition, the piece budget that gets a building from thousands of pieces to hundreds (bay grouping, BAY_SPLITS on/off, band-line thinning) plus the per-asset cut-offset search, the five silent bugs that each render as a flat brown box, and the three numeric acceptance tests that catch them.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Slicing a merged building into a kit

## Why this exists

`urban_fire.burn_building` and the whole 14-recipe ladder — gutted interiors,
windows out to a black void, floors and roof burnt through, partial collapse —
work by **taking elements away**. They need a building assembled from façade
modules, which `detail/urban_building.py` produces and
`quake_flow.describe` turns into an element table.

Every whole-asset pack in the city library is the opposite. MEASURED
(`tools/pack_structure_probe.py`): each is a SINGLE merged mesh whose 4-31
`GeomSubset`s are MATERIAL groups spanning the building's whole height, not
pieces.

    MCE SM_MERGED_BP_MBuilding01   1 mesh,  4 subsets, "LOD0"
    Muyang   BG_Building_A         1 mesh,  5 subsets
    Dmytro   Building_TypeA_A      1 mesh,  9 subsets
    GAC      SM_Building_01        1 mesh, 14 subsets

Nothing can be removed from one and nothing can be bound per storey, so the
ladder degenerates to one flat multiplier over the asset — the uniform grey
box. Slicing is what gives those recipes something to work on.

`detail/gac_storey_slice.py` is the implementation. The AEC brownstones are the
exception that needs no slicing: measured at **307 meshes / 491 instance
proxies**, they are already a bag of parts and only need de-instancing.

## The pipeline

    measure the grid  ->  choose cut lines  ->  clip  ->  write pieces
    (from the windows)    (BETWEEN rows)        (VTK)     (subsets + UVs)

### 1. Measure the grid, never assume it

`gac_slice.storey_period` is a PERIODOGRAM over the window centres: it scores
every candidate period by the circular mean of the centres' phase. Do NOT use
`median(diff(sorted(z)))` — that measures the gap between whichever two
features happen to be adjacent, so a mullion halves it and a missing row
doubles it. Measured on the same asset, naive clustering read `SM_Building_24`
as **1.48 m**; the periodogram reads **3.88 m**.

    SM_Building_01  3.98 m   SM_Building_04  4.08 m   SM_Building_24  3.88 m
    SM_Building_02  3.56 m   SM_Building_09  4.10 m

`urban_fire.MONO_STOREY_M` assumes 3.4 m. That is 0.6 m per floor — nearly two
storeys of drift over thirteen, with every cut through the glazing.

`confidence` is the circular-mean magnitude; below `MIN_CONFIDENCE` the grid
was not recovered and the asset must be left un-sliced rather than cut wrong.

### 2. Cut BETWEEN the window rows

The periodogram is phase-locked to the window CENTRES, so cutting on the
lattice it returns slices every window in half. A real floor line is in the
**spandrel** — the blind band between one window's head and the next one's
cill — which is half a period away. `cut_lines(g, offset=0.5)` does that:

    ON the window lattice    worst clearance 0.001 m   windows cut 4883/7698 (63.4%)
    SHIFTED half a storey    worst clearance 0.331 m   windows cut  140/7698 ( 1.8%)

The 1.8% residual is windows off the dominant lattice (shopfront, odd rows).
`window_clearance()` is the acceptance test.

### 3. Clip with VTK, and put the attributes INSIDE the polydata

`disaster/fracture.py` already installs VTK as its "C++ plane clipping
backend" and wraps `vtkClipPolyData`. That is the right primitive. What it
does NOT do is carry attributes — `fracture._to_vtk` builds points and polys
only, so a slice comes back with correct geometry, no UVs and no material.

So put them in, and the clipper handles them for free:

| attribute | goes in as | what VTK does with it |
|---|---|---|
| UVs | **point data** (`SetTCoords`) | interpolates at every vertex created on the cut |
| material index | **cell data** | copies onto the output triangles |

That is the whole answer to "can we combine the material into the mesh so it
gets sliced with it" — yes, and it removes the need for the barycentric UV
rebuild `monolith_damage.cut_shell` hand-rolls with Shapely.

**De-index on the way in.** USD stores UVs `faceVarying` (one per face-corner)
while VTK point data is one per POINT, so a shared vertex with two different
UVs cannot be represented. Split every triangle into its own three vertices.

**Never bin whole triangles by centroid.** The first attempt assigned each
triangle to a cell without cutting it; at a 4 m cell against metre-scale
triangles the boundary is ragged by up to a triangle, which renders as a
SAWTOOTH and diagonal steps instead of a storey line.

### 4. The ring partition is a 3x3 grid in plan

Cut the plan at `x0+leg`, `x1-leg`, `y0+leg`, `y1-leg`: four outer corners are
the CORNER pieces, four edge strips are the runs, the middle cell is the
interior CORE. Nine regions, no overlap and no gap — overlap duplicates
geometry into two pieces, a gap drops it silently.

Each run is divided into bays, and each bay into **unequal thirds** —
`BAY_SPLITS = (0.26, 0.48, 0.26)`, a wide opening panel between two narrower
piers. Equal thirds do not read as a façade.

Unconditionally, on `SM_Building_01`: 15 bands -> **1,155 pieces** (360 wall,
720 pier, 60 corner, 15 core) — the raw cost before "The piece budget" below
caps it to 255.

## The piece budget

`ring()` run unconditionally on every band is what turns "hundreds of
pieces" into thousands — `4 + 1 + bays * 3` per band (4 corners, 1 core,
`BAY_SPLITS`'s 3 sub-panels per bay cell), summed over every storey.
MEASURED (2026-08-29) at that unconditional grid: `SM_Building_02` 696,
`_24` 563, `_04` 878, `_01` 1,155, `_09` 2,055 pieces. The user: "It
shouldn't split into thousands of pieces. More like hundreds."

`plan_slice_budget` decides the whole cut BEFORE any VTK clip runs, off the
asset's bbox and its measured grid alone (cheap: no mesh, no VTK), with
three levers spent in order, only as far as needed:

1. **Bay grouping** (`choose_bay_budget`'s `k`) — ring at a coarser pitch
   (`bay_native * k`) than the asset's own bay spacing, `BAY_SPLITS` (thirds)
   still on. Tried first and increased until `TARGET_PIECES` (300) is met —
   "the cleanest lever," spent wherever the budget affords it.
2. **`BAY_SPLITS` off** — only if no grouping keeps thirds under target:
   collapse each bay to one panel instead of three, then re-search the
   grouping.
3. **Band-line thinning** (`_thin_upper_lines`, last resort) — only for a
   building neither lever gets under `MAX_PIECES` (500): merge storey lines
   above the bottom `PROTECT_BANDS` (20) into groups of `stride`, always
   keeping the mesh's own top edge so the roof band's height — and
   `roof_and_parapet`'s split decision — is unaffected. MEASURED: only 3 of
   31 GAC buildings ever reach this lever (all >55 real bands on an 84x57 m+
   footprint); on the other 28 bay-grouping alone clears `MAX_PIECES`.

VERIFIED (2026-08-30) on six benched assets, all resolved by lever 1 alone
(thirds kept, no thinning needed): `SM_Building_01` 255, `_02` 239, `_04`
188, `_09` 254, `_24` 253, `_06_Small` 271 pieces — against unbudgeted native
counts of 563-2192 on this stock. Acceptance: flat cuts (worst overshoot
<= 1e-6 m), `ring_verify.py`'s ring-partition area delta **0.0000%** on all
six, every material survives the clip, UVs stay in 0..1.

**The cut offset is per-asset too, not the 0.5 `cut_lines` used to take on
faith.** `choose_cut_offset` scores every offset in `OFFSET_GRID`
(0.025-0.975, the full period — 0.0/1.0 excluded, that is the window
lattice itself, `cut_lines`'s known worst case) against the same
`window_clearance` test `cut_clearance_verify.py` uses, and only moves off
0.5 on a STRICT improvement. MEASURED: `SM_Building_24` went from 11.7% of
its windows cut at the fixed 0.5 offset to 0.8% at 0.775; `SM_Building_01`
1.8% -> 0.2% at 0.55. **`cut_clearance_verify.py` itself still hardcodes
offset 0.5** — it now checks what the OLD fixed cut did, not what
`slice_to_kit` actually cuts on for an asset the search has moved; noted
here rather than fixed.

**`roof_and_parapet` now guarantees a `role="roof"` piece exists at all**
(`_ensure_roof`). Before it, `SM_Building_06_Small` and `SM_Building_09` had
NONE: the wall/upstand split ran without error, but the piece this function
relabels `core` -> `roof` came back with zero triangles, so every recipe
doing `_els(role="roof")` (roof_burnthrough, roof_scorch, the
fire_collapse roof loop) silently no-opped on those two buildings with
nothing to catch it.

**The padded z end caps** (`storeys()`, guarding the same
exact-plane-coincidence VTK drop `ring()`'s `OUTER_PAD_M` already guards in
plan) closed the rest of the base/roof area loss: 0.055% -> 0.000% on
`SM_Building_01`.

**Baked kits are a path, not yet used.** `tools/bake_gac_kits.py` +
`kit_bake.load_kit` can save a slice to `scene_gen/assets/kits/<asset>.usd`
once and reference it back in rather than re-slicing every launch — but
`scene_gen/assets/kits/` does not exist yet, nothing has been baked.
`kit_bake.fingerprint()` hashes `gac_storey_slice.py` + `gac_slice.py` byte
for byte, so every fix in this section (the budget, the offset search, the
roof guarantee) already changed it; whatever bakes first will fingerprint
against the code as it stands now, not against anything measured before
this section existed.

### Region-only slicing: split only the storeys a fire can reach

The budget above still rings EVERY band — cheaper per band, but still one
`ring()` per storey top to bottom, on a building where a fire only ever
touches a handful of them (`_severity` is exactly 0 below `fire["origin"]`,
and tapers to ~0 within a couple of storeys above `fire["top"]`). MEASURED
(2026-08-29) on `SM_Building_02` (12 storeys, fire band 3-7 on E/W): ringing
storeys 3..11 because the budget does not know the fire stops at 7 costs 173
pieces for 239 unregioned — most of it spent on 4 storeys (8-11) nothing in
the ladder will ever look at.

`slice_to_kit(..., region={"origin": o, "sides": (...), "top": t})` fixes
both ends. Storeys `0..o-1` become ONE merged piece (role `wall`, storey 0)
instead of being ringed at all — done by dropping every floor line below
`o`'s from the list handed to `storeys()` and letting `_closed_lines` close
the mesh's own bottom onto what is left, no separate re-merge step. From `o`
up, a run on a side not in `_region_hot_sides(sides)` (the fire's own sides
plus whichever elevation shares a CORNER with one of them — the fire still
bleeds round that corner at reduced strength) collapses to one piece per run
instead of bays/thirds; corners and the core are unaffected either way.
`top`, when given, does the SAME trick from above: storeys `o..t` inclusive
are ringed exactly as just described, and everything above `t` — the
parapet/roof band included — collapses to a second merged piece (role
`wall`, storey `t + 1`), the mirror image of the below-origin merge, with NO
`roof_and_parapet` split and NO `role="roof"` piece produced. `top` missing,
or `>= n_storeys - 1`, means "ring to the roof" — today's behaviour,
unchanged, synthetic roof slab included; `top < origin` clamps to `origin`.

`gac_fire.burn_gac` only asks for the upper merge when the fire itself never
needs the real roof either: `top = n_storeys - 1` (no merge) whenever
`fire["roof"]` is already true OR the level's own recipe list contains
`"roof_burnthrough"` or `"fire_collapse"` (`floor_burnthrough`/
`partial_collapse` never reach above `fire["top"]` on their own, and every
level that runs `partial_collapse` — F5c — has `BAND["F5c"] = (4, 99, 1.0)`,
which forces `fire["roof"]` true by construction anyway); otherwise
`top = fire["top"]`. That is also why every real consumer of a
`role="roof"`/`"parapet"` element already tolerates finding none:
`_deck_slab` returns `None` on an empty `role="roof"` list,
`r_roof_burnthrough`/`r_fire_collapse` no-op or skip cleanly on one (the
same tolerance a building whose roof tile is already `dead` from an earlier
pass needs), and `_fix_advertised_bands` removes the advertised `parapet`
band from the style spec rather than leaving one nothing backs.

MEASURED (2026-08-30), origin=3, sides=(E, W), region+top capped at storey 7:

| building | full | region (no `top`) | region, `top=7` |
|---|---:|---:|---:|
| `SM_Building_02` | 239 | 173 | 92 |
| `SM_Building_01` | 255 | 205 | 87 |
| `SM_Building_24` | 253 | 185 | 117 |
| `SM_Building_06_Small` | 272 | 222 | 87 |

`plan_slice_budget` predicts the count before either merge runs
(`_band_cost_region` for `o..top`, +1 below, +1 above) and prints it on its
own `region budget:` line; the prediction can undershoot the real saving on
a wide bay grouping (`SM_Building_02` F3: predicted 117, actual 92) — the
same slack the un-regioned budget already carries on the same building (208
predicted vs 173 actual), not something the `top` bound introduces.

**`top` exposed a pre-existing area-conservation bug in `_ensure_roof`'s
synthetic-slab fallback — it did not create one.** `_gac_region_probe.py`'s
area-delta check came back "LEAKY" at 1.95% for `SM_Building_06_Small`'s
`top=7` case and 0.0000% for the other three. Measured cause:
`SM_Building_06_Small`'s real top band never has a non-empty `ring()` core
at any height it is cut (whole-band, split-lower, split-upper alike) — the
too-thin-for-a-middle guard in `ring()` returns the WHOLE band as one
relabelled piece, which already covers 100% of that band's real area, and
`_ensure_roof` then synthesises a slab ON TOP of it anyway because no piece
came back labelled `roof`. That slab (measured ~496 m2 of surface, not the
247.2 m2 footprint the log line names — a box has six faces) is fabricated
area with no counterpart in the source mesh, and it lands in `full` and in
the no-`top` region cut alike, because both still run `roof_and_parapet` on
the real top band. `region={"top": 7}` never reaches `roof_and_parapet` for
this building at all (storey 7's band is the last one actually ringed; the
real top band is on the far side of the upper merge), so its reported area —
24927.218 m2 — matches the RAW, UN-SLICED source mesh's own triangle area
to six significant figures (`gac_storey_slice.read_mesh` on the bare asset,
measured directly). The `full` and no-`top` numbers are the ones 1.95% too
big, not a regression from this change. Left as found: fixing
`roof_and_parapet`/`_ensure_roof` is outside this change's scope, and is
noted here so the next reader does not "fix" the one number that was right.

## The bug catalogue — every one renders as a flat brown box

These cost most of a day. All five leave the geometry perfect, which is what
makes them look like each other.

1. **`uv0` vs `st`.** GAC names its UVs `st` (and `SM_Building_04`/`_24` carry
   `st1`-`st3`). Hardcoding `uv0` found nothing, wrote pieces with no UVs, and
   every face sampled one texel. Discover every faceVarying texCoord primvar
   instead of naming one. **`monolith_damage.cut_shell` still hardcodes
   `uv0`.**
2. **`familyName` must be `materialBind`.** `CreateGeomSubset(geom, name,
   elementType, indices)` leaves the family EMPTY, and renderers only consult
   the `materialBind` family for per-face materials — so every subset is
   ignored and the mesh collapses to one material. The source assets' own
   subsets are all `family='materialBind'`.
3. **`MaterialBindingAPI` must be APPLIED.** `UsdShade.MaterialBindingAPI(prim)
   .Bind(mat)` constructs a NON-applied schema: it writes the relationship but
   never adds the schema, USD warns ("Found material bindings on prim ... but
   MaterialBindingAPI is not applied") and the binding is ignored. Use
   `MaterialBindingAPI.Apply(prim).Bind(mat)`.
4. **Plane-coincident triangles are dropped.** The façade lies EXACTLY on
   `x0`/`x1`/`y0`/`y1`; a triangle lying in a clip plane has the implicit
   function zero at all three vertices and `vtkClipPolyData` discards it —
   losing the outer skin, the one surface that matters. Pad the OUTER limits
   (`OUTER_PAD_M`); only the interior cuts need to be exact. This was **2.74%**
   of every band.
5. **The lattice does not reach the ends of the mesh.** `lattice()` returns
   lines INSIDE `[z0, z1]`, so the shopfront band at the base and the cornice
   at the top fall outside every band and vanish. Close both ends. This was
   **4.03%** of the building.

A sixth, in the harness rather than the code: **an A/B camera placed between
two columns compares one building's front against the other's back.**
`SM_Building_01` is `front:E, blank:N,W,S` — it carries detail on ONE
elevation, so that framing reads as total texture loss and proves nothing.

## Acceptance tests — measure, do not look

A render cannot distinguish "no UVs", "wrong material", "dropped geometry" and
"camera in the wrong place". Three numbers can.

| test | tool | pass |
|---|---|---|
| geometry conserved | `tools/ring_verify.py` | band and ring area delta **0.0000%** — clipping conserves area exactly, so any delta is a gap or an overlap |
| cuts miss the windows | `tools/cut_clearance_verify.py` | clearance > half a window height (~0.55 m); windows crossed < 2% |
| round trip | reassemble at gap 0 beside the untouched original | indistinguishable |

The round trip is the one that proves a slicer. Run all three before looking
at a picture, and frame the picture so both buildings show the SAME elevation.

## Files

| file | role |
|---|---|
| `scene_gen/detail/gac_storey_slice.py` | `read_mesh`, `clip`, `storeys`, `cut_lines`, `ring`, `roof_and_parapet`, `plan_slice_budget`, `choose_bay_budget`, `choose_cut_offset`, `write_piece`, `slice_to_kit` |
| `scene_gen/detail/gac_slice.py` | `storey_period`, `measure_grid`, `window_centres` |
| `scene_gen/detail/kit_bake.py` | `have_kit`/`load_kit` — reference a pre-baked slice instead of re-slicing; `fingerprint()` invalidates on any change to the two files above |
| `scene_gen/tools/bake_gac_kits.py` | writes a baked kit to `scene_gen/assets/kits/<asset>.usd` + `kits.json` (not yet run — see "The piece budget") |
| `scene_gen/tools/pack_structure_probe.py` | is this pack one mesh or a bag of parts? |
| `scene_gen/tools/openings_probe.py` | are the windows modelled (glass subset -> islands)? |
| `scene_gen/tools/gac_uv_probe.py` | what is the UV primvar called? |
| `scene_gen/tools/gac_mat_probe.py` | where does each material live (own file -> reference it) |
| `scene_gen/tools/ring_verify.py`, `cut_clearance_verify.py` | the acceptance tests (`cut_clearance_verify.py` still scores the fixed 0.5 offset only) |
| `scene_gen/tools/_gac_budget_probe.py` | throwaway: actual piece count/timing/per-role breakdown `slice_to_kit` produces on real assets, one at a time |
| `simulation/isaac-sim/launch_scripts/gac_storey_launch_script.py` | one building: original / reassembled / exploded |
| `simulation/isaac-sim/launch_scripts/gac_kit_catalogue_launch_script.py` | every asset, assembled with its kit laid out in front |

## Not done

- Only GAC is validated. downtowncity has a derivable glass subset (36
  islands) and should work; Muyang DownTown has **no** glass subset and is
  excluded from fire entirely (`kit_substitute.UNBURNABLE`).
- Bay pitch on narrow N/S elevations reports exactly 9.00 m — the search
  ceiling saturating, not a measurement. Harmless on short ends.
- **The pieces are still not baked to disk**, even though the path now
  exists (`tools/bake_gac_kits.py`, `kit_bake.load_kit`, above) — every
  launch re-slices. The piece-budget and offset-search fixes changed
  `kit_bake.fingerprint()`, so this was true before them too; baking is
  still the obvious next step, as `archetypes_quake` is for the kit.
- `cut_clearance_verify.py` still hardcodes offset 0.5 (see "The piece
  budget") — it verifies the fixed cut, not the per-asset one `slice_to_kit`
  now actually makes.

## Baking and damaging a GAC building — the pipeline any disaster reuses (2026-08-30)

`build-urban-fire-scenes` built a full per-building bake-then-damage pipeline
on top of this slicer (`disaster/gac_fire.py`, `disaster/fire_bake.py`, the
`fire_bake_launch_script.py` / `fire_assembly_launch_script.py` pair). Almost
none of it is fire-specific — it is a general recipe for "damage one GAC
building, alone, in its own Kit process, export it static, put back whatever
was not baked (an effect, not geometry)". This section names, stage by
stage, what a TORNADO / EARTHQUAKE / HURRICANE damage pass reuses verbatim
and what it must replace, so the next disaster does not re-derive any of it.

### The stage list

1. **Place the merged source** — `gac_fire.place_source(stage, cell, usd,
   scale)`. Disaster-agnostic. References the asset under `<cell>/src`,
   centres it in plan on the cell, sits its base at the cell's own z. Every
   disaster reuses this call verbatim.
2. **Measure** — `gac_slice.window_centres`/`gac_storey_slice.grid_for` (the
   storey grid), `gac_fire.window_rects`/`_islands` (glass faces grouped into
   window islands by grid-hashed union-find, `ISLAND_CELL_M=0.30`),
   `gac_fire.mass_from_grid` + `side_frame` (a `quake_flow`-shaped mass box
   and four synthetic per-elevation wall frames). Disaster-agnostic. None of
   this reads a fire plan — the storey grid, the mass box and the wall frames
   are exactly what `quake_flow.describe`/`_b_face_pt`/`_outward` need to
   place ANY disaster's damage or effect sources on the real façade plane,
   with no measured window table required.
3. **PLAN the damage** — fire: `urban_fire.plan_fire` (origin storey, venting
   sides, severity level) then `soot_plume.plan_events` (per-opening
   flame/smoke event records off the window islands). **Fire-specific
   output, disaster-agnostic shape.** Whatever a new disaster's planner is
   (an earthquake shake field, a tornado track, a hurricane wind field), it
   must produce the same handful of primitives the next two stages consume:
   an origin/top storey pair and the hot elevations for a REGION cut, plus —
   only if the disaster also pre-bakes a texture skin — per-opening or
   per-band severity to drive that skin's RGBA. `gac_fire.prepare`'s own
   elevation ranking (rank sides by window-island count, take as many as the
   level's plan wants, never draw one uniformly at random against a building
   whose glazing is one or two elevations and blank party walls elsewhere)
   is itself disaster-agnostic and worth reusing outright.
   **Localized vs whole-building is a fork here, not later.** Fire damages a
   height band and a set of sides — that is exactly what `region=` (stage 5)
   exists to exploit. An earthquake shakes the WHOLE building at once:
   `quake_sliced.wreck_sliced` calls `quake_flow.describe(style, placements,
   x, y, yaw)` over every placement it is handed, and `quake_flow.describe`
   builds ITS element table off that full set — there is no "band" for a
   region cut to save. So the fork is: pass `region={...}` when the damage
   is confined to a band/sides (fire, and any future disaster shaped like
   it — a lightning strike, a localized blast); pass `region=None` (a full
   slice) when the ladder needs the whole element table (earthquake, and
   anything else that treats the building as one shaking/loading system).
4. **PRE-SLICE texture bake through the merged UVs** — `gac_fire.bake_atlases`.
   **Generic:** the UV position map (`soot_bake.uv_position_map`, run on the
   de-indexed mesh `gac_storey_slice.read_mesh` already produces for the
   slicer) and the SHARED-TEXEL test (`SHARED_TEXEL_M=2.0`,
   `SHARED_FRAC_MAX=0.08` — rasterise the same faces forward and reversed;
   a texel two faces more than 2 m apart in height both cover is shared, and
   an atlas over 8% shared is TILED) that routes a tiled atlas to a per-piece
   bake AFTER the slice instead — both are pure UV/geometry math with no fire
   semantics at all. **Fire-specific:** the skin itself (`soot_plume.skin`,
   the char/soot RGBA field) and `soot_plume.piece_material_like`'s output
   scope name (`<cell>/SootLooks/mN`). A dust-scour, crack-pattern or mud-line
   skin for another disaster plugs in at exactly this seam: call
   `bake_atlases` (or copy its loop) with a different RGBA generator in place
   of `_sample_skin_any_side`/`spl.skin`, and a differently-named Looks scope
   — the position-map call and the shared-texel routing need no change.
5. **`gac_storey_slice.slice_to_kit(stage, src, cell, style, region=)`.**
   Entirely disaster-agnostic — this is what the rest of this skill
   documents. The `region={"origin": storey, "top": storey, "sides": (...)}`
   dict is generic machinery (`_region_hot_sides`, `ring(hot_sides=)`,
   `plan_slice_budget`'s region arithmetic all live in `detail/`, one layer
   below any `disaster/` module, and know nothing about fire), but the
   VALUES a caller puts in it are the disaster's own: fire's `origin` is the
   lowest damaged storey, `top` is the highest storey ANY recipe in that
   level's ladder can reach (not just the fire itself — see the roof trap
   below), `sides` are the venting elevations. `region=None` (or omitted)
   still means "ring the whole building" — today's unconditional cut,
   unchanged for every existing caller.
6. **Rebind** — `gac_fire.rebind_sooted(stage, pls, sooted)`. Generic
   mechanism (match each piece subset's bound material by prim path, then by
   texture URL — a baked kit's rehomed materials share the texture, not the
   path); fire-specific only in that it is fed the `sooted` dict
   `bake_atlases` produced. Skip this stage entirely if a disaster has no
   pre-slice skin to rebind.
7. **The ladder over kit placements.** This is where fire and earthquake
   diverge structurally, and it is worth naming both shapes:
   `urban_fire.burn_building(stage, parent, style, placements, x, y, yaw,
   level, ...)` runs `LADDER[btype][level]` — a plain list of `(recipe_name,
   kwargs)` tuples looked up by construction type and severity level — over
   `_els()`-filtered placements, in order, each recipe a function from
   `RECIPES`; a recipe LIST can replace the level name outright (`gac_fire.
   burn_gac` does this to drop `expose_interior`/`gut_interior` when nothing
   is open for anyone to see into). `quake_sliced.wreck_sliced(stage, cell,
   placements, style, recipes, ...)` is the same shape one level up
   (`LADDER_S`/`RECIPES_S`, `construction_type()` routing by measured
   material+height instead of `info["type"]`). **The one hard constraint any
   new sliced-building ladder must honour:** `quake_sliced`'s own module
   docstring — *"a sliced piece must never be handed to
   `fracture.fracture_prim`"* — VTK segfaults on a clipped shell (three
   crashes on `SM_Building_09`, see `build-urban-fire-scenes`'s crash
   catalogue). That is why `quake_sliced` expresses masonry collapse, soft
   storey, pancake etc. as REMOVAL on the piece grid and RIGID DISPLACEMENT
   of whole pieces rather than fracturing them — architecturally the same
   choice fire's gut/burnout/collapse recipes already made, not a
   coincidence. **As of 2026-08-30 no launch script wires `slice_to_kit` and
   `wreck_sliced` together** — only `tools/_gac_region_probe.py` (the slice
   alone) and `tests/test_quake_sliced.py` (the ladder alone, on synthetic
   placements) exercise the two halves; the bake pipeline below has never
   been run end to end on an earthquake-damaged GAC building.
8. **Settle alone** — `settle.run(stage, loose, static_extra, ...,
   bake_result=True, converge=True)`. Disaster-agnostic; identical call for
   any ladder's `ctx["loose"]`/`ctx["static_extra"]`/`ctx["velocity"]`
   (both `burn_building` and `wreck_sliced` return that same ctx shape by
   construction).
9. **Export** — `fire_bake.strip_physics`, `fire_bake.rehome_for_export`,
   root-layer export, `fire_bake.sidecar`. Disaster-agnostic EXCEPT the
   sidecar's `fire`/`events` payload, which is fire's own schema — see the
   next section for what a non-fire sidecar needs instead.
10. **Assembly** — `fire_assembly_launch_script.py`: referencing the bakes at
    their column x, ground + light, are disaster-agnostic; `place_fire` and
    the Flow emitter placement are fire-specific and would be replaced by
    whatever re-places a non-fire disaster's own non-baked effect (a dust
    plume, standing water, nothing at all if the damage is pure geometry).

### What the sidecar must carry for a non-fire disaster

`fire_bake.sidecar(entry, fire, masses, events, bbox, top_z, seats, notes,
timings, counts, ...)` writes: the fire plan (`origin`/`storeys`/`top`/
`sides`/`n_storeys`/`mass`/`roof`/`level`/`state`/`finish`), the mass boxes
(`mass_to_json` — deliberately lossy, keeping only the parapet-band heights
`soot_plume.parapet_height` sums, so the sidecar cannot drift from
`urban_building`'s style table), the fire EVENTS (`events_to_json` — every
field `urban_fire._flame_sources` reads: `fr`/`out`/`m`/`side` for the
façade-plane placement, `storey`/`e` for `_severity`, `state`/`id`/`ops` for
`r_flames`'s own bookkeeping), the SETTLED `bbox`/`top_z`, and the
interior/roof plume `seats` (world points, already clamped to `top_z` — see
the smoke-height trap below). `fire_assembly_launch_script.place_fire` reads
exactly `doc["fire"]`, `masses`, `events` (via `fb.load_for_assembly`, which
rehydrates the events against the bake's own masses) and `doc["seats"]`.

A disaster with no Flow-equivalent effect needs almost none of this: geometry
alone is enough once it is baked, and the sidecar could carry only enough
provenance to identify the bake (`kind`/`name`/`level`/`seed`) plus `bbox`/
`top_z` for anything an assembly script wants to re-clamp. A disaster that
DOES have a post-bake effect (a dust plume off a collapse, standing water
seeking its level, anything not frozen into the exported mesh) needs the
same three things fire's sidecar carries for its smoke: (1) a damage-plan
dict in place of `fire` — for earthquake this is `quake_sliced.plan_damage`'s
return value (grade, `regions`, `recipes`, `stats`) rather than
`urban_fire.plan_fire`'s dict; write a `plan_to_json`/`plan_from_json` pair
the way `fire_bake.py` writes `mass_to_json`/`events_to_json`, not a reuse of
the fire ones, since the field names differ; (2) the SAME wall-frame 7-tuple
shape (`frame_to_json`/`frame_from_json`) for any opening- or face-relative
effect placement, because that tuple is `quake_flow._piece_frame`'s own
shape and every disaster's openings already come from `quake_flow`; (3) the
settled `bbox`/`top_z` and pre-selected effect `seats`, computed at bake time
while the fit-out is still addressable (exactly why `fire_bake_
launch_script._interior_seats`/`_roof_seats` run BEFORE export rather than
leaving the assembly to guess). `fire_bake.SCHEMA` is versioned (bump it, or
add a disaster-specific sibling constant, rather than overloading `fire`'s
own field names for a different disaster's plan shape).

### The commands

Bake one building (fire, today's working example — the `kind:name:level`
manifest syntax and env plumbing are disaster-agnostic; only `FB_KIND=gac`
routing to `gac_fire.burn_gac` is fire's):

    scene_gen/tools/fire_bake.sh gac:SM_Building_02:F3

Verify a slice offline — no Kit, no GPU, safe beside a live session — before
ever building on top of it:

    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tools/_gac_region_probe.py"

`_gac_region_probe.py` slices one asset FULL and with a `region=` cut side by
side and checks the area is conserved to 1e-4% and the mass box
(`ub.footprint` W/D, `_mass_specs`'s top and level count) is IDENTICAL with
and without the region — this check is disaster-agnostic, it only exercises
`slice_to_kit`, so it validates a future earthquake/tornado region cut with
no change. `tools/gac_burn_probe.py NAME LEVEL` runs the fire chain
(`burn_gac`) end to end on a bare in-memory stage; a new disaster's own probe
would call its `wreck_*`-equivalent the same way. Verify a finished BAKE the
same way, cold:

    scene_gen/tools/fire_bake.sh --verify-only

Assemble (GUI, from the container's tmux pane per `run-isaac-sim-launcher`;
`FA_BAKES` takes a directory OR an explicit comma list of `.usd` paths — use
the explicit list, a directory picks up stale stems too):

    docker exec isaac-sim tmux send-keys -t isaac \
      'ISAAC_SIM_HEADLESS=false FA_BAKES=/isaac-sim/.cache/fire_bakes/gac_SM_Building_02_F3_s7.usd \
       FA_FLOW=1 KEEP_OPEN=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
       /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/fire_assembly_launch_script.py \
       --/rtx/raytracing/fractionalCutoutOpacity=true --/rtx/pathtracing/fractionalCutoutOpacity=true \
       --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER

Each bake's log is `~/docker/isaac-sim/logs/<stem>.log` on the host
(`/isaac-sim/.nvidia-omniverse/logs/<stem>.log` in the container) —
`docker logs isaac-sim` is empty for this container.

### The traps

| trap | fix |
|---|---|
| A sliced/sooted piece binds a material living under `<cell>/src`; drop the source and every such piece renders WHITE with geometry and UVs intact | `fire_bake.rehome_for_export` re-anchors each dependent material onto its OWN Nucleus file before `<cell>/src` is dropped; if any material cannot be rehomed, KEEP the source (invisible) and record `src_kept: true` — never ship a white building |
| `stage.Flatten()` / `stage.Export()` — Kit meshes/subsets/materials carry an `assetInfo` dict core USD cannot unpack | export the **root layer only** (`stage.GetRootLayer().Export(path)`) |
| `settle.run(bake_result=True)` disables `rigidBodyEnabled` but leaves every `Physics*`/`Physx*` schema and ~22 `physics:`/`physx*` attributes per body applied | `fire_bake.strip_physics` after the settle, before export |
| `gac_storey_slice.slice_lock` is a MACHINE-WIDE `flock` — a live slice in the GUI tmux pane blocks a `fire_bake.sh` run and vice versa, and one GPU cannot hold two Kit processes anyway | never run a per-building bake driver concurrently with a live Kit build; `fire_bake.sh` is sequential on purpose, no `&` |
| `apply_placements` turns its target prim into a `Scope` (not Xformable) — building a kit style straight into the cell silently drops the cell's own transform | build into a child (`cell + "/parts"`), as `fire_bake_launch_script.build_kit` does |
| `FA_BAKES` as a directory accumulates stale stems across re-bakes at a different seed | pass the explicit comma list `fire_bake.sh` prints at the end of a successful row |
| A Flow emitter (or any effect authored under a fixed root like `/World/flow/emitters`) does not inherit a bake's column translate | `fire_bake.translate` moves wall-frame origins and mass centres by the column offset before the effect is placed — never `e["x"]/["y"]`, which `_el_jitter` hashes for each module's stable per-piece wobble |
| A plume/effect seated at a PLANNED height (`m["top"]`, `m["levels"][storey]`) hangs in clear air once a collapse has dropped the real geometry below it | re-clamp against the SETTLED bbox at bake time (`_interior_seats`/`_roof_seats` clamp to `top_z`) and again at assembly time against the referenced geometry's own measured bbox |
| `region["top"]` caps the ring from above; a recipe that needs the real roof regardless (`roof_burnthrough`, `fire_collapse`) finds no `role="roof"` piece if `top` was capped below it | look ahead at every recipe NAME the level's ladder contains before calling `slice_to_kit` (`gac_fire.burn_gac`'s `roof_needed` check) and force `top` to the real roof whenever one of them needs it |

### To add disaster X — a checklist

- Write a planner producing `{origin, top, sides}` for a localized disaster
  (or decide your damage is whole-building and pass `region=None`), plus
  whatever per-opening/per-band severity a pre-slice skin needs. Reuse
  `gac_fire.prepare`'s elevation-by-window-island ranking if your damage
  vents through openings.
- If you need a pre-slice texture skin, write only the RGBA generator and the
  Looks-scope name — feed both through `gac_fire.bake_atlases`'s existing UV
  position map and shared-texel routing rather than re-deriving either.
- Call `slice_to_kit(..., region=...)` with your planner's origin/top/sides
  (or `region=None`).
- Rebind sooted/skinned pieces (`gac_fire.rebind_sooted`'s pattern) only if
  you baked a skin.
- Write your `LADDER`/`RECIPES` as `(name, kwargs)` tuples over `_els()`-
  filtered placements (`quake_sliced.LADDER_S`/`RECIPES_S` is the shape to
  copy). Never hand a sliced piece to `fracture.fracture_prim` — express
  damage as removal and rigid displacement instead.
- Settle with `settle.run(..., bake_result=True, converge=True)`, strip
  physics with `fire_bake.strip_physics`, rehome any skin materials with
  `fire_bake.rehome_for_export` before dropping `<cell>/src`, export the
  root layer only.
- Write a sidecar carrying your own damage-plan dict (not `fire`), the mass
  boxes, `bbox`/`top_z`, and any post-bake effect seats — bump
  `fire_bake.SCHEMA` or add a sibling schema rather than overloading fire's
  field names.
- Write (or extend) an assembly script that references the bakes at their
  column x, re-derives the column offset for anything not baked into
  geometry, and re-clamps any height-seated effect against the referenced
  geometry's own measured bbox.
- Verify offline first: an area-conservation + mass-box-identity check like
  `_gac_region_probe.py` for the slice, then a bare-stage probe like
  `gac_burn_probe.py` for your ladder, before ever touching Kit.

### Painted windows are windows (2026-08-30)

10 of the 12 GreatAmericanCity towers (`SM_Building_10/12/13/18/21/22/23/25/
27/28`) carry NO glass mesh: their glazing bands are opaque faces bound to
`M_Fake_Interior_0N`, `M_Fake_Light`, `M_Images_*_Off_Light` — background-LOD
decals. `gac_slice.GLASS_TEX` matched only "glass/window/curtain/glazing/win",
so `window_centres` measured no storey grid from them and `gac_fire.window_rects`
found no openings: no fire was ever planned on a tower (probe sweep: 0 events
on all ten). `GLASS_TEX` now includes `fake_interior`, `fake_light`,
`off_light`, and `gac_slice.is_glazing(name)` is the one matcher every
consumer uses — it also rejects `GLASS_TEX_NOT = ("awning",)`, because "win"
is in "awning" and ground-floor canopies were being counted as windows and
blacked out on the band. Measured after: SM_Building_12 544 islands / 23
events, SM_Building_18 480 / 13, SM_Building_13 230 / 11, SM_Building_23
489 / 23; SM_Building_10 still 4 islands (a different window material —
not yet classified). Also in `window_rects`: a pane is filed on the
elevation its CENTROID is nearest (nearest bbox face), never by its normal —
glass is double-sided and the back face put a phantom mirror of every window
on the opposite elevation (SM_Building_02: 72 islands → 18 real), which is
what planned fires and flames on blank walls.

### Two more pipeline facts (2026-08-30, late)

**A sliced building is typed by its construction, not by the kit family.**
`gac_slice.register_style` hardcoded `family "01"`, so `quake_flow.describe`
typed every sliced building `urm` whatever `gac_fire.prepare`'s `btype` said:
the rc LADDER ran with masonry piers, timber slabs and residential furniture
and never the steel frame. `register_style(..., family=)` /
`slice_to_kit(..., family=)` now take the family; `burn_gac` maps
`btype → family` (`urm 01`, `rc 02`, `rc_glass 05`). Measured after:
SM_Building_05 F5c types `rc`, 36 steel beams, 343 columns steel, 0 piers.

**Downtown City goes through the same chain** (`gac_fire.PACKS` registry,
`dtc:NAME` names, `fire_bake.sh dtc:NAME:LEVEL`): assets are single merged
meshes with real glass on all four sides, INLINE materials (no
`Materials/*_Inst.usd` — `gac_slice._material_source` falls back to the
strongest non-anonymous layer, `fire_bake.rehome_for_export` verified with a
negative control), `metersPerUnit` measured (1.0 vs GAC's 0.01), construction
from `quake_sliced.CONSTRUCTION` (Amar_Tower `rc_glass`), glazing also by
MATERIAL NAME (`Glass_window` carries no map). Catalogue:
`scene_gen/_plans/dtc_buildings.json` (`tools/dtc_catalogue.py`); grids:
`tools/dtc_grid_probe.py` (10 of 15 fall back to the regular grid — sane
pieces); export: `tools/dtc_export_probe.py`. Row picks: `dtc:Building_11:F1
dtc:Carved_02:F2 dtc:Building_12:F3 dtc:Carved_14:F4 dtc:Carved_13:F5
dtc:Carved_18:F3` (+ two GAC F5c). Amar_Tower: roof-garden trees inflate the
bbox (trimmed via `PACKS["dtc"]["bbox_exclude"]`), 17 pieces at F3, ~50 min —
use F5c (53 pieces, curtain-wall stripe) if the silhouette is wanted. Build
time is `bake_atlases` (29-78 textures, seam-straddling UVs) — the tiled test
now runs before the full-res rasterisation.
