---
name: slice-buildings-into-kits
description: Cut a MERGED whole-building asset (GreatAmericanCity, downtowncity, AEC brownstone) into kit-bash pieces — roof, per-storey corner and wall runs — so the urban-fire/earthquake ladders can damage it. Covers measuring the storey/bay grid from the asset's own windows, cutting BETWEEN window rows rather than through them, plane clipping with VTK while carrying UVs and materials inside the polydata, the exact 3x3 ring partition, the five silent bugs that each render as a flat brown box, and the three numeric acceptance tests that catch them.
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

On `SM_Building_01`: 15 bands -> **1,155 pieces** (360 wall, 720 pier, 60
corner, 15 core).

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
| `scene_gen/detail/gac_storey_slice.py` | `read_mesh`, `clip`, `storeys`, `cut_lines`, `ring`, `write_piece` |
| `scene_gen/detail/gac_slice.py` | `storey_period`, `measure_grid`, `window_centres` |
| `scene_gen/tools/pack_structure_probe.py` | is this pack one mesh or a bag of parts? |
| `scene_gen/tools/openings_probe.py` | are the windows modelled (glass subset -> islands)? |
| `scene_gen/tools/gac_uv_probe.py` | what is the UV primvar called? |
| `scene_gen/tools/gac_mat_probe.py` | where does each material live (own file -> reference it) |
| `scene_gen/tools/ring_verify.py`, `cut_clearance_verify.py` | the acceptance tests |
| `simulation/isaac-sim/launch_scripts/gac_storey_launch_script.py` | one building: original / reassembled / exploded |
| `simulation/isaac-sim/launch_scripts/gac_kit_catalogue_launch_script.py` | every asset, assembled with its kit laid out in front |

## Not done

- Only GAC is validated. downtowncity has a derivable glass subset (36
  islands) and should work; Muyang DownTown has **no** glass subset and is
  excluded from fire entirely (`kit_substitute.UNBURNABLE`).
- Bay pitch on narrow N/S elevations reports exactly 9.00 m — the search
  ceiling saturating, not a measurement. Harmless on short ends.
- The pieces are not yet baked to disk; every launch re-slices (~10 s a
  building). A bake is the obvious next step, as `archetypes_quake` is for the
  kit.
