# NVIDIA AEC demo pack assets (git-ignored)

Everything in this directory except this file is git-ignored vendor content —
gigabytes, and re-extractable from the downloaded zips. `urban_v2.yaml`
references it through the `airstack://` pseudo-scheme, which resolves to the
repo root on both the host and inside the sim container.

## What is needed, and why both packs

`urban_v2` draws on **two** packs. Extracting only one is the failure mode this
section exists to prevent, because neither missing pack produces an error:

| directory | from | supplies |
|---|---|---|
| `brownstone/` | `AECDemo_NVD@10012.zip` | the party-wall terraces, and the street props |
| `tower/` | `AECO_TowerDemoPack_NVD@10012.zip` | the **park vegetation** (4 of the 5 tree entries) and `Bollard_01` |

> **`Materials/` is not optional, in either pack.** The USDs bind their MDL
> materials by relative path — `../../Materials/Base/Masonry/Concrete_Block.mdl`
> and twenty more — so a pack extracted without it loads the geometry and none
> of the surfacing. In Isaac the buildings then render as untextured flat
> colour, which reads as a generator bug and is not one.

Extracting `tower/` without it is the same story for the trees: the parks come
out sparse because most of the pool simply does not load.

**Check both after extracting** — this is what a half-extracted pack looks
like, and `prepare_assets` now reports it:

```bash
ls scene_gen/assets/aec/brownstone/   # Assets Materials Options Props
ls scene_gen/assets/aec/tower/        # Assets Materials
python3 scene_gen/prepare_assets.py urban_v2 --list   # warns on anything missing
```

## Re-extracting

From the NVIDIA AEC demo packs (free to use, downloaded separately). Note the
two zips nest differently — the tower pack has an extra `TowerDemopack/` level:

```bash
cd ~/Downloads

# 1. Brownstone — 2.3 GB extracted
unzip -q -o "AECDemo_NVD@10012.zip" \
  'Demos/AEC/BrownstoneDemo/Props/*' \
  'Demos/AEC/BrownstoneDemo/Options/*' \
  'Demos/AEC/BrownstoneDemo/Assets/*' \
  'Demos/AEC/BrownstoneDemo/Materials/*' \
  -d /tmp/aec_stage
mkdir -p <repo>/scene_gen/assets/aec/brownstone
mv /tmp/aec_stage/Demos/AEC/BrownstoneDemo/{Props,Options,Assets,Materials} \
   <repo>/scene_gen/assets/aec/brownstone/

# 2. Tower — 4.5 GB extracted. Only Assets/ and Materials/ are wanted;
#    Assemblies/ and Source/ are the assembled demo scene.
unzip -q -o "AECO_TowerDemoPack_NVD@10012.zip" \
  'Demos/AEC/TowerDemo/TowerDemopack/Assets/*' \
  'Demos/AEC/TowerDemo/TowerDemopack/Materials/*' \
  -d /tmp/aec_stage
mkdir -p <repo>/scene_gen/assets/aec/tower
mv /tmp/aec_stage/Demos/AEC/TowerDemo/TowerDemopack/{Assets,Materials} \
   <repo>/scene_gen/assets/aec/tower/

rm -rf /tmp/aec_stage
```

Both `mv`s are renames when `/tmp` is on the same filesystem as the repo, so
the staging directory costs no extra disk. Budget ~7 GB either way.

**`brownstone/` mirrors `Demos/AEC/BrownstoneDemo/` exactly — keep it that
way.** Every reference in the pack is relative and crosses directories:
`Props/PlanterFence` payloads `../../Assets/Vegetation/Trees/*.usd`, and the
row terraces reference `../../Options/Options_Grass_Brownstone02.usd`. A
renamed or flattened directory does not error — the reference silently resolves
to nothing and the asset loads without that part of itself. Re-measure after
any move: a missing payload shows up only as a point count that dropped.

The `World_BrownstoneDemopack_*.usd` scene files (8–20 GB) are deliberately not
extracted. They are assembled demo scenes, not a prop library.

## Measured properties

All of it is **centimetre-authored (`metersPerUnit` 0.01) and Z-up**, so every
asset-pack entry needs `scale: 0.01` and no `axis-up` override. Regenerate this
with `scene_gen/tools/measure_assets.py`.

Props are cheap — 15 assets, 196k points total, 61k worst case (the café set) —
so they are affordable at city density.

**Vegetation is not.** 24 assets, 4.6M points, 193k mean, and the worst offender
is a 1.6 m *shrub* at 820k points. `urban_v2.yaml` therefore uses only the cheap
subset as street trees:

| Asset | Points | Size (m) | Use |
|---|---|---|---|
| `Scarlet_Oak_fall` | 40k | 10.4 × 10.3 × 12.8 | primary street tree |
| `Douglas_Fir` | 56k | 3.0 × 2.8 × 6.0 | accent (conifer) |
| `White_Ash` | 66k | 7.6 × 7.6 × 9.6 | primary street tree |
| `Service_Berry` | 84k | 8.1 × 8.5 × 9.2 | primary street tree |
| `Honey_Locust` | 332k | 6.8 × 7.0 × 8.2 | sparing accent — classic NYC species but 5–8× the cost |
| `Hawthorn`, `Acacia`, `Forsythia`, `Yew` | 324k–820k | — | **unused**, too expensive to instance |

### Tower pack vegetation — what the parks are planted with

19 assets, 3.4M points. `urban_v2` draws its park trees from the cheap end;
the expensive entries are ground cover measured in *centimetres* that costs
more than an oak.

| Asset | Points | Size (m) | Use |
|---|---|---|---|
| `Scarlet_Oak` | 40k | 6.3 × 4.5 × 11.1 | park tree |
| `Black_Oak` | 49k | 20.6 × 19.8 × 17.1 | park tree — a wide canopy, needs the room |
| `Shumard_Oak` | 55k | 6.3 × 5.6 × 9.2 | park tree |
| `Common_Apple` | — | — | park tree |
| `Fountain_Grass_Short` | **174k** | 2.0 × 2.1 × 1.5 | ground cover at 3× an oak — sparingly |
| `Meadowlark`, `Buddha_Belly_Bamboo`, `Dogwood` | 83k–184k | 0.9–3.4 | unused: shrub-scale, tree-scale cost |

The park pass places ~313 trees on an 800 m `urban_v2` region, so the pool
choice is worth ~15M points against a scene that has OOM-killed at 89M. Prefer
the sub-60k entries.

### Buildings

`Assets/Create_Brownstone02/` holds pre-assembled party-wall terraces — the
cheapest useful building geometry in either pack, and the right *shape*: long
along the street, ~21 m deep, so a row of them tiles a block frontage the way
a real one does. Cost scales linearly at ~4.6k points per house.

| Asset | Size (m), x = depth | Points | Use |
|---|---|---|---|
| `Reference_Brownstone02` | 21.1 × 6.8 × 14.2 | 4.6k | single house — gap filler |
| `Reference_Brownstone2Row` | 21.1 × 13.4 × 14.6 | 9.3k | short frontage |
| `Reference_Brownstone5Row` | 21.1 × 33.4 × 14.8 | 23k | |
| `Reference_Brownstone6Row` | 21.1 × 40.0 × 14.9 | 28k | |
| `Reference_Brownstone8Row` | 21.1 × 53.3 × 14.9 | 37k | |
| `Reference_Brownstone10Row` | 21.1 × 66.6 × 15.2 | 46k | |
| `Reference_Brownstone11Row` | 21.1 × 73.4 × 15.2 | 51k | |
| `Reference_Brownstone12Row` | 21.1 × 80.1 × 15.4 | 56k | full block face |

Held back, and why:

| Asset | Points | Reason |
|---|---|---|
| `Brownstone02_Instanced` | 71k | same silhouette as `Reference_Brownstone02` at 15× the cost — it carries the interior |
| `Revit_Brownstone0{2,3}_Exterior` | 58k / 73k | **no `defaultPrim`**. `apply_placements` references a layer without naming a prim, which needs one; these compose nothing |
| `Revit_Brownstone01_Exterior` | 1.15M | as above, and 1.15M points for one house |
| `Brownstone0{1,2,3}.usd` | 0.4M–1.5M | full assemblies (interiors, chandeliers, fences) |
| `Max_BrownstoneSite_Buildings{Massing,Detailed}` | 2.7k / 103k | one 356 × 276 m mesh of a whole site, not per-building. The `BldgMassing_*` prototypes inside it *are* individually usable (6–8 m footprints, 10–17 m tall, 8–162 points) but carry no material binding — white boxes |
| `Trees_BrownstoneSite` | 48.3M | the site's whole tree layer in one file |

The `AECO_CityDemoPack` / `AECO_CityTowerDemoPack` city (`ce_Context_City_Large_Bldg`)
is CityEngine white-box massing — a median of ~20 points per building. Useful
as a source of skyline *statistics*, not as geometry.
