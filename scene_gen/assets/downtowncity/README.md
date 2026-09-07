# DowntownCity — buildings extracted from the SEI-COA Nucleus pack

Fifteen building USDs converted from `Projects/SEI-COA/DowntownCity` on
Nucleus, which ships as a **joined DCC export** and not as USD. Referenced by
[`config/asset_packs/downtown_city.yaml`](../../config/asset_packs/downtown_city.yaml).

Gitignored like every other asset directory here: 695 MB, and reproducible from
the source pack by the commands at the bottom.

## Three named buildings, and twelve that were hidden

The pack exports one file per *category*, each holding a whole district joined
together. Splitting `Grid System.fbx` on the artist's object names gives only
three buildings — `Building 1` through `Building 10` are objects with **zero
vertices in the file itself**, which is not an importer failing: reading the
binary FBX directly shows `Building 1 Geometry` … `Building 10 Geometry`
declared with `verts=0, polyIdx=0`. `Towers.gltf` is the same story:
`Empire State Tower`, `One World Trade Center Tower`, `Norman Foster Tower` and
`The Shard Tower` are all empty, and only `Amar Tower` carries geometry.
The copy on Nucleus holds exactly this — nothing was lost in the download.

**The geometry was not missing, it was merged.** The mesh named
`Path and Imperfections` is 3.2M triangles, carries 104 materials and is
**30.6 m tall** — a path is not 30 m tall. Rasterising it into a top-down
height grid and labelling the connected regions standing more than 8 m above
its own floor finds twelve building-sized masses laid out on a block grid,
which is what "Grid System" was always the name of. `--carve` separates the
faces by region and each one comes out a complete, textured perimeter block:
shopfronts, mansard roofs, courtyards, rooftop plant.

| asset | size (m) | tris | what it is |
|---|---|---|---|
| `Amar_Tower` | 42.3 x 48.8 x **231.4** | 502k | Curtain-wall tower, dark glazing in a stone grid, rooftop planting. Nearly twice the previous ceiling (BG_Building_E, 131 m). |
| `Carved_13` | 83.9 x 44.2 x 30.6 | 432k | Largest block; mansard corner pavilion. |
| `Carved_18` | 44.4 x 44.4 x 29.2 | 263k | Courtyard block. |
| `Carved_01` | 42.8 x 44.0 x 28.4 | 182k | Mansard roof over a corner. |
| `Carved_03` | 42.6 x 44.4 x 27.6 | 150k | Mansard; forecourt at the base. |
| `Carved_14` | 84.0 x 39.5 x 25.4 | 263k | Long block with a mansard wing. |
| `Carved_02` | 43.2 x 44.3 x 22.0 | 283k | Courtyard block. |
| `Carved_21` | 37.4 x 43.5 x 21.0 | 207k | U-plan around a court. |
| `Carved_15` | 83.9 x 44.4 x 19.0 | 278k | L-plan with a long low wing. |
| `Carved_06` | 39.9 x 44.4 x 19.0 | 209k | Stepped plan. |
| `Carved_05` | 44.4 x 44.3 x 18.6 | 145k | |
| `Carved_17` | 44.0 x 24.1 x 17.0 | 93k | Narrow street-wall infill. |
| `Carved_04` | 34.1 x 39.1 x 16.0 | 130k | Smallest. |
| `Building_12` | 44.3 x 18.7 x 38.9 | 87k | Narrow block, stone pilaster order. |
| `Building_11` | 30.9 x 35.4 x 32.6 | 222k | Mansard, dormers, AC units, shopfronts. |

Fourteen of the fifteen place in a stock 500 m urban scene, together supplying
**35% of all buildings** over three seeds. Amar Tower does not, and the asset
pack records exactly why and what override fixes it — the packer filters on
footprint area, and a slender supertall loses to a fat one.

Not converted, deliberately: `Grass` (3.9M tris), `Building Base`, `Fences`,
`Sidewalk`, `Road`, `Parking`, `Green Area` (empty). They are the district's own
ground and layout, and the scene generator lays its own. `Building 13` has
geometry but is 238 x 146 x **1.3 m** — paving slivers mislabelled a building.

## The trap this pack sprang

`Building 12` measured **140.2 x 75.0 m** on the first pass and is really
44.3 x 18.7. The difference was two vertices belonging to no face, 135 m away.
They render as nothing, so a preview looks perfect — and the generator sizes
and places every asset by its measured bounding box, so the building would have
landed off-centre inside a footprint three times too big, for reasons no amount
of looking at it would reveal. `extract_objects_to_usd.py` now deletes
face-less geometry before measuring (8,140 verts across two buildings) and
measures from the vertices rather than from Blender's cached `bound_box`, which
does not refresh after that edit.

## Rebuilding

```bash
# mirror the pack off Nucleus, unzip it (city.zip is inside DowntownCity.zip), then:
P=<pack>/City_Exports
V=scenegen/.venv/bin/python
X=scene_gen/tools/extract_objects_to_usd.py
D=scene_gen/assets/downtowncity

$V $X "$P/building System_unpacked_joined_FBX/Grid System.fbx" -o $D \
      --match '^Building 1[12]$' --min-tris 1000
$V $X "$P/Towers_unpacked_joined_GLTF/Towers.gltf" -o $D --match 'Amar'
$V $X "$P/Building_unpacked_joined_GLTF/Grid System.gltf" -o $D \
      --carve "Path and Imperfections"
```

`--list` on any of them prints the inventory without exporting — that is how
the empty objects were found.

Output is Z-up, metre-authored, `defaultPrim = /root`, re-centred so the
footprint centre is at the origin and the base at z = 0. So the asset pack
needs no `scale` and no `axis-up`, and the generator measures `base = 0.00` on
all fifteen.
