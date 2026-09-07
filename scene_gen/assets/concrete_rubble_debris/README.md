# concrete_rubble_debris — the FAB "Concrete Elements" pack, split

`concrete_elements_asset_pack.usdz` (233 MB) as the user dropped it in on
2026-08-29, and `split/` — the ten props taken out of it, one USD each.
Gitignored except this file; rebuild with the single command below.

Every USD is **metres, Z-up, footprint centred on the origin, base at z = 0**,
in its own folder with its own `textures/`. The source is Y-up centimetres and
that conversion is baked in, so the pack entries carry no `scale:` or
`axis-up:`.

## The split is by MATERIAL, not by object

The pack's exporter (`obj_cleaner_materialmerger_gles`) chops meshes at the
16-bit index limit, so a single prop arrives as `Object_2` (65,532 verts) plus
`Object_3` (the remainder) — same material, and one bbox **nested inside** the
other rather than beside it. Splitting on object names would have produced
half-props; three of these ten had to be reassembled that way. The material
assignment is the only place the artist's split survived, which is what
`--group-by-material` (added to the extractor for this pack) groups on.

Also dropped, by `--min-tris 100`: the pack's 14.7 m backdrop plane
(`GRID_002`, 2 triangles), plus two empty Xforms the importer skips.

| asset | size (m) | tris | what |
|---|---|---|---|
| `huge_concrete_rubble_pile` | 8.05 x 7.68 x 1.48 | 71k | the one true heap of the set |
| `brick_debris_pile` | 6.07 x 4.90 x 1.20 | 58k | brick rubble spread |
| `brick_debris_pile_hp` | 6.07 x 4.93 x 1.20 | 189k | ditto, high poly |
| `concrete_slabs` | 3.64 x 2.46 x 0.67 | 58k | broken slabs, stacked |
| `concrete_debris_elements` | 3.53 x 2.58 x 0.38 | 97k | low spread of mixed fragments |
| `concrete_debris_elements_hp` | 3.54 x 2.58 x 0.38 | 125k | ditto, high poly |
| `concrete_sidewalk_elements` | 1.72 x 1.77 x 0.34 | 29k | kerb and sidewalk fragments |
| `cracked_paving_slabs` | 1.08 x 1.60 x 0.31 | 28k | cracked paving |
| `lamppost_block` | 0.16 x 0.35 x 0.74 | 21k | lamppost base block |
| `lamppost_block_v2` | 0.16 x 0.35 x 0.74 | 21k | same block, second texture variant |

`_hp` is the same prop at higher poly, not a different one.

## Where they are referenced

All ten are in `config/asset_packs/urban_v3.yaml` under `usds.debris.piles`,
every one `material: concrete` (user's call — the two brick piles and the five
paving/street elements are filed there too; the yaml comment says so).

Paths are written **relative, i.e. against `asset_root` on Nucleus**, and
mirror this local tree exactly: `Projects/SEI-COA/scene_gen/assets/…` on the
server is the same layout as `scene_gen/assets/…` here. Uploaded by the user
2026-08-29 and verified: all ten `.usdc` and their twenty textures are
byte-identical to the local copies.

`assets/.measurements.json` has been seeded for all ten under their Nucleus
URLs, measured off the byte-identical local copies — otherwise a host-side
build cannot open `omniverse://` and would silently pack them at
`fallback_sizes`.

## Rebuilding

```bash
cd scene_gen
uv run --script tools/extract_objects_to_usd.py \
    assets/concrete_rubble_debris/concrete_elements_asset_pack.usdz \
    -o assets/concrete_rubble_debris/split \
    --group-by-material --min-tris 100 --per-asset-dir
```

The extractor names each asset after its material, so the output arrives as
`Concrete_Debbris_Elements/` etc. (the `Debbris` typo is the source's). They
were renamed by hand to the lowercase slugs in the table above; the USDs
reference their textures relatively and their default prim is `/root`, so
renaming the folder and the `.usdc` together is safe.
