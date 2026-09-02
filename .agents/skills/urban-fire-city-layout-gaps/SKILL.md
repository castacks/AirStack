---
name: urban-fire-city-layout-gaps
description: >-
  OPEN, UNRESOLVED as of 2026-09-02: the urban-fire city layout ships far
  fewer occupied blocks than its own zoning/typology mix implies (measured
  ~70% of the full 1.5 km plate's blocks and ~54% of a 1 km crop window's
  blocks carry zero buildings), and `GT_hints.json`'s own `region_m` metadata
  reports the UNCROPPED 1.5 km bounding box rather than the shipped 1 km
  window — exact repro data, the precise numbers, what is ALREADY proven
  correct (the `record_xy` frame fix — see `test_city_layout_audit.py`,
  unaffected by this), and what is NOT yet known (whether the empty blocks
  are a real generator gap or an artifact of how blocks are being counted).
  Read before touching `build_city`/`city_layout`/`districts` density code,
  or before trusting a `downtown_fire_*` cell's occupied-block count.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Urban fire city: empty blocks and the 1.5 km/1 km region mismatch

**Status 2026-09-02: OPEN. `final_disaster_dataset/Fire/Urban` was deleted
from both Nucleus and local disk pending this investigation — do NOT rebuild
until told to.** This skill exists so whoever picks this up (a different
session/agent, per the user's own framing) has the exact numbers and repro
steps without re-deriving them, and so a future rebuild doesn't get flagged
as broken again for the SAME already-understood reason.

## What triggered this

A user visual review of `Fire/Urban/level_1`'s `overview.png` (a top-down
snap) found large stretches of bare road grid with no buildings on one side
of frame. This is NOT the 2026-09-01 crop-shift/frame incident (`.agents/
skills/reassemble-buildings-in-frame`, `test_city_layout_audit.py`) — that
bug displaced EVERY record by the window-centre offset and is verified fixed
(see "What is already proven correct" below). This is a different, separate
finding: even where buildings ARE correctly framed, most blocks have none.

## The numbers (measured against the 2026-09-02 hash-seed-pinned L1 rebuild)

Source files (paths as they exist on `airstack-mission-1gpu-52`'s pod, under
`/root/AirStack/scene_gen/_plans/`, before deletion):
- `city_placements_downtown_fire_1500_hs0.json` — the FULL 1.5 km x 1.5 km
  generated plate, uncropped. `region_m: [1500.0, 1500.0]`.
- `city_placements_downtown_fire_1500_hs0_crop.json` — the 1 km x 1 km
  cropped window actually used for the shipped cell. `region_m: [1000.0,
  1000.0]`, `crop.window: [-680, -320, 320, 680]`, `crop.shift: [180, -180]`.

**Full uncropped plate**: 1022 total house placements, but only **69
distinct blocks** carry any of them (`re.match(r'.*house_(\d+)_\d+',
p["cell"])` grouped and counted — see the one-liner below). At this city's
own measured block density (~9,434 m² per block, derived from the crop's own
`blocks_kept: 49` + `blocks_dropped: 57` = 106 blocks over a 1,000,000 m²
window), a 2,250,000 m² plate should have on the order of ~240 blocks —
meaning **roughly 70% of ALL blocks in the whole generated city are empty**.

**The 1 km crop window used for level_1**: `blocks_kept: 49`,
`blocks_dropped: 57` — **54% of blocks within the shipped window are
empty**, `houses_kept: 320` of `houses_dropped: 702` from the source plate's
1022 (the crop keeps ~31% of the plate's houses over ~44% of its area, i.e.
the kept window is LOWER density than the plate's own average, not higher).

Reproduce the block count yourself:
```python
import json, re
d = json.load(open("city_placements_downtown_fire_1500_hs0.json"))
blocks = {re.match(r".*house_(\d+)_\d+", p["cell"]).group(1)
          for p in d["placements"] if re.match(r".*house_(\d+)_\d+", p["cell"])}
print(len(blocks), "distinct blocks with >=1 house, of", len(d["placements"]), "houses")
```

## Why this looks like it should NOT happen, per the preset's own design

`scene_gen/config/presets/downtown_fire_1500.yaml`'s `layout.anisotropic.
districts.rings` assigns a building-typology MIX to every one of its five
rings (core/inner/mid/outer/edge), each summing close to 1.0 coverage —
`core: {highrise: 0.85, tower: 0.15}` ... `edge: {midrise: 0.15, lowrise:
0.25, rowhouse: 0.60}`. There is no "leave this fraction of ground
unbuilt" term in any ring's mix. On paper, essentially every zoned block
should get SOME typology, not ~30% of them.

## What is ALREADY proven correct — do not re-diagnose this part

`scene_gen/tests/test_city_layout_audit.py` (25 passed, 4 skipped when the
shipped `GT_hints.json` fixtures are absent, as they are now post-deletion)
already gates, at the CODE level (AST-extracted from `urban_fire_city_
launch_script.py`, never imported — it starts Kit at module scope):
1. `record_xy` (x_orig wins, else x/y) is the SAME rule in the launcher and
   the audit tool.
2. `load_fire` shifts cropped-manifest records back to the full-city frame
   correctly without `FC_CROP_WINDOW` set.
3. `compose_bakes` reads a bake holder's position through `record_xy`,
   never `rec.get("x")`, and refuses a bake whose holder sits more than
   `CELL_MATCH_TOL_M` from its cell.
4. Synthetic-layout audit geometry (footprint rects, block fractions, road
   overlap, empty-block detection) is correct on hand-built cases.

These are unaffected by the empty-block finding above — they test the FRAME
fix (2026-09-01 incident), not building DENSITY. Don't waste a cycle
re-verifying `record_xy` consistency; it's solid.

## What is genuinely open

1. **Is 69/~240 (or 49/106 in-window) blocks-with-buildings the actual
   output of `build_city`/`city_layout`/`districts` for this preset, or an
   artifact of how "block" is being counted here** (e.g. do multiple zoning
   cells merge into one traversable block, such that many `city_placements`
   "blocks" by `cell` naming are sub-divisions of a bigger built block that
   IS in fact occupied)? This needs an `urban-layout`/`generate-urban-city`
   -literate read of `build_city`'s own block enumeration, not more manifest
   grepping — the manifest only records where a HOUSE landed, not the full
   block lattice `build_city` itself produced (roads + zoning), which is
   the actual denominator this ratio needs.
2. **`GT_hints.json`'s `meta.region_m` reports `[-750, -750, 750, 750]`** —
   a 1500 m bounding box — for a cell whose own `build_stats.json` records
   `dump_path` pointing at the CROPPED (1000 m) dump and `n_houses: 320`
   (the cropped count, not the plate's 1022). This is an internal
   inconsistency in what gets stamped into `GT_hints.json`'s metadata
   header — whichever code writes `meta.region_m` for the frozen cell
   appears to hardcode/copy the preset's `region_m` (1500, matching
   `SCENE_CONFIG=downtown_fire_1500`'s name) rather than the actual post-
   crop window size. Needs the code path that assembles `GT_hints.json`'s
   `meta` block (likely in `urban_fire_city_launch_script.py` or a shared
   GT-writing helper) checked against whether it reads `crop.window` /
   `region_m` from the CROP file or the ORIGINAL preset config.
   **Unresolved: whether this is metadata-only (the actual composed
   geometry IS correctly cropped to 1 km, just mis-labeled) or whether it
   indicates the full 1.5 km plate's geometry actually got composed into
   the shipped USD** (which would be a much bigger problem — the earlier
   session-long investigation into `d0_department_store_F3` never settled
   this either way; that specific file turned out to be a transient/
   superseded capture artifact unrelated to this question, not evidence
   either for or against it).
3. **`test_shipped_gt_shows_the_displacement`** (same test file, skipped
   without real `GT_hints.json` fixtures) asserts that FIRE-SUBSTITUTE
   buildings (prim paths under `/World/fire/`) DO cross a block edge when
   checked against the shipped scene's own bboxes, while intact buildings
   do not (`a_int["offenders"] == []`, `a_sub["offenders"] != []`). Whether
   this is EXPECTED (a damaged building's debris "apron" — see
   `build_stats.json`'s `aprons: {n_buildings, n_lumps}` — legitimately
   extends past the original footprint into the street, which is realistic
   collapse behavior, not a bug) or an actual leftover displacement is
   NOT determined here. Check `aprons` geometry specifically against
   `a_sub`'s reported offenders before concluding either way.

## Next steps for whoever picks this up

- Get a copy of `city_placements_downtown_fire_1500_hs0.json` (uncropped)
  and read `build_city`'s own returned block/zoning structure directly
  (not just `placements`) to get the TRUE total block count, not an
  estimate from block density. `scene_gen/tools/fire_city_dry_run.py`'s own
  "typology tally" (referenced in the preset's header comment) may already
  report this — run it before writing new introspection code.
- Grep for where `GT_hints.json`'s `meta` dict gets built and check whether
  it sources `region_m` from the crop or the preset.
- Once a real rebuild exists again, re-run `test_shipped_gt_shows_the_
  displacement` (no longer skipped) and inspect `a_sub["offenders"]`
  against `aprons` geometry before deciding if finding 3 is a bug.
- Do NOT touch `record_xy`/`compose_bakes`/`load_fire` as part of this —
  those are the ALREADY-FIXED frame code and are independently verified by
  the passing AST-level tests.

Related: [[reassemble-buildings-in-frame]] (the already-fixed, DIFFERENT
frame-displacement bug — don't conflate), [[build-urban-fire-city]] (the
7-stage build procedure this bug sits inside), [[urban-layout]],
[[generate-urban-city]] (the actual `build_city`/`districts` generator this
investigation needs to read).
