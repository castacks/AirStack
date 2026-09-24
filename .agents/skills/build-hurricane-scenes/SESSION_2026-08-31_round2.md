# Hurricane suburb, round 2 — session record, 2026-08-31 (05:40 →)

Continues `SESSION_2026-08-31.md`. The user's brief: "floating debris, more
representative house destruction, trees look wrong, etc." plus (mid-session)
"improve the textures on the wet dirt mounds" and "render the brown water as is,
then the same with BLUE water, show me both — at the end".

Run as a team: one lead (reviewer, no grunt work), seven Sonnet streams with
one file-set each, offline verification only, Haiku for pod/container waits.
Every stream's claims were re-measured by the lead before acceptance; the
misses are recorded here because they are the useful part.

## What the baseline actually showed (BASE_L3 / BASE_L2, `~/hurricane_previews/`)

Rendered LOCALLY (RTX 5070 Ti, 16 GB): L3 in 1 min 54 s at 6.6 GB, L2 in ~2.5
min at 11.7 GB. The local box is a fine render machine for the 500 m plate.

| complaint | what it actually was |
|---|---|
| "house destruction not representative" | the top three rungs were the TORNADO fracture+settle bake (a fan of shards on the lawn, the house gone), 38% of L3 houses; stripped roofs were empty boxes with **saturated blue floors**; nothing oriented to the wind |
| "floating debris" | rafts lay PERFECTLY FLAT ON TOP of the opaque quad (centre at `water_level_m`, ±4° tilt), one grain (planks), no strand line, and `HUR_DEBRIS` was read into `N_DEBRIS` and **never used** — zero wind debris on land |
| "trees look wrong" | one white "cotton ball" tree; `defoliated` = 100% bare wire; `snapped` = thinned crown; L2 could not produce a fallen tree (`_TREE_CUTS` above the field's max) |
| cameras | `review_points` sampled the whole region, so the deepest/driest extrema landed 5 m from the plate corners — obliques framed the plate edge (three stacked slabs over the HDRI grass); no close view of anything |

## Bug catalogue additions (item → cause → fix)

| item | cause | fix |
|---|---|---|
| **blue floors in every roofless house, with every floor mesh "bound"** | the kit's `house_floor*` meshes carry GeomSubsets `Section0`/`Section1` with their OWN direct bindings — `Section0 → /Baked/Looks/UnrealMaterial`, a UsdPreviewSurface network pointing at `omniverse://…/Carpet_01_*.png` that does not exist here. A subset binding beats the mesh binding, so the mesh-level `subfloor` wood was never rendered on the top faces. Every audit (`bake.validate`, two agents' bare-pxr walks) asked "does the MESH have a binding" | `tools/hurricane_house_floor_fix.py --fix` clears the subset binding (1,759 across 168 files) and removes the orphan material; `--verify` walks GeomSubsets. **Audit subsets, not just meshes.** Files written through `docker exec` are ROOT-OWNED — the fix had to run inside the container |
| the launcher never exited after `SCENE_DONE` | `while simulation_app.is_running(): update()` — each finished render held 6-12 GB of VRAM until killed by hand; the next launch shared the card with a zombie (the two-Isaac black-frame trap) | exits unless `HUR_KEEP_OPEN=1` / `KEEP_OPEN=1` (quake launcher contract) |
| `roof_collapsed` looked like a lid propped open; `partial_collapse` hinged the roof ABOVE the intact ridge (8.77 m vs 7.88); `leveled` walls only 35° over | `_hinge_bay_down` rotated the rigid gable about a base edge with the sign chosen from a synthetic point at `zmax`, and a 5-12° tilt on an 11 m plate is a multi-metre lever arm | `_squash_and_tilt`: Z-scale 0.30-0.50 about the eave plane (ridge drops, pitch flattens) + tilt capped by a height budget + settle to the wall top; walls ≥80° over; upper-storey floors dropped (≤1.2 m) in `leveled` and over the racked end in `partial_collapse`; porch bays down too. Measured cottage `roof_collapsed` 2.66..4.83 m (ridge 7.88). **Measure world points per mesh class vs the pristine file; a rigid hinge is not a collapse** |
| land debris was a RING around each house | `scatter_from_wreck`'s `reach·u^1.9` with `reach_m=14` put the median tail draw under 4 m — inside the base cloud | comet parameters (`LAND_DEBRIS_REACH_M=40`, `tail_pow 0.30`, lateral base/growth); cone(±35°) fraction 0.28 → 0.76, median 2.7 → 16.7 m downwind |
| no strand line | the background raft loop only placed for `depth ≥ 0.25 m`; the 0-0.15 m band had 2 pieces on 942 m of shoreline | `_strand_line_specs` marches the 0-depth contour: 1,011 pieces at L3 (1/0.93 m), 437 at L2 |
| land debris seated under a metre of water | houses standing in the flood still shed a "land" comet at ground z | pieces landing in depth > 0.10 m become rafts (`LAND_TO_RAFT_KIND`); 1,420 converted at L3 |
| the year-old `airstack://…Soil_Mud/T_pjuph20_2K_B.png` Hydra miss (SESSION §4 item 3) | `planks.skin_material` bound `Sdf.AssetPath(str(texture))` with no `sg._join_asset_root`; `washaway.apply_washaway`'s wet/scoured levels pass `surge.SILT_TEXTURE` through it | wrapped in `_join_asset_root` (no-op on absolute paths) |
| ponds/deposits = flat brown polygons with a hard outline wearing a dry-cracked-earth photo | `Soil_Mud` (linear albedo 0.0575, rough 0.78) is the only mud in the repo; the silt overlay's opacity floor jumped 0→33% at the coverage threshold | derived pack `megascans/Soil_Mud_Wet/` (albedo 0.050, cracks flattened 52%, normal 65% toward flat, rough 0.18), additive rim feather 0.7 m, opacity floor 0.04 with 24 bands, deposits darkened within 2 m of the waterline. `SURGE_PALETTE=blue` for the A/B; ponds never inherit the water palette (that leak pre-existed for `carbonate`/`blackwater`) |
| the white cotton-ball tree | NOT a pristine fallback (BASE_L3 had 0 pristine and still showed it) and NOT park content (the layout places no vegetation outside `tree_instances`). It is `t_1123`, a snapped Largetooth_Aspen whose severed top — authored as a SECOND reference of the source under `/Root/top` — is a 115k-face twig mesh plus untinted foliage lying at the plate centre; earlier, an unbound instancer prototype (Black_Oak `branch1_instancer`) rendered white the same way | (T3/T4) tinted copies rebound on kept instancer prototypes; Material-prim-level `inputs:diffuse_texture` overrides (Common_Apple's `./materials/textures/hollyprivet_basecolor.png`) re-anchored — the fix-up only walked Shader prims; the audit now walks every attribute of every prim incl. the second reference |
| L2 had no windthrow at all; the depth boost then put 41% of L3 down | single monotone ladder + boost | `windthrow_depth_boost` + re-cut: L3 31% structural, broadleaf snapped share 23%; L2 caps at ~7-8% (proven trade-off against L3); T4 adds a dry-land share so windthrow is not 100% in-water |
| review cameras on the plate edge | `review_points` lattice over the whole region + `_clear_azimuth` solving tree clearance alone | inset lattice (70 m margin), joint clear-and-inward azimuth sweep, far-field range cap, off-plate gate; five close subjects (`stripped_roof_house`, `collapsed_house`, `raft_field`, `fallen_tree`, `flooded_street`) from `tools/hurricane_cameras_png.select_review_subjects` |

## Numbers that now define the plate (L3, seed 11)

houses 94: cover_lost 35 / deck_panels_lost 22 / shingles_lost 17 / roof_stripped 14 / roof_collapsed 5 / partial_collapse 1 (structural 6.4%); per-house cardinal variants for the two bay-drop levels (`variant` in the GT record; `hurricane_flow.windward_variant`). rafts ~4,650 (incl. 1,011 strand line, 1,420 converted from land), land debris ~2,770, merged into one mesh per kind (prim count independent of piece count). trees 1,684 placed, 0 fallbacks.

## Working method notes

- **Agents report; the lead re-measures.** Three of seven first reports had a wrong root cause (floors "bound", cotton ball "pristine fallback", poses "verified"); each was overturned by one bare-pxr probe against the actual file.
- The repo is bind-mounted into the local sim, so a render picks up whatever agents have landed — a "baseline" is only a baseline if nobody is editing.
- Both OSMO pods were another session's (`:2200` the 8-robot benchmark — later identified by its own session as airstack-mission-1gpu-52, NOT dev-180 as assumed all day; `:2201` earthquake, dev-177); the new pod `airstack-dev-181` sat PENDING > 4 h. `osmo_provision.sh` now takes `OSMO_SSH_LOCAL_PORT` and opens non-2200 ports with `osmo workflow port-forward`.
- The auto-mode classifier blocked telling an agent to render concurrently with another session's job on the shared card, and blocked an agent's `kill -9`; coordinate with the other session instead (it asked for the card; the idle launchers were ours and were ended with a self-safe `pkill -f "…scrip[t]"` pattern — a bare `pkill -f` matches its own shell).
- A monthly spend limit killed three running agents at once; completed agents' transcripts are often not resumable — put enough context in a fresh brief to continue.

## Still open at the time of writing

- Final renders (brown vs `SURGE_PALETTE=blue`, L3 and L2) pending a GPU.
- Raft panel colours (pale green-grey slabs) and fences standing in 2 m of surge (D3 in flight).
- Oblique water still reads as a glossy brown sheet; no metre-scale wave detail (unchanged from round 1's item 1).
- Gable-end failure has no geometry (a GeomSubset inside `Roof_01`).

## Round-2 close-out (12:00)

The four finals exist and are the deliverable: `~/hurricane_previews/FINAL_L3_brown`,
`FINAL_L3_blue`, `FINAL_L2_brown`, `FINAL_L2_blue` (same geometry/seed per level;
the only difference is `SURGE_PALETTE=blue`). Rendered on a BORROWED earthquake
pod (dev-181 never scheduled, 6 h PENDING): all four in ~11 min on dev-177's idle
GPU, from an isolated workspace under the LOGS mount (`/root/docker/isaac-sim/
logs/hurricane_ws`) — nothing written under the owner's clone; heavy shared packs
symlinked with CONTAINER-path targets. Two traps found on the way: the workspace
needs `scene_gen/layout/` too, and H2b's host-side rebuild baked HOST-absolute
texture paths into the house archetypes (fixed: 528 paths rewritten to
archetype-relative `../aec/...`, which resolves on host, local container and pod).

D3's fence pass renders correctly (fences gone in the surge zone, flattened
downwind on land); collapse poses read as collapses; the camera set frames all
10 subjects; the blue variant leaks nowhere (ponds/deposits stay mud).

**One known cosmetic issue ships open:** the raft mats (strand line, tangles)
render as flat pastel — the authored materials are verified correct offline
(Ash planks texture bound, D3's per-channel tint exact: authored tint x texture
mean == target per channel), but in the RENDER the wood texture visibly does not
contribute and the tint shows raw (uniform mint on `sheet`/`panel`). Same look in
the local ROUND1 render, so it is not a pod path issue. Next diagnostic is a
flood-water bench render of `build_rafts` alone, toggling texture/tint/ORM
inputs — not more offline inference; the fix was NOT attempted blind.

Also noted for a future pass: L3 roof_stripped houses all wear complete rafter
cages (`_ROOF_FRAC` 1.00) — a partial-sheathing/broken-rafter variant would
break the repetition; and the launcher tree tally at L3 is now limbed-plurality
(679 limbed / 467 defoliated) — inside the brown-window story but worth an eye.

## Post-close correction from a sibling session (dev-180's owner)

Its "both 48 GB cards idle" offer was a mis-measurement: `nvidia-smi` from the
pod-host ssh session saw only 2 of 4 cards — the two Isaac does NOT use — while
inside the container the render card was at 98%. **Read GPU state from inside
the container that owns the workload (`docker exec <ctnr> nvidia-smi`) and match
cards by UUID, never by index — indices differ between namespaces.** My own
dev-177 readings were host-side too; the borrow worked because the owner's bake
was genuinely in a CPU phase, not because the reading was sound.

Addendum from the 8-robot benchmark session (runs on airstack-mission-1gpu-52; it and we both mislabelled its box as dev-180 for most of the day — a port-forward tells you nothing about which workflow is on the other end, ask the pod for its hostname and match it to `osmo workflow query`) (verified advice, not yet exercised here): to pin
Kit to one card on a multi-GPU box, pass `--/renderer/activeGpu=<idx>` AND
`--/renderer/multiGpu/enabled=false` — Kit's default multiGpu probes every NVML
device and, if CUDA cannot open one (which `CUDA_VISIBLE_DEVICES` subsetting
causes), drops ALL of them and falls back to software rendering (0% GPU, ~273%
CPU). Verify with the Kit log: zero `CUDA being in bad state` lines, and the
`device mask:` on the SceneRenderer line. Resolve the card by UUID in YOUR
container first; indices differ per namespace.

# Round 3 (afternoon) — palettes, tornado-language damage, 3x flood debris

User, on the round-2 finals: house textures wrong ("the layout+house generator…
is ignoring some things"), damage "not realistic — look at how we do tornado…
match that", and "debris in the flooded area needs to increase a lot".

| finding | cause | fix |
|---|---|---|
| every house wore the kit's raw defaults (mossy `Roof_Tiles/Roof_Felt`, cream `Cladding_01`, `Brick_01_Dirt` gables) | the hurricane bake called `apply_palette` WITHOUT stamping `"palette"` onto the placements — `apply_palette` skips every subset silently (the tornado bake stamps; the hurricane bake didn't); AND the layout never drew per-house palettes (94/94 palette=None), and the launcher's live-recolour fired only for `row` homes (0 at this seed) | stamp `STYLES[st]["palette"]` in the bake; `modular_house.draw_house_palette` (STABLE HASH keyed on seed+parcel+lot — never the shared rng stream) over a 6-palette pool; `_recolour` fires whenever the draw differs from the baked default (needs `instance=False` — `Usd.PrimRange` does not descend into instances; measured 79/94 recoloured, 2,578 subsets rebound live) |
| damage read as rigid/clean (squashed whole-plate roofs, complete rafter cages = "under construction") | pose-only rungs have no breakage language | HYBRID: the 3 structural rungs are TORNADO-CANON again (`--adopt-tornado` copies `_tornado.usd` over canon + reseat-to-convergence, 21-51 fragments/file, then verifies walls-standing at `roof_collapsed`: 4/5..11/13 whole walls per style); rafter levels get RAGGED rafters (26% removed, 15% snapped with clamped droop, 1-3 jagged sheathing patches + ridge shreds per roof) |
| flood debris sparse | round-2 densities | ~3x: L3 4,654 → 12,962 floating pieces (open water 5.3x, obstructions 4.2x) + wind-parallel DRIFT LINES (6-12 streaks, 20-60 m, bearing±12°); still one merged mesh per kind |
| raft mats flat mint | TWO layers: `build_rafts` left `diffuse_color_constant` white (OmniPBR's fallback if the texture ever fails) — fixed to carry the real colour; and the `vegetation` kind's own target resolved to PALE SAGE, which under the storm dome reads mint across the majority-vegetation open water — bench-verified (`tools/raft_material_bench.py`, 1-min pod render) and darkened to sodden foliage (0.105,0.115,0.060 linear) | |

## THE REBAKE RECIPE (the only safe order — an unscoped Kit rebake DESTROYS the tornado-canon rungs)

On a pod/container with Kit (~30 s + ~3 min of post-passes, all idempotent):
1. `ARCH_LEVELS=shingles_lost,cover_lost,deck_panels_lost,roof_stripped` Kit-rebake (`bake_hurricane_archetypes_launch_script.py`) — the env filter added 2026-08-31 exists precisely so the three structural rungs are NOT rebuilt pose-style over the fracture canon; `pristine`/`swept` re-copy from the (paletted) tornado library.
2. `hurricane_house_pose_bake.py --build --promote` (ragged rafters; NOTE its promote clobbers those levels' `_tornado.usd` A/B backups — archival only).
3. `--build-variants --promote-variants` (64 cardinal files).
4. `--adopt-tornado` (restores fracture canon for the 3 structural rungs + reseat).
5. `hurricane_house_floor_fix.py --fix` (the Kit rebake reintroduces ~2,200 `UnrealMaterial` floor-subset bindings every time).
6. `--verify --verify-variants --verify-tornado` (must end "0 with an open issue").
Backup the library dir (tar) before step 1, always.

Render env on a pod whose yaml sets `AIRSTACK_ASSET_ROOT` to Nucleus: pass
`AIRSTACK_ASSET_ROOT=` (empty) per launch so `airstack://` resolves to the
synced clone — the new packs (`Soil_Mud_Wet`) are not on Nucleus.

# Round 4 (evening) — TORNADO PARITY, by user directive

User, on the round-3 finals: "seems like house textures haven't been fixed and
neither has damage. Tornado seems to run fine so compare with that. Make sure
we're doing the same damage as tornado, including placement of extra debris.
Same house generation including material. You need to just adjust the pattern
of house damage and adjust placement of extra debris based on flooding."

**The hurricane house path is now the tornado's, verbatim** (side-by-side diff
first, then a single rewrite stream):

- Houses reference `archetypes_TORNADO` (new env `HOUSE_ARCH_DIR`; `ARCH_DIR`
  is tree-only now). Style-default baked palettes; recolour ONLY row homes
  (`bool(h.get("row")) and bool(_pal)`, batched pal_jobs); street yaw always;
  live-build fallback identical. DELETED from the render path: per-house
  palette draws + the 79-house de-instanced recolour, the
  shingles/cover/deck/roof-variant lookups, `_WIND_YAWED`. The round-3
  machinery (pose bake, rafters, cardinal variants, `draw_house_palette`)
  stays in the tree but nothing references it at render time.
- `hurricane.tornado_level_for_intensity` maps the hurricane field onto the
  SIX tornado levels (cuts fit against the real GT populations): L3 ≈ pristine
  31 / roof_stripped 53 / collapsed 12 / partial 3 / leveled 1 %, L2 ≈ 61/38/1.
  `swept` is unreachable from wind — the launcher swaps in the `swept`
  archetype only where `surge.house_water_state` says so.
- Extra debris: the tornado plank machinery (scatter_from_wreck with the
  baked-style palette skins + scatter_over_region) with exactly two deltas —
  bearing per house from `hurricane.wind_bearing_at`, and flood-awareness
  (region litter masked to depth ≤ 0.10 m; submerged pieces are the raft
  system's job, never double-placed). `HUR_DEBRIS` now defaults 140 (TOR_PLANKS
  parity).
- `suburb_scene.py`'s per-house palette hunk was REVERTED (it would have
  silently changed the approved TORNADO scenes too — shared layout code).
- The `archetypes_TORNADO` library itself got the floor-subset fix (412
  `Section0 → UnrealMaterial` bindings cleared across 48 files — the blue
  slabs came back the moment houses referenced the tornado originals). This
  also changes future tornado renders: floors render as wood, not RTX blue.
  The library is root-owned on this host — run the fix through the container.
- The tornado reference render on the pod died in the OLD viewport-capture
  path (`suburb_tornado_launch_script` has no `snapshots_rp`; the documented
  headless segfault). Port it before rendering tornado on a pod.

Lesson pinned: when the user names an approved pipeline as the standard,
DIFF AGAINST IT FIRST and inherit it verbatim — the two custom rounds built
better-in-isolation pieces that read wrong precisely because they diverged
from the reviewed look.

# Round 5 (night) — debris/texture polish, OFFLINE ONLY (user: Nucleus down, no renders)

User notes on FINAL3 → all fixed offline, render pending user go:
1. Water debris one-texture → rafts near a house now wear THAT house's baked
   siding/deck via (kind,skin) mesh groups (27 meshes, 17 skinned); the
   land→raft conversion preserves skins.
2. "Burnt" debris → it was `roof_top`'s 0.06 charcoal (+wet 0.6 ≈ 21% luma);
   now (0.13,0.13,0.14), plus a load-time ≥28%-dry-luma assertion over
   `_RAFT_TINT` and a verified zero burn/scorch textures in the debris path.
3. The strand line OUTLINED the flood → clump/gap state machine (3-15 m clumps,
   8-30 m gaps, band −4 m..+6 m): 1,988 → 633 pieces, 31.8% shoreline coverage.
4. Floating trunks/trees → logs 3.5-11 m with REAL bark texture + stub
   branches; `floating_tree_specs` places ~20 half-submerged fallen-tree
   archetypes (draft 0.41-0.59) in open water ≥12 m from houses (passing the
   TREE obstacle list starves placement — houses only).
5. Pond interiors → wet-mud pack was ALREADY bound; the flat read was TILING
   (1 m repeats). Now 4.5 m pond tiles; same story for roads: the presets have
   pointed asphalt at `Wet_Destroyed_Asphalt` since commit e6aee69b — the
   wrapper's 11 m tiles hid it; now 3.6 m (test band updated). **When a bound
   texture "isn't there", check the TILING before the binding.**
6. Trees ("forget defoliated — I see 1-2 trees"): damage now reads by CROWN
   COLOUR — defoliated = 80% crown kept + dead-leaf brown tint (the bake
   writes a pruned visible copy `leaves_cards` and hides the original — count
   the COMPOSED visible faces, not the stats), pristine 15.7% at L3 / 46.3% at
   L2, canopy budget 13.8% → 72.1% (L3). Structural shares unchanged.
7. BLUE WATER DROPPED by user decision: brown only from here.

Incident: an agent's `git stash`/`pop` momentarily wiped ~65 tracked files of
uncommitted multi-session work; recovered byte-for-byte from the stash, which
is KEPT as `stash@{0}` — verify signatures across streams before dropping it.
Never stash in this repo (concurrent sessions); test isolation must copy files
to scratch instead.

Render checklist when the user gives the go (dev-181, port 2203, brown only):
sync `scene_gen/disaster/{washaway,surge,hurricane,ground}.py`,
`scene_gen/suburb_scene.py`, the launcher, `snapshots_rp.py`,
`scene_gen/assets/archetypes_hurricane/tree_*.usd` (34 rebaked),
`megascans/Wet_Destroyed_Asphalt.usda` + `Soil_Mud_Wet*`, presets; then
FINAL4_L3_brown + FINAL4_L2_brown.

## Round 5 postmortem addition (late night)

FINAL4 rendered with STALE TREE ARCHETYPES on the pod: measured 3/60 brown-hue
crown hits at defoliated GT positions in BOTH FINAL3 and FINAL4 (identical),
while the code-side change landed (pristine 255, 18/30 visibly crowned). The
pre-render check counted files ("34 tree_*.usd") — counts pass on stale bytes.
The pod was cancelled before an autopsy was possible. Countermeasure, now
mandatory: `scene_gen/tools/render_preflight.sh` — md5 of every render-critical
file local vs render host, render REFUSES on mismatch. **Verify content, not
counts; pull the render log with the frames; never release the render host
before the frames are accepted.**

Border re-tune per user ("the border is made from the debris"): waterline
boost 4.0→1.6, strand clumps 2-8 m, gaps 25-90 m, scatter −8..+18 m →
shoreline coverage ~8-14% (was 32%, originally 100%). 47/47 debris tests.

## The render policy, as of round 5's end (user-directed)

NO RENDER until BOTH offline gates are green:
1. `scene_gen/tools/render_preflight.sh` — md5 of every render-critical file,
   local vs render host; refuses on mismatch (catches stale syncs).
2. `scene_gen/tools/hurricane_layout_png.py` — the 2D dry-run: replays the
   launcher's placement logic offline and plots every element (houses by
   level+palette, tree crowns, debris by kind/skin, fences, floats, water,
   roads) in its PREDICTED RENDERED COLOUR, derived by walking the actual
   authored material chain — texture texel mean × tint only when the shader
   is OmniPBR-family; custom MDLs are sampled raw and any tint authored on
   them is flagged DEAD_TINT; unresolved textures flag MISSING_TEX; damaged
   foliage whose predicted product is not brown fails the run (exit non-zero).
   This is the offline form of the pixel test that caught the green-crown
   dead knob (a diffuse_tint authored on a per-species leaf MDL that never
   declares the input — §2g's bug class, one level deeper, invisible to
   every count/existence check).

# Round 6 (overnight) — THE STACKED-TREES ROOT CAUSE, and the calibrated brown

The tree saga's true bottom, found via a ROOT-LAYER STAGE DUMP (new launcher
env `HUR_EXPORT_STAGE=1` writes `stage_root.usda` next to the snapshots —
references intact, cheap, auditable offline):

**Every tree archetype baked its cm→m correction as `xformOp:transform(0.01)`
ON `/Root`.** `_ref`'s `Add*Op` calls APPEND to the composed op order, and the
first-listed op is outermost — so the archetype's root scale wrapped the
launcher's translate: a tree placed at (−226,−107) landed at (−2.26,−1.07).
ALL 1,684 trees of EVERY render to date stood full-size, stacked within
±2.5 m of the plate origin. That pile IS the "cotton ball"/"park tree" seen
since the first session; there was never any park vegetation. Houses were
immune because the tornado house bake leaves `/Root` op-free.

Fix: the scale lives on the CHILD (`/Root/src`, absorbed into `/Root/top`'s
matrix for snags); roots are op-free. Locked by
`test_launcher_style_reference_lands_at_the_placement_point` — references
every archetype the launcher's way and asserts the composed bounds sit AT the
placement point at tree scale. THIS TEST IS THE LESSON: bounds-under-
reference, not bounds-in-isolation.

Also closed on the way: the launcher's close-subject helper
(`tools/hurricane_cameras_png.py`) was missing from the pod sync AND from the
preflight manifest — 4 subjects rendered instead of 10 with zero errors; the
manifest now carries every module the launcher imports at runtime.

Colour: T6's swatch-brown rendered SALMON under the storm dome (measured
sRGB (156,127,104) at 91 crowns). One measured correction — bake targets ×
(0.726,0.601,0.351) — landed (133,105,80) vs the (135,100,62) intent.
**Calibrate foliage colour against a RENDER, then bake the corrected target;
the swatch prediction is necessary (hue, dead knobs) but not sufficient
(lighting lift).**

FINAL8_{L3,L2}_brown are the deliverables: full brown canopy at true
positions, drowned forest in the flood, green survivors, tornado-language
damage, calibrated debris. dev-183 stays up for iteration (~19 h window).
