# Suburban earthquake: research, reuse, and 250 m review

2026-09-04. Scope: an early-response daylight neighbourhood, 250 × 250 m,
using the existing residential kit. This is a synthetic SAR visual prototype,
not a calibrated ground-motion, structural-response, or casualty-loss model.

## Evidence and interpretation

The relevant plans are in `_plans/`, not `plans/`. The existing
`earthquake_plan.md` and `earthquake_research.md` cover urban masonry/RC
failures; `hurricane_research.md` documents the suburban wood-frame inventory
and the distinction between structural material and exterior cladding.

1. **Wood-frame houses often retain their box and roof while a weak foundation
   connection fails.** FEMA's [South Napa cripple-wall recovery advisory](https://www.fema.gov/sites/default/files/documents/fema_earthquakes_earthquake-strengthening-of-cripple-walls-in-wood-frame-dwellings-fema-p-1024-ra2.pdf)
   identifies vulnerable crawlspace walls and inadequate anchorage. The
   [FEMA mitigation planning guide](https://www.fema.gov/sites/default/files/documents/fema_earthquake_mitigation_planning_guide_for_communities.pdf)
   describes damage concentrated beneath otherwise intact houses. Implication:
   settlement, residual lean, and displacement relative to drives/foundations
   must be visible without automatically removing the roof.
2. **Site failure differs from shaking damage.** Beattie et al.,
   [Performance of houses during the Christchurch earthquake](https://bulletin.nzsee.org.nz/index.php/bnzsee/article/view/226),
   report housing damage from ground acceleration and from liquefaction,
   lateral spreading, and rockfall. The [USGS-supported Canterbury investigation](https://earthquake.usgs.gov/cfusion/external_grants/reports/G12AP20034.pdf)
   documents impacts on timber residences and buried utilities. Implication:
   designate susceptible soil independently of the epicentre. Do not turn every
   green area into a tornado-like scour track or add rockfall on a flat site.
3. **Literature categories must be gated by actual asset inventory.**
   [FEMA P-1100](https://www.fema.gov/sites/default/files/2020-08/fema_seismic-retrofit-family-dwellings-prestandard_p-1100.pdf)
   addresses wood-frame dwelling vulnerabilities, including foundations,
   hillside support, and chimneys. The local urban research also identifies
   chimney and veneer loss as earthquake cues. This does NOT establish their
   presence in these layouts: the user confirmed there are no chimneys.
   The first prototype's invented attachments were an incorrect assumption
   and are removed. Do not treat a brick palette as proof of an URM wall.
4. **Casualty mechanisms are not an outdoor scattering model.** Peek-Asa et al.,
   [Fatal and hospitalized injuries, Northridge](https://pubmed.ncbi.nlm.nih.gov/9698136/),
   found collapse prominent among fatalities and falls/falling objects among
   admitted injuries. [Mahue-Giangreco et al.](https://pubmed.ncbi.nlm.nih.gov/11399450/)
   associate more severe injury with older housing and multifamily structures,
   among other factors. These studies do not give a universal distribution of
   post-event outdoor survivor locations.
5. **Early extrication is often local.** The
   [Bartolucci et al. review](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)
   reports relatives/neighbours/local inhabitants performing 60–100% of
   extrications in the included studies. This supports early scenes around
   damaged homes, but does not establish a universal responder density or
   justify placing every casualty outside. This review depicts visible rescue
   targets; it omits fully concealed victims and is therefore detection-biased.

The neighbourhood's shaking varies smoothly using the **same field evaluator
as urban earthquake assembly**. A kilometre-scale source is configured; a
250 m plate does not get a 25 m-radius epicentral destruction bullseye.
House variability and the soil ellipse provide local differences. Magnitude
continues through the existing compiler, but the resulting demand and grade
thresholds are artistic controls, not PGA/MMI predictions or fragility fits.

## Damage and reuse matrix

| Population/mechanism | Existing implementation | Decision for this prototype |
|---|---|---|
| Streets, parcels, fences, yards, pools, parked cars | `suburb_scene.generate_suburb_on_stage(assembly=True)` | Reuse unchanged; same 250 m street-network parameters as tornado review |
| Eight residential forms and palettes | `detail.modular_house` | Reuse unchanged; explicitly assume wood-frame stock with mixed anchorage, including brick veneer |
| Magnitude compilation and map field | `compile_disaster.compile_earthquake`, `scene_generator.make_damage_field` | Reuse; preset overrides spatial scale without changing urban defaults |
| Foundation lean/settlement and displaced soil | `quake._tilt_prim`, `_c_tilt_ground`, `quake_flow` | Reuse on standing shells; never rotate settled rubble |
| Partial wall/support failure | Modular kit selection and tornado/fire `planks` | Remove failed bays, replace with compact framing/siding/roof stock carrying the source palette |
| Local complete timber collapse | Same stock and per-house cache | Retain ground-floor slab; cache static local collapse, no whole-neighbourhood settle |
| Garage/weak-storey failure | Urban concept; suburban kit knows storeys and garage | Separate geometric residual-storey deformation, coherent upper box; only eligible multi-storey garage houses |
| Chimney failure | No chimneys in these layouts | Excluded; never synthesize an unsupported attachment |
| Veneer, stucco cracking, glazing loss | Hurricane cladding and urban scars contain related operations | Follow-on: requires a skin-only representation; removing a structural module would falsely equate veneer loss with wall collapse |
| Fissures and displaced surfaces | `quake_flow` mesh/material emitters | New shared trace field; replace bands of actual asphalt/turf with raised/dropped pieces retaining material and UV scale |
| Trees | `vegetation.tip_tree`, `root_ball` | Local root failures near ruptures/weak soil, keep green crowns; other nearby trees follow ground normal |
| Fence/pole/bin failure | Existing whole-prop transforms | Actual debris-footprint intersection topples fences; signs/props follow shaking or local displaced support; cars tilt but are not crushed |
| Under-rubble bodies | `quake_people`, `fire_people`, `tornado_people`, `planks` | Reuse support, sightlines, RP poses and solved cover; substitute timber cover for urban concrete |
| Visible exposed interiors | `quake_people._interior_person` | Attempt only partial multi-storey damage with actual support/opening; reject concealed locations |
| Injured people at exits/frontages | Poses reusable; general refuge/egress priors are fire-specific | Rubble-apron targets in this review; separate unburied exit-injury class remains follow-on |

The prototype does not call the wind or fire **damage ladders**. Wind's
roof-first selections, long thrown board trails, tree-defoliation decisions,
and tornado casualty displacement/clustering priors are not earthquake logic.
Shared mechanical and pose helpers are reused by direct calls.

## Implemented contract

`disaster/quake_suburban.py` contains a pure deterministic house planner,
the new house-damage selection, and the casualty adapter.
`config/presets/suburb_earthquake_250.yaml` pins the stock, layout, and site.
`tools/suburban_quake_plan.py` writes the 2D preview and JSON.
`suburb_earthquake_launch_script.py` builds the scene and leaves Isaac open.

Sampling knobs are explicit scenario assumptions: mild-racking share 0.65,
per-house demand score and grade cuts, a maximum of 12 visible casualties,
and the existing partial-cover distribution capped at 0.55. None are claimed
as epidemiological rates. Casualty status is `unknown`, `alive: null`; no
independent random fatality coin is carried over from the urban helper.

Revised seed 10: 60 houses; 7 pristine, 14 racked, 18 foundation,
20 partial collapse, 1 full collapse, 0 eligible soft-storey draws. Those are
synthetic sample counts, not predicted losses. No modes are forced into a
building that lacks their structural preconditions.

## Validation and review

1. Run `python3 scene_gen/tools/suburban_quake_plan.py` before Kit.
2. Check frame, deterministic sampling, soil gating, and support selection.
3. Build a new output directory using `tools/suburban_quake_cpu.sh` and
   `tools/suburban_quake_build.py` in the existing container. This uses USD
   without Kit or GPU simulation and preserves an occupied live scene.
4. Require static caches, zero global physics steps, correct source materials,
   and object responses matching the shared field. A successful build is not
   a render verification. Switch the live view only when authorized.
5. Save exact live house records and people records, inspect both wide views
   and close-up casualty views, and leave the overview camera active.

The revised review uses a dedicated per-house earthquake cache, keyed by
style/palette/mode/side/variant and recipe revision. Existing wind/fire archetype
directories and the frozen dataset are not export targets for this work.

Limitations to assess on screen: the new material-matched ground replacement
and static debris/contact responses still require close-up visual review.
Foundation offset and weak-storey geometry are static visual approximations.
The authored stock is not an engineering collapse simulation. Body-cover fraction
is modelled cover, not a measured segmentation visibility percentage.

### Verification notes

Revision `timber_bays_v3` was built CPU-only into
`/isaac-sim/.nvidia-omniverse/logs/suburban_earthquake_250_r6` in 45.4 s:
60 houses, 12 support-gated casualties, 1,625 replacement ground pieces,
112 interaction records, zero global physics steps. A cold USD reopen checked
all house shells, found zero active bodies in stage traversal, and confirmed
`sqh_022/debris_house_roof_8_35/frag_002` is absent. That house now has 209
stock pieces with 11–90 mm thickness. These are static checks, not rendered
acceptance. The user's original live scene was deliberately left untouched.

The CPU planner, shared/new casualty, ground/material and transform contracts
pass 24 tests in the Isaac container. The adapter
additionally checks nine points along the real soles-to-head footprint of
rubble casualties. It queries `/World/stage/generated/ground` explicitly:
the urban helper's alternate ground root and z=0 fallback are not evidence
of suburban support. Uneven piles and missing ground are rejected.

The initial live prototype needed a slow, whole-scene decomposition/settle.
That workflow is superseded: revised assembly references static house caches,
authors ground bands, and applies placement-local overrides. No whole-scene
settle or kick is present in the revised launcher. Cached rubble and the
standing shell are distinct children, so foundation movement cannot lift a
previously seated rubble bed wholesale.

The repository-wide strict MkDocs check encountered existing missing-link
warnings and copied large unrelated assets; it was stopped. Its temporary
output was moved to Trash. No successful full documentation build is claimed.

### Follow-up: supported collapse, canopy clearance and clear noon sky

The `supported_volume_v4` cache selects failed modules by their measured spans
and propagates loss of bearing to upper floors/walls/roofs. Debris quantity is
proportional to removed face area and concentrates within the collapsed footprint.
Foundation soil heave is restored using the same low/high sides and drop/rise
as the actual house tilt. Rupture envelopes and displacement taper at their ends.

Shared suburb-net placement is used, but yard trees only had a species-size bias,
unlike the hard canopy fit in parcel/open-ground planting. The earthquake adapter
checks composed crowns against all house footprints plus 0.6 m, substitutes an
existing smaller species or omits the station, and checks again after ground
motion or uprooting. This is a conservative bounds check, not a trunk-only test.
The r7 static build retained 438 trees, changing 248 original stations, and kept
6 of the previous 12 casualty placements after revalidation; 6 replacements use
the same support/visibility-gated placement logic. Thirty focused tests pass.

The user superseded the hurricane-sky request with
`/NVIDIA/Assets/Skies/2022_1/Skies/Clear/noon_grass.hdr`; its Nucleus asset was
verified (98,118,510 bytes). The r8 build uses that HDRI with a neutral tint and
the same v4 damage caches. Stop the previous launcher with Ctrl-C in the existing
container's tmux session, confirm it exited, then launch the new review. Never
overlap Isaac runs, recreate the container, or start global physics for review.

r8 cold-reopen audit: 60 houses, 12 people, 438 retained trees, zero crown
keepout violations, zero active rigid bodies, 18 foundation responses, and the
exact noon HDRI. Build took 83.5 s. The previous launcher exited before r8
started; r8 reached `CAPTURES_COMPLETE` and was left open. Rendered collapse
piles are substantially fuller. Remaining visual review caveats: some retained
partial-collapse floor spans still look broad/cantilevered, and the first
casualty's first review camera is obscured by tree foliage despite the numeric
sightline gate. Do not treat test success as full visual acceptance of either.

### r9: actual fractured skins and isolated foundation debris falls

User review requested noon exposure 0, actual house fragments mixed with authored
stock, fire-style probabilistic removal, irregular partial-collapse boundaries,
and bottom fracture on sinking houses. New `fractured_bearing_v5` caches use
`quake_suburban_fragments`: the shared fracture backend cuts original surfaces
without fabricated caps spanning the open kit volume, shared fire skin projection
carries original UV/materials, and 40% of detached fragments are removed with a
large-fragment bias. The stock budget is reduced to compensate, not eliminated.
Ground wall remnants must connect to the original bottom; surviving modules at
the collapse boundary are torn too. Foundation failures lose irregular lower
wall sections; upper structure follows the existing low/high bearing transform.

Decisive tilt finding: `apply_placements` redefined `structure` as Scope. The
nonzero saved matrix did not affect any child geometry. Restoring Xform AFTER
placement gives composed tilts 6.436 degrees (033) and 5.740 degrees (036).
The old retained floor `sqh_006/house/structure/house_floor_0_13` is inactive
in the candidate cache. Tests include Scope as an explicit negative control,
composed descendant tilt, source-surface fracture conservation/no thick caps,
and actual-fragment fence footprints: 34 pass in the Isaac CPU environment.

Prepared caches are not assembly-ready until the isolated bake launcher opens
each required house, settles detached pieces against that house's structure and
stock, passes the convergence and exported-point check, and saves it. Assembly refuses
pending entries. This stage is separate from the full neighbourhood and remains
one Isaac process at a time. Completed cache entries survive a restart. Collision
decomposition is reserved for genuinely thick/folded cells using PCA thickness;
applying it to almost every thin panel spent most of the first bake on cooking.

The shared settle verdict samples penetration before its final points-based
clamp. Foundation 007 converged with zero moving pieces but one 0.1016 m
penetration; final exported minimum was -0.0109 m, within contact tolerance.
The local gate records both verdicts, permits only bounded repair (0.20 m), and
still rejects unconverged/moving/unmodelled bodies and final penetration over
0.02 m. This does not change shared settle policy for any other pipeline.

r9 build and cold-read gate (2026-09-04 local): all 51 required cache entries
ready; 31 isolated physics bakes / 7,157 detached fragments, all converged with
zero moving bodies. The saved scene contains 60 houses, 12 casualties, 381 trees,
zero enabled rigid bodies, and no composed crown-clearance violations. All 18
foundation houses have measurable composed tilt; 033 is 6.437 degrees and 036
5.740 degrees. The named floor 006 is inactive; noon sky exposure is 0.
Artifacts: logs `suburban_quake_r9_{build,audit,live}.log`, scene directory
`suburban_earthquake_250_r9`, review captures `suburban_earthquake_250_r9_review`,
under the container's `.nvidia-omniverse/logs/` mount. Render review follows the
cold-read gate and is not inferred from these numeric checks.
