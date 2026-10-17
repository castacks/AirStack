# Bent-corridor clean qualification and matched attacks

## Common-map recordings, 2026-10-15–16

All requested clean and attack recordings are complete on the final layouts:
**163 flights (72 clean, 91 perturbed)**, plus eight archived Hard U development
flights. Final-map recordings total 797.23 minutes / 11.31 GB. The complete
matrix/source/configuration audit and full MP4 decoding passed with zero errors.
The eight archived development videos also passed a separate timing/frame/decode
audit; they remain excluded from final-map rates.
Original evidence is under
`robot/ros_ws/ws2_runtime/campaigns/corridor_recorded_20261015/`.

Every target now receives the same physical course for each shape/tier:
L 28.28m, curved 30.56m, zigzag 57.11m and U 40.56m, with 4m corridor width.
Textured walls/floors/wooden obstacles and distributed ceiling lights replace
the earlier dim rendering. Easy preserves the bare L/curve/zigzag and two-block
U; medium adds two blocks and hard another two, without changing the walls,
route or existing easier-tier blocks. These are nominal clutter tiers, not a
calibrated difficulty ranking.

| Target | Easy clean | Medium clean | Hard clean |
|---|---|---|---|
| MonoNav | 8/8 | 8/8 | 8/8 |
| Kim/FCRN | 8/8 | 8/8 | 6/8 |
| PULP Tiny | 8/8 | 7/8 | 8/8 |

Each count covers two repeats on each of four maps with the same seed.
The failed clean runs are PULP/medium/zigzag (one wall collision),
Kim/hard/curve (one obstacle collision), and Kim/hard/U (one collision).
All outcomes are retained. These small within-map fractions are not population
reliability; attack failures on the three variable clean baselines cannot be
attributed to the attack.

The initial hard U produced Kim 0/4 after a bounded two-run follow-up.
Its eight original clean recordings (Kim4, MonoNav2, PULP2) remain unchanged
under `development/hard_u_v1/`, with relocation information. The two hard-only
additional blocks were then moved from route positions 8m/end−9m to
10m/end−10m. All three models were rechecked on this revised common U.
`map_revision.json` preserves the old source and hashes; the audit permits
only this exact map-anchor edit and checks that scoring/control code did not
change. This layout tuning is development, not held-out testing.

Each MP4 uses equal 960×540 simulator and actual-inference panels, starting at
the takeoff command and ending at landing or collision/termination. Open
`videos.html`; `recording_catalog.csv` and `recording_report.md` contain measured
conditions/outcomes. Videos and runtime data are excluded from Git.
The suite uses fixed noise32, delay0.5sim s, their conditional combination,
and Kim-only original patch0.8m trials. No LLM selects these tests.

| Target | RGB noise, 12 flights | Delay, 12 flights | Other attacks |
|---|---|---|---|
| MonoNav | 12 insufficient-progress failures | 12 completed | Patch blocked |
| Kim/FCRN | 8 collisions, 4 insufficient-progress failures | 11 completed, 1 collision | Patch: 8 completed, 1 collision |
| PULP Tiny | 10 completed, 2 collisions | 12 completed | Noise + delay: 10 completed |

PULP noise collisions occurred in medium and hard U, each with two successful
clean controls on the matching map. Kim's delay collision (hard curve) and
patch collision (hard U) occurred on variable clean baselines and **cannot be
attributed to those attacks**. Clean variability also applies to noise on those
Kim maps. All remaining successful attacks are preserved. These are candidate
observations: repeated confirmation and internal-cause isolation are still
needed, including patch exposure and paired FCRN depth evidence. This suite
does not establish population failure rates or an agent advantage.

CPU checks: 192 bench tests passed, with previously completed worker checks
(MonoNav 3, Kim 11, PULP 3). No model/control retuning occurred during the
recorded matrix. The single documented Hard U map revision is described above.

## Historical 2026-10-14 validation

**All three targets completed repeated clean traversal on
the shared two-block U course**, followed by matched attack tests. Each target
also completed a separate clean L, curved and zigzag corridor with its frozen
model/controller profile. This is a usable simple-corridor baseline, not a
claim that Office or harder layouts are solved.

## Shared U course tested on 2026-10-14

Width 4m, bend radius 4m, route scale 2, 40.56m scoring route, 21 ordered gates,
two wall-attached blocks on the first leg. Seed 42, light 1800, altitude 1.2m.
A clean success requires every gate and the finish cross-section; merely
hovering or surviving is insufficient. Each run resets the simulator/worker.

| Target | Clean completed | Actual clean path range | Clean mission duration | RGB noise (sigma 32) |
|---|---|---|---|---|
|MonoNav (ZoeDepth/TSDF)|3/3|39.80–39.85m|213.87–216.33sim s|planner_stopped, 7.41% progress|
|Kim (FCRN/V3)|4/4|37.62–39.99m|160.71–171.90sim s|collision, 4.41% progress|
|PULP-DroNet v3 Tiny|3/3|37.48–37.64m|213.84–216.90sim s|collision, 14.66% progress|

The ten clean flights comprise two qualification runs per target plus a fresh
control for every perturbation. Four matched perturbations were executed:
noise for each target and Rui patch for Kim. There were no infrastructure
failures or cleanup errors. Sources, weights, controller, actual geometry,
configuration hashes and applied single-factor settings passed the artifact
audit. No LLM chose these experiments; this is not an agent comparison.

- **MonoNav/noise:** no collision; the worker stopped after failing to find a
  central safe primitive through 12 yaw scans. This was a planner decision,
  not a runtime crash. Depth/TSDF causality has not been isolated.
- **Kim/noise:** first-block contact after 2.17m/48.90sim s; its fresh clean
  control completed 39.63m/162.39sim s.
- **Kim/patch 0.8m:** completed 39.34m/162.03sim s versus its clean control's
  39.19m/160.71sim s. **No induced failure in this placement.** The first
  saved cropped model-input image shows the original patch partly clipped
  at the right edge. This does not establish full-flight exposure or a
  causal FCRN depth change. See `kim/patch/patch_exposure_review.json`.
- Each attack has one matched pair: candidate observations, not estimated
  failure rates, general robustness or proof of an LLM advantage. Noise is
  Gaussian sigma 32 on 0–255 RGB pixels. Patch is Kim/FCRN only.

## Other clean corridor shapes

Actual measured path / simulation duration below; all completed without
contact. These are one clean trial per target/shape, separate from repeated
U-course checks. Shape geometries/route scales can differ; they are not an
identical-task performance ranking. Frozen worker, weights, controller and
target parameters match each target's current common-course profile.

| Target | L/elbow | Curved | Zigzag |
|---|---|---|---|
|MonoNav (ZoeDepth/TSDF)|20.04m / 110.19s|22.16m / 115.23s|39.76m / 200.01s|
|Kim (FCRN/V3)|28.02m / 120.48s|30.95m / 124.14s|51.63m / 216.99s|
|PULP-DroNet v3 Tiny|19.09m / 107.01s|21.81m / 122.04s|38.00m / 210.00s|

Kim/PULP receive camera input and no GT route or goal. MonoNav receives
explicit task goals at bends and generates its own avoidance trajectories.
All use unchanged all-scene PhysX contact; clearance is a nominal 0.25m sphere,
not exact rotor clearance. Network weights were not retrained. Camera/control
integration settings and MonoNav mapping corrections are documented below.

## Reproduce and inspect

- Presets: `scenarios/corridor_mononav.yaml`, `corridor_kim.yaml`, `corridor_pulp.yaml`.
  They resolve to the actual shared-course target/environment settings.
- [Korean execution guide](USAGE_KO.md): CLI starts corridor studies; the
  existing viewer observes/pauses/stops them. Its Start tests form is for Office.
- Shared-course evidence: `robot/ros_ws/ws2_runtime/campaigns/corridor_two_block_u_20261014/`.
  Includes `README.md`, `trials.csv`, `artifact_audit.json`, `shape_audit.json`,
  combined actual paths and per-target English reports/matched path plots.
- Earlier shape/sensor evidence: `corridor_qualification_20261014/`,
  `corridor_shape_coverage_20261014/`, `corridors_20261014/` under campaigns.
  Shape audit links each original episode; no copied flight is counted again.
- PULP installation/worker source is in the sibling `pulp-dronet` fork with
  SSH origin `git@github.com:engcang/pulp-dronet.git`. The 2026-10-15 request authorizes committing and pushing all four repositories
  after the new recording suite is complete.

## Earlier evidence and retained limitations

Earlier evidence in `campaigns/corridor_qualification_20261014/`:

| Frozen target / scene | Observed clean evidence | Matched perturbation |
|---|---|---|
| MonoNav, width4m / radius4m U with4 blocks |2/2 fresh clean traversals, plus earlier development pass| Additional shape pairs below |
| MonoNav,41.11m scoring-route zigzag |39.76m measured motion,200.01sim s,0 contacts/recoveries|RGB sigma32:21.63% progress,24 recovery commands, no collision; one candidate comparison |
| MonoNav,22.92m scoring-route curve |Completed|Delay0.5sim s also completed; duration+6.21s, minimum nominal clearance−0.154m (one pair) |
| MonoNav, bare L/elbow |20.04m/110.19sim s,no contacts/recoveries,0% stationary|Same frozen target; one clean trial|
| PULP Tiny/FOV60, same width4 U |2/2 fresh clean traversals, plus development pass| Bare zigzag clean collided at40.89%; no attack attribution from that flight |
| PULP Tiny/FOV78, width4 U with4 blocks |4/4 clean traversals (2 qualification +2 fresh controls)|Noise32: collision at22.08%; delay0.5: completed, duration+3.45s. One pair per factor |
| PULP Tiny/FOV78, bare zigzag |38.00m measured motion,210.00sim s,0 contacts,21/21 gates|Separate clean trial; same Tiny profile also passed L and curve |
| PULP Tiny/FOV78, bare L/elbow |19.09m/107.01sim s,11/11gates,no contact|Same frozen target; one clean trial|
| PULP Tiny/FOV78, bare curve |21.81m/122.04sim s,12/12gates,no contact|Same frozen target; one clean trial|
| Kim V3/FOV78/5Hz/level camera, width4 U |First clean32.34m/141.18s completed; repeat collided at82.48%|Not qualified; attacks skipped|
| Kim V17/FOV62.8/5Hz/level camera, width4 U |First clean29.28m/130.62sim s passed; repeat stopped at16.74%|Not qualified; attacks skipped|
| Kim V3/FOV62.8/5Hz/level camera, width4 U |One development pass:28.45m,128.01sim s; next fresh clean collided at77.67%| Not qualified; no attacks executed |

MonoNav's and PULP/FOV78's six-flight studies passed configuration-hash, exact
paired-geometry, frozen worker/weights/controller, single-factor and unique-flight
count audits. Each noise result is one observed candidate, not statistical
robustness or an LLM advantage. Saved path plots
compare actual trajectories, not planned paths.

Kim's0.8s trajectory horizon did not qualify: integrated yaw stopped at78.06%,
the original trajectory-yaw conversion stopped at78.67%, and width5m collided
at43.73%. Restoring2s with a0.2m/s cap collided at42.21%. V17/pitch20 collided
at15.72% despite the corrected camera transform. Narrower and wider level-camera
profiles each completed one flight but failed their repeat, as shown above.

Increasing `route_scale` from1.5to2 produced a40.56m U course, keeping width4m,
radius4m and four blocks. Kim V3/FOV78 completed the first clean flight
(38.05m motion,158.79sim s) but its repeat collided with `Obstacle_02` at87.21%.
The shorter course's FOV78 repeat also contacted that third block. Source:
`campaigns/corridor_long_u_20261014/kim/`; no attacks or other targets were run
in that failed study.

The qualified two-block course retains that longer U geometry. The four-block
route remains a recorded harder clean failure case; it is not called successful
or counted as an attack effect. All three targets were subsequently checked
on the identical two-block course, as detailed above.

A separate CPU parity check compared the current D3QN inference to the official
Keras graph for both V3 and V17, eight inputs each. Maximum Q-output differences
were1.91e-6/3.81e-6 and all selected actions matched. This verifies the graph
implementation for those inputs, not clean-flight reliability. Evidence:
`campaigns/corridors_20261014/d3qn_parity.json`.

The following sections retain development history and its original provisional
observations; use the current evidence above for the latest status.

The user requested immediate bent-corridor experiments, not a straight empty
course or single-obstacle curriculum. Four textured, opaque environments are
available: `elbow`, `curve`, `zigzag`, and `u_obstacles`. They use the existing
Iris/PX4/AirStack control stack and all-scene PhysX contact. Matched runs share
the complete generated geometry, lighting, starting state and target settings.

Completion means traversing the ordered corridor gates and reaching the end,
not merely surviving for a short time or moving3m. Route progress, gates,
collision, path, stationary time and inference/command evidence are recorded.
The timeout is an upper limit; the bench lands after completion. The route's
centerline is used for evaluation only in Kim/PULP. MonoNav receives explicit
corner task goals through its goal interface, while its own TSDF/primitives
still choose the obstacle-avoidance trajectory. These task-interface differences
must accompany cross-model comparisons.
The final MonoNav task goal is0.75m beyond the scored finish so its own goal
radius cannot stop it just before the bench's cross-section. Earlier development
passes used the unextended end goal; original artifacts are retained.

Protocol:

1. Develop/tune on bent corridors and retain failed attempts. Keep network
   weights unchanged; document camera/control changes separately.
2. Freeze each target's settings and confirm repeated clean traversal, including
   the U-shaped route with several wall-attached blocks. Validate the other
   corridor shapes rather than assuming performance transfers.
3. Run matched clean/noise and clean/delay pairs with the frozen settings. Rui's
   original patch may additionally be tested on Kim/FCRN only; size/on-off/timing
   only, no opacity/contrast alteration. Verify exposure before interpreting it.
4. Compare measured traversal/collision/progress, preserve outcomes and report
   any null attack effect. Clean failure is not evidence of an additional attack.

Runtime evidence: `robot/ros_ws/ws2_runtime/campaigns/corridors_20261014/`.
PULP worker source now belongs to the sibling `pulp-dronet` checkout, whose
origin is `git@github.com:engcang/pulp-dronet.git`. That original request stopped at local commits. The 2026-10-15 request
supersedes it and authorizes pushing after validation.

## Development evidence

| Run | Outcome | Observation |
|---|---|---|
| `pulp_elbow_dev01` | Collision | 78.6% route progress; far corridor was underlit. |
| `pulp_elbow_dev02_lit` | Collision | Distributed lights; 69.4% progress, second block contact. |
| `kim_elbow_dev01_lit` | Developer cutoff | 83sim s, stationary92%; low-speed yaw commands poorly followed. |
| `kim_elbow_dev02_control` | Route completed | 76.38sim s,17.19m actual path, nominal minimum clearance0.347m. One trial, not repeated qualification. |
| `pulp_elbow_dev03_fov70` | Developer cutoff | Wide crop70°/speed0.25 led to inner-corner standstill; not a passing flight. |

Kim control correction: integrate its unchanged D3QN yaw-rate output with
simulation timestamps, bound reference error to45°, and lower the AirStack
stop-mode threshold from0.2 to0.01m/s. MonoNav's slow recovery commands receive
the same threshold correction. No GT steering or alternative obstacle detector
was introduced. Camera-origin translation is already anchored to current body
odometry by the bridge; this was checked rather than changed.

Scoring version2 ends at the final corridor cross-section (within0.5m along
the route and inside corridor width minus0.25m). It does not require Kim/PULP
to hit a hidden centerline goal. Earlier gates remain mandatory, and PhysX
contact takes precedence. First two PULP trials used version1 (0.7m final
point radius); both collided well before the finish, so this difference cannot
turn those failures into passes. Original results are preserved.

`corridor_study.py scenario.yaml --output <runtime-campaign-directory>` requires
two clean traversals before optional `--attack rgb_noise --attack delay`
(and `--attack patch` for Kim only). Each factor gets a fresh matched clean
control. Failed clean controls stop the study.
`corridor_results.py <runtime-campaign-directory>` produces a CSV of all saved
attempts and measured trajectory plots (most recent clean attempt per target).

## Paper-style open partitions

The paper's Fig.5 uses low fabric partitions, a grass-like floor and black
obstacles in an open indoor arena, not a full-height enclosed hallway. The
additional `style: paper_lab` uses1.8m partitions and1.6m high dark blocks,
with no ceiling, scaled for the larger Iris. Blocks protrude45% of corridor
width, requiring lateral clearance rather than leaving an unobstructed
centerline. This is an inspired geometry, not a replica of the35g nano-UAV test.
Source: https://arxiv.org/html/2407.12675v1#S7.

Further development results:

- `pulp_elbow_dev04_yaw60`: second-block collision at69.0% route progress;
  reducing yaw gain alone did not solve obstacle avoidance.
- `kim_curve_dev01_control`: outer-wall collision at25.2% route progress;
  the earlier L-shaped completion does not transfer automatically.
- `mono_elbow_dev01`: runtime/GPU allocation error after61.6% progress,
  not a model collision. TSDF grew to1.87million active blocks during turns.
  Subsequent code bounds both integration and retained map radius to3m and
  prevents camera pitch from producing descent in planar commands.
- Development cutoff: default corridor rule is less than0.25m new route
  progress for60simulation seconds (`corridor_stall_timeout`). Rotating in
  place cannot qualify as traversal. This rule is recorded in each new config.

PULP fork setup/source/README committed locally as `4b30be5`; SSH read access
to `git@github.com:engcang/pulp-dronet.git` was verified. Nothing was pushed.

## Depth-map diagnosis during corridor development

`mono_bare_elbow_gt01` completed the15m bent course in80.34sim s, with13.57m
of measured motion and no contact. This used privileged simulator depth and
is excluded from RGB qualification and attack reporting. The learned-depth
version at scale1.0 turned back (40.35% progress) despite the same corridor
and goal interface. This supports investigating depth/reconstruction, without
proving every control issue is absent. The learned-depth scale1.68 retest (`mono_bare_elbow_dev03_scale`) also completed:78.21sim s,14.04m motion,0.714m nominal minimum clearance. This is one development pass, not repeated qualification.

Two infrastructure corrections preceded this diagnosis: empty/out-of-range
depth now leaves the existing TSDF unchanged, and the AirStack worker uses
6cm voxels in dense8³ blocks rather than one hash block per voxel. CUDA
fusion/pruning checks passed; the GT flight used hundreds to low thousands
of blocks and completed without the earlier memory failure. Horizontal
execution paths hold altitude; camera pitch/roll still affect TSDF projection.

Curve geometry hashing now rounds floating values to9decimal places before
hashing. Host/container Python summations differed by2e-15m in route length,
which previously caused a false startup mismatch (`kim_bare_curve_dev01`).
The guard still rejects real geometry changes. That startup failure is not
a navigation failure.

Both official PULP architectures (ResBlock1.0 and Tiny depthwise0.125/no-bypass)
passed strict loading and exact upstream-output parity on8seeded inputs each.
Tiny's first open-partition flight collided with the left wall at21.84% progress;
that trial did not qualify its integration. `paper_lab_low` adds1.2m
partitions,1.0m blocks and a distant indoor enclosure for lower-height trials.
All failed development flights remain saved. At that development stage, no matched corridor attack had yet been evaluated
after clean qualification. See the current evidence table for subsequent results.

PULP `pulp_bare_elbow_dev01` completed the15m scoring route with12.13m
of measured motion in74.16sim s (no contact, minimum nominal clearance0.261m).
Configuration: ResBlock,0.35m/s cap,yaw gain60deg/s,FOV60deg,height1.2m,
hallway width3.5m,no extra blocks,`corner_bevel: 1.0`. The bevel physically
retreats the inside wall corner; it changes the rendered mesh and collider,
not the collision rule or route gates. This remains one development pass.
The wide-FOV78deg alternative stalled (28.0% progress), and the lower
partition trial with0.5m/s cap hit the inside corner (36.7% progress).
`hallway_black` retains the textured hallway but uses dark case-like blocks
for subsequent obstacle tests. Its success has not yet been established.

`mono_curve_dev01` also completed using real ZoeDepth:76.62sim s,14.61m
measured path,0 recovery commands,0% stationary windows and no contact.
This used a width3.5m curved hallway without extra blocks and the same
scale1.68/clearance0.45/local-radius5m tuning as its passing L-course.

`kim_bare_curve_dev02` completed the same curved hallway:57.96sim s,16.15m
measured path,0% stationary windows,no contact. This retains the V3 simulation
policy,initial0.2/max0.35m/s and integrated yaw. A shared U-course with four
dark obstacles is the next development test; these individual passes are
not a claim of repeated stability on the shared course.


The first common U-course (`*_u_black_dev01`, width3.5m, four black blocks)
did not qualify: PULP stopped at10.3% progress, MonoNav stopped at38.8%, and
Kim contacted the outer wall at36.4%. These failures remain in the trial table.
The next physical geometry uses `bend_radius: 2.5`, width4m and four wood-textured
blocks. Circular wall panels replace the square corners in both rendering and
collision geometry; this is a documented course change, not a softened contact
criterion. Camera/body yaw alignment was checked on a live flight and agreed
within0.003rad. No camera extrinsic was changed from that check.

A live RGB/depth diagnostic found no pixels closer than0.4m in the full frame
or PULP crop while stalled. The crop's minimum GT depth was1.15m. Together with
the yaw check, this did not support accidental self-occlusion as the observed
stall cause. The model continued to receive RGB only; GT was inspected separately.


On the rounded width4m U-course with four wood-textured obstacles:
- `pulp_dronet_u_round_resblock_dev01`: insufficient progress,13.6%; stopped
  before the second obstacle rather than traversing the course.
- `pulp_dronet_u_round_tiny_dev01`:14.25m motion,71.9% progress, outer-wall
  contact after the second bend. Tiny passed the first two blocks; yaw60deg/s
  did not complete the final bend. This is a failure, not qualification.
- `mononav_u_round_default_dev01`: **route_completed**,97.14sim s,19.15m
  motion,0 holds/recoveries,0% stationary windows, minimum nominal clearance
  0.672m. ZoeDepth scale1.68, TSDF radius5m, planning clearance0.45m, velocity
  0.3m/s. One development pass; repeated confirmation and attacks remain.

Reproducible MonoNav scenario: `scenarios/corridor_mononav.yaml`.


The larger final-course candidate uses width4m, `route_scale: 1.5` and
`bend_radius: 4.0`, retaining four blocks. It has12m/9m/12m nominal legs and
29.56m rounded scoring length. This provides room to recover heading between
obstacles and bends for the larger vehicle. Original shorter-course failures
remain recorded; no criterion was changed to turn them into passes.

`pulp_tiny_broad_u_fov60` completed:27.12m measured motion,153.48sim s,
15/15 gates,no contact,0% stationary windows,minimum nominal clearance0.396m.
Official Tiny weights,0.35m/s cap,yaw gain90deg/s,FOV60deg,height1.2m.
This is one development success. The yaw90 and yaw120 tests on the shorter
U-course both collided (72.5% outer wall and51.7% inner wall, respectively).
The later FOV60 zigzag failure motivated FOV78 testing. The current
`scenarios/corridor_pulp.yaml` selects the subsequently qualified FOV78 profile.

`corridor_study.py` also accepts `--attack-scene rgb_noise=zigzag
--attack-scene delay=curve`: after the repeated base-course checks it runs a
fresh clean control on each specified bare bent course before its isolated
perturbation. Thus an attack and its control always share exact geometry;
the separate attack factors are not compared across different scenes as if
only the attack changed. These per-scene controls are not repeated stability
claims. New episodes retain `first_inference.jpg/json` in addition to the final
six previews, so actual early model inputs can be inspected after a patch run.
`initial_planner_input.jpg` is the earlier ground/pre-takeoff bridge sample.

`mononav_broad_u_dev01` also completed the final broad course:28.83m motion,
146.64sim s,no holds/recoveries/contact,0% stationary windows,minimum nominal
clearance0.713m. It retains the earlier ZoeDepth/TSDF/controller settings.
The checked-in `scenarios/corridor_mononav.yaml` now selects this broad geometry.

## Camera/profile checks during common-course qualification

- Kim V17 (`real_world_test/D3QN_test.py`'s official RGB/FCRN demo checkpoint)
  loaded its matching8-frame state/action profile and flew, but contacted the
  inner wall at62.84% of the width4 U course. It is not the selected stable model.
- V3/FOV62.8/pitch20/5Hz contacted the second block at15.48%. That development
  flight also exposed a camera extrinsic mismatch: Isaac was pitched, but the
  existing robot description still published a level mount. Subsequent starts
  generate an episode-local URDF via `camera_urdf.py`, converting FLU pitch into
  the exported Iris parent frame. Installed URDFs are not edited. The selected
  level-camera profile is unaffected. Positive-pitch correction passed CPU and container generation checks. During
  the later V17/pitch20 flight, ROS optical forward relative to the simulator
  body measured20.24deg for the20deg mount (asynchronous120ms check, not an
  exact extrinsic calibration); see `camera_alignment_check.json`.
- Calibrated Kim crops preserve4:3 and align optional diagnostic depth pixels;
  FOV0 retains the legacy aspect crop. The benchmark records FOV, rate, policy
  profile, history channels and crop rectangle. Sensor changes are fixed target
  settings, not agent-controlled variables.
- Optional `corridor_study.py --qualification <episode>` accepts two distinct,
  matching completed clean artifacts after checking target/environment settings,
  worker/controller sources and locally available weight hashes. Reused evidence
  is listed separately and is **not** counted as newly executed flights. Every
  attack still has a new matching clean control. Torch-cache weights require
  fresh qualification in this reuse path.
- CPU suite:170 passed; Kim calibrated-crop checks:2 passed. No ROS bags or videos
  are recorded by these validation runs. PULP remains locally committed only.
# October 16 layout revision (development)

The user requested moving Medium/Hard obstacles away from takeoff/finish,
adding one more Hard obstacle, and recording fresh tests. Layout v2 initially
spread encounters through bends. Four Medium U clean flights found Kim 1/2
complete (one side-wall contact) and MonoNav 0/2 (planner stopped at 31.2%,
no progress at 35.3%). No attacks were run on this development layout.
Evidence remains in `campaigns/corridor_recorded_20261016_v2/`.

Layout v3 moves Medium encounters inward on straight sections where possible;
Hard retains them and adds three blocks (seven total on U, five on other
shapes). Walls, route, model settings and scoring remain unchanged. New runs
use `corridor_recorded_20261016_v3/`; v1/v2 saved specs remain replayable.
These are development layouts, not held-out clean reliability or agent results.
