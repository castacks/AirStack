# Current research handoff

16 November 2026. Resumed checkpoint: paired fixed-priority replay complete.
Branch `ore-ms-orb-development`; latest committed base `a41fb5640`. Start with the
[completed paired priority comparison](docs/research/paired-priority-findings-2026-11-16.md)
and [research index](docs/research/INDEX.md). Historical proposed next steps do not
supersede this page.

## Current question and decision

The user selected **deployment characterization**: quantify when RGB-only,
thermal-only and existing fusion help, relating accuracy, availability, recovery
and computation to observable image conditions. A new algorithm or an unresolved
baseline failure is not an admission gate. Preserve positive, null and negative
results; do not assume a sensor must win.

Completed initial study: NGCLAB bright/changing/dim and real SubT-MRS SmokeRoom,
using native-size SIFT and existing learned temporal matching. These are monocular
frontend diagnostics, not full native stereo-system reproductions or fusion.
One thermal camera is not a thermal-stereo MACThermal input. Whole-span inputs,
query/geometry policies, source/models and prospective protocols are frozen.
No training or checkpoint/window/clock/threshold search was run; a fixed
status-only fallback was subsequently replayed without reference-error selection.

The initial study retains36,317 image decodes/5,036 predictions and independent
lighting/smoke audits. Thermal has higher learned support in changing/dim light;
whole SmokeRoom is mixed and its last third favors thermal. Support is not pose
accuracy; global contrast is not a validated selector. All failures remain.

The lighting pose-reference sanity check failed for interpretation, including
bright RGB: raw mocap is documented/exported as camera pose, but its actual camera
coordinate binding is unresolved. Primary results are retained. Do not fit an
axis flip, hand-eye calibration or clock to these predictions. Arithmetic audits
cannot establish physical ground truth. Reference gaps are separate from tracking
failures; common-reference cohorts were frozen before inference.

Report **frontend availability separately from pose accuracy**. An essential/
cheirality-supported pair does not prove correct correspondence or motion. A
hindsight union is an availability ceiling, not an achieved fusion gain. Smoke's
reference-free protocol uses the correct MEI/pinhole ray conversion and no supplied
trajectory. Subsequent nominal reference qualification is summarized below;
independent physical reference truth remains qualified.
These are exposed real recordings, with training overlap/generalization qualified.

The [spatial study](docs/research/deployment-spatial-support-findings-2026-11-16.md)
retains5,036pairs/221,555checks; support is not verified pose or a selector.

The [reference audit](docs/research/reference-contract-findings-2026-11-16.md)
found matching intrinsics/camera→IMU but no mocap-body→camera chain. Calibration
bags only have image/IMU streams; -176ms camera–IMU shift is not a mocap correction.
Fixed world axes cancel. No fitted axes/clock or repeat search absent new material.

The [angular-amount diagnostic](docs/research/reference-contract-findings-2026-11-16.md)
favors learned RGB across bright/changing/dim, but dim SIFT favors thermal.
All1684 rows/12,084 checks remain. Scalar angle cannot validate full orientation;
NGCLAB full poses remain unqualified without the missing mocap chain.

SmokeRoom's nominal reference contract is now qualified for supplied-reference
scoring: official IMU-pose convention, all 22,800 raw images reproduce frozen input
hashes/time bindings, raw gyro matches supplied CSV on all retained samples,
and 237 moving gyro checks give median/p95 SO(3) discrepancy 0.0385°/0.1082° without
fit. Two CSV-only samples at bag splits and three unsupported gyro checks remain
explicit. CSV lacks frame headers; independent origin/exposure metrology and
fused-reference truth are not certified. Thermal's zero bottom row is explicitly
normalized from its supplied upper 3×4; no calibration fit.

Full cached scoring is complete: 3,352 rows, 837 common-reference anchors, 483 paired
translation≥1cm anchors;45,488 independent checks. On 236 identical ≥2° rotation anchors,
learned RGB/thermal pose availability 93.6%/97.9%, R≤1° 64.0%/49.2%, R≤5° 76.7%/80.5%.
Thermal adds coarse available estimates; RGB is more precise. Nominal joint
R≤5°/directed-t≤15° is72.3%RGBvs51.8%thermal on483 translation anchors. Whole-video
static/small rotations make identity controls strong; do not infer a trajectory
or smoke-density-specific sensor winner. NGCLAB's distinct frame issue remains.

The fixed RGB-first fallback is complete: 1,676 locked decisions, zero inference,
18,565 independent checks plus 59 diagnostic checks. On the 236 moving anchors,
all 15 RGB gaps receive thermal outputs; four meet 1°, eleven meet 5°. Success
changes 64.0%→65.7% and76.7%→81.4%; four new outputs exceed5°. All15 events cluster
290.5–306.5s with obscured RGB views; no causal density labels. Zero-rotation backup
adds none here. Joint success on483 translation anchors changes72.3%→73.7%.
Whole learned projected matching cost rises42.7%, versus6.6% on the moving subset;
286/337 thermal attempts over the full recording return no proposal. Exact call
counts are measured; timing is a cached projection. Sensor-tagged pair proposals
retain camera translation origins; no stateful VO/metric trajectory claim.

FlashLight qualification and unchanged matching transfer are complete:14,715
images,541 anchors/2,164 prediction rows,538 common-reference,192 moving and519
translation anchors. Own SP1 calibration/March reference qualified; July raw bag
is different.80,634 input checks and37,971 comparison checks pass. No raw header or
independent physical truth certificate; lit samples do not establish zero lux.

Learned RGB has100% outputs on192 moving anchors: R≤1°91.7%,R≤5°96.4%; thermal
29.7%/57.3%. RGB-first adds nothing there. Whole recording:17 thermal attempts,
three accurate small-rotation additions,14 failures; no joint gain (452/519=87.1%
RGB/fallback vs53.8%thermal). All17 missing RGB rotations<0.30°; zero-rotation
control outperforms thermal backup on these low-motion gaps. Whole projected
matching overhead3.3%. SIFT instead favors thermal during motion:≤5°17.2%RGB,
57.3%thermal,38.0%fallback; preserve frontend dependence and incorrect additions.

[Paired priorities](docs/research/paired-priority-findings-2026-11-16.md): learned
SmokeRoom thermal-first121/195 versusRGB-first155/192 moving1°/5°; joint256vs356/483.
FlashLight thermal-first79/134 versus176/185; joint305vs452/519. No universal swap.
Oracle beyond better order: Smoke23/22moving,28joint; Flash2/3moving,6joint.
12verified tests/71,732audit checks pass; no inference/fitted gate or pooled targets.
Next: fixed Smoke failure-stratum characterization, Flash control; review old cue nulls.

## Prior evidence and closed routes

The [synthetic learned smoke control](docs/research/smokeseer-learned-findings-2026-11-16.md)
retains3495images/3465pairs: learned RGB97.92%,thermal98.53% rotation success;
the SIFT gap largely disappears. Clean Ficus regression remains. No tuning or
physical smoke VO; this closes that motivation, not thermal characterization.

Gascola-1 is day-labelled, not verified smoke. All 2835 thermal stereo pairs and
the archived frontend/runtime qualify, but its projection/export/condition contract
remains unresolved. RGB exports are not public. The 12-commit/maintainer audit is
closed absent new evidence; do not repeat it or silently guess intrinsics. See
[asset readiness](docs/research/smoke-darkness-readiness-2026-11-15.md).

The [SNU intake](docs/research/contribution-go-no-go-2026-11-15.md#natural-transition-readiness-16november)
found only public day inputs. The user has no private adverse bags; that closed
request-only route does not block accessible data. Recovery prior art remains.

Earlier RGB/thermal backend controls remain scoped: extra optimization explains
much of the night improvement; strongest short-day thermal gain is about 6.4%,
while the 599-interval day continuation gains only 0.140% with extra cost, and
longer night regresses. Read the
[long-day findings](docs/research/long-day-fallback-findings-2026-11-15.md),
[short-day controls](docs/research/thermal-strong-rgb-findings-2026-11-15.md) and
[night optimization control](docs/research/optimization-feedback-findings-2026-11-15.md).
Do not pool their distinct populations or infer universal RGB superiority.

## Constraints and recovery

Read [operating rules](AGENTS.md). Preserve frozen protocols/source, failed attempts,
missing outputs and qualification failures. Preflight every declared interval for
every arm before inference. Fallback and eligibility precede outcomes; never use
successful-prefix substitution or post-outcome rescue. Retain bright controls.

The [paper registry](docs/research/references/README.md) distinguishes targeted
paper/source reading from reproduction. The
[original-paper gap map](docs/research/original-paper-gap-map-2026-11-13.md)
retains broader representation, supervision and estimation questions.

Evidence is ignored under `results/deployment-characterization/` and
`results/deployment-smoke-frontend/`; raw inputs under
`data/deployment-characterization/`. Local verified packets live under
`/root/ore-ms-artifacts/`. No Box upload, author outreach or Git publication occurred.
Use exact runtime/source/checkpoint pins from each study, not historical runtime
success. Learned runs require deterministic cuBLAS workspace `:4096:8`.
