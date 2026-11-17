# Crop gains exceed one fixed count control; coverage losses persist

16 November 2026 America/New_York. Completed the
[frozen matched-count protocol](matched-count-protocol-2026-11-16.md), following the
[crop ablation](pre-essential-exclusion-findings-2026-11-16.md). Branch
`ore-ms-orb-development`, committed base `4408b7fb8` plus frozen local studies.
All original and crop outputs remain exact; only the count control is newly fit.
This is an exposed engineering comparison against nominal supplied references.

## Primary comparison

The fixed lower-third exclusion beats the one-draw same-count control on learned
moving strict/coarse rotation and full translation-cohort joint success in both
records, for each fixed camera and RGB-first. Count reduction alone does not
reproduce these crop gains in this fixed comparison. It explains part of the
SmokeRoom RGB-first improvement, so the earlier gain cannot all be assigned to
region targeting. No universal crop adoption or causal rig-only conclusion.

Each cell below is original / same-count / crop. Failures stay in the original
fixed denominators. R1 means own-camera R≤1°, R5 means R≤5°; joint means R≤5° and
directed translation angle≤15° on the original both-displacements≥1cm cohort.
Joint is not metric body translation or trajectory accuracy.

| Recording | Learned policy | Moving n | Moving outputs | Moving R1 | Moving R5 | Translation n | Joint |
|---|---|---:|---|---|---|---:|---|
| FlashLight | rgb | 192 | 192 / 192 / 192 | 176 / 176 / 180 | 185 / 185 / 192 | 519 | 452 / 451 / 465 |
| FlashLight | thermal | 192 | 168 / 145 / 157 | 57 / 47 / 66 | 110 / 108 / 136 | 519 | 279 / 278 / 316 |
| FlashLight | rgb_first | 192 | 192 / 192 / 192 | 176 / 176 / 180 | 185 / 185 / 192 | 519 | 452 / 451 / 465 |
| FlashLight | thermal_first | 192 | 192 / 192 / 192 | 79 / 88 / 97 | 134 / 153 / 171 | 519 | 305 / 323 / 345 |
| SmokeRoom | rgb | 236 | 221 / 206 / 215 | 151 / 151 / 182 | 181 / 177 / 200 | 483 | 349 / 350 / 392 |
| SmokeRoom | thermal | 236 | 231 / 228 / 232 | 116 / 116 / 177 | 190 / 191 / 223 | 483 | 250 / 263 / 351 |
| SmokeRoom | rgb_first | 236 | 236 / 236 / 236 | 155 / 168 / 196 | 192 / 204 / 221 | 483 | 356 / 371 / 410 |
| SmokeRoom | thermal_first | 236 | 236 / 236 / 236 | 121 / 123 / 180 | 195 / 198 / 227 | 483 | 256 / 274 / 360 |

SmokeRoom RGB-first strict gains are +13 from original to count and another +28
from count to crop; coarse gains are +12 and +17; joint gains +15 and +39.
Crop versus count is +11.9 percentage points strict, +7.2 coarse and +8.1 joint.
FlashLight crop versus count is +4 strict, +7 coarse and +14 joint successes.

The count intervention changes status-based fallback as well as camera estimates.
On SmokeRoom, fixed RGB strict successes stay151 and fixed thermal stay116 under
count, while RGB-first strict increases155→168. RGB moving outputs fall221→206,
changing which camera supplies the chosen result. Improved fallback counts alone
do not establish that both camera estimators became more accurate. With the crop,
fixed RGB strict reaches182 and thermal177; both are stronger than count here.

## Paired losses and full-record tradeoffs

These compare count→crop on identical original cohorts, including missing outputs.

| Recording, learned RGB-first | Endpoint | Eligible | Crop gained | Crop lost | Net |
|---|---|---:|---:|---:|---:|
| FlashLight | R1 | 192 | 11 | 7 | +4 |
| FlashLight | R5 | 192 | 7 | 0 | +7 |
| FlashLight | joint | 519 | 27 | 13 | +14 |
| SmokeRoom | R1 | 236 | 35 | 7 | +28 |
| SmokeRoom | R5 | 236 | 27 | 10 | +17 |
| SmokeRoom | joint | 483 | 52 | 13 | +39 |

The crop loses previously correct individual cases even in the positive moving
comparisons. Original→variant transitions are also retained in full; the new
control does not replace or erase the parent's paired losses.

| Full recording, learned RGB-first | Declared n | Outputs original / count / crop | Common R1 original / count / crop | Common R5 original / count / crop |
|---|---:|---|---|---|
| FlashLight | 541 | 527 / 527 / 523 | 495 / 491 / 499 | 517 / 517 / 520 |
| SmokeRoom | 838 | 551 / 535 / 519 | 452 / 450 / 469 | 507 / 503 / 503 |

Common-reference denominators remain538 FlashLight and837 SmokeRoom. Relative to
count, the crop loses16 net SmokeRoom outputs while whole-common coarse success
is tied503/837; its +17 moving coarse successes are offset by −17 outside the
moving cohort. Relative to original, SmokeRoom still loses32 outputs and4whole-
common coarse successes. FlashLight crop loses4 net outputs versus either control.
These costs remain grounds against unconditional replacement or a tuned identity/
static/camera rescue. Full thirds, both orders and each fixed camera are saved.

## SIFT controls and non-dominance

Same original / count / crop order:

| Recording | SIFT policy | Moving outputs | Moving R1 | Moving R5 | Full translation joint |
|---|---|---|---|---|---|
| FlashLight | rgb | 105 / 69 / 124 | 4 / 3 / 61 | 33 / 27 / 106 | 155 / 106 / 253 |
| FlashLight | thermal | 134 / 136 / 135 | 30 / 32 / 32 | 110 / 112 / 114 | 177 / 181 / 179 |
| FlashLight | rgb_first | 157 / 149 / 167 | 14 / 19 / 68 | 73 / 90 / 139 | 206 / 201 / 290 |
| FlashLight | thermal_first | 157 / 149 / 167 | 32 / 32 / 44 | 118 / 116 / 140 | 199 / 194 / 226 |
| SmokeRoom | rgb | 162 / 131 / 173 | 24 / 21 / 149 | 85 / 62 / 168 | 144 / 87 / 304 |
| SmokeRoom | thermal | 134 / 127 / 130 | 60 / 63 / 65 | 100 / 105 / 107 | 129 / 134 / 138 |
| SmokeRoom | rgb_first | 198 / 174 / 213 | 30 / 34 / 163 | 107 / 95 / 201 | 167 / 128 / 341 |
| SmokeRoom | thermal_first | 198 / 174 / 213 | 64 / 70 / 128 | 133 / 123 / 186 | 200 / 166 / 297 |

Large SIFT RGB crop gains are not reproduced by uniform count reduction. The
thermal-only FlashLight SIFT joint endpoint is a counterexample to dominance:
count181/519 versus crop179/519 (original177/519). Preserve that result rather
than selecting the best count/crop arm per endpoint. SIFT controls do not displace
the stronger learned baseline or establish modality-independent superiority.

## Execution and validation

All5516 original rows (541 FlashLight and838 SmokeRoom anchors ×2cameras ×2frontends)
are retained. Three variants produce16548 preflighted geometry rows and8274
status-only choice rows. Each count target comes from the parent's image-only
prefit crop count. A single SHA256 ranking over original match indices, fixed
seed0tag and recording/frontend/camera/index key chooses exactly that many;
ascending original order is restored. No pixel, flow, score or label guides it.
No alternate draw, match extraction, neural inference or parameter search occurs.

There are11032 inherited parent rows with exactly equal fields plus an explicit
origin field,5516 new count rows,4295 actual new solver calls and950 exact-subset
cache reuses. The other271 count rows require no solver call under the original
input/minimum-match gates. Both qualified failures and unsupported inputs remain.
All geometry and status choices lock before labels. Same exact recovered image,
OpenCV5.0.0/NumPy2.2.6, seed0, RANSAC .999/1px/1000iterations,8matches and8positive-
depth gate. One OpenCV CPU thread; no GPU/model execution or end-to-end cost claim.

The adapter reuses the frozen pose/index helpers and temporarily scopes variant
names for unchanged parent selection/scoring, restoring globals on exit. Frozen
parent source is not edited. An independent auditor uses heap-based hash ranking,
checks exact control identity, all subset bindings and origins, proper rotations/
unit translations, original masks/gates, own reference errors, all seven cohorts,
four policies, original→variant transitions and count→crop paired contrasts.
It passes239887 checks. SVD E/R/t consistency uses the established1e-5 contract
from the outset; the parent's literal rawE-equality failure remains historical,
with no pose/E edits or retrospectively relabelled raw gate.

All7 tests pass, including finite empty/full/count/order/key controls, scoped
helper restoration after an exception, a complete unmodified-valid audit, and
rehashed subset-fingerprint, score and paired-contrast corruptions rejected by the
auditor. Before fitting, the3 finite tests passed and4 full-result tests were
explicitly skipped until results existed; both logs remain.

Evidence: `results/matched-count-control/`, including frozen preflight/source
pins, all predictions/choices/scores/contrasts, execution and test logs, and
`run-001/primary-results.png`/PDF with plot-input hashes. The independently
member-verified incremental local packet lives under
`/root/ore-ms-artifacts/matched-count-control-2026-11-16/`. Parent raw matching,
images/calibration/reference data and runtime remain pinned external dependencies.
No commit, push, Box upload, model training or deployment adoption occurred.

## Interpretation and next bounded question

One deterministic draw does not estimate sampling variance or support a general
statistical claim. Crop versus count changes spatial distribution, retained
correspondence identities and RANSAC sample effects; it does not isolate physical
rig removal from floor/scene exclusion. These are exposed nominal camera-reference
diagnostics, with inherited timing/calibration/training limitations, not independent
physical truth or full stereo/inertial VO. OutdoorNight's nominal gyro tier is
separate and was not pooled here.

Prior OpenCV ROI/decomposition and SubT-MRS reading is reused; no new paper-body
reading, sampling theory or algorithm novelty is claimed. The protocol's exact
recipe closes positively for moving/joint comparison, with retained coverage losses.
Do not add seeds, crop tuning or rescue gates to this run. The next bounded study
can test the unchanged crop on the already cached OutdoorNight matches, first
freezing whole-span native-coordinate/mask support and its own nominal-gyro
rotation endpoints. No translation claim or cross-tier pooling; that transfer
requires its own protocol before fitting. This study stops at findings and packet.
