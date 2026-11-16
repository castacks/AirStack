# FlashLight unchanged matcher and RGB-first transfer findings

16 November 2026. Completed [prospective transfer protocol](flashlight-transfer-protocol-2026-11-16.md)
on the [qualified March export](flashlight-qualification-findings-2026-11-16.md).
No training, parameter/clock/extrinsic search, new selector or error-based switching.
Prior dataset/MAC-VO/thermal reading reused; no fresh full-paper reading or
full author-system reproduction. These are calibrated monocular temporal proposals.

## Decision and primary result

The learned RGB-first fallback has **no moving-cohort gain** on this recording:
RGB already emits a valid estimate at all192 fixed paired-moving anchors. Thermal
is less accurate here. This retains a null transfer beside the positive localized
[SmokeRoom backup result](smoke-rgb-first-findings-2026-11-16.md); neither recording
establishes a universal sensor winner or a sensor-physics-only causal comparison.

| Learned policy | Outputs /192 | Rotation≤1° /192 | Rotation≤5° /192 |
|---|---:|---:|---:|
| RGB | 192 (100%) | 176 (91.7%) | 185 (96.4%) |
| Thermal | 168 (87.5%) | 57 (29.7%) | 110 (57.3%) |
| RGB-first, thermal on missing RGB | 192 (100%) | 176 (91.7%) | 185 (96.4%) |

These are the same192 anchors, with both reference rotations≥2°, missing outputs
counted as failures. Median rotation error conditional on output is0.4643° for
RGB/fallback and1.9971° for thermal. No moving thermal requests occur. The hindsight
union is178 at1° and188 at5°; the status-only rule retains accepted but sometimes
wrong RGB, so those oracle counts are not deployment gains or a new gate proposal.

On all519 paired displacement≥1cm anchors, joint R≤5°/directed-t≤15° succeeds
for learned RGB452/519 (87.1%), thermal279/519 (53.8%), fallback452/519 (87.1%).
Output counts are517/484/518 respectively. Conditional translation-direction
medians are5.7131°/10.5933°/5.7152°. No joint improvement is established.

## Whole recording, requests and static-motion control

| Learned policy, common-reference cohort | Outputs /538 | R≤1° /538 | R≤5° /538 |
|---|---:|---:|---:|
| RGB | 521 (96.8%) | 492 (91.4%) | 514 (95.5%) |
| Thermal | 488 (90.7%) | 324 (60.2%) | 430 (79.9%) |
| RGB-first | 524 (97.4%) | 495 (92.0%) | 517 (96.1%) |

There are17 thermal attempts, three valid additions and14 failed attempts. All17
RGB reference rotations are below0.30°; none lies in the paired-moving cohort.
The additions occur at96.0,98.0 and256.0 nominal seconds, with thermal rotation
errors0.1189°,0.0482° and0.6560°. All pass1°, but the first two are translation-
ineligible. The last is translation-qualified and has59.35° directed-t error,
failing the joint bound. Thus report three correct rotation additions, **zero
joint additions**, and one incorrect joint proposal among one eligible addition;
do not misclassify the two ineligible directions as translation failures.

The predeclared RGB-or-zero rotation-only diagnostic gets509/538 at1° and531/538
at5°, exceeding actual thermal fallback's495/517. All17 missing RGB outputs have
small enough reference rotation that zero succeeds at both bounds. This does not
supply translation or prove safe persistent-state behavior. It shows why the
three newly correct rotations cannot be presented as recovered larger-rotation motion.
The same diagnostic adds nothing on the192 moving anchors, where RGB never drops.

All541 declared anchors remain: learned RGB/thermal/fallback outputs524/490/527;
correct rotation counts are unchanged because the first three lack common reference.
The initial thermal input remains unsupported, so fixed thermal makes540 calls.
The three policies have common-reference≤5° counts178/160/178 in the first third,
158/133/160 in the second and178/137/179 in the third (populations178/180/180).
Thirds are time bins, not darkness or smoke-density labels. Full failures and
strata remain in `summary.json`; none was trimmed after seeing results.

## Frontend dependence: retain the SIFT countercase

On those identical192 moving anchors, SIFT results differ strongly:

| SIFT policy | Outputs /192 | R≤1° /192 | R≤5° /192 |
|---|---:|---:|---:|
| RGB | 105 (54.7%) | 4 (2.1%) | 33 (17.2%) |
| Thermal | 134 (69.8%) | 30 (15.6%) | 110 (57.3%) |
| RGB-first | 157 (81.8%) | 14 (7.3%) | 73 (38.0%) |

Thermal beats SIFT RGB, but RGB-first remains below fixed thermal at both bounds:
the validity gate accepts wrong RGB estimates. Of87 thermal attempts,52 return
proposals: ten pass1° and40 pass5°, while42/12 fail those respective bounds.
Thirty-five attempts produce no output. Joint moving successes are10/45/26 for
RGB/thermal/fallback. RGB-or-zero reaches4/43 at1°/5°, so thermal contributes
motion information beyond this static control with SIFT, unlike the learned case.

Across519 translation-qualified anchors, SIFT joint successes are155/177/206
(29.9%/34.1%/39.7%). The124 eligible added proposals yield51 joint successes and
73 failures. Across538 common-reference anchors, SIFT outputs350/401/475 and
R≤1° counts188/154/231, R≤5° counts277/356/380. Preserve these coarse/strict and
availability/accuracy distinctions. A large gain over weak SIFT RGB is not evidence
of benefit over the learned RGB baseline, nor proof of full thermal-system VO.

## Matching budget and condition limits

| Learned population | RGB calls | Extra thermal calls | RGB seconds | RGB-first seconds | Projected increase |
|---|---:|---:|---:|---:|---:|
| All541 anchors | 541 | 17 | 148.50 | 153.45 | 3.3% |
| Same192 moving anchors | 192 | 0 | 53.43 | 53.43 | 0% |
| Same519 translation anchors | 519 | 2 | 142.17 | 142.74 | 0.4% |

These sum synchronized measured per-pair times from the full control computation;
they project warm lazy matching, not an actual lazy schedule or robot latency.
Each learned call batches both temporal directions. Failed attempts count, initial
unsupported thermal input does not. Model startup/resident memory, routing, sensor
energy and stateful estimation are excluded. SIFT all-anchor calls increase541→730,
projected matching18.89→25.18s (+33.3%); moving calls192→279,6.98→9.81s (+40.5%).
SIFT cache reuse would differ under a lazy schedule, making those times approximate.

Fixed-time input images were selected before predictions and remain visibly lit;
FlashLight is officially labelled changing illumination, not demonstrated zero
illumination. Global RGB means are86.09–143.76 and include bright foreground/rig
pixels plus camera processing; these are not calibrated lux. The recording does
not answer the no-light deployment question. Motion, field of view, image content,
preprocessing and model training can all affect this comparison; no causal claim
isolates one of them from this single sequence. Intervals are temporally correlated,
with no IID significance or held-out training-generalization claim.

Own SP1 published calibration, exact exported timestamps and frozen camera-relative
references were used. Reference/gyro consistency is nominal, with yaw-dominated
excitation, missing raw image-header proof and non-independent fused reference.
Selected rotations are conjugated into IMU axes; unit translation stays at its
selected target camera origin because unknown scale prevents metric lever-arm
correction. No mixed-frame trajectory is constructed. Incorrect additions could
harm a real persistent estimator; monotonically increasing success counts are not
an abstention-safety or full VO recovery result.

## Reproducibility and stopping decision

Generic runner `tools/subt_export_comparison.py` composes unchanged source-pinned
native inference/grid/cycle/SIFT/essential helpers and pure fixed fallback chooser.
It adds configurable camera/export contracts, matcher-only input rows and separate
prediction/selection/scoring locks; frozen older scripts remain unchanged. Reuse
this configurable adapter for qualified future exports rather than copy runners.

Exact runtime image SHA256:
`44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`;
NumPy2.2.6/OpenCV5.0.0/Torch2.7.1+cu128 on RTXPRO5000Blackwell. Deterministic
cuBLAS`:4096:8`. Qualification passes22 selector/projection-equivalence checks,
bitwise native480×640 and512×640 repeated flows, and unchanged model states.
The existing RGB legacy loader removes seven unused upstream keys and initializes
a deterministic sampling-grid buffer, as in prior qualified runs; loading reports
and checkpoint hashes are retained. No learned weight update occurred.

All2,164 rows are locked before1,082 decisions, which are locked before scoring.
Actual full-run pair computations are1,081 SIFT and1,081 learned, plus four learned
repeatability calls in qualification. Two unsupported prediction rows remain.
Full-run elapsed time350.63s includes control computation/startup and is not the
lazy-policy latency. `tools/audit_subt_export_comparison.py` passes37,971 independent
checks: frozen inputs/row completeness, direct SO(3)/directed-t errors, status-only
routing, sensor/frame/timestamps, conjugation invariance, every stratum/control/
identity/cost and correct/incorrect additions. Seven derived request checks also
pass. Audit performs no new model inference; it does not certify physical truth.

Evidence is ignored under `results/flashlight-transfer/`: config/freezes,
qualification, matcher inputs, full predictions/lock, decisions/lock, scores,
summary/completion/audit, request lists/diagnostics/logs and static accuracy/time
figures. The moving-fallback list is correctly empty. Source/calibration/reference
qualification evidence remains under `results/flashlight-qualification/`.
The incremental recovery packet is
`/root/ore-ms-artifacts/flashlight-transfer-2026-11-16/`; its manifest verifies every
archived member by readback and records external data/model SHA pins without
copying raw images or checkpoints. No Box upload, author outreach or Git publication.

Stop this configuration without a threshold/model/selector rescue. The next
condition intake is the public OutdoorNight release: verify the actual images,
recording dates, own calibration and supplied reference over the full span before
any matching, using existing generic qualification/comparison adapters. A title
alone will not establish darkness. Preserve this lit-recording null result beside
the SmokeRoom localized positive result when characterizing deployment regimes.
