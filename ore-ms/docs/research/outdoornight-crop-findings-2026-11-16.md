# OutdoorNight crop transfer is positive for RGB-first and mixed across controls

16 November 2026 America/New_York. Completed the
[frozen unchanged-crop transfer](outdoornight-crop-protocol-2026-11-16.md), following
[SmokeRoom/FlashLight matched-count findings](matched-count-findings-2026-11-16.md).
Branch `ore-ms-orb-development`, committed base `4408b7fb8` plus frozen local sources.
The inherited [OutdoorNight gyro tier](outdoornight-gyro-findings-2026-11-16.md)
measures nominal angular agreement, not independently certified physical pose
accuracy. No translation scoring or pooling with supplied-reference records.

## Primary transfer result

The unchanged crop improves moving learned RGB and RGB-first gyro agreement over
original and one same-count draw. Fixed thermal is mixed: crop improves coarse
agreement but is one strict agreement behind count. Thus the hypothesis does not
hold on every planned camera/endpoint. RGB-first also remains below fixed thermal
on both moving angular bounds. Preserve partial transfer and coverage losses,
not a universal crop or priority recommendation.

Every triple below is original / same-count / crop. The fixed moving denominator
is242 for every count, with failures included. Within1°/5° means full relative
rotation agreement with each camera's own gyro interval under nominal extrinsics.

| Moving frontend | Policy | Outputs /242 | Within1° /242 | Within5° /242 |
|---|---|---|---|---|
| learned | rgb | 141 / 122 / 120 | 49 / 52 / 63 | 81 / 80 / 88 |
| learned | thermal | 235 / 226 / 231 | 158 / 166 / 165 | 212 / 211 / 215 |
| learned | rgb_first | 242 / 239 / 240 | 125 / 146 / 158 | 180 / 193 / 205 |
| learned | thermal_first | 242 / 239 / 240 | 164 / 177 / 171 | 218 / 222 / 222 |
| sift | rgb | 71 / 32 / 50 | 0 / 1 / 11 | 27 / 14 / 32 |
| sift | thermal | 146 / 145 / 138 | 92 / 103 / 104 | 141 / 140 / 134 |
| sift | rgb_first | 181 / 163 / 168 | 72 / 96 / 104 | 136 / 142 / 149 |
| sift | thermal_first | 181 / 163 / 168 | 92 / 103 / 110 | 154 / 147 / 155 |

Learned RGB-first strict agreement rises125→146→158/242 (51.7%→60.3%→65.3%);
coarse180→193→205/242 (74.4%→79.8%→84.7%). Count alone adds21strict/13coarse;
crop adds a further12/12, +5.0percentage points at either bound versus count.
Moving RGB-first output support falls242→239→240. Additional agreement is not
explained by a larger output denominator.

Thermal-only crop165/215agreements exceeds original158/212 but differs from
count166/211: −1strict,+4coarse. Thermal-first is stronger than RGB-first under
every variant, yet crop171strict is below count177; coarse is tied222.
Even original fixed thermal158/212matches cropped RGB-first's strict count and
exceeds its coarse205. These comparisons prevent a broad RGB-first or crop win.

The gain includes changes in routing. Cropped RGB strict63 plus95agreeing thermal
additions gives158; count52+94=146. Relative to count, +11strict RGB agreements
and +1strict thermal-addition agreement account for +12combined. Relative to
original49+76=125, these terms change by+14and+19. Addition sets differ, so this
accounting does not isolate a causal camera/rig component or prove calibrated
confidence. Accepted poor RGB estimates still block better thermal proposals.

## Paired gains and losses

| Learned moving comparison | Policy | Bound | Gained agreement | Lost agreement | Net |
|---|---|---|---:|---:|---:|
| original_to_bottom | rgb | 1° | 17 | 3 | +14 |
| original_to_bottom | rgb | 5° | 14 | 7 | +7 |
| original_to_bottom | thermal | 1° | 19 | 12 | +7 |
| original_to_bottom | thermal | 5° | 6 | 3 | +3 |
| original_to_bottom | rgb_first | 1° | 57 | 24 | +33 |
| original_to_bottom | rgb_first | 5° | 44 | 19 | +25 |
| original_to_bottom | thermal_first | 1° | 22 | 15 | +7 |
| original_to_bottom | thermal_first | 5° | 10 | 6 | +4 |
| count_to_bottom | rgb | 1° | 14 | 3 | +11 |
| count_to_bottom | rgb | 5° | 11 | 3 | +8 |
| count_to_bottom | thermal | 1° | 19 | 20 | -1 |
| count_to_bottom | thermal | 5° | 6 | 2 | +4 |
| count_to_bottom | rgb_first | 1° | 40 | 28 | +12 |
| count_to_bottom | rgb_first | 5° | 31 | 19 | +12 |
| count_to_bottom | thermal_first | 1° | 21 | 27 | -6 |
| count_to_bottom | thermal_first | 5° | 7 | 7 | +0 |

Cropped RGB-first gains57strict agreements but loses24versus original; gains40
and loses28versus count. Coarse gains/losses are44/19and31/19respectively.
Crop loses2moving outputs versus original; against count it gains2and loses1.
Fixed RGB crop gains24outputs but loses45old outputs versus original (net−21).
Thermal crop gains4and loses8 (net−4). Net scores do not certify casewise safety.
Full original→count transitions and every frontend/policy/cohort remain saved.

## Whole recording and thirds

Agreement denominators are964common-gyro anchors. Output support below uses all
971declared anchors; unsupported gyro cases remain explicit rather than scored.

| Learned policy | Full outputs /971 | Common within1° /964 | Common within5° /964 |
|---|---|---|---|
| rgb | 552 / 515 / 493 | 384 / 378 / 390 | 487 / 468 / 457 |
| thermal | 796 / 764 / 775 | 670 / 662 / 678 | 768 / 743 / 753 |
| rgb_first | 840 / 814 / 822 | 646 / 653 / 692 | 773 / 762 / 782 |
| thermal_first | 840 / 814 / 822 | 713 / 710 / 722 | 811 / 791 / 798 |

RGB-first improves whole-common strict646→692andcoarse773→782despite losing18
full-record outputs840→822. Versus count, crop gains39strict/20coarse agreements
and8full outputs. This differs from SmokeRoom's whole-coarse regression, and the
tiers remain separate. Fixed RGB crop loses30whole-common coarse agreements and
59full outputs; fixed thermal loses15coarse agreements and21full outputs versus
original. Thermal-first whole coarse811→798also regresses, despite strict713→722.

| Learned RGB-first temporal third | Eligible | Common outputs | Within1° | Within5° |
|---|---:|---|---|---|
| 1 | 318 | 216 / 199 / 202 | 178 / 165 / 181 | 212 / 196 / 202 |
| 2 | 324 | 323 / 321 / 323 | 237 / 258 / 265 | 287 / 302 / 303 |
| 3 | 322 | 296 / 288 / 292 | 231 / 230 / 246 | 274 / 264 / 277 |

First-third crop coarse212→202andoutputs216→202regress; middle/last coarse improve
287→303and274→277. Whole-record benefits do not erase the first-third cost or
establish condition-specific darkness labels. Camera endpoints and true physical
exposure semantics remain qualified; temporal thirds are fixed index strata.

SIFT also has substantial counterexamples: thermal-only moving coarse141→134,
RGB-first full-common strict324→318andcoarse501→460, with full outputs556→513.
Moving SIFT RGB-first104strict/149coarse under crop improves over count96/142and
original72/136, but its full-record losses rule out a general frontend-wide win.
The complete SIFT thirds and fixed-camera/priority comparisons remain in summary.

## Same-endpoint zero and thermal additions

| Learned moving variant | Thermal additions | Within1° | Within5° | Same-endpoint zero within1° /5° |
|---|---:|---:|---:|---|
| original | 101 | 76 | 99 | 0 / 13 |
| count | 117 | 94 | 113 | 0 / 16 |
| bottom | 120 | 95 | 117 | 0 / 15 |

Cropped thermal additions95/117agreements versus zero0/15show nonzero angular
information at their actual endpoints under the gyro contract. Three of120added
outputs exceed5°, and25exceed1°; neither is hidden. Changed addition sets are not
a matched causal estimate. Across the fixed moving cohort zero rotation agrees
0/242within1° in either camera,61/242RGB and60/242thermal within5°. Whole/static
zero controls and all SIFT additions are retained without identity rescue.

## Inputs, execution and independent checks

Full preflight covers11652variant rows,3884raw rows,1942sensor intervals and1944
unique uint8 endpoint images with exact own-image hashes and640×480RGB/640×512
thermal shapes. All declared endpoints, including unsupported ones, were checked.
Own calibration roundtrip maxima are below8.2e-13pixels. The unchanged native
nearest-row recipe keeps0–319RGB/0–341thermal at both endpoints; canonical4decimal
rounding and .05px roundoff guard unchanged. Count uses bottom's prefit count and
the unchanged seed0SHA256 tag with OutdoorNight as recording key, one draw only.
No change to crop size, correspondence extraction, calibration or ranking seed.

All3884original rows exactly replay28840pose fields before masked fitting in the
recovered image44ff9a1e6fa9… (OpenCV5.0.0/NumPy2.2.6, one CPU OpenCV thread).
Original controls then inherit exact fields rather than being replaced with new
predictions. The unchanged pose helper makes6313new essential fits;298masked rows
reuse identical retained-index subsets and1157need no solver under original
input/minimum-match gates. Original baseline replay uses3512actual fit calls.
All11652predictions and5826status choices lock before gyro disagreement scoring.
No new inference, feature detection, model/threshold/seed/clock/bias/axis fit.

The necessary adapter reuses frozen native/mask/index/count/pose and status-policy
helpers, plus unchanged gyro-policy evaluation. Only variant binding, preflight,
parity/origin receipts, gyro-row adaptation and paired transitions are new. It
does not edit frozen parent files. Saved matching-cost fields are inherited cache
projections, not measured end-to-end intervention latency or a compute benefit.
Existing matching/gyro/calibration/reference-source identities stay pinned;
19inherited helper files also match the preceding matched-count source freeze.

Independent audit passes150805checks. It rebuilds native MEI/pinhole pixels with
analytic distortion (separate from production OpenCV projection), crop subsets,
heap-based SHA ranking, exact parent fields/origin receipts, mask/pose gates,
properR/unit-t, SVD E/R/t consistency at1e-5, quaternion gyro support/targets,
all angular errors, original complete score/policy parity, four policies and
endpoint bindings, six cohorts, zero/addition controls and every paired change.
The earlier rawE-equality audit failure stays historical; raw E/poses are unchanged.
Inherited964common/242moving cohorts are independently reconstructed; all14gyro-
unsupported sensor intervals and the one input-unqualified sensor interval remain.

All7 tests pass: independent native roundtrip/distortion, endpoint/boundary/count
and empty/full/rank/order controls, a full unmodified-valid audit, and rehashed
subset, gyro-disagreement and paired-transition corruptions rejected. Preflight
ran3finite tests with4full-result tests explicitly skipped until scoring; both
logs are preserved.

Evidence: `results/outdoornight-crop/`, full locks/rows/changes, execution/test logs,
and `run-001/primary-results.png`/PDF with plot-input hashes. A member-readback
verified incremental packet is under `/root/ore-ms-artifacts/outdoornight-crop-2026-11-16/`.
Original matching/raw gyro/gray images/runtime and prior study packets remain
explicit pinned external dependencies. No commit, push, Box upload or adoption.

## Decision and next bounded question

Close this unchanged recipe as **partial positive transfer**, not uniform success
on every planned camera/endpoint. RGB-first moving gains exceed one count control,
while fixed thermal/thermal-first expose nulls and strict regressions, and output
coverage and several whole-record/third endpoints worsen. Do not add a seed,
mask/identity/clock/camera gate or relabel gyro consistency as physical pose truth.
One draw gives no sampling variance; exposed temporal samples give no independent
held-out or IID significance claim. Rig versus scene/spatial distribution/RANSAC
sample effects remain confounded. This is a temporal monocular pair diagnostic,
not native stereo/inertial VO, metric translation or persistent-state recovery.
Prior OpenCV/SubT-MRS reading is reused; no new paper-body review or novelty claim.

The next finite characterization should measure how the fixed variants' missing
and disagreeing outputs cluster in time, across the three complete recordings
with their reference tiers kept separate. Existing fixed1°/5° bounds and four
status policies suffice; freeze run-boundary/reference-gap handling before analysis.
That would quantify whether average gains shorten or worsen sustained unusable
intervals, without fitting a rescue selector or claiming stateful VO recovery.
This transfer study stops after findings and its verified packet.
