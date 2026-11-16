# OutdoorNight: thermal adds supported proposals, with accuracy unresolved

16 November 2026, America/New_York. Completed the
[prospective reference-free protocol](outdoornight-matching-protocol-2026-11-16.md)
after [whole-export qualification](outdoornight-qualification-findings-2026-11-16.md).
All 971 anchors, 3,884 prediction rows and 1,942 choices remain. No training,
threshold/model/window search, new selector, fitted clock/axes or pose-error
scoring. Prior source reading reused; no new paper-body reading or complete
author-system reproduction.

## Main result and meaning

The unchanged learned RGB-first rule adds **288 supported pair proposals** to
RGB's 552, reaching 840/971 outputs. Thermal has substantially greater output
availability than RGB on this recording, unlike the earlier moving FlashLight
comparison. This is a difference in essential/cheirality-supported availability,
not established relative accuracy, physical thermal superiority or VO recovery.
Without a matching reference, the correctness of every added proposal is unknown.

| Learned policy | Supported outputs / 971 | Availability | Matcher calls | Projected pair seconds | Longest no-output run, anchors |
|---|---:|---:|---:|---:|---:|
| Fixed RGB | 552 | 56.85% | 971 | 265.213 | 53 |
| Fixed thermal | 796 | 81.98% | 970 | 273.697 | 9 |
| RGB-first, thermal on missing RGB | 840 | 86.51% | 1,389 | 383.278 | 9 |

There are 418 requested thermal calls: 288 yield a supported proposal and 130
yield none. The remaining RGB gap at the initial anchor has unsupported thermal
input and incurs no thermal call. Relative to RGB, calls rise 43.05% and projected
pair-processing time rises 44.52%. Compared with fixed thermal, fallback supplies
44 additional proposals at higher cost. No claim of matched-accuracy savings.

The output-status union contains 508 both-supported anchors, 44 RGB-only,
288 thermal-only and 131 neither-supported anchors. RGB-first equals this union
by construction: accepted RGB is never replaced, even if it might be wrong.
Higher availability does not establish safe abstention replacement or intelligent
quality selection. The consecutive-run metric counts declared half-second anchors,
not actual live estimator outage/recovery latency.

## Fixed temporal populations and SIFT control

| Learned population | RGB outputs | Thermal outputs | RGB-first outputs | Thermal attempts / added / no output |
|---|---:|---:|---:|---:|
| First third, 324 anchors | 188 | 202 | 220 | 135 / 32 / 103 |
| Middle third, 324 anchors | 140 | 321 | 323 | 184 / 183 / 1 |
| Last third, 323 anchors | 224 | 273 | 297 | 99 / 73 / 26 |

The middle-third output availability changes from 43.21% RGB to 99.69% fallback;
fixed thermal is already 99.07%. Qualification's fixed-time images show outdoor
scene structure in thermal with weak distant RGB visibility, while the sequence
starts/ends indoors. These thirds are prespecified time bins, not quantitative
illumination labels or a causal isolation of darkness, motion or field of view.

| SIFT policy | Supported outputs / 971 | Matcher calls | Projected pair seconds |
|---|---:|---:|---:|
| Fixed RGB | 273 | 971 | 35.599 |
| Fixed thermal | 387 | 970 | 25.583 |
| RGB-first | 556 | 1,668 | 54.397 |

SIFT backup adds 283 proposals in 697 thermal attempts, with 414 failed attempts;
projected time rises 52.80% over SIFT RGB, calls 71.78%. SIFT middle-third
RGB/thermal/fallback outputs are 43/197/217. Both frontends show a thermal
availability increment, but learned matching is much stronger in absolute output
coverage. This does not convert SIFT's larger relative gain into a stronger system.

## Matching support and limits of the output gate

All 419 learned RGB failures are insufficient cheirality support; thermal has
174 such failures plus the single unsupported input. Across supported calls,
learned RGB/thermal median cycle-accepted matches are 3,829/2,071, whereas median
cheirality inliers are 13/564. More cycle-consistent matches therefore do not
automatically provide more support under the fixed geometric gate. This is an
internal support diagnostic, not independent correspondence or motion truth.
SIFT thermal also retains 370 insufficient-match and 213 insufficient-cheirality
failures; SIFT RGB has 698 insufficient-cheirality failures. None is discarded.

All 971 anchors and fixed thirds are dependent observations from one exposed
recording. Unknown checkpoint training membership remains; no independent
generalization or IID significance claim. Own published calibration and export
timestamp bindings are nominal; raw exposure/header and physical reference truth
remain qualified. One thermal camera is not stereo MACThermal VO.

Each selected proposal keeps its camera's source/target timestamps and origin.
Rotation is conjugated to IMU axes; translation remains a unit direction in the
selected target camera because metric lever-arm transport requires scale. No
mixed-frame trajectory or persistent-state estimator is constructed.

Projected cost sums supported-call timings, including failed attempts, with CUDA
synchronization around learned pair processing. It excludes model startup,
residency, energy and live queue behavior. SIFT feature reuse differs under a lazy
schedule. Both complete control arms were actually executed; the fallback timing
is a projection over those calls, not an actual live lazy execution benchmark.

## Reproducibility and stopping decision

Restored exact compiler image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`
from EXP106 base / EXP112 delta after archive and consumed layer verification.
Original RGB/thermal checkpoints match EXP112 member hashes. Torch2.7.1+cu128,
NumPy2.2.6, OpenCV5.0.0 on RTX PRO5000 Blackwell; cuBLAS`:4096:8`, no TF32.
Legacy RGB seven-unused-key migration is unchanged; thermal load is strict.

Both native-size flows repeat bitwise; model states remain unchanged after
qualification and full inference. All 26,371 gray hashes, source/calibration/model
and protocol pins are checked before and after capture. Runtime qualification
passes 22 selector/projection checks. Seven CPU tests cover support/cost/null
accuracy, legacy reference-backed scoring and four semantic corruptions.
All 3,884 predictions lock before 1,942 choices and reference-free summaries.
Full control execution takes 624.554s including setup/checks; this is not fallback
latency. Full-run matcher calls are 1,941 per frontend, plus four learned
repeatability calls during qualification. No model update occurs.

Independent audit passes **49,455 checks**, reconstructing full row populations,
proposal geometry, routing, frame/time bindings, every status/count/third/run/
quantile/call/cost and absent accuracy semantics without importing the runner's
summary/selection calculations. It does not reproduce neural inference or certify
motion correctness. Generic comparison and auditor now support explicit
reference-free mode; frozen older studies retain their original source pins.

Evidence and PNG/PDF figure: `results/outdoornight-matching/`. Exact restoration
receipts: `/root/ore-ms-artifacts/runtime-recovery-2026-11-16/`. A verified local
incremental packet lives at
`/root/ore-ms-artifacts/outdoornight-matching-2026-11-16/`, excluding redundant raw
images, gray arrays, weights and runtime blobs while retaining their source/hash
prerequisites. No Box upload, author outreach or Git publication.

Stop this fixed configuration. Thermal supplies substantial complementary output
support here, but added-output accuracy remains the decisive missing evidence.
The next accuracy study requires a matching qualified OutdoorNight trajectory or
a separately qualified adverse-condition recording with its own reference. Do not
fit a reference, retune the gate or relabel unsupported accuracy as zero success.
