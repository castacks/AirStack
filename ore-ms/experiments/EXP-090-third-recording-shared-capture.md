# EXP090: third-development-recording shared capture

Declared2026-10-31 America/New_York, base1e60c400f plus new independent audit
binding. User explicitly approved the bounded collection in the temporal-gate
draft. No gate fitting or reserve access is included.

Recording: already-exposed20251027/1_salon_mac_cov_day. Exact three EXP023 day
prepared manifests:day-original/day-early/day-late,20paired groups each. Historical
full-manifest offsets348/306/1132 are descriptive; prepared manifests are already
sliced and every new runner starts at0. Preserve all source rows, timestamps,
archived MSA thermal pixels/camera metadata and exact matching source RGB files.
No re-rectification or window replacement.

Six fresh independent v3 expert controls and three unchanged dual-expert shared
captures, one group lag and final pre-drain boundary, same runtime/config/seed.
Freeze sources, inputs, checkpoints, image/environment and commands before first
inference. New runtime may differ from old EXP023; require fresh same-runtime
controls instead of claiming historical numeric equivalence. Preserve failures
and all fixed windows. No retry selected for better outcomes.

Audit exact source hashes, all decoded inputs, controller schedule/prefixes and
array/event parity against fresh controls. Report all57decisions/114camera slots,
source availability, current/two-prior feature completeness and original fixed
fallback/history behavior. Any conditional pose labels use only the frozen
archived recorded-SLAM source and existing nominal frame/model rules, with
scorer separated from predictor. No fitted alignment, independent physical,
held-out, calibrated probability, live-latency or new quality-gate claim.

## Recovery and frozen runtime

The EXP023 recovery archive SHA256 matches
`20784996acf1e7a7256400fa041945ff0274e56ed92242fa0fa3d4c1fb8c4dca`;
all 1019 archive-member hashes pass. Restored 140 bounded prepared input,
plan/report and calibration/reference files. The exact three 20-row manifests
match the archived EXP023 preparation plan and original thermal reports.

That recovery contained no RGB images. Recovered the matching 120 source RGB
PNG members plus two camera-info files from the original
`1_salon_mac_cov_day.zip` through bounded authenticated ranges, using the existing
RemoteZIP reader. ZIP member CRC and local SHA256 registration pass; range reads
total 77,549,561 bytes. The 8,856,044,347-byte source ZIP was not wholly downloaded
or rehashed here. Do not equate its historical full-archive hash with a new
whole-archive verification. Copied original RGB bytes/info unchanged into each
prepared dataset; all thermal PNGs and thermal camera metadata remain archived
MSA outputs. No image regeneration, intensity recipe change or rectification fit.

Original checkpoint ZIP SHA256 and ZIP CRC pass, and both individual weights
match the pinned RGB/thermal hashes. Shared Box resolution retained the original
shared-link header only on Box API requests; OAuth headers were not forwarded
to redirected download hosts. Credentials, signed URLs and weights are excluded
from the ordinary result backup. Original model source remains models/README.md.

The GPU is an RTX PRO 5000 Blackwell with 48,935 MiB reported memory. Built
repository Dockerfile.solver-vo into image
`sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1`.
Runtime: Torch2.7.1+cu128, PyPose0.9.5, NumPy2.2.6, SciPy1.17.1,
OpenCV4.10.0. Full package freeze/build logs are preserved. Startup emits the
previously observed driverInitFileInfo/result11 diagnostics; runner imports,
CUDA matrix multiply, all decoded inputs and all nine inference runs succeed.
These checks do not certify live throughput. This is a fresh runtime, so no
historical EXP023 bitwise execution equivalence is asserted.

Before first inference, capture-plan.json freezes the image ID, environment,
334 source/config hashes, 260 input/provenance hashes, both checkpoint hashes,
all source rows and nine commands. Original declaration text is preserved in
`declaration.md`; this report's later results do not replace that snapshot.
The input decode/schedule preflight covers all 240 PNGs and every original
sensor timestamp/group. Prepared manifests are sliced: start0 is used in every
new run, rather than reapplying historical full-manifest offsets.

## Execution and preservation

All six independent 20-frame expert controls and all three shared captures
complete on the first scientific execution attempt, with no outcome retries.
The shared capture uses the existing copied event recorder, serial sensor order,
one-group lag and final decision before optimizer drain. Telemetry is disabled;
no estimator, checkpoint, seed, optimizer policy or production default changes.

| Window | Controls RGB / thermal | Shared capture | RGB arrays exact / NPZ | Thermal arrays exact / NPZ | Independent decisions |
|---|---|---|---:|---:|---:|
| day-original |PASS /PASS|PASS|533 /23|533 /23|19|
| day-early |PASS /PASS|PASS|533 /23|533 /23|19|
| day-late |PASS /PASS|PASS|533 /23|533 /23|19|

All 3198 shared numerical arrays match their fresh same-runtime controls in
shape, dtype and exact bytes. Full map/trajectory/uncertainty/first-depth outputs,
tracking reports and effective configs are preserved. Each expert's 97 recorded
events match its independent control after removing only observation-monotonic
clock values. No event health, source pose, stage or submission field is omitted
from that equality check. This establishes capture/scheduler preservation within
this runtime, not physical calibration or equality with older runtime outputs.

The independent timeline auditor is adapted from EXP073, with source manifest
and frozen capture-plan binding parameterized for these exact three windows.
Its schedule, prefix and advice checks remain unchanged; it imports no scheduler
or policy implementation. Each window passes 40 scheduled camera frames,
19 original decisions and798 decision-field checks. All copied stream prefixes
match saved full expert journals; source hashes/checkpoints and preservation
proofs are verified. All three windows remain one development-recording group.

## Feature and conditional label annotation

Unchanged EXP088 projections and existing EXP089 policies are replayed from
observed shared prefixes. Model predictions for all57decisions are written and
hashed before the separate label-scoring invocation. Static nominal transforms
use the frozen supplied vehicle/optical chain and each stream's rectification/
NED conversion. Labels use only the archived recorded-SLAM
`sensor_init -> vehicle` interpolation on each RGB target interval. No reference
bridge enters prediction, and no alignment, scale, clock shift or loss threshold
is fitted. The result remains a nominal-frame recorded-SLAM proxy, not physical
truth or a verified body-motion admission.

| Window | RGB / thermal projections available | Both supported | Complete current + two prior intervals | RGB / thermal strict translation proxy wins in complete subset |
|---|---:|---:|---:|---:|
| day-original |18 /18|18|16|8 /8|
| day-early |18 /18|18|16|12 /4|
| day-late |18 /18|18|16|9 /7|
| Total |54 /54|54|48|29 /19|

Every source camera contributes its terminal unavailable motion explicitly.
Raw advice chooses RGB18/19 per window; history chooses RGB17/19 with two
abstentions, reflecting its extra cold start. This is unchanged availability
behavior, not evidence that either policy selects the better-quality supported
expert. The 48 complete rows have a largest single squared-error-gap share of
14.52% within this recording. No fit or parameter selection is performed.

The independent annotation auditor reconstructs nominal static graph paths,
per-query scalar quaternion SLERP/linear reference interpolation, native body
motions, scipy.linalg.logm/expm projections and translation/atan2 rotation losses.
All57rows,108 available expert projections,264 scalar checks and the48eligible
row counts pass. Maximum projection matrix discrepancy is3.43e-14. Physical
frame/acquisition-clock/live-latency flags remain unverified. These are saved
observed-prefix decisions in an offline serial replay, not measured real-time
sensor deadlines.

A development-only source index joins these 48rows to the prior113:161eligible
rows from three distinct named recordings, with no duplicate recording/interval
keys. It retains parent sources/runtimes and labels rather than pretending all
historical execution was repeated in this image. Current groups are:

| Recording | Eligible rows | RGB / thermal proxy wins | Largest single squared-gap share |
|---|---:|---:|---:|
| 0_salon_so_day |96|65 /31|17.59%|
| 1_salon_mac_cov_day |48|29 /19|14.52%|
| night_mac_salon_warehousefig8 |17|6 /11|99.98%|

The third recording improves development coverage and class support; it does
not erase the known night outlier's weight concentration, make adjacent pairs
independent, or provide prospective test evidence. No model is trained, and
neither reported MS² reserve is assigned or opened.

## Validation, decision and recovery

183 existing recorder/layer/model tests pass in the new runtime before capture.
Actual capture validation additionally includes all source/checkpoint hashes,
all inputs decoded, six complete controls, three complete shared runs,3198 exact
arrays, six97-event comparisons, three independent schedule/prefix audits and
independent model/reference annotation checks. No failed scientific run or
outcome-selected retry is hidden. Logs and all fixed-window outputs are retained.

The planned third-development-recording collection is complete. The draft gate
remains unfitted and not prospectively frozen. Further fitting must bind the
complete source/feature/label/runtime ledger and retain recording groups;
physical reference and checkpoint-training provenance remain unresolved.

Artifacts: `results/third-recording-shared-capture/run-001/`, including frozen
capture-plan/declaration, execution logs, fresh controls and shared runs,
preservation/timeline proofs, prediction lock and model outputs, conditional
annotation/validation and development-index.json. The notebook contains exact
recovery, collection, parity and annotation scripts, preflight, JUnit, build log
and package freeze. Scripts refuse or require fresh capture/output paths; adapt
the recorded run directory explicitly when reproducing, then freeze a new plan.

Backup: `ore_ms:ore-ms/backups/2026-10-31-exp-090`, incremental Git base
`1e60c400f`. Includes bounded three-window source inputs, camera metadata,
calibration/SLAM references, all new outputs/evidence and source scripts. No
credentials, signed URLs, original full ZIPs, range caches, checkpoints or Docker
image binary are included. Rebuilding uses the pinned Dockerfile plus recorded
package versions; exact old runtime availability is not assumed, and fresh
same-runtime controls remain required. Original checkpoint source/hash is
preserved separately in models/README.md.
