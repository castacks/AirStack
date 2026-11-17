# Unchanged lower-third exclusion transfer to OutdoorNight

16 November 2026 America/New_York. Follow the
[matched-count comparison](matched-count-findings-2026-11-16.md) and the existing
[OutdoorNight nominal gyro contract](outdoornight-gyro-findings-2026-11-16.md).
Known exposed recording and historical outcomes; fixed engineering transfer,
not held-out generalization or physical pose accuracy. No new paper-body reading.

## Frozen inputs and hypothesis

Does the unchanged lower-third exclusion improve moving nominal gyro agreement
beyond original and one fixed same-count control, retaining availability losses?
All971anchors,3884raw rows (both cameras/frontends), three variants original,
bottom,count,11652geometry rows. Preserve all failed and unsupported inputs.
The fixed common-gyro964 and moving242 cohorts and own sensor endpoint labels
are inherited verbatim; no new motion/reference filtering after predictions.
OutdoorNight stays separate from SmokeRoom/FlashLight supplied-reference results.

Bottom is exactly the previous recipe: native nearest-pixel row<2h/3 at BOTH
source and target; RGB rows0–319, thermal0–341. Own pinhole/MEI calibration,
virtual focal400, canonical4decimal pixel rounding and .05px boundary guard,
nearest floor(x+.5), final clip only for roundoff. No persistent-zero mask (prior
no-op), different crop size, scene/rig segmentation or adaptive mask. Preflight
every native match endpoint, all input image shapes/hashes, empty/failure rows,
retained counts/index hashes and fit eligibility before ANY new essential fit.

Count keeps exactly bottom's image-only prefit count using the prior unchanged
SHA256 rank `pre-E-matched-count-v1|seed=0|OutdoorNight|{frontend}|{arm}|{index}|{match_index}`.
Digest ties use original index, restore ascending original order. One draw only;
no coordinates/flow/references/errors in ranking, no seeds or subset selection.

## Geometry, choices and endpoints

Exact recovered image sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09,
OpenCV5.0.0/NumPy2.2.6, one CPU OpenCV thread. Before masked execution, replay all
3884original rows with unchanged pose helper and require exact pose-field parity.
Preserve failure and stop if parity fails; do not repair solver parameters.
Seed0,RANSAC .999/1px/1000iterations,8matches/8cheirality gate unchanged.
Original outputs inherited exactly; exact retained-index matches may reuse
original or earlier bottom result with explicit origin and call receipts.
No matching/model inference, new feature detection, optimizer or training.
Lock all predictions then unchanged four status-only policies RGB, thermal,
RGB-first,thermal-first before calculating gyro disagreement.

Use already frozen raw left-held gyro at each camera's actual source/target ns,
20ms gap guard and own published camera-to-IMU rotation, no bias/offset/axis fit.
Reuse existing gyro-policy evaluator; saved timing fields are inherited cached
matching-cost projections only, not measured intervention/end-to-end cost.
Primary: learned moving within1°/5° of gyro for each fixed camera and RGB-first,
original/count/crop, missing outputs count against same242denominator. Also
thermal-first, all/common gyro/index thirds, SIFT, output support, paired correct
and output gains/losses versus original and count→crop. Retain same-endpoint zero
rotation and thermal-addition controls. No translation scoring or tier pooling.

## Audit, limits and stop

Independent native camera projection/ranking rebuilds subset counts/order/hashes;
check original exact identity, origin/solver receipts, all masks/gates, properR/t,
SVD(U,V) E versus [t]xR at1e-5 (raw E never edited), own quaternion gyro targets,
all disagreement values, policy/endpoints, every cohort and paired transition.
Use inherited OpenCV SVD contract from outset; earlier rawE gate failure remains
history, not relabelled pass. Finite synthetic/native boundary/count controls and
rehashed subset/disagreement/transition corruptions must fail; full valid passes.

No seed variance, IID significance, causal rig-only, gyro calibration, physical
exposure/pose-truth, metric translation, stereo/inertial VO or deployment claim.
Stop after complete findings, documentation and member-verified local packet,
positive/null/negative transfer retained. No crop/clock/identity/camera rescue,
new seed, training, Box upload, Git push or automatic commit.
