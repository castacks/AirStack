# Fixed bearing-rotation assay: day gain does not transfer to night

12 November 2026. **Stop promotion of this fixed covariance configuration.**
The primary original-flow/original-covariance PNEC prototype passes the day gate
but fails all four night accuracy conditions. Both recordings were required to
pass independently. Coverage is preserved, so this is an accuracy failure rather
than an availability stop. No retuning or substitution of a secondary winner.

## Recordings and fixed method

The input freeze names the original exposed stereo pilots:

- Day `_2021-08-06-11-23-45`, original stems000000–000019.
- Night `_2021-08-13-21-18-04`, original stems000000–000019.

These are distinct from the later whole-record cohort `_2021-08-13-16-31-10`
and `_2021-08-13-22-16-02`. Inputs, timestamps, calibration and reference files
were bound to their exact recording; no similarly named cohort was substituted.
Both pilots have previously been inspected and used in the closed stereo-fit
work. This is repetition across exposed development recordings, not held-out
evaluation or evidence about original checkpoint training membership.

EXP048/041 archives and every regular member were hash-verified. Selected raw
images, timestamps, calibration and RGB pose files were restored without
overwriting conflicts. There are160selected raw images across the two clips
and40provided RGB reference poses. The source IDs and original pairing remain
unchanged. No full collection or new publisher recording was downloaded.

The [transfer freeze](../../experiments/bearing-rotation-transfer-freeze-2026-11-12.json)
binds1,205source/input/model/reference hashes before new inference or geometry.
The corrected rain model, objective, source grid, first-order covariance
propagation, optimizer, tolerances, budget and RANSAC filter are byte-unchanged.
The same original-flow/original-covariance candidate remains primary.

Each recording has one fresh20-frame native thermal control. An observer saves
flow/covariance and runs the same frozen TartanMatch on19temporal pairs while
returning the original native depth/match objects unchanged. All40native frames
complete. The38replacement calls and original pair calls are single attempts;
no training, checkpoint/preprocessing/threshold search or native retry occurs.
These freshly captured inputs have no historical bitwise-flow parity claim.

Per-record input locks precede candidate solves. Both recordings' candidate
predictions are globally locked before either is reference-scored. Earlier
references are exposed and can be integrity-hashed before the lock, but they
do not initialize/filter the predictions.

## Primary results on19identical intervals per recording

All values below are rotation errors in degrees under the inherited nominal
common RGB frame and actual-duration stationary-twist mapping.

| Recording / method | Mean | RMSE | p95 |
|---|---:|---:|---:|
| Day NEC | 0.037645 | 0.044791 | 0.077993 |
| **Day primary PNEC** | **0.035345** | **0.039363** | **0.063185** |
| Day native thermal | 0.053821 | 0.059961 | 0.100666 |
| Night NEC | 0.022464 | 0.024190 | 0.039579 |
| **Night primary PNEC** | **0.040596** | **0.051115** | **0.106919** |
| Night native thermal | 0.019844 | 0.020650 | 0.028879 |

Day primary mean/RMSE improve6.11%/12.12%against NEC and34.33%/34.35%against
native thermal. Night mean/RMSE worsen80.72%/111.30%against NEC and
104.57%/147.53%against native thermal. Primary, NEC and native each retain
19/19eligible intervals on both recordings.

The gate requires nonempty matched support, no NEC interval-coverage loss,
strictly lower rotation mean AND RMSE than NEC, and no higher mean AND RMSE
than native thermal. Day passes all checks; night fails the four accuracy checks.
The inherited per-record scorer retains its legacy `RAIN_DEVELOPMENT_GATE_PASS`
string for a pass; `summary.json` identifies day/night explicitly and the
overall disposition is **STOP_TRANSFER_PROMOTION_NO_RETUNING**.

The preceding rain pass remains a valid bounded result, not a reason to relax
the per-record rule or pool away the night regression. Its exposed clip differs
from these two recordings; do not treat all57intervals as independent samples.

## Secondary controls retained

Replacement-flow controls use the same retained source points within each pair.
They cannot rescue the primary decision. Their19-interval means/RMSE are:

| Recording / replacement-flow objective | Mean | RMSE |
|---|---:|---:|
| Day NEC | 0.033013 | 0.035880 |
| Day unit pixel-covariance PNEC | 0.030806 | 0.034135 |
| Day original-covariance PNEC | 0.034601 | 0.041189 |
| Night NEC | 0.010936 | 0.011361 |
| Night unit pixel-covariance PNEC | 0.013041 | 0.013536 |
| Night original-covariance PNEC | 0.016381 | 0.018608 |

For original means, the unit pixel-covariance control is0.035865/0.042363 on
all19day intervals. One night unit-covariance solve is explicitly degenerate,
so its matched comparison has18intervals: unit PNEC0.031656/0.040510,
NEC0.022848/0.024582, primary0.038473/0.049121, native0.019632/0.020470.
These18-interval figures must not be mixed with the19-interval primary table.

The fixed replacement-flow NEC control is below native thermal in mean/RMSE on
rain, day and night. This is a useful exploratory lead for a separately declared
larger-record/fresh-record comparison, not a retrospectively promoted winner or
evidence of gain over RGB. Do not combine it with translation results from the
earlier native-flow replacement experiment to claim improved full VO.

## Execution and audit

There are228attempted solver calls:19pairs ×2means ×3objectives ×2recordings.
All114day solves converge. Night retains113converged and one
`DEGENERATE_GEOMETRY` result, without retry or relaxed thresholds. That
degenerate result remains in the114method slots and its stored objective is
audited, even though it is unavailable for motion scoring.

Day support is10,731joint finite/inbound/SPD observations before the unchanged
geometric filter and10,684after it,527–580per pair. Night support is11,115before
and11,114after,584–585per pair. All22,230grid covariance matrices are finite/SPD.
These are predicted correspondences with geometric filtering, not certified
visible physical-point labels.

All39synthetic numerical/filter/mapping/identity/lock/decision tests pass before
execution. The saved-output audit reconstructs228objective costs and227candidate
motion mappings/errors, checks all228retained slots, native rotation context,
coverage, summary metrics and both recording/overall decisions. It uses scalar
cost arithmetic and independent matrix powers for the time mapping; reference
serialization qualification and calibration lookup are shared. It does not
rerun networks/optimizers or certify physical reference truth.

The exact recovered runtime is unchanged; no package build or update occurred.
The original rain inputs, predictions, scores, code and frozen outcome remain
unchanged. New artifacts are in `results/bearing-rotation-transfer/run-001/`.
Code: `bearing_rotation_transfer.py`, its test module and
`audit_bearing_rotation_transfer.py`. Original core code remains the frozen
`bearing_rotation.py` and `run_bearing_rotation_assay.py`.

## Research implication and stopping decision

[PNEC](https://arxiv.org/html/2204.02256v1) explicitly depends on useful
positional uncertainty and outlier handling. Our result supports conditional
benefit from the particular covariance/geometry combination, not universal
improvement from weighting. It does not isolate whether covariance error,
source point distribution, model approximation or local optimization causes
the night regression. Our first-order propagation and local solver also differ
from the published full optimizer, so this does not reject PNEC as published.

[DNLS](https://arxiv.org/html/2305.09527v2) remains relevant because it learns
correspondence uncertainty through a pose objective. Such learning would be a
new hypothesis and requires a separate gradient/supervision/recording-split
contract; it is not justified by relabeling this existing-covariance failure as
successful calibration. Absolute covariance scale remains unestablished.

Close promotion of the unchanged existing-covariance configuration. Preserve the
positive day/rain evidence and negative night evidence together. No night
retuning, secondary-winner substitution, automatic training or full-VO/default
change follows from this task. A further method proposal should use the papers
and these controls to name a distinct mechanism and a finite evaluation.
