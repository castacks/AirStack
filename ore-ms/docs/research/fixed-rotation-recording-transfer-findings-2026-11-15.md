# Unchanged-method recording transfer: gains on both short day/night prefixes

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus source-pinned local harness. Native fixed-rotation method is unchanged.

## Outcome

The thermal-assisted translation method transfers to both predeclared recordings
in this100-group replay. It beats **native RGB and matched RGB-only translation
refinement** on trajectory position RMSE, with native RGB rotation exactly retained.

- Day:1.584581→1.449779m, **8.507%lower** than native; RGB-only refinement1.548927m,
  so thermal adds **6.401%improvement** over the matched control.
- Night:0.577259→0.571310m, **1.031%lower** than native; RGB-only refinement0.588115m,
  so thermal adds **2.857%improvement** over the matched control.

All99intervals are available in allthreearms for each recording. Position mean,
final position error and relative-translation mean/RMSE/p95 also improve for the
thermal arm versus both controls. Rotation is preserved by design, not a separate
rotation improvement. There is no compute-saving or physical uncertainty-calibration
claim.

This gives a repeatable **scoped application candidate** beyond the prior single
day recording. It does not erase the original rain regression: in that600-group
study thermal remained worse than RGB-only refinement. No weight or solver
setting changed between the studies.

## What transferred and what did not

| New transfer prefix | Sequence | Groups | Recorded RGB span |
|---|---|---:|---:|
|day|_2021-08-06-11-23-45|100|10.265844s|
|night|_2021-08-13-21-18-04|100|9.998790s|

These differ from the candidate's prior600-group day `_2021-08-13-16-31-10`
and night `_2021-08-13-22-16-02`. They were selected before recovery and new
predictions from the already existing primary100-group cohort. The prior rain
recording is the same `_2021-08-06-16-19-00` and was deliberately excluded from
a new transfer claim.

Both transfer recordings were exposed in older research, including pose-blending
and weighting experiments. This is transfer of an **unchanged configuration**
to different records, not a blind test or unseen-data generalization result.
These≈10second prefixes are shorter than the prior≈60second paths. Do not infer
that recording conditions caused the difference in gain magnitude or that the
short gain persists over a longer trajectory. No favorable segment was selected
after scoring, and no additional windows or retries were run.

## All complete100-pose paths

| Record | Method | Position RMSE m | Final position error m | Orientation RMSE deg |
|---|---|---:|---:|---:|
| day | Native RGB | 1.584581 | 3.343009 | 2.063867 |
| day | RGB-only refinement | 1.548927 | 3.292846 | 2.063867 |
| day | RGB + thermal refinement | 1.449779 | 3.150027 | 2.063867 |
| night | Native RGB | 0.577259 | 1.792987 | 0.500441 |
| night | RGB-only refinement | 0.588115 | 1.806032 | 0.500441 |
| night | RGB + thermal refinement | 0.571310 | 1.754909 | 0.500441 |

Every path starts in an identity gauge. No SE3/Sim3, scale, yaw, time or offset
alignment is fitted. The same supplied per-ID RGB references are used across
all arms. Reference files were restored and hash-bound but not newly parsed
for scoring until all new predictions were locked.

## All99relative intervals per arm

| Record | Method | Translation mean m | Translation RMSE m | Translation p95 m |
|---|---|---:|---:|---:|
| day | Native RGB | 0.046390 | 0.052889 | 0.096042 |
| day | RGB-only refinement | 0.045992 | 0.052460 | 0.095999 |
| day | RGB + thermal refinement | 0.044870 | 0.051262 | 0.094531 |
| night | Native RGB | 0.022281 | 0.035981 | 0.084698 |
| night | RGB-only refinement | 0.022341 | 0.035833 | 0.078719 |
| night | RGB + thermal refinement | 0.021885 | 0.035092 | 0.076882 |

Relative-rotation mean/RMSE/p95 are identical across the three arms within each
recording and retained in scores.json. There is no missingness-based metric
selection, zero-error imputation or thermal-only coverage rescue.

## Exact implementation controls

Three source-pinned functions from the completed studies are reused unchanged:
`native_pose_feedback.step` for the native RGB backbone and point registration,
`component_constrained_native.step` for both constrained arms, and the same
native disparity graph/analytic LM implementation. Parent file hashes match the
previous candidate's freeze. The only new code selects the predeclared inputs,
orchestrates those functions and scores/audits their outputs.

Thermal precision0.1, three world-translation coordinates, fixed current native
RGB quaternion, native residual/covariance/Jacobian, Huber/PINV/trust/scheduler,
warm start and float32 writeback are all unchanged. Each candidate owns its
translation state and uses current RGB backbone rotation. Missing-state behavior
is inherited unchanged, although no unavailable output occurs in these prefixes.
Both recovered transfer RGB configurations explicitly match disparity,
static motion, all-keyframe, covariance-only admission and mapping=false.

594new optimizer calls (198per arm) complete, plus18calls in the six-pair
qualification. All198thermal attempts have valid thermal packets. The native
RGB backbone is included as an explicit control and computational requirement;
no claim that extra thermal processing is free.

## Recovery, qualification and independent verification

Recovered216selected members from the historical unweighted-longer archive.
Whole archive SHA256:
`e0454be29174cf98d2ccf87d49a246d520ea74133e655cd926a21a3071de4601`.
Every member matches its historical manifest and applicable prior100-group
input pin. This includes200reference files plus original native graphs, events,
source manifests, calibrations and effective configurations. No old result or
input was overwritten.

The original first3pairs in eachrecording qualify native motion reproduction
within the already declared1e-3m/1e-4rad bounds and exact constrained quaternion
preservation. The full run independently verifies all198native increments:
worst translation difference3.814e-6m and rotation4.723e-8rad versus original
native first-write receipts. No solver/source/tolerance amendment was needed.

Independent audit passes **2453checks** over source and input pins, all recovered
reference hashes, native reproduction, source slot counts, state continuity,
exact native rotation preservation, held-state semantics, emitted transforms,
full-path metrics computed directly from committed endpoint states and separate
scalar relative-translation statistics. All predictions are locked before
new reference scoring. All source files from the parent candidate still match
its previous freeze.

## Interpretation and next evidence boundary

The useful mechanism here is restricted measurement use: retain the strong RGB
orientation and let thermal constrain metric translation through the shared
geometry. The matched RGB-only re-solve prevents attributing its improvement to
thermal. This study establishes the numerical comparison, not a causal account
of environmental conditions or a learned predictor of future thermal utility.

Keep the candidate and its gains as an engineering/research result. The current
evidence supports improvement on the declared nominal cached-measurement
benchmarks; it does not establish a broadly better deployed VO system. Longer
transfer sequences, new-record evaluation and full frontend/runtime comparisons
remain distinct validation questions. Do not tune on these exposed prefixes.

Existing SFVO/Buczko/MSO overlap recorded in the
[parent component-constrained report](component-constrained-native-findings-2026-11-15.md)
remains applicable: generic R/T separation and RGB–thermal factors are prior art.
This transfer is not a new model, full author-system reproduction or verified
novel algorithm. No new primary paper reading was performed for this execution.

Artifacts: `results/fixed-rotation-transfer/{recovery,qualification-001,run-001}`.
Tools: `recover_fixed_rotation_transfer.py`, `fixed_rotation_recording_transfer.py`,
`audit_fixed_rotation_transfer.py`. Runtime: Torch2.7.1+cpu/PyPose0.6.8, one thread.
Physical exposure/clock, source-depth and provided-reference limitations remain;
no duration correction or claim of independently instrumented ground truth.
