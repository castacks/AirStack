# Persistent-track replay and five-frame geometry: no joint advancement

12 November 2026, America/New_York. Branch `ore-ms-orb-development`, Git baseline
`4f28219d9` plus separately SHA-frozen working-tree source. New observation replay
explicitly authorized after the [saved-track admission stop](windowed-track-admission-findings-2026-11-12.md).

**Decision: CLOSE_THIS_WINDOWED_CONFIGURATION_NO_RETUNING.** Persistent tracks are
successfully recorded and verified without changing native outputs. The complete
four-arm pilot then runs. Shared RGB–thermal pose/depth optimization fails every
recording's predeclared advancement rule against matched RGB-only window
optimization and original RGB-cov. The failure is now a geometry-comparison
result; the earlier pair-only archive limitation has been resolved for this run.
No new training or automatic longer-window/weight/matcher search follows.

## What was implemented and preserved

The exact original compiler runtime image
`44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`
is restored from the verified EXP106 base and EXP112 delta, with original RGB
and MACThermal checkpoints. All2,782 sensor/model packet members and3,196 longer-
prefix packet members are hash-verified. All1,200 selected raw images match
their frozen source hashes. No dependencies, weights or native defaults change.

A process-local opt-in frontend observer copies unchanged original flow,
flow covariance, target stereo disparity/variance and bad-depth mask. Each
five-group window starts with all retained source points of its first native
pair from a fresh recorder-off control. It transports explicit camera/window/seed
IDs through successive original dense flow fields. No independent pair graph
IDs are joined by pixel/world proximity; no cross-camera pixel matching occurs.

Bilinear sampling uses full four-corner interior support. The observer records
raw corner witnesses, coordinates, integer timestamps and all loss reasons.
It propagates UV covariance with `(I+J_flow) C (I+J_flow)^T + C_flow`, keeps
original disparity variance, and permanently ends invalid/out-of-bounds/masked
tracks. No track is replenished within a window. This is a declared first-order,
independent-increment approximation; cross-observation/network/image correlations
and UV–disparity cross-covariance are not modeled. It does not calibrate physical
measurement uncertainty or verify that tracked pixels are true correspondences.

Twelve100-frame native runs complete: six recorder-off and six recorder-on.
All234 arrays in24 native NPZ files match bitwise, including poses, times,
flags, matches and serialized covariances. All2,980 arbiter event payloads match
exactly after excluding only `observation_monotonic_ns`. Sensor timestamps,
sequence, stage and motion payloads remain equal. A separate overlap check finds
all66 archived pair-solver input fields identical to fresh off controls, retaining
comparability with the original RGB-cov baseline.

## Track admission and retained audit failure

The replay produces17,870 seed identities,17,838 tracks reaching at least3 frames,
17,317 tracks reaching all5 frames and88,567 valid observations. All120 camera-
windows meet the fixed count requirements: at least6 three-frame tracks and at
least6 observations in each of the5 frames. Every loss, short track and invalid
sample remains in the observation packet; no window or threshold is reselected.

Six synthetic recorder tests pass. The independent corner-witness auditor
reconstructs advection, spatial flow derivatives, covariance propagation,
validity/loss decisions, frame times and saved windows. It also reproduces all
17,870 native first-pair endpoints with the declared float32 comparison.
The amended audit passes2,125,502 checks.

The first auditor version failed an extra covariance cross-check: it compared
raw dense covariance with the native graph's post-floor covariance. The inspected
native `MatchCovariance.estimate` clamps diagonal variance to
`float32(min_flow_cov**2)` before storing that field. The amendment applies that
exact conversion only in the cross-stage assertion, retaining the same numeric
tolerances. Raw recorder covariance, captured tracks, masks, sampling, hypotheses
and native outputs are unchanged; no capture was repeated. The original failed
source and failure receipt are retained under the capture validation subtree.
Amendment `experiments/persistent-track-auditor-stage-amendment-2026-11-12.json`
precedes track admission and all real candidate solves. The original audit remains
historically failed, not retroactively passed.

## Four-arm fixed-window estimator

Use20 fixed, nonoverlapping five-frame windows per recording. The first body pose
of each window is identity. Optimize four remaining body poses and, in variable
arms, one log inverse depth per track; all tracks keep their original source ray.
Each camera retains its own K/baseline and source/target measurements under the
supplied rigid RGB–thermal extrinsic. Stereo disparity residuals at all retained
observations anchor metric depth. Source pixel/ray uncertainty is not jointly
optimized, and landmark identities remain camera-local.

Compare `rgb_fixed`, `rgb_variable`, `joint_fixed`, `joint_variable`, with identical
observations/covariances within each camera choice. The objective is the sum of
full2D-UV plus independent-disparity whitened squared residuals. No robust kernel,
per-camera normalization, learned weight or modified mean is introduced.

Float64 CPU; identity pose initialization and initial inverse depths from the
original seed disparities. Twenty iterations maximum, diagonal damping1e-6,
pose-Schur singular ratio cutoff1e-10, at most12 Armijo trials with coefficient
1e-4 and fixed halvings. Pose and depth Jacobians are analytic; inverse depths
are eliminated through the diagonal Schur complement. Gradient stops use the
frozen normal-diagonal-scaled1e-6 test. Seven numerical tests pass, including
camera-specific recovery, log-depth refinement, all24 pose/four depth finite-
difference columns and an independently assembled dense normal solve.

All240 declared attempts run once:224 reach the internal scaled-gradient stop,
12 hit the fixed iteration budget and4 fail line search. The four failures are
RGB-fixed day/start30, night/start30, rain/start95, and joint-fixed rain/start85.
Their attempts and missing intervals remain explicit. No failed window is retried
or silently given a fallback. The fixed-budget completions do not imply
stationarity/convergence.236 windows yield944 relative-motion outputs across the
four arms. Both variable-depth methods deliver all240 planned intervals.

## Matched primary results

Every row below uses the same80 declared within-window adjacent intervals in
that recording. The19 between-window intervals remain unscored and are listed
explicitly. No inter-window trajectory is assembled. Original/native baseline
outputs are evaluated only on these same slots. Complete own/common and pairwise-
matched metrics, p95, maximum, coverage and rows remain in score files.

| Recording | Method | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---|---:|---:|
| day | original_rgb_cov | 0.045240 / 0.051442 | 0.059710 / 0.064414 |
| day | rgb_variable | 0.042382 / 0.048562 | 0.049162 / 0.053487 |
| day | joint_variable | 0.050748 / 0.055536 | 0.051793 / 0.055490 |
| night | original_rgb_cov | 0.019871 / 0.033911 | 0.034880 / 0.053734 |
| night | rgb_variable | 0.018235 / 0.029574 | 0.033155 / 0.053485 |
| night | joint_variable | 0.021610 / 0.033734 | 0.041751 / 0.077026 |
| rain | original_rgb_cov | 0.199761 / 0.388018 | 0.039684 / 0.046033 |
| rain | rgb_variable | 0.241213 / 0.437051 | 0.034364 / 0.040788 |
| rain | joint_variable | 0.242550 / 0.439273 | 0.034224 / 0.040646 |

Adding thermal changes translation error relative to the matched RGB-only
window solver as follows; positive values are worse:

| Recording | Joint versus RGB-window translation mean | Translation RMSE |
|---|---:|---:|
| day | +19.74% | +14.36% |
| night | +18.51% | +14.07% |
| rain | +0.55% | +0.51% |

The primary requires at least5% lower translation mean AND RMSE against BOTH
RGB-variable and original RGB-cov on each recording, with no rotation mean/RMSE
or coverage regression. All three fail both mandatory comparison gates. Rain
improves over native RGB, but its original RGB-cov control is better; that native
comparison cannot establish thermal contribution. RGB-only window refinement
improves translation over original RGB-cov on day/night and worsens it on rain;
it is a secondary exposed-development observation, not an adopted replacement.

All-method common support is76/76/71 day/night/rain, reflecting the retained
frozen-depth failures and one prior native thermal unavailability. These smaller
masks do not replace the full80-slot primary matched comparison.

Local exportable figures: `results/windowed-geometry/run-001/figures/comparison.png`
and `.pdf`. Plot uses the same80 slots and shows mean errors only; RMSE and
coverage are retained in the table/scores. The exported figure was inspected.

## Independent verification and scope

All predictions, prepared numerical packets and attempted-step journals are
hash-locked before scoring parses provided reference pose values. Reference files
are previously exposed development inputs, not a fresh/blind test; hashes are
opaque integrity checks before scoring. Actual sensor timestamps remain unchanged
under the declared nominal same-ID grouping assumption. Physical exposure timing,
independent reference accuracy and checkpoint training overlap remain unverified.

The independent auditor imports no producer, solver or scorer. It reconstructs
prepared ray/depth/observation/extrinsic packets directly from captured tracks,
independently projects saved states, checks every line-trial cost and acceptance
receipt, checks first-step directional derivatives for all240 attempts, validates
the fixed root/frozen depths/relative-pose convention, and recomputes errors,
statistics, masks and gates. All7,123,339 checks pass, including4,563 objective
reconstructions and240 central finite differences. It covers2,200 attempted line
trials. It reruns neither neural inference nor an optimizer, and does not certify
a physically correct covariance model or final global optimum.

Close this exact shared-window conditional-ray/stereo configuration. This is not
a reproduction of Dai et al.'s direct photometric MSVO and does not exhaust joint
geometry, multispectral VO or representation learning. No training, protected
reserve access, runtime/compute-saving claim, physical synchronization claim or
fresh-record accuracy result follows. Retain the working recorder and controlled
negative joint-geometry result for future hypothesis selection.

Protocols: `experiments/persistent-track-replay-freeze-2026-11-12.json`,
`windowed-geometry-execution-freeze-2026-11-12.json`, and
`windowed-geometry-scoring-freeze-2026-11-12.json`.
Local artifacts: `results/persistent-track-replay/run-001` and
`results/windowed-geometry/run-001`; source snapshots and outcome hashes preserve
this execution independently of future working-tree edits.
