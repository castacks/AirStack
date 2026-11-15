# Native RGB optimizer with thermal constraints: working baseline, scoped gains

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus source-pinned local implementation. No new frontend inference or training.

## Outcome

A calibrated thermal-factor extension to the native disparity optimizer is
implemented and qualified. RGB-only replay reproduces all1796supported original
native motions within the prespecified bounds. This resolves the previous
conditional-backend mismatch for this saved-input comparison.

Day full-path position RMSE improves **0.910%**, with orientation RMSE worsening
**7.320%**. Rain position/orientation RMSE worsen **7.901%/2.949%**. Night retains
the common missing pair247. The result is mixed, not a broadly improved VO system.

There are useful scoped positives: **four of17complete, predefined100-group
blocks improve both position and orientation** (day starts200/300; night300/500).
All18blocks are reported below, including the incomplete night block. These are
within-cohort evidence that thermal can add value on top of the native optimizer,
without establishing an environmental explanation or held-out improvement.

## Full600-pose paths

| Condition | Method | Position RMSE m | Orientation RMSE deg |
|---|---|---:|---:|
| day | Native RGB replay | 11.115267 | 4.037354 |
| day | Native RGB + thermal | 11.014108 | 4.332896 |
| night | Native RGB replay | Incomplete | Incomplete |
| night | Native RGB + thermal | Incomplete | Incomplete |
| rain | Native RGB replay | 64.140839 | 4.346089 |
| rain | Native RGB + thermal | 69.208533 | 4.474261 |

Full paths use an initial identity gauge, no fitted alignment or scale, and no
stitching across missing outputs. Day final position error actually worsens
17.657963→17.800411m despite lower path RMSE; rain124.087532→135.077940m. Preserve
these endpoint distinctions rather than describing every translation metric as
improved. Nominal provided references and their physical limitations remain.

## Fixed blocks: thermal change relative to native RGB replay

Negative favors thermal. Same six start indices fixed before this run.

| Condition | Start | Position RMSE change | Orientation RMSE change |
|---|---:|---:|---:|
| day | 0 | -0.581% | +1.526% |
| day | 100 | -1.210% | +8.050% |
| day | 200 | -1.054% | -1.763% |
| day | 300 | -0.571% | -16.050% |
| day | 400 | -1.001% | +11.370% |
| day | 500 | -1.099% | +12.669% |
| night | 0 | +0.041% | +0.654% |
| night | 100 | -1.593% | +5.199% |
| night | 200 | Incomplete | Incomplete |
| night | 300 | -2.219% | -0.759% |
| night | 400 | -0.085% | +16.253% |
| night | 500 | -0.647% | -0.884% |
| rain | 0 | +9.966% | -4.947% |
| rain | 100 | +4.054% | +3.427% |
| rain | 200 | +0.842% | +1.873% |
| rain | 300 | -1.607% | +3.604% |
| rain | 400 | +16.682% | +0.783% |
| rain | 500 | +10.205% | +6.919% |

No block was selected for inference, weight choice or exclusion based on its
error. The favorable subset is descriptive after scoring, not a validation set
or a deployed activation rule. Adjacent poses/windows are not independent trials.

## What changed in the implementation

`tools/native_thermal_constraints.py` imports the actual local native
`Analytic_ReprojDisp_TwoFramePGO` and `LM_analytic` class bodies unchanged.
The RGB graph retains original world points, accepted observations, intrinsic
matrix, baseline, covariance entries and receipt-parent initialization.
The extension appends thermal UV/disparity residuals on that same RGB pose,
using calibrated RGB-to-thermal extrinsics and thermal source points expressed
in the original RGB world frame. Thermal source points remain fixed.

The native RGB graph methods are called directly for their rows. Thermal rows
have an analytic Jacobian independently checked by finite differences. The fixed
thermal precision0.1 is inherited from the earlier comparison; no new parameter
search. Native LM, PINV, Huber0.1, FastTriggs, trust radius1000 and scheduler
10steps/patience2/decreasing1e-5 remain. Only the GPU timing wrapper is omitted in
this CPU-only runtime. Namespace package loading skips unrelated frontend package
initializers; it does not substitute graph/optimizer implementations.

Native source bodies and all input files are pinned in the protocol. Runtime:
PyTorch2.7.1+cpu, PyPose0.6.8, one CPU thread. This is a local saved-input
implementation, not a new reproduction of a complete published benchmark.

## Coverage and run accounting

| Condition | RGB supported /599 | Fused supported /599 | Actual thermal additions | Unchanged RGB fallback |
|---|---:|---:|---:|---:|
|day|599|599|595|4|
|night|598|598|558|40|
|rain|599|599|598|1|

There are1796native RGB and1751thermal-augmented optimizer calls,3547total.
All complete without an exception;45missing thermal inputs copy the native RGB
result. The one missing native RGB interval remains absent in both arms. There
is no thermal-only gap rescue, zero-error imputation or new availability claim.
Native ten-step budget is not a guarantee of optimizer convergence. Native loss
can increase under its existing weighted-step/unweighted-acceptance conventions;
no solver repair or scientific objective substitution was made.

## Qualification and independent checks

Nine fixed pairs (0/1/2 for each condition) pass exact zero-thermal residual,
Jacobian and covariance equality; calibrated optical projection agreement; and
finite-difference checks. Maximum normalized Jacobian discrepancy is3.738e-7.
All1796RGB replay motions reproduce original native first-write deltas; worst
translation difference1.538e-5m and rotation4.080e-7rad, below the frozen1e-4bounds.
Small accumulated differences from original committed float32 trajectories remain
explicit; scores are for the replay, not byte-identical original endpoints.

Predictions were locked before new reference scoring. Independent auditor passes
3697checks, including all prior hashes, supported/missing source slots,
original-native delta agreement, independent component trajectory accumulation,
all complete full/block position/orientation metrics and missing-path refusal.

Two audit-only corrections are preserved. Initial projection qualification assumed
that a float32-origin quaternion is exactly unit after double promotion. Using
the actual unchanged native-pose matrix resolves it without changing tolerances
or the estimator (`qualification-001/initial-failure.json`). The independent path
auditor initially omitted the existing per-reference properSO3 roundoff policy;
implementing that stated policy resolves the disagreement without changing scores
(`run-001/audit-initial-failure.json`). Final source-pinned qualification003 and
independent audit pass; failures are retained rather than discarded.

## Limits and decision

This is **native optimizer replay with fixed historical RGB parent states and
saved observations**. Fused estimates are not fed back into frontend selection,
map point creation, motion initialization or subsequent native histories. It is
not a continuous two-camera VO integration, live deployment, compute saving,
physical-clock qualification or unseen-condition test. It uses the declared
supplied-pair convention without duration scaling or reference-assisted fitting.

Keep the implementation as a functioning application baseline and retain its
four positive fixed windows. The observed regressions do not justify replacing
the native RGB default or promoting this fixed-weight recipe as general-purpose.
Do not tune another weight on the exposed results. A full feedback replay would
be a separate engineering test, and would need to retain these same controls.

Generic shared RGB/thermal factors are established prior art: the already-read
[MSO Chapter5 review](mso-method-overlap-review-2026-11-12.md) documents that
architecture. This code is a native MAC-VO extension, not MSO reproduction or
an established novel algorithm. Its purpose here is to establish the correct
application baseline after the previous RGB replay mismatch.

Artifacts: `results/native-thermal-constraints/{qualification-001,qualification-002,
qualification-003,run-001}` and source-pinned freeze/prediction lock/score/audit.
