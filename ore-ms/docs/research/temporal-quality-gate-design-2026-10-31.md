# Temporal quality gate: a concrete development design

2026-10-31 America/New_York; branch `ore_ms`, design base `ca3238af5`.

**Status: draft candidate, not fitted and not a complete prospective freeze.**
This document specifies one small model and its controls. It assigns no reserve
recording to a test partition, starts no training/inference and supplies no
physical or held-out performance claim. The [exposure ledger](recording-exposure-ledger-2026-10-31.md)
remains in force.

## Hypothesis and distinction

The candidate tests whether joint current/prior estimator health and matching
uncertainty can select the lower *whole-motion translation proxy loss* when both
experts are supported. EXP062–064 learned point-level stereo geometry selectors;
EXP078–087 tested individual diagnostics, history availability and solver/flow
controls. This candidate changes the target and joint causal inputs. It is not
novel merely because it is called an agent, uses uncertainty or has more features.
Any added value must survive fixed-expert/fallback controls and recording-level
comparisons. The existing history rule remains a baseline, not evidence that a
new quality gate already works.

Keep both experts, MSA preprocessing, checkpoints and the EXP088 interval model
unchanged. Use EXP089's original observed controller boundaries and nominal
body/SLAM-proxy evaluation scope during development. Its unverified physical
frame/acquisition-clock/live-latency flags remain false. No reference-assisted
endpoint bridge is a predictor input.

## Exact candidate inputs

For each camera, take the current native interval and its two immediately
preceding native intervals, using only their pair-health records received by
the current controller decision. A prior interval may have arrived after its
own old decision but must be present before this current decision. Do not use
cached future-refined poses, run outcomes, reference errors, recording IDs,
absolute timestamps or scene labels as features.

At each camera/interval, construct these seven coordinates:

| Coordinate | Exact construction from an observed pair-health event |
|---|---|
| Retained fraction | `retained_count / max(selected_count, 1)` |
| Inbound fraction | `inbound_count / max(selected_count, 1)` |
| Spatial occupancy | `occupied_3x3_cells / 9` |
| Finite positive-diagonal grid fraction | `grid_positive_uu_vv_count / max(grid_count, 1)` |
| Native uncertainty magnitude | `log1p(q50)` from `grid_trace_q10_q50_q90_px2` |
| Native uncertainty spread | `log1p(q90) - log1p(q10)` from that same event |
| Original support flag | `1` iff `original_need_interp is False` and `execution_error is None`, else `0` |

These give 42 ordered coordinates: camera RGB then thermal; lag 0,1,2; table
order within each lag. Quantiles must be present, finite, nonnegative and
ordered. Counts must be observed valid integers with inherited ordering checks.
The recorder's grid mask checks finite covariance entries and positive diagonal
terms; it does **not** independently certify positive semidefiniteness or a
calibrated uncertainty probability. Quantiles summarize raw image-space temporal
covariance, not 3D or pose covariance. Different camera units/populations remain
separate columns. Do not reinterpret their raw ratio as physical risk.

Missing health/quantiles or two-lag cold start produce no gate decision; use the
unchanged fallback. Explicit zero counts use the formulas above and retain the
support flag; they are not interpolated measurements. No mean imputation or
fitted missing-value threshold is proposed. Optimizer convergence, pose covariance
and angular-grid summaries requiring later raw files are excluded because the
present observed-event contract does not establish those runtime inputs.

## Fixed model, objective and action

Use one linear score `s=b+w^T z`, with independent training-only feature means
and population standard deviations for every coordinate. A constant training
column has scale 1; validation/test data never update normalizers. No polynomial
terms, model search, learned recurrence or per-record identity parameters.

For a training pair with both supported projected motions, let `e_R,e_T` be the
existing nominal-body translation losses on the same RGB interval. Set
`y=1` for `e_R<e_T`, otherwise `y=0` for strict thermal wins; exact ties carry
zero cost and stay in coverage/evaluation reports. Let `g=abs(e_T^2-e_R^2)` so the pointwise cost uses the squared
translation loss underlying the primary RMSE.
Within each training recording, normalize positive-gap sample weights to sum
1; give each nonzero-gap recording equal total weight. Record all tie/zero-gap
recordings explicitly. Fit weighted binary logistic loss plus
`0.01/2 * ||w||^2`, leaving the intercept unpenalized. This fixed regularization
is a design choice, not an optimized value or a claim of suitability.

Proposed deterministic solver: float64 L-BFGS, zero initialization, at most
2000 iterations and gradient tolerance `1e-9`. Save coefficients, normalizers,
objective, convergence status and source/config hashes. If training has no
positive costs, one weighted target class, or solver failure, report that
failure; do not add pseudolabels, switch optimizers or select another penalty
after outcomes. No fitting is performed by this document.

At prediction, invoke the gate only when both current model projections are
available and all 42 current/prior features are complete. Choose RGB for
`s>=0`, thermal for `s<0`; no score-threshold search or confidence abstention.
Otherwise use EXP089's unchanged `raw_advice` **model action**, including its
explicit source/model unavailability. This makes cold start fall back rather
than pay the extra abstention of the history baseline. Any eventual code must
verify fallback coverage is preserved. The weighted score is a selection
margin, not a calibrated probability or physical covariance.

The chosen increment remains an EXP088 model projection. It does not become a
verified body-motion admission, continuous trajectory, live deadline result or
independent physical estimate through this gate.

## Prespecified controls and comparisons

Primary candidate: the combined 42-coordinate temporal gate. Report these
ablations without selecting a new primary after outcomes:

- uncertainty-only: the two quantile coordinates per camera/lag, 12 inputs;
- health/history-only: the remaining five coordinates, 30 inputs;
- current-only combined: both cameras at lag 0, 14 inputs.

Use the same objective, penalty and training-only normalization procedure for
each family. Baselines remain fixed RGB, fixed thermal, raw availability fallback
and the existing symmetric two-interval history rule, all through the same
interval/output contract. An oracle is scorer-only. Training targets and
predictor inputs must be persisted separately, with label-removal/mutation and
future-event tests before fitting.

Report full declared-population availability, exact matched interval IDs,
translation RMSE and separate rotation outcomes per recording. Add an
equal-recording summary rather than pooling long clips. Keep failures, ties,
empty comparisons, unsupported motions and cold starts explicit. Development
loss reduction alone is not transfer evidence. Any future claim of consistent
improvement follows the existing strict [recording protocol](rig-reference-contract-2026-10-30.md):
beat both fixed experts and fallback on each test recording's matched primary
population, with coverage at least fallback and visible rotation/timing costs.
No new empirical success threshold is chosen from the audit below.

## Current development adequacy, before any fit

A descriptive audit reads existing EXP089 timelines and scores, verifies their
saved PASS/hash chain (including all34 frozen EXP089 input hashes)
and checks candidate feature availability at each original
controller prefix. It introduces no predictor, new error computation or fit.
The 217 decisions come from **two** recordings: early/middle night share one.

| Recording | Decisions | Both supported | Both supported + complete two-lag features | RGB / thermal strict proxy wins in complete subset | Largest absolute-gap share | Largest squared-gap fit-weight share |
|---|---:|---:|---:|---:|---:|---:|
| `0_salon_so_day` |99|98|96|65 /31|10.35%|17.59%|
| `night_mac_salon_warehousefig8` |118|19|17|6 /11|98.80%|99.98%|

There are 117 both-supported pairs overall and only 113 with complete two-lag
features. The night maximum is the known middle pair96 isolated RGB outlier;
it contributes98.80% of the absolute-error gap and99.98% of the
RMSE-aligned squared-gap fit weight in that recording. This is a direct warning about the proposed objective's
sample concentration, not a reason to clip the target after seeing it. Removing
the outlier or changing weights to improve results is not part of this design.

The feature-availability audit checks the actual recorder fields, including
its finite/positive-diagonal grid definition. A separate Decimal aggregation
recomputes target classes, counts and absolute-gap shares: 254 checks pass.
A supplemental Decimal calculation checks the squared-loss difference identity
for all113 eligible rows and reports the squared-gap weight concentration.
The squared-loss objective is specified to match RMSE, not selected through a
comparison of fitted outcomes; no gate has been fitted under either weighting. It is an
arithmetic check, not a statistical sample-size guarantee. These data do not
support a credible new performance claim merely by fitting a 42-input score.
No candidate is trained, and neither reserve is consumed.

## Next bounded collection: a third development recording

Use already-exposed `20251027/1_salon_mac_cov_day`, not a new reserve. Reuse
**all three** exact EXP023 day windows (original/early/late), 20 paired groups
per window. EXP023 records original manifest starts348/306/1132 respectively;
these are historical full-manifest positions, not offsets to apply blindly to
an already sliced prepared manifest. Restore its saved `input_rows`, prepared
MSA thermal image/camera-info hashes and corresponding unchanged RGB inputs,
then bind every source timestamp/hash before preparing the shared replay.

Preserve the existing body/rectification/reference conventions and original
checkpoints. Reuse the sensor-ordered, one-group-lag shared controller and its
final pre-drain boundary. Capture all three windows and corresponding unchanged
controls; retain technical failures and every output. No brightness/outcome-based
window replacement, clock shift, thermal recipe change or repeated attempt
chosen for a better score. A source/protocol mismatch stops the planned capture;
it does not authorize a substitute dataset. This is a proposed collection plan,
not an executed or currently source-hash-frozen acquisition.

The next implementation must first restore the bounded EXP023 source/control
artifacts, check GPU/runtime availability and freeze the exact source/image/
checkpoint/config/manifest/controller hashes. The current session's GCS CPU
image suffices for this design audit; a compatible MAC-VO GPU runtime and those
bounded inputs are not asserted ready. No new image upload is needed to inspect
the existing recovery provenance. This draft does not start that GPU job.

A third recording would expand development conditions and permit a recording-
excluded diagnostic across three groups; it does not guarantee adequate classes,
statistical power or prospective validation. Multiple windows remain one group.
Before fitting, bind a complete recording/feature/label ledger and publish the
fixed development-only fitting declaration. Before any future test access, freeze
actual whole-recording train/validation/test IDs, method artifacts and reference/
clock/deadline scope. The two MS² reported reserves remain unassigned; this draft
does not assert their temporal target or cross-rig transport is ready. Checkpoint
training membership and recording-specific physical provenance remain separate
unresolved inputs.

## Artifacts and status

Design feasibility artifacts: `results/temporal-gate-design/run-001/`.
No unit tests were added for this documentation/analysis step. Existing model,
predictor, recorder, checkpoint and production defaults are unchanged. No neural
inference, fitting, new scientific payload, reserve access or external message
occurred.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-temporal-gate-design`. Includes this
draft, feasibility input/proof snapshots and output, arithmetic-check scripts and
results, notebook and incremental Git history based on `ca3238af5`. No scientific
image archives, checkpoints or credentials are included. This is a design record,
not a completed prospective experimental freeze.
