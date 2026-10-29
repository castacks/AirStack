# EXP-078: causal uncertainty and health diagnostics for pose quality

2026-10-29 America/New_York. Branch `ore_ms`, base `758240925`.

Five fixed uncertainty/health diagnostics have been characterized against the
existing conditional pose proxy. None consistently beats the best fixed expert
across the examined windows. In the day window, native and angular flow-trace
preferences choose the lower translation-loss expert58/98 times but have RMSE
about0.0440m, worse than fixed RGB0.03146m. Both avoid the known night RGB outlier;
retained-match fraction selects that outlier instead. No diagnostic is adopted
or combined into a fitted gate.

## Fixed scope and populations

Reuse all217 original frozen deadlines: night early19 and middle99 from
EXP073/074, and day middle99 from EXP077. There are434 expert slots and331
deadline-available first motions. Both experts are eligible on18 night early,
1 night middle and98 day middle intervals:117 comparisons total. Keep every
missing/unsupported slot and original final abstention; do not replace them
with post-refinement poses. Only two recordings, with all outcomes already
examined. No new inference, scene, reserved MS² data, physical labels or IID/
significance/generalization claim.

Freeze feature definitions/directions before this feature/loss join. Extract
features without loading labels, then separately join independently validated
EXP075/077 nominal-body, reference-assisted interval SLAM-proxy losses. All
reference, physical body-anchor, signed geometry and timing-bridge limitations
remain. These are conditional software diagnostics, not calibrated pose risk.

## Five predeclared diagnostics

| Diagnostic | Fixed preference | Definition |
|---|---|---|
| Native flow trace | Lower | Median of raw temporal uu+vv, px² |
| Angular flow trace | Lower | Median trace(J C Jᵀ), nominal rad², at model target ray |
| Retained fraction | Higher | Retained matches divided by selected points |
| Spatial coverage | Higher | Number of occupied3×3 source-image cells |
| Prior support | Higher | Previous original pair supported and no execution error; cold start unknown |

Native/angular traces share the exact same fixed-grid population: finite model
endpoints, finite PSD uu/vv/uv covariance with positive diagonal entries. Keep
the full cross term. Do not mask by target inbound state, reference loss or a
camera-specific fitted threshold. Record all valid/invalid counts; no valid
points yields null uncertainty rather than zero. Grid locations and fields of
view differ across cameras, so common angular units do not imply identical
scene support or physical calibration. Use the unchanged EXP030 ray Jacobian.
For exact source reproduction, preserve the supplied K dtype during inversion;
the independent audit reconstructs propagation through a separate analytic metric.

Counts/coverage and prior support come only from current/preceding pair records
in the original observed prefix. Ignore newer-than-target intervals. The raw
NPZ files were serialized at run finish but contain copied tensors that existed
during selection/pair processing. This is offline computation from causally
derivable tensors, **not measured runtime feature availability or latency**.
Exact time0, source pair/event IDs, raw counts/coverage and prefix/deadline checks
bind every row to the correct interval. No future/refined trajectory or outcome
summary is a feature. Missing prior support at pair0 stays null.

For each diagnostic, compare both values only when both experts are originally
eligible and both feature values exist. Exact feature ties use the fixed RGB
preference; a missing value abstains diagnostically and remains in missing-count
accounting. A reference translation tie uses the fixed1e-12m tolerance. Report
decisive cases separately from tie-default cases. Also report within-window,
per-camera Spearman associations against separate translation/rotation losses;
fewer than3 samples or constant ranks yields null, not zero association. No
feature signs, thresholds or combinations are selected from these outputs.

## Matched proxy translation results

The first four diagnostics have complete values on all117 both-eligible pairs.
Every cell below uses the same eligible population as its column's fixed experts.
RMSE values are in meters under the conditional SLAM proxy.

| Preference | Night early,18 | Night middle,1 | Day middle,98 |
|---|---:|---:|---:|
| Fixed RGB |0.009891|1.003081|0.031459|
| Fixed thermal |0.009248|0.040287|0.077786|
| Lower native trace |0.009315|0.040287|0.043952|
| Lower angular trace |0.009315|0.040287|0.043991|
| Higher retained fraction |0.009745|1.003081|0.050389|
| Higher spatial coverage |0.009750|0.040287|0.042032|

Angular conversion changes some day choices/losses but does not make this scalar
preference better than RGB on the day window. Native/angular preferences each
agree with the lower translation-loss camera13/18 early,1/1 middle and58/98 day.
Fixed RGB agrees63/98 day. Avoid interpreting correct-choice counts as risk:
spatial coverage agrees65/98 day yet has higher translation RMSE than fixed RGB.
The severity of the remaining mistakes matters.

Day spatial coverage is decisive only22/98 times (16 correct);76 cases tie and
inherit RGB. Early coverage is decisive5/18 times (4 correct), with13 RGB-default
ties. Retained fraction is decisive97/98 day (48 correct), with one RGB tie.
The full per-window decisive/tie/missing ledger is retained, preventing default
choices from being credited as additional feature discrimination.

Prior support is undefined at the two cold starts, leaving17 early,1 middle
and97 day comparable pairs. It ties on all17 early and97 day intervals; their
preferences therefore reproduce RGB. Only the known middle interval is decisive
and favors thermal, reproducing the recovery context already identified in
EXP076. It supplies no both-healthy discrimination in these two supported windows.
No correlation is estimated from the one common-eligible night middle sample.

On that middle RGB outlier, retained fraction is1.0 (18/18) while thermal is
0.94915 (168/177).
The diagnostic favors RGB despite its1.00308m proxy residual versus thermal
0.04029m. Native/angular uncertainty, spatial coverage and prior support favor
thermal there. A ratio alone cannot summarize absolute geometric support or
motion quality; no new feature is tuned or added after observing this result.

## Optimizer information audit

All331 eligible first-motion records have null `optimizer_convergence` and null
`pose_covariance`. There are no recorded nonnull values for either field. Their
absence is retained explicitly; match counts, covariance-grid summaries and
observation clocks are not substituted or relabeled as these quantities. Native
flow covariance describes temporal image correspondence, not a6-DoF pose covariance.

## Implementation and verification

`arbiter_quality_features.py prepare/extract` freezes source/raw/proof hashes and
materializes the label-free feature table. `analyze_arbiter_quality.py` separately
joins validated proxy labels and reports all five predeclared directions,
matched fixed/oracle losses, full decisive/tie/missing counts and per-camera
rank associations. It does not emit an adopted selector or alter prior decisions.

Fourteen Docker CPU tests pass: analytic angular units, resolution scaling,
nonzero covariance cross term, non-PSD/nonfinite/zero-diagonal/empty grids,
unknown history, newer-pair invariance, raw-time/count/coverage/duplicate failures,
fixed preferences and undefined rank populations. An independent validator
imports neither producer nor analyzer and reconstructs all217 rows,434 slots,
609336 raw grid points,117 both-eligible preferences and all summary populations.
Its separate analytic Aᵀ(I−nnᵀ)A/||r||² covariance metric checks the Jacobian trace.

Eight semantic mutations are rejected after refreshing outer digests: native
uncertainty, angular term, future prior ID, history bit, invented optimizer
covariance availability, decisive count, loss population and label-loaded claim.
A read-text/read-bytes guard prohibits proxy/reference paths during extraction
and yields a byte-identical feature table. No frozen source/raw input changes.
Checks and adjacent intervals are not independent scientific samples.

Authoritative artifacts: `results/arbiter-quality-signals/`, containing CPU JUnit,
label-access guard, mutation results and `run-001/{protocol.json,features.json,
analysis.json,validation.json}`. Feature SHA256:
`aa2e57d6b835f3a4b4fea375e851acf9016116f4d0febfbd8009c8cc66d3d707`.
Analysis SHA256:
`d79ce05334026b0ce02707fc8cea0869d93eb9dfb40d4117f3b5e4c9c9f71a2d`.
The existing `macvo:event-tests` CPU image suffices; no model weights/GPU needed.
From the MAC-VO root in that runtime, with a fresh output:

```bash
python -m Scripts.Setup.arbiter_quality_features prepare --root /workspace --output <fresh-run>
python -m Scripts.Setup.arbiter_quality_features extract --root /workspace --output <fresh-run>
python -m Scripts.Setup.analyze_arbiter_quality --root /workspace --run <fresh-run>
python -m Scripts.Setup.validate_arbiter_quality --root /workspace --run <fresh-run>
```

Recovery: `backups/2026-10-29-exp-078`, with EXP073–077 journals/raw fields/proxy
labels as dependencies. No images, weights, credentials or full source archives
are included.

## Decision

Retain these as diagnostics, with history as the existing recovery control.
Common angular units and healthy tracking do not by themselves provide a reliable
cross-camera pose-quality ordering here. This does not refute published MAC or
thermal VO gains: their observation uncertainty can help their own estimators
without the median trace serving as a transferable expert-selection score.
Next inspect whether saved optimizer factors can support read-only residual and
pose-information summaries, with explicit observability/approximation limits,
before fitting a richer gate. Do not fabricate the missing pose covariance or
promote a retrospective diagnostic winner as a validated selector.
