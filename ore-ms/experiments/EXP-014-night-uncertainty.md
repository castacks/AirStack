# EXP-014: Raw uncertainty and retention around night RGB tracking loss

Date: October 20, 2026 (America/New_York). Branch `ore_ms`; base commit
`3ac07f047` plus EXP-010..014 working changes. **Completed; raw uncertainty/selection association observed, no calibration claim.**

## Result and decision

The unmodified RGB model produces a large raw preselection uncertainty signal
in both failing night windows. Median fixed-grid variance trace is 5.74 px² in
the tracked early window, 6499.89 in middle and 6435.59 in late. Candidate
selection collapses before inbound/outlier filtering: middle selects four
points in total, all four survive, while late selects none. Thermal remains
trackable in all three windows. This is an internal warning/availability
signal on these fixed windows, not evidence of independent geometric-error
calibration or physical cross-modal covariance comparability.

All five exported baseline graphs/trajectories, including interpolated middle
RGB, are bitwise unchanged with recording enabled. Six initial depth arrays,
configs and reports/failure evidence are unchanged. The late RGB failure still
occurs after its 19 diagnostic pairs are saved; no absent-graph equality is
claimed. No source pipeline, threshold, weight, mask or preprocessing changed.

## Question and frozen protocol

EXP-013 rejected a general u-only replacement on a separate night recording.
Characterize the original learned uncertainty around its RGB tracking failures
instead of tuning weights or changing windows. Reuse exactly the six 20-frame
cases at starts 123/619/789 from `20251027/night_thermal_so_salon2`.

Enable only the existing `--record-uncertainty` option in the unchanged bounded
runner. Keep checkpoints, uint8 preprocessing, calibration, provisional camera
geometry, fp32, seed 0, mapping disabled, synchronous optimizer, eager math and
one CPU thread unchanged. Launch the same immutable GPU image ID as EXP-013.
Freeze the inherited inputs, original baseline artifacts, recorder, new analysis
and test source before new inference. Retain all failures, including late RGB's
expected no-match error after diagnostic saving.

The existing recorder captures a 16-pixel fixed grid within the 32-pixel border
before selection/floors (RGB 1800 samples, thermal 1008), selected candidates,
target inbound flags, outlier retention, source/target depth and covariance
observer snapshots. Arrays are copies. No selector, recorder, threshold,
covariance floor, model or optimizer code is changed.

Primary per-pair/window measurements are raw uu/vv/trace and native selector
quality `q=uu+vv-2uv`, in camera-local px², plus candidate/inbound/retained counts.
Preserve nonfinite/empty populations explicitly. Report fractions below the
existing component floor 0.0625 and selector quality cap 100, image thirds and
source forward-depth bins [0,10), [10,20), [20,50), [50,100), [100,infinity), plus
invalid source depths. Depth is NED forward depth0, not Euclidean range.

The cap diagnostic is not a reconstruction of the dense selector: the native
threshold is `min(100, 1.5*median(q at dense NMS minima))`, followed by border,
optional matcher-mask and random-count selection. Sparse grid values cannot
identify every rejected dense candidate. Selection already uses covariance,
so association with retention is not independent geometric-error evidence.

Tracking loss is contemporaneous `<10 retained matches`, matching the native
optimization gate. Compute descriptive within-modality Spearman of fixed-grid
median trace versus retained count and contemporaneous AUROC versus this label.
Use no fitted score threshold, significance test, time-ahead prediction claim
or cross-modal pooling of pixel variances. A single-class AUROC is undefined.
These metrics do not establish covariance calibration, comparable physical
confidence or an adaptive-fusion benefit.

## Source-audit correction: boundary flags

Native `MotionInterpolate.elaborate_map` obtains a bool view of motion flags,
then clears its first and last two entries before interpolation. The mutation
is reflected in the final saved flags. Therefore 15 reported interpolation
flags in middle RGB do not count every pair with inadequate observations;
cleared boundary flags do not prove tracking. Retention counts remain the loss
label. Compare stored graph flags after applying this unchanged boundary policy,
while preserving raw low-retention labels separately. No native code is fixed
or changed as part of this characterization.

## Controls and execution status

Five original cases export graphs/trajectories, including interpolated middle
RGB. Require them to be bitwise identical with recording enabled. Require
initial depth arrays, effective configs, input rows, checkpoint, status/error,
tracking counts and reports (except start time) unchanged in all six cases.
Late RGB has no original graph: verify matching failure/input/depth evidence
and retain its 19 raw pair records, without claiming absent-graph equality.

Every recorder pair must join exactly to the original source acquisition time;
inbound/outlier masks must reconcile with retained graph edges where available.
Postprocessed flags must match the documented boundary policy, and all original
source/baseline hashes remain unchanged. Image/depth partitions must sum to
their original populations; no failed or empty pair is dropped.

Forty-five tests pass (six new array/statistic/boundary tests plus 39 relevant
regressions, including existing recorder mutation isolation). The frozen plan
pins 514 source/input/baseline files. Recording and analysis artifacts are under
`results/night-uncertainty/run-001/`. Observer-on/off and association checks completed; results follow below.


## Case-level raw uncertainty and retention

Trace is `uu+vv` before flooring. Raw grid cross-covariance is zero in every
saved pair, so the native selector quality equals trace in these runs. Each
trace/cap value below is the **unweighted median of 19 per-pair values**;
counts are totals over the window. Empty populations stay explicit. Comparisons
of numerical pixel variances across cameras are not physical confidence ranks.

| Window | Modality | Grid trace (px²) | Grid fraction q<100 (%) | Selected | Inbound | Retained | Pairs with <10 retained |
|---|---|---:|---:|---:|---:|---:|---:|
| early | rgb | 5.74 | 91.83 | 3800 | 3791 | 3791 | 0/19 |
| early | thermal | 1.10 | 94.54 | 3800 | 3748 | 3748 | 0/19 |
| middle | rgb | 6499.89 | 0.00 | 4 | 4 | 4 | 19/19 |
| middle | thermal | 15.46 | 72.72 | 3796 | 3677 | 3677 | 0/19 |
| late | rgb | 6435.59 | 0.00 | 0 | 0 | 0 | 19/19 |
| late | thermal | 0.54 | 100.00 | 3336 | 3313 | 3313 | 0/19 |

RGB middle has nonempty selection in only one of 19 pairs. Its selected/retained
trace median of 42.56 px² is conditional on those four surviving points; it
would obscure the broad fixed-grid signal around 6500 px². At most 2 of 1800
middle grid samples per pair (0.111%) have q<100; every late grid sample has
q>=100 in all 19 pairs. These sampled cap statistics support a selector-stage
availability diagnosis, but do not reconstruct the full dense NMS, adaptive
threshold, border and matcher-mask stages or isolate their separate causes.

Inbound and retained totals are identical in all six cases: the covariance
sanity outlier filter drops none of the inbound candidates. The observed RGB
starvation therefore occurs at selection, before inbound/outlier filtering;
changing downstream pose weights cannot create the missing observations.
This does not identify the physical reason for large network uncertainty or
prove that its numerical scale is calibrated.

The median fraction of RGB raw variance components below the existing floor
is zero in all three windows. The fixed-grid warning is measured before
flooring; it is not a propagated-3D-covariance/floor measurement.

## Image-region and forward-depth sensitivity

RGB raw grid trace, median of per-pair medians within each source-image third:

| Window | Upper (px²) | Middle (px²) | Lower (px²) |
|---|---:|---:|---:|
| early | 21.00 | 4.73 | 2.49 |
| middle | 7933.16 | 6146.35 | 5836.59 |
| late | 7133.79 | 5670.44 | 6318.26 |

The high signal appears in all three thirds of the failed windows; it is not
confined to a single image strip in this diagnostic. This does not justify a
region mask. Grid shares differ slightly from exact thirds because of the
32-pixel border and 16-pixel sampling pitch.

Median per-pair source forward depth estimates shift from 4.47 m early to
0.88/1.02 m middle/late. Median grid fraction in the [0,10) m bin is
74.1%/100%/100%; larger-depth bins are often empty in the failed windows.
These are model-derived source depths, not reference-validated distances or
evidence for a fitted scale correction. The full `strata.csv` records defined
pair counts and invalid/empty bins, so sparse nonempty tails are not treated
as a full-window population. No depth cap is used in primary summaries.

## Contemporaneous association and label interpretation

Across 57 RGB pairs, fixed-grid median trace versus retained count has
Spearman -0.8255, and contemporaneous AUROC versus `<10 retained` is 1.0.
All 38 failed pairs belong to middle/late; all 19 nonfailed pairs belong to
early. The observations are dependent, window conditions are confounded with
labels, and covariance already gates selection. No threshold was trained,
no p-value or future-prediction test is reported, and this is **not** an
independent geometric-error or generalization result.

Thermal has no low-retention pairs, so AUROC is undefined. Its trace/retained
Spearman is +0.4195, warning that feature count alone is not a universal
geometric-reliability target. Lower raw uncertainty need not yield more NMS
points. Do not pool camera-local variance scales across modalities.

Raw middle RGB has 19 low-retention pairs even though its original saved
postprocessing report has 15 interpolation flags: first/last two motion flags
were cleared by the unchanged native interpolation policy. Late RGB also has
19 low-retention pairs; its baseline exports no graph/trajectory, so 15 is only
the policy's computed boundary-cleared count, not a newly exported flag count.
The analysis preserves both raw labels and the postprocessing distinction.

## Controls, artifacts and scope

Six unchanged-case recording attempts produce 114 diagnostic pair files, each
joined to its exact acquisition time. All five graph and trajectory comparisons
and all six initial depth comparisons are bitwise equal; effective configs,
status/error, input rows, checkpoint and tracking reports match except start
time. Retained masks join to original graph edges and the native boundary-flag
policy wherever a graph exists. All image/depth population counts reconcile.
All 514 frozen source/input/original-baseline file hashes pass after analysis.
All 45 tests pass. The expected late RGB no-match failure is retained.

Runtime IDs/settings are identical to EXP-013; `execution-plan.json` records
the sole added `--record-uncertainty` option and exact commands before reruns.
`results/night-uncertainty/run-001/` contains the frozen plan, six recordings
and logs (including failure), 114 raw NPZs, full per-pair/stratum `analysis.json`,
`pairs.csv`, `cases.csv`, `strata.csv`, `case-summary.json`,
`uncertainty-retention.png/pdf` and validation/output-hash provenance.
`results/night-uncertainty/staging/` retains the exact execution/reporting
scripts. No raw input, weight or credential is committed.

This experiment does not compare uncertainty to independently measured actual
geometric error, establish physical covariance comparability, or calibrate a
fusion confidence. Provisional thermal geometry, original preprocessing,
reference quality and checkpoint-training membership remain unresolved.

## Next bounded measurement

Use the already recorded fixed grid to test raw uncertainty against a geometric
consistency target that is not defined by the selector: reference-conditioned
reprojection and depth-independent epipolar residuals, with invalid/inbound
counts retained, explicit motion and depth strata, and no fitted scale or
threshold. Keep rejected grid samples instead of looking only at retained
features. Recorded SLAM and provisional calibration still make these proxy
consistency measurements rather than independent ground-truth errors. Define
that measurement protocol before evaluation; do not adopt fusion or change
selector/weight defaults from the retention association alone.
