# EXP-080: fixed factor diagnostics against the qualified pose proxy

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `f803da2ee`.

Four fixed factor-diagnostic preferences improve some inspected night cases but
all worsen translation and rotation proxy RMSE relative to fixed RGB in the day
window. This supports retaining factor diagnostics for investigation, without
adopting a cross-camera selector or changing the history/estimator defaults.

## Frozen question and data boundary

Following EXP079, declare four separate lower-is-preferred diagnostics before
this factor/proxy join: median reprojection residual, median absolute disparity
residual, whitened mean square residual per component, and the mixed metre/radian
right-camera Jacobian condition number. No feature direction, threshold,
combination or subset is fitted. Protocol and label-free predictions were saved
before the label join. Historical proxy results and EXP079 descriptive factor
summaries had already been inspected; this is a retrospective declaration on
previously examined data, not an unseen-data preregistration or held-out test.

Use all 217 original decisions: early night19, middle night99, middle day99.
The two night windows share a recording; day is the only other recording.
EXP079's six terminal first writes are excluded from the original decision
features. Every available diagnostic is bound to its exact source motion-event
ID and observation time before that decision deadline. Native fixed-landmark
factor calculations were reconstructed offline; their computation latency was
not measured. Write-time eligibility does not establish a real-time gate.

When both experts are available, compare their fixed diagnostic values; exact
feature ties prefer RGB and missing/nonfinite values abstain diagnostically.
When just one expert is available, use that expert as an explicit availability
fallback; no available expert means abstain. Fixed RGB-only, thermal-only,
RGB-preference and unchanged two-interval history controls are retained.

A separate scoring step joins only the independently validated EXP075/077
`common_loss` labels with exact original decision identities. Primary loss is
translation norm in metres and secondary loss is rotation geodesic in degrees;
there is no combined score. The nominal body transforms and reference-assisted
interval transport are inherited, not recalculated or promoted to physical
truth. Recorded SLAM is a qualified proxy; source dependence, asynchronous
bridging and physical body/calibration limitations remain. No new images,
weights, reference, inference, solver run or reserved MS² recording is used.

## Matched both-available results

Primary common support is 117 intervals: early18, middle1, day98. All four
factor diagnostics are finite/comparable and decisive in all these cases: zero
feature ties or missing values. Neither loss has a reference tie within the
fixed absolute1e-12 tolerance. Every table row below uses the same both-available
intervals as its window's fixed-camera controls. Correctness is an interval
count, not an independent-trial accuracy estimate.

| Window | Diagnostic | Pairs | Translation RMSE m | Rotation RMSE deg | Correct translation / rotation |
|---|---|---:|---:|---:|---:|
| Early night | Reprojection residual |18|0.009230|0.030553|14 / 16|
| Early night | Disparity residual |18|0.009248|0.032309|13 / 15|
| Early night | Whitened residual |18|0.009248|0.032309|13 / 15|
| Early night | Condition number |18|0.009891|0.069551|5 / 3|
| Middle night | Reprojection residual |1|0.040287|1.326845|1 / 1|
| Middle night | Disparity residual |1|0.040287|1.326845|1 / 1|
| Middle night | Whitened residual |1|0.040287|1.326845|1 / 1|
| Middle night | Condition number |1|0.040287|1.326845|1 / 1|
| Middle day | Reprojection residual |98|0.074255|0.482090|40 / 17|
| Middle day | Disparity residual |98|0.045588|0.218343|56 / 83|
| Middle day | Whitened residual |98|0.074820|0.592250|33 / 14|
| Middle day | Condition number |98|0.044139|0.388001|51 / 60|

| Window | Fixed camera | Pairs | Translation RMSE m | Rotation RMSE deg |
|---|---|---:|---:|---:|
| Early night | RGB |18|0.009891|0.069551|
| Early night | THERMAL |18|0.009248|0.032309|
| Middle night | RGB |1|1.003081|50.187642|
| Middle night | THERMAL |1|0.040287|1.326845|
| Middle day | RGB |98|0.031459|0.124174|
| Middle day | THERMAL |98|0.077786|0.615784|

Residual preferences improve over fixed RGB in early night; disparity and
whitened residual reproduce thermal-only choices there. Reprojection differs
from thermal at one interval and gives a small further proxy improvement.
Conditioning chooses RGB throughout early night, matching its loss. All four
choose thermal in the single both-available middle-night case, avoiding the
known RGB1.00308m outlier versus thermal0.04029m. One interval cannot establish
transfer or distinguish the four diagnostics.

Every diagnostic is worse than fixed RGB in both loss RMSEs in day. For example,
disparity chooses the lower translation-proxy-error camera in56/98 cases, but
its0.045588m translation RMSE is worse than fixed RGB0.031459m. Winner counts and
aggregate error answer different questions; neither overrides the other.
All per-camera Spearman coefficients and their count/constant/fewer-than-three
states are retained in the machine report, without significance testing or
choosing a score/direction from those correlations.

## History and coverage

On primary both-available support, compare history only on the identical
intersection where history returns a camera: early17, middle1, day97 (115 total).
Cold-start abstentions are excluded from both sides and their coverage cost is
reported separately. No policy's own full-support loss is compared as though
it used this intersection.

| Window | Diagnostic | Matched pairs | Diagnostic / history translation RMSE m | Diagnostic / history rotation RMSE deg |
|---|---|---:|---:|---:|
| Early night | Reprojection residual |17|0.009353 / 0.010000|0.031234 / 0.062613|
| Early night | Disparity residual |17|0.009372 / 0.010000|0.033052 / 0.062613|
| Early night | Whitened residual |17|0.009372 / 0.010000|0.033052 / 0.062613|
| Early night | Condition number |17|0.010000 / 0.010000|0.062613 / 0.062613|
| Middle night | Reprojection residual |1|0.040287 / 0.040287|1.326845 / 1.326845|
| Middle night | Disparity residual |1|0.040287 / 0.040287|1.326845 / 1.326845|
| Middle night | Whitened residual |1|0.040287 / 0.040287|1.326845 / 1.326845|
| Middle night | Condition number |1|0.040287 / 0.040287|1.326845 / 1.326845|
| Middle day | Reprojection residual |97|0.074498 / 0.031505|0.483463 / 0.124781|
| Middle day | Disparity residual |97|0.045595 / 0.031505|0.217012 / 0.124781|
| Middle day | Whitened residual |97|0.075066 / 0.031505|0.594395 / 0.124781|
| Middle day | Condition number |97|0.044285 / 0.031505|0.389986 / 0.124781|

Full deadline populations are also preserved: early18both/1none, middle1both/
97thermal-only/1none, day98both/1none. Each diagnostic with availability fallback
returns214/217 estimates; unchanged history returns211/217. RGB-only returns117
and thermal-only214. Three end-of-stream deadlines abstain before optimizer
drain. History has an additional cold-start abstention in each window. Full
coverage, losses and all pairwise matched comparisons with four controls are
recorded, including the97 middle-night fallback intervals; those availability
gains do not constitute an accuracy gain over thermal-only.

## Differences from existing quality signals

All four new diagnostics are compared against all five EXP078 signals on their
same both-available support. All60 window×new×old panels retain agreement,
disagreement, missing-old, pair IDs, matched translation/rotation losses and
better/worse/tie counts. Missing prior-support at cold start is retained.
The complete table is `disagreements.csv` beside the analysis JSON.

For an explicit check, early reprojection differs from native-flow trace at
one interval and is better in both proxy losses there. At day, reprojection
differs from native trace on70 intervals: translation is worse44/better26,
rotation worse70/better0. Day disparity differs on24: translation worse13/
better11 and rotation worse14/better10. Day conditioning differs on35:
translation worse21/better14 and rotation worse31/better4. Whitened residual
differs on79: translation worse52/better27 and rotation worse76/better3.
Other old-signal comparisons, including spatial and prior support, are retained
without choosing favorable panels. These differences describe additional
choices, not an independent predictive contribution, calibrated risk model or
causal explanation of pose error.

## Validation and reproduction

15 CPU fixtures pass: lower/higher directions, exact ties, missing/nonfinite
features, availability fallback, empty moments, insufficient/constant ranks,
opposite translation/rotation winners, reference ties, old-feature missingness,
matched support and history abstention. An independent validator imports neither
predictor nor scorer. It reconstructs217 decisions,868 new and1085 old choices,
qualified label joins, all coverage/population IDs, both losses,60 disagreement
panels and every reported moment/correlation. All checks pass.

A runtime guard blocks proxy/reference/model data reads during prediction and
reproduces byte-identical prediction JSON. Nine semantic mutations, with updated
outer prediction digests, are rejected: terminal/future values, changed metric,
fixed-camera control, history control, omitted row, fabricated coverage, changed
proxy loss, changed disagreement count and source hash. Frozen source/input
hashes are checked at extraction and scoring; no historical artifacts changed.

Execution used the existing CPU AirStack robot image
`sha256:ae59352813af040d1a60000f2b13eed990b8364f8a416f5b87e385d4496aade7`,
Python3.12, NumPy1.26.4, SciPy1.17.1, `OPENBLAS_NUM_THREADS=1`, `runc`, and
`NVIDIA_VISIBLE_DEVICES=void`. No GPU was used. Standalone pytest disables the
repository's jaxtyping addopts; no neural/estimator test rerun is claimed.

From MAC-VO root, ore-ms mounted at `/workspace`, using a fresh output directory:

```bash
python3 -m pytest -o addopts= Scripts/UnitTest/test_factor_proxy_comparison.py -q
python3 -m Scripts.Setup.compare_factor_proxy prepare --root /workspace --run <fresh-run>
python3 -m Scripts.Setup.compare_factor_proxy predict --root /workspace --run <fresh-run>
python3 -m Scripts.Setup.compare_factor_proxy score --root /workspace --run <fresh-run>
python3 -m Scripts.Setup.validate_factor_proxy --root /workspace --run <fresh-run>
```

Authoritative output: `results/factor-proxy-comparison/run-001/` with frozen
protocol/predictions, complete analysis, validation, guard, mutation evidence and
CSV. Recovery target `backups/2026-10-29-exp-080`, dependent on EXP075–079 saved
labels, old signals, history and factor audit, plus EXP073/074/077 raw provenance.

## Decision

Keep factor diagnostics and the unchanged history control. Observed night gains
and the known collapse case do not justify a transferable quality gate; the
separate inspected day condition exposes substantial errors. Do not tune a new
threshold on these windows, train against support flags supplied as inputs, or
present their SLAM-proxy losses as independent physical accuracy. A next useful
step is to inspect concrete disagreement cases and objective/geometry failure
mechanisms in the same saved data under a separately declared audit, before
another fitted selector or additional data consumption.
