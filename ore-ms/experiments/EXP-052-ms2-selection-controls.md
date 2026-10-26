# EXP-052: Depth and image-location controls for uncertainty selection

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `b9ee6349c`.

EXP051 established that low covariance trace selects lower squared error than random retention against MS²'s provided depth references. This follow-up tests whether simple depth/location selection can do as well, and whether the advantage persists when selection preserves coarse scene composition. It is an exploratory follow-up on previously inspected windows, with the controls specified before reading these results.

## Fixed analysis

Preserve the four windows,20 original source IDs each,both cameras,all three reference products and own/common support:960 cases,each at100%,75%,50%,25% retention. Primary endpoint is single-scan own-support risk at50% retention; other reference products,supports and coverages remain sensitivities. Repeated views and adjacent images are not independent samples. No new data,model inference,clock correction,training,fitting,gate changes or fusion.

Loss is squared L2 normalized reference residual. Global uncertainty ranking uses normalized covariance trace, exactly as EXP051. Four fixed baselines rank farther depth,nearer depth,central location and peripheral location,all separately. Both directions are retained; there is no outcome-selected winning direction or per-image baseline fit. Depth uses the provided reference and is a diagnostic baseline,not a deployable depth source. Image radius squared is `((x-(W-1)/2)/(W/2))² + ((y-(H-1)/2)/(H/2))²`, using full dimensions from each source-ID manifest,not the extent of sparse labels.

A second comparison retains the same fraction separately within each of the existing six depth bands `[0,5),[5,10),[10,20),[20,40),[40,80),[80,inf)` crossed with a3×3 image grid. Thus selection cannot move its retained mass between these54 bins. Own support uses the case's depth product;common support uses single-scan depth at exactly the same coordinates for every product,as in earlier controls. The grouping is coarse and reference-dependent;it does not eliminate all scene/depth confounding.

Within each nonempty bin retain low uncertainty,and also compute an ideal loss-ranked oracle. Fractional equal inclusion of cutoff ties gives exact expected coverage,including singleton/constant bins. Every pixel remains in the evaluation population;there is no minimum-count exclusion. Counts for all54 bins and pixels in bins smaller than30 are retained. This is a new selection estimand,not a relaxation or rerun of EXP047's minimum30 rank-correlation eligibility. For primary own support,the sparse-bin pixel fraction ranges0.012%–3.166% (median0.745%);for the repeated single-scan common-support sensitivity it ranges1.108%–40.159% (median4.875%). Small bins can limit selection freedom,especially for common support,without making the expectation undefined.

Matched random retention has expected MSE equal to the full-support mean because every bin retains the same fraction. The primary ratio is stratified uncertainty MSE / matched-random MSE. Global complementary ratios compare uncertainty MSE with each of the four fixed baseline MSEs at the same coverage. Report per-image paired ratios,then median/min/max and counts below1 across20 images. Zero-denominator comparisons would be unavailable rather than repaired;none occur here. No pooled pixel inference,p-values or causal attribution.

## Primary50% retention

Ratios below1 favor uncertainty. The unconstrained column reproduces EXP051's global uncertainty/random median. The stratified column is a different policy with fixed mass per bin,not a decomposition of the original improvement. Counts are descriptive within each20-image group.

|Window|Camera|Unconstrained median|Stratified min / median / max|Stratified below random|Common-support stratified median / below random|
|---|---|---:|---:|---:|---:|
|night-opening|RGB|0.080|0.782 / 0.937 / 1.044|18/20|0.942 / 15/20|
|night-opening|THR|0.128|0.187 / 0.217 / 0.260|20/20|0.565 / 20/20|
|day-opening|RGB|0.245|0.193 / 0.386 / 0.989|20/20|0.476 / 19/20|
|day-opening|THR|0.549|0.579 / 0.779 / 0.859|20/20|0.753 / 20/20|
|day-middle|RGB|0.513|0.210 / 0.645 / 0.922|20/20|0.627 / 20/20|
|day-middle|THR|0.459|0.544 / 0.750 / 0.978|20/20|0.764 / 20/20|
|day-late|RGB|0.535|0.320 / 0.778 / 1.070|18/20|0.788 / 17/20|
|day-late|THR|0.320|0.292 / 0.597 / 0.826|20/20|0.595 / 20/20|

Night RGB's median reduction is about92% under unconstrained selection but only6% when the retained depth/location mix is fixed. This makes the earlier large gain insufficient evidence of information beyond coarse scene composition. It does not identify a causal fraction explained by depth or location:the policy has changed. Most other groups retain a clearer benefit under the stratified policy,with156/160 primary image/modality cases below matched random and151/160 repeated common-support cases below it. Those are descriptive totals,not an effective sample size or statistical test.

## Global fixed baselines

Each cell is the median paired uncertainty-MSE / baseline-MSE ratio,followed by the number of20 images where uncertainty has lower risk. These are all prespecified complementary comparisons;the primary endpoint remains the stratified comparison above.

|Window|Camera|Farther depth|Nearer depth|Central pixels|Peripheral pixels|
|---|---|---:|---:|---:|---:|
|night-opening|RGB|0.485 (18/20)|0.045 (20/20)|0.232 (20/20)|0.048 (20/20)|
|night-opening|THR|0.141 (20/20)|0.116 (20/20)|0.320 (20/20)|0.080 (20/20)|
|day-opening|RGB|0.258 (15/20)|0.244 (20/20)|0.430 (13/20)|0.169 (20/20)|
|day-opening|THR|0.467 (20/20)|0.642 (20/20)|1.267 (1/20)|0.349 (20/20)|
|day-middle|RGB|0.380 (20/20)|0.625 (19/20)|0.574 (13/20)|0.406 (20/20)|
|day-middle|THR|0.415 (19/20)|0.632 (20/20)|1.116 (8/20)|0.290 (20/20)|
|day-late|RGB|0.539 (19/20)|0.392 (20/20)|0.470 (18/20)|0.351 (19/20)|
|day-late|THR|0.275 (20/20)|0.407 (19/20)|0.517 (20/20)|0.220 (20/20)|

Uncertainty is not uniformly the best selector. In day-opening thermal,central selection beats it in19/20 images (median uncertainty/central ratio1.267);in day-middle thermal,central selection beats it in12/20 (median1.116). Thus the prior all-cases advantage over random does not establish superiority to simple spatial selection. Depth baselines and the other groups are retained rather than used to tune a preferred selector. The full report contains all per-image MSEs,paired ratios,minima/maxima,oracle values and sensitivity curves.

![Fixed selection controls](../results/ms2-selection-controls/run-001/primary-selection-controls.png)

## Validation,replay and scope

All180,158 independent checks pass across960 cases and3840 controlled curve points. Partition-based cutoff masks reproduce every risk;separate algebra/thresholds reproduce radius,depth bands,image cells,source-ID/dimension joins,primary/common depth joins,54-bin counts and retained mass. Tied/constant/singleton/permutation/bin-edge fixtures pass. Oracle bounds and full-retention identities pass. All3840 EXP051 global uncertainty/oracle/random points reproduce;492 frozen inputs and1,949 historical result hashes remain unchanged. Validator source SHA256 is recorded in `validation.json`. No historical script was edited.

From `MAC-VO-main`,in `macvo:ms2-smoke` with AirStack at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_selection_controls --root /root/AirStack/ore-ms --output /path/to/fresh-run
python -m Scripts.Setup.validate_ms2_selection_controls --root /root/AirStack/ore-ms --run /path/to/fresh-run --output /path/to/fresh-validation.json
python -m Scripts.Setup.plot_ms2_selection_controls --run /path/to/fresh-run --previous /root/AirStack/ore-ms/results/ms2-selective-risk/run-001/report.json
```

Scoring requires original manifests/proxy reports and480 NPZs from EXP046/048/049 plus EXP051's scalar report;no raw imagery,models or GPU computation. Full validation additionally checks restored historical result payloads against EXP043/044-complete/046/047/048/049/050/051 recovery manifests. Fresh output paths are required. Runtime image SHA256 `8dc9fef8c6da18d5a05d7fc44f53b27c932ed67bfc42d2fccf60fd08cc0a29a8`. CPU numerical execution completed2026-10-25 20:25–20:26 America/New_York;startup driver diagnostics remain in logs without failed numerical checks. PNG/PDF figure inspected for legibility.

These are selective risks against supplied reference products,not physical covariance calibration or a cross-modal winner test. Camera supports/FOVs differ;the day windows share one recording;training-held-out status is unverified. Squared loss emphasizes high-error tails. Preserving coarse bins does not control fine geometry,visibility,texture or reference error. Exact exposure semantics do not block this provided-pair benchmarking;EXP050's scope correction remains in force.

The next useful existing-data check is sensitivity to the loss function and error tails,with the same selectors and coverage,including robust absolute residual risk. No baseline is adopted or tuned from this experiment. More frames or a weighting/fusion rule are not justified by these results alone;no new upload is needed for that bounded check,and CATS remains pending.
