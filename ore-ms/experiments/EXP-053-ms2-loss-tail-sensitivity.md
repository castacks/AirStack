# EXP-053: Loss sensitivity and concentration of reference error

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `501e03dd6`.

Uncertainty selection beat random retention under squared loss,but EXP052 showed sensitivity to depth/location composition and stronger simple spatial baselines in some windows. This follow-up asks whether selection still helps under mean Euclidean residual magnitude,and measures how much total error comes from the largest residuals. It is exploratory on previously inspected recordings,with loss/tail definitions fixed before inspecting these results.

## Unchanged selection,less tail-sensitive loss

Keep all four windows,20 original IDs each,both cameras,three depth products,own/common supports and100/75/50/25% coverage:960 cases and3840 curve points. For each pixel the new loss is `sqrt(du_normalized²+dv_normalized²)`,averaged over the retained population. This is mean L2 magnitude,not squared loss,coordinate-wise L1 or RMSE. It gives large residuals less influence than squared loss but is still unbounded;it is not a bounded-influence robust estimator. No clipping,trimming,relabeling or support exclusions.

Selection still ranks normalized covariance trace,farther/nearer reference depth,central/peripheral radius,or ideal reference error. Directions,geometry,fractional cutoff ties,54 depth/location bins and same-coverage budgets remain exactly as EXP052. All nonempty bins contribute,including singleton/constant bins. Own support uses its depth product;common support uses primary single-scan depth at identical coordinates for every variant. The oracle's ordering is unchanged by taking a square root;it is a diagnostic lower bound,not a deployable policy. Depth baselines and stratification use reference information and remain diagnostic.

Primary endpoint is single-scan own-support stratified uncertainty/random ratio at50% retained mass. Global uncertainty/random and uncertainty/each fixed baseline are complementary comparisons;other coverages,references and support are sensitivities. Report paired per-image ratios,then median/min/max and descriptive counts over20 images. Repeated variants/supports and adjacent images are not independent observations. No pixel pooling,p-values,tuned selector or causal decomposition.

## Primary50% retention

Ratios below1 favor uncertainty. MSE columns reproduce the unchanged EXP052 policy scores;mean-L2 columns change the loss,not the selected mass or population. The stratified policy retains half independently within every depth/location bin and compares against matched random expectation.

|Window|Camera|Global MSE ratio|Global mean-L2 ratio|Stratified MSE ratio|Stratified mean-L2 min / median / max|Mean-L2 stratified below random|
|---|---|---:|---:|---:|---:|---:|
|night-opening|RGB|0.080|0.412|0.937|0.796 / 0.880 / 0.969|20/20|
|night-opening|THR|0.128|0.526|0.217|0.578 / 0.628 / 0.689|20/20|
|day-opening|RGB|0.245|0.715|0.386|0.664 / 0.787 / 0.979|20/20|
|day-opening|THR|0.549|0.778|0.779|0.824 / 0.910 / 0.953|20/20|
|day-middle|RGB|0.513|0.788|0.645|0.669 / 0.861 / 0.921|20/20|
|day-middle|THR|0.459|0.737|0.750|0.836 / 0.889 / 0.991|20/20|
|day-late|RGB|0.535|0.680|0.778|0.681 / 0.880 / 0.951|20/20|
|day-late|THR|0.320|0.636|0.597|0.666 / 0.810 / 0.863|20/20|

The benefit persists:160/160 primary image/modality cases improve against matched random under stratified mean L2,compared with156/160 under stratified MSE. Group median mean-L2 reductions range9%–37%. Global mean-L2 reductions range21%–59%,generally smaller than the earlier MSE reductions. Night RGB is a useful exception within bins:mean-L2 reduction12% exceeds its MSE reduction6%,even though its unconstrained reduction falls from92% under MSE to59% under mean L2. Loss sensitivity is not uniform,and changing the loss is not a causal estimate of how much benefit was due to outliers.

Single-scan common-support stratified mean-L2 sensitivity improves in157/160 repeated image/modality cases;camera/window medians are0.943/0.803 night RGB/thermal,0.866/0.896 opening day,0.823/0.899 middle,and0.853/0.823 late. Different supports and repeated labels do not create independent evidence or a cross-modal winner.

## Fixed global baselines under mean L2

Each cell is the median paired uncertainty/baseline risk ratio,followed by images below1 out of20. All four prespecified alternatives remain visible;no best direction is selected after seeing errors.

|Window|Camera|Farther depth|Nearer depth|Central pixels|Peripheral pixels|
|---|---|---:|---:|---:|---:|
|night-opening|RGB|0.789 (17/20)|0.275 (20/20)|0.624 (20/20)|0.306 (20/20)|
|night-opening|THR|0.528 (20/20)|0.530 (20/20)|0.823 (20/20)|0.389 (20/20)|
|day-opening|RGB|0.785 (14/20)|0.635 (20/20)|0.938 (12/20)|0.578 (20/20)|
|day-opening|THR|0.711 (20/20)|0.835 (20/20)|1.135 (0/20)|0.590 (20/20)|
|day-middle|RGB|0.775 (20/20)|0.871 (19/20)|0.927 (12/20)|0.687 (20/20)|
|day-middle|THR|0.718 (19/20)|0.770 (20/20)|1.043 (8/20)|0.551 (20/20)|
|day-late|RGB|0.800 (20/20)|0.620 (20/20)|0.772 (19/20)|0.578 (20/20)|
|day-late|THR|0.597 (20/20)|0.697 (18/20)|0.836 (19/20)|0.504 (20/20)|

The spatial-baseline limitation remains:central selection beats thermal uncertainty in20/20 opening-day images (median uncertainty/central1.135) and12/20 middle-day images (1.043). Improving over random does not establish superiority to fixed spatial selection or numerical covariance calibration. All ratio denominators are positive here;zero denominators would remain explicitly unavailable.

## Error-tail concentration

For each case,rank by residual solely to describe the largest1%,5%,10% of exact expected pixel mass. Equal-valued cutoff ties have equal fractional inclusion. Measure this mass's contribution to total squared error and total L2 magnitude;no pixels are removed from any score. The following are medians of20 per-image fractions,not fractions of a pooled population. Minima/maxima and all sensitivities remain in the report.

|Window|Camera|Worst1%: MSE / L2 contribution|Worst5%: MSE / L2 contribution|Worst10%: MSE / L2 contribution|
|---|---|---:|---:|---:|
|night-opening|RGB|27.0% / 11.8%|83.3% / 45.4%|94.7% / 61.7%|
|night-opening|THR|32.5% / 9.6%|71.3% / 29.7%|81.3% / 41.5%|
|day-opening|RGB|38.5% / 10.8%|75.3% / 28.5%|81.0% / 37.8%|
|day-opening|THR|16.5% / 5.0%|36.1% / 15.7%|50.2% / 26.3%|
|day-middle|RGB|21.5% / 6.6%|54.0% / 22.1%|67.1% / 32.5%|
|day-middle|THR|12.4% / 4.6%|37.4% / 17.2%|55.7% / 29.1%|
|day-late|RGB|24.1% / 8.3%|70.7% / 33.8%|88.2% / 49.0%|
|day-late|THR|20.4% / 6.4%|52.7% / 22.9%|68.1% / 35.1%|

The worst1% contributes12%–39% of squared error across group medians,versus5%–12% of L2 magnitude. The largest primary per-image1% contribution is67.0% of squared error in middle-day RGB source001942,but17.8% of L2 magnitude. This establishes error concentration,not that those pixels are mislabeled or physically invalid. Tail diagnostics do not identify reference error versus model error,and do not authorize trimming them away. The mean-L2 selection benefit shows that the observed advantage is not confined to the squared-loss definition;it does not isolate a tail-free causal effect.

![Loss and tail sensitivity](../results/ms2-loss-tail-sensitivity/run-001/primary-loss-tail-sensitivity.png)

## Validation and replay

All229,331 independent checks pass:960 cases,3840 mean-L2 points,2880 tail records,and all3840 previous EXP052 MSE curve points reproduced against the same arrays. Independent partition/mask sums check tail mass,contributions,bounds and monotonicity;known analytic tail examples pass. Independent source-ID/dimension/common-depth joins,bin counts,tied/constant/singleton/permutation fixtures,oracle bounds,full-retention identities and scalar aggregate quantiles pass. All493 frozen inputs and1,963 historical result hashes are unchanged. Validator source SHA256 is recorded in `validation.json`. No historical code or result is modified.

From `MAC-VO-main`,in the existing `macvo:ms2-smoke` image with AirStack mounted at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_loss_tail_sensitivity --root /root/AirStack/ore-ms --output /path/to/fresh-run
python -m Scripts.Setup.validate_ms2_loss_tail_sensitivity --root /root/AirStack/ore-ms --run /path/to/fresh-run --output /path/to/fresh-validation.json
python -m Scripts.Setup.plot_ms2_loss_tail_sensitivity --run /path/to/fresh-run --previous /root/AirStack/ore-ms/results/ms2-selection-controls/run-001/report.json
```

Scoring uses original manifests/proxy reports,480 NPZs from EXP046/048/049 and EXP052's scalar report. No raw imagery,model weights,GPU computation or new data is required. Full validation also needs recovery manifests EXP043/044-complete/046/047/048/049/050/051/052 and restored historical result payloads. Fresh output paths are mandatory. Runtime image SHA256 `8dc9fef8c6da18d5a05d7fc44f53b27c932ed67bfc42d2fccf60fd08cc0a29a8`;CPU scoring completed2026-10-25 23:37 America/New_York. Startup driver diagnostics are retained;all numerical checks pass. PNG/PDF plot inspected for legibility.

## Research decision

Retain both mean-L2 and squared-loss diagnostics rather than reporting the most favorable one. Uncertainty selection has useful provided-reference association beyond the squared-loss definition,but depth/location composition and strong simple baselines remain material. No training,fusion,calibration claim or selection default is adopted. Camera supports differ,three day windows share a recording,training-held-out status is unverified,and reference labels are not independent physical ground truth. Exact exposure semantics do not block these scoped benchmark checks;EXP050 remains in force.

The next priority is a fixed independent recording/condition replication under these frozen loss and baseline protocols,rather than more selector tuning on the same windows. Inspect the existing MS² rain access/coverage before requesting an upload or downloading more data. Complementary CATS validation remains pending;no assumption of access or author replies.
