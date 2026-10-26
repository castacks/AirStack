# EXP-054: Fixed MS² rain recording replication

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `3bff03b20`.

This experiment replicates the frozen provided-reference uncertainty-selection diagnostics on a distinct recording labeled rain in the existing MS² access metadata. It does not isolate a causal weather effect or establish checkpoint training-held-out status. Source replies and CATS remain separate,not prerequisites for this scoped provided-pair benchmark.

## Selection and source acquisition

Before seeing rain images or model outcomes,freeze `_2021-08-06-16-19-00`,original source IDs000000–000019. Use all20 provided stereo-ID pairs for both cameras with no outcome/quality/timing-based replacement. Existing EXP040 access pointers work under exact HTTP206 range and bzip2-signature checks. Do not execute source all-sequence download scripts.

Acquire only the two rain transport archives,35,849,111,509bytes,in bounded64MiB ranges with exact status,Content-Range and byte-count validation. Local observed hashes below are not publisher-provided checksums:

|Archive|Bytes|Observed SHA256|
|---|---:|---|
|sync_data|24,906,100,204|`b6bd4e5ad17cfc9adcce5d6b52fccfe254609919c7ddc5c7601a9de43666cd54`|
|proj_depth|10,943,011,305|`ab2f7344ed29040f04babe3d2200cb882b76109afc5cc6a26f2b0c44c3fd4f90`|

Sequential archive layout requires scanning the full archives to reach the selected members. Safe extraction rejects absolute/traversal paths and links,enforces the sequence prefix,and bounds selected members to32MiB. Preserve full tar indices and bzip2 block maps;stage only80 camera images,120 depth maps,calibration/readmes/full timestamp arrays,20 selected navigation records and40 selected poses from the already cached odometry archive. Full transport archives remain local and are excluded from the replay backup. The bounded source payloads and all numerical outputs are backed up.

The full camera metadata has11,868 rows per stereo stream. Selected source tensors preserve native RGB uint8 at384×1224×3,raw thermal uint16 at256×640,and the explicit unchanged thermal preprocessing recipe. Calibration matches the earlier day/night decoded dictionaries exactly. The80 image hashes have zero overlap with the four earlier windows. All40 selected pose matrices pass the unchanged rigidity screen;they are not assigned as physical `gt_pose`.

|Recorded-reference/metadata quantity|RGB|Thermal|
|---|---:|---:|
|Selected left-camera duration (s)|1.933113|1.933658|
|Endpoint reference translation-column change (m),not accuracy|21.365186|21.354450|
|Full rows within inherited2ms stereo diagnostic|11809/11868|2249/11868|
|Full absolute stereo offset min / median / max (ms)|0.011097 / 0.197676 / 66.785039|0.000926 / 4.700084 / 72.439694|

The inherited joint recorded-time flag passes1/20 groups. It remains a diagnostic,not a physical acceptance criterion;all20 groups are scored. The copied preparation logger's legacy phrase “physical joint timing eligibility” refers only to this inherited metadata counter;the structured audit/report keeps physical cross-modal eligibility false. No exposure timing,synchronization or actual pose-accuracy conclusion follows. All20 navigation records have filter state2,dynamics mode2,status flags0;these fields do not certify RTK or an accuracy bound. EXP050's timing-scope correction remains in force.

## Frozen frontend and analysis

Use the unchanged source-ID-preserving stereo-only runner,original checkpoints,native resolution,FP32/seed0/thread1,and `td3-firestereo-v2-clahe-bilateral-v1`. No resizing,re-rectification,checkpoint adaptation,clock correction or training. New sequence-specific preparation/scoring/validation scripts preserve historical code and output hashes. Independent validation compares runtime,configuration,checkpoint load rules,recipe and decoded calibration with the earlier fixed day window.

Primary reference remains single-scan depth;merged32-scan and network-filtered products are separate sensitivities. Preserve every nonzero depth pixel whose projected right coordinate is in bounds. Normalize residuals by focal lengths and covariance components by focal lengths squared. Own support remains camera/product-specific;common support intersects all three products within the same camera,not matched physical RGB/thermal points.

Run both frozen squared residual and mean Euclidean residual losses with identical selectors,100/75/50/25% coverage,fractional cutoff ties,global far/near depth and central/peripheral baselines,and proportional retention inside the fixed six depth bands×3×3 grid. Every nonempty bin contributes,including singleton/constant bins;no sparse-support exclusions. For common support,use primary depth at identical coordinates for every product. Each loss has240 case/support views and960 curve points;the views and the two losses repeat the same40 image/modality cases. Tail diagnostics describe top1/5/10% residual mass without removing labels. Primary comparisons remain single-scan own support at50% retained coverage;no new favorable endpoint or threshold is selected.

## Rain results at50% retained coverage

Each ratio compares the selected risk with random expected risk on the same image/support. Below1 is better. Values are median per-image ratios across20 adjacent images;the two cameras have different FOV/support/reference populations,so these are not RGB-versus-thermal winner scores.

|Loss|Camera|Global ratio min / median / max|Global below random|Stratified ratio min / median / max|Stratified below random|
|---|---|---:|---:|---:|---:|
|Squared residual|RGB|0.095 / 0.166 / 0.956|20/20|0.516 / 0.623 / 1.029|19/20|
|Squared residual|THR|0.106 / 0.188 / 0.381|20/20|0.387 / 0.505 / 0.947|20/20|
|Mean L2 magnitude|RGB|0.461 / 0.552 / 0.778|20/20|0.770 / 0.827 / 0.925|20/20|
|Mean L2 magnitude|THR|0.417 / 0.530 / 0.643|20/20|0.667 / 0.739 / 0.878|20/20|

The mean-L2 advantage replicates under the composition constraint:40/40 primary image/modality cases improve,with median reductions17%RGB/26%thermal. Squared risk improves in39/40,with median reductions38%/49%. Global selection has larger median reductions83%/81% under MSE and45%/47% under mean L2. The loss and population controls therefore still matter on this distinct recording. Descriptive counts do not turn the adjacent pairs or repeated losses into independent samples.

![Frozen selection replication](../results/ms2-rain-replication/selection-comparison.png)

## Fixed baseline comparisons

Cells show median paired uncertainty/baseline ratio,followed by images below1. Both depth directions and both image-location directions remain separately visible;no post-hoc best selector is adopted.

|Loss|Camera|Farther depth|Nearer depth|Central pixels|Peripheral pixels|
|---|---|---:|---:|---:|---:|
|Squared residual|RGB|0.303 (18/20)|0.118 (20/20)|0.255 (18/20)|0.126 (20/20)|
|Squared residual|THR|0.249 (20/20)|0.191 (20/20)|0.429 (19/20)|0.124 (20/20)|
|Mean L2 magnitude|RGB|0.658 (19/20)|0.480 (20/20)|0.634 (19/20)|0.483 (20/20)|
|Mean L2 magnitude|THR|0.575 (20/20)|0.502 (20/20)|0.779 (20/20)|0.408 (20/20)|

Rain uncertainty is stronger than each fixed global baseline at the group median under both losses,but it is not uniformly superior per image. For example,central selection beats RGB uncertainty in2/20 MSE cases and1/20 mean-L2 cases;far-depth selection also wins some RGB cases. Earlier day thermal central-baseline counterexamples remain unchanged. One successful recording does not erase those limitations or identify an optimal fusion rule.

## Reference/support sensitivities

Raw rho is the within-image covariance-trace/L2 residual Spearman median from the unchanged depth-proxy scorer. The stratified selection columns use50% retention at fixed depth/location composition. These are different statistics;neither is physical covariance calibration. Filtered depth remains network-dependent and is not an independent surveyed reference.

|Camera / product / support|Total supported pixels,not independent n|Raw rho median|Stratified MSE median (below random)|Stratified mean-L2 median (below random)|
|---|---:|---:|---:|---:|
|RGB / depth / own|78,363|0.322|0.623 (19/20)|0.827 (20/20)|
|RGB / depth / common|25,590|0.212|0.673 (20/20)|0.878 (20/20)|
|RGB / depth_multi / own|902,466|0.322|0.772 (20/20)|0.870 (20/20)|
|RGB / depth_multi / common|25,590|0.219|0.686 (20/20)|0.897 (20/20)|
|RGB / depth_filtered / own|770,790|0.258|0.653 (20/20)|0.867 (20/20)|
|RGB / depth_filtered / common|25,590|0.244|0.695 (20/20)|0.911 (20/20)|
|THR / depth / own|121,474|0.630|0.505 (20/20)|0.739 (20/20)|
|THR / depth / common|49,724|0.520|0.552 (20/20)|0.784 (20/20)|
|THR / depth_multi / own|601,053|0.550|0.682 (20/20)|0.800 (20/20)|
|THR / depth_multi / common|49,724|0.512|0.510 (20/20)|0.781 (20/20)|
|THR / depth_filtered / own|525,568|0.583|0.469 (20/20)|0.752 (20/20)|
|THR / depth_filtered / common|49,724|0.595|0.494 (20/20)|0.781 (20/20)|

Primary own-support raw rho medians are0.322RGB/0.630thermal;common-support medians0.212/0.520. Primary common-support stratified mean-L2 ratios are0.878RGB/0.784thermal,below random in20/20 each;MSE ratios0.673/0.552 also20/20. Common support holds reference-product coordinates fixed within a camera,not across cameras. All other coverages,per-image minima/maxima,oracle values,bin counts and sparse-bin memberships remain in the reports;no support/product is dropped to strengthen the result.

## Tail concentration

The same top-residual1/5/10% expected-mass diagnostic is computed under both loss reports and agrees exactly. The table is the median per-image contribution to total MSE / total L2 magnitude,not a pooled fraction. These diagnostics do not mark bad labels or justify trimming.

|Camera|Worst1%: MSE / L2|Worst5%: MSE / L2|Worst10%: MSE / L2|
|---|---:|---:|---:|
|RGB|26.9% / 9.6%|69.3% / 33.1%|87.4% / 50.1%|
|THR|14.9% / 5.6%|48.4% / 22.3%|68.0% / 37.0%|

## Validation and replay

All40 original frontend fields pass at native resolution;all80 producer-input tensors are exact,and both first-pair flow/covariance repeats are bitwise identical. Runtime,effective configuration,checkpoint hashes/load results,thermal recipe and calibration match the earlier day stereo-only run. The independent full source/frontend/proxy audit passes18,714 checks across120 depth cases and2,999,714 selected points,including1,920 scalar residual checks.

Independent squared- and mean-L2 selection validators pass50,203 checks each across240 case/support views and960 curve points per loss. They separately recompute support joins,primary depth at common coordinates,bin counts,retained mass,cutoff masks,oracle bounds,baseline risks,tail sums and aggregate scalar quantiles. This is119,120 checks across the three final audits;the count is implementation validation,not a statistical sample size. The earlier source-only audit separately passed7,887 checks. Each final audit also verifies1,977 historical result hashes through EXP053. The proxy scorer freezes321 inputs;each loss analysis freezes127. No historical code,data,defaults,thresholds or results change.

Primary result files are under `results/ms2-rain-replication`:the bounded `pilot`,source/protocol manifests,`frontend-run-001`,`proxy-run-001`,`mse-run-001`,`mean-l2-run-001`,validation JSON and comparison PNG/PDF. Acquisition/extraction/pipeline source is preserved in the replay archive;full35.85GB transport archives are excluded.

From `MAC-VO-main`,inside the existing images with AirStack mounted at `/root/AirStack`,use the following commands after restoring the bounded pilot and original checkpoints. Source preparation outputs must be fresh;for replay of existing manifests skip preparation.

```bash
python -m Scripts.Setup.ms2_rain_replication prepare --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-rain-replay
python -m Scripts.Setup.ms2_manifest_stereo --root /root/AirStack/ore-ms --manifest /root/AirStack/ore-ms/results/ms2-rain-replication/source-manifest.json --rgb-checkpoint /root/AirStack/ore-ms/models/macslam/MACVO_FrontendCov.pth --thermal-checkpoint /root/AirStack/ore-ms/models/macslam/MACT_Offroad.pth --output /path/to/fresh-frontend
python -m Scripts.Setup.ms2_rain_replication score --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-rain-replication --source /path/to/fresh-frontend --output /path/to/fresh-proxy
python -m Scripts.Setup.validate_ms2_rain_replication --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-rain-replication --frontend /root/AirStack/ore-ms/results/ms2-rain-replication/frontend-run-001 --proxy /root/AirStack/ore-ms/results/ms2-rain-replication/proxy-run-001 --output /path/to/fresh-source-array-validation.json
python -m Scripts.Setup.ms2_rain_selection --root /root/AirStack/ore-ms --loss mse --output /path/to/fresh-mse
python -m Scripts.Setup.ms2_rain_selection --root /root/AirStack/ore-ms --loss mean_l2 --output /path/to/fresh-mean-l2
python -m Scripts.Setup.validate_ms2_rain_selection --root /root/AirStack/ore-ms --run /path/to/fresh-mse --output /path/to/fresh-mse-validation.json
python -m Scripts.Setup.validate_ms2_rain_selection --root /root/AirStack/ore-ms --run /path/to/fresh-mean-l2 --output /path/to/fresh-mean-l2-validation.json
python -m Scripts.Setup.plot_ms2_rain_replication --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-rain-replication
```

The selection scripts intentionally read the restored rain proxy at its fixed path;fresh frontend/proxy replay does not redirect that input automatically. Preparation needs the selected files and extraction manifests staged in a fresh base inside the ore-ms root. Plot outputs must also be absent. Source validation additionally needs the original day/previous-window calibration/frontend metadata,the archived thermal producer,and original checkpoint files for hash checks;full historical validation requires EXP043/044-complete/046/047/048/049/050/051/052/053 recovery manifests and restored result payloads. Running saved-array scoring/validation requires CPU libraries,not a new model execution. New frontend inference needs the original models and GPU.

Runtime image `macvo:ms2-smoke` SHA256 `8dc9fef8c6da18d5a05d7fc44f53b27c932ed67bfc42d2fccf60fd08cc0a29a8`;extraction uses the existing `macvo:ms2-pilot` image with indexed_bzip2. New inference/scoring/validation ran2026-10-25 23:55–23:57 America/New_York. Startup driver diagnostics remain in logs;all numerical checks pass. Comparison figure inspected for legibility. Source/model hashes tie this run to the starting commit plus the new sequence-specific scripts,which are archived verbatim.

## Decision

Useful uncertainty-based selection replicates against supplied references on this fixed rain recording under both losses and coarse composition controls. Preserve the counterexamples,reference/support/loss sensitivities and simple baselines rather than treating replication as calibrated physical uncertainty. RGB and thermal supports differ;references share the MS² production pipeline;training-held-out status remains unknown. No causal rain effect,cross-modal winner,VO validation,training or fusion policy follows.

No additional upload is needed for this bounded result. A complementary independent reference dataset such as the pending CATS access would strengthen geometric validation more than repeatedly tuning these selectors. If more MS² frames are needed later,the full rain/day archives and indices are already cached;choose future intervals before outcomes and keep their same-recording dependence explicit.
