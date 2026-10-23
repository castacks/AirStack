# EXP-039: Frozen MAC ranking rules on a different historical recording

Recorded:2026-10-23T00:03:13.863988-04:00;branch `ore_ms`;starting commit `6be7ef589` plus frozen CPU derivation source. **Both primary and full-grid nominal-angular covariance rules choose RGB on all57 paired transitions, and every diagnostic target also favors RGB. They equal the fixed RGB baseline here; there are no thermal-favored examples with which to replicate EXP-038's one helpful full-grid switch. No inference,training,retuning or new VO method.**

## Fixed recording-level replication

EXP-038 examined `20251027/0_salon_so_day`:its primary rule underperformed always RGB,and a predeclared full-grid-U sensitivity helped once. The next user-authorized step applies both rules unchanged to `20251027/1_salon_mac_cov_day`,using all three previously established20-frame early/original/late windows (EXP-004/006),not newly chosen outcomes. Both captures and these windows have been studied previously,share date/environment,and checkpoint training membership is unknown. This is a different-recording rule replication,not network-training-held-out or unseen-condition validation.

Recover historical RGB temporal covariance/endpoints from the2026-10-18 preserved artifact archive. Its whole109719262-byte SHA256 matches the manifest. Recover corresponding MSA-thermal angular/LK arrays from verified EXP-030 members. Stage only60 original RGB-left PNGs through authenticated ZIP ranges,with per-member CRC/size/SHA;the whole8.86GB dataset ZIP is not downloaded/hashed. Original row metadata match across RGB and MSA-thermal exactly. RGB source acquisition indices are retained,without input replacement or model reruns.

The historical RGB checkpoint hash is recorded in each report;raw covariance,model endpoints,K and grid arrays are copied unchanged. MSA-thermal retains EXP-023's separately regenerated experimental calibration baseline. No checkpoint,preprocessing,calibration,pose,weighting or optimizer change. All prior EXP-038 files in the new freeze registry remain unchanged. Acquisition timestamps do not overlap EXP-038's recording.

## Measurement and frozen rules

For RGB,derive the same nominal ray variance and exact MAC–LK angular disagreement as EXP-030 from saved raw temporal covariance,using its unchanged ray/Jacobian and independent LK helper functions. Use full2x2 uu/vv/uv covariance,not diagonal-only scaling or a covariance floor. Source images are decoded to native grayscale with inherited32px-border/16px-grid,LK21x21/maxLevel3/30iterations/.01epsilon/minEig1e-4,forward/backward/status/finite/inbounds gates and1px primary threshold. Thermal uses already saved MSA-derived angular arrays without another image/matching computation.

Apply the EXP-038 `choice`, `summarize` and `POPULATIONS` code unchanged. Primary compares median U and median D on each camera's own LK-valid points. Tight<=.5px and texture>=5 are fixed sensitivities. Full-grid U uses every fixed-grid uncertainty prediction,with the primary LK-valid D target unchanged. Lower U selects the camera;exact ties abstain. Raw temporal image covariance is the scope,not full3D metric covariance or optimized pose covariance.

Different fields of view/features/populations remain. D is independent-algorithm matching disagreement with unknown LK error and shared ambiguities,not physical ground truth. Nominal ray units improve dimensional comparability but do not certify camera calibration or correspondence accuracy. No fusion mechanism is built or evaluated.

All57 transitions pass the inherited stereo<=2ms/cross-modal<=30ms gates at both endpoints;cross offsets span23.683–24.228ms. Actual distinct timestamps are retained. These recorded-time checks do not verify physical hardware simultaneity.

## Results

| Window | Primary angular U | Full-grid angular U | Always RGB | Mean RGB D(mrad) | Mean thermal D(mrad) | Primary log-ratio rho |
|---|---:|---:|---:|---:|---:|---:|
| early | 19/19 | 19/19 | 19/19 | 0.659 | 1.626 | -0.221 |
| original | 19/19 | 19/19 | 19/19 | 0.912 | 1.897 | -0.005 |
| late | 19/19 | 19/19 | 19/19 | 0.860 | 1.372 | -0.416 |

The primary and full-grid angular rules each select RGB57/57 times and agree with the diagnostic winner57/57 times. Always RGB and the hindsight lower-D oracle have the same agreement and mean D. There is no adaptive selection benefit and no measurement of successful thermal detection,as the thermal-favored class is absent. Balanced accuracy is undefined,not silently filled with a favorable score.

Pooled primary mean selected/RGB disagreement is0.8103mrad versus1.6316mrad for always thermal. Primary and full-grid angular rules have zero switches and zero oracle regret on this recording. Their equality to RGB does not replicate a switching gain:EXP-038 had two thermal-favored transitions,while this recording has none. Do not pool this easy-class sample into EXP-038 as proof of generalization.

| Population | Angular rule agreement | Thermal choices | Pixel rule agreement | Pixel thermal choices(all harmful) | Pixel mean D change vs RGB |
|---|---:|---:|---:|---:|---:|
| primary | 57/57 | 0 | 54/57 | 3 | +5.96% |
| tight | 57/57 | 0 | 55/57 | 2 | +3.83% |
| texture | 57/57 | 0 | 55/57 | 2 | +3.81% |
| all_uncertainty | 57/57 | 0 | 48/57 | 9 | +19.12% |

Angular normalization avoids false thermal selections that the native-pixel policy makes here. The native primary rule's three thermal switches are all harmful by the same angular-D target,raising mean D5.96%;native full-grid makes nine harmful switches,raising19.12%. This supports retaining explicit units in this diagnostic;it does not establish physical covariance calibration.

Pooled primary log covariance-ratio versus log disagreement-ratio Spearman is−0.728 on this recording,versus+0.499 in EXP-038. All three present within-window signs are negative(−0.221/−0.005/−0.416). Thus high winner agreement can coexist with an opposing continuous ratio association when every target is on the same side of the decision boundary. No shared monotonic ratio mapping is established. Adjacent pairs/window samples are dependent,and no significance,causal or forecasting claim is made.

![Recording-level MAC ranking replication](../results/mac-recording-replication/run-001/recording-replication.png)

## Verification and replay

314,020 independent controls pass:180 current input/code hashes plus20 frozen EXP-038 files and the source helper/archive registries;selected image CRC/SHA;raw-prediction time0,K,grid,model endpoint/covariance identities;independent full-grid LK masks;399 scalar ray-angle/finite-difference covariance/patch-texture checks;scalar median and original thermal-quantile agreement;paired timing/choice/baseline/cost/rank calculations;disjoint acquisition timestamps. Three windows provide57 new RGB-derived point arrays and57 inherited thermal arrays. Outputs preserve all four populations and both unit choices,not a post-hoc winning subset.

An initial independent ray check used a direct solve that rounded its inputs differently from the historical helper. The helper's inverse-K uses the source float32 precision before float64 ray algebra. The validator now honors that source inverse rounding and independently computes scalar normalized rays;finite-difference covariance checks pass. Original failure log is retained. No source prediction,derived output,policy choice or scientific threshold changed.

CPU Python3.12.3,NumPy2.2.6,OpenCV4.10.0,single thread/seed0;plot Matplotlib3.10.1. Tracked derivation entry point `Scripts/Setup/td3_recording_replication.py --root <ore-ms> --run <fresh-prepared-run>` consumes frozen input plans/raw RGB predictions/source images/inherited thermal arrays. It refuses replacement of the derived RGB folder. Source staging,independent validation,plot/documentation,exact commands and runtime fingerprints are under `results/mac-recording-replication/staging/`;source plans,raw inputs,derived arrays,per-pair decisions,CSV summaries,validation and PNG/PDF are under `run-001/`. Restore a fresh prepared copy before rerunning;preserve existing outputs.

A restore-map.json and recovery instructions preserve the small historical archive manifest referenced outside the repository by the frozen input registry;restore that metadata to its recorded path before analysis/validation replay.

Recovery target Box `backups/2026-10-22-exp-039` contains all inputs needed for this CPU replay,unchanged helper/rule source,new derivation/report/notebook,outputs and incremental Git bundle. Historical recording/model provenance remains earlier notes;there is no need for weights,a GPU or a full dataset ZIP to replay this diagnostic. Private credentials,packages,range caches and whole upstream archives are excluded.

## Decision

The two nominal-angular rules avoid false switches on this recording but add nothing over the fixed RGB comparator. EXP-038's one helpful full-grid switch remains unreplicated as a benefit,not disproved or confirmed. Continuous ratio association changes sign across recordings. Keep all current defaults and do not fit a fusion rule or a cross-camera calibration from these samples.

The remaining evidence gap is relative-error variation while both modalities remain geometrically observable,with a stronger target than independent-tracker disagreement. More RGB-dominant daytime examples or interpolation-dominated night failures will not by themselves validate adaptive camera selection. This result concerns a fixed aggregate temporal-covariance rule,not the usefulness of every MAC covariance output.
