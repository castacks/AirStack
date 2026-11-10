# RGB–thermal combination accuracy: native backend remains the stronger baseline

2026-11-09, America/New_York. Prediction/scorer freeze `1a08fb7ac`, completed-output accuracy amendment `3b0cba3b9`, independent audit `29a78a9c2`. Runtime is excluded by user instruction. All recordings are already development-exposed; errors use the existing provided-reference/nominal-frame convention.

**Result: the new Gaussian 3D replay plus full-covariance combination does not improve accuracy over native RGB. Do not adopt this backend/fusion variant.** This result does not establish that native covariance-aware RGB–thermal fusion is ineffective: the replay changes the underlying poses, and those changes regress accuracy before fusion.

## Controlled experiment

Use original groups 0–599 of day `_2021-08-13-16-31-10`, night `_2021-08-13-22-16-02` and rain `_2021-08-06-16-19-00`:599 adjacent intervals each. Day/night use cached native outputs. Rain's continuous prefix was recovered from six verified existing Box shards, with 2,400 member-hashed/PNG-validated images and original timestamps/calibration. Both original MAC/MACT experts completed 600 frames each on the exact recovered image and original weights/source. No training, checkpoint change, normalization change or outcome-based native retry occurred.

Native controls already use covariance-weighted disparity/Huber optimization. They are not unweighted baselines. The new replay is a separate two-endpoint Gaussian3D likelihood backend. For each family, native/replay single cameras, equal Lie midpoint and the existing angular selector are retained. Replay scalar/full methods use the same replay means and local covariance-intersection construction; scalar covariance preserves separate translation/rotation block traces while removing directional/cross terms.

All means/covariances use the official source-backed extrinsics into provided RGB optical coordinates and the existing stationary-twist duration mapping. Physical frame/clock assumptions remain nominal. Tests verify lever-arm and duration covariance effects. In this completed-output accuracy task, valid exact terminal first writes are included and labelled; this was declared before reference scoring and does not revise earlier online-availability experiments.

Missing covariance triggers an explicitly frozen equal-mean fallback; capped replay means remain labelled rather than acquiring invented covariance. A sole available mean remains available. All source intervals and all failures are retained.

All three recordings' ten-method predictions locked before extraction/read of 1,800 reference endpoints. Source-ID reference direction, rigidity qualification, small serialization projection and rotation/translation losses match the existing reference protocol. No scale, clock, reference alignment or method parameter was fitted.

## Accuracy against native RGB

Each comparison uses exactly the same source intervals for both methods. Entries are **mean relative rotation error (degrees) / mean relative translation error (metres)**, not trajectory ATE or RMSE.

| Recording | Matched intervals | Native RGB | Replay + full covariance |
|---|---:|---:|---:|
| Day |599|0.017523 /0.046604|0.038433 /0.057849|
| Night |598|0.030350 /0.077280|0.360826 /0.351439|
| Rain |599|0.041860 /0.231334|0.092440 /0.320584|

Native RGB is better in both error components on every recording. Predictions exist for 599/598/599 intervals in native RGB and full combination respectively. Native thermal supplies 590/551/594. All 599 reference intervals per recording pass the frozen qualification. Unavailable predictions are never scored as zero error.

## Isolating weighting from the backend change

For comparison across all ten methods, the identical all-method support contains 590 day, 551 night and 594 rain intervals. Equal-record means on that support are:

| Method | Mean rotation error (deg) | Mean translation error (m) |
|---|---:|---:|
| Native RGB |0.029798|0.117703|
| Native thermal |0.130283|0.372243|
| Native equal fusion |0.071524|0.221087|
| Native angular selector |0.029803|0.117590|
| Replay RGB |0.055298|0.151563|
| Replay thermal |1.584228|1.555067|
| Replay equal fusion |0.799645|0.822881|
| Replay angular selector |0.055261|0.151502|
| Replay scalar covariance |0.172867|0.248988|
| Replay full covariance |0.172462|0.248681|

Full covariance reduces rotation error 78.43% relative to replay equal fusion, but this is a weak baseline after the backend regression. It is about 5.79 times native RGB's rotation error. The native RGB baseline is stronger than both replay single-camera estimates, so the improvement over replay equal fusion is not a system accuracy improvement. The small scalar/full difference is not evidence of a robust directional-covariance advantage.

The declared development advancement criteria fail: no recording has a full-candidate rotation improvement over the strongest fixed single-camera control. Translation increases versus native RGB on all-method matched support are 24.60%day,381.94%night and 37.35%rain. Full/equal replay prediction coverage matches, including fallbacks, so the result is not explained away by omitting unavailable candidate predictions.

## Failure attribution, not a rescued result

A post-score descriptive breakdown retains the complete population and changes no predictions. In night's 551 all-method matched intervals,54 equal-missing-covariance fallbacks contribute 88.96% of full-candidate rotation-error sum. Their mean rotation error is3.51685 degrees, versus 0.047424 degrees for 497full-CI intervals. On those497 intervals replay RGB is 0.047425 degrees and native RGB0.030471 degrees. Even this restricted diagnostic does not beat native RGB; it is not a new primary result.

This identifies two distinct issues: the replay means regress, and equal mixing with a capped thermal replay can be damaging. It does not establish a physical cause, covariance calibration or general camera preference. No additional budget, fallback rule or solver parameter was chosen after these errors were read.

## Verification and next work

31 numerical/scoring tests pass. Independent reconstruction imports neither producer nor scorer and verifies 6,940combination cases and 35,676error components, plus common-support means/RMSEs. All three audits pass. Prediction/source hashes remain unchanged. The rain reference packet used a documented archive alias; an initial lookup failure was corrected before scoring, with no native/prediction rerun.

Keep the native backend as the accuracy baseline. The next meaningful experiment should hold native pose estimates fixed and test weighting or uncertainty calibration independently of a new optimizer. Any missing-covariance fallback must be tested explicitly; another experiment can compare using the certified expert with equal mixing, with its rule frozen before scoring. MACThermal feature/normalization improvements remain a separate hypothesis if native thermal errors, rather than weight selection, are targeted. No broader RGB–thermal or learned-uncertainty route is closed by this variant's result.

Artifacts: `results/pose-covariance/accuracy-001/` contains native rain captures, source/recovery manifests, old processing-only predictions, final completed-output predictions, global lock, references/locks, complete scores, descriptive attribution and independent audits. Large original source/runtime/model prerequisites remain in their verified Box backups. New native output and scientific result payloads are included in the incremental experiment backup.
