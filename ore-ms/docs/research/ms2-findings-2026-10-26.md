# MS² uncertainty findings and research decision

Consolidated 2026-10-26, America/New_York, from EXP-046–054 at code commit `4b238c517`. This is a synthesis of completed experiments, not a new model run or an additional validation dataset.

Low predicted covariance trace selects lower stereo reference error within each camera on the tested MS² windows. The benefit survives coarse depth/location composition controls and a change from squared residual to mean Euclidean residual magnitude. This supports uncertainty as a useful within-camera ranking signal against the supplied references. It does not yet answer the project's original question of whether RGB and thermal uncertainty values predict relative reliability across modalities.

**Subsequent cross-modal pilot:** [EXP-056](../../experiments/EXP-056-ms2-cross-modal-support.md) evaluates25,754 mutual/frontmost candidate pairs from the existing rain window,25,554 with both predictions valid. Learned propagated uncertainty beats equal-angular-noise geometry in all20 frames, but a per-image/per-camera median-variance baseline largely matches it: full pointwise learned/control median risk ratios0.988 for squared3D error and1.002 for mean3D distance (11/20 and9/20 improve). Thus spatially varying learned uncertainty has no consistent demonstrated advantage over camera-wide scale in this pilot. Sparse projected references do not certify exact shared physical points. The next empirical priority is frozen replication on a distinct existing recording, retaining the camera-scale baseline and both reference anchors; fusion is not justified yet.

## What was evaluated

Five windows from three distinct recordings: night opening, day opening/middle/late, and rain opening. Each window has 20 provided stereo-ID pairs for each of RGB and thermal: 100 paired groups, or 200 image/modality cases. The day windows share one recording; neighboring frames are dependent. Depth products, own/common supports, losses and coverages repeat those same cases rather than increasing the independent sample size. Rain is a different recording labeled rain, not an isolated weather intervention or a verified training-held-out recording.

The fixed primary endpoint retains 50% of supported pixel mass against single-scan projected depth, on each camera's own support. Uncertainty is covariance trace normalized by focal lengths squared; residual components are likewise normalized by their focal lengths. Lower uncertainty is selected, with equal fractional inclusion of cutoff ties. The controlled policy retains half independently within each of six reference-depth bands crossed with a 3×3 image grid. Its matched-random expected risk is the full-support mean. Reference depth is used for the control, so this policy is a diagnostic rather than a deployment-ready selector.

## How much benefit survives the controls?

The table uses **mean L2 residual magnitude**, averaged over selected pixels. It is neither RMSE nor coordinate-wise L1. Every number is a median over 20 per-image ratios; no pixels or windows are pooled. Below 1 favors uncertainty. The reduction is 1 minus the controlled median ratio; it is not a causal fraction explained by uncertainty.

The last column compares global uncertainty selection with global central-pixel selection at the same coverage. Above 1 means the central baseline is better. This is a separate comparison from the policy that fixes the depth/location mix.

| Window | Camera | Global / random | Controlled / random | Controlled reduction | Controlled below random | Global / central |
|---|---|---:|---:|---:|---:|---:|
| night-opening | RGB | 0.412 | 0.880 | 12.0% | 20/20 | 0.624 |
| night-opening | Thermal | 0.526 | 0.628 | 37.2% | 20/20 | 0.823 |
| day-opening | RGB | 0.715 | 0.787 | 21.3% | 20/20 | 0.938 |
| day-opening | Thermal | 0.778 | 0.910 | 9.0% | 20/20 | 1.135 |
| day-middle | RGB | 0.788 | 0.861 | 13.9% | 20/20 | 0.927 |
| day-middle | Thermal | 0.737 | 0.889 | 11.1% | 20/20 | 1.043 |
| day-late | RGB | 0.680 | 0.880 | 12.0% | 20/20 | 0.772 |
| day-late | Thermal | 0.636 | 0.810 | 19.0% | 20/20 | 0.836 |
| rain-opening | RGB | 0.552 | 0.827 | 17.3% | 20/20 | 0.634 |
| rain-opening | Thermal | 0.530 | 0.739 | 26.1% | 20/20 | 0.779 |

The controlled mean-L2 advantage appears in all 20 images in each of the ten groups. Median reductions range from about 9% to 37%. The rain replication contributes about 17% for RGB and 26% for thermal. These are descriptive within-window results, not 200 independent trials or evidence that thermal outperforms RGB. See [EXP-053](../../experiments/EXP-053-ms2-loss-tail-sensitivity.md) and [EXP-054](../../experiments/EXP-054-ms2-rain-replication.md).

Three findings prevent overstating the result:

- **Scene composition matters.** Night RGB's unconstrained squared-risk ratio was 0.080, but its depth/location-controlled ratio was 0.937. These are different selection policies, so the change is not a causal decomposition. The controlled mean-L2 ratio is 0.880. The large initial benefit cannot alone demonstrate scene-independent reliability. See [EXP-051](../../experiments/EXP-051-ms2-selective-risk.md) and [EXP-052](../../experiments/EXP-052-ms2-selection-controls.md).
- **A simple baseline sometimes wins.** Central-pixel selection beats thermal uncertainty in all 20 opening-day images under mean L2 and in 12/20 middle-day images. Their median uncertainty/central ratios are 1.135 and 1.043. Rain does not erase those counterexamples; the same baseline also wins individual rain RGB images.
- **Loss and reference support matter.** The controlled squared-risk advantage occurs in 156/160 earlier cases and 39/40 rain cases; mean L2 improves in all cases on primary own support. The corresponding common-support mean-L2 sensitivity improves in 157/160 earlier cases and 40/40 rain cases. Common support intersects reference products within a camera, not physical points shared by RGB and thermal. The worst 1% of residuals contributes about 12–39% of squared loss across earlier group medians; the mean-L2 benefit shows the result is not confined to squared loss. No tail pixels were removed or designated erroneous labels.

## What the evidence establishes

| Question | Current answer | Evidence / boundary |
|---|---|---|
| Does within-camera uncertainty rank useful measurements? | Yes, descriptively on these supplied-reference windows. | Lower selective risk than matched random, including controlled mean-L2 results. |
| Is the benefit entirely an artifact of squared loss? | No. | It persists with unchanged selection under mean L2. This loss is still unbounded. |
| Has coarse depth/location composition been addressed? | Yes, with a constrained-policy control. | Remaining benefit does not eliminate fine geometry, texture, visibility or reference-error confounding. |
| Is uncertainty always better than simple selection? | No. | Central-pixel thermal counterexamples remain. |
| Does the frozen protocol replicate beyond one recording? | Yes, on the selected night/day/rain recordings. | Small dependent windows; no causal weather or verified training-held-out claim. |
| Are the numerical covariance values calibrated? | Not established. | Ranking/selective risk is not covariance calibration. |
| Does lower RGB uncertainty mean RGB is more reliable than thermal? | Not established by EXP046–054; conditional pilot now in EXP056. | Common metric candidate-support results depend on camera scale and do not certify physical same-point identity. |
| Will selection or fusion improve VO? | Not established. | Stereo-reference diagnostics are not an end-to-end motion experiment. |

The earlier rank-association studies [EXP-046](../../experiments/EXP-046-ms2-stereo-depth-proxy.md), [EXP-047](../../experiments/EXP-047-ms2-depth-spatial-controls.md), [EXP-048](../../experiments/EXP-048-ms2-day-replication.md) and [EXP-049](../../experiments/EXP-049-ms2-day-later-windows.md) motivate the selection analysis; their correlations are not interchangeable with selective-risk reductions.

## What is necessary next

No additional experiment is necessary to report the bounded within-camera finding above. Preserve both losses, the scene controls, reference sensitivities and central-baseline counterexamples in the report. Do not keep adding nearby frames or tuning selectors merely to strengthen this same claim.

To advance the **original cross-modal research question**, the next necessary design step is to define a defensible comparison target: what RGB and thermal measurements refer to, which scene/support is shared, and what error quantity makes one modality more reliable. Audit whether the existing MS² calibration and selected reference data can support that comparison before implementing it. A per-camera ratio-to-random table cannot substitute for this definition. If matched physical support cannot be justified, keep the conclusion at within-camera ranking rather than manufacturing a modality winner from separate camera populations. Any future cross-modal experiment should freeze its target, support, baseline and recording split before examining its outcome.

The subsequent [EXP-055 feasibility audit](../../experiments/EXP-055-ms2-cross-modal-feasibility.md) finds promising supplied-calibration overlap on the existing rain pairs: a median85.4% of supported RGB depth pixels project within1 target pixel of a thermal depth label. Nearest-label agreement is not verified common-point identity. The next bounded experiment is a frozen correspondence/visibility pilot, followed by a common metric point-error comparison if that support is credible. No new images or model inference were needed for this feasibility audit.

Two parallel items strengthen specific claims but do not block the current benchmark report:

- Yifei's checkpoint training/split information is needed before calling recordings training-held-out or unseen by the model. Until a reply arrives, state that membership is unknown; no further calibration questions to Geethika are needed now.
- CATS access is still pending per the user. If obtained, inspect a bounded sample for independent geometric-reference evaluation. Do not assume access or repeat requests as a prerequisite for current MS² work.

Exact exposure semantics, TD3 target-capture provenance and GNSS axis/RTK details are not prerequisites for the scoped MS² provided-pair result. Timing/alignment questions become necessary when the chosen physical or motion-sensitive comparison requires them; follow [EXP-050](../../experiments/EXP-050-ms2-timing-metadata.md). TD3 calibration investigation and VO/fusion changes remain separate from this synthesis.

## Evidence and verification

The table was extracted from restored machine-readable EXP-053 and EXP-054 reports. For each of ten primary groups, all 20 global and controlled risk ratios were recomputed from the saved scalar risks; minima/median/maxima and below-random counts were checked against the stored aggregates. Central-baseline medians were independently recomputed from those scalar risks. All 30 group-level checks passed. This checks the consolidation against saved reports, not the original prediction/reference arrays or a new execution of the historical numerical validators.

The EXP-054 recovery archive and all 501 member hashes were verified this session. Its recovery includes the EXP-053 scalar report used here. Historical experiment validation outcomes remain historical; they were not rerun for this document. No new inference, training, acquisition, selection threshold, calibration or fusion change was made.

Source reports (paths relative to ore-ms), with SHA256:

- `results/ms2-loss-tail-sensitivity/run-001/report.json`: `f0b38ac895dddbe6ae924fad6a230175128ede32f4169feb0e4e64c7aa8200d5`.
- `results/ms2-rain-replication/mean-l2-run-001/report.json`: `5581a24536badf9f044ee82cff7e2e0155ccc5df747c39e57cfa0256e6d1c7dd`.
