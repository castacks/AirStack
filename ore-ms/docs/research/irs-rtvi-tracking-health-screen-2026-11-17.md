# IRS RTVI warning signals before visual pose degradation

The [paced replay](irs-rtvi-paced-recovery-2026-11-17.md) maintained
publication during missing visual input. On fast dark, the visual estimate
was already inaccurate while frames still arrived on time. This screen
asks whether three observable signals rank future visual displacement
error while input continues.

Both signals use the previous audit's frozen one-second displacement
labels and 0.15 m error threshold. Observations are at least 150 ms old
in sensor time, with no future feature or thermal output used as a
predictor. This guard approximates the measured replay delivery latency.
It does not qualify hardware availability. The tracked-count comparator
uses the same eligible anchors and age guard as each signal. The
prespecified fast-dark gate requires at least 70% coverage, ten examples
of each class, AUROC of at least 0.75, and an improvement of at least
0.10 over the matched tracked-count reference.

## Fast-dark outcomes

| Warning signal | Eligible anchors | High / low error | Signal AUROC | Matched count AUROC |
|---|---:|---:|---:|---:|
| Rejected fraction of attempted visual updates | 48 / 68 | 21 / 27 | 0.600 | 0.690 |
| Visual–thermal motion disagreement, preceding samples | 8 / 68 | 4 / 4 | 0.813 | 0.625 |
| Visual–thermal motion disagreement, causal resampling | 61 / 68 | 31 / 30 | 0.670 | 0.621 |
| Raw visual spatial texture | 68 / 68 | 31 / 37 | 0.594 | 0.632 |

The visual diagnostic counts actual feature-update attempts and accepted
final tracking statuses before adding new features. It aggregates a
250 ms history with at least four health reports and eight attempted
updates. The first fast-dark subscriber capture missed the 99% pose coverage
requirement. A rerun in isolation retained 1,413 of 1,418 native
reference-associated poses. Every common pose row matched exactly in all
three recordings with instrumentation enabled. Rejected fraction ranks
fast-dark risk below the matched tracked-count control.

The dual-estimator diagnostic measures disagreement between trailing
0.5 s displacement vectors in each filter's initial local body frame. It
uses independently maintained camera estimates and their common IMU. It
uses no ground truth or world alignment to compute disagreement. A
strict preceding-sample association produced eight fast-dark anchors
because the two camera timelines have separate phases. That subset is
insufficient to qualify the signal. A separate coverage follow-up
interpolated a common earlier interval only when both bracketing
samples were already old enough and at most 75 ms apart. It recovered
61 anchors and missed both discrimination targets. The initial follow-up
run exposed a JSON serializer incompatibility with a NumPy Boolean. The
serialization fix left the risk scores and gate unchanged.

The raw-image diagnostic pools adjacent pixel gradients before estimation
and uses the median over the same age-gated window. It scores all
fast-dark anchors and ranks future risk below the matched count control.

Easy and dark each have at most one eligible example in the minority
class under the well-covered methods. They remain controls for the
condition contrast. Their per-record AUCs are too unstable to establish
warning quality. The auditable result is that none of the screened signals
qualifies to trigger a handoff at the tested horizons. Another selector
trial needs a separately qualified health signal and a recording with
enough naturally changing visual conditions to assess it independently.
The fixed-thermal result remains the measured comparison in known dark
conditions. The locally staged [ViViD++ handheld data](vivid-three-condition-support-2026-11-17.md)
include a varying-flashlight sequence. A stateful comparison there still
needs to qualify the different image encodings, camera rates and supplied
reference before it can validate another switch policy.

The frozen protocols, source patches, failed captures, per-anchor ledgers
and independent AUC check are in the `innovation-health`,
`thermal-disagreement-health`, `thermal-disagreement-resampled` and `image-texture-health`
directories under `/root/ore-ms-artifacts/irs-rtvi-intake/stateful/`.
No additional bags or Box storage were used.
