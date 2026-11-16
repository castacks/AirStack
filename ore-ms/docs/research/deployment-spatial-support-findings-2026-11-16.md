# Where thermal adds spatial and temporal support

16 November 2026. Completed the [prospective cached-analysis protocol](deployment-spatial-support-protocol-2026-11-16.md)
on all 5,036 previously locked predictions: 10,008 sliding-window records across
conditions, modalities, frontend families and support types. **Zero new inference,
training, calibration fitting or parameter search.** These exposed-data diagnostics
measure image support, not verified feature identities, world tracks or pose error.

Reference follow-up: [released calibration audit and scalar angular motion](reference-contract-findings-2026-11-16.md)
now establish a usable rotation-amount diagnostic while full camera/mocap frame
binding remains unresolved. This supersedes the next-task suggestion below;
current execution priority remains in the handoff.

## Decision

The [earlier minimal-pose availability gains](deployment-characterization-findings-2026-11-16.md)
do not imply a broad, sustained geometry advantage. Learned RGB generally retains
more spatial support in these recordings. Thermal still adds support in intervals
and windows where RGB lacks it, especially late in SmokeRoom, so neither an
always-thermal policy nor a universal RGB-only conclusion follows.

The next priority is to qualify the camera/reference binding independently of
these predictions before interpreting full pose accuracy. More availability-only
measurements cannot by themselves establish thermal VO accuracy. Keep the current
spatial evidence as deployment characterization, without a new selector or method.

## Definition and whole-cohort results

Partition each native image into 8×8 cells. A supported cell has ≥3 accepted
correspondences. Broad support requires ≥16 cells in both endpoints. Sustained
support requires that condition for all three adjacent half-second intervals
(1.5 seconds) with exactly linked source/target images. All windows count; none
were selected for success. All 5,036 input pair records link consistently.

Report accepted matches separately from the cached cheirality-filtered subset.
In particular, a learned forward/backward-consistent dense grid can cover an image
without estimating correct motion. Pure rotation and static views can also have
plentiful correct correspondences without translation support.

| Learned frontend, condition | Broad accepted support sustained, RGB | Thermal | Broad geometry-filtered support sustained, RGB | Thermal |
|---|---:|---:|---:|---:|
| Bright | 134/134 (100.0%) | 131/134 (97.8%) | 61/134 (45.5%) | 49/134 (36.6%) |
| Changing light | 145/145 (100.0%) | 145/145 (100.0%) | 52/145 (35.9%) | 43/145 (29.7%) |
| Dim | 136/136 (100.0%) | 133/136 (97.8%) | 55/136 (40.4%) | 54/136 (39.7%) |
| Entire mixed SmokeRoom | 833/836 (99.6%) | 828/836 (99.0%) | 269/836 (32.2%) | 184/836 (22.0%) |

These are overlapping-window proportions, not independent-trial probabilities or
stateful VO recovery rates. A window is three intervals; its four endpoint images
span 1.5 seconds. The repeated six occupancy maps include shared intermediate images.

Median minimum supported cells per pair, learned accepted correspondences:
bright RGB 63.5 / thermal 52; changing 63 / 53; dim 62 / 51; smoke 63 / 50.
For cheirality-filtered matches: bright 55.5 / 39.5; changing 28 / 31;
dim 54 / 32.5; smoke 6 / 7.5. The changing-light pair median favors thermal while
the sustained-window proportion favors RGB, illustrating why these endpoints
must remain separate.

Dim learned accepted matches put a median 7.9% of RGB points in the four most
populated cells, versus 12.2% for thermal. With cheirality filtering, these are
9.4% and 20.8% among nonempty sets. Thermal support is more concentrated by this
measure. This does not identify the supporting object or prove that matches lie
on a person. Normalized native-image areas also cover different physical fields
of view; no equal-world-area comparison is implied.

SIFT's dim RGB collapse remains: median accepted-cell coverage is zero versus
two for thermal. Neither SIFT arm sustains the declared broad spatial support in
dim light. The earlier thermal SIFT advantage supplied sparse constraints rather
than the broad coverage required by this particular diagnostic. The 16-cell
criterion is a declared descriptive convention, not a demonstrated requirement
for successful VO; it was not retuned to favor either modality.

## Complementarity survives, with narrower interpretation

Learned dim windows: both modalities satisfy broad geometry support in 34 cases,
RGB alone in 21, thermal alone in 20, neither in 61. The hindsight union is
75/136 (55.1%), versus RGB's 55/136. This is a support ceiling, not an achieved
fusion, deployable selector or pose-accuracy benefit.

Mixed SmokeRoom has 52 thermal-only broad-geometry windows and 137 RGB-only
windows. Its fixed thirds clarify the mixture:

| SmokeRoom fixed region | Windows | RGB broad geometry | Thermal broad geometry | Thermal-only / RGB-only |
|---|---:|---:|---:|---:|
| First third | 278 | 14 (5.0%) | 11 (4.0%) | 0 / 3 |
| Middle third | 277 | 149 (53.8%) | 66 (23.8%) | 5 / 88 |
| Final third | 277 | 105 (37.9%) | 106 (38.3%) | 47 / 46 |
| Crossing a third boundary | 4 | 1 | 1 | 0 / 0 |

The final third contains 47 of the 52 thermal-only windows, but its aggregate
rates are nearly tied. The previous +5.4 percentage-point thermal advantage in
minimal per-pair geometry there becomes about +0.4 points with this sustained,
spatially distributed criterion. Every third remains included. Boundary-crossing
windows are a separate stratum so no windows disappear from the full denominator.

Fixed-time images establish that SmokeRoom mixes clear, stationary, dynamic-person
and visibly obscured segments. A third is not a smoke-density label. Neither the
late concentration of thermal-only windows nor the contrast/displacement bins
is a causal estimate of smoke transmission or a validated sensor switch.

## Qualification, audit and limits

Points were cached in undistorted/virtual coordinates. We mapped them back to
native distorted pixels using the unchanged calibrated projection adapters.
Maximum full-grid round-trip discrepancy was 2.8e-8 native pixels for lighting
and about 1.2e-12 for smoke, below the prospective 0.05-pixel gate. Native points
are rounded to four decimals for stable cell boundaries. All point/mask lengths,
finite values, frozen input hashes and source/target associations qualify.

The cached-analysis CPU runtime used the same pinned geometry image as the parent
study (OpenCV 5.0.0, NumPy 2.2.6); no GPU/model calls were required.

Empty/uniform/concentrated/boundary tests and the actual unchanged runner on
synthetic broken-link/zero-mask windows passed before real cohort analysis.
An independent auditor uses explicit radial/tangential projection polynomials,
histogram bins and window recomputation without importing the analysis runner:
221,555 checks passed, plus 160 independent paired-window/third-stratum checks. These validate arithmetic and provenance, not physical
correspondence correctness, camera metrology, full pose accuracy or persistent
world-point identity. No semantic/background segmentation was performed.

All original pairs, missing input, frontend differences and nominal clock
assumptions remain. Static views are not discarded. Within-condition results,
fixed contrast bins (<40, 40–120, ≥120) and RGB-matcher displacement bins
(missing, <1, 1–5, ≥5 pixels) are saved. Displacement is an estimate-conditioned
proxy, not independent motion truth. For example, 133/138 learned dim pairs lie
in the ≥5-pixel RGB-displacement bin, so this condition's result is not dominated
by the near-static proxy bin. Do not treat that as physical motion qualification.

The earlier lighting reference-frame sanity failure remains unresolved; no axis
flip, hand-eye transform or clock was fitted. Reading/source contracts from the
NGCLAB and SubT-MRS entries are reused; no fresh paper reading or author-system
reproduction occurred. No model, weight, query or threshold rescue followed.

## Evidence and recovery

Ignored `results/deployment-spatial-support/` contains the native cached points,
per-pair occupancy/entropy/concentration/hull/displacement metrics, every sliding
window, fixed strata, paired cells, synthetic fixtures, freeze and audits.
`spatial-continuity.png/.pdf` compares accepted and geometry-filtered support.
The small incremental recovery packet is
`/root/ore-ms-artifacts/deployment-spatial-support-2026-11-16/`; it includes consumed
cached predictions and calibrations, without duplicating raw images or checkpoints.
The parent full-input packet remains
`/root/ore-ms-artifacts/deployment-characterization-2026-11-16/`.
No author outreach, Box upload or Git publication occurred.
