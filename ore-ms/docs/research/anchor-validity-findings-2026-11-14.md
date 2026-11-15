# Controlled anchor validity: existing uncertainty rejects the constructed negatives

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.
Fixed36candidate assessment complete. No training or method tuning.

## Result and disposition

The existing native uncertainty-based selector supplies usable support for all
12same-world candidates and zero proposed queries for all24different-world
candidates, in both directions. These are successful abstentions on this
constructed negative set, not solver failures. All24eligible positive-direction
pose fits converge;48negative directions stop before pose fitting.

The fixed-crop uncertainty readout also separates these classes (AUROC1.0),
but no negative survives the original gate. Therefore this assay cannot identify
additional uncertainty value beyond the existing gate: conditional AUROC is
undefined, not zero or evidence that all future confidence modeling is useless.
The added cycle gate rejects one same-world candidate without rejecting any
additional negative. Preserve this positive baseline outcome rather than
inventing a new detector or escalating severity to manufacture a failure.

## References bank confirmed

All eleven papers from the expanded reading map have unique JSON entries and
BibTeX entries. Five were new additions (DeepTIO,TI-SLAM,ETIO,TRGS-SLAM,DSO);
the other six already existed and received reading-history updates.
Machine-readable check:`results/anchor-validity/references-bank-check.json`.
No duplicate paper records were added during this confirmation.

## Paper connection and frozen design

[TI-SLAM](https://arxiv.org/html/2104.07196v1) already combines probabilistic
thermal odometry/loop constraints with geometric loop rejection.
[TRGS-SLAM](https://arxiv.org/html/2603.20443v1) already has persistent thermal
mapping/recovery and explicitly limits past-keyframe sampling and relocalization
by overlap/drift. See the [expanded source review](expanded-vo-thermal-literature-2026-11-14.md).
These sources motivated an evaluation boundary, not a claim of a new detector
or of failure in either author system.

Reuse the six artificial worlds from the prior continuity study: three exposed
thermal texture sources ×plane4m/two-depth4/8m geometry. Source frame0 is queried
against target frame6 from the same world and each of the two other texture
worlds with the same geometry. Each of18world pairs has clean and target-gamma1.6
versions,36candidates total:12positives and24negatives. Gamma changes both target
stereo views identically and is present in both validity classes. No new rendering,
image severity search, checkpoint, selector threshold, training or pose solver.

World-instance labels are an explicit artificial scene construction. They are
not independent natural place identities; the source photographs' day/night/rain
labels do not prove whether the original recordings depict the same physical
place. Separate artificial worlds have no declared shared landmarks even though
their plane geometry and numerical camera coordinates are identical. These
negatives may be easy to distinguish through texture/domain differences, so
perfect separation does not establish general place-recognition capability.

Both directions are predicted for each candidate,72three-pair batches. Released
MACT checkpoint/native selector/covariance bodies and existing optical IRLS
harness remain fixed. An explicit insufficient-support guard rejects fewer than
three proposed/inbound queries before empty tensor kernels; it changes no native
selection rule or positive-input math. The application gate requires10points
and convergence. Prepared inputs contain images only; no scene labels/true pose
are supplied as estimator inputs. The capture function ignores manifest labels.

## Controls and exact endpoints

Native fit/support is followed by two established-style controls:

- Independent unweighted3D rigid RANSAC:256fixed-seed three-point hypotheses,
  non-collinear samples,0.03m residual bound; at least10inliers and50%inlier
  fraction in each direction. No final RANSAC-refined native estimate is promoted.
- A bidirectional cycle using the two RANSAC transforms: translation closure
  at most0.03m and rotation at most1deg, after geometry passes.

These are predeclared engineering gates in the synthetic metric scene, not
probability-calibrated thresholds or a reproduction of TI-SLAM's multi-loop
GraphTinker-style checks or TRGS-SLAM's map/render rollback.

The uncertainty readout is the median of forward temporal x/y covariance trace
on a fixed32pixel-border crop, before model-selected queries; lower is the
predeclared direction for a positive connection. No new head, fitted scale,
combined learned score or adaptive threshold. Primary counts include every
attempted candidate; report clean/gamma separately and retain undefined
conditional ranking when either class is absent. Pose errors exist only for
same-world positives and are distinct from association correctness.

## Results

| Fixed gate | Same-world retained | Different-world passed | Clean same-world retained | Gamma same-world retained |
|---|---:|---:|---:|---:|
| Native fit/support |12/12|0/24|6/6|6/6|
| Plus geometric verification |12/12|0/24|6/6|6/6|
| Plus bidirectional cycle |11/12|0/24|5/6|6/6|

All24negative candidates have zero proposed native queries in both directions;
the24positive directions propose44–94queries. Negative rejection happens before
RANSAC and pose fitting. Existing learned uncertainty/selection is already doing
useful work in this fixed assay; geometry cannot gain further negative rejection
when no such candidates reach it.

Median covariance trace ranges are0.017664–0.070413for positives and
50,422.2–464,723.5for negatives, in native covariance output units. Overall,
clean and gamma AUROC are each1.0on these dependent constructed samples.
There is no physical covariance or association-probability calibration claim.
Every post-gate conditional AUROC is undefined because its negative count is0.

The rejected same-world candidate is clean day-plane4m. Its RANSAC cycle
translation discrepancy is31.712mm versus the fixed30mm bound; rotation0.4328deg
passes. Its forward native weighted pose error is1.518mm/0.01912deg. These are
different estimates/endpoints: the unweighted sampled geometric cycle can
reject a valid association despite accurate native forward pose. Do not widen
the gate, substitute the native pose into the cycle after inspection, or promote
the favorable gamma counterpart as a fix.

Positive forward native errors range0.714–5.736mm and0.00871–0.07248deg across
all12positives. Artificial texture/pose/appearance effects remain; no natural
outage, wrong-place distribution, physical sensor noise, online deadline or
independent recording generalization is established.

![Controlled gate counts and uncertainty readout](../../results/anchor-validity/scoring-001/anchor-validity.png)

## Verification and stopping decision

Before inference275checks qualify case identity, gamma stereo construction,
rigid-transform fitting, degeneracy/empty-support handling, known outliers,
AUC direction/ties and the fact that a coherent false cycle can pass geometry.
After the complete prediction lock,1,629checks pass input/model/source locks,
all72directions, native selection/packets/covariances, all24eligible fits'
stationarity/information, independently reconstructed RANSAC consensus through
SciPy alignment, cycles, counts, positive pose errors and conditional AUC rules.
No producer/auditor/threshold amendment after freeze; first executions pass.

Close the fixed assay as informative positive evidence for the existing frontend.
It does not establish an incremental confidence method or exhaust realistic
thermal association challenges. A future proposal needs a distinct informative
regime and qualified labels, rather than another score or a search for a failing
anchor. Thermal representation, sensor transfer, sensor-physics models and the
other original-paper gaps remain available directions.

Input/estimator freezes:
`experiments/anchor-validity-{input,estimator}-freeze-2026-11-14.json`.
Raw case metadata/images,72dense predictions,24pose packets/fit receipts,
48explicit insufficient-support receipts, prediction lock, scores/CSV/figures
and audits:`results/anchor-validity/`.
