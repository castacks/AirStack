# Near-stationary support is enriched in essential inliers of fixed failures

16 November2026 America/New_York. Completed the
[frozen essential-inlier mixture protocol](essential-inlier-mixture-protocol-2026-11-16.md)
after the [match/E location diagnostic](motion-compression-location-findings-2026-11-16.md).
All1,712moving rows remain across both records, cameras and frontends. This is
an exposed-cache diagnostic, not a new pose method, selector or causal intervention.
No matches, E matrices, masks, estimator settings or outputs were changed.

## The fixed exception cases now have a more specific explanation

The prior study found cases with substantial accepted match motion but a small
rotation encoded by E. The original essential mask enriches near-stationary
correspondences in these cases. Near-stationary here means per-match unit-ray
change≤0.1°, a fixed descriptive cut distinct from the prior pose-angle assay.

| Fixed old sample | Low / accepted | Low / essential inliers | Low / cheirality | Accepted median ray change | Essential median | Cheirality median |
|---|---:|---:|---:|---:|---:|---:|
| SmokeRoom 634 rgb | 400/1330 | 365/692 | 0/193 | 9.614° | 0.086° | 12.054° |
| SmokeRoom 748 thermal | 321/874 | 310/591 | 0/246 | 8.351° | 0.089° | 9.150° |
| FlashLight 476 rgb | 697/2206 | 666/1157 | 0/373 | 5.253° | 0.075° | 7.057° |

SmokeRoom634RGB starts with400/1330near-stationary matches (30.1%). The essential
mask retains365of400(91.3%), versus327of930larger-motion matches(35.2%). Its
inlier set is365/692near-stationary(52.7%) and median ray change falls9.614→0.086°.
The same fixed E encodes0.153°/179.966°rotation candidates, both wrong against
12.900°reference motion. Selecting a branch was already ruled out as a repair.

Cheirality then removes all365near-stationary E inliers. The193remaining points
have median ray change12.054°, but recovery returns a rotation from the already
estimated E. That filtering step does not refit E or its two rotation candidates.
This resolves why substantial motion could survive in the final positive-depth
subset while the returned rotation stayed compressed.

SmokeRoom748thermal andFlashLight476RGB show the same mask progression in the
table, preserving both camera and recording controls. The latter retains95.6%of
near-stationary accepted matches versus32.5%oflarger-motion ones. These are
properties of fixed model-selected masks, not evidence a different mask would
improve poses safely.

## Whole fixed strata preserve the controls and empty subsets

Each fraction below is the median per-pair fraction; n is the number of pairs
with a nonempty mask. Outputs/anchors is separately reported because a rejected
proposal can retain some essential or cheirality points. No matches are pooled
across pairs and no failed proposal is removed from the original cell population.

| Recording | Prior moving R≤5° correctness cell | Camera | Outputs / anchors | Near-stationary accepted | Near-stationary essential | Near-stationary cheirality |
|---|---|---|---:|---:|---:|---:|
| FlashLight | both_correct | rgb | 107/107 | 16.4% (n=107) | 0.2% (n=107) | 0.0% (n=107) |
| FlashLight | both_correct | thermal | 107/107 | 8.1% (n=107) | 0.0% (n=107) | 0.0% (n=107) |
| FlashLight | rgb_only_correct | rgb | 78/78 | 17.8% (n=78) | 0.0% (n=78) | 0.0% (n=78) |
| FlashLight | rgb_only_correct | thermal | 54/78 | 64.2% (n=75) | 66.7% (n=74) | 22.3% (n=64) |
| FlashLight | thermal_only_correct | rgb | 3/3 | 28.1% (n=3) | 45.7% (n=3) | 0.0% (n=3) |
| FlashLight | thermal_only_correct | thermal | 3/3 | 17.7% (n=3) | 3.6% (n=3) | 0.0% (n=3) |
| FlashLight | neither_correct | rgb | 4/4 | 30.8% (n=4) | 50.0% (n=4) | 0.0% (n=4) |
| FlashLight | neither_correct | thermal | 4/4 | 42.8% (n=4) | 53.9% (n=4) | 33.8% (n=4) |
| SmokeRoom | both_correct | rgb | 154/154 | 20.6% (n=154) | 0.1% (n=154) | 0.0% (n=154) |
| SmokeRoom | both_correct | thermal | 154/154 | 0.0% (n=154) | 0.0% (n=154) | 0.0% (n=154) |
| SmokeRoom | rgb_only_correct | rgb | 27/27 | 19.8% (n=27) | 0.0% (n=27) | 0.0% (n=27) |
| SmokeRoom | rgb_only_correct | thermal | 22/27 | 40.6% (n=27) | 55.5% (n=27) | 0.0% (n=25) |
| SmokeRoom | thermal_only_correct | rgb | 25/36 | 62.0% (n=36) | 68.2% (n=36) | 0.0% (n=33) |
| SmokeRoom | thermal_only_correct | thermal | 36/36 | 15.6% (n=36) | 0.0% (n=36) | 0.0% (n=36) |
| SmokeRoom | neither_correct | rgb | 15/19 | 43.9% (n=19) | 66.9% (n=19) | 0.0% (n=17) |
| SmokeRoom | neither_correct | thermal | 19/19 | 37.1% (n=19) | 53.5% (n=19) | 0.0% (n=19) |

For SmokeRoom thermal-only-correct RGB cases, the median near-stationary fraction
rises62.0%→68.2% from accepted to essential inliers; the cheirality median is0%.
RGB-only-correct thermal cases rise40.6%→55.5%, also with0%cheirality median.
In both-correct cases, essential fractions have medians about0.06%RGB/0%thermal.
This is a labelled group difference, not an evaluated quality classifier.
FlashLight's three thermal-only-correct cases are a small set and remain visibly
separate from its many RGB-only-correct cases.

Essential retention medians, conditional on a nonzero low/high denominator:

| Recording | Correctness cell | Camera | Near-stationary retention | Larger-motion retention |
|---|---|---|---:|---:|
| FlashLight | both_correct | rgb | 0.7% (n=107) | 75.1% (n=107) |
| FlashLight | thermal_only_correct | rgb | 95.6% (n=3) | 40.5% (n=3) |
| FlashLight | rgb_only_correct | thermal | 100.0% (n=73) | 89.4% (n=72) |
| FlashLight | neither_correct | thermal | 100.0% (n=4) | 53.0% (n=4) |
| SmokeRoom | both_correct | rgb | 0.2% (n=154) | 83.8% (n=154) |
| SmokeRoom | thermal_only_correct | rgb | 96.8% (n=36) | 74.9% (n=36) |
| SmokeRoom | rgb_only_correct | thermal | 100.0% (n=27) | 65.0% (n=26) |
| SmokeRoom | neither_correct | thermal | 100.0% (n=19) | 51.3% (n=19) |

Retention n can differ between low and high groups; absent near-stationary
matches yield null retention rather than zero. In particular a high conditional
retention median for a group with few low-motion matches does not mean the E
inlier set is dominated by them. The fraction and count tables must be read
together. Exact contingencies, all quartiles and missing counts remain saved.

## Spatial coverage and fixed overlays narrow the mechanism hypothesis

SmokeRoom634RGB essential inliers occupy23of64source cells and have a source
convex-hull fraction0.832. FlashLight476RGB has36occupied cells and hull0.904.
Both are wrong rotations despite wide image coverage. A large hull/cell count
alone is not validated protection against the motion mixture; this remains
consistent with the earlier coverage/geometry null results.

All24previously locked sample anchors were rendered as48camera source overlays.
Gray marks accepted points, red essential points with≤0.1°ray change, blue other
essential points, and black outlines the cheirality subset. Source/native mapping,
file hashes and exact timestamps are preserved. There was no point subsampling,
new image selection, segmentation or changed field of view.

In the inspected SmokeRoom634RGB andFlashLight476RGB overlays, many red points
lie over the visible platform and image-margin regions, while scene structures
carry larger-motion points. SmokeRoom748thermal similarly places much of its
red support near the visible lower rig region. Other views, such as SmokeRoom604
RGB, have near-stationary support spread across a veiled image area as well.
The evidence motivates a camera-fixed/invalid-region hypothesis but does not
assign every point to an object or prove correspondence identity, a physical
mask boundary or a causal source of error. Thermal and RGB can both exhibit it.

Accepted points, essential inliers and positive-depth points are different
populations. A correct mask intervention must be tested before fitting E if it
is intended to change that model; the existing post-E subset statistics are not
an achieved intervention or a reason to relax cheirality gates.

## Verification, evidence and stopping decision

All8tests pass, including mixture/null/camera-association/nested-mask controls
and rehashed low-motion-count, spatial-grid and aggregate corruptions. The
independent audit passes1,391,633numeric checks on1,712rows. It reconstructs
native Brown/MEI projection independently, ray angles via atan2, masks and motion
contingencies,8×8grid counts, hulls through a separate SciPy hull implementation,
entropy/concentration and every fixed cohort/stratum aggregate. The overlay audit
passes58source/sample/time/figure-hash checks on48source images.

Native projection controls are within the unchanged0.05pxcontract for each own
record/camera. Original accepted/essential/cheirality counts and angular medians
match the prior location cache. Empty masks and null denominators remain explicit.
Source-only feature rows lock before reference-cell joins. No source/protocol or
frozen estimator output was revised after joining labels.

These are nominal supplied-reference development data, with physical exposure/
reference independence and training overlap still qualified. Previous SubT-MRS,
OpenCV and uncertainty/closure/geometry source reading was reused; no new paper
body reading, author-system reproduction or novel gating claim.

Results: `results/essential-inlier-mixture/`; numeric features/summary/locks under
`run-001/`, eight fixed overlay sheets under `run-001/overlays/`, and mixture
PNG/PDF under `figures/`. Verified incremental local packet:
`/root/ore-ms-artifacts/essential-inlier-mixture-2026-11-16/`.
No Box upload, Git push or automatic commit occurred.

Stop decision: the fixed exceptions show a stationary/moving mixture being
selected into E, followed by removal of much stationary support during cheirality
without a new E fit. Camera-visible rig/margin support is a plausible contributor,
not a proven sole cause. Next qualify image-only masks for rig/invalid regions
and freeze a controlled pre-E exclusion ablation on full records with unchanged
unmasked controls and all failures retained. Do not select masks from error gains
or assume the ablation must improve either camera; review the existing source
and prior-art overlap before recommending a method.
