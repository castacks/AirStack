# DiTer++ PARK held-out epipolar consistency

Use all 118 temporal anchors and the saved
[SIFT correspondences](diter-plusplus-park-sift-2026-11-19.md). Compare
RGB, stretched thermal, and RGB randomly subsampled with one fixed seed
per anchor to the exact thermal correspondence count. Preflight pair
identities, counts, and input hashes before fitting. Use two complementary
half splits per arm and anchor. Fit a fundamental matrix on one half with
OpenCV FM_RANSAC at 1 px, confidence 0.999, and a 1,000-iteration cap.
Score the other half at a fixed 1 px square-root Sampson distance, then
swap halves. A target permutation within each held-out half provides a
chance control under the same fitted matrix. Run on native encoded
pixel coordinates with OpenCV 4.10.0 and NumPy 2.2.6.

| Window | Anchors | Median held-out passes, RGB / thermal / equal-count RGB | Median pass fraction, RGB / thermal / equal-count RGB |
| --- | ---: | ---: | ---: |
| Shaded approach | 30 | 595.5 / 55.5 / 60.5 | 98.1% / 88.2% / 97.2% |
| Passage exit | 59 | 640 / 95 / 101 | 98.5% / 92.5% / 98.1% |
| Daylight | 29 | 921 / 118 / 123 | 99.0% / 92.2% / 98.5% |

Median chance-control pass fractions range from 0.9% to 2.8% across the
nine window/arm summaries. One half-fit failed on an equal-count RGB
anchor during the exit and contributes zero passes for that half. All
other half-fits succeeded. A separate scalar audit reproduces 208,370
real and permuted distances, all 354 arm/anchor rows and all nine
aggregate summaries.

The stretched thermal matches retain substantial held-out geometric
support in each window. RGB has a higher median pass fraction even with
the same number of input correspondences. Count matching leaves spatial
coverage, lens distortion, and camera view different. A fundamental
matrix can agree with a dominant plane or a coherently moving object.
These fractions measure image-space model consistency on one exposed
episode. Pose accuracy and tracking recovery remain unmeasured.

The frozen protocol, hashes, folds, and fit results are in
`/root/ore-ms-artifacts/diter-park/frontend-geometry/`, with the runner
and scalar auditor in its parent directory.
