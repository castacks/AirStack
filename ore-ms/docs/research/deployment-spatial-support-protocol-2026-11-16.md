# Cached real-data spatial support protocol

16 November 2026. Exploratory characterization of exposed, locked predictions
from the [lighting/smoke study](deployment-characterization-findings-2026-11-16.md).
No new inference, model, trajectory, reference alignment or selector. Previous
pair outcomes are known; the spatial endpoints below precede their calculation.
Use all 5,036 predictions from all four conditions and both frontends/modalities.

## Coordinates and populations

Cached points are undistorted/virtual, not native pixels. Return lighting points
to the native distorted image with its published K/distortion. Return smoke points
through the published MEI/pinhole forward projection. Validate a full 64×64 query
grid round trip for each camera, maximum 0.05 native pixels, and record corrections.
Round reconstructed native coordinates to four decimal places for reproducible
boundary assignment. Admit at most 0.05-pixel boundary roundoff, then clip to the
native image rectangle; stop on larger excursions. Verify point/mask lengths,
finite values, frozen metadata/prediction hashes and source/target associations
for every declared interval before calculating cohort outcomes.

Normalize by each sensor's native width/height. The fields of view differ; cells
are fractions of an image, not equal physical areas across cameras. Keep SIFT and
learned frontends separate. SIFT uses keypoints, learned uses a fixed dense grid;
coverage comparisons across frontend families are not causal sensor effects.

## Spatial and temporal endpoints

Use an 8×8 image grid. A cell has support when it contains at least three accepted
correspondences. For each pair report source/target cell counts and their minimum,
convex-hull area fraction, normalized cell entropy, largest-four-cell fraction,
central-quarter-area fraction, and median native displacement. Empty sets have
zero coverage/hull/entropy and undefined concentration/displacement. Compute the
same diagnostics separately for all accepted matches and the cached cheirality
mask, including masks from low-support estimates. Do not replace matching failure
or missing inputs with an identity estimate or a successful image.

Primary descriptive continuity endpoint: at least 16 of 64 supported cells in
both endpoints for each of three consecutive intervals (1.5 seconds). Report all
fixed sliding windows; broken source/target image linkage is a reported failure,
not an exclusion. Also report the minimum cell count over the window and number
of cells shared across all six endpoint occupancy maps. These describe sustained
image-area support: **they do not link point identities or certify persistent
world tracks**. Nearest-neighbor linking of sparse grid queries is deliberately
avoided because resampling can replace point identity.

Report continuous distributions alongside the 3-points/16-cells/3-interval
convention; do not tune those choices after outcomes. Preserve all conditions,
frontends and smoke's prospectively fixed thirds. Relate results descriptively to
existing RGB contrast bins <40, 40–120, ≥120, and median image-displacement bins
<1, 1–5, ≥5 native pixels. Displacement is matcher-derived, not physical motion.
Static intervals remain included. Do not label concentration as a moving person
without semantic/world-track evidence or equate coverage with correct motion.

## Qualification and interpretation

Qualify empty, uniform, concentrated and boundary point sets; masks and broken
window linkage; projection round trips across the full grid. Freeze protocol,
runner and consumed inputs before calculating cohort summaries. An independent
auditor recomputes spatial histograms, window definitions, paired outcomes and
aggregates from the cached data, with no import of the analysis runner.

This study can distinguish dense/localized and intermittent/sustained matcher
support. The prior camera/reference issue still blocks physical pose-accuracy
interpretation. A stable hallucination can cover the image. A hindsight RGB/thermal
union is a descriptive ceiling, not achieved fusion performance. Stop after this
finite cached analysis; retain adverse and null cases and archive evidence.
