# IRS RTVI faster-motion dark replication

The [initial IRS comparison](irs-rtvi-mocap-comparison-2026-11-17.md) found
higher nominal agreement for thermal-first on `mocap_dark`, and for
visible-first on `mocap_easy`. Before running inference on
`mocap_dark_fast`, fixed the directional replication target:
thermal-first should give more agreements within 5° of both nominal motion
signals than visible-first, on the common reference-supported population.

Used the same native mono8 input, published equidistant intrinsics,
exposure-header offsets, SIFT and learned checkpoints, status-only policies
and complete half-second population as for the initial IRS test.
Acquired only the 559,105,332-byte compressed member of the public archive,
verified the decompressed 1,153,848,762-byte bag by ZIP CRC and recorded its
SHA-256. The decoded streams contain 1,736 visible images, 1,737 thermal
images and 35,560 IMU messages. No archive or Box copy was staged.

The released Vicon–IMU trajectory has 1,420 rows over 71.549 s. Exclude
brackets spanning more than three nominal 50 ms mocap steps from the
agreement comparison. They remain in the support ledger. This excludes
three intervals per camera around the 650 ms reference gap. Both cameras
have 139 paired reference intervals, with recorded-time mocap/gyro angular
disagreement p95 of 0.619° visual and 0.600° thermal. The camera estimators
do not consume the reference signals. The gyro contributes to the published
Vicon–IMU trajectory, so the consistency comparison has known dependence.
No physical 1° accuracy, translation or trajectory recovery is inferred.

## Frozen half-second result

There are 173 full-span anchors and 139 common reference-supported anchors.
Median nominal camera rotation on the bracketed intervals is about 28.5°.

| Frontend | Full visible / thermal / either proposals | Visible-first within 5° of both | Thermal-first within 5° of both |
|---|---:|---:|---:|
| SIFT | 19 / 52 / 69 | 35 / 139 | 36 / 139 |
| Learned | 66 / 61 / 91 | 33 / 139 | 56 / 139 |

The learned direction replicates the earlier dark result. Both orders
produce 81 proposals on the 139 common reference-supported anchors.
Thermal-first adds 23 nominal agreements through different choices where
both outputs exist. Fixed learned thermal yields 48/56 dual-reference
agreements on its own paired proposals, versus 7/56 for identity.
Fixed learned visible yields 17/60 versus 10/60 for identity.
All 692 primary prediction rows passed audit.

## Shorter-interval control

A separate, frozen control kept all 173 source anchors and each exact
source frame, and requested the nearest target to source plus 100 ms.
All camera, matcher, calibration, policy and reference-gap rules remained
fixed. This changes the inter-frame motion and is an interval-sensitivity
check, with the original half-second result retained.

| Frontend | 0.5 s full visual / thermal / either proposals | 0.1 s full visual / thermal / either proposals |
|---|---:|---:|
| SIFT | 19 / 52 / 69 | 35 / 116 / 130 |
| Learned | 66 / 61 / 91 | 118 / 127 / 132 |

On the 139 anchors jointly reference-supported at both durations, learned
thermal-first produces 81→127 proposals and 56→127 nominal 5° agreements.
Within the 85 thermal anchors where both references indicate more than 5°
of motion at **both** durations, the learned thermal arm rises from
21→85 proposals and 15→85 agreements. SIFT thermal rises from 17→78
agreements on the same subset. Identity cannot satisfy the 5° criterion
on that moving subset. This limits the explanation that shorter intervals
helped only by shrinking rotations below the threshold.

Whole-record support improves, while the longest no-proposal block grows
from six to 22 sampled anchors. The 22-anchor block spans indices 13–34,
with four bracketed anchors near static (median nominal thermal motion
0.021°). These are sampled proposal outcomes, with no live continuity or
stateful outage claim. At the short duration the two learned priority
orders still produce 128 proposals on 140 common bracketed anchors.
Thermal-first agrees on 128 and visible-first on 107.

Audited all 692 additional prediction rows and verified same-source frame
identities, motion-conditioned denominators and checksums. Evidence is
under `/root/ore-ms-artifacts/irs-rtvi-intake/`. Only the fast bag was
added this turn. All three IRS bags occupy about 4.0 GiB locally, and no
Box storage was added.
