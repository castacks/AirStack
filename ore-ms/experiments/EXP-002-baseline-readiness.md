# EXP-002: RGB compatibility, timestamp pairing, and geometry readiness

Date: 2026-10-17 (America/New_York). Branch `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c`, with the accompanying uncommitted
checkpoint/pairing/audit implementation. Runtime remains the Blackwell image
recorded in EXP-001. See EXP-001 for archive checksum and input provenance.

## Results

| Check | Result |
|---|---|
| RGB legacy migration | PASS: exact known state signature, strict active-state load |
| RGB real temporal inference | PASS: finite flow and positive covariance, `(1,2,544,1024)` |
| MACT real temporal inference | PASS: native strict load, `(1,2,512,640)` |
| Active parameters/buffers vs original permissive wrapper | Exactly equal for both checkpoints |
| Focused regression checks | 11 passed |
| Four-camera acquisition pairing | 1320 unique groups; all paths and time bounds verified |
| Thermal TF vs stereo projection | Unresolved under both tested serialized conventions |
| Full VO / geometric accuracy | Not run |

## RGB compatibility evidence

FlowFormer commit `39f63e657ccf2085977aad717f849842e18c77dd` removed unused
encoder final norms and the unused GMA attention position-embedding module.
Commit `4bcccbd153ca50dc86332e6db7cb59f48c4c239b` registered the fixed
9x9 local sampling grid as `memory_decoder.delta`. The RGB checkpoint contains
exactly seven obsolete entries and lacks that buffer. The migration removes
only this exact legacy signature, generates the deterministic grid, and loads
all remaining model state strictly. Other missing/extra keys and shape mismatches
are rejected. Original checkpoint files remain unchanged.

Real-image checks use frames 0 and 1 of RGB-left rectified color and thermal-left
processed, BGR→RGB, float32 /255, native dimensions, seed 0, no model downloads.
Flow/covariance arrays and reports are under local ignored
`results/data-access/real-frontend-compatible.json` and the corresponding NPZs.
This is execution validation, not proof of original thermal training preprocessing.
Exact state equality against the previous wrapper was checked; equivalence to
an unrecovered historical training implementation is not claimed.

## Acquisition manifest

`prepare_td3_sample.py` uses chronological nearest matching without reuse.
Missing image/timestamp rows are dropped, repeated acquisition timestamps are
deduplicated, and source indices/paths remain explicit. Ties prefer earlier
times. This is a greedy policy, not a globally optimal assignment.

For this engineering run, stereo limit = 2 ms and cross-modal limit = 30 ms.
Every RGB/thermal acquisition combination within a group satisfies the cross
bound. These are exposed exploratory thresholds, not a finalized research
synchronization protocol. 1320 groups pass; every source exists, timestamps
increase strictly in each stream, and no acquisition is reused. The first ten
selected source indices are 0–9 in all streams. Manifest:
`results/data-access/paired-manifest.json`.

## Geometry findings and remaining dependency

The old TF serialization is a graph with repeated child-frame names under
multiple sources. Treating each stored transform as a graph constraint avoids
silently dropping entries via a child-keyed dictionary. There are no independent
static cycles, so small graph residuals cannot independently validate calibration.

Tested both possible serialized transform directions against camera-info stereo
projection/rectification matrices:

| Serialized transform hypothesis | RGB horizontal baseline error | Thermal horizontal baseline error | Thermal off-axis translation |
|---|---:|---:|---:|
| parent←child | -0.41938 m | +0.33573 m | 0.33506 m |
| child←parent | -0.00000154 m | +0.67011 m | 0.00614 m |

RGB projection baseline is +0.209692 m; thermal is -0.335113 m. Thermal TF
baseline magnitude is about 0.335056 m, but magnitude agreement alone does not
resolve axis/sign conventions or calibration consistency. Rectified rotational
residuals are about 0.198° RGB and 0.617° thermal for these hypotheses.
No convention is selected automatically, and raw K is not substituted for
rectified P. Report: `results/data-access/geometry-audit.json`.

Next obtain the pinned `static_tf_publisher/config/offroad/yamaha.yaml` from
commit `688c34472166f93b5129173030acccc96499b8fc`, through the user's authorized
GitHub session. Reconcile its frame directions and calibration with recorded TF
and processed-image camera-info, then run bounded stereo/reference-pose extraction
and VO. Do not report accuracy or cross-modal covariance calibration yet.

## Reproduction

From `ore-ms/MAC-VO-main` inside `macvo:blackwell`, use the commands in
`Scripts/Setup/README.md` for verifier, manifest, geometry audit, and regression
checks. Required project mount remains `/root/AirStack/ore-ms:/workspace`.
The manifest command may run with host Python because it uses only the standard
library; geometry and checkpoint tools use the project Python environment.
