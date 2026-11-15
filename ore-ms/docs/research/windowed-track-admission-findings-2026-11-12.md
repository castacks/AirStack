# Five-frame geometry pilot: saved tracks not admissible

12 November 2026, America/New_York. Branch `ore-ms-orb-development`; Git baseline
`4f28219d9` plus SHA-pinned admission source. Protocol frozen before graph counts.

**Decision: STOP_SAVED_TRACK_CONFIGURATION_NO_SOLVER_RUN.** All120 camera-windows
fail the declared multi-view-track requirement. The88,680 serialized landmark
IDs each connect exactly one two-frame match. None connects three distinct
frames. No window optimizer, new frontend inference, training or reference-pose
scoring was run. This is an input-admission result, not a failed geometry method
or an accuracy result.

## Fixed scope and observed support

The [proposed pilot](windowed-joint-geometry-next-2026-11-12.md) specified20
nonoverlapping five-group windows from the existing exposed first100 groups in
day/night/rain. Each camera-window must contain at least6 explicit landmark
tracks spanning3 or more distinct frames before the matched RGB/joint,
frozen/variable-depth comparison is admitted. Six is a declared minimum count,
not a rank or observability guarantee; here even one qualifying track is absent.
No count threshold adjustment could create a multi-frame identity in this packet.

| Recording | Camera | Match / landmark count | Matches per landmark | Frames per landmark | Admitted windows |
|---|---|---:|---:|---:|---:|
| day | rgb | 19,176 | 1 | 2 | 0/20 |
| day | thermal | 10,885 | 1 | 2 | 0/20 |
| night | rgb | 19,126 | 1 | 2 | 0/20 |
| night | thermal | 12,977 | 1 | 2 | 0/20 |
| rain | rgb | 19,336 | 1 | 2 | 0/20 |
| rain | thermal | 7,180 | 1 | 2 | 0/20 |

All88,680 match rows have valid point/frame indices and reciprocal serialized
`match2point` / `point2match` incidence. Each point has degree1, exactly two
distinct endpoint frames, and no repeated point/frame observation. All120
window-specific counts are retained, including empty/missing-support windows.
The proposed80 within-window motion intervals per recording remain unscored;
all19 between-window slots per recording remain outside the pilot. No coverage
or pose-error metric is inferred from these graph counts.

## Why these are not multi-view tracks

The current inspected producer in `MAC-VO-main/Odometry/MACVO.py` allocates a new
point batch in `graph.points.push` for each new pair, then associates that batch
with the just-created matches. `VisualMap.py` serializes point/edge identities;
`Graph.py` defines the reciprocal adjacency and its-1 padding. Storage capacity
for5 observations does not establish that a point was observed5 times.

This source inspection explains the observed structure; it does not newly prove
that the current source revision exactly matches every capture's runtime. The
decisive evidence is the hash-verified saved graph itself: all points are linked
to one match only. Pairwise target UV in one edge must not be declared identical
to a source UV on the next edge without an explicit identity association.
Nearby pixels, similar depths, stored world positions and common frame numbers
are not substituted for that missing association. The camera-local single-edge
point IDs also cannot be equated across RGB and thermal streams.

## Verification and disposition

The read-only admission script validates six original tensor-map hashes,
monotone integer frame times, field sizes/finite endpoint measurements,
point/frame bounds, reciprocal degrees, valid adjacency entries and padding.
It derives track frame sets from the forward mapping and records every window.
Original source/input hashes remain unchanged after execution.

An independent auditor imports neither producer nor admission code. It starts
from the reverse point-to-match adjacency, reconstructs all88,680 point frame
sets and120 window memberships, checks reciprocal identities and recomputes the
stopping decision. All178,138 assertions pass. Both tools complete on their
first execution; no tolerance, ID association, selection or support amendment.
No new unit tests duplicate these read-only checks.

Close this **saved-track configuration**. Do not implement a pair-edge-chain
optimizer and label it five-frame multi-view bundle adjustment, or launch a
matcher/data search automatically to rescue the gate. The broader shared-window
hypothesis remains untested. Pursuing it later would require a separately
specified observation-producing run that explicitly retains stable camera-local
landmark identities and their UV, stereo, covariance and timestamp observations
across at least3 frames. That run has not been selected, frozen or executed here.
Nominal same-ID timing and reference/provenance qualifications remain unchanged.

Protocol: `experiments/windowed-track-admission-freeze-2026-11-12.json`.
Read-only tools: `tools/check_windowed_track_admission.py` and
`tools/audit_windowed_track_admission.py`.
Local artifacts: `results/windowed-track-admission/run-001/report.json` and
`independent-audit.json`. The outcome receipt pins both reports and tool source.
This check supplies no new paper result or author-system reproduction.
