# EXP141: complete-record bounded DROID engineering

Started 2026-11-04 America/New_York; completed 2026-11-05T06:00:35.485233+00:00. Parent EXP140.
[Findings](../docs/research/bounded-full-record-findings-2026-11-04.md).

The latest declared attempts complete **6/6 streams**, returning **55,618/55,618 finite offline poses**. All six original attempts, two capacity retries and two same-capacity feature-layout corrections are retained. A finite offline pose does not establish tracking correctness or online availability.

All 27,809 original day/night/rain groups / 111,236 raw RGB/thermal PNGs are
acquired and independently checked. Original 2,048-slot attempts complete day/rain;
night capacity aborts remain explicit. One declared 4,096-slot retry per night case
returns all tracking calls but fails the native packed32 backend on unused feature
padding. A separate correction at the same capacity preserves bitwise every active
feature plus 17 reserve rows and removes only unused padding before termination.
Native correlation equivalence is checked at both real feature grids. Thresholds,
weights, source axes, geometry and original core/native source remain unchanged.

139 tests pass. Prefix audit: 18,998 checks / 21 mutations. The six original full attempts pass **1,030,620 checks / 24 metric mutations**. The failed resource retries pass **324,274 checks / 8 mutations**; corrected layout attempts pass **425,612 checks / 8 mutations**. A supplemental audit ties each compacted active count to the final tracking receipt, guard and native estimate.
Source audit: 10,675,581 checks. Isolated cache-omission source replay: 10,666,876.
Raw recovery: 39 member/part-hash-verified USTAR shards. Recovery packet:
`backups/2026-11-04-bounded-full-record-exp141` (see external manifest and RECOVERY).

Returned track calls and finite offline outputs are separate facts. Physical
exposure clocks/calibration, actual trajectory accuracy, causal final-pose timing
and adaptive RGB–thermal benefit remain unverified. No reference scoring, training,
reserves, default adoption or EXP133 benchmark promotion occurs. Next scientific
step requires acquisition/export timing provenance and a separately frozen qualified
evaluation; completed full-record engineering alone does not resolve those gates.
