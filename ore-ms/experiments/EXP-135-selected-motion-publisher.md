# EXP135: reference-free selected-motion trajectory publisher

2026-11-04. Parent EXP134 / `7071ee4d2`; publication freeze `c7a6714d1`.
[Full findings](../docs/research/selected-motion-publisher-findings-2026-11-04.md).

Implements exact source-time publication of already selected motion, static frame
conjugation with lever arm, explicit initialization, original receipt/deadline
checks, no fallback and unfilled missing/reset segments. No implicit time model.

Six saved exposed day/night/rain own-clock streams reconstruct20/20 poses each.
Three fixed break controls yield18/20 poses and four segments each; all57 thermal
motions requested on RGB intervals are refused. Official pinned MS² extrinsic
method and independent algebra agree, but no cross-stream acquisition-clock or
physical calibration verification is claimed.

23new+27ORB tests,2505independent checks and24semantic mutations pass; two extra
analytic common-v2-scorer integration tests pass. No new inference/reference
scoring/fitting/reserve access/default change. Original benchmark remains unready.
Next explicit common-time adapter/source evidence, DROIDmetric stereo and complete
recording development before a separately assigned final benchmark.

Recovery: `backups/2026-11-04-selected-motion-publisher-exp135`.
