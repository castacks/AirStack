# EXP134: pinned ORB stereo/MS² development adapter

2026-11-04. Parent EXP133. Initial freeze `eda9ac5c8`; final compatibility
freeze `3d29197e9`. [Full findings](../docs/research/orb-ms2-development-findings-2026-11-04.md).

Six seed-0 exposed-development runs (day/night/rain × RGB/thermal) each complete
20 original source frames with 20 tracking-OK outputs in one segment. All
27 analytic adapter tests and 2,628 independent input/output checks pass.
Original PNGs, integer stereo clocks, calibrated metre baseline and failures remain
explicit; no reference payload is used by the new adapter/auditor or scored.

Build work required a rectified-settings diagnostic guard and consistent C++11/
native-SIMD ABI across ORB, g2o and wrapper. Three separately frozen failed native
attempts remain retained; no estimator parameter or input tuning occurred.

The output is received camera-local motion, with thermal common-RGB frame proof
still pending. No full-recording benchmark, pose-accuracy or real-time claim.
EXP133 remains execution-not-ready with reserves closed. Next: selected-motion
publisher, common-frame/time proofs, DROID metric stereo and full-recording smoke.

Exact runtime `sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445`.
Final manifest `orb-ms2-development-2026-11-04-v4.json`.
Recovery `backups/2026-11-04-orb-ms2-development-exp134`.
