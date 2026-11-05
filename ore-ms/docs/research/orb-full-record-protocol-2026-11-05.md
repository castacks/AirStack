# EXP143: complete-record ORB-SLAM3 provided-reference comparison

2026-11-05 America/New_York; parent EXP142 / bf0e1493e. Development-only,
pre-inference settings and output-policy freeze. References and DROID scores are
already exposed; no claim of blind or heldout evaluation.

Run all six complete day/night/rain RGB/thermal streams once, sequentially, seed0,
with the exact EXP134 runtime/binary, original 2000-feature configuration and loop
closure disabled. Reuse the byte-exact EXP141 canonical inputs/settings and source
axes. No native estimator changes, resizing, rerectification, threshold changes,
subsampling, model fitting, reference-directed retry or reserve access. Each case
has a 3600-second native subprocess timeout. Run the remaining cases after any
failure. Preserve all native receipts, errors, missing/unattempted source slots
and original publisher output. A Python publication/parse failure produces a
separate all-null evaluation envelope, not salvaged unfinished poses or a rerun.

The old ORB publisher splits output segments at >150ms timestamp gaps, whereas
DROID EXP142 retains a single map segment across such gaps. Preserve that legacy
estimate unchanged. The new evaluation view changes segment identifiers only:
missing output and native map identity changes break segments; a timestamp-only
gap does not imply a native map reset. Poses and original clocks remain identical.
This matches DROID's map-continuity semantics for trajectory ATE. The unchanged v2
scorer still excludes RPE across >150ms source gaps and reports longest gap-free
source runs. Native map transitions, missing-output splits, legacy segments and
evaluation segments are reported separately. Never rank short separately aligned
fragments as complete-record wins.

Use the same full same-stem, own-camera provided odometry and rigidity/exclusion
rules as EXP142. No camera mixing, clock/scale/extrinsic fitting or interpolation.
Report SE3-only and origin ATE, approximately-one-second translation/rotation RPE,
source/reference/pose/RPE coverage, segments, terminal/native statuses and errors.
All six fixed DROID results remain context, unchanged. Physical exposure timing,
calibration, causal live availability and adaptive superiority stay unverified;
EXP133 gates and final reserves remain unchanged.

Freeze settings, copied TSV/plan bytes, native binary/library/vocabulary, compile
flags, upstream patch, runtime, new study/publisher/auditor/tests, existing adapter,
scorer, original input plans, EXP142 references and DROID results before the first
native invocation. The native executable validates immutable input-image hashes
through the original adapter before each run. Use cv2 native thread count1 as in
the existing adapter, with OMP/OPENBLAS/MKL thread environment variables1. Native
SLAM background scheduling is not claimed bitwise deterministic. Record the exact
container command and environment.

Report wrapper wall duration and native per-frame processing/receipt times
separately. Peak child-process RSS is a CPU-process measure, not total system
memory or GPU allocation. ORB uses CPU received poses; DROID uses GPU offline
backend/filler output. These are engineering costs with different boundaries,
not an equal-hardware real-time comparison. The same canonical image bytes may
undergo each method's declared internal processing, as already frozen.

Validate gap-only continuity, map/missing splits, exact source identity, native
pose inversion, failure envelopes and legacy preservation on analytic fixtures
before inference. Independently reconstruct all native states, input/calibration
bindings, segment breaks and raw-reference metrics. Reject source/time/map/pose,
gap erasure, coverage and scope corruptions. Lock all six predictions before any
new ORB reference score is computed. Keep failures and provenance in recovery.
