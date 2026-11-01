# EXP092: opt-in root initialization preservation capture

2026-11-01 America/New_York; implementation `5c7dd244c`; runtime amendment
committed as `dbec3cf6d` before inference. Both fixed 20-frame shared runs complete
and preserve all 1,066 original numerical arrays. The witness run adds one
source/receipt-bound root per camera. All original advice remains unchanged.
This is initialization bookkeeping and preservation evidence.

## Frozen scope and recovery

Executed the [declared protocol](../docs/research/root-initialization-capture-2026-11-01.json)
with its [pre-inference runtime amendment](../docs/research/root-initialization-runtime-amendment-2026-11-01.json).
Only the exact already-exposed EXP090 `day-original` prepared window from
`20251027/1_salon_mac_cov_day` is used: start0,20rows, both cameras, seed0.
Neither recording/window nor a numerical threshold was selected after outcomes.

EXP090 archive SHA256
`c3420b785f41e044f66548b119e34df9e0545ea19f76e045b696c2c1abb18706`
and all1,016 archive-member hashes verify. Restored the85files of the single
bounded prepared dataset; other restored files are prior plan/runtime/audit
metadata. No historical source or journal was overwritten. The original model
ZIP SHA/CRC and both pinned checkpoint hashes pass; credentials and redirected
URLs were not included in outputs or forwarded to download hosts.

The historical required image was absent. Rebuilt the recorded
`Docker/Dockerfile.solver-vo` and froze replacement image
`sha256:8f7bf1c66f251755fe796a0ebd281b2131d870bc1f936c5585dcc28e08569a33`.
Torch2.7.1+cu128, PyPose0.9.5, NumPy2.2.6, SciPy1.17.1, OpenCV4.10.0;
RTXPRO5000Blackwell. Full package freeze/build logs retained. Imports,80input
image decodes, original sensor timestamps, schedule and CUDA matmul pass;
199 recorder/predecessor/admission/chain/pairing/replay/history tests pass in the
new GPU runtime. No numerical equivalence with the historical image is claimed.

Preflight initially stopped on effective-config hashes because the checkpoint
path was `/root/AirStack/ore-ms/...` rather than the original `/workspace/...`.
The original `/workspace` bind mount resolves that difference; both effective
config hashes then exactly match EXP090. No model inference preceded the
successful preflight and frozen amendment. Shared-link metadata initially
omitted folder entries; explicit folder-item listing resolved the documented
checkpoint source. These recovery/preflight attempts are preserved.

Before either inference run, capture-plan.json freezes580source/config/dependency
file hashes,85input files,2checkpoints, the amendment,6recovery/driver/audit scripts,
image, original rows and2exact commands. Its SHA256 is
`646afddaef7814448c003c3a4b3211081d5fb56a2ab35f3b44ac10f0d93c07d9`.
Original declaration and sources remain unchanged. Driver-init result11 startup
diagnostics are retained; the actual imports, CUDA checks and both runs succeed.

## Same-runtime preservation

Control: unchanged shared replay with root logging off. Witness: the identical
shared replay with only `--record-root-initialization` enabled. Each completes
20frames per camera on its first scientific execution attempt. The original
sensor schedule, group lag and final decision before drain are unchanged.

| Camera | Bitwise-equal arrays / NPZ files | Equal legacy event payloads | Added root events |
|---|---:|---:|---:|
| RGB | 533 /23 | 97 | 1 |
| Thermal | 533 /23 | 97 | 1 |

Every saved NPZ member matches in key, shape, dtype and exact bytes, including
first-depth, frontend uncertainty, graph/map, tracking flags and final trajectory.
All report fields except run-start and event-summary metadata match, and the
effective configs match exactly. Legacy event equality omits only top-level
worker sequence and observation-monotonic timestamps; the root shifts all
following worker sequence IDs by one. Every other health, motion, pose, stage,
submission and failure field is compared. Zero observer errors occur.

The existing independent schedule/prefix auditor imports no recorder/scheduler/
policy code. Both runs pass40scheduled camera frames,19decisions and798decision
field checks. Streamed prefixes exactly match saved expert journals. All19
cross-run decisions preserve action, availability, reason and deadline boundary;
health/motion evidence matches after mapping sequence IDs. Both runs have18RGB
choices and1final pre-drain abstention. Receipt times are audited independently,
not assumed identical. These are offline serial replay observations, not live
latency measurements.

## New root evidence

Each witness root is observed on the successful empty-to-one initialization
call, before its first child write, and streams through the controller journal.
The exact integer source timestamp and copied camera-map pose match the
first-write parent. Explicitly passing each scoped initialization envelope from
the original first decision prefix produces2`PASS_PREDECESSOR_EVIDENCE` results,
both `documented_root_initialization`. No root is auto-discovered by the
validator. Both results retain `physical_motion_admitted=False`.

A separate auditor imports neither recorder nor predecessor validator. It binds
requests back to the original controller timeline and reconstructs root/child
identities, receipt ordering, source timestamp/pose, documentation reference,
result source fields and source/input/checkpoint hashes. It rejects12directed
root corruptions across both cameras (missing, late, duplicate, pose, scope,
time). The production validator separately rejects16mutated requests including
clock and ordinal collisions. These are bookkeeping checks, not independent
scientific observations. Final status: `PASS_INDEPENDENT_ROOT_CAPTURE`.

The eleven missing historical initialization records remain missing. The new
two records belong only to this new capture and cannot backfill EXP089/EXP090.
No quality scores, fit, reserve, calibration, physical motion, recording-specific
reference verification, checkpoint-membership conclusion or production-default
change follows from these passes.

## Reproduction and next step

Artifacts are under `results/root-initialization-capture/run-001/`, with the
frozen plan/declaration, execution record, both full shared runs and worker logs,
original journals, preservation proofs, independent timeline validations,
explicit root requests, production replay/mutations and final validation.
Recovery includes the standalone frozen driver/preflight/replay/audit scripts
and local test/build/recovery logs; no checkpoint/runtime binaries are bundled.
Use exclusive new outputs if repeating capture; preserve the saved run.

The useful next bounded step is to apply the separate predecessor evidence
validator across all original decision prefixes of this new capture, reporting
complete candidate/root/ordinary/missing populations while preserving existing
advice. It does not authorize rejection policies, quality fitting or physical
motion admission. Owner rig/reference and checkpoint provenance remain separate.

Recovery: `ore_ms:ore-ms/backups/2026-11-01-exp-092-root-initialization-capture`.
