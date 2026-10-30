# Reference, frame and recording evaluation readiness

2026-10-30, America/New_York. Branch `ore_ms`, base `112586732`.
After the negative [EXP087 flow-closure probe](../../experiments/EXP-087-flow-cycle-quality.md),
the existing data support reproducible offline proxy comparisons and availability
analysis. They do not yet support a prospective, physically validated pose-quality
gate. Do not fit another selector to these three repeatedly inspected windows and
describe its result as held-out improvement.

## Existing population and timing

The audit checks original arms from EXP084–086, their evaluation/protocol/proof
hash chains, manifest timestamps, five reference-file hashes per recording and
motion availability. These are 434 camera slots over 217 paired transitions,
from **two recordings**, all acquired on October27,2025. Early and middle night
are nonoverlapping windows of `night_mac_salon_warehousefig8`; day is
`0_salon_so_day`. Window separation within a recording is insufficient to make
independent recording-level train/test partitions. Both recordings have already
informed extensive exploratory decisions. Neither is a prospective holdout.

| Window | Paired slots | RGB / thermal motions | Both / both processing-stage | RGB minus thermal endpoint offset, min / median / max (ms) |
|---|---:|---:|---:|---|
| Early night |19|19 /19|19 /18|8.014917 /8.032561 /8.046151|
| Middle night |99|2 /99|2 /1|6.508827 /6.831884 /6.901980|
| Day |99|99 /99|99 /98|−23.000002 /2.741933 /28.414965|

Offsets use unique frame endpoints (20/100/100), not double-counted transition
endpoints. None of the217 RGB and thermal intervals has exactly identical
endpoints. Day RGB intervals span99.634–200.001ms and thermal100.000–150.000ms;
assuming a fixed100ms interval would change the target. Offsets are measured
timestamp differences, not established sensor latency or clock corrections.
Processing-stage counts exclude terminal writes but do not prove common-clock
availability. Missing motion remains missing, including97 middle-night RGB slots.

## Frame and reference contract

`Scripts/Setup/arbiter_pose_proxy.py` conjugates camera motion with a nominal
body-from-camera transform, `D_body = E D_camera E^-1`. Its `extrinsic` helper
combines a static vehicle-to-optical chain, the camera-info rectification rotation
and the native optical/NED basis conversion. The saved matrices are constant
within each window and identical across these windows for each camera. This
is internal consistency, not verification of physical mounting, optical identity
or the applicable calibration revision. Existing protocol fields explicitly
retain `physical_body_transform_verified=false`.

For each expert's acquisition interval `[a,b]`, the evaluator transports its
body motion onto RGB's interval `[r0,r1]` using recorded reference poses:

`D_common = R(r0)^-1 R(a) D_body R(b)^-1 R(r1)`.

The two bridges depend on the reference. They are permitted for explicitly
qualified offline labels and cannot supply an online selector with deployable
motion synchronization. A live system must specify its own frame/interval policy,
missing-result behavior and deadline, then measure its approximation and latency.
Do not infer physical equivalence from matched row numbers or this offline bridge.

`td3_reference.py::load_reference` reads interpolated Super Odometry poses with
`sensor_init -> vehicle` metadata. It linearly interpolates positions and uses
quaternion SLERP, prohibiting extrapolation. This remains recorded SLAM rather
than independent physical truth. No new interpolation or error computation is
introduced by this audit. The millimetre-scale oracle headroom in the
[selection analysis](solver-decision-and-selection-headroom-2026-10-30.md) is not
evidence that the reference resolves that physical gain.

The [attributed owner replies](source-replies-2026-10-22.md) support MSA calibration
applicability and identify the auxiliary odometry as direct GNSS-system output.
They also tentatively report that SO does not use the NovAtel odometry. Retain
that progress: GNSS origin is no longer wholly unknown. Exact axis permutation,
orientation, receiver origin/lever arm, timestamp assignment, quality/RTK membership
and possible shared inputs remain unresolved. Axis-invariant antenna displacement
checks are useful context; they do not replace camera/body pose labels. The
[earlier source audit](filtered-odometry-provenance-2026-10-21.md) records the
workspace/driver leads. No coordinate swap, time offset, scale, lever arm or
extrinsic is fitted to the evaluated VO errors.

## Evaluation decision and smallest next inputs

| Claim or task | Readiness |
|---|---|
| Reproduce known-window proxy losses and availability | Ready under recorded assumptions; use exact matched IDs and separate missing coverage. |
| Deploy a body-motion selector over common intervals | Pending physical rig transform and a sensor-causal asynchronous policy with measured timing/deadlines. |
| Train a physical pose-quality gate | Pending applicable independent reference, origin/axes/timing and interval quality evidence. |
| Claim recording-level generalization | Pending recording-level split frozen before access and checkpoint training membership. Current two recordings are development evidence. |
| Use CATS as temporal pose validation | Not applicable: inspected data are static stereo; signed-reference conventions also remain unresolved. |

The next useful work is to assemble a source-backed rig/reference contract from
the actual recording configuration, and a prospective recording-level protocol.
The most useful missing evidence is: (1) applicable camera/body and GNSS-origin
transforms with frame definitions; (2) exact GNSS axes and timestamp behavior plus
recording-specific covariance/fix/RTK information, if saved; (3) MACT_Offroad
training/finetuning recording list and preprocessing recipe. Existing owner-reply
drafts identify the likely contacts; none has been sent. A bounded connected-mail
search for the checkpoint, frame and night-recording identifiers returned no
matches during this audit; this does not establish their absence elsewhere.

Before opening any new evaluation recording, freeze whole-recording development,
validation and test membership, checkpoint hashes and known/unknown training
overlap; define the primary translation target, separate rotation outcome,
availability population, common-interval policy and fixed-camera/fallback
baselines. Keep missing results and recording-level outcomes explicit. Do not
randomly split adjacent pairs or treat the two night windows as independent runs.
Any pilot learned from belongs to development. Unknown checkpoint membership
must remain explicit even for a newly accessed recording. This document neither
selects nor opens reserved MS² captures or additional CATS scenes.

## Verification and preservation

`Scripts/Setup/evaluation_readiness.py` is a new stdlib-only inventory command;
it leaves existing frozen experiment sources unchanged. Final artifacts are
`results/evaluation-readiness/run-002/{inventory,repeat}.json`. Two CPU Docker
runs are byte-identical, verify22 distinct input hashes before/after and check
all217 paired slots against manifest endpoints. This is an inventory/contract
audit, not an independent revalidation of all numerical experiment arithmetic.
EXP086's two near-cancellation gradient-component exceptions and its documented
post-inference contribution-scale/whole-gradient validation refinement are
retained verbatim in the inventory; they are not erased by a PASS here.

Reproduce from `MAC-VO-main` in a CPU container with ore-ms at `/workspace`:

```bash
python -m Scripts.Setup.evaluation_readiness --root /workspace --output /workspace/results/evaluation-readiness/new-run/inventory.json
```

Run001 is retained as preliminary evidence before adding offset-sign and
long-interval counts; its initial GPU-image CPU execution emitted driver-init
diagnostics without affecting the stdlib audit. Final run002 uses the CPU image.
No neural inference, fitting, new dataset access, production setting change or
external message occurred. Recovery target:
`ore_ms:ore-ms/backups/2026-10-30-evaluation-readiness`, with prior EXP084–087
and underlying dataset/model backups remaining reconstruction prerequisites.
