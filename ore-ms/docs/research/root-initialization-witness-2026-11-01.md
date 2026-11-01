# Opt-in graph-root initialization witness

2026-11-01; branch `ore_ms`; implementation base `d029f7235`.

The event recorder can now observe a successful `MACVO.initialize` call and emit
`graph_root_initialized`. This supplies a future observation point for the root
case of the [predecessor contract](predecessor-state-contract-2026-10-31.md).
The eleven missing roots in historical journals remain missing. No existing
selector, default or recorded advice is changed.

## Observation contract

Enable `ArbiterEventRecorder(..., record_root_initialization=True)` before the
first initialization. The default is false. With the option enabled the recorder
wraps the instance's `initialize` method in addition to its existing hooks. The
original method runs exactly once; its return and exceptions are preserved.
After a successful return, the observer requires an observed empty-to-one frame
transition, an exact nonnegative integer sensor timestamp equal to the input
frame's timestamp, and a finite seven-component pose with a unit quaternion
within the existing serialization tolerance. It copies the committed pose without
normalizing or changing graph state. The pose need not be identity.

The reviewed `Odometry/MACVO.py::MACVO.initialize` estimates depth, predicts the
initial pose, pushes one frame with the input sensor timestamp, initializes filter
metadata and sets `prev_keyframe`. The evidence reference
`Odometry/MACVO.py:MACVO.initialize/root-v1` denotes this source contract; the
capture protocol binds the exact source hash. `documented=True` asserts this
observation contract. It does not certify a physically correct pose, neural
training membership, sensor/controller synchronization or an authenticated
source. Use on an unrelated implementation requires reviewing that contract.

The event includes `frame_idx=0`, exact `time_ns`, copied
`pose_camera_map_xyzw`, `documented`, `evidence_ref`,
`initialization_observation=successful_initialize_empty_to_one` and
`physical_motion_admitted=False`, plus the usual modality, worker sequence,
observation clock and stage. It is emitted after the successful initializer
returns, rather than at the internal push instruction. A failure after the push
therefore supplies no successful-initialization witness. Observer errors are
retained by the recorder and do not change estimator control flow. Late
attachment never invents a root from an existing frame.

## Controller receipt binding

The v3 runner accepts `--record-root-initialization` only with
`--record-arbiter-events`. The shared replay accepts the same root flag, forwards
it to both workers and records it in its protocol. The existing streaming path
sends the new event and journals the controller's receipt exactly like other
worker events. Worker observation time is never substituted for controller time.

To use the predecessor validator, explicitly supply the initialization envelope
from the scoped controller prefix as `root_initialization`. Preserve its exact
scope, camera, declared controller clock, timeline ordinal and receipt time. Keep
it in `observed_events` too, so worker sequence IDs remain contiguous. The
validator now accepts an exactly identical envelope bound to that journal
ordinal; a differing envelope at the same ordinal or multiple root records
reject. Separate explicit root envelopes remain supported, with collision checks
unchanged. The validator does not auto-discover or fabricate initialization.

Root documentation does not override an unsupported ordinary predecessor or
conflicting root health/write. The root still must precede the child write and
match its timestamp and captured parent pose exactly. Root acceptance emits no
motion or expert choice; the physical admission flag remains false.

## Validation

The CPU Docker suite passes 199 cases (16 new witness cases plus 183 existing regressions). It covers copied pose/exact nanosecond timestamps, successful
initialization, receipt-prefix binding, missing/conflicting/duplicate/late root
records, late attachment, failed or malformed initialization, observer-clock
failure, method restoration and unchanged advisory actions/availability. Native
`MACVO.initialize` with deterministic depth/motion fixtures preserves every graph
array and initialization call; a nonidentity pose proves the event does not
substitute identity. Existing native CPU optimizer parity and predecessor,
admission, exact-chain, pairing, shared replay and history regressions are
included. Synthetic receipt envelopes exercise the same event streaming API;
no real multiprocess/GPU capture or live latency result is claimed.

An initial synthetic integration test flushed a delayed result outside pair
processing and correctly failed the existing child-stage check. The fixture was
corrected to observe the write during the following pair. No production
criterion was loosened. Test logs and source fingerprints are preserved with the
recovery snapshot. Historical reported scientific tests were not replayed.

## Frozen next capture

[root-initialization-capture-2026-11-01.json](root-initialization-capture-2026-11-01.json)
freezes one already-exposed `EXP090/day-original` 20-row window, both cameras,
and the exact archive/manifest/source bindings. It is a preservation capture,
not a quality comparison. Run fresh flag-off controls and flag-on shared replay
under the same frozen runtime, seeds, inputs and settings. Verify all original
frontend, graph, optimizer and post-refinement arrays bitwise per camera; check
all event payloads excluding only the added root event and rebased sequence/time
metadata. Preserve schedule/deadline policies; controller timestamps and receipt
order must be audited independently for each run. Check original advice and
availability with event identities mapped, reporting any difference rather than
assuming wall-clock timing parity.

Require exactly one documented initialization per camera before its first child
write, exact source timestamp/pose matches, contiguous worker sequences, scoped
controller binding, unchanged protected source hashes, zero observer errors,
and exclusive new output directories. Check missing/late/duplicate/pose/scope
corruptions are rejected. Stop on any mismatch; do not tune, choose a different
window or overwrite history. Pin/check the declared runtime before any inference;
if unavailable, record a separately frozen runtime amendment first. At declaration this protocol
was unexecuted. [EXP092](../../experiments/EXP-092-root-initialization-capture.md)
now records the completed capture under its frozen runtime amendment; no real
historical root is newly proven.

Recovery: `ore_ms:ore-ms/backups/2026-11-01-root-initialization-witness`.
