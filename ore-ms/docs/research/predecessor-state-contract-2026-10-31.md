# Opt-in predecessor-state evidence validator

2026-10-31 UTC; branch `ore_ms`; implementation base `64a73b3dc32e69672426a84d6d071621ca546424`.

Implemented a separate predecessor-state evidence check with an explicit documented-root case. The frozen replay yields 427 supported-parent evidence passes, eleven missing-root-initialization rejections, one unsupported-parent rejection and 109 slots without a child to check. No existing selector, motion-admission rule or production default changed. No result emits or admits physical motion.

This follows the [predecessor inventory](predecessor-evidence-2026-10-31.md). The purpose is to bind recorded graph state, not infer accuracy from a known error. No labels, fit, new neural inference, recording, reserve or retrospective error comparison was used.

## API and scope

`Scripts/Setup/predecessor_state.py::validate_predecessor` is a pure opt-in function. It receives a scoped camera/controller journal prefix, the chosen child first-write envelope, the original controller deadline and an optional root-initialization envelope. It returns `PASS_PREDECESSOR_EVIDENCE` or `REJECT_PREDECESSOR_EVIDENCE`, a reason and supporting source identities on a pass. It returns no expert action or motion transform. `physical_motion_admitted` is always false.

| Envelope field | Meaning |
|---|---|
| scope_id | Caller-declared capture/graph namespace; separate windows cannot share graph indices accidentally. |
| camera | rgb or thermal; every supplied journal envelope must match. |
| clock_id | Declared controller receipt clock; worker/sensor clocks are not substituted. |
| sequence | Exact integer controller-journal ordinal; increasing in the supplied camera prefix. |
| received_ns | Exact integer nonnegative controller receipt time; nondecreasing and at/before the original deadline. |
| event | Original worker event with contiguous exact worker sequence IDs. |

The chosen child must be bound to the supplied prefix, a unique first write with ordinal 1, supported processing state, increasing graph indices/native endpoints and a matching earlier optimization submission. The captured parent pose must be a finite xyz+xyzw pose with a unit quaternion, allowing only the existing bounded serialization roundoff check. Boolean/floating timestamps and graph identities are rejected where exact integers are required.

## Ordinary predecessor evidence

Parent health must be unique, match the graph parent index and sensor timestamp, describe a valid processing interval, have original need_interp=false, and report no execution error. It must have been received before the child first write. A later parent-health event cannot repair the earlier snapshot.

There must also be a prior parent first-write with its own matching earlier submission. Repeated writes must retain the same job/frame/interval identity, supported processing state and contiguous write ordinals. The latest observed committed parent pose before the child must exactly match the child’s captured parent pose components. No later interpolation, nearest-time match, quaternion-sign substitution or pose correction is performed.

Passing establishes these recorded evidence bindings only. It does not certify optimizer convergence, target-pose correctness, covariance, physical extrinsics or acquisition-clock synchronization. Scope and clock labels are caller attestations; this API does not authenticate a journal’s origin.

## Explicit root initialization

Graph parent zero is a distinct case. Index zero or an identity pose alone never passes. An explicit initialization envelope must match the scope/camera/controller clock and arrive before the child write. Originally it required a separate noncolliding journal ordinal. The [2026-11-01 opt-in witness](root-initialization-witness-2026-11-01.md) additionally permits the identical initialization envelope already bound to the supplied prefix; differing ordinal collisions and multiple root records reject. Its event must contain:

- `kind=graph_root_initialized`, `documented=True` (a real boolean), and a nonempty `evidence_ref`;
- exact `frame_idx=0` and `time_ns` matching the child’s native start;
- a valid `pose_camera_map_xyzw` exactly matching the captured parent pose.

Conflicting ordinary root health or prior root writes reject the root case. A root declaration cannot override unsupported non-root health. `documented=True` and the reference are caller attestations to an initialization contract, not independent physical verification. The function does not read the reference or fabricate a source event.

Root acceptance is exercised only in explicitly synthetic unit fixtures. The eleven real roots have no such initialization record, so the replay supplies none and reports them as unproven. This preserves the distinction between missing initialization evidence and the single observed unsupported recovery.

## Frozen metadata replay

`replay_predecessor_state.py` reconstructs the original six camera journals from EXP089/EXP090, binds the prior independently audited inventory, invokes the new validator on unique exact child first writes, and preserves original advice as a separate unchanged field. It never chooses a replacement expert. All 548 slots remain in the output.

| Recording | Evidence passes | Missing root record | Unsupported parent | No child checked |
|---|---:|---:|---:|---:|
| Day 0 | 194 | 2 | 0 | 2 |
| Day 1 | 102 | 6 | 0 | 6 |
| Night | 131 | 3 | 1 | 101 |
| Total | 427 | 11 | 1 | 109 |

The unsupported-parent case is the already-known `EXP089/middle/96/rgb`. These counts are evidence coverage, not motion/trajectory coverage or a claim of error improvement. The existing [motion-admission contract](arbiter-motion-output-2026-10-30.md) remains authoritative for physical frame/time evidence and exact motion output. Its defaults and all existing advice/history/gate implementations remain unchanged; their source hashes are protected in the replay protocol.

## Validation

169 CPU Docker tests pass: 50 new predecessor-state cases plus 119 existing admission, exact-chain, event-pair, shared-replay and history regressions. Tests cover supported and explicit-root paths, absent/unsupported state, root documentation/scope/clock/pose mismatches, exact large timestamps, cutoffs, future health, child/parent submissions, duplicate first writes, repeated parent writes, pose mutation, input preservation and no function-side file reads. Before the real replay, parent-submission and health-interval bindings were tightened and tested; no criterion changed in response to a replay outcome.

The independent auditor imports neither validator nor replay. It checks the frozen inventory and original source submissions, first writes, parent poses, receipts, exact slot IDs, original advice and every output/count. It passes 11,234 recursive checks and rejects eleven semantic corruptions, including promoted roots/recovery, altered parent/child bindings, scope aliases, changed advice, false physical admission, missing populations and changed counts. Key reordering passes. Status: `PASS_INDEPENDENT_PREDECESSOR_REPLAY`. These are bookkeeping/contract tests, not independent scientific observations.

## Reproduction and next step

From MAC-VO-main in the recorded CPU image with ore-ms at `/root/AirStack/ore-ms`:

```bash
python -m pytest -o addopts= -q Scripts/UnitTest/test_predecessor_state.py
python -m Scripts.Setup.replay_predecessor_state
python -m Scripts.Setup.validate_predecessor_state_replay
```

The replay output directory must contain the preserved pre-replay `declaration.md`; output writes are exclusive. Preserve existing protocol/results/validation files before rerunning. Source metadata must match the frozen hashes. Artifacts are under `results/predecessor-state-validator/run-001/`; the protocol also binds tests, implementation, replay, unchanged production sources and the declaration.

The next useful instrumentation is an opt-in initialization witness recorded at actual graph-root creation, with exact scope/camera/time/pose and controller receipt binding. Existing historical journals must remain unchanged; their missing records cannot be backfilled as observed facts. That instrumentation would need preservation tests and a separately frozen capture protocol before any new inference. It would still not resolve recording-specific physical/reference provenance, checkpoint training membership or measured live timing.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-predecessor-state-validator`, incremental history based on `64a73b3dc`, new source/tests/docs, bounded inputs, replay/audit outputs and lab records. No images, neural checkpoints, credentials or runtime image binary are included.
