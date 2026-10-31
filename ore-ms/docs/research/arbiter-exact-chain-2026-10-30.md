# Exact-interval composition for the arbiter layer

2026-10-30 America/New_York; branch `ore_ms`, implementation base `b157bb064`.

`Scripts/Setup/arbiter_motion_chain.py` adds an opt-in composition API after
[admission](arbiter-motion-output-2026-10-30.md). It can combine already-admitted
increments, including from different experts, into a requested larger interval
when their sensor endpoints join exactly. The chronological product is
`D(t0,t2) = D(t0,t1) D(t1,t2)`. Each delta must already use the same target-body
frame convention. This is interval bookkeeping, not a learned or tuned quality
selector and not evidence of improved VO.

## Contract

`compose(entries, target_interval_ns=..., target_frame=..., sensor_clock_id=...,
controller_clock_id=..., deadline_ns=...)` accepts a controller-received set of
admission outputs. Each envelope provides a unique `id`, `received_ns`,
`clock_id`, `output`, `sensor_clock_id` and `sensor_clock_evidence_ref`.
`received_ns` is when the complete admission output became available to this
controller. It must follow the admission's decision time, which must follow the
source-health/motion receipt. Times are compared only in the declared controller
clock domain. Future payloads are not inspected, and later arrivals do not alter
an earlier-deadline query. This is causal-prefix software behavior; it does not
prove measured live latency or sensor/controller synchronization.

For a motion to be eligible, its acquisition-clock ID must match the requested
clock and have a nonempty evidence reference. Matching timestamp numbers alone
cannot establish a shared physical clock. The evidence reference is a caller
attestation of the clock mapping, not an estimated offset, an implementation of
a mapping or independent verification. Existing frame evidence is likewise
inherited from admission and is not certified by composition. Sensor endpoint
integers remain exact above2^53; worker and sensor times are never subtracted
from controller timestamps.

The interval graph uses forward edges wholly inside the target interval. A
unique complete path is required. It returns `COMPOSED` with the chronological
SE(3) product and every contributing entry, camera, sensor interval, controller
receipt and frame/clock evidence reference. A direct admitted increment is a
one-edge path. Missing endpoints, unsupported motions and incomplete coverage
return `ABSTAIN` with null motion. Multiple complete paths also abstain, even
if their matrices agree; choosing among overlapping estimates belongs to the
separate arbiter policy. Duplicate received IDs or impossible receipt ordering
are malformed inputs and raise errors. Excluded evidence has explicit counts.

The query is stateless: it can be repeated, but a downstream integrator must
avoid integrating overlapping query results twice. The API does not fill gaps,
interpolate, extrapolate, estimate constant velocity, invent covariance, stitch
separate trajectory origins or use reference-derived endpoint bridges. It
does not consume previously composed results as if they were original expert
admissions. Path counting saturates at two and uses a forward DAG; it does not
enumerate exponentially many candidate paths.

## Validation and historical boundary

**119 CPU tests pass**, including33 new composition checks and84 existing
admission/pairing/shared-replay/history regressions. Independent geometric checks
use a90-degree turn followed by translation and a three-increment path with
explicit world-pose endpoints; these distinguish multiplication order. Tests
cover mixed cameras, input-order independence, immutable provenance copies,
gaps/overlaps/partial intervals, ambiguous paths, duplicate IDs, clock/frame
mismatches, invalid geometry, exact deadline equality, future-payload exclusion,
and an exponentially branching graph whose path count is capped at two.
Runtime: existing GCS image, Python3.12.3, pytest7.4.4; use `-o addopts=` for
these stdlib tests because the image lacks the unrelated jaxtyping plugin.

The existing EXP073 timeline still matches its independent saved PASS proof.
The unchanged admission boundary reproduces18`unverified_frame_contract` and
1`advisory_abstain`. Querying the19 historical intervals from those admissions
returns19`no_exact_chain` abstentions. No historical frame or shared-acquisition
clock attestation was fabricated. This replay verifies that composition cannot
create poses from missing admitted motion; it is not an empirical tracking,
quality, accuracy, generalization or latency experiment.

The notebook replay harness initially needed its repository PYTHONPATH supplied
explicitly and its recorded base commit supplied without invoking git, which is
absent from the runtime image. Both harness environment issues were corrected;
the final replay and source/proof hash checks pass. No estimator inference,
training, new scientific recording, reserved capture, production default change
or external message is part of this step.

Reproduce from MAC-VO-main in the same CPU container:

```bash
python3 -m pytest -o addopts= -q Scripts/UnitTest/test_arbiter_motion_chain.py Scripts/UnitTest/test_arbiter_motion_output.py Scripts/UnitTest/test_pair_arbiter_events.py Scripts/UnitTest/test_shared_arbiter_replay.py Scripts/UnitTest/test_arbiter_history.py
```

The remaining asynchronous boundary is explicit: if sensors do not share exact
endpoints, actual sensor-causal bridge measurements or a separately declared and
validated motion model are needed. This composer supplies neither. Physical
frame/clock provenance and a valid quality-evaluation target remain pending.

Recovery: `ore_ms:ore-ms/backups/2026-10-30-arbiter-exact-chain`. Includes new
source/tests/docs, test and replay artifacts, four existing EXP073 replay inputs,
and incremental Git history requiring `b157bb064` (prior admission backup).
