# Implementation and validation — 2026-09-13 UTC

Source baseline: RRM archive `9d26a58eb8516b6754c5d12b041cdc789950e047`, AirStack `ore_proj` at `a6dad8caf54722e5eba3481367e914ce213e6135`. Changes are staged source files, not a new Git revision or published baseline.

## Implemented increment

[`rrm/contracts.py`](../../rrm/contracts.py) adds independent primitives for C02 evidence truth, C03 declared semantic support, C06 immutable context/single-use admission and C08 stop-generation/reset checks. A fresh authority epoch prevents permits from a previous guard instance being reused after reset. The initial guard starts inhibited. These primitives do not change `rrm/loop.py`, `rrm/verbs.py`, the oracle, mock safety or the Isaac stubs.

Trusted supervision must supply validated safety decisions and fresh safe-state evidence. The guard does not authenticate callers or move/stop a robot. Its lock protects only local admission bookkeeping; transport/adapter checks, current-state synchronization, durable deduplication and independent stop delivery remain required. Capability support does not validate physical limits, required operation/resource combinations or permission. The old loop still has the handoff's uncertainty, capability, permission and active-stop gaps.

## Results

Verification ran inside the existing AirStack robot image `v0.20.8_robot-x86-64_dev`, with explicit CPU runtime `runc`, 2 CPU / 2 GiB limits, and a source bind mount. Container-local Pydantic 2.13.5 satisfied the original dependency. No GPU/model package or weight was downloaded.

| Check | Result | Interpretation |
| --- | --- | --- |
| Original oracle suite | 5/5 passed: T1, T2, T6, T8, T9 | Regression baseline only; T6 begins with a human present, not entry during motion |
| Original relation inference | PASS | Geometry helper self-test, not an Isaac integration test |
| New contract unit tests | 14/14 passed | Unknown negation, capability/profile handling, immutable inputs, stale context, expiry/nonallow, duplicate admission, stop/reset, restart epoch |
| Architecture allocation | 11/11 rows linked to owner, contract, scenario and planned evidence | Documentation coverage, not requirement satisfaction |
| GPU execution | NOT REQUESTED | This transfer workspace was deliberately submitted with `gpu: 0`; its NVML failure is not an OSMO infrastructure finding |
| Integrated S01–S10 SIL | NOT RUN | No running Isaac scene or conforming adapter/supervisor |

Reproduce CPU checks inside a container with source mounted at `/workspace/rrm` and Pydantic v2 installed:

```sh
cd /workspace/rrm
python3 scripts/oracle_loop.py --suite --trace-dir /evidence/traces
python3 simulation/isaac_backend.py
python3 -m unittest discover -s tests -v
```

Source/evidence checksums, container identity/digest, dependency versions, timestamps and raw test output accompany the export bundle. Runtime files from the input archive were compared byte-for-byte; only the new contract module was added. The original input ZIP and checksum files remain intact.

## Remaining work

Review the architecture against Phase 1; implement task/context interaction and evidence-backed state; connect capability-aware planning, safety/permission authority, monitoring and replayable telemetry; select and validate the hand scene; restore remote GPU access; run the planned deterministic SIL campaign before comparing model candidates. Model selection and adapter numeric/safe-state limits need measured feasibility. No SCRUM-8 Done transition or external publication was made.

## Continuation — 2026-09-17 UTC

`rrm/task_contracts.py` now adds proposal-only C01/C04/C05 records for immutable task
revisions, scoped clarification/approval interactions, interpreted intents and versioned
plans. Plans require a ready intent with matching task/state/capability references,
authored verb arity, unique action IDs and acyclic in-plan dependencies. The records do
not implement C06 admission, C07 dispatch, ROS transport, a controller command or any
execution authority. Five new unit tests passed, alongside all 24 shadow/contract/unit
tests and the 5/5 Oracle regression suite. The active Isaac process remains the Pegasus
drone baseline; no scene, controller, Foxglove client or RRM execution path was changed.

`rrm/state_contracts.py` now adds C02 records for immutable fact evidence and
task-relevant snapshots. A fact resolves as TRUE or FALSE only from fresh,
non-contradictory explicit evidence. Missing, stale, UNKNOWN, contradictory and negated
UNKNOWN evidence resolve to UNKNOWN; coverage metadata does not make an absent fact
negative evidence. This layer is deliberately separate from the legacy mock
`WorldState`, does not establish a closed-world rule, and creates no adapter or dispatch
surface. Five state-contract tests passed; the total unit suite is now 29/29 and the
Oracle regression remains 5/5.

`rrm/airstack_drone.py` and `scripts/airstack_drone_dispatch.py` now add the RRM
output seam for AirStack's existing public `task_msgs` actions. Typed proposals map only
to `/{robot}/tasks/takeoff`, `/navigate` and `/land`; navigation accepts robot-local
`map` waypoints. Dry-run is the default and does not import ROS. `--execute` is
required to create one corresponding ActionClient and send one goal. The adapter does
not use direct PX4, MAVROS, service, publisher or trajectory-control paths, and it is
not a completed C06/C08 supervisor. Four adapter tests passed; the total unit suite is
now 33/33 and the Oracle regression remains 5/5. No task goal was sent during this work.

## Controlled public-task-action SIL evidence — 2026-09-17 UTC

With explicit user approval, RRM submitted a bounded two-goal sequence to the already
running Isaac/PX4 drone through the existing AirStack task-action interface. The runner
used temporary in-container paths `/tmp/rrm-sim-adapter` and `/tmp/rrm-sim-deps` because
the RRM source is not a robot-container mount; this did not modify the AirStack image,
scene, Foxglove, PX4 or source workspace. Dry-run in that identical environment printed
the exact proposed `TakeoffTask(2.0 m, 1.0 m/s)` and `LandTask(1.0 m/s)` goals with
`execution_requested: false`.

After verifying one live server for each of `/robot_1/tasks/takeoff` and
`/robot_1/tasks/land`, preserving the image's existing ROS `PYTHONPATH`, and importing
both `rclpy` and `task_msgs.action.TakeoffTask`, execution produced these task-server
terminal results in order:

| Goal | Result evidence |
| --- | --- |
| `takeoff-1` — `/robot_1/tasks/takeoff` — 2.0 m at 1.0 m/s | `success: true`, `takeoff complete` |
| `land-1` — `/robot_1/tasks/land` — 1.0 m/s | `success: true`, `landing complete` |

The first, incorrectly staged execution invocation raised `ModuleNotFoundError:
rclpy` before it constructed an ActionClient, so it sent no goal. The corrected
invocations used the adapter's single `ActionClient` task route only; they did not use
direct PX4, MAVROS, ROS publishers, ROS services or trajectory interfaces. This is
transport-level SIL evidence only. It needs externally observed state/pose agreement,
negative-path trials, C06 admission, C08 independent stop proof and repeated trials
before it can support an autonomy or flight-readiness conclusion.

## Read-only post-flight observation and outcome-verifier increment — 2026-09-17 UTC

An eight-second `scripts/airstack_shadow.py` run after the controlled sequence produced
`observation_complete: true` at final snapshot eight with dispatch inhibited. Its fresh
independent state was `map -> base_link`, position `(-0.502, -0.328, 0.020) m`, MAVROS
connected, and `armed: false`. Two snapshots transiently reported incomplete evidence,
so the record retains freshness rather than claiming that every sample was complete.
This supports the final landed-state interpretation but was collected after the task
results and therefore does not prove per-goal causality.

To close that correlation gap for any separately approved future run,
`verify_drone_outcome` now requires a pre-dispatch sample fresh at dispatch time and a
post-result sample fresh at completion time. It is ROS-free and returns `VERIFIED`,
`MISMATCH` or `UNCONFIRMED`; takeoff checks absolute map altitude within 0.3 m and
connected/armed state, landing checks near-ground altitude and connected/disarmed state.
The optional dispatcher verification mode has read-only odometry/state subscriptions,
the pre-existing explicit task-action gate, and optional JSON evidence output. No live
goal was sent to validate this code increment; its focused synthetic outcome tests are
recorded with the general regression results.
