# Remote inspection lessons — 2026-09-13–14 UTC

Historical environment findings, not a live readiness report. Source baseline:
AirStack `ore_proj` / `a6dad8caf54722e5eba3481367e914ce213e6135`,
RRM transfer `9d26a58eb8516b6754c5d12b041cdc789950e047`.
See [current measured status](end-to-end-status.md), [persistence runbook](model-and-artifact-persistence.md)
and [handoff history](../../HANDOFF.md#remote-airstack--osmo-integration).

## Allocation and launch pitfalls

- The initial transfer workspace deliberately requested `gpu: 0`. Four host-visible
  device nodes without NVML/CUDA userspace libraries did not prove GPU entitlement;
  its nested-container failure was not an OSMO infrastructure defect.
- The later user-started GPU workflow had Isaac, robot desktop and GCS running.
  Host visibility still did not establish the control-plane allocation or usable VRAM.
  The user-supplied pool baseline was three four-GPU machines, not a job allocation.
  The fair-use planning profile was 1 GPU / 12 CPU / 48 GiB RAM / 500 GiB storage;
  confirm accepted limits and actual allocation for each new workflow.
- Historical Isaac inspection found Compose autolaunch without the intended pinning
  flags below. A separately authorized future launch must verify its actual arguments.
  Do not restart a live simulator merely to normalize its configuration.

```text
--/renderer/activeGpu=0
--/renderer/multiGpu/enabled=false
--/physics/cudaDevice=0
```

OSMO overlay files, Docker layers, chat history and gitignored notebooks are not
durable project storage. Commit/push selected source and export checksum-bound
evidence separately. Credentials never belong in either.

## State and transport findings

A stopped Pegasus timeline stopped PX4 while retaining vehicle registrations.
After the user pressed Play, PX4 heartbeat and MAVROS connection returned.
The remaining canonical-state failure was an extra MAVROS namespace:
`/robot_1/interface/mavros/mavros/local_position/odom` had data while the converter
expected `/robot_1/interface/mavros/local_position/odom`. Passing an empty namespace
to the included MAVROS launch repaired this; only robot desktop was recreated.
Canonical odometry and `map -> base_link` TF then streamed, and bounded readiness
checks returned ready. Readiness was not permission to dispatch.

The passive shadow adapter recorded 15 snapshots (14 fresh after one incomplete
startup), with execution inhibited throughout. A later user/GCS takeoff–land session
recorded 160 snapshots (158 ready after two incomplete startup samples), task status
correlation and final connected/disarmed near-ground state. RRM sent no command.
The [dedicated GCS evidence record](evidence/gcs-takeoff-land-2026-09-14.md)
retains capture scope and source revisions. Neither session qualifies RRM autonomy.
