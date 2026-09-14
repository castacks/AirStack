# GCS takeoff/land shadow evidence — 2026-09-14 UTC

## Scope and authority boundary

This run was performed in the live AirStack Isaac/PX4 SIL workflow after canonical
MAVROS state and TF readiness had passed. The user, through Foxglove Desktop's AirStack
Robot Tasks panel, initiated one takeoff and one land. The RRM shadow observer only
subscribed to state and task-status topics; it had no action client, publisher, service
client, trajectory, or PX4 command path. `execution_dispatch_enabled` remained false.

Implementation under test: `7af80e52` (`feat(rrm): add read-only AirStack shadow adapter`).
Documentation state at capture: `d4d1c33b`.

## Inputs and observed task sequence

| Source | Value |
|---|---|
| Robot | `robot_1` |
| Odometry | `/robot_1/interface/mavros/local_position/odom` |
| MAVROS state | `/robot_1/interface/mavros/state` |
| Transform | `/tf`, `map -> base_link` |
| Task status topics | takeoff, navigate, land ROS action status topics |
| Takeoff status | `EXECUTING` (2) → `SUCCEEDED` (4) |
| Land status | `EXECUTING` (2) → `SUCCEEDED` (4) |

## Evidence result

| Metric | Result |
|---|---|
| Snapshots | 160 |
| Observation-ready snapshots | 158 |
| Explicit startup-incomplete snapshots | 2 |
| Snapshot execution-inhibited | 160 / 160 |
| Final replay verdict | observation complete; execution dispatch disabled |
| Final MAVROS state | connected, disarmed |
| Final odometry | `map -> base_link`; approximately `z=-0.013 m`; near-zero velocity |

The task actions succeeded at the AirStack action boundary and the final state was
consistent with landing. This is evidence of RRM *shadow correlation* only; it does
not authorize RRM task dispatch, safety admission, navigation, recovery, learned
reasoning, or policy execution.

## Raw-bundle manifest

The full local evidence bundle is deliberately gitignored under the AirStack feature
notebook and must be exported separately before terminating an OSMO workflow. Its
contents and SHA-256 digests at capture were:

| File | Bytes | SHA-256 |
|---|---:|---|
| `events.jsonl` | 2,986,990 | `401beadec69f1b42c9bf43f44b19691375893b583298ab05828cf8d69b0fda09` |
| `manifest.json` | 593 | `23e1a25bfd05a96b5c623c7123db40ba7c7cb8a0c6e30dedd3490cd212e0163b` |
| `replay-report.json` | 211 | `1205768dcbf81904cb9095fed7b173480d9a075b4e7c452e765c1678914bf55e` |

The committed document preserves the result and provenance. Preserve the raw bundle
outside the ephemeral pod if later replay at message granularity is required.
