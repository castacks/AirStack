# Controller-local fenced input development record

2026-11-17, `ore_proj`, starting at `5f3e8d63f` with source hashes in the
[compact report](../controller-fenced-20261117.json). Actual C++ controller,
isolated DDS194, fixed synthetic odometry. No aircraft endpoint or flight.

The opt-in profile passes all 60 contract checks. It covers startup hold from
measured pose, a latched stop, immutable configuration, explicit reset/mode/path,
wrong process/caller/generation, invalid/stale/colliding sequences, exact last-command
retries, delayed pre-stop reset/mode/replace/append, legacy topic rejection while
latched and after reset, buffer discard, restart identity and malformed trajectories.
Reference freeze and zero feedforward are observed; physical stopping is unverified.
Both package suites pass: message package 2/2 CTest, controller 8/8; aggregate24,
zero failures/errors, two skips. Legacy-profile reference tests still pass.

A failed precursor accepted an almost-zero speed: the test incorrectly expected
duration overflow, while `trajectory_library` explicitly floors timing speed at
0.01m/s. That changed the loaded reference and consumed sequence. The final profile
requires requested speeds at least the existing library floor. Exact-boundary speed
passes; a separate actual coordinate-overflow case rejects. Failed source, harness,
trace and summary are preserved alongside the final outputs.

The default profile remains legacy; no running node enabled fencing or received
test commands. Live installed executable hash matches the earlier baseline.
The isolated overlay build emitted an old/new message-library RPATH ordering warning;
`ldd` under its sourced environment resolved both message libraries to the isolated
install, and actual service/reference tests passed. The installed colcon lacks
`--allow-overriding`; the initial rejected command is retained locally.

This is local process/generation ordering. A caller label is not authentication,
RESET is not a safe-state proof, and durable deduplication, expiry, high-level safety
admission and exclusive downstream actuator authority remain open. Existing task
writers have not been migrated. See the controller README for the exact protocol.

The offline audit starts no ROS node:

```bash
python docs/uav-results/controller-fenced-20261117/audit.py
```
