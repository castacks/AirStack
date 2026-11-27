# Static Isaac R4 native physics witness live qualification

27 November 2026 UTC. One isolated diagnostic from `aeb0ef8dc74f250424d36dedaa33f91f9c7e47ae` on `ore-ms-orb-development`. [Offline source investigation](STATIC_ISAAC_PHYSICS_WITNESS_R4.md)

## Preflight and protocol

The launcher confirmed the cached Isaac image digest and image ID, the original USD fixture, the unchanged clock probe and recorder, and the staged witness source. It inspected the stopped diagnostic container and verified exactly two approved scratch mounts, no network attachment, private IPC, and nonprivileged execution on the selected GPU. The host attach timeout was 150 seconds with a 15-second kill grace. All 84 existing CPU regressions and four new witness tests passed before execution.

The witness holds the pinned `subscribe_physics_on_step_events(..., pre_step=False, order=0)` handle from before `World.reset()`. Every callback writes a sequence number, step delta and monotonic receipt time. Each control brackets its operation with the native callback count, World index, Simulation Manager count and time, Kit timeline, and clock-graph and ROS observations. The controls use the existing 0.1-second physics and render configuration.

1. **A. Poll only.** Poll the existing ROS subscription 15 times with a 20 ms per-poll cap and make no Kit updates.
2. **B. Explicit physics step.** Call `World.step(render=False)` once. Require exactly one positive native post-step delta and reconciled native, World and available manager counts. Poll again to check that ROS polling adds no events before the render control.
3. **C. Original capture.** Run the original synchronous Replicator render once with `rt_subframes=1`, `pause_timeline=True`, `delta_time=.1` and `wait_for_render=True`. The unmodified R4 clock probe applies its existing single-bracket checks, retains duplicates, and bounds subscriber wait at three seconds after render return. This diagnostic uses one bracket. The separate two-epoch qualification still requires two distinct advancing epochs.

The only code addition was the separate witness worker and its deterministic tests. ROS domain 91 used Jazzy Fast DDS with localhost discovery and the original graph on `/clock`. No geometry observations, depth/instance-ID capture or TF checks were scheduled. The frozen `2,000,000,000 ns` geometry requirement remains intact. [Diagnostic source](static-isaac-geometry-r4/physics_witness_r4.py) · [CPU tests](static-isaac-geometry-r4/test_physics_witness.py)

## Result

**NO_PHYSICS_ADVANCEMENT.** The native post-step witness qualified against both controls. The original first Replicator capture returned with zero new physics events.

| Control | Native events | World index | Manager count | Result |
| --- | --- | --- | --- | --- |
| A · poll only | 0 | 3 → 3 | 3 → 3 | **PASS** |
| B · explicit physics-only step | 1, sequence 4, delta `0.10000000149011612 s` | 3 → 4 | 3 → 4 | **PASS** |
| C · original first Replicator capture | 0 | 4 → 4 | 4 → 4 | **NO_PHYSICS_ADVANCEMENT** |

The journal contains three startup callbacks and one callback within B's recorded bracket. The independent review matched event times and sequence numbers to the boundary counts and found no new events during polling. B advanced native monotonic simulation time from `0.30000000447034836 s` to `0.4000000059604645 s` with no graph evaluations or ROS messages. During C, both native clock readbacks stayed at `0.4000000059604645 s`. The Kit timeline time stayed at `−0.09999999701976778 s` as playback paused.

C evaluated Tick, Time and Publish 3,579 times each and delivered two callbacks. Both ROS messages encoded the native timestamp that already existed **before** the render, `400000005 ns`. The graph port's transition from zero was its first observed read of that time. One compatible publisher was discovered and duplicate callbacks were retained. The unchanged probe recorded `CLOCK_ASSOCIATION_UNVERIFIED` and accepted zero epochs. Native events, both counters and direct native-time reads agree for this first capture after the recorded setup and manual control. The run supplies no evidence about subsequent Replicator captures or other configurations. [Machine-readable gates and provenance](static-isaac-geometry-r4/physics-witness-live-verification.json)

## Shutdown, isolation and evidence

The isolated container exited cleanly. The lifecycle journal records the shutdown request, Kit reports `Simulation App Shutting Down`, and host Docker inspection confirms exit code zero without an OOM or restart. The journal records no Python return from native close. The completed diagnostic classified the render outcome before shutdown. The three operational snapshots matched before and after in container IDs, image IDs, start times, restart counts and running status. Frozen and historical hashes verified.

Raw evidence and the independent review are retained at `/root/ore-ms-artifacts/static-isaac-physics-live-r4.yvYhONYk/`. The 44-entry `SHA256SUMS.txt` verifies the staged inputs, tests, event log, boundary and gate records, wire callbacks, lifecycle and Docker records, and independent review. Key files are `run/capture/physics-post-steps.jsonl`, `run/capture/physics-controls.jsonl`, `run/capture/clock-raw.jsonl`, `run/capture/clock-diagnostic.jsonl`, `run/capture/phases.jsonl`, `run/child.log` and `run/container-exit.json`. The manifest digest is pinned in the version-controlled result. [Evidence paths and hashes](static-isaac-geometry-r4/physics-witness-live-verification.json)

## Smallest subsequent action

Design a separate clock-only bracket that explicitly steps physics, then uses a capture with zero requested timeline delta to evaluate and transport the clock. For each of two brackets, require one native post-step event at the explicit step, stable counts through render and ROS drain, and a distinct wire epoch that exactly encodes the matching native time. The required two-epoch association remains unqualified. The explicit-step/render bracket has yet to be qualified and the geometry recorder stays outside this work.
