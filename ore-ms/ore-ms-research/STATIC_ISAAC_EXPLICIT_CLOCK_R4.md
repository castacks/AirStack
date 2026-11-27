# Static Isaac R4 explicit-step clock qualification

27 November 2026 UTC. One isolated clock-only execution from `5352a812c8eaf52fdd3624010e28313851fda802` on `ore-ms-orb-development`. [Prior native PhysX witness qualification](STATIC_ISAAC_PHYSICS_WITNESS_LIVE_R4.md)

## Protocol and preflight

The diagnostic retains the native post-step subscription, existing R4 graph and raw ROS callback recorder. It records five boundaries for each bracket. It resumes a paused timeline through play and commit without pumping Kit, and checks that the operation leaves physics counters and native times unchanged. It then calls `World.step(render=False)` exactly once. Acceptance requires one valid native event, a matching World index increase, agreement with the Simulation Manager count when present, and strictly increasing manager and monotonic simulation times.

The subsequent synchronous Replicator call uses `delta_time=0.0`, `rt_subframes=1`, `pause_timeline=True` and `wait_for_render=True`. The diagnostic verifies unchanged physics counts and native timestamps across render and ROS polling. It requires fresh Tick, Time and Publish computes with graph time and publisher input exactly matching the post-step native monotonic clock. Polling is bounded at three seconds per render return. Every delivered callback is retained. The new epoch must equal the pinned bridge's truncation of `native_seconds × 10⁹`. Repeated copies of that epoch and queued copies of the prior accepted epoch are allowed in nondecreasing order. Backward or unexpected stamps, missing or incompatible publishers, and work outside the witnessed step prevent acceptance. The run stops at the first failed gate.

All 88 existing and five new CPU tests passed, and the analytic geometry-to-M08 qualification passed separately. Preflight verified the archived fixture, isolated source inputs, historical and frozen evidence and pinned cached Isaac image. The stopped container had exactly two approved scratch mounts, no network attachment, private IPC and no privileged access. The authorized run used domain 91 and localhost discovery on the existing `/clock` graph. Host attach was bounded at 180 seconds with a 15-second kill grace. The clock-only worker and tests are separate from the existing clock probe and geometry recorder. [Worker](static-isaac-geometry-r4/explicit_clock_r4.py) · [CPU tests](static-isaac-geometry-r4/test_explicit_clock.py)

## Live result

**PASS. Both independently witnessed physics steps produced distinct, correctly encoded ROS clock epochs.** Two zero-delta renders returned. Neither the renders nor the subscriber polling added a physics event or changed either sampled native clock. The Kit playhead stayed at `−0.09999999701976778 s`. Bracket two's play/commit restored playback without advancing physics or either native clock.

| Bracket | Native event and counter change | Post-step native monotonic time | Tick, Time and Publish computes, each | Delivered callbacks and accepted epoch |
| --- | --- | ---: | ---: | --- |
| 1 | Sequence 4. Native, World and Manager all 3 → 4. | `0.4000000059604645 s` | 0 → 4,837 | 10 deliveries at `400000005 ns`. Accepted once. |
| 2 | Sequence 5. Native, World and Manager all 4 → 5. | `0.5000000074505806 s` | 4,837 → 4,839 | 2 deliveries at `500000007 ns`. Accepted once. |

Both recorded step deltas were `0.10000000149011612 s`. The step sequences were independently matched to their operation brackets by host-monotonic receipt time and event count. The bridge inputs matched the corresponding direct native timestamps. Each bracket had a unique compatible Clock publisher, and the independent review recomputed the encoded nanoseconds from the retained native values and compared them with all 12 raw messages. No unexpected callbacks or Replicator scheduling errors appeared in the retained window. The first accepted epoch came from the explicit step, consistent with the prior finding that the original first Replicator capture alone did not advance physics. [Gate and provenance JSON](static-isaac-geometry-r4/explicit-clock-live-verification.json)

## Shutdown, isolation and evidence

The journal records the shutdown request and the worker receipt. Kit logged `Simulation App Shutting Down`, and host Docker inspection confirmed exit code zero, zero restarts and no OOM. Python did not record a return from native close. All three operational snapshots matched before and after, including IDs, start times and restart counts. The worker persisted its result before shutdown, and the independent audit checked it after exit. All retained and frozen package hashes checked before and after execution matched.

The 66-entry manifest is retained at `/root/ore-ms-artifacts/static-isaac-explicit-clock-r4.Al3TbO9c/SHA256SUMS.txt` with its digest pinned in the gate JSON. Important evidence in `run/capture/` includes `physics-post-steps.jsonl`, `explicit-brackets.jsonl`, `explicit-gates.jsonl`, `clock-raw.jsonl`, `clock-diagnostic.jsonl`, `clock-configuration.json` and `phases.jsonl`. The Kit log and Docker exit inspection are under `run/`. The independent review script and JSON, operational snapshots, tested inputs and CPU logs are alongside the manifest.

## Next action

Prepare an offline geometry integration design that binds the witnessed native clock to the rendered depth/instance-ID reference and exact-time TF. The two accepted clock epochs are `100000002 ns` apart, so a nominal step count does not establish exact geometry timing. Require the measured endpoint stamps to differ by exactly `2,000,000,000 ns` with no rounding or tolerance, and retain all frozen receipt checks. This run did not modify or execute the geometry recorder, or qualify any frame-to-clock or TF association.
