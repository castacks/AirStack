# Static Isaac clock delivery live diagnostic R4

27 November 2026 UTC. **FAIL — INVALID_CLOCK_DATA.** The first render step delivered two identical Isaac `/clock` timestamps, both `300000004 ns`. The probe requires one advancing callback per step, so it stopped before step two. This diagnostic verified graph execution, publisher discovery, QoS compatibility and subscriber delivery in the isolated run. The original geometry retry's missing-sample cause remains unresolved. [Machine-readable gate results](static-isaac-geometry-r4/clock-live-verification.json)

## Execution and preflight

The staged sources matched `482a2db25ef8030b9196e19224ddf93c49834ce2` on `ore-ms-orb-development`. All 76 CPU tests passed. The launcher verified the pinned Isaac Sim image and USD stage, then checked the stopped container for the approved two scratch mounts, private IPC, no network and disabled privileged mode. It issued one start under a 150-second attach deadline. Docker measured about 118.20 seconds from start to exit. No camera arrays, target translations, geometry controls, attenuation, or forecasting were run. [Qualified clock-only procedure](STATIC_ISAAC_CLOCK_DIAGNOSTIC_R4.md)

## Observed clock path

| Gate | Independent finding |
| --- | --- |
| Graph and node execution | `/World/ReceiptClock` existed and was enabled. The Tick, Time and Publish compute counts each rose from 0 to 6,413 across the first render. These are observed node invocations. |
| Simulator clock | The graph clock output and publisher timestamp input moved from 0 to `0.30000000447034836 s`. The sampled application timeline remained at `-0.09999999701976778 s`. Only one step returned, so no consecutive-step progression was established. |
| Domain and topic | The subscriber recorded rclpy domain 91, Jazzy Fast DDS and the resolved absolute topic `/clock`. The graph used context 0, an empty namespace and topic basename `clock`. |
| Endpoint discovery and QoS | No publisher was visible before render. After render, the subscriber discovered `/_World_ReceiptClock_Publish` offering `rosgraph_msgs/msg/Clock`. Both endpoints were reliable and volatile and the Jazzy QoS compatibility result was `OK`. |
| Subscriber delivery | Two executor polls yielded two callbacks. Both raw messages have `sec=0` and `nanosec=300000004`. |
| Timestamp validation | The first callback supplied a valid individual timestamp. The second repeated it. Both the duplicate stamp and the two-callback count violate the per-step gate. No `clock-check.json` was written. |

The duplicate is the first supported failure in this diagnostic. The evidence does not identify how many DDS samples the publisher emitted or which scheduler events produced the duplicate delivery. The original R4 retry brought up the bridge after the depth controls, while this clock-only run initialized it before its first rendered step. The successful delivery here cannot retrospectively explain the earlier timeout. A next diagnostic should correlate publication timestamps and executor delivery with individual simulation steps and retain the one-callback and exact nanosecond requirements. [Original R4 retry](STATIC_ISAAC_GEOMETRY_RETRY_R4.md)

## Shutdown and retained evidence

The lifecycle journal records the clock validation failure before the durable shutdown request. Kit logged fast shutdown and Docker reported a clean exit with no restart, Docker error or OOM kill. Python shutdown return was unobserved. Exit `0` provides teardown evidence, and the clock verdict remains **FAIL**. No R3 completion behavior was changed.

All three operational AirStack container snapshots matched on container/image IDs, start times, restart counts and status. The diagnostic container was removed after its final state was recorded. Prior R1–R4 evidence and frozen M05–M08 checksums matched before and after.

The raw evidence directory is `/root/ore-ms-artifacts/static-isaac-clock-live-r4.MNZQantf`. It contains the runtime configuration, graph/endpoint journal, two raw ROS messages, worker lifecycle and Kit logs, the host Docker exit record and the independent review script and results. All 44 retained hashes passed `SHA256SUMS.txt`, whose digest is `a086605c77957e4469b4f9fa3da89fda97dbb7275504cd9dfa9ccf7a1204273b`. No retry, DDS routing change, geometry capture, M12 or M13 work followed.
