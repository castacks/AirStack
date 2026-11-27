# Static Isaac clock delivery diagnostic R4

27 November 2026 UTC. **PASS offline.** The retained live retry failed at the subscriber wait. The clock-path cause remains undetermined. The clock-only probe is ready for a separately authorized isolated run. Isaac was not launched during this work.

## Retained failure boundary

| Clock-path stage | What the retained R4 evidence supports |
| --- | --- |
| Action graph creation | The exporter requested an execution graph with `OnPlaybackTick → ROS2PublishClock` and `IsaacReadSimulationTime → timeStamp`. Creation and subscriber setup returned during `ros_clock_setup`. No graph introspection was recorded. |
| Graph activation and evaluation | Unverified. The log contains a timeline deprecation warning. It supplies no node evaluation record. |
| Render stepping | The epoch's blocking Replicator step returned. The next call reached the subscriber wait. The epoch had no recorded simulator time from the native clock node. Prior control timestamps came from writer references. |
| Bridge initialization | The bridge logged that it loaded rclpy, and the setup phase completed. Publisher creation and publish calls remain unverified. |
| Domain, namespace and topic | The launcher assigned ROS domain 91, Jazzy Fast DDS and local discovery. The subscriber requested absolute `/clock`. The publisher used its default topic and global context. Effective runtime endpoints were never recorded. |
| Discovery and QoS | Unverified. The worker did not query the graph for publisher endpoints or QoS. |
| Executor and reception | The timeout arose inside the loop that spins the local node. No callback wrote `clock-raw.jsonl`. Node spin completion counts and DDS activity were unobserved. |
| Timestamp validation | Unreached. No received stamp could establish an epoch. The exact two-second gate had no input. |

The earliest supported failure is the three-second subscriber wait at `clock_epoch`, not a particular clock graph or DDS component. The camera and 5 m and 15 m controls passed; the previous live verdict remains unchanged. [Retained retry report and evidence paths](STATIC_ISAAC_GEOMETRY_RETRY_R4.md) · [Retry gates](static-isaac-geometry-r4/retry-verification.json)

The historical isolated graph relies on the publisher’s context-input default of zero. NVIDIA defines it as the global context and documents the default basename `clock` and empty namespace. NVIDIA’s clock graph recipe and AirStack’s Pegasus graph wire a `ROS2Context` node explicitly. Pegasus reads the configured domain from its graph variable. The isolated launcher’s `ROS_DOMAIN_ID=91` establishes intent for rclpy and the bridge, but the retained evidence does not establish the domain of the bridge's resolved global context. The subscriber passed depth 10 to Jazzy rclpy; the probe uses the equivalent explicit `QoSProfile(depth=10)` and records discovered QoS when available. The launch script requested no additional remap. None of these configuration observations establish a domain or QoS mismatch. [NVIDIA clock recipe](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html) · [NVIDIA clock publisher inputs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishClock.html) · [Pegasus clock graph](../../simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/ogn/api/spawn_multirotor.py) · [Jazzy rclpy node and QoS](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/node.py) · [Jazzy compatibility API](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/qos.py)

## Bounded instrumentation

The exporter uses the same graph, bridge loading, render step, subscriber QoS and three-second wait. The probe retains `clock-configuration.json` with the resolved subscriber topic, node namespace, requested ROS environment and observed rclpy domain where available. The synced `clock-diagnostic.jsonl` records graph validity and enabled state, Tick/Time/Publish compute counts, graph time and publisher inputs, timeline time, publisher identities and QoS compatibility, callback counts, executor polls and exact phase outcomes. It samples before stepping, after render returns and at the delivery outcome. Each raw message keeps its original integer seconds and nanoseconds in `clock-raw.jsonl`. A started record without an outcome identifies the last entered phase if the process is terminated abruptly. [Probe source](static-isaac-geometry-r4/clock_probe_r4.py) · [OmniGraph graph introspection](https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Graph.html) · [Node compute counters](https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Node.html)

| Outcome | Evidence required |
| --- | --- |
| `NO_GRAPH_EXECUTION` | The graph is absent or disabled, or all three observed node counters stayed unchanged across the bracketed step. |
| `NO_PUBLISHER_DISCOVERY` | Publisher-node computation advanced and endpoint snapshots showed no publisher. This describes the observed window, with transient discovery between snapshots unresolved. |
| `INCOMPATIBLE_QOS` | Publisher-node computation advanced and every discovered Clock offer tested incompatible. |
| `NO_SUBSCRIBER_DELIVERY` | Publisher-node computation advanced, a compatible Clock endpoint was visible at the deadline, and executor polls returned without a callback. A compute count proves invocation, while the delivery boundary still includes native publish and DDS transport. |
| `INVALID_CLOCK_DATA` | The callback received a malformed or nonadvancing stamp or the step received multiple callbacks. |
| `CLOCK_PATH_UNVERIFIED` | The data do not resolve the boundary, for example unavailable instrumentation, counter rollover, ambiguous QoS or an endpoint that disappeared. |

The geometry exporter still requires one clock callback per step and exactly the frozen nanosecond cadence, with exact-time TF and the original receipt verifier. No controls, camera setup, native association, recorder, shutdown contract or operational DDS configuration were changed.

## Smallest future live check

The isolation procedure below still applies. The clock-only entry point now uses the witnessed epoch gate described in the [clock semantics qualification](STATIC_ISAAC_CLOCK_SEMANTICS_R4.md). The original clock-only live result and geometry gate remain unchanged.

Run only after separate authorization. Use the same pinned image and stationary USD, unchanged isolated ROS environment, and two blocking Replicator steps with a 0.1 s increment. The clock-only entry point initializes a render product on the existing camera and records no camera arrays. It performs no plane translations, TF or two-second observation sequence. A live transport pass requires two valid advancing Clock callbacks, an advancing native graph time and a witnessed compatible publisher. `clock-check.json` is written only after those checks. Require a clean container exit and the retained shutdown journal and Kit log before treating the process as complete.

Use the retained R4 retry launcher as the isolation reference. Stage only `stage.usda`, `clock_probe_r4.py`, `recorder.py`, `validate_mounts.py`, and a copy of `input/run-r4.sh` with its final Python argument changed to `/capture-input/clock_probe_r4.py`. Record the Git commit, pinned image config and input hashes. Create one fresh container with `--pull never --network none --ipc private`, GPU 0, and the original headless graphics and consent settings. Bind scratch input read-only at `/capture-input` and a fresh evidence subdirectory writable at `/capture-output`. Verify the stopped image, container name, isolation settings and structured mounts with `validate_mounts.py --exact` before start. Run the staged Bash entry point with a 150-second host deadline for `docker start --attach`. Save stdout/stderr, the Docker exit record before isolated cleanup, and before/after operational IDs, restart counts and start times. Check and hash all retained evidence. Preserve the first failure and stop. If a later full geometry run is separately authorized, include `clock_probe_r4.py` in its pinned inputs. [Isolated R4 launcher and evidence bundle](STATIC_ISAAC_GEOMETRY_RETRY_R4.md)

## Offline results

The 13 new clock tests cover graph appearance and execution, publisher absence, incompatible QoS, missing delivery, valid advancing samples, repeated or malformed stamps, timeout, interruption and durable records. Original R4 camera, R3/R2 recorder, and frozen receipt regressions passed. The archived exporter matches its executed bytes at `2020db31c`. The prior R1–R4 manifests and frozen M05–M08 package and plan hashes verified. No live execution counters, DDS endpoints or clock timestamps were generated in this task. [Clock tests](static-isaac-geometry-r4/test_clock_probe.py) · [Machine-readable offline results](static-isaac-geometry-r4/clock-qualification.json)
