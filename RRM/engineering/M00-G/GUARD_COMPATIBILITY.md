# Original guard compatibility assessment

The authoritative local source is `robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp`, SHA256 e522a5f6ac10e293e6985b1aaca52d59fad7d80f91a5d5eed0aca0876f785ea2. The frozen F00 source/approval chain is still unauthenticated; unchanged checked-out guard does not establish full frozen equivalence.

`guard_host/src/guard_host.cpp` includes this header by reference. CMake requires the exact include directory/checksum and fails rather than copying or patching it. No build, ROS execution or C++ runtime test occurred. Package/source filenames define the new executable genuinely; its existence in an image/install tree is UNRESOLVED. No fake upstream executable or digest is supplied.

| Behavior | Original source | M00-G handling / limit |
|---|---|---|
| Deadline source | `steady_clock` member `started_ = Clock::now()` before constructor body; parameter actuation_startup_timeout_s defaults10.0, valid finite >0 <=60 | Same original class; explicit10.0 CLI parameter is the source default, not a new F00 threshold. Host allocation before construction differs from production plugin context. |
| Evaluation | 200ms wall timer; try query lock first; config_valid && !attempted; unsafe takes priority, otherwise elapsed>=timeout | No timer/lock changes. Synthetic reference cannot model lock delays, service wait progress, executor scheduling or jitter. |
| Failure latch | attempted set on expiry/unsafe/attempt; failed initializer does not retry. Later connection/readback cannot clear failed initialization | No reset API or recovery transition added. Tests check a synthetic deadline/latch subset and audit forbidden same-instance reinitialization; not C++ behavioral proof. |
| Real safe-state observation | State/extended-state callbacks with actual receipt steady times, connected/disarmed/on-ground <=500ms, unsafe armed/in-air pre-init latch | Original subscriptions preserved; no simulated safety inputs or observer-state injection. Runtime counterpart/plugins unverified. |
| Startup transaction | Typed double1.0 Set and exact typed Get/readback plus safe checks, one attempt; service waits250ms | Original unchanged; this is a parameter write, so host is not an observational recorder. Cannot run under current authorization. |
| Application complete | Not an input to guard | Independent recorder event only; never reinterpreted as vehicle readiness. |
| Vehicle graph | Not an input to guard | Independent producer event; no guard-state modification. |
| Recovery | Failed instance requires restart according to source log; recovery cannot clear its latch | Recovery events only describe evidence handling. A future new instance is a new identified attempt needing separate scope; no automatic restart implemented. |
| Host / executor | Production MAVROSInterface plugin creates guard then mode/arming/takeoff clients and setpoint publishers; robot_interface_node uses4-thread executor | New node hosts only unchanged class with4 threads, namespace /robot_1/interface; removed plugin loading, other callbacks, prior setup, clients/publishers. It is a material configuration/context delta, not integrated-host equivalence. |

Compose references explicit original px4_config.yaml as a read-only file and passes mavros_actuation_config plus default timeout. No external commands are exposed by the wrapper, but original MAVROS plugins may still expose command services; DDS peer configuration is not service-level access control. Independent device/network denial and plugin/API audit remain required. P1 finite observer not implemented; no readiness bypass. All build/image/ROS/YAML/installed-package/namespace/executor/overhead equivalence remains unverified.
