# Proposed native build authorization package

**BLOCKED — NOT APPROVED OR EXECUTED.** This is a bounded future compilation proposal; no request to start the stock AirStack profile. Review policy JSON and input checksums in the J source manifest. M00-I source/interface findings remain authoritative within their documented limits.

| Approval parameter | Concrete proposal / missing essential evidence |
|---|---|
| Image | Documented `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev`; historical config ID `sha256:3b98f4333f33dbbe734363f19f9ea65f52f4af8a98f4ac3e78f724124af9c290` is the exact candidate to revalidate. Approved current ID=null; robot registry digest unknown. Never run by mutable tag or pull automatically |
| Host | Observed workspace node `4585761b76004bc7-696082aefb054d52`; actual daemon execution host/controller, bind namespace visibility and permission reference UNKNOWN; approved host=null |
| Inputs | Unchanged I `guard_host/{CMakeLists.txt,package.xml,src/guard_host.cpp}`; original guard header SHA256 e522a5f6ac10e293e6985b1aaca52d59fad7d80f91a5d5eed0aca0876f785ea2; genuine existing image Jazzy/toolchain/message/YAML underlay with exact manifest. No robot workspace overlay or interface_bringup runtime resource needed for compilation |
| Mounts | I guard_host→/inputs/guard_host RO; original header→/inputs/upstream/include/mavros_interface/px4_actuation_guard.hpp RO; private new RRM/build-output/M00-native-guard-next→/output RW. Three binds only, no recursive submounts; host visibility/ownership not verified |
| Network/devices | network none; no published ports, physical-device binds/rules/requests or GPU. No Docker socket inside container |
| Privilege/security | UID:GID10001:10001, read-only root, cap_drop ALL, no-new-privileges, confining seccomp/LSM and private namespaces; explicit non-GPU runc runtime availability UNVERIFIED; no privileged mode, inherited startup/volumes/healthcheck or extra mounts |
| Resources | Proposed2CPU,4GiB memory with total memory+swap4GiB,256PIDs,600s wall maximum,2GiB output quota,10s stop grace. Demand/performance unknown; entitlement and quota/enforcement must be supplied before approval |
| Output | `/root/AirStack/RRM/build-output/M00-native-guard-next/{native,tmp,home}`; directory not created in J. Supervisor logs/receipts belong in a separately authorized successor evidence directory under RRM; no I/J sealed report overwrite |

Expected **in-container compilation command**, for a future approval only, with the clean entrypoint/environment in policy JSON:

```sh
mkdir -p /output/tmp /output/home && cmake -S /inputs/guard_host -B /output/native -DRRM_UPSTREAM_INCLUDE_DIR=/inputs/upstream/include -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cmake --build /output/native --target rrm_startup_guard --parallel 2
```

No container launcher is supplied or executed. Target is the named executable only; no install/test/workspace/application target. CMake's normal compiler probes may execute; the built `rrm_startup_guard` must never be invoked. Native compile/link success alone establishes no runtime readiness.

**Entry/stop conditions:** obtain current image config/platform/full selected startup metadata, genuine nine-prerequisite package/version/header/library inventory, approved host/controller/resource/stop scope, effective non-GPU security and path permissions. Resolve missing fields with exact evidence, not defaults. Before starting, stop if any essential field remains unknown or effective mounts/runtime/entrypoint differ. Once separately authorized, stop on first configure/build failure, dependency/network/install attempt, original/input hash change, disallowed process/device/network behavior, output/CPU/memory/PID/wall ceiling, inability to supervise/stop, or request for elevated privilege. Preserve failed evidence; no auto-retry/fallback/provisioning.

**Capture:** exact image ID/config metadata and platform; host/runtime/security/user/mount/resource identity and authorization; package versions/generated headers/typesupport hashes; input hashes; explicit command/env; UTC+steady start/end and exit status; separate configure/compile/link logs and CMakeCache/compile_commands; binary size/SHA256 if linked; quota/stop/failure records. Check host input/history hashes before/after. No ROS runtime check.

**Cleanup:** supervisor must stop only the named future build container if needed, retain its selected metadata/logs and all failed output, then remove only that disposable container after evidence is sealed and reviewed. Preserve/hashes output; no Docker prune/image deletion, unrelated stop, workspace cleanup or protected write. Private scratch may be removed only after evidence retention is confirmed under successor scope. Stop/cleanup permissions are essential proposal fields, not granted in J.

**Exact next evidence action:** the controller of the already exposed local runtime supplies the selected existing metadata/package export above, confirming whether the historical ID remains cached, and confirms scoped host/mount/resource/stop permission. Alternatively, PD may separately authorize narrowly bounded read-only inspection of that specific privileged local endpoint. No remote daemon, registry/pull or organizational contact campaign is proposed. Until supplied, native-build execution approval is not decision-ready; this package is an explicit blocked decision record, as §8 permits.
