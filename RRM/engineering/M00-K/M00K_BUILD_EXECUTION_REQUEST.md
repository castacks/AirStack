# M00-K proposed compilation-only request

**BLOCKED — NOT AUTHORIZED — DO NOT EXECUTE.** Single recommended route: **Path 2 — Authorized container build**, conditional on current image/host verification. This does not assert K-B or grant daemon access. Reuse J's compilation-only policy and resource/stop/evidence conditions; no stock Compose profile, current robot container exec, installation or image construction is requested.

Candidate image is the declared development tag in K findings; exact historical config ID is recorded there. **Approved current local image ID, approved daemon-host identity, access reference and effective runtime/security/output permissions are UNKNOWN.** The literal marker below must remain unresolved until verified existing metadata and a separate PD decision supply them. No tag, guessed digest or privileged fallback may replace it.

Proposed invocation shape (documentary text; not a launcher):

```text
docker --host unix:///var/run/docker.sock run \
  --name rrm-native-guard-build-next --pull=never --runtime=runc \
  --user=10001:10001 --read-only --cap-drop=ALL \
  --security-opt=no-new-privileges:true \
  --network=none --ipc=private --cgroupns=private \
  --cpus=2 --memory=4g --memory-swap=4g --pids-limit=256 \
  --restart=no --stop-timeout=10 --no-healthcheck \
  --mount type=bind,src=/root/AirStack/RRM/engineering/M00-I/guard_host,dst=/inputs/guard_host,readonly,bind-propagation=rprivate \
  --mount type=bind,src=/root/AirStack/robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp,dst=/inputs/upstream/include/mavros_interface/px4_actuation_guard.hpp,readonly,bind-propagation=rprivate \
  --mount type=bind,src=/root/AirStack/RRM/build-output/M00-native-guard-next,dst=/output,bind-propagation=rprivate \
  --workdir=/output --entrypoint=/usr/bin/env \
  UNVERIFIED_CURRENT_LOCAL_IMAGE_ID \
  -i PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  HOME=/output/home TMPDIR=/output/tmp LANG=C.UTF-8 ROS_DISTRO=jazzy \
  AMENT_PREFIX_PATH=/opt/ros/jazzy CMAKE_PREFIX_PATH=/opt/ros/jazzy \
  LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib \
  PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages \
  PYTHONDONTWRITEBYTECODE=1 /bin/bash --noprofile --norc -c \
  'mkdir -p /output/tmp /output/home && cmake -S /inputs/guard_host -B /output/native -DRRM_UPSTREAM_INCLUDE_DIR=/inputs/upstream/include -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cmake --build /output/native --target rrm_startup_guard --parallel 2'
```

The endpoint is an observed candidate only; its use and host path namespace require approval. No privileged flag, extra capabilities, devices/GPU, published ports, host network/PID/IPC sharing, stdin/TTY, daemon bind or application entrypoint is permitted. PID and UTS use private defaults. Verify effective confining seccomp/LSM, private namespaces, runc/no injected GPU hooks, no image VOLUME/anonymous mounts or nested bind submounts, and exactly three binds before approving. Overriding image ENTRYPOINT/CMD and disabling healthchecks is mandatory. No bash setup/.bashrc, bws/sws, ROS nodes, MAVROS/PX4/Isaac or linked-guard execution. CMake compiler probes are the only expected generated-code execution.

The2CPU/4GiB/256PID limits are proposed ceilings, not measured demand or entitlement. A host controller must additionally enforce a **600s wall ceiling,2GiB private output quota and target-only stop within the10s grace**; the displayed Docker command alone does not enforce wall time or bind-directory quota. Approve no run without that supervision. Prepare output only in a separately authorized successor scope, non-symlink/private and writable by mapped10001:10001; no permission change outside RRM. Sources/header remain read-only and hash-pinned as in I/J. No output directory was created in K.

Capture current image ID/platform/startup metadata, host/access/runtime/security/mount/resource identity, genuine compiler/CMake/package/generated-header/typesupport versions and hashes, exact command, input hashes, UTC/steady start/end, exit status, separate configure/build logs, CMakeCache/compile_commands and linked binary SHA256 if produced. Do not overwrite I/J/K sealed evidence. Missing tools, configuration/build failure, input drift, install/download attempt, disallowed process/device/network behavior, resource ceiling or unavailable stop control ends the attempt without retry/fallback.

Cleanup, separately authorized: preserve logs/failed outputs and named-container metadata first; stop only `rrm-native-guard-build-next` if needed, then remove only that disposable container after sealing evidence. No automatic --rm, image deletion/prune, unrelated stop or workspace cleanup. Preserve output for review. Successful native compilation/linking would not verify runtime containment or authorize the guard binary.
