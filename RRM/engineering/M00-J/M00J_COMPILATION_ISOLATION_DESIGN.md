# Proposed compilation-only isolation

**DESIGN ONLY — NOT EXECUTED — AUTHORIZATION PACKAGE BLOCKED.** `M00J_COMPILATION_ONLY_POLICY.json` is a reviewable intent record, not a Compose/OCI deployment manifest or launcher. It intentionally leaves approved current image, daemon-host identity and permission fields null. Effective containment remains **NOT VERIFIED**.

Use a fresh, separately authorized compilation-only container from the verified existing image. Do not reuse/enter the historically privileged robot container, inherit robot_base or invoke AirStack/Compose profiles. The stock profile's GPU, SSH/display/shell/workspace integration is unnecessary for a compiler. No simulator/PX4/MAVROS/ROS node or startup script may run, including the linked guard binary (whose main would start ROS).

| Control | Required design |
|---|---|
| Image/runtime | Exact approved local image config ID, pull never; explicit **runc** or separately reviewed non-GPU runtime. Local daemon.json declares default-runtime=nvidia, so omitting GPU requests alone is insufficient. runc availability is UNVERIFIED |
| User/root | Proposed numeric10001:10001, no added groups; verify mapped UID can read inputs/write private output without broadening protected input modes. Read-only container root |
| Mounts | Only I guard_host directory read-only, original guard header file read-only, and designated output directory read-write; private propagation, no nested submounts. No repository-wide, /dev, /sys, host /proc, socket, display, SSH, home or credential binds |
| Output | Proposed host `/root/AirStack/RRM/build-output/M00-native-guard-next`, container `/output`; native build `/output/native`, HOME=/output/home, TMPDIR=/output/tmp. Not created in J. All generated artifacts/scratch remain under this single output bind |
| Network/namespaces | network none, no ports/DNS/extra hosts; private PID default, IPC/UTS/cgroup namespaces; no host/container namespace joins. Network isolation not a device-isolation substitute |
| Privilege/devices | privileged=false; drop ALL capabilities, add none; no devices/device requests/cgroup device grants/GPU/runtime hooks. Standard minimal runtime pseudodevices only; verify no physical FC/serial/USB/CAN/network-device access |
| Security | no-new-privileges, default confining seccomp, host-supported AppArmor/equivalent; no unconfined profile. Effective enforcement and user namespace support require controller evidence |
| Startup | Override both image entrypoint and command; disable image healthcheck. Reject inherited/anonymous Config.Volumes. Clean env via env -i, bash --noprofile --norc; no setup files, bws/sws/.bashrc, tmux or autolaunch |
| Limits/lifecycle | Proposed2CPU,4GiB memory/no added swap,256PIDs,600s wall ceiling,2GiB output quota; no restart, no TTY/stdin. Proposed ceilings, not measured demand or granted quota. Host controller must prove enforcement/stop ability |

Entrypoint is explicit `/usr/bin/env -i` with only the underlay PATH, Python3.12/ROS library paths, locale and output HOME/TMPDIR recorded in policy. No source shell is evaluated. Non-root compiler/tool/header access and these image paths are not execution-verified. CMake configuration may run ordinary compiler detection probes; this is compilation behavior, not permission to execute the built guard, colcon tests, ROS nodes or arbitrary hooks.

Input build files are reviewed source without download steps; CMake uses only the existing Jazzy underlay. Missing generated interfaces/typesupport are a stop, never a reason to fetch or synthesize dependencies. Block before run if image/runtime configuration would add an entrypoint hook, device, mount, network, writable path or privilege beyond policy. Any deviation returns to PD review; no fallback to privileged/NVIDIA/default robot execution. All14 original isolation-gate rows remain unpassed; this narrower design closes no runtime gate.
