# Remote inspection — 2026-09-13 UTC

## Current live workflow correction

The observations below originally describe a separate, deliberately CPU-only
transfer workspace (`gpu: 0`). They must not be used to diagnose the later live
GPU workflow. The user has since started the normal AirStack OSMO workflow at the
fair-use profile (1 GPU, 12 CPU, 48 GiB RAM), connected through
`./airstack.sh osmo ide`, and has a running Isaac livestream, robot desktop and
GCS stack. This checkout observed those three containers running; the exact OSMO
allocation remains an OSMO control-plane fact rather than something inferred from
host-visible devices.

The desired manual Pegasus PX4 launch inside the Isaac container uses livestream and
these runtime arguments:

```text
--/renderer/activeGpu=0
--/renderer/multiGpu/enabled=false
--/physics/cudaDevice=0
```

The flags are required because this nested runtime can expose all four physical
GPUs to Isaac despite a one-GPU workflow request. They concentrate the renderer
and physics workload on GPU 0. However, read-only inspection on 2026-09-14 found
the currently running Isaac Python process was Compose auto-launched with
`AUTOLAUNCH=true` and only `--/app/livestream/enabled=true`; it did not carry the
three pinning arguments. A controlled future launch must use the pinned command.
Do not alter Compose or restart the live simulator merely to change configuration.
The user uses the patched Mac-side OSMO WebRTC forwarder and connects the streaming
client to `127.0.0.1`.

Neither OSMO workspace files nor Codex chat history are persistent across a new
workflow. The durable project record is the committed, pushed repository; export
runtime evidence separately, and do not rely on gitignored `notebook/` content.

### Read-only ROS 2 task-interface inventory (2026-09-13 UTC)

From `airstack-robot-desktop-1`, a read-only `ros2 action list -t` confirmed these
task endpoints: `/robot_1/tasks/takeoff` (`TakeoffTask`),
`/robot_1/tasks/navigate` (`NavigateTask`), and `/robot_1/tasks/land` (`LandTask`),
along with exploration, fixed-trajectory, and semantic-search actions. One server
was present for each candidate first-RRM action: takeoff/land were served by
`/robot_1/takeoff_landing_planner/takeoff_landing_task`, and navigate by
`/robot_1/droan/disparity_expander_node`. No goal, cancellation, service, trajectory,
or PX4 command was sent.

The graph also advertised `/robot_1/odometry_conversion/odometry`
(`nav_msgs/msg/Odometry`), but a bounded eight-second `ros2 topic echo --once`
produced no sample. The later live follow-up below establishes the cause. The first
integration remains a model-free, read-only shadow adapter; it must mark state
availability unknown until the canonical stream is repaired and a fresh observation
has been confirmed.

### Live state-stream diagnosis and lifecycle update (2026-09-14 UTC)

The first observation occurred while the Pegasus timeline was stopped. Its local
`OgnPegasusMultirotorNodeBase` stop callback explicitly stops each PX4 backend while
retaining the Isaac process and vehicle registrations. After the user pressed **Play**
in the streaming client, read-only checks confirmed a new PX4 process, a MAVROS state
with `connected: true`, and Isaac's `Received first heartbeat` / `Ready for takeoff!`
log messages. Isaac Sim therefore needs to remain open and playing; it is not the
remaining cause of the robot stack appearing hung.

The remaining blocker is a confirmed ROS namespace error in the live interface launch.
MAVROS publishes valid odometry at
`/robot_1/interface/mavros/mavros/local_position/odom`, while the canonical
`odometry_conversion` node and `robot_interface` subscribe to
`/robot_1/interface/mavros/local_position/odom`. The expected topic has zero
publishers. This leaves canonical odometry and TF absent, makes the trajectory
controller wait for odometry, and also disconnects the interface's MAVROS state,
command, and service paths.

Local launch source explains the extra level: `interface.launch.py` pushes the
`interface` namespace and includes `mavros_px4.launch.xml`, whose default
`namespace="mavros"` names the MAVROS node under `/interface/mavros`. MAVROS itself
uses relative topic names beginning `mavros/`, producing `/interface/mavros/mavros/*`.
The intended canonical topology is MAVROS under `/interface` (node name `mavros`) so
those topics resolve to `/interface/mavros/*`. The prospective fix is to pass an empty
MAVROS launch namespace from `interface.launch.py` (or equivalently correct the
include configuration), then recreate the affected robot stack. It has not been
applied: a restart/reconfiguration or any task command requires user direction.

This is a live SIL-readiness blocker for any RRM world-state or command integration,
not a reason to weaken RRM's uncertainty semantics. Until the correction is deployed,
the shadow adapter must not attach to the nested topic as a permanent workaround.

## Observed state

## Pool baseline and allocation distinction

The AirStack OSMO pool baseline supplied by the user is three machines, each with four RTX PRO 5000 Blackwell GPUs, 48 CPU cores, and 216 GB RAM: **12 GPUs, 144 CPU cores, and 648 GB RAM total**. The fair-use target is **12 CPU cores and 48 GB RAM per GPU**; the suggested maximum is four GPUs per person/project. LOW-priority work may consume spare capacity and can be preempted. The live workspace's 48 CPUs, roughly 216 GiB RAM and four detected RTX PRO 5000 Blackwell GPUs corroborate the per-machine portion of that baseline.

This distinguishes pool capacity and policy from a particular job's allocation. A one-GPU RRM SIL job should initially request 12 CPU cores, 48 GiB RAM and 500 GiB storage, subject to the active pool's accepted limits. A four-GPU request maps to 48 CPU cores and 192 GiB RAM at the same target ratio. The active workflow ID, assigned GPU count/VRAM, QoS/priority, and persistent-volume behavior still must be queried from OSMO; they cannot be inferred from the node inventory.

| Item | Observation |
| --- | --- |
| AirStack path/branch | `/root/AirStack`, `ore_proj` |
| HEAD | `a6dad8caf54722e5eba3481367e914ce213e6135`, version 0.20.8 docs hotfix |
| Existing edits | No tracked modifications; `RRM-transfer/` untracked; no preexisting notebook entries found |
| Remote refs | Read-only `git ls-remote origin` confirms main at the same hash; no remote `ore_proj` ref returned |
| Transfer | SHA-256 verified `rrm-9d26a58-with-handoff.zip`; extracted 64 entries to `RRM-transfer/rrm-9d26a58`; archive contains no `.git` |
| Docker before work | Default context, zero containers; robot 0.20.8/cache image and GCS 0.20.8 image available; no Isaac image listed |
| Docker storage | `overlay2`, data root `/osmo/run/docker` |
| Host inventory | Docker reports 48 CPUs and 232195321856 bytes RAM (~216 GiB); root overlay ~4.6 TiB, ~3.9 TiB free at inspection |
| GPU inventory | `/proc/driver/nvidia/gpus` lists four NVIDIA RTX PRO 5000 Blackwell devices; kernel driver 580.126.20 |
| GPU accessibility | The workspace has `/dev/nvidia0`–`/dev/nvidia3`, `/dev/nvidiactl`, `/dev/nvidia-uvm`, and driver entries for four RTX PRO 5000 Blackwell GPUs. It lacks `nvidia-smi`, `libnvidia-ml.so.1`, any NVML library in the dynamic-linker cache, and CDI specs. `nvidia-container-cli info` fails before container creation: `load library failed: libnvidia-ml.so.1: cannot open shared object file`. |
| CPU execution | Explicit `--runtime runc` starts the existing robot image successfully; temporary test container limited to 2 CPUs / 2 GiB, no GPU requested |
| OSMO access | OSMO CLI absent; no SSH client config/alias or OSMO config found; active workflow ID and assigned GPU/CPU/memory/storage allocation not established. The user subsequently identified this as a deliberately CPU-only transfer workspace submitted with `gpu: 0`. |
| Checked-in workflow | Requests 16 CPUs, 1 GPU, 64 GiB RAM, 500 GiB storage; this is a template, not evidence of the running job request |
| Persistence | Source resides on workspace overlay; no verified persistent cache/result volume; export before termination |

The host inventory does not establish a job's active allocation, but the pool baseline above establishes the intended fair-use request: 12 CPUs, 1 GPU, 48 GiB RAM and 500 GiB storage. This session did not edit or submit a workflow. The user confirmed that this transfer workspace was intentionally submitted with `gpu: 0`, so it is not an Isaac/SIL-ready environment. The `nvidia-smi` absence and nested-container failure are expected in that CPU-only context and must not be reported as an OSMO infrastructure defect.

### GPU runtime diagnosis

GPU device nodes alone are insufficient for nested GPU containers. A GPU-requesting workspace must also expose compatible NVIDIA driver userspace libraries, at minimum NVML through `libnvidia-ml.so.1`; the runtime also needs `libcuda.so` for CUDA workloads. In this `gpu: 0` transfer workspace, device nodes were observable but those libraries were absent. That combination is not a valid GPU entitlement or a meaningful Isaac readiness test. The installed NVIDIA Container Toolkit consequently cannot initialize NVML or generate an inner GPU-container specification. The inner Docker daemon correctly registers `nvidia` as its default runtime; changing its Docker runtime configuration, downloading CUDA/PyTorch, or changing the RRM source cannot supply a GPU allocation.

For GPU SIL, submit a new workspace that actually requests `gpu: 1` (with the pool's 12 CPU / 48 GiB fair-use target and 500 GiB storage, unless the active pool rejects it). Then establish that the allocated workspace has both device nodes and compatible driver userspace libraries:

```sh
ls -l /dev/nvidia*
ldconfig -p | grep -E 'libnvidia-ml|libcuda'
find /usr /lib /lib64 -name 'libnvidia-ml.so*' 2>/dev/null
which nvidia-smi || true
```

After those checks show the libraries are available, test nested GPU execution:

```sh
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

The CUDA image is an infrastructure smoke test only. It neither changes RRM nor authorizes model downloads. If it succeeds, rerun AirStack's normal `airstack up`/`airstack ready` flow and record the actual assigned GPU UUID/count, VRAM, driver/runtime versions and container mount mapping. Isaac SIL can then proceed without changing RRM code or downloading a different model. Do not manually bind host driver paths or install a driver inside the pod.

## Historical readiness and next remote operation

The [AirStack OSMO guide](https://docs.theairlab.org/0.20/docs/tutorials/airstack_on_osmo/) describes a workspace with nested Docker and port-forwarded access. The preceding paragraph applies only to the earlier CPU-only transfer workspace. It is superseded for the current live GPU workflow by the correction at the top of this file: do not diagnose driver injection, alter Compose, or restart Isaac merely because host visibility differs from the one-GPU workflow request. Do not infer VRAM or user allocation from host `/proc` entries. No unrelated containers or workflows were stopped.

For CPU verification, the source is explicitly bind-mounted at `/workspace/rrm` and session evidence at `/evidence` inside an isolated robot-image container. Pydantic is installed into a temporary container directory, without altering the base image or using the historical GPU provisioning script. This mount mapping must be reconsidered for future Isaac and robot containers; host paths are not automatically available inside them.

Before GPU SIL: establish persistent remote artifacts/cache and quota, select a CONOPS-compatible scene/controller, pin simulator/runtime versions, validate stop/observation channels and then run deterministic integration before adding models. Download model weights only on verified remote compute into the identified cache, recording revision, license, precision and hashes; exclude weights from source archives and Git. No model weights or datasets were downloaded in this session.
