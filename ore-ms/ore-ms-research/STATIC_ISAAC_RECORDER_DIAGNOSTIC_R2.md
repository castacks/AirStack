# Static Isaac recorder diagnostic R2

27 November 2026 UTC. **R1 root cause undetermined. R2 recorder completion qualified offline.** The one-frame diagnostic is ready for a separately authorized isolated run. Isaac execution, native output delivery and shutdown behavior remain unverified for R2.

## R1 lifecycle and evidence

The retained bundle is `/root/ore-ms-artifacts/static-isaac-geometry-capture-r1-2026-11-27/`. The raw bundle, the committed R1 input hashes and the artifact hashes recorded in R1 verification were checked offline. Checkout and execution provenance are recorded in the [R1 execution report](STATIC_ISAAC_GEOMETRY_CAPTURE_R1.md) and [R1 verification JSON](static-isaac-capture-r1/verification.json). Relevant files are `output/capture.log`, `process-result.txt`, `pinned-app-close.txt` and `SHA256SUMS.txt`. The exact [launcher](static-isaac-capture-r1/command.sh), [container entry script](static-isaac-capture-r1/input/run.sh), [USD stage](static-isaac-capture-r1/input/stage.usda) and [capture source](static-isaac-capture-r1/input/capture.py) remain available.

| Phase | Direct evidence | Assessment |
|---|---|---|
| Isaac startup and stage | Log lines 455–456 report app ready and startup complete. Line 462 names the isolated USD stage during closure. | Kit started and the stage reached the USD context. There is no completed stage-load checkpoint. |
| ROS bridge | Lines 457–459 show the bridge extension starting and loading rclpy. | Bridge initialization reached the rclpy loader. The capture node, clock graph and message delivery remain unverified. |
| Camera and rendering | Line 461 contains a timeline deprecation warning. No camera or step checkpoints exist. | The warning provides no evidence of a completed camera initialization, a rendered sensor frame, or a failure in either. Startup updates can render before sensor data is available. |
| Clock, TF and sensor readout | No clock files, native depth/ID buffers, TF files or control-pass lines were retained. | The R1 code steps and waits for a clock sample before attempting each control readout. Clock samples are mostly held in memory. Exact-time TF collection comes after both controls and a subsequent observation readout. No frame is evidenced by the raw files, and the record cannot locate the last successful step. |
| Shutdown and exit | Line 463 reports app shutdown. `process-result.txt` records child exit 0 and zero geometry artifacts. No traceback is present. | App teardown was reached. Its log line and the shell exit code provide no proof that the Python capture finished. |

R1 calls `app.close()` in `finally` without persisting exceptions before cleanup. The retained source excerpt shows logging being disabled ahead of framework shutdown. It does not establish that cleanup swallowed an exception or returned control to Python. The R1 invocation also had no artifact gate to catch an apparently clean exit. Camera or annotator setup, the first render step, clock graph or delivery, native buffer readout, and an exception obscured by teardown remain plausible. The evidence rules out exit before Kit startup, but cannot distinguish exit before sensor initialization, exit before the first **usable sensor frame**, or exit after an unreported exception.

## R2 completion contract

The [worker](static-isaac-recorder-r2/capture_one.py) writes and syncs a JSON Lines journal covering Kit startup, USD open, camera and annotator initialization, playback, each of three bounded render steps, frame readout, artifact writes and shutdown. It records full Python tracebacks at the failing phase before app teardown can obscure them. The worker reads both annotators and the native render timestamp from one cloned camera frame. It retains the same 33 × 25 perspective camera and stage with one target fixed at its authored 10 m position. Only one native frame is exported. The three render steps are a fixed diagnostic budget, independent of the first valid output. [NVIDIA Isaac Sim 5.1 camera API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html)

The [host supervisor](static-isaac-recorder-r2/recorder.py) creates a new output directory for each attempt and keeps `child.log` outside the worker mount. It returns success only when the child exits 0, all ordered journal phases complete through shutdown, the two raw buffers and camera metadata pass validation, and an atomic `COMPLETED.json` records their SHA-256 hashes and the child log hash. Depth is little-endian float32 axial metres and IDs are little-endian uint32. Both buffers are row-major, 33 columns by 25 rows, 3,300 bytes each. The validation requires a finite positive depth pixel with the target's instance ID, while preserving every depth sample, including background values. It imposes no 10 m accuracy threshold. Metadata retains the render frame and time in seconds, USD stage units and up axis, camera and target prim paths, native intrinsics, the USD local-to-world matrix and the instance label map. [NVIDIA axial depth and instance ID documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html)

A detected timeout, child exit or validation error returns nonzero and leaves `FAILED.json`, available checkpoints and child output. Abrupt supervisor loss may leave neither marker. Treat such a directory as incomplete. An apparently clean exit during `app.close()` still fails the gate if the shutdown completion record is missing, even when both raw buffers exist. Mock fixtures can complete only in explicit mock mode. Native provenance requires the pinned invocation and an independent evidence review in addition to structural validation.

## One-frame runbook for separate authorization

Follow this only after a separate execution authorization. Run from a clean copy of the R2 commit or a descendant with unchanged diagnostic files. Use a host with the pinned image already cached, available GPU device 0 and Linux Docker. The host must have Python 3, Bash and writable scratch storage for the bind mount. The default mode of the supervisor is `isaac`. The example uses private IPC, an isolated container name, no network and no operational workspace mounts. The only stage asset is the pinned USD below. Verify the input manifest immediately before and after execution.

```bash
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
research="$PWD/ore-ms/ore-ms-research"
diagnostic="$research/static-isaac-recorder-r2"
r1="$research/static-isaac-capture-r1"
image=airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697
image_id=sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd
scratch="$(mktemp -d /root/ore-ms-artifacts/static-isaac-recorder-r2.XXXXXXXX)"
name="ore-${scratch##*/}"
test "$(docker image inspect --format '{{.Id}}' "$image")" = "$image_id"
git diff --quiet HEAD -- "$diagnostic" "$r1"
git rev-parse HEAD > "$scratch/source-commit.txt"
printf '%s\n%s\n' "$image" "$image_id" > "$scratch/image.txt"
mkdir "$scratch/input"
cp "$r1/input/stage.usda" "$diagnostic/"{capture_one,recorder}.py "$scratch/input/"
(cd "$scratch/input" && sha256sum stage.usda capture_one.py recorder.py) > "$scratch/input.sha256"
(cd "$scratch/input" && sha256sum --check "$r1/input.sha256" --ignore-missing)
(cd "$scratch/input" && sha256sum --check "$scratch/input.sha256")
docker ps -q | sort > "$scratch/running-containers-before.txt"

result=0
python3 "$diagnostic/recorder.py" --output "$scratch/run" --timeout 150 --mode isaac -- \
  docker run --name "$name" \
  --pull never --network none --ipc private --shm-size 1g --gpus '"device=0"' \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e PYTHONUNBUFFERED=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  --mount "type=bind,src=$scratch/input,dst=/capture-input,readonly" \
  --mount "type=bind,src=$scratch/run/capture,dst=/capture-output" \
  --entrypoint /isaac-sim/python.sh "$image" /capture-input/capture_one.py || result=$?
if cid="$(docker inspect --type container --format '{{.Id}}' "$name" 2>"$scratch/container-inspect.stderr")"; then
  printf '%s\n' "$cid" > "$scratch/container.cid"
  docker inspect --type container --format \
    '{{.Id}} {{.Name}} {{.Image}} {{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{json .Mounts}} {{json .State}}' \
    "$cid" > "$scratch/container.txt"
  test "$(docker inspect --type container --format '{{.Name}}' "$cid")" = "/$name"
  docker rm -f "$cid" > "$scratch/cleanup.txt"
elif [ "$result" -eq 0 ]; then
  result=1
fi
docker ps -q | sort > "$scratch/running-containers-after.txt"
(cd "$scratch/input" && sha256sum --check "$scratch/input.sha256")
printf '%s\n' "$result" > "$scratch/supervisor-exit.txt"
printf 'Evidence directory %s\n' "$scratch"
exit "$result"
```

Keep the scratch directory, including the provenance files, both container snapshots, `run/child.log` and all partial or final outputs. If a terminal interruption prevents cleanup, look up the unique name derived from the scratch directory, then check the container ID and mounts before removing only the diagnostic container. Confirm after execution that the observed container image ID equals the pinned config, network mode is `none`, IPC is private and the only user bind mounts are the declared input/output. Review changes between the before and after running-container lists before drawing any isolation conclusion.

For independent review, verify the input and completion hashes, match the exact command and image to the container record, inspect Kit startup and the phase journal, and decode the two buffers independently with the recorded dimensions and types. Check that the camera/annotator path in the pinned worker is the source of the files, the matching ID resolves to the authored target where there is usable depth, and the frame index and time come from that same native camera frame. A nonzero process result, missing marker or unusable frame closes this attempt as a diagnostic failure. A locally complete run awaits a reviewer if provenance cannot yet be corroborated. R2 intentionally uses native render time and omits ROS bridge setup, `/clock` and exact-time TF checks, distance controls, and the two-second association. Those are outside the one-frame qualification.

## Offline results

From the repository root, run the [CPU-only tests](static-isaac-recorder-r2/test_recorder.py).

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v \
  -s ore-ms/ore-ms-research/static-isaac-recorder-r2
python3 ore-ms/maintenance/check_docs.py
```

Ten tests passed. They cover exit 0 before initialization completes, exit 0 after frame files exist but before shutdown completes, missing or invalid buffers and metadata, durable startup and readout exceptions, a traceback retained across a simulated native exit 0, termination by signal, timeout, a valid mock completion with rechecked hashes, and mock rejection in Isaac mode. The valid case also verifies that reusing its output directory cannot replace its completion record. The source compiles offline. The R1 raw and committed input hashes and the frozen M05–M08 package and plan hashes pass. No Isaac process, OSMO action or container action was used for this qualification.
