# Static Isaac annotator readiness R3

27 November 2026 UTC. **PASS for offline qualification.** R3 requires a separately authorized Isaac diagnostic to verify the native writer payloads on the pinned image. The implementation uses a documented common-frame association mechanism. No Isaac process was launched for R3.

## R2 failure and API audit

R2 initialized the camera and made both annotator attachment calls before rendering. Its lifecycle journal records that playback and three `world.step(render=True)` calls returned. At the first readout, the cloned camera dictionary's instance-ID entry was `None`. Dereferencing it raised before depth validation or native timestamp extraction. Those records establish the missing instance-ID value at readout. They do not establish whether an annotated render had completed, whether more updates would have populated the value, or whether the annotator had a separate problem. A shutdown start with no matching completion was also recorded. [R2 report](STATIC_ISAAC_ONE_FRAME_DIAGNOSTIC_R2.md) · [R2 evidence references](static-isaac-one-frame-r2/verification.json)

The Isaac Sim 5.1 source shows that `Camera.get_current_frame(clone=True)` deep-copies a cache. Its render callback evaluates the scene-data graph, sets one render reference and simulation time on the cache, then reads each attached annotator in turn. R2 did not check for temporary unavailability or independently bind each payload to a frame. The public source also uses a rational `ReferenceTime` value for `rendering_frame`, while the archived R2 worker expected an integer. R2 exited before reaching that conversion. [Isaac Sim 5.1 Camera source](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py#L363) · [Render callback and native time mapping](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py#L466)

## Association and completion gates

The worker initializes the stationary camera and attaches two Replicator writers to its single render product. Each writer declares exactly one sensor annotator and its own `ReferenceTime` dependency. Replicator documents writer delivery as a package of required annotations and describes `ReferenceTime` as the rational time of the render and its associated annotations. R3 compares the exact reference pair from each writer packet and resolves the common reference through Isaac's native simulation-time mapping. No wall time, render-attempt number or camera-cache timestamp is used as an association key. Implicit capture on playback is disabled. [Replicator 1.12.27 writer contract](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html#writer-base-class) · [Native ReferenceTime contract](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html#referencetime)

Each of at most **eight attempts** clears both writer slots, requests one synchronous Replicator capture step with `wait_for_render=True`, one subframe and a 0.1 s timeline increment, and checks the new packets. The first pair with equal native references, resolved simulation time, valid float32 axial depth and uint32 IDs at the expected 33 × 25 dimensions, and finite positive depth at a pixel labeled as the target is exported. Invalid background values stay intact. The bound is a diagnostic budget and leaves the scientific distance criteria untouched. The host supervisor caps the entire worker command at 150 s in the procedure below. [Replicator 1.12.27 step contract](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html#simulation-control-sync) · [Worker](static-isaac-annotator-r3/capture_one_r3.py) · [Readiness gate](static-isaac-annotator-r3/readiness.py)

Every attempt starts with a durable record and records its outcome, payload status, observed native references, dimensions and types when available. A thrown acquisition exception persists its traceback. An exhausted run with usable-shaped payloads but missing, mismatched or unmappable identity raises `UNVERIFIED_FRAME_ASSOCIATION`. A run that remains unable to produce usable payloads raises `READINESS_BOUND_EXHAUSTED`. The supervisor checks that the last journal result is the first ready pair, that its native references and simulation time match the frame metadata, and that the lifecycle journal includes completed shutdown. Only then can it hash the arrays, metadata, both journals and child log and write `COMPLETED.json`. R2's default supervisor contract still applies to R2. The exact historical R2 inputs remain in its execution bundle and at commit `9ced69687`. [R3 supervisor policy](static-isaac-annotator-r3/supervise_r3.py)

## Offline verification

Seventeen R3 tests and all ten R2 tests passed. The writer-boundary test fed independently tagged mock packets and verified the common reference and mapped time. Adversarial cases included disjoint render references, a missing per-payload reference despite an available envelope timestamp, an unmappable time, missing or empty buffers, mismatched dimensions, invalid target depth, attempt exhaustion, traceback retention, metadata/journal disagreement, and process exit 0 during incomplete acquisition or shutdown. These results qualify the offline gates. They leave the loaded native packet layout, callback delivery and return from `SimulationApp.close()` open. The R1 and R2 retained manifests and frozen M05–M08 checksums passed. [Test and frame-association evidence](static-isaac-annotator-r3/verification.json) · [CPU test suite](static-isaac-annotator-r3/test_readiness.py)

From the AirStack root:

```bash
research=ore-ms/ore-ms-research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$research/static-isaac-recorder-r2" \
  python3 -m unittest discover -v -s "$research/static-isaac-annotator-r3"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v -s "$research/static-isaac-recorder-r2"
python3 ore-ms/maintenance/check_docs.py
```

## One-frame procedure for separate authorization

Use the same cached Isaac Sim 5.1.0 image, verified GPU 0, and isolated stationary USD stage as R2. Run only with separate authorization, from the committed R3 sources. Check the image config, verify the stage and source digests, and compare operational container IDs, images, start times and restarts before and after. The launcher creates a unique disposable container, checks isolation while it is still stopped, and starts it once. The only mounts are immutable scratch input and the new recorder output path.

```bash
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
research="$PWD/ore-ms/ore-ms-research"
r3="$research/static-isaac-annotator-r3"
shared="$research/static-isaac-recorder-r2/recorder.py"
r1="$research/static-isaac-capture-r1"
scratch="$(mktemp -d /root/ore-ms-artifacts/static-isaac-annotator-r3.XXXXXXXX)"
image=airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697
image_id=sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd
test "$(docker image inspect --format '{{.Id}}' "$image")" = "$image_id"
git diff --quiet HEAD -- "$r3" "$shared" "$r1/input/stage.usda"
git rev-parse HEAD > "$scratch/source-commit.txt"
mkdir "$scratch/input"
cp "$r1/input/stage.usda" "$shared" "$r3/"{capture_one_r3,readiness,supervise_r3}.py "$scratch/input/"
(cd "$scratch/input" && sha256sum --check "$r1/input.sha256" --ignore-missing)
(cd "$scratch/input" && sha256sum stage.usda recorder.py capture_one_r3.py readiness.py supervise_r3.py) > "$scratch/input.sha256"
printf '%s\n%s\n' "$image" "$image_id" > "$scratch/image.txt"

snapshot() {
  docker inspect --type container --format '{{.Name}} {{.Id}} {{.Image}} {{.State.StartedAt}} {{.RestartCount}} {{.State.Status}}' \
    isaac-sim-livestream airstack-gcs-1 airstack-robot-desktop-1
}
snapshot > "$scratch/operational-before.txt"
cat > "$scratch/launch-once.sh" <<'SH'
#!/bin/bash
set -euo pipefail
p="$(cd "$(dirname "$0")" && pwd)"
mapfile -t pins < "$p/image.txt"
name="ore-${p##*/}"
test ! -e "$p/container.cid"
(cd "$p/input" && sha256sum --check "$p/input.sha256")
cid="$(docker create --name "$name" --pull never --network none --ipc private \
  --shm-size 1g --gpus '"device=0"' -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N \
  -e PYTHONUNBUFFERED=1 -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  --mount "type=bind,src=$p/input,dst=/capture-input,readonly" \
  --mount "type=bind,src=$p/run/capture,dst=/capture-output" \
  --entrypoint /isaac-sim/python.sh "${pins[0]}" /capture-input/capture_one_r3.py)"
printf '%s\n' "$cid" > "$p/container.cid"
docker inspect --type container --format '{{.Image}} {{.Name}} {{.State.Status}} {{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{.HostConfig.Privileged}} {{json .Mounts}}' "$cid" > "$p/container-created.txt"
test "$(docker inspect --type container --format '{{.Image}} {{.Name}} {{.State.Status}} {{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{.HostConfig.Privileged}}' "$cid")" = "${pins[1]} /$name created none private false"
actual="$(docker inspect --type container --format '{{range .Mounts}}{{printf "%s %s %s %t\n" .Type .Source .Destination .RW}}{{end}}' "$cid" | sort)"
expected="$(printf '%s\n' "bind $p/input /capture-input false" "bind $p/run/capture /capture-output true" | sort)"
test "$actual" = "$expected"
exec docker start --attach "$cid"
SH
status=0
python3 "$scratch/input/supervise_r3.py" --output "$scratch/run" --timeout 150 --mode isaac -- \
  bash "$scratch/launch-once.sh" || status=$?
printf '%s\n' "$status" > "$scratch/supervisor-exit.txt"
if [ -s "$scratch/container.cid" ]; then
  cid="$(cat "$scratch/container.cid")"
  test "$(docker inspect --type container --format '{{.Name}}' "$cid")" = "/ore-${scratch##*/}"
  docker inspect --type container --format '{{.Id}} {{.RestartCount}} {{json .State}}' "$cid" > "$scratch/container-after.txt"
  docker rm -f "$cid" > "$scratch/cleanup.txt"
fi
snapshot > "$scratch/operational-after.txt"
(cd "$scratch/input" && sha256sum --check "$scratch/input.sha256") > "$scratch/input-after.txt"
(
  cd "$scratch"
  find . -type f ! -name SHA256SUMS.txt -print0 | LC_ALL=C sort -z | xargs -0 sha256sum > SHA256SUMS.txt
  sha256sum --check --quiet SHA256SUMS.txt
)
printf 'Evidence directory %s\n' "$scratch"
cmp "$scratch/operational-before.txt" "$scratch/operational-after.txt"
exit "$status"
```

Keep the scratch directory, including the launcher, logs, hashes and partial outputs. Independently check the attempted render count and both ordered journals, compare the native references for the first ready pair, verify the raw buffer dimensions and native simulation timestamp, and verify the completion and scratch manifests. A ready pair with incomplete shutdown is an acquisition observation and a failed run. The smallest live uncertainty is whether the pinned Replicator runtime actually emits both payloads with the required writer-native references within the bound. R3 covers one diagnostic frame and omits the 5 m and 15 m controls, ROS clock and TF qualification, and the two-second association. Preserve any failure and stop.
