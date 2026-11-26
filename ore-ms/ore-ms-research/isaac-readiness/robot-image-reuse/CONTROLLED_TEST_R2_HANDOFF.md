# ORE-MS-OSMO-ROBOT-IMAGE-004-R2 controlled test handoff

**CONTROLLED TEST READY — USER SUBMISSION REQUIRED**

Prepared from accepted commit `e11226c353f7752ca45b8563704412ce8275f389`. Registry metadata from the existing workspace resolved the published 0.20.7 robot index `sha256:bf6a9c210e64615fb45b38ec324b6a1085e1b407c7865d513381e744dc6a49a1` to Linux/amd64 child manifest `sha256:f2c7bce4797008bff889dbef2881fd015780c972f16b8e344c13a22bba02cbf4`. Hashing the raw child manifest reproduced that digest. The child config is `sha256:3d8fa569dfed6f7249faee9c205bc49eb44b711991db9e7c65d1fa406d46fdda`. No image layers were pulled during preparation. [Accepted reuse report](ORE_PROJ_REUSE_REPORT.md)

## Mac-local YAML

In the existing `workspace` task, add or set this one key under the existing `environment:` mapping:

```yaml
OSMO_AIRSTACK_UP: "false"
```

Keep the existing outer image reference, rclone bootstrap, credentials and one-GPU resource request. Preserve `AIRSTACK_BRANCH: "ore-ms-orb-development"`, `AUTOLAUNCH: "true"`, `ISAAC_SIM_LIVESTREAM: "true"`, `NUM_ROBOTS: "1"`, and the qualified `ISAAC_SIM_IMAGE` value given below. If the task already sets `COMPOSE_PROFILES`, use `desktop,isaac-sim-livestream`.

In the shell block in the workspace task's `args:`, replace only the final `exec /usr/local/bin/entrypoint.sh` line with the following lines at the same YAML indentation. Keep the preceding rclone command and arguments. For a task whose args invoke the baked filename directly, use a Bash `-c` args block containing the snippet below. The actual Mac file was unavailable to this session.

```bash
set -eu
entrypoint_src=/usr/local/bin/entrypoint.sh
entrypoint_sha=d75e199301afc31bc9c5bdb384a54efe12256cf53be200a678ab28a4a3772bc9
test "${OSMO_AIRSTACK_UP:-}" = false
printf '%s  %s\n' "$entrypoint_sha" "$entrypoint_src" | sha256sum -c -
entrypoint_tmp=$(mktemp)
sed '/^\.\/airstack\.sh up || log /c\log "OSMO_AIRSTACK_UP=false — skipping airstack up for the controlled test"' \
  "$entrypoint_src" > "$entrypoint_tmp"
bash -n "$entrypoint_tmp"
exec bash "$entrypoint_tmp"
```

The baked script in `airstack-dev-314` invoked bare `./airstack.sh up` even with `OSMO_AIRSTACK_UP=false`. The checked-in research script honors the flag, but the workspace starts from the baked file. The wrapper changes the single unconditional call to a skip log in a temporary copy and leaves the baked file intact. SSH, Docker, clone, NVIDIA setup and credential initialization remain in the bootstrap. If the outer tag resolves to different entrypoint bytes, the checksum makes the task stop before executing the bootstrap. Obtain the new entrypoint's provenance and review it before adapting the wrapper. Validate the Mac-local workflow with the user's installed OSMO validator before submission.

## Workspace startup after user submission

Wait for the skip line and entrypoint-complete line in workspace logs, then connect over the existing SSH forward. Run from a fresh workspace. The Docker listing must be empty before launch. The checks below compare the exact images and verify that the merged robot service has no build stanza. They print only the selected service image references.

```bash
set -euo pipefail
cd /root/AirStack
test "$(git branch --show-current)" = ore-ms-orb-development
git merge-base --is-ancestor e11226c353f7752ca45b8563704412ce8275f389 HEAD
test -z "$(git status --porcelain)"
container_ids=$(docker ps -aq)
test -z "$container_ids"
umask 077
evidence_dir=$(mktemp -d /tmp/ore-ms-robot-reuse.XXXXXX)
overlay="$evidence_dir/robot-reuse.yaml"
export ROBOT_REUSE_IMAGE=airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:f2c7bce4797008bff889dbef2881fd015780c972f16b8e344c13a22bba02cbf4
export ISAAC_SIM_IMAGE=airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697
export COMPOSE_PROFILES=desktop,isaac-sim-livestream NUM_ROBOTS=1 AUTOLAUNCH=true ISAAC_SIM_LIVESTREAM=true
export AIRSTACK_NO_IMAGE_BUILD=1 AIRSTACK_REGISTRY_CACHE=0
cat > "$overlay" <<EOF_OVERLAY
services:
  robot-desktop:
    image: $ROBOT_REUSE_IMAGE
    build: !reset null
EOF_OVERLAY
git rev-parse HEAD | tee "$evidence_dir/source.txt"
docker compose --env-file .env -f docker-compose.yaml -f "$overlay" \
  config --no-env-resolution --format json robot-desktop isaac-sim-livestream gcs |
  jq -er --arg robot "$ROBOT_REUSE_IMAGE" --arg isaac "$ISAAC_SIM_IMAGE" '
    .services |
    if (."robot-desktop".image == $robot and
        (."robot-desktop" | has("build") | not) and
        ."robot-desktop".deploy.replicas == 1 and
        ."isaac-sim-livestream".image == $isaac and
        ."isaac-sim-livestream".deploy.resources.reservations.devices[0].count == 1 and
        .gcs.image == "airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_gcs")
    then to_entries[] | "\(.key) \(.value.image)"
    else error("unexpected controlled-test Compose resolution")
    end' | tee "$evidence_dir/resolved-images.txt"
printf 'Startup evidence directory: %s\n' "$evidence_dir"
./airstack.sh up -f "$overlay" 2>&1 | tee "$evidence_dir/first-up.log"
```

## First start and optional recovery

1. Capture container states, configured images and IDs. Require robot config ID `sha256:3d8fa569dfed6f7249faee9c205bc49eb44b711991db9e7c65d1fa406d46fdda`. Preserve timestamped Isaac, robot and GCS logs and confirm the mounted workspace's colcon build and desktop launch succeeded. Keep credential-bearing logs private and redact them before sharing.
2. Run `./airstack.sh ready --json > "$evidence_dir/first-ready.json" 2> "$evidence_dir/first-ready.stderr"` and record its exit code. In robot domain 1 observe multiple advancing `/clock` samples, connected/disarmed `/robot_1/interface/mavros/state`, and fresh `/robot_1/interface/mavros/local_position/odom`. Check dynamic TF at odometry timestamps and required static TF. Confirm live RGB, CameraInfo, depth and LiDAR publishers and bounded message delivery. [Existing read-only ROS observer](../evidence/observe_ros.py)
3. If the first start fails, preserve the logs, readiness result, clock/TF samples and container start timestamps before any recovery. A robot-only restart requires confirmed stale clock or TF state, a grounded and disarmed vehicle, no active mission, and separate authorization. Then run `docker restart airstack-robot-desktop-1` once, rerun readiness into a separate evidence file, and repeat clock, odometry and TF checks. Preserve Isaac and GCS as they are. [Documented recovery](../robot-image-recovery/ORE_PROJ_STARTUP_TIMING_REVIEW.md)

## Offline verification

Compose v5.1.3 resolved the overlay after the base file, removed the robot build stanza and changed only the robot image and build fields. A stubbed Docker daemon captured the real research CLI issuing `docker compose --env-file <repo>/.env -f <repo>/docker-compose.yaml -f <overlay> up --no-build -d`. The CLI returned success in the positive fixture and propagated simulated exit 42 without another launch. With the false flag, the baked launch block caused one mock Compose up call. The temporary script with the one-line replacement and the tracked research entrypoint each caused zero. The patched full Bash script passed syntax validation. Live workspace bootstrap, package compatibility and runtime remain for the separately submitted controlled test.
