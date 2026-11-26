# ORE-MS-OSMO-ROBOT-IMAGE-001-R1

**IMAGE UNAVAILABLE — PUBLICATION REQUIRED**

Harbor currently has no manifest at `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev`. Buildx using the workspace's existing Docker configuration returned `not found`. The public Harbor artifact API independently returned HTTP 404 and `NOT_FOUND` for that reference and HTTP 200 for `v0.20.8_gcs` in the same repository. This reproduces the artifact lookup failure in `airstack-dev-314` and gives no evidence of an authentication failure. [Phase B timeline](../coldstart-runtime/COLD_START_TIMELINE.md) · [Registry and CI diagnostics](evidence/registry-ci.txt)

## How the tag was selected

- `.env` sets `PROJECT_DOCKER_REGISTRY=airlab-docker.andrew.cmu.edu/airstack`, `PROJECT_NAME=airstack`, `VERSION=0.20.8`, and `DOCKER_IMAGE_BUILD_MODE=dev`. The desktop service in `robot/docker/docker-compose.yaml` combines them as `v${VERSION}_robot-x86-64_${DOCKER_IMAGE_BUILD_MODE}`. It also defines the floating `cache_robot-x86-64_dev` build-cache tag. The desktop build uses Ubuntu 24.04 and ROS 2 Jazzy.
- The OSMO development workspace enables the desktop and livestream profiles and calls `./airstack.sh up`. The tracked workflow and entrypoint supply no robot-image override. The digest guard applies no-build to Compose startup, so the missing robot artifact blocks a clean workspace.
- Main and the research branch select `v0.20.8_robot-x86-64_dev`. Develop at `f677fa272a3ea93e82d44a7b1c7fcceaea66727b` selects `v0.21.0-dev.15_robot-x86-64_dev`. The image-publishing workflow runs automatically on main/develop `.env` updates when the version changes, and supports manual dispatch. It retags matching image-build fingerprints or builds and pushes services individually.

## Publication evidence

Main commit `a6dad8caf54722e5eba3481367e914ce213e6135` introduced version 0.20.8. The corresponding automatic run failed. GitHub records zero executed steps for its failed `docker-build` job and exposes no successful push step for the robot tag. Logs were unavailable via the public API. This run does not establish why the runner job failed or rule out a separate manual publication followed by removal. Harbor's current 404 and inventory establish absence now. The administrator must consult tag audit history for definitive historical publication or deletion. [Release build run](https://github.com/castacks/AirStack/actions/runs/33933686507) · [Build job](https://github.com/castacks/AirStack/actions/runs/33933686507/job/101217394736)

## Compatibility and prior workflows

The earlier manually repaired ORE-MS pod ran the robot using local image configuration ID `sha256:191cbf60c589dac0167518e4d3c0fa3b07807eac4d3d4df0b5d25bc003a9e1`, configured under the same release tag. It ran with the research branch and later alongside the exact qualified Isaac digest. The evidence records no registry manifest for that robot image. A local tag and image configuration ID alone do not establish a published artifact. [Earlier image provenance](../IMAGE_PROVENANCE.md) · [Container recreation record](../numpy-remediation/evidence/recreated-containers.txt)

At `169c83a58bbb387e9a9086ef49ca2dd5887a7eb7`, the tracked `ore_proj` workflow selects its own outer workspace tag and branch. Its .env and robot Compose resolve the same missing 0.20.8 robot tag. It selects a different inner Isaac digest and has its own epoch reconciliation and robot runtime changes. No authoritative manifest for the robot used by a running `ore_proj` workflow was available from this configuration. [Prior ore_proj comparison](../ORE_PROJ_NUMPY_COMPARISON.md)

The public Harbor inventory returned 160 artifacts. I inspected Linux/amd64 manifests for the artifacts tagged with the desktop robot suffix. None had the earlier working robot's configuration ID. The published 0.20.7, cache and 0.21.0-dev.15 images report Jazzy and contain MAVROS installation history, but each has a different image configuration ID. The 0.20.7 release and the research branch share the tracked robot Docker and common Docker inputs reviewed. The research branch adds a MAVROS namespace correction in mounted launch source. The registry metadata does not establish the installed MAVROS package behavior, mounted workspace build, or interoperability with the qualified Isaac and PX4 stack for any of these candidates. The floating cache currently aliases 0.21.0-dev.13. No alternate image meets the requested compatibility gate on the available evidence.

## Smallest correction

Have the registry maintainer restore or publish only the exact desktop robot tag. If the previous qualified local image still exists, the maintainer can verify its full configuration ID against the record above and publish that artifact under the expected tag through the approved registry process. This path requires no rebuild. If that image and any equivalent approved artifact are unavailable, publish `robot-desktop` from reviewed research-compatible inputs, recording the source SHA, Linux/amd64 manifest, configuration ID and ROS 2/MAVROS package versions. Check the resulting image against the mounted research workspace, PX4 SITL connectivity and canonical MAVROS topic namespace before treating a newly built image as equivalent.

Before another cold-start attempt, run these read-only checks using the intended workflow's registry credential on an authorized workstation. Record the Linux/amd64 platform manifest and verify the approved configuration ID from its manifest:

```bash
repo=airlab-docker.andrew.cmu.edu/airstack/airstack
docker buildx imagetools inspect "$repo:v0.20.8_robot-x86-64_dev"
docker buildx imagetools inspect "$repo@sha256:<approved-amd64-manifest>" --raw | jq -r .config.digest
```

The exact-tag restoration requires no Compose or workflow change. The qualified Isaac digest `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`, GCS tag and no-build behavior are preserved. The existing four-case offline startup check passed. No workflow submission, cancellation, runtime restart, image pull, build, tag or publication was performed.
