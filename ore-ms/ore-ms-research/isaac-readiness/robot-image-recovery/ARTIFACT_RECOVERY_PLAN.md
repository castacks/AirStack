# Robot artifact recovery plan

**IMAGE RECOVERY BLOCKED — MAINTAINER ACTION REQUIRED**

The exact destination is `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev`. The earlier qualified ORE-MS pod used robot image configuration ID `sha256:191cbf60c589dac0167518e4d3c0fa3b07807eac4d3d4df0b5d25bc003a9e1` with that local tag. Its registry manifest was unrecorded. No matching source artifact was found on 26 November 2026. [Prior image provenance](../IMAGE_PROVENANCE.md) · [Prior runtime identities](../numpy-remediation/evidence/recreated-containers.txt)

- The only configured Docker context was `default` in `airstack-dev-314`. `docker image inspect` by full configuration ID returned `No such image`. Isaac and GCS remained in `Created`. No alternate authorized workspace endpoint or archived image was available.
- Harbor's public inventory for `airstack/airstack` returned 160 artifacts across two pages, with zero untagged entries. Buildx read all 160 top-level manifests and resolved Linux/amd64 children for indexes that exposed them. The scan returned 160 configuration references and zero lookup errors or matches. This covers that repository's visible inventory at inspection time. Other repositories, retired workspaces and backups require maintainer information.
- A fresh lookup of the destination again returned `not found`. [Previous missing-tag diagnosis](../robot-image-remediation/DIAGNOSTIC_REPORT.md)

## Restore the original if a maintainer finds it

1. Have the maintainer check the earlier workspace, approved image backups and Harbor audit history for the exact artifact. For a local image, run `docker image inspect --format '{{.Id}} {{.Os}}/{{.Architecture}}' <candidate>`. Require the full ID above and `linux/amd64`. For a registry candidate, resolve its Linux/amd64 child manifest and compare `.config.digest` from its raw manifest.
2. Obtain explicit publication authorization. Confirm the destination is still absent and coordinate the publication so an intervening tag cannot be overwritten. From the authorized daemon holding the matching local image, use the following commands only after approval.

```bash
original=sha256:191cbf60c589dac0167518e4d3c0fa3b07807eac4d3d4df0b5d25bc003a9e1
target=airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev
test "$(docker image inspect --format '{{.Id}} {{.Os}}/{{.Architecture}}' "$original")" = "$original linux/amd64" &&
docker image tag "$original" "$target" &&
docker image push "$target"
```

If the artifact is another approved registry manifest, the authorized maintainer can use a registry-native copy or `docker buildx imagetools create --tag "$target" "$source_digest_ref"` after verifying the Linux/amd64 config match. The published manifest or index digest can differ from the source wrapper. The platform image configuration must match.

## Build only if the original cannot be recovered

Use the research tree at `7eae5aaee90c747a85ccbd94f746691595fcf385` in a clean, detached authorized builder checkout. Compose resolves context `robot/docker`, Dockerfile `Dockerfile.robot`, Ubuntu 24.04 CUDA base `nvidia/cuda:13.0.2-base-ubuntu24.04`, and `ROS_DISTRO=jazzy`. Dockerfile SHA-256 is `b946dca94ddb09f9805e8522415148378d632d23158acd922d2db9bf2087ace2`. Windows, Jetson and VOXL variants are outside this path. Direct Buildx invocation avoids the additional floating cache tag in Compose. After explicit build authorization, build only the robot image into a local candidate tag:

```bash
test "$(git rev-parse HEAD)" = 7eae5aaee90c747a85ccbd94f746691595fcf385 &&
test -z "$(git status --porcelain)" &&
docker buildx build --platform linux/amd64 --load \
  --file robot/docker/Dockerfile.robot \
  --build-arg BASE_IMAGE=nvidia/cuda@sha256:605fb0c8acf8674e164d822da8a8521f3a655056e569f0899e72ae940e1fe7dc \
  --build-arg ROS_DISTRO=jazzy \
  --tag airstack-robot-recovery:7eae5aaee robot/docker
```

The base is the Linux/amd64 manifest resolved for the Dockerfile's Compose-selected base tag during this review. Keep the build log, package inventory, image configuration ID, and resolved dependency versions. APT inputs, the ROS key, PyPI dependencies, the shallow TPM clone and the Foxglove latest download include moving inputs. The command reproduces the build selection with a pinned base and yields a new artifact. Byte-for-byte equivalence with the prior local image cannot be assumed. [Cold-start checklist](NEXT_COLDSTART_CHECKLIST.md)

Under separate authorization, verify the candidate with the mounted research workspace, Jazzy and installed MAVROS, and a bounded grounded integration check with the qualified Isaac image. Require the workspace build and launch to succeed with connected MAVROS, fresh odometry, and the canonical MAVROS topics. Record the image ID and gates before any production tag publication. The tag and publish procedure above then applies with the explicitly approved new configuration ID in place of the original.

## Verify publication without pulling layers

Using the intended workflow's authorized registry credential, inspect the target with `docker buildx imagetools inspect "$target"`. For an index, select the Linux/amd64 child. For a single manifest, verify its config reports Linux/amd64. Run `docker buildx imagetools inspect "$target_repo@$amd64_manifest" --raw | jq -r .config.digest`, where `target_repo=airlab-docker.andrew.cmu.edu/airstack/airstack` and `amd64_manifest` is the observed `sha256:...` value. Record the tag's digest, the platform manifest and the expected config ID. Treat 401/403 as credential/access evidence and a confirmed 404 as unresolved artifact availability. This gate remains pending until there is a published candidate.
