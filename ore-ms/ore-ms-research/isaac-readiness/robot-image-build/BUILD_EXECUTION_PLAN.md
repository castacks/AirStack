# Build execution plan

## Stage 1 — Local build after explicit approval

Use an authorized, dedicated Linux/amd64 builder with Buildx and a clean, detached checkout at the frozen source. Give the builder its own Docker daemon and use a fresh private evidence directory outside the checkout. Confirm the builder points to that daemon and the original image remains unavailable. The following Bash commands are prepared for that stage.

```bash
set -euo pipefail
source_sha=99bee6554120f0fad4e6483433f93a26f1c985f3
base=nvidia/cuda@sha256:605fb0c8acf8674e164d822da8a8521f3a655056e569f0899e72ae940e1fe7dc
test "$(git rev-parse HEAD)" = "$source_sha"
test -z "$(git status --porcelain --untracked-files=all)"
printf '%s  %s\n' \
  b946dca94ddb09f9805e8522415148378d632d23158acd922d2db9bf2087ace2 \
  robot/docker/Dockerfile.robot | sha256sum --check
umask 077
out=$(mktemp -d)
candidate="airstack-robot-recovery:99bee655-$(date -u +%Y%m%dt%H%M%Sz)"
if docker image inspect "$candidate" >/dev/null 2>&1; then
  printf 'Candidate tag already exists: %s\n' "$candidate" >&2
  exit 1
fi
printf '%s\n' "$source_sha" "$base" "$candidate" > "$out/build-inputs.txt"
docker version > "$out/docker-version.txt"
docker buildx version > "$out/buildx-version.txt"
docker image ls -a --digests --no-trunc > "$out/images-before.txt"
docker buildx build --platform linux/amd64 --load --progress plain \
  --file robot/docker/Dockerfile.robot \
  --build-arg "BASE_IMAGE=$base" --build-arg ROS_DISTRO=jazzy \
  --iidfile "$out/image-id.txt" --metadata-file "$out/build-metadata.json" \
  --tag "$candidate" robot/docker 2>&1 | tee "$out/build.log"
docker image inspect --format '{{.Id}} {{.Os}}/{{.Architecture}} {{json .RepoDigests}}' \
  "$candidate" | tee "$out/image-identity.txt"
docker image ls -a --digests --no-trunc > "$out/images-after.txt"
printf 'Build evidence directory: %s\n' "$out"
```

The Dockerfile's default final stage is the desktop runtime stage. Direct Buildx uses the single temporary local tag and skips Compose's additional floating cache tag. Stage 1 starts no AirStack services. Preserve the first failure log and verify existing image references and IDs stayed stable. Use image inspect as the authority for the local configuration ID. Record exporter descriptor or manifest digests from Buildx metadata when emitted. A local `--load` image can have empty RepoDigests. Capture its actual registry manifest during Stage 4.

## Subsequent gates

| Stage | Required outcome |
|---|---|
| 2 — Static image inspection | Verify configuration ID, platform, ROS/MAVROS inventory and binary/package checks in a disposable network-disabled probe. [Compatibility gate](COMPATIBILITY_GATE.md) |
| 3 — Runtime integration | Obtain separate authorization for one grounded, isolated test with the qualified Isaac image and mounted frozen source. Require the compatibility gates before considering publication. |
| 4 — Registry publication | Obtain separate authorization and coordinate with the maintainer. Recheck destination availability, publish only the approved robot image, and verify the manifest and config ID through the intended workflow credential. |
| 5 — Fresh OSMO cold start | The user controls submission after Stage 4. Observe the preserved startup behavior and retain first-failure evidence. [Cold-start checklist](../robot-image-recovery/NEXT_COLDSTART_CHECKLIST.md) |
