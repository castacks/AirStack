# Follow-up issues to file after the RFC #379/#380 campaign

Append-style scratchpad: each `##` section is one issue to open on GitHub.

## prebuilt image mode

**Problem.** `.env` documents `DOCKER_IMAGE_BUILD_MODE="dev" | "prebuilt"`, but the
variable is consumed nowhere except image tag strings
(`...:v${VERSION}_robot-x86-64_${DOCKER_IMAGE_BUILD_MODE}` in
`robot/docker/docker-compose.yaml` and siblings). Setting `prebuilt` today
changes the tag and nothing else — no Dockerfile stage copies `robot/ros_ws`
into the image or runs `colcon build` at build time. The old comments claimed
otherwise ("prebuilt is for built ros_ws baked into the image"); those have been
corrected to call it a tag discriminator only.

**Value.** A real prebuilt mode gives:
- field/deploy images that boot straight into a built workspace (no `bws` on
  first start, no source bind-mount requirement on Jetson/VOXL targets)
- reproducible releases — the image content IS the tested workspace, matching
  the cosign-signed release flow in `docker-build.yml`
- faster CI system tests when paired with the registry layer cache (skip the
  in-container colcon build entirely)

**Sketch.**
1. Add a `prebuilt` build stage to `robot/docker/Dockerfile.robot` (and l4t
   variants): `COPY robot/ros_ws/src` + `colcon build` + bake `install/`;
   select it via a compose `build.target` keyed on `DOCKER_IMAGE_BUILD_MODE`
   (or a `build.args` toggle — target keeps the dev stage untouched).
2. Entry point honors baked workspace: `sws` sources the baked
   `install/setup.bash` when present and no bind-mount shadows it; compose
   omits the `ros_ws` volume in prebuilt mode (profile or generated override).
3. Fingerprint the workspace into `docker_image_plan.py` so prebuilt images
   rebuild when `robot/ros_ws/src/**` changes (today's plan only fingerprints
   docker inputs — correct for dev mode, wrong once code is baked).
4. Re-truth the docs (`docs/robot/docker/index.md` env table, `.env` comment,
   `bump-version-and-release` skill) back to two real modes once it lands.
