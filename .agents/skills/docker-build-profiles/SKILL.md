---
name: docker-build-profiles
description: Build-time validation and guidance for AirStack Docker compose profiles and build args — adding a robot profile, quoting numeric-like YAML args, the L4T/Jetson build chain, and how module-owned dependencies enter images via module layers (airstack module lock --build) now that per-capability SKIP_* build args are gone. Use when adding/updating a compose profile or debugging a Dockerfile.robot build.
license: BSD-3-Clause-Clear
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Docker Build Profiles and Build Args

Summary
- Purpose: Provide actionable build-time validation snippets and YAML guidance for AirStack Docker builds. Designed for agents that automate repo changes, CI checks, or PR review suggestions.
- Location: .agents/skills/docker-build-profiles/SKILL.md

When to use
- When adding or updating a `docker-compose` profile / service that builds from `robot/docker/Dockerfile.robot`.
- When a build arg looks numeric (`PYTHON_VERSION`, versions like `36.4.0`) and YAML float parsing could corrupt it.
- When deciding where a new dependency belongs: the trunk Dockerfile vs a **module layer**.

## Current profile landscape (robot/docker/docker-compose.yaml)

Robot services select by compose profile: `desktop` (robot-desktop, the default dev
target), `desktop_split` / `offboard` (split-stack pairs), `simple`, `voxl` /
`voxl_onboard`, `l4t` (Jetson: `robot-l4t` + `robot-l4t-stack-base` + `zed-l4t`),
`l4t_lite`, and `test`. Build args passed by compose today are `BASE_IMAGE`,
`ROS_DISTRO` (jazzy), `REAL_ROBOT`, and (l4t) the stack-base image ref.
`PYTHON_VERSION` is an `ARG` **defaulted inside `Dockerfile.robot`** (currently
`3.12`) — compose does not pass it; only quote-and-pass it if a new profile genuinely
needs a different Python.

## Module-owned dependencies: NOT build args anymore

The per-capability skip args are **gone**: `SKIP_MACVO` and `SKIP_TENSORRT` no longer
exist in `Dockerfile.robot` or any compose file. MAC-VO (and with it the TensorRT apt
blocks, torch/onnx wheels, and model weights) was extracted to the external
`asm_macvo` module, whose own `Dockerfile.module` owns those deps.

Module deps enter images via **module layers** (RFC #379 §6, `docs/development/modules.md`):

- **Tier 1** — `module.yaml` `deps: {apt: [...], pip: [...]}` → one generated `RUN`
  layer per module in `.airstack/generated/layers/<host>/Dockerfile.composed`.
- **Tier 2** — the module's `Dockerfile.module`, built with
  `--build-arg BASE_IMAGE=<previous chain link>` (always `ARG BASE_IMAGE`, never a
  fixed base).
- **Tier 3** — a prebuilt `overlay_image` ref.

`airstack module sync` (and `airstack module lock`) is **plan-only**: it writes
`.airstack/generated/layer_plan.json`, the composed Dockerfile, and `modules.lock`,
and never calls docker. To actually build the chain:

```bash
airstack module lock --build     # = tools/compose_module_layers.py --build
```

Rule of thumb: a dependency used by exactly one optional capability belongs in that
capability's module (tier 1 or 2), not in `Dockerfile.robot`. Trunk Dockerfiles thin
out as deps migrate into the modules that own them.

Actions the agent can perform
1. Validate `docker-compose.yaml` args are quoted when numeric-like (e.g. `PYTHON_VERSION: "3.12"`).
2. Insert a build-time validation `RUN` into `robot/docker/Dockerfile.robot` to fail early when the ROS Python path does not exist.
3. Add or update a short test in documentation showing how to build the `builder` stage and check `ament_package` import.
4. Suggest `network: host` under `build:` for L4T/Jetson profiles only when necessary (kernel iptables workarounds).
5. Route new capability-specific deps to a module layer instead of a trunk build arg (see above).

Snippets (copyable)

- YAML-check rule (agent pseudocode):

  - If a `build.args` key named `PYTHON_VERSION` (or any version-shaped value) exists and the value matches `/^\d+\.\d+$/`, ensure it's a quoted string in YAML; otherwise update to `"<value>"`.

- Dockerfile validation snippet (recommended, place before using `PYTHON_VERSION` to compose `PYTHONPATH`):

```dockerfile
RUN test -d /opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION} \
  || (echo "Invalid PYTHON_VERSION=${PYTHON_VERSION} or missing ROS python path" && exit 1)
```

- Quick builder-stage test commands (agent can run or instruct user to run):

```bash
DOCKER_BUILDKIT=1 docker build --target builder \
  -f robot/docker/Dockerfile.robot \
  --build-arg BASE_IMAGE=<base> \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg PYTHON_VERSION="3.12" \
  -t airstack-builder-test:local robot/docker

docker run --rm airstack-builder-test:local bash -c "python3 -c 'import ament_package; print(ament_package.__file__)'"
```

Guidance for agents when editing the repo
- Prefer making minimal, reversible changes: add the `RUN test -d ...` check early in the Dockerfile and gate it with informative message text.
- When updating `docker-compose.yaml`, only quote the numeric-like values; do not change unrelated fields.
- Every service with a `build:` section needs **both** `cache_from` entries (the versioned tag and the floating `${CACHE_TAG:-cache}` tag) or its CI builds will always be cold (see AGENTS.md "Docker layer cache").
- If creating PRs, include a short note in the PR description instructing maintainers to run the builder-stage sanity build on both an amd64 desktop profile and an arm64 L4T profile.

Troubleshooting notes
- YAML quirk: unquoted `3.10` may be parsed as float `3.1` — this changes path strings and breaks imports (e.g., `python3.1` instead of `python3.10`).
- Jetson/L4T builds may require `network: host` during the build to avoid kernel iptables/raw table missing-module errors.
- Jetson **`robot-l4t`** builds from **`robot-l4t-stack-base`** (`robot/docker/Dockerfile.l4t-stack-base`), not raw dustynv, so **`Dockerfile.robot` stays Ubuntu-shaped.** `airstack images build --profile l4t robot-l4t` triggers **`robot-l4t-stack-base`** first (`airstack.sh`); bare `compose build robot-l4t` can still parallelize badly, so list stack-base explicitly if not using AirStack CLI.
- **dustynv `/ros_entrypoint.sh` shadows the apt Jazzy runtime (mavros symbol-lookup crash).** The dustynv base sources a prebuilt *source* ROS at `$ROS_ROOT/install` from PID 1, prepending its older libs (e.g. `fastcdr` 2.2.5) ahead of the apt Jazzy (2.2.7) that `Dockerfile.robot` layers on top — apt-built nodes like mavros then die with symbol-lookup errors under tmux autolaunch. `Dockerfile.l4t-stack-base` neutralizes it by overwriting `/ros_entrypoint.sh` with a `exec "$@"` passthrough; shells get ROS from `/opt/ros/jazzy/setup.bash` via `.bashrc`. If a Jetson node suddenly can't resolve symbols after a base-image bump, check whether the entrypoint passthrough is still in place.
- **ZED SDK version is pinned across `zed/Dockerfile.zed-l4t`** — the `ZED_SDK_URL` (e.g. `.../zedsdk/5.2/...`) and the ROS dep args (`ZED_MSGS_VERSION`, `POINTCLOUD_TRANSPORT*_VERSION`, `BACKWARD_ROS_VERSION`) must move together; a mismatched `zed_msgs` vs SDK breaks the driver build. Bumping the SDK is camera-firmware-coupled, so confirm the target camera runs that SDK line before merging.
- **`pytest` is pinned in `Dockerfile.robot` — do not remove or bump past 8.0.** The builder stage installs `pytest==7.4.*` and a later `RUN` constrains `pytest>=7.4,<8.1`: ROS Jazzy's `launch_testing` still implements `pytest_pycollect_makemodule(path=...)`, which pluggy rejects after pytest 8.1 removed the `py.path` hook argument — an unpinned pytest aborts **every** pytest run in the container at plugin registration, breaking `colcon test` for `ament_python` packages while `ament_cmake` gtest packages are unaffected. The `tests/docker` runner is a separate interpreter and is free to use a newer pytest.
- **Robot build suddenly missing a MAC-VO / TensorRT dep?** Those deps left trunk with the `asm_macvo` extraction. Add the module (`airstack module add https://github.com/castacks/asm_macvo --version <tag>`), then `airstack module lock --build`.

Examples of agent prompts
- "Check `robot/docker/docker-compose.yaml` for numeric-like build-arg values and quote any unquoted ones; open a PR with the fixes and include a test log from a builder-stage build."
- "Insert a build-time validation `RUN` in `robot/docker/Dockerfile.robot` that ensures `/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}` exists; push as a separate small commit."

Notes
- This SKILL is intended for agent workflows (automated PRs, repo fixes, review suggestions). Keep changes explicit and reversible.
- For human-facing docs, maintain a high-level page in `docs/` that links to this SKILL for actionable snippets and agent tasks.

SKILL vs human docs

- Keep SKILLs low-level and exact: this file contains raw `docker` commands and copyable build-time snippets intended for agents and automation.
- Keep human-facing docs (`docs/`) showing the `airstack` CLI equivalents and higher-level workflows. This reduces cognitive load for maintainers while preserving exact commands in SKILLs for automation and debugging.
- For the robot profile, human docs should prefer `airstack images build --target builder --progress=plain <service>` when showing how to inspect build output.

Creating a new profile (step-by-step)

This section shows the minimal, recommended steps an agent or maintainer should perform to add a new `docker-compose` profile that builds from `Dockerfile.robot`.

1. Pick a sensible service name and base image

  - Choose a service name that clearly indicates the platform, e.g. `robot-desktop`, `robot-l4t`, or `robot-myboard`.
  - Select an appropriate `BASE_IMAGE` (amd64 desktop base or an L4T/JetPack base for Jetson; Jetson goes through `robot-l4t-stack-base`).

2. Add the profile with quoted numeric args

  - Add a service block in `robot/docker/docker-compose.yaml` (or an override file) and set `build.args` for the profile.
  - Quote any numeric-like values (e.g. `PYTHON_VERSION: "3.12"` if you must override it) so YAML does not convert them to floats.

  Example snippet to add:

  ```yaml
  robot-myboard:
    build:
     context: ./robot/docker
     dockerfile: ./Dockerfile.robot
     args:
      BASE_IMAGE: <platform base image>
      ROS_DISTRO: jazzy
      REAL_ROBOT: true
     # module-owned deps (e.g. MAC-VO's torch/TensorRT) are NOT build-args:
     # they layer on via `airstack module lock --build` (RFC #379 §6)
     # for L4T builds only when necessary
     # network: host
  ```

3. Add an optional validate-early check (recommended)

  - Insert the `RUN test -d /opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}` check near the top of `Dockerfile.robot` (before `ENV PYTHONPATH` or any Python-dependent operations). This ensures the build fails fast with a clear message.

4. Run the builder-stage sanity build

  - Run the builder-target build locally (or in CI) to verify the image picks up the correct Python/ROS paths and that `ament_package` imports (see the snippet above).

5. Smoke-run the full compose build (optional but recommended)

  - Use `airstack images build robot-myboard` (or `docker compose -f robot/docker/docker-compose.yaml build robot-myboard`) to ensure compose passes the args correctly.

6. Prepare the PR with clear validation notes

  - Make the code change small and focused (one commit to `docker-compose.yaml`, one optional commit for the `Dockerfile` validation line).
  - In the PR description include the builder-stage test command output and request a reviewer to run the builder-stage test on both an amd64 and arm64 profile if possible.

7. Merge and monitor

  - After merge, ensure CI (if configured) runs the sanity build or that maintainers run the checks on the target hardware.

Agent implementation tips

- When automating the change, produce a single commit that updates only the new service block and, if needed, a second commit that adds the `RUN` check to `Dockerfile.robot`.
- If the target is Jetson/L4T, add `network: host` under `build:` only when prior builds show iptables/kernel errors; do not enable it by default.
- If you detect a pre-existing unquoted numeric-like build arg in the repo, prefer to update that entry in-place and include an explanatory commit message about YAML float parsing.
