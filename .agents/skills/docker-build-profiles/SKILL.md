# docker-build-profiles SKILL

Summary
- Purpose: Provide actionable build-time validation snippets and YAML guidance for AirStack Docker builds. Designed for Claude/GPT-style agents that automate repo changes, CI checks, or PR review suggestions.
- Location: .agents/skills/docker-build-profiles/SKILL.md

When to use
- When adding or updating a `docker-compose` profile that passes `PYTHON_VERSION`, `ROS_DISTRO`, or other numeric-like build args.
- When an automated agent needs to verify a new profile will produce a correct `PYTHONPATH` and avoid YAML float-parsing bugs.

Actions the agent can perform
1. Validate `docker-compose.yaml` args are quoted when numeric-like (e.g. `PYTHON_VERSION: "3.10"`).
2. Insert a build-time validation `RUN` into `robot/docker/Dockerfile.robot` to fail early when the ROS Python path does not exist.
3. Add or update a short test in documentation showing how to build the `builder` stage and check `ament_package` import.
4. Suggest `network: host` under `build:` for L4T/Jetson profiles only when necessary (kernel iptables workarounds).

Snippets (copyable)

- YAML-check rule (agent pseudocode):

  - If a `build.args` key named `PYTHON_VERSION` exists and the value matches `/^\d+\.\d+$/`, ensure it's a quoted string in YAML; otherwise update to `"<value>"`.

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
  --build-arg ROS_DISTRO=<distro> \
  --build-arg PYTHON_VERSION="<x.y>" \
  -t airstack-builder-test:local robot/docker

docker run --rm -it airstack-builder-test:local bash -c "python3 -c 'import ament_package; print(ament_package.__file__)'"
```

Guidance for agents when editing the repo
- Prefer making minimal, reversible changes: add the `RUN test -d ...` check early in the Dockerfile and gate it with informative message text.
- When updating `docker-compose.yaml`, only quote the numeric-like values; do not change unrelated fields.
- If creating PRs, include a short note in the PR description instructing maintainers to run the builder-stage sanity build on both an amd64 desktop profile and an arm64 L4T profile.

Troubleshooting notes
- YAML quirk: unquoted `3.10` may be parsed as float `3.1` — this changes path strings and breaks imports (e.g., `python3.1` instead of `python3.10`).
- Jetson/L4T builds may require `network: host` during the build to avoid kernel iptables/raw table missing-module errors.
- Jetson **`robot-l4t`** builds from **`robot-l4t-stack-base`** (`robot/docker/Dockerfile.l4t-stack-base`), not raw dustynv, so **`Dockerfile.robot` stays Ubuntu-shaped.** `airstack image-build --profile l4t robot-l4t` triggers **`robot-l4t-stack-base`** first (`airstack.sh`); bare `compose build robot-l4t` can still parallelize badly, so list stack-base explicitly if not using AirStack CLI.

Examples of agent prompts
- "Check `robot/docker/docker-compose.yaml` for `PYTHON_VERSION` entries and quote any unquoted numeric values; open a PR with the fixes and include a test log from a builder-stage build."
- "Insert a build-time validation `RUN` in `robot/docker/Dockerfile.robot` that ensures `/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}` exists; push as a separate small commit."

Notes
- This SKILL is intended for agent workflows (automated PRs, repo fixes, review suggestions). Keep changes explicit and reversible.
- For human-facing docs, maintain a high-level page in `docs/` that links to this SKILL for actionable snippets and agent tasks.

SKILL vs human docs

- Keep SKILLs low-level and exact: this file contains raw `docker` commands and copyable build-time snippets intended for agents and automation.
- Keep human-facing docs (`docs/`) showing the `airstack` CLI equivalents and higher-level workflows. This reduces cognitive load for maintainers while preserving exact commands in SKILLs for automation and debugging.
- For the robot profile, human docs should prefer `airstack image-build --target builder --progress=plain <service>` when showing how to inspect build output.

Creating a new profile (step-by-step)

This section shows the minimal, recommended steps an agent or maintainer should perform to add a new `docker-compose` profile that builds from `Dockerfile.robot`.

1. Pick a sensible service name and base image

  - Choose a service name that clearly indicates the platform, e.g. `robot-desktop`, `robot-l4t`, or `robot-myboard`.
  - Select an appropriate `BASE_IMAGE` (amd64 desktop base or `nvcr.io/nvidia/l4t-jetpack:...` for Jetson).

2. Add the profile with quoted numeric args

  - Add a service block in `robot/docker/docker-compose.yaml` (or an override file) and set `build.args` for the profile.
  - Always quote `PYTHON_VERSION` values (e.g. `"3.10"`) so YAML does not convert them to floats.

  Example snippet to add:

  ```yaml
  robot-myboard:
    build:
     context: ./robot/docker
     dockerfile: ./Dockerfile.robot
     args:
      BASE_IMAGE: nvcr.io/nvidia/l4t-jetpack:r36.4.0
      ROS_DISTRO: humble
      PYTHON_VERSION: "3.10"
      REAL_ROBOT: true
      SKIP_MACVO: true
     # for L4T builds only when necessary
     # network: host
  ```

3. Add an optional validate-early check (recommended)

  - Insert the `RUN test -d /opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}` check near the top of `Dockerfile.robot` (before `ENV PYTHONPATH` or any Python-dependent operations). This ensures the build fails fast with a clear message.

4. Run the builder-stage sanity build

  - Run the builder-target build locally (or in CI) to verify the image picks up the correct Python/ROS paths and that `ament_package` imports:

  ```bash
  DOCKER_BUILDKIT=1 docker build --target builder \
    -f robot/docker/Dockerfile.robot \
    --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0 \
    --build-arg ROS_DISTRO=humble \
    --build-arg PYTHON_VERSION="3.10" \
    -t airstack-builder-test:local robot/docker

  docker run --rm airstack-builder-test:local python3 -c "import ament_package; print('ok', ament_package.__file__)"
  ```

5. Smoke-run the full compose build (optional but recommended)

  - Use `docker compose -f robot/docker/docker-compose.yaml build robot-myboard` to ensure compose passes the args correctly.

6. Prepare the PR with clear validation notes

  - Make the code change small and focused (one commit to `docker-compose.yaml`, one optional commit for the `Dockerfile` validation line).
  - In the PR description include the builder-stage test command output and request a reviewer to run the builder-stage test on both an amd64 and arm64 profile if possible.

7. Merge and monitor

  - After merge, ensure CI (if configured) runs the sanity build or that maintainers run the checks on the target hardware.

Agent implementation tips

- When automating the change, produce a single commit that updates only the new service block and, if needed, a second commit that adds the `RUN` check to `Dockerfile.robot`.
- If the target is Jetson/L4T, add `network: host` under `build:` only when prior builds show iptables/kernel errors; do not enable it by default.
- If you detect a pre-existing unquoted `PYTHON_VERSION` in the repo, prefer to update that entry in-place and include an explanatory commit message about YAML float parsing.
