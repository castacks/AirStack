# Docker build profiles and build-args

High-level overview of the build-time knobs used to create AirStack robot images. For step-by-step profile creation, validation snippets, and YAML quoting rules, use the **`docker-build-profiles` agent skill** at `.agents/skills/docker-build-profiles/` — it is intended for maintainers and AI agents generating or reviewing new compose profiles.

## What this covers

- AirStack builds robot images from one Dockerfile: `robot/docker/Dockerfile.robot`
- The active variant is selected by `build.args` on each service in `robot/docker/docker-compose.yaml`
- Shared defaults live in `robot/docker/robot-base-docker-compose.yaml`

Key inputs:

| Arg | Purpose |
|-----|---------|
| `BASE_IMAGE` | OS / vendor base image |
| `ROS_DISTRO` | ROS 2 distro to install (e.g. `jazzy`) |
| `PYTHON_VERSION` | Must match the ROS Python path; **quote in YAML** (e.g. `"3.12"`) |
| `REAL_ROBOT` | Platform-specific content toggles |
| `SKIP_MACVO` | Skip MACVO in image when set |
| `SKIP_TENSORRT` | Skip TensorRT in image when set |

## Example profiles

**Desktop** (`robot-desktop`) — x86-64 dev / simulation:

```yaml
# robot/docker/docker-compose.yaml (excerpt)
robot-desktop:
  build:
    dockerfile: ./Dockerfile.robot
    args:
      BASE_IMAGE: nvidia/cuda:13.0.2-base-ubuntu24.04
      ROS_DISTRO: jazzy
```

**Jetson L4T** (`robot-l4t`) — uses `network: host` during build:

```yaml
# robot/docker/docker-compose.yaml (excerpt)
robot-l4t:
  build:
    dockerfile: ./Dockerfile.robot
    network: host
    args:
      BASE_IMAGE: *l4t_stack_base_image   # from robot-l4t-stack-base
      REAL_ROBOT: true
      SKIP_MACVO: true
      SKIP_TENSORRT: true
      ROS_DISTRO: jazzy
```

**New profile skeleton** — when adding a platform, follow the agent skill for full steps:

```yaml
robot-myboard:
  build:
    context: ./robot/docker
    dockerfile: ./Dockerfile.robot
    args:
      BASE_IMAGE: nvcr.io/nvidia/l4t-jetpack:r36.4.0
      ROS_DISTRO: jazzy
      PYTHON_VERSION: "3.12"
      REAL_ROBOT: true
      SKIP_MACVO: true
    # L4T only, when needed:
    # network: host
```

## YAML quoting

Unquoted `3.12` can be parsed as float `3.1`, which breaks `PYTHONPATH` paths. Always quote numeric Python versions:

```yaml
PYTHON_VERSION: "3.12"   # correct
PYTHON_VERSION: 3.12     # avoid
```

## Build-time validation (optional)

The agent skill recommends an early Dockerfile check before composing `PYTHONPATH`:

```dockerfile
# robot/docker/Dockerfile.robot
RUN test -d /opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION} \
  || (echo "Invalid PYTHON_VERSION=${PYTHON_VERSION} or missing ROS python path" && exit 1)
```

## Network mode

- `network: host` under `build:` is a workaround for Jetson/L4T build issues (Docker networking / host kernel modules).
- Generally **not** needed for the desktop profile.
- L4T stack-base images are built via `robot/docker/Dockerfile.l4t-stack-base` before `Dockerfile.robot` runs on Jetson targets.

## Human-friendly `airstack` commands

Prefer the `airstack` CLI over raw `docker compose` for day-to-day use:

```bash
# Build a compose service image
airstack image-build robot-desktop

# Build and start a service
airstack image-build robot-desktop
airstack up robot-desktop

# Inspect the robot image build with full output
airstack image-build --target builder --progress=plain robot-desktop

# Jetson: stack-base is built first automatically
airstack image-build --profile l4t robot-l4t

# Open a shell in a running container
airstack connect robot-desktop --command=bash

# View container logs
airstack logs robot-desktop
```

## Adding a new profile

1. Pick a service name and `BASE_IMAGE` for the target platform.
2. Add a service block in `robot/docker/docker-compose.yaml` with quoted `build.args`.
3. For L4T/Jetson, consider `network: host` on `build:` and the stack-base image pattern.
4. Run `airstack image-build <service>` and verify the builder stage imports ROS Python packages.

For copy-paste validation commands, compose templates, and agent-oriented checklists, see `.agents/skills/docker-build-profiles/SKILL.md`.
