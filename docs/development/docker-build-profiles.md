# Docker build profiles and build-args

This page gives a high-level overview of the build-time knobs used to create AirStack robot images. The low-level, copyable snippets and validation checks live in [.agents/skills/docker-build-profiles/SKILL.md](../../.agents/skills/docker-build-profiles/SKILL.md).

## What this covers

- AirStack uses one Dockerfile, [robot/docker/Dockerfile.robot](../../robot/docker/Dockerfile.robot), to build multiple image variants.
- The active variant is selected by `build.args` in [robot/docker/docker-compose.yaml](../../robot/docker/docker-compose.yaml).
- The key inputs are `BASE_IMAGE`, `ROS_DISTRO`, and `PYTHON_VERSION`.

## Common build args

- `BASE_IMAGE` selects the OS/vendor base image.
- `ROS_DISTRO` selects the ROS 2 distro to install.
- `PYTHON_VERSION` must match the ROS Python path for the distro. Quote it in YAML, for example `"3.10"`.
- `REAL_ROBOT`, `SKIP_MACVO`, and `SKIP_TENSORRT` toggle image content for platform-specific builds.

## Network mode

- `network: host` is used as a workaround for Jetson/L4T build issues with Docker networking and host kernel modules.
- It is generally not needed for the desktop profile.

## Where to look for examples

- The current service profiles in [robot/docker/docker-compose.yaml](../../robot/docker/docker-compose.yaml) show the desktop and L4T build args.
- The agent SKILL contains the detailed profile-creation steps, validation snippet, and YAML quoting guidance.

## Human-friendly `airstack` commands

For readers, prefer the `airstack` workflow instead of raw Docker commands.

```bash
# Build a compose service image
airstack image-build robot-desktop

# Build and start a service
airstack image-build robot-desktop
airstack up robot-desktop

# Inspect the robot image build with full output
airstack image-build --target builder --progress=plain robot-desktop

# Open a shell in a running container
airstack connect robot-desktop --command=bash

# View container logs
airstack logs robot-desktop
```

For agent-facing automation and low-level debugging, use the SKILL instead of this page.
