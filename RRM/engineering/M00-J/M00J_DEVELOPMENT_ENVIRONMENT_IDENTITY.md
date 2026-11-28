# M00-J development environment identity

2026-11-28 UTC; ore_proj; starting commit `d678846801e859c68902bbb9c2e6f0c77760e8b0`. Reference M00-I's build-environment assessment and native/interface matrices; no broad source reassessment was needed. Source bytes are identified in `RRM/evidence/M00-J/source-manifest.json`.

Required family: AirStack **robot-desktop development image**, Linux amd64, ROS2 Jazzy. `robot/docker/docker-compose.yaml:12` interpolates the image from `.env:11–21`. Repository defaults resolve exactly to:

`airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev`

This is a **documented tag**, not a new daemon observation or immutable digest. Shell/CLI overrides could select something else. `.env` explicitly states that dev/prebuilt is only a tag discriminator and that workspace-baked prebuilt is future work. No configured tag proves the image contains a built robot workspace.

| Identity field | Evidence and limit |
|---|---|
| Definition | `robot/docker/Dockerfile.robot`; compose desktop args BASE_IMAGE=nvidia/cuda:13.0.2-base-ubuntu24.04, ROS_DISTRO=jazzy; TARGET_ARCH defaults x86_64; final stage runtime |
| Toolchain/workspace | Dockerfile compiler/CMake/ros-dev-tools declarations; final WORKDIR=/root/AirStack/robot/ros_ws; `.bashrc` bws/sws; package coverage remains declaration only |
| Historical image config ID | `sha256:3b98f4333f33dbbe734363f19f9ea65f52f4af8a98f4ac3e78f724124af9c290`, observed E1A 2026-11-27T22:08:14.871469+00:00; matching tags and Linux/amd64 recorded |
| Registry manifest digest | No robot RepoDigest recorded; config ID must not be relabeled a registry digest |
| Current local image/container | NOT INSPECTABLE through the permitted path; privileged system Docker control socket not queried |
| Entrypoint | No explicit ENTRYPOINT/CMD/USER in inspected robot Dockerfile; inherited base-image entrypoint/config remains uninspected; stock compose command starts SSH/tmux/Xvfb and may autolaunch ROS |
| Development profile/mounts | Stock desktop extends privileged robot_base, requests GPU, publishes SSH ports and mounts writable workspace/shell/display paths; unsuitable unchanged for compilation-only isolation |

Classification evidence: **D, repository build definition available**; this does not mean image construction is necessary. **A** image and **B** container were historically observed, but current local image existence and stopped/running state are unverified. **C** cannot be claimed because absence has not been established. Overall readiness is J4 for the current access boundary. Stale IDE service `robot` and `/home/robot/AirStack` remain the incompatible reference identified in M00-I; no protected correction or image construction occurred.
