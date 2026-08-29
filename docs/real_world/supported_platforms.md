# Supported Platforms

AirStack targets one compute platform per Docker Compose profile: each service in [`robot/docker/docker-compose.yaml`](https://github.com/castacks/AirStack/blob/main/robot/docker/docker-compose.yaml) extends the shared `robot_base` and pins the base image and platform toggles for its target. This page is the honest status matrix — what CI actually tests, what has been used in the field, and what merely exists as a profile. Deployment *topologies* (which autonomy stack runs where) are a separate axis, covered in [Autonomy Modes](../robot/autonomy_modes.md).

## Platform Matrix

| Platform | Compose profile → service(s) | Base image (from compose `build.args`) | Status |
| -------- | ---------------------------- | -------------------------------------- | ------ |
| x86-64 desktop/laptop (development + simulation) | `desktop` → `robot-desktop` (+ `gcs`) | `nvidia/cuda:13.0.2-base-ubuntu24.04` | **CI-tested (sim)**: `system-tests.yml` runs build, liveliness, sensor, and flight marks against this profile on ephemeral x86 GPU runners; `test_build_docker.py` builds the `robot-desktop`, `gcs`, `isaac-sim`, and `ms-airsim` images |
| x86-64 desktop, split-topology debugging | `desktop_split` → `robot-desktop-onboard` + `robot-offboard` (+ `gcs`) | Same image as `robot-desktop` | Profile exists; shares the CI-tested desktop image, but the split topology itself is not CI-exercised |
| x86-64 ground station (field, offboard half) | `offboard` → `robot-offboard` + `gcs-real` | Same image as `robot-desktop`; GCS from `osrf/ros:jazzy-desktop-full` | Profile exists for field use paired with `l4t_lite`/`voxl` vehicles; not CI-exercised |
| NVIDIA Jetson Orin AGX/NX (full stack onboard) | `l4t` → `robot-l4t` (+ `robot-l4t-stack-base`, `zed-l4t`) | `dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04` via the intermediate `Dockerfile.l4t-stack-base` image | **Supported, field-used**: the [installation guide](installation/index.md) documents tested install and run on Jetson Orin AGX/NX with Ubuntu 22.04 (L4T / JetPack); not covered by CI |
| NVIDIA Jetson Orin, lite (global planning offloaded) | `l4t_lite` → `robot-l4t-onboard` | Same image as `robot-l4t` | Same support level as `l4t`; defaults to the `lite_default` stack |
| ModalAI VOXL 2 | `voxl` (alias `voxl_onboard`) → `robot-voxl-onboard` | `ubuntu:24.04` (aarch64, `REAL_ROBOT=true`, no CUDA) | Profile exists; docs in progress (per [About](../about.md#supported-platforms)); defaults to the `lite_default` stack (compute-constrained); do NOT assume test coverage |

Companion services on the same profiles: `zed-l4t` (profile `l4t`, base `dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04`) runs only the ZED stereo camera driver next to `robot-l4t`; `simple-robot` (profile `simple`) and `robot-test` (profile `test`) are desktop-image variants for the lightweight simulator and colcon-test runs, not separate platforms.

## OS / JetPack Requirements

Only requirements actually stated by a doc or Dockerfile in this repo:

| Platform | Stated requirement | Source |
| -------- | ------------------ | ------ |
| Development machine | Ubuntu 22.04 or 24.04, NVIDIA GPU (RTX 3070 minimum, RTX 4080+ recommended), 16GB+ RAM, ~100GB free storage | [About — FAQ](../about.md#faq) |
| Jetson Orin AGX/NX | Ubuntu 22.04 (L4T / JetPack); container stack pinned to L4T r36.4.0 | [Installation guide](installation/index.md); `r36.4.0` image tags in `robot/docker/docker-compose.yaml` |

## What CI Does and Does Not Cover

GPU simulation jobs (`system-tests.yml`) run on `[self-hosted, airstack-ephemeral]` x86 runners — every CI-verified result is the **desktop x86 simulation path only**. No CI job builds or runs the `l4t`, `l4t_lite`, or `voxl` images; their status above comes from the installation docs and profile definitions, not automated testing.

## See Also

- [Installation on Orin AGX/NX](installation/index.md) — the hardware install walkthrough
- [Autonomy Modes](../robot/autonomy_modes.md) — deployment topologies (which stack runs on which machine, per profile)
- [Docker Build Profiles](../development/intermediate/docker-build-profiles.md) — how compose build args map to image variants (including the L4T/Jetson build chain)
- [Docker Services](../robot/docker/index.md) — the full service hierarchy
