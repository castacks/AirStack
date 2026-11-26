# Robot-only build inputs

The accepted recovery commit `99bee6554120f0fad4e6483433f93a26f1c985f3` matched the clean local checkout and live research branch before preparation. Use this exact source for the proposed build and mounted-workspace validation. The recovery plan previously identified `7eae5aaee90c747a85ccbd94f746691595fcf385`. Comparing the two revisions showed no changes in the image context, top-level Compose, launch scripts, interface sources, default stack, or simulator inputs reviewed. [Accepted recovery plan](../robot-image-recovery/ARTIFACT_RECOVERY_PLAN.md)

| Input | Verified value |
|---|---|
| Compose service | `robot-desktop` |
| Context and Dockerfile | `robot/docker` and `robot/docker/Dockerfile.robot` |
| Dockerfile SHA-256 | `b946dca94ddb09f9805e8522415148378d632d23158acd922d2db9bf2087ace2` |
| Platform | `linux/amd64` |
| CUDA reference | `nvidia/cuda:13.0.2-base-ubuntu24.04` |
| Pinned CUDA platform manifest | `nvidia/cuda@sha256:605fb0c8acf8674e164d822da8a8521f3a655056e569f0899e72ae940e1fe7dc` |
| ROS build argument | `ROS_DISTRO=jazzy` |
| Mode | Development, with the mounted workspace built via colcon during autolaunch. `DOCKER_IMAGE_BUILD_MODE=dev` is the Compose tag discriminator. |
| Missing production tag | `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev` |

The earlier configuration ID `sha256:191cbf60c589dac0167518e4d3c0fa3b07807eac4d3d4df0b5d25bc003a9e1` remains absent from the accessible daemon. The production-tag manifest still returns `not found`. Buildx confirmed the pinned base resolves as Linux/amd64 and remains the amd64 child of the base tag. The accepted recovery scan covers the approved Harbor repository. Recheck the production tag and any maintainer-provided recovery location before build execution.
