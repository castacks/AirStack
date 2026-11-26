# ORE-MS-OSMO-ROBOT-IMAGE-004-R1

**EXISTING IMAGE REUSE READY FOR CONTROLLED TEST**

## Image identity and selection

The successful `ore_proj` checkpoint records recreation of the robot while retaining the **existing inner v0.20.8 image**. At the reviewed `ore_proj` ref `169c83a58bbb387e9a9086ef49ca2dd5887a7eb7`, Compose resolves that image to `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev`. Earlier CPU validation names the same tag. Available deployment evidence establishes local use of the tag. It does not record its robot configuration ID, registry manifest or original acquisition method. The exact tag still returns `not found`. The ORE-MS historical local robot ID is evidence for that pod only. The `ore-proj-epoch-numpy126-1` image is the outer workspace. The separately published `ore_proj` digest in the checkpoint belongs to Isaac. [Live checkpoint](https://github.com/castacks/AirStack/blob/169c83a58bbb387e9a9086ef49ca2dd5887a7eb7/RRM-transfer/rrm-9d26a58/HANDOFF.md#L2974) · [Tag record](https://github.com/castacks/AirStack/blob/169c83a58bbb387e9a9086ef49ca2dd5887a7eb7/RRM-transfer/rrm-9d26a58/docs/scrum-8/validation.md#L56)

Recommend a controlled test of the immutable Linux/amd64 manifest behind the adjacent published 0.20.7 desktop release. Buildx resolved it and confirmed `ROS_DISTRO=jazzy`. Image history records the CUDA 13.0.2 base and installs for MAVROS, MAVROS extras, colcon, SSH, tmux and Xvfb. The tracked robot Docker directory, common Docker inputs and top-level Compose are identical between 0.20.7 release source `8aed8fc1a` and the research checkout. This supports testing the candidate. Its configuration ID `sha256:3d8fa569dfed6f7249faee9c205bc49eb44b711991db9e7c65d1fa406d46fdda` differs from the previously observed ORE-MS robot. Installed package versions, successful mounted build and live integration remain unverified.

## Critical interfaces

| Interface | Evidence and test boundary |
|---|---|
| Mounted ROS workspace | Both branches mount source and build it on autolaunch. Keep the ORE-MS checkout, mounts and development launch. The candidate's ability to build that checkout remains a runtime gate. |
| MAVROS and PX4 | Both branches scope MAVROS under `interface` and pass an empty MAVROS namespace. The domain-1/default robot contract uses local UDP 14541, simulator `172.31.0.200:14581`, target system 2 and `/robot_1/interface/mavros/{state,local_position/odom}`. The candidate package and live behavior at those endpoints remain to be checked. |
| Clock and TF | `ore_proj` documented stale TF after an Isaac clock reset. ORE-MS recovered in an earlier recreation with its qualified Isaac image and retained robot. This establishes a recovery observation for that earlier artifact. The candidate needs live clock progression and dynamic TF at representative odometry stamps. |
| Workspace entrypoint | Retain the research entrypoint and the digest-triggered no-build guard. The `ore_proj` outer entrypoint includes automatic epoch reconciliation. The controlled test uses the research branch with manual observation and separately authorized recovery. |

## Smallest reversible test configuration

Use a temporary Compose overlay outside Git to replace the robot image alone. For a future user-authorized test, disable automatic stack startup with `OSMO_AIRSTACK_UP=false` if using an OSMO task. Keep `AUTOLAUNCH=true`, one robot, the desktop/livestream profiles and the qualified Isaac digest. Once the isolated test environment is authorized, pass the overlay as an additional `-f` to `./airstack.sh up`. The stock OSMO entrypoint invokes bare `up`, so the overlay must be passed explicitly. Removing it restores the original resolution. No branch or Mac-local workflow edits were made.

```yaml
services:
  robot-desktop:
    image: airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:f2c7bce4797008bff889dbef2881fd015780c972f16b8e344c13a22bba02cbf4
    build: !reset null
```

Offline Compose resolution verified the overlay changed only the robot image and removed its build stanza. GCS remained `v0.20.8_gcs` and Isaac remained `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`. The four existing startup fixture checks passed, including digest-triggered no-build and failure without fallback. The registry checks fetched manifest/config metadata only.

## Controlled test checklist

1. Verify the robot digest and image config with the intended registry credential. In the isolated environment, record Jazzy/MAVROS versions and verify the mounted ORE-MS workspace builds and launches.
2. Run `./airstack.sh ready --json`. Record robot identity, MAVROS heartbeat/state, canonical odometry, advancing robot-domain `/clock` and dynamic TF at odometry timestamps. Confirm the observed PX4 endpoint and preserve first-start logs.
3. If stale clock/TF state is confirmed and the vehicle is grounded, disarmed and has no active mission, obtain separate authorization for the documented robot-only `docker restart airstack-robot-desktop-1`. Rerun readiness and repeat the clock, odometry and TF checks. [Manual recovery record](../robot-image-recovery/ORE_PROJ_STARTUP_TIMING_REVIEW.md)

No image build, image publication, workflow operation or service restart was performed. This result prepares a controlled test of an existing release. It establishes no runtime compatibility for that candidate.
