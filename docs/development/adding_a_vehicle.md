# Adding a Vehicle Type, Unit, or Platform

This guide shows how to add a new airframe to AirStack at each level of the vehicle hierarchy ([fleets guide](fleets.md#the-hierarchy)): a **platform class** is code (interface, controller — `px4_multirotor` is the only one today), a **vehicle type** is data (`config/vehicles/<name>/vehicle.yaml`), and a **vehicle unit** is one serial number's calibration overlay. A fleet entry binds all three to a robot instance. The three asks are very different sizes: a new type is a YAML file, a new unit is a gitignored directory, a new compute platform is a Docker build chain.

The canonical schema reference is [`config/vehicles/README.md`](../../config/vehicles/README.md) — this guide doesn't restate it.

## A. Adding a new vehicle type

The common case: a different airframe or sensor suite on the existing `px4_multirotor` platform.

1. **Copy the reference type.** `quad_default` (the Pegasus Iris with stereo camera + 3D lidar) is the only in-tree type and the template:

    ```bash
    cp -r config/vehicles/quad_default config/vehicles/my_quad
    ```

2. **Edit `config/vehicles/my_quad/vehicle.yaml`** per the [schema README](../../config/vehicles/README.md): set `name:`, keep `platform: px4_multirotor` (the only platform today), point `airframe.base_urdf:` at your URDF (package-relative, exactly what `URDF_FILE` carries — the file must exist under the robot description packages), and declare each sensor as one `{type, id, frame, driver, sim}` entry. `sim_asset:` names the sim asset — but note vehicles are **pass-through in sim today**: [`fleet_spawn.py`](https://github.com/castacks/AirStack/blob/develop/simulation/isaac-sim/launch_scripts/fleet_spawn.py) spawns the Pegasus Iris asset for every `px4_multirotor` regardless. What *does* take effect per vehicle is the sensor list: any `lidar*`-typed entry enables the RTX lidar subgraph for that robot, any `stereo_cam` entry the camera subgraph (the per-vehicle `ENABLE_LIDAR` equivalent).

3. **Reference it from a fleet file.** Copy `config/fleets/sim_one_default.yaml` and change the vehicle binding:

    ```yaml
    defaults: {vehicle: my_quad, stack: stacks/full_default}
    robots:
      robot_1: {spawn: [0, 0, 0.07]}
    sim: {scene: default}
    network: {domain_policy: auto, gossip_domain: 99}
    ```

4. **Validate before launching.** The resolver names schema errors (unknown vehicle, missing `airframe.base_urdf`, unknown keys):

    ```bash
    python3 tools/fleet/resolve_fleet.py config/fleets/my_fleet.yaml --validate   # "OK: 1 robot(s), homogeneous fleet"
    python3 tools/fleet/resolve_fleet.py config/fleets/my_fleet.yaml --table      # ROBOT/DOMAIN/VEHICLE/STACK/ENTRY/HOSTS/SPAWN
    airstack fleet list
    ```

5. **Launch and verify in the container.** `airstack up --fleet my_fleet --sim isaac`, then `airstack ready`. Inside the robot container, [`tools/fleet/resolve_fleet.py`](https://github.com/castacks/AirStack/blob/develop/tools/fleet/resolve_fleet.py) has resolved the whole entry from `FLEET_CONFIG_FILE` (called by `robot/docker/.bashrc`); confirm the exports:

    ```bash
    docker exec airstack-robot-desktop-1 bash -c 'echo $VEHICLE $URDF_FILE'
    # my_quad robot_descriptions/.../my_urdf.urdf
    ```

Caveats worth knowing: an explicitly set env var wins over the resolver per variable (`URDF_FILE`, `ROBOT_NAME`, …), and `airstack up --sim airsim` keeps exporting ms-airsim's reduced stereo-only URDF regardless of the manifest. Vehicle types beyond trunk are intended to arrive as data modules (`type: data`, RFC #379) rather than commits to `config/vehicles/`.

## B. Adding a new unit of an existing type

A **unit** is one physical serial number whose calibration drifts and gets re-measured — it never touches the shared type. Full story: [`config/local/README.md`](../../config/local/README.md).

1. **Create the calibration overlay** on the machine that flies (or resolves) that airframe — `config/local/` is gitignored, per-machine:

    ```bash
    mkdir -p config/local/calibration/SN-0042
    # drop the intrinsics/extrinsics files your drivers consume — no enforced layout yet
    ```

2. **Bind it in the fleet entry:**

    ```yaml
    robots:
      robot_1: {vehicle: quad_default, unit: SN-0042}
    ```

3. **Consume it via `CALIBRATION_DIR`.** The resolver exports `CALIBRATION_DIR=/root/AirStack/config/local/calibration/SN-0042` into the robot container (`config/` is bind-mounted read-only at `/root/AirStack/config`); point driver configs at `$CALIBRATION_DIR`. It is empty when the fleet entry declares no `unit:`. Verify:

    ```bash
    python3 tools/fleet/resolve_fleet.py config/fleets/my_fleet.yaml --robot robot_1 | grep CALIBRATION_DIR
    ```

Recalibrating in the field writes into the unit directory — never into `config/vehicles/<name>/`.

## C. Adding a new compute platform

This is a much bigger lift than A or B: a platform is a **Docker compose profile + build-arg chain**, and there are exactly two non-desktop precedents — Jetson L4T (`l4t` profile) and ModalAI VOXL 2 (`voxl` profile), both in [`robot/docker/docker-compose.yaml`](https://github.com/castacks/AirStack/blob/develop/robot/docker/docker-compose.yaml). Read [Docker build profiles](intermediate/docker-build-profiles.md) and the [`docker-build-profiles` skill](../../.agents/skills/docker-build-profiles/SKILL.md) first; current status of every platform is in [Supported Platforms](../real_world/supported_platforms.md).

1. **Add a service block** in `robot/docker/docker-compose.yaml` under its own profile, extending `robot_base` from `robot-base-docker-compose.yaml`. Everything builds from the single `Dockerfile.robot`; the variant is selected by `build.args`: `BASE_IMAGE`, `ROS_DISTRO: jazzy`, `REAL_ROBOT: true`, `TARGET_ARCH: aarch64` (both hardware precedents), and a **quoted** `PYTHON_VERSION` when needed (`"3.12"` — unquoted YAML parses it as float `3.1`).

2. **Study the precedent closest to your board.** L4T is the full worked example: an intermediate `robot-l4t-stack-base` image (`Dockerfile.l4t-stack-base` on `dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04`) feeds `Dockerfile.robot` as `BASE_IMAGE`, both with `network: host` under `build:` (a Jetson build workaround), plus `runtime: nvidia`, `network_mode: host`, and a companion `zed-l4t` driver service. VOXL is the minimal CUDA-less variant: `BASE_IMAGE: ubuntu:24.04`, `deploy: !reset {}` to drop the GPU reservation, and a compute-constrained default of `AIRSTACK_STACK_DIR=.../stacks/lite_default`.

3. **Set the runtime environment** the way both precedents do: `ROBOT_NAME_SOURCE=hostname` (real robots resolve identity from the host, not the container name), `LAUNCH_PACKAGE=autonomy_bringup` (no RViz), an overridable `AIRSTACK_STACK_DIR` default, and a tmux `autolaunch ... sim:=false` command.

4. **Give the image both cache tags.** Every service with a `build:` section lists the versioned image *and* the floating `${CACHE_TAG:-cache}_...` tag in `tags:` and `cache_from:` — skip this and CI builds of your service are always cold (see the cache section in [`AGENTS.md`](https://github.com/castacks/AirStack/blob/develop/AGENTS.md)).

5. **Build and verify:**

    ```bash
    airstack images build --profile myboard robot-myboard   # l4t builds its stack-base first this way
    airstack up robot-myboard --no-autolaunch
    docker exec airstack-robot-myboard-1 bash -c "bws && sws && ros2 node list"
    ```

Be honest about what you get: **CI covers only the desktop x86 simulation path** ([Supported Platforms — what CI covers](../real_world/supported_platforms.md#what-ci-does-and-does-not-cover)). No CI job builds or runs `l4t`, `voxl`, or your new profile — its status is whatever you verify on hardware. Add a row to the [platform matrix](../real_world/supported_platforms.md) stating exactly that.
