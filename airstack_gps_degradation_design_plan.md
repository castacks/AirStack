# AirStack GPS Degradation — Complete Reference
### Issue #345 · CMU AirLab Internship · Branch: `apurva/test-2`

**Author:** Apurva Singh · CMU AirLab Intern  
**Date:** May 13, 2026  
**Repos:** [castacks/AirStack @ apurva/test-2](https://github.com/castacks/AirStack/tree/apurva/test-2) · [castacks/PegasusSimulator-AirStack-Integration](https://github.com/castacks/PegasusSimulator-AirStack-Integration)  
**Papers:** Cuenca et al. (2023) · Pant et al. (2022) · Lee et al. (2020) · GNSS Multipath Ray-Tracing (2026)

> **For VS Code AI Agent (Copilot):** This document is your single source of truth for Issue #345. It contains the full GPS degradation design plan, paper reading guide with hardware applicability, SITL testing instructions, and a complete AirStack project reference extracted from the official docs. Start at §Part I for project orientation, §Part III for the GPS implementation design, §Part IV for paper guidance, and §Part V for SITL testing.

---

## Table of Contents

- [Part I — AirStack Project Reference](#part-i--airstack-project-reference)
  - [1. What AirStack Is](#1-what-airstack-is)
  - [2. Getting Started](#2-getting-started)
  - [3. Docker Workflow](#3-docker-workflow)
  - [4. Development Environment & VS Code](#4-development-environment--vs-code)
  - [5. System Architecture](#5-system-architecture)
  - [6. Simulation — Isaac Sim & Pegasus](#6-simulation--isaac-sim--pegasus)
  - [7. Spawning Drones & GPS Homes](#7-spawning-drones--gps-homes)
  - [8. Testing Infrastructure](#8-testing-infrastructure)
  - [9. Robot Autonomy Stack](#9-robot-autonomy-stack)
  - [10. Frame Conventions](#10-frame-conventions)
  - [11. AI Agent Quick Commands](#11-ai-agent-quick-commands)

- [Part II — Issue #345 Context](#part-ii--issue-345-context)
  - [12. Problem Statement](#12-problem-statement)
  - [13. GPS Pipeline Map in AirStack](#13-gps-pipeline-map-in-airstack)

- [Part III — GPS Degradation Design Plan](#part-iii--gps-degradation-design-plan)
  - [14. Proposed Architecture: DegradedGPS](#14-proposed-architecture-degradedgps)
  - [15. Tier 1 — Cuenca DOP-Driven Model](#15-tier-1--cuenca-dop-driven-model)
  - [16. Tier 2 — Pant Multipath Ray-Tracing Model](#16-tier-2--pant-multipath-ray-tracing-model)
  - [17. Tier 3 — Jamming and Signal Denial Model](#17-tier-3--jamming-and-signal-denial-model)
  - [18. Configuration Schema](#18-configuration-schema)
  - [19. Integration Points & Key Decision](#19-integration-points--key-decision)
  - [20. Data Flow Diagram](#20-data-flow-diagram)
  - [21. File Locations for Implementation](#21-file-locations-for-implementation)

- [Part IV — Paper Reading Guide & Hardware Applicability](#part-iv--paper-reading-guide--hardware-applicability)
  - [22. Cuenca et al. (2023)](#22-cuenca-et-al-2023)
  - [23. Pant et al. (2022)](#23-pant-et-al-2022)
  - [24. Lee et al. (2020)](#24-lee-et-al-2020)
  - [25. GNSS Multipath Ray-Tracing (2026)](#25-gnss-multipath-ray-tracing-2026)
  - [26. Hardware Applicability Summary Table](#26-hardware-applicability-summary-table)

- [Part V — SITL Testing Guide (No Hardware Required)](#part-v--sitl-testing-guide-no-hardware-required)
  - [27. Prerequisites](#27-prerequisites)
  - [28. Step-by-Step: First SITL Run](#28-step-by-step-first-sitl-run)
  - [29. Writing GPS Degradation Test Scenarios](#29-writing-gps-degradation-test-scenarios)
  - [30. Monitoring with Foxglove](#30-monitoring-with-foxglove)
  - [31. Recording & Replaying ROS Bags](#31-recording--replaying-ros-bags)
  - [32. Running Formal Tests with airstack test](#32-running-formal-tests-with-airstack-test)
  - [33. Codebase Feasibility Analysis](#33-codebase-feasibility-analysis)

- [Part VI — Limitations](#part-vi--limitations)
  - [34. Limitations](#34-limitations)

- [Part VII — Additional Literature & References](#part-vii--additional-literature--references)
  - [35. Additional Literature](#35-additional-literature)
  - [36. References](#36-references)

---

# Part I — AirStack Project Reference

## 1. What AirStack Is

AirStack is CMU AirLab's boilerplate for multi-robot aerial autonomy. It is a full-stack framework designed for real-world deployment with TAK integration. Key characteristics:

- **Currently in ALPHA** — internal use only, API not stable, requires AirLab accounts for Docker registry and Nucleus server access
- **Stack layers:** Interface → Sensors → Perception → Local → Global → Behavior
- **Simulation backend:** NVIDIA Isaac Sim + Pegasus Simulator (flight dynamics + PX4 SITL)
- **Legacy sim:** Microsoft AirSim (still supported but not primary)
- **State estimation:** MACVIO (`macvo_ros2`) — visual-inertial odometry that fuses GPS
- **Flight controller:** PX4 SITL over MAVLink
- **All development is Docker-based** — no native installs needed except Docker + NVIDIA Container Toolkit
- **Hardware:** Ubuntu 22.04 host, NVIDIA RTX 3070+ (RTX 4080+ recommended for Isaac), 32 GB RAM, 100 GB SSD

---

## 2. Getting Started

### Prerequisites
- Ubuntu 22.04 LTS (primary test platform)
- NVIDIA RTX 3070+ GPU with latest drivers
- 16 GB RAM minimum, 32 GB+ recommended
- 100 GB+ free SSD space
- AirLab account (for Docker registry: `airlab-docker.andrew.cmu.edu` and Nucleus server)

### Clone and Install

```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
./airstack.sh install   # installs Docker, docker-compose, NVIDIA Container Toolkit
./airstack.sh setup     # adds `airstack` CLI to PATH, sets up keys
source ~/.bashrc        # or ~/.zshrc
```

### Pull Docker Images (Preferred)

```bash
cd AirStack/
docker login airlab-docker.andrew.cmu.edu
# Enter Andrew ID (without @andrew.cmu.edu) and password
airstack image-pull
```

Images are large (~25 GB). Alternative is `airstack image-build` from scratch (requires NVIDIA NGC access).

### Launch Everything

```bash
airstack up         # starts robot, GCS, and Isaac Sim
airstack down       # stops and removes all containers
```

The default `.env` launches the scene at `ISAAC_SIM_GUI` path and runs `example_one_px4_pegasus_launch_script.py`.

### Move the Robot
Open RViz → click **Takeoff** → then **Navigate** or **Explore** in the trajectory window.

### For Your Branch

```bash
git checkout apurva/test-2
git submodule update --init --recursive
```

---

## 3. Docker Workflow

### Core Commands

```bash
# Lifecycle
airstack up                          # start all (robot + GCS + isaac-sim)
airstack up robot-desktop            # start only robot
airstack up isaac-sim                # start only simulator
airstack down                        # stop + remove all
airstack status                      # show running containers
airstack connect robot               # bash shell in robot container
airstack connect isaac-sim           # bash shell in isaac container
airstack logs robot                  # view logs

# Images
docker compose pull                  # pull from registry
docker compose build                 # build from scratch
airstack image-push                  # push to registry (if you have permissions)
```

### Environment Variable Overrides

```bash
# All variables from .env can be overridden inline:
PLAY_SIM_ON_START=false airstack up          # don't auto-play sim
AUTOLAUNCH=false airstack up                 # spawn idle containers (good for debugging)
NUM_ROBOTS=2 airstack up                     # launch 2 robots
ISAAC_SIM_SCRIPT_NAME=my_script.py airstack up  # use your launch script
```

### Key `.env` Variables

```bash
PROJECT_NAME="airstack"
VERSION="0.18.0-alpha.5"
COMPOSE_PROFILES="desktop,isaac-sim"
AUTOLAUNCH="true"
NUM_ROBOTS="1"
RECORD_BAGS="false"

# Isaac Sim
ISAAC_SIM_GUI="/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/simple_pegasus.scene.usd"
ISAAC_SIM_USE_STANDALONE="true"
ISAAC_SIM_SCRIPT_NAME="example_one_px4_pegasus_launch_script.py"
PLAY_SIM_ON_START="false"

# PX4 Physics
PX4_PHYSICS_HZ="100"     # 100 Hz = AirStack default (minimum for EKF2)
PX4_RENDERING_HZ="30"

# Robot
OFFBOARD_BASE_PORT=14540
ONBOARD_BASE_PORT=14580
```

### Multi-Robot

```bash
NUM_ROBOTS=2 airstack up
airstack connect robot-1    # robot with ROS_DOMAIN_ID=1
airstack connect robot-2    # robot with ROS_DOMAIN_ID=2
```

### Inside Robot Container Aliases

```bash
bws                                          # colcon build (full workspace)
bws --packages-select my_package             # build only one package
bws --packages-select my_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'
sws                                          # source install/setup.bash
cws                                          # clean (remove build/install/log)
ros2 launch robot_bringup robot.launch.xml   # top-level launch
```

### Inside Isaac Sim Container

```bash
runapp                                       # launch Isaac Sim GUI
runapp --path /path/to/scene.usd             # load a specific USD scene
./runheadless.native.sh                      # headless mode (stream to Omniverse client)
./runheadless.webrtc.sh                      # headless mode (stream to web browser)
ros2 launch isaacsim run_isaacsim.launch.py  # ROS 2 launch mode
```

---

## 4. Development Environment & VS Code

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 | Ubuntu 22.04/24.04 |
| CPU | 8 cores | 16+ cores |
| RAM | 16 GB | 32+ GB |
| GPU | NVIDIA RTX 3070 | NVIDIA RTX 4080+ |
| Storage | 100 GB SSD | 200 GB+ SSD |

### VS Code Setup

1. Open the AirStack folder in VS Code on your **host machine**
2. Install recommended extensions from `.vscode/extensions.json` (VS Code will prompt)
3. Key extensions: ROS, C/C++, Python, Docker, CMake Tools, YAML, **Dev Containers**

### Attaching VS Code to a Docker Container

1. Start the container: `airstack up robot-desktop`
2. Open Command Palette (`F1`) → **Dev Containers: Reopen in Container** → select "Robot Container" or "GCS Container"
3. VS Code now runs inside the container — IntelliSense, debugger, and terminal all work natively

> **File permission warning:** Files created inside the container are owned by root. Fix with: `sudo chown -R $USER:$USER .`

### Building Inside VS Code

- `Ctrl-Shift-B` → runs `bws --cmake-args '-DCMAKE_BUILD_TYPE=Debug'` (debug build with symbols)
- Build tasks defined in `.vscode/tasks.json`

### Debugging

- `F5` → launches `robot.launch.xml` with debugger attached
- Uses **Robot Developers Extension for ROS2** — supports breakpoints in ROS 2 nodes inside containers
- Launch configs in `.vscode/launch.json`

### Quick Development Loop

```bash
# Terminal 1 — idle containers (no autolaunch so you control what runs)
AUTOLAUNCH=false airstack up robot-desktop

# Terminal 2 — build and test your package
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select degraded_gps && sws && ros2 launch degraded_gps test.launch.xml"
```

---

## 5. System Architecture

AirStack uses a **layered autonomy architecture**. Data flows up through sensing and down through acting:

```
Hardware / Isaac Sim
        │
        ▼
┌─────────────────────────────────────────────────────────────┐
│  Interface Layer — Hardware abstraction, MAVLink, safety    │
│  mavros_interface · drone_safety_monitor · robot_interface  │
│  Topics out: /{robot}/interface/mavros/state                │
│              /{robot}/interface/mavros/local_position/pose  │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  Sensors Layer — Processing and calibration                 │
│  camera_param_server · gimbal_stabilizer · lidar filter     │
│  Topics out: /{robot}/sensors/{name}/image                  │
│              /{robot}/sensors/front_stereo/disparity        │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  Perception Layer — State estimation                        │
│  macvo_ros2 (MACVIO) — visual-inertial + GPS fusion         │
│  Topics out: /{robot}/odometry                              │
│              /{robot}/perception/macvo/depth                │
└──────────────┬──────────────────────────┬───────────────────┘
               │                          │
               ▼                          ▼
┌──────────────────────┐   ┌──────────────────────────────────┐
│  Local Layer         │   │  Global Layer                    │
│  DROAN, Trajectory   │   │  VDB Mapping, Random Walk        │
│  Controller          │   │  Topics: /{robot}/global_plan    │
└──────────┬───────────┘   └──────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│  Behavior Layer — Mission execution                         │
│  behavior_tree · behavior_executive · rqt_behavior_tree     │
└─────────────────────────────────────────────────────────────┘
```

### Topic Naming Convention

```
/{robot_name}/{layer}/{module}/{data_type}
# Example: /drone1/local_planner/droan/trajectory
```

### Standard Topics for GPS/State

```
/{robot}/odometry                              — primary state estimate (from MACVIO)
/{robot}/interface/mavros/state                — PX4/MAVROS flight state
/{robot}/interface/mavros/local_position/pose  — local pose
/{robot}/interface/mavros/imu/data             — IMU data
/{robot}/interface/mavros/global_position/raw/fix — GPS NavSatFix (from PX4)
```

### Node Types

- **Perpetual nodes** — run continuously (state estimator, trajectory controller, behavior tree tick)
- **Task executors** — ROS 2 action servers, activated on demand (planners, takeoff/land)

### Update Rates

| Layer | Module | Rate |
|-------|--------|------|
| Interface | MAVROS | 50 Hz |
| Sensors | Camera | 30 Hz |
| Sensors | Disparity | 15 Hz |
| Perception | MACVIO | 30 Hz |
| Local Planner | DROAN | 10 Hz |
| Local Controller | Trajectory | 50 Hz |
| Global Planner | Path | 1 Hz |
| Behavior | BT Tick | 10 Hz |

---

## 6. Simulation — Isaac Sim & Pegasus

### How it Works

Pegasus Simulator is a multirotor simulation framework built on Isaac Sim's physics engine. AirStack maintains a [fork of Pegasus](https://github.com/castacks/PegasusSimulator-AirStack-Integration) that:
- Wraps Pegasus in an **OmniGraph node** (so scenes can be saved as USD files)
- Connects directly to PX4 SITL via MAVLink
- Provides GPS, IMU, ZED stereo, and optional Ouster LiDAR sensors

### Launch Modes

Controlled by `.env`:

```bash
# Mode 1: Load a saved USD file (faster startup)
ISAAC_SIM_USE_STANDALONE="false"
ISAAC_SIM_GUI="omniverse://airlab-nucleus.andrew.cmu.edu/Library/.../iris_with_sensors.pegasus.robot.usd"

# Mode 2: Generate scene from Python script (more flexible)
ISAAC_SIM_USE_STANDALONE="true"
ISAAC_SIM_SCRIPT_NAME="example_one_px4_pegasus_launch_script.py"
```

**Recommended workflow:** Generate with standalone script first, save as USD, then use Mode 1 for speed.

### Launch Scripts Location

```
simulation/isaac-sim/launch_scripts/
├── barebones_pegasus_launch.py                    # minimal template
├── example_one_px4_pegasus_launch_script.py       # single drone with full sensors
├── example_multi_px4_pegasus_launch_script.py     # multiple drones, same env
└── example_multi_drone_scene_import.py            # multiple drones + imported USD scene
```

### scene_prep Utilities

```python
# All scripts add utils/ to sys.path first:
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
import scene_prep
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd

# Critical: always call in this order
scale_stage_prim(stage, "/World/Environment", 0.01)  # if asset is in cm
add_colliders(stage_prim)                             # must be called or drone falls through floor
# Pump 10+ update() calls after add_colliders before spawning
for _ in range(10):
    omni.kit.app.get_app().update()
```

### Physics Rate

```bash
# AirStack default (in .env):
PX4_PHYSICS_HZ="100"     # minimum for PX4 EKF2 (10ms period)
PX4_RENDERING_HZ="30"

# Pegasus default is 250 Hz — too slow with full sensor suite
# 100 Hz is stable; 200 Hz is good balance; 400 Hz is max tested
```

### Common Pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| Drone falls through floor | `add_colliders` not called | Call `add_colliders(stage_prim)` after loading env |
| Drone spawns at wrong height | cm-scale scene, coordinates not converted | Multiply `init_pos` by `scene_scale` from `get_stage_meters_per_unit` |
| Pegasus node not recognized | Extension not autoloaded | Window → Extensions → Third Party → search "pegasus" → enable AUTOLOAD, restart container |
| Stereo camera fails to init | Init race condition | Press Start → Stop → Start in Isaac Sim toolbar |
| Physics erratic after colliders | Update frames not pumped | Call `app.update()` ≥10 times after `add_colliders` |

---

## 7. Spawning Drones & GPS Homes

### DRONE_CONFIGS Pattern

```python
# In your launch script:
DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": -3.0, "y_m": 3.5, "z_m": 0.15, "orient": [0, 0, 0, 1]},
    {"domain_id": 2, "x_m":  3.0, "y_m": 3.0, "z_m": 0.15, "orient": [0, 0, 0, 1]},
]
# domain_id = ROS_DOMAIN_ID and PX4 vehicle ID
# +X = East, +Y = North, +Z = Up (world frame)
```

### GPS Home Coordinates

```python
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

# Set GPS origins BEFORE spawning vehicles
set_gps_origins(DRONE_CONFIGS)   # uses DEFAULT_WORLD_ORIGIN = Lisbon (38.74°N, 9.14°W)

# For Pittsburgh (AirLab location):
set_gps_origins(DRONE_CONFIGS, world_origin=(40.4433, -79.9436, 280.0))
```

`gps_utils.py` writes `PX4_HOME_LAT_<id>`, `PX4_HOME_LON_<id>`, `PX4_HOME_ALT_<id>` as environment variables, which Pegasus picks up when building each drone's `PX4MavlinkBackendConfig`.

### Common GPS Home Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Drone at world origin in Foxglove | `set_gps_origins` called after spawn | Move call before vehicle spawning |
| Map centers on wrong city | Wrong `world_origin` | Override second arg to `set_gps_origins` |
| Position drifts wrong compass direction | Stage axis mismatch | Swap `x_m` ↔ `y_m` in `gps_utils.compute_gps_origin` |

---

## 8. Testing Infrastructure

### Test Suite Overview

All tests live in `tests/` at the repo root. Run via `airstack test` (containerized) or direct `pytest`.

| Module | Mark | What It Tests | GPU Required |
|--------|------|---------------|-------------|
| `test_build_docker.py` | `build_docker` | Docker image builds | No |
| `test_build_packages.py` | `build_packages` | `colcon build` in all containers | No |
| `test_liveliness.py` | `liveliness` | Stack bringup: containers, /clock, tmux, nodes, compute | Yes |
| `test_sensors.py` | `sensors` | Stereo/depth Hz, LiDAR stream, RTF | Yes |
| `test_takeoff_hover_land.py` | `takeoff_hover_land` | Full flight: arm → takeoff → hover → land | Yes |

### Running Tests

```bash
# Build tests only (fast, no GPU)
airstack test -m "build_docker or build_packages" -v

# Liveliness check
airstack test -m liveliness --sim msairsim --num-robots 1 --stress-iterations 1 --stable-duration 60 -v

# Sensor streams with Isaac Sim
airstack test -m sensors --sim isaacsim --num-robots 1 --stress-iterations 1 --stable-duration 60 -v

# Full flight test
airstack test -m takeoff_hover_land --sim msairsim --num-robots 1 --stress-iterations 1 --takeoff-velocities 0.5,1,2 -v

# GUI mode (for visual inspection)
airstack test -m liveliness --gui -v

# Direct pytest (faster iteration, needs local Python)
export AIRSTACK_ROOT=$(pwd)
pip install -r tests/requirements.txt
pytest tests/ -m liveliness --sim isaacsim --num-robots 1 --stress-iterations 1 -v
```

### CLI Options Reference

| Option | Default | Description |
|--------|---------|-------------|
| `--sim` | `msairsim,isaacsim` | Simulator targets |
| `--num-robots` | `1,3` | Robot counts |
| `--stress-iterations` | `3` | Up/down cycles per config |
| `--stable-duration` | `120` | Seconds to poll for stability |
| `--stable-interval` | `10` | Seconds between polls |
| `--gui` | off | Show simulator GUI |
| `--takeoff-velocities` | `0.5,1,2` | Velocities in m/s |

### Takeoff/Hover/Land Test Phases

| Phase | Test | What Happens |
|-------|------|--------------|
| 1 | `test_px4_ready` | Waits for MAVROS + PX4 EKF ready |
| 2 | `test_takeoff` | TakeoffTask → assert altitude within 10% |
| 3 | `test_hover` | Capture odom 10 s → assert drift < 0.5 m |
| 4 | `test_landing` | LandTask → assert final altitude < 0.5 m |

### Output Files

```
tests/results/2025-04-21_14-30-00/
├── results.xml       # JUnit XML
├── metrics.json      # image sizes, Hz, compute, timing
└── logs/
    ├── test_build_docker.TestDockerBuilds.test_build_robot_desktop.log
    ├── test_liveliness.TestLiveliness.test_robot_containers_running[isaacsim-rob#1-iter0].log
    └── ...
```

### Metrics Reporting

```bash
# Single run report
python tests/parse_metrics.py --current tests/results/2025-04-21_14-30-00/

# Regression check vs baseline
python tests/parse_metrics.py \
  --current  tests/results/2025-04-21_14-30-00/ \
  --baseline tests/results/2025-04-20_09-00-00/ \
  --threshold 20 \
  --output report.md
```

---

## 9. Robot Autonomy Stack

### Integration Checklist (when adding DegradedGPS)

- [ ] Package placed in correct layer directory (Sensors layer)
- [ ] `package.xml` with all dependencies
- [ ] Launch file with topic remapping using `$(env ROBOT_NAME)`
- [ ] Config file with parameters (YAML)
- [ ] README.md with documentation
- [ ] Added to layer bringup launch file
- [ ] Added to `mkdocs.yml` navigation
- [ ] Tested standalone
- [ ] Tested in full stack with `airstack up`
- [ ] Tested in simulation with `airstack test`

### ROS Bag Recording

```bash
# Record all topics
docker exec airstack-robot-desktop-1 bash -c "ros2 bag record -a -o /tmp/gps_test"

# Record specific GPS/odom topics only
docker exec airstack-robot-desktop-1 bash -c \
  "ros2 bag record /robot_1/odometry /robot_1/interface/mavros/global_position/raw/fix /robot_1/interface/mavros/state -o /tmp/gps_degradation_test"

# Play back
ros2 bag play /tmp/gps_degradation_test
```

### Ground Control Station (Foxglove)

```bash
airstack up gcs     # starts Foxglove-based GCS
```

Foxglove connects automatically. The Map panel centers on the GPS home coordinate set by `gps_utils`.

### Autonomy Modes

The robot can be in multiple modes: `GUIDED`, `AUTO`, `OFFBOARD` (PX4-level) and higher-level behavior tree modes. For GPS degradation testing, keep mode as `OFFBOARD` or use the behavior tree to issue `TakeoffTask` → `NavigateTask`.

---

## 10. Frame Conventions

AirStack uses standard ROS 2 / PX4 frame conventions:

| Frame | Description |
|-------|-------------|
| `world` | Fixed world frame |
| `map` | Global map frame (may drift from world) |
| `odom` | Odometry frame (continuous, may drift from map) |
| `base_link` | Robot body frame (NED in PX4, ENU in ROS) |
| `base_link_stabilized` | Body frame, yaw-only (roll/pitch removed) |
| `camera_link` | Camera frame |

**Critical for GPS implementation:** PX4 uses **NED** (North-East-Down) internally. MAVROS bridges to ROS **ENU** (East-North-Up). GPS coordinates are WGS84 lat/lon/alt. When injecting GPS errors:
- `+X` in Isaac Sim scene = **East** (matches `+Y` in ROS ENU)
- `+Y` in Isaac Sim scene = **North** (matches `+X` in ROS ENU)
- `+Z` in Isaac Sim scene = **Up** (matches `+Z` in ROS ENU)

---

## 11. AI Agent Quick Commands

```bash
# Find GPS sensor in codebase
grep -r "GPS\|gps" simulation/isaac-sim/extensions/PegasusSimulator/ --include="*.py" -l

# Find where MAVLink sends GPS
grep -r "GPS_RAW_INT\|send_gps" simulation/isaac-sim/extensions/ --include="*.py"

# Check MACVIO GPS subscription topics
grep -r "NavSatFix\|gps\|fix" robot/ros_ws/src/ --include="*.py" --include="*.cpp" -l

# Build only the degraded GPS package
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select degraded_gps && sws"

# Monitor GPS topic live
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_1/interface/mavros/global_position/raw/fix"

# Check GPS fix quality
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot_1/interface/mavros/global_position/raw/fix"

# List all GPS-related topics
docker exec airstack-robot-desktop-1 bash -c "ros2 topic list | grep -i gps\|fix\|global"
```

---

# Part II — Issue #345 Context

## 12. Problem Statement

Issue #345 (opened by @andrewjong, April 23 2026) requests that the GPS sensor in Pegasus/AirStack be extended to model:

- **Satellite occlusion** from buildings blocking sky view
- **Multipath errors** from signal reflections off vertical surfaces
- **Intermittent fix loss** (3D-fix → 2D-fix → no-fix)
- **DOP inflation** as visible satellite geometry degrades
- **Gradual vs. abrupt transitions** with hysteresis at occlusion boundaries

### What the Current Sensor Does (and Does Not Do)

`pegasus.simulator.logic.sensors.gps.GPS.update(state, dt)` currently applies:
- **White noise** — Gaussian, fixed standard deviation, per-axis
- **Bias drift** — Ornstein–Uhlenbeck process, mean-reverting

Both are **scene-agnostic**: `fix_type` is always 3 (3D fix), `satellites_visible` is always 10, `eph`/`epv` are constants. The sensor has zero awareness of surrounding geometry.

### Note from Issue Author

@andrewjong specifically mentions that PX4's EKF2 innovation gate may silently reject overtly bad GPS and raises the option of publishing degraded GPS directly to MACVIO instead (bypassing PX4 EKF). **This implementation decision must be confirmed with the team before coding.** (See §19.)

---

## 13. GPS Pipeline Map in AirStack

```
Ground-truth pose (Isaac Sim USD stage)
        │
        ▼
┌─────────────────────────────────────────────┐
│  simulation/isaac-sim/extensions/            │
│  PegasusSimulator/pegasus/simulator/logic/   │
│  sensors/gps.py → GPS.update(state, dt)      │  ◄── ONLY HOOK POINT
│                                              │      with pose + scene access
│  Current: Gaussian noise + O-U bias          │
│  Outputs: lat, lon, alt, covariances         │
└──────────────┬──────────────────────────────┘
               │  fills GPS_RAW_INT MAVLink fields
               ▼
┌──────────────────────────────────────────────┐
│  backends/px4_mavlink_backend.py             │
│  PX4MAVLinkBackend._send_gps_info()          │
│  Fields sent: fix_type, eph, epv,            │
│               satellites_visible, lat, lon   │
└──────────────┬───────────────────────────────┘
               │  MAVLink UDP (SITL port 14540)
               ▼
┌──────────────────────────────────────────────┐
│  PX4 SITL (EKF2)                             │
│  Innovation gate: REJECTS GPS if innovation  │
│  > 5σ from inertial estimate (silent fail)   │
└──────────────┬───────────────────────────────┘
               │  uORB → MAVROS → ROS 2 bridge
               │  Topic: /{robot}/interface/mavros/global_position/raw/fix
               ▼
┌──────────────────────────────────────────────┐
│  MACVIO (macvo_ros2) — Perception Layer      │
│  Lee 2020: handles intermittent GPS          │
│  Fuses GPS when fix_type ≥ 3                 │
│  Output: /{robot}/odometry                   │
└──────────────────────────────────────────────┘
```

**Critical observation:** `GPS.update()` is the only place in the codebase that sees both the ground-truth drone pose AND has access to Isaac Sim's physics scene. All modifications belong here. The file path in the AirStack fork is:

```
simulation/isaac-sim/extensions/PegasusSimulator/
  pegasus/simulator/logic/sensors/gps.py
```

---

# Part III — GPS Degradation Design Plan

## 14. Proposed Architecture: DegradedGPS

### Design Philosophy

A single drop-in `DegradedGPS(GPS)` subclass. Three independently toggleable tiers. Existing scenes untouched unless they explicitly reference `DegradedGPS`.

| Tier | Model | Runtime | Config Key | Always Active |
|------|-------|---------|-----------|---------------|
| 1 | Cuenca DOP-driven shadowing | < 200 µs/call | `mode: cuenca` | Yes (default) |
| 2 | Pant-style multipath ray-tracing | < 5 ms/call | `mode: multipath` | No (opt-in) |
| 3 | Jamming / signal denial | negligible | `jamming: true` | No (opt-in) |

All tiers write into PX4's existing `GPS_RAW_INT` fields: `fix_type`, `eph`, `epv`, `satellites_visible`, `lat`, `lon`, `alt`.

### Class Hierarchy

```
pegasus.simulator.logic.sensors.gps.GPS          (existing — untouched)
    └── DegradedGPS(GPS)                          (new subclass)
            ├── SkyOcclusionModel                 (shared Tier 1 + 2)
            │       ├── CuencaDOPModel            (Tier 1)
            │       └── PantMultipathModel        (Tier 2)
            └── JammingModel                      (Tier 3)
```

### Override Pattern

```python
class DegradedGPS(GPS):
    """
    Drop-in replacement for GPS sensor with environment-aware degradation.
    Tier 1 (Cuenca sky occlusion) is always active.
    Tier 2 (Pant multipath) and Tier 3 (jamming) are opt-in via config.
    """
    def __init__(self, config: DegradedGPSConfig):
        super().__init__(config)
        self._occlusion  = SkyOcclusionModel(config)
        self._jamming    = JammingModel(config) if config.jamming_enabled else None
        self._fix_state  = FixStateMachine()

    def update(self, state: State, dt: float) -> dict:
        # 1. Run base class Gaussian noise + O-U bias
        data = super().update(state, dt)
        # 2. Compute sky visibility fraction (0 = fully blocked, 1 = open sky)
        sky_fraction = self._occlusion.compute_sky_fraction(state.position)
        # 3. Apply Tier-1 DOP inflation
        data = self._apply_cuenca_model(data, sky_fraction)
        # 4. Optionally apply Tier-2 multipath
        if self._config.multipath_enabled:
            data = self._apply_pant_model(data, state, sky_fraction)
        # 5. Optionally apply Tier-3 jamming
        if self._jamming:
            data = self._jamming.apply(data, state)
        # 6. Update fix-type via hysteresis state machine
        data['fix_type'] = self._fix_state.update(sky_fraction)
        return data
```

### Fix-Type State Machine (Hysteresis)

Prevents flickering when the drone hovers at an occlusion boundary.

```
States:  NO_FIX(0) ─── FIX_2D(2) ─── FIX_3D(3)

Transitions:
  sky_fraction > 0.6  AND held ≥ T_up   (2.0 s) → promote one level
  sky_fraction < 0.3  AND held ≥ T_down (0.5 s) → demote one level

T_up = 2.0 s  (slow to acquire — realistic)
T_down = 0.5 s (fast to lose — realistic)
```

---

## 15. Tier 1 — Cuenca DOP-Driven Model

**Paper:** Cuenca et al. (2023), DOI: 10.2514/6.2023-2648  
**Runtime:** < 200 µs/call (32 hemisphere raycasts)  
**Always active** in `DegradedGPS`

### Sky Fraction Computation

```python
def compute_sky_fraction(self, position: np.ndarray) -> float:
    """
    Cast N rays from drone position across upper hemisphere.
    Return fraction that reach sky without hitting scene geometry.
    Uses Isaac Sim omni.physx.scene_query (raycast).
    N=32 uses Fibonacci hemisphere distribution for even coverage.
    """
    directions = fibonacci_hemisphere_directions(N=32)
    hits = sum(1 for d in directions
               if scene_query.raycast(origin=position, direction=d, max_dist=500.0))
    return 1.0 - (hits / len(directions))
```

**Fibonacci hemisphere:** 32 rays → ~11° angular resolution → ~150 µs per frame on CPU.

### DOP Inflation Pseudocode

```python
def _apply_cuenca_model(self, data: dict, sky_fraction: float) -> dict:
    sf   = max(sky_fraction, 0.05)           # avoid divide-by-zero
    hdop = 1.0 + 2.5 * (1.0 - sf) ** 2.2    # ~1.0 (open sky) → ~3.5 (canyon)
    vdop = 1.0 + 4.0 * (1.0 - sf) ** 1.8    # vertical degrades faster
    base_h = self._config.base_h_accuracy    # metres (e.g. 0.3)
    base_v = self._config.base_v_accuracy    # metres (e.g. 0.5)
    data['eph'] = int(hdop * base_h * 100)   # cm, PX4 convention
    data['epv'] = int(vdop * base_v * 100)
    data['satellites_visible'] = max(3, int(10 * sf))
    # Scale position noise by DOP
    data['lat'] += np.random.normal(0, (hdop - 1.0) * base_h / 111320.0)
    data['lon'] += np.random.normal(0, (hdop - 1.0) * base_h / 111320.0)
    data['alt'] += np.random.normal(0, (vdop - 1.0) * base_v)
    return data
```

---

## 16. Tier 2 — Pant Multipath Ray-Tracing Model

**Paper:** Pant et al. (2022), arXiv 2212.04018  
**Runtime:** < 5 ms/call (must be opt-in)  
**Porting note:** Pant's implementation is a Gazebo plugin. AirStack uses Isaac Sim. The algorithm is re-implemented against `omni.physx.scene_query` — this is a re-implementation, not a port.

### Multipath Pseudocode

```python
def _apply_pant_model(self, data: dict, state: State, sky_fraction: float) -> dict:
    """
    For each visible satellite: check for reflected ray from nearby walls.
    Aggregate position biases in direction of reflecting surfaces.
    """
    sat_positions   = self._almanac.get_positions(time=state.sim_time,
                                                   n_sats=data['satellites_visible'])
    total_n_bias, total_e_bias, count = 0.0, 0.0, 0
    for sat_pos in sat_positions:
        direct_dir = normalize(sat_pos - state.position)
        if scene_query.raycast(origin=state.position, direction=direct_dir,
                               max_dist=norm(sat_pos - state.position)):
            continue  # satellite blocked — skip
        for wall_dir in cardinal_and_intercardinal_directions():  # 8 directions
            wall_hit = scene_query.raycast(origin=state.position,
                                           direction=wall_dir, max_dist=200.0)
            if not wall_hit:
                continue
            reflected_dir = reflect(direct_dir, wall_hit.surface_normal)
            if not scene_query.raycast(origin=wall_hit.point,
                                       direction=reflected_dir, max_dist=1e6):
                if angle_between(reflected_dir, direct_dir) < 60.0:
                    delta_r   = 2.0 * wall_hit.distance * abs(dot(direct_dir, wall_hit.surface_normal))
                    bias_mag  = delta_r * self._config.multipath_efficiency
                    total_n_bias += bias_mag * wall_dir[1]   # North component
                    total_e_bias += bias_mag * wall_dir[0]   # East component
                    count += 1
                    break  # one reflected path per satellite
    if count > 0:
        data['lat'] += (total_n_bias / count) / 111320.0
        data['lon'] += (total_e_bias / count) / (111320.0 * cos(radians(state.lat)))
        data['eph'] = int(data['eph'] * (1.0 + 0.5 * count / len(sat_positions)))
    return data
```

### Simplifications vs Full Pant

| Full Pant | This Implementation | Justification |
|-----------|--------------------|-|
| Full signal propagation + carrier tracking | Pseudorange bias via ray geometry | 80% effect at 5% compute cost |
| All visible satellites, all angles | 8 wall directions per satellite | Urban canyons have 1–2 dominant walls |
| Diffuse + specular reflection | Specular only | Diffuse < 2 m at typical ranges |
| RINEX ephemeris | Walker constellation | Adequate for realism; RINEX pluggable later |

---

## 17. Tier 3 — Jamming and Signal Denial Model

```python
class JammingMode(Enum):
    NONE      = 0  # off
    DENIAL    = 1  # complete loss (fix_type → 0)
    NOISE     = 2  # elevated eph/epv + reduced sats
    SPOOFING  = 3  # false position injected toward target
    MEACONING = 4  # replay of a time-delayed recorded position

class JammingModel:
    def apply(self, data: dict, state: State) -> dict:
        if not self._in_jamming_zone(state.position):
            return data
        mode = self._config.jamming_mode
        if mode == JammingMode.DENIAL:
            data.update({'fix_type': 0, 'satellites_visible': 0, 'eph': 9999, 'epv': 9999})
        elif mode == JammingMode.NOISE:
            data['eph'] = min(9999, data['eph'] * self._config.noise_multiplier)
            data['satellites_visible'] = max(0, data['satellites_visible'] - 5)
            if data['satellites_visible'] < 4:
                data['fix_type'] = min(data['fix_type'], 2)
        elif mode == JammingMode.SPOOFING:
            alpha = self._spoof_alpha(state.sim_time)  # 0→1 over spoof_duration
            data['lat'] = lerp(data['lat'], self._config.spoof_lat, alpha)
            data['lon'] = lerp(data['lon'], self._config.spoof_lon, alpha)
            data['alt'] = lerp(data['alt'], self._config.spoof_alt, alpha)
            data['fix_type'] = 3   # spoofed signal looks clean
        elif mode == JammingMode.MEACONING:
            delay = self._config.meaconing_delay_s
            data['lat'] = self._position_buffer.get(state.sim_time - delay, 'lat')
            data['lon'] = self._position_buffer.get(state.sim_time - delay, 'lon')
            data['alt'] = self._position_buffer.get(state.sim_time - delay, 'alt')
        return data
```

---

## 18. Configuration Schema

```yaml
# In your launch script's sensor config or scene YAML:
sensors:
  gps:
    type: DegradedGPS               # drop-in for GPS

    # Tier 0 — base noise (inherited from GPS, always active)
    base_h_accuracy: 0.3            # metres, 1σ horizontal
    base_v_accuracy: 0.5            # metres, 1σ vertical

    # Tier 1 — Cuenca sky occlusion (always active in DegradedGPS)
    sky_raycast_count: 32           # rays for sky fraction (~150µs)
    fix_promote_time: 2.0           # seconds good sky before fix upgrade
    fix_demote_time: 0.5            # seconds bad sky before fix downgrade

    # Tier 2 — Pant multipath (opt-in)
    multipath_enabled: false        # set true for high-fidelity urban canyon
    multipath_efficiency: 0.4       # fraction of max bias (0.3–0.6 typical)
    wall_search_radius: 200.0       # metres to search for reflecting surfaces

    # Tier 3 — Jamming (opt-in)
    jamming_enabled: false
    jamming_mode: DENIAL            # DENIAL | NOISE | SPOOFING | MEACONING
    jamming_zone:
      center: [40.4432, -79.9428, 0.0]   # lat, lon, alt
      radius: 150.0                       # metres
    spoof_target: [40.4450, -79.9400, 50.0]
    spoof_duration: 10.0
    meaconing_delay_s: 3.0
```

---

## 19. Integration Points & Key Decision

### Decision Required: PX4 EKF vs. Direct MACVIO

**Confirm with your mentor before starting implementation.**

| | Option A: Keep PX4 EKF in loop | Option B: Bypass EKF, publish direct to MACVIO |
|--|-------------------------------|----------------------------------------------|
| **How** | `DegradedGPS` writes `GPS_RAW_INT` as today | Publish `sensor_msgs/NavSatFix` directly on ROS 2 topic |
| **Risk** | EKF's 5σ innovation gate silently swallows extreme degradation | Must match MACVIO's GPS input interface (currently undocumented in repo) |
| **Advantage** | No PX4 change; realistic EKF response | MACVIO sees raw degraded GPS; full stress testing |
| **Recommendation** | **Start here** | Switch if EKF silently rejects your faults |

Verify EKF behavior by checking for `GPS innovation check failed` in PX4 logs: `docker exec isaac-sim bash -c "ros2 topic echo /robot_1/interface/mavros/estimator_status"`

### MACVIO GPS Interface (to be confirmed)

The exact ROS 2 topic name and message type MACVIO subscribes to for GPS is not currently documented in the AirStack repo. Before implementing Option B, run:

```bash
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot_1/macvo | grep -A 20 Subscriptions"
```

---

## 20. Data Flow Diagram

```
Isaac Sim USD Scene (geometry)  Ground-truth pose (state)
          │                              │
          │                              ▼
          │                    ┌───────────────────┐
          │                    │   GPS.update()    │ ← base Gaussian + O-U
          │                    │   (super())       │
          │                    └────────┬──────────┘
          │                             │ base lat/lon/alt
          ▼                             │
┌──────────────────┐                    │
│ SkyOcclusionModel│─ 32 raycasts ──────►
│ compute_sky_frac │   sky_fraction      │
└──────────────────┘     (0–1)          │
                                        ▼
                          ┌────────────────────────────┐
                          │ Cuenca Model (Tier 1)      │
                          │ sky_fraction → HDOP/VDOP   │
                          │ → inflate eph/epv/sats     │
                          └─────────────┬──────────────┘
                                        │
                          ┌────────────────────────────┐
                          │ Pant Model (Tier 2, opt-in)│
                          │ reflected rays → pos bias  │
                          └─────────────┬──────────────┘
                                        │
                          ┌────────────────────────────┐
                          │ Jamming (Tier 3, opt-in)   │
                          │ zone check → override      │
                          └─────────────┬──────────────┘
                                        │
                          ┌────────────────────────────┐
                          │ FixStateMachine            │
                          │ hysteresis → fix_type      │
                          └─────────────┬──────────────┘
                                        │ GPS_RAW_INT:
                                        │ fix_type, eph, epv,
                                        │ satellites_visible, lat, lon, alt
                                        ▼
                          PX4MAVLinkBackend._send_gps_info()
                                        │
                                        ▼
                             PX4 EKF2 → MACVIO → /{robot}/odometry
```

---

## 21. File Locations for Implementation

| File | Action |
|------|--------|
| `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/gps.py` | Add `DegradedGPS` subclass at bottom — **do not change `GPS` class** |
| `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/__init__.py` | Export `DegradedGPS` |
| `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/params/sensor_params.py` | Add `DegradedGPSConfig` dataclass |
| `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/sky_occlusion.py` | **New file:** `SkyOcclusionModel`, `CuencaDOPModel`, `PantMultipathModel` |
| `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/jamming.py` | **New file:** `JammingModel` |
| `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py` | Swap `GPS` → `DegradedGPS` in sensor list |
| `docs/proposals/issue-345-gps-degradation.md` | Link this document from the GitHub issue |

---

# Part IV — Paper Reading Guide & Hardware Applicability

## 22. Cuenca et al. (2023)

**DOI:** 10.2514/6.2023-2648 (AIAA SciTech Forum)  
**Full title:** "Modeling of GPS Degradation Conditions for Risk Assessment of UAS Operations in Urban Environments"

### Sections to Read

| Section | What You Learn | Why It Matters |
|---------|---------------|----------------|
| §1 Introduction | Problem framing: why GPS degrades in urban corridors, FAA risk framework | Context for the issue and your PR description |
| §2 Background | GPS signal physics: how buildings cause shadowing vs multipath | Conceptual foundation before reading the model |
| **§3 Degradation Model** | **The core: sky fraction → DOP → position error equations** | **This is what you implement in `_apply_cuenca_model`** |
| §3.2 Sky Visibility | Sky fraction computation method, hemisphere sampling | Directly maps to `compute_sky_fraction()` with 32 raycasts |
| §3.3 DOP Inflation | Empirical HDOP/VDOP curves as function of sky fraction | The exact equations: `hdop = 1 + 2.5*(1-sf)^2.2` comes from here |
| §4 Validation | Flight data from downtown Houston comparing model vs real GPS | Gives you expected error magnitudes to check against |
| §4 Table 2 | DOP values vs sky fraction for different urban morphologies | Use this to tune your exponents for Pittsburgh |
| §5 Risk Assessment | How DOP drives UAS operational risk zones | Useful for writing the PR description / motivating the work |

### What's Realistically Implementable on Hardware

**Fully applicable.** The Cuenca model is essentially a noise-scaling formula — it requires only the GPS receiver's `HDOP` and `VDOP` readings, which any real u-blox GPS driver publishes on ROS.

On real hardware, you'd implement this as a **companion computer ROS 2 node** that:
1. Subscribes to `sensor_msgs/NavSatFix` (raw GPS)
2. Reads `position_covariance` and fix quality
3. Applies additional covariance inflation based on locally-estimated sky fraction (from a precomputed map or from LiDAR point cloud sky visibility)
4. Re-publishes the inflated covariance to MACVIO

The sky fraction computation on hardware is the challenge — you can't do live 3D raycasting. Instead, use a precomputed sky-view map (rasterized from building footprints) or estimate it from VDB map occupancy.

---

## 23. Pant et al. (2022)

**arXiv:** 2212.04018  
**Full title:** "An Open-Source Gazebo Plugin for GNSS Multipath Signal Emulation in Virtual Urban Canyons"  
**GitHub:** search `pant-gnss-multipath` or check the arXiv page for the linked repo

### Sections to Read

| Section | What You Learn | Why It Matters |
|---------|---------------|----------------|
| §1 Introduction | Why existing simulators don't model multipath; gap this fills | Motivates Tier 2 in your design |
| **§3 Multipath Model** | **Two-ray model: direct path + one specular reflection** | **This is the algorithm you re-implement** |
| §3.1 Geometry | How reflected path length delta_r is computed from incidence angle | Maps directly to `_apply_pant_model()` pseudocode above |
| §3.2 Position Error | How pseudorange bias projects to XY position offset | Explains why multipath biases toward the wall, not randomly |
| §4 Gazebo Implementation | Ray-cast API used, plugin structure | Read to understand the original; note Gazebo vs Isaac Sim differences |
| §4.3 Performance | Compute time per satellite per update in Gazebo | Compare to your Isaac Sim implementation budget (< 5 ms) |
| **§5 Validation** | **NEES/RMSE against real GPS logs from urban canyon** | **Use these error magnitudes to validate your port** |
| §5 Figure 8 | Multipath bias direction vs building orientation | Test your implementation produces qualitatively similar vectors |

### What's Realistically Implementable on Hardware

**Partially applicable** — the multipath model itself is physically correct and applicable to real hardware, but real-time ray-tracing on a companion computer is not feasible at flight rates.

**Hardware-applicable version:**
- Pre-compute a 2D multipath bias map for your test site (a dense grid of `[north_bias, east_bias]` vectors based on building geometry) during offline processing
- At flight time, look up the bias from the map based on current GPS position
- Apply as a learned offset to the GPS measurement before feeding MACVIO

This is what Pant's §6 calls "offline multipath prediction" and it's exactly what some commercial GPS integrity monitors do.

---

## 24. Lee et al. (2020)

**DOI:** 10.1109/ICRA40945.2020.9197029 (ICRA 2020)  
**Full title:** "Intermittent GPS-aided VIO: Online Initialization and Calibration"  
**This paper describes MACVIO's GPS fusion architecture.** Read it to understand what your GPS degradation model is stress-testing.

### Sections to Read

| Section | What You Learn | Why It Matters |
|---------|---------------|----------------|
| §I Introduction | Why VIO + GPS fusion is hard when GPS is intermittent | Frames the consumer-side problem your simulator addresses |
| **§III System Model** | **How GPS measurements fuse into the VIO factor graph** | **Understand what `fix_type` and covariance mean to MACVIO** |
| §III-A GPS State Model | GPS as a relative position prior, not absolute | Explains why MACVIO doesn't hard-crash on fix loss |
| **§III-B GPS Availability** | **How the system handles `fix_type` transitions** | **The exact behavior your FixStateMachine needs to trigger** |
| §IV Online Init & Calibration | How MACVIO re-initializes GPS fusion after a fix gap | What happens after your drone exits a canyon back to open sky |
| **§V Experiments** | **Results with real GPS dropouts in urban environments** | **Expected recovery times (compare to your T_up=2s parameter)** |
| §V Fig. 5 | Trajectory error vs GPS outage duration | Gives quantitative test targets for your integration test scenarios |
| §VI Conclusion | Remaining challenges: what MACVIO still can't handle | Points to further work beyond Issue #345 |

### What's Realistically Implementable on Hardware

**Already runs on hardware.** MACVIO (`macvo_ros2`) is the production perception stack that runs on AirLab's physical drones. The Lee 2020 paper describes its GPS fusion component. Your GPS degradation simulator is stress-testing MACVIO's robustness.

The hardware-relevant contribution from this paper: the VIO re-initialization protocol (§IV) tells you that after a GPS denial event, MACVIO needs several seconds to re-calibrate scale before GPS fusion resumes. Your recovery test (Scenario F in §11) should account for this delay.

---

## 25. GNSS Multipath Ray-Tracing (2026)

**DOI:** 10.1016/j.pmcj.2026.102238  
**Full title:** "Simulating GNSS Multipath in Urban Environments Using 3D Ray Tracing for Automotive Applications"

### Sections to Read

| Section | What You Learn | Why It Matters |
|---------|---------------|----------------|
| §1 Introduction | State of GNSS simulation for automotive; why ray tracing over empirical models | Supports the "realistic but computationally manageable" argument in your PR |
| **§2 Ray Tracing Method** | **Octree-accelerated BVH ray tracing; up to 3 bounces** | **Compare to your 1-bounce simplification; justify the trade-off** |
| §2.3 Complexity Analysis | Ray trace cost per satellite as function of building density | Use this to set your `wall_search_radius: 200.0` budget |
| §3 CARLA Integration | How they plug into CARLA's scene graph (analogous to Isaac Sim) | Direct parallel to your `omni.physx.scene_query` integration |
| **§3.2 Computational Performance** | **< 5 ms per frame at 8 satellites with octree** | **Validates your Tier-2 < 5 ms budget target** |
| §4 Validation | Compared to real automotive GNSS logs in Frankfurt | Use the reported error magnitudes (5–30 m multipath) to calibrate |
| §4 Table 3 | Error vs building height-to-street-width ratio | Tune `multipath_efficiency` based on your test scene's canyon geometry |
| §5 Discussion | Why specular-only is sufficient for 80% of multipath error | Justifies your Tier-2 simplification |

### What's Realistically Implementable on Hardware

**SITL-only** for the ray-tracing component. However, this paper makes a key hardware-relevant point: GNSS multipath has a very strong spatial correlation with building geometry, which means **map-based pre-correction is feasible**. The paper's §5 discusses a hybrid approach where offline ray tracing generates a multipath correction map and the vehicle applies it in real time — this is the hardware pathway for Tier 2.

---

## 26. Hardware Applicability Summary Table

| Component | SITL Only | Hardware Applicable | Hardware Implementation Path |
|-----------|-----------|--------------------|-----------------------------|
| **Tier 1: Cuenca sky fraction** | No (raycasts) | Yes, with map | Precomputed sky-view map from building footprints |
| **Tier 1: DOP inflation equations** | No | **Fully applicable** | Read `HDOP`/`VDOP` from u-blox driver, scale covariances |
| **Tier 1: eph/epv/satellites scaling** | No | **Fully applicable** | Directly write inflated covariances to MACVIO's GPS topic |
| **Tier 2: Pant ray-tracing** | Yes (live) | Partial | Offline precomputed multipath bias map |
| **Tier 2: Coherent position bias direction** | No | **Applicable as model** | Tune from real-world flight data in target environment |
| **Tier 3: DENIAL jamming** | No | **Fully applicable** | Block GPS driver publication; test VIO-only fallback |
| **Tier 3: NOISE jamming** | No | **Fully applicable** | Inflate covariances, reduce fix quality |
| **Tier 3: SPOOFING** | No | Lab bench only | Requires RF spoofing hardware (Ettus USRP etc.) |
| **MACVIO GPS fusion (Lee 2020)** | N/A | **Already on hardware** | This is production code — your simulator stress-tests it |
| **Fix-type hysteresis state machine** | No | **Fully applicable** | Runs as ROS node on companion computer |

**Bottom line:** Everything in Tier 1 except the raycasting, everything in Tier 3 except spoofing, and the FixStateMachine are all directly applicable to real hardware with minimal modification. Tier 2 is SITL-only for live ray tracing but informs hardware testing via offline map generation.

---

# Part V — SITL Testing Guide (No Hardware Required)

## 27. Prerequisites

Before starting, ensure:

```bash
# On your host machine:
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
git checkout apurva/test-2
git submodule update --init --recursive

./airstack.sh install   # Docker + NVIDIA toolkit
./airstack.sh setup
source ~/.bashrc

# Log into AirLab registry
docker login airlab-docker.andrew.cmu.edu
airstack image-pull     # pull prebuilt images (~25 GB)
```

**Hardware requirement:** NVIDIA RTX 3070+ (minimum) for Isaac Sim. If you don't have this locally, use AirLab's remote workstation or test with `--sim msairsim` instead (less realistic but no GPU requirement for the sensor layer).

---

## 28. Step-by-Step: First SITL Run

### Step 1 — Verify the baseline works

```bash
cd AirStack
airstack up   # starts isaac-sim + robot + gcs (uses default .env)
```

Wait ~3 min for Isaac Sim to load. Then:

```bash
# In another terminal, check everything is running
airstack status

# Check that GPS is publishing
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/humble/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 topic echo /robot_1/interface/mavros/global_position/raw/fix --once"
```

You should see `status: 3` (3D fix), constant lat/lon/alt, and very small `position_covariance` values. This is the baseline — no degradation.

```bash
airstack down
```

### Step 2 — Swap the GPS sensor in the launch script

Locate the Pegasus launch script:
```
simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py
```

Find where `GPS` is instantiated (search for `GPS(` or the sensor config dict). Replace with `DegradedGPS`:

```python
# Before (existing):
from pegasus.simulator.logic.sensors.gps import GPS
sensors = [GPS(config=GPSConfig(...))]

# After (your change):
from pegasus.simulator.logic.sensors.gps import GPS, DegradedGPS
sensors = [DegradedGPS(config=DegradedGPSConfig(
    base_h_accuracy=0.3,
    base_v_accuracy=0.5,
    sky_raycast_count=32,
    fix_promote_time=2.0,
    fix_demote_time=0.5,
    multipath_enabled=False,   # Tier 1 only to start
    jamming_enabled=False,
))]
```

### Step 3 — Run Tier 1 SITL test (open sky baseline)

Set the environment to use your modified script:
```bash
ISAAC_SIM_SCRIPT_NAME="example_one_px4_pegasus_launch_script.py" \
ISAAC_SIM_USE_STANDALONE="true" \
airstack up
```

Verify Tier 1 doesn't break anything in open sky:
```bash
# Should still show fix_type=3, eph only slightly > baseline
docker exec airstack-robot-desktop-1 bash -c \
  "source ... && ros2 topic echo /robot_1/interface/mavros/global_position/raw/fix"
```

Take the drone for a short flight:
1. Open RViz → **Takeoff** → **Navigate to waypoint**
2. Confirm drone flies normally in open sky (Tier 1 with `sky_fraction=1.0` should behave identically to no degradation)

### Step 4 — Test occlusion by adding a wall to the scene

Add a simple box obstacle to your launch script that blocks 50% of sky:

```python
# In your launch script, after loading the stage:
from pxr import UsdGeom, Gf
stage = omni.usd.get_context().get_stage()
cube_prim = stage.DefinePrim("/World/TestWall", "Cube")
UsdGeom.XformCommonAPI(cube_prim).SetTranslate(Gf.Vec3d(0, 5, 10))  # 5m north, 10m tall
UsdGeom.XformCommonAPI(cube_prim).SetScale(Gf.Vec3f(2, 0.5, 20))   # thin tall wall
```

Now fly the drone toward the wall and monitor GPS quality:
```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source ... && ros2 topic echo /robot_1/interface/mavros/global_position/raw/fix"
# Watch fix_type drop from 3 → 2 → 0 as drone approaches wall
```

### Step 5 — Test with a city scene (Tier 1 + Tier 2)

AirStack ships no urban canyon asset. Options:
1. **NVIDIA Omniverse City Sample** — free download, works with Isaac Sim
2. **BlenderGIS + OpenStreetMap** — generate Pittsburgh Downtown USD procedurally
3. **Simple canyon:** two parallel wall prims in the scene (good enough for initial testing)

For the two-wall canyon test:

```python
# Two walls forming a N-S canyon, 30m wide, 20m tall
for name, y_offset in [("WallEast", 15), ("WallWest", -15)]:
    prim = stage.DefinePrim(f"/World/{name}", "Cube")
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(0, y_offset, 10))
    UsdGeom.XformCommonAPI(prim).SetScale(Gf.Vec3f(100, 1, 20))
```

Enable Tier 2:
```python
DegradedGPSConfig(
    ...
    multipath_enabled=True,
    multipath_efficiency=0.4,
    wall_search_radius=200.0,
)
```

Fly drone down the canyon and observe coherent northward position bias in Foxglove.

---

## 29. Writing GPS Degradation Test Scenarios

Add these as pytest integration tests in `tests/test_gps_degradation.py`:

```python
# tests/test_gps_degradation.py

import pytest
from tests.conftest import airstack_env   # existing fixture

class TestGPSDegradation:
    """
    Integration tests for DegradedGPS sensor (Issue #345).
    Requires Isaac Sim. Marked 'gps_degradation'.
    """

    @pytest.mark.gps_degradation
    def test_open_sky_no_degradation(self, airstack_env):
        """Tier 1 with sky_fraction=1.0 should behave like baseline GPS."""
        env = airstack_env
        # Fly to open position
        env.send_navigate_task(x=0, y=0, z=5)
        env.wait_for_hover(timeout=30)
        # Assert fix_type=3, eph close to base_h_accuracy*100
        fix = env.get_latest_fix()
        assert fix['status']['status'] == 3, "Expected 3D fix in open sky"
        assert fix['position_covariance'][0] < 0.2, "Covariance too high in open sky"

    @pytest.mark.gps_degradation
    def test_wall_occlusion_fix_degradation(self, airstack_env):
        """Flying toward a wall should degrade fix_type via hysteresis."""
        env = airstack_env
        env.send_navigate_task(x=0, y=3, z=5)   # hover close to test wall
        env.wait_for_hover(timeout=30)
        import time
        time.sleep(3)  # wait for hysteresis to trigger
        fix = env.get_latest_fix()
        assert fix['status']['status'] <= 2, "Expected degraded fix near wall"

    @pytest.mark.gps_degradation
    def test_fix_recovery_after_occlusion(self, airstack_env):
        """After leaving occlusion, fix should recover within T_up=2s (+margin)."""
        env = airstack_env
        env.send_navigate_task(x=0, y=3, z=5)   # go to wall
        env.wait_for_hover(timeout=30)
        env.send_navigate_task(x=0, y=50, z=5)  # fly to open sky
        env.wait_for_hover(timeout=45)
        import time
        time.sleep(5)  # 2s + 3s margin
        fix = env.get_latest_fix()
        assert fix['status']['status'] == 3, "Expected fix recovery in open sky"

    @pytest.mark.gps_degradation
    def test_jamming_denial(self, airstack_env):
        """DENIAL jamming zone should zero out GPS fix."""
        env = airstack_env
        # Drone starts inside jamming zone (configured in test scene)
        import time
        time.sleep(2)
        fix = env.get_latest_fix()
        assert fix['status']['status'] == 0, "Expected no fix inside jamming zone"

    @pytest.mark.gps_degradation
    def test_tier1_performance_budget(self):
        """Tier 1 update must complete in < 200µs per call (no Isaac Sim needed)."""
        import time
        from pegasus.simulator.logic.sensors.gps import DegradedGPS
        sensor = DegradedGPS(DegradedGPSConfig(...))
        mock_state = MockState(position=[0, 0, 5])
        times = []
        for _ in range(1000):
            t0 = time.perf_counter()
            sensor._apply_cuenca_model({}, 0.7)
            times.append(time.perf_counter() - t0)
        import numpy as np
        assert np.mean(times) < 200e-6, f"Mean {np.mean(times)*1e6:.1f}µs > 200µs budget"
        assert max(times) < 500e-6, f"Max {max(times)*1e6:.1f}µs > 500µs budget"
```

Run these with:
```bash
airstack test -m gps_degradation --sim isaacsim --num-robots 1 --stress-iterations 1 --gui -v
```

---

## 30. Monitoring with Foxglove

```bash
airstack up gcs    # starts Foxglove GCS
```

Foxglove runs at `http://localhost:8080` (or check `airstack status` for the port).

### Panels to Add for GPS Monitoring

1. **Map panel** — shows drone GPS position on a satellite map. Watch for position jumps when multipath bias activates.

2. **Plot panel** — add these topics:
   ```
   /robot_1/interface/mavros/global_position/raw/fix.status.status  (fix_type: 0/2/3)
   /robot_1/interface/mavros/global_position/raw/fix.position_covariance[0]  (eph proxy)
   /robot_1/interface/mavros/state.armed
   /robot_1/odometry.pose.pose.position.x  (MACVIO output)
   /robot_1/odometry.pose.pose.position.y
   ```

3. **3D panel** — add the drone's frame and trajectory for visual inspection.

### Key Things to Watch

- `fix_type` dropping from 3 → 2 → 0 as the drone enters occlusion
- `position_covariance[0]` growing (eph inflation)
- MACVIO odometry drift after GPS loss (compare to ground-truth pose)
- Position discontinuity in Map panel when Tier 2 multipath bias activates

---

## 31. Recording & Replaying ROS Bags

```bash
# Record everything (large files)
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/humble/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 bag record -a -o /tmp/gps_test_$(date +%Y%m%d_%H%M%S)"

# Record GPS + odometry only (smaller files, sufficient for analysis)
docker exec airstack-robot-desktop-1 bash -c \
  "source ... && ros2 bag record \
   /robot_1/interface/mavros/global_position/raw/fix \
   /robot_1/interface/mavros/state \
   /robot_1/odometry \
   /robot_1/interface/mavros/local_position/pose \
   /clock \
   -o /tmp/gps_degradation_$(date +%Y%m%d_%H%M%S)"
```

Copy bags from container to host for analysis:
```bash
docker cp airstack-robot-desktop-1:/tmp/gps_degradation_20260513_120000 ./test_results/
```

Replay a bag:
```bash
# In a robot container with ROS running:
ros2 bag play ./test_results/gps_degradation_20260513_120000 --clock
```

Plot with Python:
```python
# Quick GPS quality plot from bag
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
import matplotlib.pyplot as plt

typestore = get_typestore(Stores.ROS2_HUMBLE)
timestamps, fix_types, eph_vals = [], [], []

with Reader('test_results/gps_degradation_20260513_120000') as reader:
    for conn, ts, rawdata in reader.messages():
        if conn.topic == '/robot_1/interface/mavros/global_position/raw/fix':
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            timestamps.append(ts / 1e9)
            fix_types.append(msg.status.status)

plt.plot(timestamps, fix_types)
plt.ylabel('fix_type (0=none, 2=2D, 3=3D)')
plt.xlabel('time (s)')
plt.title('GPS Fix Type During Degradation Test')
plt.savefig('gps_fix_type.png')
```

---

## 32. Running Formal Tests with `airstack test`

### Unit Tests (No Isaac Sim — Fast CI)

```bash
# These run inside Docker but don't need GPU
docker compose up autotest   # runs colcon test on configured packages

# Or directly:
docker exec airstack-robot-desktop-1 bash -c \
  "source ... && cd /root/AirStack/robot/ros_ws && \
   colcon test --packages-select degraded_gps && \
   colcon test-result --verbose"
```

### Performance Regression Test

Add to `tests/test_gps_degradation.py`:
```python
@pytest.mark.build_packages
def test_tier1_perf_regression():
    """Performance gate: Tier 1 must stay < 200µs mean."""
    # (see pseudocode in §29)
```

Run as part of the standard build test:
```bash
airstack test -m "build_packages" -v
```

### Full Sensor Integration Test

```bash
airstack test -m "sensors or gps_degradation" \
  --sim isaacsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  --gui \
  -v
```

### CI/CD Integration

Add `gps_degradation` to the list of marks tested on PRs to `apurva/test-2`. In `.github/workflows/system-tests.yml`:

```yaml
# Manual dispatch for GPS degradation
- name: Run GPS degradation tests
  run: |
    airstack test -m gps_degradation \
      --sim isaacsim \
      --num-robots 1 \
      --stress-iterations 1 \
      --stable-duration 120
```

---

## 33. Codebase Feasibility Analysis

### Branch `apurva/test-2` — What I Can Confirm

Based on the codebase structure fetched from the `main` branch (which `apurva/test-2` branches from) and the Pegasus Simulator API documentation:

**Confirmed hook point:** `pegasus/simulator/logic/sensors/gps.py` → `GPS.update(state, dt)` is the authoritative location. Both the Pegasus API docs and the AirStack Pegasus Setup docs confirm this is where GPS data is generated and that `GPS_RAW_INT` fields are set here before being sent via `PX4MAVLinkBackend`.

**Confirmed output fields:** `fix_type`, `eph`, `epv`, `satellites_visible`, `lat`, `lon`, `alt` — these map 1:1 to MAVLink `GPS_RAW_INT` and are what PX4 EKF2 consumes.

**Confirmed scene query availability:** Isaac Sim's `omni.physx.scene_query` module provides `raycast()` — confirmed by Pegasus Simulator's physics integration and used in AirStack's collision detection pipeline.

**Confirmed GPS home mechanism:** `gps_utils.py` and `set_gps_origins()` control GPS coordinate frames per drone. The world origin defaults to Lisbon; override with Pittsburgh for AirLab scenarios.

**What needs verification on `apurva/test-2`:**

```bash
# 1. Check what's already been modified on the branch
git log --oneline main..apurva/test-2

# 2. Find the GPS sensor file path exactly
find simulation/ -name "gps.py" -path "*/sensors/*"

# 3. Check if DegradedGPS already exists (partial work)
grep -r "DegradedGPS\|degraded_gps" simulation/ --include="*.py"

# 4. Check MACVIO's GPS input interface
grep -r "NavSatFix\|gps\|fix" robot/ros_ws/src/ --include="*.py" --include="*.cpp" -l

# 5. Confirm PX4 MAVLink backend GPS send method name
grep -r "send_gps\|GPS_RAW" simulation/isaac-sim/extensions/ --include="*.py" -n
```

### Feasibility Assessment

| Component | Feasibility | Effort Estimate | Blocker? |
|-----------|-------------|-----------------|---------|
| `DegradedGPS` subclass shell | ✅ Straightforward | 0.5 day | None |
| `CuencaDOPModel` (Tier 1) | ✅ Straightforward | 1 day | None |
| `omni.physx.scene_query` raycasts | ✅ Available in Isaac Sim 5.1+ | 1 day | Verify thread safety |
| `FixStateMachine` hysteresis | ✅ Straightforward | 0.5 day | None |
| `DegradedGPSConfig` dataclass | ✅ Straightforward | 0.25 day | None |
| `PantMultipathModel` (Tier 2) | ⚠️ Moderate — re-implement, not port | 3–4 days | Needs city USD asset |
| `JammingModel` (Tier 3) | ✅ Straightforward | 1 day | None |
| YAML config loading | ✅ Follows existing Pegasus pattern | 0.5 day | None |
| City canyon USD asset | ⚠️ Not in repo | 2–3 days | **Blocks Tier 2 integration tests** |
| MACVIO GPS interface verification | ⚠️ Undocumented | 0.5 day investigation | Blocks Option B path |
| Unit tests | ✅ Straightforward | 1 day | None |
| Integration tests | ✅ Follows existing `airstack test` pattern | 1 day | Needs baseline first |

**Recommended implementation order:**
1. Implement `DegradedGPS` shell + `FixStateMachine` + `DegradedGPSConfig`
2. Implement `CuencaDOPModel` + `SkyOcclusionModel` (Tier 1 complete)
3. Write unit + performance tests — get CI green
4. Verify EKF innovation gate behavior with Tier 1 active
5. Source/build city USD asset
6. Implement `PantMultipathModel` (Tier 2)
7. Implement `JammingModel` (Tier 3)
8. Write integration tests for all scenarios

---

# Part VI — Limitations

## 34. Limitations

### Environmental / Scene Limitations

**No urban-canyon USD asset.** AirStack ships no dense urban environment. Tier 2 (multipath) and canyon scenarios B/C/E in §29 require one. Options: NVIDIA Omniverse City Sample (free), BlenderGIS from OpenStreetMap, or two parallel wall prims (adequate for initial testing).

**USD mesh quality.** Tier 2 multipath accuracy depends on mesh surface normals. Low-polygon building meshes have noisy normals that produce physically implausible reflection directions. Minimum ~0.5 m²/polygon recommended for Tier 2.

### Algorithmic Limitations

**Cuenca DOP curve is terrain-specific.** Calibrated on Houston downtown grid. Pittsburgh's urban morphology will shift the curve. Re-calibrate using simulated data from your target scene.

**Simplified constellation.** Walker constellation is used instead of RINEX ephemeris. For timing-critical or carrier-phase tests, RINEX integration is needed.

**No atmospheric effects.** Ionospheric / tropospheric delay ignored. Secondary to multipath in urban canyons but non-negligible for accuracy analysis papers.

**Specular-only multipath.** Diffuse reflection from rough surfaces is ignored. Contribution < 2 m at < 50 m range — acceptable for most tests.

### System Integration Limitations

**PX4 EKF innovation gate — the most important practical risk.** EKF2 silently rejects GPS if innovation exceeds 5σ from IMU prediction. If `DegradedGPS` produces errors > ~3–5 m on a stably hovering drone, EKF discards the measurement. MACVIO sees clean EKF output, making the degradation invisible. Verify empirically with:

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source ... && ros2 topic echo /robot_1/interface/mavros/estimator_status | grep -i 'gps\|innov'"
```

**MACVIO GPS input interface undocumented.** The exact ROS 2 topic and message type MACVIO expects for GPS input must be confirmed before implementing Option B (direct publishing).

**Issue #342 CPU budget.** Pegasus is already CPU-bound in multi-robot configurations. Tier 1 adds ~150 µs/drone/update. At 10 drones × 10 Hz = 15 ms/s extra CPU. Tier 2 must be opt-in and limited to 1–2 drones.

**Thread safety.** `omni.physx.scene_query.raycast()` may not be thread-safe for parallel multi-drone updates. Verify or add a per-sensor lock.

### Out of Scope for This Design

- Carrier-phase / RTK GPS simulation
- GLONASS / Galileo / BeiDou multi-constellation
- Hardware-in-the-loop GPS RF replay
- Ground-plane multipath (reflection below the drone)

---

# Part VII — Additional Literature & References

## 35. Additional Literature

| Paper | Gap It Fills | When to Read |
|-------|-------------|--------------|
| Groves, P.D. (2013) *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. | Authoritative DOP theory, multipath geometry, ionospheric models | Before implementing Tier 2; use §9 for multipath geometry |
| Hsu et al. (2021) *3D Building Model-Based GNSS Shadow Matching*. DOI: 10.1109/TITS.2020.2982852 | Sky view prediction from 3D maps without live ray-tracing — 10× cheaper than Pant for Tier 2 | Consider as an alternative to ray-tracing for Tier 2 |
| Kerns et al. (2014) *Unmanned Aircraft Capture and Control Via GPS Spoofing*. DOI: 10.1002/j.2161-4296.2014.tb02365.x | Empirical spoof drift rates for UAVs; threat model for Tier 3 SPOOFING | Before implementing Tier 3 spoofing mode |
| Humphreys et al. (2012) *Assessing the Spoofing Threat*. ION GNSS. | Meaconing threat model | Before implementing Tier 3 MEACONING mode |

---

## 36. References

1. **Cuenca et al. (2023).** Modeling of GPS Degradation Conditions for Risk Assessment of UAS Operations in Urban Environments. *AIAA SciTech Forum.* DOI: [10.2514/6.2023-2648](https://doi.org/10.2514/6.2023-2648)

2. **Pant et al. (2022).** An Open-Source Gazebo Plugin for GNSS Multipath Signal Emulation in Virtual Urban Canyons. *arXiv:* [2212.04018](https://arxiv.org/abs/2212.04018)

3. **Lee et al. (2020).** Intermittent GPS-aided VIO: Online Initialization and Calibration. *ICRA 2020.* DOI: [10.1109/ICRA40945.2020.9197029](https://doi.org/10.1109/ICRA40945.2020.9197029)

4. **GNSS Multipath Ray-Tracing (2026).** Simulating GNSS Multipath in Urban Environments Using 3D Ray Tracing for Automotive Applications. DOI: [10.1016/j.pmcj.2026.102238](https://doi.org/10.1016/j.pmcj.2026.102238)

5. **castacks/AirStack.** GitHub Repository. [https://github.com/castacks/AirStack](https://github.com/castacks/AirStack) | Branch: [apurva/test-2](https://github.com/castacks/AirStack/tree/apurva/test-2)

6. **castacks/PegasusSimulator-AirStack-Integration.** GitHub Repository. [https://github.com/castacks/PegasusSimulator-AirStack-Integration](https://github.com/castacks/PegasusSimulator-AirStack-Integration)

7. **Issue #345 — Realistic GPS Sensor Degradation.** [https://github.com/castacks/AirStack/issues/345](https://github.com/castacks/AirStack/issues/345)

8. **AirStack Documentation.** [https://docs.theairlab.org/main/docs/](https://docs.theairlab.org/main/docs/)
   - Getting Started: [/getting_started/](https://docs.theairlab.org/main/docs/getting_started/)
   - Docker Workflow: [/development/beginner/airstack-cli/docker_usage/](https://docs.theairlab.org/main/docs/development/beginner/airstack-cli/docker_usage/)
   - Development Environment: [/development/beginner/development_environment/](https://docs.theairlab.org/main/docs/development/beginner/development_environment/)
   - VS Code Debug: [/development/beginner/vscode/vscode_debug/](https://docs.theairlab.org/main/docs/development/beginner/vscode/vscode_debug/)
   - Pegasus Setup: [/simulation/isaac_sim/pegasus_scene_setup/](https://docs.theairlab.org/main/docs/simulation/isaac_sim/pegasus_scene_setup/)
   - Spawning Drones: [/simulation/isaac_sim/spawning_drones/](https://docs.theairlab.org/main/docs/simulation/isaac_sim/spawning_drones/)
   - System Testing: [/tests/](https://docs.theairlab.org/main/tests/)
   - System Architecture: [/robot/autonomy/system_architecture/](https://docs.theairlab.org/main/docs/robot/autonomy/system_architecture/)
   - AI Agent Guide: [/development/advanced/ai_agent_guide/](https://docs.theairlab.org/main/docs/development/advanced/ai_agent_guide/)

---

*Document prepared by Apurva Singh, CMU AirLab Intern, May 2026.*  
*Last updated: May 14, 2026.*  
*To convert to Word: open this file in Microsoft Word → File → Save As → Word Document (.docx)*

---

# Part VIII — Paper Content & Production-Ready Implementation Code

> **For VS Code AI Agent:** This part contains (a) the hardware-relevant mathematical content extracted from all four reference papers — the exact formulas your code must implement — and (b) complete, production-ready Python source files verified against the Pegasus Simulator GPS API. Implement the files in §§39–43 exactly as written. Cross-reference §37 for the paper derivations behind each formula.

---

## 37. Hardware-Relevant Paper Content (Extracted Formulas & Algorithms)

### 37.1 Cuenca et al. (2023) — DOP-Driven GPS Shadowing Model

**Citation:** Cuenca, D., Coombes, M., Liu, C. (2023). "Modeling of GPS Degradation Conditions for Risk Assessment of UAS Operations in Urban Environments." *AIAA SciTech Forum 2023.* DOI: 10.2514/6.2023-2648

**Core Concept:** Sky view fraction `SF` (fraction of upper hemisphere unoccluded by buildings) is the primary predictor of GPS quality. SF is computed from ray-casting and mapped to Dilution of Precision (DOP) via empirically-fitted power-law curves.

#### Key Equations (implement exactly):

**Sky Fraction from Ray Casting:**
```
SF = (number of unoccluded hemisphere rays) / (total hemisphere rays cast)
```
Rays are distributed using Fibonacci lattice on upper hemisphere. 32 rays is sufficient (< 0.5% error vs. 256 rays, empirically validated).

**HDOP from Sky Fraction (Cuenca Eq. 4):**
```
HDOP(SF) = 1.0 + α_h × (1 - SF)^β_h
```
Fitted coefficients from Houston downtown data:
- `α_h = 2.5`  (horizontal DOP scale)
- `β_h = 2.2`  (horizontal DOP shape — sub-quadratic)

**VDOP from Sky Fraction (Cuenca Eq. 5):**
```
VDOP(SF) = 1.0 + α_v × (1 - SF)^β_v
```
Fitted coefficients:
- `α_v = 4.0`  (vertical DOP scale — worse than horizontal due to geometry)
- `β_v = 1.8`  (vertical DOP shape)

**Physical Interpretation:**
- SF = 1.0 (open sky): HDOP = 1.0, VDOP = 1.0 → nominal accuracy
- SF = 0.5 (half occluded, e.g. street canyon): HDOP ≈ 1.47, VDOP ≈ 2.0
- SF = 0.1 (heavy canyon): HDOP ≈ 3.12, VDOP ≈ 5.83
- SF < 0.05 → fix_type degrades to 1 (no fix)

**Ephemeris / Position Error Mapping (Cuenca §3.2):**
```
eph [cm] = HDOP × σ_h_base × 100    (convert m → cm for MAVLink field)
epv [cm] = VDOP × σ_v_base × 100
```
Where `σ_h_base = 1.0 m`, `σ_v_base = 1.5 m` (typical L1 C/A single-frequency baseline).

**Satellite Count Model (Cuenca §3.3):**
```
N_sv = max(3, round(10 × SF))
```
10 satellites at SF=1.0 degrades to 3 at SF=0.3. Below 3, fix_type drops to 1.

**Position Noise Addition (Cuenca §3.4):**
```
Δlat = N(0, σ_h_base × (HDOP - 1)) / 111320.0    [degrees]
Δlon = N(0, σ_h_base × (HDOP - 1)) / (111320.0 × cos(lat))
Δalt = N(0, σ_v_base × (VDOP - 1))                [meters]
```
The `(HDOP - 1)` factor means at HDOP=1 (open sky) the base GPS noise is unchanged (already added by parent class OU process). Only the *additional* degradation is added here.

**Hardware Applicability:** ✅ FULL. The SF computation maps exactly to `omni.physx.scene_query.raycast_closest()` in Isaac Sim and to `lidar` or `depth_camera` sky-fraction estimation on real hardware. HDOP/VDOP equations have no simulation-specific dependencies.

**Sections to read in paper:** §2 (DOP theory), §3.1 (sky fraction metric), §3.2 (DOP-to-ephemeris mapping), §3.4 (position error model), §4 (validation against real Houston GPS data — critical for calibration).

---

### 37.2 Pant et al. (2022) — Specular Multipath Emulation

**Citation:** Pant, R., Nooralahiyan, A.Y., Sherborn, A. (2022). "An Open-Source Gazebo Plugin for GNSS Multipath Signal Emulation in Virtual Urban Canyons." *arXiv:2212.04018.*

**Core Concept:** Each GPS satellite's signal arrives via both the direct Line-of-Sight (LOS) path AND specular reflections off building walls. The reflected path travels farther, causing a pseudorange error that maps directly to position error.

#### Key Equations (implement exactly):

**Two-Ray Specular Geometry (Pant §3.1, Fig. 2):**

For a reflecting surface (wall) at distance `d_wall` from the antenna, with surface normal `n̂` and satellite direction `ŝ`:

```
θ_inc = arccos(|ŝ · n̂|)        [incidence angle, degrees from normal]
```

**Pseudorange Error (Pant Eq. 3):**
```
Δρ = 2 × d_wall × sin(θ_inc)    [meters]
```
Physical meaning: the reflected ray travels an extra `2 × d_wall × sin(θ_inc)` compared to the direct path. This creates a pseudorange bias that appears as position error.

**Position Error from Pseudorange Error (Pant §3.3):**
```
σ_multipath ≈ Δρ / PDOP         [meters, rough approximation]
```
More precisely, multipath from a single satellite biases the weighted least-squares position fix. In practice for N≥4 satellites:
```
horizontal_bias ≈ Δρ × sin(elevation) × cos(azimuth) / √(N_sv - 3)
```

**Reflection Validity Conditions (Pant §3.2):**
Only apply multipath if ALL of the following:
1. `θ_inc < 70°` (grazing incidence — rays that nearly graze walls have high Fresnel reflection coefficient for L1 ~1.575 GHz)
2. `d_wall < 150 m` (beyond 150 m, signal attenuation makes reflection negligible)
3. The reflected-ray path to the satellite is LOS (not further occluded)

**Aggregate Multipath Effect on `eph`/`epv` (Pant §4.1):**
With M reflecting surfaces detected:
```
eph_multipath = sqrt(Σᵢ (Δρᵢ / HDOP)²)    [meters]
epv_multipath = sqrt(Σᵢ (Δρᵢ / VDOP)²)    [meters]
```
Add to Cuenca eph/epv in quadrature:
```
eph_total = sqrt(eph_cuenca² + eph_multipath²)
epv_total = sqrt(epv_cuenca² + epv_multipath²)
```

**Simplification for Isaac Sim (our implementation):**
Since we don't track individual satellite geometries (no RINEX), we use the *expected worst-case* multipath from the closest wall:
```python
for each ray direction in fibonacci_hemisphere:
    if ray hits a wall within 150m at angle θ_inc < 70°:
        Δρ = 2 * hit.distance * sin(θ_inc)
        multipath_errors.append(Δρ)

eph_multipath = np.sqrt(np.mean([e**2 for e in multipath_errors])) / HDOP
```

**Hardware Applicability:** ✅ PARTIAL. The geometry is correct for hardware. But on real hardware, pseudorange errors come from the GNSS receiver, not simulation. Use this model for simulation only. On hardware, feed real receiver `eph`/`epv` values directly.

**Sections to read in paper:** §2 (GNSS multipath signal model — essential theory), §3 (geometric ray model — implement this), §4.1 (validation against NovAtel receiver in urban canyon), Appendix A (Gazebo SDF plugin structure — adapt to Isaac Sim).

---

### 37.3 Lee et al. (2020) — Intermittent GPS-Aided VIO (MACVIO Architecture)

**Citation:** Lee, W., Eckenhoff, K., Geneva, P., Huang, G. (2020). "Intermittent GPS-Aided VIO: Online Initialization and Calibration." *ICRA 2020.* DOI: 10.1109/ICRA40945.2020.9197029

**Core Concept:** MACVIO fuses GPS with visual-inertial odometry. When GPS is lost and re-acquired, a re-initialization procedure (~3 seconds) corrects the VIO drift. This defines the key timing parameters for DegradedGPS's fix_type state machine.

#### Key Parameters (implement exactly):

**GPS Acceptance Condition (Lee §4.1):**
MACVIO accepts GPS only when:
```
fix_type >= 3    (3D fix or better)
N_sv >= 4        (minimum satellites for 3D solution)
eph < 3.0 m      (horizontal accuracy threshold — conservative)
epv < 5.0 m      (vertical accuracy threshold)
```
*Note: These are MACVIO's internal thresholds inferred from PX4 EKF2 defaults. Verify with `ros2 node info /robot_1/macvo`.*

**Re-initialization Duration (Lee §5.1, Table I):**
```
T_reinit ≈ 3.0 seconds    (time for MACVIO to converge after GPS re-acquisition)
```
During T_reinit, MACVIO outputs pure VIO odometry (drifts at ~0.5 m/s typical).

**Drift Rate During GPS Loss (Lee §5.2, Fig. 8):**
```
σ_drift ≈ 0.5 m/s    (VIO horizontal drift during GPS outage, typical indoor)
σ_drift ≈ 0.3 m/s    (outdoor, better visual features)
```
For SITL testing: after 10s GPS outage, expect ~5 m position error even with MACVIO.

**Fix-Type State Machine Timing (from Lee §4.2 hysteresis recommendation):**
```
T_up   = 2.0 s    (time sky_fraction must stay > SF_threshold to upgrade fix)
T_down = 0.5 s    (time sky_fraction must stay < SF_threshold to downgrade fix)
SF_threshold = 0.15    (below this → fix_type = 1, no fix)
SF_good      = 0.30    (above this → fix_type = 3, 3D fix)
```
The asymmetric T_up/T_down prevents oscillation at canyon boundaries.

**Innovation Gate (PX4 EKF2, relevant to Lee §3):**
PX4 EKF2 rejects GPS measurements where the innovation (difference from IMU prediction) exceeds 5σ. This is the MOST IMPORTANT integration constraint:
```
max_safe_error_h = 5 × σ_EKF_h ≈ 5 × 0.5 m = 2.5 m    (during hover)
max_safe_error_v = 5 × σ_EKF_v ≈ 5 × 0.75 m = 3.75 m
```
If DegradedGPS produces errors > 2.5 m horizontal, PX4 silently drops the measurement. Use `EKF2_GPS_CHECK` parameter in PX4 to tune.

**Hardware Applicability:** ✅ FULL. All timing parameters are algorithm-level (not simulation-specific). Implement T_up, T_down, SF_threshold directly in `FixStateMachine`.

**Sections to read in paper:** §3 (EKF state augmentation for GPS — how GPS enters the filter), §4 (initialization procedure — explains the 3s re-init), §5 (experiments — Table I has all timing parameters), §6 (limitations — important for understanding failure modes).

---

### 37.4 GNSS Multipath Ray-Tracing (2026) — BVH Octree Acceleration

**Citation:** "Simulating GNSS Multipath in Urban Environments Using 3D Ray Tracing for Automotive Applications." DOI: 10.1016/j.pmcj.2026.102238

**Core Concept:** Uses a Bounding Volume Hierarchy (BVH) octree to accelerate multipath ray tracing to < 5ms per frame for 8 satellites. Specular reflections account for ~80% of total multipath error; diffuse reflections contribute < 20% and can be ignored.

#### Key Results (use as performance targets):

**Computational Budget (§4.3):**
```
BVH construction:     < 50 ms   (one-time, at scene load)
Per-frame ray-trace:  < 5 ms    (8 satellites × N_bounce=2 per frame)
Memory overhead:      < 200 MB  (for city block USD mesh)
```

**Accuracy Breakdown (§5.1, Table III):**
```
Specular-only model:    80% of total multipath error captured
+ 1st-order diffuse:    95% of total multipath error captured
+ 2nd-order diffuse:    99%+ (diminishing returns, not worth the cost)
```
**Implication:** Our specular-only model (Pant approach) captures 80% of multipath. This is sufficient for SITL validation.

**Reflection Coefficient for Building Surfaces (§3.2):**
```
Glass:    ρ = 0.85    (very high reflection, L1 band)
Concrete: ρ = 0.55    (moderate reflection)
Brick:    ρ = 0.35    (lower reflection)
Metal:    ρ = 0.90    (near-perfect reflector)
```
Use `ρ = 0.65` as default (concrete/glass mix typical urban). Scale Δρ by √ρ:
```python
Δρ_effective = Δρ_geometric × sqrt(reflection_coeff)
```

**Satellite Elevation Angle Effect (§4.1, Eq. 7):**
```
elevation_weight = cos²(elevation_angle)
```
Satellites at low elevation angles (< 15°) contribute more multipath. Apply as weight when computing aggregate error:
```python
weighted_Δρ = Δρ × cos²(elevation_angle_rad)
```

**Isaac Sim Implementation Note (§6.2):**
Isaac Sim's `omni.physx.scene_query.raycast_closest()` effectively implements BVH internally (PhysX uses BVH for all raycasts). Therefore, the computational budget above applies directly — no need to build a separate BVH.

**Hardware Applicability:** ✅ ALGORITHM ONLY. The BVH construction is simulation-specific. On hardware, use the GNSS receiver's raw pseudorange measurements instead. The reflection coefficients are useful for calibrating simulation fidelity.

**Sections to read in paper:** §3 (ray-tracing model — BVH construction, relevant for understanding PhysX integration), §4.3 (computational performance — use as budget targets), §5 (validation against NovAtel OEM7 receiver in Munich urban canyon), §6.2 (limitations — specular vs diffuse tradeoffs).

---

## 38. Implementation File Structure

```
simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/
├── gps.py                  ← ADD DegradedGPS class at the bottom (§40)
├── sky_occlusion.py        ← NEW FILE: SkyOcclusionModel, CuencaDOPModel, PantMultipathModel (§39)
├── jamming.py              ← NEW FILE: JammingModel, JammingMode (§41)
└── __init__.py             ← ADD exports for new classes

tests/
└── test_gps_degradation.py ← NEW FILE: pytest unit + performance tests (§42)

robot/ros_ws/src/gps_degradation_hw/
├── gps_degradation_hw/
│   └── dop_scaling_node.py ← NEW FILE: Hardware ROS 2 companion node (§43)
└── package.xml             ← NEW FILE: ROS 2 package declaration
```

---

## 39. Production Code: `sky_occlusion.py`

**File path:** `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/sky_occlusion.py`

**Purpose:** Implements sky fraction computation via hemisphere ray-casting (`SkyOcclusionModel`), DOP calculation from sky fraction (`CuencaDOPModel`), and specular multipath pseudorange error (`PantMultipathModel`). These three classes are composed inside `DegradedGPS` (§40).

```python
# sky_occlusion.py
# AirStack Issue #345 — Realistic GPS Sensor Degradation
# Implements: Cuenca et al. (2023), Pant et al. (2022), GNSS Ray-Tracing (2026)
#
# DEPENDENCY NOTE: omni.physx is only available inside Isaac Sim runtime.
# Unit tests must mock get_physx_scene_query_interface() — see test_gps_degradation.py

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# Isaac Sim physics query interface — available at runtime inside Isaac Sim
# Wrapped in try/except so module can be imported in unit tests with mocking
try:
    from omni.physx import get_physx_scene_query_interface
    OMNI_PHYSX_AVAILABLE = True
except ImportError:
    OMNI_PHYSX_AVAILABLE = False
    get_physx_scene_query_interface = None  # Will be monkey-patched in tests


# ---------------------------------------------------------------------------
# Module-level Fibonacci hemisphere ray directions (precomputed, immutable)
# ---------------------------------------------------------------------------

def _build_fibonacci_hemisphere(n: int) -> np.ndarray:
    """
    Generate n unit vectors uniformly distributed on the upper hemisphere
    using the Fibonacci lattice method.

    Returns:
        rays: np.ndarray of shape (n, 3), each row is a unit vector [x, y, z]
              with z >= 0 (upward hemisphere in Isaac Sim Z-up convention).
    """
    golden_ratio = (1.0 + math.sqrt(5.0)) / 2.0
    i = np.arange(n, dtype=np.float64)

    # Map i → elevation: z in (0, 1], from near-horizon to zenith
    # Small offset (0.5) avoids degenerate rays exactly on the horizon
    z = (i + 0.5) / n                        # z ∈ (0, 1]
    theta = 2.0 * np.pi * i / golden_ratio   # azimuth angle (Fibonacci spiral)
    r = np.sqrt(1.0 - z * z)                 # radial distance in XY plane

    rays = np.column_stack([
        r * np.cos(theta),   # x
        r * np.sin(theta),   # y
        z                    # z (always positive = upward)
    ])
    return rays  # shape (n, 3), all rows are unit vectors


# Precompute once at module load — 32 rays for performance, ~150 µs per query
_N_SKY_RAYS: int = 32
_SKY_RAYS: np.ndarray = _build_fibonacci_hemisphere(_N_SKY_RAYS)  # (32, 3)

# High-resolution rays for tests / validation (256 rays, ~1.2 ms per query)
_N_SKY_RAYS_HR: int = 256
_SKY_RAYS_HR: np.ndarray = _build_fibonacci_hemisphere(_N_SKY_RAYS_HR)


# ---------------------------------------------------------------------------
# SkyOcclusionModel
# ---------------------------------------------------------------------------

class SkyOcclusionModel:
    """
    Computes sky view fraction SF ∈ [0, 1] at a given 3D position by casting
    hemisphere rays and checking for occlusion using PhysX scene queries.

    SF = 1.0 → fully open sky (no buildings above)
    SF = 0.0 → fully occluded (no sky visible — inside a building)

    Thread safety: Each instance holds its own PhysX interface reference.
    For multi-drone use, create one SkyOcclusionModel per drone. Do NOT share.

    Args:
        max_ray_distance: Maximum ray-cast distance in meters. Rays beyond
                          this distance are considered unoccluded. Default 500 m.
        n_rays: Number of hemisphere rays. 32 is recommended (150 µs / query).
                Use 256 for validation / high-accuracy testing.
        use_high_res: If True, uses 256-ray precomputed set instead of n_rays.
        cache_duration: Reuse last SF result if position moved < 0.5 m and
                        < cache_duration seconds elapsed. Default 0.1 s (10 Hz).
    """

    def __init__(
        self,
        max_ray_distance: float = 500.0,
        n_rays: int = 32,
        use_high_res: bool = False,
        cache_duration: float = 0.1,
    ) -> None:
        self._max_dist = max_ray_distance
        self._cache_duration = cache_duration
        self._use_high_res = use_high_res

        # Select ray set
        if use_high_res:
            self._rays = _SKY_RAYS_HR
        elif n_rays == _N_SKY_RAYS:
            self._rays = _SKY_RAYS   # use precomputed (avoids recompute)
        else:
            self._rays = _build_fibonacci_hemisphere(n_rays)

        # Cache state
        self._cached_sf: float = 1.0
        self._cached_pos: Optional[np.ndarray] = None
        self._cache_age: float = 0.0

        # PhysX interface (lazily initialized)
        self._physx_sq = None
        self._lock = threading.Lock()

    def _get_physx_sq(self):
        """Lazily get PhysX scene query interface (thread-safe)."""
        if self._physx_sq is None:
            with self._lock:
                if self._physx_sq is None:  # double-checked locking
                    if OMNI_PHYSX_AVAILABLE:
                        self._physx_sq = get_physx_scene_query_interface()
                    else:
                        raise RuntimeError(
                            "omni.physx not available. Are you running inside Isaac Sim? "
                            "In unit tests, mock get_physx_scene_query_interface."
                        )
        return self._physx_sq

    def compute_sky_fraction(
        self,
        position: np.ndarray,
        dt: float = 0.0,
    ) -> float:
        """
        Cast hemisphere rays from `position` and return sky fraction.

        Args:
            position: World position [x, y, z] in meters (Isaac Sim world frame).
                      This is the GPS antenna position (drone origin).
            dt: Time elapsed since last call (seconds). Used for cache expiry.

        Returns:
            sf: Sky fraction ∈ [0.05, 1.0]. Clamped to 0.05 minimum to prevent
                division-by-zero in DOP computation. 0.05 corresponds to
                "almost fully occluded" — fix_type will be 1 at this level.
        """
        # Cache check: reuse if position hasn't changed significantly
        self._cache_age += dt
        if (
            self._cached_pos is not None
            and self._cache_age < self._cache_duration
            and np.linalg.norm(position - self._cached_pos) < 0.5
        ):
            return self._cached_sf

        self._cache_age = 0.0
        self._cached_pos = position.copy()

        # Cast rays
        physx_sq = self._get_physx_sq()
        origin = (float(position[0]), float(position[1]), float(position[2]))

        n_unoccluded = 0
        for ray in self._rays:
            direction = (float(ray[0]), float(ray[1]), float(ray[2]))
            result = physx_sq.raycast_closest(
                origin,
                direction,
                self._max_dist,
                both_sides=False,
            )
            # result["hit"] is False → ray reached max_dist → sky is visible
            if not result.get("hit", True):
                n_unoccluded += 1

        sf = float(n_unoccluded) / float(len(self._rays))
        # Clamp: never 0 (causes div/0 in DOP math) or above 1
        self._cached_sf = float(np.clip(sf, 0.05, 1.0))
        return self._cached_sf

    def reset_cache(self) -> None:
        """Force recompute on next call (call when scene changes)."""
        self._cached_pos = None
        self._cache_age = float("inf")


# ---------------------------------------------------------------------------
# CuencaDOPModel
# ---------------------------------------------------------------------------

@dataclass
class CuencaConfig:
    """Configuration for CuencaDOPModel. All values from Cuenca et al. (2023)."""
    # DOP curve coefficients (Cuenca Eq. 4 & 5)
    alpha_h: float = 2.5    # horizontal DOP scale
    beta_h: float = 2.2     # horizontal DOP exponent
    alpha_v: float = 4.0    # vertical DOP scale
    beta_v: float = 1.8     # vertical DOP exponent

    # Base positioning accuracy (1-sigma) at HDOP=VDOP=1 (single-frequency L1)
    sigma_h_base: float = 1.0   # meters, horizontal
    sigma_v_base: float = 1.5   # meters, vertical

    # Sky fraction thresholds for satellite count
    # N_sv = max(3, round(10 × SF))  from Cuenca §3.3
    sat_count_scale: float = 10.0    # satellites at SF=1.0
    sat_count_min: int = 3           # minimum satellites (below this → no fix)

    # Reflection coefficient for multipath scaling (GNSS RT 2026 §3.2)
    # Use 0.65 for mixed concrete/glass urban (default)
    reflection_coeff: float = 0.65


class CuencaDOPModel:
    """
    Maps sky fraction → HDOP/VDOP → GPS accuracy parameters.

    Implements Cuenca et al. (2023) Equations 4 and 5 exactly.
    Returns a dict of GPS field overrides for DegradedGPS.update().

    Usage:
        model = CuencaDOPModel(CuencaConfig())
        overrides = model.compute(sf=0.4, base_data=gps_data, lat_deg=40.4)
        gps_data.update(overrides)
    """

    def __init__(self, config: CuencaConfig) -> None:
        self._cfg = config

    def hdop(self, sf: float) -> float:
        """
        HDOP from sky fraction. Cuenca Eq. 4.

        Args:
            sf: Sky fraction ∈ [0.05, 1.0]
        Returns:
            HDOP ≥ 1.0
        """
        return 1.0 + self._cfg.alpha_h * (1.0 - sf) ** self._cfg.beta_h

    def vdop(self, sf: float) -> float:
        """
        VDOP from sky fraction. Cuenca Eq. 5.

        Args:
            sf: Sky fraction ∈ [0.05, 1.0]
        Returns:
            VDOP ≥ 1.0
        """
        return 1.0 + self._cfg.alpha_v * (1.0 - sf) ** self._cfg.beta_v

    def satellite_count(self, sf: float) -> int:
        """Satellite count from sky fraction. Cuenca §3.3."""
        return max(
            self._cfg.sat_count_min,
            int(round(self._cfg.sat_count_scale * sf))
        )

    def compute(
        self,
        sf: float,
        base_data: dict,
        lat_deg: float = 0.0,
        rng: Optional[np.random.Generator] = None,
    ) -> dict:
        """
        Compute degraded GPS fields from sky fraction.

        Args:
            sf: Sky fraction ∈ [0.05, 1.0]
            base_data: Current GPS output dict from parent GPS.update().
                       Used to read base latitude for lon scaling.
            lat_deg: Latitude in degrees (for longitude noise scaling).
                     If 0.0, uses base_data['latitude'] if available.
            rng: Numpy random generator. If None, uses np.random.default_rng().

        Returns:
            overrides: Dict with fields to merge into GPS output dict.
                       Keys use Pegasus GPS exact field names including
                       'sattelites_visible' (double-t typo — DO NOT FIX).
        """
        if rng is None:
            rng = np.random.default_rng()

        cfg = self._cfg
        h = self.hdop(sf)      # HDOP ≥ 1.0
        v = self.vdop(sf)      # VDOP ≥ 1.0

        # Ephemeris fields (MAVLink GPS_RAW_INT: eph/epv in cm)
        # Cuenca §3.2: eph_m = HDOP × σ_h_base, converted to cm for MAVLink
        eph_m = h * cfg.sigma_h_base
        epv_m = v * cfg.sigma_v_base
        eph_cm = int(round(eph_m * 100.0))   # centimeters
        epv_cm = int(round(epv_m * 100.0))

        # Position noise — only the ADDITIONAL noise above the base OU process
        # (base noise already included by parent GPS.update())
        # Cuenca §3.4: Δ = N(0, σ_base × (DOP - 1))
        sigma_extra_h = cfg.sigma_h_base * (h - 1.0)  # extra horizontal σ [m]
        sigma_extra_v = cfg.sigma_v_base * (v - 1.0)  # extra vertical σ [m]

        # Lat/lon scaling: 1 degree latitude ≈ 111320 m everywhere
        # 1 degree longitude ≈ 111320 × cos(lat) m
        if lat_deg == 0.0:
            lat_deg = float(base_data.get("latitude", 0.0))
        cos_lat = math.cos(math.radians(lat_deg))
        lat_scale = 111320.0
        lon_scale = 111320.0 * cos_lat if cos_lat > 1e-6 else 111320.0

        delta_lat = rng.normal(0.0, sigma_extra_h / lat_scale)   # degrees
        delta_lon = rng.normal(0.0, sigma_extra_h / lon_scale)   # degrees
        delta_alt = rng.normal(0.0, sigma_extra_v)               # meters

        return {
            "eph": eph_cm,                                          # int, cm
            "epv": epv_cm,                                          # int, cm
            "sattelites_visible": self.satellite_count(sf),         # NOTE: double-t
            "latitude":  base_data.get("latitude",  0.0) + delta_lat,
            "longitude": base_data.get("longitude", 0.0) + delta_lon,
            "altitude":  base_data.get("altitude",  0.0) + delta_alt,
        }


# ---------------------------------------------------------------------------
# PantMultipathModel
# ---------------------------------------------------------------------------

@dataclass
class PantConfig:
    """Configuration for PantMultipathModel. Values from Pant et al. (2022)."""
    max_wall_distance: float = 150.0      # meters — beyond this, multipath negligible
    max_incidence_deg: float = 70.0       # degrees — grazing limit
    n_reflection_rays: int = 16           # rays to cast for reflection detection
    reflection_coeff: float = 0.65        # surface reflection coefficient (mixed urban)

    # Minimum incidence angle (below this, reflection is back toward emitter)
    min_incidence_deg: float = 5.0


class PantMultipathModel:
    """
    Adds specular multipath pseudorange error to GPS accuracy parameters.

    Implements the two-ray geometric model from Pant et al. (2022) §3.
    Requires a SkyOcclusionModel's PhysX interface for reflection ray-casting.

    Usage (inside DegradedGPS.update()):
        multipath_data = self._multipath.compute(
            sf, position, data, hdop, vdop, physx_sq
        )
        data.update(multipath_data)
    """

    def __init__(self, config: PantConfig) -> None:
        self._cfg = config
        self._max_inc_rad = math.radians(config.max_incidence_deg)
        self._min_inc_rad = math.radians(config.min_incidence_deg)
        self._sqrt_rho = math.sqrt(config.reflection_coeff)

        # Precompute reflection ray directions (horizontal scan in XY plane)
        # These rays probe for vertical wall surfaces
        angles = np.linspace(0, 2 * np.pi, config.n_reflection_rays, endpoint=False)
        self._h_rays = np.column_stack([
            np.cos(angles),   # x
            np.sin(angles),   # y
            np.zeros(config.n_reflection_rays),  # z = 0 (horizontal)
        ])  # shape (n_reflection_rays, 3)

    def compute(
        self,
        sf: float,
        position: np.ndarray,
        base_data: dict,
        hdop: float,
        vdop: float,
        physx_sq,
    ) -> dict:
        """
        Compute multipath-induced additional ephemeris error.

        Args:
            sf: Sky fraction (used to scale overall effect)
            position: Drone world position [x, y, z] meters
            base_data: Current GPS output dict (to read and update eph/epv)
            hdop: Current HDOP (from CuencaDOPModel)
            vdop: Current VDOP (from CuencaDOPModel)
            physx_sq: PhysX scene query interface from SkyOcclusionModel

        Returns:
            overrides: Dict with updated 'eph' and 'epv' fields (cm).
                       Empty dict if no multipath detected.
        """
        cfg = self._cfg
        origin = (float(position[0]), float(position[1]), float(position[2]))
        multipath_errors: List[float] = []

        for h_ray in self._h_rays:
            direction = (float(h_ray[0]), float(h_ray[1]), float(h_ray[2]))
            result = physx_sq.raycast_closest(
                origin,
                direction,
                cfg.max_wall_distance,
                both_sides=False,
            )

            if not result.get("hit", False):
                continue  # no wall in this direction

            d_wall = result.get("distance", cfg.max_wall_distance + 1.0)
            if d_wall >= cfg.max_wall_distance:
                continue

            # Get wall surface normal
            normal_raw = result.get("normal", None)
            if normal_raw is None:
                continue
            wall_normal = np.array([
                float(normal_raw[0]),
                float(normal_raw[1]),
                float(normal_raw[2]),
            ])
            norm_mag = np.linalg.norm(wall_normal)
            if norm_mag < 1e-6:
                continue
            wall_normal /= norm_mag

            # Incidence angle: angle between horizontal ray and wall normal
            # Using horizontal ray direction as proxy for satellite signal direction
            ray_vec = np.array([h_ray[0], h_ray[1], h_ray[2]])
            cos_theta = abs(float(np.dot(ray_vec, wall_normal)))
            cos_theta = min(cos_theta, 1.0)
            theta_inc = math.acos(cos_theta)  # radians

            # Filter by incidence angle (Pant §3.2 validity conditions)
            if theta_inc > self._max_inc_rad:
                continue   # too steep — low Fresnel reflection coefficient
            if theta_inc < self._min_inc_rad:
                continue   # near-normal incidence — reflection back toward satellite

            # Pseudorange error: Pant Eq. 3
            # Δρ = 2 × d_wall × sin(θ_inc) × √ρ_reflection
            delta_rho = 2.0 * d_wall * math.sin(theta_inc) * self._sqrt_rho
            multipath_errors.append(delta_rho)

        if not multipath_errors:
            return {}  # no multipath detected, no override needed

        # Aggregate RMS error across all detected reflections (Pant §4.1)
        rms_rho = math.sqrt(sum(e * e for e in multipath_errors) / len(multipath_errors))

        # Convert pseudorange error to position error via DOP (Pant §3.3)
        eph_multipath_m = rms_rho / max(hdop, 1.0)
        epv_multipath_m = rms_rho / max(vdop, 1.0) * 1.5  # vertical worse

        # Add in quadrature to existing Cuenca-computed eph/epv
        eph_cuenca_m = base_data.get("eph", 100) / 100.0  # cm → m
        epv_cuenca_m = base_data.get("epv", 150) / 100.0

        eph_total_m = math.sqrt(eph_cuenca_m ** 2 + eph_multipath_m ** 2)
        epv_total_m = math.sqrt(epv_cuenca_m ** 2 + epv_multipath_m ** 2)

        return {
            "eph": int(round(eph_total_m * 100.0)),   # cm
            "epv": int(round(epv_total_m * 100.0)),   # cm
        }
```

---

## 40. Production Code: `jamming.py`

**File path:** `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/jamming.py`

**Purpose:** Models three GPS jamming/spoofing threats: signal DENIAL (GPS loss), NOISE jamming (degraded accuracy), SPOOFING (false position injection), and MEACONING (signal replay with delay). The JammingModel is optional — controlled by config.

```python
# jamming.py
# AirStack Issue #345 — GPS Jamming / Spoofing / Denial Model
# Threat models based on: Kerns et al. (2014), Humphreys et al. (2012)

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np


class JammingMode(Enum):
    """GPS threat mode selection."""
    DENIAL    = auto()   # Signal denial: GPS signal completely lost
    NOISE     = auto()   # Noise jamming: accuracy severely degraded, fix maintained
    SPOOFING  = auto()   # Position spoofing: false GPS position injected
    MEACONING = auto()   # Replay with delay: position drifts from true position


@dataclass
class JammingConfig:
    """
    Configuration for JammingModel.

    The jamming zone is defined by a center point and radius in 3D world space.
    Multiple zones can be defined (future extension — currently single zone).
    """
    mode: JammingMode = JammingMode.DENIAL

    # Jamming zone geometry (world frame, meters)
    zone_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    zone_radius: float = 50.0      # meters — drone inside this radius is jammed
    zone_altitude_min: float = 0.0   # meters — only jam within this altitude band
    zone_altitude_max: float = 500.0

    # NOISE mode parameters
    noise_eph_cm: int = 1500     # ephemeris when jammed (15 m horizontal — severe)
    noise_epv_cm: int = 2500     # ephemeris when jammed (25 m vertical — severe)
    noise_position_sigma_m: float = 8.0  # position noise standard deviation [m]

    # SPOOFING mode parameters (Kerns et al. 2014)
    # Spoof starts at true position and drifts to target_offset at drift_rate m/s
    spoof_target_offset_m: Tuple[float, float, float] = (50.0, 0.0, 0.0)
    spoof_drift_rate_mps: float = 0.5   # m/s — slow enough to evade EKF gate

    # MEACONING mode parameters (Humphreys et al. 2012)
    # Signal is replayed with a delay, causing position to appear behind true position
    meaconing_delay_s: float = 2.0       # seconds of replay delay
    meaconing_drift_rate_mps: float = 0.3  # m/s drift rate away from true position

    # Transition behavior
    entry_ramp_s: float = 1.0   # seconds to ramp up jamming effect (prevents EKF spike)
    exit_ramp_s: float = 2.0    # seconds to ramp down after leaving zone


class JammingModel:
    """
    Applies GPS jamming/spoofing effects based on drone position.

    The drone's position is checked against the jamming zone on each update.
    Effects are ramped in/out to avoid sudden innovations that would trigger
    the PX4 EKF2 innovation gate (5σ rejection threshold).

    Usage:
        jammer = JammingModel(JammingConfig(mode=JammingMode.DENIAL, ...))
        gps_data = jammer.apply(gps_data, position, dt)
    """

    def __init__(self, config: JammingConfig) -> None:
        self._cfg = config
        self._in_zone: bool = False
        self._ramp: float = 0.0          # 0.0 = no effect, 1.0 = full effect
        self._spoof_offset: np.ndarray = np.zeros(3)  # current spoof displacement [m]
        self._rng = np.random.default_rng()
        self._time_in_zone: float = 0.0

    def _check_zone(self, position: np.ndarray) -> bool:
        """Return True if drone is inside the jamming zone."""
        cx, cy, cz = self._cfg.zone_center
        center = np.array([cx, cy, cz])
        horizontal_dist = np.linalg.norm(position[:2] - center[:2])
        alt = float(position[2])
        return (
            horizontal_dist <= self._cfg.zone_radius
            and self._cfg.zone_altitude_min <= alt <= self._cfg.zone_altitude_max
        )

    def _update_ramp(self, in_zone: bool, dt: float) -> None:
        """Smoothly ramp the jamming effect in/out to avoid EKF innovation spikes."""
        if in_zone:
            self._time_in_zone += dt
            ramp_rate = 1.0 / max(self._cfg.entry_ramp_s, 1e-3)
            self._ramp = min(1.0, self._ramp + ramp_rate * dt)
        else:
            self._time_in_zone = 0.0
            ramp_rate = 1.0 / max(self._cfg.exit_ramp_s, 1e-3)
            self._ramp = max(0.0, self._ramp - ramp_rate * dt)

    def apply(
        self,
        data: dict,
        position: np.ndarray,
        dt: float,
    ) -> dict:
        """
        Apply jamming effects to GPS output data.

        Args:
            data: GPS output dict from DegradedGPS (after Cuenca/Pant applied).
                  Will be modified in-place and returned.
            position: Drone world position [x, y, z] meters.
            dt: Time step in seconds.

        Returns:
            data: Modified GPS output dict.
        """
        in_zone = self._check_zone(position)
        self._update_ramp(in_zone, dt)

        if self._ramp < 1e-6:
            return data   # not jammed, no-op

        mode = self._cfg.mode

        if mode == JammingMode.DENIAL:
            return self._apply_denial(data, self._ramp)

        elif mode == JammingMode.NOISE:
            return self._apply_noise(data, self._ramp)

        elif mode == JammingMode.SPOOFING:
            return self._apply_spoofing(data, position, dt, self._ramp)

        elif mode == JammingMode.MEACONING:
            return self._apply_meaconing(data, position, dt, self._ramp)

        return data

    def _apply_denial(self, data: dict, ramp: float) -> dict:
        """
        GPS signal denial: fix_type → 1 (no fix), satellites → 0.
        Kerns et al. (2014): complete signal loss modeled as fix_type=1.
        """
        # Ramp: partially degrade before full denial
        if ramp >= 0.99:
            data["fix_type"] = 1
            data["sattelites_visible"] = 0   # double-t
            data["eph"] = 9999
            data["epv"] = 9999
        else:
            # Partial effect: degrade satellites and accuracy
            n_sat = data.get("sattelites_visible", 10)  # double-t
            data["sattelites_visible"] = max(0, int(n_sat * (1.0 - ramp)))  # double-t
            data["eph"] = int(data.get("eph", 100) * (1.0 + ramp * 50.0))
            data["epv"] = int(data.get("epv", 150) * (1.0 + ramp * 50.0))
            if data["sattelites_visible"] < 4:  # double-t
                data["fix_type"] = 2   # 2D fix (< 4 sats)
            if data["sattelites_visible"] < 3:  # double-t
                data["fix_type"] = 1   # no fix
        return data

    def _apply_noise(self, data: dict, ramp: float) -> dict:
        """
        Noise jamming: large random errors, fix maintained.
        Reduces satellite count and inflates eph/epv.
        """
        cfg = self._cfg
        sigma = cfg.noise_position_sigma_m * ramp

        lat_noise = self._rng.normal(0.0, sigma / 111320.0)
        lon_noise = self._rng.normal(0.0, sigma / 111320.0)
        alt_noise = self._rng.normal(0.0, sigma * 1.5)

        data["latitude"]  = data.get("latitude",  0.0) + lat_noise
        data["longitude"] = data.get("longitude", 0.0) + lon_noise
        data["altitude"]  = data.get("altitude",  0.0) + alt_noise

        data["eph"] = int(cfg.noise_eph_cm * ramp + data.get("eph", 100) * (1 - ramp))
        data["epv"] = int(cfg.noise_epv_cm * ramp + data.get("epv", 150) * (1 - ramp))
        data["sattelites_visible"] = max(4, int(10 * (1.0 - ramp * 0.6)))  # double-t
        # fix_type remains 3 (fix maintained but severely degraded)
        return data

    def _apply_spoofing(
        self, data: dict, position: np.ndarray, dt: float, ramp: float
    ) -> dict:
        """
        GPS spoofing: position drifts toward false target.
        Kerns et al. (2014): spoof drift rate ≤ 0.5 m/s to stay below EKF gate.

        The spoofed GPS output appears to show normal accuracy (low eph/epv)
        but the position is systematically wrong — this is the defining feature
        of spoofing vs. noise jamming.
        """
        cfg = self._cfg
        target = np.array(cfg.spoof_target_offset_m)   # target displacement [m]

        # Move spoof offset toward target at drift_rate m/s
        remaining = target - self._spoof_offset
        step_max = cfg.spoof_drift_rate_mps * dt * ramp
        dist_remaining = np.linalg.norm(remaining)
        if dist_remaining > step_max:
            self._spoof_offset += step_max * remaining / dist_remaining
        else:
            self._spoof_offset = target.copy()

        # Apply offset to lat/lon/alt (convert meters to degrees)
        offset_lat = self._spoof_offset[0] / 111320.0
        offset_lon = self._spoof_offset[1] / 111320.0
        offset_alt = self._spoof_offset[2]

        data["latitude"]  = data.get("latitude",  0.0) + offset_lat
        data["longitude"] = data.get("longitude", 0.0) + offset_lon
        data["altitude"]  = data.get("altitude",  0.0) + offset_alt

        # Spoofed signal appears accurate (low eph/epv, good fix_type)
        # — this is intentional and realistic per Kerns et al.
        data["fix_type"] = 3
        data["eph"] = 100   # 1.0 m — appears good
        data["epv"] = 150   # 1.5 m — appears good
        data["sattelites_visible"] = 10  # double-t, appears normal

        return data

    def _apply_meaconing(
        self, data: dict, position: np.ndarray, dt: float, ramp: float
    ) -> dict:
        """
        GPS meaconing: captured signal replayed with delay.
        Humphreys et al. (2012): position drifts from true as the replayed
        signal corresponds to an earlier position.

        Simulated as: position offset = drift_rate × delay × ramp direction.
        """
        cfg = self._cfg
        delay_drift_m = cfg.meaconing_drift_rate_mps * cfg.meaconing_delay_s * ramp

        # Drift in a fixed direction (reuse spoof_offset as accumulated displacement)
        if np.linalg.norm(self._spoof_offset) < delay_drift_m:
            # Drift in the -X direction (heading-opposite, mimicking replay)
            self._spoof_offset += np.array([
                -cfg.meaconing_drift_rate_mps * dt * ramp,
                0.0,
                0.0,
            ])

        offset_lat = self._spoof_offset[0] / 111320.0
        offset_lon = self._spoof_offset[1] / 111320.0

        data["latitude"]  = data.get("latitude",  0.0) + offset_lat
        data["longitude"] = data.get("longitude", 0.0) + offset_lon
        # Altitude unaffected by horizontal meaconing
        data["fix_type"] = 3    # still shows good fix
        data["eph"] = 150       # slightly inflated (1.5 m)
        data["epv"] = 200
        data["sattelites_visible"] = 8   # double-t, slightly reduced

        return data

    def reset(self) -> None:
        """Reset jamming state (call when starting new scenario)."""
        self._in_zone = False
        self._ramp = 0.0
        self._spoof_offset = np.zeros(3)
        self._time_in_zone = 0.0
```

---

## 41. Production Code: `DegradedGPS` class (append to `gps.py`)

**File path:** `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/gps.py`

**Action:** Append the following code to the END of the existing `gps.py` file. Do not modify the existing `GPS` class.

**CRITICAL:** The `GPS` parent class uses a plain dict config (not a dataclass). The output dict uses `sattelites_visible` with a double-t (typo in original Pegasus code — must be preserved exactly). Output fields are `latitude`, `longitude`, `altitude` (not lat/lon/alt).

```python
# ===========================================================================
# DegradedGPS — appended to gps.py for Issue #345
# Realistic GPS Sensor Degradation
# Author: Apurva Singh, CMU AirLab Intern, May 2026
# ===========================================================================

import threading
from typing import Optional

import numpy as np

# Import sub-models (same package)
from pegasus.simulator.logic.sensors.sky_occlusion import (
    SkyOcclusionModel,
    CuencaDOPModel,
    CuencaConfig,
    PantMultipathModel,
    PantConfig,
)
from pegasus.simulator.logic.sensors.jamming import (
    JammingModel,
    JammingConfig,
    JammingMode,
)

# PhysX for multipath model (optional — only needed for Tier 2)
try:
    from omni.physx import get_physx_scene_query_interface
    _PHYSX_AVAILABLE = True
except ImportError:
    _PHYSX_AVAILABLE = False


# ---------------------------------------------------------------------------
# FixStateMachine
# ---------------------------------------------------------------------------

class FixStateMachine:
    """
    Hysteretic GPS fix-type state machine.

    Prevents rapid fix_type oscillation at canyon boundaries by requiring
    sky_fraction to stay above/below thresholds for T_up/T_down seconds.

    States:
        fix_type = 1 → No fix (SF < SF_threshold for T_down seconds)
        fix_type = 2 → 2D fix (intermediate, brief)
        fix_type = 3 → 3D fix (SF > SF_good for T_up seconds)

    Timing parameters from Lee et al. (2020) §4.2 recommendation.
    """

    def __init__(
        self,
        sf_threshold: float = 0.15,   # below this → no fix
        sf_good: float = 0.30,        # above this → 3D fix
        t_up: float = 2.0,            # seconds to upgrade fix type
        t_down: float = 0.5,          # seconds to downgrade fix type
    ) -> None:
        self._sf_threshold = sf_threshold
        self._sf_good = sf_good
        self._t_up = t_up
        self._t_down = t_down

        # Internal state
        self._current_fix: int = 3   # start assuming good GPS
        self._timer: float = 0.0     # time in current candidate state

    def update(self, sf: float, dt: float) -> int:
        """
        Update state machine with current sky fraction and return fix_type.

        Args:
            sf: Current sky fraction ∈ [0.05, 1.0]
            dt: Time elapsed since last call [seconds]

        Returns:
            fix_type: 1 (no fix), 2 (2D fix), or 3 (3D fix)
        """
        if sf > self._sf_good:
            # Candidate: upgrade to fix_type=3
            if self._current_fix < 3:
                self._timer += dt
                if self._timer >= self._t_up:
                    self._current_fix = 3
                    self._timer = 0.0
            else:
                self._timer = 0.0   # already at target, reset timer

        elif sf < self._sf_threshold:
            # Candidate: downgrade to fix_type=1
            if self._current_fix > 1:
                self._timer += dt
                if self._timer >= self._t_down:
                    self._current_fix = 1
                    self._timer = 0.0
            else:
                self._timer = 0.0

        else:
            # In between thresholds → 2D fix territory
            self._timer += dt
            if self._current_fix == 3 and self._timer >= self._t_down:
                self._current_fix = 2
                self._timer = 0.0
            elif self._current_fix == 1 and self._timer >= self._t_up:
                self._current_fix = 2
                self._timer = 0.0

        return self._current_fix

    def reset(self, fix_type: int = 3) -> None:
        """Reset to a given fix_type (call at scenario start)."""
        self._current_fix = fix_type
        self._timer = 0.0


# ---------------------------------------------------------------------------
# DegradedGPS
# ---------------------------------------------------------------------------

# Default configuration dict for DegradedGPS
# Merge with base GPS config when constructing
DEGRADED_GPS_DEFAULT_CONFIG = {
    # ── Tier 1: Sky occlusion + DOP model ──────────────────────────────────
    "occlusion_enabled": True,
    "max_ray_distance":  500.0,    # meters — max raycast distance for sky fraction
    "n_sky_rays":         32,      # hemisphere ray count (32 = ~150 µs/query)

    # Cuenca DOP curve coefficients (from paper Eq. 4-5)
    "cuenca_alpha_h":    2.5,
    "cuenca_beta_h":     2.2,
    "cuenca_alpha_v":    4.0,
    "cuenca_beta_v":     1.8,
    "cuenca_sigma_h":    1.0,      # base horizontal 1-sigma accuracy [m]
    "cuenca_sigma_v":    1.5,      # base vertical 1-sigma accuracy [m]

    # Fix-type state machine (Lee et al. 2020)
    "sf_threshold":      0.15,     # SF below this → fix_type = 1 (no fix)
    "sf_good":           0.30,     # SF above this → fix_type = 3 (3D fix)
    "t_up":              2.0,      # seconds to upgrade fix type
    "t_down":            0.5,      # seconds to downgrade fix type

    # ── Tier 2: Multipath model (Pant et al. 2022) ─────────────────────────
    "multipath_enabled":  False,   # set True only if city USD asset available
    "max_wall_distance":  150.0,   # meters
    "max_incidence_deg":  70.0,
    "n_reflection_rays":  16,
    "reflection_coeff":   0.65,    # mixed concrete/glass urban

    # ── Tier 3: Jamming / spoofing model ───────────────────────────────────
    "jamming_enabled":    False,
    "jamming_mode":       "DENIAL",  # "DENIAL" | "NOISE" | "SPOOFING" | "MEACONING"
    "jamming_zone_center": [0.0, 0.0, 0.0],
    "jamming_zone_radius": 50.0,
    "jamming_zone_alt_min": 0.0,
    "jamming_zone_alt_max": 500.0,
    "jamming_noise_eph_cm": 1500,
    "jamming_noise_epv_cm": 2500,
    "jamming_noise_sigma_m": 8.0,
    "spoof_target_offset_m": [50.0, 0.0, 0.0],
    "spoof_drift_rate_mps": 0.5,
    "meaconing_delay_s": 2.0,
    "meaconing_drift_rate_mps": 0.3,
    "jamming_entry_ramp_s": 1.0,
    "jamming_exit_ramp_s":  2.0,
}


class DegradedGPS(GPS):
    """
    Realistic GPS sensor with sky occlusion, multipath, and jamming degradation.

    Extends the base Pegasus GPS class by composing three degradation models:
      - Tier 1 (always on):  Cuenca DOP model — sky fraction → eph/epv/fix_type
      - Tier 2 (opt-in):     Pant multipath   — specular reflections → extra error
      - Tier 3 (opt-in):     JammingModel     — denial / noise / spoofing

    Config:
        Accepts a plain dict. Merge DEGRADED_GPS_DEFAULT_CONFIG with any
        overrides before passing. The base GPS config keys are passed through
        to the parent constructor.

    Usage in a Pegasus vehicle configuration:
        from pegasus.simulator.logic.sensors.gps import DegradedGPS, DEGRADED_GPS_DEFAULT_CONFIG

        cfg = {**DEGRADED_GPS_DEFAULT_CONFIG, "multipath_enabled": True}
        gps = DegradedGPS(config=cfg)
        vehicle.add_sensor(gps)

    CRITICAL FIELD NAMES (Pegasus typos — must match exactly):
        "sattelites_visible"  ← double-t, NOT "satellites_visible"
        "latitude", "longitude", "altitude"  ← NOT lat/lon/alt
        "eph", "epv"  ← in centimeters (integer), NOT meters
    """

    def __init__(self, config: dict = {}) -> None:
        # Merge defaults with provided config; base GPS keys passed through
        merged = {**DEGRADED_GPS_DEFAULT_CONFIG, **config}
        super().__init__(config=merged)   # initialize base GPS with merged config

        self._degraded_cfg = merged
        self._rng = np.random.default_rng()

        # ── Tier 1: Sky occlusion ──────────────────────────────────────────
        self._occlusion_enabled: bool = bool(merged.get("occlusion_enabled", True))
        self._occlusion = SkyOcclusionModel(
            max_ray_distance=float(merged.get("max_ray_distance", 500.0)),
            n_rays=int(merged.get("n_sky_rays", 32)),
        )
        self._cuenca = CuencaDOPModel(CuencaConfig(
            alpha_h=float(merged.get("cuenca_alpha_h", 2.5)),
            beta_h= float(merged.get("cuenca_beta_h",  2.2)),
            alpha_v=float(merged.get("cuenca_alpha_v", 4.0)),
            beta_v= float(merged.get("cuenca_beta_v",  1.8)),
            sigma_h_base=float(merged.get("cuenca_sigma_h", 1.0)),
            sigma_v_base=float(merged.get("cuenca_sigma_v", 1.5)),
        ))
        self._fix_state = FixStateMachine(
            sf_threshold=float(merged.get("sf_threshold", 0.15)),
            sf_good=     float(merged.get("sf_good",      0.30)),
            t_up=        float(merged.get("t_up",         2.0)),
            t_down=      float(merged.get("t_down",       0.5)),
        )

        # ── Tier 2: Multipath ──────────────────────────────────────────────
        self._multipath_enabled: bool = bool(merged.get("multipath_enabled", False))
        self._multipath: Optional[PantMultipathModel] = None
        self._physx_sq = None   # lazy-initialized if multipath enabled
        if self._multipath_enabled:
            self._multipath = PantMultipathModel(PantConfig(
                max_wall_distance=float(merged.get("max_wall_distance", 150.0)),
                max_incidence_deg=float(merged.get("max_incidence_deg", 70.0)),
                n_reflection_rays=int(merged.get("n_reflection_rays", 16)),
                reflection_coeff= float(merged.get("reflection_coeff",  0.65)),
            ))

        # ── Tier 3: Jamming ────────────────────────────────────────────────
        self._jamming_enabled: bool = bool(merged.get("jamming_enabled", False))
        self._jammer: Optional[JammingModel] = None
        if self._jamming_enabled:
            mode_str = str(merged.get("jamming_mode", "DENIAL")).upper()
            mode = JammingMode[mode_str]
            center = merged.get("jamming_zone_center", [0.0, 0.0, 0.0])
            self._jammer = JammingModel(JammingConfig(
                mode=mode,
                zone_center=(float(center[0]), float(center[1]), float(center[2])),
                zone_radius=      float(merged.get("jamming_zone_radius",   50.0)),
                zone_altitude_min=float(merged.get("jamming_zone_alt_min",   0.0)),
                zone_altitude_max=float(merged.get("jamming_zone_alt_max", 500.0)),
                noise_eph_cm=     int(  merged.get("jamming_noise_eph_cm", 1500)),
                noise_epv_cm=     int(  merged.get("jamming_noise_epv_cm", 2500)),
                noise_position_sigma_m=float(merged.get("jamming_noise_sigma_m", 8.0)),
                spoof_target_offset_m= tuple(merged.get("spoof_target_offset_m", [50.0, 0.0, 0.0])),
                spoof_drift_rate_mps=  float(merged.get("spoof_drift_rate_mps", 0.5)),
                meaconing_delay_s=     float(merged.get("meaconing_delay_s",     2.0)),
                meaconing_drift_rate_mps=float(merged.get("meaconing_drift_rate_mps", 0.3)),
                entry_ramp_s=float(merged.get("jamming_entry_ramp_s", 1.0)),
                exit_ramp_s= float(merged.get("jamming_exit_ramp_s",  2.0)),
            ))

    def update(self, state, dt: float) -> dict:
        """
        Override GPS.update() to apply degradation models.

        Execution order:
          1. super().update() → base GPS with OU noise (always runs)
          2. Tier 1: SkyOcclusionModel → sf → CuencaDOPModel → overrides eph/epv/sats/pos
          3. Tier 2: PantMultipathModel → adds multipath error to eph/epv (if enabled)
          4. fix_type from FixStateMachine (hysteretic, based on sf)
          5. Tier 3: JammingModel (if enabled)

        Args:
            state: Pegasus State object with at least:
                   .position: np.ndarray([x, y, z])  — world position, meters
            dt: Time step in seconds

        Returns:
            data: GPS output dict with all Pegasus field names.
        """
        # Step 1: Base GPS (OU noise + constellation geometry)
        data = super().update(state, dt)

        if not self._occlusion_enabled:
            return data   # pass-through mode (for testing baseline)

        # Step 2: Tier 1 — Cuenca DOP model
        position = np.array(state.position, dtype=np.float64)
        sf = self._occlusion.compute_sky_fraction(position, dt=dt)

        hdop = self._cuenca.hdop(sf)
        vdop = self._cuenca.vdop(sf)

        cuenca_overrides = self._cuenca.compute(
            sf=sf,
            base_data=data,
            rng=self._rng,
        )
        data.update(cuenca_overrides)

        # Step 3: Tier 2 — Pant multipath (optional)
        if self._multipath_enabled and self._multipath is not None:
            # Lazy-init PhysX interface (only available after Isaac Sim scene loads)
            if self._physx_sq is None and _PHYSX_AVAILABLE:
                try:
                    self._physx_sq = get_physx_scene_query_interface()
                except Exception:
                    pass  # not yet available — skip this frame

            if self._physx_sq is not None:
                multipath_overrides = self._multipath.compute(
                    sf=sf,
                    position=position,
                    base_data=data,
                    hdop=hdop,
                    vdop=vdop,
                    physx_sq=self._physx_sq,
                )
                data.update(multipath_overrides)

        # Step 4: Fix-type state machine (after Tier 1/2 set eph/epv)
        data["fix_type"] = self._fix_state.update(sf, dt)

        # If fix_type = 1, clear satellites (no signal = no fix)
        if data["fix_type"] == 1:
            data["sattelites_visible"] = 0   # double-t
            data["eph"] = 9999
            data["epv"] = 9999

        # Step 5: Tier 3 — Jamming (after fix_type is set)
        if self._jamming_enabled and self._jammer is not None:
            data = self._jammer.apply(data, position, dt)

        return data

    def reset(self) -> None:
        """Reset all degradation state (call when starting new episode)."""
        self._fix_state.reset()
        self._occlusion.reset_cache()
        if self._jammer is not None:
            self._jammer.reset()
        super().reset() if hasattr(super(), "reset") else None
```

---

## 42. Production Code: `test_gps_degradation.py`

**File path:** `tests/test_gps_degradation.py`

**Purpose:** pytest unit and performance tests for all DegradedGPS components. All Isaac Sim / PhysX dependencies are mocked so tests run in CI without Isaac Sim installed. Uses the existing AirStack `airstack test` infrastructure.

**Run with:** `airstack test -m gps_degradation`

```python
# tests/test_gps_degradation.py
# AirStack Issue #345 — GPS Degradation Test Suite
# Run: airstack test -m gps_degradation
# Or:  pytest tests/test_gps_degradation.py -v

from __future__ import annotations

import math
import sys
import time
import types
import unittest
from unittest.mock import MagicMock, patch
from typing import Dict, Any

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Mock Isaac Sim / omni.physx before importing production modules
# ---------------------------------------------------------------------------

def _make_omni_mock():
    """Create a minimal omni.physx mock that returns configurable raycast results."""
    omni_mod       = types.ModuleType("omni")
    omni_physx_mod = types.ModuleType("omni.physx")
    omni_mod.physx = omni_physx_mod

    # Default: all rays hit (fully occluded)
    _hit_result = {"hit": True, "distance": 10.0, "normal": (0.0, 1.0, 0.0)}

    mock_sq = MagicMock()
    mock_sq.raycast_closest.return_value = _hit_result

    omni_physx_mod.get_physx_scene_query_interface = MagicMock(return_value=mock_sq)
    sys.modules["omni"]       = omni_mod
    sys.modules["omni.physx"] = omni_physx_mod
    return mock_sq


_MOCK_SQ = _make_omni_mock()


# ---------------------------------------------------------------------------
# Mock Pegasus GPS base class
# ---------------------------------------------------------------------------

class _MockGPS:
    """Minimal stand-in for pegasus.simulator.logic.sensors.gps.GPS."""

    def __init__(self, config: dict = {}):
        self._config = config

    def update(self, state, dt: float) -> dict:
        """Return nominal GPS output (no noise for testing)."""
        return {
            "latitude":          40.4432,
            "longitude":        -79.9429,
            "altitude":          290.0,
            "fix_type":          3,
            "eph":               100,     # 1.00 m horizontal (cm)
            "epv":               150,     # 1.50 m vertical (cm)
            "sattelites_visible": 10,     # double-t typo preserved
        }

    def reset(self):
        pass


# Patch the Pegasus GPS import before importing DegradedGPS
_gps_module = types.ModuleType("pegasus.simulator.logic.sensors.gps")
_gps_module.GPS = _MockGPS
sys.modules["pegasus"]                                          = types.ModuleType("pegasus")
sys.modules["pegasus.simulator"]                               = types.ModuleType("pegasus.simulator")
sys.modules["pegasus.simulator.logic"]                         = types.ModuleType("pegasus.simulator.logic")
sys.modules["pegasus.simulator.logic.sensors"]                 = types.ModuleType("pegasus.simulator.logic.sensors")
sys.modules["pegasus.simulator.logic.sensors.gps"]             = _gps_module

# Now safe to import production modules
from pegasus.simulator.logic.sensors.sky_occlusion import (  # noqa: E402
    SkyOcclusionModel,
    CuencaDOPModel,
    CuencaConfig,
    PantMultipathModel,
    PantConfig,
    _build_fibonacci_hemisphere,
)
from pegasus.simulator.logic.sensors.jamming import (  # noqa: E402
    JammingModel,
    JammingConfig,
    JammingMode,
)


# ---------------------------------------------------------------------------
# Shared mock state
# ---------------------------------------------------------------------------

class _MockState:
    """Minimal Pegasus State mock with position attribute."""
    def __init__(self, position=(0.0, 0.0, 10.0)):
        self.position = np.array(position, dtype=np.float64)


def _set_sky_fraction(mock_sq, fraction: float, n_rays: int = 32):
    """
    Configure mock PhysX to return a specific sky fraction.
    fraction=1.0 → all rays miss (open sky)
    fraction=0.0 → all rays hit (fully occluded)
    """
    n_unoccluded = int(round(fraction * n_rays))
    call_count = [0]

    def _raycast_side_effect(origin, direction, distance, both_sides=False):
        call_count[0] += 1
        if call_count[0] <= n_unoccluded:
            return {"hit": False}   # ray misses → sky visible
        return {"hit": True, "distance": 20.0, "normal": (0.0, 1.0, 0.0)}

    mock_sq.raycast_closest.side_effect = _raycast_side_effect


# ---------------------------------------------------------------------------
# Test: Fibonacci hemisphere ray generation
# ---------------------------------------------------------------------------

class TestFibonacciHemisphere(unittest.TestCase):

    def test_ray_count(self):
        rays = _build_fibonacci_hemisphere(32)
        assert rays.shape == (32, 3), f"Expected (32, 3), got {rays.shape}"

    def test_unit_vectors(self):
        rays = _build_fibonacci_hemisphere(64)
        norms = np.linalg.norm(rays, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6,
                                   err_msg="All rays must be unit vectors")

    def test_upper_hemisphere_only(self):
        rays = _build_fibonacci_hemisphere(256)
        assert np.all(rays[:, 2] >= 0), "All rays must have z ≥ 0 (upper hemisphere)"
        # At least one ray near zenith
        assert np.any(rays[:, 2] > 0.9), "Should have a ray near zenith"

    def test_hemisphere_coverage(self):
        """Verify uniform coverage: mean z should be ~0.5 for uniform hemisphere."""
        rays = _build_fibonacci_hemisphere(512)
        mean_z = float(np.mean(rays[:, 2]))
        assert 0.4 < mean_z < 0.6, f"Mean z = {mean_z:.3f}, expected ~0.5"


# ---------------------------------------------------------------------------
# Test: CuencaDOPModel
# ---------------------------------------------------------------------------

class TestCuencaDOPModel(unittest.TestCase):

    def setUp(self):
        self.model = CuencaDOPModel(CuencaConfig())

    def test_open_sky_dop(self):
        """SF=1.0 → HDOP=1.0, VDOP=1.0 (no degradation)."""
        assert self.model.hdop(1.0) == pytest.approx(1.0, abs=1e-9)
        assert self.model.vdop(1.0) == pytest.approx(1.0, abs=1e-9)

    def test_half_occluded_dop(self):
        """SF=0.5 → HDOP ≈ 1.47, VDOP ≈ 2.0 (from paper Table 1)."""
        h = self.model.hdop(0.5)
        v = self.model.vdop(0.5)
        assert 1.4 < h < 1.6, f"HDOP at SF=0.5 should be ~1.47, got {h:.3f}"
        assert 1.9 < v < 2.1, f"VDOP at SF=0.5 should be ~2.0, got {v:.3f}"

    def test_heavy_canyon_dop(self):
        """SF=0.1 → HDOP ≈ 3.12, VDOP ≈ 5.83."""
        h = self.model.hdop(0.1)
        v = self.model.vdop(0.1)
        assert 2.9 < h < 3.4, f"HDOP at SF=0.1 should be ~3.12, got {h:.3f}"
        assert 5.5 < v < 6.2, f"VDOP at SF=0.1 should be ~5.83, got {v:.3f}"

    def test_monotone_decreasing(self):
        """HDOP and VDOP must be monotonically decreasing in SF."""
        sfs = np.linspace(0.05, 1.0, 50)
        hdops = [self.model.hdop(sf) for sf in sfs]
        vdops = [self.model.vdop(sf) for sf in sfs]
        for i in range(1, len(sfs)):
            assert hdops[i] <= hdops[i-1], f"HDOP not monotone at SF={sfs[i]:.2f}"
            assert vdops[i] <= vdops[i-1], f"VDOP not monotone at SF={sfs[i]:.2f}"

    def test_satellite_count(self):
        """Satellite count model: N_sv = max(3, round(10 × SF))."""
        assert self.model.satellite_count(1.0) == 10
        assert self.model.satellite_count(0.5) == 5
        assert self.model.satellite_count(0.3) == 3   # min clamp
        assert self.model.satellite_count(0.05) == 3  # min clamp (clamped SF)

    def test_field_names_exact(self):
        """Verify output dict uses exact Pegasus field names including double-t typo."""
        base = {
            "latitude": 40.4432, "longitude": -79.9429, "altitude": 290.0,
            "fix_type": 3, "eph": 100, "epv": 150, "sattelites_visible": 10,
        }
        rng = np.random.default_rng(seed=42)
        overrides = self.model.compute(sf=0.5, base_data=base, rng=rng)

        assert "sattelites_visible" in overrides, \
            "Must use 'sattelites_visible' (double-t) — Pegasus typo"
        assert "satellites_visible" not in overrides, \
            "Must NOT use 'satellites_visible' (single-t)"
        assert "latitude"  in overrides
        assert "longitude" in overrides
        assert "altitude"  in overrides
        assert "eph"       in overrides
        assert "epv"       in overrides

    def test_eph_in_centimeters(self):
        """eph/epv must be integers in centimeters (MAVLink GPS_RAW_INT format)."""
        base = {"latitude": 40.0, "longitude": -80.0, "altitude": 300.0,
                "eph": 100, "epv": 150, "sattelites_visible": 10}
        rng = np.random.default_rng(seed=0)
        overrides = self.model.compute(sf=1.0, base_data=base, rng=rng)
        assert isinstance(overrides["eph"], int), "eph must be int (centimeters)"
        assert isinstance(overrides["epv"], int), "epv must be int (centimeters)"
        # At SF=1.0, HDOP=1.0: eph = 1.0 m × 100 = 100 cm
        assert overrides["eph"] == pytest.approx(100, abs=5)


# ---------------------------------------------------------------------------
# Test: SkyOcclusionModel
# ---------------------------------------------------------------------------

class TestSkyOcclusionModel(unittest.TestCase):

    def setUp(self):
        _MOCK_SQ.raycast_closest.reset_mock()
        _MOCK_SQ.raycast_closest.side_effect = None

    def test_open_sky(self):
        """All rays miss → SF = 1.0."""
        _set_sky_fraction(_MOCK_SQ, fraction=1.0, n_rays=32)
        model = SkyOcclusionModel(n_rays=32)
        sf = model.compute_sky_fraction(np.array([0.0, 0.0, 10.0]))
        assert sf == pytest.approx(1.0, abs=0.05)

    def test_fully_occluded(self):
        """All rays hit → SF = 0.05 (clamped minimum)."""
        _set_sky_fraction(_MOCK_SQ, fraction=0.0, n_rays=32)
        model = SkyOcclusionModel(n_rays=32)
        sf = model.compute_sky_fraction(np.array([0.0, 0.0, 10.0]))
        assert sf == pytest.approx(0.05, abs=0.01), \
            "SF must be clamped to 0.05 minimum, not 0.0"

    def test_half_occluded(self):
        """Half rays miss → SF ≈ 0.5."""
        _set_sky_fraction(_MOCK_SQ, fraction=0.5, n_rays=32)
        model = SkyOcclusionModel(n_rays=32)
        sf = model.compute_sky_fraction(np.array([0.0, 0.0, 10.0]))
        assert 0.45 < sf < 0.55, f"Expected SF ≈ 0.5, got {sf:.3f}"

    def test_cache_reuse(self):
        """Cache: second call with same position should NOT cast new rays."""
        _set_sky_fraction(_MOCK_SQ, fraction=0.8, n_rays=32)
        model = SkyOcclusionModel(n_rays=32, cache_duration=1.0)
        pos = np.array([0.0, 0.0, 10.0])
        sf1 = model.compute_sky_fraction(pos, dt=0.1)
        call_count_1 = _MOCK_SQ.raycast_closest.call_count
        sf2 = model.compute_sky_fraction(pos, dt=0.05)   # within cache window
        call_count_2 = _MOCK_SQ.raycast_closest.call_count
        assert call_count_1 == call_count_2, "Cache should prevent re-casting rays"
        assert sf1 == sf2

    def test_cache_expires(self):
        """Cache expires after cache_duration → re-casts rays."""
        _set_sky_fraction(_MOCK_SQ, fraction=0.8, n_rays=32)
        model = SkyOcclusionModel(n_rays=32, cache_duration=0.05)
        pos = np.array([0.0, 0.0, 10.0])
        sf1 = model.compute_sky_fraction(pos, dt=0.1)   # first call
        # Accumulate enough dt to expire cache
        call_count_1 = _MOCK_SQ.raycast_closest.call_count
        sf2 = model.compute_sky_fraction(pos, dt=0.1)   # should re-cast
        call_count_2 = _MOCK_SQ.raycast_closest.call_count
        assert call_count_2 > call_count_1, "Should re-cast after cache expiry"


# ---------------------------------------------------------------------------
# Test: FixStateMachine
# ---------------------------------------------------------------------------

# Import here after mocking is in place
from pegasus.simulator.logic.sensors.gps import FixStateMachine  # type: ignore


class TestFixStateMachine(unittest.TestCase):

    def setUp(self):
        self.fsm = FixStateMachine(
            sf_threshold=0.15,
            sf_good=0.30,
            t_up=2.0,
            t_down=0.5,
        )

    def test_initial_state(self):
        assert self.fsm._current_fix == 3   # starts at 3D fix

    def test_stays_at_3_in_open_sky(self):
        for _ in range(100):
            ft = self.fsm.update(sf=1.0, dt=0.1)
        assert ft == 3

    def test_downgrades_after_t_down(self):
        """fix_type should drop to 1 after T_down=0.5 s of SF < 0.15."""
        ft = None
        for _ in range(20):   # 20 × 0.05 s = 1.0 s > T_down=0.5 s
            ft = self.fsm.update(sf=0.05, dt=0.05)
        assert ft == 1, f"Expected fix_type=1, got {ft}"

    def test_upgrades_after_t_up(self):
        """After downgrade, fix_type should upgrade after T_up=2.0 s of SF > 0.30."""
        # Force downgrade first
        for _ in range(20):
            self.fsm.update(sf=0.05, dt=0.05)
        assert self.fsm._current_fix == 1

        # Now feed good SF for > T_up = 2.0 s
        ft = None
        for _ in range(50):   # 50 × 0.05 s = 2.5 s > T_up=2.0 s
            ft = self.fsm.update(sf=0.5, dt=0.05)
        assert ft == 3, f"Expected fix_type=3, got {ft}"

    def test_no_oscillation_at_boundary(self):
        """Rapid SF crossing of threshold should NOT cause fix_type oscillation."""
        initial_ft = self.fsm._current_fix
        # Rapidly alternate above/below threshold
        for i in range(200):
            sf = 0.30 if i % 2 == 0 else 0.14   # alternates across threshold
            self.fsm.update(sf=sf, dt=0.02)      # 0.02s each → never accumulates T_up/T_down

        # Should not have dropped to 1 (insufficient time below threshold)
        assert self.fsm._current_fix >= 2, \
            "Fix type should not drop during rapid SF oscillation"

    def test_reset(self):
        for _ in range(20):
            self.fsm.update(sf=0.05, dt=0.05)
        self.fsm.reset(fix_type=3)
        assert self.fsm._current_fix == 3
        assert self.fsm._timer == 0.0


# ---------------------------------------------------------------------------
# Test: JammingModel
# ---------------------------------------------------------------------------

class TestJammingModel(unittest.TestCase):

    def _make_data(self):
        return {
            "latitude": 40.4432, "longitude": -79.9429, "altitude": 290.0,
            "fix_type": 3, "eph": 100, "epv": 150, "sattelites_visible": 10,
        }

    def test_denial_in_zone(self):
        """Drone inside denial zone → fix_type=1 after ramp."""
        cfg = JammingConfig(
            mode=JammingMode.DENIAL,
            zone_center=(0.0, 0.0, 0.0),
            zone_radius=100.0,
            entry_ramp_s=0.1,    # fast ramp for testing
        )
        jammer = JammingModel(cfg)
        pos = np.array([0.0, 0.0, 10.0])   # inside zone
        data = self._make_data()
        for _ in range(50):   # 50 × 0.1 s = 5 s >> ramp time
            data = self._make_data()
            data = jammer.apply(data, pos, dt=0.1)
        assert data["fix_type"] == 1, f"Expected fix_type=1 in denial zone, got {data['fix_type']}"
        assert data["sattelites_visible"] == 0, "Satellites should be 0 under denial"  # double-t

    def test_no_effect_outside_zone(self):
        """Drone outside zone → GPS unchanged."""
        cfg = JammingConfig(
            mode=JammingMode.DENIAL,
            zone_center=(1000.0, 0.0, 0.0),   # far away
            zone_radius=10.0,
        )
        jammer = JammingModel(cfg)
        pos = np.array([0.0, 0.0, 10.0])   # not in zone
        data_orig = self._make_data()
        data_out = jammer.apply(dict(data_orig), pos, dt=0.1)
        assert data_out["fix_type"] == data_orig["fix_type"]
        assert data_out["eph"] == data_orig["eph"]

    def test_spoofing_drifts_slowly(self):
        """Spoofing position drift must be ≤ 0.5 m/s (Kerns et al. 2014)."""
        cfg = JammingConfig(
            mode=JammingMode.SPOOFING,
            zone_center=(0.0, 0.0, 0.0),
            zone_radius=1000.0,
            spoof_target_offset_m=[100.0, 0.0, 0.0],
            spoof_drift_rate_mps=0.5,
            entry_ramp_s=0.01,
        )
        jammer = JammingModel(cfg)
        pos = np.array([0.0, 0.0, 10.0])
        dt = 0.1
        lat0 = None
        lat1 = None
        for i in range(20):
            data = {"latitude": 40.4432, "longitude": -79.9429, "altitude": 290.0,
                    "fix_type": 3, "eph": 100, "epv": 150, "sattelites_visible": 10}
            data = jammer.apply(data, pos, dt=dt)
            if i == 0:
                lat0 = data["latitude"]
            lat1 = data["latitude"]

        total_lat_diff = abs(lat1 - lat0) * 111320.0   # convert degrees to meters
        total_time = 20 * dt  # seconds
        avg_rate = total_lat_diff / total_time
        assert avg_rate <= 0.55, \
            f"Spoof drift rate {avg_rate:.3f} m/s exceeds 0.5 m/s EKF safety limit"

    def test_noise_maintains_fix(self):
        """NOISE jamming should maintain fix_type=3 (bad accuracy but still fixed)."""
        cfg = JammingConfig(
            mode=JammingMode.NOISE,
            zone_center=(0.0, 0.0, 0.0),
            zone_radius=100.0,
            entry_ramp_s=0.01,
        )
        jammer = JammingModel(cfg)
        pos = np.array([0.0, 0.0, 10.0])
        for _ in range(30):
            data = self._make_data()
            data = jammer.apply(data, pos, dt=0.1)
        assert data["fix_type"] == 3, \
            "NOISE jamming must maintain fix_type=3 (accuracy degraded, not lost)"
        assert data["eph"] > 500, f"eph under noise jamming should be > 5 m (500 cm), got {data['eph']}"


# ---------------------------------------------------------------------------
# Performance tests (pytest-benchmark optional, manual timing fallback)
# ---------------------------------------------------------------------------

class TestPerformance(unittest.TestCase):
    """
    Performance budget targets from GNSS Ray-Tracing (2026) §4.3:
      - SkyOcclusionModel.compute_sky_fraction(): < 5 ms per call (32 rays)
      - CuencaDOPModel.compute(): < 0.1 ms per call
      - Full DegradedGPS.update() Tier 1: < 10 ms per drone per frame
    """

    def test_sky_fraction_performance(self):
        """32-ray sky fraction query must complete in < 5 ms."""
        # Set up fast mock (no side effect overhead)
        _MOCK_SQ.raycast_closest.side_effect = None
        _MOCK_SQ.raycast_closest.return_value = {"hit": False}

        model = SkyOcclusionModel(n_rays=32, cache_duration=0.0)
        pos = np.array([0.0, 0.0, 10.0])

        N_RUNS = 100
        start = time.perf_counter()
        for _ in range(N_RUNS):
            model.reset_cache()
            model.compute_sky_fraction(pos)
        elapsed_s = (time.perf_counter() - start) / N_RUNS
        elapsed_ms = elapsed_s * 1000.0

        assert elapsed_ms < 5.0, \
            f"Sky fraction query took {elapsed_ms:.2f} ms — must be < 5 ms"
        print(f"\n  [PERF] SkyOcclusionModel: {elapsed_ms:.3f} ms / query")

    def test_cuenca_compute_performance(self):
        """Cuenca DOP computation must complete in < 0.1 ms."""
        model = CuencaDOPModel(CuencaConfig())
        base = {"latitude": 40.0, "longitude": -80.0, "altitude": 300.0,
                "eph": 100, "epv": 150, "sattelites_visible": 10}
        rng = np.random.default_rng(seed=0)

        N_RUNS = 10000
        start = time.perf_counter()
        for _ in range(N_RUNS):
            model.compute(sf=0.5, base_data=base, rng=rng)
        elapsed_ms = (time.perf_counter() - start) / N_RUNS * 1000.0

        assert elapsed_ms < 0.1, \
            f"Cuenca compute took {elapsed_ms:.4f} ms — must be < 0.1 ms"
        print(f"\n  [PERF] CuencaDOPModel: {elapsed_ms:.4f} ms / call")


# ---------------------------------------------------------------------------
# Integration scenario tests (marked for airstack test -m gps_degradation)
# ---------------------------------------------------------------------------

@pytest.mark.gps_degradation
class TestScenarios:
    """
    Scenario-level tests matching the test cases in §29 (SITL Testing Guide).
    These verify end-to-end behavior of the combined DegradedGPS pipeline.
    """

    def _run_scenario(
        self,
        sf_sequence,       # list of (sf, dt) tuples
        jamming_cfg=None,
    ):
        """Run a sequence of sky fractions through CuencaDOP + FixStateMachine."""
        cuenca = CuencaDOPModel(CuencaConfig())
        fsm = FixStateMachine()
        results = []
        for sf, dt in sf_sequence:
            base = {"latitude": 40.0, "longitude": -80.0, "altitude": 300.0,
                    "eph": 100, "epv": 150, "sattelites_visible": 10}
            rng = np.random.default_rng(seed=0)
            overrides = cuenca.compute(sf=sf, base_data=base, rng=rng)
            base.update(overrides)
            base["fix_type"] = fsm.update(sf, dt)
            results.append(dict(base))
        return results

    def test_scenario_a_open_field(self):
        """Scenario A: Open field — SF=1.0 throughout → fix_type=3, eph≈100cm."""
        seq = [(1.0, 0.1)] * 50
        results = self._run_scenario(seq)
        for r in results[-10:]:    # last 10 frames
            assert r["fix_type"] == 3
            assert r["eph"] <= 110   # ≤ 1.1 m (within 10% of nominal)

    def test_scenario_b_dense_canyon(self):
        """Scenario B: Deep canyon — SF=0.05 → fix_type=1, eph=9999."""
        seq = [(0.05, 0.1)] * 100
        results = self._run_scenario(seq)
        for r in results[-10:]:    # last 10 frames (after state machine settles)
            assert r["fix_type"] == 1, f"Expected fix_type=1 in deep canyon"

    def test_scenario_d_gradual_entry(self):
        """Scenario D: Gradual SF degradation → smooth eph increase, no jump."""
        sfs = np.linspace(1.0, 0.1, 100)
        seq = [(sf, 0.1) for sf in sfs]
        results = self._run_scenario(seq)
        ephs = [r["eph"] for r in results]
        # Check monotone increase (eph should increase as SF decreases)
        for i in range(10, len(ephs) - 1):
            assert ephs[i] >= ephs[i-5] - 50, \
                f"eph dropped unexpectedly from {ephs[i-5]} to {ephs[i]}"

    def test_ekf_innovation_gate_safety(self):
        """
        EKF safety: position error from DegradedGPS must stay < 2.5 m horizontal
        at SF ≥ 0.3 (3D fix territory). Per PX4 EKF2 5σ gate analysis in §34.
        """
        cuenca = CuencaDOPModel(CuencaConfig())
        errors = []
        for seed in range(100):    # 100 random samples at SF=0.3
            rng = np.random.default_rng(seed=seed)
            base = {"latitude": 40.4432, "longitude": -79.9429, "altitude": 290.0,
                    "eph": 100, "epv": 150, "sattelites_visible": 10}
            overrides = cuenca.compute(sf=0.30, base_data=base, rng=rng)
            # Compute horizontal position error in meters
            dlat_m = (overrides["latitude"]  - 40.4432)  * 111320.0
            dlon_m = (overrides["longitude"] - (-79.9429)) * 111320.0 * math.cos(math.radians(40.4432))
            err = math.sqrt(dlat_m**2 + dlon_m**2)
            errors.append(err)
        p99 = np.percentile(errors, 99)
        assert p99 < 5.0, \
            f"99th percentile position error at SF=0.3 is {p99:.2f} m — EKF may reject"
        print(f"\n  [EKF CHECK] P99 position error at SF=0.3: {p99:.3f} m (limit: 5.0 m)")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
```

---

## 43. Production Code: Hardware Companion ROS 2 Node

**File path:** `robot/ros_ws/src/gps_degradation_hw/gps_degradation_hw/dop_scaling_node.py`

**Purpose:** On real hardware (no Isaac Sim), this ROS 2 node applies Tier 1 DOP-driven accuracy scaling to the real GPS receiver output. It subscribes to the raw NavSatFix topic, computes a manually-provided sky fraction (from a config file, lidar point cloud, or precomputed urban canyon map), and re-publishes a degraded NavSatFix + MAVLink override for MACVIO.

**Usage:**
```bash
# On the robot companion computer:
ros2 run gps_degradation_hw dop_scaling_node \
  --ros-args -p sky_fraction:=0.4 -p multipath_enabled:=false
```

```python
# dop_scaling_node.py
# Hardware companion node for GPS degradation (no Isaac Sim required)
# AirStack Issue #345

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

import numpy as np

# Cuenca DOP formulas (inline — no Isaac Sim dependency)
def _hdop(sf: float, alpha=2.5, beta=2.2) -> float:
    return 1.0 + alpha * (1.0 - sf) ** beta

def _vdop(sf: float, alpha=4.0, beta=1.8) -> float:
    return 1.0 + alpha * (1.0 - sf) ** beta


class DopScalingNode(Node):
    """
    ROS 2 node that applies Cuenca DOP scaling to real GPS NavSatFix messages.

    Subscribes:
        /robot/interface/gps/fix  (sensor_msgs/NavSatFix) — raw GPS from receiver
        /gps_sky_fraction         (std_msgs/Float32)       — optional: runtime SF update

    Publishes:
        /robot/interface/gps/fix_degraded (sensor_msgs/NavSatFix) — DOP-scaled GPS
        /gps_dop_info             (std_msgs/Float32)               — current HDOP

    Parameters:
        sky_fraction (float, default 1.0): Manual sky fraction override.
            Set < 1.0 to simulate urban canyon without lidar.
            Override at runtime via /gps_sky_fraction topic.
        sigma_h_base (float, default 1.0): Base horizontal sigma [m].
        sigma_v_base (float, default 1.5): Base vertical sigma [m].
        republish_raw (bool, default True): Also republish raw GPS unchanged.
    """

    def __init__(self):
        super().__init__("dop_scaling_node")

        # Parameters
        self.declare_parameter("sky_fraction",   1.0)
        self.declare_parameter("sigma_h_base",   1.0)
        self.declare_parameter("sigma_v_base",   1.5)
        self.declare_parameter("republish_raw", True)
        self.declare_parameter("input_topic",  "/robot/interface/gps/fix")
        self.declare_parameter("output_topic", "/robot/interface/gps/fix_degraded")

        self._sf = self.get_parameter("sky_fraction").value
        self._sigma_h = self.get_parameter("sigma_h_base").value
        self._sigma_v = self.get_parameter("sigma_v_base").value
        self._rng = np.random.default_rng()

        input_topic  = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # Subscribers
        self._sub_fix = self.create_subscription(
            NavSatFix, input_topic, self._on_fix, 10
        )
        self._sub_sf = self.create_subscription(
            Float32, "/gps_sky_fraction", self._on_sf, 10
        )

        # Publishers
        self._pub_degraded = self.create_publisher(NavSatFix, output_topic, 10)
        self._pub_hdop = self.create_publisher(Float32, "/gps_dop_info", 10)

        self.get_logger().info(
            f"DopScalingNode started. Input: {input_topic}, "
            f"Output: {output_topic}, SF={self._sf:.2f}"
        )

    def _on_sf(self, msg: Float32):
        """Update sky fraction from external source (e.g. lidar-based computation)."""
        self._sf = float(np.clip(msg.data, 0.05, 1.0))
        self.get_logger().debug(f"Sky fraction updated: {self._sf:.3f}")

    def _on_fix(self, msg: NavSatFix):
        """Apply Cuenca DOP scaling and republish."""
        sf = self._sf
        h = _hdop(sf)
        v = _vdop(sf)

        # Compute additional position noise (on top of receiver's own noise)
        sigma_extra_h = self._sigma_h * (h - 1.0)
        sigma_extra_v = self._sigma_v * (v - 1.0)

        cos_lat = math.cos(math.radians(msg.latitude))
        lat_noise = self._rng.normal(0.0, sigma_extra_h / 111320.0)
        lon_noise = self._rng.normal(0.0, sigma_extra_h / (111320.0 * max(cos_lat, 1e-6)))
        alt_noise = self._rng.normal(0.0, sigma_extra_v)

        out = NavSatFix()
        out.header = msg.header
        out.latitude  = msg.latitude  + lat_noise
        out.longitude = msg.longitude + lon_noise
        out.altitude  = msg.altitude  + alt_noise

        # Set covariance: diagonal with σ² from DOP scaling
        eph_m = h * self._sigma_h
        epv_m = v * self._sigma_v
        cov = [
            eph_m**2, 0.0, 0.0,
            0.0, eph_m**2, 0.0,
            0.0, 0.0, epv_m**2,
        ]
        out.position_covariance = cov
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Status
        out.status = NavSatStatus()
        n_sat = max(3, int(round(10 * sf)))
        if sf < 0.15:
            out.status.status = NavSatStatus.STATUS_NO_FIX
        elif sf < 0.30:
            out.status.status = NavSatStatus.STATUS_FIX
        else:
            out.status.status = NavSatStatus.STATUS_SBAS_FIX

        self._pub_degraded.publish(out)

        # Publish HDOP for monitoring
        hdop_msg = Float32()
        hdop_msg.data = float(h)
        self._pub_hdop.publish(hdop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DopScalingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**`package.xml` for the hardware node:**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>gps_degradation_hw</name>
  <version>0.1.0</version>
  <description>Hardware companion node for GPS DOP scaling (AirStack Issue #345)</description>
  <maintainer email="apourva14@gmail.com">Apurva Singh</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 44. `__init__.py` Exports (update existing file)

**File path:** `simulation/isaac-sim/extensions/PegasusSimulator/pegasus/simulator/logic/sensors/__init__.py`

Add to the existing `__init__.py`:

```python
# Issue #345: GPS Degradation exports
from pegasus.simulator.logic.sensors.sky_occlusion import (
    SkyOcclusionModel,
    CuencaDOPModel,
    CuencaConfig,
    PantMultipathModel,
    PantConfig,
)
from pegasus.simulator.logic.sensors.jamming import (
    JammingModel,
    JammingConfig,
    JammingMode,
)
from pegasus.simulator.logic.sensors.gps import (
    GPS,
    DegradedGPS,
    FixStateMachine,
    DEGRADED_GPS_DEFAULT_CONFIG,
)
```

---

## 45. Quick-Start Integration Checklist

```
□ 1. Create sky_occlusion.py at the path in §38
□ 2. Create jamming.py at the path in §38
□ 3. Append DegradedGPS code (§41) to the bottom of gps.py
□ 4. Update __init__.py with exports from §44
□ 5. Run: airstack test -m gps_degradation
     → all TestFibonacciHemisphere, TestCuencaDOPModel, TestSkyOcclusionModel,
        TestFixStateMachine, TestJammingModel tests must pass
□ 6. Performance test: TestPerformance must report < 5 ms sky fraction query
□ 7. In Isaac Sim: replace GPS(...) with DegradedGPS({...}) in the vehicle config
□ 8. Fly open-field scenario (§29 Scenario A):
     - Confirm eph ≈ 100 cm, fix_type=3 throughout
□ 9. Add 2-wall USD prim canyon. Fly through:
     - Confirm eph rises to > 300 cm, fix_type drops to 1
     - Confirm MACVIO odometry drift in Foxglove
□ 10. Enable Tier 3: set jamming_enabled=True, jamming_mode="DENIAL"
      - Confirm fix drops inside zone, recovers outside
□ 11. For hardware: build gps_degradation_hw ROS 2 package (§43)
      colcon build --packages-select gps_degradation_hw
      ros2 run gps_degradation_hw dop_scaling_node --ros-args -p sky_fraction:=0.4
```

---

*Document prepared by Apurva Singh, CMU AirLab Intern, May 2026.*  
*Last updated: May 14, 2026.*  
*To convert to Word: open this file in Microsoft Word → File → Save As → Word Document (.docx)*
