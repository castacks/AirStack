# AGENTS.md

This file provides guidance to AI coding agents (OpenHands, Claude Code, etc.) when working with the AirStack repository.

## Quick Start for AI Agents

**Project:** AirStack - Comprehensive autonomous aerial robotics stack developed by AirLab CMU

**Stack:** ROS 2 Jazzy | Docker-based development | Isaac Sim with Pegasus extension | Microsoft AirSim (legacy, UE4) | Field robotics

**Primary Goal:** Enable agents to understand the architecture, implement new algorithms/modules, and integrate them correctly into the layered autonomy stack.

## Repository Purpose

AirStack provides a complete end-to-end system for autonomous drone operations including:
- Modular autonomy stack (interface, sensors, perception, local planning, global planning, behavior)
- High-fidelity simulation environments (Isaac Sim with Pegasus extension, Microsoft AirSim legacy/UE4)
- Ground Control Station for mission planning and monitoring
- Multi-robot coordination capabilities
- Hardware deployment tools

The architecture is designed to allow easy swapping of algorithm modules (e.g., different planners, controllers, perception systems) through a standardized ROS 2 interface pattern.

## Repository Architecture

### High-Level Structure
```
AirStack/
├── robot/                    # Onboard autonomy stack (ROS 2 Jazzy)
│   └── ros_ws/src/          # Layered autonomy modules
│       ├── interface/       # Hardware interface & safety
│       ├── sensors/         # Sensor integration
│       ├── perception/      # State estimation & perception
│       ├── local/           # Local planning, world models, control
│       ├── global/          # Global planning & mapping
│       └── behavior/        # High-level mission execution
├── simulation/isaac-sim/    # Isaac Sim (Pegasus extension)
├── simulation/ms-airsim/       # AirSim (UE4 + PX4 SITL)
├── gcs/                     # Ground Control Station
├── common/                  # Shared packages & utilities
├── docs/                    # MkDocs documentation
├── mkdocs.yml               # MkDocs config file
├── tests/                   # System tests (pytest) + metrics reporting
├── .github/
│   ├── workflows/           # GitHub Actions CI (system-tests, docker-build, etc.)
│   └── orchestrator/        # OpenStack-backed ephemeral self-hosted runners
└── .agents/skills/          # Detailed workflow guides for agents
```

### Layered Autonomy Pattern

The autonomy stack follows a **layered architecture** where data flows through processing stages:

```
Sensors → Perception → World Models → Planners → Controllers → Interface → Hardware
```

Each layer has:
- **Module packages**: Individual algorithm implementations (e.g., `droan_local_planner`)
- **Bringup package**: Orchestrates layer launch with topic remapping (e.g., `local_bringup`)

**Key Insight:** Understanding "what connects to what" is critical. See [Integration Checklist](docs/robot/autonomy/integration_checklist.md) and [System Architecture](docs/robot/autonomy/system_architecture.md).

## Standard Topic Patterns

Modules communicate via ROS 2 topics. Common standard topics:

| Topic Pattern | Type | Purpose |
|--------------|------|---------|
| `/{robot_name}/odometry` | nav_msgs/Odometry | Robot state estimation |
| `/{robot_name}/global_plan` | nav_msgs/Path | Global waypoint path |
| `/{robot_name}/trajectory_controller/trajectory_override` | airstack_msgs/TrajectoryOverride | Direct trajectory commands |
| `/{robot_name}/trajectory_controller/trajectory_segment_to_add` | airstack_msgs/TrajectorySegment | Planned trajectory segment |
| `/{robot_name}/trajectory_controller/look_ahead` | geometry_msgs/PointStamped | Look-ahead point for planning |

**Note:** Topics are remapped in bringup launch files to connect modules. Input/output topics should be configurable via launch arguments.

See [Integration Checklist](docs/robot/autonomy/integration_checklist.md) for comprehensive topic conventions.

## Common Workflows (Skills)

For detailed step-by-step instructions, refer to the **`.agents/skills/`** directory:

| Skill | When to Use |
|-------|------------|
| [add-ros2-package](.agents/skills/add-ros2-package) | Creating a new algorithm module package |
| [add-task-executor](.agents/skills/add-task-executor) | Implementing a task executor as a ROS 2 action server |
| [integrate-module-into-layer](.agents/skills/integrate-module-into-layer) | Adding module to layer bringup |
| [write-launch-file](.agents/skills/write-launch-file) | Authoring ROS 2 launch files with AirStack conventions (ROBOT_NAME namespacing, topic remapping, allow_substs) |
| [write-isaac-sim-scene](.agents/skills/write-isaac-sim-scene) | Creating custom simulation scenes |
| [visualize-in-foxglove](.agents/skills/visualize-in-foxglove) | Adding topic visualization to Foxglove/GCS |
| [attach-gossip-payload](.agents/skills/attach-gossip-payload) | Broadcasting custom ROS messages to peers via PeerProfile gossip payloads |
| [debug-module](.agents/skills/debug-module) | Autonomous debugging of ROS 2 modules |
| [update-documentation](.agents/skills/update-documentation) | Documenting new modules and updating mkdocs |
| [test-in-simulation](.agents/skills/test-in-simulation) | End-to-end simulation testing of a module |
| [run-system-tests](.agents/skills/run-system-tests) | Running the pytest system test harness (marks, MetricsRecorder, /pytest PR trigger) |
| [add-behavior-tree-node](.agents/skills/add-behavior-tree-node) | Creating behavior tree nodes |
| [use-airstack-cli](.agents/skills/use-airstack-cli) | Using the `airstack` CLI and the non-interactive `docker exec` pattern |
| [configure-multi-robot](.agents/skills/configure-multi-robot) | Setting up multiple robots, ROBOT_NAME namespacing, and ROS_DOMAIN_ID isolation |
| [bump-version-and-release](.agents/skills/bump-version-and-release) | Bumping `.env` VERSION and CHANGELOG before merge to clear the version-check gate |
| [capture-discovered-knowledge](.agents/skills/capture-discovered-knowledge) | After long context-discovery / surprising findings, persist to AGENTS.md or a new skill so the next agent doesn't redo the work |

**Agent Workflow Example:**
1. Study reference implementation for module type
2. Follow `add_ros2_package.md` to create package structure
3. Implement algorithm with proper topic interfaces
4. Follow `integrate_module_into_layer.md` to add to bringup
5. Follow `update_documentation.md` to document
6. Follow `debug_module.md` and `test_in_simulation.md` to verify

Also see: [AI Agent Quick Guide](docs/development/ai_agent_guide.md)

## Reference Implementations

Study these well-structured modules as examples for different types:

| Module Type | Reference Package | Location |
|------------|------------------|----------|
| **Local Planner** | DROAN Local Planner | `robot/ros_ws/src/local/planners/droan_local_planner` |
| **Local World Model** | Disparity Expansion | `robot/ros_ws/src/local/world_models/disparity_expansion` |
| **Controller** | Trajectory Controller | `robot/ros_ws/src/local/c_controls/trajectory_controller` |
| **Global Planner** | Random Walk | `robot/ros_ws/src/global/planners/random_walk` |
| **Global World Model** | VDB Mapping | `robot/ros_ws/src/global/world_models/vdb_mapping_ros2` |
| **Behavior** | Behavior Tree | `robot/ros_ws/src/behavior/behavior_tree` |

Each reference shows:
- Package structure (CMakeLists.txt, package.xml, config, launch)
- ROS 2 node implementation patterns
- Topic subscription/publishing
- Parameter configuration
- README documentation

## Development Commands

### AirStack CLI Tool
The repository uses a custom CLI tool for common operations:

```bash
# Setup and installation
airstack setup          # Configure AirStack and add to PATH
airstack install        # Install Docker and dependencies

# Container management
airstack up [service]    # Start services (robot, isaac-sim, gcs)
airstack stop [service]  # Stop services
airstack status          # Show container status
airstack connect [name]  # Connect to running container
airstack logs [name]     # View container logs

# Development tasks
airstack build          # Build ROS workspace
airstack test           # Run tests
airstack docs           # Build and serve documentation
```

### Docker Development Workflow

**All development happens inside Docker containers.** To run commands in the robot container:

```bash
# Start robot container without autolaunch (for development)
AUTOLAUNCH=false airstack up robot-desktop

# Build ROS 2 workspace (inside container)
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <package_name>"

# Build with debug symbols
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <package_name> --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"

# Source workspace (inside container)
docker exec airstack-robot-desktop-1 bash -c "sws"

# Run a launch file
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch <package> <launch_file>"

# List ROS 2 nodes
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"

# Echo a topic
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo <topic_name> --once"
```

**Important:** Do NOT run commands in interactive mode as you can get stuck on prompts. Always use `docker exec <container> bash -c "<command>"`.

### ROS 2 Aliases (inside containers)
- `bws`: Build workspace (`colcon build` with common flags)
- `sws`: Source workspace (`source install/setup.bash`)

## Testing Philosophy

**Goal:** Enable autonomous debugging and testing by agents.

### Testing Levels
1. **Module Level:** Integration tests with mock inputs
   - Verify module behavior in isolation
   - Test with synthetic data
   - Located in module's `test/` directory

2. **System Level:** Full simulation tests (Isaac Sim or Microsoft AirSim legacy)
   - End-to-end autonomy stack testing
   - Real sensor simulation
   - Multi-robot scenarios
   - Implemented in [`tests/`](tests/) — see below

### System Test Suite (`tests/`)

Pytest-based system tests live at the repo root in [`tests/`](tests/). They bring up the full Docker stack (sim + robot + GCS) and verify container health, ROS 2 node presence, sensor publishing rates, compute usage, and end-to-end flight behavior.

| File | Mark | What it tests | Hardware |
|------|------|---------------|----------|
| [`tests/test_build_docker.py`](tests/test_build_docker.py) | `build_docker` | Docker image builds (robot-desktop, gcs, isaac-sim, ms-airsim) | Docker |
| [`tests/test_build_packages.py`](tests/test_build_packages.py) | `build_packages` | `colcon build` inside each container | Docker |
| [`tests/test_liveliness.py`](tests/test_liveliness.py) | `liveliness` | Full-stack health: containers, tmux, ROS 2 nodes, topic Hz, compute, sustained stability | Docker, GPU, sim license |
| [`tests/test_takeoff_hover_land.py`](tests/test_takeoff_hover_land.py) | `takeoff_hover_land` | 4-phase flight chain (PX4 ready → takeoff → hover → land) per (sim, num_robots, iter, velocity) | Docker, GPU, sim license |

Shared fixtures, the `airstack_env` parametrized fixture, and `MetricsRecorder` live in [`tests/conftest.py`](tests/conftest.py). Each run produces a timestamped directory under `tests/results/<timestamp>/` with `results.xml`, `metrics.json`, and per-test logs. [`tests/parse_metrics.py`](tests/parse_metrics.py) generates a markdown report (single-run or diff-vs-baseline; exits 1 on regression).

**Run via the CLI** (containerized runner — no local Python needed):

```bash
airstack test -m "build_docker or build_packages" -v
airstack test -m liveliness --sim msairsim --num-robots 1 --stress-iterations 1 -v
airstack test -m takeoff_hover_land --sim msairsim --takeoff-velocities 0.5,1,2 -v
```

Full reference: [`tests/README.md`](tests/README.md).

### Autonomous Debugging Approach
When a module doesn't work:
1. Verify module is running (`ros2 node list`)
2. Check topic connections (`ros2 topic info`, `ros2 topic hz`)
3. Inspect data quality (`ros2 topic echo`)
4. Review logs (`docker logs airstack-robot-desktop-1`)
5. Compare with reference implementation
6. Add instrumentation (debug publishers, logging)
7. Create minimal reproduction test

See detailed debugging workflow: [.agents/skills/debug_module](.agents/skills/debug_module)

## CI/CD

GitHub Actions workflows live in [`.github/workflows/`](.github/workflows/):

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| [`system-tests.yml`](.github/workflows/system-tests.yml) | PR opened, `/pytest` PR comment (write-access only), or `workflow_dispatch` | Runs the `tests/` suite on an ephemeral GPU runner; posts metrics report (with regression diff vs base branch / `main`) as a PR comment and to the job summary |
| [`docker-build.yml`](.github/workflows/docker-build.yml) | Push to `main`/`develop` that changes `.env` (`VERSION=`), or manual dispatch | Builds, pushes, and cosign-signs all compose images on the ephemeral runner |
| [`check-version-increment.yml`](.github/workflows/check-version-increment.yml) | Pull request | Validates `.env` `VERSION=` is valid semver and strictly greater than the base branch |
| `deploy_docs_from_{main,develop,release}.yaml` | Push to the matching branch (`docs/**`, `mkdocs.yml`, `*.md`) | Publishes versioned MkDocs site via `mike` |

**`/pytest` PR comments** trigger `system-tests.yml` for users with write access (OWNER/MEMBER/COLLABORATOR), pulling args from the first line of the comment (e.g. `/pytest -m liveliness --sim msairsim`). Fork PRs are blocked — same-repo only — to keep arbitrary code off the self-hosted runner.

### Ephemeral Runner Orchestrator

GPU-required jobs (`runs-on: [self-hosted, airstack-ephemeral]`) execute on **OpenStack VMs spawned per-job and destroyed on completion**. The orchestrator service code lives in [`.github/orchestrator/`](.github/orchestrator/):

- [`orchestrator.py`](.github/orchestrator/orchestrator.py) — Python service: spawn loop polls GitHub for queued jobs matching configured runner labels, mints single-use JIT runner tokens, creates an OpenStack server with cloud-init bootstrap; reap loop deletes the server when the job completes (or after `max_job_minutes`)
- [`cloud-init.yaml.j2`](.github/orchestrator/cloud-init.yaml.j2) — bootstraps Docker + nvidia-container-toolkit + GH Actions runner on the worker, registers with the JIT token, runs one job, then `shutdown -h`
- [`config.example.yaml`](.github/orchestrator/config.example.yaml) — flavor / network / keypair / floating-IP pool / runner labels / repo
- [`airstack-orchestrator.service`](.github/orchestrator/airstack-orchestrator.service) + [`setup.sh`](.github/orchestrator/setup.sh) — systemd unit and one-time installer

**Why ephemeral:** clean Docker cache per run, no leaked containers, GitHub PAT and OpenStack credentials only on the orchestrator host (workers receive a single-use JIT token bound to one runner registration). State map at `/var/lib/airstack-orchestrator/state.json`; logs via `journalctl -u airstack-orchestrator.service -f`.

**Setup, debugging a failed job, and SSH-into-worker procedures:** [`.github/orchestrator/README.md`](.github/orchestrator/README.md) (also exposed as [`tests/ci-cd-orchestrator.md`](tests/ci-cd-orchestrator.md) symlink for the docs site).

## Documentation Requirements

When implementing a new feature/module, you must:

### 1. Module README.md
Create `README.md` in the package directory with:
- Overview and purpose
- Algorithm description
- Architecture diagram (mermaid)
- Dependencies and interfaces (input/output topics, parameters)
- Configuration
- Usage examples

**Template:** See `.agents/skills/add-ros2-package/assets/package_template/README.md`

### 2. Update mkdocs.yml
Add the module README to the navigation structure:
```yaml
nav:
  - Robot:
      - Autonomy Modules:
          - Local:
              - Planning:
                  - Your Module:
                      - robot/ros_ws/src/local/planners/your_package/README.md
```

The `same-dir` plugin allows linking to README files outside the `docs/` directory.

### 3. System-Level Documentation (if needed)
For major features or cross-cutting concerns, create docs in `docs/`:
- Tutorials: `docs/tutorials/<feature>.md`
- Integration guides: `docs/robot/autonomy/<layer>/<topic>.md`

### 4. Update Layer Overview
Edit `docs/robot/autonomy/<layer>/index.md` to mention the new module.

**Complete workflow:** [.agents/skills/update_documentation](.agents/skills/update_documentation)

## Package Templates

Use standardized templates when creating new packages:

**Location:** `.agents/skills/add-ros2-package/assets/package_template/`

Templates include:
- `CMakeLists.txt` (C++ template with TODOs)
- `setup.py` (Python template with TODOs)
- `package.xml` (dependency template)
- `config/template.yaml` (parameter configuration)
- `launch/template.launch.xml` (launch file with remapping)
- `README.md` (comprehensive documentation template)
- Example source files with best practices

Follow the template structure for consistency across the codebase.

## Docker Architecture

Each major component has its own Docker container:
- **robot**: ROS 2 autonomy stack (Jazzy)
- **isaac-sim**: NVIDIA Isaac Sim with Pegasus extension (profile: `desktop` or `robot`)
- **airsim**: AirSim UE4 binary + PX4 SITL + ROS 2 bridge (profile: `ms-airsim`)
- **gcs**: Ground Control Station
- **docs**: Documentation building (MkDocs)

**Configuration:**
- Main compose file: `docker-compose.yaml` (includes all component compose files)
- Environment variables: top-level [`.env`](.env) (image tags, `VERSION`, `NUM_ROBOTS`, `ROBOT_NAME_MAP_CONFIG_FILE`, `ISAAC_SIM_SCRIPT_NAME`, `AUTONOMY_ROLE`, etc.)
- Per-container shell init: [`robot/docker/.bashrc`](robot/docker/.bashrc) — resolves `ROBOT_NAME` and `ROS_DOMAIN_ID` at startup (see Multi-Robot Configuration below)

**Networking:** Custom bridge network (172.31.0.0/24) for inter-container communication.

## Multi-Robot Configuration

Multi-robot is implemented via Docker Compose **replicas**, not multiple namespaces in one container. Setting `NUM_ROBOTS=3` in [`.env`](.env) spawns three separate containers (`airstack-robot-desktop-1`, `-2`, `-3`) via `deploy.replicas: ${NUM_ROBOTS:-1}` in [`robot/docker/docker-compose.yaml`](robot/docker/docker-compose.yaml).

`ROBOT_NAME` is **not** set directly in `.env`. Each container computes it at startup: [`robot/docker/.bashrc`](robot/docker/.bashrc) reads `ROBOT_NAME_SOURCE` (`container_name` or `hostname`) and runs [`resolve_robot_name.py`](robot/docker/robot_name_map/resolve_robot_name.py) against the mapping in [`robot/docker/robot_name_map/`](robot/docker/robot_name_map/) (default: [`default_robot_name_map.yaml`](robot/docker/robot_name_map/default_robot_name_map.yaml)). The resolver exports both `ROBOT_NAME` and `ROS_DOMAIN_ID` — robot N gets domain N by default, so each robot is on its own DDS partition.

The autonomy bringup variant is selected by `AUTONOMY_ROLE` (`full` | `onboard` | `offboard`), dispatched in [`robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`](robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml):

- **full** — every autonomy module runs on this machine (sim/dev desktop, autonomous Jetson)
- **onboard** — lite modules only (interface, sensors, perception, local planning, behavior); pairs with **offboard**
- **offboard** — global planning only; runs on GCS paired with onboard robots

For Isaac Sim, the default `ISAAC_SIM_SCRIPT_NAME=example_one_px4_pegasus_launch_script.py` only spawns a single drone. Multi-robot Isaac Sim requires `ISAAC_SIM_SCRIPT_NAME=example_multi_px4_pegasus_launch_script.py` (the system test harness sets this automatically when `--num-robots > 1`).

**Full workflow:** [.agents/skills/configure-multi-robot](.agents/skills/configure-multi-robot)

## Critical Pitfalls to Avoid

Common mistakes when adding modules:

1. **Topic Connection Issues**
   - ❌ Hardcoding topic names in node code
   - ✅ Use launch arguments for topic remapping
   - ✅ Verify connections with `ros2 topic info`

2. **Integration Failures**
   - ❌ Forgetting to add module to layer bringup launch file
   - ✅ Follow `integrate_module_into_layer.md` workflow
   - ✅ Add package dependency to bringup `package.xml`

3. **Build Issues**
   - ❌ Missing dependencies in `package.xml`
   - ✅ Declare all ROS 2 and external dependencies
   - ❌ Not installing launch/config files in `CMakeLists.txt`
   - ✅ Use `install()` directives for all resources

4. **Documentation Gaps**
   - ❌ Not updating `mkdocs.yml` navigation
   - ✅ Add module to appropriate nav section
   - ❌ Missing module README
   - ✅ Use README template with all sections

5. **Launch File Issues**
   - ❌ Not using `$(env ROBOT_NAME)` for multi-robot support
   - ✅ Always namespace with robot name
   - ❌ Missing `allow_substs="true"` for parameter files
   - ✅ Enable substitution for environment variables in configs

6. **Testing Oversights**
   - ❌ Only testing module in isolation
   - ✅ Test in full autonomy stack context
   - ✅ Verify in Isaac Sim or Microsoft AirSim (legacy) simulation

## Key Differences from CLAUDE.md

This guide supersedes `CLAUDE.md` (which now symlinks here). Key updates:

- **ROS 2 Jazzy** (was Humble)
- **`airstack` command** (not `./airstack.sh` in most contexts)
- **Skills directory** for detailed workflows
- **Module integration focus** with checklist and templates
- **Autonomous debugging guidance** for AI agents
- **Package templates** for consistency
- **Documentation automation** requirements

## Additional Resources

### Comprehensive Guides
- [AI Agent Quick Guide](docs/development/ai_agent_guide.md) - Quick reference for agents
- [Integration Checklist](docs/robot/autonomy/integration_checklist.md) - Module integration requirements
- [System Architecture](docs/robot/autonomy/system_architecture.md) - Architecture diagrams and data flow
- [Contributing Guide](docs/development/contributing.md) - Development workflow and PR process

### ROS 2 Documentation
- [ROS 2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Creating a ROS 2 Package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

### AirStack Documentation
- [Getting Started](docs/getting_started/index.md)
- [Autonomy Stack Overview](docs/robot/autonomy/index.md)
- [Isaac Sim Setup](docs/simulation/isaac_sim/index.md)
- [Microsoft AirSim (legacy) Setup](docs/simulation/ms-airsim/index.md)
- [Testing Guide](docs/development/testing/index.md)

### External Tools
- [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) - Documentation framework
- [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) - Isaac Sim extension

---

**For Agents:** Start with the [AI Agent Quick Guide](docs/development/ai_agent_guide.md) and refer to `.agents/skills/` for specific workflows. Study reference implementations before creating new modules.
