# AirStack Repository Overview

## Purpose

AirStack is a comprehensive, modular autonomy stack for embodied AI and robotics developed by the [AirLab](https://theairlab.org) at Carnegie Mellon University's Robotics Institute. It provides a complete framework for developing, testing, and deploying autonomous mobile systems in both simulated and real-world environments.

**Key Features:**
- Modular architecture allowing easy component swapping and customization
- ROS 2-based inter-process communication
- Integrated NVIDIA Isaac Sim support for high-fidelity simulation
- Multi-robot capability for coordinated operations
- Comprehensive autonomy stack covering interface, sensors, perception, planning, control, and behavior management
- Ground Control Station for monitoring and control

**Current Status:** ALPHA - Internal usage only, requires AirLab account access

## General Setup

AirStack uses a Docker-based development environment with the following setup approach:

### Prerequisites
- Docker with NVIDIA Container Toolkit support
- NVIDIA GPU (RTX 3070 or better recommended for Isaac Sim)
- At least 25GB free storage space
- Ubuntu 22.04 (recommended)

### Installation Process
1. Clone repository with submodules: `git clone --recursive -j8 git@github.com:castacks/AirStack.git`
2. Install dependencies: `./airstack.sh install` (installs Docker and docker-compose)
3. Setup environment: `./airstack.sh setup` (configures CLI tool and keys)
4. Pull Docker images from AirLab registry or build from scratch
5. Launch system: `airstack up` (starts robot, ground control station, and Isaac Sim)

### Docker Images
- **Option 1 (Preferred):** Pull from AirLab Docker registry (`airlab-storage.andrew.cmu.edu:5001`)
- **Option 2:** Build from scratch (requires NVIDIA NGC access and Ascent Spirit SITL package)

## Repository Structure

```
AirStack/
├── robot/                          # Robot autonomy stack (ROS 2 workspace)
│   ├── docker/                     # Robot containerization
│   ├── installation/               # Robot setup scripts
│   └── ros_ws/src/autonomy/        # Layered autonomy modules:
│       ├── 0_interface/            # Robot interface layer
│       ├── 1_sensors/              # Sensor integration
│       ├── 2_perception/           # State estimation & environment understanding
│       ├── 3_local/                # Local planning, world model, and control
│       ├── 4_global/               # Global planning and mapping
│       └── 5_behavior/             # High-level decision making
├── ground_control_station/         # Monitoring and control interface
│   ├── docker/                     # GCS containerization
│   └── ros_ws/                     # GCS ROS 2 workspace
├── simulation/                     # Simulation environments
│   ├── isaac-sim/                  # NVIDIA Isaac Sim integration
│   └── gazebo/                     # Gazebo simulation support
├── docs/                           # MkDocs documentation
├── common/                         # Shared libraries and utilities
├── tests/                          # Testing infrastructure
├── git-hooks/                      # Git hooks for development workflow
├── .airstack/                      # CLI tool modules
├── airstack.sh                     # Main CLI tool script
├── docker-compose.yaml             # Top-level compose file (includes all components)
├── .env                            # Environment configuration
└── mkdocs.yml                      # Documentation configuration
```

### Key Configuration Files
- **docker-compose.yaml**: Top-level compose file that includes all component compose files
- **.env**: Main environment configuration with Docker image tags, launch parameters, and simulation settings
- **mkdocs.yml**: Documentation site configuration with comprehensive navigation structure
- **airstack.sh**: Unified CLI tool for development tasks and container management

## CI/CD and Development Workflow

### GitHub Actions Workflows

**Docker Build Pipeline** (`.github/workflows/docker-build.yml`):
- Triggers on pushes to `main` branch when Docker-related files change
- Builds and pushes both x86-64 and ARM64 (L4T) images to GitHub Container Registry
- Uses AirLab Docker registry credentials

**Documentation Deployment**:
- **Main Branch** (`deploy_docs_from_main.yaml`): Deploys docs to main version on pushes to `main`
- **Develop Branch** (`deploy_docs_from_develop.yaml`): Deploys docs to develop version on pushes to `develop`  
- **Release** (`deploy_docs_from_release.yaml`): Deploys docs to versioned release on GitHub releases
- Uses MkDocs Material with mike for versioning
- Deploys to `docs.theairlab.org`

### Development Tools

**Git Hooks**:
- Docker versioning hook automatically updates `DOCKER_IMAGE_TAG` in `.env` with git commit hash when Docker files are modified
- Ensures Docker images are tagged with exact commit for version consistency across branches

**CLI Tool** (`airstack.sh`):
- Modular command system with extensible modules in `.airstack/modules/`
- Provides unified interface for setup, installation, and container management
- Color-coded output and comprehensive help system

**Pull Request Template**:
- Structured template requiring description, implementation details, testing information, and documentation updates
- Emphasizes video/image documentation and comprehensive testing

### Code Quality and Standards
- ROS 2-based architecture with standardized package structure
- Docker-first development approach for consistent environments
- Comprehensive documentation with MkDocs Material
- Modular design enabling component-level development and testing