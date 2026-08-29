# AirStack: Democratizing Intelligent Mobile Robotics

<div align="center">
  <img src="assets/logo_horizontal_color.png" alt="AirStack Logo" width="400"/>
</div>

AirStack is a comprehensive, modular autonomy stack for embodied AI and robotics developed by the [AirLab](https://theairlab.org) at Carnegie Mellon University's Robotics Institute. It provides a complete framework for developing, testing, and deploying autonomous mobile systems in both simulated and real-world environments.

[![GitHub](https://img.shields.io/github/license/castacks/AirStack)](https://github.com/castacks/AirStack/blob/main/LICENSE)
[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue)](https://docs.theairlab.org)

## 🚀 Features

- **Modular Architecture**: Easily swap out components to customize for your specific needs
- **ROS 2 Integration**: Built on ROS 2 for robust inter-process communication
- **Simulation Support**: Integrated with NVIDIA Isaac Sim for high-fidelity simulation
- **Multi-Robot Capability**: Control and coordinate multiple robots simultaneously
- **Ground Control Station**: Monitor and control robots through an intuitive interface
- **Comprehensive Autonomy Stack**:
  - Robot Interface Layer
  - Sensor Integration
  - Perception Systems
  - Local Planning & Control
  - Global Planning
  - Behavior Management

## 📋 System Requirements

- **Docker**: With NVIDIA Container Toolkit support
- **NVIDIA GPU**: RTX 3070 minimum, RTX 4080 or better recommended (for local Isaac Sim)
- **Storage**: Docker images take ~25GB; 100GB free disk space recommended
- **OS**: Ubuntu 22.04 or 24.04

## 🔧 Quick Start

Follow the instructions in [Getting Started](./getting_started/index.md) to set up AirStack on your machine, then take the [Modular AirStack Walkthrough](getting_started/modular_airstack.md).

## 🏗️ System Architecture

AirStack follows a layered architecture; the topology that actually launches is selected by a **stack** (a folder under `stacks/` with pinned modules and a CI-observed `wiring.md`):

```
Robot
├── Interface Layer: Communication with robot controllers
├── Sensors Layer: Data acquisition from various sensors
├── Perception Layer: State estimation and environment understanding
├── Local Layer:
│   ├── World Models: Local environment representation
│   ├── Planners: Trajectory generation and obstacle avoidance
│   └── Controls: Trajectory following
├── Global Layer:
│   ├── World Models: Global environment mapping
│   └── Planners: Mission-level path planning
└── Behavior Layer: High-level decision making
```

Capabilities beyond the trunk live in **modules** — thin external repos pulled on demand (`airstack module add <url> --version <pin>`) and listed in the [module registry](https://github.com/castacks/airstack-modules-index) — and multi-robot deployments are declared by **fleet files** under `config/fleets/`.

## 📁 Repository Structure

- `robot/`: Contains the ROS 2 workspace for the robot autonomy stack
- `stacks/`: Reference autonomy stacks (launch topology + pinned `modules.repos` + `wiring.md`)
- `config/`: Fleet files (`config/fleets/`) and vehicle definitions (`config/vehicles/`)
- `gcs/`: Software for monitoring and controlling robots
- `simulation/`: Integration with Isaac Sim and simulation environments
- `docs/`: Comprehensive documentation
- `common/`: Shared libraries and utilities (including `module_schema/` for `module.yaml`)
- `tools/`: Repo tooling (docs catalog generator, fleet resolver, wiring/DDS generators)
- `tests/`: Testing infrastructure (system, integration, and contract tests)

## 🧪 Development

AirStack is designed with modularity in mind, making it straightforward to extend or replace components. The development workflow is centered around Docker containers for consistent environments.

For detailed development guidelines, see the [Developer Guide](https://docs.theairlab.org/latest/docs/development/).

## 📚 Documentation

Comprehensive documentation is available at [https://docs.theairlab.org](https://docs.theairlab.org)

The documentation covers:

- Getting started guides
- Development workflows
- Component descriptions
- API references
- Simulation setup
- Real-world deployment

## 🤝 Contributing

We welcome contributions to AirStack! Please see our [Contributing Guidelines](development/intermediate/contributing.md) for more information.

## 📄 License

AirStack is licensed under the BSD 3-Clause Clear License (SPDX: BSD-3-Clause-Clear). See the repository [LICENSE](https://github.com/castacks/AirStack/blob/main/LICENSE) file. Vendored third-party packages retain their upstream licenses.

## 📧 Contact

For questions or support, please contact the AirLab team at [theairlab.org](https://theairlab.org).
