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
- **NVIDIA GPU**: RTX 3070 or better (for local Isaac Sim)
- **Storage**: At least 25GB free space for Docker images
- **OS**: Ubuntu 22.04 recommended

## 🔧 Quick Start

Follow the instructions at https://docs.theairlab.org/main/docs/getting_started/ to set up AirStack on your machine.

## 🏗️ System Architecture

AirStack follows a layered architecture approach:

```
Robot
├── Interface Layer: Communication with robot controllers
├── Sensors Layer: Data acquisition from various sensors
├── Perception Layer: State estimation and environment understanding
├── Local Layer:
│   ├── World Model: Local environment representation
│   ├── Planning: Trajectory generation and obstacle avoidance
│   └── Controls: Trajectory following
├── Global Layer:
│   ├── World Model: Global environment mapping
│   └── Planning: Mission-level path planning
└── Behavior Layer: High-level decision making
```

## 📁 Repository Structure

- `robot/`: Contains the ROS 2 workspace for the robot autonomy stack
- `gcs/`: Software for monitoring and controlling robots
- `simulation/`: Integration with Isaac Sim and simulation environments
- `docs/`: Comprehensive documentation
- `common/`: Shared libraries and utilities
- `tests/`: Testing infrastructure

## 🧪 Development

AirStack is designed with modularity in mind, making it straightforward to extend or replace components. The development workflow is centered around Docker containers for consistent environments.

For detailed development guidelines, see the [Developer Guide](https://docs.theairlab.org/main/docs/development/).

## 📚 Documentation

Comprehensive documentation is available at [https://docs.theairlab.org/main/docs/](https://docs.theairlab.org/main/docs/)

The documentation covers:

- Getting started guides
- Development workflows
- Component descriptions
- API references
- Simulation setup
- Real-world deployment

## 🤝 Contributing

We welcome contributions to AirStack! Please see our [Contributing Guidelines](https://docs.theairlab.org/main/docs/development/contributing/) for more information.

## 📄 License

AirStack is licensed under the Apache 2.0 or MIT license (to be finalized).

## 📧 Contact

For questions or support, please contact the AirLab team at [theairlab.org](https://theairlab.org).
