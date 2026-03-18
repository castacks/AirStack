# About AirStack

AirStack is a comprehensive, modular autonomy stack for aerial robotics developed by the [AirLab](https://theairlab.org) at Carnegie Mellon University's Robotics Institute.

## Project Mission

AirStack aims to provide a complete, field-tested autonomy framework that enables researchers and developers to:

- **Deploy autonomous aerial systems** in simulation and real-world environments
- **Rapidly prototype** new algorithms and approaches
- **Integrate custom modules** through standardized interfaces
- **Scale from single to multi-robot** operations
- **Transition from simulation to hardware** with minimal changes

## Team

AirStack is developed and maintained by the AirLab at CMU's Robotics Institute, with contributions from students, researchers, and collaborators.

**Core Contributors**: Andrew Jong, John Keller, John Liu, et al. See [GitHub Contributors](https://github.com/castacks/AirStack/graphs/contributors) for more.

**Principal Investigator**: [Sebastian Scherer](https://theairlab.org/team/sebastian/)

## Research

AirStack builds upon years of research in autonomous aerial systems:

- Obstacle avoidance in cluttered environments
- Multi-robot coordination and exploration
- Vision-based state estimation
- Field robotics for search and rescue

**Related Publications**: See [AirLab Publications](https://theairlab.org/publications/)

## Status

!!! warning "Alpha Release"
    AirStack is currently in **ALPHA** status and intended for internal AirLab use.
    
    - API and functionality may change
    - Requires AirLab account for access to resources
    - Not yet ready for public release
    
    We welcome feedback from early users to improve the system for eventual public release.

## Architecture

AirStack features:

- **Modular Design**: Swap components easily through ROS 2 interfaces
- **Layered Autonomy**: Interface, Sensors, Perception, Local Planning, Global Planning, Behavior
- **Docker-Based**: Consistent development and deployment environments
- **Simulation Support**: NVIDIA Isaac Sim with Pegasus extension
- **Multi-Robot**: Independent robots with coordinated operation

See: [System Architecture](robot/autonomy/system_architecture.md)

## Supported Platforms

### Simulation
- NVIDIA Isaac Sim (primary)
- Gazebo (planned)

### Hardware
- NVIDIA Jetson (Orin, Xavier NX, TX2)
- ModalAI VOXL (VOXL 2, VOXL Flight)
- x86-64 desktop/laptop (development)

### Software
- ROS 2 Jazzy
- Ubuntu 22.04
- Docker with NVIDIA Container Toolkit

## License

!!! note "License Pending"
    License to be determined. Likely Apache 2.0 or MIT for open source components.
    
    For now, access requires AirLab account and approval.

## FAQ

### General Questions

!!! question "Who can use AirStack?"
    Currently, AirStack is in alpha and limited to AirLab members and collaborators. We plan to release it publicly once it reaches a stable state.

!!! question "What hardware do I need?"
    **For development**: Ubuntu 22.04 machine with NVIDIA GPU (RTX 3070+ recommended), 16GB+ RAM, and 100GB+ free storage.
    
    **For deployment**: NVIDIA Jetson or ModalAI VOXL onboard computer.

!!! question "Do I need a real drone to use AirStack?"
    No! You can develop and test entirely in Isaac Sim simulation before deploying to hardware.

### Technical Questions

!!! question "What's the difference between local and global planning?"
    **Local planning** handles obstacle avoidance and trajectory generation at high frequency (10+ Hz) for immediate safety.
    
    **Global planning** generates waypoint paths at lower frequency (1 Hz) for exploration and goal navigation.
    
    See: [Autonomy Modes Tutorial](tutorials/autonomy_modes.md)

!!! question "Can I use my own planner/controller?"
    Yes! AirStack is designed for easy module swapping. Create a ROS 2 package with the correct interface, integrate it into the bringup layer, and you're ready.
    
    See: [Integration Checklist](robot/autonomy/integration_checklist.md)

!!! question "How do I add a new sensor?"
    Add sensor integration to the sensors layer, ensuring data is published to standard topics (with correct frame IDs and timestamps).
    
    See: [Sensors Overview](robot/autonomy/sensors/index.md)

!!! question "Does AirStack support multi-robot missions?"
    Yes! Each robot gets a unique ROS_DOMAIN_ID and namespace. All run independently while coordinating through the Ground Control Station.
    
    See: [Multi-Robot Simulation Tutorial](tutorials/multi_robot_simulation.md)

!!! question "Can I run AirStack without Docker?"
    While technically possible, we **strongly recommend using Docker**. It ensures consistent environments, simplifies dependency management, and matches how we test and deploy.

### Development Questions

!!! question "How do I debug a module?"
    Build with debug symbols, use ROS 2 tools (`ros2 topic echo`, `ros2 node info`), and leverage the debug-module skill from the AI Agent Guide.
    
    See: [Development Environment](development/development_environment.md)

!!! question "Where should I add my custom module?"
    Place it in the appropriate layer under `robot/ros_ws/src/<layer>/<type>/<your_module>/`. For example, a new planner goes in `robot/ros_ws/src/local/planners/my_planner/`.
    
    See: [AI Agent Guide](development/ai_agent_guide.md)

!!! question "How do I test in simulation?"
    Launch AirStack with Isaac Sim, run your module, and verify behavior. Record ROS bags for offline analysis.
    
    See: [Testing Guide](development/testing/index.md)

## Contact and Support

- **Slack**: Join `#airstack` channel (AirLab members)
- **GitHub**: [AirStack Repository](https://github.com/castacks/AirStack)
- **Email**: Contact [AirLab](https://theairlab.org/contact/)
- **Documentation**: [docs.theairlab.org](https://docs.theairlab.org)

## Acknowledgments

AirStack development is supported by:

- Carnegie Mellon University Robotics Institute
- AirLab research sponsors and collaborators
- Open source ROS 2 community
- NVIDIA Isaac Sim and Pegasus simulator teams

Special thanks to all contributors and early adopters who provide valuable feedback.

## Contributing

We welcome contributions from AirLab members and collaborators!

See: [Contributing Guide](development/contributing.md)

Areas where we especially need help:

- Documentation improvements
- Bug reports and fixes
- New module implementations
- Testing and validation
- Hardware platform support

## Roadmap

Planned features and improvements:

- **Public Release**: Stable API and public documentation
- **Additional Simulators**: Gazebo support
- **More Hardware Platforms**: Broader Jetson family, additional flight controllers
- **Enhanced Multi-Robot**: Improved coordination and communication
- **Expanded Tutorials**: More examples and use cases
- **Performance Optimization**: Faster planning and perception
- **Better Testing**: Comprehensive test suite and CI/CD

Check [GitHub Issues](https://github.com/castacks/AirStack/issues) for current priorities.