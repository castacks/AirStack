# Developer Guide

Welcome developers! This guide documents how to extend the autonomy stack for your own needs.
The stack has been designed with modularity in mind, and aims to make it straightforward to swap out any component.

We assume you're developing first on a local machine with simulation.

## Quick Start

1. **Complete [Getting Started](../getting_started.md)** - Set up your environment
2. **Learn the [System Architecture](../robot/autonomy/system_architecture.md)** - Understand how components interact
3. **Use the [Integration Checklist](../robot/autonomy/integration_checklist.md)** - Add new modules correctly

## Development Workflow

### Setting Up Your Project

- [**Creating New Projects**](create_new_project.md) - Fork or template AirStack for your own use
- [**Development Environment**](development_environment.md) - Configure your development tools
- [**VSCode Setup**](vscode/index.md) - IDE configuration for AirStack development

### Tools and Utilities

- [**AirStack CLI Tool**](airstack-cli/index.md) - Command-line interface for common tasks
  - [Docker Usage](airstack-cli/docker_usage.md) - Container management
  - [Extending](airstack-cli/extending.md) - Add custom commands
  - [Architecture](airstack-cli/architecture.md) - How the CLI works

### AI-Assisted Development

- [**AI Agent Guide**](ai_agent_guide.md) - Leverage AI agents (OpenHands, Claude) for development
  - Automated module creation
  - Integration and testing
  - Documentation generation

### Testing

- [**Testing Overview**](testing/index.md) - Testing philosophy and approaches
- [**Testing Frameworks**](testing/testing_frameworks.md) - Available testing tools
- [**Unit Testing**](testing/unit_testing.md) - Test individual components
- [**Integration Testing**](testing/integration_testing.md) - Test module interactions
- [**System Testing**](testing/system_testing.md) - End-to-end testing in simulation
- [**CI/CD**](testing/ci_cd.md) - Continuous integration and deployment

### Best Practices

- [**Documentation Guide**](documentation.md) - Writing good documentation
- [**Frame Conventions**](frame_conventions.md) - Coordinate frame standards
- [**Contributing**](contributing.md) - How to contribute to AirStack

## Common Tasks

| Task | Guide |
|------|-------|
| Add a new planner | [Integration Checklist](../robot/autonomy/integration_checklist.md) → Local Planning |
| Add a new world model | [Integration Checklist](../robot/autonomy/integration_checklist.md) → World Models |
| Create custom simulation scene | [Isaac Sim Scene Setup](../simulation/isaac_sim/scene_setup.md) |
| Deploy to hardware | [Deploying to Hardware](../tutorials/deploying_to_hardware.md) |
| Debug ROS 2 modules | [AI Agent Guide](ai_agent_guide.md) → Debug Module Skill |

## Architecture Resources

- [**System Architecture**](../robot/autonomy/system_architecture.md) - Detailed architecture diagrams
- [**Autonomy Modules Overview**](../robot/autonomy/index.md) - Layer-by-layer breakdown
- [**Integration Checklist**](../robot/autonomy/integration_checklist.md) - Module integration guide

## Getting Help

- **AirStack Slack**: Join `#airstack` channel for questions and discussions
- **GitHub Issues**: Report bugs and request features
- **Documentation**: Search this documentation site
- **AI Agents**: Use the [AI Agent Guide](ai_agent_guide.md) for automated assistance

