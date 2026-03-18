# Developer Guide

Welcome to AirStack development! This guide will help you extend and customize the autonomy stack for your needs. AirStack is designed with modularity in mind, making it straightforward to swap components and add new capabilities.

!!! tip "New to AirStack?"
    Start with the **Beginner Tutorials** to understand core concepts, then progress through intermediate and advanced topics as needed.

## Learning Path

### 🎯 Beginner Tutorials

**Start here** if you're new to AirStack development:

1. **[Key Concepts](beginner/key_concepts.md)** - Understand the AirStack workflow and architecture
2. **[Development Environment](beginner/development_environment.md)** - Set up your IDE and tools
3. **[Fork Your Own Project](beginner/fork_your_own_project.md)** - Create your own AirStack-based project
4. **[VSCode Debugging](beginner/vscode/vscode_debug.md)** - Debug ROS 2 nodes in containers

**Reference Documentation:**
- [AirStack CLI Introduction](beginner/airstack-cli/index.md) - Essential CLI commands
- [Docker Workflow](beginner/airstack-cli/docker_usage.md) - Container management details

### 📈 Intermediate Tutorials

**Build on the basics** with testing and best practices:

- **[Testing Guide](intermediate/testing/index.md)** - Test your modules effectively
  - [Unit Testing](intermediate/testing/unit_testing.md)
  - [Integration Testing](intermediate/testing/integration_testing.md)
  - [System Testing](intermediate/testing/system_testing.md)
  - [CI/CD](intermediate/testing/ci_cd.md)
- **[Frame Conventions](intermediate/frame_conventions.md)** - Coordinate frame standards
- **[Contributing](intermediate/contributing.md)** - Contribute to AirStack
- **[Documentation Guide](intermediate/documentation.md)** - Write great documentation

### 🚀 Advanced Tutorials

**Deep dives** into advanced topics:

- **[AI Agent Guide](advanced/ai_agent_guide.md)** - Automate development with AI
- **[Extending the CLI](advanced/airstack-cli/extending.md)** - Add custom CLI commands
- **[CLI Architecture](advanced/airstack-cli/architecture.md)** - Understand CLI internals

## Quick Reference

### Common Tasks

| Task | Guide |
|------|-------|
| Add a new planner | [Integration Checklist](../robot/autonomy/integration_checklist.md) |
| Add a world model | [Integration Checklist](../robot/autonomy/integration_checklist.md) |
| Create simulation scene | [Isaac Sim Setup](../simulation/isaac_sim/pegasus_scene_setup.md) |
| Debug a module | [VSCode Debugging](beginner/vscode/vscode_debug.md) |
| Write tests | [Testing Guide](intermediate/testing/index.md) |

### Essential Commands

```bash
# Start development environment
airstack up robot-desktop

# Build and test your code
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package"

# Connect to container
airstack connect robot
```

See [CLI Introduction](beginner/airstack-cli/index.md) for complete command reference.

## Architecture Resources

Before diving into development, understand the system:

- **[System Architecture](../robot/autonomy/system_architecture.md)** - How components interact
- **[Autonomy Modules](../robot/autonomy/index.md)** - Layer-by-layer breakdown
- **[Integration Checklist](../robot/autonomy/integration_checklist.md)** - Module integration guide

## Getting Help

- **Documentation**: Search this site for guides and references
- **GitHub Issues**: Report bugs and request features
- **AirStack Slack**: Join `#airstack` for discussions (AirLab members)

