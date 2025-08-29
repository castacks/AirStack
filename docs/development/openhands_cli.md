# OpenHands CLI for AI-Assisted Development

AirStack is configured to support [OpenHands CLI](https://docs.all-hands.dev/usage/local-setup#option-1%3A-using-the-cli-launcher-with-uv-recommended) for AI-assisted vibe coding. OpenHands provides an intelligent development assistant that can help you write implementation code, create tests, and interact with Docker containers to build and launch your applications.

## What is OpenHands CLI?

OpenHands CLI is an AI-powered development tool that can:

- **Write Implementation Code**: Generate and modify code based on natural language descriptions
- **Create Test Code**: Automatically write unit tests, integration tests, and system tests
- **Docker Integration**: Interact with Docker containers to build, run, and debug applications
- **Repository Context**: Understand your entire codebase structure and dependencies
- **Multi-language Support**: Work with various programming languages and frameworks

## Repository Configuration

AirStack is pre-configured with OpenHands support through the `.openhands` directory, which provides:

- **Repository Context**: Complete understanding of the AirStack codebase structure
- **Development Environment**: Integration with the Docker-based development workflow
- **Project-Specific Knowledge**: Understanding of ROS 2 packages, autonomy modules, and system architecture

## Installation and Setup

To get started with OpenHands CLI for AirStack development:

1. **Install OpenHands CLI** following the official documentation:
   [https://docs.all-hands.dev/usage/local-setup#option-1%3A-using-the-cli-launcher-with-uv-recommended](https://docs.all-hands.dev/usage/local-setup#option-1%3A-using-the-cli-launcher-with-uv-recommended)

2. **Navigate to your AirStack directory**:
   ```bash
   cd /path/to/your/AirStack
   ```

3. **Launch OpenHands CLI**:
   ```bash
   openhands
   ```

The CLI will automatically detect the `.openhands` configuration and load the repository context.

## Use Cases for AirStack Development

OpenHands CLI is particularly useful for AirStack development in these scenarios:

### Code Implementation
- Creating new ROS 2 packages and nodes
- Implementing autonomy algorithms for perception, planning, and control
- Adding new sensor integrations
- Developing behavior tree nodes and actions

### Testing
- Writing unit tests for individual ROS 2 packages
- Creating integration tests for multi-node systems
- Developing system-level tests for autonomy stack components
- Setting up test fixtures and mock data

### Docker Operations
- Building and debugging Docker containers
- Running tests within the containerized environment
- Troubleshooting container networking and volume mounting
- Optimizing Docker build processes

### Documentation and Configuration
- Generating code documentation
- Creating configuration files for new components
- Writing launch files and parameter configurations
- Updating package.xml and CMakeLists.txt files

## Best Practices

When using OpenHands CLI with AirStack:

1. **Leverage Repository Context**: OpenHands understands the modular autonomy stack structure, so reference existing patterns and conventions
2. **Docker-First Development**: Remember that all code runs in containers, so test and debug within the Docker environment
3. **ROS 2 Conventions**: Follow ROS 2 best practices for package structure, naming, and interfaces
4. **Modular Design**: Maintain the layered autonomy architecture when adding new components
5. **Testing Integration**: Use the existing testing frameworks and CI/CD pipeline

## Getting Help

For OpenHands CLI-specific questions, refer to the [official documentation](https://docs.all-hands.dev/). For AirStack-specific development questions, consult the other sections of this developer guide or reach out to the AirLab team.